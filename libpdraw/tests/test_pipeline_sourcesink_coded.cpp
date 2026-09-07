/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — coded video source -> sink roundtrip on a real
 * H.264/H.265 elementary stream (Tier B, self-contained fixture)
 *
 * Copyright (c) 2026 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Adapted from libpdraw-backend/tests/pdraw_codedsourcesink_test.c, which
 * feeds a raw H.264/H.265 elementary stream file into an ICodedVideoSource
 * and writes back out whatever an ICodedVideoSink attached to that same
 * media receives, exercising a real Source -> Channel -> Sink passthrough
 * within a single pdraw session (no demuxer/decoder/muxer involved).
 *
 * Two differences from that program, both intentional:
 *  - This uses libpdraw directly (IPdraw::ICodedVideoSource/ICodedVideoSink),
 *    not libpdraw-backend: no marshalling thread, so no pthread mutex/cond
 *    needed -- everything runs synchronously on this test's thread via
 *    TestPompLoop::pumpUntil(), like every other test_pipeline_*.cpp file.
 *  - Parsing the input file into frames is fully decoupled from feeding
 *    them to pdraw (see runCodedSourceSinkRoundtrip() below) instead of
 *    interleaving both, because pushing a frame into the source's queue
 *    and pumping the loop before a sink exists would cause
 *    ExternalCodedVideoSource::process() to silently pop and discard it
 *    (zero connected output channels): the sink must exist first.
 *
 * Since the source only adds ancillary metadata and never touches NALU
 * payload bytes (see ExternalCodedVideoSource::processFrame() in
 * pdraw_external_coded_video_source.cpp), and every NALU in the input
 * (including SPS/PPS/VPS) ends up inside some frame (see nalu_end() below,
 * ported near-verbatim from the reference program), the frames written
 * back out should reconstruct the input file exactly -- checked with a
 * plain byte-for-byte file comparison. This assumes the input elementary
 * stream uses a uniform 4-byte start code (matching what append_to_frame()
 * always (re)writes); if crow_run_240p.264/.265 turns out to mix 3- and
 * 4-byte start codes, this comparison would need to become NALU-payload-
 * based instead (parse both files and compare per-NALU payloads, ignoring
 * start code length) -- deliberately not done upfront without evidence
 * it's needed. */

#define ULOG_TAG pdraw_test_pipeline_sourcesink_coded
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include "pdraw_external_coded_video_sink.hpp"
#include "pdraw_external_coded_video_source.hpp"

#include <h264/h264.h>
#include <h265/h265.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-defs/vdefs.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── H.264/H.265 elementary stream fixtures ──────────────────────────────
 * (NAS assets, same ASSETS_ROOT/PDRAW_GET_ASSET_PATH mechanism as every
 * other test_pipeline_*.cpp / test_api_demuxer.cpp file). 100 frames each,
 * 240p. */

enum { ASSET_H264 = 0, ASSET_H265 = 1 };

static constexpr struct {
	const char *relative_path;
} s_assets_sourcesink[] = {
	{"Tests/miscellaneous/crowd_run_240p.264"},
	{"Tests/miscellaneous/crowd_run_240p.265"},
};


/* Session-wide listener: correlates onMediaAdded() with our own source
 * instance via elementUserData (documented as "the pipeline element that
 * created the media"; ExternalCodedVideoSource::callOnMediaAdded() passes
 * its own IPdraw::ICodedVideoSource* wrapper there), matching
 * media_added_cb()'s "element_userdata != self->source" filter in the
 * reference program. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class SourceMediaListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override
	{
		if (elementUserData != mSource)
			return;
		mMediaId = info->id;
		mGotMediaAdded = true;
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	/* Must be set (to the IPdraw::ICodedVideoSource* returned by
	 * createCodedVideoSource()) right after creation and before pumping
	 * the loop, so onMediaAdded() can filter on it. */
	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	unsigned int mMediaId = 0;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Tracks ICodedVideoSource::flush()/drain() completion. Unlike
 * g_stub_coded_video_source_listener (shared by every other test in this
 * file, which never needs to observe these events), this listener is used
 * only by the happy-path roundtrip test below, which -- like
 * pdraw_codedsourcesink_test.c's reference program -- calls drain() before
 * tearing down the source. */
class DrainTrackingCodedVideoSourceListener
		: public IPdraw::ICodedVideoSource::Listener {
public:
	void
	onCodedVideoSourceFlushed(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*src*/) override
	{
		mGotFlushed = true;
	}

	void
	onCodedVideoSourceDrained(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*src*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* Implements the sink side of the flush/drain protocol for real, mirroring
 * sink_flush_cb()/sink_drain_cb() in the reference pdraw_codedsourcesink_test.c
 * program: pop (and unref) every frame already sitting in the sink's own
 * queue, then acknowledge via queueFlushed()/queueDrained(). Without this,
 * calling ICodedVideoSource::drain() against g_stub_coded_video_sink_listener
 * (a no-op stub, used by every other test in this file) would hang forever --
 * the stub never acknowledges, so Channel::drainDone() never fires and
 * onCodedVideoSourceDrained() never fires either. */
class QueueDrainingCodedVideoSinkListener
		: public IPdraw::ICodedVideoSink::Listener {
public:
	void onCodedVideoSinkMediaAdded(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct pdraw_media_info * /*i*/) override
	{
	}

	void onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
					  IPdraw::ICodedVideoSink * /*sk*/,
					  const struct pdraw_media_info * /*i*/,
					  bool /*restart*/) override
	{
	}

	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		/* Flush really means discard: unref outright. */
		discardQueue();
		sk->queueFlushed();
		/* Lets a test wait for this specific callback to have fired
		 * -- e.g. resync()'s own internal self-flush is async, and a
		 * test pushing a new frame right after resync() needs to know
		 * that self-flush is done discarding *before* the new frame
		 * arrives, or it could get discarded too. */
		mGotFlushCallback = true;
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		/* Unlike flush, drain must not lose data: whatever is in the
		 * queue at this point may be real frames process() just
		 * forwarded (see ExternalCodedVideoSource::flush(discard=
		 * false), which calls process() BEFORE draining the output
		 * channel) -- accumulate into mDrainedFrames instead of
		 * unreffing outright, so a test can inspect what was
		 * delivered. The caller owns the refs afterward. */
		while (true) {
			struct mbuf_coded_video_frame *f = nullptr;
			if (mQueue == nullptr ||
			    mbuf_coded_video_frame_queue_pop(mQueue, &f) != 0)
				break;
			mDrainedFrames.push_back(f);
		}
		sk->queueDrained();
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session *meta) override
	{
		if (meta != nullptr)
			mLastSessionMeta = *meta;
		mGotSessionMetaUpdate = true;
	}

	/* Must be set right after createCodedVideoSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_coded_video_frame_queue *mQueue = nullptr;

	/* Populated by onCodedVideoSinkDrain(); empty in the roundtrip tests
	 * (outQueue is already drained by the test itself before drain() is
	 * ever called there), populated for real in
	 * testCxxCodedSourceDrainForwardsUnprocessedFrames. Caller must unref
	 * each entry. */
	std::vector<struct mbuf_coded_video_frame *> mDrainedFrames;

	bool mGotFlushCallback = false;

	/* Populated by onCodedVideoSinkSessionMetaUpdate(). */
	bool mGotSessionMetaUpdate = false;
	struct vmeta_session mLastSessionMeta = {};

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_coded_video_frame_unref(f);
	}
};


/* Acknowledges flush/drain with the WRONG companion function on purpose, to
 * exercise ExternalCodedVideoSink::flushDone()'s mFlushDiscard mismatch
 * warning path (pdraw_external_coded_video_sink.cpp:333-338) via
 * testCxxCodedVideoSinkFlushAckMismatchFollowsRequestedDiscardState below.
 * Only onCodedVideoSinkFlush() needs to misbehave for that test;
 * onCodedVideoSinkDrain() still acks correctly since it is not exercised
 * there. */
class MismatchAckCodedVideoSinkListener
		: public IPdraw::ICodedVideoSink::Listener {
public:
	void onCodedVideoSinkMediaAdded(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
					  IPdraw::ICodedVideoSink * /*sk*/,
					  const struct pdraw_media_info * /*i*/,
					  bool /*restart*/) override
	{
	}

	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		/* Wrong ack on purpose: a real flush(discard=true) is
		 * underway (mFlushDiscard==true), but acknowledge as if it
		 * were a drain. */
		sk->queueDrained();
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		sk->queueDrained();
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	bool mGotMediaAdded = false;
};


/* Tracks ICodedVideoSink media-added/media-removed callbacks; used only by
 * testCxxCodedSourceSinkSwitchMediaId below to observe setMediaId()'s
 * effects. Flush/drain are left as no-ops, like g_stub_coded_video_sink_
 * listener (shared by most other tests in this file): that test never calls
 * flush()/drain()/resync() itself, so no acknowledgement is required. */
class CodedVideoSinkTrackingListener
		: public IPdraw::ICodedVideoSink::Listener {
public:
	void onCodedVideoSinkMediaAdded(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
					  IPdraw::ICodedVideoSink * /*sk*/,
					  const struct pdraw_media_info * /*i*/,
					  bool restart) override
	{
		mGotMediaRemoved = true;
		mLastRemovedRestart = restart;
	}

	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink * /*sk*/) override
	{
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink * /*sk*/) override
	{
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	bool mGotMediaAdded = false;
	bool mGotMediaRemoved = false;
	bool mLastRemovedRestart = false;
};


/* ── Elementary stream parsing into mbuf_coded_video_frame's (adapted
 * near-verbatim from pdraw_codedsourcesink_test.c's h264_to_vdef_info/
 * h265_to_vdef_info/configure/au_process/append_to_frame/au_end/nalu_end/
 * *_cb functions -- only configure() differs: it no longer touches pdraw
 * at all, see the file-level comment above) ──────────────────────────── */

union nalu_type {
	enum h264_nalu_type h264;
	enum h265_nalu_type h265;
};

struct ParseContext {
	struct vdef_coded_frame in_info = {};
	struct vdef_format_info format_info = {};
	bool configured = false;
	uint64_t ts_inc = 33333;

	struct mbuf_mem *in_mem = nullptr;
	size_t in_mem_offset = 0;
	struct mbuf_coded_video_frame *in_frame = nullptr;

	uint8_t *vps = nullptr;
	size_t vps_size = 0;
	uint8_t *sps = nullptr;
	size_t sps_size = 0;
	uint8_t *pps = nullptr;
	size_t pps_size = 0;

	unsigned int input_count = 0;

	/* Collected frames, deferred from pdraw until a sink exists (see the
	 * file-level comment above for why). */
	std::vector<struct mbuf_coded_video_frame *> frames;

	~ParseContext()
	{
		if (in_frame != nullptr)
			mbuf_coded_video_frame_unref(in_frame);
		if (in_mem != nullptr)
			mbuf_mem_unref(in_mem);
		for (auto *f : frames)
			mbuf_coded_video_frame_unref(f);
		free(vps);
		free(sps);
		free(pps);
	}
};


static inline unsigned int gcd_u(unsigned int a, unsigned int b)
{
	unsigned int c;
	while (a != 0) {
		c = a;
		a = b % a;
		b = c;
	}
	return b;
}


static void h264_to_vdef_info(const struct h264_info *in,
			      struct vdef_format_info *out)
{
	out->framerate.num = in->framerate_num;
	out->framerate.den = in->framerate_den;
	if (out->framerate.den) {
		unsigned int divider =
			gcd_u(out->framerate.num, out->framerate.den);
		out->framerate.den /= divider;
		out->framerate.num /= divider;
	}
	out->bit_depth = in->bit_depth_luma;
	out->full_range = in->full_range;
	if (in->colour_description_present) {
		out->color_primaries =
			vdef_color_primaries_from_h264(in->colour_primaries);
		out->transfer_function = vdef_transfer_function_from_h264(
			in->transfer_characteristics);
		out->matrix_coefs =
			vdef_matrix_coefs_from_h264(in->matrix_coefficients);
	} else {
		out->color_primaries = VDEF_COLOR_PRIMARIES_UNKNOWN;
		out->transfer_function = VDEF_TRANSFER_FUNCTION_UNKNOWN;
		out->matrix_coefs = VDEF_MATRIX_COEFS_UNKNOWN;
	}
	out->resolution.width = in->crop_width;
	out->resolution.height = in->crop_height;
	out->sar.width = in->sar_width;
	out->sar.height = in->sar_height;
}


static void h265_to_vdef_info(const struct h265_info *in,
			      struct vdef_format_info *out)
{
	out->framerate.num = in->framerate_num;
	out->framerate.den = in->framerate_den;
	if (out->framerate.den) {
		unsigned int divider =
			gcd_u(out->framerate.num, out->framerate.den);
		out->framerate.den /= divider;
		out->framerate.num /= divider;
	}
	out->bit_depth = in->bit_depth_luma;
	out->full_range = in->full_range;
	if (in->colour_description_present) {
		out->color_primaries =
			vdef_color_primaries_from_h265(in->colour_primaries);
		out->transfer_function = vdef_transfer_function_from_h265(
			in->transfer_characteristics);
		out->matrix_coefs =
			vdef_matrix_coefs_from_h265(in->matrix_coefficients);
	} else {
		out->color_primaries = VDEF_COLOR_PRIMARIES_UNKNOWN;
		out->transfer_function = VDEF_TRANSFER_FUNCTION_UNKNOWN;
		out->matrix_coefs = VDEF_MATRIX_COEFS_UNKNOWN;
	}
	out->resolution.width = in->crop_width;
	out->resolution.height = in->crop_height;
	out->sar.width = in->sar_width;
	out->sar.height = in->sar_height;
}


/* Unlike the reference program's configure(), this only computes
 * format_info from the just-found parameter sets and marks the context
 * "configured" so au_process() (below) starts collecting frames -- no
 * pdraw object is created here (see the file-level comment above). */
static void configure(struct ParseContext *ctx,
		      const struct vdef_coded_format *format)
{
	switch (format->encoding) {
	case VDEF_ENCODING_H264: {
		struct h264_info info;
		int res = h264_get_info(ctx->sps,
					ctx->sps_size,
					ctx->pps,
					ctx->pps_size,
					&info);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		h264_to_vdef_info(&info, &ctx->format_info);
		break;
	}
	case VDEF_ENCODING_H265: {
		struct h265_info info;
		int res = h265_get_info(ctx->vps,
					ctx->vps_size,
					ctx->sps,
					ctx->sps_size,
					ctx->pps,
					ctx->pps_size,
					&info);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		h265_to_vdef_info(&info, &ctx->format_info);
		break;
	}
	default:
		break;
	}

	vdef_format_to_frame_info(&ctx->format_info, &ctx->in_info.info);
	ctx->in_info.info.timescale = 1000000;
	if ((ctx->format_info.framerate.num != 0) &&
	    (ctx->format_info.framerate.den != 0)) {
		ctx->ts_inc = ctx->format_info.framerate.den * 1000000 /
			      ctx->format_info.framerate.num;
	}

	ctx->configured = true;
}


static void au_process(struct ParseContext *ctx)
{
	int res;

	if ((ctx->in_frame == nullptr) || (!ctx->configured))
		return;

	res = mbuf_coded_video_frame_set_frame_info(ctx->in_frame,
						    &ctx->in_info);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(ctx->in_frame);
	CU_ASSERT_EQUAL(res, 0);

	/* Collected here instead of pushed into a pdraw queue (see the
	 * file-level comment above); one ref transfers into the vector. */
	ctx->frames.push_back(ctx->in_frame);
	ctx->in_frame = nullptr;
	ctx->input_count++;

	if (ctx->in_mem != nullptr) {
		mbuf_mem_unref(ctx->in_mem);
		ctx->in_mem = nullptr;
	}
	ctx->in_info.info.index++;
	ctx->in_info.info.timestamp += ctx->ts_inc;
	ctx->in_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;
}


static void append_to_frame(struct ParseContext *ctx,
			    struct mbuf_coded_video_frame *frame,
			    struct mbuf_mem *mem,
			    const uint8_t *data,
			    size_t len,
			    union nalu_type type,
			    enum vdef_encoding encoding)
{
	size_t au_offset = ctx->in_mem_offset;
	size_t capacity;
	size_t nalu_offset = 4;
	uint8_t *au_data;
	uint8_t *nalu_data;
	uint32_t start;
	int res;

	res = mbuf_mem_get_data(mem, (void **)&au_data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_FATAL(capacity >= au_offset + nalu_offset + len);

	nalu_data = au_data + au_offset;
	start = htonl(0x00000001);
	memcpy(nalu_data, &start, sizeof(uint32_t));
	memcpy(nalu_data + nalu_offset, data, len);
	ctx->in_mem_offset = au_offset + nalu_offset + len;

	struct vdef_nalu nalu = {};
	nalu.size = len + nalu_offset;
	switch (encoding) {
	case VDEF_ENCODING_H264:
		nalu.h264.type = type.h264;
		break;
	case VDEF_ENCODING_H265:
		nalu.h265.type = type.h265;
		break;
	default:
		CU_FAIL("unsupported encoding");
		return;
	}
	res = mbuf_coded_video_frame_add_nalu(frame, mem, au_offset, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
}


static void au_end(struct ParseContext *ctx)
{
	if (ctx->in_frame != nullptr)
		au_process(ctx);
}


static void h264_au_end_cb(struct h264_ctx * /*ctx*/, void *userdata)
{
	au_end(static_cast<struct ParseContext *>(userdata));
}


static void h265_au_end_cb(struct h265_ctx * /*ctx*/, void *userdata)
{
	au_end(static_cast<struct ParseContext *>(userdata));
}


#define SOURCESINK_DEFAULT_FRAME_LEN (426 * 240 * 3 / 4)

static void nalu_end(struct ParseContext *ctx,
		     const struct vdef_coded_format *format,
		     union nalu_type type,
		     const uint8_t *buf,
		     size_t len)
{
	int ps_ready = 0;

	/* mbuf_coded_video_frame_new() validates frame_info->format, so it
	 * must be set before the first frame is created below (it never
	 * changes across calls for a single elementary stream). */
	ctx->in_info.format = *format;

	switch (format->encoding) {
	case VDEF_ENCODING_H264:
		if ((type.h264 == H264_NALU_TYPE_SPS) &&
		    (ctx->sps == nullptr)) {
			ctx->sps_size = len;
			ctx->sps = static_cast<uint8_t *>(malloc(len));
			CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->sps);
			memcpy(ctx->sps, buf, len);
		} else if ((type.h264 == H264_NALU_TYPE_PPS) &&
			   (ctx->pps == nullptr)) {
			ctx->pps_size = len;
			ctx->pps = static_cast<uint8_t *>(malloc(len));
			CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->pps);
			memcpy(ctx->pps, buf, len);
		} else if (type.h264 == H264_NALU_TYPE_SLICE_IDR) {
			ctx->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		ps_ready = (ctx->sps != nullptr) && (ctx->pps != nullptr);
		break;
	case VDEF_ENCODING_H265:
		if ((type.h265 == H265_NALU_TYPE_VPS_NUT) &&
		    (ctx->vps == nullptr)) {
			ctx->vps_size = len;
			ctx->vps = static_cast<uint8_t *>(malloc(len));
			CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->vps);
			memcpy(ctx->vps, buf, len);
		} else if ((type.h265 == H265_NALU_TYPE_SPS_NUT) &&
			   (ctx->sps == nullptr)) {
			ctx->sps_size = len;
			ctx->sps = static_cast<uint8_t *>(malloc(len));
			CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->sps);
			memcpy(ctx->sps, buf, len);
		} else if ((type.h265 == H265_NALU_TYPE_PPS_NUT) &&
			   (ctx->pps == nullptr)) {
			ctx->pps_size = len;
			ctx->pps = static_cast<uint8_t *>(malloc(len));
			CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->pps);
			memcpy(ctx->pps, buf, len);
		} else if (type.h265 == H265_NALU_TYPE_IDR_W_RADL ||
			   type.h265 == H265_NALU_TYPE_IDR_N_LP) {
			ctx->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		ps_ready = (ctx->vps != nullptr) && (ctx->sps != nullptr) &&
			   (ctx->pps != nullptr);
		break;
	default:
		break;
	}

	if (!ctx->configured && ps_ready)
		configure(ctx, format);

	/* Get an input buffer */
	if (ctx->in_mem == nullptr) {
		size_t frame_len = ctx->format_info.resolution.width *
				   ctx->format_info.resolution.height * 3 / 4;
		if (frame_len == 0)
			frame_len = SOURCESINK_DEFAULT_FRAME_LEN;
		int res = mbuf_mem_generic_new(frame_len, &ctx->in_mem);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		ctx->in_mem_offset = 0;
	}

	/* Create the frame */
	if (ctx->in_frame == nullptr) {
		int res = mbuf_coded_video_frame_new(&ctx->in_info,
						     &ctx->in_frame);
		CU_ASSERT_EQUAL_FATAL(res, 0);
	}

	append_to_frame(ctx,
			ctx->in_frame,
			ctx->in_mem,
			buf,
			len,
			type,
			format->encoding);
}


static void h264_nalu_end_cb(struct h264_ctx * /*ctx*/,
			     enum h264_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h264_nalu_header * /*nh*/,
			     void *userdata)
{
	auto *ctx = static_cast<struct ParseContext *>(userdata);
	struct vdef_coded_format format = {};
	format.encoding = VDEF_ENCODING_H264;
	format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	union nalu_type nt;
	nt.h264 = type;
	nalu_end(ctx, &format, nt, buf, len);
}


static void h265_nalu_end_cb(struct h265_ctx * /*ctx*/,
			     enum h265_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h265_nalu_header * /*nh*/,
			     void *userdata)
{
	auto *ctx = static_cast<struct ParseContext *>(userdata);
	struct vdef_coded_format format = {};
	format.encoding = VDEF_ENCODING_H265;
	format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	union nalu_type nt;
	nt.h265 = type;
	nalu_end(ctx, &format, nt, buf, len);
}


/* Built with '= {}' (zero-init) plus plain field assignment rather than a
 * partial designated initializer, so -Wall's -Wmissing-field-initializers
 * doesn't fire for every one of these callback structs' many unused
 * members. */
static const struct h264_ctx_cbs kH264Cbs = [] {
	struct h264_ctx_cbs cbs = {};
	cbs.au_end = h264_au_end_cb;
	cbs.nalu_end = h264_nalu_end_cb;
	return cbs;
}();

static const struct h265_ctx_cbs kH265Cbs = [] {
	struct h265_ctx_cbs cbs = {};
	cbs.nalu_end = h265_nalu_end_cb;
	cbs.au_end = h265_au_end_cb;
	return cbs;
}();


/* ── File I/O helpers ─────────────────────────────────────────────────── */

struct MappedFile {
	int fd = -1;
	void *data = nullptr;
	size_t len = 0;

	~MappedFile()
	{
		if (data != nullptr && len > 0)
			munmap(data, len);
		if (fd >= 0)
			close(fd);
	}
};

static bool mapFile(const char *path, struct MappedFile *out)
{
	out->fd = open(path, O_RDONLY);
	if (out->fd < 0)
		return false;
	off_t size = lseek(out->fd, 0, SEEK_END);
	if (size < 0)
		return false;
	out->len = static_cast<size_t>(size);
	out->data = mmap(nullptr, out->len, PROT_READ, MAP_PRIVATE, out->fd, 0);
	return out->data != MAP_FAILED;
}


static bool filesAreIdentical(const char *pathA, const char *pathB)
{
	struct stat stA = {};
	struct stat stB = {};
	if (stat(pathA, &stA) != 0 || stat(pathB, &stB) != 0)
		return false;
	if (stA.st_size != stB.st_size)
		return false;

	FILE *fa = fopen(pathA, "rb");
	FILE *fb = fopen(pathB, "rb");
	if (fa == nullptr || fb == nullptr) {
		if (fa != nullptr)
			fclose(fa);
		if (fb != nullptr)
			fclose(fb);
		return false;
	}

	bool identical = true;
	uint8_t bufA[4096];
	uint8_t bufB[4096];
	while (true) {
		size_t ra = fread(bufA, 1, sizeof(bufA), fa);
		size_t rb = fread(bufB, 1, sizeof(bufB), fb);
		if (ra != rb || memcmp(bufA, bufB, ra) != 0) {
			identical = false;
			break;
		}
		if (ra == 0)
			break;
	}
	fclose(fa);
	fclose(fb);
	return identical;
}


/* ── Shared setup: parse a whole file, and build source params from it ──── */

/* Parses the whole input file into ctx->frames (see the file-level comment
 * for why pdraw is not involved yet). Factored out of the roundtrip test so
 * the negative tests below can get a handful of valid frames + SPS/PPS/VPS
 * without duplicating the NALU parsing machinery. */
static void parseCodedFile(enum vdef_encoding encoding,
			   size_t assetIndex,
			   struct ParseContext *ctx)
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, assetIndex, s_assets_sourcesink);

	struct MappedFile input;
	CU_ASSERT_TRUE_FATAL(mapFile(inPath, &input));

	struct h264_reader *h264Reader = nullptr;
	struct h265_reader *h265Reader = nullptr;
	int ret;
	switch (encoding) {
	case VDEF_ENCODING_H264:
		ret = h264_reader_new(&kH264Cbs, ctx, &h264Reader);
		break;
	case VDEF_ENCODING_H265:
		ret = h265_reader_new(&kH265Cbs, ctx, &h265Reader);
		break;
	default:
		CU_FAIL_FATAL("unsupported encoding");
		return;
	}
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	size_t inOff = 0;
	while (inOff < input.len) {
		size_t off = 0;
		const uint8_t *buf =
			static_cast<const uint8_t *>(input.data) + inOff;
		switch (encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_reader_parse(
				h264Reader, 0, buf, input.len - inOff, &off);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_reader_parse(
				h265Reader, 0, buf, input.len - inOff, &off);
			break;
		default:
			ret = -ENOSYS;
			break;
		}
		CU_ASSERT_FATAL(ret >= 0);
		inOff += off;
		if (off == 0)
			break; /* no progress: avoid an infinite loop */
	}
	/* Process the last AU in the file (mirrors au_end() being called on
	 * reader_stop() below for every prior AU already flushed). */
	au_process(ctx);
	if (h264Reader != nullptr) {
		h264_reader_stop(h264Reader);
		h264_reader_destroy(h264Reader);
	}
	if (h265Reader != nullptr) {
		h265_reader_stop(h265Reader);
		h265_reader_destroy(h265Reader);
	}

	CU_ASSERT_EQUAL_FATAL(ctx->frames.size(), 100u);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->pps);
	if (encoding == VDEF_ENCODING_H265)
		CU_ASSERT_PTR_NOT_NULL_FATAL(ctx->vps);
}

/* Fills sourceParams from a parsed ParseContext (SPS/PPS/VPS + format
 * info); shared between the roundtrip test and the negative tests below. */
static void
fillCodedSourceParams(enum vdef_encoding encoding,
		      const struct ParseContext &ctx,
		      struct pdraw_video_source_params *sourceParams)
{
	*sourceParams = {};
	sourceParams->queue_max_count = 0; /* unbounded by default */
	sourceParams->playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams->video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams->video.coded.format.encoding = encoding;
	sourceParams->video.coded.format.data_format =
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	sourceParams->video.coded.info = ctx.format_info;
	snprintf(sourceParams->session_meta.friendly_name,
		 sizeof(sourceParams->session_meta.friendly_name),
		 "pdraw_test_pipeline_sourcesink_coded");
	switch (encoding) {
	case VDEF_ENCODING_H264:
		CU_ASSERT_FATAL(ctx.sps_size <=
				sizeof(sourceParams->video.coded.h264.sps));
		memcpy(sourceParams->video.coded.h264.sps,
		       ctx.sps,
		       ctx.sps_size);
		sourceParams->video.coded.h264.spslen = ctx.sps_size;
		CU_ASSERT_FATAL(ctx.pps_size <=
				sizeof(sourceParams->video.coded.h264.pps));
		memcpy(sourceParams->video.coded.h264.pps,
		       ctx.pps,
		       ctx.pps_size);
		sourceParams->video.coded.h264.ppslen = ctx.pps_size;
		break;
	case VDEF_ENCODING_H265:
		CU_ASSERT_FATAL(ctx.vps_size <=
				sizeof(sourceParams->video.coded.h265.vps));
		memcpy(sourceParams->video.coded.h265.vps,
		       ctx.vps,
		       ctx.vps_size);
		sourceParams->video.coded.h265.vpslen = ctx.vps_size;
		CU_ASSERT_FATAL(ctx.sps_size <=
				sizeof(sourceParams->video.coded.h265.sps));
		memcpy(sourceParams->video.coded.h265.sps,
		       ctx.sps,
		       ctx.sps_size);
		sourceParams->video.coded.h265.spslen = ctx.sps_size;
		CU_ASSERT_FATAL(ctx.pps_size <=
				sizeof(sourceParams->video.coded.h265.pps));
		memcpy(sourceParams->video.coded.h265.pps,
		       ctx.pps,
		       ctx.pps_size);
		sourceParams->video.coded.h265.ppslen = ctx.pps_size;
		break;
	default:
		break;
	}
}


/* ── The test itself ──────────────────────────────────────────────────── */

static void runCodedSourceSinkRoundtrip(enum vdef_encoding encoding,
					size_t assetIndex,
					const char *outSuffix)
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, assetIndex, s_assets_sourcesink);

	struct ParseContext ctx;
	parseCodedFile(encoding, assetIndex, &ctx);

	/* ── Real pdraw pipeline: source -> sink, same session ── */

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(encoding, ctx, &sourceParams);
	sourceParams.queue_max_count = 0; /* unbounded: holds all 100 frames */

	DrainTrackingCodedVideoSourceListener sourceListener;
	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	/* Pump now, before pushing any frame: the source's queue is empty at
	 * this point, so this only lets onMediaAdded() fire (it cannot
	 * prematurely drain frames that have not been pushed yet). */
	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Now that the sink is connected, push every collected frame. */
	for (auto *frame : ctx.frames) {
		ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Pump until the sink has received every frame (see the file-level
	 * comment: the source -> sink hop is dispatched via the pomp loop,
	 * not synchronously on push). */
	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotAllFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= ctx.frames.size();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAllFrames);
	CU_ASSERT_EQUAL(outputFrames.size(), ctx.frames.size());

	/* Write every NALU of every output frame, in order, to a fresh
	 * output file. */
	char outPath[512];
	snprintf(outPath,
		 sizeof(outPath),
		 "/tmp/pdraw_test_pipeline_sourcesink%s",
		 outSuffix);
	FILE *outFile = fopen(outPath, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(outFile);
	for (auto *frame : outputFrames) {
		int naluCountRet = mbuf_coded_video_frame_get_nalu_count(frame);
		CU_ASSERT_FATAL(naluCountRet >= 0);
		size_t naluCount = static_cast<size_t>(naluCountRet);
		for (size_t i = 0; i < naluCount; i++) {
			const void *naluData = nullptr;
			struct vdef_nalu nalu = {};
			ret = mbuf_coded_video_frame_get_nalu(
				frame, i, &naluData, &nalu);
			CU_ASSERT_EQUAL(ret, 0);
			size_t written =
				fwrite(naluData, nalu.size, 1, outFile);
			CU_ASSERT_EQUAL(written, 1u);
			ret = mbuf_coded_video_frame_release_nalu(
				frame, i, naluData);
			CU_ASSERT_EQUAL(ret, 0);
		}
		mbuf_coded_video_frame_unref(frame);
	}
	fclose(outFile);

	/* The whole point: the roundtrip reproduces the input exactly. */
	CU_ASSERT_TRUE(filesAreIdentical(inPath, outPath));
	(void)remove(outPath);

	/* Drain, THEN flush, the source before tearing down -- in that order,
	 * deliberately. ExternalCodedVideoSource::flush(bool discard):
	 *   - discard=false (drain()): calls process() first, which forces
	 *     anything still sitting in the source's own input queue
	 *     (mFrameQueue, i.e. our inQueue) to be popped and forwarded
	 *     downstream before completing -- no data loss, by construction.
	 *   - discard=true (flush()): skips process() and calls
	 *     mFrameQueue->flush() directly, discarding whatever is still in
	 *     inQueue outright.
	 * So drain() first is what actually guarantees nothing is lost (e.g.
	 * a frame still sitting unprocessed in inQueue); calling flush()
	 * afterward is then safe by construction -- both queues are
	 * necessarily empty by then -- rather than relying on this test
	 * having already popped everything from outQueue above (true here,
	 * but that's a property of *this* test, not of flush() itself: a
	 * frame still queued in inQueue at flush() time would be discarded
	 * outright, never forwarded downstream). */
	ret = source->drain();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE(gotDrained);

	ret = source->flush();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE(gotFlushed);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxCodedSourceSinkRoundtripH264()
{
	runCodedSourceSinkRoundtrip(VDEF_ENCODING_H264, ASSET_H264, ".264");
}


static void testCxxCodedSourceSinkRoundtripH265()
{
	runCodedSourceSinkRoundtrip(VDEF_ENCODING_H265, ASSET_H265, ".265");
}


/* ── Negative / edge-case tests: exercise ExternalCodedVideoSource::
 * inputFilter() (format intersect check, strictly-increasing timestamp
 * check) and the underlying mbuf queue's max_frames drop-oldest behavior --
 * all previously only read from source, never directly tested. ──────────── */

static void
createCodedSourceAndSink(IPdraw *session,
			 TestPompLoop *loop,
			 SourceMediaListener *mediaListener,
			 const struct pdraw_video_source_params *sourceParams,
			 IPdraw::ICodedVideoSource **source,
			 IPdraw::ICodedVideoSink **sink,
			 struct mbuf_coded_video_frame_queue **inQueue,
			 struct mbuf_coded_video_frame_queue **outQueue)
{
	int ret = session->createCodedVideoSource(
		sourceParams, &g_stub_coded_video_source_listener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);
	mediaListener->mSource = *source;

	bool gotMediaAdded = loop->pumpUntil(
		[mediaListener]() { return mediaListener->mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener->mMediaId, 0u);

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createCodedVideoSink(mediaListener->mMediaId,
					    &sinkParams,
					    &g_stub_coded_video_sink_listener,
					    sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);
	*outQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outQueue);
}

/* A minimal, validly-finalized frame carrying whatever encoding/timestamp
 * the caller wants -- content is irrelevant since these tests only care
 * about acceptance/rejection at the queue's filter, before any real NALU
 * data would ever be looked at. */
static struct mbuf_coded_video_frame *
makeDummyCodedFrame(enum vdef_encoding encoding, uint64_t timestamp)
{
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format.encoding = encoding;
	frameInfo.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(16, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);

	struct vdef_nalu nalu = {};
	nalu.size = capacity;
	if (encoding == VDEF_ENCODING_H264)
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	else if (encoding == VDEF_ENCODING_H265)
		nalu.h265.type = H265_NALU_TYPE_IDR_W_RADL;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem); /* the frame holds its own ref via add_nalu */

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxCodedSourceRejectsFormatMismatch()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 1);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	struct mbuf_coded_video_frame_queue *inQueue = nullptr;
	struct mbuf_coded_video_frame_queue *outQueue = nullptr;
	createCodedSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* A valid frame is accepted... */
	int ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...but a frame declaring the OTHER encoding is rejected outright,
	 * per ExternalCodedVideoSource::inputFilter()'s
	 * vdef_coded_format_intersect() check -- never reaches the queue. */
	struct mbuf_coded_video_frame *badFrame =
		makeDummyCodedFrame(VDEF_ENCODING_H265, UINT64_C(999999999));
	ret = mbuf_coded_video_frame_queue_push(inQueue, badFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_coded_video_frame_unref(badFrame);

	/* The valid frame still made it through undisturbed. */
	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_coded_video_frame_unref(f);

	/* stopSessionAndWait() must run BEFORE the owners are reset, so that
	 * Session::asyncElementDelete() destroys the ExternalCodedVideoSource/
	 * ExternalCodedVideoSink elements (and thus runs their wrappers'
	 * clearElement() overrides) while the wrappers are still alive --
	 * resetting first would run ~ElementWrapper() instead and the
	 * overrides would never execute. */
	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(static_cast<CodedVideoSourceWrapper *>(source)
				   ->getCodedVideoSource());
	CU_ASSERT_PTR_NULL(static_cast<CodedVideoSinkWrapper *>(sink)
				   ->getCodedVideoSink());

	sinkOwner.reset();
	sourceOwner.reset();
}


static struct mbuf_coded_video_frame *
makeDummyCodedFrameEx(enum vdef_encoding encoding,
		      uint64_t timestamp,
		      unsigned int bitDepth,
		      bool fullRange)
{
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format.encoding = encoding;
	frameInfo.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.bit_depth = bitDepth;
	frameInfo.info.full_range = fullRange;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(16, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);

	struct vdef_nalu nalu = {};
	nalu.size = capacity;
	if (encoding == VDEF_ENCODING_H264)
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	else if (encoding == VDEF_ENCODING_H265)
		nalu.h265.type = H265_NALU_TYPE_IDR_W_RADL;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxCodedSourceRejectsBitDepthOrRangeMismatch()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 1);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	struct mbuf_coded_video_frame_queue *inQueue = nullptr;
	struct mbuf_coded_video_frame_queue *outQueue = nullptr;
	createCodedSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* 1. A valid frame is accepted */
	int ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* We need to get the media's expected bit_depth and full_range.
	 * Since H.264 parser resolved it, we can inspect
	 * sourceParams.video.coded.info: */
	unsigned int expectedBitDepth = sourceParams.video.coded.info.bit_depth;
	bool expectedFullRange = sourceParams.video.coded.info.full_range;

	/* 2. Frame with bit_depth mismatch is rejected */
	struct mbuf_coded_video_frame *badBitDepthFrame =
		makeDummyCodedFrameEx(VDEF_ENCODING_H264,
				      UINT64_C(999999991),
				      expectedBitDepth + 2,
				      expectedFullRange);
	ret = mbuf_coded_video_frame_queue_push(inQueue, badBitDepthFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_coded_video_frame_unref(badBitDepthFrame);

	/* 3. Frame with full_range mismatch is rejected */
	struct mbuf_coded_video_frame *badRangeFrame =
		makeDummyCodedFrameEx(VDEF_ENCODING_H264,
				      UINT64_C(999999992),
				      expectedBitDepth,
				      !expectedFullRange);
	ret = mbuf_coded_video_frame_queue_push(inQueue, badRangeFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_coded_video_frame_unref(badRangeFrame);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxCodedSourceRejectsNonMonotonicTimestamp()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 2);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	struct mbuf_coded_video_frame_queue *inQueue = nullptr;
	struct mbuf_coded_video_frame_queue *outQueue = nullptr;
	createCodedSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* Push frame[0] first (the stream's actual IDR/keyframe): this both
	 * establishes mLastTimestamp AND satisfies ExternalCodedVideoSink's
	 * mNeedSync, so no synthetic grey-IDR filler frame gets generated
	 * (see writeGreyIdr() in pdraw_external_coded_video_sink.cpp, which
	 * would otherwise inject an extra frame ahead of the first
	 * non-IDR frame ever delivered to a fresh sink -- confirmed by an
	 * earlier version of this test that pushed frame[1] first and
	 * observed 2 frames at the sink instead of 1). */
	int ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...then frame[0] again: its timestamp now equals mLastTimestamp,
	 * rejected as non-strictly-monotonic (the check is "<=", so an equal
	 * timestamp is rejected too, not just an earlier one). */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxCodedSourceQueueMaxCountDropsOldest()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 4);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);
	sourceParams.queue_max_count = 2;

	IPdraw::ICodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	struct mbuf_coded_video_frame_queue *inQueue = nullptr;
	struct mbuf_coded_video_frame_queue *outQueue = nullptr;
	createCodedSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* Push frame[0] (the stream's IDR) alone first and drain it, so the
	 * sink's mNeedSync is satisfied before the eviction burst below --
	 * otherwise the first non-IDR frame ever delivered to the sink would
	 * trigger a synthetic grey-IDR filler frame (see writeGreyIdr() in
	 * pdraw_external_coded_video_sink.cpp), adding an extra frame that
	 * has nothing to do with what this test is actually checking. */
	int ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::vector<struct mbuf_coded_video_frame *> primerFrames;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				primerFrames.push_back(f);
			return !primerFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	for (auto *f : primerFrames)
		mbuf_coded_video_frame_unref(f);

	/* Now push frame[1..3] back-to-back with no pump() in between: the
	 * source's own queue (max_frames == queue_max_count == 2) must
	 * silently drop frame[1] (the oldest of this burst) when frame[3] is
	 * pushed, before anything is ever forwarded to the sink. */
	for (int i = 1; i <= 3; i++) {
		ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[i]);
		CU_ASSERT_EQUAL(ret, 0);
	}

	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= 2;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrames);
	CU_ASSERT_EQUAL_FATAL(outputFrames.size(), 2u);

	/* The 2 survivors must be frame[2] and frame[3] (by index), NOT
	 * frame[1]: confirms the OLDEST of the burst was dropped, not an
	 * arbitrary one. */
	for (auto *f : outputFrames) {
		struct vdef_coded_frame info = {};
		int infoRet = mbuf_coded_video_frame_get_frame_info(f, &info);
		CU_ASSERT_EQUAL(infoRet, 0);
		CU_ASSERT_TRUE(info.info.index == 2 || info.info.index == 3);
		mbuf_coded_video_frame_unref(f);
	}

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── flush() vs drain(): what happens to frames still queued in the
 * source's own input queue (inQueue) when neither has ever been forwarded
 * by a loop pump. Both push a burst of frames with NO pump() in between,
 * then call flush()/drain() directly: ExternalCodedVideoSource::flush()
 * (pdraw_external_coded_video_source.cpp) only calls process() -- which
 * forwards inQueue's contents to the sink -- for discard=false (drain());
 * discard=true (flush()) skips straight to discarding inQueue's contents
 * outright. Uses DrainTrackingCodedVideoSourceListener/
 * QueueDrainingCodedVideoSinkListener directly (not the
 * createCodedSourceAndSink() helper, which hardcodes the no-op stub
 * listeners) so mGotFlushed/mGotDrained can actually be observed. ──────── */

static void testCxxCodedSourceFlushDiscardsUnprocessedFrames()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 4);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	DrainTrackingCodedVideoSourceListener sourceListener;
	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Prime: push frame[0] (the genuine IDR) alone and let it fully
	 * arrive, so the sink's mNeedSync is already satisfied before the
	 * unpumped burst below -- same rationale as
	 * testCxxCodedSourceQueueMaxCountDropsOldest above. */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::vector<struct mbuf_coded_video_frame *> primerFrames;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				primerFrames.push_back(f);
			return !primerFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	for (auto *f : primerFrames)
		mbuf_coded_video_frame_unref(f);

	/* Push frame[1..3] back-to-back with NO pump() in between: they sit
	 * unprocessed in inQueue, never having had a chance to be forwarded
	 * by the source's own queue-attached loop callback. */
	for (int i = 1; i <= 3; i++) {
		ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[i]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	/* Nothing made it to the sink: all 3 queued frames were discarded
	 * outright, never forwarded. */
	struct mbuf_coded_video_frame *leftover = nullptr;
	CU_ASSERT_NOT_EQUAL(
		mbuf_coded_video_frame_queue_pop(outQueue, &leftover), 0);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxCodedSourceDrainForwardsUnprocessedFrames()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 4);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	DrainTrackingCodedVideoSourceListener sourceListener;
	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Same primer as above. */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::vector<struct mbuf_coded_video_frame *> primerFrames;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				primerFrames.push_back(f);
			return !primerFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	for (auto *f : primerFrames)
		mbuf_coded_video_frame_unref(f);

	/* Same unpumped burst as above. */
	for (int i = 1; i <= 3; i++) {
		ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[i]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	/* drain(discard=false) calls process() synchronously, right there in
	 * the call: by the time drain() returns, frame[1..3] have already
	 * been forwarded all the way to the sink's queue (the source-to-sink
	 * hop through the channel is a plain synchronous call once
	 * process() runs -- the loop pump is only needed to trigger
	 * process() in the first place when frames arrive via the normal
	 * queue-attached-to-loop path, not to move data that process() has
	 * already forwarded). outputChannel->drain() is called AFTER
	 * process(), so by the time onCodedVideoSinkDrain() fires on
	 * sinkListener, the 3 frames are already sitting in outQueue --
	 * checked via sinkListener.mDrainedFrames (populated by that very
	 * callback), NOT by popping outQueue ourselves afterward, since the
	 * listener itself must pop the queue as part of draining it (see
	 * onCodedVideoSinkDrain()'s doc comment: "the application must drain
	 * the sink queue"). */
	ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	CU_ASSERT_EQUAL_FATAL(sinkListener.mDrainedFrames.size(), 3u);
	for (auto *frame : sinkListener.mDrainedFrames)
		mbuf_coded_video_frame_unref(frame);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Exercises ExternalCodedVideoSink::flushDone()'s mFlushDiscard mismatch path
 * (pdraw_external_coded_video_sink.cpp:333-338): the application acknowledges
 * a real flush (mFlushDiscard==true, set when the source called flush()) by
 * calling queueDrained() instead of queueFlushed() -- see
 * MismatchAckCodedVideoSinkListener::onCodedVideoSinkFlush() above. The
 * mismatch only logs a warning; flushDone()'s actual channel operation still
 * follows mFlushDiscard, not the discard argument the application passed, so
 * channel->flushDone() runs (matching the flush the source actually
 * requested, since only Channel::mFlushPending -- not mDrainPending -- was
 * ever set) and the source still receives onCodedVideoSourceFlushed(), never
 * onCodedVideoSourceDrained().
 *
 * Two preconditions, both confirmed missing by real runs before this test
 * could ever reach onCodedVideoSinkFlush():
 * 1. The SOURCE's own frame queue must be non-empty when flush() is called:
 *    per ExternalCodedVideoSource::flush(), an empty queue takes an early
 *    "already flushed, nothing to do" shortcut that completes its own flush
 *    via an idle callback without ever calling outputChannel->flush() --
 *    the sink never even receives the downstream FLUSH event.
 * 2. The SINK's own FlushingState must already be UNFLUSHED (not its default
 *    FLUSHED) when the channel's FLUSH event arrives: ExternalCodedVideoSink::
 *    flush() has the exact same "already flushed, nothing to do" shortcut,
 *    and it stays in FLUSHED (the default, per Element's constructor) until
 *    it has actually received a frame at least once (onCodedVideoChannelQueue()
 *    is the only place that flips it to UNFLUSHED, pdraw_external_coded_
 *    video_sink.cpp:907) -- a source-side discard, which never forwards
 *    anything, does not do this. Confirmed by a real run: even with
 *    precondition 1 fixed, the sink logged "video sink is already flushed,
 *    nothing to do" and never called onCodedVideoSinkFlush().
 *
 * So: prime the sink first with a real frame that is pumped all the way
 * through (exactly like testCxxCodedSourceFlushDiscardsUnprocessedFrames'
 * "primer", reused here for a different reason -- there it is about
 * mNeedSync/grey-IDR synthesis, here it is about FlushingState), THEN push a
 * second frame unpumped into the source's own queue before calling flush(). */
static void testCxxCodedVideoSinkFlushAckMismatchFollowsRequestedDiscardState()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 2);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	DrainTrackingCodedVideoSourceListener sourceListener;
	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	MismatchAckCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	bool gotSinkMediaAdded = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	/* Prime: push ctx.frames[0] (the genuine IDR) and let it fully arrive
	 * at the sink -- this alone flips the sink's FlushingState to UNFLUSHED
	 * (precondition 2 above), and that state persists regardless of the pop
	 * below (nothing resets it back to FLUSHED before flush() is called).
	 */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	struct mbuf_coded_video_frame *primerFrame = nullptr;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			return mbuf_coded_video_frame_queue_pop(
				       outQueue, &primerFrame) == 0;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	mbuf_coded_video_frame_unref(primerFrame);

	/* Unpumped push: sits in the source's own queue so flush() below finds
	 * it non-empty (precondition 1 above). */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[1]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Real flush (discard=true): sets mFlushDiscard=true on the sink. */
	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* onCodedVideoSinkFlush() fires and acks with queueDrained() instead
	 * of queueFlushed() -- yet the source still sees its flush complete,
	 * not a drain. */
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);
	CU_ASSERT_FALSE(sourceListener.mGotDrained);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── ExternalCodedVideoSink::writeGreyIdr() (H.264 grey-IDR synthesis) ────
 * Every other H.264 test in this file deliberately AVOIDS this path by
 * pushing the real IDR first (mNeedSync defaults to true on a fresh sink,
 * so pushing a non-IDR frame first synthesizes a filler grey IDR ahead of
 * it -- see the "primer" comments throughout this file). These two tests
 * deliberately trigger it instead, to exercise writeGreyIdr() itself for
 * real (~230 lines, real H.264 bitstream synthesis via h264_reader/
 * h264_write_grey_i_slice using the media's actual SPS/PPS). ──────────── */

static void testCxxCodedSourceSinkFirstNonIdrFrameSynthesizesGreyIdr()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 2);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	struct mbuf_coded_video_frame_queue *inQueue = nullptr;
	struct mbuf_coded_video_frame_queue *outQueue = nullptr;
	createCodedSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* Push a NON-IDR frame as the very first frame this fresh sink ever
	 * receives. */
	int ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[1]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= 2;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrames);
	CU_ASSERT_EQUAL_FATAL(outputFrames.size(), 2u);

	/* Frame 0: the synthesized grey IDR -- IDR type, SILENT flag, a
	 * single synthetic H264_NALU_TYPE_SLICE_IDR NALU (never read from
	 * the input file). */
	struct vdef_coded_frame greyInfo = {};
	ret = mbuf_coded_video_frame_get_frame_info(outputFrames[0], &greyInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(greyInfo.type, VDEF_CODED_FRAME_TYPE_IDR);
	CU_ASSERT_TRUE((greyInfo.info.flags & VDEF_FRAME_FLAG_SILENT) != 0);
	int greyNaluCount =
		mbuf_coded_video_frame_get_nalu_count(outputFrames[0]);
	CU_ASSERT_EQUAL_FATAL(greyNaluCount, 1);
	const void *greyNaluData = nullptr;
	struct vdef_nalu greyNalu = {};
	ret = mbuf_coded_video_frame_get_nalu(
		outputFrames[0], 0, &greyNaluData, &greyNalu);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(greyNalu.h264.type, H264_NALU_TYPE_SLICE_IDR);
	ret = mbuf_coded_video_frame_release_nalu(
		outputFrames[0], 0, greyNaluData);
	CU_ASSERT_EQUAL(ret, 0);

	/* Frame 1: frame[1] itself, forwarded right after -- its own type
	 * (non-IDR) is untouched by grey-IDR synthesis, only its timestamp
	 * metadata is nudged (see writeGreyIdr()'s *Delta out-params). */
	struct vdef_coded_frame realInfo = {};
	ret = mbuf_coded_video_frame_get_frame_info(outputFrames[1], &realInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(realInfo.info.index, 1u);

	for (auto *f : outputFrames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxCodedSourceSinkResyncForcesFreshGreyIdr()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 3);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Prime with the real IDR: satisfies mNeedSync, matching every other
	 * test in this file. */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[0]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::vector<struct mbuf_coded_video_frame *> primerFrames;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				primerFrames.push_back(f);
			return !primerFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	CU_ASSERT_EQUAL_FATAL(primerFrames.size(), 1u);
	for (auto *f : primerFrames)
		mbuf_coded_video_frame_unref(f);

	/* resync(): forces mNeedSync back to true, as if the application had
	 * detected a decoding error and asked for a fresh sync frame. Its
	 * internal self-flush (ExternalCodedVideoSink::flush(), unrelated to
	 * the channel-level flush/drain tested elsewhere in this file) is
	 * dispatched asynchronously via an idle callback -- wait for
	 * mGotFlushCallback before pushing the next frame, so that self-
	 * flush's discardQueue() call can never race with (and discard) the
	 * frames pushed below. */
	ret = sink->resync();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotFlushCallback = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotFlushCallback; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotFlushCallback);

	/* Push a non-IDR frame: mNeedSync is true again, so this must
	 * trigger a fresh writeGreyIdr() synthesis, exactly like the very
	 * first frame ever delivered to a fresh sink. */
	ret = mbuf_coded_video_frame_queue_push(inQueue, ctx.frames[2]);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	std::vector<struct mbuf_coded_video_frame *> outputFrames;
	bool gotFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= 2;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrames);
	CU_ASSERT_EQUAL_FATAL(outputFrames.size(), 2u);

	struct vdef_coded_frame greyInfo = {};
	ret = mbuf_coded_video_frame_get_frame_info(outputFrames[0], &greyInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(greyInfo.type, VDEF_CODED_FRAME_TYPE_IDR);
	CU_ASSERT_TRUE((greyInfo.info.flags & VDEF_FRAME_FLAG_SILENT) != 0);

	struct vdef_coded_frame realInfo = {};
	ret = mbuf_coded_video_frame_get_frame_info(outputFrames[1], &realInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(realInfo.info.index, 2u);

	for (auto *f : outputFrames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── setSessionMetadata()/getSessionMetadata() propagation ────────────────
 * Neither is ever called anywhere else in this file. setSessionMetadata()
 * propagates downstream via Channel::DownstreamEvent::SESSION_META_UPDATE,
 * exercising ExternalCodedVideoSink::onChannelSessionMetaUpdate() (also
 * never exercised elsewhere) -- no frame push needed, this is purely a
 * channel-level event independent of any frame flow. ───────────────────── */

static void testCxxCodedSourceSetSessionMetadataPropagatesToSink()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 1);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* getSessionMetadata() reflects what fillCodedSourceParams() set. */
	struct vmeta_session initialMeta = {};
	ret = source->getSessionMetadata(&initialMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(initialMeta.friendly_name,
			       "pdraw_test_pipeline_sourcesink_coded");

	struct vmeta_session newMeta = {};
	snprintf(newMeta.friendly_name,
		 sizeof(newMeta.friendly_name),
		 "updated_friendly_name");
	ret = source->setSessionMetadata(&newMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotUpdate = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotSessionMetaUpdate;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotUpdate);
	CU_ASSERT_STRING_EQUAL(sinkListener.mLastSessionMeta.friendly_name,
			       "updated_friendly_name");

	/* getSessionMetadata() reflects the update too. */
	struct vmeta_session updatedMeta = {};
	ret = source->getSessionMetadata(&updatedMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(updatedMeta.friendly_name,
			       "updated_friendly_name");

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies setMediaId() / getMediaId() on ICodedVideoSink by creating two
 * coded video sources, attaching the sink to source1, then switching it to
 * source2 mid-stream -- the ICodedVideoSink counterpart of
 * testCxxVideoRendererSwitchMediaId (test_pipeline_renderer_video.cpp).
 *
 * Why this test exists: every other sink in this file is created either
 * with media_id=0 (auto-attach to whatever media turns up next) or with an
 * already-existing media's id at creation time -- both cases are resolved
 * through the SAME broadcast+filter path (Session::onElementStateChanged /
 * PipelineFactory::onOutputMediaAdded -> addAllMediaToCodedVideoSink() /
 * addMediaToAllToCodedVideoSinks(), which iterate every element and let
 * ExternalCodedVideoSink::addInputMedia() filter by mTargetMediaId). Neither
 * ever calls Session::addMediaToCodedVideoSink(unsigned int mediaId,
 * Sink*)/PipelineFactory's same-named overload -- the single-lookup-by-id
 * path -- which per the coverage report is the only 0%-covered overload of
 * the four. The ONLY caller of that path is
 * ExternalCodedVideoSink::idleRenewMedia(), itself only scheduled by
 * setMediaId(). Confirmed by reading pdraw_external_coded_video_sink.cpp:
 * setMediaId(m2) sets mTargetMediaId=m2 and schedules idleRenewMedia() on
 * the pomp loop; when that idle fires: removeInputMedia(source1's media) ->
 * mSession->addMediaToCodedVideoSink(m2, this) -> addInputMedia(source2's
 * media). Both onCodedVideoSinkMediaRemoved and onCodedVideoSinkMediaAdded
 * fire synchronously within that same idle callback. */
static void testCxxCodedSourceSinkSwitchMediaId()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	/* --- Source 1 and its media --- */
	IPdraw::ICodedVideoSource *source1 = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source1);
	auto source1Owner = std::unique_ptr<IPdraw::ICodedVideoSource>(source1);
	mediaListener.mSource = source1;

	bool gotMedia1 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia1);
	unsigned int media1Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, 0u);

	/* --- Source 2 and its media --- */
	mediaListener.mGotMediaAdded = false;
	IPdraw::ICodedVideoSource *source2 = nullptr;
	ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::ICodedVideoSource>(source2);
	mediaListener.mSource = source2;

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media2Id, 0u);
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, media2Id);

	/* --- Sink on source1's media --- */
	struct pdraw_video_sink_params sinkParams = {};
	CodedVideoSinkTrackingListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		media1Id, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	bool gotSinkMedia1 = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMedia1);
	CU_ASSERT_EQUAL(sink->getMediaId(), media1Id);

	/* --- Switch the sink to source2 --- */
	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	ret = sink->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* idleRenewMedia() fires as an idle callback: removeInputMedia (fires
	 * onCodedVideoSinkMediaRemoved) then
	 * Session::addMediaToCodedVideoSink(media2Id, this) -> addInputMedia
	 * (fires onCodedVideoSinkMediaAdded) happen within that same idle
	 * iteration. */
	bool gotRemoved = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaRemoved; });
	CU_ASSERT_TRUE_FATAL(gotRemoved);
	/* setMediaId() does not force a resync, so restart=false. */
	CU_ASSERT_FALSE(sinkListener.mLastRemovedRestart);

	bool gotAdded2 = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotAdded2);
	CU_ASSERT_EQUAL(sink->getMediaId(), media2Id);

	sinkOwner.reset();
	source1Owner.reset();
	source2Owner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies ExternalCodedVideoSink::setMediaId()'s early-return branch
 * (pdraw_external_coded_video_sink.cpp:230-231): calling setMediaId() with
 * the SAME id as the sink's current mTargetMediaId returns 0 immediately
 * without scheduling idleRenewMedia() -- unlike
 * testCxxCodedSourceSinkSwitchMediaId
 * above (different id), which does trigger a media remove+re-add. */
static void testCxxCodedVideoSinkSetMediaIdSameIdIsNoop()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);

	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	CodedVideoSinkTrackingListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	bool gotSinkMediaAdded = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

	unsigned int mediaId = sink->getMediaId();
	CU_ASSERT_NOT_EQUAL_FATAL(mediaId, 0u);

	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	ret = sink->setMediaId(mediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* No idleRenewMedia() is scheduled: pump for a short, bounded time and
	 * confirm neither callback fires (contrast with
	 * testCxxCodedSourceSinkSwitchMediaId, where switching to a different
	 * id always fires both). */
	bool gotEvent = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotMediaRemoved ||
			       sinkListener.mGotMediaAdded;
		},
		300);
	CU_ASSERT_FALSE(gotEvent);
	CU_ASSERT_EQUAL(sink->getMediaId(), mediaId);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Regression test for two real use-after-free bugs found via ASan while
 * investigating whether ExternalCodedVideoSink::~ExternalCodedVideoSink()'s
 * "input media has not been removed" branch
 * (pdraw_external_coded_video_sink.cpp:117-125) was reachable/testable. It
 * turned out only reachable by destroying the whole Session abruptly (skipping
 * stop()) while a source and sink are both still fully wired up --
 * Session::~Session()'s forced mElements.clear() (pdraw_session.cpp:284-291) is
 * the only path where an Element gets destroyed with its media/channels still
 * attached. That scenario crashed twice before being fixed:
 * 1. ExternalCodedVideoSource::~ExternalCodedVideoSource() freed its
 *    mOutputMedia while a channel was still attached to it, then
 *    ~Source()'s own removeOutputPorts() walked that already-freed media
 *    pointer (fixed: the destructor now calls Source::teardownOutputChannels()
 *    before removeOutputPort(), see pdraw_source.cpp).
 * 2. Once #1 was fixed, the still-attached Sink (destroyed right after, in
 *    the same mElements.clear() sweep) crashed in turn: its own mInputMedia
 *    pointed at the same Media object the Source had just freed, since
 *    nothing had told the Sink its media was gone (fixed: the same
 *    teardownOutputChannels() now synchronously runs the channel->teardown()
 *    round-trip -- Sink::onChannelTeardown() -> removeInputMedia(), while
 *    the media is still valid -- before the Source frees it).
 *
 * As a consequence of fixing #2 properly, the sink's own "input media has
 * not been removed" branch is no longer reachable even in this abrupt
 * scenario (its mInputMedia is already null by the time its own destructor
 * runs) -- this test therefore does not target that branch specifically;
 * it targets Source::teardownOutputChannels() and confirms the abrupt
 * teardown no longer crashes, with the sink properly notified of the
 * removal along the way. */
static void testCxxCodedSourceSinkAbruptSessionDestructionDoesNotCrash()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);

	SourceMediaListener mediaListener;
	CodedVideoSinkTrackingListener sinkListener;
	std::unique_ptr<IPdraw::ICodedVideoSource> sourceOwner;
	std::unique_ptr<IPdraw::ICodedVideoSink> sinkOwner;

	{
		TestPompLoop loop;
		TestSession testSession(&loop, &mediaListener);
		IPdraw *session = testSession.get();

		struct pdraw_video_source_params sourceParams;
		fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &sourceParams);

		IPdraw::ICodedVideoSource *source = nullptr;
		int ret = session->createCodedVideoSource(
			&sourceParams,
			&g_stub_coded_video_source_listener,
			&source);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(source);
		sourceOwner =
			std::unique_ptr<IPdraw::ICodedVideoSource>(source);
		mediaListener.mSource = source;

		bool gotMediaAdded = loop.pumpUntil([&mediaListener]() {
			return mediaListener.mGotMediaAdded;
		});
		CU_ASSERT_TRUE_FATAL(gotMediaAdded);

		struct pdraw_video_sink_params sinkParams = {};
		IPdraw::ICodedVideoSink *sink = nullptr;
		ret = session->createCodedVideoSink(mediaListener.mMediaId,
						    &sinkParams,
						    &sinkListener,
						    &sink);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
		sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

		bool gotSinkMediaAdded = loop.pumpUntil([&sinkListener]() {
			return sinkListener.mGotMediaAdded;
		});
		CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

		/* Deliberately no stop()/stopSessionAndWait() call here:
		 * testSession and loop are destroyed right below, at the end
		 * of this block, forcing ~Session() to run while mState is
		 * still READY -- the only way to reach the scenario described
		 * above. */
	}

	/* If we get here, the abrupt teardown did not crash. The sink was
	 * synchronously notified of its media's removal as part of the
	 * source's own destructor (Source::teardownOutputChannels() ->
	 * channel->teardown() -> Sink::onChannelTeardown() ->
	 * removeInputMedia(), all before the source frees the media) --
	 * confirming the fix actually ran, not just that nothing crashed. */
	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
}


/* ── C-wrapper listener callback test (C API) ───
 * Self-contained pdraw_new() fixture with counting C callbacks.
 * SessionCbState / make_pdraw() are shared via test_pipeline_common.hpp. */

struct CodedSrcCbState {
	int flushedCount = 0;
	int drainedCount = 0;
};

static void coded_src_flushed_cb(struct pdraw * /*p*/,
				 struct pdraw_coded_video_source * /*s*/,
				 void *ud)
{
	static_cast<CodedSrcCbState *>(ud)->flushedCount++;
}

static void coded_src_drained_cb(struct pdraw * /*p*/,
				 struct pdraw_coded_video_source * /*s*/,
				 void *ud)
{
	static_cast<CodedSrcCbState *>(ud)->drainedCount++;
}


static void testCCodedVideoSourceListenerFlushedDrained()
{
	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	struct pdraw_coded_video_source_cbs cbs = {};
	cbs.flushed = coded_src_flushed_cb;
	cbs.drained = coded_src_drained_cb;
	CodedSrcCbState srcState;

	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	struct pdraw_coded_video_source *src = nullptr;
	int ret =
		pdraw_coded_video_source_new(p, &params, &cbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	ret = pdraw_coded_video_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&srcState]() { return srcState.flushedCount >= 1; });
	CU_ASSERT_TRUE(gotFlushed);
	CU_ASSERT_EQUAL(srcState.flushedCount, 1);

	ret = pdraw_coded_video_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&srcState]() { return srcState.drainedCount >= 1; });
	CU_ASSERT_TRUE(gotDrained);
	CU_ASSERT_EQUAL(srcState.drainedCount, 1);

	pdraw_coded_video_source_destroy(p, src);
	pdraw_destroy(p);
}


/* ── C-wrapper listener callback tests (C API) ── */

struct CodedSinkCbState {
	int mediaAddedCount = 0;
	int mediaRemovedCount = 0;
	int flushCount = 0;
	int drainCount = 0;
	int sessionMetaCount = 0;
	/* Back-pointers set before first callback arrives so the ACK callbacks
	 * (queue_flushed / queue_drained) can be issued inside flush/drain. */
	struct pdraw *p = nullptr;
	struct pdraw_coded_video_sink *snk = nullptr;
};

static void coded_sink_media_added_cb(struct pdraw * /*p*/,
				      struct pdraw_coded_video_sink * /*s*/,
				      const struct pdraw_media_info * /*info*/,
				      void *ud)
{
	static_cast<CodedSinkCbState *>(ud)->mediaAddedCount++;
}

static void
coded_sink_media_removed_cb(struct pdraw * /*p*/,
			    struct pdraw_coded_video_sink * /*s*/,
			    const struct pdraw_media_info * /*info*/,
			    int /*restart*/,
			    void *ud)
{
	static_cast<CodedSinkCbState *>(ud)->mediaRemovedCount++;
}

static void coded_sink_flush_cb(struct pdraw *p,
				struct pdraw_coded_video_sink *snk,
				void *ud)
{
	auto *s = static_cast<CodedSinkCbState *>(ud);
	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_sink_get_queue(p, snk);
	if (q)
		mbuf_coded_video_frame_queue_flush(q);
	pdraw_coded_video_sink_queue_flushed(p, snk);
	s->flushCount++;
}

static void coded_sink_drain_cb(struct pdraw *p,
				struct pdraw_coded_video_sink *snk,
				void *ud)
{
	auto *s = static_cast<CodedSinkCbState *>(ud);
	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_sink_get_queue(p, snk);
	if (q) {
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(q, &f) == 0)
			mbuf_coded_video_frame_unref(f);
	}
	pdraw_coded_video_sink_queue_drained(p, snk);
	s->drainCount++;
}

static void coded_sink_session_meta_cb(struct pdraw * /*p*/,
				       struct pdraw_coded_video_sink * /*s*/,
				       const struct vmeta_session * /*meta*/,
				       void *ud)
{
	static_cast<CodedSinkCbState *>(ud)->sessionMetaCount++;
}

/* Covers PdrawCodedVideoSinkListener: media_added, flush+ACK, drain+ACK,
 * session_metadata_update, media_removed — all via C API wrappers. */
static void testCCodedVideoSinkListenerCallbacks()
{
	struct ParseContext ctx;
	parseCodedFile(VDEF_ENCODING_H264, ASSET_H264, &ctx);
	CU_ASSERT_FATAL(ctx.frames.size() >= 2);

	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	/* Create coded video source with real H264 SPS/PPS so the source
	 * produces a properly formatted coded media (required for the input
	 * filter to accept pushed frames). */
	struct pdraw_video_source_params srcParams;
	fillCodedSourceParams(VDEF_ENCODING_H264, ctx, &srcParams);
	CodedSrcCbState srcCbState;
	struct pdraw_coded_video_source_cbs srcCbs = {};
	srcCbs.flushed = coded_src_flushed_cb;
	srcCbs.drained = coded_src_drained_cb;
	struct pdraw_coded_video_source *src = nullptr;
	int ret = pdraw_coded_video_source_new(
		p, &srcParams, &srcCbs, &srcCbState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	bool gotMedia = loop.pumpUntil(
		[&sessState]() { return sessState.lastMediaId != 0; });
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Create coded video sink on that media */
	CodedSinkCbState sinkState;
	sinkState.p = p;
	struct pdraw_video_sink_params sinkParams = {};
	struct pdraw_coded_video_sink_cbs sinkCbs = {};
	sinkCbs.media_added = coded_sink_media_added_cb;
	sinkCbs.media_removed = coded_sink_media_removed_cb;
	sinkCbs.flush = coded_sink_flush_cb;
	sinkCbs.drain = coded_sink_drain_cb;
	sinkCbs.session_metadata_update = coded_sink_session_meta_cb;
	struct pdraw_coded_video_sink *snk = nullptr;
	ret = pdraw_coded_video_sink_new(p,
					 sessState.lastMediaId,
					 &sinkParams,
					 &sinkCbs,
					 &sinkState,
					 &snk);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(snk);
	sinkState.snk = snk;

	bool gotSinkAdded = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaAddedCount >= 1; });
	CU_ASSERT_TRUE(gotSinkAdded);
	CU_ASSERT_EQUAL(sinkState.mediaAddedCount, 1);

	struct mbuf_coded_video_frame_queue *inQ =
		pdraw_coded_video_source_get_queue(p, src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);

	/* Flush: push IDR frame (ctx.frames[0]) → pump once to deliver it to
	 * the sink's input queue → flush source → sink flush callback + ACK. */
	ret = mbuf_coded_video_frame_queue_push(inQ, ctx.frames[0]);
	CU_ASSERT_EQUAL(ret, 0);
	loop.runOnce(); /* source processFrame → frame in sink input queue */
	ret = pdraw_coded_video_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlush = loop.pumpUntil([&sinkState, &srcCbState]() {
		return sinkState.flushCount >= 1 &&
		       srcCbState.flushedCount >= 1;
	});
	CU_ASSERT_TRUE(gotFlush);
	CU_ASSERT_EQUAL(sinkState.flushCount, 1);
	CU_ASSERT_EQUAL(srcCbState.flushedCount, 1);

	/* Drain: push frame[1] → drain source (internally calls processFrame
	 * before channel drain) → sink drain callback + ACK. */
	ret = mbuf_coded_video_frame_queue_push(inQ, ctx.frames[1]);
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_coded_video_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrain = loop.pumpUntil([&sinkState, &srcCbState]() {
		return sinkState.drainCount >= 1 &&
		       srcCbState.drainedCount >= 1;
	});
	CU_ASSERT_TRUE(gotDrain);
	CU_ASSERT_EQUAL(sinkState.drainCount, 1);
	CU_ASSERT_EQUAL(srcCbState.drainedCount, 1);

	/* session_metadata_update: set metadata on source → propagates to sink
	 */
	struct vmeta_session meta = {};
	snprintf(meta.friendly_name,
		 sizeof(meta.friendly_name),
		 "pdraw_test_coded_sink_meta");
	ret = pdraw_coded_video_source_set_session_metadata(p, src, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMeta = loop.pumpUntil(
		[&sinkState]() { return sinkState.sessionMetaCount >= 1; });
	CU_ASSERT_TRUE(gotMeta);
	CU_ASSERT_EQUAL(sinkState.sessionMetaCount, 1);

	/* media_removed: fires when source is destroyed */
	pdraw_coded_video_source_destroy(p, src);
	src = nullptr;
	bool gotRemoved = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaRemovedCount >= 1; });
	CU_ASSERT_TRUE(gotRemoved);
	CU_ASSERT_EQUAL(sinkState.mediaRemovedCount, 1);

	pdraw_coded_video_sink_destroy(p, snk);
	pdraw_destroy(p);
}


static void testCxxCodedVideoSourceWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* MJPEG encoding: start() skips the SPS/PPS setPs() path (default:
	 * break in ExternalCodedVideoSource::start()), so no bitstream headers
	 * are needed and the source element starts immediately. */
	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format.encoding = VDEF_ENCODING_MJPEG;
	sourceParams.video.coded.format.data_format =
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	sourceParams.video.coded.info.resolution.width = 640;
	sourceParams.video.coded.info.resolution.height = 480;
	sourceParams.video.coded.info.framerate.num = 30;
	sourceParams.video.coded.info.framerate.den = 1;

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(source->getQueue());
	CU_ASSERT_EQUAL(source->flush(), -EPROTO);
	CU_ASSERT_EQUAL(source->drain(), -EPROTO);
	struct vmeta_session meta = {};
	CU_ASSERT_EQUAL(source->setSessionMetadata(&meta), -EPROTO);
	CU_ASSERT_EQUAL(source->getSessionMetadata(&meta), -EPROTO);
}


static void testCxxCodedVideoSinkWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	SourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format.encoding = VDEF_ENCODING_MJPEG;
	sourceParams.video.coded.format.data_format =
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	sourceParams.video.coded.info.resolution.width = 640;
	sourceParams.video.coded.info.resolution.height = 480;
	sourceParams.video.coded.info.framerate.num = 30;
	sourceParams.video.coded.info.framerate.den = 1;

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(mediaListener.mMediaId,
					    &sinkParams,
					    &g_stub_coded_video_sink_listener,
					    &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_EQUAL(sink->setMediaId(0), -EPROTO);
	CU_ASSERT_EQUAL(sink->getMediaId(), (unsigned int)-EPROTO);
	CU_ASSERT_EQUAL(sink->resync(), -EPROTO);
	CU_ASSERT_PTR_NULL(sink->getQueue());
	CU_ASSERT_EQUAL(sink->queueFlushed(), -EPROTO);
	CU_ASSERT_EQUAL(sink->queueDrained(), -EPROTO);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_sourcesink_coded[] = {
	{FN("testCxxCodedSourceSinkRoundtripH264"),
	 testCxxCodedSourceSinkRoundtripH264},
	{FN("testCxxCodedSourceSinkRoundtripH265"),
	 testCxxCodedSourceSinkRoundtripH265},
	{FN("testCxxCodedSourceRejectsFormatMismatch"),
	 testCxxCodedSourceRejectsFormatMismatch},
	{FN("testCxxCodedSourceRejectsBitDepthOrRangeMismatch"),
	 testCxxCodedSourceRejectsBitDepthOrRangeMismatch},
	{FN("testCxxCodedSourceRejectsNonMonotonicTimestamp"),
	 testCxxCodedSourceRejectsNonMonotonicTimestamp},
	{FN("testCxxCodedSourceQueueMaxCountDropsOldest"),
	 testCxxCodedSourceQueueMaxCountDropsOldest},
	{FN("testCxxCodedSourceFlushDiscardsUnprocessedFrames"),
	 testCxxCodedSourceFlushDiscardsUnprocessedFrames},
	{FN("testCxxCodedSourceDrainForwardsUnprocessedFrames"),
	 testCxxCodedSourceDrainForwardsUnprocessedFrames},
	{FN("testCxxCodedVideoSinkFlushAckMismatchFollowsRequestedDiscardState"),
	 testCxxCodedVideoSinkFlushAckMismatchFollowsRequestedDiscardState},
	{FN("testCxxCodedSourceSinkFirstNonIdrFrameSynthesizesGreyIdr"),
	 testCxxCodedSourceSinkFirstNonIdrFrameSynthesizesGreyIdr},
	{FN("testCxxCodedSourceSinkResyncForcesFreshGreyIdr"),
	 testCxxCodedSourceSinkResyncForcesFreshGreyIdr},
	{FN("testCxxCodedSourceSetSessionMetadataPropagatesToSink"),
	 testCxxCodedSourceSetSessionMetadataPropagatesToSink},
	{FN("testCxxCodedSourceSinkSwitchMediaId"),
	 testCxxCodedSourceSinkSwitchMediaId},
	{FN("testCxxCodedVideoSinkSetMediaIdSameIdIsNoop"),
	 testCxxCodedVideoSinkSetMediaIdSameIdIsNoop},
	{FN("testCxxCodedSourceSinkAbruptSessionDestructionDoesNotCrash"),
	 testCxxCodedSourceSinkAbruptSessionDestructionDoesNotCrash},
	{FN("testCCodedVideoSourceListenerFlushedDrained"),
	 testCCodedVideoSourceListenerFlushedDrained},
	{FN("testCCodedVideoSinkListenerCallbacks"),
	 testCCodedVideoSinkListenerCallbacks},
	{FN("testCxxCodedVideoSourceWrapperGuardsAfterElementCleared"),
	 testCxxCodedVideoSourceWrapperGuardsAfterElementCleared},
	{FN("testCxxCodedVideoSinkWrapperGuardsAfterElementCleared"),
	 testCxxCodedVideoSinkWrapperGuardsAfterElementCleared},
	CU_TEST_INFO_NULL,
};
