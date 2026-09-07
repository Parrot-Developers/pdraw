/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — raw video source -> sink roundtrip on real
 * headerless YUV files (Tier B, self-contained fixture)
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

/* Adapted from libpdraw-backend/tests/pdraw_rawsourcesink_test.c, which
 * feeds a raw YUV elementary stream file into an IRawVideoSource and writes
 * back out whatever an IRawVideoSink attached to that same media receives.
 * Much simpler than the coded video counterpart (test_pipeline_sourcesink_
 * coded.cpp): a raw YUV plane buffer has no bitstream structure to parse, so
 * libvideo-raw's vraw_reader/vraw_writer do all the file I/O and per-plane
 * layout bookkeeping (stride, plane count...) instead of a hand-rolled NALU
 * parser.
 *
 * Unlike the coded video source/sink test's inputs, these headerless *.yuv
 * files carry no embedded format description (no SPS/PPS, no y4m text
 * header): resolution/pixel-format/framerate are supplied by the caller
 * (here, hardcoded to match each known asset, mirroring the demo's
 * -f/-W/-H/-F command-line flags).
 *
 * Same two-phase structure and the same ordering rationale as the coded
 * video test: parse the whole input file into a vector of
 * mbuf_raw_video_frame's first (independently of pdraw), then only once a
 * sink exists on the source's media, push every frame and drain the sink's
 * queue -- pushing into the source before a sink is connected would let
 * ExternalRawVideoSource::process() silently drop the frames (no output
 * channel to forward to). */

#define ULOG_TAG pdraw_test_pipeline_sourcesink_raw
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include "pdraw_external_raw_video_sink.hpp"
#include "pdraw_external_raw_video_source.hpp"

#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>
#include <video-raw/vraw.h>

#include <sys/stat.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── Headerless YUV elementary stream fixtures ───────────────────────────
 * (NAS assets, same ASSETS_ROOT/PDRAW_GET_ASSET_PATH mechanism as every
 * other test_pipeline_*.cpp file). Both 214x120 (16:9), 30/1 fps, 100
 * frames; only the pixel format differs. */

enum { ASSET_I420 = 0, ASSET_NV12 = 1 };

static constexpr struct {
	const char *relative_path;
} s_assets_sourcesink_raw[] = {
	{"Tests/miscellaneous/crowd_run_120p50_i420_100frames.yuv"},
	{"Tests/miscellaneous/crowd_run_120p50_nv12_100frames.yuv"},
};

#define SOURCESINK_RAW_WIDTH 214
#define SOURCESINK_RAW_HEIGHT 120
#define SOURCESINK_RAW_FRAMERATE_NUM 30
#define SOURCESINK_RAW_FRAMERATE_DEN 1
#define SOURCESINK_RAW_FRAME_COUNT 100u


/* Session-wide listener: correlates onMediaAdded() with our own source
 * instance via elementUserData, same rationale as the coded video test's
 * SourceMediaListener. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class RawSourceMediaListener : public IPdraw::Listener {
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

	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	unsigned int mMediaId = 0;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Tracks IRawVideoSource::flush()/drain() completion. Same rationale as the
 * coded video test's DrainTrackingCodedVideoSourceListener: used only by the
 * happy-path roundtrip test below, which calls drain() before tearing down
 * the source, matching pdraw_rawsourcesink_test.c's reference lifecycle. */
class DrainTrackingRawVideoSourceListener
		: public IPdraw::IRawVideoSource::Listener {
public:
	void onRawVideoSourceFlushed(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*src*/) override
	{
		mGotFlushed = true;
	}

	void onRawVideoSourceDrained(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*src*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* Implements the sink side of the flush/drain protocol for real, mirroring
 * sink_flush_cb()/sink_drain_cb() in the reference pdraw_rawsourcesink_test.c
 * program: pop (and unref) every frame already sitting in the sink's own
 * queue, then acknowledge via queueFlushed()/queueDrained(). Without this,
 * calling IRawVideoSource::drain() against g_stub_raw_video_sink_listener (a
 * no-op stub, used by every other test in this file) would hang forever --
 * the stub never acknowledges, so Channel::drainDone() never fires and
 * onRawVideoSourceDrained() never fires either. */
class QueueDrainingRawVideoSinkListener
		: public IPdraw::IRawVideoSink::Listener {
public:
	void
	onRawVideoSinkMediaAdded(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/,
				 const struct pdraw_media_info * /*i*/) override
	{
	}

	void onRawVideoSinkMediaRemoved(IPdraw * /*p*/,
					IPdraw::IRawVideoSink * /*sk*/,
					const struct pdraw_media_info * /*i*/,
					bool /*restart*/) override
	{
	}

	void onRawVideoSinkFlush(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		/* Flush really means discard: unref outright. */
		discardQueue();
		sk->queueFlushed();
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		/* Unlike flush, drain must not lose data: whatever is in the
		 * queue at this point may be real frames process() just
		 * forwarded (see ExternalRawVideoSource::flush(discard=
		 * false), which calls process() BEFORE draining the output
		 * channel) -- accumulate into mDrainedFrames instead of
		 * unreffing outright, so a test can inspect what was
		 * delivered. The caller owns the refs afterward. */
		while (true) {
			struct mbuf_raw_video_frame *f = nullptr;
			if (mQueue == nullptr ||
			    mbuf_raw_video_frame_queue_pop(mQueue, &f) != 0)
				break;
			mDrainedFrames.push_back(f);
		}
		sk->queueDrained();
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	/* Must be set right after createRawVideoSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_raw_video_frame_queue *mQueue = nullptr;

	/* Populated by onRawVideoSinkDrain(); empty in the roundtrip tests
	 * (outQueue is already drained by the test itself before drain() is
	 * ever called there), populated for real in
	 * testCxxRawSourceDrainForwardsUnprocessedFrames. Caller must unref
	 * each entry. */
	std::vector<struct mbuf_raw_video_frame *> mDrainedFrames;

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_raw_video_frame *f = nullptr;
		while (mbuf_raw_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_raw_video_frame_unref(f);
	}
};


/* Acknowledges flush/drain with the WRONG companion function on purpose, to
 * exercise ExternalRawVideoSink::flushDone()'s mFlushDiscard mismatch warning
 * path (pdraw_external_raw_video_sink.cpp:287-292) via
 * testCxxRawVideoSinkFlushAckMismatchFollowsRequestedDiscardState below. Only
 * onRawVideoSinkFlush() needs to misbehave for that test; onRawVideoSinkDrain()
 * still acks correctly since it is not exercised there. */
class MismatchAckRawVideoSinkListener : public IPdraw::IRawVideoSink::Listener {
public:
	void
	onRawVideoSinkMediaAdded(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/,
				 const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onRawVideoSinkMediaRemoved(IPdraw * /*p*/,
					IPdraw::IRawVideoSink * /*sk*/,
					const struct pdraw_media_info * /*i*/,
					bool /*restart*/) override
	{
	}

	void onRawVideoSinkFlush(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		/* Wrong ack on purpose: a real flush(discard=true) is
		 * underway (mFlushDiscard==true), but acknowledge as if it
		 * were a drain. */
		sk->queueDrained();
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		sk->queueDrained();
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	bool mGotMediaAdded = false;
};


/* Tracks IRawVideoSink media-added/media-removed callbacks; used by
 * testCxxRawSourceSinkSwitchMediaId below to observe setMediaId()'s effects,
 * and by testCxxRawVideoSinkResolutionChangeSetsRestartFlag/
 * testCxxRawVideoSinkFramerateChangeSetsRestartFlag to observe the "restart"
 * argument of onRawVideoSinkMediaRemoved(). Flush/drain are left as no-ops,
 * like g_stub_raw_video_sink_listener (shared by most other tests in this
 * file): none of those tests ever call flush()/drain()/resync() themselves,
 * so no acknowledgement is required. */
class RawVideoSinkTrackingListener : public IPdraw::IRawVideoSink::Listener {
public:
	void
	onRawVideoSinkMediaAdded(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/,
				 const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onRawVideoSinkMediaRemoved(IPdraw * /*p*/,
					IPdraw::IRawVideoSink * /*sk*/,
					const struct pdraw_media_info * /*i*/,
					bool restart) override
	{
		mGotMediaRemoved = true;
		mLastRemovedRestart = restart;
	}

	void onRawVideoSinkFlush(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/) override
	{
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/) override
	{
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	bool mGotMediaAdded = false;
	bool mGotMediaRemoved = false;
	bool mLastRemovedRestart = false;
};


/* ── File comparison helper (same as the coded video test) ──────────────── */

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

/* Reads the whole input file into *frames (see the file-level comment for
 * why pdraw is not involved yet). Factored out of the roundtrip test so the
 * negative tests below can get a handful of valid frames without
 * duplicating the vraw_reader loop. */
static void collectRawFrames(const struct vdef_raw_format *format,
			     size_t assetIndex,
			     struct vdef_raw_format *resolvedFormat,
			     struct vdef_format_info *resolvedInfo,
			     std::vector<struct mbuf_raw_video_frame *> *frames,
			     std::vector<struct mbuf_mem *> *mems)
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, assetIndex, s_assets_sourcesink_raw);

	struct vraw_reader_config readerConfig = {};
	readerConfig.format = *format;
	readerConfig.info.resolution.width = SOURCESINK_RAW_WIDTH;
	readerConfig.info.resolution.height = SOURCESINK_RAW_HEIGHT;
	readerConfig.info.framerate.num = SOURCESINK_RAW_FRAMERATE_NUM;
	readerConfig.info.framerate.den = SOURCESINK_RAW_FRAMERATE_DEN;

	struct vraw_reader *reader = nullptr;
	int ret = vraw_reader_new(inPath, &readerConfig, &reader);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = vraw_reader_get_config(reader, &readerConfig);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FALSE_FATAL(vdef_dim_is_null(&readerConfig.info.resolution));

	ssize_t frameLenRet = vraw_reader_get_min_buf_size(reader);
	CU_ASSERT_FATAL(frameLenRet > 0);
	size_t frameLen = static_cast<size_t>(frameLenRet);

	while (true) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(frameLen, &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct vraw_frame inFrame = {};
		ret = vraw_reader_frame_read(reader, data, capacity, &inFrame);
		if (ret == -ENOENT) {
			mbuf_mem_unref(mem);
			break;
		}
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&inFrame.frame, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		unsigned int planeCount =
			vdef_get_raw_frame_plane_count(&inFrame.frame.format);
		CU_ASSERT_FATAL(planeCount > 0);
		size_t nextOffset = frameLen;
		for (unsigned int i = planeCount; i-- > 0;) {
			size_t planeOffset =
				static_cast<size_t>(inFrame.data[i] - data);
			size_t planeSize = nextOffset - planeOffset;
			nextOffset = planeOffset;
			ret = mbuf_raw_video_frame_set_plane(
				frame, i, mem, planeOffset, planeSize);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
		}

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		frames->push_back(frame);
		mems->push_back(mem);
	}
	vraw_reader_destroy(reader);

	CU_ASSERT_EQUAL_FATAL(frames->size(), SOURCESINK_RAW_FRAME_COUNT);
	*resolvedFormat = readerConfig.format;
	*resolvedInfo = readerConfig.info;
}


static void fillRawSourceParams(const struct vdef_raw_format &format,
				const struct vdef_format_info &info,
				struct pdraw_video_source_params *sourceParams)
{
	*sourceParams = {};
	sourceParams->queue_max_count = 0; /* unbounded by default */
	sourceParams->playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams->video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams->video.raw.format = format;
	sourceParams->video.raw.info = info;
	snprintf(sourceParams->session_meta.friendly_name,
		 sizeof(sourceParams->session_meta.friendly_name),
		 "pdraw_test_pipeline_sourcesink_raw");
}


/* ── The test itself ──────────────────────────────────────────────────── */

static void runRawSourceSinkRoundtrip(const struct vdef_raw_format *format,
				      size_t assetIndex,
				      const char *outSuffix)
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, assetIndex, s_assets_sourcesink_raw);

	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(format,
			 assetIndex,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);

	/* ── Phase 2: real pdraw pipeline: source -> sink, same session ── */

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	/* Pump now, before pushing any frame: see the coded video test for
	 * why the sink must exist before any frame is pushed. */
	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Now that the sink is connected, push every collected frame. */
	for (auto *frame : inputFrames) {
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL(ret, 0);
	}

	std::vector<struct mbuf_raw_video_frame *> outputFrames;
	bool gotAllFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= inputFrames.size();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAllFrames);
	CU_ASSERT_EQUAL(outputFrames.size(), inputFrames.size());

	/* Write every output frame's planes, in order, to a fresh output
	 * file via vraw_writer (mirrors the demo's process_output()). */
	char outPath[512];
	snprintf(outPath,
		 sizeof(outPath),
		 "/tmp/pdraw_test_pipeline_sourcesink%s",
		 outSuffix);
	struct vraw_writer_config writerConfig = {};
	writerConfig.format = resolvedFormat;
	writerConfig.info = resolvedInfo;
	struct vraw_writer *writer = nullptr;
	ret = vraw_writer_new(outPath, &writerConfig, &writer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	for (auto *frame : outputFrames) {
		struct vdef_raw_frame frameInfo = {};
		ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
		CU_ASSERT_EQUAL(ret, 0);

		unsigned int planeCount =
			vdef_get_raw_frame_plane_count(&frameInfo.format);
		struct vraw_frame outFrame = {};
		outFrame.frame = frameInfo;
		for (unsigned int i = 0; i < planeCount; i++) {
			size_t len = 0;
			ret = mbuf_raw_video_frame_get_plane(
				frame,
				i,
				(const void **)&outFrame.cdata[i],
				&len);
			CU_ASSERT_EQUAL(ret, 0);
		}

		ret = vraw_writer_frame_write(writer, &outFrame);
		CU_ASSERT_EQUAL(ret, 0);

		for (unsigned int i = 0; i < planeCount; i++) {
			ret = mbuf_raw_video_frame_release_plane(
				frame, i, outFrame.cdata[i]);
			CU_ASSERT_EQUAL(ret, 0);
		}
		mbuf_raw_video_frame_unref(frame);
	}
	vraw_writer_destroy(writer);

	/* The whole point: the roundtrip reproduces the input exactly. */
	CU_ASSERT_TRUE(filesAreIdentical(inPath, outPath));
	(void)remove(outPath);

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	/* Drain, THEN flush, the source before tearing down -- in that order,
	 * deliberately: ExternalRawVideoSource::flush(discard=false, i.e.
	 * drain()) calls process() first, forcing anything still sitting in
	 * the source's own input queue (inQueue) to be forwarded downstream
	 * before completing, while flush(discard=true) skips that and
	 * discards whatever is left in inQueue outright. Draining first is
	 * what actually guarantees nothing is lost; flush() afterward is
	 * then safe by construction, not because this test happens to have
	 * already popped everything from outQueue above. */
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


static void testCxxRawSourceSinkRoundtripI420()
{
	runRawSourceSinkRoundtrip(&vdef_i420, ASSET_I420, ".i420.yuv");
}


static void testCxxRawSourceSinkRoundtripNv12()
{
	runRawSourceSinkRoundtrip(&vdef_nv12, ASSET_NV12, ".nv12.yuv");
}


/* ── Negative / edge-case tests: exercise ExternalRawVideoSource::
 * inputFilter() (format intersect check, strictly-increasing timestamp
 * check) and the underlying mbuf queue's max_frames drop-oldest behavior --
 * mirrors the coded video test's negative tests (see there for the
 * ref-counting rationale, identical here). ─────────────────────────────── */

static void
createRawSourceAndSink(IPdraw *session,
		       TestPompLoop *loop,
		       RawSourceMediaListener *mediaListener,
		       const struct pdraw_video_source_params *sourceParams,
		       IPdraw::IRawVideoSource **source,
		       IPdraw::IRawVideoSink **sink,
		       struct mbuf_raw_video_frame_queue **inQueue,
		       struct mbuf_raw_video_frame_queue **outQueue)
{
	int ret = session->createRawVideoSource(
		sourceParams, &g_stub_raw_video_source_listener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);
	mediaListener->mSource = *source;

	bool gotMediaAdded = loop->pumpUntil(
		[mediaListener]() { return mediaListener->mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener->mMediaId, 0u);

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createRawVideoSink(mediaListener->mMediaId,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);
	*outQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outQueue);
}

/* A minimal, validly-finalized frame with placeholder plane content --
 * content is irrelevant, these tests only care about acceptance/rejection
 * at the queue's filter. */
static struct mbuf_raw_video_frame *
makeDummyRawFrame(const struct vdef_raw_format &format, uint64_t timestamp)
{
	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = format;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.resolution.width = SOURCESINK_RAW_WIDTH;
	frameInfo.info.resolution.height = SOURCESINK_RAW_HEIGHT;

	struct mbuf_raw_video_frame *frame = nullptr;
	int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	unsigned int planeCount = vdef_get_raw_frame_plane_count(&format);
	CU_ASSERT_FATAL(planeCount > 0);

	const size_t planeSize = 64;
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(planeSize * planeCount, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	for (unsigned int i = 0; i < planeCount; i++) {
		ret = mbuf_raw_video_frame_set_plane(
			frame, i, mem, i * planeSize, planeSize);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}
	mbuf_mem_unref(mem); /* the frame holds its own ref via set_plane */

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxRawSourceRejectsFormatMismatch()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 1);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	struct mbuf_raw_video_frame_queue *outQueue = nullptr;
	createRawSourceAndSink(session,
			       &loop,
			       &mediaListener,
			       &sourceParams,
			       &source,
			       &sink,
			       &inQueue,
			       &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	/* A valid I420 frame is accepted... */
	int ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...but an NV12 frame is rejected outright (the source was created
	 * for I420), per ExternalRawVideoSource::inputFilter()'s
	 * vdef_raw_format_intersect() check -- never reaches the queue. */
	struct mbuf_raw_video_frame *badFrame =
		makeDummyRawFrame(vdef_nv12, UINT64_C(999999999));
	ret = mbuf_raw_video_frame_queue_push(inQueue, badFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_raw_video_frame_unref(badFrame);

	/* The valid frame still made it through undisturbed. */
	std::vector<struct mbuf_raw_video_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_raw_video_frame_unref(f);

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	/* stopSessionAndWait() must run BEFORE the owners are reset, so that
	 * Session::asyncElementDelete() destroys the ExternalRawVideoSource/
	 * ExternalRawVideoSink elements (and thus runs their wrappers'
	 * clearElement() overrides) while the wrappers are still alive --
	 * resetting first would run ~ElementWrapper() instead and the
	 * overrides would never execute. */
	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(static_cast<RawVideoSourceWrapper *>(source)
				   ->getRawVideoSource());
	CU_ASSERT_PTR_NULL(
		static_cast<RawVideoSinkWrapper *>(sink)->getRawVideoSink());

	sinkOwner.reset();
	sourceOwner.reset();
}


static struct mbuf_raw_video_frame *
makeDummyRawFrameEx(const struct vdef_raw_format &format,
		    uint64_t timestamp,
		    unsigned int bitDepth,
		    bool fullRange)
{
	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = format;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.resolution.width = SOURCESINK_RAW_WIDTH;
	frameInfo.info.resolution.height = SOURCESINK_RAW_HEIGHT;
	frameInfo.info.bit_depth = bitDepth;
	frameInfo.info.full_range = fullRange;

	struct mbuf_raw_video_frame *frame = nullptr;
	int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	unsigned int planeCount = vdef_get_raw_frame_plane_count(&format);
	CU_ASSERT_FATAL(planeCount > 0);

	const size_t planeSize = 64;
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(planeSize * planeCount, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	for (unsigned int i = 0; i < planeCount; i++) {
		ret = mbuf_raw_video_frame_set_plane(
			frame, i, mem, i * planeSize, planeSize);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}
	mbuf_mem_unref(mem);

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxRawSourceRejectsBitDepthOrRangeMismatch()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 1);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	struct mbuf_raw_video_frame_queue *outQueue = nullptr;
	createRawSourceAndSink(session,
			       &loop,
			       &mediaListener,
			       &sourceParams,
			       &source,
			       &sink,
			       &inQueue,
			       &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	/* 1. A valid frame is accepted */
	int ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* 2. Frame with bit_depth mismatch is rejected */
	struct mbuf_raw_video_frame *badBitDepthFrame =
		makeDummyRawFrameEx(resolvedFormat,
				    UINT64_C(999999991),
				    resolvedInfo.bit_depth + 2,
				    resolvedInfo.full_range);
	ret = mbuf_raw_video_frame_queue_push(inQueue, badBitDepthFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_raw_video_frame_unref(badBitDepthFrame);

	/* 3. Frame with full_range mismatch is rejected */
	struct mbuf_raw_video_frame *badRangeFrame =
		makeDummyRawFrameEx(resolvedFormat,
				    UINT64_C(999999992),
				    resolvedInfo.bit_depth,
				    !resolvedInfo.full_range);
	ret = mbuf_raw_video_frame_queue_push(inQueue, badRangeFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_raw_video_frame_unref(badRangeFrame);

	/* Clean up resources */
	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxRawSourceRejectsNonMonotonicTimestamp()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 2);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	struct mbuf_raw_video_frame_queue *outQueue = nullptr;
	createRawSourceAndSink(session,
			       &loop,
			       &mediaListener,
			       &sourceParams,
			       &source,
			       &sink,
			       &inQueue,
			       &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	/* Push frame[1] (the later of the two by timestamp) first... */
	int ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[1]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...then frame[0], whose earlier timestamp is now <= mLastTimestamp:
	 * rejected as non-strictly-monotonic. */
	ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[0]);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	std::vector<struct mbuf_raw_video_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_raw_video_frame_unref(f);

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxRawSourceQueueMaxCountDropsOldest()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 3);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);
	sourceParams.queue_max_count = 2;

	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	struct mbuf_raw_video_frame_queue *outQueue = nullptr;
	createRawSourceAndSink(session,
			       &loop,
			       &mediaListener,
			       &sourceParams,
			       &source,
			       &sink,
			       &inQueue,
			       &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	/* Push 3 frames back-to-back with no pump() in between: the source's
	 * own queue (max_frames == queue_max_count == 2) must silently drop
	 * frame[0] (the oldest) when frame[2] is pushed, before anything is
	 * ever forwarded to the sink. */
	for (int i = 0; i < 3; i++) {
		int ret = mbuf_raw_video_frame_queue_push(inQueue,
							  inputFrames[i]);
		CU_ASSERT_EQUAL(ret, 0);
	}

	std::vector<struct mbuf_raw_video_frame *> outputFrames;
	bool gotFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= 2;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrames);
	CU_ASSERT_EQUAL_FATAL(outputFrames.size(), 2u);

	/* The 2 survivors must be frame[1] and frame[2] (by index), NOT
	 * frame[0]: confirms the OLDEST was dropped, not an arbitrary one. */
	for (auto *f : outputFrames) {
		struct vdef_raw_frame info = {};
		int ret = mbuf_raw_video_frame_get_frame_info(f, &info);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_TRUE(info.info.index == 1 || info.info.index == 2);
		mbuf_raw_video_frame_unref(f);
	}

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── flush() vs drain(): what happens to frames still queued in the
 * source's own input queue (inQueue) when neither has ever been forwarded
 * by a loop pump -- same rationale as the coded video counterparts in
 * test_pipeline_sourcesink_coded.cpp. Uses
 * DrainTrackingRawVideoSourceListener/QueueDrainingRawVideoSinkListener
 * directly (not createRawSourceAndSink(), which hardcodes the no-op stub
 * listeners) so mGotFlushed/mGotDrained can actually be observed. Unlike
 * the coded video file, no "primer" push is needed first: raw video sinks
 * have no grey-IDR-style synthesis quirk on their first-ever frame. ───── */

static void testCxxRawSourceFlushDiscardsUnprocessedFrames()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 3);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Push 3 frames back-to-back with NO pump() in between: they sit
	 * unprocessed in inQueue. */
	for (int i = 0; i < 3; i++) {
		ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[i]);
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
	struct mbuf_raw_video_frame *leftover = nullptr;
	CU_ASSERT_NOT_EQUAL(mbuf_raw_video_frame_queue_pop(outQueue, &leftover),
			    0);

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxRawSourceDrainForwardsUnprocessedFrames()
{
	struct vdef_raw_format resolvedFormat;
	struct vdef_format_info resolvedInfo;
	std::vector<struct mbuf_raw_video_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectRawFrames(&vdef_i420,
			 ASSET_I420,
			 &resolvedFormat,
			 &resolvedInfo,
			 &inputFrames,
			 &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 3);

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams;
	fillRawSourceParams(resolvedFormat, resolvedInfo, &sourceParams);

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Same unpumped burst as above. */
	for (int i = 0; i < 3; i++) {
		ret = mbuf_raw_video_frame_queue_push(inQueue, inputFrames[i]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	/* drain(discard=false) calls process() synchronously, right there in
	 * the call: by the time drain() returns, all 3 frames have already
	 * been forwarded all the way to the sink's queue. outputChannel->
	 * drain() is called AFTER process(), so by the time
	 * onRawVideoSinkDrain() fires on sinkListener, the 3 frames are
	 * already sitting in outQueue -- checked via
	 * sinkListener.mDrainedFrames (populated by that very callback), NOT
	 * by popping outQueue ourselves afterward, since the listener itself
	 * must pop the queue as part of draining it. */
	ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	CU_ASSERT_EQUAL_FATAL(sinkListener.mDrainedFrames.size(), 3u);
	for (auto *frame : sinkListener.mDrainedFrames)
		mbuf_raw_video_frame_unref(frame);

	for (auto *frame : inputFrames)
		mbuf_raw_video_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Exercises ExternalRawVideoSink::flushDone()'s mFlushDiscard mismatch path
 * (pdraw_external_raw_video_sink.cpp:287-292): the application acknowledges a
 * real flush (mFlushDiscard==true, set when the source called flush()) by
 * calling queueDrained() instead of queueFlushed() -- see
 * MismatchAckRawVideoSinkListener::onRawVideoSinkFlush() above. The mismatch
 * only logs a warning; flushDone()'s actual channel operation still follows
 * mFlushDiscard, not the discard argument the application passed, so
 * channel->flushDone() runs (matching the flush the source actually
 * requested, since only Channel::mFlushPending -- not mDrainPending -- was
 * ever set) and the source still receives onRawVideoSourceFlushed(), never
 * onRawVideoSourceDrained(). Uses the same synthetic LIVE-mode source
 * construction as testCxxRawSourceSinkSwitchMediaId.
 *
 * Two preconditions, both confirmed missing by real runs before this test
 * could ever reach onRawVideoSinkFlush():
 * 1. The SOURCE's own frame queue must be non-empty when flush() is called:
 *    per ExternalRawVideoSource::flush(), an empty queue takes an early
 *    "already flushed, nothing to do" shortcut that completes its own flush
 *    via an idle callback without ever calling outputChannel->flush() --
 *    the sink never even receives the downstream FLUSH event.
 * 2. The SINK's own FlushingState must already be UNFLUSHED (not its default
 *    FLUSHED) when the channel's FLUSH event arrives: ExternalRawVideoSink::
 *    flush() has the exact same "already flushed, nothing to do" shortcut,
 *    and it stays in FLUSHED (the default) until it has actually received a
 *    frame at least once (the only place that flips it to UNFLUSHED,
 *    pdraw_external_raw_video_sink.cpp:568) -- a source-side discard, which
 *    never forwards anything, does not do this.
 *
 * So: prime the sink first with a real frame that is pumped all the way
 * through (reusing makeDummyRawFrame(), SOURCESINK_RAW_WIDTH/HEIGHT
 * resolution and zero-default bit_depth/full_range to match this source's
 * params, which set neither), THEN push a second frame unpumped into the
 * source's own queue before calling flush(). */
static void testCxxRawVideoSinkFlushAckMismatchFollowsRequestedDiscardState()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = SOURCESINK_RAW_WIDTH;
	sourceParams.video.raw.info.resolution.height = SOURCESINK_RAW_HEIGHT;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	MismatchAckRawVideoSinkListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		mediaListener.mMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	bool gotSinkMediaAdded = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	/* Prime: push a frame and let it fully arrive at the sink -- this alone
	 * flips the sink's FlushingState to UNFLUSHED (precondition 2 above),
	 * and that state persists regardless of the pop below (nothing resets
	 * it back to FLUSHED before flush() is called). */
	struct mbuf_raw_video_frame *primerFrame =
		makeDummyRawFrame(vdef_i420, UINT64_C(1));
	ret = mbuf_raw_video_frame_queue_push(inQueue, primerFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(primerFrame);
	struct mbuf_raw_video_frame *poppedPrimer = nullptr;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       outQueue, &poppedPrimer) == 0;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	mbuf_raw_video_frame_unref(poppedPrimer);

	/* Unpumped push: sits in the source's own queue so flush() below finds
	 * it non-empty (precondition 1 above). */
	struct mbuf_raw_video_frame *frame =
		makeDummyRawFrame(vdef_i420, UINT64_C(2));
	ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	/* Real flush (discard=true): sets mFlushDiscard=true on the sink. */
	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* onRawVideoSinkFlush() fires and acks with queueDrained() instead of
	 * queueFlushed() -- yet the source still sees its flush complete, not
	 * a drain. */
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);
	CU_ASSERT_FALSE(sourceListener.mGotDrained);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies setMediaId() / getMediaId() on IRawVideoSink by creating two raw
 * video sources, attaching the sink to source1, then switching it to
 * source2 mid-stream -- the IRawVideoSink counterpart of
 * testCxxVideoRendererSwitchMediaId (test_pipeline_renderer_video.cpp) and
 * testCxxCodedSourceSinkSwitchMediaId (test_pipeline_sourcesink_coded.cpp).
 *
 * Why this test exists: every other sink in this file is created either
 * with media_id=0 (auto-attach to whatever media turns up next) or with an
 * already-existing media's id at creation time -- both resolved through the
 * same broadcast+filter path (Session::onElementStateChanged /
 * PipelineFactory::onOutputMediaAdded -> addAllMediaToRawVideoSink() /
 * addMediaToAllToRawVideoSinks(), filtered internally by
 * ExternalRawVideoSink::addInputMedia()'s mTargetMediaId check). Neither
 * ever calls Session::addMediaToRawVideoSink(unsigned int mediaId, Sink*) /
 * PipelineFactory's same-named overload -- the single-lookup-by-id path,
 * 0%-covered per the coverage report. The only caller of that path is
 * ExternalRawVideoSink::idleRenewMedia(), itself only scheduled by
 * setMediaId(): setMediaId(m2) sets mTargetMediaId=m2 and schedules
 * idleRenewMedia() on the pomp loop; when that idle fires:
 * removeInputMedia(source1's media) ->
 * mSession->addMediaToRawVideoSink(m2, this) -> addInputMedia(source2's
 * media). Both onRawVideoSinkMediaRemoved and onRawVideoSinkMediaAdded fire
 * synchronously within that same idle callback.
 *
 * Uses the same synthetic LIVE-mode source construction as
 * testCxxVideoRendererSwitchMediaId (no asset file needed -- raw video
 * requires no SPS/PPS-like out-of-band parameter set, unlike coded video),
 * since this test only cares about the id-switch plumbing, not real frame
 * data. */
static void testCxxRawSourceSinkSwitchMediaId()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	/* --- Source 1 and its media --- */
	IPdraw::IRawVideoSource *source1 = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source1);
	auto source1Owner = std::unique_ptr<IPdraw::IRawVideoSource>(source1);
	mediaListener.mSource = source1;

	bool gotMedia1 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia1);
	unsigned int media1Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, 0u);

	/* --- Source 2 and its media --- */
	mediaListener.mGotMediaAdded = false;
	IPdraw::IRawVideoSource *source2 = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::IRawVideoSource>(source2);
	mediaListener.mSource = source2;

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media2Id, 0u);
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, media2Id);

	/* --- Sink on source1's media --- */
	struct pdraw_video_sink_params sinkParams = {};
	RawVideoSinkTrackingListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		media1Id, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	bool gotSinkMedia1 = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMedia1);
	CU_ASSERT_EQUAL(sink->getMediaId(), media1Id);

	/* --- Switch the sink to source2 --- */
	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	ret = sink->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

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


/* Builds a single synthetic LIVE-mode raw video source (same construction as
 * testCxxRawSourceSinkSwitchMediaId -- no asset file needed) and attaches a
 * sink to it, tracked by a RawVideoSinkTrackingListener. Factored out of
 * testCxxRawVideoSinkResolutionChangeSetsRestartFlag/
 * testCxxRawVideoSinkFramerateChangeSetsRestartFlag below, which are
 * otherwise identical except for which Channel::DownstreamEvent they send. */
static void
createLiveRawSourceAndTrackedSink(IPdraw *session,
				  TestPompLoop *loop,
				  RawSourceMediaListener *mediaListener,
				  RawVideoSinkTrackingListener *sinkListener,
				  IPdraw::IRawVideoSource **source,
				  IPdraw::IRawVideoSink **sink)
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);
	mediaListener->mSource = *source;

	bool gotMediaAdded = loop->pumpUntil(
		[mediaListener]() { return mediaListener->mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	unsigned int mediaId = mediaListener->mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(mediaId, 0u);

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createRawVideoSink(
		mediaId, &sinkParams, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);

	bool gotSinkMediaAdded = loop->pumpUntil(
		[sinkListener]() { return sinkListener->mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);
}


/* Exercises ExternalRawVideoSink::onChannelResolutionChange()
 * (pdraw_external_raw_video_sink.cpp:666-678), 0%-covered per the coverage
 * report -- no other test in this file ever sends a RESOLUTION_CHANGE
 * downstream event on a raw video sink's input channel.
 *
 * onChannelResolutionChange() unconditionally sets mPendingRestart=true
 * before forwarding to Sink::onChannelResolutionChange(); structurally
 * identical to GlVideoRenderer::onChannelResolutionChange() (see
 * testCxxVideoRendererResolutionChangeSetsRestartFlag,
 * test_pipeline_renderer_video.cpp) minus the transition-arming side effect
 * (ExternalRawVideoSink has no transitions).
 *
 * mPendingRestart has no public getter, but (as already used by
 * testCxxRawSourceSinkSwitchMediaId, which asserts restart=false specifically
 * *because* setMediaId() never goes through onChannelResolutionChange/
 * onChannelFramerateChange/onChannelReconfigure) it surfaces as the "restart"
 * argument of onRawVideoSinkMediaRemoved() once the source media is
 * destroyed -- reused here as the observable proof that this exact function
 * set it to true. */
static void testCxxRawVideoSinkResolutionChangeSetsRestartFlag()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RawVideoSinkTrackingListener sinkListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	createLiveRawSourceAndTrackedSink(
		session, &loop, &mediaListener, &sinkListener, &source, &sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	ExternalRawVideoSink *extSink =
		static_cast<RawVideoSinkWrapper *>(sink)->getRawVideoSink();
	CU_ASSERT_PTR_NOT_NULL_FATAL(extSink);
	Media *media = extSink->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = extSink->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RESOLUTION_CHANGE);
	CU_ASSERT_EQUAL(err, 0);

	sourceOwner.reset();
	source = nullptr;

	bool gotRemoved = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaRemoved; });
	CU_ASSERT_TRUE_FATAL(gotRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Same as testCxxRawVideoSinkResolutionChangeSetsRestartFlag but for
 * ExternalRawVideoSink::onChannelFramerateChange()
 * (pdraw_external_raw_video_sink.cpp:681-693) -- structurally identical
 * (mPendingRestart unconditional), but a distinct function/downstream event,
 * so it needs its own coverage. */
static void testCxxRawVideoSinkFramerateChangeSetsRestartFlag()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RawVideoSinkTrackingListener sinkListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	createLiveRawSourceAndTrackedSink(
		session, &loop, &mediaListener, &sinkListener, &source, &sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	ExternalRawVideoSink *extSink =
		static_cast<RawVideoSinkWrapper *>(sink)->getRawVideoSink();
	CU_ASSERT_PTR_NOT_NULL_FATAL(extSink);
	Media *media = extSink->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = extSink->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::FRAMERATE_CHANGE);
	CU_ASSERT_EQUAL(err, 0);

	sourceOwner.reset();
	source = nullptr;

	bool gotRemoved = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaRemoved; });
	CU_ASSERT_TRUE_FATAL(gotRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies ExternalRawVideoSink::setMediaId()'s early-return branch
 * (pdraw_external_raw_video_sink.cpp:221-222): calling setMediaId() with the
 * SAME id as the sink's current mTargetMediaId returns 0 immediately without
 * scheduling idleRenewMedia() -- unlike testCxxRawSourceSinkSwitchMediaId
 * above (different id), which does trigger a media remove+re-add. Reuses
 * createLiveRawSourceAndTrackedSink() like the two *SetsRestartFlag tests
 * above. */
static void testCxxRawVideoSinkSetMediaIdSameIdIsNoop()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RawVideoSinkTrackingListener sinkListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	createLiveRawSourceAndTrackedSink(
		session, &loop, &mediaListener, &sinkListener, &source, &sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	unsigned int mediaId = sink->getMediaId();
	CU_ASSERT_NOT_EQUAL_FATAL(mediaId, 0u);

	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	int ret = sink->setMediaId(mediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* No idleRenewMedia() is scheduled: pump for a short, bounded time and
	 * confirm neither callback fires (contrast with
	 * testCxxRawSourceSinkSwitchMediaId, where switching to a different id
	 * always fires both). */
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


/* Regression test for two real use-after-free bugs found via ASan on the
 * coded video counterpart of this scenario
 * (testCxxCodedSourceSinkAbruptSessionDestructionDoesNotCrash,
 * test_pipeline_sourcesink_coded.cpp -- see that test's comment for the
 * full history). Destroying the whole Session abruptly (skipping stop())
 * while a source and sink are both still fully wired up used to crash
 * twice: (1) ExternalRawVideoSource::~ExternalRawVideoSource() freeing its
 * mOutputMedia while a channel was still attached, walked as a dangling
 * pointer by ~Source()'s own removeOutputPorts() right after; (2) once
 * fixed, the still-attached Sink crashing in turn on its own mInputMedia,
 * pointing at that same already-freed Media object. Both are now fixed by
 * Source::teardownOutputChannels() (pdraw_source.cpp), called from the
 * destructor before removeOutputPort(): it synchronously runs the
 * channel->teardown() round-trip (Sink::onChannelTeardown() ->
 * removeInputMedia()) while the media is still valid, before the source
 * frees it -- which as a side effect also means the sink's own "input media
 * has not been removed" branch is no longer reachable even here (its
 * mInputMedia is already null by the time its own destructor runs). This
 * test therefore targets Source::teardownOutputChannels(), not that
 * branch. */
static void testCxxRawSourceSinkAbruptSessionDestructionDoesNotCrash()
{
	RawSourceMediaListener mediaListener;
	RawVideoSinkTrackingListener sinkListener;
	std::unique_ptr<IPdraw::IRawVideoSource> sourceOwner;
	std::unique_ptr<IPdraw::IRawVideoSink> sinkOwner;

	{
		TestPompLoop loop;
		TestSession testSession(&loop, &mediaListener);
		IPdraw *session = testSession.get();

		IPdraw::IRawVideoSource *source = nullptr;
		IPdraw::IRawVideoSink *sink = nullptr;
		createLiveRawSourceAndTrackedSink(session,
						  &loop,
						  &mediaListener,
						  &sinkListener,
						  &source,
						  &sink);
		sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
		sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

		/* Deliberately no stop()/stopSessionAndWait() call here:
		 * testSession and loop are destroyed right below, at the end
		 * of this block, forcing ~Session() to run while mState is
		 * still READY -- the only way to reach the scenario described
		 * above. */
	}

	/* If we get here, the abrupt teardown did not crash. The sink was
	 * synchronously notified of its media's removal as part of the
	 * source's own destructor -- confirming the fix actually ran, not
	 * just that nothing crashed. */
	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
}


/* ── C-wrapper listener callback tests (C API) ──
 * Self-contained pdraw_new() fixtures with counting C callbacks.
 * SessionCbState / make_pdraw() are shared via test_pipeline_common.hpp. */

struct RawSrcCbState {
	int flushedCount = 0;
	int drainedCount = 0;
};

static void raw_src_flushed_cb(struct pdraw * /*p*/,
			       struct pdraw_raw_video_source * /*s*/,
			       void *ud)
{
	static_cast<RawSrcCbState *>(ud)->flushedCount++;
}

static void raw_src_drained_cb(struct pdraw * /*p*/,
			       struct pdraw_raw_video_source * /*s*/,
			       void *ud)
{
	static_cast<RawSrcCbState *>(ud)->drainedCount++;
}

struct RawSinkCbState {
	/* Pointer to the enclosing pdraw handle, needed to call
	 * pdraw_raw_video_sink_queue_flushed/drained from callbacks. */
	struct pdraw *p = nullptr;

	int mediaAddedCount = 0;
	int flushCount = 0;
	int drainCount = 0;
	int sessionMetaCount = 0;
	int mediaRemovedCount = 0;
};

static void raw_sink_media_added_cb(struct pdraw * /*p*/,
				    struct pdraw_raw_video_sink * /*sk*/,
				    const struct pdraw_media_info * /*info*/,
				    void *ud)
{
	static_cast<RawSinkCbState *>(ud)->mediaAddedCount++;
}

static void raw_sink_media_removed_cb(struct pdraw * /*p*/,
				      struct pdraw_raw_video_sink * /*sk*/,
				      const struct pdraw_media_info * /*info*/,
				      int /*restart*/,
				      void *ud)
{
	static_cast<RawSinkCbState *>(ud)->mediaRemovedCount++;
}

static void
raw_sink_flush_cb(struct pdraw *p, struct pdraw_raw_video_sink *sk, void *ud)
{
	static_cast<RawSinkCbState *>(ud)->flushCount++;
	struct mbuf_raw_video_frame_queue *q =
		pdraw_raw_video_sink_get_queue(p, sk);
	if (q != nullptr)
		mbuf_raw_video_frame_queue_flush(q);
	pdraw_raw_video_sink_queue_flushed(p, sk);
}

static void
raw_sink_drain_cb(struct pdraw *p, struct pdraw_raw_video_sink *sk, void *ud)
{
	static_cast<RawSinkCbState *>(ud)->drainCount++;
	/* No frames queued (source was just flushed/drained) — pop loop is
	 * effectively a no-op, then acknowledge drain. */
	struct mbuf_raw_video_frame_queue *q =
		pdraw_raw_video_sink_get_queue(p, sk);
	if (q != nullptr) {
		struct mbuf_raw_video_frame *frame = nullptr;
		while (mbuf_raw_video_frame_queue_pop(q, &frame) == 0)
			mbuf_raw_video_frame_unref(frame);
	}
	pdraw_raw_video_sink_queue_drained(p, sk);
}

static void raw_sink_session_meta_cb(struct pdraw * /*p*/,
				     struct pdraw_raw_video_sink * /*sk*/,
				     const struct vmeta_session * /*meta*/,
				     void *ud)
{
	static_cast<RawSinkCbState *>(ud)->sessionMetaCount++;
}


static void testCRawVideoSourceListenerFlushedDrained()
{
	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	struct pdraw_raw_video_source_cbs cbs = {};
	cbs.flushed = raw_src_flushed_cb;
	cbs.drained = raw_src_drained_cb;
	RawSrcCbState srcState;

	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_RAW;
	struct pdraw_raw_video_source *src = nullptr;
	int ret = pdraw_raw_video_source_new(p, &params, &cbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* No connected sink → completeFlush fires immediately via idle */
	ret = pdraw_raw_video_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&srcState]() { return srcState.flushedCount >= 1; });
	CU_ASSERT_TRUE(gotFlushed);
	CU_ASSERT_EQUAL(srcState.flushedCount, 1);

	/* Source is FLUSHED with empty queue → drain re-queues idle with
	 * discard=false → fires onRawVideoSourceDrained */
	ret = pdraw_raw_video_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&srcState]() { return srcState.drainedCount >= 1; });
	CU_ASSERT_TRUE(gotDrained);
	CU_ASSERT_EQUAL(srcState.drainedCount, 1);

	pdraw_raw_video_source_destroy(p, src);
	pdraw_destroy(p);
}


static void testCRawVideoSinkListenerCallbacks()
{
	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	/* Create raw video source with minimal valid format so the output media
	 * is well-formed enough to let the session connect a raw sink to it. */
	struct pdraw_video_source_params srcParams = {};
	srcParams.video.format = VDEF_FRAME_TYPE_RAW;
	srcParams.video.raw.format = vdef_raw8;
	srcParams.video.raw.info.resolution.width = 1;
	srcParams.video.raw.info.resolution.height = 1;
	srcParams.video.raw.info.bit_depth = 8;
	struct pdraw_raw_video_source_cbs srcCbs = {};
	srcCbs.flushed = raw_src_flushed_cb;
	srcCbs.drained = raw_src_drained_cb;
	RawSrcCbState srcState;
	struct pdraw_raw_video_source *src = nullptr;
	int ret = pdraw_raw_video_source_new(
		p, &srcParams, &srcCbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* Wait for session-level media_added to learn the live media ID */
	bool gotAdded = loop.pumpUntil(
		[&sessState]() { return sessState.lastMediaId != 0; });
	CU_ASSERT_TRUE_FATAL(gotAdded);
	unsigned int mediaId = sessState.lastMediaId;

	/* Create raw video sink on that media */
	RawSinkCbState sinkState;
	sinkState.p = p;
	struct pdraw_video_sink_params sinkParams = {};
	struct pdraw_raw_video_sink_cbs sinkCbs = {};
	sinkCbs.media_added = raw_sink_media_added_cb;
	sinkCbs.media_removed = raw_sink_media_removed_cb;
	sinkCbs.flush = raw_sink_flush_cb;
	sinkCbs.drain = raw_sink_drain_cb;
	sinkCbs.session_metadata_update = raw_sink_session_meta_cb;
	struct pdraw_raw_video_sink *snk = nullptr;
	ret = pdraw_raw_video_sink_new(
		p, mediaId, &sinkParams, &sinkCbs, &sinkState, &snk);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(snk);

	/* Wait for the sink's own onRawVideoSinkMediaAdded to fire */
	bool gotSinkAdded = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaAddedCount >= 1; });
	CU_ASSERT_TRUE(gotSinkAdded);

	/* Flush: push frame, pump once to deliver it to sink, then flush */
	{
		struct mbuf_raw_video_frame_queue *inQ =
			pdraw_raw_video_source_get_queue(p, src);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
		struct mbuf_raw_video_frame *f =
			PdrawTest::makeDummyRawVideoFrame(1000);
		ret = mbuf_raw_video_frame_queue_push(inQ, f);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_raw_video_frame_unref(f);
	}
	loop.runOnce(); /* queueEventCb → processFrame → frame in sink queue */
	ret = pdraw_raw_video_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlush = loop.pumpUntil([&sinkState, &srcState]() {
		return sinkState.flushCount >= 1 && srcState.flushedCount >= 1;
	});
	CU_ASSERT_TRUE(gotFlush);
	CU_ASSERT_EQUAL(sinkState.flushCount, 1);
	CU_ASSERT_EQUAL(srcState.flushedCount, 1);

	/* Drain: push another frame → call drain without pumping first. */
	{
		struct mbuf_raw_video_frame_queue *inQ =
			pdraw_raw_video_source_get_queue(p, src);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
		struct mbuf_raw_video_frame *f =
			PdrawTest::makeDummyRawVideoFrame(2000);
		ret = mbuf_raw_video_frame_queue_push(inQ, f);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_raw_video_frame_unref(f);
	}
	ret = pdraw_raw_video_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrain = loop.pumpUntil([&sinkState, &srcState]() {
		return sinkState.drainCount >= 1 && srcState.drainedCount >= 1;
	});
	CU_ASSERT_TRUE(gotDrain);
	CU_ASSERT_EQUAL(sinkState.drainCount, 1);
	CU_ASSERT_EQUAL(srcState.drainedCount, 1);

	/* Set session metadata: triggers SESSION_META_UPDATE down the channel
	 */
	struct vmeta_session meta = {};
	ret = pdraw_raw_video_source_set_session_metadata(p, src, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMeta = loop.pumpUntil(
		[&sinkState]() { return sinkState.sessionMetaCount >= 1; });
	CU_ASSERT_TRUE(gotMeta);

	/* Destroy source: fires onRawVideoSinkMediaRemoved on the sink */
	pdraw_raw_video_source_destroy(p, src);
	src = nullptr;
	bool gotRemoved = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaRemovedCount >= 1; });
	CU_ASSERT_TRUE(gotRemoved);

	pdraw_raw_video_sink_destroy(p, snk);
	pdraw_destroy(p);
}


static void testCxxRawVideoSourceWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = SOURCESINK_RAW_WIDTH;
	sourceParams.video.raw.info.resolution.height = SOURCESINK_RAW_HEIGHT;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	stopSessionAndWait(&loop, session, &mediaListener);

	/* After session stop, ElementWrapper::mElementStopped is true for all
	 * elements; every isElementStopped() guard must now fire. */
	CU_ASSERT_PTR_NULL(source->getQueue());
	CU_ASSERT_EQUAL(source->flush(), -EPROTO);
	CU_ASSERT_EQUAL(source->drain(), -EPROTO);
	struct vmeta_session meta = {};
	CU_ASSERT_EQUAL(source->setSessionMetadata(&meta), -EPROTO);
	CU_ASSERT_EQUAL(source->getSessionMetadata(&meta), -EPROTO);
}


static void testCxxRawVideoSinkWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	RawSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = SOURCESINK_RAW_WIDTH;
	sourceParams.video.raw.info.resolution.height = SOURCESINK_RAW_HEIGHT;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(mediaListener.mMediaId,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_EQUAL(sink->setMediaId(0), -EPROTO);
	CU_ASSERT_EQUAL(sink->getMediaId(), (unsigned int)-EPROTO);
	CU_ASSERT_PTR_NULL(sink->getQueue());
	CU_ASSERT_EQUAL(sink->queueFlushed(), -EPROTO);
	CU_ASSERT_EQUAL(sink->queueDrained(), -EPROTO);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_sourcesink_raw[] = {
	{FN("testCxxRawSourceSinkRoundtripI420"),
	 testCxxRawSourceSinkRoundtripI420},
	{FN("testCxxRawSourceSinkRoundtripNv12"),
	 testCxxRawSourceSinkRoundtripNv12},
	{FN("testCxxRawSourceRejectsFormatMismatch"),
	 testCxxRawSourceRejectsFormatMismatch},
	{FN("testCxxRawSourceRejectsBitDepthOrRangeMismatch"),
	 testCxxRawSourceRejectsBitDepthOrRangeMismatch},
	{FN("testCxxRawSourceRejectsNonMonotonicTimestamp"),
	 testCxxRawSourceRejectsNonMonotonicTimestamp},
	{FN("testCxxRawSourceQueueMaxCountDropsOldest"),
	 testCxxRawSourceQueueMaxCountDropsOldest},
	{FN("testCxxRawSourceFlushDiscardsUnprocessedFrames"),
	 testCxxRawSourceFlushDiscardsUnprocessedFrames},
	{FN("testCxxRawSourceDrainForwardsUnprocessedFrames"),
	 testCxxRawSourceDrainForwardsUnprocessedFrames},
	{FN("testCxxRawVideoSinkFlushAckMismatchFollowsRequestedDiscardState"),
	 testCxxRawVideoSinkFlushAckMismatchFollowsRequestedDiscardState},
	{FN("testCxxRawSourceSinkSwitchMediaId"),
	 testCxxRawSourceSinkSwitchMediaId},
	{FN("testCxxRawVideoSinkResolutionChangeSetsRestartFlag"),
	 testCxxRawVideoSinkResolutionChangeSetsRestartFlag},
	{FN("testCxxRawVideoSinkFramerateChangeSetsRestartFlag"),
	 testCxxRawVideoSinkFramerateChangeSetsRestartFlag},
	{FN("testCxxRawVideoSinkSetMediaIdSameIdIsNoop"),
	 testCxxRawVideoSinkSetMediaIdSameIdIsNoop},
	{FN("testCxxRawSourceSinkAbruptSessionDestructionDoesNotCrash"),
	 testCxxRawSourceSinkAbruptSessionDestructionDoesNotCrash},
	{FN("testCRawVideoSourceListenerFlushedDrained"),
	 testCRawVideoSourceListenerFlushedDrained},
	{FN("testCRawVideoSinkListenerCallbacks"),
	 testCRawVideoSinkListenerCallbacks},
	{FN("testCxxRawVideoSourceWrapperGuardsAfterElementCleared"),
	 testCxxRawVideoSourceWrapperGuardsAfterElementCleared},
	{FN("testCxxRawVideoSinkWrapperGuardsAfterElementCleared"),
	 testCxxRawVideoSinkWrapperGuardsAfterElementCleared},
	CU_TEST_INFO_NULL,
};
