/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — video encoder on a real decoded media, and on a
 * standalone synthetic source (Tier B, self-contained fixture)
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

/* Split out of test_pipeline_decode.cpp (see test_pipeline_decoder_video.cpp's
 * header comment for the full split history): this file holds every
 * IVideoEncoder test. The shared demuxer-fixture
 * helpers (PipelineDemuxerListener, openDecodingDemuxerAndPlay, etc.) are
 * now in test_pipeline_common.hpp; local listener classes
 * (MediaTrackingListener, DrainTrackingRawVideoSourceListener,
 * AckingCodedVideoSinkListener) remain per-translation-unit.
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the decoder's/
 * encoder's output media requires a real session-wide listener, so each
 * test below builds its own private TestPompLoop + TestSession, fully
 * self-contained. No suite init/cleanup is registered for this file (see
 * test_main.c: NULL, NULL). */

#define ULOG_TAG pdraw_test_pipeline_encoder_video

#include <complex>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "pdraw_element.hpp"
#include "pdraw_encoder_video.hpp"
#undef protected
#undef private

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

/* struct venc_config is already visible transitively via pdraw_defs.h
 * (included from test_api_common.hpp). */

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <string.h>

#include <atomic>
#include <mutex>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── MP4 fixture (same NAS asset as test_api_demuxer.cpp) ────────────────── */

enum { ASSET_VIDEO_H264 = 0 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_encoder_video[] = {
	{"Tests/anafi/4k/video_recording/champs_240p30_h264.mp4"},
};


/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

/* Common body (Added, onMediaAdded, mAdded, …) is in
 * PdrawTest::MediaTrackingListenerBase (test_pipeline_common.hpp). */
class MediaTrackingListener : public MediaTrackingListenerBase {
public:
	const Added *findRawVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_RAW)
				return &a;
		}
		return nullptr;
	}

	/* First coded video media added, or nullptr if none yet -- e.g. a
	 * VideoEncoder's own (lazily created) output media, added once its
	 * first frame has been successfully encoded (see
	 * VideoEncoder::createOutputMedia). */
	const Added *findCodedVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_CODED)
				return &a;
		}
		return nullptr;
	}

	/* Overrides the no-op base implementation: needed to observe the
	 * encoder's own output media being torn down as a consequence of
	 * VideoEncoder::onChannelTeardown -> stop() actually running (see
	 * testCxxVideoEncoderWrapperGuardsAfterElementCleared below). */
	void onMediaRemoved(Pdraw::IPdraw * /*p*/,
			    const struct pdraw_media_info *info,
			    void * /*u*/) override
	{
		mRemovedIds.push_back(info->id);
	}

	bool wasRemoved(unsigned int id) const
	{
		for (unsigned int removedId : mRemovedIds) {
			if (removedId == id)
				return true;
		}
		return false;
	}

	std::vector<unsigned int> mRemovedIds;
};


/* Unlike ExternalCodedVideoSource/Sink or the demuxer/muxer, IVideoEncoder
 * has no pollable getQueue(): output frames are pushed directly to
 * Listener::videoEncoderFrameOutput(), documented as "called with a
 * Listener mutex locked, therefore no API function must be called from
 * this callback" -- that warning is about calling back into the pdraw API
 * (e.g. session/encoder methods), not about reading frame metadata via
 * mbuf_coded_video_frame_get_frame_info()/get_nalu_count(), which the same
 * doc explicitly says this callback exists for ("can be used to extract
 * frame info ... from the frame"). Thread affinity is documented as
 * "usually the pomp_loop thread, but it may depend on the ... encoder
 * implementation", so captured fields use std::atomic/std::mutex rather
 * than assuming same-thread delivery, mirroring PipelineMuxerListener's
 * reasoning in test_pipeline_muxer_record.cpp. */
class VideoEncoderOutputListener : public IPdraw::IVideoEncoder::Listener {
public:
	void
	videoEncoderFrameOutput(IPdraw * /*p*/,
				IPdraw::IVideoEncoder * /*e*/,
				struct mbuf_coded_video_frame *frame) override
	{
		struct vdef_coded_frame info = {};
		int ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
		if (ret < 0)
			return;
		int naluCountRet = mbuf_coded_video_frame_get_nalu_count(frame);
		{
			std::lock_guard<std::mutex> lock(mMutex);
			mEncoding = info.format.encoding;
			mNaluCount = naluCountRet;
			mFrameTypes.push_back(info.type);
		}
		mGotFrame.store(true);
	}

	void videoEncoderFramePreRelease(
		IPdraw * /*p*/,
		IPdraw::IVideoEncoder * /*e*/,
		struct mbuf_coded_video_frame * /*f*/) override
	{
	}

	/* Number of frames received so far; thread-safe (see mMutex below). */
	size_t frameCount()
	{
		std::lock_guard<std::mutex> lock(mMutex);
		return mFrameTypes.size();
	}

	std::mutex mMutex;
	enum vdef_encoding mEncoding = VDEF_ENCODING_UNKNOWN;
	int mNaluCount = 0;
	std::vector<enum vdef_coded_frame_type> mFrameTypes;
	std::atomic<bool> mGotFrame{false};
};


/* Tracks IRawVideoSource::flush()/drain() completion -- same rationale as
 * DrainTrackingCodedVideoSourceListener in test_pipeline_sourcesink_coded.cpp
 * (duplicated here rather than shared, per this suite's convention: each
 * test file's listeners are private to its own translation unit). Used
 * below to prove a flush()/drain() issued on a raw video source cascades
 * all the way through an intervening VideoEncoder (exercising its
 * onChannelFlush/onChannelDrain/onChannelFlushed/onChannelDrained, entirely
 * uncovered otherwise: IVideoEncoder has no public flush()/drain() of its
 * own to call directly) and back, rather than just checking the encoder
 * element doesn't crash. */
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


/* Implements the sink side of the flush/drain protocol for real -- same
 * rationale and near-identical body as QueueDrainingCodedVideoSinkListener
 * in test_pipeline_sourcesink_coded.cpp (duplicated, not shared, same
 * convention as above): without a real acknowledgment via queueFlushed()/
 * queueDrained(), the encoder's own completeFlush() would never see its
 * output channel's flush/drain complete, and the flush/drain would never
 * cascade back upstream to the source. */
class AckingCodedVideoSinkListener : public IPdraw::ICodedVideoSink::Listener {
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
		discardQueue();
		sk->queueFlushed();
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		/* Drain must not lose data, but this test has nothing further
		 * to verify about the frames themselves -- unref immediately
		 * rather than accumulating (contrast
		 * QueueDrainingCodedVideoSinkListener::onCodedVideoSinkDrain,
		 * which keeps them for inspection). */
		discardQueue();
		sk->queueDrained();
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session *meta) override
	{
		if (meta != nullptr)
			mMeta = *meta;
		mGotSessionMetaUpdate = true;
	}

	/* Must be set right after createCodedVideoSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_coded_video_frame_queue *mQueue = nullptr;
	struct vmeta_session mMeta = {};
	bool mGotSessionMetaUpdate = false;

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


/* A minimal, correctly-strided/sized I420 frame (see
 * vdef_calc_raw_frame_size() usage in test_pipeline_muxer_record.cpp for the
 * same pattern) -- unlike a dummy queue-filter-only frame, this one is fed to a
 * real x264 encoder, which reads actual pixel data according to the
 * declared resolution/stride, so the planes must be genuinely, correctly
 * sized (content itself is irrelevant: flat grey). */
static struct mbuf_raw_video_frame *makeI420Frame(uint32_t width,
						  uint32_t height,
						  uint64_t timestamp,
						  unsigned int index)
{
	struct vdef_dim resolution = {width, height};
	size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
	int ret = vdef_calc_raw_frame_size(&vdef_i420,
					   &resolution,
					   planeStride,
					   nullptr,
					   nullptr,
					   nullptr,
					   planeSize,
					   nullptr);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_i420);
	CU_ASSERT_EQUAL_FATAL(planeCount, 3u);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_i420;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;
	frameInfo.info.resolution.width = width;
	frameInfo.info.resolution.height = height;
	frameInfo.info.bit_depth = 8;
	for (unsigned int p = 0; p < planeCount; p++)
		frameInfo.plane_stride[p] = planeStride[p];

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	for (unsigned int p = 0; p < planeCount; p++) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(planeSize[p], &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0x80, capacity);
		ret = mbuf_raw_video_frame_set_plane(
			frame, p, mem, 0, planeSize[p]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);
	}

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	return frame;
}


/* Build a standalone ExternalRawVideoSource -> VideoEncoder -> (real)
 * ExternalCodedVideoSink chain -- no demuxer/decoder involved -- and push
 * one priming frame through it so the encoder's output media exists.
 * sourceListener is passed in (rather than hardcoded to a stub) because
 * IRawVideoSource::Listener is fixed at creation time and cannot be swapped
 * afterward: a caller that needs to observe flush()/drain() completion via
 * onRawVideoSourceFlushed()/onRawVideoSourceDrained() must supply its own
 * tracking listener from the very start. Returns once the coded sink is
 * attached and ready for more frames to be pushed by the caller. */
static void
createEncoderChainAndPrime(IPdraw *session,
			   TestPompLoop *loop,
			   MediaTrackingListener *mediaListener,
			   IPdraw::IRawVideoSource::Listener *sourceListener,
			   IPdraw::IRawVideoSource **source,
			   IPdraw::IVideoEncoder **encoder,
			   IPdraw::ICodedVideoSink **sink,
			   AckingCodedVideoSinkListener *sinkListener,
			   struct mbuf_raw_video_frame_queue **inQueue)
{
	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_encoder_video_flush");

	int ret = session->createRawVideoSource(
		&sourceParams, sourceListener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);

	bool gotRawMedia = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener->findRawVideoMedia()->id;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	ret = session->createVideoEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_video_encoder_listener,
					  encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*encoder);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);

	/* Prime: push a small burst of frames (not just one -- libx264's own
	 * frame-parallel auto-threading, per the comment on
	 * testCxxVideoEncoderRequestKeyFrameProducesIdrFrame, may need more
	 * than a single input frame buffered before it flushes any output at
	 * all) so the encoder produces its first output frame, which is when
	 * its output media is created (see VideoEncoder::createOutputMedia).
	 * No coded sink exists yet, so these first encoded frames have
	 * nowhere to go and are dropped -- irrelevant here, only the media's
	 * existence matters. */
	for (unsigned int i = 0; i < 4; i++) {
		struct mbuf_raw_video_frame *primer =
			makeI420Frame(kWidth, kHeight, i * 33333, i);
		ret = mbuf_raw_video_frame_queue_push(*inQueue, primer);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(primer);
	}

	bool gotCodedMedia = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findCodedVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotCodedMedia);
	unsigned int codedMediaId = mediaListener->findCodedVideoMedia()->id;

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createCodedVideoSink(
		codedMediaId, &sinkParams, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);
	sinkListener->mQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);
}


/* Neither ICodedVideoSource/IRawVideoSource's own flush()/drain() (already
 * covered extensively in test_pipeline_sourcesink_*.cpp) nor
 * IVideoEncoder's configure()/getConfig()/requestKeyFrame() exercise
 * VideoEncoder::onChannelFlush/onChannelDrain/onChannelFlushed/
 * onChannelDrained at all: IVideoEncoder has no public flush()/drain() of
 * its own (it is a pure filter element, only ever flushed/drained by
 * whatever feeds it), so the only way to reach that code is to build a real
 * Source -> VideoEncoder -> Sink chain and flush/drain the *source*,
 * letting the event cascade through the encoder in the middle and back:
 * onRawVideoSourceFlushed() only fires once VideoEncoder::onChannelFlush()
 * ran, flushed its own input/output, got acknowledged by the coded sink
 * (AckingCodedVideoSinkListener::onCodedVideoSinkFlush() ->
 * queueFlushed()), and VideoEncoder::onChannelFlushed()/completeFlush()
 * propagated the completion back upstream via the input channel -- so
 * observing it here is proof the whole chain, including the encoder's own
 * handlers, actually ran. */
static void testCxxVideoEncoderFlushCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoEncoder *encoder = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	AckingCodedVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createEncoderChainAndPrime(session,
				   &loop,
				   &mediaListener,
				   &sourceListener,
				   &source,
				   &encoder,
				   &sink,
				   &sinkListener,
				   &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	(void)inQueue;

	int ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	/* stopSessionAndWait() must run BEFORE the owners are reset, so that
	 * Session::asyncElementDelete() destroys the VideoEncoder element (and
	 * thus runs VideoEncoderWrapper::clearElement()) while encoderOwner is
	 * still alive -- resetting first would run ~ElementWrapper() instead
	 * and the override would never execute. */
	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(
		static_cast<VideoEncoderWrapper *>(encoder)->getVideoEncoder());

	sinkOwner.reset();
	encoderOwner.reset();
	sourceOwner.reset();
}


/* Same rationale as testCxxVideoEncoderFlushCascadesThroughSourceAndSink
 * above, for drain() instead of flush() -- reaches
 * VideoEncoder::onChannelDrain/onChannelDrained instead of onChannelFlush/
 * onChannelFlushed. */
static void testCxxVideoEncoderDrainCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoEncoder *encoder = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	AckingCodedVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createEncoderChainAndPrime(session,
				   &loop,
				   &mediaListener,
				   &sourceListener,
				   &source,
				   &encoder,
				   &sink,
				   &sinkListener,
				   &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	(void)inQueue;

	int ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	sinkOwner.reset();
	encoderOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoEncoder::onChannelSessionMetaUpdate is a Sink-side callback invoked
 * when SESSION_META_UPDATE propagates downstream through the encoder's
 * input channel; it then forwards to the encoder's own output channel(s)
 * (see FilterElement::onChannelSessionMetaUpdate in pdraw_element.cpp,
 * which VideoEncoder::onChannelSessionMetaUpdate calls into after its own
 * handling) and reaches the downstream coded video sink. Entirely
 * uncovered otherwise (0% per a real gcov run): none of the other tests in
 * this file ever change session metadata after creation. Only
 * IRawVideoSource has a public setSessionMetadata() to trigger this from a
 * test (see ExternalRawVideoSource::setSessionMetadata in
 * pdraw_external_raw_video_source.cpp) -- same "flush/drain the source,
 * observe the cascade through the encoder and back" idiom as the tests
 * above, but for session metadata instead of flush/drain. */
static void testCxxVideoEncoderSessionMetaUpdateCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoEncoder *encoder = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	AckingCodedVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createEncoderChainAndPrime(session,
				   &loop,
				   &mediaListener,
				   &sourceListener,
				   &source,
				   &encoder,
				   &sink,
				   &sinkListener,
				   &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	(void)inQueue;

	struct vmeta_session meta = {};
	snprintf(meta.friendly_name,
		 sizeof(meta.friendly_name),
		 "pdraw_test_session_meta_update");
	int ret = source->setSessionMetadata(&meta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotUpdate = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotSessionMetaUpdate;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotUpdate);
	CU_ASSERT_STRING_EQUAL(sinkListener.mMeta.friendly_name,
			       "pdraw_test_session_meta_update");

	sinkOwner.reset();
	encoderOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* An entirely empty/all-zero venc_config (in particular .encoding left at
 * VDEF_ENCODING_UNKNOWN) was never exercised: every other test in this file
 * always sets .encoding explicitly. Read venc_new()/create() for every
 * concrete backend in packages/libvideo-encode (x264, x265, turbojpeg, png,
 * ffmpeg, videotoolbox, mediacodec): each one's create() rejects any encoding
 * that isn't the single specific one it implements (e.g. venc_x264.c:1278
 * "if (base->config.encoding != VDEF_ENCODING_H264) return -EINVAL") --
 * unlike get_supported_input_formats(), which ignores the encoding
 * parameter entirely (venc_x264.c:1049 "UNUSED(encoding)") and so does NOT
 * reject the empty config earlier, in VideoEncoder's own constructor.
 * VDEF_ENCODING_UNKNOWN can never match any backend's specific check, so
 * createVideoEncoder() with a fully empty config is expected to fail
 * predictably (unlike e.g. an empty vscale_config, which can legitimately
 * default to something), regardless of which concrete encoder backend(s)
 * happen to be compiled into this product. */
static void testCxxCreateVideoEncoderWithEmptyConfigFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawVideoMedia()->id;

	struct venc_config encoderParams = {};
	IPdraw::IVideoEncoder *encoder = nullptr;
	ret = session->createVideoEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_video_encoder_listener,
					  &encoder);
	CU_ASSERT_NOT_EQUAL(ret, 0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Variant of testCxxCreateVideoEncoderWithEmptyConfigFails targeting a
 * different failure point. There, .encoding is left at VDEF_ENCODING_UNKNOWN,
 * which venc_new()'s own generic switch on config.encoding rejects directly
 * (venc.c:766 "default: res = -EINVAL"), before ever reaching a specific
 * backend's create(). Here .implem is explicitly forced to
 * VENC_ENCODER_IMPLEM_X264 -- bypassing VideoEncoder::start()'s
 * AUTO-implem-by-encoding lookup (pdraw_encoder_video.cpp:202-217, only run
 * when .implem == VENC_ENCODER_IMPLEM_AUTO), which would otherwise fail with
 * -ENOENT before venc_new() is even called -- while .encoding is set to
 * VDEF_ENCODING_JPEG: a recognized, valid encoding, just not the one x264
 * implements. venc.c's generic switch (venc.c:737 "case
 * VDEF_ENCODING_MJPEG") accepts this combination (it only validates
 * config.mjpeg.* fields, filled here exactly as in
 * testCxxVideoEncoderProducesRealJpegOutput so that check can't be what
 * fails), so venc_new() reaches self->ops->create() (venc.c:771), and it is
 * genuinely the x264 backend's own create() that rejects the mismatch, at
 * venc_x264.c:1278 ("if (base->config.encoding != VDEF_ENCODING_H264) return
 * -EINVAL"). */
static void testCxxCreateVideoEncoderWithInvalidEncodingFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawVideoMedia()->id;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_JPEG;
	encoderParams.mjpeg.quality = 80;
	encoderParams.mjpeg.max_bitrate = 2000000;
	IPdraw::IVideoEncoder *encoder = nullptr;
	ret = session->createVideoEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_video_encoder_listener,
					  &encoder);
	CU_ASSERT_NOT_EQUAL(ret, 0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoEncoder::VideoEncoder()'s .name/.device copy branches
 * (pdraw_encoder_video.cpp:106-111: "if (mEncoderConfig->name != nullptr)
 * mEncoderName = std::string(mEncoderConfig->name);" and the analogous
 * .device -> mEncoderDevice one) were never exercised: every other test in
 * this file leaves both fields at their zero-init nullptr, so these two
 * `if`s were always false. venc_config_copy() (venc.c:346-347, called from
 * the constructor at pdraw_encoder_video.cpp:101) always runs regardless of
 * .encoding/.implem, so a config with only .name/.device added on top of an
 * otherwise normal, successful H.264 config reaches both branches while
 * still letting createVideoEncoder() succeed end-to-end -- no need to
 * inspect the (private) mEncoderName/mEncoderDevice members themselves, a
 * successful creation already proves the copy path ran without issue. */
static void testCxxCreateVideoEncoderWithNameAndDeviceSucceeds()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawVideoMedia()->id;

	struct venc_config encoderParams = {};
	encoderParams.name = "test-video-encoder";
	encoderParams.device = "/dev/test-video-encoder0";
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	IPdraw::IVideoEncoder *encoder = nullptr;
	ret = session->createVideoEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_video_encoder_listener,
					  &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	encoderOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoEncoder::start()'s else branch (pdraw_encoder_video.cpp:219-241) runs
 * when the VideoEncoder was constructed with params==nullptr: it allocates a
 * default venc_config and fills it with DEFAULT_ENCODING (H264), AUTO implem,
 * and a max_bitrate of 10 Mbps. Session::createVideoEncoder() guards against
 * null params with -EINVAL (tested in test_api_encoder_video.cpp::testCxxCreate
 * NullParams), so the else branch is only reachable via direct VideoEncoder
 * construction — exactly as addVideoEncoderForMedia does when it allocates its
 * own VideoEncoder with encoder==nullptr (see pdraw_session.cpp:1970-1975).
 * Verify that start() succeeds: a broken else branch (e.g. a misconfigured
 * default config) would cause venc_new() to fail and start() to return
 * non-zero. */
static void testCxxVideoEncoderNullConfigDefaultsToH264()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	/* Kept well above NVENC's hardware minimum encode dimensions
	 * (empirically ~148 width / ~50 height on a Blackwell RTX 5080's
	 * h264_nvenc; varies by GPU/driver generation): this test's whole
	 * point is exercising the nullptr-params AUTO-implem default, which
	 * on a machine with ffmpeg+NVENC resolves to h264_nvenc, not x264. */
	constexpr uint32_t kWidth = 320;
	constexpr uint32_t kHeight = 240;

	/* Standalone media — not registered in any source's output port.
	 * addInputMedia() only checks format caps (set from
	 * venc_get_supported_input_formats at construction time) and creates a
	 * channel; it never walks the session's element list. */
	RawVideoMedia media(session);
	media.format = vdef_i420;
	media.info.resolution.width = kWidth;
	media.info.resolution.height = kHeight;
	media.info.bit_depth = 8;
	media.info.framerate.num = 30;
	media.info.framerate.den = 1;

	/* Construct directly with params==nullptr — triggers the else branch.
	 */
	VideoEncoder encoder(session,
			     session, /* Element::Listener */
			     session, /* Source::Listener */
			     &g_stub_video_encoder_listener,
			     nullptr, /* wrapper */
			     nullptr /* params — null triggers else branch */);

	int ret = encoder.addInputMedia(&media);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = encoder.start();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STARTED);

	/* start()'s own idempotency guard (pdraw_encoder_video.cpp:152-153:
	 * "if ((mState == State::STARTED) || (mState == State::STARTING))
	 * return 0;") was never exercised: every other test calls start()
	 * exactly once. Calling it again while already STARTED must be a
	 * harmless no-op (early "return 0", nothing re-initialized). */
	ret = encoder.start();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STARTED);

	ret = encoder.stop();
	CU_ASSERT_EQUAL(ret, 0);

	bool stopped = loop.pumpUntil(
		[&encoder]() {
			return encoder.getState() == Element::State::STOPPED;
		},
		5000);
	CU_ASSERT_TRUE(stopped);
	/* One extra iteration: asyncElementDelete idle was queued when the
	 * encoder transitioned to STOPPED; let it fire and log its "not found"
	 * warning (encoder was never added to mElements, which is expected for
	 * a directly-constructed VideoEncoder). */
	loop.runOnce();

	/* encoder and media go out of scope here (LIFO: encoder before media),
	 * then session is stopped by the TestSession destructor. */
}


/* VideoEncoder::configure()/getConfig()/requestKeyFrame()
 * (pdraw_encoder_video.cpp:620-624/637-641/654-658, respectively) all share
 * the exact same guard -- "if (mState != State::STARTED) { ...; return
 * -EPROTO; }" -- and were all at 0% on that branch: every other test
 * exercising these three methods
 * (testCxxVideoEncoderConfigureAndGetConfigRoundTrips,
 * testCxxVideoEncoderRequestKeyFrameProducesIdrFrame) only calls them once
 * the encoder is already STARTED. createVideoEncoder() always starts the
 * encoder synchronously before returning a handle to the caller, so
 * reaching CREATED (never started) requires bypassing the public factory
 * and directly constructing a VideoEncoder, exactly as
 * testCxxVideoEncoderNullConfigDefaultsToH264 above does -- except here
 * start() is deliberately never called. No input media is needed: the
 * guard is checked before mInputMedia/mVenc are ever touched. */
static void testCxxVideoEncoderNotStartedGuards()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	VideoEncoder encoder(session,
			     session, /* Element::Listener */
			     session, /* Source::Listener */
			     &g_stub_video_encoder_listener,
			     nullptr, /* wrapper */
			     nullptr /* params */);
	CU_ASSERT_EQUAL_FATAL(encoder.getState(), Element::State::CREATED);

	struct venc_dyn_config dynConfig = {};
	CU_ASSERT_EQUAL(encoder.configure(&dynConfig), -EPROTO);
	CU_ASSERT_EQUAL(encoder.getConfig(&dynConfig), -EPROTO);
	CU_ASSERT_EQUAL(encoder.requestKeyFrame(), -EPROTO);

	int ret = encoder.stop();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STOPPED);

	/* Same reasoning as testCxxVideoEncoderNullConfigDefaultsToH264: let
	 * the asyncElementDelete idle (queued synchronously by setState()
	 * above, through Session::onElementStateChanged()) fire before
	 * encoder goes out of scope. */
	loop.runOnce();
}


/* VideoEncoder::onRawVideoChannelQueue()'s "not started" guard
 * (pdraw_encoder_video.cpp:834-836: "if (mState != State::STARTED) {
 * PDRAW_LOGE(...); return; }") was never exercised: every test that pushes
 * frames into an encoder's input channel does so only after start()
 * succeeded. RawVideoChannel::queue() (public,
 * pdraw_channel_raw_video.cpp:79-87) directly invokes this callback, and
 * Sink::getInputChannel() (public) returns the channel created by
 * addInputMedia() -- independently of start() -- so a frame can be pushed
 * on a never-started (CREATED) encoder without any private-member access. */
static void testCxxVideoEncoderQueueBeforeStartedGuard()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;

	RawVideoMedia media(session);
	media.format = vdef_i420;
	media.info.resolution.width = kWidth;
	media.info.resolution.height = kHeight;
	media.info.bit_depth = 8;
	media.info.framerate.num = 30;
	media.info.framerate.den = 1;

	VideoEncoder encoder(session,
			     session, /* Element::Listener */
			     session, /* Source::Listener */
			     &g_stub_video_encoder_listener,
			     nullptr, /* wrapper */
			     nullptr /* params */);

	int ret = encoder.addInputMedia(&media);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(encoder.getState(), Element::State::CREATED);

	Channel *c = encoder.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(c);
	auto *channel = dynamic_cast<RawVideoChannel *>(c);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	struct mbuf_raw_video_frame *frame =
		makeI420Frame(kWidth, kHeight, 0, 0);
	CU_ASSERT_EQUAL(channel->queue(frame), 0);
	mbuf_raw_video_frame_unref(frame);

	int stopRet = encoder.stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STOPPED);

	/* Same reasoning as testCxxVideoEncoderNullConfigDefaultsToH264. */
	loop.runOnce();
}


/* VideoEncoderWrapper::configure()/getConfig()/requestKeyFrame()
 * (pdraw_encoder_video.cpp:1183-1204) each have their own "if
 * (isElementStopped()) return -EPROTO;" guard, layered on top of (and
 * distinct from) VideoEncoder's own "not started" guard already covered by
 * testCxxVideoEncoderNotStartedGuards above -- that test calls the internal
 * VideoEncoder directly, never through the wrapper, so this guard's
 * "return -EPROTO" side was still at 0% (the "if" itself is already
 * exercised every time any of these 3 methods succeeds elsewhere in this
 * file, since isElementStopped() is evaluated regardless of the outcome).
 *
 * isElementStopped() only becomes true once the underlying Element has been
 * destroyed: Element::~Element() (pdraw_element.cpp:58-67) calls
 * mWrapper->clearElement() as its very last step. Getting there SAFELY
 * requires letting the real, asynchronous teardown cascade run to
 * completion while the wrapper itself is kept alive (not yet reset) -- same
 * idiom as testCxxVideoScalerTeardownWhenSourceStops
 * (test_pipeline_scaler_video.cpp): destroy the *source* only, and wait for
 * the wrapper's own getVideoEncoder() to observe nullptr.
 *
 * A first version of this test tried to shortcut this by calling
 * clearElement() directly on the still-live wrapper instead of waiting for
 * a real teardown. That is unsound: clearElement() is one half of a mutual
 * handshake with ElementWrapper::~ElementWrapper() -> Element::
 * clearWrapper() (pdraw_element.cpp:198-203), which nulls the *Element's*
 * own back-pointer to the wrapper. Calling clearElement() out of band nulls
 * the wrapper's mElement first, so when the wrapper is later reset,
 * ~ElementWrapper()'s "if (mElement != nullptr)" guard sees it already null
 * and skips clearWrapper() -- leaving the real (still-alive) Element's
 * mWrapper dangling once the session's async delete finally destroys it,
 * which then dereferences the already-freed wrapper. Confirmed the hard
 * way: this shortcut produced a real ASan heap-use-after-free in
 * Element::~Element() on a run of this test. Fixed by only ever observing
 * the real teardown, never forcing it. */
static void testCxxVideoEncoderWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoEncoder *encoder = nullptr;
	IPdraw::ICodedVideoSink *sink = nullptr;
	AckingCodedVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createEncoderChainAndPrime(session,
				   &loop,
				   &mediaListener,
				   &sourceListener,
				   &source,
				   &encoder,
				   &sink,
				   &sinkListener,
				   &inQueue);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	(void)inQueue;

	const MediaTrackingListener::Added *codedMediaPtr =
		mediaListener.findCodedVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedMediaPtr);
	unsigned int codedMediaId = codedMediaPtr->id;

	/* Destroy the source only, while the encoder+sink are still alive --
	 * see the comment above for why the order matters here. */
	{
		auto sourceOwner =
			std::unique_ptr<IPdraw::IRawVideoSource>(source);
	}

	bool gotRemoved = loop.pumpUntil(
		[&]() { return mediaListener.wasRemoved(codedMediaId); },
		15000);
	CU_ASSERT_TRUE_FATAL(gotRemoved);

	/* Wait for the real VideoEncoder element to fully stop and be
	 * destroyed by the session -- only then is it safe to call the
	 * wrapper's own methods and expect isElementStopped() == true. */
	bool gotEncoderStopped = loop.pumpUntil(
		[&]() {
			return static_cast<VideoEncoderWrapper *>(encoder)
				       ->getVideoEncoder() == nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotEncoderStopped);

	struct venc_dyn_config dynConfig = {};
	CU_ASSERT_EQUAL(encoder->configure(&dynConfig), -EPROTO);
	CU_ASSERT_EQUAL(encoder->getConfig(&dynConfig), -EPROTO);
	CU_ASSERT_EQUAL(encoder->requestKeyFrame(), -EPROTO);

	stopSessionAndWait(&loop, session, &mediaListener);

	sinkOwner.reset();
	encoderOwner.reset();
}


/* Only checks that createVideoEncoder() succeeds against a real decoded
 * media -- deeper verification (does it actually encode?) is
 * testCxxVideoEncoderProducesRealH264Output below. */
static void testCxxCreateVideoEncoderWithRealDecodedMedia()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

	/* Wait for the internally auto-created VideoDecoder to output its
	 * first decoded frame: that's when it creates its raw output media
	 * and onMediaAdded() fires for it (see
	 * VideoDecoder::createOutputMedia in pdraw_decoder_video.cpp). */
	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideo =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideo);
	CU_ASSERT_NOT_EQUAL_FATAL(rawVideo->id, 0u);

	/* The input sub-structure is ignored; only the encoding type and
	 * (for CBR/VBR, the default rate control) max_bitrate are mandatory. */
	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(rawVideo->id,
					      &encoderParams,
					      &g_stub_video_encoder_listener,
					      &encoder);
	/* CONFIG_VENC_X264 (or another concrete H.264 encoder backend) is
	 * required for this build: without it, VideoEncoder's constructor
	 * finds zero supported input formats (venc_get_supported_input_formats
	 * fails), so addInputMedia() rejects the raw media's format before
	 * start() is ever reached, propagating -ENOSYS up through
	 * PipelineFactory::addVideoEncoderForMedia -- confirmed the hard way
	 * (an earlier version of this test tolerated -ENOSYS here, which
	 * silently hid a missing CONFIG_VENC_X264 in the product config:
	 * VideoEncoder::start() had 0% coverage despite this test "passing").
	 */
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Unlike testCxxCreateVideoEncoderWithRealDecodedMedia above (creation
 * only), this pushes real decoded frames through the encoder and verifies
 * the output is genuinely encoded H.264: right encoding tag, at least one
 * NALU per frame. Requires a concrete H.264 encoder backend (e.g.
 * CONFIG_VENC_X264) to be enabled in the product config, same as the other
 * test -- see its comment for why this is asserted strictly rather than
 * tolerating -ENOSYS. */
static void testCxxVideoEncoderProducesRealH264Output()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	/* REALTIME mode: same rationale as
	 * testCxxVideoEncoderProducesRealH265Output
	 * -- limits in-flight frames to ~10-15 at gotFrame time so teardown
	 * stays within the default closeAndDestroyDemuxer() timeout. */
	IPdraw::IDemuxer *demuxer =
		openDecodingDemuxerAndPlay(session,
					   &loop,
					   &demuxListener,
					   demuxerPath,
					   PDRAW_PLAYBACK_MODE_REALTIME);

	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	VideoEncoderOutputListener encoderListener;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(
		rawVideo.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	bool gotFrame = loop.pumpUntil(
		[&encoderListener]() {
			return encoderListener.mGotFrame.load();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	{
		std::lock_guard<std::mutex> lock(encoderListener.mMutex);
		CU_ASSERT_EQUAL(encoderListener.mEncoding, VDEF_ENCODING_H264);
		CU_ASSERT_FATAL(encoderListener.mNaluCount >= 1);
	}

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Same setup as testCxxVideoEncoderProducesRealH264Output, but requests
 * H.265 output instead: the source asset stays H.264 (already proven to
 * decode correctly), only the *encoder*'s target encoding changes -- this
 * isolates the thing actually under test (a real HEVC encoder backend, e.g.
 * CONFIG_VENC_X265) from decode concerns (covered in
 * test_pipeline_decoder_video.cpp instead). venc_x265's create() rejects
 * anything but VDEF_ENCODING_H265 (see venc_x265.c), so a successful start()
 * here is proof a real x265 (or equivalent) backend is wired in, not a
 * fallback. */
static void testCxxVideoEncoderProducesRealH265Output()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	/* Use REALTIME mode so the demuxer paces itself at the video's natural
	 * frame rate (~30 fps). OFFLINE mode floods x265's internal queue with
	 * all frames before the first encoded output arrives; on a slow machine
	 * (x265 built without NASM, ~24 fps) flushing ~387 frames takes ~16 s
	 * and outlasts any reasonable closeAndDestroyDemuxer() timeout. In
	 * REALTIME mode only ~10-15 frames are in flight when gotFrame fires.
	 */
	IPdraw::IDemuxer *demuxer =
		openDecodingDemuxerAndPlay(session,
					   &loop,
					   &demuxListener,
					   demuxerPath,
					   PDRAW_PLAYBACK_MODE_REALTIME);

	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;

	struct venc_config encoderParams = {};
	encoderParams.encoding = VDEF_ENCODING_H265;
	encoderParams.h265.max_bitrate = 2000000;
	VideoEncoderOutputListener encoderListener;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(
		rawVideo.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	bool gotFrame = loop.pumpUntil(
		[&encoderListener]() {
			return encoderListener.mGotFrame.load();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	{
		std::lock_guard<std::mutex> lock(encoderListener.mMutex);
		CU_ASSERT_EQUAL(encoderListener.mEncoding, VDEF_ENCODING_H265);
		CU_ASSERT_FATAL(encoderListener.mNaluCount >= 1);
	}

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Same rationale as testCxxVideoEncoderProducesRealH265Output above, for a
 * JPEG (MJPEG) output encoder. quality ([1..99]) is unconditionally
 * mandatory; max_bitrate is ALSO mandatory here despite venc_turbojpeg's own
 * create() never reading it (it always drives the encoder off mjpeg.quality
 * directly) -- the generic pre-dispatch validation shared by every backend
 * (venc_new() in venc.c) rejects any encoding whose default rate-control
 * mode (CBR, since rate_control is left at 0/VENC_RATE_CONTROL_CBR here) has
 * max_bitrate == 0, before the backend-specific create() is ever reached.
 * Confirmed the hard way: omitting it produced "invalid bitrate" from
 * venc_new(). */
static void testCxxVideoEncoderProducesRealJpegOutput()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	/* REALTIME mode: same rationale as
	 * testCxxVideoEncoderProducesRealH265Output
	 * -- limits in-flight frames at gotFrame time so teardown stays within
	 * the closeAndDestroyDemuxer() timeout. */
	IPdraw::IDemuxer *demuxer =
		openDecodingDemuxerAndPlay(session,
					   &loop,
					   &demuxListener,
					   demuxerPath,
					   PDRAW_PLAYBACK_MODE_REALTIME);

	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;

	struct venc_config encoderParams = {};
	encoderParams.encoding = VDEF_ENCODING_JPEG;
	encoderParams.mjpeg.quality = 80;
	encoderParams.mjpeg.max_bitrate = 2000000;
	VideoEncoderOutputListener encoderListener;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(
		rawVideo.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	bool gotFrame = loop.pumpUntil(
		[&encoderListener]() {
			return encoderListener.mGotFrame.load();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	{
		std::lock_guard<std::mutex> lock(encoderListener.mMutex);
		CU_ASSERT_EQUAL(encoderListener.mEncoding, VDEF_ENCODING_JPEG);
		CU_ASSERT_FATAL(encoderListener.mNaluCount >= 1);
	}

	/* venc_request_idr() (venc.c:1025-1038) rejects any encoding other
	 * than H264/H265 with -EINVAL before ever consulting the backend --
	 * IDR/key frames are an H264/H265-specific concept, meaningless for
	 * JPEG. VideoEncoder::requestKeyFrame() (pdraw_encoder_video.cpp:
	 * 654-666) just propagates that failure ("if (ret < 0) PDRAW_LOG_ERRNO
	 * (...); return ret;"), previously only exercised on its success path
	 * (H264/H265, see testCxxVideoEncoderRequestKeyFrameProducesIdrFrame).
	 */
	CU_ASSERT_EQUAL(encoder->requestKeyFrame(), -EINVAL);

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* IVideoEncoder::configure()/getConfig() are both synchronous (see
 * VideoEncoder::configure/getConfig in pdraw_encoder_video.cpp: no writer
 * thread, no idle dispatch, just a direct venc_set_dyn_config/
 * venc_get_dyn_config call gated on mState == STARTED, which is already true
 * as soon as createVideoEncoder() returns 0 -- see VideoEncoder::start(),
 * called synchronously from Session::PipelineFactory::addVideoEncoderForMedia
 * before createVideoEncoder() ever returns). No frame needs to be pushed
 * through first. venc_x264's set_dyn_config/get_dyn_config store/read
 * target_bitrate directly in base->config.h264.target_bitrate with no
 * rounding or clamping (see venc_x264.c), so an exact round-trip is a valid,
 * strict assertion here (as opposed to e.g. x264's own internal qp/bitrate
 * translation, which is not observed at this level). */
static void testCxxVideoEncoderConfigureAndGetConfigRoundTrips()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	VideoEncoderOutputListener encoderListener;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(
		rawVideo.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	/* Baseline: readable right after creation, before any frame flows.
	 * target_bitrate is left at 0 in encoderParams above (only max_bitrate
	 * is set); the generic pre-dispatch validation shared by every backend
	 * (venc_new() in venc.c, case VDEF_ENCODING_H264: "if
	 * (self->config.h264.target_bitrate == 0) { ... = max_bitrate; }")
	 * fills it in from max_bitrate before the backend-specific create()
	 * ever runs, so it reads back as 2000000 here, not 0. */
	struct venc_dyn_config baseline = {};
	ret = encoder->getConfig(&baseline);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(baseline.target_bitrate, 2000000u);

	/* Reconfigure to a different bitrate and read it back. */
	struct venc_dyn_config newCfg = {};
	newCfg.target_bitrate = 4000000;
	ret = encoder->configure(&newCfg);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct venc_dyn_config readBack = {};
	ret = encoder->getConfig(&readBack);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(readBack.target_bitrate, 4000000u);

	/* Passing nullptr to configure() or getConfig() triggers
	 * venc_set_dyn_config / venc_get_dyn_config error handling (-EINVAL).
	 */
	CU_ASSERT_EQUAL(encoder->configure(nullptr), -EINVAL);
	CU_ASSERT_EQUAL(encoder->getConfig(nullptr), -EINVAL);

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* IVideoEncoder::requestKeyFrame() only sets an atomic flag consumed by the
 * next real encode call (see venc_x264.c: request_idr()/insert_idr), so
 * proving it actually works requires observing a real, running encode
 * pipeline transition from a non-IDR frame back to an IDR frame on demand --
 * not just that the call returns 0.
 *
 * The forced IDR is not necessarily the very next *output* frame:
 * preferred_thread_count is never read by venc_x264.c at all (confirmed:
 * grepping the file for it finds nothing, no i_threads assignment either),
 * so libx264 always falls back to its own X264_THREADS_AUTO frame-parallel
 * threading -- there is no config knob available through this API to force
 * single-threaded, in-order output. Output frames can therefore lag input
 * submission order by a machine-dependent number of frames even with
 * i_bframe=0 (confirmed the hard way: both "assert the immediate next frame
 * is the IDR" and "assert one of the next 5 frames is" flaked on a real
 * test machine where the actual lag was larger than 5).
 *
 * Rather than guess an even bigger window (still a race against the ~30-
 * frame natural GOP boundary on a fast/many-core machine), the confound is
 * removed instead: gop_length_sec is set far larger than this short clip
 * could ever reach, so the *only* possible source of a post-request IDR is
 * requestKeyFrame() itself -- any IDR found anywhere after the baseline,
 * however many frames later, is unambiguous proof. */
static void testCxxVideoEncoderRequestKeyFrameProducesIdrFrame()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);
	/* REALTIME mode: same rationale as
	 * testCxxVideoEncoderProducesRealH265Output
	 * -- limits in-flight frames at gotFrame time so teardown stays within
	 * the closeAndDestroyDemuxer() timeout. */
	IPdraw::IDemuxer *demuxer =
		openDecodingDemuxerAndPlay(session,
					   &loop,
					   &demuxListener,
					   demuxerPath,
					   PDRAW_PLAYBACK_MODE_REALTIME);

	bool gotRawVideo = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	/* Push the natural GOP boundary far beyond this short clip's reach --
	 * see the comment above this test for why. */
	encoderParams.h264.gop_length_sec = 1000.f;
	VideoEncoderOutputListener encoderListener;
	IPdraw::IVideoEncoder *encoder = nullptr;
	int ret = session->createVideoEncoder(
		rawVideo.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	/* Wait for at least the first (IDR) frame, then a second one -- frame
	 * #2 is expected to already be a P frame (gop_length_sec above only
	 * suppresses *later* natural IDRs; the very first frame is always an
	 * IDR), giving a genuine non-IDR baseline before requesting a fresh
	 * key frame. */
	bool gotTwoFrames = loop.pumpUntil(
		[&encoderListener]() {
			return encoderListener.frameCount() >= 2;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotTwoFrames);

	size_t baselineCount = encoderListener.frameCount();
	ret = encoder->requestKeyFrame();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* No fixed window: with the natural GOP boundary pushed out of reach,
	 * an IDR anywhere after the baseline -- however many frames later,
	 * depending on this machine's x264 thread-parallel pipeline depth --
	 * can only be the one requestKeyFrame() just forced. */
	bool gotIdrAfterRequest = loop.pumpUntil(
		[&]() {
			std::lock_guard<std::mutex> lock(
				encoderListener.mMutex);
			for (size_t i = baselineCount;
			     i < encoderListener.mFrameTypes.size();
			     i++) {
				if (encoderListener.mFrameTypes[i] ==
				    VDEF_CODED_FRAME_TYPE_IDR)
					return true;
			}
			return false;
		},
		15000);
	CU_ASSERT_TRUE(gotIdrAfterRequest);

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


/* ── C API shim test ─────────────────────────────────────────────────────── */

namespace {

struct EncSessionCbState {
	unsigned int lastRawMediaId = 0;
	int stopRespCount = 0;
};

struct EncDmxCbState {
	int openRespStatus = -1;
	int openRespCount = 0;
	int readyToPlayCount = 0;
	int playRespCount = 0;
	int playRespStatus = -1;
	int closeRespCount = 0;
};

struct EncCbState {
	std::atomic<int> frameOutputCount{0};
	std::atomic<int> framePreReleaseCount{0};
};

} /* anonymous namespace */

static void enc_session_stop_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<EncSessionCbState *>(ud)->stopRespCount++;
}

static void enc_session_media_added_cb(struct pdraw * /*p*/,
				       const struct pdraw_media_info *info,
				       void * /*elem*/,
				       void *ud)
{
	if (info->type == PDRAW_MEDIA_TYPE_VIDEO &&
	    info->video.format == VDEF_FRAME_TYPE_RAW)
		static_cast<EncSessionCbState *>(ud)->lastRawMediaId = info->id;
}

static void enc_session_media_removed_cb(struct pdraw * /*p*/,
					 const struct pdraw_media_info * /*i*/,
					 void * /*elem*/,
					 void * /*ud*/)
{
}

static void enc_dmx_open_resp_cb(struct pdraw * /*p*/,
				 struct pdraw_demuxer * /*d*/,
				 int status,
				 void *ud)
{
	auto *s = static_cast<EncDmxCbState *>(ud);
	s->openRespStatus = status;
	s->openRespCount++;
}

static void enc_dmx_close_resp_cb(struct pdraw * /*p*/,
				  struct pdraw_demuxer * /*d*/,
				  int /*status*/,
				  void *ud)
{
	static_cast<EncDmxCbState *>(ud)->closeRespCount++;
}

static void enc_dmx_ready_to_play_cb(struct pdraw * /*p*/,
				     struct pdraw_demuxer * /*d*/,
				     int ready,
				     void *ud)
{
	if (ready)
		static_cast<EncDmxCbState *>(ud)->readyToPlayCount++;
}

static void enc_dmx_play_resp_cb(struct pdraw * /*p*/,
				 struct pdraw_demuxer * /*d*/,
				 int status,
				 uint64_t /*ts*/,
				 float /*speed*/,
				 void *ud)
{
	auto *s = static_cast<EncDmxCbState *>(ud);
	s->playRespStatus = status;
	s->playRespCount++;
}

static void enc_frame_output_cb(struct pdraw * /*p*/,
				struct pdraw_video_encoder * /*e*/,
				struct mbuf_coded_video_frame * /*frame*/,
				void *ud)
{
	static_cast<EncCbState *>(ud)->frameOutputCount.fetch_add(
		1, std::memory_order_relaxed);
}

static void enc_frame_pre_release_cb(struct pdraw * /*p*/,
				     struct pdraw_video_encoder * /*e*/,
				     struct mbuf_coded_video_frame * /*frame*/,
				     void *ud)
{
	static_cast<EncCbState *>(ud)->framePreReleaseCount.fetch_add(
		1, std::memory_order_relaxed);
}

/* Covers both PdrawVideoEncoderListener shims (frame_output, frame_pre_release)
 * and the four most important PdrawDemuxerListener shims (open_resp,
 * close_resp, ready_to_play, play_resp) via a real file-based decode pipeline.
 * frame_pre_release fires on the encoder's own output thread when the frame's
 * last reference is dropped; std::atomic counters handle cross-thread delivery.
 */
static void testCVideoEncoderListenerCallbacks()
{
	TestPompLoop loop;

	EncSessionCbState sessState;
	struct pdraw_cbs sessCbs = {};
	sessCbs.stop_resp = enc_session_stop_cb;
	sessCbs.media_added = enc_session_media_added_cb;
	sessCbs.media_removed = enc_session_media_removed_cb;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessCbs, &sessState, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_encoder_video);

	EncDmxCbState dmxState;
	struct pdraw_demuxer_params dmxParams = {};
	dmxParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL;
	struct pdraw_demuxer_cbs dmxCbs = {};
	dmxCbs.open_resp = enc_dmx_open_resp_cb;
	dmxCbs.close_resp = enc_dmx_close_resp_cb;
	dmxCbs.ready_to_play = enc_dmx_ready_to_play_cb;
	dmxCbs.play_resp = enc_dmx_play_resp_cb;
	struct pdraw_demuxer *demuxer = nullptr;
	ret = pdraw_demuxer_new_from_url(
		p, demuxerPath, &dmxParams, &dmxCbs, &dmxState, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	/* open_resp → demuxerOpenResponse shim */
	bool gotOpen = loop.pumpUntil(
		[&dmxState]() { return dmxState.openRespCount >= 1; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotOpen);
	CU_ASSERT_EQUAL_FATAL(dmxState.openRespStatus, 0);

	/* ready_to_play → demuxerReadyToPlay shim */
	bool gotReady = loop.pumpUntil(
		[&dmxState]() { return dmxState.readyToPlayCount >= 1; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	ret = pdraw_demuxer_play(p, demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* play_resp → demuxerPlayResponse shim */
	bool gotPlayResp = loop.pumpUntil(
		[&dmxState]() { return dmxState.playRespCount >= 1; }, 5000);
	CU_ASSERT_TRUE(gotPlayResp);
	CU_ASSERT_EQUAL(dmxState.playRespStatus, 0);

	/* Wait for the decoder to produce a raw video media */
	bool gotRawMedia = loop.pumpUntil(
		[&sessState]() { return sessState.lastRawMediaId != 0; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	EncCbState encState;
	struct pdraw_video_encoder_cbs encCbs = {};
	encCbs.frame_output = enc_frame_output_cb;
	encCbs.frame_pre_release = enc_frame_pre_release_cb;
	struct pdraw_video_encoder *encoder = nullptr;
	ret = pdraw_video_encoder_new(p,
				      sessState.lastRawMediaId,
				      &encoderParams,
				      &encCbs,
				      &encState,
				      &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);

	/* frame_output → videoEncoderFrameOutput shim */
	bool gotFrame = loop.pumpUntil(
		[&encState]() {
			return encState.frameOutputCount.load(
				       std::memory_order_acquire) >= 1;
		},
		15000);
	CU_ASSERT_TRUE(gotFrame);
	CU_ASSERT_TRUE(encState.frameOutputCount.load() >= 1);

	/* frame_pre_release → videoEncoderFramePreRelease shim:
	 * fires on the encoder thread when the last ref to the frame is
	 * dropped, which happens as soon as the frame is pushed to output
	 * channels (or immediately if no downstream sink holds a reference). */
	bool gotPreRelease = loop.pumpUntil(
		[&encState]() {
			return encState.framePreReleaseCount.load(
				       std::memory_order_acquire) >= 1;
		},
		5000);
	CU_ASSERT_TRUE(gotPreRelease);
	CU_ASSERT_TRUE(encState.framePreReleaseCount.load() >= 1);

	pdraw_video_encoder_destroy(p, encoder);

	/* close_resp → demuxerCloseResponse shim */
	ret = pdraw_demuxer_close(p, demuxer);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&dmxState]() { return dmxState.closeRespCount >= 1; }, 15000);
	CU_ASSERT_TRUE(gotClose);
	pdraw_demuxer_destroy(p, demuxer);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&sessState]() { return sessState.stopRespCount >= 1; }, 15000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


static void testCxxVideoEncoderStartWithoutInputMediaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	auto encoder = std::unique_ptr<VideoEncoder>(new VideoEncoder(
		session, nullptr, nullptr, nullptr, nullptr, nullptr));
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder.get());

	/* 1. start() without input media -> fails with -EPROTO */
	int res = encoder->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 2. start() when state is not CREATED (now STOPPED) -> fails with
	 * -EPROTO */
	res = encoder->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 3. stop() when state is STOPPED -> returns 0 */
	res = encoder->stop();
	CU_ASSERT_EQUAL(res, 0);
}


static void testCxxVideoEncoderFrameOutputCbCoverage()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	VideoEncoder encoder(
		session, nullptr, nullptr, nullptr, nullptr, nullptr);

	/* 1. status != 0 -> error logging */
	VideoEncoder::frameOutputCb(nullptr, -EINVAL, nullptr, &encoder);

	/* 2. userdata == nullptr */
	VideoEncoder::frameOutputCb(nullptr, 0, nullptr, nullptr);

	/* 3. out_frame == nullptr */
	VideoEncoder::frameOutputCb(nullptr, 0, nullptr, &encoder);

	/* 4. state != STARTED */
	struct mbuf_coded_video_frame *frame = nullptr;
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format.encoding = VDEF_ENCODING_H264;
	frameInfo.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* 4a. state != STARTED */
	CU_ASSERT_NOT_EQUAL(encoder.getState(), Element::State::STARTED);
	VideoEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4b. state == STARTED with flush pending -> discards frame */
	encoder.setState(Element::State::STARTED);
	encoder.setFlushingState(Element::FlushingState::FLUSHING, true);
	encoder.mVencFlushPending = true;
	CU_ASSERT_TRUE(encoder.mVencFlushPending);
	VideoEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4c. state == STARTED with mInputMedia == nullptr -> invalid input
	 * media */
	encoder.mVencFlushPending = false;
	CU_ASSERT_PTR_NULL(encoder.mInputMedia);
	VideoEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4d. state == STARTED with mInputMedia set, but frame has no ancillary
	 * data */
	RawVideoMedia media(session);
	encoder.mInputMedia = &media;
	CU_ASSERT_PTR_NOT_NULL(encoder.mInputMedia);
	VideoEncoder::frameOutputCb(nullptr, 0, frame, &encoder);
	encoder.mInputMedia = nullptr;
	encoder.setState(Element::State::STOPPED);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STOPPED);

	mbuf_coded_video_frame_unref(frame);

	/* 5. nullptr guards for callbacks and channel events */
	VideoEncoder::flushCb(nullptr, nullptr);
	VideoEncoder::stopCb(nullptr, nullptr);

	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);
	encoder.onChannelFlush(nullptr);
	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);

	encoder.onChannelDrain(nullptr);
	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);

	encoder.mVencFlushPending = true;
	VideoEncoder::flushCb(nullptr, &encoder);
	CU_ASSERT_FALSE(encoder.mVencFlushPending);
}


CU_TestInfo g_pdraw_test_pipeline_encoder_video[] = {
	{FN("testCxxCreateVideoEncoderWithEmptyConfigFails"),
	 testCxxCreateVideoEncoderWithEmptyConfigFails},
	{FN("testCxxCreateVideoEncoderWithInvalidEncodingFails"),
	 testCxxCreateVideoEncoderWithInvalidEncodingFails},
	{FN("testCxxCreateVideoEncoderWithNameAndDeviceSucceeds"),
	 testCxxCreateVideoEncoderWithNameAndDeviceSucceeds},
	{FN("testCxxVideoEncoderNullConfigDefaultsToH264"),
	 testCxxVideoEncoderNullConfigDefaultsToH264},
	{FN("testCxxVideoEncoderNotStartedGuards"),
	 testCxxVideoEncoderNotStartedGuards},
	{FN("testCxxVideoEncoderQueueBeforeStartedGuard"),
	 testCxxVideoEncoderQueueBeforeStartedGuard},
	{FN("testCxxVideoEncoderWrapperGuardsAfterElementCleared"),
	 testCxxVideoEncoderWrapperGuardsAfterElementCleared},
	{FN("testCxxCreateVideoEncoderWithRealDecodedMedia"),
	 testCxxCreateVideoEncoderWithRealDecodedMedia},
	{FN("testCxxVideoEncoderProducesRealH264Output"),
	 testCxxVideoEncoderProducesRealH264Output},
	{FN("testCxxVideoEncoderProducesRealH265Output"),
	 testCxxVideoEncoderProducesRealH265Output},
	{FN("testCxxVideoEncoderProducesRealJpegOutput"),
	 testCxxVideoEncoderProducesRealJpegOutput},
	{FN("testCxxVideoEncoderConfigureAndGetConfigRoundTrips"),
	 testCxxVideoEncoderConfigureAndGetConfigRoundTrips},
	{FN("testCxxVideoEncoderRequestKeyFrameProducesIdrFrame"),
	 testCxxVideoEncoderRequestKeyFrameProducesIdrFrame},
	{FN("testCxxVideoEncoderFlushCascadesThroughSourceAndSink"),
	 testCxxVideoEncoderFlushCascadesThroughSourceAndSink},
	{FN("testCxxVideoEncoderDrainCascadesThroughSourceAndSink"),
	 testCxxVideoEncoderDrainCascadesThroughSourceAndSink},
	{FN("testCxxVideoEncoderSessionMetaUpdateCascadesThroughSourceAndSink"),
	 testCxxVideoEncoderSessionMetaUpdateCascadesThroughSourceAndSink},
	{FN("testCVideoEncoderListenerCallbacks"),
	 testCVideoEncoderListenerCallbacks},
	{FN("testCxxVideoEncoderStartWithoutInputMediaFails"),
	 testCxxVideoEncoderStartWithoutInputMediaFails},
	{FN("testCxxVideoEncoderFrameOutputCbCoverage"),
	 testCxxVideoEncoderFrameOutputCbCoverage},
	CU_TEST_INFO_NULL,
};
