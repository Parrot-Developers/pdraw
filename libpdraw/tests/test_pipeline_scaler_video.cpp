/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — video scaler on a real decoded media, and on a
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
 * VideoScaler test. The shared demuxer-fixture helpers
 * (PipelineDemuxerListener, openDecodingDemuxerAndPlay, etc.) are now in
 * test_pipeline_common.hpp; local listener classes (MediaTrackingListener,
 * DrainTrackingRawVideoSourceListener, AckingRawVideoSinkListener) remain
 * per-translation-unit.
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the decoder's/
 * scaler's output media requires a real session-wide listener, so each test
 * below builds its own private TestPompLoop + TestSession, fully
 * self-contained. No suite init/cleanup is registered for this file (see
 * test_main.c: NULL, NULL). */

#define ULOG_TAG pdraw_test_pipeline_scaler

#include <complex>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "pdraw_element.hpp"
#include "pdraw_scaler_video.hpp"
#undef protected
#undef private

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include "pdraw_external_raw_video_sink.hpp"

/* struct vscale_config is already visible transitively via pdraw_defs.h
 * (included from test_api_common.hpp). */

#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <string.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── MP4 fixture (same NAS asset as test_api_demuxer.cpp) ────────────────── */

enum { ASSET_VIDEO_H264 = 0 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_scaler[] = {
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

	/* First raw video media matching a specific resolution, or nullptr if
	 * none yet. Used to distinguish a VideoScaler's own (lazily created)
	 * output media from the decoder's/source's, once both exist and both
	 * are raw video: since the scaler is deliberately configured with a
	 * different resolution than its input, matching on resolution
	 * unambiguously identifies which is which. */
	const Added *findRawVideoMediaWithResolution(uint32_t width,
						     uint32_t height) const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_RAW &&
			    a.videoInfo.resolution.width == width &&
			    a.videoInfo.resolution.height == height)
				return &a;
		}
		return nullptr;
	}

	/* Overrides the no-op base implementation: needed to observe the
	 * scaler's own output media being torn down as a consequence of
	 * VideoScaler::onChannelTeardown -> stop() actually running (see
	 * testCxxVideoScalerTeardownWhenSourceStops below). */
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


/* Tracks IRawVideoSource::flush()/drain() completion -- same rationale as
 * DrainTrackingCodedVideoSourceListener in test_pipeline_sourcesink_coded.cpp
 * (duplicated here rather than shared, per this suite's convention: each
 * test file's listeners are private to its own translation unit). Used
 * below to prove a flush()/drain() issued on a raw video source cascades
 * all the way through an intervening VideoScaler (exercising its
 * onChannelFlush/onChannelDrain/onChannelFlushed/onChannelDrained, entirely
 * uncovered otherwise: IVideoScaler has no public flush()/drain() of its
 * own to call directly) and back, rather than just checking the scaler
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


/* Implements the sink side of the flush/drain protocol for real, for a raw
 * video sink -- same rationale as QueueDrainingRawVideoSinkListener in
 * test_pipeline_sourcesink_raw.cpp (duplicated, not shared, same
 * convention). */
class AckingRawVideoSinkListener : public IPdraw::IRawVideoSink::Listener {
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
		discardQueue();
		sk->queueFlushed();
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		discardQueue();
		sk->queueDrained();
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session *meta) override
	{
		if (meta != nullptr)
			mMeta = *meta;
		mGotSessionMetaUpdate = true;
	}

	/* Must be set right after createRawVideoSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_raw_video_frame_queue *mQueue = nullptr;
	struct vmeta_session mMeta = {};
	bool mGotSessionMetaUpdate = false;

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


/* A minimal, correctly-strided/sized I420 frame (see
 * vdef_calc_raw_frame_size() usage in test_pipeline_muxer_record.cpp for the
 * same pattern) -- this is fed to a real libyuv scaler, which reads actual
 * pixel data according to the declared resolution/stride, so the planes must be
 * genuinely, correctly sized (content itself is irrelevant: flat grey). */
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


/* Build a standalone ExternalRawVideoSource -> VideoScaler -> (real)
 * ExternalRawVideoSink chain and push a priming frame so the scaler's
 * (lazily created, see VideoScaler::onOutputFrame) output media exists.
 * sourceListener is passed in (rather than hardcoded to a stub) because
 * IRawVideoSource::Listener is fixed at creation time and cannot be swapped
 * afterward: a caller that needs to observe flush()/drain() completion via
 * onRawVideoSourceFlushed()/onRawVideoSourceDrained() must supply its own
 * tracking listener from the very start. */
static void
createScalerChainAndPrime(IPdraw *session,
			  TestPompLoop *loop,
			  MediaTrackingListener *mediaListener,
			  IPdraw::IRawVideoSource::Listener *sourceListener,
			  IPdraw::IRawVideoSource **source,
			  IPdraw::IVideoScaler **scaler,
			  IPdraw::IRawVideoSink **sink,
			  AckingRawVideoSinkListener *sinkListener,
			  struct mbuf_raw_video_frame_queue **inQueue)
{
	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	constexpr uint32_t kScaledWidth = 16;
	constexpr uint32_t kScaledHeight = 16;

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
		 "pdraw_test_pipeline_scaler_flush");

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

	struct vscale_config scalerParams = {};
	scalerParams.output.info.resolution.width = kScaledWidth;
	scalerParams.output.info.resolution.height = kScaledHeight;
	ret = session->createVideoScaler(rawMediaId,
					 &scalerParams,
					 &g_stub_video_scaler_listener,
					 scaler);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*scaler);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);

	for (unsigned int i = 0; i < 3; i++) {
		struct mbuf_raw_video_frame *primer =
			makeI420Frame(kWidth, kHeight, i * 33333, i);
		ret = mbuf_raw_video_frame_queue_push(*inQueue, primer);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(primer);
	}

	bool gotScaledMedia = loop->pumpUntil(
		[&]() {
			return mediaListener->findRawVideoMediaWithResolution(
				       kScaledWidth, kScaledHeight) != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotScaledMedia);
	unsigned int scaledMediaId =
		mediaListener
			->findRawVideoMediaWithResolution(kScaledWidth,
							  kScaledHeight)
			->id;

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createRawVideoSink(
		scaledMediaId, &sinkParams, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);
	sinkListener->mQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);
}


/* IVideoScaler has no public flush()/drain(): only reachable by
 * flushing/draining a real Source feeding it and observing the completion
 * cascade back through VideoScaler::onChannelFlush/onChannelFlushed. */
static void testCxxVideoScalerFlushCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoScaler *scaler = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createScalerChainAndPrime(session,
				  &loop,
				  &mediaListener,
				  &sourceListener,
				  &source,
				  &scaler,
				  &sink,
				  &sinkListener,
				  &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	(void)inQueue;

	int ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	sinkOwner.reset();
	scalerOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Same rationale as testCxxVideoScalerFlushCascadesThroughSourceAndSink
 * above, for drain() instead of flush(). */
static void testCxxVideoScalerDrainCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoScaler *scaler = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createScalerChainAndPrime(session,
				  &loop,
				  &mediaListener,
				  &sourceListener,
				  &source,
				  &scaler,
				  &sink,
				  &sinkListener,
				  &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	(void)inQueue;

	int ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	sinkOwner.reset();
	scalerOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoScaler::onChannelSessionMetaUpdate is a Sink-side callback invoked
 * when SESSION_META_UPDATE propagates downstream through the scaler's input
 * channel; it then forwards to the scaler's own output channel(s) (see
 * FilterElement::onChannelSessionMetaUpdate in pdraw_element.cpp) and
 * reaches the downstream raw video sink. Entirely uncovered otherwise (0%
 * per a real gcov run): none of the other tests in this file ever change
 * session metadata after creation. Only IRawVideoSource has a public
 * setSessionMetadata() to trigger this from a test (see
 * ExternalRawVideoSource::setSessionMetadata in
 * pdraw_external_raw_video_source.cpp) -- same "flush/drain the source,
 * observe the cascade through the filter and back" idiom as the tests
 * above, but for session metadata instead of flush/drain. */
static void testCxxVideoScalerSessionMetaUpdateCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoScaler *scaler = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createScalerChainAndPrime(session,
				  &loop,
				  &mediaListener,
				  &sourceListener,
				  &source,
				  &scaler,
				  &sink,
				  &sinkListener,
				  &inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	(void)inQueue;

	struct vmeta_session meta = {};
	snprintf(meta.friendly_name,
		 sizeof(meta.friendly_name),
		 "pdraw_test_scaler_session_meta_update");
	int ret = source->setSessionMetadata(&meta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotUpdate = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotSessionMetaUpdate;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotUpdate);
	CU_ASSERT_STRING_EQUAL(sinkListener.mMeta.friendly_name,
			       "pdraw_test_scaler_session_meta_update");

	sinkOwner.reset();
	scalerOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoScaler::onChannelTeardown was entirely uncovered (0% per a real gcov
 * run): every other test in this file destroys the scaler itself before (or
 * without ever) destroying the source, so the scaler's own INPUT channel
 * teardown (a DownstreamEvent, only ever sent by whatever produces on that
 * channel -- see Channel::teardown() in pdraw_channel.cpp) never has a
 * chance to run: onChannelTeardown can only fire while the scaler is still
 * alive to receive it. Reversing the order here -- stopping the *source*
 * first, while the scaler and sink are still alive -- lets
 * ExternalRawVideoSource::tryStop() (pdraw_external_raw_video_source.cpp)
 * call channel->teardown() on its own output channel (to the scaler), which
 * dispatches to VideoScaler::onChannelTeardown -> stop(). The scaler
 * completing its own stop() (and, cascading further, its own output channel
 * teardown reaching the sink, unlinking, and completeStop() finally seeing
 * zero remaining output channels -- see VideoScaler::completeStop() in
 * pdraw_scaler_video.cpp) is observed here via the scaler's own (scaled
 * resolution) output media being removed: that can only happen once
 * onChannelTeardown's stop() call has fully run its course. */
static void testCxxVideoScalerTeardownWhenSourceStops()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoScaler *scaler = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	struct mbuf_raw_video_frame_queue *inQueue = nullptr;
	createScalerChainAndPrime(session,
				  &loop,
				  &mediaListener,
				  &sourceListener,
				  &source,
				  &scaler,
				  &sink,
				  &sinkListener,
				  &inQueue);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	(void)inQueue;

	const MediaTrackingListener::Added *scaledMediaPtr =
		mediaListener.findRawVideoMediaWithResolution(16, 16);
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaledMediaPtr);
	unsigned int scaledMediaId = scaledMediaPtr->id;

	/* Destroy the source only, while scaler+sink are still alive --
	 * see the comment above for why the order matters here. */
	{
		auto sourceOwner =
			std::unique_ptr<IPdraw::IRawVideoSource>(source);
	}

	bool gotRemoved = loop.pumpUntil(
		[&]() { return mediaListener.wasRemoved(scaledMediaId); },
		15000);
	CU_ASSERT_TRUE_FATAL(gotRemoved);

	/* Wait for VideoScaler element to complete stopping and clear its
	 * wrapper */
	bool gotScalerStopped = loop.pumpUntil(
		[&]() {
			return static_cast<VideoScalerWrapper *>(scaler)
				       ->getVideoScaler() == nullptr;
		},
		15000);
	CU_ASSERT_TRUE(gotScalerStopped);

	stopSessionAndWait(&loop, session, &mediaListener);

	/* Proves VideoScalerWrapper::clearElement() and
	 * RawVideoSinkWrapper::clearElement() actually ran (not just that
	 * resetting the owners below is harmless). */
	CU_ASSERT_PTR_NULL(
		static_cast<VideoScalerWrapper *>(scaler)->getVideoScaler());
	CU_ASSERT_PTR_NULL(
		static_cast<RawVideoSinkWrapper *>(sink)->getRawVideoSink());

	sinkOwner.reset();
	scalerOwner.reset();
}


/* An entirely empty/all-zero vscale_config (in particular
 * .output.info.resolution left at {0, 0}) was never exercised: every other
 * test in this file always sets it explicitly. Read vscale_new() in
 * packages/libvideo-scale/src/vscale.c: unlike VideoEncoder's equivalent gap
 * (where the empty-config rejection only happens deep in each individual
 * backend's create()), this check is in the generic core, before any
 * backend-specific dispatch (vscale.c:183-193: "if (vdef_dim_is_null(...
 * input...) || vdef_dim_is_null(...output...)) ... return -EINVAL") -- so
 * this is expected to fail predictably regardless of which concrete scaler
 * backend(s) are compiled into this product. Unlike VideoEncoder (whose
 * constructor always deep-copies a non-null params into mEncoderConfig, see
 * pdraw_encoder_video.cpp), VideoScaler behaves the same way here: passing a
 * non-null but all-zero vscale_config takes the "override the input config"
 * branch in VideoScaler::start() (pdraw_scaler_video.cpp:187), not the
 * "default to 1280x720" branch (only reachable when the constructor's params
 * argument itself was nullptr, which createVideoScaler() never allows). */
static void testCxxCreateVideoScalerWithEmptyConfigFails()
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

	struct vscale_config scalerParams = {};
	IPdraw::IVideoScaler *scaler = nullptr;
	ret = session->createVideoScaler(rawMediaId,
					 &scalerParams,
					 &g_stub_video_scaler_listener,
					 &scaler);
	CU_ASSERT_NOT_EQUAL(ret, 0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoScaler::VideoScaler()'s .name copy branch (pdraw_scaler_video.cpp:
 * 99-104: "if (params->name != nullptr) { mScalerName = params->name;
 * mScalerConfig->name = mScalerName.c_str(); } else { mScalerConfig->name =
 * nullptr; }") was never exercised on its non-null side -- every other test
 * in this file leaves .name at its zero-init nullptr. Unlike venc_config/
 * aenc_config (deep-copied via a library-level *_config_copy() call),
 * vscale_config has no .device field and the constructor copies .name
 * itself, inline, whenever params != nullptr (which createVideoScaler()
 * always guarantees) -- regardless of .output.info.resolution -- so setting
 * only .name on top of an otherwise normal, successful config reaches the
 * non-null branch while still letting createVideoScaler() succeed
 * end-to-end. */
static void testCxxCreateVideoScalerWithNameSucceeds()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	constexpr uint32_t kScaledWidth = 16;
	constexpr uint32_t kScaledHeight = 16;
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

	struct vscale_config scalerParams = {};
	scalerParams.name = "test-video-scaler";
	scalerParams.output.info.resolution.width = kScaledWidth;
	scalerParams.output.info.resolution.height = kScaledHeight;
	IPdraw::IVideoScaler *scaler = nullptr;
	ret = session->createVideoScaler(rawMediaId,
					 &scalerParams,
					 &g_stub_video_scaler_listener,
					 &scaler);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaler);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);

	scalerOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoScaler::start()'s else branch (pdraw_scaler_video.cpp:192-206) runs
 * when the VideoScaler was constructed with params==nullptr: it allocates a
 * default vscale_config (AUTO implem, output defaulting to 1280×720).
 * Session::createVideoScaler() guards against null params with -EINVAL, so
 * the else branch is only reachable via direct VideoScaler construction —
 * exactly as addVideoScalerForMedia does with scaler==nullptr internally. */
static void testCxxVideoScalerNullConfigDefaultsTo1280x720()
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

	VideoScaler scaler(session,
			   session, /* Element::Listener */
			   session, /* Source::Listener */
			   &g_stub_video_scaler_listener,
			   nullptr, /* wrapper */
			   nullptr /* params — null triggers else branch */);

	int ret = scaler.addInputMedia(&media);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = scaler.start();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(scaler.getState(), Element::State::STARTED);

	ret = scaler.stop();
	CU_ASSERT_EQUAL(ret, 0);

	bool stopped = loop.pumpUntil(
		[&scaler]() {
			return scaler.getState() == Element::State::STOPPED;
		},
		5000);
	CU_ASSERT_TRUE(stopped);
	loop.runOnce(); /* drain asyncElementDelete idle */
}


/* Only checks that createVideoScaler() succeeds against a real decoded
 * media (mirrors testCxxCreateAudioEncoderWithRealDecodedMedia's naming and
 * "creation only" scope in test_pipeline_encoder_audio.cpp) -- deeper
 * verification (does it actually resize?) is
 * testCxxVideoScalerProducesRequestedResolution below. */
static void testCxxCreateVideoScalerWithRealDecodedMedia()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_scaler);
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

	/* The input sub-structure is ignored by createVideoScaler() (it is
	 * derived internally from the connected media); only output.info
	 * (width/height) is mandatory. Requesting the same resolution as the
	 * source is a trivial, always-valid identity scale. */
	struct vscale_config scalerParams = {};
	scalerParams.output.info = rawVideo->videoInfo;
	IPdraw::IVideoScaler *scaler = nullptr;
	int ret = session->createVideoScaler(rawVideo->id,
					     &scalerParams,
					     &g_stub_video_scaler_listener,
					     &scaler);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(scaler);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);

	scalerOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Unlike testCxxCreateVideoScalerWithRealDecodedMedia above (creation
 * only), this pushes the scaler for real and verifies the frame that comes
 * out the other end actually has the requested resolution -- proof of a
 * genuine resize, not just a media descriptor claiming one. The requested
 * resolution is deliberately half the source's (derived at runtime from the
 * real decoded media, not hardcoded): if the scaler silently passed frames
 * through unmodified, this would still be caught, since the sink would then
 * never see a frame at half size at all (findRawVideoMediaWithResolution
 * would time out on the halved dimensions the scaler was asked to reach, or
 * the sink's frame_info would show the original resolution instead). */
static void testCxxVideoScalerProducesRequestedResolution()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_scaler);
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
	/* Copied by value: creating the scaler below pumps the loop, and any
	 * resulting onMediaAdded() could reallocate mAdded and invalidate a
	 * pointer into it (same rationale as the other pipeline tests). */
	MediaTrackingListener::Added rawVideo = *rawVideoPtr;
	CU_ASSERT_FATAL(rawVideo.videoInfo.resolution.width >= 2);
	CU_ASSERT_FATAL(rawVideo.videoInfo.resolution.height >= 2);

	uint32_t scaledWidth = rawVideo.videoInfo.resolution.width / 2;
	uint32_t scaledHeight = rawVideo.videoInfo.resolution.height / 2;
	/* Must genuinely differ from the source: otherwise this test could
	 * not tell a real scale apart from an accidental passthrough. */
	CU_ASSERT_NOT_EQUAL_FATAL(scaledWidth,
				  rawVideo.videoInfo.resolution.width);

	struct vscale_config scalerParams = {};
	scalerParams.output.info = rawVideo.videoInfo;
	scalerParams.output.info.resolution.width = scaledWidth;
	scalerParams.output.info.resolution.height = scaledHeight;
	IPdraw::IVideoScaler *scaler = nullptr;
	int ret = session->createVideoScaler(rawVideo.id,
					     &scalerParams,
					     &g_stub_video_scaler_listener,
					     &scaler);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaler);
	auto scalerOwner = std::unique_ptr<IPdraw::IVideoScaler>(scaler);

	/* Wait for the scaler's own lazily-created output media (see
	 * VideoScaler::onOutputFrame in pdraw_scaler_video.cpp: mOutputMedia
	 * is only created once the first scaled frame is ready), identified
	 * by carrying the requested (halved) resolution. */
	bool gotScaledMedia = loop.pumpUntil(
		[&]() {
			return mediaListener.findRawVideoMediaWithResolution(
				       scaledWidth, scaledHeight) != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotScaledMedia);
	const MediaTrackingListener::Added *scaledMediaPtr =
		mediaListener.findRawVideoMediaWithResolution(scaledWidth,
							      scaledHeight);
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaledMediaPtr);
	MediaTrackingListener::Added scaledMedia = *scaledMediaPtr;

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(scaledMedia.id,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	/* Any frame past this point is a genuinely scaled one: the scaler has
	 * been running continuously since creation (fed by the decoder), so
	 * by the time the sink connects, playback simply keeps producing
	 * more -- no "sink must exist before the first frame" ordering
	 * concern here (that only applies to ExternalXxxSource elements
	 * fed by explicit pushes, not to an internal pipeline element
	 * continuously fed by an upstream decoder). */
	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(outQueue,
							      &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(frameInfo.info.resolution.width, scaledWidth);
	CU_ASSERT_EQUAL(frameInfo.info.resolution.height, scaledHeight);
	mbuf_raw_video_frame_unref(frame);

	sinkOwner.reset();
	scalerOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


/* ── C API: PdrawVideoScalerListener shim coverage ──────────────────────
 * Covers the sole virtual method of PdrawVideoScalerListener
 * (pdraw_wrapper.cpp): videoScalerFrameOutput → frame_output C callback. No
 * NAS/decoder needed: the VideoScaler is fed by a synthetic
 * ExternalRawVideoSource (same 32×32 I420 frames as createScalerChainAndPrime)
 * and scales them to 16×16. The listener callback fires before any downstream
 * channel push (pdraw_scaler_video.cpp:879),
 * so no downstream sink is required to observe it. */

namespace {

struct ScalerSessionCbState {
	unsigned int lastRawMediaId = 0;
	int stopRespCount = 0;
};

struct ScalerCbState {
	int frameOutputCount = 0;
};

} /* anonymous namespace */

static void
scaler_session_stop_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<ScalerSessionCbState *>(ud)->stopRespCount++;
}

static void scaler_session_media_added_cb(struct pdraw * /*p*/,
					  const struct pdraw_media_info *info,
					  void * /*elem*/,
					  void *ud)
{
	if (info->type == PDRAW_MEDIA_TYPE_VIDEO &&
	    info->video.format == VDEF_FRAME_TYPE_RAW)
		static_cast<ScalerSessionCbState *>(ud)->lastRawMediaId =
			info->id;
}

static void
scaler_session_media_removed_cb(struct pdraw * /*p*/,
				const struct pdraw_media_info * /*info*/,
				void * /*elem*/,
				void * /*ud*/)
{
}

static void scaler_src_flushed_cb(struct pdraw * /*p*/,
				  struct pdraw_raw_video_source * /*s*/,
				  void * /*ud*/)
{
}

static void scaler_src_drained_cb(struct pdraw * /*p*/,
				  struct pdraw_raw_video_source * /*s*/,
				  void * /*ud*/)
{
}

static void scaler_frame_output_cb(struct pdraw * /*p*/,
				   struct pdraw_video_scaler * /*sc*/,
				   struct mbuf_raw_video_frame * /*frame*/,
				   void *ud)
{
	static_cast<ScalerCbState *>(ud)->frameOutputCount++;
}

static void testCVideoScalerListenerFrameOutput()
{
	constexpr uint32_t kWidth = 32;
	constexpr uint32_t kHeight = 32;
	constexpr uint32_t kScaledWidth = 16;
	constexpr uint32_t kScaledHeight = 16;

	TestPompLoop loop;
	ScalerSessionCbState sessState;
	struct pdraw_cbs sessCbs = {};
	sessCbs.stop_resp = scaler_session_stop_cb;
	sessCbs.media_added = scaler_session_media_added_cb;
	sessCbs.media_removed = scaler_session_media_removed_cb;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessCbs, &sessState, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	/* Raw video source: I420 32×32, same format/resolution as
	 * createScalerChainAndPrime's priming frames. */
	struct pdraw_video_source_params srcParams = {};
	srcParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	srcParams.video.format = VDEF_FRAME_TYPE_RAW;
	srcParams.video.raw.format = vdef_i420;
	srcParams.video.raw.info.resolution.width = kWidth;
	srcParams.video.raw.info.resolution.height = kHeight;
	srcParams.video.raw.info.bit_depth = 8;
	srcParams.video.raw.info.framerate.num = 30;
	srcParams.video.raw.info.framerate.den = 1;
	struct pdraw_raw_video_source_cbs srcCbs = {};
	srcCbs.flushed = scaler_src_flushed_cb;
	srcCbs.drained = scaler_src_drained_cb;
	struct pdraw_raw_video_source *src = nullptr;
	ret = pdraw_raw_video_source_new(p, &srcParams, &srcCbs, nullptr, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* Wait for session-level media_added to learn the raw media ID */
	bool gotMedia = loop.pumpUntil(
		[&sessState]() { return sessState.lastRawMediaId != 0; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Create scaler via C API: 32×32 → 16×16, frame_output wired */
	struct vscale_config scalerParams = {};
	scalerParams.output.info.resolution.width = kScaledWidth;
	scalerParams.output.info.resolution.height = kScaledHeight;
	ScalerCbState scalerState;
	struct pdraw_video_scaler_cbs scalerCbs = {};
	scalerCbs.frame_output = scaler_frame_output_cb;
	struct pdraw_video_scaler *scaler = nullptr;
	ret = pdraw_video_scaler_new(p,
				     sessState.lastRawMediaId,
				     &scalerParams,
				     &scalerCbs,
				     &scalerState,
				     &scaler);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaler);

	/* Push 3 priming frames — same count as createScalerChainAndPrime.
	 * frame_output fires in VideoScaler::frameOutputCb
	 * (pdraw_scaler_video.cpp:879) before any downstream channel push, so
	 * no sink is needed. */
	struct mbuf_raw_video_frame_queue *inQ =
		pdraw_raw_video_source_get_queue(p, src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
	for (unsigned int i = 0; i < 3; i++) {
		struct mbuf_raw_video_frame *frame =
			makeI420Frame(kWidth, kHeight, i * 33333, i);
		ret = mbuf_raw_video_frame_queue_push(inQ, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotOutput = loop.pumpUntil(
		[&scalerState]() { return scalerState.frameOutputCount >= 1; },
		15000);
	CU_ASSERT_TRUE(gotOutput);
	CU_ASSERT_TRUE(scalerState.frameOutputCount >= 1);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&sessState]() { return sessState.stopRespCount >= 1; }, 15000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_video_scaler_destroy(p, scaler);
	pdraw_raw_video_source_destroy(p, src);
	pdraw_destroy(p);
}


static void testCxxVideoScalerStartWithoutInputMediaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	auto scaler = std::unique_ptr<VideoScaler>(new VideoScaler(
		session, nullptr, nullptr, nullptr, nullptr, nullptr));
	CU_ASSERT_PTR_NOT_NULL_FATAL(scaler.get());

	/* 1. start() without input media -> fails with -EPROTO */
	int res = scaler->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 2. start() when state is not CREATED (now STOPPED) -> fails with
	 * -EPROTO */
	res = scaler->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 3. stop() when state is STOPPED -> returns 0 */
	res = scaler->stop();
	CU_ASSERT_EQUAL(res, 0);
}


static void testCxxVideoScalerFrameOutputCbCoverage()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	VideoScaler scaler(
		session, nullptr, nullptr, nullptr, nullptr, nullptr);

	/* 1. status != 0 -> error logging */
	VideoScaler::frameOutputCb(nullptr, -EINVAL, nullptr, &scaler);

	/* 2. userdata == nullptr */
	VideoScaler::frameOutputCb(nullptr, 0, nullptr, nullptr);

	/* 3. out_frame == nullptr */
	VideoScaler::frameOutputCb(nullptr, 0, nullptr, &scaler);

	/* 4. state != STARTED */
	struct mbuf_raw_video_frame *frame = nullptr;
	struct vdef_raw_frame frameInfo = {};
	frameInfo.info.resolution.width = 16;
	frameInfo.info.resolution.height = 16;
	frameInfo.format = vdef_i420;
	int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* 4a. state != STARTED */
	CU_ASSERT_NOT_EQUAL(scaler.getState(), Element::State::STARTED);
	VideoScaler::frameOutputCb(nullptr, 0, frame, &scaler);

	/* 4b. state == STARTED with flush pending -> discards frame */
	scaler.setState(Element::State::STARTED);
	scaler.setFlushingState(Element::FlushingState::FLUSHING, true);
	scaler.mVscaleFlushPending = true;
	CU_ASSERT_TRUE(scaler.mVscaleFlushPending);
	VideoScaler::frameOutputCb(nullptr, 0, frame, &scaler);

	/* 4c. state == STARTED with mInputMedia == nullptr -> invalid input
	 * media */
	scaler.mVscaleFlushPending = false;
	CU_ASSERT_PTR_NULL(scaler.mInputMedia);
	VideoScaler::frameOutputCb(nullptr, 0, frame, &scaler);

	/* 4d. state == STARTED with mInputMedia set, but frame has no ancillary
	 * data */
	RawVideoMedia media(session);
	scaler.mInputMedia = &media;
	CU_ASSERT_PTR_NOT_NULL(scaler.mInputMedia);
	VideoScaler::frameOutputCb(nullptr, 0, frame, &scaler);
	scaler.mInputMedia = nullptr;
	scaler.setState(Element::State::STOPPED);
	CU_ASSERT_EQUAL(scaler.getState(), Element::State::STOPPED);

	mbuf_raw_video_frame_unref(frame);

	/* 5. nullptr guards for callbacks and channel events */
	VideoScaler::flushCb(nullptr, nullptr);
	VideoScaler::stopCb(nullptr, nullptr);

	CU_ASSERT_FALSE(scaler.mInputChannelFlushPending);
	scaler.onChannelFlush(nullptr);
	CU_ASSERT_FALSE(scaler.mInputChannelFlushPending);

	scaler.onChannelDrain(nullptr);
	CU_ASSERT_FALSE(scaler.mInputChannelFlushPending);

	scaler.mVscaleFlushPending = true;
	VideoScaler::flushCb(nullptr, &scaler);
	CU_ASSERT_FALSE(scaler.mVscaleFlushPending);
}


CU_TestInfo g_pdraw_test_pipeline_scaler[] = {
	{FN("testCxxCreateVideoScalerWithRealDecodedMedia"),
	 testCxxCreateVideoScalerWithRealDecodedMedia},
	{FN("testCxxVideoScalerProducesRequestedResolution"),
	 testCxxVideoScalerProducesRequestedResolution},
	{FN("testCxxVideoScalerFlushCascadesThroughSourceAndSink"),
	 testCxxVideoScalerFlushCascadesThroughSourceAndSink},
	{FN("testCxxVideoScalerDrainCascadesThroughSourceAndSink"),
	 testCxxVideoScalerDrainCascadesThroughSourceAndSink},
	{FN("testCxxVideoScalerSessionMetaUpdateCascadesThroughSourceAndSink"),
	 testCxxVideoScalerSessionMetaUpdateCascadesThroughSourceAndSink},
	{FN("testCxxVideoScalerTeardownWhenSourceStops"),
	 testCxxVideoScalerTeardownWhenSourceStops},
	{FN("testCxxCreateVideoScalerWithEmptyConfigFails"),
	 testCxxCreateVideoScalerWithEmptyConfigFails},
	{FN("testCxxCreateVideoScalerWithNameSucceeds"),
	 testCxxCreateVideoScalerWithNameSucceeds},
	{FN("testCxxVideoScalerNullConfigDefaultsTo1280x720"),
	 testCxxVideoScalerNullConfigDefaultsTo1280x720},
	{FN("testCVideoScalerListenerFrameOutput"),
	 testCVideoScalerListenerFrameOutput},
	{FN("testCxxVideoScalerStartWithoutInputMediaFails"),
	 testCxxVideoScalerStartWithoutInputMediaFails},
	{FN("testCxxVideoScalerFrameOutputCbCoverage"),
	 testCxxVideoScalerFrameOutputCbCoverage},
	CU_TEST_INFO_NULL,
};
