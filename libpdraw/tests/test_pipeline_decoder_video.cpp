/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — video decoder on a real demuxed media (Tier B,
 * self-contained fixture)
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

/* This file used to also hold the scaler / video encoder / audio encoder
 * pipeline tests; it grew past 2000 lines mixing four largely orthogonal
 * concerns, so it was split into test_pipeline_decode.cpp (decode only),
 * test_pipeline_scaler_video.cpp, test_pipeline_encoder_video.cpp and
 * test_pipeline_encoder_audio.cpp. test_pipeline_decode.cpp itself still
 * mixed the video-side and audio-side VideoDecoder/AudioDecoder tests, so it
 * was further split into this file (video only) and its sibling
 * test_pipeline_decoder_audio.cpp (audio only). The shared demuxer-fixture
 * helpers (PipelineDemuxerListener, openDecodingDemuxerAndPlay,
 * closeAndDestroyDemuxer, stopSessionAndWait) live in
 * test_pipeline_common.hpp; MediaTrackingListener remains
 * per-translation-unit because its finder methods differ per file (Added and
 * the common body live in the base class).
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the decoder's output
 * media requires a real session-wide listener, so each test below builds its
 * own private TestPompLoop + TestSession, fully self-contained. No suite
 * init/cleanup is registered for this file (see test_main.c: NULL, NULL). */

#define ULOG_TAG pdraw_test_pipeline_decoder_video

#include <complex>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "pdraw_decoder_video.hpp"
#include "pdraw_element.hpp"
#undef protected
#undef private

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include <media-buffers/mbuf_raw_video_frame.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── MP4 fixtures (same NAS assets as test_api_demuxer.cpp) ──────────────── */

enum { ASSET_VIDEO_H265 = 0, ASSET_VIDEO_H264 = 1 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_decoder_video[] = {
	{"Tests/anafi/4k/video_recording/champs_240p30_h265.mp4"},
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
};


/* Implements the sink side of the flush/drain protocol for a raw video sink
 * (same rationale and near-identical body as AckingRawVideoSinkListener in
 * test_pipeline_scaler_video.cpp: duplicated rather than shared, per this
 * suite's translation-unit-private convention). Used below to prove that a
 * flush()/drain() triggered on the demuxer cascades all the way through the
 * internally auto-created VideoDecoder (exercising its
 * onChannelFlush/onChannelDrain/onChannelFlushed/onChannelDrained, entirely
 * unreachable otherwise: VideoDecoder has no public API of its own) and
 * reaches the downstream raw video sink -- and that the acknowledgment
 * propagates back, allowing the demuxer operation to complete. */
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
		mGotFlushed = true;
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		discardQueue();
		sk->queueDrained();
		mGotDrained = true;
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	struct mbuf_raw_video_frame_queue *mQueue = nullptr;
	bool mGotFlushed = false;
	bool mGotDrained = false;

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


/* Open the H264 fixture asset with full auto-decoding, play, attach an
 * ExternalRawVideoSink, and wait until it has received at least one frame
 * (which transitions it to UNFLUSHED state).  Returns only once that has
 * happened so callers can immediately trigger close()/EoF and reliably
 * observe the resulting flush/drain cascade.
 *
 * VideoDecoder is never exposed through the public C++ API (no
 * createVideoDecoder() exists: it is always auto-created by PipelineFactory
 * when autodecoding_mode = DECODE_ALL). Flush/drain cascades can only be
 * triggered indirectly:
 *   flush (CHANNEL_SIGNAL_FLUSH): demuxer close() → stop() → flush(true)
 *   drain (CHANNEL_SIGNAL_DRAIN): EoF reached in onTimer() → drain()
 * Both are observed from the downstream ExternalRawVideoSink's listener
 * (onRawVideoSinkFlush / onRawVideoSinkDrain).
 *
 * ExternalRawVideoSink transitions to UNFLUSHED ONLY in
 * onRawVideoChannelQueue() when a frame is queued — not from a channel-link
 * handshake.  Until a frame arrives, any flush cascade silently acks
 * (FLUSHED + empty queue path in flush()) without calling the listener.
 *
 * playback_mode: REALTIME keeps the clip alive for an explicit close();
 * OFFLINE lets the clip exhaust quickly so EoF fires for the drain test. */
static IPdraw::IDemuxer *createDecoderChainAndPlay(
	IPdraw *session,
	TestPompLoop *loop,
	MediaTrackingListener *mediaListener,
	PipelineDemuxerListener *demuxListener,
	IPdraw::IRawVideoSink **sink,
	AckingRawVideoSinkListener *sinkListener,
	enum pdraw_playback_mode mode = PDRAW_PLAYBACK_MODE_REALTIME)
{
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_decoder_video);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, loop, demuxListener, demuxerPath, mode);

	bool gotRawVideo = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener->findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);

	struct pdraw_video_sink_params sinkParams = {};
	int ret = session->createRawVideoSink(
		rawVideoPtr->id, &sinkParams, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);
	sinkListener->mQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);

	/* Wait until at least one decoded frame arrives in the sink queue.
	 * ExternalRawVideoSink transitions to UNFLUSHED ONLY in
	 * onRawVideoChannelQueue() → setFlushingState(UNFLUSHED), i.e. when
	 * the first frame is actually queued — NOT from any channel-link
	 * handshake.  Until a frame arrives the sink stays in FLUSHED state;
	 * any flush/drain cascade then hits the silent early-return path
	 * (flush(): FLUSHED + empty queue → idleAdd(mFlushDoneHandler),
	 * listener never called).
	 * Pop and discard the frame: the internal UNFLUSHED state persists
	 * until flushDone() / drainDone() is called, which only happens from
	 * queueFlushed() / queueDrained() — i.e. only after
	 * onRawVideoSinkFlush / onRawVideoSinkDrain fires. */
	bool gotFrame = loop->pumpUntil(
		[sinkListener]() {
			if (sinkListener->mQueue == nullptr)
				return false;
			struct mbuf_raw_video_frame *f = nullptr;
			if (mbuf_raw_video_frame_queue_pop(sinkListener->mQueue,
							   &f) == 0) {
				mbuf_raw_video_frame_unref(f);
				return true;
			}
			return false;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	return demuxer;
}


/* Every other demuxer-backed test in this whole suite (this file,
 * test_api_demuxer.cpp, test_pipeline_muxer_record.cpp,
 * test_pipeline_scaler_video.cpp, test_pipeline_encoder_video.cpp) uses the
 * H.264 asset (champs_240p30_h264.mp4): RecordDemuxer::DemuxerCodedVideoMedia's
 * H.265/HEVC branch (VPS/SPS/PPS parsing via h265_reader, HEVC NALU type
 * extraction in processSample() -- pdraw_demuxer_record_coded_video_media.cpp,
 * mirroring the H.264 branch line-for-line) would otherwise never be exercised.
 * This drives the real decode pipeline just far enough to confirm a real HEVC
 * stream decodes into a genuine raw frame -- no scaler/encoder chaining
 * needed for that (those live in the other pipeline_* files, all of which
 * re-encode/re-scale from the H.264 asset instead, since the encoding under
 * test there is independent of the *decoded* source's own codec). */
static void testCxxH265VideoDecodesToRawFrames()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H265, s_assets_pipeline_decoder_video);
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

	/* The whole point: decoding a real H.265 stream produced a raw frame
	 * with a plausible resolution -- confirms the demuxer's HEVC VPS/SPS/
	 * PPS parsing and per-frame NALU handling actually worked, not just
	 * that some placeholder media got created. */
	CU_ASSERT(rawVideo->videoInfo.resolution.width > 0);
	CU_ASSERT(rawVideo->videoInfo.resolution.height > 0);

	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Symmetric to testCxxH265VideoDecodesToRawFrames: verifies that the H.264
 * decode path (ffmpeg/libav H.264 decoder backend, AnnexB/bytestream input)
 * works end-to-end. Although H.264 decode is implicitly exercised by the
 * encoder and scaler pipeline tests (which all use the H.264 asset), those
 * tests have encoding or scaling as their primary concern. Having an explicit
 * decode-only test here makes the coverage intent clear and provides a
 * dedicated failure point if the H.264 decoder backend is unavailable or
 * broken -- independently of whether encoding is also broken. */
static void testCxxH264VideoDecodesToRawFrames()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_VIDEO_H264, s_assets_pipeline_decoder_video);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

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

	CU_ASSERT(rawVideo->videoInfo.resolution.width > 0);
	CU_ASSERT(rawVideo->videoInfo.resolution.height > 0);

	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoDecoder::onChannelFlush/onChannelFlushed are triggered when
 * RecordDemuxer::stop() calls flush(true), sending CHANNEL_SIGNAL_FLUSH on
 * its coded video output channels (pdraw_demuxer_record.cpp: stop() →
 * flush() → DemuxerMedia::flush(discard=true) → channel->flush() →
 * CHANNEL_SIGNAL_FLUSH). The flush cascades through VideoDecoder and reaches
 * the downstream ExternalRawVideoSink (onRawVideoSinkFlush), whose
 * acknowledgment (queueFlushed()) propagates back through
 * VideoDecoder::onChannelFlushed. Observing mGotFlushed (downstream
 * evidence) and mGotCloseResponse (upstream completion, only possible once
 * the full bidirectional cascade succeeded and completeTeardown() fired)
 * proves the decoder's flush handlers actually ran. */
static void testCxxVideoDecoderFlushCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	/* REALTIME: keep the demuxer alive when close() is called. */
	IPdraw::IDemuxer *demuxer =
		createDecoderChainAndPlay(session,
					  &loop,
					  &mediaListener,
					  &demuxListener,
					  &sink,
					  &sinkListener,
					  PDRAW_PLAYBACK_MODE_REALTIME);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	/* close() → stop() → flush(true) → CHANNEL_SIGNAL_FLUSH cascades
	 * through VideoDecoder → ExternalRawVideoSink. Only reaches the sink
	 * if the sink is already in UNFLUSHED state (createDecoderChainAndPlay
	 * ensures this). */
	int ret = demuxer->close();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* mGotFlushed: proof that CHANNEL_SIGNAL_FLUSH reached the raw video
	 * sink, i.e. VideoDecoder::onChannelFlush ran and forwarded the event
	 * downstream. The acking listener already called queueFlushed(). */
	bool gotFlushed = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotFlushed; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	/* mGotCloseResponse: proof that the full bidirectional flush cascade
	 * completed -- closeResponse only fires once queueFlushed() propagated
	 * back through VideoDecoder::onChannelFlushed to the demuxer and
	 * completeTeardown() ran. */
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; },
		15000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);

	sinkOwner.reset();
	demuxerOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* VideoDecoder::onChannelDrain/onChannelDrained are triggered when the
 * demuxer's onTimer() reaches end-of-file and calls drain()
 * (pdraw_demuxer_record.cpp: EoF → drain() → DemuxerMedia::flush(
 * discard=false) → CHANNEL_SIGNAL_DRAIN on coded channels).  The drain
 * cascades through VideoDecoder and reaches the downstream
 * ExternalRawVideoSink (onRawVideoSinkDrain), whose acknowledgment
 * (queueDrained()) propagates back through VideoDecoder::onChannelDrained.
 * Note: close() sends CHANNEL_SIGNAL_FLUSH (not DRAIN); CHANNEL_SIGNAL_DRAIN
 * is only produced by natural EoF in onTimer(). OFFLINE mode is used so
 * the clip exhausts quickly without explicit close(). */
static void testCxxVideoDecoderDrainCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	AckingRawVideoSinkListener sinkListener;
	/* OFFLINE: let the clip exhaust quickly so EoF fires naturally and
	 * triggers the drain cascade without any explicit close().  REALTIME
	 * would pace frames at ~33ms each, making EoF take minutes. */
	IPdraw::IDemuxer *demuxer =
		createDecoderChainAndPlay(session,
					  &loop,
					  &mediaListener,
					  &demuxListener,
					  &sink,
					  &sinkListener,
					  PDRAW_PLAYBACK_MODE_OFFLINE);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	/* Let the clip exhaust naturally.  EoF in RecordDemuxer::onTimer()
	 * → drain() → DemuxerMedia::flush(discard=false) →
	 * CHANNEL_SIGNAL_DRAIN cascades through VideoDecoder →
	 * ExternalRawVideoSink.  The acking listener calls queueDrained(). */
	bool gotDrained = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotDrained; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	/* Explicit close() after natural EoF.  The sink is already in FLUSHED
	 * state so flush(true) from stop() takes the silent ack path (queue
	 * empty → idleAdd(mFlushDoneHandler), no listener call), but
	 * completeTeardown() still fires once all channels ack. */
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);

	sinkOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


static void testCxxVideoDecoderStartWithoutInputMediaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	auto decoder = std::unique_ptr<VideoDecoder>(
		new VideoDecoder(session, nullptr, nullptr));
	CU_ASSERT_PTR_NOT_NULL_FATAL(decoder.get());

	/* 1. start() without input media -> fails with -EPROTO */
	int res = decoder->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 2. start() when state is not CREATED (now STOPPED) -> fails with
	 * -EPROTO */
	res = decoder->start();
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* 3. stop() when state is STOPPED -> returns 0 */
	res = decoder->stop();
	CU_ASSERT_EQUAL(res, 0);
}


static void testCxxVideoDecoderStartWithoutPsFails()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	CodedVideoMedia media(session);
	media.format.encoding = VDEF_ENCODING_H264;
	media.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	VideoDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_EQUAL(ret, 0);

	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCxxVideoDecoderStateAndFlushGuards()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	CodedVideoMedia media(session);
	media.format.encoding = VDEF_ENCODING_H264;
	media.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	VideoDecoder decoder(session, nullptr, nullptr);

	/* 1. stop() when in CREATED state -> sets state to STOPPED and returns
	 * 0 */
	int ret = decoder.stop();
	CU_ASSERT_EQUAL(ret, 0);

	/* 2. start() when in STOPPED state -> returns -EPROTO */
	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* 3. stop() when already STOPPED -> returns 0 */
	ret = decoder.stop();
	CU_ASSERT_EQUAL(ret, 0);

	/* 4. resync() on flushed decoder */
	decoder.resync();
}


static void testCxxVideoDecoderFrameOutputCbCoverage()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	VideoDecoder decoder(session, nullptr, nullptr);

	/* 1. status < 0 -> triggers resync() */
	CU_ASSERT_FALSE(decoder.mResyncPending);
	VideoDecoder::frameOutputCb(nullptr, -EINVAL, nullptr, &decoder);

	/* 2. userdata == nullptr */
	VideoDecoder::frameOutputCb(nullptr, 0, nullptr, nullptr);

	/* 3. out_frame == nullptr */
	VideoDecoder::frameOutputCb(nullptr, 0, nullptr, &decoder);

	/* 4. frameOutputCb branch coverage */
	struct mbuf_raw_video_frame *frame = nullptr;
	struct vdef_raw_frame frameInfo = {};
	frameInfo.info.resolution.width = 16;
	frameInfo.info.resolution.height = 16;
	frameInfo.format = vdef_i420;
	int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* 4a. state != STARTED */
	CU_ASSERT_NOT_EQUAL(decoder.getState(), Element::State::STARTED);
	VideoDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4b. state == STARTED with flush pending -> discards frame */
	decoder.setState(Element::State::STARTED);
	decoder.setFlushingState(Element::FlushingState::FLUSHING, true);
	decoder.mVdecFlushPending = true;
	CU_ASSERT_TRUE(decoder.mVdecFlushPending);
	VideoDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4c. state == STARTED with mInputMedia == nullptr -> invalid input
	 * media */
	decoder.mVdecFlushPending = false;
	CU_ASSERT_PTR_NULL(decoder.mInputMedia);
	VideoDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4d. state == STARTED with mInputMedia set, but frame has no ancillary
	 * data */
	CodedVideoMedia media(session);
	decoder.mInputMedia = &media;
	CU_ASSERT_PTR_NOT_NULL(decoder.mInputMedia);
	VideoDecoder::frameOutputCb(nullptr, 0, frame, &decoder);
	decoder.mInputMedia = nullptr;
	decoder.setState(Element::State::STOPPED);
	CU_ASSERT_EQUAL(decoder.getState(), Element::State::STOPPED);

	mbuf_raw_video_frame_unref(frame);

	/* 5. nullptr guards for callbacks and channel events */
	VideoDecoder::flushCb(nullptr, nullptr);
	VideoDecoder::stopCb(nullptr, nullptr);

	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);
	decoder.onChannelFlush(nullptr);
	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);

	decoder.onChannelDrain(nullptr);
	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);

	decoder.mVdecFlushPending = true;
	VideoDecoder::flushCb(nullptr, &decoder);
	CU_ASSERT_FALSE(decoder.mVdecFlushPending);
}


/* Requires CONFIG_VDEC_TURBOJPEG=y (see products/groundsdk/linux/config/
 * libvideo-decode.config): without it, VideoDecoder never advertises
 * JPEG/JFIF as a supported input format and addInputMedia() returns
 * -ENOSYS, asserted via CU_ASSERT_JPEG_DECODER_INPUT_GUARD (test_api_common.
 * hpp), before start() is ever reached. */
static void testCxxVideoDecoderStartJpeg()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	/* Create a JPEG media with valid format info for vdec_set_jpeg_params
	 */
	CodedVideoMedia media(session);
	media.format = vdef_jpeg_jfif;
	media.info.resolution.width = 640;
	media.info.resolution.height = 480;
	media.info.framerate.num = 30;
	media.info.framerate.den = 1;
	media.info.bit_depth = 8;
	media.info.full_range = true;
	media.info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	media.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	media.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;

	VideoDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_JPEG_DECODER_INPUT_GUARD(ret);
	if (ret != 0)
		return;

	/* start() should follow the VDEF_ENCODING_JPEG branch and call
	 * vdec_set_jpeg_params. */
	ret = decoder.start();
	CU_ASSERT_TRUE(ret == 0 || ret == -EPROTO);
}


static void testCxxVideoDecoderResyncAndCompleteResync()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	CodedVideoMedia media(session);
	media.format = vdef_jpeg_jfif;
	media.info.resolution.width = 640;
	media.info.resolution.height = 480;
	media.info.framerate.num = 30;
	media.info.framerate.den = 1;
	media.info.bit_depth = 8;
	media.info.full_range = true;
	media.info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	media.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	media.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;

	VideoDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_JPEG_DECODER_INPUT_GUARD(ret);
	if (ret != 0)
		return;

	ret = decoder.start();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* 1. completeResync when mResyncPending is false -> early return */
	CU_ASSERT_FALSE(decoder.mResyncPending);
	decoder.completeResync();
	CU_ASSERT_FALSE(decoder.mResyncPending);

	/* 2. resync() when not FLUSHED -> calls vdec_flush, sets mResyncPending
	 * = true */
	decoder.setFlushingState(Element::FlushingState::UNFLUSHED, false);
	decoder.resync();
	CU_ASSERT_TRUE(decoder.mResyncPending);
	CU_ASSERT_TRUE(decoder.mVdecFlushPending);

	/* 3. Second resync() while mResyncPending is true -> early return */
	decoder.resync();
	CU_ASSERT_TRUE(decoder.mResyncPending);

	/* 4. completeResync with valid inputChannel -> calls
	 * inputChannel->resync(), resets mResyncPending */
	decoder.completeResync();
	CU_ASSERT_FALSE(decoder.mResyncPending);

	/* 5. completeResync when mResyncPending is true but mInputMedia is
	 * nullptr -> inputChannel == nullptr branch */
	decoder.mResyncPending = true;
	decoder.mInputMedia = nullptr;
	decoder.completeResync();
	CU_ASSERT_FALSE(decoder.mResyncPending);
}


static void testCxxVideoDecoderStartUnsupportedEncoding()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	CodedVideoMedia media(session);
	media.format.encoding = VDEF_ENCODING_H264;
	media.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	VideoDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	media.format.encoding = static_cast<enum vdef_encoding>(999);
	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


CU_TestInfo g_pdraw_test_pipeline_decoder_video[] = {
	{FN("testCxxH265VideoDecodesToRawFrames"),
	 testCxxH265VideoDecodesToRawFrames},
	{FN("testCxxH264VideoDecodesToRawFrames"),
	 testCxxH264VideoDecodesToRawFrames},
	{FN("testCxxVideoDecoderFlushCascadesThroughSourceAndSink"),
	 testCxxVideoDecoderFlushCascadesThroughSourceAndSink},
	{FN("testCxxVideoDecoderDrainCascadesThroughSourceAndSink"),
	 testCxxVideoDecoderDrainCascadesThroughSourceAndSink},
	{FN("testCxxVideoDecoderStartWithoutInputMediaFails"),
	 testCxxVideoDecoderStartWithoutInputMediaFails},
	{FN("testCxxVideoDecoderStartWithoutPsFails"),
	 testCxxVideoDecoderStartWithoutPsFails},
	{FN("testCxxVideoDecoderStateAndFlushGuards"),
	 testCxxVideoDecoderStateAndFlushGuards},
	{FN("testCxxVideoDecoderFrameOutputCbCoverage"),
	 testCxxVideoDecoderFrameOutputCbCoverage},
	{FN("testCxxVideoDecoderStartJpeg"), testCxxVideoDecoderStartJpeg},
	{FN("testCxxVideoDecoderResyncAndCompleteResync"),
	 testCxxVideoDecoderResyncAndCompleteResync},
	{FN("testCxxVideoDecoderStartUnsupportedEncoding"),
	 testCxxVideoDecoderStartUnsupportedEncoding},
	CU_TEST_INFO_NULL,
};
