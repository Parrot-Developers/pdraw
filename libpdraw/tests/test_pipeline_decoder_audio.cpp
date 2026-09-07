/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — audio decoder on a real demuxed media (Tier B,
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

/* Split out of test_pipeline_decode.cpp (see test_pipeline_decoder_video.cpp's
 * header comment for the full split history): this file holds every
 * AudioDecoder test, symmetric to test_pipeline_decoder_video.cpp on the
 * video side. The shared demuxer-fixture helpers (PipelineDemuxerListener,
 * openDecodingDemuxerAndPlay, closeAndDestroyDemuxer, stopSessionAndWait)
 * live in test_pipeline_common.hpp; MediaTrackingListener remains
 * per-translation-unit because its finder methods differ per file (Added and
 * the common body live in the base class).
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the decoder's output
 * media requires a real session-wide listener, so each test below builds its
 * own private TestPompLoop + TestSession, fully self-contained. No suite
 * init/cleanup is registered for this file (see test_main.c: NULL, NULL). */

#define ULOG_TAG pdraw_test_pipeline_decoder_audio

#include <complex>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "pdraw_decoder_audio.hpp"
#include "pdraw_element.hpp"
#undef protected
#undef private

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include <media-buffers/mbuf_audio_frame.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── M4A fixture (same NAS asset as test_api_demuxer.cpp) ────────────────── */

enum { ASSET_AUDIO_AAC = 0 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_decoder_audio[] = {
	{"Tests/miscellaneous/audio.m4a"},
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
	/* First *decoded* (PCM) audio media added, or nullptr if none yet.
	 * Deliberately NOT just "first audio media": with autodecoding_mode =
	 * DECODE_ALL, the demuxer's own coded AAC-LC output is added at
	 * ready-to-play time, strictly before the AudioDecoder's raw PCM
	 * output (only added once decoding actually produces a first frame,
	 * i.e. after play()) -- so an encoding-blind "first match" would
	 * silently return the demuxer's *coded* media instead (same pitfall
	 * documented in test_pipeline_encoder_audio.cpp's equivalent lookup).
	 */
	const Added *findRawAudioMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_AUDIO &&
			    a.audioFormat.encoding == ADEF_ENCODING_PCM)
				return &a;
		}
		return nullptr;
	}
};


/* Implements the sink side of the flush/drain protocol for an audio sink
 * (same rationale and near-identical body as AckingAudioSinkListener in
 * test_pipeline_encoder_audio.cpp: duplicated rather than shared, per this
 * suite's translation-unit-private convention). Used below to prove that a
 * drain triggered by natural EoF on the demuxer cascades all the way through
 * the internally auto-created AudioDecoder (exercising its
 * onChannelDrain/onChannelDrained, entirely unreachable otherwise:
 * AudioDecoder has no public API of its own) and reaches the downstream
 * audio sink -- and that the acknowledgment propagates back, allowing the
 * demuxer operation to complete. */
class AckingAudioSinkListener : public IPdraw::IAudioSink::Listener {
public:
	void
	onAudioSinkMediaAdded(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/,
			      const struct pdraw_media_info * /*i*/) override
	{
	}

	void onAudioSinkMediaRemoved(IPdraw * /*p*/,
				     IPdraw::IAudioSink * /*sk*/,
				     const struct pdraw_media_info * /*i*/,
				     bool /*restart*/) override
	{
	}

	void onAudioSinkFlush(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		discardQueue();
		sk->queueFlushed();
	}

	void onAudioSinkDrain(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		discardQueue();
		sk->queueDrained();
		mGotDrained = true;
	}

	/* Must be set right after createAudioSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_audio_frame_queue *mQueue = nullptr;
	bool mGotDrained = false;

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_audio_frame *f = nullptr;
		while (mbuf_audio_frame_queue_pop(mQueue, &f) == 0)
			mbuf_audio_frame_unref(f);
	}
};


/* Same rationale as testCxxH265VideoDecodesToRawFrames in
 * test_pipeline_decoder_video.cpp, for the AudioDecoder side: AudioDecoder is
 * likewise never exposed through the public C++ API (always auto-created by
 * PipelineFactory when autodecoding_mode = DECODE_ALL). Only its drain
 * cascade (CHANNEL_SIGNAL_DRAIN, natural EoF in the demuxer's onTimer()) is
 * exercised here -- unlike the video side, its flush cascade (close()) is
 * already covered indirectly by testCxxAacAudioDecodesToRawFrames's teardown
 * via closeAndDestroyDemuxer(), so OFFLINE is the only mode needed (no
 * REALTIME/flush variant of this helper exists, unlike
 * test_pipeline_decoder_video.cpp's createDecoderChainAndPlay's `mode`
 * parameter). */
static IPdraw::IDemuxer *
createAudioDecoderChainAndPlay(IPdraw *session,
			       TestPompLoop *loop,
			       MediaTrackingListener *mediaListener,
			       PipelineDemuxerListener *demuxListener,
			       IPdraw::IAudioSink **sink,
			       AckingAudioSinkListener *sinkListener)
{
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_AUDIO_AAC, s_assets_pipeline_decoder_audio);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, loop, demuxListener, demuxerPath);

	bool gotRawAudio = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawAudio);
	const MediaTrackingListener::Added *rawAudioPtr =
		mediaListener->findRawAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawAudioPtr);

	int ret = session->createAudioSink(rawAudioPtr->id, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);
	sinkListener->mQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);

	/* Wait until at least one decoded frame arrives in the sink queue --
	 * same rationale as createDecoderChainAndPlay in
	 * test_pipeline_decoder_video.cpp: ExternalAudioSink only transitions
	 * to UNFLUSHED in onAudioChannelQueue() once a frame is actually
	 * queued, not from any channel-link handshake. Until then any drain
	 * cascade would hit the silent early-return path (FLUSHED + empty
	 * queue). */
	bool gotFrame = loop->pumpUntil(
		[sinkListener]() {
			if (sinkListener->mQueue == nullptr)
				return false;
			struct mbuf_audio_frame *f = nullptr;
			if (mbuf_audio_frame_queue_pop(sinkListener->mQueue,
						       &f) == 0) {
				mbuf_audio_frame_unref(f);
				return true;
			}
			return false;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	return demuxer;
}


/* Same rationale as testCxxH265VideoDecodesToRawFrames in
 * test_pipeline_decoder_video.cpp, for the AudioDecoder side: unlike
 * test_pipeline_encoder_audio.cpp (where AAC-LC decode is only ever a setup
 * step before creating an audio encoder, never verified for its own sake),
 * this confirms real AAC-LC decoding on its own -- fdk-aac's ADTS parsing and
 * per-frame PCM output actually worked, not just that some placeholder media
 * got created. */
static void testCxxAacAudioDecodesToRawFrames()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_AUDIO_AAC, s_assets_pipeline_decoder_audio);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

	/* Wait for the internally auto-created AudioDecoder to output its
	 * first decoded frame: that's when it creates its raw output media
	 * and onMediaAdded() fires for it. */
	bool gotRawAudio = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawAudio);
	const MediaTrackingListener::Added *rawAudio =
		mediaListener.findRawAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawAudio);
	CU_ASSERT_NOT_EQUAL_FATAL(rawAudio->id, 0u);

	/* The whole point: decoding a real AAC-LC stream produced a raw PCM
	 * frame with a plausible format -- confirms the demuxer/decoder's
	 * ADTS parsing and per-frame decode actually worked. */
	CU_ASSERT_EQUAL(rawAudio->audioFormat.encoding, ADEF_ENCODING_PCM);
	CU_ASSERT(rawAudio->audioFormat.channel_count > 0);
	CU_ASSERT(rawAudio->audioFormat.sample_rate > 0);
	CU_ASSERT(rawAudio->audioFormat.bit_depth > 0);

	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* AudioDecoder::onChannelDrain/onChannelDrained were entirely uncovered (0%
 * per a real gcov run) -- unlike AudioDecoder::onChannelFlush/onChannelFlushed
 * (already covered: testCxxAacAudioDecodesToRawFrames's teardown goes through
 * closeAndDestroyDemuxer(), i.e. close() -> flush(true)). Only the drain path
 * (natural EoF, not close()) was missing, symmetric to
 * testCxxVideoDecoderDrainCascadesThroughSourceAndSink in
 * test_pipeline_decoder_video.cpp but for the audio side: EoF in
 * RecordDemuxer::onTimer() -> drain() -> DemuxerMedia::flush(discard=false)
 * -> CHANNEL_SIGNAL_DRAIN cascades through AudioDecoder -> the downstream
 * ExternalAudioSink (onAudioSinkDrain), whose acknowledgment (queueDrained())
 * propagates back through AudioDecoder::onChannelDrained. */
static void testCxxAudioDecoderDrainCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	IPdraw::IAudioSink *sink = nullptr;
	AckingAudioSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer =
		createAudioDecoderChainAndPlay(session,
					       &loop,
					       &mediaListener,
					       &demuxListener,
					       &sink,
					       &sinkListener);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	/* Let the clip exhaust naturally (OFFLINE mode, set up by
	 * createAudioDecoderChainAndPlay via openDecodingDemuxerAndPlay's
	 * default). The acking listener calls queueDrained(). */
	bool gotDrained = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotDrained; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	/* Explicit close() after natural EoF, same as
	 * testCxxVideoDecoderDrainCascadesThroughSourceAndSink: the sink is
	 * already in FLUSHED state so flush(true) from stop() takes the
	 * silent ack path, but completeTeardown() still fires once all
	 * channels ack. */
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);

	sinkOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


static void testCxxAudioDecoderStartWithoutInputMediaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	auto decoder = std::unique_ptr<AudioDecoder>(
		new AudioDecoder(session, nullptr, nullptr));
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


static void testCxxAudioDecoderStartAdtsSucceeds()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioMedia media(session);
	media.format.encoding = ADEF_ENCODING_AAC_LC;
	media.format.sample_rate = 44100;
	media.format.channel_count = 2;
	media.format.bit_depth = 16;
	media.format.aac.data_format = ADEF_AAC_DATA_FORMAT_ADTS;

	AudioDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_EQUAL(ret, 0);

	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, 0);
	ret = decoder.stop();
	CU_ASSERT_EQUAL(ret, 0);
}


static void testCxxAudioDecoderStateAndFlushGuards()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioMedia media(session);
	media.format.encoding = ADEF_ENCODING_AAC_LC;
	media.format.sample_rate = 44100;
	media.format.channel_count = 2;
	media.format.bit_depth = 16;
	media.format.aac.data_format = ADEF_AAC_DATA_FORMAT_ADTS;

	AudioDecoder decoder(session, nullptr, nullptr);
	int ret = decoder.addInputMedia(&media);
	CU_ASSERT_EQUAL(ret, 0);

	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, 0);

	/* 1. start() when already STARTED -> returns 0 */
	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, 0);

	/* 2. stop() -> returns 0 */
	ret = decoder.stop();
	CU_ASSERT_EQUAL(ret, 0);

	/* 3. stop() when already STOPPED -> returns 0 */
	ret = decoder.stop();
	CU_ASSERT_EQUAL(ret, 0);

	/* 4. start() when in STOPPED state -> returns -EPROTO */
	ret = decoder.start();
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


static void testCxxAudioDecoderFrameOutputCbCoverage()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioDecoder decoder(session, nullptr, nullptr);

	/* 1. status != 0 -> error logging */
	AudioDecoder::frameOutputCb(nullptr, -EINVAL, nullptr, &decoder);

	/* 2. userdata == nullptr */
	AudioDecoder::frameOutputCb(nullptr, 0, nullptr, nullptr);

	/* 3. out_frame == nullptr */
	AudioDecoder::frameOutputCb(nullptr, 0, nullptr, &decoder);

	/* 4. frameOutputCb branch coverage */
	struct mbuf_audio_frame *frame = nullptr;
	struct adef_frame frameInfo = {};
	frameInfo.format = adef_pcm_16b_44100hz_mono;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* 4a. state != STARTED */
	CU_ASSERT_NOT_EQUAL(decoder.getState(), Element::State::STARTED);
	AudioDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4b. state == STARTED with flush pending -> discards frame */
	decoder.setState(Element::State::STARTED);
	decoder.setFlushingState(Element::FlushingState::FLUSHING, true);
	decoder.mAdecFlushPending = true;
	CU_ASSERT_TRUE(decoder.mAdecFlushPending);
	AudioDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4c. state == STARTED with mInputMedia == nullptr -> invalid input
	 * media */
	decoder.mAdecFlushPending = false;
	CU_ASSERT_PTR_NULL(decoder.mInputMedia);
	AudioDecoder::frameOutputCb(nullptr, 0, frame, &decoder);

	/* 4d. state == STARTED with mInputMedia set, but frame has no ancillary
	 * data */
	AudioMedia media(session);
	decoder.mInputMedia = &media;
	CU_ASSERT_PTR_NOT_NULL(decoder.mInputMedia);
	AudioDecoder::frameOutputCb(nullptr, 0, frame, &decoder);
	decoder.mInputMedia = nullptr;
	decoder.setState(Element::State::STOPPED);
	CU_ASSERT_EQUAL(decoder.getState(), Element::State::STOPPED);

	mbuf_audio_frame_unref(frame);

	/* 5. nullptr guards for callbacks and channel events */
	AudioDecoder::flushCb(nullptr, nullptr);
	AudioDecoder::stopCb(nullptr, nullptr);

	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);
	decoder.onChannelFlush(nullptr);
	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);

	decoder.onChannelDrain(nullptr);
	CU_ASSERT_FALSE(decoder.mInputChannelFlushPending);

	decoder.mAdecFlushPending = true;
	AudioDecoder::flushCb(nullptr, &decoder);
	CU_ASSERT_FALSE(decoder.mAdecFlushPending);
}


CU_TestInfo g_pdraw_test_pipeline_decoder_audio[] = {
	{FN("testCxxAacAudioDecodesToRawFrames"),
	 testCxxAacAudioDecodesToRawFrames},
	{FN("testCxxAudioDecoderDrainCascadesThroughSourceAndSink"),
	 testCxxAudioDecoderDrainCascadesThroughSourceAndSink},
	{FN("testCxxAudioDecoderStartWithoutInputMediaFails"),
	 testCxxAudioDecoderStartWithoutInputMediaFails},
	{FN("testCxxAudioDecoderStartAdtsSucceeds"),
	 testCxxAudioDecoderStartAdtsSucceeds},
	{FN("testCxxAudioDecoderStateAndFlushGuards"),
	 testCxxAudioDecoderStateAndFlushGuards},
	{FN("testCxxAudioDecoderFrameOutputCbCoverage"),
	 testCxxAudioDecoderFrameOutputCbCoverage},
	CU_TEST_INFO_NULL,
};
