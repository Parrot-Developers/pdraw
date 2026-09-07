/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — ALSA audio renderer against a real ExternalAudioSource
 * pipeline (Tier B, self-contained fixture)
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

/* Requires PDRAW_USE_ALSA.  Uses the ALSA "null" virtual device (part of
 * alsa-lib on any standard Linux host) so no audio hardware is needed.
 * AlsaAudioRenderer's constructor does not open the ALSA device -- that is
 * deferred to addInputMedia() -> startAlsa(); the tests below fail cleanly
 * with CU_ASSERT_TRUE_FATAL if the null device is unavailable rather than
 * hanging indefinitely.
 *
 * test_api_renderer_audio.cpp already covers createAudioRenderer()/
 * setParams()/getParams() null-arg guards and the not-compiled-in path
 * (-EPROTO without PDRAW_USE_ALSA).  This file closes the remaining gap:
 * exercising the renderer in a real pipeline with PCM frames flowing
 * through it and verifying drain completion. */

#ifdef PDRAW_USE_ALSA
#	define PDRAW_TEST_RENDERER_AUDIO_ALSA_ENABLED 1
#endif

#define ULOG_TAG pdraw_test_pipeline_renderer_audio
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#include <audio-defs/adefs.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

#include <cstring>
#include <vector>

#ifdef PDRAW_TEST_RENDERER_AUDIO_ALSA_ENABLED
#	include "pdraw_channel.hpp"
#	include "pdraw_media.hpp"
#	include "pdraw_renderer_audio.hpp"
/* pdraw_renderer_audio_alsa.hpp only includes pdraw_session.hpp, which
 * test_fixtures.hpp has already pulled in (its include guard is set).
 * The macro therefore only affects AlsaAudioRenderer's own class body,
 * letting tests access mHandle / mAlsaReady without touching any other
 * class or standard-library header. */
#	define private public
#	define protected public
#	include "pdraw_renderer_audio_alsa.hpp"
#	undef protected
#	undef private
#endif

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

#ifdef PDRAW_TEST_RENDERER_AUDIO_ALSA_ENABLED

/* ALSA period size used by AlsaAudioRenderer (ALSA_AUDIO_DEFAULT_SAMPLE_COUNT
 * from pdraw_alsa_audio.hpp, duplicated here to avoid including an internal
 * header from a test file).  render() checks that each incoming frame has
 * exactly this many samples, so makePcmFrame() below must match. */
static constexpr size_t kAlsaSampleCount = 1024;

static constexpr const char *kAlsaNullDevice = "null";


/* Anonymous namespace: internal linkage so these classes do not collide with
 * identically-named classes in sibling test_*.cpp files (same ODR-safety
 * pattern as every other test_pipeline_*.cpp file). */
namespace {

/* Session-wide listener tracking every audio media addition. */
class MediaTrackingListener : public IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		struct adef_format format;
	};

	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*u*/) override
	{
		if (info->type != PDRAW_MEDIA_TYPE_AUDIO)
			return;
		Added a = {};
		a.id = info->id;
		a.format = info->audio.format;
		mAdded.push_back(a);
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	const Added *findAudioMedia() const
	{
		if (!mAdded.empty())
			return &mAdded.front();
		return nullptr;
	}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Tracks IAudioRenderer lifecycle events. */
class AudioRendererTrackingListener : public IPdraw::IAudioRenderer::Listener {
public:
	void onAudioRendererMediaAdded(
		IPdraw * /*p*/,
		IPdraw::IAudioRenderer * /*r*/,
		const struct pdraw_media_info * /*info*/) override
	{
		mGotMediaAdded = true;
	}

	void onAudioRendererMediaRemoved(
		IPdraw * /*p*/,
		IPdraw::IAudioRenderer * /*r*/,
		const struct pdraw_media_info * /*info*/) override
	{
		mGotMediaRemoved = true;
	}

	bool mGotMediaAdded = false;
	bool mGotMediaRemoved = false;
};


/* Tracks IAudioSource flush/drain completion callbacks. */
class AckingAudioSourceListener : public IPdraw::IAudioSource::Listener {
public:
	void onAudioSourceFlushed(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*s*/) override
	{
		mGotFlushed = true;
	}

	void onAudioSourceDrained(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*s*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};

} /* anonymous namespace */


/* Creates a silence PCM frame matching the ALSA renderer's required buffer
 * size: kAlsaSampleCount samples * 2 channels * 2 bytes (16-bit) = 4096 B.
 * Content is irrelevant; AlsaAudioRenderer passes it directly to
 * snd_pcm_writei() (which discards it on the "null" device). */
static struct mbuf_audio_frame *makePcmFrame(uint64_t timestamp,
					     unsigned int index)
{
	static constexpr size_t kChannels = 2;
	static constexpr size_t kBytesPerSample = 2;
	static constexpr size_t kBufSize =
		kAlsaSampleCount * kChannels * kBytesPerSample;

	struct adef_frame frameInfo = {};
	frameInfo.format = adef_pcm_16b_44100hz_stereo;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(kBufSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	void *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);

	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, kBufSize);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	return frame;
}


/* Shared setup: creates an ExternalAudioSource (PCM stereo 44100 Hz),
 * waits for the session to advertise the resulting audio media, then creates
 * an AudioRenderer on the ALSA null device and waits for
 * onAudioRendererMediaAdded.  Callers own *outSource and *outRenderer and
 * are responsible for deleting them in the right order. */
static void
setupRendererWithSource(TestPompLoop &loop,
			IPdraw *session,
			MediaTrackingListener &mediaListener,
			AudioRendererTrackingListener &rendererListener,
			AckingAudioSourceListener &sourceListener,
			IPdraw::IAudioSource **outSource,
			IPdraw::IAudioRenderer **outRenderer)
{
	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	unsigned int mediaId = mediaListener.findAudioMedia()->id;

	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = kAlsaNullDevice;
	ret = session->createAudioRenderer(
		mediaId, &rendererParams, &rendererListener, outRenderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outRenderer);

	/* onAudioRendererMediaAdded fires once addInputMedia() -> startAlsa()
	 * succeeds.  Fails here if the ALSA null device is not available. */
	bool gotRendererMedia = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaAdded;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRendererMedia);
}


/* Basic lifecycle: createAudioSource + createAudioRenderer -> verify
 * onAudioRendererMediaAdded fires and getMediaId() returns a non-zero ID. */
static void testCxxAudioRendererMediaAdded()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	/* Stop the session (and wait for it) BEFORE resetting the owners
	 * below, so that Session::asyncElementDelete() destroys the
	 * AudioRenderer element (and thus runs AudioRendererWrapper::
	 * clearElement()) while rendererOwner is still alive -- resetting
	 * first would run ~ElementWrapper() instead and the override would
	 * never execute. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	CU_ASSERT_PTR_NULL(static_cast<Pdraw::AudioRendererWrapper *>(renderer)
				   ->getAudioRenderer());

	rendererOwner.reset();
	sourceOwner.reset();
}


/* setParams()/getParams() roundtrip.  The address field is "only honored at
 * creation time": setParams() with the same address succeeds; a different
 * address returns -EPROTO (confirmed by reading AlsaAudioRenderer::
 * setParams()). */
static void testCxxAudioRendererSetParamsRoundtrip()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	struct pdraw_audio_renderer_params gotParams = {};
	int ret = renderer->getParams(&gotParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(gotParams.address);
	if (gotParams.address != nullptr)
		CU_ASSERT_STRING_EQUAL(gotParams.address, kAlsaNullDevice);

	/* Same address: must succeed */
	struct pdraw_audio_renderer_params sameParams = {};
	sameParams.address = kAlsaNullDevice;
	ret = renderer->setParams(&sameParams);
	CU_ASSERT_EQUAL(ret, 0);

	/* Different address: must fail (creation-time-only field) */
	struct pdraw_audio_renderer_params diffParams = {};
	diffParams.address = "default";
	ret = renderer->setParams(&diffParams);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Destroying the source removes its media from the renderer:
 * onAudioRendererMediaRemoved must fire and getMediaId() must return 0.
 * A PCM frame is pushed first to confirm frames flow through the renderer;
 * AlsaAudioRenderer::removeInputMedia() flushes the internal queue when the
 * source is torn down, so the pending frame is discarded without error. */
static void testCxxAudioRendererMediaRemovedWhenSourceDestroyed()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Push one PCM frame to show that frames flow to the ALSA renderer.
	 * Deleting the source calls removeInputMedia() which flushes the
	 * queue, so the frame is discarded cleanly even if not yet rendered. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	sourceOwner.reset();
	source = nullptr;

	bool gotMediaRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMediaRemoved);
	CU_ASSERT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* End-to-end drain: push one PCM frame, call source->drain(), and verify
 * that onAudioSourceDrained() fires.
 *
 * ExternalAudioSource::drain() calls process() synchronously before sending
 * CHANNEL_SIGNAL_DRAIN, so the frame lands in the renderer's internal queue
 * before onChannelDrain() is called.  onChannelDrain() therefore finds a
 * non-empty queue and defers the ack.  render() (triggered by the renderCb
 * pomp event) then consumes the frame; since the queue is now empty and the
 * flushing state is still FLUSHING (fixed in AlsaAudioRenderer::render()),
 * idleDrain() is scheduled, which calls asyncDrainDone() and propagates the
 * drain ack back to onAudioSourceDrained(). */
static void testCxxAudioRendererDrainCompletesViaSource()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int ret = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_audio_frame_unref(frame);

	/* Trigger the drain cascade.  The frame is still in the renderer's
	 * queue; render() will consume it and complete the drain via
	 * idleDrain() thanks to the fix in AlsaAudioRenderer::render(). */
	ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Verifies setMediaId() / getMediaId() on IAudioRenderer by creating two
 * audio sources, attaching the renderer to source1, then switching it to
 * source2 mid-stream -- the IAudioRenderer counterpart of
 * testCxxVideoRendererSwitchMediaId (test_pipeline_renderer_video.cpp) and
 * the sink-side switch tests in test_pipeline_sourcesink_{coded,raw,audio}.cpp.
 *
 * Why this test exists: every other renderer in this file is attached to a
 * media at creation time via createAudioRenderer(mediaId, ...), resolved
 * through the broadcast+filter path (Session::onElementStateChanged ->
 * addAllMediaToAudioRenderer(), filtered internally by
 * AlsaAudioRenderer::addInputMedia()'s mMediaId check). None of them ever
 * calls Session::addMediaToAudioRenderer(unsigned int mediaId,
 * AudioRenderer*) / PipelineFactory's same-named overload -- the single-
 * lookup-by-id path, 0%-covered per the coverage report. The only caller of
 * that path is AlsaAudioRenderer::idleRenewMedia(), itself only scheduled by
 * setMediaId(): setMediaId(m2) sets mMediaId=m2 and schedules
 * idleRenewMedia() on the pomp loop; when that idle fires:
 * removeInputMedia(source1's media) -> mSession->addMediaToAudioRenderer(m2,
 * this) -> addInputMedia(source2's media). Both onAudioRendererMediaRemoved
 * and onAudioRendererMediaAdded fire synchronously within that same idle
 * callback. */
static void testCxxAudioRendererSwitchMediaId()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source1 = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source1,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto source1Owner = std::unique_ptr<IPdraw::IAudioSource>(source1);

	CU_ASSERT_EQUAL_FATAL(mediaListener.mAdded.size(), 1u);
	unsigned int media1Id = mediaListener.mAdded[0].id;
	CU_ASSERT_EQUAL(renderer->getMediaId(), media1Id);

	/* --- Source 2 and its media --- */
	struct pdraw_audio_source_params source2Params = {};
	source2Params.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	source2Params.audio.format = adef_pcm_16b_44100hz_stereo;

	AckingAudioSourceListener source2Listener;
	IPdraw::IAudioSource *source2 = nullptr;
	int ret = session->createAudioSource(
		&source2Params, &source2Listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::IAudioSource>(source2);

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 2; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mAdded[1].id;
	CU_ASSERT_NOT_EQUAL_FATAL(media2Id, 0u);
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, media2Id);

	/* --- Switch the renderer to source2 --- */
	rendererListener.mGotMediaAdded = false;
	rendererListener.mGotMediaRemoved = false;

	ret = renderer->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* idleRenewMedia() fires as an idle callback: removeInputMedia (fires
	 * onAudioRendererMediaRemoved) then
	 * Session::addMediaToAudioRenderer(media2Id, this) -> addInputMedia
	 * (fires onAudioRendererMediaAdded) happen within that same idle
	 * iteration. */
	bool gotRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRemoved);

	bool gotAdded2 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaAdded;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAdded2);
	CU_ASSERT_EQUAL(renderer->getMediaId(), media2Id);

	rendererOwner.reset();
	source1Owner.reset();
	source2Owner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

/* ── Helper: get the AlsaAudioRenderer's input AudioChannel ────────────
 * Pattern taken from testCxxVideoRendererBlurAndTransitions in
 * test_pipeline_renderer_video.cpp: cast the public IAudioRenderer handle
 * down to AudioRendererWrapper (defined in pdraw_renderer_audio.hpp), call
 * getAudioRenderer() to reach AlsaAudioRenderer, then use the inherited
 * Sink::getInputMedia() / Sink::getInputChannel() accessors. */
static Pdraw::Channel *getAlsaChannel(IPdraw::IAudioRenderer *renderer)
{
	auto *wrapper = static_cast<Pdraw::AudioRendererWrapper *>(renderer);
	if (wrapper == nullptr)
		return nullptr;
	Pdraw::AlsaAudioRenderer *alsa =
		static_cast<Pdraw::AlsaAudioRenderer *>(
			wrapper->getAudioRenderer());
	if (alsa == nullptr)
		return nullptr;
	Pdraw::Media *media = alsa->getInputMedia(0);
	if (media == nullptr)
		return nullptr;
	return alsa->getInputChannel(media);
}


static Pdraw::AlsaAudioRenderer *
getAlsaRenderer(IPdraw::IAudioRenderer *renderer)
{
	auto *wrapper = static_cast<Pdraw::AudioRendererWrapper *>(renderer);
	if (wrapper == nullptr)
		return nullptr;
	return static_cast<Pdraw::AlsaAudioRenderer *>(
		wrapper->getAudioRenderer());
}


/* Exercises AlsaAudioRenderer::onChannelEos().
 *
 * sendDownstreamEvent(EOS) on the renderer's AudioChannel triggers
 * onChannelEos(), which:
 *   1. Sets mEos = true.
 *   2. Calls Sink::onChannelEos() (no-op for AlsaAudioRenderer).
 *   3. Clears the watchdog timer so it no longer fires.
 *
 * Observable side-effects used for verification:
 *  - After EOS the renderer is still alive (getMediaId() returns a
 *    non-zero ID) — it does not tear down the pipeline.
 *  - A subsequent SOS resets mEos to false (verified by the following
 *    test); here we confirm the EOS round-trip does not crash or hang. */
static void testCxxAudioRendererOnChannelEos()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Push one frame so the renderer is in a normal running state with
	 * the watchdog timer armed before we send EOS. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	/* Let the frame be consumed by the renderer. */
	(void)loop.pumpUntil([]() { return false; }, 100);

	/* Obtain the AudioChannel that feeds AlsaAudioRenderer and send
	 * an EOS downstream event. */
	Pdraw::Channel *channel = getAlsaChannel(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Pdraw::Channel::DownstreamEvent::EOS);
	CU_ASSERT_EQUAL(err, 0);

	/* Pump the loop to let the EOS message be dispatched. */
	(void)loop.pumpUntil([]() { return false; }, 50);

	/* The renderer must still be alive and attached to its media. */
	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises AlsaAudioRenderer::onChannelSos().
 *
 * sendDownstreamEvent(EOS) followed by sendDownstreamEvent(SOS) verifies
 * the full EOS→SOS round-trip:
 *   1. EOS sets mEos = true and clears the watchdog timer.
 *   2. SOS resets mEos = false and calls Sink::onChannelSos().
 *
 * After the SOS the renderer must accept new frames normally, confirming
 * that mEos is false again (renderCb would re-arm the watchdog timer on
 * the next frame). */
static void testCxxAudioRendererOnChannelSos()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	Pdraw::Channel *channel = getAlsaChannel(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	/* Step 1: send EOS to put the renderer in "end-of-stream" state.
	 * mEos becomes true, watchdog timer is cleared. */
	int err = channel->sendDownstreamEvent(
		Pdraw::Channel::DownstreamEvent::EOS);
	CU_ASSERT_EQUAL(err, 0);
	(void)loop.pumpUntil([]() { return false; }, 50);

	/* Step 2: send SOS — mEos must be reset to false. */
	err = channel->sendDownstreamEvent(
		Pdraw::Channel::DownstreamEvent::SOS);
	CU_ASSERT_EQUAL(err, 0);
	(void)loop.pumpUntil([]() { return false; }, 50);

	/* Step 3: push a new frame after SOS and verify it is consumed
	 * cleanly (no crash, drain still completes). mEos == false means
	 * renderCb will call mWatchdogTimer->set() again on this frame. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	AckingAudioSourceListener sourceListener2;
	err = source->drain();
	CU_ASSERT_EQUAL_FATAL(err, 0);

	/* Verify drain completes (identical to testCxxAudioRendererDrain
	 * CompletesViaSource), confirming the renderer is fully operational
	 * after the EOS→SOS round-trip. */
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	/* Renderer must still be attached to its media. */
	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises AlsaAudioRenderer::onWatchdogTimer().
 *
 * The watchdog is a pomp::Timer that fires after
 * ALSA_RENDERER_WATCHDOG_TIME_S (2 s) without a new frame.  renderCb()
 * sets it each time a frame arrives.  onChannelEos() clears it.
 *
 * This test:
 *  1. Pushes one frame, which causes renderCb() to call
 *     mWatchdogTimer->set(2000 ms).
 *  2. Withholds further frames for > 2 s while pumping the pomp loop.
 *  3. Verifies that the renderer survives the watchdog callback (i.e.
 *     no crash or hang) and is still attached to its media.
 *
 * The watchdog only logs a warning (PDRAW_LOGW) and sets the internal
 * mWatchdogTriggered flag; it does not tear down the pipeline.  Since
 * mWatchdogTriggered is private we rely on the absence of a crash and
 * the renderer's continued liveness as the observable assertion. */
static void testCxxAudioRendererOnWatchdogTimer()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Push exactly one frame.  renderCb() will arm the 2 s watchdog
	 * timer when this frame is consumed by the render callback. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	/* Keep pumping the loop for 3 s (> ALSA_RENDERER_WATCHDOG_TIME_S=2 s)
	 * without sending more frames.  The watchdog fires within this
	 * window (logs a PDRAW_LOGW) and sets mWatchdogTriggered = true
	 * without touching the pipeline structure. */
	static constexpr int kWatchdogWaitMs = 3000;
	(void)loop.pumpUntil([]() { return false; }, kWatchdogWaitMs);

	/* The renderer must still be alive and attached after the watchdog
	 * fired. */
	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	/* Confirm that a new frame can still flow through the renderer after
	 * the watchdog — the watchdog is purely diagnostic and must not
	 * break normal operation. */
	struct mbuf_audio_frame *frame2 = makePcmFrame(23219200, 1);
	pushRet = mbuf_audio_frame_queue_push(inQueue, frame2);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame2);
	(void)loop.pumpUntil([]() { return false; }, 100);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

static void testCxxAudioRendererWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	CU_ASSERT_PTR_NULL(static_cast<Pdraw::AudioRendererWrapper *>(renderer)
				   ->getAudioRenderer());

	struct pdraw_audio_renderer_params params = {};
	CU_ASSERT_EQUAL(renderer->setMediaId(0), -EPROTO);
	CU_ASSERT_EQUAL(renderer->getMediaId(), (unsigned int)-EPROTO);
	CU_ASSERT_EQUAL(renderer->setParams(&params), -EPROTO);
	CU_ASSERT_EQUAL(renderer->getParams(&params), -EPROTO);

	rendererOwner.reset();
	sourceOwner.reset();
}


/* Exercises AlsaAudioRenderer::onChannelDrain() fast path (lines 403-407):
 * when the DRAIN signal arrives and the frame queue is empty, onChannelDrain()
 * skips the early return, calls setFlushingState(FLUSHED), and calls
 * asyncDrainDone() immediately.
 *
 * ExternalAudioSource::drain() → flush(false) cannot reach this code: when
 * the source queue is empty the source is already in FLUSHED state and
 * flush(false) posts mCompleteFlushHandler early WITHOUT calling
 * outputChannel->drain(), so onChannelDrain() is never triggered.
 *
 * Calling channel->drain() directly bypasses the source state machine and
 * fires onChannelDrain() unconditionally, with no frames in the renderer's
 * queue → fast path (lines 403-407) is taken. */
static void testCxxAudioRendererDrainEmptyQueue()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Send DRAIN downstream while the renderer's queue is empty.
	 * onChannelDrain() finds queue->getCount() == 0, skips the early
	 * return at line 401, and takes the fast path (lines 403-407):
	 * setFlushingState(FLUSHED) + asyncDrainDone(). */
	Pdraw::Channel *channel = getAlsaChannel(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->drain();
	CU_ASSERT_EQUAL(err, 0);

	/* Pump to let the drain signal and asyncDrainDone propagate. */
	(void)loop.pumpUntil([]() { return false; }, 100);

	/* Renderer must still be alive after the fast-path drain. */
	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Verifies that a source with an invalid audio format (channel_count=0)
 * never causes onAudioRendererMediaAdded to fire.
 *
 * AlsaAudioRenderer's caps only list the 24 supported 16-bit PCM formats.
 * Sink::addInputMedia() checks adef_format_intersect() against those caps
 * and returns -ENOSYS for any incompatible format, before fillMediaInfo()
 * and startAlsa() are ever called.  So startAlsa()'s own validation at
 * lines 163-168 is structurally unreachable via the public API. */
static void testCxxAudioRendererStartAlsaInvalidFormat()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format.encoding = ADEF_ENCODING_PCM;
	sourceParams.audio.format.channel_count = 0; /* Invalid format */

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	unsigned int mediaId = mediaListener.findAudioMedia()->id;

	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = kAlsaNullDevice;
	IPdraw::IAudioRenderer *renderer = nullptr;
	ret = session->createAudioRenderer(
		mediaId, &rendererParams, &rendererListener, &renderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);

	/* startAlsa() fails with -EINVAL due to invalid format: media_added
	 * shim will not fire. */
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(rendererListener.mGotMediaAdded);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Verifies that a source with an unsupported bit depth (32-bit) never causes
 * onAudioRendererMediaAdded to fire.
 *
 * Same reasoning as testCxxAudioRendererStartAlsaInvalidFormat: the renderer's
 * format caps contain only 16-bit entries, so Sink::addInputMedia() rejects
 * the 32-bit format via adef_format_intersect() before startAlsa() is called.
 * The adefFormatToAlsa() check at lines 171-175 is therefore also
 * structurally unreachable via the public API. */
static void testCxxAudioRendererStartAlsaUnsupportedFormat()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;
	sourceParams.audio.format.bit_depth = 32; /* not in renderer caps */

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	unsigned int mediaId = mediaListener.findAudioMedia()->id;

	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = kAlsaNullDevice;
	IPdraw::IAudioRenderer *renderer = nullptr;
	ret = session->createAudioRenderer(
		mediaId, &rendererParams, &rendererListener, &renderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);

	/* startAlsa() fails with -EINVAL: media_added shim will not fire. */
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(rendererListener.mGotMediaAdded);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises AlsaAudioRenderer::onChannelFlush(). */
static void testCxxAudioRendererOnChannelFlush()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	int ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		5000);
	CU_ASSERT_TRUE(gotFlushed);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises render() error path when frame buffer size is invalid. */
static void testCxxAudioRendererInvalidFrameSizeInRender()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	struct adef_frame frameInfo = {};
	frameInfo.format = adef_pcm_16b_44100hz_stereo;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(100, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, 100);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_audio_frame_unref(frame);

	(void)loop.pumpUntil([]() { return false; }, 200);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises AlsaAudioRenderer::addInputMedia error guards:
 *  - passing a RawVideoMedia (wrong type) returns -ENOSYS (l.648-651)
 *  - passing an AudioMedia when mLastAddedMedia != nullptr returns -EBUSY
 * (l.655-656)
 *
 * Both calls run inside a pomp_loop idle callback so they execute on the loop
 * thread, matching the threading model of addInputMedia(). */
static void testCxxAudioRendererAddInputMediaErrorGuards()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	Pdraw::AlsaAudioRenderer *alsa = getAlsaRenderer(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(alsa);

	struct Ctx {
		Pdraw::AlsaAudioRenderer *alsa;
		Pdraw::Session *session;
		int enosysRet = 1;
		int ebusyRet = 1;
		bool done = false;

		static void run(void *p)
		{
			auto *c = static_cast<Ctx *>(p);
			/* -ENOSYS: wrong media type (RawVideoMedia ≠
			 * AudioMedia) */
			Pdraw::RawVideoMedia mockVideo(c->session);
			c->enosysRet = c->alsa->addInputMedia(&mockVideo);
			/* -EBUSY: re-add the already-connected media (same ID,
			 * mLastAddedMedia != nullptr).  A fresh AudioMedia
			 * would
			 * have a different ID and return -EPERM instead. */
			Pdraw::Media *existing = c->alsa->getInputMedia(0);
			c->ebusyRet = (existing != nullptr)
					      ? c->alsa->addInputMedia(existing)
					      : -1;
			c->done = true;
		}
	} ctx{alsa, testSession.get()};

	pomp_loop_idle_add(loop.raw(), Ctx::run, &ctx);
	bool done = loop.pumpUntil([&ctx]() { return ctx.done; }, 5000);
	CU_ASSERT_TRUE_FATAL(done);
	CU_ASSERT_EQUAL(ctx.enosysRet, -ENOSYS);
	CU_ASSERT_EQUAL(ctx.ebusyRet, -EBUSY);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

/* Exercises render() empty-queue path (lines 888-894): signals the queue's
 * pomp_evt with no frames present, so renderCb fires and render() finds
 * count < 1.  render() schedules mIdleDrainHandler via idleAdd and returns
 * -EAGAIN.  idleDrain() then calls asyncDrainDone() (no-op, no pending drain)
 * and setFlushingState(FLUSHED). */
static void testCxxAudioRendererRenderEmptyQueue()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	Pdraw::Channel *channel = getAlsaChannel(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	Pdraw::Sink *owner = channel->getOwner();
	CU_ASSERT_PTR_NOT_NULL_FATAL(owner);

	mbuf::Queue *queue = channel->getQueue(owner);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct pomp_evt *evt = nullptr;
	int err = queue->getEvent(&evt);
	CU_ASSERT_EQUAL(err, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(evt);

	/* Signal the event with an empty queue → renderCb fires on next loop
	 * iteration → render() finds count=0 → lines 888-894 are executed. */
	err = pomp_evt_signal(evt);
	CU_ASSERT_EQUAL(err, 0);

	/* Pump to let renderCb and the subsequent idleDrain() run. */
	(void)loop.pumpUntil([]() { return false; }, 200);

	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

/* Exercises addInputMedia() error: label (lines 721-723):
 * the format is valid so Sink::addInputMedia() and attachToLoop() both
 * succeed, but snd_pcm_open() fails for the unknown device name →
 * startAlsa() returns an error → goto error: → removeInputMedia() is
 * called to roll back the partial setup.
 *
 * onAudioRendererMediaAdded must NOT fire because the renderer never
 * becomes fully operational. */
static void testCxxAudioRendererAddInputMediaStartAlsaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	unsigned int mediaId = mediaListener.findAudioMedia()->id;

	/* Non-existent device: snd_pcm_open() fails, triggering error: */
	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = "pdraw_test_nonexistent_device";

	IPdraw::IAudioRenderer *renderer = nullptr;
	ret = session->createAudioRenderer(
		mediaId, &rendererParams, &rendererListener, &renderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);

	/* Let addInputMedia() run and fail through startAlsa(). */
	(void)loop.pumpUntil([]() { return false; }, 500);

	/* Renderer was created but never became operational. */
	CU_ASSERT_FALSE(rendererListener.mGotMediaAdded);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Session::createAudioRenderer() validates params->address != nullptr before
 * creating any element (pdraw_session.cpp, same guard as createAlsaSource()).
 * Passing nullptr is therefore rejected synchronously with -EINVAL, before
 * any ALSA call or addInputMedia() is ever reached. */
static void testCxxAudioRendererAddInputMediaNullAddress()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	unsigned int mediaId = mediaListener.findAudioMedia()->id;

	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = nullptr;

	IPdraw::IAudioRenderer *renderer = nullptr;
	ret = session->createAudioRenderer(
		mediaId, &rendererParams, &rendererListener, &renderer);
	/* Null address is rejected at the session validation level, before any
	 * element or ALSA handle is created. */
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_PTR_NULL(renderer);
	CU_ASSERT_FALSE(rendererListener.mGotMediaAdded);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises render() error path at lines 946-948: non-EPIPE error from
 * snd_pcm_writei().  snd_pcm_drop() moves the PCM to SETUP state; a
 * subsequent snd_pcm_writei() in that state returns -EBADFD.  render()
 * logs the error and breaks out of the retry loop (no snd_pcm_prepare
 * retry, unlike the -EPIPE path at lines 942-944). */
static void testCxxAudioRendererRenderWriteError()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Drop the PCM to SETUP state so that snd_pcm_writei() returns
	 * -EBADFD on the next render() call. */
	Pdraw::AlsaAudioRenderer *alsa = getAlsaRenderer(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(alsa);
	snd_pcm_drop(alsa->mHandle);

	/* Push a frame: renderCb fires → render() pops the frame → reaches
	 * snd_pcm_writei() on a SETUP-state PCM → -EBADFD → lines 946-948. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	(void)loop.pumpUntil([]() { return false; }, 200);

	/* Renderer must survive a render() write error. */
	CU_ASSERT_NOT_EQUAL(renderer->getMediaId(), 0u);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises render() guard at lines 872-875: render() called while
 * mAlsaReady is false returns -EPROTO immediately without touching
 * the queue or the ALSA handle. */
static void testCxxAudioRendererRenderNotReady()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AudioRendererTrackingListener rendererListener;
	AckingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				mediaListener,
				rendererListener,
				sourceListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IAudioRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	/* Clear the ready flag so render() takes the early-exit at line 872. */
	Pdraw::AlsaAudioRenderer *alsa = getAlsaRenderer(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(alsa);
	alsa->mAlsaReady = false;

	/* Signal the render event with a frame queued → renderCb fires →
	 * render() finds mAlsaReady==false → lines 872-875, returns -EPROTO. */
	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame *frame = makePcmFrame(0, 0);
	int pushRet = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(pushRet, 0);
	mbuf_audio_frame_unref(frame);

	(void)loop.pumpUntil([]() { return false; }, 200);

	/* Restore so stopAlsa() / removeInputMedia() work normally. */
	alsa->mAlsaReady = true;

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

/* Exercises the 2 PdrawAudioRendererListener C API shims (pdraw_wrapper.cpp):
 * onAudioRendererMediaAdded and onAudioRendererMediaRemoved.
 *
 * Pipeline: ExternalAudioSource (PCM stereo 44100Hz) → AudioRenderer (ALSA
 * null device).  No frames need to be pushed: media_added fires when
 * AudioRendererAlsa::addInputMedia() → startAlsa() succeeds, and media_removed
 * fires when the source is destroyed and the session removes the raw audio
 * media. */
static void testCAudioRendererListenerCallbacks()
{
	struct Ud {
		unsigned int rawAudioMediaId = 0;
		bool gotRawAudioMedia = false;
		int rendererMediaAddedCount = 0;
		int rendererMediaRemovedCount = 0;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *u) {
		static_cast<Ud *>(u)->gotStopResp = true;
	};
	sessionCbs.media_added = [](struct pdraw *,
				    const struct pdraw_media_info *info,
				    void * /*elem_ud*/,
				    void *u) {
		auto *d = static_cast<Ud *>(u);
		if (info->type == PDRAW_MEDIA_TYPE_AUDIO &&
		    info->audio.format.encoding == ADEF_ENCODING_PCM &&
		    !d->gotRawAudioMedia) {
			d->rawAudioMediaId = info->id;
			d->gotRawAudioMedia = true;
		}
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	/* Raw PCM stereo source (same format as the ALSA renderer tests) */
	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	struct pdraw_audio_source_cbs sourceCbs = {};
	sourceCbs.flushed =
		[](struct pdraw *, struct pdraw_audio_source *, void *) {};

	struct pdraw_audio_source *src = nullptr;
	ret = pdraw_audio_source_new(
		p, &sourceParams, &sourceCbs, nullptr, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	bool gotMedia =
		loop.pumpUntil([&ud]() { return ud.gotRawAudioMedia; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Audio renderer: ALSA null device, media_added/media_removed shims */
	struct pdraw_audio_renderer_params rendererParams = {};
	rendererParams.address = "null";

	struct pdraw_audio_renderer_cbs rendererCbs = {};
	rendererCbs.media_added = [](struct pdraw *,
				     struct pdraw_audio_renderer *,
				     const struct pdraw_media_info *,
				     void *u) {
		static_cast<Ud *>(u)->rendererMediaAddedCount++;
	};
	rendererCbs.media_removed = [](struct pdraw *,
				       struct pdraw_audio_renderer *,
				       const struct pdraw_media_info *,
				       void *u) {
		static_cast<Ud *>(u)->rendererMediaRemovedCount++;
	};

	struct pdraw_audio_renderer *rnd = nullptr;
	ret = pdraw_audio_renderer_new(p,
				       ud.rawAudioMediaId,
				       &rendererParams,
				       &rendererCbs,
				       &ud,
				       &rnd);
	/* Should succeed: null device is always present with ALSA compiled in
	 */
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rnd);

	/* media_added fires when addInputMedia() → startAlsa() succeeds */
	bool gotAdded = loop.pumpUntil(
		[&ud]() { return ud.rendererMediaAddedCount >= 1; }, 5000);
	CU_ASSERT_TRUE(gotAdded);

	/* Destroying the source removes its media → renderer fires
	 * media_removed */
	ret = pdraw_audio_source_destroy(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotRemoved = loop.pumpUntil(
		[&ud]() { return ud.rendererMediaRemovedCount >= 1; }, 5000);
	CU_ASSERT_TRUE(gotRemoved);

	ret = pdraw_audio_renderer_destroy(p, rnd);
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


#endif /* PDRAW_TEST_RENDERER_AUDIO_ALSA_ENABLED */


CU_TestInfo g_pdraw_test_pipeline_renderer_audio[] = {
#ifdef PDRAW_TEST_RENDERER_AUDIO_ALSA_ENABLED
	{FN("testCxxAudioRendererMediaAdded"), testCxxAudioRendererMediaAdded},
	{FN("testCxxAudioRendererSetParamsRoundtrip"),
	 testCxxAudioRendererSetParamsRoundtrip},
	{FN("testCxxAudioRendererMediaRemovedWhenSourceDestroyed"),
	 testCxxAudioRendererMediaRemovedWhenSourceDestroyed},
	{FN("testCxxAudioRendererDrainCompletesViaSource"),
	 testCxxAudioRendererDrainCompletesViaSource},
	{FN("testCxxAudioRendererSwitchMediaId"),
	 testCxxAudioRendererSwitchMediaId},
	{FN("testCxxAudioRendererOnChannelEos"),
	 testCxxAudioRendererOnChannelEos},
	{FN("testCxxAudioRendererOnChannelSos"),
	 testCxxAudioRendererOnChannelSos},
	{FN("testCxxAudioRendererOnWatchdogTimer"),
	 testCxxAudioRendererOnWatchdogTimer},
	{FN("testCxxAudioRendererWrapperGuardsAfterElementCleared"),
	 testCxxAudioRendererWrapperGuardsAfterElementCleared},
	{FN("testCxxAudioRendererDrainEmptyQueue"),
	 testCxxAudioRendererDrainEmptyQueue},
	{FN("testCxxAudioRendererStartAlsaInvalidFormat"),
	 testCxxAudioRendererStartAlsaInvalidFormat},
	{FN("testCxxAudioRendererStartAlsaUnsupportedFormat"),
	 testCxxAudioRendererStartAlsaUnsupportedFormat},
	{FN("testCxxAudioRendererOnChannelFlush"),
	 testCxxAudioRendererOnChannelFlush},
	{FN("testCxxAudioRendererInvalidFrameSizeInRender"),
	 testCxxAudioRendererInvalidFrameSizeInRender},
	{FN("testCxxAudioRendererAddInputMediaErrorGuards"),
	 testCxxAudioRendererAddInputMediaErrorGuards},
	{FN("testCxxAudioRendererRenderEmptyQueue"),
	 testCxxAudioRendererRenderEmptyQueue},
	{FN("testCxxAudioRendererAddInputMediaStartAlsaFails"),
	 testCxxAudioRendererAddInputMediaStartAlsaFails},
	{FN("testCxxAudioRendererAddInputMediaNullAddress"),
	 testCxxAudioRendererAddInputMediaNullAddress},
	{FN("testCxxAudioRendererRenderWriteError"),
	 testCxxAudioRendererRenderWriteError},
	{FN("testCxxAudioRendererRenderNotReady"),
	 testCxxAudioRendererRenderNotReady},
	{FN("testCAudioRendererListenerCallbacks"),
	 testCAudioRendererListenerCallbacks},
#endif
	CU_TEST_INFO_NULL,
};
