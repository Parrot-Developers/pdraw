/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — ALSA source against the "null" capture device
 * (Tier B, self-contained fixture)
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

/* Requires CONFIG_PDRAW_USE_ALSA=y.
 *
 * Uses the ALSA "null" virtual device (part of alsa-lib's standard PCM plugin
 * set on any Linux host) for creation and pause/drain tests.  The null device
 * is suitable for PLAYBACK (discards all data) but NOT for CAPTURE: its
 * avail_update() always returns 0 for SND_PCM_STREAM_CAPTURE, so
 * AlsaSource::readFrame() perpetually returns -EAGAIN and no frames are ever
 * captured with it.
 *
 * AlsaSource opens the device during start() (unlike AlsaAudioRenderer which
 * defers to addInputMedia()), so a missing null device causes
 * createAlsaSource() to fail immediately rather than hanging.
 *
 * test_api_alsa_source.cpp covers null-arg guards and getCapabilities with an
 * invalid address.  This file covers the remaining observable behaviours that
 * ARE testable with the null device: readyToPlay, play/pause state changes,
 * pause/drain completion, and getCapabilities against a real device.
 *
 * A few behaviours can only be exercised with REAL captured samples flowing
 * through the pipeline (AlsaSource::onTimer/readFrame/processFrame, and the
 * onChannelFlushed/onChannelDrained callbacks, which require the source's
 * FlushingState to have left its default FLUSHED value -- something that
 * only processFrame() does, and only once it has actually queued a frame).
 * The null device can never provide this.  Real audio hardware isn't
 * available in headless CI either, but the "snd-aloop" kernel module's
 * virtual "Loopback" card usually is: capturing from its capture subdevice
 * (hw:Loopback,1,0) produces a continuous stream of real (silent, since
 * nothing writes to the paired playback subdevice) PCM frames, driven by the
 * module's own virtual hardware clock -- no writer needed on the other end.
 * The tests below that need this probe for the device first via
 * alsaLoopbackCaptureAvailable() and skip gracefully (CU_PASS) if it isn't
 * there -- unlike PDRAW_GET_ASSET_PATH (missing NAS assets), which fails the
 * test outright: a missing kernel module is an environment difference we
 * want to tolerate silently, not a configuration mistake to flag. */

#ifdef PDRAW_USE_ALSA
#	define PDRAW_TEST_ALSA_SOURCE_ENABLED 1
#endif

#define ULOG_TAG pdraw_test_pipeline_alsa_source
#include "test_api_common.hpp"
#include "test_common.h"

#include "pdraw_alsa_audio.hpp"
#include "pdraw_element.hpp"
#define private public
#define protected public
#include "pdraw_alsa_source.hpp"
#undef protected
#undef private

#include "test_fixtures.hpp"

#include <audio-defs/adefs.h>
#include <media-buffers/mbuf_audio_frame.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

#ifdef PDRAW_TEST_ALSA_SOURCE_ENABLED

static constexpr const char *kAlsaNullDevice = "null";

/* Capture subdevice of the "snd-aloop" kernel module's virtual "Loopback"
 * card (see the file-level comment above). Not guaranteed to exist. */
static constexpr const char *kAlsaLoopbackCaptureDevice = "hw:Loopback,1,0";


/* Probes whether kAlsaLoopbackCaptureDevice can actually be opened on this
 * host, without needing to start a real capture (getCapabilities() only
 * opens the device to read its hw_params, then closes it -- see
 * AlsaSource::getCapabilities()). Returns false both when the "snd-aloop"
 * module isn't loaded at all and when it is but this specific subdevice
 * doesn't exist (e.g. a different pcm_substreams configuration). */
static bool alsaLoopbackCaptureAvailable()
{
	struct pdraw_alsa_source_caps caps = {};
	return pdrawAlsaSourceGetCapabilities(kAlsaLoopbackCaptureDevice,
					      &caps) == 0;
}


/* Opens kAlsaLoopbackCaptureDevice with the exact same hw_params
 * AlsaSource::start() configures (format/rate from
 * adef_pcm_16b_44100hz_stereo, period_size = ALSA_AUDIO_DEFAULT_SAMPLE_COUNT,
 * matching access/channels), reads back the buffer_size alsa-lib actually
 * picked for it on THIS host, then closes the device again -- ALSA doesn't
 * allow two concurrent opens of the same capture subdevice, so this must run
 * to completion before the real AlsaSource is created.
 *
 * Used by testCxxAlsaSourceCapturedFrameRecoversFromOverrun below to compute
 * a stall duration that reliably exceeds whatever buffer duration this
 * host's alsa-lib/snd-aloop combination happens to choose, rather than
 * guessing a fixed number of seconds: a first attempt at this test used a
 * fixed 3 s stall and it was NOT enough to trigger a real overrun on at
 * least one host (see TEST_PROGRESS.md) -- the buffer_size alsa-lib picks
 * for an otherwise-unconstrained hw_params_any() is host/driver-dependent
 * and was empirically >= 100 periods (>= 2.3 s) there. */
static bool probeLoopbackCaptureBufferDurationUs(uint64_t *durationUs)
{
	int ret;
	snd_pcm_t *handle = nullptr;
	snd_pcm_hw_params_t *hwParams = nullptr;
	snd_pcm_uframes_t bufferSize = 0;
	unsigned int rate = adef_pcm_16b_44100hz_stereo.sample_rate;
	snd_pcm_format_t format =
		AlsaAudio::adefFormatToAlsa(&adef_pcm_16b_44100hz_stereo);

	ret = snd_pcm_open(&handle,
			   kAlsaLoopbackCaptureDevice,
			   SND_PCM_STREAM_CAPTURE,
			   SND_PCM_NONBLOCK);
	if (ret < 0)
		return false;

	ret = snd_pcm_hw_params_malloc(&hwParams);
	if (ret < 0) {
		snd_pcm_close(handle);
		return false;
	}

	ret = snd_pcm_hw_params_any(handle, hwParams);
	if (ret >= 0)
		ret = snd_pcm_hw_params_set_access(
			handle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (ret >= 0)
		ret = snd_pcm_hw_params_set_format(handle, hwParams, format);
	if (ret >= 0)
		ret = snd_pcm_hw_params_set_rate(handle, hwParams, rate, 0);
	if (ret >= 0)
		ret = snd_pcm_hw_params_set_period_size(
			handle, hwParams, ALSA_AUDIO_DEFAULT_SAMPLE_COUNT, 0);
	if (ret >= 0)
		ret = snd_pcm_hw_params_set_channels(
			handle,
			hwParams,
			adef_pcm_16b_44100hz_stereo.channel_count);
	if (ret >= 0)
		ret = snd_pcm_hw_params(handle, hwParams);
	if (ret >= 0)
		ret = snd_pcm_hw_params_get_buffer_size(hwParams, &bufferSize);

	snd_pcm_hw_params_free(hwParams);
	snd_pcm_close(handle);

	if (ret < 0)
		return false;

	*durationUs = (static_cast<uint64_t>(bufferSize) * 1000000ULL) / rate;
	return true;
}


/* Anonymous namespace: same ODR-safety rationale as all other
 * test_pipeline_*.cpp files (see test_pipeline_vipc_source.cpp's namespace
 * comment for the full explanation). */
namespace {

/* Session-wide listener recording the ALSA source's output audio media. */
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


/* Tracks IAlsaSource lifecycle and frame-production events. */
class AlsaSourceTrackingListener : public IPdraw::IAlsaSource::Listener {
public:
	void alsaSourceReadyToPlay(
		IPdraw * /*p*/,
		IPdraw::IAlsaSource * /*s*/,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override
	{
		mReady = ready;
		mGotReadyToPlay = true;
		mReadyToPlayCallCount++;
		mLastEosReason = eosReason;
	}

	void alsaSourcePlayResponse(IPdraw * /*p*/,
				    IPdraw::IAlsaSource * /*s*/) override
	{
		/* AlsaSource::play() is synchronous and never calls
		 * playResponse() in the current implementation. */
		mGotPlayResponse = true;
	}

	void alsaSourcePauseResponse(IPdraw * /*p*/,
				     IPdraw::IAlsaSource * /*s*/) override
	{
		mGotPauseResponse = true;
	}

	void alsaSourceFrameReady(IPdraw * /*p*/,
				  IPdraw::IAlsaSource * /*s*/,
				  struct mbuf_audio_frame * /*frame*/) override
	{
		/* The null capture device never produces data (avail_update
		 * always returns 0 for SND_PCM_STREAM_CAPTURE), so with it
		 * this callback is never invoked in practice; with a real (or
		 * loopback) capture device it is, hence this counter. Not
		 * ours to unref: AlsaSource::processFrame() finalizes and
		 * unrefs the frame itself right after this call returns. */
		mFrameReadyCount++;
	}

	bool mGotReadyToPlay = false;
	bool mReady = false;
	int mReadyToPlayCallCount = 0;
	enum pdraw_alsa_source_eos_reason mLastEosReason =
		PDRAW_ALSA_SOURCE_EOS_REASON_NONE;
	bool mGotPlayResponse = false;
	bool mGotPauseResponse = false;
	int mFrameReadyCount = 0;
};


/* Acknowledges flush/drain requests from a connected AlsaSource — required for
 * AlsaSource::pause()'s internal drain() to ever complete.  The pause path
 * sends a drain signal down the output channel; the sink must pop the queue
 * and call queueDrained() before AlsaSource::onChannelDrained() fires and
 * eventually calls alsaSourcePauseResponse(). */
class QueueDrainingAudioSinkListener : public IPdraw::IAudioSink::Listener {
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
		struct mbuf_audio_frame *f = nullptr;
		while (mQueue != nullptr &&
		       mbuf_audio_frame_queue_pop(mQueue, &f) == 0)
			mbuf_audio_frame_unref(f);
		sk->queueFlushed();
		mGotFlush = true;
	}

	void onAudioSinkDrain(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		struct mbuf_audio_frame *f = nullptr;
		while (mQueue != nullptr &&
		       mbuf_audio_frame_queue_pop(mQueue, &f) == 0)
			mbuf_audio_frame_unref(f);
		sk->queueDrained();
	}

	/* Must be set immediately after createAudioSink() returns. */
	struct mbuf_audio_frame_queue *mQueue = nullptr;
	bool mGotFlush = false;
};

} /* anonymous namespace */


/* Shared setup: creates an AlsaSource configured for the given capture device
 * (the null device by default) and waits for readyToPlay.  Optionally also
 * creates an AudioSink on the resulting media and sets sinkListener.mQueue
 * (pass nullptr for both outSink and sinkListener when a downstream consumer
 * is not needed). */
static void setupAlsaSource(TestPompLoop &loop,
			    IPdraw *session,
			    MediaTrackingListener &mediaListener,
			    AlsaSourceTrackingListener &sourceListener,
			    QueueDrainingAudioSinkListener *sinkListener,
			    IPdraw::IAlsaSource **outSource,
			    IPdraw::IAudioSink **outSink,
			    const char *address = kAlsaNullDevice)
{
	struct pdraw_alsa_source_params params = {};
	params.address = address;
	params.audio.format = adef_pcm_16b_44100hz_stereo;
	/* sample_count = 0: AlsaSource defaults to
	 * ALSA_AUDIO_DEFAULT_SAMPLE_COUNT (1024), period ≈ 23 ms */

	int ret =
		session->createAlsaSource(&params, &sourceListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	/* readyToPlay fires synchronously from createMedia() (before the idle
	 * callOnMediaAdded).  Pump once to be safe in case it is already set.
	 */
	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(sourceListener.mReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	if (outSink == nullptr || sinkListener == nullptr)
		return;

	unsigned int mediaId = mediaListener.findAudioMedia()->id;
	ret = session->createAudioSink(mediaId, sinkListener, outSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSink);
	sinkListener->mQueue = (*outSink)->getQueue();
}


/* Creates an ALSA source on the null capture device and verifies the ready-
 * to-play callback fires with ready=true, and that the source output media
 * appears in the session.  At this point play() has not been called yet:
 * isReadyToPlay() must be true but isPaused() must also be true (capture has
 * not started). */
static void testCxxAlsaSourceCreationReadyToPlay()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr);

	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);
	CU_ASSERT_TRUE(source->isReadyToPlay());
	CU_ASSERT_TRUE(source->isPaused());
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_ALSA_SOURCE_EOS_REASON_NONE);

	const MediaTrackingListener::Added *media =
		mediaListener.findAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	CU_ASSERT_EQUAL(media->format.channel_count,
			adef_pcm_16b_44100hz_stereo.channel_count);
	CU_ASSERT_EQUAL(media->format.sample_rate,
			adef_pcm_16b_44100hz_stereo.sample_rate);

	/* Stop the session (and wait for it) BEFORE resetting sourceOwner, so
	 * that Session::asyncElementDelete() destroys the AlsaSource element
	 * (and thus runs AlsaSourceWrapper::clearElement()) while sourceOwner
	 * is still alive -- resetting first would run ~ElementWrapper()
	 * instead and the override would never execute. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	CU_ASSERT_PTR_NULL(
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource());

	sourceOwner.reset();
}


/* Calls play() and verifies that isPaused() flips to false and
 * isReadyToPlay() remains true.  The null device produces no capture data
 * (see the file-level comment), so no frame-arrival assertion is made here;
 * the pause/drain test below
 * (testCxxAlsaSourcePauseDrainsAndCallsPauseResponse)
 * exercises the full play → pause cycle. */
static void testCxxAlsaSourcePlayChangesState()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr);

	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);
	CU_ASSERT_TRUE(source->isPaused());

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_TRUE(source->isReadyToPlay());
	CU_ASSERT_FALSE(source->isPaused());

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises the pause() path with a downstream AudioSink attached: pause()
 * stops the ALSA timer and initiates a drain (even with an empty queue — the
 * null capture device produces no data, so the sink queue is always empty at
 * the time of the call); the sink acknowledges via queueDrained();
 * AlsaSource::onChannelDrained() fires and eventually calls
 * alsaSourcePauseResponse().  A second pause() while the first is still
 * pending must return -EALREADY. */
static void testCxxAlsaSourcePauseDrainsAndCallsPauseResponse()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAlsaSource *source = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			&sinkListener,
			&source,
			&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FALSE(source->isPaused());

	ret = source->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* The drain initiated by pause() is still in flight at this point
	 * (the sink's onAudioSinkDrain has not fired yet — it will only be
	 * dispatched after the current loop iteration): a second pause() must
	 * return -EALREADY rather than silently succeeding or dropping the
	 * first drain. */
	int secondRet = source->pause();
	CU_ASSERT_EQUAL(secondRet, -EALREADY);

	bool gotPauseResponse = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotPauseResponse;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotPauseResponse);
	CU_ASSERT_TRUE(source->isPaused());

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Shared setup for the two tests below that need REAL captured frames (as
 * opposed to just create/play/pause state-machine behaviour, which the tests
 * above already cover with the null device): probes
 * kAlsaLoopbackCaptureDevice first and CU_PASS-skips if it's unavailable on
 * this host (returns false so the caller's test function can return right
 * away), otherwise creates an AlsaSource + AudioSink on it, calls play(), and
 * waits for at least one real frame to reach alsaSourceFrameReady() -- proof
 * that AlsaSource::onTimer()/readFrame()/processFrame() actually ran, and
 * that the FlushingState has left its default FLUSHED value (only
 * processFrame() does that, and only once it has queued a frame into an
 * output channel -- hence outSink/sinkListener are mandatory here, unlike
 * setupAlsaSource()). */
static bool
setupAlsaSourceWithCapturedFrame(TestPompLoop &loop,
				 IPdraw *session,
				 MediaTrackingListener &mediaListener,
				 AlsaSourceTrackingListener &sourceListener,
				 QueueDrainingAudioSinkListener &sinkListener,
				 IPdraw::IAlsaSource **outSource,
				 IPdraw::IAudioSink **outSink)
{
	if (!alsaLoopbackCaptureAvailable()) {
		CU_PASS("hw:Loopback capture device (snd-aloop kernel module) "
			"not available on this host -- skipping real ALSA "
			"capture test");
		return false;
	}

	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			&sinkListener,
			outSource,
			outSink,
			kAlsaLoopbackCaptureDevice);

	int ret = (*outSource)->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFrame = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mFrameReadyCount >= 1;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	return true;
}


/* Covers AlsaSource::onTimer()/readFrame()/processFrame() (0% with the null
 * device, see the file-level comment) and onChannelFlushed() -- the latter is
 * only reached once a real frame has been queued into an output channel,
 * taking the FlushingState out of its default FLUSHED value.
 *
 * Getting there is not as simple as tearing the pipeline down and letting
 * flush() run: a real run showed line 534 still at 0 hits even after
 * fixing an earlier ordering bug in this test (calling session->stop()
 * before resetting sinkOwner/sourceOwner instead of after -- see
 * TEST_PROGRESS.md). The actual log from that run explains why:
 *   ExternalAudioSink#317: element state change to STOPPING
 *   ExternalAudioSink#317: unlink media name=AudioMedia#248
 *   AlsaSource#316: channel upstream event UNLINK
 *   AlsaSource#316: unlink media name=AudioMedia#248
 *   AlsaSource#316: element state change to STOPPING
 *   AlsaSource#316: element flushing state change to FLUSHING (discard=1)
 *   AlsaSource#316: element flushing state change to FLUSHED
 * Session::stop() (pdraw_session.cpp) iterates mElements and calls
 * elem->stop() on each in turn, but it stops the AudioSink BEFORE the
 * AlsaSource regardless of creation order -- the sink's own stop()
 * synchronously unlinks the channel (channelTeardown()) before
 * AlsaSource::stop()'s flush() ever runs, so flush() finds no output
 * channel left and takes the "nothing to flush" shortcut straight to
 * completeFlush(), same as the FLUSHED-shortcut case with the null device.
 * There is no ordering of session->stop() vs owner resets that fixes this:
 * the sink-before-source order is internal to Session::stop() itself.
 *
 * The fix is to not go through Session::stop() (nor the wrapper
 * destructors, which have the same problem when reset in the wrong order)
 * for the AlsaSource side at all: reach the real production AlsaSource
 * object via AlsaSourceWrapper::getAlsaSource() (same test-only-cast
 * pattern as testCxxAlsaSourceCreationReadyToPlay above) and call its
 * stop() directly, while the sink and the channel are still fully alive.
 * That runs the real outputChannel->flush() -> ExternalAudioSink::
 * onChannelFlush() -> ... -> channel->flushDone() ->
 * AlsaSource::onChannelFlushed() chain for real. session->stop() is still
 * called afterwards for a clean, fully-torn-down pipeline (a no-op on the
 * AlsaSource side by then, since its state is no longer STARTED/STARTING). */
static void testCxxAlsaSourceCapturedFrameFlushesOnStop()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAlsaSource *source = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	if (!setupAlsaSourceWithCapturedFrame(loop,
					      session,
					      mediaListener,
					      sourceListener,
					      sinkListener,
					      &source,
					      &sink))
		return;
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	auto *rawSource =
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSource);
	int rawStopRet = rawSource->stop();
	CU_ASSERT_EQUAL_FATAL(rawStopRet, 0);

	/* Proof that the round trip through the still-live sink completed --
	 * by the time onAudioSinkFlush() (and thus sk->queueFlushed()) has
	 * run, AlsaSource::onChannelFlushed() has already fired too: it is
	 * the very next synchronous step in flushDone()'s call chain. */
	bool gotFlush = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotFlush; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotFlush);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE_FATAL(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	sinkOwner.reset();
	sourceOwner.reset();
}


/* Covers AlsaSource::onChannelDrained() -- like onChannelFlushed() above,
 * only reached once a real frame has taken the FlushingState out of its
 * default FLUSHED value. Unlike testCxxAlsaSourcePauseDrainsAndCallsPause
 * Response (null device, no frame ever queued, drain short-circuits),
 * pause() here drains for real. */
static void testCxxAlsaSourceCapturedFrameDrainsOnPause()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAlsaSource *source = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	if (!setupAlsaSourceWithCapturedFrame(loop,
					      session,
					      mediaListener,
					      sourceListener,
					      sinkListener,
					      &source,
					      &sink))
		return;
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	int ret = source->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotPauseResponse = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotPauseResponse;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotPauseResponse);
	CU_ASSERT_TRUE(source->isPaused());

	/* Stop the session before resetting the owners, same reasoning as in
	 * testCxxAlsaSourceCapturedFrameFlushesOnStop() above -- harmless here
	 * since completeFlush() already reset FlushingState to FLUSHED when
	 * the pause()/drain() above completed, but kept consistent to avoid
	 * the same latent ordering trap. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


/* Covers AlsaSource::readFrame()'s overrun ("xrun") recovery path
 * (recover_xrun label, pdraw_alsa_source.cpp:1217-1237), reached when
 * snd_pcm_avail() or snd_pcm_readi() returns -EPIPE. Neither is exercised by
 * any other test in this file, nor observable through the null device (its
 * avail_update() never signals an xrun: it always returns 0 for CAPTURE, see
 * the file-level comment).
 *
 * mHandle is a real snd_pcm_t* with no mock/seam around it (see
 * TEST_PROGRESS.md), so forcing a real -EPIPE doesn't need one: it only
 * requires NOT draining the driver's capture ring buffer for long enough
 * that it genuinely overflows. The snd-aloop kernel module's virtual capture
 * clock keeps running on its own (see setupAlsaSourceWithCapturedFrame's
 * comment above) -- it keeps producing silent PCM into the ring buffer
 * whether or not anything reads it. So: get capture going and confirm the
 * first frame arrives normally (proves the fixture is sound), then simply
 * DON'T pump the test loop for long enough -- AlsaSource::onTimer() never
 * fires, readFrame() is never called, and the ring buffer is never drained.
 *
 * How long is "long enough" is NOT guessed: a first attempt at this test used
 * a fixed 3 s stall through a connected-but-never-drained AudioSink (mirroring
 * setupAlsaSourceWithCapturedFrame) and it did NOT reproduce a real overrun --
 * worse, it masked the question entirely, because AlsaSource::onTimer()'s
 * `while ((err = readFrame()) != -EAGAIN)` loop caught up on ~100 backlogged
 * periods in one burst once resumed, each queuing an mbuf_pool buffer that the
 * test's sink never popped (it only drained on flush/drain, not continuously),
 * so the bounded (100-buffer, NO_GROW) mbuf_pool exhausted on its own --
 * readFrame() returned -EAGAIN from mbuf_pool_get() (pdraw_alsa_source.cpp:
 * 1172-1176) well before ever reaching a real ALSA-level -EPIPE, and the test
 * just hung until its own timeout. See TEST_PROGRESS.md for the full log.
 *
 * The fix has two parts:
 *  1. No downstream AudioSink at all here (unlike
 *     setupAlsaSourceWithCapturedFrame): alsaSourceFrameReady() fires
 *     regardless of whether any output channel is linked (processFrame()
 *     calls it unconditionally, before the per-channel channel->queue()
 *     loop), and with zero output channels each read's mbuf_pool buffer is
 *     released again immediately once processFrame() returns (both
 *     processFrame()'s own mbufFrame ref and readFrame()'s mem ref are
 *     dropped synchronously within the same readFrame() call) -- so the pool
 *     can never fill up and mask a real overrun with an unrelated one of its
 *     own.
 *  2. The stall duration is derived from a real probe
 *     (probeLoopbackCaptureBufferDurationUs() above), which opens the same
 *     device with the exact hw_params AlsaSource::start() would use and asks
 *     alsa-lib what buffer_size it actually picked on THIS host, rather than
 *     assuming any particular default.
 *
 * There's still no counter or callback exposing "an xrun was handled"
 * (readFrame() is a private method with no observable side effect specific
 * to this path), but AlsaSource::onTimer() gives an indirect, still-precise
 * signal: on recover_xrun's success branch it returns -EAGAIN, so onTimer()'s
 * while loop simply exits and capture resumes on the next timer tick --
 * observable as alsaSourceFrameReady() firing again. On recover_xrun's
 * failure branch (only if snd_pcm_recover() itself fails, e.g. an unusually
 * broken host ALSA setup), readFrame() returns the original -EPIPE, so
 * onTimer() takes its unrecoverable_error path and calls
 * alsaSourceReadyToPlay(false, PDRAW_ALSA_SOURCE_EOS_REASON_UNRECOVERABLE_
 * ERROR) -- also observable. Either way, reaching recover_xrun (and logging
 * "overrun occured") is a prerequisite for both outcomes: without it,
 * neither new frames nor a ready-to-play(false) transition would ever
 * happen, since ALSA does not self-heal from an xrun without an explicit
 * snd_pcm_recover() call from the application. So this test accepts both
 * outcomes as proof the recovery path ran, and only fails if NEITHER happens
 * within the timeout. */
static void testCxxAlsaSourceCapturedFrameRecoversFromOverrun()
{
	uint64_t bufferDurationUs = 0;
	if (!probeLoopbackCaptureBufferDurationUs(&bufferDurationUs)) {
		/* CU_PASS()'s message is only echoed on failure, never on a
		 * pass, so without a real log line here this skip is
		 * indistinguishable from the test actually having exercised
		 * (and passed) the overrun path -- see TEST_PROGRESS.md. */
		ULOGW("%s: hw:Loopback capture device (snd-aloop kernel "
		      "module) not available on this host -- skipping real "
		      "ALSA overrun test",
		      __func__);
		CU_PASS("hw:Loopback capture device (snd-aloop kernel module) "
			"not available on this host -- skipping real ALSA "
			"overrun test");
		return;
	}

	/* Stall for comfortably more than one full buffer's worth of unread
	 * capture, so the kernel-level ring buffer is guaranteed to overflow
	 * for real. A flat 2x+1s margin was tried first and turned out to
	 * exceed kMaxStallUs on a real host where hw:Loopback,1,0's
	 * unconstrained buffer_size (same unconstrained hw_params_any() call
	 * AlsaSource::start() makes) resolved to ~11.9s -- alsa-lib appears
	 * to pick close to the driver-reported maximum when buffer_size is
	 * left totally unpinned, rather than some small "near" default (see
	 * TEST_PROGRESS.md). A relative 25% margin (floored at 1s for hosts
	 * with a tiny buffer_duration) still safely exceeds the real buffer
	 * on any host while keeping the total comfortably under the cap even
	 * for that ~11.9s case (~14.9s stall). Capped so a truly pathological
	 * buffer_size still skips gracefully instead of making this test take
	 * minutes. */
	constexpr uint64_t kMaxStallUs = 20000000; /* 20 s */
	uint64_t margin = bufferDurationUs / 4;
	if (margin < 1000000)
		margin = 1000000;
	uint64_t stallUs = bufferDurationUs + margin;
	if (stallUs > kMaxStallUs) {
		ULOGW("%s: hw:Loopback capture buffer_size on this host is "
		      "unusually large (%" PRIu64
		      "us) -- skipping to avoid an excessively long test",
		      __func__,
		      bufferDurationUs);
		CU_PASS("hw:Loopback capture buffer_size on this host is "
			"unusually large -- skipping to avoid an excessively "
			"long test");
		return;
	}

	ULOGI("%s: hw:Loopback capture buffer_size=%" PRIu64
	      "us, stalling for %" PRIu64 "us before resuming",
	      __func__,
	      bufferDurationUs,
	      stallUs);

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr,
			kAlsaLoopbackCaptureDevice);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFrame = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mFrameReadyCount >= 1;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	int framesBeforeStall = sourceListener.mFrameReadyCount;
	int readyToPlayCallsBeforeStall = sourceListener.mReadyToPlayCallCount;

	/* Deliberately do NOT pump the loop here: AlsaSource::onTimer() must
	 * not run so that readFrame() never drains the capture ring buffer,
	 * letting the snd-aloop module's own virtual hardware clock overflow
	 * it for real. */
	std::this_thread::sleep_for(std::chrono::microseconds(stallUs));

	bool recovered = loop.pumpUntil(
		[&sourceListener,
		 framesBeforeStall,
		 readyToPlayCallsBeforeStall]() {
			return (sourceListener.mFrameReadyCount >
				framesBeforeStall) ||
			       (sourceListener.mReadyToPlayCallCount >
				readyToPlayCallsBeforeStall);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(recovered);

	if (sourceListener.mReadyToPlayCallCount >
	    readyToPlayCallsBeforeStall) {
		/* snd_pcm_recover() itself failed (rare/host-dependent): the
		 * recover_xrun path still ran (that's what got us here), but
		 * the source gave up rather than retrying forever. */
		ULOGI("%s: recover_xrun ran, snd_pcm_recover() failed -- "
		      "source gave up (unrecoverable error)",
		      __func__);
		CU_ASSERT_FALSE(sourceListener.mReady);
		CU_ASSERT_EQUAL(
			sourceListener.mLastEosReason,
			PDRAW_ALSA_SOURCE_EOS_REASON_UNRECOVERABLE_ERROR);
	} else {
		/* The common case: recovered and capture resumed normally. */
		ULOGI("%s: recover_xrun ran, snd_pcm_recover() succeeded -- "
		      "capture resumed",
		      __func__);
		CU_ASSERT_TRUE(sourceListener.mReady);
	}

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* C API shim coverage for AlsaSource::alsaSourceFrameReady (pdraw_wrapper.cpp).
 * Uses a dedicated struct pdraw * (created via pdraw_new()) and the snd-aloop
 * "Loopback" virtual capture device, which produces real silent PCM frames
 * driven by the kernel module's virtual clock — no writer is needed on the
 * paired playback subdevice.
 *
 * Skips gracefully (CU_PASS) when the snd-aloop kernel module is not loaded:
 * this is an environment difference, not a test-configuration mistake.
 *
 * The null capture device (used by testCAlsaSourceListenerCallbacks in
 * test_api_alsa_source.cpp) cannot produce captured frames (snd_pcm_avail()
 * always returns 0 for CAPTURE), so frame_ready cannot be observed with it.
 *
 * Also re-exercises ready_to_play and pause_resp via the C API shim path.
 *
 * pause_resp via FLUSHED shortcut: processFrame() only sets FlushingState to
 * UNFLUSHED when a frame is successfully queued to a downstream channel.  With
 * no downstream AudioSink connected, FlushingState stays FLUSHED even after
 * real captured frames arrive.  pause() → drain() → flush() takes the FLUSHED
 * shortcut → async completeFlush() → pauseResponse() → C callback. */
static void testCAlsaSourceListenerCbsWithLoopback()
{
	if (!alsaLoopbackCaptureAvailable()) {
		CU_PASS("hw:Loopback capture (snd-aloop) not available -- "
			"skipping C API frame_ready shim test");
		return;
	}

	struct Ud {
		int readyToPlayCount = 0;
		bool lastReady = false;
		int pauseRespCount = 0;
		int frameReadyCount = 0;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *userdata) {
		static_cast<Ud *>(userdata)->gotStopResp = true;
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	struct pdraw_alsa_source_params params = {};
	params.address = kAlsaLoopbackCaptureDevice;
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	struct pdraw_alsa_source_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_alsa_source *,
			       int ready,
			       enum pdraw_alsa_source_eos_reason,
			       void *userdata) {
		auto *u = static_cast<Ud *>(userdata);
		u->lastReady = (ready != 0);
		u->readyToPlayCount++;
	};
	cbs.pause_resp =
		[](struct pdraw *, struct pdraw_alsa_source *, void *userdata) {
			static_cast<Ud *>(userdata)->pauseRespCount++;
		};
	cbs.frame_ready = [](struct pdraw *,
			     struct pdraw_alsa_source *,
			     struct mbuf_audio_frame *,
			     void *userdata) {
		static_cast<Ud *>(userdata)->frameReadyCount++;
	};

	struct pdraw_alsa_source *src = nullptr;
	ret = pdraw_alsa_source_new(p, &params, &cbs, &ud, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* ready_to_play(true) fires from createMedia() inside start() */
	bool gotReady = loop.pumpUntil(
		[&ud]() { return ud.readyToPlayCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(ud.lastReady);

	/* play(): arm the ALSA timer → loopback produces real (silent) PCM
	 * frames
	 */
	ret = pdraw_alsa_source_play(p, src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* frame_ready shim fires once onTimer()/readFrame()/processFrame() runs
	 */
	bool gotFrame = loop.pumpUntil(
		[&ud]() { return ud.frameReadyCount >= 1; }, 10000);
	CU_ASSERT_TRUE(gotFrame);

	/* pause() → drain() → FLUSHED shortcut (no downstream channels) →
	 * pause_resp */
	ret = pdraw_alsa_source_pause(p, src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause =
		loop.pumpUntil([&ud]() { return ud.pauseRespCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotPause);

	ret = pdraw_alsa_source_destroy(p, src);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


/* pdrawAlsaSourceGetCapabilities() against the null capture device (which is
 * always present when ALSA support is compiled in): must succeed and return a
 * non-trivial capability range (at least one channel, at least one sample
 * rate).  test_api_alsa_source.cpp covers the null-caps guard and an invalid
 * address; this test covers the successful path with a real device. */
static void testCxxAlsaSourceGetCapabilitiesNullDevice()
{
	struct pdraw_alsa_source_caps caps = {};
	int ret = pdrawAlsaSourceGetCapabilities(kAlsaNullDevice, &caps);
	CU_ASSERT_EQUAL(ret, 0);
	if (ret == 0) {
		CU_ASSERT(caps.channel_count.min > 0);
		CU_ASSERT(caps.channel_count.max >= caps.channel_count.min);
		CU_ASSERT(caps.sample_rate.min > 0);
		CU_ASSERT(caps.sample_rate.max >= caps.sample_rate.min);
	}
}

static void testCxxAlsaSourceWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	CU_ASSERT_PTR_NULL(
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource());

	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_FALSE(source->isPaused());
	CU_ASSERT_EQUAL(source->play(), -EPROTO);
	CU_ASSERT_EQUAL(source->pause(), -EPROTO);

	sourceOwner.reset();
}


/* Covers AlsaSource::play() and pause() when the element is not in
 * STARTED state (lines 584/624: "invalid state" LOGE, return -EPROTO).
 *
 * After stop() returns the element is in STOPPED state (asyncElementDelete
 * is pending but not yet run — no loop pump between stop() and the asserts).
 * play() and pause() check mState first, find it != STARTED, and return
 * -EPROTO without touching any ALSA handle. */
static void testCxxAlsaSourcePlayPauseInvalidState()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	AlsaSource *rawSource =
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSource);

	/* Transition element to STOPPED; asyncElementDelete is scheduled as
	 * idle but not yet dispatched. */
	int stopRet = rawSource->stop();
	CU_ASSERT_EQUAL(stopRet, 0);

	/* Both play() and pause() check mState != STARTED first. */
	CU_ASSERT_EQUAL(rawSource->play(), -EPROTO);
	CU_ASSERT_EQUAL(rawSource->pause(), -EPROTO);

	/* Pump to let the idle element deletion run before session stop. */
	(void)loop.pumpUntil([]() { return false; }, 200);

	int sessionStopRet = session->stop();
	CU_ASSERT_EQUAL(sessionStopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	sourceOwner.reset();
}


/* Covers three previously-uncovered paths with a single loopback run:
 *
 *  1. pdraw_alsa_source.cpp lines 1195-1196: "non-interleaved format is
 *     unsupported" (-ENOSYS). readFrame() only reaches that branch after
 *     snd_pcm_avail() returns >= sample_count AND the pool-get/mem-get
 *     steps succeed, so real captured data is required.  Flipping
 *     mParams.audio.format.pcm.interleaved to false while capture is
 *     running makes the very next onTimer() tick take that branch.
 *
 *  2. pdraw_alsa_source.cpp lines 1270-1285: unrecoverable_error: label.
 *     readFrame() returns -ENOSYS (not -EAGAIN); onTimer()'s while-loop
 *     takes its err<0 branch and jumps to unrecoverable_error:, setting
 *     mReady=false, clearing the timer, and calling readyToPlay(false,
 *     UNRECOVERABLE_ERROR).
 *
 *  3. pdraw_alsa_source.cpp line 590: play() when !mReady ("not ready to
 *     play", -EPROTO).  After unrecoverable_error: mState is still STARTED
 *     but mReady=false, so the second guard in play() fires. */
static void testCxxAlsaSourceNonInterleavedUnrecoverableError()
{
	if (!alsaLoopbackCaptureAvailable()) {
		CU_PASS("hw:Loopback device (snd-aloop) not available -- "
			"skipping non-interleaved unrecoverable-error test");
		return;
	}

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr,
			kAlsaLoopbackCaptureDevice);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Wait for at least one frame: confirms readFrame() can reach the
	 * snd_pcm_readi guard, i.e. snd_pcm_avail() returns >= sample_count
	 * and pool/mem allocations succeed. */
	bool gotFrame = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mFrameReadyCount >= 1;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	AlsaSource *rawSource =
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSource);

	/* Flip to non-interleaved: next readFrame() → -ENOSYS →
	 * unrecoverable_error: */
	rawSource->mParams.audio.format.pcm.interleaved = false;

	bool gotError = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mReadyToPlayCallCount >= 2;
		},
		5000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_FALSE(sourceListener.mReady);
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_ALSA_SOURCE_EOS_REASON_UNRECOVERABLE_ERROR);

	/* mState is still STARTED but mReady=false → play() hits line 590. */
	CU_ASSERT_EQUAL(rawSource->play(), -EPROTO);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	sourceOwner.reset();
}


/* Session::createAlsaSource() enforces params->address != nullptr at the
 * top of its validation block (before any element is created), so passing
 * nullptr returns -EINVAL synchronously without touching ALSA at all.
 * Covers that early-return guard path. */
static void testCxxAlsaSourceStartNullAddress()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;

	struct pdraw_alsa_source_params params = {};
	params.address = nullptr;
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	int ret = session->createAlsaSource(&params, &sourceListener, &source);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_PTR_NULL(source);
	CU_ASSERT_FALSE(sourceListener.mGotReadyToPlay);

	/* No element was inserted into the session (rejection was before
	 * element creation); stop() is still called for a clean teardown. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Covers AlsaSource::start()'s snd_pcm_open() failure path
 * (pdraw_alsa_source.cpp ~line 163): a device name that snd_pcm_open()
 * cannot resolve causes start() to fail and reach the error: label, which
 * calls stop() and returns the ALSA error.  Session::createAlsaSource()
 * calls start() synchronously and propagates the error directly, so the
 * caller sees a non-zero return without the wrapper ever being set. */
static void testCxxAlsaSourceStartBadAddress()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;

	struct pdraw_alsa_source_params params = {};
	params.address = "pdraw_test_nonexistent_device";
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	int ret = session->createAlsaSource(&params, &sourceListener, &source);
	CU_ASSERT_NOT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NULL(source);
	CU_ASSERT_FALSE(sourceListener.mGotReadyToPlay);

	/* The element was pushed to mElements before start() was called;
	 * start() itself called stop() which schedules async deletion.  Pump
	 * once to let the idle deletion callback complete, then stop the
	 * session for a clean teardown. */
	(void)loop.pumpUntil([]() { return false; }, 200);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Covers state-machine guard lines and the mOutputMediaChanging media-change
 * cycle using direct white-box access to AlsaSource methods.
 *
 * Phase 1 (STARTED state, null device):
 *   play() when mRunning=true        → line 595
 *   start() when STARTED             → line 124
 *   onTimer() when !mRunning         → lines 1257, 1258
 *   tryStop() when not STOPPING      → line 320
 *   setupMedia() with media (1st)    → lines 930, 937
 *   setupMedia() while changing (2nd)→ lines 923, 925, 926
 *   createMedia() when EALREADY      → lines 991–993
 *   destroyMedia() (success)         → —
 *   destroyMedia() (null)            → lines 1060–1062
 *   pump completeFlush cycle         → lines 522–526
 *
 * Phase 2 (STOPPING state):
 *   stop() transitions to STOPPING
 *   onTimer() in STOPPING            → lines 1247, 1250
 *   start()                          → lines 126, 129
 *   setupMedia()                     → lines 914, 917
 *   createMedia()                    → lines 982, 985
 *   processFrame(nullptr, 0)         → lines 769, 772
 *   pump to STOPPED
 *
 * Phase 3 (STOPPED state):
 *   setupMedia()                     → line 911
 */
static void testCxxAlsaSourceStateGuardsAndMediaCycle()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	AlsaSourceTrackingListener sourceListener;
	IPdraw::IAlsaSource *source = nullptr;
	setupAlsaSource(loop,
			session,
			mediaListener,
			sourceListener,
			nullptr,
			&source,
			nullptr);
	auto sourceOwner = std::unique_ptr<IPdraw::IAlsaSource>(source);

	AlsaSource *rawSource =
		static_cast<AlsaSourceWrapper *>(source)->getAlsaSource();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSource);

	/* ── Phase 1: STARTED state ─────────────────────────────────────── */

	/* onTimer() when mRunning == false → lines 1257, 1258 */
	rawSource->onTimer();

	/* play() when mRunning == true → line 595 */
	int ret = source->play();
	CU_ASSERT_EQUAL(ret, 0);
	ret = rawSource->play(); /* mRunning already true → line 595 */
	CU_ASSERT_EQUAL(ret, 0);

	/* start() when STARTED → line 124 */
	ret = rawSource->start();
	CU_ASSERT_EQUAL(ret, 0);

	/* tryStop() when not STOPPING → line 320 */
	ret = rawSource->tryStop();
	CU_ASSERT_EQUAL(ret, 0);

	/* setupMedia() when mOutputMedia != nullptr → lines 930, 937 */
	ret = rawSource->setupMedia();
	CU_ASSERT_EQUAL(ret, 0);

	/* setupMedia() again while mOutputMediaChanging == true → lines 923,
	 * 925, 926 */
	ret = rawSource->setupMedia();
	CU_ASSERT_EQUAL(ret, 0);

	/* createMedia() when mOutputMedia already exists → lines 991–993 */
	ret = rawSource->createMedia();
	CU_ASSERT_EQUAL(ret, -EALREADY);

	/* Pump the loop to let the completeFlush idle (queued by the first
	 * setupMedia() call above) run the media-change cycle:
	 * teardownChannels → destroyMedia → setupMedia → createMedia.
	 * Lines 522–526 are covered inside this completeFlush invocation.
	 * We pump BEFORE calling destroyMedia() so that teardownChannels()
	 * is called while mOutputMedia is still valid (not nullptr). */
	bool cycleCompleted = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 2; },
		5000);
	CU_ASSERT_TRUE(cycleCompleted);

	/* destroyMedia() first call: removes the freshly-recreated media */
	ret = rawSource->destroyMedia();
	CU_ASSERT_EQUAL(ret, 0);

	/* destroyMedia() second call when mOutputMedia == nullptr → lines
	 * 1060–1062 */
	ret = rawSource->destroyMedia();
	CU_ASSERT_EQUAL(ret, 0);

	/* ── Phase 2: STOPPING state ────────────────────────────────────── */

	ret = rawSource->stop();
	CU_ASSERT_EQUAL(ret, 0);

	/* onTimer() in STOPPING → lines 1247, 1250 */
	rawSource->onTimer();

	/* start() in STOPPING → lines 126, 129 */
	ret = rawSource->start();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* setupMedia() in STOPPING → lines 914, 917 */
	ret = rawSource->setupMedia();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* createMedia() in STOPPING → lines 982, 985 */
	ret = rawSource->createMedia();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* processFrame(nullptr, 0) in STOPPING → lines 769, 772 */
	ret = rawSource->processFrame(nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* Pump to STOPPED (completeFlush idle fires → completeStop) */
	bool gotStopped = loop.pumpUntil(
		[rawSource]() {
			return rawSource->getState() == Element::State::STOPPED;
		},
		5000);
	CU_ASSERT_TRUE(gotStopped);

	/* ── Phase 3: STOPPED state ─────────────────────────────────────── */

	/* setupMedia() in STOPPED → line 911 */
	ret = rawSource->setupMedia();
	CU_ASSERT_EQUAL(ret, 0);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	sourceOwner.reset();
}


/* Constructs AlsaSource directly (bypassing Session::createAlsaSource()
 * which guards params->address != nullptr at line 861), exercising:
 *
 *   AlsaSource ctor with address==nullptr → line 80
 *   stop() in CREATED state           → lines 267, 270 (-EPROTO)
 */
static void testCxxAlsaSourceStopInCreatedState()
{
	TestPompLoop loop;
	MediaTrackingListener ml;
	TestSession ts(&loop, &ml);
	Session *rawSession = ts.get();

	AlsaSourceTrackingListener srcListener;
	struct pdraw_alsa_source_params params = {};
	params.address =
		nullptr; /* line 80: else branch, mParams.address = nullptr */
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	std::unique_ptr<AlsaSource> directSrc(
		new AlsaSource(rawSession,
			       nullptr, /* elementListener */
			       nullptr, /* sourceListener */
			       &srcListener,
			       nullptr, /* wrapper */
			       &params));

	/* CREATED state; stop() in CREATED triggers lines 267, 270 */
	int ret = directSrc->stop();
	CU_ASSERT_EQUAL(ret, -EPROTO);
}

#endif /* PDRAW_TEST_ALSA_SOURCE_ENABLED */


CU_TestInfo g_pdraw_test_pipeline_alsa_source[] = {
#ifdef PDRAW_TEST_ALSA_SOURCE_ENABLED
	{FN("testCxxAlsaSourceCreationReadyToPlay"),
	 testCxxAlsaSourceCreationReadyToPlay},
	{FN("testCxxAlsaSourcePlayChangesState"),
	 testCxxAlsaSourcePlayChangesState},
	{FN("testCxxAlsaSourcePauseDrainsAndCallsPauseResponse"),
	 testCxxAlsaSourcePauseDrainsAndCallsPauseResponse},
	{FN("testCxxAlsaSourceCapturedFrameFlushesOnStop"),
	 testCxxAlsaSourceCapturedFrameFlushesOnStop},
	{FN("testCxxAlsaSourceCapturedFrameDrainsOnPause"),
	 testCxxAlsaSourceCapturedFrameDrainsOnPause},
	{FN("testCxxAlsaSourceCapturedFrameRecoversFromOverrun"),
	 testCxxAlsaSourceCapturedFrameRecoversFromOverrun},
	{FN("testCxxAlsaSourceGetCapabilitiesNullDevice"),
	 testCxxAlsaSourceGetCapabilitiesNullDevice},
	{FN("testCAlsaSourceListenerCbsWithLoopback"),
	 testCAlsaSourceListenerCbsWithLoopback},
	{FN("testCxxAlsaSourceWrapperGuardsAfterElementCleared"),
	 testCxxAlsaSourceWrapperGuardsAfterElementCleared},
	{FN("testCxxAlsaSourcePlayPauseInvalidState"),
	 testCxxAlsaSourcePlayPauseInvalidState},
	{FN("testCxxAlsaSourceNonInterleavedUnrecoverableError"),
	 testCxxAlsaSourceNonInterleavedUnrecoverableError},
	{FN("testCxxAlsaSourceStartNullAddress"),
	 testCxxAlsaSourceStartNullAddress},
	{FN("testCxxAlsaSourceStartBadAddress"),
	 testCxxAlsaSourceStartBadAddress},
	{FN("testCxxAlsaSourceStateGuardsAndMediaCycle"),
	 testCxxAlsaSourceStateGuardsAndMediaCycle},
	{FN("testCxxAlsaSourceStopInCreatedState"),
	 testCxxAlsaSourceStopInCreatedState},
#endif
	CU_TEST_INFO_NULL,
};
