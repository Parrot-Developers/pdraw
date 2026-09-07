/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — audio encoder on a real decoded media, and on a
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

/* Split out of test_pipeline_decode.cpp (see test_pipeline_decoder_audio.cpp's
 * header comment for the full split history): this file holds every
 * IAudioEncoder test. The shared demuxer-fixture
 * helpers (PipelineDemuxerListener, openDecodingDemuxerAndPlay, etc.) are
 * now in test_pipeline_common.hpp; local listener classes
 * (MediaTrackingListener, DrainTrackingAudioSourceListener,
 * AckingAudioSinkListener) remain per-translation-unit.
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the decoder's/
 * encoder's output media requires a real session-wide listener, so each
 * test below builds its own private TestPompLoop + TestSession, fully
 * self-contained. No suite init/cleanup is registered for this file (see
 * test_main.c: NULL, NULL). */

#define ULOG_TAG pdraw_test_pipeline_encoder_audio

#include <complex>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "pdraw_element.hpp"
#include "pdraw_encoder_audio.hpp"
#undef protected
#undef private

#include "pdraw_external_audio_sink.hpp"
#include "pdraw_external_audio_source.hpp"
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

/* struct aenc_config is already visible transitively via pdraw_defs.h
 * (included from test_api_common.hpp). */

#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

#include <string.h>

#include <atomic>
#include <mutex>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── M4A fixture (same NAS asset as test_api_demuxer.cpp) ────────────────── */

enum { ASSET_AUDIO_AAC = 0 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_encoder_audio[] = {
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
	 * silently return the demuxer's *coded* media instead. Confirmed the
	 * hard way: an earlier version of this test used exactly that
	 * encoding-blind lookup, feeding coded AAC-LC audio straight into
	 * createAudioEncoder() (nonsensical -- encoders take raw input) and
	 * only getting away with it because that also happened to return
	 * -ENOSYS in an environment with no encoder backend either way,
	 * masking the real bug. */
	const Added *findRawAudioMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_AUDIO &&
			    a.audioFormat.encoding == ADEF_ENCODING_PCM)
				return &a;
		}
		return nullptr;
	}

	/* First AAC-LC (coded) audio media added, or nullptr if none yet --
	 * e.g. an AudioEncoder's own (lazily created) output media. */
	const Added *findCodedAudioMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_AUDIO &&
			    a.audioFormat.encoding == ADEF_ENCODING_AAC_LC)
				return &a;
		}
		return nullptr;
	}

	/* Overrides the no-op base implementation: needed to observe the
	 * encoder's own output media being torn down as a consequence of
	 * AudioEncoder::onChannelUnlink -> completeStop() actually running
	 * (see testCxxAudioEncoderUnlinkWhenSourceStops below). */
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


/* Same rationale as VideoEncoderOutputListener in
 * test_pipeline_encoder_video.cpp, for IAudioEncoder. */
class AudioEncoderOutputListener : public IPdraw::IAudioEncoder::Listener {
public:
	void audioEncoderFrameOutput(IPdraw * /*p*/,
				     IPdraw::IAudioEncoder * /*e*/,
				     struct mbuf_audio_frame *frame) override
	{
		struct adef_frame info = {};
		int ret = mbuf_audio_frame_get_frame_info(frame, &info);
		if (ret < 0)
			return;
		const void *data = nullptr;
		size_t len = 0;
		ret = mbuf_audio_frame_get_buffer(frame, &data, &len);
		if (ret == 0)
			mbuf_audio_frame_release_buffer(frame, data);
		{
			std::lock_guard<std::mutex> lock(mMutex);
			mEncoding = info.format.encoding;
			mBufferLen = (ret == 0) ? len : 0;
		}
		mGotFrame.store(true);
	}

	void
	audioEncoderFramePreRelease(IPdraw * /*p*/,
				    IPdraw::IAudioEncoder * /*e*/,
				    struct mbuf_audio_frame * /*f*/) override
	{
	}

	std::mutex mMutex;
	enum adef_encoding mEncoding = ADEF_ENCODING_UNKNOWN;
	size_t mBufferLen = 0;
	std::atomic<bool> mGotFrame{false};
};


/* Tracks IAudioSource::flush()/drain() completion -- same rationale as
 * DrainTrackingAudioSourceListener in test_pipeline_sourcesink_audio.cpp
 * (duplicated, not shared, same convention as the video listeners in
 * test_pipeline_encoder_video.cpp / test_pipeline_scaler_video.cpp). */
class DrainTrackingAudioSourceListener : public IPdraw::IAudioSource::Listener {
public:
	void onAudioSourceFlushed(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*src*/) override
	{
		mGotFlushed = true;
	}

	void onAudioSourceDrained(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*src*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* Implements the sink side of the flush/drain protocol for real, for an
 * audio sink -- same rationale as QueueDrainingAudioSinkListener in
 * test_pipeline_sourcesink_audio.cpp (duplicated, not shared, same
 * convention). IAudioSink is generic (no separate raw/coded interface
 * split, unlike video), so this same class would work whether attached to
 * a raw PCM or coded AAC-LC media -- here it's attached to the
 * AudioEncoder's coded AAC-LC output. */
class AckingAudioSinkListener : public IPdraw::IAudioSink::Listener {
public:
	void
	onAudioSinkMediaAdded(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/,
			      const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
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
	}

	/* Must be set right after createAudioSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_audio_frame_queue *mQueue = nullptr;
	bool mGotMediaAdded = false;

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


/* A minimal, correctly-sized silent PCM frame -- this is fed to a real
 * fdk-aac encoder, which reads actual sample data, so the buffer must be
 * genuinely sized for one AAC frame's worth of samples (1024 samples/
 * channel, same convention as collectWavFrames() in
 * test_pipeline_sourcesink_audio.cpp), even though the content itself
 * (silence) is irrelevant. */
static struct mbuf_audio_frame *
makeSilentPcmFrame(const struct adef_format &format,
		   uint64_t timestamp,
		   unsigned int index)
{
	struct adef_frame frameInfo = {};
	frameInfo.format = format;
	frameInfo.info.timescale = format.sample_rate;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	size_t frameLen = 1024 * format.channel_count * (format.bit_depth / 8);
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(frameLen, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);
	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	return frame;
}


/* Build a standalone ExternalAudioSource (raw PCM) -> AudioEncoder (AAC-LC)
 * -> (real) IAudioSink chain and push a priming burst so the encoder's
 * output media exists. sourceListener is passed in (rather than hardcoded
 * to a stub) because IAudioSource::Listener is fixed at creation time and
 * cannot be swapped afterward: a caller that needs to observe flush()/
 * drain() completion via onAudioSourceFlushed()/onAudioSourceDrained() must
 * supply its own tracking listener from the very start. */
static void
createAudioEncoderChainAndPrime(IPdraw *session,
				TestPompLoop *loop,
				MediaTrackingListener *mediaListener,
				IPdraw::IAudioSource::Listener *sourceListener,
				IPdraw::IAudioSource **source,
				IPdraw::IAudioEncoder **encoder,
				IPdraw::IAudioSink **sink,
				AckingAudioSinkListener *sinkListener,
				struct mbuf_audio_frame_queue **inQueue)
{
	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_mono;

	int ret = session->createAudioSource(
		&sourceParams, sourceListener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);

	bool gotRawMedia = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener->findRawAudioMedia()->id;

	struct aenc_config encoderParams = {};
	encoderParams.encoding = ADEF_ENCODING_AAC_LC;
	encoderParams.aac_lc.max_bitrate = 128000;
	/* Left unset (ADEF_AAC_DATA_FORMAT_UNKNOWN), aenc_fdk_aac.c's create()
	 * defaults this to ADTS (see aenc_fdk_aac.c:843-846) -- but
	 * AudioEncoder::createOutputMedia() in pdraw_encoder_audio.cpp
	 * unconditionally requires a fetchable ASC via aenc_get_aac_asc() for
	 * any AAC_LC output, and the fdk_aac backend only ever populates
	 * that ASC buffer when preferred_format == RAW (aenc_fdk_aac.c:
	 * 1066-1078; in ADTS mode it stays permanently null, so
	 * aenc_get_aac_asc() returns -EAGAIN forever, not just until some
	 * frame count -- confirmed the hard way). RAW must be requested
	 * explicitly. */
	encoderParams.output.preferred_format = ADEF_AAC_DATA_FORMAT_RAW;
	ret = session->createAudioEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_audio_encoder_listener,
					  encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*encoder);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);

	/* Prime with a small burst (same rationale as
	 * createEncoderChainAndPrime() in test_pipeline_encoder_video.cpp). */
	for (unsigned int i = 0; i < 4; i++) {
		struct mbuf_audio_frame *primer = makeSilentPcmFrame(
			adef_pcm_16b_44100hz_mono, i * 23219, i);
		ret = mbuf_audio_frame_queue_push(*inQueue, primer);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_audio_frame_unref(primer);
	}

	bool gotCodedMedia = loop->pumpUntil(
		[mediaListener]() {
			return mediaListener->findCodedAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotCodedMedia);
	unsigned int codedMediaId = mediaListener->findCodedAudioMedia()->id;

	ret = session->createAudioSink(codedMediaId, sinkListener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);
	sinkListener->mQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);

	/* Wait until the sink's input channel is actually linked to the
	 * encoder's output media before letting the caller flush()/drain()
	 * the chain. Confirmed the hard way via a real gcov run: without this
	 * wait, AudioEncoder::flush()/drain() -- called synchronously right
	 * after createAudioSink() returns, with no intervening pump -- sees
	 * getOutputChannelCount() == 0 (the link is established later, via an
	 * idle callback), so it never forwards to the output channel at all.
	 * completeFlush() then finds zero output channels pending and
	 * completes immediately, WITHOUT ever reaching the sink or coming
	 * back through AudioEncoder::onChannelFlushed/onChannelDrained --
	 * both were 0% covered because of exactly this race (only
	 * onChannelFlush/onChannelDrain, the *input*-side half of the
	 * cascade, ever ran). */
	bool gotMediaAdded = loop->pumpUntil(
		[sinkListener]() { return sinkListener->mGotMediaAdded; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
}


/* IAudioEncoder has no public flush()/drain() (confirmed earlier: unlike
 * IVideoEncoder it doesn't even have configure()/getConfig()/
 * requestKeyFrame() -- it is a pure filter element just like VideoEncoder/
 * VideoScaler): only reachable by flushing/draining a real Source feeding
 * it and observing the completion cascade back through
 * AudioEncoder::onChannelFlush/onChannelFlushed. */
static void testCxxAudioEncoderFlushCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioEncoder *encoder = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	AckingAudioSinkListener sinkListener;
	struct mbuf_audio_frame_queue *inQueue = nullptr;
	createAudioEncoderChainAndPrime(session,
					&loop,
					&mediaListener,
					&sourceListener,
					&source,
					&encoder,
					&sink,
					&sinkListener,
					&inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
	(void)inQueue;

	int ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	/* stopSessionAndWait() must run BEFORE the owners are reset, so that
	 * Session::asyncElementDelete() destroys the AudioSource/AudioEncoder/
	 * AudioSink elements (and thus runs their wrappers' clearElement()
	 * overrides) while the wrappers are still alive -- resetting first
	 * would run ~ElementWrapper() instead and the overrides would never
	 * execute. */
	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(
		static_cast<AudioSourceWrapper *>(source)->getAudioSource());
	CU_ASSERT_PTR_NULL(
		static_cast<AudioEncoderWrapper *>(encoder)->getAudioEncoder());
	CU_ASSERT_PTR_NULL(
		static_cast<AudioSinkWrapper *>(sink)->getAudioSink());

	sinkOwner.reset();
	encoderOwner.reset();
	sourceOwner.reset();
}


/* Same rationale as testCxxAudioEncoderFlushCascadesThroughSourceAndSink
 * above, for drain() instead of flush(). */
static void testCxxAudioEncoderDrainCascadesThroughSourceAndSink()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioEncoder *encoder = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	AckingAudioSinkListener sinkListener;
	struct mbuf_audio_frame_queue *inQueue = nullptr;
	createAudioEncoderChainAndPrime(session,
					&loop,
					&mediaListener,
					&sourceListener,
					&source,
					&encoder,
					&sink,
					&sinkListener,
					&inQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
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


/* AudioEncoder::onChannelUnlink was entirely uncovered (0% per a real gcov
 * run): every other test in this file destroys the encoder itself before
 * (or without ever) destroying the source, so the encoder's own INPUT
 * channel teardown/unlink never has a chance to cascade back through its
 * OUTPUT side. onChannelUnlink is a Source-side callback, dispatched via
 * Channel::unlink() (pdraw_channel.cpp) sent BY a sink WHEN it removes its
 * own input media as part of its own stop() -- see Sink::removeInputMedia()
 * in pdraw_sink.cpp. Reversing the order here -- stopping the *source*
 * first, while the encoder and sink are still alive -- lets
 * ExternalAudioSource::tryStop() tear down its own output channel (to the
 * encoder), cascading: AudioEncoder::onChannelTeardown -> stop() (sets
 * STOPPING) -> the encoder's own output channel torn down in turn -> the
 * sink stops and calls removeInputMedia() -> channel->unlink() ->
 * AudioEncoder::onChannelUnlink, which (since mState is now STOPPING) calls
 * completeStop() -> removes the encoder's own coded output media. That
 * removal (onMediaRemoved) is the observable proof that onChannelUnlink ran
 * -- same idiom as testCxxVideoScalerTeardownWhenSourceStops in
 * test_pipeline_scaler_video.cpp. */
static void testCxxAudioEncoderUnlinkWhenSourceStops()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioEncoder *encoder = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	AckingAudioSinkListener sinkListener;
	struct mbuf_audio_frame_queue *inQueue = nullptr;
	createAudioEncoderChainAndPrime(session,
					&loop,
					&mediaListener,
					&sourceListener,
					&source,
					&encoder,
					&sink,
					&sinkListener,
					&inQueue);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);
	(void)inQueue;

	const MediaTrackingListener::Added *codedMediaPtr =
		mediaListener.findCodedAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedMediaPtr);
	unsigned int codedMediaId = codedMediaPtr->id;

	/* Destroy the source only, while encoder+sink are still alive -- see
	 * the comment above for why the order matters here. */
	{
		auto sourceOwner =
			std::unique_ptr<IPdraw::IAudioSource>(source);
	}

	bool gotRemoved = loop.pumpUntil(
		[&]() { return mediaListener.wasRemoved(codedMediaId); },
		15000);
	CU_ASSERT_TRUE_FATAL(gotRemoved);

	sinkOwner.reset();
	encoderOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* An entirely empty/all-zero aenc_config (in particular .encoding left at
 * ADEF_ENCODING_UNKNOWN) was never exercised: every other test in this file
 * always sets .encoding explicitly. Read aenc_new()/create() for both
 * concrete backends in packages/libaudio-encode (fdk-aac, fakeaac): each
 * one's create() rejects any encoding that isn't ADEF_ENCODING_AAC_LC, the
 * only one either supports (e.g. aenc_fdk_aac.c:817: "if (base->config.
 * encoding != ADEF_ENCODING_AAC_LC) ... return -EINVAL") -- same pattern as
 * VideoEncoder's equivalent gap (see
 * testCxxCreateVideoEncoderWithEmptyConfigFails in
 * test_pipeline_encoder_video.cpp). ADEF_ENCODING_UNKNOWN can never match
 * either backend's specific check, so createAudioEncoder() with a fully
 * empty config is expected to fail predictably, regardless of which
 * concrete AAC backend happens to be compiled into this product. */
static void testCxxCreateAudioEncoderWithEmptyConfigFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_mono;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawAudioMedia()->id;

	struct aenc_config encoderParams = {};
	IPdraw::IAudioEncoder *encoder = nullptr;
	ret = session->createAudioEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_audio_encoder_listener,
					  &encoder);
	CU_ASSERT_NOT_EQUAL(ret, 0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Variant of testCxxCreateVideoEncoderWithInvalidEncodingFails
 * (test_pipeline_encoder_video.cpp), for the audio encoder. As there,
 * .implem is forced explicitly to AENC_ENCODER_IMPLEM_FDK_AAC, bypassing
 * AudioEncoder::start()'s AUTO-implem-by-encoding pre-check
 * (pdraw_encoder_audio.cpp:193-208, only run when .implem ==
 * AENC_ENCODER_IMPLEM_AUTO -- with only .encoding set and .implem left AUTO,
 * this pre-check would return -ENOENT before aenc_new() is ever called),
 * while .encoding is set to a recognized but wrong value, ADEF_ENCODING_PCM
 * (the raw/uncompressed encoding -- never a valid *encoder* output), so that
 * aenc_new() is genuinely reached.
 *
 * Unlike the video case, though, aenc_new() (libaudio-encode/src/aenc.c) has
 * no per-backend dispatch for the encoding value: unlike venc_new()'s switch
 * on config.encoding (one case per real encoding -- H264/H265/JPEG/PNG --
 * deferring the actual encoding-vs-implem match to each backend's own
 * create()), aenc_new() enforces a single blanket check directly in its own
 * generic code, common to every backend (aenc.c:420: "if (self->config.
 * encoding != ADEF_ENCODING_AAC_LC) { res = -EINVAL; goto error; }"),
 * BEFORE ever calling self->ops->create() (aenc.c:437). Both compiled
 * backends (fdk-aac, fakeaac) only ever implement AAC_LC anyway, so there is
 * no "recognized elsewhere, wrong here" encoding able to reach a
 * backend-specific create() rejection the way JPEG-vs-x264 does for video:
 * any non-AAC_LC encoding is already rejected by this blanket check,
 * regardless of .implem. This still genuinely exercises aenc_new() failing
 * (as opposed to pdraw's own -ENOENT pre-check above it), just via aenc.c's
 * shared code path instead of a backend's create(). */
static void testCxxCreateAudioEncoderWithInvalidEncodingFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_mono;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawAudioMedia()->id;

	struct aenc_config encoderParams = {};
	encoderParams.implem = AENC_ENCODER_IMPLEM_FDK_AAC;
	encoderParams.encoding = ADEF_ENCODING_PCM;
	IPdraw::IAudioEncoder *encoder = nullptr;
	ret = session->createAudioEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_audio_encoder_listener,
					  &encoder);
	CU_ASSERT_NOT_EQUAL(ret, 0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* AudioEncoder::AudioEncoder()'s .name/.device copy branches
 * (pdraw_encoder_audio.cpp:102-107: "if (mEncoderConfig->name != nullptr)
 * mEncoderName = std::string(mEncoderConfig->name);" and the analogous
 * .device -> mEncoderDevice one), same gap as
 * testCxxCreateVideoEncoderWithNameAndDeviceSucceeds in
 * test_pipeline_encoder_video.cpp: every other test in this file leaves
 * both fields at their zero-init nullptr. aenc_config_copy() (called from
 * the constructor at pdraw_encoder_audio.cpp:97) always runs regardless of
 * .encoding/.implem, so adding .name/.device on top of an otherwise normal,
 * successful AAC-LC config reaches both branches while still letting
 * createAudioEncoder() succeed end-to-end. */
static void testCxxCreateAudioEncoderWithNameAndDeviceSucceeds()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_mono;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawAudioMedia()->id;

	struct aenc_config encoderParams = {};
	encoderParams.name = "test-audio-encoder";
	encoderParams.device = "/dev/test-audio-encoder0";
	encoderParams.encoding = ADEF_ENCODING_AAC_LC;
	encoderParams.aac_lc.max_bitrate = 128000;
	IPdraw::IAudioEncoder *encoder = nullptr;
	ret = session->createAudioEncoder(rawMediaId,
					  &encoderParams,
					  &g_stub_audio_encoder_listener,
					  &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);

	encoderOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* AudioEncoder::start()'s else branch (pdraw_encoder_audio.cpp:210-221) runs
 * when the AudioEncoder was constructed with params==nullptr: it allocates a
 * default aenc_config (AUTO implem, ADEF_ENCODING_AAC_LC, input = media).
 * Session::createAudioEncoder() guards against null params with -EINVAL, so
 * the else branch is only reachable via direct AudioEncoder construction —
 * exactly as addAudioEncoderForMedia does with encoder==nullptr internally. */
static void testCxxAudioEncoderNullConfigDefaultsToAacLc()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioMedia media(session);
	media.format = adef_pcm_16b_44100hz_mono;

	AudioEncoder encoder(session,
			     session, /* Element::Listener */
			     session, /* Source::Listener */
			     &g_stub_audio_encoder_listener,
			     nullptr, /* wrapper */
			     nullptr /* params — null triggers else branch */);

	int ret = encoder.addInputMedia(&media);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

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
	loop.runOnce(); /* drain asyncElementDelete idle */
}


static void testCxxCreateAudioEncoderWithRealDecodedMedia()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_AUDIO_AAC, s_assets_pipeline_encoder_audio);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

	/* Wait for the internally auto-created AudioDecoder to output its
	 * first decoded frame and register its output media. */
	bool gotAudio = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAudio);
	const MediaTrackingListener::Added *audio =
		mediaListener.findRawAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(audio);
	CU_ASSERT_NOT_EQUAL_FATAL(audio->id, 0u);

	/* Input sub-structure ignored (derived from the connected media);
	 * only the encoding type and max_bitrate (default rate control is
	 * CBR) are mandatory. */
	struct aenc_config encoderParams = {};
	encoderParams.encoding = ADEF_ENCODING_AAC_LC;
	encoderParams.aac_lc.max_bitrate = 128000;
	IPdraw::IAudioEncoder *encoder = nullptr;
	int ret = session->createAudioEncoder(audio->id,
					      &encoderParams,
					      &g_stub_audio_encoder_listener,
					      &encoder);
	/* CONFIG_AENC_FDK_AAC (or another concrete AAC-LC encoder backend) is
	 * required for this build -- same reasoning as the video encoder in
	 * test_pipeline_encoder_video.cpp. */
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Same rationale as testCxxVideoEncoderProducesRealH264Output in
 * test_pipeline_encoder_video.cpp, for the audio encoder. */
static void testCxxAudioEncoderProducesRealAacOutput()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	PipelineDemuxerListener demuxListener;
	char demuxerPath[512];
	PDRAW_GET_ASSET_PATH(
		demuxerPath, ASSET_AUDIO_AAC, s_assets_pipeline_encoder_audio);
	IPdraw::IDemuxer *demuxer = openDecodingDemuxerAndPlay(
		session, &loop, &demuxListener, demuxerPath);

	bool gotAudio = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawAudioMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAudio);
	const MediaTrackingListener::Added *audioPtr =
		mediaListener.findRawAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(audioPtr);
	MediaTrackingListener::Added audio = *audioPtr;

	struct aenc_config encoderParams = {};
	encoderParams.encoding = ADEF_ENCODING_AAC_LC;
	encoderParams.aac_lc.max_bitrate = 128000;
	/* Mandatory for the output media to ever be created successfully --
	 * see the comment in createAudioEncoderChainAndPrime() earlier in
	 * this file for the full explanation (aenc_fdk_aac.c defaults to
	 * ADTS when unset, and AudioEncoder::createOutputMedia() can only
	 * ever fetch an ASC in RAW mode). Without this, gotFrame below would
	 * never become true. */
	encoderParams.output.preferred_format = ADEF_AAC_DATA_FORMAT_RAW;
	AudioEncoderOutputListener encoderListener;
	IPdraw::IAudioEncoder *encoder = nullptr;
	int ret = session->createAudioEncoder(
		audio.id, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IAudioEncoder>(encoder);

	bool gotFrame = loop.pumpUntil(
		[&encoderListener]() {
			return encoderListener.mGotFrame.load();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	{
		std::lock_guard<std::mutex> lock(encoderListener.mMutex);
		CU_ASSERT_EQUAL(encoderListener.mEncoding,
				ADEF_ENCODING_AAC_LC);
		CU_ASSERT_FATAL(encoderListener.mBufferLen > 0);
	}

	encoderOwner.reset();
	closeAndDestroyDemuxer(demuxer, &loop, &demuxListener);
	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


static void testCxxAudioEncoderStartWithoutInputMediaFails()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *ipdraw = testSession.get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ipdraw);
	Session *session = static_cast<Session *>(ipdraw);

	auto encoder = std::unique_ptr<AudioEncoder>(new AudioEncoder(
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


static void testCxxAudioEncoderStartUnsupportedEncodingFails()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioMedia media(session);
	media.format = adef_pcm_16b_44100hz_mono;

	struct aenc_config params = {};
	params.implem = AENC_ENCODER_IMPLEM_AUTO;
	params.encoding = static_cast<enum adef_encoding>(999);

	AudioEncoder encoder(
		session, session, session, nullptr, nullptr, &params);
	int ret = encoder.addInputMedia(&media);
	CU_ASSERT_EQUAL(ret, 0);

	ret = encoder.start();
	CU_ASSERT_EQUAL(ret, -ENOENT);
}


static void testCxxAudioEncoderFrameOutputCbCoverage()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	Session *session = testSession.get();

	AudioEncoder encoder(
		session, session, session, nullptr, nullptr, nullptr);

	/* 1. status != 0 -> error logging */
	AudioEncoder::frameOutputCb(nullptr, -EINVAL, nullptr, &encoder);

	/* 2. userdata == nullptr */
	AudioEncoder::frameOutputCb(nullptr, 0, nullptr, nullptr);

	/* 3. out_frame == nullptr */
	AudioEncoder::frameOutputCb(nullptr, 0, nullptr, &encoder);

	/* 4. state != STARTED */
	struct mbuf_audio_frame *frame = nullptr;
	struct adef_frame frameInfo = {};
	frameInfo.format = adef_pcm_16b_44100hz_mono;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* 4a. state != STARTED */
	CU_ASSERT_NOT_EQUAL(encoder.getState(), Element::State::STARTED);
	AudioEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4b. state == STARTED with flush pending -> discards frame */
	encoder.setState(Element::State::STARTED);
	encoder.setFlushingState(Element::FlushingState::FLUSHING, true);
	encoder.mAencFlushPending = true;
	CU_ASSERT_TRUE(encoder.mAencFlushPending);
	AudioEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4c. state == STARTED with mInputMedia == nullptr -> invalid input
	 * media */
	encoder.mAencFlushPending = false;
	CU_ASSERT_PTR_NULL(encoder.mInputMedia);
	AudioEncoder::frameOutputCb(nullptr, 0, frame, &encoder);

	/* 4d. state == STARTED with mInputMedia set, but frame has no ancillary
	 * data */
	AudioMedia media(session);
	encoder.mInputMedia = &media;
	CU_ASSERT_PTR_NOT_NULL(encoder.mInputMedia);
	AudioEncoder::frameOutputCb(nullptr, 0, frame, &encoder);
	encoder.mInputMedia = nullptr;
	encoder.setState(Element::State::STOPPED);
	CU_ASSERT_EQUAL(encoder.getState(), Element::State::STOPPED);

	mbuf_audio_frame_unref(frame);

	/* 5. nullptr guards for callbacks and channel events */
	AudioEncoder::flushCb(nullptr, nullptr);
	AudioEncoder::stopCb(nullptr, nullptr);

	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);
	encoder.onChannelFlush(nullptr);
	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);

	encoder.onChannelDrain(nullptr);
	CU_ASSERT_FALSE(encoder.mInputChannelFlushPending);

	encoder.mAencFlushPending = true;
	AudioEncoder::flushCb(nullptr, &encoder);
	CU_ASSERT_FALSE(encoder.mAencFlushPending);
}


CU_TestInfo g_pdraw_test_pipeline_encoder_audio[] = {
	{FN("testCxxCreateAudioEncoderWithRealDecodedMedia"),
	 testCxxCreateAudioEncoderWithRealDecodedMedia},
	{FN("testCxxAudioEncoderProducesRealAacOutput"),
	 testCxxAudioEncoderProducesRealAacOutput},
	{FN("testCxxAudioEncoderFlushCascadesThroughSourceAndSink"),
	 testCxxAudioEncoderFlushCascadesThroughSourceAndSink},
	{FN("testCxxAudioEncoderDrainCascadesThroughSourceAndSink"),
	 testCxxAudioEncoderDrainCascadesThroughSourceAndSink},
	{FN("testCxxAudioEncoderUnlinkWhenSourceStops"),
	 testCxxAudioEncoderUnlinkWhenSourceStops},
	{FN("testCxxCreateAudioEncoderWithEmptyConfigFails"),
	 testCxxCreateAudioEncoderWithEmptyConfigFails},
	{FN("testCxxCreateAudioEncoderWithInvalidEncodingFails"),
	 testCxxCreateAudioEncoderWithInvalidEncodingFails},
	{FN("testCxxCreateAudioEncoderWithNameAndDeviceSucceeds"),
	 testCxxCreateAudioEncoderWithNameAndDeviceSucceeds},
	{FN("testCxxAudioEncoderNullConfigDefaultsToAacLc"),
	 testCxxAudioEncoderNullConfigDefaultsToAacLc},
	{FN("testCxxAudioEncoderStartWithoutInputMediaFails"),
	 testCxxAudioEncoderStartWithoutInputMediaFails},
	{FN("testCxxAudioEncoderStartUnsupportedEncodingFails"),
	 testCxxAudioEncoderStartUnsupportedEncodingFails},
	{FN("testCxxAudioEncoderFrameOutputCbCoverage"),
	 testCxxAudioEncoderFrameOutputCbCoverage},
	CU_TEST_INFO_NULL,
};
