/**
 * Parrot Drones Audio and Video Vector library
 * ALSA audio renderer
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
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

#pragma once

#include "pdraw_session.hpp"

#ifdef PDRAW_USE_ALSA

#	include <alsa/asoundlib.h>
#	include <atomic>

#	include <audio-defs/adefs.h>

#	include "pdraw_alsa_audio.hpp"
#	include "pdraw_renderer_audio.hpp"


namespace Pdraw {

class AlsaAudioRenderer : public AudioRenderer {
public:
	AlsaAudioRenderer(Session *session,
			  Element::Listener *listener,
			  AudioRendererWrapper *wrapper,
			  IPdraw::IAudioRenderer::Listener *rndListener,
			  uint32_t mediaTypeCaps,
			  unsigned int mediaId,
			  const struct pdraw_audio_renderer_params *params);

	~AlsaAudioRenderer() override;

	int start() override;

	int stop() override;

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId() const override;

	int
	setParams(const struct pdraw_audio_renderer_params *params) override;

	int getParams(struct pdraw_audio_renderer_params *params) override;

	int addInputMedia(Media *media) override;

	int removeInputMedia(Media *media) override;

	int removeInputMedias() override;

	void completeStop() override;

private:
	int startAlsa();

	int stopAlsa();

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelSos(Channel *channel) override;

	void onChannelEos(Channel *channel) override;

	static void idleStart(void *renderer);

	static void idleDrain(void *renderer);

	int render();

	static void renderCb(struct pomp_evt *event, void *userdata);

	static void watchdogTimerCb(struct pomp_timer *timer, void *userdata);

	static bool queueFilter(struct mbuf_audio_frame *frame, void *userdata);

	mbuf::Queue *getLastAddedMediaQueue();

	static void idleRenewMedia(void *userdata);

	unsigned int mMediaId = 0;
	unsigned int mCurrentMediaId = 0;
	bool mRunning = false;
	AudioMedia *mLastAddedMedia = nullptr;
	struct pdraw_media_info mMediaInfo {
	};
	bool mAlsaReady = false;
	struct pdraw_audio_renderer_params mParams {
	};
	std::string mAddress{};
	snd_pcm_t *mHandle = nullptr;
	snd_pcm_hw_params_t *mHwParams = nullptr;
	snd_pcm_sw_params_t *mSwParams = nullptr;
	size_t mFrameSize = 0;
	size_t mSampleCount = ALSA_AUDIO_DEFAULT_SAMPLE_COUNT;

	/* Watchdog timer: triggered if no new frame is received for a given
	 * amount of time */
	struct pomp_timer *mWatchdogTimer = nullptr;
	std::atomic_bool mWatchdogTriggered{false};
	std::atomic_bool mEos{false};
};

} /* namespace Pdraw */

#endif /* PDRAW_USE_ALSA */
