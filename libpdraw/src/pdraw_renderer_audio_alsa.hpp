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

#ifndef _PDRAW_RENDERER_AUDIO_ALSA_HPP_
#define _PDRAW_RENDERER_AUDIO_ALSA_HPP_

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

	~AlsaAudioRenderer(void);

	int start(void) override;

	int stop(void) override;

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId(void) const override;

	int
	setParams(const struct pdraw_audio_renderer_params *params) override;

	int getParams(struct pdraw_audio_renderer_params *params) override;

	int addInputMedia(Media *media) override;

	int removeInputMedia(Media *media) override;

	int removeInputMedias(void) override;

	void completeStop(void) override;

private:
	int startAlsa(void);

	int stopAlsa(void);

	void onChannelFlush(Channel *channel) override;

	void onChannelSos(Channel *channel) override;

	void onChannelEos(Channel *channel) override;

	static void idleStart(void *renderer);

	int render(void);

	static void renderCb(pomp_evt *event, void *userdata);

	static void watchdogTimerCb(struct pomp_timer *timer, void *userdata);

	static bool queueFilter(struct mbuf_audio_frame *frame, void *userdata);

	struct mbuf_audio_frame_queue *getLastAddedMediaQueue(void);

	int removeQueueFdFromPomp(struct mbuf_audio_frame_queue *queue);

	static void idleRenewMedia(void *userdata);

	unsigned int mMediaId;
	unsigned int mCurrentMediaId;
	bool mRunning;
	AudioMedia *mLastAddedMedia;
	struct pdraw_media_info mMediaInfo;
	bool mAlsaReady;
	struct pdraw_audio_renderer_params mParams;
	std::string mAddress;
	snd_pcm_t *mHandle;
	snd_pcm_hw_params_t *mHwParams;
	snd_pcm_sw_params_t *mSwParams;
	size_t mFrameSize;
	size_t mSampleCount;

	/* Watchdog timer: triggered if no new frame is received for a given
	 * amount of time */
	struct pomp_timer *mWatchdogTimer;
	std::atomic_bool mWatchdogTriggered;
	std::atomic_bool mEos;
};

} /* namespace Pdraw */

#endif /* PDRAW_USE_ALSA */

#endif /* !_PDRAW_RENDERER_AUDIO_ALSA_HPP_ */
