/**
 * Parrot Drones Audio and Video Vector library
 * ALSA source
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

#ifndef _PDRAW_ALSA_SOURCE_HPP_
#define _PDRAW_ALSA_SOURCE_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <string>

#include "pdraw_alsa_audio.hpp"
#include <pdraw/pdraw.hpp>

#ifdef PDRAW_USE_ALSA
#	include <alsa/asoundlib.h>

#	include <media-buffers/mbuf_audio_frame.h>
#endif

namespace Pdraw {


#ifdef PDRAW_USE_ALSA

class AlsaSourceWrapper;


class AlsaSource : public SourceElement {
public:
	AlsaSource(Session *session,
		   Element::Listener *elementListener,
		   Source::Listener *sourceListener,
		   IPdraw::IAlsaSource::Listener *listener,
		   AlsaSourceWrapper *wrapper,
		   const struct pdraw_alsa_source_params *params);

	~AlsaSource(void);

	int start(void) override;

	int stop(void) override;

	bool isReadyToPlay(void);

	bool isPaused(void);

	int play(void);

	int pause(void);

	static int getCapabilities(const std::string &address,
				   struct pdraw_alsa_source_caps *caps);

	IPdraw::IAlsaSource *getAlsaSource(void) const
	{
		return mAlsaSource;
	}

private:
	int readFrame(void);

	int processFrame(struct mbuf_mem *mem, size_t len);

	int setupMedia(void);

	int createMedia(void);

	int destroyMedia(void);

	int teardownChannels(void);

	int flush(void);

	void completeFlush(void);

	int tryStop(void);

	void completeStop(void);

	void onChannelFlushed(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	const char *getSourceName(void) const;

	/* Alsa source listener calls from idle functions */
	static void callOnMediaAdded(void *userdata);

	static void timerCb(struct pomp_timer *timer, void *userdata);

	IPdraw::IAlsaSource *mAlsaSource;
	IPdraw::IAlsaSource::Listener *mAlsaSourceListener;
	struct pdraw_alsa_source_params mParams;
	std::string mAddress;
	enum pdraw_alsa_source_eos_reason mLastEosReason;
	AudioMedia *mOutputMedia;
	bool mOutputMediaChanging;
	bool mReady;
	bool mRunning;
	bool mFirstFrame;
	unsigned int mFrameIndex;
	uint32_t mTimescale;
	uint64_t mLastTimestamp;
	bool mFlushPending;
	snd_pcm_t *mHandle;
	snd_pcm_hw_params_t *mHwParams;
	struct pomp_timer *mTimer;
	struct mbuf_pool *mPool;
	size_t mFrameSize;
	uint64_t mFirstTimestamp;
	uint64_t mCurTimestamp;
};

#endif /* PDRAW_USE_ALSA */


class AlsaSourceWrapper : public IPdraw::IAlsaSource, public ElementWrapper {
public:
	AlsaSourceWrapper(Session *session,
			  const struct pdraw_alsa_source_params *params,
			  IPdraw::IAlsaSource::Listener *listener);

	~AlsaSourceWrapper(void);

	bool isReadyToPlay(void) override;

	bool isPaused(void) override;

	int play(void) override;

	int pause(void) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
#ifdef PDRAW_USE_ALSA
		mSource = nullptr;
#endif
	}

#ifdef PDRAW_USE_ALSA
	Source *getSource() const
	{
		return mSource;
	}

	AlsaSource *getAlsaSource() const
	{
		return mSource;
	}
#endif

private:
#ifdef PDRAW_USE_ALSA
	AlsaSource *mSource;
#endif
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ALSA_SOURCE_HPP_ */
