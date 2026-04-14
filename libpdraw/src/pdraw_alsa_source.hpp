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

#pragma once

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

constexpr size_t ALSA_SOURCE_DEFAULT_TIMESCALE = 1000000;

class AlsaSourceWrapper;


class AlsaSource : public SourceElement {
public:
	AlsaSource(Session *session,
		   Element::Listener *elementListener,
		   Source::Listener *sourceListener,
		   IPdraw::IAlsaSource::Listener *listener,
		   AlsaSourceWrapper *wrapper,
		   const struct pdraw_alsa_source_params *params);

	~AlsaSource() override;

	int start() override;

	int stop() override;

	bool isReadyToPlay() const;

	bool isPaused() const;

	int play();

	int pause();

	inline int drain()
	{
		return flush(false);
	}

	static int getCapabilities(const std::string &address,
				   struct pdraw_alsa_source_caps *caps);

	IPdraw::IAlsaSource *getAlsaSource() const
	{
		return mAlsaSource;
	}

private:
	int readFrame();

	int processFrame(struct mbuf_mem *mem, size_t len);

	int setupMedia();

	int createMedia();

	int destroyMedia();

	int teardownChannels();

	int flush(bool discard = true);

	void completeFlush();

	int tryStop();

	void completeStop();

	void playResponse();

	void pauseResponse();

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	const char *getSourceName() const;

	/* Alsa source listener calls from idle functions */
	static void callOnMediaAdded(void *userdata);

	static void callPlayResponse(void *userdata);

	static void callPauseResponse(void *userdata);

	static void timerCb(struct pomp_timer *timer, void *userdata);

	static void idleCompleteFlush(void *userdata);

	IPdraw::IAlsaSource *mAlsaSource = nullptr;
	IPdraw::IAlsaSource::Listener *mAlsaSourceListener = nullptr;
	struct pdraw_alsa_source_params mParams {
	};
	std::string mAddress{};
	std::unique_ptr<AudioMedia> mOutputMedia{};
	bool mOutputMediaChanging = false;
	bool mReady = false;
	bool mRunning = false;
	bool mFirstFrame = true;
	bool mPausePending = false;
	unsigned int mFrameIndex = 0;
	uint32_t mTimescale = ALSA_SOURCE_DEFAULT_TIMESCALE;
	uint64_t mLastTimestamp = UINT64_MAX;
	snd_pcm_t *mHandle = nullptr;
	snd_pcm_hw_params_t *mHwParams = nullptr;
	struct pomp_timer *mTimer = nullptr;
	struct mbuf_pool *mPool = nullptr;
	size_t mFrameSize = 0;
	uint64_t mFirstTimestamp = 0;
	uint64_t mCurTimestamp = 0;
};

#endif /* PDRAW_USE_ALSA */


class AlsaSourceWrapper : public IPdraw::IAlsaSource, public ElementWrapper {
public:
	AlsaSourceWrapper(Session *session,
			  const struct pdraw_alsa_source_params *params,
			  IPdraw::IAlsaSource::Listener *listener);

	~AlsaSourceWrapper() override;

	bool isReadyToPlay() override;

	bool isPaused() override;

	int play() override;

	int pause() override;

	void clearElement() override
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
	bool isElementStopped() const override
	{
		return (ElementWrapper::isElementStopped()
#ifdef PDRAW_USE_ALSA
			|| mSource == nullptr
#endif
		);
	}

#ifdef PDRAW_USE_ALSA
	AlsaSource *mSource = nullptr;
#endif
};

} /* namespace Pdraw */
