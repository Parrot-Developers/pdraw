/**
 * Parrot Drones Audio and Video Vector library
 * Video encoder element
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
#include <memory>

#include <mutex>
#include <string>

#include <audio-encode/aenc.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_queue.hpp>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

DECLARE_UNIQUE_C_PTR(struct aenc_config, AencConfig, aenc_config_free);

class AudioEncoderWrapper;


class AudioEncoder : public FilterElement {
public:
	AudioEncoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener,
		     IPdraw::IAudioEncoder::Listener *listener,
		     AudioEncoderWrapper *wrapper,
		     const struct aenc_config *params);

	~AudioEncoder() override;

	int start() override;

	int stop() override;

	void completeFlush();

	void completeStop();

	IPdraw::IAudioEncoder *getAudioEncoder() const
	{
		return mEncoder;
	}

private:
	int createOutputMedia(const struct adef_frame *frame_info,
			      const AudioMedia::Frame &frame);

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	int tryStop();

	void removeEncoderListener();

	void onAudioChannelQueue(AudioChannel *channel,
				 struct mbuf_audio_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	static void frameOutputCb(struct aenc_encoder *enc,
				  int status,
				  struct mbuf_audio_frame *out_frame,
				  void *userdata);

	static void flushCb(struct aenc_encoder *enc, void *userdata);

	static void stopCb(struct aenc_encoder *enc, void *userdata);

	/* Can be called from any thread */
	static void framePreReleaseCb(struct mbuf_audio_frame *frame,
				      void *userdata);

	void idleCompleteFlush();

	IPdraw::IAudioEncoder *mEncoder = nullptr;
	IPdraw::IAudioEncoder::Listener *mEncoderListener = nullptr;
	std::recursive_mutex mListenerMutex{};
	AudioMedia *mInputMedia = nullptr;
	std::unique_ptr<AudioMedia> mOutputMedia{};
	struct mbuf_pool *mInputBufferPool = nullptr;
	std::unique_ptr<mbuf::Queue> mInputBufferQueue;
	AencConfigPtr mEncoderConfig;
	std::string mEncoderName{};
	std::string mEncoderDevice{};
	struct aenc_encoder *mAenc = nullptr;
	bool mInputChannelFlushPending = false;
	bool mOutputChannelDrainRequired = false;
	bool mAencFlushPending = false;
	bool mAencStopPending = false;
	bool mAencStopIssued = false;
	pomp::Loop::IdleHandlerFunc mCompleteFlushHandler;
	static const struct aenc_cbs mEncoderCbs;
};


class AudioEncoderWrapper : public IPdraw::IAudioEncoder,
			    public ElementWrapper {
public:
	AudioEncoderWrapper(Session *session,
			    const struct aenc_config *params,
			    IPdraw::IAudioEncoder::Listener *listener);

	~AudioEncoderWrapper() override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mEncoder = nullptr;
	}

	Sink *getEncoder() const
	{
		return mEncoder;
	}

	AudioEncoder *getAudioEncoder() const
	{
		return mEncoder;
	}

private:
	bool isElementStopped() const final
	{
		return (ElementWrapper::isElementStopped() ||
			mEncoder == nullptr);
	}

	AudioEncoder *mEncoder = nullptr;
};

} /* namespace Pdraw */
