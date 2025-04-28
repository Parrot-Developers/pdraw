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

#ifndef _PDRAW_ENCODER_AUDIO_HPP_
#define _PDRAW_ENCODER_AUDIO_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <string>

#include <audio-encode/aenc.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class AudioEncoderWrapper;


class AudioEncoder : public FilterElement {
public:
	AudioEncoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener,
		     IPdraw::IAudioEncoder::Listener *listener,
		     AudioEncoderWrapper *wrapper,
		     const struct aenc_config *params);

	~AudioEncoder(void);

	int start(void) override;

	int stop(void) override;

	void completeFlush(void);

	void completeStop(void);

	IPdraw::IAudioEncoder *getAudioEncoder(void) const
	{
		return mEncoder;
	}

private:
	int createOutputMedia(struct adef_frame *frame_info,
			      AudioMedia::Frame &frame);

	int flush(void);

	int tryStop(void);

	void removeEncoderListener(void);

	void onAudioChannelQueue(AudioChannel *channel,
				 struct mbuf_audio_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelFlushed(Channel *channel) override;

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

	static void idleCompleteFlush(void *userdata);

	IPdraw::IAudioEncoder *mEncoder;
	IPdraw::IAudioEncoder::Listener *mEncoderListener;
	pthread_mutex_t mListenerMutex;
	AudioMedia *mInputMedia;
	AudioMedia *mOutputMedia;
	struct mbuf_pool *mInputBufferPool;
	struct mbuf_audio_frame_queue *mInputBufferQueue;
	struct aenc_config *mEncoderConfig;
	std::string mEncoderName;
	std::string mEncoderDevice;
	struct aenc_encoder *mAenc;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mAencFlushPending;
	bool mAencStopPending;
	static const struct aenc_cbs mEncoderCbs;
};


class AudioEncoderWrapper : public IPdraw::IAudioEncoder,
			    public ElementWrapper {
public:
	AudioEncoderWrapper(Session *session,
			    const struct aenc_config *params,
			    IPdraw::IAudioEncoder::Listener *listener);

	~AudioEncoderWrapper(void);

	void clearElement(void) override
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
	AudioEncoder *mEncoder;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ENCODER_AUDIO_HPP_ */
