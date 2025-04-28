/**
 * Parrot Drones Audio and Video Vector library
 * Application external audio sink
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

#ifndef _PDRAW_EXTERNAL_AUDIO_SINK_HPP_
#define _PDRAW_EXTERNAL_AUDIO_SINK_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_audio_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class AudioSinkWrapper;


class ExternalAudioSink : public SinkElement {
public:
	ExternalAudioSink(Session *session,
			  Element::Listener *elementListener,
			  IPdraw::IAudioSink::Listener *listener,
			  AudioSinkWrapper *wrapper);

	~ExternalAudioSink(void);

	int start(void) override;

	int stop(void) override;

	int flushDone(void);

	struct mbuf_audio_frame_queue *getQueue(void) const
	{
		return mInputFrameQueue;
	}

	IPdraw::IAudioSink *getAudioSink(void) const
	{
		return mAudioSink;
	}

private:
	int flush(void);

	int channelTeardown(AudioChannel *channel);

	void onAudioChannelQueue(AudioChannel *channel,
				 struct mbuf_audio_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	int prepareAudioFrame(AudioChannel *channel,
			      struct mbuf_audio_frame *frame);

	static void idleFlushDone(void *userdata);

	/* Audio sink listener calls from idle functions */
	static void callAudioSinkFlush(void *userdata);

	IPdraw::IAudioSink *mAudioSink;
	IPdraw::IAudioSink::Listener *mAudioSinkListener;
	AudioMedia *mInputMedia;
	struct mbuf_audio_frame_queue *mInputFrameQueue;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mTearingDown;
};


class AudioSinkWrapper : public IPdraw::IAudioSink, public ElementWrapper {
public:
	AudioSinkWrapper(Session *session,
			 IPdraw::IAudioSink::Listener *listener);

	~AudioSinkWrapper(void);

	struct mbuf_audio_frame_queue *getQueue(void) override;

	int queueFlushed(void) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mSink = nullptr;
	}

	Sink *getSink() const
	{
		return mSink;
	}

	ExternalAudioSink *getAudioSink() const
	{
		return mSink;
	}

private:
	ExternalAudioSink *mSink;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_EXTERNAL_AUDIO_SINK_HPP_ */
