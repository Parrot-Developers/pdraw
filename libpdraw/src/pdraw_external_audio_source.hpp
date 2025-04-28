/**
 * Parrot Drones Audio and Video Vector library
 * Application external audio source
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

#ifndef _PDRAW_EXTERNAL_AUDIO_SOURCE_HPP_
#define _PDRAW_EXTERNAL_AUDIO_SOURCE_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_audio_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class AudioSourceWrapper;


class ExternalAudioSource : public SourceElement {
public:
	ExternalAudioSource(Session *session,
			    Element::Listener *elementListener,
			    Source::Listener *sourceListener,
			    IPdraw::IAudioSource::Listener *listener,
			    AudioSourceWrapper *wrapper,
			    const struct pdraw_audio_source_params *params);

	~ExternalAudioSource(void);

	int start(void) override;

	int stop(void) override;

	int flush(void);

	struct mbuf_audio_frame_queue *getQueue(void) const
	{
		return mFrameQueue;
	}

	IPdraw::IAudioSource *getAudioSource(void) const
	{
		return mAudioSource;
	}

private:
	int processFrame(struct mbuf_audio_frame *frame);

	void completeFlush(void);

	int tryStop(void);

	void completeStop(void);

	void onChannelFlushed(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	static bool inputFilter(struct mbuf_audio_frame *frame, void *userdata);

	/* Audio source listener calls from idle functions */
	static void callOnMediaAdded(void *userdata);

	static void callAudioSourceFlushed(void *userdata);

	IPdraw::IAudioSource *mAudioSource;
	IPdraw::IAudioSource::Listener *mAudioSourceListener;
	struct pdraw_audio_source_params mParams;
	struct mbuf_audio_frame_queue *mFrameQueue;
	AudioMedia *mOutputMedia;
	uint64_t mLastTimestamp;
	bool mFlushPending;
};


class AudioSourceWrapper : public IPdraw::IAudioSource, public ElementWrapper {
public:
	AudioSourceWrapper(Session *session,
			   const struct pdraw_audio_source_params *params,
			   IPdraw::IAudioSource::Listener *listener);

	~AudioSourceWrapper(void);

	struct mbuf_audio_frame_queue *getQueue(void) override;

	int flush(void) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mSource = nullptr;
	}

	Source *getSource() const
	{
		return mSource;
	}

	ExternalAudioSource *getAudioSource() const
	{
		return mSource;
	}

private:
	ExternalAudioSource *mSource;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_EXTERNAL_AUDIO_SOURCE_HPP_ */
