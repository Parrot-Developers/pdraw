/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline source to sink channel for audio
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

#define ULOG_TAG pdraw_channel_audio
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_channel_audio.hpp"
#include "pdraw_media.hpp"

#include <errno.h>

namespace Pdraw {


AudioChannel::AudioChannel(Sink *owner,
			   SinkListener *sinkListener,
			   AudioSinkListener *audioSinkListener,
			   struct pomp_loop *loop) :
		Channel(owner, sinkListener, loop),
		mAudioSinkListener(audioSinkListener),
		mAudioMediaFormatCaps(nullptr), mAudioMediaFormatCapsCount(0),
		mQueue(nullptr)
{
}


int AudioChannel::getAudioMediaFormatCaps(const struct adef_format **caps)
{
	if (caps == nullptr)
		return -EINVAL;
	*caps = mAudioMediaFormatCaps;
	return mAudioMediaFormatCapsCount;
}


void AudioChannel::setAudioMediaFormatCaps(Sink *owner,
					   const struct adef_format *caps,
					   int count)
{
	if (owner != mOwner) {
		ULOGE("AudioChannel::setAudioMediaFormatCaps: "
		      "wrong owner");
		return;
	}
	mAudioMediaFormatCaps = caps;
	mAudioMediaFormatCapsCount = count;
}


struct mbuf_audio_frame_queue *AudioChannel::getQueue(Sink *owner)
{
	if (owner != mOwner) {
		ULOGE("AudioChannel::getQueue: wrong owner");
		return nullptr;
	}
	return mQueue;
}


void AudioChannel::setQueue(Sink *owner, struct mbuf_audio_frame_queue *queue)
{
	if (owner != mOwner) {
		ULOGE("AudioChannel::setQueue: wrong owner");
		return;
	}
	mQueue = queue;
}


int AudioChannel::queue(mbuf_audio_frame *frame)
{
	if (frame == nullptr)
		return -EINVAL;
	if (mAudioSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	mAudioSinkListener->onAudioChannelQueue(this, frame);
	return 0;
}

} /* namespace Pdraw */
