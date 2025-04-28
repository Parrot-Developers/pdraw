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

#ifndef _PDRAW_CHANNEL_AUDIO_HPP_
#define _PDRAW_CHANNEL_AUDIO_HPP_

#include "pdraw_channel.hpp"

#include <inttypes.h>

#include <audio-defs/adefs.h>
#include <libpomp.h>
#include <media-buffers/mbuf_audio_frame.h>

namespace Pdraw {

class AudioChannel : public Channel {
public:
	class AudioSinkListener {
	public:
		virtual ~AudioSinkListener(void) {}

		virtual void
		onAudioChannelQueue(AudioChannel *channel,
				    struct mbuf_audio_frame *frame) = 0;
	};

	AudioChannel(Sink *owner,
		     SinkListener *sinkListener,
		     AudioSinkListener *audioSinkListener,
		     struct pomp_loop *loop);

	~AudioChannel(void) {}

	int queue(mbuf_audio_frame *frame);

	int getAudioMediaFormatCaps(const struct adef_format **caps);

	void setAudioMediaFormatCaps(Sink *owner,
				     const struct adef_format *caps,
				     int count);

	struct mbuf_audio_frame_queue *getQueue(Sink *owner);

	void setQueue(Sink *owner, struct mbuf_audio_frame_queue *queue);

private:
	AudioSinkListener *mAudioSinkListener;
	const struct adef_format *mAudioMediaFormatCaps;
	int mAudioMediaFormatCapsCount;
	struct mbuf_audio_frame_queue *mQueue;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_CHANNEL_AUDIO_HPP_ */
