/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline source to sink channel for raw video
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

#ifndef _PDRAW_CHANNEL_RAW_VIDEO_HPP_
#define _PDRAW_CHANNEL_RAW_VIDEO_HPP_

#include "pdraw_channel.hpp"

#include <inttypes.h>

#include <libpomp.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>

namespace Pdraw {

class RawVideoChannel : public Channel {
public:
	class RawVideoSinkListener {
	public:
		virtual ~RawVideoSinkListener(void) {}

		virtual void
		onRawVideoChannelQueue(RawVideoChannel *channel,
				       struct mbuf_raw_video_frame *frame) = 0;
	};

	RawVideoChannel(Sink *owner,
			SinkListener *sinkListener,
			RawVideoSinkListener *rawVideoSinkListener,
			struct pomp_loop *loop);

	~RawVideoChannel(void) {}

	int queue(mbuf_raw_video_frame *frame);

	int
	getRawVideoMediaFormatCaps(const struct vdef_raw_format **caps) const;

	void setRawVideoMediaFormatCaps(const Sink *owner,
					const struct vdef_raw_format *caps,
					int count);

	struct mbuf_raw_video_frame_queue *getQueue(const Sink *owner) const;

	void setQueue(const Sink *owner,
		      struct mbuf_raw_video_frame_queue *queue);

private:
	RawVideoSinkListener *mRawVideoSinkListener;
	const struct vdef_raw_format *mRawVideoMediaFormatCaps;
	int mRawVideoMediaFormatCapsCount;
	struct mbuf_raw_video_frame_queue *mQueue;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_CHANNEL_RAW_VIDEO_HPP_ */
