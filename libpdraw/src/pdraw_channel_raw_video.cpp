/**
 * Parrot Drones Awesome Video Viewer Library
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

#define ULOG_TAG pdraw_channel_raw_video
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <errno.h>

namespace Pdraw {


RawVideoChannel::RawVideoChannel(Sink *owner,
				 SinkListener *sinkListener,
				 RawVideoSinkListener *rawVideoSinkListener) :
		Channel(owner, sinkListener),
		mRawVideoSinkListener(rawVideoSinkListener),
		mRawVideoMediaFormatCaps(nullptr),
		mRawVideoMediaFormatCapsCount(0), mQueue(nullptr)
{
}


int RawVideoChannel::getRawVideoMediaFormatCaps(
	const struct vdef_raw_format **caps)
{
	if (caps == nullptr)
		return -EINVAL;
	*caps = mRawVideoMediaFormatCaps;
	return mRawVideoMediaFormatCapsCount;
}


void RawVideoChannel::setRawVideoMediaFormatCaps(
	Sink *owner,
	const struct vdef_raw_format *caps,
	int count)
{
	if (owner != mOwner) {
		ULOGE("RawVideoChannel::setRawVideoMediaFormatCaps: "
		      "wrong owner");
		return;
	}
	mRawVideoMediaFormatCaps = caps;
	mRawVideoMediaFormatCapsCount = count;
}


struct mbuf_raw_video_frame_queue *RawVideoChannel::getQueue(Sink *owner)
{
	if (owner != mOwner) {
		ULOGE("RawVideoChannel::getQueue: wrong owner");
		return nullptr;
	}
	return mQueue;
}


void RawVideoChannel::setQueue(Sink *owner,
			       struct mbuf_raw_video_frame_queue *queue)
{
	if (owner != mOwner) {
		ULOGE("RawVideoChannel::setQueue: wrong owner");
		return;
	}
	mQueue = queue;
}


int RawVideoChannel::queue(mbuf_raw_video_frame *frame)
{
	if (frame == nullptr)
		return -EINVAL;
	if (mRawVideoSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	mRawVideoSinkListener->onRawVideoChannelQueue(this, frame);
	return 0;
}

} /* namespace Pdraw */
