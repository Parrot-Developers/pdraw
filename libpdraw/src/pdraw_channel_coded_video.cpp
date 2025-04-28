/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline source to sink channel for coded video
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

#define ULOG_TAG pdraw_channel_coded_video
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_channel_coded_video.hpp"
#include "pdraw_media.hpp"

#include <errno.h>

namespace Pdraw {


CodedVideoChannel::CodedVideoChannel(
	Sink *owner,
	SinkListener *sinkListener,
	CodedVideoSinkListener *codedVideoSinkListener,
	struct pomp_loop *loop) :
		Channel(owner, sinkListener, loop),
		mCodedVideoSinkListener(codedVideoSinkListener),
		mCodedVideoMediaFormatCaps(nullptr),
		mCodedVideoMediaFormatCapsCount(0), mQueue(nullptr)
{
}


int CodedVideoChannel::getCodedVideoMediaFormatCaps(
	const struct vdef_coded_format **caps) const
{
	if (caps == nullptr)
		return -EINVAL;
	*caps = mCodedVideoMediaFormatCaps;
	return mCodedVideoMediaFormatCapsCount;
}


void CodedVideoChannel::setCodedVideoMediaFormatCaps(
	const Sink *owner,
	const struct vdef_coded_format *caps,
	int count)
{
	if (owner != mOwner) {
		ULOGE("CodedVideoChannel::setRawVideoMediaFormatCaps: "
		      "wrong owner");
		return;
	}
	mCodedVideoMediaFormatCaps = caps;
	mCodedVideoMediaFormatCapsCount = count;
}


bool CodedVideoChannel::onlySupportsByteStream() const
{
	for (int i = 0; i < mCodedVideoMediaFormatCapsCount; i++) {
		if (mCodedVideoMediaFormatCaps[i].data_format !=
		    VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
			return false;
	}
	return true;
}


struct mbuf_coded_video_frame_queue *
CodedVideoChannel::getQueue(const Sink *owner) const
{
	if (owner != mOwner) {
		ULOGE("CodedVideoChannel::getQueue: wrong owner");
		return nullptr;
	}
	return mQueue;
}


void CodedVideoChannel::setQueue(const Sink *owner,
				 struct mbuf_coded_video_frame_queue *queue)
{
	if (owner != mOwner) {
		ULOGE("CodedVideoChannel::setQueue: wrong owner");
		return;
	}
	mQueue = queue;
}


int CodedVideoChannel::queue(mbuf_coded_video_frame *frame)
{
	if (frame == nullptr)
		return -EINVAL;
	if (mCodedVideoSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	mCodedVideoSinkListener->onCodedVideoChannelQueue(this, frame);
	return 0;
}

} /* namespace Pdraw */
