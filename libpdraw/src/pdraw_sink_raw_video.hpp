/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline sink element for raw video
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

#ifndef _PDRAW_SINK_RAW_VIDEO_HPP_
#define _PDRAW_SINK_RAW_VIDEO_HPP_

#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace Pdraw {

class RawSink : public RawChannel::SinkListener {
public:
	virtual ~RawSink(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	int getRawVideoMediaFormatCaps(const struct vdef_raw_format **caps)
	{
		if (!caps)
			return -EINVAL;
		*caps = mRawVideoMediaFormatCaps;
		return mRawVideoMediaFormatCapsCount;
	}

	unsigned int getInputMediaCount(void);

	RawVideoMedia *getInputMedia(unsigned int index);

	RawVideoMedia *findInputMedia(RawVideoMedia *media);

	virtual int addInputMedia(RawVideoMedia *media);

	virtual int removeInputMedia(RawVideoMedia *media);

	RawChannel *getInputChannel(RawVideoMedia *media);

protected:
	struct InputPort {
		RawVideoMedia *media;
		RawChannel *channel;
	};

	RawSink(unsigned int maxInputMedias,
		const struct vdef_raw_format *rawVideoMediaFormatCaps,
		int rawVideoMediaFormatCapsCount);

	void setRawVideoMediaFormatCaps(const struct vdef_raw_format *caps,
					int count)
	{
		mRawVideoMediaFormatCaps = caps;
		mRawVideoMediaFormatCapsCount = count;
	}

	InputPort *getInputPort(RawVideoMedia *media);

	virtual int removeInputMedias(void);

	virtual void onChannelQueue(RawChannel *channel,
				    struct mbuf_raw_video_frame *frame);

	virtual void onChannelDownstreamEvent(RawChannel *channel,
					      struct pomp_msg *event);

	virtual void onChannelFlush(RawChannel *channel);

	virtual void onChannelTeardown(RawChannel *channel);

	virtual void onChannelSos(RawChannel *channel);

	virtual void onChannelEos(RawChannel *channel);

	virtual void onChannelReconfigure(RawChannel *channel);

	virtual void onChannelTimeout(RawChannel *channel);

	virtual void onChannelPhotoTrigger(RawChannel *channel);

	pthread_mutex_t mMutex;
	unsigned int mMaxInputMedias;
	std::vector<InputPort> mInputPorts;
	const struct vdef_raw_format *mRawVideoMediaFormatCaps;
	int mRawVideoMediaFormatCapsCount;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SINK_HPP_ */
