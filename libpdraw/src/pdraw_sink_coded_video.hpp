/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline sink element for coded video
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

#ifndef _PDRAW_SINK_CODED_VIDEO_HPP_
#define _PDRAW_SINK_CODED_VIDEO_HPP_

#include "pdraw_channel_coded_video.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace Pdraw {

class CodedSink : public CodedChannel::SinkListener {
public:
	virtual ~CodedSink(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	int getCodedVideoMediaFormatCaps(const struct vdef_coded_format **caps)
	{
		if (!caps)
			return -EINVAL;
		*caps = mCodedVideoMediaFormatCaps;
		return mCodedVideoMediaFormatCapsCount;
	}

	unsigned int getInputMediaCount(void);

	CodedVideoMedia *getInputMedia(unsigned int index);

	CodedVideoMedia *findInputMedia(CodedVideoMedia *media);

	virtual int addInputMedia(CodedVideoMedia *media);

	virtual int removeInputMedia(CodedVideoMedia *media);

	CodedChannel *getInputChannel(CodedVideoMedia *media);

protected:
	struct InputPort {
		CodedVideoMedia *media;
		CodedChannel *channel;
	};

	CodedSink(unsigned int maxInputMedias,
		  const struct vdef_coded_format *codedVideoMediaFormatCaps,
		  int codedVideoMediaFormatCapsCount);

	void setCodedVideoMediaFormatCaps(const struct vdef_coded_format *caps,
					  int count)
	{
		mCodedVideoMediaFormatCaps = caps;
		mCodedVideoMediaFormatCapsCount = count;
	}

	InputPort *getInputPort(CodedVideoMedia *media);

	virtual int removeInputMedias(void);

	virtual void onChannelQueue(CodedChannel *channel,
				    struct mbuf_coded_video_frame *frame);

	virtual void onChannelDownstreamEvent(CodedChannel *channel,
					      const struct pomp_msg *event);

	virtual void onChannelFlush(CodedChannel *channel);

	virtual void onChannelTeardown(CodedChannel *channel);

	virtual void onChannelSos(CodedChannel *channel);

	virtual void onChannelEos(CodedChannel *channel);

	virtual void onChannelReconfigure(CodedChannel *channel);

	virtual void onChannelTimeout(CodedChannel *channel);

	virtual void onChannelPhotoTrigger(CodedChannel *channel);

	pthread_mutex_t mMutex;
	unsigned int mMaxInputMedias;
	std::vector<InputPort> mInputPorts;
	const struct vdef_coded_format *mCodedVideoMediaFormatCaps;
	int mCodedVideoMediaFormatCapsCount;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SINK_CODED_VIDEO_HPP_ */
