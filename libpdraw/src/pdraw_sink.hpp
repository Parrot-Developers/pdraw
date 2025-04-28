/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline media sink for elements
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

#ifndef _PDRAW_SINK_HPP_
#define _PDRAW_SINK_HPP_

#include "pdraw_channel.hpp"
#include "pdraw_channel_audio.hpp"
#include "pdraw_channel_coded_video.hpp"
#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace Pdraw {

class Session;


class Sink : public Channel::SinkListener,
	     public CodedVideoChannel::CodedVideoSinkListener,
	     public RawVideoChannel::RawVideoSinkListener,
	     public AudioChannel::AudioSinkListener {
public:
	virtual ~Sink(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	int getCodedVideoMediaFormatCaps(
		const struct vdef_coded_format **caps) const
	{
		if (caps == nullptr)
			return -EINVAL;
		*caps = mCodedVideoMediaFormatCaps;
		return mCodedVideoMediaFormatCapsCount;
	}

	int
	getRawVideoMediaFormatCaps(const struct vdef_raw_format **caps) const
	{
		if (caps == nullptr)
			return -EINVAL;
		*caps = mRawVideoMediaFormatCaps;
		return mRawVideoMediaFormatCapsCount;
	}

	int getAudioMediaFormatCaps(const struct adef_format **caps)
	{
		if (caps == nullptr)
			return -EINVAL;
		*caps = mAudioMediaFormatCaps;
		return mAudioMediaFormatCapsCount;
	}

	unsigned int getInputMediaCount(void);

	Media *getInputMedia(unsigned int index);

	Media *findInputMedia(Media *media);

	virtual int addInputMedia(Media *media);

	virtual int removeInputMedia(Media *media);

	Channel *getInputChannel(Media *media);

protected:
	struct InputPort {
		Media *media;
		Channel *channel;
	};

	Sink(Session *session,
	     unsigned int maxInputMedias,
	     const struct vdef_coded_format *codedVideoMediaFormatCaps,
	     int codedVideoMediaFormatCapsCount,
	     const struct vdef_raw_format *rawVideoMediaFormatCaps,
	     int rawVideoMediaFormatCapsCount,
	     const struct adef_format *audioMediaFormatCaps,
	     int audioMediaFormatCapsCount);

	void setCodedVideoMediaFormatCaps(const struct vdef_coded_format *caps,
					  int count)
	{
		mCodedVideoMediaFormatCaps = caps;
		mCodedVideoMediaFormatCapsCount = count;
	}

	void setRawVideoMediaFormatCaps(const struct vdef_raw_format *caps,
					int count)
	{
		mRawVideoMediaFormatCaps = caps;
		mRawVideoMediaFormatCapsCount = count;
	}

	void setAudioMediaFormatCaps(const struct adef_format *caps, int count)
	{
		mAudioMediaFormatCaps = caps;
		mAudioMediaFormatCapsCount = count;
	}

	InputPort *getInputPort(Media *media);

	virtual int removeInputMedias(void);

	virtual void
	onCodedVideoChannelQueue(CodedVideoChannel *channel,
				 struct mbuf_coded_video_frame *frame);

	virtual void onRawVideoChannelQueue(RawVideoChannel *channel,
					    struct mbuf_raw_video_frame *frame);

	virtual void onAudioChannelQueue(AudioChannel *channel,
					 struct mbuf_audio_frame *frame);

	virtual void onChannelDownstreamEvent(Channel *channel,
					      const struct pomp_msg *event);

	virtual void onChannelFlush(Channel *channel) = 0;

	virtual void onChannelTeardown(Channel *channel);

	virtual void onChannelSos(Channel *channel);

	virtual void onChannelEos(Channel *channel);

	virtual void onChannelReconfigure(Channel *channel);

	virtual void onChannelResolutionChange(Channel *channel);

	virtual void onChannelFramerateChange(Channel *channel);

	virtual void onChannelTimeout(Channel *channel);

	virtual void onChannelPhotoTrigger(Channel *channel);

	virtual void onChannelSessionMetaUpdate(Channel *channel);

	struct pomp_loop *mLoop;
	pthread_mutex_t mMutex;
	unsigned int mMaxInputMedias;
	std::vector<InputPort> mInputPorts;
	const struct vdef_coded_format *mCodedVideoMediaFormatCaps;
	int mCodedVideoMediaFormatCapsCount;
	const struct vdef_raw_format *mRawVideoMediaFormatCaps;
	int mRawVideoMediaFormatCapsCount;
	const struct adef_format *mAudioMediaFormatCaps;
	int mAudioMediaFormatCapsCount;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SINK_HPP_ */
