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

#pragma once

#include "pdraw_channel.hpp"
#include "pdraw_channel_audio.hpp"
#include "pdraw_channel_coded_video.hpp"
#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace Pdraw {

class Session;


class Sink : public Channel::SinkListener,
	     public CodedVideoChannel::CodedVideoSinkListener,
	     public RawVideoChannel::RawVideoSinkListener,
	     public AudioChannel::AudioSinkListener {
public:
	~Sink() override;

	void lock();

	void unlock();

	virtual const std::string &getName() const = 0;

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

	int getAudioMediaFormatCaps(const struct adef_format **caps) const
	{
		if (caps == nullptr)
			return -EINVAL;
		*caps = mAudioMediaFormatCaps;
		return mAudioMediaFormatCapsCount;
	}

	unsigned int getInputMediaCount();

	Media *getInputMedia(unsigned int index);

	Media *findInputMedia(const Media *media);

	virtual int addInputMedia(Media *media);

	virtual int removeInputMedia(Media *media);

	/* Null the media pointer in the InputPort matching 'media', without
	 * unlinking the channel. Called by
	 * Source::clearAttachedSinksInputMedia() just before the source media
	 * object is freed (EBUSY teardown path), so that Sink::~Sink() →
	 * removeInputMediasImpl() does not dereference freed memory. */
	void clearInputMedia(const Media *media);

	Channel *getInputChannel(const Media *media);

protected:
	struct InputPort {
		Media *media = nullptr;
		std::unique_ptr<Channel> channel{};
	};

	Sink(const Session *session,
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

	InputPort *getInputPort(const Media *media);

	virtual int removeInputMedias();

	void
	onCodedVideoChannelQueue(CodedVideoChannel *channel,
				 struct mbuf_coded_video_frame *frame) override;

	void
	onRawVideoChannelQueue(RawVideoChannel *channel,
			       struct mbuf_raw_video_frame *frame) override;

	void onAudioChannelQueue(AudioChannel *channel,
				 struct mbuf_audio_frame *frame) override;

	void onChannelDownstreamEvent(Channel *channel,
				      const pomp::Message &event) override;

	virtual void onChannelFlush(Channel *channel) = 0;

	virtual void onChannelDrain(Channel *channel) = 0;

	virtual void onChannelTeardown(Channel *channel);

	virtual void onChannelSos(Channel *channel);

	virtual void onChannelEos(Channel *channel);

	virtual void onChannelReconfigure(Channel *channel);

	virtual void onChannelResolutionChange(Channel *channel);

	virtual void onChannelFramerateChange(Channel *channel);

	virtual void onChannelTimeout(Channel *channel);

	virtual void onChannelPhotoTrigger(Channel *channel);

	virtual void onChannelSessionMetaUpdate(Channel *channel);

	pomp::Loop *mLoop = nullptr;
	std::recursive_mutex mMutex{};
	unsigned int mMaxInputMedias = 0;
	std::vector<InputPort> mInputPorts{};
	const struct vdef_coded_format *mCodedVideoMediaFormatCaps = nullptr;
	int mCodedVideoMediaFormatCapsCount = 0;
	const struct vdef_raw_format *mRawVideoMediaFormatCaps = nullptr;
	int mRawVideoMediaFormatCapsCount = 0;
	const struct adef_format *mAudioMediaFormatCaps = nullptr;
	int mAudioMediaFormatCapsCount = 0;

private:
	int removeInputMediasImpl(const char *name);
};

} /* namespace Pdraw */
