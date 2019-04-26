/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline sink element
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
#include "pdraw_media.hpp"

#include <string>
#include <vector>

namespace Pdraw {

class Sink : public Channel::SinkListener {
public:
	virtual ~Sink(void);

	void lock(void);

	void unlock(void);

	uint32_t getMediaTypeCaps(void)
	{
		return mMediaTypeCaps;
	}

	uint32_t getVideoMediaFormatCaps(void)
	{
		return mVideoMediaFormatCaps;
	}

	uint32_t getVideoMediaSubFormatCaps(void)
	{
		return mVideoMediaSubFormatCaps;
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

	Sink(uint32_t mediaTypeCaps,
	     uint32_t videoMediaFormatCaps,
	     uint32_t videoMediaSubFormatCaps);

	void setMediaTypeCaps(uint32_t caps)
	{
		mMediaTypeCaps = caps;
	}

	void setVideoMediaFormatCaps(uint32_t caps)
	{
		mVideoMediaFormatCaps = caps;
	}

	void setVideoMediaSubFormatCaps(uint32_t caps)
	{
		mVideoMediaSubFormatCaps = caps;
	}

	InputPort *getInputPort(Media *media);

	virtual int removeInputMedias(void);

	virtual void onChannelQueue(Channel *channel, vbuf_buffer *buf);

	virtual void onChannelDownstreamEvent(Channel *channel,
					      struct pomp_msg *event);

	virtual void onChannelFlush(Channel *channel);

	virtual void onChannelTeardown(Channel *channel);

	virtual void onChannelSos(Channel *channel);

	virtual void onChannelEos(Channel *channel);

	virtual void onChannelReconfigure(Channel *channel);

	virtual void onChannelTimeout(Channel *channel);

	virtual void onChannelPhotoTrigger(Channel *channel);

	pthread_mutex_t mMutex;
	std::string mName;
	std::vector<InputPort> mInputPorts;
	uint32_t mMediaTypeCaps;
	uint32_t mVideoMediaFormatCaps;
	uint32_t mVideoMediaSubFormatCaps;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SINK_HPP_ */
