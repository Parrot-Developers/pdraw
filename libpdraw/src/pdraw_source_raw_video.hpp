/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline source element for raw video
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

#ifndef _PDRAW_SOURCE_RAW_VIDEO_HPP_
#define _PDRAW_SOURCE_RAW_VIDEO_HPP_

#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace Pdraw {

class RawSource : public RawChannel::SourceListener {
public:
	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onOutputMediaAdded(RawSource *source,
						RawVideoMedia *media) = 0;

		virtual void onOutputMediaRemoved(RawSource *source,
						  RawVideoMedia *media) = 0;
	};

	virtual ~RawSource(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	unsigned int getOutputMediaCount(void);

	RawVideoMedia *getOutputMedia(unsigned int index);

	RawVideoMedia *findOutputMedia(RawVideoMedia *media);

	unsigned int getOutputChannelCount(RawVideoMedia *media);

	RawChannel *getOutputChannel(RawVideoMedia *media, unsigned int index);

	RawChannel *getOutputChannel(RawVideoMedia *media, void *key);

	int addOutputChannel(RawVideoMedia *media, RawChannel *channel);

	int removeOutputChannel(RawVideoMedia *media, void *key);

protected:
	struct OutputPort {
		RawVideoMedia *media;
		std::vector<RawChannel *> channels;
		struct mbuf_pool *pool;
		bool sharedPool;

		inline OutputPort() :
				media(nullptr), pool(nullptr), sharedPool(false)
		{
		}
	};

	RawSource(unsigned int maxOutputMedias, Listener *listener);

	RawVideoMedia *getOutputMediaFromChannel(void *key);

	OutputPort *getOutputPort(RawVideoMedia *media);

	int addOutputPort(RawVideoMedia *media);

	int removeOutputPort(RawVideoMedia *media);

	int removeOutputPorts(void);

	int createOutputPortMemoryPool(RawVideoMedia *media,
				       unsigned int count,
				       size_t capacity);

	int destroyOutputPortMemoryPool(RawVideoMedia *media);

	int sendDownstreamEvent(RawVideoMedia *media,
				RawChannel::DownstreamEvent event);

	virtual void onChannelUpstreamEvent(RawChannel *channel,
					    const struct pomp_msg *event);

	virtual void onChannelUnlink(RawChannel *channel);

	virtual void onChannelFlushed(RawChannel *channel);

	virtual void onChannelResync(RawChannel *channel);

	virtual void onChannelVideoPresStats(RawChannel *channel,
					     VideoPresStats *stats);

	pthread_mutex_t mMutex;
	unsigned int mMaxOutputMedias;
	std::vector<OutputPort> mOutputPorts;
	Listener *mListener;

private:
	int destroyOutputPortMemoryPool(OutputPort *port);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SOURCE_RAW_VIDEO_HPP_ */
