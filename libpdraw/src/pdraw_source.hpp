/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline source element
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

#ifndef _PDRAW_SOURCE_HPP_
#define _PDRAW_SOURCE_HPP_

#include "pdraw_channel.hpp"
#include "pdraw_media.hpp"

#include <string>
#include <vector>

namespace Pdraw {

class Source : public Channel::SourceListener {
public:
	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onOutputMediaAdded(Source *source,
						Media *media) = 0;

		virtual void onOutputMediaRemoved(Source *source,
						  Media *media) = 0;
	};

	virtual ~Source(void);

	void lock(void);

	void unlock(void);

	unsigned int getOutputMediaCount(void);

	Media *getOutputMedia(unsigned int index);

	Media *findOutputMedia(Media *media);

	unsigned int getOutputChannelCount(Media *media);

	Channel *getOutputChannel(Media *media, unsigned int index);

	Channel *getOutputChannel(Media *media, void *key);

	int addOutputChannel(Media *media, Channel *channel);

	int removeOutputChannel(Media *media, void *key);

protected:
	struct OutputPort {
		Media *media;
		std::vector<Channel *> channels;
		struct vbuf_pool *pool;

		inline OutputPort() : media(NULL), pool(NULL) {}
	};

	Source(Listener *listener);

	Media *getOutputMediaFromChannel(void *key);

	OutputPort *getOutputPort(Media *media);

	int addOutputPort(Media *media);

	int removeOutputPort(Media *media);

	int removeOutputPorts(void);

	int createOutputPortBuffersPool(Media *media,
					unsigned int count,
					size_t capacity);

	int destroyOutputPortBuffersPool(Media *media);

	int getH264OutputBuffer(VideoMedia *videoMedia,
				struct vbuf_buffer **buffer,
				bool *byteStreamRequired);

	int sendDownstreamEvent(Media *media, Channel::DownstreamEvent event);

	virtual void onChannelUpstreamEvent(Channel *channel,
					    struct pomp_msg *event);

	virtual void onChannelUnlink(Channel *channel);

	virtual void onChannelFlushed(Channel *channel);

	virtual void onChannelResync(Channel *channel);

	pthread_mutex_t mMutex;
	std::string mName;
	std::vector<OutputPort> mOutputPorts;
	Listener *mListener;

private:
	int destroyOutputPortBuffersPool(OutputPort *port);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SOURCE_HPP_ */
