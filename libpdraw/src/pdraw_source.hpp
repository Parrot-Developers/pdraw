/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline media source for elements
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

#include <vector>

namespace Pdraw {

class Source : public Channel::SourceListener {
public:
	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onOutputMediaAdded(Source *source,
						Media *media,
						void *elementUserData) = 0;

		virtual void onOutputMediaRemoved(Source *source,
						  Media *media,
						  void *elementUserData) = 0;
	};

	virtual ~Source(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	unsigned int getOutputMediaCount(void);

	Media *getOutputMedia(unsigned int index);

	Media *findOutputMedia(Media *media);

	unsigned int getOutputChannelCount(Media *media);

	Channel *getOutputChannel(Media *media, unsigned int index);

	Channel *findOutputChannel(Media *media, Channel *channel);

	int addOutputChannel(Media *media, Channel *channel);

	int removeOutputChannel(Media *media, Channel *channel);

protected:
	struct OutputPort {
		Media *media;
		std::vector<Channel *> channels;
		struct mbuf_pool *pool;
		bool sharedPool;
		void *elementUserData;

		inline OutputPort() :
				media(nullptr), pool(nullptr),
				sharedPool(false), elementUserData(nullptr)
		{
		}
	};

	Source(unsigned int maxOutputMedias, Listener *listener);

	Media *getOutputMediaFromChannel(Channel *channel);

	OutputPort *getOutputPort(Media *media);

	int addOutputPort(Media *media, void *elementUserData = nullptr);

	int removeOutputPort(Media *media);

	int removeOutputPorts(void);

	int createOutputPortMemoryPool(Media *media,
				       unsigned int count,
				       size_t capacity);

	int destroyOutputPortMemoryPool(Media *media);

	int sendDownstreamEvent(Media *media, Channel::DownstreamEvent event);

	virtual void onChannelUpstreamEvent(Channel *channel,
					    const struct pomp_msg *event);

	virtual void onChannelUnlink(Channel *channel);

	virtual void onChannelFlushed(Channel *channel);

	virtual void onChannelResync(Channel *channel);

	virtual void onChannelVideoPresStats(Channel *channel,
					     VideoPresStats *stats);

	int getRawVideoOutputMemory(RawVideoMedia *media,
				    struct mbuf_mem **mem);

	int getCodedVideoOutputMemory(CodedVideoMedia **videoMedias,
				      unsigned int nbVideoMedias,
				      struct mbuf_mem **mem,
				      unsigned int *defaultMediaIndex);

	int copyCodedVideoOutputFrame(CodedVideoMedia *srcMedia,
				      struct mbuf_coded_video_frame *srcFrame,
				      CodedVideoMedia *dstMedia,
				      struct mbuf_coded_video_frame **dstFrame);

	pthread_mutex_t mMutex;
	unsigned int mMaxOutputMedias;
	std::vector<OutputPort> mOutputPorts;
	Listener *mListener;

private:
	int destroyOutputPortMemoryPool(OutputPort *port);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SOURCE_HPP_ */
