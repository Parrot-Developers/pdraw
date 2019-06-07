/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline source element for coded video
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

#ifndef _PDRAW_SOURCE_CODED_VIDEO_HPP_
#define _PDRAW_SOURCE_CODED_VIDEO_HPP_

#include "pdraw_channel_coded_video.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace Pdraw {

class CodedSource : public CodedChannel::SourceListener {
public:
	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onOutputMediaAdded(CodedSource *source,
						CodedVideoMedia *media) = 0;

		virtual void onOutputMediaRemoved(CodedSource *source,
						  CodedVideoMedia *media) = 0;
	};

	virtual ~CodedSource(void);

	void lock(void);

	void unlock(void);

	virtual std::string &getName(void) = 0;

	unsigned int getOutputMediaCount(void);

	CodedVideoMedia *getOutputMedia(unsigned int index);

	CodedVideoMedia *findOutputMedia(CodedVideoMedia *media);

	unsigned int getOutputChannelCount(CodedVideoMedia *media);

	CodedChannel *getOutputChannel(CodedVideoMedia *media,
				       unsigned int index);

	CodedChannel *getOutputChannel(CodedVideoMedia *media, void *key);

	int addOutputChannel(CodedVideoMedia *media, CodedChannel *channel);

	int removeOutputChannel(CodedVideoMedia *media, void *key);

protected:
	struct OutputPort {
		CodedVideoMedia *media;
		std::vector<CodedChannel *> channels;
		struct mbuf_pool *pool;
		bool sharedPool;

		inline OutputPort() :
				media(nullptr), pool(nullptr), sharedPool(false)
		{
		}
	};

	CodedSource(unsigned int maxOutputMedias, Listener *listener);

	CodedVideoMedia *getOutputMediaFromChannel(void *key);

	OutputPort *getOutputPort(CodedVideoMedia *media);

	int addOutputPort(CodedVideoMedia *media);

	int removeOutputPort(CodedVideoMedia *media);

	int removeOutputPorts(void);

	int createOutputPortMemoryPool(CodedVideoMedia *media,
				       unsigned int count,
				       size_t capacity);

	int destroyOutputPortMemoryPool(CodedVideoMedia *media);

	int getOutputMemory(CodedVideoMedia **videoMedias,
			    unsigned int nbVideoMedias,
			    struct mbuf_mem **mem,
			    unsigned int *defaultMediaIndex);

	int copyOutputFrame(CodedVideoMedia *srcMedia,
			    struct mbuf_coded_video_frame *srcFrame,
			    CodedVideoMedia *dstMedia,
			    struct mbuf_coded_video_frame **dstFrame);

	int sendDownstreamEvent(CodedVideoMedia *media,
				CodedChannel::DownstreamEvent event);

	virtual void onChannelUpstreamEvent(CodedChannel *channel,
					    struct pomp_msg *event);

	virtual void onChannelUnlink(CodedChannel *channel);

	virtual void onChannelFlushed(CodedChannel *channel);

	virtual void onChannelResync(CodedChannel *channel);

	pthread_mutex_t mMutex;
	unsigned int mMaxOutputMedias;
	std::vector<OutputPort> mOutputPorts;
	Listener *mListener;

private:
	int destroyOutputPortMemoryPool(OutputPort *port);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SOURCE_CODED_VIDEO_HPP_ */
