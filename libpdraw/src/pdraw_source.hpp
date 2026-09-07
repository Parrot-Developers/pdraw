/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_channel.hpp"
#include "pdraw_media.hpp"

#include <mutex>
#include <vector>

namespace Pdraw {

class Source : public Channel::SourceListener {
public:
	class Listener {
	public:
		virtual ~Listener() = default;

		virtual void onOutputMediaAdded(Source *source,
						Media *media,
						void *elementUserData) = 0;

		virtual void onOutputMediaRemoved(Source *source,
						  Media *media,
						  void *elementUserData) = 0;
	};

	~Source() override;

	void lock();

	void unlock();

	virtual const std::string &getName() const = 0;

	unsigned int getOutputMediaCount();

	Media *getOutputMedia(unsigned int index);

	Media *findOutputMedia(const Media *media);

	unsigned int getOutputChannelCount(const Media *media);

	Channel *getOutputChannel(const Media *media, unsigned int index);

	Channel *findOutputChannel(const Media *media, const Channel *channel);

	int addOutputChannel(const Media *media, Channel *channel);

	int removeOutputChannel(const Media *media, const Channel *channel);

protected:
	struct OutputPort {
		Media *media = nullptr;
		std::vector<Channel *> channels;
		struct mbuf_pool *pool = nullptr;
		bool sharedPool = false;
		void *elementUserData = nullptr;

		inline OutputPort() = default;
	};

	Source(unsigned int maxOutputMedias, Listener *listener);

	Media *getOutputMediaFromChannel(const Channel *channel);

	OutputPort *getOutputPort(const Media *media);

	int addOutputPort(Media *media, void *elementUserData = nullptr);

	int removeOutputPort(const Media *media);

	/* Null the media pointer of the output port for 'media' without
	 * removing the port.  Called when removeOutputPort() returned -EBUSY
	 * (channels still in async teardown) and the media object is about to
	 * be freed, to prevent Source::~Source() → removeOutputPorts() from
	 * dereferencing a dangling pointer. */
	void clearOutputPortMedia(const Media *media);

	/* Walk every channel in the output port for 'media' and:
	 *  - null the owning Sink's InputPort::media pointer, so that
	 *    Sink::~Sink() → removeInputMediasImpl() does not dereference
	 *    freed media memory; and
	 *  - null the channel's SourceListener pointer, so that
	 *    Channel::unlink() does not dispatch into the freed source object.
	 * Must be called before clearOutputPortMedia() (so the port can still
	 * be found by media pointer) and before both the media object and the
	 * source object are freed. */
	void clearAttachedSinksInputMedia(const Media *media);

	int teardownOutputChannels(const Media *media);

	int removeOutputPorts();

	int createOutputPortMemoryPool(const Media *media,
				       unsigned int count,
				       size_t capacity);

	int destroyOutputPortMemoryPool(const Media *media);

	int sendDownstreamEvent(const Media *media,
				Channel::DownstreamEvent event);

	void onChannelUpstreamEvent(Channel *channel,
				    const pomp::Message &event) override;

	virtual void onChannelUnlink(Channel *channel);

	virtual void onChannelFlushed(Channel *channel);

	virtual void onChannelDrained(Channel *channel);

	virtual void onChannelResync(Channel *channel);

	virtual void onChannelVideoPresStats(Channel *channel,
					     VideoPresStats *stats);

	int getOutputMemory(const Media *media, struct mbuf_mem **mem);

	int
	getCodedVideoOutputMemory(std::vector<CodedVideoMedia *> &videoMedia,
				  struct mbuf_mem **mem,
				  unsigned int *defaultMediaIndex);

	int copyCodedVideoOutputFrame(const CodedVideoMedia *srcMedia,
				      struct mbuf_coded_video_frame *srcFrame,
				      CodedVideoMedia *dstMedia,
				      struct mbuf_coded_video_frame **dstFrame);

	std::recursive_mutex mMutex{};
	unsigned int mMaxOutputMedias = 0;
	std::vector<OutputPort> mOutputPorts;
	Listener *mListener = nullptr;

private:
	int removeOutputPorts(const char *name);

	int destroyOutputPortMemoryPool(OutputPort *port);
};

} /* namespace Pdraw */
