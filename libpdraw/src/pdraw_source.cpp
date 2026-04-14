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

#define ULOG_TAG pdraw_source
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_source.hpp"

#include <errno.h>

#include <media-buffers/mbuf_mem_generic.h>

namespace Pdraw {


Source::Source(unsigned int maxOutputMedias, Listener *listener) :
		mMaxOutputMedias(maxOutputMedias), mListener(listener)
{
}


Source::~Source()
{
	int ret = removeOutputPorts(true);
	if (ret < 0)
		ULOG_ERRNO("removeOutputPorts", -ret);

	unsigned int count = getOutputMediaCount();
	if (count > 0) {
		ULOGW("not all output ports have been removed! (count=%d)",
		      count);
	}
}


void Source::lock()
{
	mMutex.lock();
}


void Source::unlock()
{
	mMutex.unlock();
}


unsigned int Source::getOutputMediaCount()
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	auto ret = static_cast<unsigned int>(mOutputPorts.size());
	return ret;
}


Media *Source::getOutputMedia(unsigned int index)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	Media *ret = (index < mOutputPorts.size())
			     ? mOutputPorts.at(index).media
			     : nullptr;
	return ret;
}


Media *Source::findOutputMedia(const Media *media)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	Media *ret = nullptr;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = p->media;
		break;
	}
	return ret;
}


Media *Source::getOutputMediaFromChannel(const Channel *channel)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	Media *ret = nullptr;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		auto c = p->channels.begin();

		while (c != p->channels.end()) {
			if ((*c) != channel) {
				c++;
				continue;
			}
			ret = p->media;
			break;
		}

		if (ret == nullptr) {
			p++;
			continue;
		}
		break;
	}

	return ret;
}


Source::OutputPort *Source::getOutputPort(const Media *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *ret = nullptr;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = &(*p);
		break;
	}

	return ret;
}


int Source::addOutputPort(Media *media, void *elementUserData)
{
	if (media == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	if (mOutputPorts.size() >= mMaxOutputMedias) {
		return -ENOBUFS;
	}

	OutputPort port;
	port.media = media;
	port.elementUserData = elementUserData;
	mOutputPorts.push_back(port);

	ULOGI("%s: add port for media name=%s",
	      getName().c_str(),
	      media->getName().c_str());

	return 0;
}


int Source::removeOutputPort(const Media *media)
{
	if (media == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	bool found = false;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		found = true;

		size_t count = p->channels.size();
		if (count > 0) {
			ULOGW("not all output channels have been removed! "
			      "(count=%zd)",
			      count);
			return -EBUSY;
		}
		p->media = nullptr;
		destroyOutputPortMemoryPool(&(*p));
		mOutputPorts.erase(p);
		break;
	}

	if (!found)
		return -ENOENT;

	ULOGI("%s: delete port for media name=%s",
	      getName().c_str(),
	      media->getName().c_str());

	return 0;
}


int Source::removeOutputPorts()
{
	return removeOutputPorts(false);
}


int Source::removeOutputPorts(bool calledFromDtor)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (mListener) {
			mListener->onOutputMediaRemoved(
				this, p->media, p->elementUserData);
		}

		size_t count = p->channels.size();
		if (count > 0) {
			ULOGW("not all output channels have been removed! "
			      "(count=%zd)",
			      count);
			return -EBUSY;
		}
		ULOG_PRI(calledFromDtor ? ULOG_WARN : ULOG_INFO,
			 "%s: delete port for media name=%s",
			 calledFromDtor ? "Source" : getName().c_str(),
			 p->media->getName().c_str());
		p->media = nullptr;
		destroyOutputPortMemoryPool(&(*p));
		p++;
	}

	mOutputPorts.clear();

	return 0;
}


int Source::createOutputPortMemoryPool(const Media *media,
				       unsigned int count,
				       size_t capacity)
{
	int ret;

	if (media == nullptr)
		return -EINVAL;
	if (count == 0)
		return -EINVAL;
	if (capacity == 0)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *port = nullptr;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == nullptr)
		return -ENOENT;

	ret = mbuf_pool_new(mbuf_mem_generic_impl,
			    capacity,
			    count,
			    MBUF_POOL_NO_GROW,
			    0,
			    getName().c_str(),
			    &port->pool);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_pool_new", -ret);
		return ret;
	}

	return 0;
}


int Source::destroyOutputPortMemoryPool(OutputPort *port)
{
	int ret = 0;

	if (port == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	if (port->pool == nullptr) {
		return 0;
	}

	if (!port->sharedPool) {
		ret = mbuf_pool_destroy(port->pool);
		if (ret < 0)
			ULOG_ERRNO("mbuf_pool_destroy", -ret);
	}

	port->pool = nullptr;
	return ret;
}


int Source::destroyOutputPortMemoryPool(const Media *media)
{
	if (media == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *port = nullptr;
	auto p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == nullptr)
		return -ENOENT;

	int ret = destroyOutputPortMemoryPool(port);
	return ret;
}


unsigned int Source::getOutputChannelCount(const Media *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return 0;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		ULOG_ERRNO("port", ENOENT);
		return 0;
	}

	auto ret = static_cast<unsigned int>(port->channels.size());
	return ret;
}


Channel *Source::getOutputChannel(const Media *media, unsigned int index)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}
	if (index >= port->channels.size()) {
		ULOG_ERRNO("index", ENOENT);
		return nullptr;
	}

	Channel *ret = port->channels.at(index);

	return ret;
}


Channel *Source::findOutputChannel(const Media *media, const Channel *channel)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return nullptr;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}

	Channel *ret = nullptr;
	auto c = port->channels.begin();

	while (c != port->channels.end()) {
		if ((*c) != channel) {
			c++;
			continue;
		}
		ret = *c;
		break;
	}

	return ret;
}


int Source::addOutputChannel(const Media *media, Channel *channel)
{
	if (media == nullptr)
		return -EINVAL;
	if (channel == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Channel *c = findOutputChannel(media, channel);
	if (c != nullptr)
		return -EEXIST;

	OutputPort *port = getOutputPort(media);
	if (port == nullptr)
		return -ENOENT;

	channel->setSourceListener(this);
	port->channels.push_back(channel);

	ULOGI("%s: link media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());
	return 0;
}


int Source::removeOutputChannel(const Media *media, const Channel *channel)
{
	if (media == nullptr)
		return -EINVAL;
	if (channel == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr)
		return -ENOENT;

	Sink *owner = channel->getOwner();
	bool found = false;
	auto c = port->channels.begin();

	while (c != port->channels.end()) {
		if ((*c) != channel) {
			c++;
			continue;
		}
		found = true;
		(*c)->setSourceListener(nullptr);
		port->channels.erase(c);
		break;
	}

	if (!found)
		return -ENOENT;

	ULOGI("%s: unlink media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      owner);
	return 0;
}


int Source::sendDownstreamEvent(const Media *media,
				Channel::DownstreamEvent event)
{
	int ret;
	unsigned int outputChannelCount;
	Channel *channel;

	if (media == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	/* Send the event downstream */
	outputChannelCount = getOutputChannelCount(media);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(media, i);
		if (channel == nullptr) {
			ULOGW("invalid channel");
			continue;
		}
		ret = channel->sendDownstreamEvent(event);
		if (ret < 0)
			ULOG_ERRNO("channel->sendDownstreamEvent", -ret);
	}

	return 0;
}


void Source::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		ULOGE("%s: output media not found", __func__);
		return;
	}

	int ret = removeOutputChannel(media, channel);
	if (ret < 0)
		ULOG_ERRNO("removeOutputChannel", -ret);
}


void Source::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		ULOGE("%s: output media not found", __func__);
		return;
	}
	ULOGD("%s: channel flushed media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelDrained(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		ULOGE("%s: output media not found", __func__);
		return;
	}
	ULOGD("%s: channel drained media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelResync(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		ULOGE("%s: output media not found", __func__);
		return;
	}
	ULOGD("%s: channel resync media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelVideoPresStats(Channel *channel, VideoPresStats *stats)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		ULOGE("%s: output media not found", __func__);
		return;
	}
	ULOGD("%s: channel video stats media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelUpstreamEvent(Channel *channel,
				    const struct pomp_msg *event)
{
	VideoPresStats stats;
	int err;

	ULOGD("%s: channel upstream event %s",
	      getName().c_str(),
	      Channel::getUpstreamEventStr(static_cast<Channel::UpstreamEvent>(
		      pomp_msg_get_id(event))));

	switch (static_cast<Channel::UpstreamEvent>(pomp_msg_get_id(event))) {
	case Channel::UpstreamEvent::UNLINK:
		onChannelUnlink(channel);
		break;
	case Channel::UpstreamEvent::FLUSHED:
		onChannelFlushed(channel);
		break;
	case Channel::UpstreamEvent::DRAINED:
		onChannelDrained(channel);
		break;
	case Channel::UpstreamEvent::RESYNC:
		onChannelResync(channel);
		break;
	case Channel::UpstreamEvent::VIDEO_PRES_STATS:
		err = stats.readMsg(event);
		if (err < 0)
			ULOG_ERRNO("stats.readMsg", -err);
		else
			onChannelVideoPresStats(channel, &stats);
		break;
	default:
		ULOG_ERRNO("event id %d", ENOSYS, pomp_msg_get_id(event));
		break;
	}
}


int Source::getOutputMemory(const Media *media, struct mbuf_mem **mem)
{
	int ret = 0;
	OutputPort *port = nullptr;
	struct mbuf_pool *pool = nullptr;

	if (media == nullptr)
		return -EINVAL;
	if (mem == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	auto p = mOutputPorts.begin();
	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		pool = port->pool;
		break;
	}

	if (pool == nullptr) {
		ret = -EPROTO;
		ULOGE("no pool available");
		return ret;
	}

	ret = mbuf_pool_get(pool, mem);
	if ((ret < 0) || (*mem == nullptr)) {
		if (ret != -EAGAIN)
			ULOG_ERRNO("mbuf_pool_get:source", -ret);
		return ret;
	}

	return 0;
}


int Source::getCodedVideoOutputMemory(
	std::vector<CodedVideoMedia *> &videoMedias,
	struct mbuf_mem **mem,
	unsigned int *defaultMediaIndex)
{
	int ret = 0;
	unsigned int outputChannelCount = 0;
	unsigned int firstUsedMediaIndex = UINT_MAX;
	unsigned int firstUsedMediaCount = 0;
	Channel *channel;
	OutputPort *port;
	struct mbuf_pool *pool = nullptr;
	struct mbuf_pool *extPool = nullptr;
	bool externalPool = false;

	if (videoMedias.empty())
		return -EINVAL;
	if (mem == nullptr)
		return -EINVAL;
	if (defaultMediaIndex == nullptr)
		return -EINVAL;

	for (auto m : videoMedias) {
		if ((m->format.encoding != VDEF_ENCODING_H264) &&
		    (m->format.encoding != VDEF_ENCODING_H265))
			return -EINVAL;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	for (auto m = videoMedias.begin(); m != videoMedias.end(); m++) {
		outputChannelCount = getOutputChannelCount(*m);
		if (outputChannelCount > 0 && firstUsedMediaIndex == UINT_MAX) {
			firstUsedMediaCount = outputChannelCount;
			firstUsedMediaIndex = static_cast<unsigned int>(
				std::distance(videoMedias.begin(), m));
			channel = getOutputChannel((*m), (unsigned int)0);
			if (channel == nullptr) {
				ULOGW("invalid channel");
			} else {
				/* getPool() should only be called by the
				 * Channel owner, but this is a special case */
				extPool = channel->getPool(channel->getOwner());
			}
		}
	}

	/* If the first used media have a single output channel with an
	 * input pool, use it */
	if (firstUsedMediaCount == 1 && extPool != nullptr) {
		pool = extPool;
		externalPool = true;
	}

	/* Try external pool first, then fallback to internal one */
	for (unsigned int pass = 0; pass < 2; pass++) {

		/* Otherwise, use the pool of the prefered media */
		if (pool == nullptr && firstUsedMediaIndex != UINT_MAX &&
		    firstUsedMediaIndex < videoMedias.size()) {
			port = getOutputPort(videoMedias[firstUsedMediaIndex]);
			if (port != nullptr)
				pool = port->pool;
		}

		/* If still no pool, pick one from any media */
		if (pool == nullptr) {
			for (auto m = videoMedias.begin();
			     m != videoMedias.end();
			     m++) {
				port = getOutputPort(*m);
				if (port != nullptr && port->pool != nullptr) {
					pool = port->pool;
					firstUsedMediaIndex = static_cast<
						unsigned int>(std::distance(
						videoMedias.begin(), m));
					break;
				}
			}
		}

		/* If still no pool, return an error */
		if (pool == nullptr) {
			ret = -EPROTO;
			ULOGE("no pool available");
			goto exit;
		}

		ret = mbuf_pool_get(pool, mem);
		if ((ret < 0) || (*mem == nullptr)) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("mbuf_pool_get:source", -ret);
			else if (externalPool) {
				ULOGD("no memory available in the external "
				      "pool, fall back to the source "
				      "output port pool");
				externalPool = false;
				pool = nullptr;
				continue; /* Second attempt */
			}
		}

		/* Otherwise, stop retrying */
		break;
	}

exit:
	*defaultMediaIndex = firstUsedMediaIndex;
	return ret;
}


int Source::copyCodedVideoOutputFrame(const CodedVideoMedia *srcMedia,
				      struct mbuf_coded_video_frame *srcFrame,
				      CodedVideoMedia *dstMedia,
				      struct mbuf_coded_video_frame **dstFrame)
{
	int ret;
	unsigned int dummy;
	struct mbuf_mem *dstMem = nullptr;
	uint8_t *dstData = nullptr;
	size_t dstLen;
	struct vdef_coded_frame info;

	if (srcMedia == nullptr || srcFrame == nullptr || dstMedia == nullptr ||
	    dstFrame == nullptr)
		return -EINVAL;

	/* Encoding must stay the same */
	if (srcMedia->format.encoding != dstMedia->format.encoding)
		return -EINVAL;

	std::vector<CodedVideoMedia *> codedMedias;
	codedMedias.push_back(dstMedia);
	ret = getCodedVideoOutputMemory(codedMedias, &dstMem, &dummy);
	if (ret < 0) {
		ULOG_ERRNO("getCodedVideoOutputMemory", -ret);
		return ret;
	}

	ret = mbuf_coded_video_frame_copy(srcFrame, dstMem, dstFrame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_copy", -ret);
		goto exit;
	}

	/* Update dstFrame format */
	ret = mbuf_coded_video_frame_get_frame_info(*dstFrame, &info);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto exit;
	}
	info.format = dstMedia->format;
	ret = mbuf_coded_video_frame_set_frame_info(*dstFrame, &info);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_set_frame_info", -ret);
		goto exit;
	}

	ret = mbuf_coded_video_frame_finalize(*dstFrame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto exit;
	}

	/* If we have the same format, nothing more to do */
	if (vdef_coded_format_cmp(&srcMedia->format, &dstMedia->format))
		goto exit;

	/* We cannot convert to/from raw_nalus */
	if (srcMedia->format.data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU ||
	    dstMedia->format.data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU) {
		ULOGE("conversion from/to raw_nalu not supported");
		ret = -ENOSYS;
		goto exit;
	}

	/* Get the actual length of the frame */
	ret = mbuf_coded_video_frame_get_rw_packed_buffer(
		*dstFrame, reinterpret_cast<void **>(&dstData), &dstLen);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_rw_packed_buffer", -ret);
		goto exit;
	}

	if (srcMedia->format.data_format == VDEF_CODED_DATA_FORMAT_AVCC &&
	    dstMedia->format.data_format ==
		    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		switch (srcMedia->format.encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_avcc_to_byte_stream(dstData, dstLen);
			if (ret < 0)
				ULOG_ERRNO("h264_avcc_to_byte_stream", -ret);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_hvcc_to_byte_stream(dstData, dstLen);
			if (ret < 0)
				ULOG_ERRNO("h265_hvcc_to_byte_stream", -ret);
			break;
		default:
			break;
		}
	} else if (srcMedia->format.data_format ==
			   VDEF_CODED_DATA_FORMAT_BYTE_STREAM &&
		   dstMedia->format.data_format ==
			   VDEF_CODED_DATA_FORMAT_AVCC) {
		switch (srcMedia->format.encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_byte_stream_to_avcc(dstData, dstLen);
			if (ret < 0)
				ULOG_ERRNO("h264_byte_stream_to_avcc", -ret);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_byte_stream_to_hvcc(dstData, dstLen);
			if (ret < 0)
				ULOG_ERRNO("h265_byte_stream_to_hvcc", -ret);
			break;
		default:
			break;
		}
	} else {
		ULOGE("bad conversion from " VDEF_CODED_FORMAT_TO_STR_FMT
		      " to " VDEF_CODED_FORMAT_TO_STR_FMT,
		      VDEF_CODED_FORMAT_TO_STR_ARG(&srcMedia->format),
		      VDEF_CODED_FORMAT_TO_STR_ARG(&dstMedia->format));
		ret = -EINVAL;
	}

exit:
	if (dstData != nullptr)
		mbuf_coded_video_frame_release_rw_packed_buffer(*dstFrame,
								dstData);
	if (dstMem != nullptr)
		mbuf_mem_unref(dstMem);
	if (ret != 0 && *dstFrame != nullptr) {
		mbuf_coded_video_frame_unref(*dstFrame);
		*dstFrame = nullptr;
	}
	return ret;
}

} /* namespace Pdraw */
