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

#define ULOG_TAG pdraw_source_raw_video
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_source_raw_video.hpp"

#include <errno.h>

#include <media-buffers/mbuf_mem_generic.h>

namespace Pdraw {


RawSource::RawSource(unsigned int maxOutputMedias, Listener *listener) :
		mMaxOutputMedias(maxOutputMedias), mListener(listener)
{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		goto error;
	}

	pthread_mutexattr_destroy(&attr);
	return;

error:
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


RawSource::~RawSource(void)
{
	int ret = removeOutputPorts();
	if (ret < 0)
		ULOG_ERRNO("removeOutputPorts", -ret);

	unsigned int count = getOutputMediaCount();
	if (count > 0) {
		ULOGW("not all output ports have been removed! (count=%d)",
		      count);
	}

	pthread_mutex_destroy(&mMutex);
}


void RawSource::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void RawSource::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


unsigned int RawSource::getOutputMediaCount(void)
{
	pthread_mutex_lock(&mMutex);
	unsigned int ret = mOutputPorts.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


RawVideoMedia *RawSource::getOutputMedia(unsigned int index)
{
	pthread_mutex_lock(&mMutex);
	RawVideoMedia *ret = (index < mOutputPorts.size())
				     ? mOutputPorts.at(index).media
				     : nullptr;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


RawVideoMedia *RawSource::findOutputMedia(RawVideoMedia *media)
{
	pthread_mutex_lock(&mMutex);
	RawVideoMedia *ret = nullptr;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = p->media;
		break;
	}
	pthread_mutex_unlock(&mMutex);
	return ret;
}


RawVideoMedia *RawSource::getOutputMediaFromChannel(void *key)
{
	pthread_mutex_lock(&mMutex);
	RawVideoMedia *ret = nullptr;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		std::vector<RawChannel *>::iterator c = p->channels.begin();

		while (c != p->channels.end()) {
			if ((*c)->getKey() != key) {
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

	pthread_mutex_unlock(&mMutex);
	return ret;
}


RawSource::OutputPort *RawSource::getOutputPort(RawVideoMedia *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *ret = nullptr;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = &(*p);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


int RawSource::addOutputPort(RawVideoMedia *media)
{
	if (media == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	if (mOutputPorts.size() >= mMaxOutputMedias) {
		pthread_mutex_unlock(&mMutex);
		return -ENOBUFS;
	}

	OutputPort port;
	port.media = media;
	mOutputPorts.push_back(port);

	pthread_mutex_unlock(&mMutex);

	ULOGI("%s: add port for media name=%s",
	      getName().c_str(),
	      media->getName().c_str());

	return 0;
}


int RawSource::removeOutputPort(RawVideoMedia *media)
{
	if (media == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	bool found = false;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		found = true;

		unsigned int count = p->channels.size();
		if (count > 0) {
			pthread_mutex_unlock(&mMutex);
			ULOGW("not all output channels have been removed! "
			      "(count=%d)",
			      count);
			return -EBUSY;
		}
		p->media = nullptr;
		destroyOutputPortMemoryPool(&(*p));
		mOutputPorts.erase(p);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	ULOGI("%s: delete port for media name=%s",
	      getName().c_str(),
	      media->getName().c_str());

	return 0;
}


int RawSource::removeOutputPorts(void)
{
	pthread_mutex_lock(&mMutex);
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (mListener)
			mListener->onOutputMediaRemoved(this, p->media);

		unsigned int count = p->channels.size();
		if (count > 0) {
			pthread_mutex_unlock(&mMutex);
			ULOGW("not all output channels have been removed! "
			      "(count=%d)",
			      count);
			return -EBUSY;
		}
		ULOGI("%s: delete port for media name=%s",
		      getName().c_str(),
		      p->media->getName().c_str());
		/* Note: unlike removeOutputPort(), here the media is deleted */
		delete p->media;
		p->media = nullptr;
		destroyOutputPortMemoryPool(&(*p));
		p++;
	}

	mOutputPorts.clear();

	pthread_mutex_unlock(&mMutex);
	return 0;
}


int RawSource::createOutputPortMemoryPool(RawVideoMedia *media,
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

	pthread_mutex_lock(&mMutex);
	OutputPort *port = nullptr;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	ret = mbuf_pool_new(mbuf_mem_generic_impl,
			    capacity,
			    count,
			    MBUF_POOL_NO_GROW,
			    0,
			    getName().c_str(),
			    &port->pool);
	if (ret < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("mbuf_pool_new", -ret);
		return ret;
	}

	pthread_mutex_unlock(&mMutex);
	return 0;
}


int RawSource::destroyOutputPortMemoryPool(OutputPort *port)
{
	int ret = 0;

	if (port == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	if (port->pool == nullptr) {
		pthread_mutex_unlock(&mMutex);
		return 0;
	}

	if (!port->sharedPool) {
		ret = mbuf_pool_destroy(port->pool);
		if (ret < 0)
			ULOG_ERRNO("mbuf_pool_destroy", -ret);
	}

	port->pool = nullptr;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int RawSource::destroyOutputPortMemoryPool(RawVideoMedia *media)
{
	if (media == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	OutputPort *port = nullptr;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	int ret = destroyOutputPortMemoryPool(port);
	pthread_mutex_unlock(&mMutex);
	return ret;
}


unsigned int RawSource::getOutputChannelCount(RawVideoMedia *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return 0;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return 0;
	}

	unsigned int ret = port->channels.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


RawChannel *RawSource::getOutputChannel(RawVideoMedia *media,
					unsigned int index)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}
	if (index >= port->channels.size()) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("index", ENOENT);
		return nullptr;
	}

	RawChannel *ret = port->channels.at(index);
	pthread_mutex_unlock(&mMutex);

	return ret;
}


RawChannel *RawSource::getOutputChannel(RawVideoMedia *media, void *key)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}
	if (key == nullptr) {
		ULOG_ERRNO("key", EINVAL);
		return nullptr;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}

	RawChannel *ret = nullptr;
	std::vector<RawChannel *>::iterator c = port->channels.begin();

	while (c != port->channels.end()) {
		if ((*c)->getKey() != key) {
			c++;
			continue;
		}
		ret = *c;
		break;
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


int RawSource::addOutputChannel(RawVideoMedia *media, RawChannel *channel)
{
	if (media == nullptr)
		return -EINVAL;
	if (channel == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	RawChannel *c = getOutputChannel(media, channel->getKey());
	if (c != nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -EEXIST;
	}
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	channel->setSourceListener(this);
	port->channels.push_back(channel);
	pthread_mutex_unlock(&mMutex);

	ULOGI("%s: link media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());
	return 0;
}


int RawSource::removeOutputChannel(RawVideoMedia *media, void *key)
{
	if (media == nullptr)
		return -EINVAL;
	if (key == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	bool found = false;
	std::vector<RawChannel *>::iterator c = port->channels.begin();

	while (c != port->channels.end()) {
		if ((*c)->getKey() != key) {
			c++;
			continue;
		}
		found = true;
		(*c)->setSourceListener(nullptr);
		port->channels.erase(c);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	ULOGI("%s: unlink media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      key);
	return 0;
}


int RawSource::sendDownstreamEvent(RawVideoMedia *media,
				   RawChannel::DownstreamEvent event)
{
	int ret;
	unsigned int outputChannelCount, i;
	RawChannel *channel;

	if (media == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	/* Send the event downstream */
	outputChannelCount = getOutputChannelCount(media);
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(media, i);
		if (channel == nullptr) {
			ULOGW("invalid channel");
			continue;
		}
		ret = channel->sendDownstreamEvent(event);
		if (ret < 0)
			ULOG_ERRNO("channel->sendDownstreamEvent", -ret);
	}

	pthread_mutex_unlock(&mMutex);

	return 0;
}


void RawSource::onChannelUnlink(RawChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	RawVideoMedia *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		ULOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel->getKey());
	if (ret < 0)
		ULOG_ERRNO("removeOutputChannel", -ret);
}


void RawSource::onChannelFlushed(RawChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	RawVideoMedia *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		ULOGE("media not found");
		return;
	}
	ULOGD("%s: channel flushed media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void RawSource::onChannelResync(RawChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	RawVideoMedia *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		ULOGE("media not found");
		return;
	}
	ULOGD("%s: channel resync media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void RawSource::onChannelVideoPresStats(RawChannel *channel,
					VideoPresStats *stats)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	RawVideoMedia *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		ULOGE("media not found");
		return;
	}
	ULOGD("%s: channel video stats media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void RawSource::onChannelUpstreamEvent(RawChannel *channel,
				       const struct pomp_msg *event)
{
	VideoPresStats stats;
	int err;

	ULOGD("%s: channel upstream event %s",
	      getName().c_str(),
	      RawChannel::getUpstreamEventStr(
		      (RawChannel::UpstreamEvent)pomp_msg_get_id(event)));

	switch (pomp_msg_get_id(event)) {
	case RawChannel::UpstreamEvent::UNLINK:
		onChannelUnlink(channel);
		break;
	case RawChannel::UpstreamEvent::FLUSHED:
		onChannelFlushed(channel);
		break;
	case RawChannel::UpstreamEvent::RESYNC:
		onChannelResync(channel);
		break;
	case RawChannel::UpstreamEvent::VIDEO_PRES_STATS:
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

} /* namespace Pdraw */
