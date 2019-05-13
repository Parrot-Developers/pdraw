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

#include "pdraw_source.hpp"

#include <errno.h>

#include <video-buffers/vbuf_generic.h>
#define ULOG_TAG pdraw_source
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_source);

namespace Pdraw {


Source::Source(Listener *listener) : mName(""), mListener(listener)
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


Source::~Source(void)
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


void Source::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void Source::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


unsigned int Source::getOutputMediaCount(void)
{
	pthread_mutex_lock(&mMutex);
	unsigned int ret = mOutputPorts.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Source::getOutputMedia(unsigned int index)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = (index < mOutputPorts.size())
			     ? mOutputPorts.at(index).media
			     : NULL;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Source::findOutputMedia(Media *media)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = NULL;
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


Media *Source::getOutputMediaFromChannel(void *key)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = NULL;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		std::vector<Channel *>::iterator c = p->channels.begin();

		while (c != p->channels.end()) {
			if ((*c)->getKey() != key) {
				c++;
				continue;
			}
			ret = p->media;
			break;
		}

		if (ret == NULL) {
			p++;
			continue;
		}
		break;
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


Source::OutputPort *Source::getOutputPort(Media *media)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *ret = NULL;
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


int Source::addOutputPort(Media *media)
{
	if (media == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	OutputPort port;
	port.media = media;
	mOutputPorts.push_back(port);

	pthread_mutex_unlock(&mMutex);

	ULOGI("'%s': add port for media id=%d type=%s",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type));

	return 0;
}


int Source::removeOutputPort(Media *media)
{
	if (media == NULL)
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
		p->media = NULL;
		destroyOutputPortBuffersPool(&(*p));
		mOutputPorts.erase(p);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	ULOGI("'%s': delete port for media id=%d type=%s",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type));

	return 0;
}


int Source::removeOutputPorts(void)
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
		ULOGI("'%s': delete port for media id=%d type=%s",
		      mName.c_str(),
		      p->media->id,
		      Media::getMediaTypeStr(p->media->type));
		/* Note: unlike removeOutputPort(), here the media is deleted */
		delete p->media;
		p->media = NULL;
		destroyOutputPortBuffersPool(&(*p));
		p++;
	}

	mOutputPorts.clear();

	pthread_mutex_unlock(&mMutex);
	return 0;
}


int Source::createOutputPortBuffersPool(Media *media,
					unsigned int count,
					size_t capacity)
{
	int ret;
	struct vbuf_cbs cbs;

	if (media == NULL)
		return -EINVAL;
	if (count == 0)
		return -EINVAL;
	if (capacity == 0)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	OutputPort *port = NULL;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	ret = vbuf_generic_get_cbs(&cbs);
	if (ret < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("vbuf_generic_get_cbs", -ret);
		return ret;
	}

	ret = vbuf_pool_new(count, capacity, 0, &cbs, &port->pool);
	if (ret < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("vbuf_pool_new", -ret);
		return ret;
	}

	pthread_mutex_unlock(&mMutex);
	return 0;
}


int Source::destroyOutputPortBuffersPool(OutputPort *port)
{
	int ret;

	if (port == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	if (port->pool == NULL) {
		pthread_mutex_unlock(&mMutex);
		return 0;
	}

	ret = vbuf_pool_abort(port->pool);
	if (ret < 0)
		ULOG_ERRNO("vbuf_pool_abort", -ret);
	ret = vbuf_pool_destroy(port->pool);
	if (ret < 0)
		ULOG_ERRNO("vbuf_pool_destroy", -ret);

	port->pool = NULL;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Source::destroyOutputPortBuffersPool(Media *media)
{
	if (media == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	OutputPort *port = NULL;
	std::vector<OutputPort>::iterator p = mOutputPorts.begin();

	while (p != mOutputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		port = &(*p);
		break;
	}

	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	int ret = destroyOutputPortBuffersPool(port);
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Source::getH264OutputBuffer(VideoMedia *videoMedia,
				struct vbuf_buffer **buffer,
				bool *byteStreamRequired)
{
	int ret = 0;
	unsigned int outputChannelCount = 0, i;
	Channel *channel;
	OutputPort *port;
	struct vbuf_pool *pool, *extPool = NULL;

	if (videoMedia == NULL)
		return -EINVAL;
	if (buffer == NULL)
		return -EINVAL;
	if (byteStreamRequired == NULL)
		return -EINVAL;
	if (videoMedia->format != VideoMedia::Format::H264)
		return -EINVAL;

	*byteStreamRequired = false;
	pthread_mutex_lock(&mMutex);
	outputChannelCount = getOutputChannelCount(videoMedia);

	/* Find out if byte stream format is required: it is necessary if
	 * at least one output channel only supports byte stream format */
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(videoMedia, i);
		if (channel == NULL) {
			ULOGW("invalid channel");
			continue;
		}
		bool byteStream =
			(channel->getVideoMediaSubFormatCaps() ==
			 VideoMedia::H264BitstreamFormat::BYTE_STREAM);

		pool = channel->getPool();
		if (pool != NULL)
			extPool = pool;
		*byteStreamRequired |= byteStream;
	}

	/* Use an external pool if we are in a case where there is only 1
	 * channel with its own external pool: in that case there is no need
	 * to use the source output port pool as it would add a useless copy,
	 * just use a buffer from the external pool */
	if ((outputChannelCount == 1) && (extPool != NULL)) {
		ret = vbuf_pool_get(extPool, 0, buffer);
		if ((ret < 0) || (*buffer == NULL)) {
			if (ret != -EAGAIN) {
				pthread_mutex_unlock(&mMutex);
				ULOG_ERRNO("vbuf_pool_get:external", -ret);
				return ret;
			} else {
				ULOGD("no buffer available in the external "
				      "pool, fall back to the source "
				      "output port pool");
			}
		} else {
			pthread_mutex_unlock(&mMutex);
			return ret;
		}
	}

	/* Use the source pool */
	if (*buffer == NULL) {
		port = getOutputPort(videoMedia);
		if (port == NULL) {
			pthread_mutex_unlock(&mMutex);
			ULOGW("no output port found");
			return -EPROTO;
		}
		if (port->pool == NULL) {
			pthread_mutex_unlock(&mMutex);
			ULOGW("invalid output port pool");
			return -EPROTO;
		}
		ret = vbuf_pool_get(port->pool, 0, buffer);
		if ((ret < 0) || (*buffer == NULL)) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("vbuf_pool_get:source", -ret);
		}
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


unsigned int Source::getOutputChannelCount(Media *media)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return 0;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return 0;
	}

	unsigned int ret = port->channels.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Channel *Source::getOutputChannel(Media *media, unsigned int index)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return NULL;
	}
	if (index >= port->channels.size()) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("index", ENOENT);
		return NULL;
	}

	Channel *ret = port->channels.at(index);
	pthread_mutex_unlock(&mMutex);

	return ret;
}


Channel *Source::getOutputChannel(Media *media, void *key)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return NULL;
	}
	if (key == NULL) {
		ULOG_ERRNO("key", EINVAL);
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return NULL;
	}

	Channel *ret = NULL;
	std::vector<Channel *>::iterator c = port->channels.begin();

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


int Source::addOutputChannel(Media *media, Channel *channel)
{
	if (media == NULL)
		return -EINVAL;
	if (channel == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	Channel *c = getOutputChannel(media, channel->getKey());
	if (c != NULL) {
		pthread_mutex_unlock(&mMutex);
		return -EEXIST;
	}
	OutputPort *port = getOutputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	channel->setSourceListener(this);
	port->channels.push_back(channel);
	pthread_mutex_unlock(&mMutex);

	ULOGI("'%s': link media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());
	return 0;
}


int Source::removeOutputChannel(Media *media, void *key)
{
	if (media == NULL)
		return -EINVAL;
	if (key == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	OutputPort *port = getOutputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	bool found = false;
	std::vector<Channel *>::iterator c = port->channels.begin();

	while (c != port->channels.end()) {
		if ((*c)->getKey() != key) {
			c++;
			continue;
		}
		found = true;
		(*c)->setSourceListener(NULL);
		port->channels.erase(c);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	ULOGI("'%s': unlink media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      key);
	return 0;
}


int Source::sendDownstreamEvent(Media *media, Channel::DownstreamEvent event)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *channel;

	if (media == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	/* Send the event downstream */
	outputChannelCount = getOutputChannelCount(media);
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(media, i);
		if (channel == NULL) {
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


void Source::onChannelUnlink(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel->getKey());
	if (ret < 0)
		ULOG_ERRNO("removeOutputChannel", -ret);
}


void Source::onChannelFlushed(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}
	ULOGD("'%s': channel flushed media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelResync(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}
	ULOGD("'%s': channel resync media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Source::onChannelUpstreamEvent(Channel *channel, struct pomp_msg *event)
{
	ULOGD("'%s': channel upstream event %s",
	      mName.c_str(),
	      Channel::getUpstreamEventStr(
		      (Channel::UpstreamEvent)pomp_msg_get_id(event)));

	switch (pomp_msg_get_id(event)) {
	case Channel::UpstreamEvent::UNLINK:
		onChannelUnlink(channel);
		break;
	case Channel::UpstreamEvent::FLUSHED:
		onChannelFlushed(channel);
		break;
	case Channel::UpstreamEvent::RESYNC:
		onChannelResync(channel);
		break;
	default:
		ULOG_ERRNO("event id %d", ENOSYS, pomp_msg_get_id(event));
		break;
	}
}

} /* namespace Pdraw */
