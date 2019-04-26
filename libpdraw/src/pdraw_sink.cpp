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

#include "pdraw_sink.hpp"

#include <errno.h>

#define ULOG_TAG pdraw_sink
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_sink);

namespace Pdraw {


Sink::Sink(uint32_t mediaTypeCaps,
	   uint32_t videoMediaFormatCaps,
	   uint32_t videoMediaSubFormatCaps) :
		mName(""),
		mMediaTypeCaps(mediaTypeCaps),
		mVideoMediaFormatCaps(videoMediaFormatCaps),
		mVideoMediaSubFormatCaps(videoMediaSubFormatCaps)
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


Sink::~Sink(void)
{
	int ret = removeInputMedias();
	if (ret < 0)
		ULOG_ERRNO("removeInputMedias", -ret);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		ULOGW("not all input ports have been removed! (count=%d)",
		      count);
	}

	pthread_mutex_destroy(&mMutex);
}


void Sink::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void Sink::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


unsigned int Sink::getInputMediaCount(void)
{
	pthread_mutex_lock(&mMutex);
	unsigned int ret = mInputPorts.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Sink::getInputMedia(unsigned int index)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = (index < mInputPorts.size()) ? mInputPorts.at(index).media
						  : NULL;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Sink::findInputMedia(Media *media)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
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


Sink::InputPort *Sink::getInputPort(Media *media)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	InputPort *ret = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
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


int Sink::addInputMedia(Media *media)
{
	if (media == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	if (getInputPort(media) != NULL) {
		pthread_mutex_unlock(&mMutex);
		return -EEXIST;
	}
	if ((mMediaTypeCaps & media->type) == 0) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("media type %s not supported",
		      Media::getMediaTypeStr(media->type));
		return -ENOSYS;
	}
	if (media->type == Media::Type::VIDEO) {
		VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
		if (videoMedia == NULL) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("invalid video media");
			return -EPROTO;
		}
		if ((mVideoMediaFormatCaps & videoMedia->format) == 0) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("video media format %s not supported",
			      VideoMedia::getVideoFormatStr(
				      videoMedia->format));
			return -ENOSYS;
		}
		if ((videoMedia->subFormat !=
		     VideoMedia::SubFormat::SUBFORMAT_UNKNOWN) &&
		    ((mVideoMediaSubFormatCaps & videoMedia->subFormat) == 0)) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("video media sub-format %s not supported",
			      VideoMedia::getVideoSubFormatStr(
				      videoMedia->subFormat));
			return -ENOSYS;
		}
	}

	InputPort port;
	memset(&port, 0, sizeof(port));
	port.media = media;
	port.channel = new Channel(this);
	if (port.channel == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("failed to create channel");
		return -ENOMEM;
	}
	port.channel->setMediaTypeCaps(mMediaTypeCaps);
	port.channel->setVideoMediaFormatCaps(mVideoMediaFormatCaps);
	port.channel->setVideoMediaSubFormatCaps(mVideoMediaSubFormatCaps);

	mInputPorts.push_back(port);
	pthread_mutex_unlock(&mMutex);

	ULOGI("'%s': link media id=%d type=%s",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type));
	return 0;
}


int Sink::removeInputMedia(Media *media)
{
	if (media == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	bool found = false;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		found = true;
		ULOGI("'%s': unlink media id=%d type=%s",
		      mName.c_str(),
		      media->id,
		      Media::getMediaTypeStr(media->type));
		int ret = p->channel->unlink();
		if (ret < 0)
			ULOG_ERRNO("channel->unlink", -ret);
		delete p->channel;
		p->channel = NULL;
		mInputPorts.erase(p);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	return 0;
}


int Sink::removeInputMedias(void)
{
	pthread_mutex_lock(&mMutex);
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		ULOGI("'%s': unlink media id=%d type=%s",
		      mName.c_str(),
		      p->media->id,
		      Media::getMediaTypeStr(p->media->type));
		int ret = p->channel->unlink();
		if (ret < 0)
			ULOG_ERRNO("channel->unlink", -ret);
		delete p->channel;
		p->channel = NULL;
		p++;
	}

	mInputPorts.clear();

	pthread_mutex_unlock(&mMutex);
	return 0;
}


Channel *Sink::getInputChannel(Media *media)
{
	if (media == NULL) {
		ULOG_ERRNO("media", EINVAL);
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	InputPort *port = getInputPort(media);
	if (port == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return NULL;
	}

	Channel *ret = port->channel;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Sink::onChannelQueue(Channel *channel, vbuf_buffer *buf)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	if (buf == NULL) {
		ULOG_ERRNO("buf", EINVAL);
		return;
	}
	struct vbuf_queue *queue = channel->getQueue();
	if (queue == NULL) {
		ULOGE("invalid queue");
		return;
	}

	int ret = vbuf_queue_push(queue, buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_push", -ret);
		return;
	}
}


void Sink::onChannelFlush(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	struct vbuf_queue *queue = channel->getQueue();
	if (queue == NULL) {
		ULOGE("invalid queue");
		return;
	}

	int ret = vbuf_queue_flush(queue);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_flush", -ret);
		return;
	}

	ret = channel->flushDone();
	if (ret < 0)
		ULOG_ERRNO("channel->flushDone", -ret);
}


void Sink::onChannelTeardown(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	int ret = removeInputMedia(media);
	if (ret < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("removeInputMedia", -ret);
		return;
	}

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelSos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("'%s': channel SOS media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelEos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("'%s': channel EOS media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelReconfigure(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("'%s': channel reconfigure media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelTimeout(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("'%s': channel timeout media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = NULL;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("'%s': channel photo_trigger "
	      "media id=%d type=%s (channel key=%p)",
	      mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelDownstreamEvent(Channel *channel, struct pomp_msg *event)
{
	ULOGD("'%s': channel downstream event %s",
	      mName.c_str(),
	      Channel::getDownstreamEventStr(
		      (Channel::DownstreamEvent)pomp_msg_get_id(event)));

	switch (pomp_msg_get_id(event)) {
	case Channel::DownstreamEvent::FLUSH:
		onChannelFlush(channel);
		break;
	case Channel::DownstreamEvent::TEARDOWN:
		onChannelTeardown(channel);
		break;
	case Channel::DownstreamEvent::SOS:
		onChannelSos(channel);
		break;
	case Channel::DownstreamEvent::EOS:
		onChannelEos(channel);
		break;
	case Channel::DownstreamEvent::RECONFIGURE:
		onChannelReconfigure(channel);
		break;
	case Channel::DownstreamEvent::TIMEOUT:
		onChannelTimeout(channel);
		break;
	case Channel::DownstreamEvent::PHOTO_TRIGGER:
		onChannelPhotoTrigger(channel);
		break;
	default:
		ULOG_ERRNO("event id %d", ENOSYS, pomp_msg_get_id(event));
		break;
	}
}

} /* namespace Pdraw */
