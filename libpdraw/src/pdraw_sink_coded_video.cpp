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

#define ULOG_TAG pdraw_sink_coded_video
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_sink_coded_video.hpp"

#include <errno.h>

namespace Pdraw {


CodedSink::CodedSink(unsigned int maxInputMedias,
		     const struct vdef_coded_format *codedVideoMediaFormatCaps,
		     int codedVideoMediaFormatCapsCount) :
		mMaxInputMedias(maxInputMedias),
		mCodedVideoMediaFormatCaps(codedVideoMediaFormatCaps),
		mCodedVideoMediaFormatCapsCount(codedVideoMediaFormatCapsCount)
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


CodedSink::~CodedSink(void)
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


void CodedSink::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void CodedSink::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


unsigned int CodedSink::getInputMediaCount(void)
{
	pthread_mutex_lock(&mMutex);
	unsigned int ret = mInputPorts.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


CodedVideoMedia *CodedSink::getInputMedia(unsigned int index)
{
	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *ret = (index < mInputPorts.size())
				       ? mInputPorts.at(index).media
				       : nullptr;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


CodedVideoMedia *CodedSink::findInputMedia(CodedVideoMedia *media)
{
	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *ret = nullptr;
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


CodedSink::InputPort *CodedSink::getInputPort(CodedVideoMedia *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	pthread_mutex_lock(&mMutex);
	InputPort *ret = nullptr;
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


int CodedSink::addInputMedia(CodedVideoMedia *media)
{
	if (media == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	if (getInputPort(media) != nullptr) {
		pthread_mutex_unlock(&mMutex);
		return -EEXIST;
	}
	if (mInputPorts.size() >= mMaxInputMedias) {
		pthread_mutex_unlock(&mMutex);
		return -ENOBUFS;
	}
	if (!vdef_coded_format_intersect(&media->format,
					 mCodedVideoMediaFormatCaps,
					 mCodedVideoMediaFormatCapsCount)) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: coded video media"
		      " format " VDEF_CODED_FORMAT_TO_STR_FMT " not supported",
		      getName().c_str(),
		      VDEF_CODED_FORMAT_TO_STR_ARG(&media->format));
		return -ENOSYS;
	}

	InputPort port;
	memset(&port, 0, sizeof(port));
	port.media = media;
	port.channel = new CodedChannel(this);
	if (port.channel == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("failed to create channel");
		return -ENOMEM;
	}
	port.channel->setCodedVideoMediaFormatCaps(
		mCodedVideoMediaFormatCaps, mCodedVideoMediaFormatCapsCount);

	mInputPorts.push_back(port);
	pthread_mutex_unlock(&mMutex);

	ULOGI("%s: link media name=%s",
	      getName().c_str(),
	      media->getName().c_str());
	return 0;
}


int CodedSink::removeInputMedia(CodedVideoMedia *media)
{
	if (media == nullptr)
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
		ULOGI("%s: unlink media name=%s",
		      getName().c_str(),
		      media->getName().c_str());
		int ret = p->channel->unlink();
		if (ret < 0)
			ULOG_ERRNO("channel->unlink", -ret);
		delete p->channel;
		p->channel = nullptr;
		mInputPorts.erase(p);
		break;
	}

	pthread_mutex_unlock(&mMutex);
	if (!found)
		return -ENOENT;

	return 0;
}


int CodedSink::removeInputMedias(void)
{
	pthread_mutex_lock(&mMutex);
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		ULOGI("%s: unlink media name=%s",
		      getName().c_str(),
		      p->media->getName().c_str());
		int ret = p->channel->unlink();
		if (ret < 0)
			ULOG_ERRNO("channel->unlink", -ret);
		delete p->channel;
		p->channel = nullptr;
		p++;
	}

	mInputPorts.clear();

	pthread_mutex_unlock(&mMutex);
	return 0;
}


CodedChannel *CodedSink::getInputChannel(CodedVideoMedia *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	pthread_mutex_lock(&mMutex);
	InputPort *port = getInputPort(media);
	if (port == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}

	CodedChannel *ret = port->channel;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void CodedSink::onChannelQueue(CodedChannel *channel,
			       struct mbuf_coded_video_frame *frame)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		ULOG_ERRNO("frame", EINVAL);
		return;
	}
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue();
	if (queue == nullptr) {
		ULOGE("invalid queue");
		return;
	}

	int ret = mbuf_coded_video_frame_queue_push(queue, frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -ret);
		return;
	}
}


void CodedSink::onChannelFlush(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue();
	if (queue == nullptr) {
		ULOGE("invalid queue");
		return;
	}

	int ret = mbuf_coded_video_frame_queue_flush(queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_flush", -ret);
		return;
	}

	ret = channel->flushDone();
	if (ret < 0)
		ULOG_ERRNO("channel->flushDone", -ret);
}


void CodedSink::onChannelTeardown(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
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


void CodedSink::onChannelSos(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel SOS media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void CodedSink::onChannelEos(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel EOS media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void CodedSink::onChannelReconfigure(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel reconfigure media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void CodedSink::onChannelTimeout(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel timeout media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void CodedSink::onChannelPhotoTrigger(CodedChannel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	CodedVideoMedia *media = nullptr;
	std::vector<InputPort>::iterator p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel photo_trigger "
	      "media name=%s (channel key=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getKey());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void CodedSink::onChannelDownstreamEvent(CodedChannel *channel,
					 const struct pomp_msg *event)
{
	ULOGD("%s: channel downstream event %s",
	      getName().c_str(),
	      CodedChannel::getDownstreamEventStr(
		      (CodedChannel::DownstreamEvent)pomp_msg_get_id(event)));

	switch (pomp_msg_get_id(event)) {
	case CodedChannel::DownstreamEvent::FLUSH:
		onChannelFlush(channel);
		break;
	case CodedChannel::DownstreamEvent::TEARDOWN:
		onChannelTeardown(channel);
		break;
	case CodedChannel::DownstreamEvent::SOS:
		onChannelSos(channel);
		break;
	case CodedChannel::DownstreamEvent::EOS:
		onChannelEos(channel);
		break;
	case CodedChannel::DownstreamEvent::RECONFIGURE:
		onChannelReconfigure(channel);
		break;
	case CodedChannel::DownstreamEvent::TIMEOUT:
		onChannelTimeout(channel);
		break;
	case CodedChannel::DownstreamEvent::PHOTO_TRIGGER:
		onChannelPhotoTrigger(channel);
		break;
	default:
		ULOG_ERRNO("event id %d", ENOSYS, pomp_msg_get_id(event));
		break;
	}
}

} /* namespace Pdraw */
