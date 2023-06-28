/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline media sink for elements
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

#define ULOG_TAG pdraw_sink
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_sink.hpp"

#include <errno.h>

namespace Pdraw {


Sink::Sink(unsigned int maxInputMedias,
	   const struct vdef_coded_format *codedVideoMediaFormatCaps,
	   int codedVideoMediaFormatCapsCount,
	   const struct vdef_raw_format *rawVideoMediaFormatCaps,
	   int rawVideoMediaFormatCapsCount) :
		mMaxInputMedias(maxInputMedias),
		mCodedVideoMediaFormatCaps(codedVideoMediaFormatCaps),
		mCodedVideoMediaFormatCapsCount(codedVideoMediaFormatCapsCount),
		mRawVideoMediaFormatCaps(rawVideoMediaFormatCaps),
		mRawVideoMediaFormatCapsCount(rawVideoMediaFormatCapsCount)
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
						  : nullptr;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Sink::findInputMedia(Media *media)
{
	pthread_mutex_lock(&mMutex);
	Media *ret = nullptr;
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


int Sink::addInputMedia(Media *media)
{
	InputPort port = {};

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

	CodedVideoMedia *cvmedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rvmedia = dynamic_cast<RawVideoMedia *>(media);

	if (cvmedia != nullptr) {
		/* Coded video media */
		if (!vdef_coded_format_intersect(
			    &cvmedia->format,
			    mCodedVideoMediaFormatCaps,
			    mCodedVideoMediaFormatCapsCount)) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("%s: coded video media"
			      " format " VDEF_CODED_FORMAT_TO_STR_FMT
			      " not supported",
			      getName().c_str(),
			      VDEF_CODED_FORMAT_TO_STR_ARG(&cvmedia->format));
			return -ENOSYS;
		}

		port.media = cvmedia;
		CodedVideoChannel *channel =
			new CodedVideoChannel(this, this, this);
		if (channel == nullptr) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		channel->setCodedVideoMediaFormatCaps(
			this,
			mCodedVideoMediaFormatCaps,
			mCodedVideoMediaFormatCapsCount);
		port.channel = channel;
	} else if (rvmedia != nullptr) {
		/* Raw video media */
		if (!vdef_raw_format_intersect(&rvmedia->format,
					       mRawVideoMediaFormatCaps,
					       mRawVideoMediaFormatCapsCount)) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("raw video media"
			      " format " VDEF_RAW_FORMAT_TO_STR_FMT
			      " not supported",
			      VDEF_RAW_FORMAT_TO_STR_ARG(&rvmedia->format));
			return -ENOSYS;
		}

		port.media = rvmedia;
		RawVideoChannel *channel =
			new RawVideoChannel(this, this, this);
		if (channel == nullptr) {
			pthread_mutex_unlock(&mMutex);
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		channel->setRawVideoMediaFormatCaps(
			this,
			mRawVideoMediaFormatCaps,
			mRawVideoMediaFormatCapsCount);
		port.channel = channel;
	} else {
		pthread_mutex_unlock(&mMutex);
		ULOGE("unsupported media type");
		return -ENOSYS;
	}

	mInputPorts.push_back(port);
	pthread_mutex_unlock(&mMutex);

	ULOGI("%s: link media name=%s",
	      getName().c_str(),
	      media->getName().c_str());
	return 0;
}


int Sink::removeInputMedia(Media *media)
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


int Sink::removeInputMedias(void)
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


Channel *Sink::getInputChannel(Media *media)
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

	Channel *ret = port->channel;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Sink::onCodedVideoChannelQueue(CodedVideoChannel *channel,
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
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue(this);
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


void Sink::onRawVideoChannelQueue(RawVideoChannel *channel,
				  struct mbuf_raw_video_frame *frame)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		ULOG_ERRNO("frame", EINVAL);
		return;
	}
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		ULOGE("invalid queue");
		return;
	}

	int ret = mbuf_raw_video_frame_queue_push(queue, frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_push", -ret);
		return;
	}
}


void Sink::onChannelTeardown(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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


void Sink::onChannelSos(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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

	ULOGD("%s: channel SOS media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelEos(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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

	ULOGD("%s: channel EOS media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelReconfigure(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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

	ULOGD("%s: channel reconfigure media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelTimeout(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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

	ULOGD("%s: channel timeout media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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
	      "media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelSessionMetaUpdate(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);
	Media *media = nullptr;
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

	ULOGD("%s: channel session_meta_update "
	      "media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */

	pthread_mutex_unlock(&mMutex);
}


void Sink::onChannelDownstreamEvent(Channel *channel,
				    const struct pomp_msg *event)
{
	ULOGD("%s: channel downstream event %s",
	      getName().c_str(),
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
	case Channel::DownstreamEvent::SESSION_META_UPDATE:
		onChannelSessionMetaUpdate(channel);
		break;
	default:
		ULOG_ERRNO("event id %d", ENOSYS, pomp_msg_get_id(event));
		break;
	}
}

} /* namespace Pdraw */
