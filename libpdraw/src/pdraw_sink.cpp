/**
 * Parrot Drones Audio and Video Vector library
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

#include "pdraw_session.hpp"
#include "pdraw_sink.hpp"

#include <errno.h>

namespace Pdraw {


Sink::Sink(const Session *session,
	   unsigned int maxInputMedias,
	   const struct vdef_coded_format *codedVideoMediaFormatCaps,
	   int codedVideoMediaFormatCapsCount,
	   const struct vdef_raw_format *rawVideoMediaFormatCaps,
	   int rawVideoMediaFormatCapsCount,
	   const struct adef_format *audioMediaFormatCaps,
	   int audioMediaFormatCapsCount) :
		mLoop(session ? session->getLoop() : nullptr),
		mMaxInputMedias(maxInputMedias),
		mCodedVideoMediaFormatCaps(codedVideoMediaFormatCaps),
		mCodedVideoMediaFormatCapsCount(codedVideoMediaFormatCapsCount),
		mRawVideoMediaFormatCaps(rawVideoMediaFormatCaps),
		mRawVideoMediaFormatCapsCount(rawVideoMediaFormatCapsCount),
		mAudioMediaFormatCaps(audioMediaFormatCaps),
		mAudioMediaFormatCapsCount(audioMediaFormatCapsCount)
{
}


Sink::~Sink()
{
	int ret = removeInputMedias();
	if (ret < 0)
		ULOG_ERRNO("removeInputMedias", -ret);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		ULOGW("not all input ports have been removed! (count=%d)",
		      count);
	}
}


void Sink::lock()
{
	mMutex.lock();
}


void Sink::unlock()
{
	mMutex.unlock();
}


unsigned int Sink::getInputMediaCount()
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	auto ret = static_cast<unsigned int>(mInputPorts.size());
	return ret;
}


Media *Sink::getInputMedia(unsigned int index)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	Media *ret = (index < mInputPorts.size()) ? mInputPorts.at(index).media
						  : nullptr;
	return ret;
}


Media *Sink::findInputMedia(const Media *media)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	Media *ret = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = p->media;
		break;
	}
	return ret;
}


Sink::InputPort *Sink::getInputPort(const Media *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	InputPort *ret = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->media != media) {
			p++;
			continue;
		}
		ret = &(*p);
		break;
	}

	return ret;
}


int Sink::addInputMedia(Media *media)
{
	InputPort port = {};

	if (media == nullptr)
		return -EINVAL;
	if (media->isTearingDown())
		return -EPERM;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	if (getInputPort(media) != nullptr) {
		return -EEXIST;
	}
	if (mInputPorts.size() >= mMaxInputMedias) {
		return -ENOBUFS;
	}

	auto *cvmedia = dynamic_cast<CodedVideoMedia *>(media);
	auto *rvmedia = dynamic_cast<RawVideoMedia *>(media);
	auto *amedia = dynamic_cast<AudioMedia *>(media);

	if (cvmedia != nullptr) {
		/* Coded video media */
		if (!vdef_coded_format_intersect(
			    &cvmedia->format,
			    mCodedVideoMediaFormatCaps,
			    mCodedVideoMediaFormatCapsCount)) {
			ULOGE("%s: coded video media"
			      " format " VDEF_CODED_FORMAT_TO_STR_FMT
			      " not supported",
			      getName().c_str(),
			      VDEF_CODED_FORMAT_TO_STR_ARG(&cvmedia->format));
			return -ENOSYS;
		}

		std::unique_ptr<CodedVideoChannel> channel;
		port.media = cvmedia;
		try {
			channel = make_unique<CodedVideoChannel>(
				this, this, this, mLoop);
		} catch (const std::bad_alloc &) {
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		channel->setCodedVideoMediaFormatCaps(
			this,
			mCodedVideoMediaFormatCaps,
			mCodedVideoMediaFormatCapsCount);
		port.channel = std::move(channel);
	} else if (rvmedia != nullptr) {
		/* Raw video media */
		if (!vdef_raw_format_intersect(&rvmedia->format,
					       mRawVideoMediaFormatCaps,
					       mRawVideoMediaFormatCapsCount)) {
			ULOGE("=> raw video media"
			      " format " VDEF_RAW_FORMAT_TO_STR_FMT
			      " not supported (count: %d)",
			      VDEF_RAW_FORMAT_TO_STR_ARG(&rvmedia->format),
			      mRawVideoMediaFormatCapsCount);
			return -ENOSYS;
		}

		std::unique_ptr<RawVideoChannel> channel;
		port.media = rvmedia;
		try {
			channel = make_unique<RawVideoChannel>(
				this, this, this, mLoop);
		} catch (const std::bad_alloc &) {
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		channel->setRawVideoMediaFormatCaps(
			this,
			mRawVideoMediaFormatCaps,
			mRawVideoMediaFormatCapsCount);
		port.channel = std::move(channel);
	} else if (amedia != nullptr) {
		/* Audio media */
		if (!adef_format_intersect(&amedia->format,
					   mAudioMediaFormatCaps,
					   mAudioMediaFormatCapsCount)) {
			ULOGE("audio media"
			      " format " ADEF_FORMAT_TO_STR_FMT
			      " not supported",
			      ADEF_FORMAT_TO_STR_ARG(&amedia->format));
			return -ENOSYS;
		}

		std::unique_ptr<AudioChannel> channel;
		port.media = amedia;
		try {
			channel = make_unique<AudioChannel>(
				this, this, this, mLoop);
		} catch (const std::bad_alloc &) {
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		if (channel == nullptr) {
			ULOGE("failed to create channel");
			return -ENOMEM;
		}
		channel->setAudioMediaFormatCaps(this,
						 mAudioMediaFormatCaps,
						 mAudioMediaFormatCapsCount);
		port.channel = std::move(channel);
	} else {
		ULOGE("unsupported media type");
		return -ENOSYS;
	}

	mInputPorts.push_back(std::move(port));

	ULOGI("%s: link media name=%s",
	      getName().c_str(),
	      media->getName().c_str());
	return 0;
}


int Sink::removeInputMedia(Media *media)
{
	if (media == nullptr)
		return -EINVAL;

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	bool found = false;
	auto p = mInputPorts.begin();

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
		p->channel.reset();
		mInputPorts.erase(p);
		break;
	}

	if (!found)
		return -ENOENT;

	return 0;
}


int Sink::removeInputMedias()
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		ULOGI("%s: unlink media name=%s",
		      getName().c_str(),
		      p->media->getName().c_str());
		int ret = p->channel->unlink();
		if (ret < 0)
			ULOG_ERRNO("channel->unlink", -ret);
		p->channel.reset();
		p++;
	}

	mInputPorts.clear();

	return 0;
}


Channel *Sink::getInputChannel(const Media *media)
{
	if (media == nullptr) {
		ULOG_ERRNO("media", EINVAL);
		return nullptr;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	InputPort *port = getInputPort(media);
	if (port == nullptr) {
		ULOG_ERRNO("port", ENOENT);
		return nullptr;
	}

	Channel *ret = port->channel.get();
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

	mbuf::Queue *queue = channel->getQueue(this);
	if (queue == nullptr)
		return;
	int err = queue->pushFrame(frame);
	if (err < 0) {
		ULOG_ERRNO("queue::pushFrame", -err);
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
	mbuf::Queue *queue = channel->getQueue(this);
	if (queue == nullptr)
		return;
	int err = queue->pushFrame(frame);
	if (err < 0) {
		ULOG_ERRNO("queue::pushFrame", -err);
		return;
	}
}


void Sink::onAudioChannelQueue(AudioChannel *channel,
			       struct mbuf_audio_frame *frame)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		ULOG_ERRNO("frame", EINVAL);
		return;
	}

	mbuf::Queue *queue = channel->getQueue(this);
	if (queue == nullptr)
		return;
	int err = queue->pushFrame(frame);
	if (err < 0) {
		ULOG_ERRNO("queue::pushFrame", -err);
		return;
	}
}


void Sink::onChannelTeardown(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);

	Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	int ret = removeInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("removeInputMedia", -ret);
		return;
	}
}


void Sink::onChannelSos(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel SOS media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelEos(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel EOS media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelReconfigure(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel reconfigure media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelResolutionChange(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel resolution change media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelFramerateChange(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel framerate change media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelTimeout(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
		ULOG_ERRNO("media", ENOENT);
		return;
	}

	ULOGD("%s: channel timeout media name=%s (channel owner=%p)",
	      getName().c_str(),
	      media->getName().c_str(),
	      channel->getOwner());

	/* Nothing to do here, the function should be
	 * overloaded by sub-classes */
}


void Sink::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
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
}


void Sink::onChannelSessionMetaUpdate(Channel *channel)
{
	if (channel == nullptr) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	std::unique_lock<std::recursive_mutex> lock(mMutex);
	const Media *media = nullptr;
	auto p = mInputPorts.begin();

	while (p != mInputPorts.end()) {
		if (p->channel.get() != channel) {
			p++;
			continue;
		}
		media = p->media;
		break;
	}

	if (media == nullptr) {
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
}


void Sink::onChannelDownstreamEvent(Channel *channel,
				    const struct pomp_msg *event)
{
	ULOGD("%s: channel downstream event %s",
	      getName().c_str(),
	      Channel::getDownstreamEventStr(
		      static_cast<Channel::DownstreamEvent>(
			      pomp_msg_get_id(event))));

	switch (static_cast<Channel::DownstreamEvent>(pomp_msg_get_id(event))) {
	case Channel::DownstreamEvent::FLUSH:
		onChannelFlush(channel);
		break;
	case Channel::DownstreamEvent::DRAIN:
		onChannelDrain(channel);
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
	case Channel::DownstreamEvent::RESOLUTION_CHANGE:
		onChannelResolutionChange(channel);
		break;
	case Channel::DownstreamEvent::FRAMERATE_CHANGE:
		onChannelFramerateChange(channel);
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
