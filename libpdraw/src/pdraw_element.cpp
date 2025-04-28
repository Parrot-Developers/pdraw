/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline element
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

#define ULOG_TAG pdraw_elmt
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_element.hpp"
#include "pdraw_session.hpp"

#include <errno.h>

namespace Pdraw {


std::atomic<unsigned int> Element::mIdCounter(0);


Element::Element(Session *session,
		 Listener *listener,
		 ElementWrapper *wrapper) :
		mSession(session),
		mListener(listener), mWrapper(wrapper), mState(INVALID)
{
	mId = ++mIdCounter;
	std::string name = std::string(__func__) + "#" + std::to_string(mId);
	Loggable::setName(name);
}


Element::~Element(void)
{
	mState = INVALID;

	/* Clear the element in the element wrapper */
	if (mWrapper != nullptr)
		mWrapper->clearElement();

	PDRAW_LOGI("element DESTROYED");
}


unsigned int Element::getId(void)
{
	return mId;
}


ElementWrapper *Element::getWrapper(void)
{
	return mWrapper;
}


void Element::clearWrapper(void)
{
	mWrapper = nullptr;
}


void Element::setClassName(std::string &name)
{
	std::string new_name = name + "#" + std::to_string(mId);
	Loggable::setName(new_name);
}


void Element::setClassName(const char *name)
{
	std::string new_name = std::string(name) + "#" + std::to_string(mId);
	Loggable::setName(new_name);
}


Element::State Element::getState(void)
{
	return mState;
}


void Element::setState(Element::State state)
{
	Element::State old = mState.exchange(state);

	if (old == state)
		return;

	PDRAW_LOGI("element state change to %s", getElementStateStr(state));

	if (mListener)
		mListener->onElementStateChanged(this, state);
}


void Element::setStateAsyncNotify(Element::State state)
{
	Element::State old = mState.exchange(state);

	if (old == state)
		return;

	PDRAW_LOGI("element state change to %s (async notify)",
		   getElementStateStr(state));

	if (mListener)
		mListener->asyncElementStateChange(this, state);
}


const char *Element::getElementStateStr(Element::State val)
{
	switch (val) {
	case Element::State::INVALID:
		return "INVALID";
	case Element::State::CREATED:
		return "CREATED";
	case Element::State::STARTING:
		return "STARTING";
	case Element::State::STARTED:
		return "STARTED";
	case Element::State::STOPPING:
		return "STOPPING";
	case Element::State::STOPPED:
		return "STOPPED";
	default:
		return nullptr;
	}
}


ElementWrapper::ElementWrapper(void) : mElement(nullptr)
{
	return;
}


ElementWrapper::~ElementWrapper(void)
{
	/* Clear the element wrapper in the element */
	if (mElement != nullptr)
		mElement->clearWrapper();
}


Element *ElementWrapper::getElement(void) const
{
	return mElement;
}


void ElementWrapper::clearElement(void)
{
	mElement = nullptr;
}


void FilterElement::onChannelSos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelSos(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::SOS);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelEos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelEos(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::EOS);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelReconfigure(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelReconfigure(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::RECONFIGURE);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelResolutionChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelResolutionChange(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::RESOLUTION_CHANGE);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelFramerateChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelFramerateChange(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::FRAMERATE_CHANGE);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelTimeout(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelTimeout(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::TIMEOUT);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelPhotoTrigger(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::PHOTO_TRIGGER);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelSessionMetaUpdate(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelSessionMetaUpdate(channel);

	Source::lock();
	unsigned int count = Source::getOutputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr)
			continue;
		int ret = Source::sendDownstreamEvent(
			media, Channel::DownstreamEvent::SESSION_META_UPDATE);
		if (ret < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void FilterElement::onChannelVideoPresStats(Channel *channel,
					    VideoPresStats *stats)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelVideoPresStats(channel, stats);

	/* Propagate the video presentation statistics to all
	 * input media channels */
	Sink::lock();
	unsigned int count = Sink::getInputMediaCount();
	for (unsigned int i = 0; i < count; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr)
			continue;
		Channel *inChannel = getInputChannel(media);
		if (inChannel == nullptr)
			continue;
		int err = inChannel->sendVideoPresStats(stats);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendVideoPresStats", -err);
	}
	Sink::unlock();
}

} /* namespace Pdraw */
