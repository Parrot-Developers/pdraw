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

#pragma once

#include "pdraw_sink.hpp"
#include "pdraw_source.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>

#include <atomic>
#include <climits>
#include <string>
#include <vector>

namespace Pdraw {

class Session;
class ElementWrapper;


class Element : public Loggable {
	PDRAW_DISABLE_COPY(Element)

public:
	enum class State {
		INVALID,
		CREATED,
		STARTING,
		STARTED,
		STOPPING,
		STOPPED,
	};

	enum class FlushingState {
		UNFLUSHED,
		FLUSHING,
		FLUSHED,
	};

	class Listener {
	public:
		virtual ~Listener() = default;

		virtual void onElementStateChanged(Element *element,
						   Element::State state) = 0;

		virtual void asyncElementStateChange(Element *element,
						     Element::State state) = 0;
	};

	~Element() override;

	virtual int start() = 0;

	virtual int stop() = 0;

	unsigned int getId() const;

	ElementWrapper *getWrapper() const;

	void clearWrapper();

	Element::State getState() const;

	Element::FlushingState getFlushingState() const;

	bool isFlushing() const
	{
		return (mFlushingState == Element::FlushingState::FLUSHING) &&
		       isFlushDiscard();
	}

	bool isDraining() const
	{
		return (mFlushingState == Element::FlushingState::FLUSHING) &&
		       !isFlushDiscard();
	}

	static const char *getElementStateStr(Element::State val);

	static const char *
	getElementFlushingStateStr(Element::FlushingState val);

protected:
	Element(Session *session, Listener *listener, ElementWrapper *wrapper);

	void setClassName(const std::string &name);

	void setClassName(const char *name);

	void setState(Element::State state);

	void setFlushingState(Element::FlushingState substate,
			      bool discard = true);

	void setStateAsyncNotify(Element::State state);

	bool isFlushDiscard() const
	{
		return mFlushDiscard;
	}

	Session *mSession = nullptr;
	Listener *mListener = nullptr;
	ElementWrapper *mWrapper = nullptr;
	std::atomic<Element::State> mState{Element::State::INVALID};
	std::atomic<Element::FlushingState> mFlushingState{
		Element::FlushingState::FLUSHED};
	bool mFlushDiscard = false;
	unsigned int mId = UINT_MAX;
	static std::atomic<unsigned int> mIdCounter;
};


class ElementWrapper {
	PDRAW_DISABLE_COPY(ElementWrapper)

public:
	ElementWrapper() = default;

	explicit ElementWrapper(Element *element) : mElement(element) {}

	virtual ~ElementWrapper();

	Element *getElement() const;

	virtual void clearElement();

protected:
	virtual bool isElementStopped() const;

	Element *mElement = nullptr;
	bool mElementStopped = false;
};


class SourceElement : public Element, public Source {
public:
	SourceElement(Session *session,
		      Element::Listener *listener,
		      ElementWrapper *wrapper,
		      unsigned int maxOutputMedias,
		      Source::Listener *sourceListener) :
			Element(session, listener, wrapper),
			Source(maxOutputMedias, sourceListener)
	{
	}

	~SourceElement() override = default;

protected:
	const std::string &getName() const override
	{
		return Element::getName();
	}
};


class SinkElement : public Element, public Sink {
public:
	SinkElement(Session *session,
		    Element::Listener *listener,
		    ElementWrapper *wrapper,
		    unsigned int maxInputMedias,
		    const struct vdef_coded_format *codedVideoMediaFormatCaps,
		    int codedVideoMediaFormatCapsCount,
		    const struct vdef_raw_format *rawVideoMediaFormatCaps,
		    int rawVideoMediaFormatCapsCount,
		    const struct adef_format *audioMediaFormatCaps,
		    int audioMediaFormatCapsCount) :
			Element(session, listener, wrapper),
			Sink(session,
			     maxInputMedias,
			     codedVideoMediaFormatCaps,
			     codedVideoMediaFormatCapsCount,
			     rawVideoMediaFormatCaps,
			     rawVideoMediaFormatCapsCount,
			     audioMediaFormatCaps,
			     audioMediaFormatCapsCount)
	{
	}

	~SinkElement() override = default;

protected:
	const std::string &getName() const override
	{
		return Element::getName();
	}
};


class FilterElement : public Element, public Sink, public Source {
public:
	FilterElement(Session *session,
		      Element::Listener *listener,
		      ElementWrapper *wrapper,
		      unsigned int maxInputMedias,
		      const struct vdef_coded_format *codedVideoMediaFormatCaps,
		      int codedVideoMediaFormatCapsCount,
		      const struct vdef_raw_format *rawVideoMediaFormatCaps,
		      int rawVideoMediaFormatCapsCount,
		      const struct adef_format *audioMediaFormatCaps,
		      int audioMediaFormatCapsCount,
		      unsigned int maxOutputMedias,
		      Source::Listener *sourceListener) :
			Element(session, listener, wrapper),
			Sink(session,
			     maxInputMedias,
			     codedVideoMediaFormatCaps,
			     codedVideoMediaFormatCapsCount,
			     rawVideoMediaFormatCaps,
			     rawVideoMediaFormatCapsCount,
			     audioMediaFormatCaps,
			     audioMediaFormatCapsCount),
			Source(maxOutputMedias, sourceListener)
	{
	}

	~FilterElement() override = default;

protected:
	const std::string &getName() const override
	{
		return Element::getName();
	}

	void onChannelSos(Channel *channel) override;

	void onChannelEos(Channel *channel) override;

	void onChannelReconfigure(Channel *channel) override;

	void onChannelResolutionChange(Channel *channel) override;

	void onChannelFramerateChange(Channel *channel) override;

	void onChannelTimeout(Channel *channel) override;

	void onChannelPhotoTrigger(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	void onChannelVideoPresStats(Channel *channel,
				     VideoPresStats *stats) override;
};

} /* namespace Pdraw */
