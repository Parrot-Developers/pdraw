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

#ifndef _PDRAW_ELEMENT_HPP_
#define _PDRAW_ELEMENT_HPP_

#include "pdraw_sink.hpp"
#include "pdraw_source.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>

#include <atomic>
#include <string>
#include <vector>

namespace Pdraw {

class Session;
class ElementWrapper;


class Element : public Loggable {
public:
	enum State {
		INVALID,
		CREATED,
		STARTING,
		STARTED,
		STOPPING,
		STOPPED,
	};

	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onElementStateChanged(Element *element,
						   Element::State state) = 0;

		virtual void asyncElementStateChange(Element *element,
						     Element::State state) = 0;
	};

	virtual ~Element(void);

	virtual int start(void) = 0;

	virtual int stop(void) = 0;

	unsigned int getId(void);

	ElementWrapper *getWrapper(void);

	void clearWrapper(void);

	Element::State getState(void);

	static const char *getElementStateStr(Element::State val);

protected:
	Element(Session *session, Listener *listener, ElementWrapper *wrapper);

	void setClassName(std::string &name);

	void setClassName(const char *name);

	void setState(Element::State state);

	void setStateAsyncNotify(Element::State state);

	Session *mSession;
	Listener *mListener;
	ElementWrapper *mWrapper;
	std::atomic<Element::State> mState;
	unsigned int mId;
	static std::atomic<unsigned int> mIdCounter;
};


class ElementWrapper {
public:
	ElementWrapper(void);

	virtual ~ElementWrapper(void);

	Element *getElement(void) const;

	virtual void clearElement(void);

protected:
	Element *mElement;
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

	virtual ~SourceElement(void) {}

protected:
	std::string &getName(void)
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

	virtual ~SinkElement(void) {}

protected:
	std::string &getName(void)
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

	virtual ~FilterElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}

	virtual void onChannelSos(Channel *channel);

	virtual void onChannelEos(Channel *channel);

	virtual void onChannelReconfigure(Channel *channel);

	virtual void onChannelResolutionChange(Channel *channel);

	virtual void onChannelFramerateChange(Channel *channel);

	virtual void onChannelTimeout(Channel *channel);

	virtual void onChannelPhotoTrigger(Channel *channel);

	virtual void onChannelSessionMetaUpdate(Channel *channel);

	virtual void onChannelVideoPresStats(Channel *channel,
					     VideoPresStats *stats);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ELEMENT_HPP_ */
