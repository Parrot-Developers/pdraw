/**
 * Parrot Drones Awesome Video Viewer Library
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

#include "pdraw_channel_coded_video.hpp"
#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"
#include "pdraw_sink_coded_video.hpp"
#include "pdraw_sink_raw_video.hpp"
#include "pdraw_source_coded_video.hpp"
#include "pdraw_source_raw_video.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>
#include <pthread.h>

#include <atomic>
#include <string>
#include <vector>

namespace Pdraw {

class Session;

class Element : public Loggable {
public:
	friend class Session;

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

	void lock(void);

	void unlock(void);

	unsigned int getId(void);

	Element::State getState(void);

	static const char *getElementStateStr(Element::State val);

protected:
	Element(Session *session, Listener *listener);

	void setClassName(std::string &name);

	void setClassName(const char *name);

	void setState(Element::State state);

	void setStateAsyncNotify(Element::State state);

	Session *mSession;
	Listener *mListener;
	Element::State mState;
	unsigned int mId;
	pthread_mutex_t mMutex;
	static std::atomic<unsigned int> mIdCounter;
};


class CodedSourceElement : public Element, public CodedSource {
public:
	friend class Session;

	CodedSourceElement(Session *session,
			   Element::Listener *listener,
			   unsigned int maxOutputMedias,
			   CodedSource::Listener *sourceListener) :
			Element(session, listener),
			CodedSource(maxOutputMedias, sourceListener)
	{
	}

	virtual ~CodedSourceElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}
};


class RawSourceElement : public Element, public RawSource {
public:
	friend class Session;

	RawSourceElement(Session *session,
			 Element::Listener *listener,
			 unsigned int maxOutputMedias,
			 RawSource::Listener *sourceListener) :
			Element(session, listener),
			RawSource(maxOutputMedias, sourceListener)
	{
	}

	virtual ~RawSourceElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}
};


class CodedSinkElement : public Element, public CodedSink {
public:
	friend class Session;

	CodedSinkElement(
		Session *session,
		Element::Listener *listener,
		unsigned int maxInputMedias,
		const struct vdef_coded_format *codedVideoMediaFormatCaps,
		int codedVideoMediaFormatCapsCount) :
			Element(session, listener),
			CodedSink(maxInputMedias,
				  codedVideoMediaFormatCaps,
				  codedVideoMediaFormatCapsCount)
	{
	}

	virtual ~CodedSinkElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}
};


class RawSinkElement : public Element, public RawSink {
public:
	friend class Session;

	RawSinkElement(Session *session,
		       Element::Listener *listener,
		       unsigned int maxInputMedias,
		       const struct vdef_raw_format *rawVideoMediaFormatCaps,
		       int rawVideoMediaFormatCapsCount) :
			Element(session, listener),
			RawSink(maxInputMedias,
				rawVideoMediaFormatCaps,
				rawVideoMediaFormatCapsCount)
	{
	}

	virtual ~RawSinkElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}
};


class CodedToRawFilterElement : public Element,
				public CodedSink,
				public RawSource {
public:
	friend class Session;

	CodedToRawFilterElement(
		Session *session,
		Element::Listener *listener,
		unsigned int maxInputMedias,
		const struct vdef_coded_format *codedVideoMediaFormatCaps,
		int codedVideoMediaFormatCapsCount,
		unsigned int maxOutputMedias,
		RawSource::Listener *sourceListener) :
			Element(session, listener),
			CodedSink(maxInputMedias,
				  codedVideoMediaFormatCaps,
				  codedVideoMediaFormatCapsCount),
			RawSource(maxOutputMedias, sourceListener)
	{
	}

	virtual ~CodedToRawFilterElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}

	virtual void onChannelSos(CodedChannel *channel);

	virtual void onChannelEos(CodedChannel *channel);

	virtual void onChannelReconfigure(CodedChannel *channel);

	virtual void onChannelTimeout(CodedChannel *channel);

	virtual void onChannelPhotoTrigger(CodedChannel *channel);

	virtual void onChannelVideoPresStats(RawChannel *channel,
					     VideoPresStats *stats);
};


class RawToCodedFilterElement : public Element,
				public RawSink,
				public CodedSource {
public:
	friend class Session;

	RawToCodedFilterElement(
		Session *session,
		Element::Listener *listener,
		unsigned int maxInputMedias,
		const struct vdef_raw_format *rawVideoMediaFormatCaps,
		int rawVideoMediaFormatCapsCount,
		unsigned int maxOutputMedias,
		CodedSource::Listener *sourceListener) :
			Element(session, listener),
			RawSink(maxInputMedias,
				rawVideoMediaFormatCaps,
				rawVideoMediaFormatCapsCount),
			CodedSource(maxOutputMedias, sourceListener)
	{
	}

	virtual ~RawToCodedFilterElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}

	virtual void onChannelSos(RawChannel *channel);

	virtual void onChannelEos(RawChannel *channel);

	virtual void onChannelReconfigure(RawChannel *channel);

	virtual void onChannelTimeout(RawChannel *channel);

	virtual void onChannelPhotoTrigger(RawChannel *channel);

	virtual void onChannelVideoPresStats(CodedChannel *channel,
					     VideoPresStats *stats);
};


class RawToRawFilterElement : public Element, public RawSink, public RawSource {
public:
	friend class Session;

	RawToRawFilterElement(
		Session *session,
		Element::Listener *listener,
		unsigned int maxInputMedias,
		const struct vdef_raw_format *rawVideoMediaFormatCaps,
		int rawVideoMediaFormatCapsCount,
		unsigned int maxOutputMedias,
		RawSource::Listener *sourceListener) :
			Element(session, listener),
			RawSink(maxInputMedias,
				rawVideoMediaFormatCaps,
				rawVideoMediaFormatCapsCount),
			RawSource(maxOutputMedias, sourceListener)
	{
	}

	virtual ~RawToRawFilterElement(void) {}

protected:
	std::string &getName(void)
	{
		return Element::getName();
	}

	virtual void onChannelSos(RawChannel *channel);

	virtual void onChannelEos(RawChannel *channel);

	virtual void onChannelReconfigure(RawChannel *channel);

	virtual void onChannelTimeout(RawChannel *channel);

	virtual void onChannelPhotoTrigger(RawChannel *channel);

	virtual void onChannelVideoPresStats(RawChannel *channel,
					     VideoPresStats *stats);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ELEMENT_HPP_ */
