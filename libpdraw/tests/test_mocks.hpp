/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — test doubles (mocks / stubs)
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

/* NOTE: Include this header only AFTER defining ULOG_TAG. */

#pragma once

/* pdraw_element.hpp includes pdraw_utils.hpp which requires ULOG_TAG. */
#include "pdraw_element.hpp"
#include "pdraw_media.hpp"

#include <vector>

namespace PdrawTest {


/* ── Element listener ────────────────────────────────────────────────── */

struct ElementEvent {
	Pdraw::Element *element;
	Pdraw::Element::State state;
	bool async; /* true if from asyncElementStateChange */
};

class TestElementListener : public Pdraw::Element::Listener {
public:
	void onElementStateChanged(Pdraw::Element *element,
				   Pdraw::Element::State state) override
	{
		mEvents.push_back({element, state, false});
	}

	void asyncElementStateChange(Pdraw::Element *element,
				     Pdraw::Element::State state) override
	{
		mEvents.push_back({element, state, true});
	}

	const std::vector<ElementEvent> &getEvents() const
	{
		return mEvents;
	}

	int eventCount() const
	{
		return (int)mEvents.size();
	}

	Pdraw::Element::State lastState() const
	{
		if (mEvents.empty())
			return Pdraw::Element::State::INVALID;
		return mEvents.back().state;
	}

	bool lastWasAsync() const
	{
		if (mEvents.empty())
			return false;
		return mEvents.back().async;
	}

	void clear()
	{
		mEvents.clear();
	}

private:
	std::vector<ElementEvent> mEvents;
};


/* ── Source listener ─────────────────────────────────────────────────── */

struct SourceEvent {
	Pdraw::Source *source;
	Pdraw::Media *media;
	bool added; /* true = onOutputMediaAdded, false = onOutputMediaRemoved
		     */
};

class TestSourceListener : public Pdraw::Source::Listener {
public:
	void onOutputMediaAdded(Pdraw::Source *source,
				Pdraw::Media *media,
				void * /*elementUserData*/) override
	{
		mEvents.push_back({source, media, true});
	}

	void onOutputMediaRemoved(Pdraw::Source *source,
				  Pdraw::Media *media,
				  void * /*elementUserData*/) override
	{
		mEvents.push_back({source, media, false});
	}

	int addedCount() const
	{
		int n = 0;
		for (auto &e : mEvents)
			if (e.added)
				n++;
		return n;
	}

	int removedCount() const
	{
		int n = 0;
		for (auto &e : mEvents)
			if (!e.added)
				n++;
		return n;
	}

	Pdraw::Media *lastAdded() const
	{
		for (int i = (int)mEvents.size() - 1; i >= 0; --i)
			if (mEvents[i].added)
				return mEvents[i].media;
		return nullptr;
	}

	void clear()
	{
		mEvents.clear();
	}

private:
	std::vector<SourceEvent> mEvents;
};


/* ── Channel listeners ────────────────────────────────────────────────── */

/* Standalone Channel::SinkListener, independent of any real Sink: lets a
 * test observe exactly which downstream event (SOS/EOS/RECONFIGURE/...)
 * arrives on a given output channel, without needing the receiving side to
 * be a fully-wired production Sink (whose onChannelDownstreamEvent()
 * dispatches to per-event virtuals rather than being directly
 * observable). */
class TestChannelSinkListener : public Pdraw::Channel::SinkListener {
public:
	void onChannelDownstreamEvent(Pdraw::Channel * /*channel*/,
				      const pomp::Message &event) override
	{
		mLastEventId = event.getId();
		mEventCount++;
	}

	void clear()
	{
		mLastEventId = 0;
		mEventCount = 0;
	}

	uint32_t mLastEventId = 0;
	int mEventCount = 0;
};


/* Standalone Channel::SourceListener: same rationale as
 * TestChannelSinkListener, for the upstream direction (e.g.
 * FilterElement::onChannelVideoPresStats(), which re-forwards stats
 * upstream via the filter's own INPUT channels). */
class TestChannelSourceListener : public Pdraw::Channel::SourceListener {
public:
	void onChannelUpstreamEvent(Pdraw::Channel * /*channel*/,
				    const pomp::Message &event) override
	{
		mLastEventId = event.getId();
		mEventCount++;
	}

	void clear()
	{
		mLastEventId = 0;
		mEventCount = 0;
	}

	uint32_t mLastEventId = 0;
	int mEventCount = 0;
};


/* ── Concrete Element subclass ───────────────────────────────────────── */

/* Minimal concrete Element used in test_element. start()/stop() transition
 * through the standard states synchronously. */
class TestElement : public Pdraw::Element {
public:
	TestElement(Pdraw::Session *session,
		    Pdraw::Element::Listener *listener) :
			Element(session, listener, nullptr)
	{
		setClassName("TestElement");
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	/* Expose protected setState to tests. */
	void callSetState(Pdraw::Element::State s)
	{
		setState(s);
	}

	/* Expose protected setStateAsyncNotify to tests. */
	void callSetStateAsyncNotify(Pdraw::Element::State s)
	{
		setStateAsyncNotify(s);
	}

	/* Expose protected setFlushingState to tests. */
	void callSetFlushingState(Pdraw::Element::FlushingState s,
				  bool discard = true)
	{
		setFlushingState(s, discard);
	}
};


/* ── SourceElement subclass ──────────────────────────────────────────── */

/* Exposes the protected addOutputPort / removeOutputPort via public wrappers.
 * Source::Listener is optional; pass nullptr if the test doesn't need it. */
class TestSourceElement : public Pdraw::SourceElement {
public:
	TestSourceElement(Pdraw::Session *session,
			  Pdraw::Element::Listener *elemListener,
			  Pdraw::Source::Listener *srcListener,
			  unsigned int maxOutputMedias) :
			SourceElement(session,
				      elemListener,
				      nullptr,
				      maxOutputMedias,
				      srcListener)
	{
		setClassName("TestSourceElement");
		setState(State::CREATED);
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	int callAddOutputPort(Pdraw::Media *media, void *userData = nullptr)
	{
		int ret = addOutputPort(media, userData);
		/* Mimic production elements: notify listener after adding port.
		 */
		if (ret == 0 && Source::mListener)
			Source::mListener->onOutputMediaAdded(
				this, media, userData);
		return ret;
	}

	int callRemoveOutputPort(const Pdraw::Media *media)
	{
		/* Capture userData before the port is destroyed. */
		OutputPort *port = getOutputPort(media);
		void *userData = port ? port->elementUserData : nullptr;
		int ret = removeOutputPort(media);
		/* Mimic production elements: notify listener after removing. */
		if (ret == 0 && Source::mListener)
			Source::mListener->onOutputMediaRemoved(
				this,
				const_cast<Pdraw::Media *>(media),
				userData);
		return ret;
	}

	int callRemoveOutputPorts()
	{
		return removeOutputPorts();
	}

	int callCreateOutputPortMemoryPool(const Pdraw::Media *media,
					   unsigned int count,
					   size_t capacity)
	{
		return createOutputPortMemoryPool(media, count, capacity);
	}

	int callDestroyOutputPortMemoryPool(const Pdraw::Media *media)
	{
		return destroyOutputPortMemoryPool(media);
	}

	void callOnChannelFlushed(Pdraw::Channel *channel)
	{
		onChannelFlushed(channel);
	}

	void callOnChannelDrained(Pdraw::Channel *channel)
	{
		onChannelDrained(channel);
	}

	int
	callCopyCodedVideoOutputFrame(const Pdraw::CodedVideoMedia *srcMedia,
				      struct mbuf_coded_video_frame *srcFrame,
				      Pdraw::CodedVideoMedia *dstMedia,
				      struct mbuf_coded_video_frame **dstFrame)
	{
		return copyCodedVideoOutputFrame(
			srcMedia, srcFrame, dstMedia, dstFrame);
	}
};


/* ── SinkElement subclass ────────────────────────────────────────────── */

/* TestSinkElement accepts H.264 byte-stream coded-video (kTestSinkCodedCaps).
 * Tests that add CodedVideoMedia must set media.format = vdef_h264_byte_stream
 * to satisfy Sink::addInputMedia's format-intersection check. */
extern const struct vdef_coded_format kTestSinkCodedCaps[1];

/* TestSinkElementRaw accepts I420 raw-video (kTestSinkRawCaps). */
extern const struct vdef_raw_format kTestSinkRawCaps[1];

/* TestSinkElementAudio accepts AAC-LC stereo ADTS. */
extern const struct adef_format kTestSinkAudioCaps[1];

class TestSinkElement : public Pdraw::SinkElement {
public:
	TestSinkElement(Pdraw::Session *session,
			Pdraw::Element::Listener *elemListener,
			unsigned int maxInputMedias) :
			SinkElement(session,
				    elemListener,
				    nullptr,
				    maxInputMedias,
				    kTestSinkCodedCaps,
				    1,
				    nullptr,
				    0,
				    nullptr,
				    0)
	{
		setClassName("TestSinkElement");
		setState(State::CREATED);
	}

	~TestSinkElement() override
	{
		/* Sink::~Sink() calls removeInputMedias(), but by then
		 * getName() is pure-virtual (Sink's vtable).  Pre-empt it here
		 * while the full vtable is still in place, matching the
		 * production pattern (see
		 * ExternalCodedVideoSink::~ExternalCodedVideoSink). */
		(void)removeInputMedias();
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	int callRemoveInputMedias()
	{
		return removeInputMedias();
	}

	/* onChannelFlush and onChannelDrain are pure-virtual in Sink. */
	void onChannelFlush(Pdraw::Channel * /*channel*/) override {}
	void onChannelDrain(Pdraw::Channel * /*channel*/) override {}
};


/* ── Raw-video SinkElement subclass ──────────────────────────────────── */

class TestSinkElementRaw : public Pdraw::SinkElement {
public:
	TestSinkElementRaw(Pdraw::Session *session,
			   Pdraw::Element::Listener *elemListener,
			   unsigned int maxInputMedias) :
			SinkElement(session,
				    elemListener,
				    nullptr,
				    maxInputMedias,
				    nullptr,
				    0,
				    kTestSinkRawCaps,
				    1,
				    nullptr,
				    0)
	{
		setClassName("TestSinkElementRaw");
		setState(State::CREATED);
	}

	~TestSinkElementRaw() override
	{
		(void)removeInputMedias();
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	void onChannelFlush(Pdraw::Channel * /*channel*/) override {}
	void onChannelDrain(Pdraw::Channel * /*channel*/) override {}
};


/* ── Audio SinkElement subclass ──────────────────────────────────────── */

class TestSinkElementAudio : public Pdraw::SinkElement {
public:
	TestSinkElementAudio(Pdraw::Session *session,
			     Pdraw::Element::Listener *elemListener,
			     unsigned int maxInputMedias) :
			SinkElement(session,
				    elemListener,
				    nullptr,
				    maxInputMedias,
				    nullptr,
				    0,
				    nullptr,
				    0,
				    kTestSinkAudioCaps,
				    1)
	{
		setClassName("TestSinkElementAudio");
		setState(State::CREATED);
	}

	~TestSinkElementAudio() override
	{
		(void)removeInputMedias();
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	void onChannelFlush(Pdraw::Channel * /*channel*/) override {}
	void onChannelDrain(Pdraw::Channel * /*channel*/) override {}
};


/* ── FilterElement subclass ────────────────────────────────────────────── */

/* A FilterElement is simultaneously a Sink and a Source — the shared base
 * of VideoDecoder/VideoEncoder/VideoScaler/AudioDecoder/AudioEncoder.
 * Accepts H.264 byte-stream coded-video on input (kTestSinkCodedCaps, same
 * as TestSinkElement); output side has no format restriction (mirrors
 * TestSourceElement). */
class TestFilterElement : public Pdraw::FilterElement {
public:
	TestFilterElement(Pdraw::Session *session,
			  Pdraw::Element::Listener *elemListener,
			  Pdraw::Source::Listener *srcListener,
			  unsigned int maxInputMedias,
			  unsigned int maxOutputMedias) :
			FilterElement(session,
				      elemListener,
				      nullptr,
				      maxInputMedias,
				      kTestSinkCodedCaps,
				      1,
				      nullptr,
				      0,
				      nullptr,
				      0,
				      maxOutputMedias,
				      srcListener)
	{
		setClassName("TestFilterElement");
		setState(State::CREATED);
	}

	~TestFilterElement() override
	{
		/* Same reasoning as TestSinkElement::~TestSinkElement(): call
		 * removeInputMedias() before the Sink vtable slice is gone. */
		(void)removeInputMedias();
	}

	int start() override
	{
		setState(State::STARTING);
		setState(State::STARTED);
		return 0;
	}

	int stop() override
	{
		setState(State::STOPPING);
		setState(State::STOPPED);
		return 0;
	}

	int callRemoveInputMedias()
	{
		return removeInputMedias();
	}

	/* onChannelFlush and onChannelDrain are pure-virtual in Sink. */
	void onChannelFlush(Pdraw::Channel * /*channel*/) override {}
	void onChannelDrain(Pdraw::Channel * /*channel*/) override {}

	/* Expose the protected Source-side port management, same wrapper
	 * pattern as TestSourceElement. */
	int callAddOutputPort(Pdraw::Media *media, void *userData = nullptr)
	{
		int ret = addOutputPort(media, userData);
		if (ret == 0 && Source::mListener)
			Source::mListener->onOutputMediaAdded(
				this, media, userData);
		return ret;
	}

	int callRemoveOutputPort(const Pdraw::Media *media)
	{
		OutputPort *port = getOutputPort(media);
		void *userData = port ? port->elementUserData : nullptr;
		int ret = removeOutputPort(media);
		if (ret == 0 && Source::mListener)
			Source::mListener->onOutputMediaRemoved(
				this,
				const_cast<Pdraw::Media *>(media),
				userData);
		return ret;
	}

	/* Expose the protected downstream-event propagation hooks
	 * (FilterElement::onChannelXxx, pdraw_element.cpp): each forwards a
	 * specific event from the Sink side to every channel on every output
	 * media. The channel argument only needs to be non-null (see
	 * Sink::onChannelXxx's early "channel == nullptr" guard); it does not
	 * need to be a real, registered input channel for the OUTPUT
	 * propagation to run, since that part is unconditional. */
	void callOnChannelSos(Pdraw::Channel *channel)
	{
		onChannelSos(channel);
	}
	void callOnChannelEos(Pdraw::Channel *channel)
	{
		onChannelEos(channel);
	}
	void callOnChannelReconfigure(Pdraw::Channel *channel)
	{
		onChannelReconfigure(channel);
	}
	void callOnChannelResolutionChange(Pdraw::Channel *channel)
	{
		onChannelResolutionChange(channel);
	}
	void callOnChannelFramerateChange(Pdraw::Channel *channel)
	{
		onChannelFramerateChange(channel);
	}
	void callOnChannelTimeout(Pdraw::Channel *channel)
	{
		onChannelTimeout(channel);
	}
	void callOnChannelPhotoTrigger(Pdraw::Channel *channel)
	{
		onChannelPhotoTrigger(channel);
	}
	void callOnChannelSessionMetaUpdate(Pdraw::Channel *channel)
	{
		onChannelSessionMetaUpdate(channel);
	}

	/* onChannelVideoPresStats propagates the OTHER way (Source side
	 * receiving from downstream, re-forwarded upstream via the filter's
	 * OWN input channels) -- see FilterElement::onChannelVideoPresStats
	 * in pdraw_element.cpp. */
	void callOnChannelVideoPresStats(Pdraw::Channel *channel,
					 Pdraw::VideoPresStats *stats)
	{
		onChannelVideoPresStats(channel, stats);
	}
};


} /* namespace PdrawTest */
