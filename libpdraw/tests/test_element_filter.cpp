/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — FilterElement dual Sink/Source wiring (Tier B)
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

/* FilterElement (base of VideoDecoder/VideoEncoder/VideoScaler/
 * AudioDecoder/AudioEncoder) is simultaneously a Sink and a Source. These
 * tests exercise both sides on the same TestFilterElement instance, mirroring
 * test_sink.cpp / test_source.cpp but checking that neither side interferes
 * with the other (shared getName()/state, independent port bookkeeping). */

#define ULOG_TAG pdraw_test_element_filter
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_media.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testFilterElementAddInputMedia()
{
	TestElementListener l;
	/* media must outlive filter: Sink::~Sink() logs media->getName() */
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream; /* matches kTestSinkCodedCaps */
	TestFilterElement filter(g_test_session->get(), &l, nullptr, 4, 4);

	CU_ASSERT_EQUAL(filter.addInputMedia(&media), 0);
	CU_ASSERT_EQUAL(filter.getInputMediaCount(), 1u);
}


static void testFilterElementAddInputMediaFormatMismatch()
{
	TestElementListener l;
	/* TestFilterElement only accepts vdef_h264_byte_stream on input. */
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h265_byte_stream;
	TestFilterElement filter(g_test_session->get(), &l, nullptr, 4, 4);

	CU_ASSERT_EQUAL(filter.addInputMedia(&media), -ENOSYS);
}


static void testFilterElementAddOutputPortNotifiesListener()
{
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia outMedia(nullptr); /* outlives filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);

	int ret = filter.callAddOutputPort(&outMedia);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(sl.addedCount(), 1);
	CU_ASSERT_PTR_EQUAL(sl.lastAdded(), &outMedia);
}


static void testFilterElementOutputMediaCountGrows()
{
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr); /* outlive filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);

	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 0u);
	filter.callAddOutputPort(&m1);
	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 1u);
	filter.callAddOutputPort(&m2);
	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 2u);
}


static void testFilterElementAddOutputBeyondMaxRejected()
{
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr),
		m3(nullptr); /* outlive filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 2);

	CU_ASSERT_EQUAL(filter.callAddOutputPort(&m1), 0);
	CU_ASSERT_EQUAL(filter.callAddOutputPort(&m2), 0);
	CU_ASSERT_EQUAL(filter.callAddOutputPort(&m3), -ENOBUFS);
}


static void testFilterElementRemoveOutputPortNotifiesListener()
{
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia media(nullptr); /* outlives filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);

	filter.callAddOutputPort(&media);
	sl.clear();

	CU_ASSERT_EQUAL(filter.callRemoveOutputPort(&media), 0);
	CU_ASSERT_EQUAL(sl.removedCount(), 1);
}


static void testFilterElementInputAndOutputCountsAreIndependent()
{
	TestElementListener l;
	TestSourceListener sl;
	CodedVideoMedia inMedia(g_test_session->get()); /* outlives filter */
	inMedia.format = vdef_h264_byte_stream;
	RawVideoMedia outMedia(nullptr); /* outlives filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);

	filter.addInputMedia(&inMedia);
	CU_ASSERT_EQUAL(filter.getInputMediaCount(), 1u);
	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 0u);

	filter.callAddOutputPort(&outMedia);
	CU_ASSERT_EQUAL(filter.getInputMediaCount(), 1u);
	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 1u);

	CU_ASSERT_EQUAL(filter.callRemoveInputMedias(), 0);
	CU_ASSERT_EQUAL(filter.getInputMediaCount(), 0u);
	/* Removing all input medias must not touch the output side. */
	CU_ASSERT_EQUAL(filter.getOutputMediaCount(), 1u);
}


static void testFilterElementGetNameSharedBetweenSinkAndSource()
{
	TestElementListener l;
	TestFilterElement filter(g_test_session->get(), &l, nullptr, 4, 4);

	/* FilterElement::getName() resolves the Sink/Source diamond by
	 * delegating to Element::getName(); both bases must agree. */
	const Sink *asSink = &filter;
	const Source *asSource = &filter;
	CU_ASSERT_STRING_EQUAL(asSink->getName().c_str(),
			       asSource->getName().c_str());
}


static void testFilterElementStartStopLifecycle()
{
	TestElementListener l;
	TestFilterElement filter(g_test_session->get(), &l, nullptr, 4, 4);

	CU_ASSERT_EQUAL(filter.start(), 0);
	CU_ASSERT_EQUAL(l.lastState(), Element::State::STARTED);
	CU_ASSERT_EQUAL(filter.stop(), 0);
	CU_ASSERT_EQUAL(l.lastState(), Element::State::STOPPED);
}


/* ── Downstream-event propagation (FilterElement::onChannelXxx) ─────────
 *
 * Each of these forwards a specific downstream event from the Sink side to
 * every channel of every output media (see pdraw_element.cpp). The channel
 * argument passed to the wrapper only needs to be non-null -- it does not
 * need to be a real, *registered input* channel: Sink::onChannelXxx's own
 * bookkeeping (looking the channel up in mInputPorts) is a harmless no-op
 * when not found, and the output propagation this test actually cares
 * about is unconditional. So the output channel itself is reused as that
 * argument, avoiding the need for a second, separate input-side channel. */

static void
checkDownstreamPropagation(void (TestFilterElement::*callFn)(Channel *),
			   Channel::DownstreamEvent expectedEvent)
{
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia outMedia(nullptr); /* outlives filter */
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);
	filter.callAddOutputPort(&outMedia);

	/* dummyOwner: only needed as a valid Sink* for RawVideoChannel's
	 * bookkeeping (getOwner()); the actual observation point is
	 * outListener, a standalone Channel::SinkListener. */
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener outListener;
	RawVideoChannel outChannel(&dummyOwner,
				   &outListener,
				   nullptr,
				   g_test_session->get()->getPompLoop());
	CU_ASSERT_EQUAL_FATAL(filter.addOutputChannel(&outMedia, &outChannel),
			      0);

	(filter.*callFn)(&outChannel);

	CU_ASSERT_EQUAL(outListener.mEventCount, 1);
	CU_ASSERT_EQUAL(outListener.mLastEventId,
			Channel::toMsgId(expectedEvent));

	/* Unregister before outChannel (a local, owned by this function) goes
	 * out of scope: ~Source() tolerates a dangling leftover channel
	 * pointer without dereferencing it (just logs a warning and leaves
	 * mOutputPorts non-empty), so this isn't a crash risk either way, but
	 * cleaning up here avoids that log noise. */
	filter.removeOutputChannel(&outMedia, &outChannel);
}


static void testFilterElementOnChannelSosPropagatesDownstream()
{
	checkDownstreamPropagation(&TestFilterElement::callOnChannelSos,
				   Channel::DownstreamEvent::SOS);
}


static void testFilterElementOnChannelEosPropagatesDownstream()
{
	checkDownstreamPropagation(&TestFilterElement::callOnChannelEos,
				   Channel::DownstreamEvent::EOS);
}


static void testFilterElementOnChannelReconfigurePropagatesDownstream()
{
	checkDownstreamPropagation(&TestFilterElement::callOnChannelReconfigure,
				   Channel::DownstreamEvent::RECONFIGURE);
}


static void testFilterElementOnChannelResolutionChangePropagatesDownstream()
{
	checkDownstreamPropagation(
		&TestFilterElement::callOnChannelResolutionChange,
		Channel::DownstreamEvent::RESOLUTION_CHANGE);
}


static void testFilterElementOnChannelFramerateChangePropagatesDownstream()
{
	checkDownstreamPropagation(
		&TestFilterElement::callOnChannelFramerateChange,
		Channel::DownstreamEvent::FRAMERATE_CHANGE);
}


static void testFilterElementOnChannelTimeoutPropagatesDownstream()
{
	checkDownstreamPropagation(&TestFilterElement::callOnChannelTimeout,
				   Channel::DownstreamEvent::TIMEOUT);
}


static void testFilterElementOnChannelPhotoTriggerPropagatesDownstream()
{
	checkDownstreamPropagation(
		&TestFilterElement::callOnChannelPhotoTrigger,
		Channel::DownstreamEvent::PHOTO_TRIGGER);
}


static void testFilterElementOnChannelSessionMetaUpdatePropagatesDownstream()
{
	checkDownstreamPropagation(
		&TestFilterElement::callOnChannelSessionMetaUpdate,
		Channel::DownstreamEvent::SESSION_META_UPDATE);
}


static void testFilterElementOnChannelXxxNullChannelIsNoop()
{
	/* All 9 onChannelXxx methods guard against a null channel at the
	 * top of their body (pdraw_element.cpp: identical early-return
	 * pattern in each).  Passing nullptr must not touch the output side
	 * at all — verified by asserting that the downstream sink listener
	 * has received zero events after every call. */
	TestElementListener l;
	TestSourceListener sl;
	RawVideoMedia outMedia(nullptr);
	TestFilterElement filter(g_test_session->get(), &l, &sl, 4, 4);
	filter.callAddOutputPort(&outMedia);

	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener outListener;
	RawVideoChannel outChannel(&dummyOwner,
				   &outListener,
				   nullptr,
				   g_test_session->get()->getPompLoop());
	filter.addOutputChannel(&outMedia, &outChannel);

	/* 1. onChannelSos(nullptr) */
	filter.callOnChannelSos(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 2. onChannelEos(nullptr) */
	filter.callOnChannelEos(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 3. onChannelReconfigure(nullptr) */
	filter.callOnChannelReconfigure(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 4. onChannelResolutionChange(nullptr) */
	filter.callOnChannelResolutionChange(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 5. onChannelFramerateChange(nullptr) */
	filter.callOnChannelFramerateChange(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 6. onChannelTimeout(nullptr) */
	filter.callOnChannelTimeout(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 7. onChannelPhotoTrigger(nullptr) */
	filter.callOnChannelPhotoTrigger(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 8. onChannelSessionMetaUpdate(nullptr) */
	filter.callOnChannelSessionMetaUpdate(nullptr);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	/* 9. onChannelVideoPresStats(nullptr, ...) */
	VideoPresStats stats;
	filter.callOnChannelVideoPresStats(nullptr, &stats);
	CU_ASSERT_EQUAL(outListener.mEventCount, 0);

	filter.removeOutputChannel(&outMedia, &outChannel);
}


/* ── Upstream propagation (FilterElement::onChannelVideoPresStats) ──────
 *
 * Unlike the 8 tests above, video presentation stats flow the OTHER way:
 * from the Source side back upstream via the filter's own INPUT channels
 * (see pdraw_element.cpp). This needs a real input media/channel (created
 * by addInputMedia()), not just an output one. */

static void testFilterElementOnChannelVideoPresStatsPropagatesUpstream()
{
	TestElementListener l;
	/* Declared before filter (destroyed AFTER it, C++ locals unwind in
	 * reverse declaration order): ~TestFilterElement() calls
	 * removeInputMedias(), which calls channel->unlink(), which calls
	 * mSourceListener->onChannelUpstreamEvent() if a listener is set (see
	 * Channel::unlink() in pdraw_channel.cpp) -- srcListener must still
	 * be alive at that point. Confirmed the hard way: declaring it after
	 * filter crashed with "pure virtual method called" (calling through
	 * srcListener's vtable after it was already destroyed). */
	TestChannelSourceListener srcListener;
	CodedVideoMedia inMedia(g_test_session->get()); /* outlives filter */
	inMedia.format = vdef_h264_byte_stream;
	TestFilterElement filter(g_test_session->get(), &l, nullptr, 4, 4);

	CU_ASSERT_EQUAL_FATAL(filter.addInputMedia(&inMedia), 0);
	Channel *inChannel = filter.getInputChannel(&inMedia);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inChannel);

	inChannel->setSourceListener(&srcListener);

	VideoPresStats stats;
	filter.callOnChannelVideoPresStats(inChannel, &stats);

	CU_ASSERT_EQUAL(srcListener.mEventCount, 1);
	CU_ASSERT_EQUAL(
		srcListener.mLastEventId,
		Channel::toMsgId(Channel::UpstreamEvent::VIDEO_PRES_STATS));

	/* Belt-and-suspenders on top of the declaration-order fix above. */
	inChannel->setSourceListener(nullptr);
}


CU_TestInfo g_pdraw_test_element_filter[] = {
	{FN("testFilterElementAddInputMedia"), testFilterElementAddInputMedia},
	{FN("testFilterElementAddInputMediaFormatMismatch"),
	 testFilterElementAddInputMediaFormatMismatch},
	{FN("testFilterElementAddOutputPortNotifiesListener"),
	 testFilterElementAddOutputPortNotifiesListener},
	{FN("testFilterElementOutputMediaCountGrows"),
	 testFilterElementOutputMediaCountGrows},
	{FN("testFilterElementAddOutputBeyondMaxRejected"),
	 testFilterElementAddOutputBeyondMaxRejected},
	{FN("testFilterElementRemoveOutputPortNotifiesListener"),
	 testFilterElementRemoveOutputPortNotifiesListener},
	{FN("testFilterElementInputAndOutputCountsAreIndependent"),
	 testFilterElementInputAndOutputCountsAreIndependent},
	{FN("testFilterElementGetNameSharedBetweenSinkAndSource"),
	 testFilterElementGetNameSharedBetweenSinkAndSource},
	{FN("testFilterElementStartStopLifecycle"),
	 testFilterElementStartStopLifecycle},
	{FN("testFilterElementOnChannelSosPropagatesDownstream"),
	 testFilterElementOnChannelSosPropagatesDownstream},
	{FN("testFilterElementOnChannelEosPropagatesDownstream"),
	 testFilterElementOnChannelEosPropagatesDownstream},
	{FN("testFilterElementOnChannelReconfigurePropagatesDownstream"),
	 testFilterElementOnChannelReconfigurePropagatesDownstream},
	{FN("testFilterElementOnChannelResolutionChangePropagatesDownstream"),
	 testFilterElementOnChannelResolutionChangePropagatesDownstream},
	{FN("testFilterElementOnChannelFramerateChangePropagatesDownstream"),
	 testFilterElementOnChannelFramerateChangePropagatesDownstream},
	{FN("testFilterElementOnChannelTimeoutPropagatesDownstream"),
	 testFilterElementOnChannelTimeoutPropagatesDownstream},
	{FN("testFilterElementOnChannelPhotoTriggerPropagatesDownstream"),
	 testFilterElementOnChannelPhotoTriggerPropagatesDownstream},
	{FN("testFilterElementOnChannelSessionMetaUpdatePropagatesDownstream"),
	 testFilterElementOnChannelSessionMetaUpdatePropagatesDownstream},
	{FN("testFilterElementOnChannelXxxNullChannelIsNoop"),
	 testFilterElementOnChannelXxxNullChannelIsNoop},
	{FN("testFilterElementOnChannelVideoPresStatsPropagatesUpstream"),
	 testFilterElementOnChannelVideoPresStatsPropagatesUpstream},
	CU_TEST_INFO_NULL,
};
