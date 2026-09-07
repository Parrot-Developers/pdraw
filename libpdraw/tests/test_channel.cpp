/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Channel cap getters and base flush/drain/teardown/
 * resync/unlink protocol (Tier B)
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

/* Channels are created implicitly by Sink::addInputMedia and accessed via
 * Sink::getInputChannel(media). The typed channel headers are transitively
 * available through test_mocks.hpp → pdraw_element.hpp → pdraw_sink.hpp.
 *
 * The tests below exercise base Channel behavior only (properties,
 * queue/pool owner guards, flush/drain/teardown/resync/unlink protocol,
 * event-string helpers). They are built as a standalone RawVideoChannel
 * purely as a concrete stand-in — RawVideoChannel-specific behavior is
 * covered separately in test_channel_raw_video.cpp. */

#define ULOG_TAG pdraw_test_channel
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_channel_raw_video.hpp"

#include <errno.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── Base Channel properties ─────────────────────────────────────────── */

static void testChannelOwnerIsSink()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&media);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);

	/* getOwner() returns Sink* — cast to the same type before comparing
	 * to avoid multiple-inheritance pointer adjustment. */
	CU_ASSERT_PTR_EQUAL(ch->getOwner(), static_cast<Sink *>(&sink));
}


static void testChannelInitialPendingFlags()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&media);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);

	/* No flush or drain has been initiated yet. */
	CU_ASSERT_EQUAL(ch->isFlushPending(), false);
	CU_ASSERT_EQUAL(ch->isDrainPending(), false);
}


static void testChannelGetInputChannelMissingReturnsNull()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	TestSinkElement sink(g_test_session->get(), &l, 4);

	/* media was never added → no port → no channel */
	CU_ASSERT_PTR_NULL(sink.getInputChannel(&media));
}


/* ── getQueue/setQueue, getPool/setPool: owner check ─────────────────────
 * Neither accessor ever dereferences the queue/pool pointer (see
 * Channel::getQueue/setQueue/getPool/setPool in pdraw_channel.cpp: they only
 * compare `owner` against mOwner and store/return the pointer as-is), so a
 * fake non-null sentinel pointer is safe here — these tests only check
 * pointer round-tripping and the owner guard, never dereference it. ────── */

static void testChannelSetQueueWrongOwnerIsNoop()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestSinkElementRaw otherOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakeQueue = reinterpret_cast<mbuf::Queue *>(0x1);
	ch.setQueue(&otherOwner, fakeQueue);
	CU_ASSERT_PTR_NULL(ch.getQueue(&dummyOwner));
}


static void testChannelGetQueueWrongOwnerReturnsNull()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestSinkElementRaw otherOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakeQueue = reinterpret_cast<mbuf::Queue *>(0x1);
	ch.setQueue(&dummyOwner, fakeQueue);
	CU_ASSERT_PTR_NULL(ch.getQueue(&otherOwner));
}


static void testChannelSetQueueCorrectOwnerRoundtrips()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakeQueue = reinterpret_cast<mbuf::Queue *>(0x1);
	ch.setQueue(&dummyOwner, fakeQueue);
	CU_ASSERT_PTR_EQUAL(ch.getQueue(&dummyOwner), fakeQueue);
	CU_ASSERT_TRUE(ch.hasQueue(fakeQueue));
}


static void testChannelSetPoolWrongOwnerIsNoop()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestSinkElementRaw otherOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakePool = reinterpret_cast<struct mbuf_pool *>(0x1);
	ch.setPool(&otherOwner, fakePool);
	CU_ASSERT_PTR_NULL(ch.getPool(&dummyOwner));
}


static void testChannelGetPoolWrongOwnerReturnsNull()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestSinkElementRaw otherOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakePool = reinterpret_cast<struct mbuf_pool *>(0x1);
	ch.setPool(&dummyOwner, fakePool);
	CU_ASSERT_PTR_NULL(ch.getPool(&otherOwner));
}


static void testChannelSetPoolCorrectOwnerRoundtrips()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	auto *fakePool = reinterpret_cast<struct mbuf_pool *>(0x1);
	ch.setPool(&dummyOwner, fakePool);
	CU_ASSERT_PTR_EQUAL(ch.getPool(&dummyOwner), fakePool);
}


/* ── Flush / drain / teardown / resync / unlink protocol ─────────────────
 * Built as a standalone RawVideoChannel, same rationale as
 * test_element_filter.cpp's checkDownstreamPropagation(): dummyOwner is only
 * needed as a valid Sink* for RawVideoChannel's bookkeeping (getOwner()), and
 * TestChannelSinkListener/ TestChannelSourceListener stand in for the real
 * Sink/Source so each test observes Channel's own logic directly, independent
 * of Sink's dispatch (already exercised via testFilterElementOnChannelXxx) or
 * Source's downstream propagation (Source::sendDownstreamEvent).
 * ───────────────────────────── */

static void testChannelFlushNoSinkListenerFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.flush(), -EPROTO);
}


static void testChannelFlushNotifiesSinkListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_TRUE(ch.isFlushPending());
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 1);
	CU_ASSERT_EQUAL(sinkListener.mLastEventId,
			Channel::toMsgId(Channel::DownstreamEvent::FLUSH));
}


static void testChannelFlushWhilePendingFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_EQUAL(ch.flush(), -EALREADY);
}


static void testChannelDrainWhilePendingFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_EQUAL(ch.drain(), -EALREADY);
}


static void testChannelDrainFailsWhileFlushPending()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_EQUAL(ch.drain(), -EALREADY);
}


static void testChannelFlushFailsWhileDrainPending()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_EQUAL(ch.flush(), -EALREADY);
}


static void testChannelFlushDoneClearsPendingAndNotifiesSourceListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_EQUAL(ch.flushDone(), 0);

	CU_ASSERT_FALSE(ch.isFlushPending());
	CU_ASSERT_EQUAL(srcListener.mEventCount, 1);
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::FLUSHED));
}


static void testChannelFlushDoneNotPendingIsNoop()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	/* flush() was never called: old==substate-style early return. */
	CU_ASSERT_EQUAL(ch.flushDone(), 0);
	CU_ASSERT_EQUAL(srcListener.mEventCount, 0);
}


static void testChannelFlushDoneNoSourceListenerNoCrash()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.flush(), 0);
	/* No source listener set: flushDone() must still clear the pending
	 * flag and return without dereferencing a null listener. */
	CU_ASSERT_EQUAL(ch.flushDone(), 0);
	CU_ASSERT_FALSE(ch.isFlushPending());
}


static void testChannelAsyncFlushDoneNoLoopFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner, &sinkListener, nullptr, nullptr);

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_EQUAL(ch.asyncFlushDone(), -EPROTO);
}


static void testChannelAsyncFlushDoneCompletesViaIdleCallback()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.flush(), 0);
	CU_ASSERT_EQUAL(ch.asyncFlushDone(), 0);

	bool gotFlushed = g_test_loop->pumpUntil(
		[&srcListener]() { return srcListener.mEventCount > 0; });
	CU_ASSERT_TRUE_FATAL(gotFlushed);
	CU_ASSERT_FALSE(ch.isFlushPending());
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::FLUSHED));
}


static void testChannelDrainNoSinkListenerFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.drain(), -EPROTO);
}


static void testChannelDrainNotifiesSinkListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_TRUE(ch.isDrainPending());
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 1);
	CU_ASSERT_EQUAL(sinkListener.mLastEventId,
			Channel::toMsgId(Channel::DownstreamEvent::DRAIN));
}


static void testChannelDrainDoneClearsPendingAndNotifiesSourceListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_EQUAL(ch.drainDone(), 0);

	CU_ASSERT_FALSE(ch.isDrainPending());
	CU_ASSERT_EQUAL(srcListener.mEventCount, 1);
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::DRAINED));
}


static void testChannelDrainDoneNotPendingIsNoop()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.drainDone(), 0);
	CU_ASSERT_EQUAL(srcListener.mEventCount, 0);
}


static void testChannelDrainDoneNoSourceListenerNoCrash()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.drain(), 0);
	/* No source listener set: drainDone() must still clear the pending
	 * flag and return without dereferencing a null listener. */
	CU_ASSERT_EQUAL(ch.drainDone(), 0);
	CU_ASSERT_FALSE(ch.isDrainPending());
}


static void testChannelAsyncDrainDoneNoLoopFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner, &sinkListener, nullptr, nullptr);

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_EQUAL(ch.asyncDrainDone(), -EPROTO);
}


static void testChannelAsyncDrainDoneCompletesViaIdleCallback()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.drain(), 0);
	CU_ASSERT_EQUAL(ch.asyncDrainDone(), 0);

	bool gotDrained = g_test_loop->pumpUntil(
		[&srcListener]() { return srcListener.mEventCount > 0; });
	CU_ASSERT_TRUE_FATAL(gotDrained);
	CU_ASSERT_FALSE(ch.isDrainPending());
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::DRAINED));
}


static void testChannelTeardownNoSinkListenerFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.teardown(), -EPROTO);
}


static void testChannelTeardownNotifiesSinkListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.teardown(), 0);
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 1);
	CU_ASSERT_EQUAL(sinkListener.mLastEventId,
			Channel::toMsgId(Channel::DownstreamEvent::TEARDOWN));
}


static void testChannelResyncNoSourceListenerNoCrash()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.resync(), 0);
}


static void testChannelResyncNotifiesSourceListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.resync(), 0);
	CU_ASSERT_EQUAL(srcListener.mEventCount, 1);
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::RESYNC));
}


static void testChannelUnlinkNoSourceListenerNoCrash()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.unlink(), 0);
}


static void testChannelUnlinkNotifiesSourceListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSourceListener srcListener;
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());
	ch.setSourceListener(&srcListener);

	CU_ASSERT_EQUAL(ch.unlink(), 0);
	CU_ASSERT_EQUAL(srcListener.mEventCount, 1);
	CU_ASSERT_EQUAL(srcListener.mLastEventId,
			Channel::toMsgId(Channel::UpstreamEvent::UNLINK));
}


static void testChannelSendVideoPresStatsNoSourceListenerNoCrash()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	/* The has-listener path is exercised via
	 * testFilterElementOnChannelVideoPresStatsPropagatesUpstream in
	 * test_element_filter.cpp; only the null-listener guard is new here. */
	VideoPresStats stats;
	CU_ASSERT_EQUAL(ch.sendVideoPresStats(&stats), 0);
}


/* ── sendDownstreamEvent(): generic event dispatch ────────────────────────
 * FLUSH/DRAIN/TEARDOWN are rejected here since they have dedicated methods
 * (flush()/drain()/teardown() above); any other event is dispatched as-is
 * to the sink listener. ─────────────────────────────────────────────────── */

static void testChannelSendDownstreamEventRejectsFlush()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.sendDownstreamEvent(Channel::DownstreamEvent::FLUSH),
			-EPROTO);
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 0);
}


static void testChannelSendDownstreamEventRejectsDrain()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.sendDownstreamEvent(Channel::DownstreamEvent::DRAIN),
			-EPROTO);
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 0);
}


static void testChannelSendDownstreamEventRejectsTeardown()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(
		ch.sendDownstreamEvent(Channel::DownstreamEvent::TEARDOWN),
		-EPROTO);
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 0);
}


static void testChannelSendDownstreamEventNoSinkListenerFails()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	RawVideoChannel ch(&dummyOwner,
			   nullptr,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.sendDownstreamEvent(Channel::DownstreamEvent::SOS),
			-EPROTO);
}


static void testChannelSendDownstreamEventNotifiesSinkListener()
{
	TestElementListener l;
	TestSinkElementRaw dummyOwner(g_test_session->get(), &l, 4);
	TestChannelSinkListener sinkListener;
	RawVideoChannel ch(&dummyOwner,
			   &sinkListener,
			   nullptr,
			   g_test_session->get()->getPompLoop());

	CU_ASSERT_EQUAL(ch.sendDownstreamEvent(Channel::DownstreamEvent::SOS),
			0);
	CU_ASSERT_EQUAL(sinkListener.mEventCount, 1);
	CU_ASSERT_EQUAL(sinkListener.mLastEventId,
			Channel::toMsgId(Channel::DownstreamEvent::SOS));
}


/* ── getDownstreamEventStr / getUpstreamEventStr ─────────────────────────
 * Same pattern as Element::getElementStateStr in test_element.cpp: every
 * enum value must resolve to a non-null string, and an out-of-range value
 * must hit the default case (nullptr). ─────────────────────────────────── */

static void testChannelGetDownstreamEventStrAllValues()
{
	using DSEvent = Channel::DownstreamEvent;

	struct TestParam {
		DSEvent event;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 11> validCases{
		{{DSEvent::FLUSH, "FLUSH"},
		 {DSEvent::DRAIN, "DRAIN"},
		 {DSEvent::TEARDOWN, "TEARDOWN"},
		 {DSEvent::SOS, "SOS"},
		 {DSEvent::EOS, "EOS"},
		 {DSEvent::RECONFIGURE, "RECONFIGURE"},
		 {DSEvent::RESOLUTION_CHANGE, "RESOLUTION_CHANGE"},
		 {DSEvent::FRAMERATE_CHANGE, "FRAMERATE_CHANGE"},
		 {DSEvent::TIMEOUT, "TIMEOUT"},
		 {DSEvent::PHOTO_TRIGGER, "PHOTO_TRIGGER"},
		 {DSEvent::SESSION_META_UPDATE, "SESSION_META_UPDATE"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(
			Channel::getDownstreamEventStr(test.event),
			test.expectedStr);
	}

	DSEvent invalidEvent = static_cast<DSEvent>(0xffff);
	CU_ASSERT_PTR_NULL(Channel::getDownstreamEventStr(invalidEvent));
}


static void testChannelGetUpstreamEventStrAllValues()
{
	using USEvent = Channel::UpstreamEvent;

	struct TestParam {
		USEvent event;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 5> validCases{
		{{USEvent::UNLINK, "UNLINK"},
		 {USEvent::FLUSHED, "FLUSHED"},
		 {USEvent::DRAINED, "DRAINED"},
		 {USEvent::RESYNC, "RESYNC"},
		 {USEvent::VIDEO_PRES_STATS, "VIDEO_PRES_STATS"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(Channel::getUpstreamEventStr(test.event),
				       test.expectedStr);
	}

	USEvent invalidEvent = static_cast<USEvent>(-1);
	CU_ASSERT_PTR_NULL(Channel::getUpstreamEventStr(invalidEvent));
}


CU_TestInfo g_pdraw_test_channel[] = {
	{FN("testChannelOwnerIsSink"), testChannelOwnerIsSink},
	{FN("testChannelInitialPendingFlags"), testChannelInitialPendingFlags},
	{FN("testChannelGetInputChannelMissingReturnsNull"),
	 testChannelGetInputChannelMissingReturnsNull},
	{FN("testChannelSetQueueWrongOwnerIsNoop"),
	 testChannelSetQueueWrongOwnerIsNoop},
	{FN("testChannelGetQueueWrongOwnerReturnsNull"),
	 testChannelGetQueueWrongOwnerReturnsNull},
	{FN("testChannelSetQueueCorrectOwnerRoundtrips"),
	 testChannelSetQueueCorrectOwnerRoundtrips},
	{FN("testChannelSetPoolWrongOwnerIsNoop"),
	 testChannelSetPoolWrongOwnerIsNoop},
	{FN("testChannelGetPoolWrongOwnerReturnsNull"),
	 testChannelGetPoolWrongOwnerReturnsNull},
	{FN("testChannelSetPoolCorrectOwnerRoundtrips"),
	 testChannelSetPoolCorrectOwnerRoundtrips},
	{FN("testChannelFlushNoSinkListenerFails"),
	 testChannelFlushNoSinkListenerFails},
	{FN("testChannelFlushNotifiesSinkListener"),
	 testChannelFlushNotifiesSinkListener},
	{FN("testChannelFlushWhilePendingFails"),
	 testChannelFlushWhilePendingFails},
	{FN("testChannelDrainWhilePendingFails"),
	 testChannelDrainWhilePendingFails},
	{FN("testChannelDrainFailsWhileFlushPending"),
	 testChannelDrainFailsWhileFlushPending},
	{FN("testChannelFlushFailsWhileDrainPending"),
	 testChannelFlushFailsWhileDrainPending},
	{FN("testChannelFlushDoneClearsPendingAndNotifiesSourceListener"),
	 testChannelFlushDoneClearsPendingAndNotifiesSourceListener},
	{FN("testChannelFlushDoneNotPendingIsNoop"),
	 testChannelFlushDoneNotPendingIsNoop},
	{FN("testChannelFlushDoneNoSourceListenerNoCrash"),
	 testChannelFlushDoneNoSourceListenerNoCrash},
	{FN("testChannelAsyncFlushDoneNoLoopFails"),
	 testChannelAsyncFlushDoneNoLoopFails},
	{FN("testChannelAsyncFlushDoneCompletesViaIdleCallback"),
	 testChannelAsyncFlushDoneCompletesViaIdleCallback},
	{FN("testChannelDrainNoSinkListenerFails"),
	 testChannelDrainNoSinkListenerFails},
	{FN("testChannelDrainNotifiesSinkListener"),
	 testChannelDrainNotifiesSinkListener},
	{FN("testChannelDrainDoneClearsPendingAndNotifiesSourceListener"),
	 testChannelDrainDoneClearsPendingAndNotifiesSourceListener},
	{FN("testChannelDrainDoneNotPendingIsNoop"),
	 testChannelDrainDoneNotPendingIsNoop},
	{FN("testChannelDrainDoneNoSourceListenerNoCrash"),
	 testChannelDrainDoneNoSourceListenerNoCrash},
	{FN("testChannelAsyncDrainDoneNoLoopFails"),
	 testChannelAsyncDrainDoneNoLoopFails},
	{FN("testChannelAsyncDrainDoneCompletesViaIdleCallback"),
	 testChannelAsyncDrainDoneCompletesViaIdleCallback},
	{FN("testChannelTeardownNoSinkListenerFails"),
	 testChannelTeardownNoSinkListenerFails},
	{FN("testChannelTeardownNotifiesSinkListener"),
	 testChannelTeardownNotifiesSinkListener},
	{FN("testChannelResyncNoSourceListenerNoCrash"),
	 testChannelResyncNoSourceListenerNoCrash},
	{FN("testChannelResyncNotifiesSourceListener"),
	 testChannelResyncNotifiesSourceListener},
	{FN("testChannelUnlinkNoSourceListenerNoCrash"),
	 testChannelUnlinkNoSourceListenerNoCrash},
	{FN("testChannelUnlinkNotifiesSourceListener"),
	 testChannelUnlinkNotifiesSourceListener},
	{FN("testChannelSendVideoPresStatsNoSourceListenerNoCrash"),
	 testChannelSendVideoPresStatsNoSourceListenerNoCrash},
	{FN("testChannelSendDownstreamEventRejectsFlush"),
	 testChannelSendDownstreamEventRejectsFlush},
	{FN("testChannelSendDownstreamEventRejectsDrain"),
	 testChannelSendDownstreamEventRejectsDrain},
	{FN("testChannelSendDownstreamEventRejectsTeardown"),
	 testChannelSendDownstreamEventRejectsTeardown},
	{FN("testChannelSendDownstreamEventNoSinkListenerFails"),
	 testChannelSendDownstreamEventNoSinkListenerFails},
	{FN("testChannelSendDownstreamEventNotifiesSinkListener"),
	 testChannelSendDownstreamEventNotifiesSinkListener},
	{FN("testChannelGetDownstreamEventStrAllValues"),
	 testChannelGetDownstreamEventStrAllValues},
	{FN("testChannelGetUpstreamEventStrAllValues"),
	 testChannelGetUpstreamEventStrAllValues},
	CU_TEST_INFO_NULL,
};
