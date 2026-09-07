/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Element state machine (Tier B)
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

#define ULOG_TAG pdraw_test_element
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include <string.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testElementInitialStateIsInvalid()
{
	TestElementListener l;
	/* Session can be nullptr: Element ctor never dereferences it. */
	TestElement e(nullptr, &l);
	CU_ASSERT_EQUAL(e.getState(), Element::State::INVALID);
}


static void testElementGetElementStateStrAllValues()
{
	struct TestParam {
		Element::State state;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 6> validCases{
		{{Element::State::INVALID, "INVALID"},
		 {Element::State::CREATED, "CREATED"},
		 {Element::State::STARTING, "STARTING"},
		 {Element::State::STARTED, "STARTED"},
		 {Element::State::STOPPING, "STOPPING"},
		 {Element::State::STOPPED, "STOPPED"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(Element::getElementStateStr(test.state),
				       test.expectedStr);
	}

	Element::State invalidState = static_cast<Element::State>(-1);
	CU_ASSERT_PTR_NULL(Element::getElementStateStr(invalidState));
}


static void testElementSetStateNotifiesListenerSync()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetState(Element::State::CREATED);

	CU_ASSERT_EQUAL(l.eventCount(), 1);
	CU_ASSERT_EQUAL(l.lastState(), Element::State::CREATED);
	CU_ASSERT_EQUAL(l.lastWasAsync(), false);
}


static void testElementSetStateSameStateIsNoop()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetState(Element::State::CREATED);
	e.callSetState(Element::State::CREATED); /* duplicate — no-op */

	/* Only one event emitted (pdraw_element.cpp:117: old==state guard). */
	CU_ASSERT_EQUAL(l.eventCount(), 1);
}


static void testElementAnyTransitionAllowed()
{
	/* Element has no transition guard — any state → any state is accepted.
	 */
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetState(Element::State::STARTED); /* skip CREATED/STARTING */
	CU_ASSERT_EQUAL(e.getState(), Element::State::STARTED);
	CU_ASSERT_EQUAL(l.eventCount(), 1);
}


static void testElementSetStateAsyncNotifyCallsAsyncCallback()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	/* setStateAsyncNotify calls asyncElementStateChange synchronously on
	 * the listener (idle dispatch only happens when Session is the
	 * listener — see pdraw_session.cpp:1456). */
	e.callSetStateAsyncNotify(Element::State::STARTING);

	CU_ASSERT_EQUAL(l.eventCount(), 1);
	CU_ASSERT_EQUAL(l.lastState(), Element::State::STARTING);
	CU_ASSERT_EQUAL(l.lastWasAsync(), true);
}


static void testElementDestructorDoesNotNotifyListener()
{
	TestElementListener l;
	{
		TestElement e(nullptr, &l);
		e.callSetState(Element::State::CREATED);
		l.clear();
		/* Destructor assigns mState = INVALID directly
		 * (pdraw_element.cpp:60), bypassing setState — no listener
		 * notification is emitted. */
	}
	CU_ASSERT_EQUAL(l.eventCount(), 0);
}


static void testElementNoListenerNoCrash()
{
	/* Passing nullptr as listener must not crash on setState. */
	TestElement e(nullptr, nullptr);
	e.callSetState(Element::State::CREATED);
	CU_ASSERT_EQUAL(e.getState(), Element::State::CREATED);
}


static void testElementStartStopLifecycle()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	CU_ASSERT_EQUAL(e.start(), 0);
	CU_ASSERT_EQUAL(e.getState(), Element::State::STARTED);
	/* start() fires STARTING then STARTED */
	CU_ASSERT_EQUAL(l.eventCount(), 2);

	l.clear();

	CU_ASSERT_EQUAL(e.stop(), 0);
	CU_ASSERT_EQUAL(e.getState(), Element::State::STOPPED);
	/* stop() fires STOPPING then STOPPED */
	CU_ASSERT_EQUAL(l.eventCount(), 2);
}


static void testElementGetName()
{
	TestElementListener l;
	TestElement e(nullptr, &l);
	/* setClassName("TestElement") produces "TestElement#<id>". */
	CU_ASSERT_PTR_NOT_NULL(strstr(e.getName().c_str(), "TestElement"));
}


static void testElementGetElementFlushingStateStrAllValues()
{
	using FlushingState = Element::FlushingState;

	struct TestParam {
		FlushingState state;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 3> validCases{
		{{FlushingState::UNFLUSHED, "UNFLUSHED"},
		 {FlushingState::FLUSHING, "FLUSHING"},
		 {FlushingState::FLUSHED, "FLUSHED"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(
			Element::getElementFlushingStateStr(test.state),
			test.expectedStr);
	}

	FlushingState invalidState = static_cast<FlushingState>(-1);
	CU_ASSERT_PTR_NULL(Element::getElementFlushingStateStr(invalidState));
}


static void testElementInitialFlushingStateIsFlushed()
{
	/* pdraw_element.hpp: mFlushingState defaults to FLUSHED, not
	 * UNFLUSHED. */
	TestElementListener l;
	TestElement e(nullptr, &l);

	CU_ASSERT_EQUAL(e.getFlushingState(), Element::FlushingState::FLUSHED);
	CU_ASSERT_FALSE(e.isFlushing());
	CU_ASSERT_FALSE(e.isDraining());
}


static void testElementSetFlushingStateDiscardTrueIsFlushing()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetFlushingState(Element::FlushingState::FLUSHING, true);

	CU_ASSERT_EQUAL(e.getFlushingState(), Element::FlushingState::FLUSHING);
	CU_ASSERT_TRUE(e.isFlushing());
	CU_ASSERT_FALSE(e.isDraining());
}


static void testElementSetFlushingStateDiscardFalseIsDraining()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetFlushingState(Element::FlushingState::FLUSHING, false);

	CU_ASSERT_TRUE(e.isDraining());
	CU_ASSERT_FALSE(e.isFlushing());
}


static void testElementSetFlushingStateSameSubstateKeepsOldDiscard()
{
	/* pdraw_element.cpp: setFlushingState()'s old==substate guard returns
	 * before mFlushDiscard is ever touched -- a same-substate call with a
	 * DIFFERENT discard value is silently ignored, keeping whichever
	 * discard flag was already in effect. Same guard shape as
	 * setState()'s (testElementSetStateSameStateIsNoop), but here it also
	 * gates a second piece of state (mFlushDiscard), not just the
	 * listener notification. */
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetFlushingState(Element::FlushingState::FLUSHING, true);
	CU_ASSERT_TRUE(e.isFlushing());

	e.callSetFlushingState(Element::FlushingState::FLUSHING, false);
	CU_ASSERT_TRUE(e.isFlushing()); /* still discard=true, not false */
}


static void testElementSetFlushingStateBackToFlushedClearsFlushingFlags()
{
	TestElementListener l;
	TestElement e(nullptr, &l);

	e.callSetFlushingState(Element::FlushingState::FLUSHING, true);
	e.callSetFlushingState(Element::FlushingState::FLUSHED);

	CU_ASSERT_EQUAL(e.getFlushingState(), Element::FlushingState::FLUSHED);
	CU_ASSERT_FALSE(e.isFlushing());
	CU_ASSERT_FALSE(e.isDraining());
}


CU_TestInfo g_pdraw_test_element[] = {
	{FN("testElementInitialStateIsInvalid"),
	 testElementInitialStateIsInvalid},
	{FN("testElementGetElementStateStrAllValues"),
	 testElementGetElementStateStrAllValues},
	{FN("testElementSetStateNotifiesListenerSync"),
	 testElementSetStateNotifiesListenerSync},
	{FN("testElementSetStateSameStateIsNoop"),
	 testElementSetStateSameStateIsNoop},
	{FN("testElementAnyTransitionAllowed"),
	 testElementAnyTransitionAllowed},
	{FN("testElementSetStateAsyncNotifyCallsAsyncCallback"),
	 testElementSetStateAsyncNotifyCallsAsyncCallback},
	{FN("testElementDestructorDoesNotNotifyListener"),
	 testElementDestructorDoesNotNotifyListener},
	{FN("testElementNoListenerNoCrash"), testElementNoListenerNoCrash},
	{FN("testElementStartStopLifecycle"), testElementStartStopLifecycle},
	{FN("testElementGetName"), testElementGetName},
	{FN("testElementGetElementFlushingStateStrAllValues"),
	 testElementGetElementFlushingStateStrAllValues},
	{FN("testElementInitialFlushingStateIsFlushed"),
	 testElementInitialFlushingStateIsFlushed},
	{FN("testElementSetFlushingStateDiscardTrueIsFlushing"),
	 testElementSetFlushingStateDiscardTrueIsFlushing},
	{FN("testElementSetFlushingStateDiscardFalseIsDraining"),
	 testElementSetFlushingStateDiscardFalseIsDraining},
	{FN("testElementSetFlushingStateSameSubstateKeepsOldDiscard"),
	 testElementSetFlushingStateSameSubstateKeepsOldDiscard},
	{FN("testElementSetFlushingStateBackToFlushedClearsFlushingFlags"),
	 testElementSetFlushingStateBackToFlushedClearsFlushingFlags},
	CU_TEST_INFO_NULL,
};
