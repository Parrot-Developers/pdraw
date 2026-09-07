/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RAII test fixtures
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

/* pdraw_session.hpp transitively includes pdraw_utils.hpp which requires
 * ULOG_TAG to be defined by the including translation unit. */
#include "pdraw_session.hpp"

#include <libpomp.h>
#include <pthread.h>

#include <functional>
#include <memory>

/* Forward-declare the opaque public C handle so g_test_pdraw_c can be typed
 * correctly without pulling in the full pdraw.h here. */
struct pdraw;

namespace PdrawTest {


/* RAII wrapper around a raw pomp_loop. */
class TestPompLoop {
public:
	TestPompLoop() : mLoop(pomp_loop_new()) {}

	~TestPompLoop()
	{
		if (mLoop)
			pomp_loop_destroy(mLoop);
	}

	struct pomp_loop *raw() const
	{
		return mLoop;
	}

	/* Run one iteration of the loop (non-blocking). */
	int runOnce()
	{
		return pomp_loop_wait_and_process(mLoop, 0);
	}

	/* Pump the loop until predicate() returns true or timeoutMs has
	 * elapsed. Needed for tests exercising async pdraw APIs (e.g. the
	 * demuxer), whose completion callbacks are dispatched via idle
	 * handlers on this loop rather than being invoked synchronously.
	 * Returns true if predicate() became true, false on timeout. */
	[[nodiscard]] bool pumpUntil(const std::function<bool()> &predicate,
				     int timeoutMs = 5000);

private:
	struct pomp_loop *mLoop;
};


/* RAII wrapper creating a real Pdraw::Session for Tier B tests.
 * The Session ctor is cheap: it captures pthread_self() and installs
 * idle handlers, but does not spawn threads or open sockets.
 * The underlying loop is NOT owned by Session; destruction order
 * must be: Session first, then TestPompLoop. */
class TestSession {
public:
	/* listener defaults to nullptr for the common case (Tier B/C suites
	 * that don't need session-wide onMediaAdded/onMediaRemoved
	 * notifications). Pass a real IPdraw::Listener to observe medias
	 * created internally by the pipeline (e.g. a decoder's raw output,
	 * auto-created by a demuxer with autodecoding_mode = DECODE_ALL). */
	explicit TestSession(TestPompLoop *loop,
			     Pdraw::IPdraw::Listener *listener = nullptr) :
			mSession(std::make_unique<Pdraw::Session>(loop->raw(),
								  listener))
	{
	}

	~TestSession() = default;

	Pdraw::Session *get() const
	{
		return mSession.get();
	}

private:
	std::unique_ptr<Pdraw::Session> mSession;
};


/* Globals used by Tier B suites; initialised by pdraw_test_loop_init()
 * and freed by pdraw_test_loop_cleanup() (both declared with C linkage
 * in test_common.h so that test_main.c can pass them as CU_SuiteInfo
 * pInitFunc / pCleanupFunc). */
extern std::unique_ptr<TestPompLoop> g_test_loop;
extern std::unique_ptr<TestSession> g_test_session;

/* Globals used by Tier C suites (C-API tests).
 * Initialised by pdraw_test_api_init() / freed by pdraw_test_api_cleanup().
 * g_test_pdraw_c is the public C handle created via pdraw_new().
 * g_test_loop_thread records the loop thread id (used by the thread-contract
 * test in test_api_session). */
extern struct pdraw *g_test_pdraw_c;
extern pthread_t g_test_loop_thread;


} /* namespace PdrawTest */
