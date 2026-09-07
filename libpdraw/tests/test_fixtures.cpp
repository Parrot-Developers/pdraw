/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RAII test fixtures implementation
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

#define ULOG_TAG pdraw_test_fixtures
#include "test_fixtures.hpp"
#include "test_common.h"

/* pdraw.h is needed for pdraw_new / pdraw_destroy used by pdraw_test_api_*. */
#include "pdraw/pdraw.h"

#include <chrono>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace PdrawTest {

std::unique_ptr<TestPompLoop> g_test_loop;
std::unique_ptr<TestSession> g_test_session;

struct pdraw *g_test_pdraw_c = nullptr;
pthread_t g_test_loop_thread = 0;


bool TestPompLoop::pumpUntil(const std::function<bool()> &predicate,
			     int timeoutMs)
{
	auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds(timeoutMs);
	while (!predicate()) {
		if (std::chrono::steady_clock::now() >= deadline)
			return predicate();
		/* Short wait: let idle/fd handlers fire without busy-spinning
		 * the CPU while still polling the predicate frequently. */
		pomp_loop_wait_and_process(mLoop, 20);
	}
	return true;
}

} /* namespace PdrawTest */


extern "C" {

int pdraw_test_loop_init(void)
{
	PdrawTest::g_test_loop = std::make_unique<PdrawTest::TestPompLoop>();
	PdrawTest::g_test_session = std::make_unique<PdrawTest::TestSession>(
		PdrawTest::g_test_loop.get());
	return 0;
}

int pdraw_test_loop_cleanup(void)
{
	PdrawTest::g_test_session.reset();
	PdrawTest::g_test_loop.reset();
	return 0;
}

int pdraw_test_api_init(void)
{
	/* Tier B part: loop + C++ session */
	int ret = pdraw_test_loop_init();
	if (ret < 0)
		return ret;

	/* Record the thread that runs tests (= the CUnit runner thread). */
	PdrawTest::g_test_loop_thread = pthread_self();

	/* Tier C part: public C handle */
	static const struct pdraw_cbs s_stub_cbs = {};
	ret = pdraw_new(PdrawTest::g_test_loop->raw(),
			&s_stub_cbs,
			nullptr,
			&PdrawTest::g_test_pdraw_c);
	if (ret < 0) {
		pdraw_test_loop_cleanup();
		return ret;
	}
	return 0;
}

int pdraw_test_api_cleanup(void)
{
	if (PdrawTest::g_test_pdraw_c) {
		pdraw_destroy(PdrawTest::g_test_pdraw_c);
		PdrawTest::g_test_pdraw_c = nullptr;
	}
	PdrawTest::g_test_loop_thread = 0;
	return pdraw_test_loop_cleanup();
}

} /* extern "C" */
