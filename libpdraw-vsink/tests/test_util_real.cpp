/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- real-backend pdraw_vsink_start() helper
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

#include "test_util_real.hpp"

#include <time.h>

#include <CUnit/CUnit.h>

void startRealVsinkAndWait(TestRtspServer *server,
			   struct test_vsink_start_ctx *ctx,
			   int timeoutMs)
{
	/* Deliberately no CU_ASSERT_*_FATAL anywhere in this function: FATAL
	 * macros longjmp out of the calling test on failure, which skips
	 * the destructors of any live C++ object further up the stack --
	 * including the caller's `TestRtspServer server` (its own thread
	 * would then run on, touching a stack frame that no longer exists).
	 * Every failure path below returns normally instead, so the caller's
	 * objects are destroyed the ordinary way regardless of outcome. */
	CU_ASSERT_TRUE(server->isStarted());
	if (!server->isStarted())
		return;

	ctx->params.url = server->url();

	testVsinkStartAsync(ctx);

	int res = server->waitVideoRtpReady(timeoutMs);
	CU_ASSERT_EQUAL(res, 0);
	if (res != 0) {
		/* The spawned thread is presumably still blocked inside
		 * pdraw_vsink_start() (the real backend never completed the
		 * RTSP handshake) -- join anyway; testVsinkStartJoin()'s
		 * own watchdog aborts the whole process after its own
		 * timeout rather than hang, which is still better than
		 * leaking the thread here and returning right away. */
		testVsinkStartJoin(ctx);
		return;
	}

	/* waitVideoRtpReady() returning only means *this* (server) side
	 * has replied to PLAY -- the real client processes that TCP reply
	 * asynchronously on its own loop thread, so its RTP receive path may
	 * not be listening yet at this exact instant. Resending a few times
	 * with a short delay costs nothing when the first one already
	 * landed (the extras are just a few more early, harmless frames --
	 * anything already queued is drained by tests that need an empty
	 * queue) and closes that race instead of risking another indefinite
	 * pdraw_vsink_start() hang. */
	for (int i = 0; i < 10; i++) {
		server->sendVideoIdrAccessUnit();
		struct timespec ts = {0, 100 * 1000 * 1000};
		nanosleep(&ts, nullptr);
	}

	testVsinkStartJoin(ctx);

	CU_ASSERT_EQUAL(ctx->result, 0);
}
