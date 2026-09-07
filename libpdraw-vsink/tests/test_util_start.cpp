/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- pdraw_vsink_start() async helper
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

#include "test_util_start.hpp"

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

/* pdraw_vsink_start() has no timeout of its own: it blocks unconditionally
 * until the real backend calls back with success or an error. If a test's
 * RTSP/RTP setup has a bug and the real demuxer never completes the
 * handshake, that wait -- and this join -- would otherwise hang forever.
 * This alarm-based watchdog turns that into a clear, fast failure instead:
 * process-wide and coarse (not per-thread), but sufficient for a test
 * binary that runs its suites sequentially in one process. */
static void joinWatchdogHandler(int signum)
{
	(void)signum;
	static const char msg[] =
		"\ntst-libpdraw-vsink: pdraw_vsink_start()/stop() blocked "
		"for too long (the real backend never completed) -- "
		"aborting instead of hanging indefinitely.\n";
	write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(124);
}

static void *startThreadFn(void *arg)
{
	struct test_vsink_start_ctx *ctx =
		static_cast<struct test_vsink_start_ctx *>(arg);

	ctx->result = pdraw_vsink_start(&ctx->params,
					&ctx->cbs,
					ctx->cbsUserdata,
					&ctx->mediaInfo,
					&ctx->vsink);

	return nullptr;
}

void testVsinkStartAsync(struct test_vsink_start_ctx *ctx)
{
	ctx->mediaInfo = nullptr;
	ctx->vsink = nullptr;
	ctx->result = 0;

	pthread_create(&ctx->thread, nullptr, startThreadFn, ctx);
}

void testVsinkStartJoin(struct test_vsink_start_ctx *ctx)
{
	struct sigaction sa = {};
	struct sigaction oldSa = {};
	sa.sa_handler = &joinWatchdogHandler;
	sigaction(SIGALRM, &sa, &oldSa);
	alarm(20);

	pthread_join(ctx->thread, nullptr);

	alarm(0);
	sigaction(SIGALRM, &oldSa, nullptr);
}
