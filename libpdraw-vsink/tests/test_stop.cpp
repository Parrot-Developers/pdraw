/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- pdraw_vsink_stop() teardown
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

#include "test_pdraw_vsink.hpp"
#include "test_util_real.hpp"

#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <media-buffers/mbuf_coded_video_frame.h>

static void testStopCleanCoded(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;

	startRealVsinkAndWait(&server, &ctx);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	int res = pdraw_vsink_stop(ctx.vsink);
	CU_ASSERT_EQUAL(res, 0);

	pdraw_media_info_free(ctx.mediaInfo);
}

/* RAW mode needs a real, working H.264 decoder in this build -- see the
 * comment on testStartSuccessRaw() in test_start.cpp. */
static void testStopCleanRaw(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;

	startRealVsinkAndWait(&server, &ctx, 10000);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	int res = pdraw_vsink_stop(ctx.vsink);
	CU_ASSERT_EQUAL(res, 0);

	pdraw_media_info_free(ctx.mediaInfo);
}

/*
 * A dedicated "pdraw_vsink_start() failure cleans up its own
 * partially-built instance" test (pointing at a port nothing listens on)
 * was tried here and dropped: it needs pdraw_vsink_start() to fail
 * promptly, but the real RTSP client doesn't fail fast on a refused
 * connection -- confirmed by testVsinkStartJoin()'s watchdog firing
 * after 20s rather than the call returning an error. That's real
 * production behavior (a resilient streaming client retrying rather than
 * giving up immediately), not something a test should fight -- and forcing
 * it would need mocking libpdraw's connection handling, which is exactly
 * the kind of internal behavior libpdraw's own test suite (tst-libpdraw)
 * already covers, not libpdraw-vsink's.
 */

/*
 * pdraw_vsink_stop() used to destroy self->mutex/self->cond and free self
 * without ever unblocking a thread parked inside pdraw_vsink_get_frame()'s
 * cond_wait (confirmed by reading pdraw_vsink_stop()/get_coded_frame() in
 * pdraw_vsink.c/pdraw_vsink_coded.c: stop() only ever signalled its own,
 * separate self->cond_ready flag) -- a concurrent blocking get_frame()
 * (timeout_ms=-1) was left waiting forever on now-destroyed/freed memory.
 * Fixed with a self->stopping flag + self->waiters count: stop() now
 * broadcasts stopping and waits for every in-flight get_frame() call to
 * observe it and return -ECANCELED before tearing anything down. This test
 * pins that fix down; a watchdog bounds the join in case a regression
 * brings the hang back, instead of wedging the whole test binary.
 */
struct StopRaceCtx {
	struct pdraw_vsink *vsink;
	int result;
	pthread_t thread;
};

static void *blockingGetFrameThreadFn(void *arg)
{
	struct StopRaceCtx *ctx = static_cast<struct StopRaceCtx *>(arg);
	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};

	ctx->result = pdraw_vsink_get_frame(
		ctx->vsink, -1, nullptr, &frameInfo, &retFrame);

	if (ctx->result == 0 && retFrame.coded != nullptr)
		mbuf_coded_video_frame_unref(retFrame.coded);

	return nullptr;
}

static void stopRaceWatchdogHandler(int signum)
{
	(void)signum;
	static const char msg[] =
		"\ntst-libpdraw-vsink: pdraw_vsink_get_frame(timeout_ms=-1) "
		"never returned after a concurrent pdraw_vsink_stop() -- "
		"regression on the stopping/waiters unblock fix. Aborting "
		"instead of hanging forever.\n";
	write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(124);
}

static void testStopUnblocksConcurrentBlockingGetFrame(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;

	startRealVsinkAndWait(&server, &ctx);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	/* Drain whatever the startup handshake already delivered so the
	 * spawned thread below genuinely blocks instead of popping a
	 * leftover frame immediately. */
	int drainRes;
	do {
		struct pdraw_video_frame drainInfo = {};
		struct pdraw_vsink_frame drainFrame = {};
		drainRes = pdraw_vsink_get_frame(
			ctx.vsink, 200, nullptr, &drainInfo, &drainFrame);
		if (drainRes == 0 && drainFrame.coded != nullptr)
			mbuf_coded_video_frame_unref(drainFrame.coded);
	} while (drainRes == 0);

	struct StopRaceCtx race = {};
	race.vsink = ctx.vsink;
	pthread_create(&race.thread, nullptr, blockingGetFrameThreadFn, &race);

	/* Best-effort: give the spawned thread time to actually enter
	 * cond_wait() before racing pdraw_vsink_stop() against it -- same
	 * reasoning as test_util_real.cpp's own retries. */
	struct timespec ts = {0, 200 * 1000 * 1000};
	nanosleep(&ts, nullptr);

	int stopRes = pdraw_vsink_stop(ctx.vsink);
	CU_ASSERT_EQUAL(stopRes, 0);

	struct sigaction sa = {};
	struct sigaction oldSa = {};
	sa.sa_handler = &stopRaceWatchdogHandler;
	sigaction(SIGALRM, &sa, &oldSa);
	alarm(10);

	pthread_join(race.thread, nullptr);

	alarm(0);
	sigaction(SIGALRM, &oldSa, nullptr);

	CU_ASSERT_EQUAL(race.result, -ECANCELED);

	pdraw_media_info_free(ctx.mediaInfo);
}

/*
 * Backend-agnostic guard-clause tests below: these internal (priv.h-declared)
 * functions each start with an early "already torn down / never set up"
 * check before touching anything else, so they can be exercised directly on
 * a zero-initialized struct pdraw_vsink -- no server/pipeline needed, same
 * idiom as the "-EAGAIN when no sink/queue exists yet" tests in
 * test_get_frame_raw.cpp/test_get_frame_coded.cpp.
 */

static void testStopDetachRawEventNoQueue(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	int res = self.detachRawEvent();
	CU_ASSERT_EQUAL(res, 0);
}

static void testStopDetachCodedEventNoQueue(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	int res = self.detachCodedEvent();
	CU_ASSERT_EQUAL(res, 0);
}

static void testStopDestroyRawSinkAlreadyNull(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	int res = self.destroyRawSink();
	CU_ASSERT_EQUAL(res, 0);
}

static void testStopDestroyCodedSinkAlreadyNull(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	int res = self.destroyCodedSink();
	CU_ASSERT_EQUAL(res, 0);
}

CU_TestInfo s_stop_tests[] = {
	{FN("stop: clean teardown after successful start (coded)"),
	 &testStopCleanCoded},
	{FN("stop: clean teardown after successful start (raw + decode)"),
	 &testStopCleanRaw},
	{FN("stop: unblocks a concurrent blocking get_frame() with "
	    "-ECANCELED"),
	 &testStopUnblocksConcurrentBlockingGetFrame},
	{FN("stop: detach_raw_event() is a no-op with no queue"),
	 &testStopDetachRawEventNoQueue},
	{FN("stop: detach_coded_event() is a no-op with no queue"),
	 &testStopDetachCodedEventNoQueue},
	{FN("stop: destroy_raw_sink() is a no-op with sink already NULL"),
	 &testStopDestroyRawSinkAlreadyNull},
	{FN("stop: destroy_coded_sink() is a no-op with sink already NULL"),
	 &testStopDestroyCodedSinkAlreadyNull},
	CU_TEST_INFO_NULL,
};
