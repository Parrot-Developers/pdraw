/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- pdraw_vsink_get_frame() polling (raw)
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

/* RAW mode needs libpdraw-vsink's requested
 * PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL to actually succeed, i.e. a
 * real, working H.264 decoder backend in this build -- see the comment on
 * testStartSuccessRaw() in test_start.cpp for why that's an intentional
 * dependency, not an oversight. */

#include "test_pdraw_vsink.hpp"
#include "test_util_real.hpp"

#include <errno.h>
#include <pthread.h>
#include <time.h>

#include <media-buffers/mbuf_raw_video_frame.h>

static void drainRawQueue(struct pdraw_vsink *vsink)
{
	int res;
	do {
		struct pdraw_video_frame frameInfo = {};
		struct pdraw_vsink_frame retFrame = {};
		res = pdraw_vsink_get_frame(
			vsink, 500, nullptr, &frameInfo, &retFrame);
		if (res == 0 && retFrame.raw != nullptr)
			mbuf_raw_video_frame_unref(retFrame.raw);
	} while (res == 0);
}

static void testGetFrameRawDeliveredWithinTimeout(void)
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

	server.sendVideoIdrAccessUnit();

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 5000, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(retFrame.type, PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW);
	CU_ASSERT_PTR_NOT_NULL(retFrame.raw);

	if (retFrame.raw != nullptr)
		mbuf_raw_video_frame_unref(retFrame.raw);
	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameRawNonblockingEmpty(void)
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

	drainRawQueue(ctx.vsink);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EAGAIN);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

struct producer_ctx {
	pthread_t thread;
	TestRtspServer *server;
};

static void *producerThreadFn(void *arg)
{
	struct producer_ctx *ctx = static_cast<struct producer_ctx *>(arg);
	struct timespec ts = {0, 50 * 1000 * 1000};
	nanosleep(&ts, nullptr);
	ctx->server->sendVideoIdrAccessUnit();
	return nullptr;
}

static void testGetFrameRawBlockingConcurrentPush(void)
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

	drainRawQueue(ctx.vsink);

	struct producer_ctx producer = {};
	producer.server = &server;
	pthread_create(&producer.thread, nullptr, producerThreadFn, &producer);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 5000, nullptr, &frameInfo, &retFrame);

	pthread_join(producer.thread, nullptr);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(retFrame.raw);

	if (retFrame.raw != nullptr)
		mbuf_raw_video_frame_unref(retFrame.raw);
	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameRawTimeout(void)
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

	drainRawQueue(ctx.vsink);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 200, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -ETIMEDOUT);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameRawNoSinkYet(void)
{
	/* Backend-agnostic: exercises Vsink::getRawFrame()'s own guard
	 * directly, no server/pipeline involved -- constructed but never
	 * start()ed, so mRaw.queue is still null. */
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	struct pdraw_video_frame frameInfo = {};
	struct mbuf_raw_video_frame *retFrame = nullptr;

	int res = self.getRawFrame(0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EAGAIN);
}

CU_TestInfo s_get_frame_raw_tests[] = {
	{FN("get_frame(raw): delivered within timeout (real pipeline + "
	    "decode)"),
	 &testGetFrameRawDeliveredWithinTimeout},
	{FN("get_frame(raw): timeout_ms=0, empty queue -> -EAGAIN"),
	 &testGetFrameRawNonblockingEmpty},
	{FN("get_frame(raw): blocking wait, frame arrives concurrently "
	    "(real pipeline + decode)"),
	 &testGetFrameRawBlockingConcurrentPush},
	{FN("get_frame(raw): -ETIMEDOUT"), &testGetFrameRawTimeout},
	{FN("get_frame(raw): -EAGAIN when no sink/queue exists yet"),
	 &testGetFrameRawNoSinkYet},
	CU_TEST_INFO_NULL,
};
