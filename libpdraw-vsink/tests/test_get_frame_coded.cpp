/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- pdraw_vsink_get_frame() polling (coded)
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
#include <pthread.h>
#include <time.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

/* Repeatedly pops (short per-call timeout) until the queue is genuinely
 * empty (-EAGAIN/-ETIMEDOUT), so timing-sensitive tests below start from a
 * known state regardless of whatever the real pipeline had already
 * delivered by the time pdraw_vsink_start() returned. */
static void drainCodedQueue(struct pdraw_vsink *vsink)
{
	int res;
	do {
		struct pdraw_video_frame frameInfo = {};
		struct pdraw_vsink_frame retFrame = {};
		res = pdraw_vsink_get_frame(
			vsink, 500, nullptr, &frameInfo, &retFrame);
		if (res == 0 && retFrame.coded != nullptr)
			mbuf_coded_video_frame_unref(retFrame.coded);
	} while (res == 0);
}

static void testGetFrameCodedDeliveredWithinTimeout(void)
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

	server.sendVideoIdrAccessUnit();

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 3000, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(retFrame.type, PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED);
	CU_ASSERT_PTR_NOT_NULL(retFrame.coded);

	if (retFrame.coded != nullptr)
		mbuf_coded_video_frame_unref(retFrame.coded);
	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

/* Every other test here passes frame_memory=NULL, exercising only the
 * "allocate our own memory" branch (ownMem=true) inside
 * Vsink::getCodedFrame(). This covers the other branch: a
 * caller-provided struct mbuf_mem, which the function must use as-is
 * (ownMem=false) instead of allocating and later unref'ing internally --
 * the caller keeps its own reference and is responsible for it. */
static void testGetFrameCodedCallerProvidedMemory(void)
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

	server.sendVideoIdrAccessUnit();

	struct mbuf_mem *memory = nullptr;
	int res = mbuf_mem_generic_new(4096, &memory);
	CU_ASSERT_EQUAL(res, 0);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	res = pdraw_vsink_get_frame(
		ctx.vsink, 3000, memory, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(retFrame.coded);

	if (retFrame.coded != nullptr)
		mbuf_coded_video_frame_unref(retFrame.coded);
	if (memory != nullptr)
		mbuf_mem_unref(memory);
	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameCodedNonblockingEmpty(void)
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

	drainCodedQueue(ctx.vsink);

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

static void testGetFrameCodedBlockingConcurrentPush(void)
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

	drainCodedQueue(ctx.vsink);

	struct producer_ctx producer = {};
	producer.server = &server;
	pthread_create(&producer.thread, nullptr, producerThreadFn, &producer);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 5000, nullptr, &frameInfo, &retFrame);

	pthread_join(producer.thread, nullptr);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(retFrame.coded);

	if (retFrame.coded != nullptr)
		mbuf_coded_video_frame_unref(retFrame.coded);
	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameCodedTimeout(void)
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

	drainCodedQueue(ctx.vsink);

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 200, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -ETIMEDOUT);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testGetFrameCodedNoSinkYet(void)
{
	/* Backend-agnostic: exercises Vsink::getCodedFrame()'s own guard
	 * directly, no server/pipeline involved -- constructed but never
	 * start()ed, so mCoded.queue is still null. */
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	PdrawVsink::Vsink self(&params, nullptr);
	struct pdraw_video_frame frameInfo = {};
	struct mbuf_coded_video_frame *retFrame = nullptr;

	int res = self.getCodedFrame(0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EAGAIN);
}

CU_TestInfo s_get_frame_coded_tests[] = {
	{FN("get_frame(coded): delivered within timeout (real pipeline)"),
	 &testGetFrameCodedDeliveredWithinTimeout},
	{FN("get_frame(coded): caller-provided frame_memory"),
	 &testGetFrameCodedCallerProvidedMemory},
	{FN("get_frame(coded): timeout_ms=0, empty queue -> -EAGAIN"),
	 &testGetFrameCodedNonblockingEmpty},
	{FN("get_frame(coded): blocking wait, frame arrives concurrently "
	    "(real pipeline)"),
	 &testGetFrameCodedBlockingConcurrentPush},
	{FN("get_frame(coded): -ETIMEDOUT"), &testGetFrameCodedTimeout},
	{FN("get_frame(coded): -EAGAIN when no sink/queue exists yet"),
	 &testGetFrameCodedNoSinkYet},
	CU_TEST_INFO_NULL,
};
