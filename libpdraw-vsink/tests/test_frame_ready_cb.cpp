/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- frame_ready callback vs. get_frame() -EPERM
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
#include <time.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>

struct frame_ready_ctx {
	pthread_mutex_t lock;
	pthread_cond_t cond;
	bool called;
	int callCount;
	enum pdraw_vsink_video_media_type lastType;
};

static void frameReadyCb(struct pdraw_vsink_frame *frame,
			 const struct pdraw_video_frame *frameInfo,
			 void *userdata)
{
	(void)frameInfo;

	struct frame_ready_ctx *ctx =
		static_cast<struct frame_ready_ctx *>(userdata);

	pthread_mutex_lock(&ctx->lock);
	ctx->callCount++;
	ctx->called = true;
	ctx->lastType = frame->type;
	pthread_cond_signal(&ctx->cond);
	pthread_mutex_unlock(&ctx->lock);
}

static int waitForFrameReady(struct frame_ready_ctx *ctx, int timeoutMs)
{
	struct timespec ts = {};
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += timeoutMs / 1000;
	ts.tv_nsec += (timeoutMs % 1000) * 1000 * 1000;
	if (ts.tv_nsec >= 1000000000) {
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000;
	}

	pthread_mutex_lock(&ctx->lock);
	int res = 0;
	while (!ctx->called && res == 0)
		res = pthread_cond_timedwait(&ctx->cond, &ctx->lock, &ts);
	pthread_mutex_unlock(&ctx->lock);
	return res;
}

static void testFrameReadyCbInvokedCoded(void)
{
	struct frame_ready_ctx cbCtx = {PTHREAD_MUTEX_INITIALIZER,
					PTHREAD_COND_INITIALIZER,
					false,
					0,
					PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW};

	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
	ctx.cbs.frame_ready = &frameReadyCb;
	ctx.cbsUserdata = &cbCtx;

	startRealVsinkAndWait(&server, &ctx);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	/* The IDR access unit sent by startRealVsinkAndWait() to trigger
	 * media_added may itself already have reached the callback by now;
	 * send one more to guarantee at least one call happens after this
	 * point regardless of that race. */
	server.sendVideoIdrAccessUnit();

	CU_ASSERT_EQUAL(waitForFrameReady(&cbCtx, 5000), 0);
	CU_ASSERT_TRUE(cbCtx.callCount >= 1);
	CU_ASSERT_EQUAL(cbCtx.lastType, PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

/* RAW mode needs a real, working H.264 decoder in this build -- see the
 * comment on testStartSuccessRaw() in test_start.cpp. */
static void testFrameReadyCbInvokedRaw(void)
{
	struct frame_ready_ctx cbCtx = {PTHREAD_MUTEX_INITIALIZER,
					PTHREAD_COND_INITIALIZER,
					false,
					0,
					PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW};

	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;
	ctx.cbs.frame_ready = &frameReadyCb;
	ctx.cbsUserdata = &cbCtx;

	startRealVsinkAndWait(&server, &ctx, 10000);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	server.sendVideoIdrAccessUnit();

	CU_ASSERT_EQUAL(waitForFrameReady(&cbCtx, 5000), 0);
	CU_ASSERT_TRUE(cbCtx.callCount >= 1);
	CU_ASSERT_EQUAL(cbCtx.lastType, PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testFrameReadyCbGetFrameEperm(void)
{
	struct frame_ready_ctx cbCtx = {PTHREAD_MUTEX_INITIALIZER,
					PTHREAD_COND_INITIALIZER,
					false,
					0,
					PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW};

	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
	ctx.cbs.frame_ready = &frameReadyCb;
	ctx.cbsUserdata = &cbCtx;

	startRealVsinkAndWait(&server, &ctx);
	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	if (ctx.vsink == nullptr) {
		pdraw_media_info_free(ctx.mediaInfo);
		return;
	}

	struct pdraw_video_frame frameInfo = {};
	struct pdraw_vsink_frame retFrame = {};
	int res = pdraw_vsink_get_frame(
		ctx.vsink, 0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EPERM);

	pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

/* Dummy IPdrawVsink::Listener implementation: only used to make
 * mListener != nullptr in the two whitebox tests below (no frame is ever
 * dispatched to it, so onFrameReady() is never actually called). */
class DummyListener : public PdrawVsink::IPdrawVsink::Listener {
public:
	void onFrameReady(PdrawVsink::IPdrawVsink *vsink,
			  struct pdraw_vsink_frame *frame,
			  const struct pdraw_video_frame *frameInfo) override
	{
		(void)vsink;
		(void)frame;
		(void)frameInfo;
	}
};

/* Backend-agnostic: exercises Vsink::getRawFrame()'s check ordering
 * directly, no server/pipeline involved -- the mRaw.queue == nullptr
 * guard must run *before* the mListener -EPERM check, so a caller that
 * registered a listener but hasn't seen a matching media added yet still
 * gets -EAGAIN, not -EPERM. */
static void testFrameReadyCbGetRawFrameEagainBeforeMediaAdded(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	DummyListener listener;
	PdrawVsink::Vsink self(&params, &listener);

	struct pdraw_video_frame frameInfo = {};
	struct mbuf_raw_video_frame *retFrame = nullptr;

	int res = self.getRawFrame(0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EAGAIN);
}

static void testFrameReadyCbGetCodedFrameEagainBeforeMediaAdded(void)
{
	struct pdraw_vsink_params params = {};
	params.url = "unused";
	DummyListener listener;
	PdrawVsink::Vsink self(&params, &listener);

	struct pdraw_video_frame frameInfo = {};
	struct mbuf_coded_video_frame *retFrame = nullptr;

	int res = self.getCodedFrame(0, nullptr, &frameInfo, &retFrame);

	CU_ASSERT_EQUAL(res, -EAGAIN);
}

CU_TestInfo s_frame_ready_cb_tests[] = {
	{FN("frame_ready_cb: invoked with correct data (coded, real "
	    "pipeline)"),
	 &testFrameReadyCbInvokedCoded},
	{FN("frame_ready_cb: invoked with correct data (raw, real "
	    "pipeline + decode)"),
	 &testFrameReadyCbInvokedRaw},
	{FN("frame_ready_cb: get_frame() -> -EPERM once registered"),
	 &testFrameReadyCbGetFrameEperm},
	{FN("frame_ready_cb: get_raw_frame() -> -EAGAIN before media_added "
	    "(not -EPERM)"),
	 &testFrameReadyCbGetRawFrameEagainBeforeMediaAdded},
	{FN("frame_ready_cb: get_coded_frame() -> -EAGAIN before "
	    "media_added (not -EPERM)"),
	 &testFrameReadyCbGetCodedFrameEagainBeforeMediaAdded},
	CU_TEST_INFO_NULL,
};
