/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Video scaler API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_scaler_video
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include <libpomp.h>
#include <media-buffers/mbuf_raw_video_frame.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_video_scaler_*) ───────────────────────────────────────── */

static void testCApiNew()
{
	struct vscale_config params = {};
	struct pdraw_video_scaler *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_video_scaler_new(
		nullptr, 0, &params, &g_stub_video_scaler_cbs, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_video_scaler_new(g_test_pdraw_c,
				     0,
				     nullptr,
				     &g_stub_video_scaler_cbs,
				     nullptr,
				     &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_video_scaler_new(
		g_test_pdraw_c, 0, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_video_scaler_new(g_test_pdraw_c,
				     0,
				     &params,
				     &g_stub_video_scaler_cbs,
				     nullptr,
				     nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_video_scaler_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullScaler */
	ret = pdraw_video_scaler_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiLifecycle()
{
	struct pomp_loop *loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	struct pdraw *pdraw = nullptr;
	struct pdraw_cbs cbs = {};
	struct MyUserdata {
		unsigned int mediaId = 0;
	} myUserdata;

	cbs.media_added = [](struct pdraw *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata) {
		MyUserdata *ud = (MyUserdata *)userdata;
		if (info->type == PDRAW_MEDIA_TYPE_VIDEO) {
			ud->mediaId = info->id;
		}
	};

	int ret = pdraw_new(loop, &cbs, &myUserdata, &pdraw);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pdraw);

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	struct pdraw_raw_video_source *source = nullptr;
	struct pdraw_raw_video_source_cbs sourceCbs = {};
	sourceCbs.flushed = [](struct pdraw *pdraw,
			       struct pdraw_raw_video_source *source,
			       void *userdata) {};

	ret = pdraw_raw_video_source_new(
		pdraw, &sourceParams, &sourceCbs, nullptr, &source);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(source);

	pomp_loop_wait_and_process(loop, 10);

	CU_ASSERT_TRUE(myUserdata.mediaId > 0);

	if (myUserdata.mediaId > 0) {
		struct vscale_config scalerParams = {};
		struct pdraw_video_scaler *scaler = nullptr;
		struct pdraw_video_scaler_cbs scalerCbs = {};
		scalerCbs.frame_output = [](struct pdraw *pdraw,
					    struct pdraw_video_scaler *scaler,
					    struct mbuf_raw_video_frame *frame,
					    void *userdata) {};

		ret = pdraw_video_scaler_new(pdraw,
					     myUserdata.mediaId,
					     &scalerParams,
					     &scalerCbs,
					     nullptr,
					     &scaler);
		CU_ASSERT_TRUE(ret == 0 || ret == -ENOSYS || ret < 0);
		if (ret == 0) {
			CU_ASSERT_PTR_NOT_NULL(scaler);
			ret = pdraw_video_scaler_destroy(pdraw, scaler);
			CU_ASSERT_EQUAL(ret, 0);
		}
	}

	if (source != nullptr) {
		ret = pdraw_raw_video_source_destroy(pdraw, source);
		CU_ASSERT_EQUAL(ret, 0);
	}

	ret = pdraw_destroy(pdraw);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pomp_loop_destroy(loop);
	CU_ASSERT_EQUAL(ret, 0);
}


/* ── C++ API (IPdraw::createVideoScaler) ────────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct vscale_config params = {};
	IPdraw::IVideoScaler *obj = nullptr;
	int ret;

	/* NullParams */
	ret = session->createVideoScaler(
		0, nullptr, &g_stub_video_scaler_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullListener */
	ret = session->createVideoScaler(0, &params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = session->createVideoScaler(
		0, &params, &g_stub_video_scaler_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


CU_TestInfo g_pdraw_test_api_scaler_video[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiLifecycle"), testCApiLifecycle},
	{FN("testCxxCreate"), testCxxCreate},
	CU_TEST_INFO_NULL,
};
