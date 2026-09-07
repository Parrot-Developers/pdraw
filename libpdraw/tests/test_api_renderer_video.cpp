/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Video renderer API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_renderer_video
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_video_renderer_*) ─────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_video_renderer *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_video_renderer_new(
		nullptr, 0, nullptr, nullptr, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_video_renderer_new(
		g_test_pdraw_c, 0, nullptr, nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	struct pdraw_video_renderer stub = {nullptr};
	ret = pdraw_video_renderer_destroy(nullptr, &stub);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRenderer */
	ret = pdraw_video_renderer_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiResize()
{
	int ret = pdraw_video_renderer_resize(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_resize(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiSetParams()
{
	int ret = pdraw_video_renderer_set_params(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_set_params(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiGetParams()
{
	int ret = pdraw_video_renderer_get_params(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_get_params(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiRender()
{
	int ret = pdraw_video_renderer_render(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_render(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* get_media_id()/set_media_id()/render_mat() previously had no test at all
 * (0 hits in gcov). Unlike most pdraw_video_renderer_*() wrappers,
 * get_media_id() returns a plain 0 rather than a negative errno, even for
 * null arguments. */
static void testCApiGetMediaId()
{
	CU_ASSERT_EQUAL(pdraw_video_renderer_get_media_id(nullptr, nullptr),
			0u);
	CU_ASSERT_EQUAL(
		pdraw_video_renderer_get_media_id(g_test_pdraw_c, nullptr), 0u);
}


static void testCApiSetMediaId()
{
	int ret = pdraw_video_renderer_set_media_id(nullptr, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_set_media_id(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiRenderMat()
{
	int ret = pdraw_video_renderer_render_mat(
		nullptr, nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_video_renderer_render_mat(
		g_test_pdraw_c, nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* ── C++ API (IPdraw::createVideoRenderer) ──────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	IPdraw::IVideoRenderer *obj = nullptr;
	int ret;

	/* NullListener */
	ret = session->createVideoRenderer(0, nullptr, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = session->createVideoRenderer(
		0, nullptr, nullptr, &g_stub_video_renderer_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCxxCreateValid()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_rect renderPos = {0, 0, 640, 480};
	struct pdraw_video_renderer_params params = {};
	IPdraw::IVideoRenderer *obj = nullptr;

	/* mediaId=0: no media attached yet, matching the doc'd behavior of
	 * using the first raw media found once one becomes available.
	 *
	 * NOTE: unlike the audio/ALSA renderer, GlVideoRenderer::setup()
	 * (called from its constructor) issues a real glGetIntegerv() call
	 * (see pdraw_renderer_video_gl.cpp), so a successful construction
	 * requires an actual bound GL context. A CUnit run has no such
	 * context and PDRAW_USE_GL alone does not guarantee one exists in
	 * whatever environment this binary runs in, so — unlike the ALSA
	 * case, where device opening is deferred past construction — we
	 * deliberately do not assert or exercise the PDRAW_USE_GL success
	 * path here: doing so could crash the test binary rather than just
	 * fail an assertion. Only the deterministic not-compiled-in path is
	 * verified. */
	int ret = session->createVideoRenderer(
		0, &renderPos, &params, &g_stub_video_renderer_listener, &obj);
	auto objOwner = std::unique_ptr<IPdraw::IVideoRenderer>(obj);
#ifndef PDRAW_USE_GL
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
#else
	(void)ret;
#endif
}


CU_TestInfo g_pdraw_test_api_renderer_video[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiResize"), testCApiResize},
	{FN("testCApiSetParams"), testCApiSetParams},
	{FN("testCApiGetParams"), testCApiGetParams},
	{FN("testCApiRender"), testCApiRender},
	{FN("testCApiGetMediaId"), testCApiGetMediaId},
	{FN("testCApiSetMediaId"), testCApiSetMediaId},
	{FN("testCApiRenderMat"), testCApiRenderMat},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxCreateValid"), testCxxCreateValid},
	CU_TEST_INFO_NULL,
};
