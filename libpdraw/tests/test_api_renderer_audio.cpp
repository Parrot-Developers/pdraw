/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Audio renderer API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_renderer_audio
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_audio_renderer_*) ─────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_audio_renderer *obj = nullptr;
	int ret = pdraw_audio_renderer_new(
		nullptr, 0, nullptr, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_audio_renderer_new(
		g_test_pdraw_c, 0, nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	struct pdraw_audio_renderer stub = {nullptr};
	int ret = pdraw_audio_renderer_destroy(nullptr, &stub);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_audio_renderer_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiSetParams()
{
	int ret = pdraw_audio_renderer_set_params(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_audio_renderer_set_params(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiGetParams()
{
	int ret = pdraw_audio_renderer_get_params(nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_audio_renderer_get_params(g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* get_media_id(): previously had no test at all (0 hits in gcov on both
 * the null-pdraw and null-renderer guards). */
static void testCApiGetMediaId()
{
	CU_ASSERT_EQUAL(pdraw_audio_renderer_get_media_id(nullptr, nullptr),
			0u);
	CU_ASSERT_EQUAL(
		pdraw_audio_renderer_get_media_id(g_test_pdraw_c, nullptr), 0u);
}


/* ── C++ API (IPdraw::createAudioRenderer) ──────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	IPdraw::IAudioRenderer *obj = nullptr;
	int ret = session->createAudioRenderer(0, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = session->createAudioRenderer(
		0, nullptr, &g_stub_audio_renderer_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiMethodsValid()
{
	struct pdraw_audio_renderer_params params = {};
	params.address = "default";
	struct pdraw_audio_renderer *obj = nullptr;

	int ret = pdraw_audio_renderer_new(g_test_pdraw_c,
					   0,
					   &params,
					   &g_stub_audio_renderer_cbs,
					   nullptr,
					   &obj);
	CU_ASSERT_AUDIO_RENDERER_CREATE_GUARD(ret);

#ifdef PDRAW_USE_ALSA
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	unsigned int id =
		pdraw_audio_renderer_get_media_id(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(id, 0u);

	ret = pdraw_audio_renderer_set_media_id(g_test_pdraw_c, obj, 42);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_audio_renderer_set_params(g_test_pdraw_c, obj, &params);
	CU_ASSERT_EQUAL(ret, 0);

	struct pdraw_audio_renderer_params outParams = {};
	ret = pdraw_audio_renderer_get_params(g_test_pdraw_c, obj, &outParams);
	CU_ASSERT_EQUAL(ret, 0);

	pdraw_audio_renderer_destroy(g_test_pdraw_c, obj);
#endif
}


static void testCxxCreateValid()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_audio_renderer_params params = {};
	params.address = "default";
	IPdraw::IAudioRenderer *rawObj = nullptr;

	/* mediaId=0: no media attached yet, matching the doc'd behavior of
	 * using the first audio media found once one becomes available. */
	int ret = session->createAudioRenderer(
		0, &params, &g_stub_audio_renderer_listener, &rawObj);
	CU_ASSERT_AUDIO_RENDERER_CREATE_GUARD(ret);

#ifdef PDRAW_USE_ALSA
	std::unique_ptr<IPdraw::IAudioRenderer> obj(rawObj);

	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());
	CU_ASSERT_EQUAL(obj->getMediaId(), 0u);
	CU_ASSERT_EQUAL(obj->setMediaId(42), 0);
	CU_ASSERT_EQUAL(obj->setParams(nullptr), -EINVAL);
	CU_ASSERT_EQUAL(obj->setParams(&params), 0);
	struct pdraw_audio_renderer_params outParams = {};
	CU_ASSERT_EQUAL(obj->getParams(&outParams), 0);
	CU_ASSERT_EQUAL(obj->getParams(nullptr), 0);
#else
	CU_ASSERT_PTR_NULL(rawObj);
#endif
}


CU_TestInfo g_pdraw_test_api_renderer_audio[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiSetParams"), testCApiSetParams},
	{FN("testCApiGetParams"), testCApiGetParams},
	{FN("testCApiGetMediaId"), testCApiGetMediaId},
	{FN("testCApiMethodsValid"), testCApiMethodsValid},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxCreateValid"), testCxxCreateValid},
	CU_TEST_INFO_NULL,
};
