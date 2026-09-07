/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Coded video source API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_coded_video_source
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_coded_video_source_*) ─────────────────────────── */

static void testCApiNew()
{
	struct pdraw_video_source_params params = {};
	struct pdraw_coded_video_source *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_coded_video_source_new(nullptr,
					   &params,
					   &g_valid_coded_video_source_cbs,
					   nullptr,
					   &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_coded_video_source_new(g_test_pdraw_c,
					   nullptr,
					   &g_valid_coded_video_source_cbs,
					   nullptr,
					   &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_coded_video_source_new(
		g_test_pdraw_c, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullFlushedCb */
	ret = pdraw_coded_video_source_new(g_test_pdraw_c,
					   &params,
					   &g_stub_coded_video_source_cbs,
					   nullptr,
					   &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_coded_video_source_new(g_test_pdraw_c,
					   &params,
					   &g_valid_coded_video_source_cbs,
					   nullptr,
					   nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_coded_video_source_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_coded_video_source_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiFlush()
{
	int ret = pdraw_coded_video_source_flush(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDrain()
{
	int ret = pdraw_coded_video_source_drain(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* get_queue(): previously had no null-arg test at all (0 hits in gcov on
 * both the null-pdraw and null-source guards). */
static void testCApiGetQueue()
{
	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_source_get_queue(nullptr, nullptr);
	CU_ASSERT_PTR_NULL(q);
	q = pdraw_coded_video_source_get_queue(g_test_pdraw_c, nullptr);
	CU_ASSERT_PTR_NULL(q);
}


static void testCApiSetSessionMeta()
{
	int ret = pdraw_coded_video_source_set_session_metadata(
		g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiGetSessionMeta()
{
	int ret = pdraw_coded_video_source_get_session_metadata(
		g_test_pdraw_c, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiNewValid()
{
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	struct pdraw_coded_video_source *obj = nullptr;
	int ret = pdraw_coded_video_source_new(g_test_pdraw_c,
					       &params,
					       &g_valid_coded_video_source_cbs,
					       nullptr,
					       &obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(obj);
	if (obj)
		pdraw_coded_video_source_destroy(g_test_pdraw_c, obj);
}


static void testCApiMethodsValid()
{
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	struct pdraw_coded_video_source *obj = nullptr;
	int ret = pdraw_coded_video_source_new(g_test_pdraw_c,
					       &params,
					       &g_valid_coded_video_source_cbs,
					       nullptr,
					       &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_source_get_queue(g_test_pdraw_c, obj);
	CU_ASSERT_PTR_NOT_NULL(q);

	struct vmeta_session meta_in = {};
	ret = pdraw_coded_video_source_set_session_metadata(
		g_test_pdraw_c, obj, &meta_in);
	CU_ASSERT_EQUAL(ret, 0);

	struct vmeta_session meta_out = {};
	ret = pdraw_coded_video_source_get_session_metadata(
		g_test_pdraw_c, obj, &meta_out);
	CU_ASSERT_EQUAL(ret, 0);

	pdraw_coded_video_source_destroy(g_test_pdraw_c, obj);
}


/* ── C++ API (IPdraw::createCodedVideoSource) ───────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	IPdraw::ICodedVideoSource *obj = nullptr;
	int ret;

	/* NullParams */
	ret = session->createCodedVideoSource(
		nullptr, &g_stub_coded_video_source_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullListener */
	ret = session->createCodedVideoSource(&params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCxxCreateValid()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	IPdraw::ICodedVideoSource *rawObj = nullptr;
	int ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, &rawObj);
	CU_ASSERT_EQUAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSource> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL(obj.get());
}


static void testCxxSetSessionMetaNull()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	IPdraw::ICodedVideoSource *rawObj = nullptr;
	int ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSource> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	CU_ASSERT_EQUAL(obj->setSessionMetadata(nullptr), -EINVAL);
}


static void testCxxGetSessionMetaNull()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	IPdraw::ICodedVideoSource *rawObj = nullptr;
	int ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSource> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	CU_ASSERT_EQUAL(obj->getSessionMetadata(nullptr), -EINVAL);
}


static void testCxxSessionMetaRoundtrip()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	IPdraw::ICodedVideoSource *rawObj = nullptr;
	int ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSource> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	struct vmeta_session meta_in = {};
	CU_ASSERT_EQUAL(obj->setSessionMetadata(&meta_in), 0);

	struct vmeta_session meta_out = {};
	CU_ASSERT_EQUAL(obj->getSessionMetadata(&meta_out), 0);
}


static void testCxxGetQueueNotNull()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_CODED;
	IPdraw::ICodedVideoSource *rawObj = nullptr;
	int ret = session->createCodedVideoSource(
		&params, &g_stub_coded_video_source_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSource> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	CU_ASSERT_PTR_NOT_NULL(obj->getQueue());
}


CU_TestInfo g_pdraw_test_api_coded_video_source[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiFlush"), testCApiFlush},
	{FN("testCApiDrain"), testCApiDrain},
	{FN("testCApiGetQueue"), testCApiGetQueue},
	{FN("testCApiSetSessionMeta"), testCApiSetSessionMeta},
	{FN("testCApiGetSessionMeta"), testCApiGetSessionMeta},
	{FN("testCApiNewValid"), testCApiNewValid},
	{FN("testCApiMethodsValid"), testCApiMethodsValid},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxCreateValid"), testCxxCreateValid},
	{FN("testCxxSetSessionMetaNull"), testCxxSetSessionMetaNull},
	{FN("testCxxGetSessionMetaNull"), testCxxGetSessionMetaNull},
	{FN("testCxxSessionMetaRoundtrip"), testCxxSessionMetaRoundtrip},
	{FN("testCxxGetQueueNotNull"), testCxxGetQueueNotNull},
	CU_TEST_INFO_NULL,
};
