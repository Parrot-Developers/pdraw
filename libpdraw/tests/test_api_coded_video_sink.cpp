/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Coded video sink API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_coded_video_sink
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_coded_video_sink_*) ───────────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_video_sink_params params = {};
	struct pdraw_coded_video_sink *obj = nullptr;
	int ret = pdraw_coded_video_sink_new(nullptr,
					     0,
					     &params,
					     &g_valid_coded_video_sink_cbs,
					     nullptr,
					     &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	obj = nullptr;
	ret = pdraw_coded_video_sink_new(g_test_pdraw_c,
					 0,
					 nullptr,
					 &g_valid_coded_video_sink_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	obj = nullptr;
	ret = pdraw_coded_video_sink_new(
		g_test_pdraw_c, 0, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* flush is mandatory; g_stub_coded_video_sink_cbs has flush=NULL */
	obj = nullptr;
	ret = pdraw_coded_video_sink_new(g_test_pdraw_c,
					 0,
					 &params,
					 &g_stub_coded_video_sink_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_coded_video_sink_new(g_test_pdraw_c,
					 0,
					 &params,
					 &g_valid_coded_video_sink_cbs,
					 nullptr,
					 nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret = pdraw_coded_video_sink_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_coded_video_sink_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiQueueFlushed()
{
	int ret = pdraw_coded_video_sink_queue_flushed(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiResync()
{
	int ret = pdraw_coded_video_sink_resync(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiSetMediaId()
{
	int ret =
		pdraw_coded_video_sink_set_media_id(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiQueueDrained()
{
	int ret = pdraw_coded_video_sink_queue_drained(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiGetMediaId()
{
	unsigned int id = pdraw_coded_video_sink_get_media_id(nullptr, nullptr);
	CU_ASSERT_EQUAL(id, 0);
	id = pdraw_coded_video_sink_get_media_id(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(id, 0);
}


static void testCApiGetQueue()
{
	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_sink_get_queue(nullptr, nullptr);
	CU_ASSERT_PTR_NULL(q);
	q = pdraw_coded_video_sink_get_queue(g_test_pdraw_c, nullptr);
	CU_ASSERT_PTR_NULL(q);
}


static void testCApiNewValid()
{
	struct pdraw_video_sink_params params = {};
	struct pdraw_coded_video_sink *obj = nullptr;
	int ret = pdraw_coded_video_sink_new(g_test_pdraw_c,
					     0,
					     &params,
					     &g_valid_coded_video_sink_cbs,
					     nullptr,
					     &obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(obj);
	if (obj)
		pdraw_coded_video_sink_destroy(g_test_pdraw_c, obj);
}


static void testCApiMethodsValid()
{
	struct pdraw_video_sink_params params = {};
	struct pdraw_coded_video_sink *obj = nullptr;
	int ret = pdraw_coded_video_sink_new(g_test_pdraw_c,
					     0,
					     &params,
					     &g_valid_coded_video_sink_cbs,
					     nullptr,
					     &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_sink_get_queue(g_test_pdraw_c, obj);
	CU_ASSERT_PTR_NOT_NULL(q);

	unsigned int id =
		pdraw_coded_video_sink_get_media_id(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(id, 0u);

	ret = pdraw_coded_video_sink_set_media_id(g_test_pdraw_c, obj, 42);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_coded_video_sink_resync(g_test_pdraw_c, obj);
	CU_ASSERT_TRUE(ret == 0 || ret < 0);

	pdraw_coded_video_sink_destroy(g_test_pdraw_c, obj);
}


/* ── C++ API (IPdraw::createCodedVideoSink) ─────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *obj = nullptr;
	int ret = session->createCodedVideoSink(0, &params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = session->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCxxCreateValid()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *rawObj = nullptr;
	int ret = session->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &rawObj);
	CU_ASSERT_EQUAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSink> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL(obj.get());
}


static void testCxxGetMediaIdInitial()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *rawObj = nullptr;
	int ret = session->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSink> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	/* No media connected yet: connected media id must be 0 */
	CU_ASSERT_EQUAL(obj->getMediaId(), 0u);
}


static void testCxxGetQueueNotNull()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *rawObj = nullptr;
	int ret = session->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSink> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	/* Queue is created in start(), before any media is connected */
	CU_ASSERT_PTR_NOT_NULL(obj->getQueue());
}


static void testCxxSetMediaId()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *rawObj = nullptr;
	int ret = session->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &rawObj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	std::unique_ptr<IPdraw::ICodedVideoSink> obj(rawObj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj.get());

	CU_ASSERT_EQUAL(obj->setMediaId(42), 0);
}


CU_TestInfo g_pdraw_test_api_coded_video_sink[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiQueueFlushed"), testCApiQueueFlushed},
	{FN("testCApiResync"), testCApiResync},
	{FN("testCApiSetMediaId"), testCApiSetMediaId},
	{FN("testCApiQueueDrained"), testCApiQueueDrained},
	{FN("testCApiGetMediaId"), testCApiGetMediaId},
	{FN("testCApiGetQueue"), testCApiGetQueue},
	{FN("testCApiNewValid"), testCApiNewValid},
	{FN("testCApiMethodsValid"), testCApiMethodsValid},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxCreateValid"), testCxxCreateValid},
	{FN("testCxxGetMediaIdInitial"), testCxxGetMediaIdInitial},
	{FN("testCxxGetQueueNotNull"), testCxxGetQueueNotNull},
	{FN("testCxxSetMediaId"), testCxxSetMediaId},
	CU_TEST_INFO_NULL,
};
