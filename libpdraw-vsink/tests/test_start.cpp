/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- pdraw_vsink_start() lifecycle
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

static void testStartInvalidParams(void)
{
	struct pdraw_media_info *mediaInfo = nullptr;
	struct pdraw_vsink *vsink = nullptr;
	struct pdraw_vsink_params params = {};
	params.url = "rtsp://127.0.0.1/test";

	CU_ASSERT_EQUAL(pdraw_vsink_start(nullptr, nullptr, nullptr, &mediaInfo, &vsink),
			-EINVAL);

	struct pdraw_vsink_params noUrl = {};
	CU_ASSERT_EQUAL(
		pdraw_vsink_start(&noUrl, nullptr, nullptr, &mediaInfo, &vsink),
		-EINVAL);

	CU_ASSERT_EQUAL(pdraw_vsink_start(&params, nullptr, nullptr, nullptr, nullptr),
			-EINVAL);
}

static void testStartSuccessCoded(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;

	startRealVsinkAndWait(&server, &ctx);

	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	CU_ASSERT_PTR_NOT_NULL(ctx.mediaInfo);
	if (ctx.mediaInfo != nullptr)
		CU_ASSERT_EQUAL(ctx.mediaInfo->type, PDRAW_MEDIA_TYPE_VIDEO);

	if (ctx.vsink != nullptr)
		pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

/* Unlike the CODED case above, RAW requires libpdraw-vsink's requested
 * PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL to actually succeed, i.e. a
 * real, working H.264 decoder backend must be present in this build -- this
 * is a deliberate, meaningful dependency (it validates the real decode
 * integration path too), not an oversight; if this is the only test
 * failing while testStartSuccessCoded passes, check the product's video
 * decoder configuration first. */
static void testStartSuccessRaw(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;

	startRealVsinkAndWait(&server, &ctx, 10000);

	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	CU_ASSERT_PTR_NOT_NULL(ctx.mediaInfo);
	if (ctx.mediaInfo != nullptr)
		CU_ASSERT_EQUAL(ctx.mediaInfo->type, PDRAW_MEDIA_TYPE_VIDEO);

	if (ctx.vsink != nullptr)
		pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

/* select_media_cb's is_default branch requires self->camera_type ==
 * VMETA_CAMERA_TYPE_UNKNOWN, so requesting a specific, non-UNKNOWN
 * camera_type against a server whose SDP carries no matching vmeta
 * camera_type metadata (the plain default TestRtspServer below never
 * embeds any) skips that branch and falls through to no match at all --
 * a real "no media selected" -ECANCELED, which per pdraw_demuxer_cbs'
 * documented contract also drives open_resp_cb's status != 0 branch.
 * No RTP is ever sent/expected here: with no media selected, the real
 * demuxer has no reason to ever issue a SETUP request. */
static void testStartSelectMediaNoMatch(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
	ctx.params.camera_type = VMETA_CAMERA_TYPE_FRONT;

	testVsinkStartAsync(&ctx);
	testVsinkStartJoin(&ctx);

	CU_ASSERT_TRUE(ctx.result < 0);
	CU_ASSERT_PTR_NULL(ctx.vsink);
}

/* Same reasoning as above for why the is_default branch is skipped, but
 * this time the server's video track carries a real, SDP-embedded vmeta
 * camera_type attribute (see TestRtspServer's videoCameraType
 * constructor parameter) matching what's requested -- the only way
 * selection can still succeed is via select_media_cb's explicit
 * camera_type-match branch. */
static void testStartSelectMediaCameraTypeMatch(void)
{
	TestRtspServer server(
		/* withAudioTrack */ false, VMETA_CAMERA_TYPE_FRONT);
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
	ctx.params.camera_type = VMETA_CAMERA_TYPE_FRONT;

	startRealVsinkAndWait(&server, &ctx);

	CU_ASSERT_PTR_NOT_NULL(ctx.vsink);
	CU_ASSERT_PTR_NOT_NULL(ctx.mediaInfo);

	if (ctx.vsink != nullptr)
		pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

CU_TestInfo s_start_tests[] = {
	{FN("start: invalid params -> -EINVAL"), &testStartInvalidParams},
	{FN("start: success (coded, real RTSP+RTP)"), &testStartSuccessCoded},
	{FN("start: success (raw, real RTSP+RTP + decode)"),
	 &testStartSuccessRaw},
	{FN("start: select_media no match -> -ECANCELED / open_resp error"),
	 &testStartSelectMediaNoMatch},
	{FN("start: select_media camera_type match (real vmeta SDP attr)"),
	 &testStartSelectMediaCameraTypeMatch},
	CU_TEST_INFO_NULL,
};
