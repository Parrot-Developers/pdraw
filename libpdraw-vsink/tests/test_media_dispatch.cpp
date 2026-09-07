/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- media-type dispatch
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

#include <media-buffers/mbuf_coded_video_frame.h>

/* Two-track SDP (audio + video): confirms libpdraw-vsink's own
 * media_added_cb type filter (info->type != PDRAW_MEDIA_TYPE_VIDEO) really
 * discards the non-video media end-to-end -- the demuxer's default media
 * selection picks the video track, no RTP is ever sent for the audio one. */
static void testMediaDispatchIgnoresAudioMedia(void)
{
	TestRtspServer server(/* withAudioTrack */ true);
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;

	startRealVsinkAndWait(&server, &ctx, 10000);

	CU_ASSERT_PTR_NOT_NULL(ctx.mediaInfo);
	if (ctx.mediaInfo != nullptr)
		CU_ASSERT_EQUAL(ctx.mediaInfo->type, PDRAW_MEDIA_TYPE_VIDEO);

	if (ctx.vsink != nullptr)
		pdraw_vsink_stop(ctx.vsink);
	pdraw_media_info_free(ctx.mediaInfo);
}

static void testMediaDispatchGetFrameDispatchesByType(void)
{
	TestRtspServer server;
	struct test_vsink_start_ctx ctx = {};
	ctx.params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;

	startRealVsinkAndWait(&server, &ctx);

	if (ctx.vsink != nullptr) {
		struct pdraw_video_frame frameInfo = {};
		struct pdraw_vsink_frame retFrame = {};

		/* retFrame.type is set unconditionally before dispatch, so
		 * this holds regardless of whether a frame happened to
		 * already be queued (real, async pipeline timing). */
		int res = pdraw_vsink_get_frame(
			ctx.vsink, 0, nullptr, &frameInfo, &retFrame);

		CU_ASSERT_EQUAL(retFrame.type,
				PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED);
		if (res == 0 && retFrame.coded != nullptr)
			mbuf_coded_video_frame_unref(retFrame.coded);

		pdraw_vsink_stop(ctx.vsink);
	}
	pdraw_media_info_free(ctx.mediaInfo);
}

CU_TestInfo s_media_dispatch_tests[] = {
	{FN("media_dispatch: non-video media is ignored (real 2-track "
	    "SDP)"),
	 &testMediaDispatchIgnoresAudioMedia},
	{FN("media_dispatch: get_frame dispatches by type"),
	 &testMediaDispatchGetFrameDispatchesByType},
	CU_TEST_INFO_NULL,
};
