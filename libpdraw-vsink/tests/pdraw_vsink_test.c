/**
 * Parrot Drones Awesome Video Viewer
 * Video sink wrapper library test program
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
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

#include <errno.h>
#include <unistd.h>

#define ULOG_TAG pdraw_vsink_test
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_vsink_test);

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw/pdraw_defs.h>
#include <video-defs/vdefs.h>
#include <video-metadata/vmeta.h>


int main(int argc, char **argv)
{
	int status = EXIT_SUCCESS, res, i;
	struct pdraw_vsink *vsink = NULL;

	if (argc < 2) {
		ULOGE("usage: %s <url>", argv[0]);
		exit(EXIT_FAILURE);
	}

	res = pdraw_vsink_start(argv[1], &vsink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_vsink_start", -res);
		exit(EXIT_FAILURE);
	}
	ULOGI("started");

	for (i = 0; i < 20; i++) {
		struct pdraw_video_frame frame_info = {0};
		struct mbuf_raw_video_frame *frame = NULL;
		struct vmeta_frame *frame_meta = NULL;
		unsigned int plane_count;

		/* Get a new frame */
		res = pdraw_vsink_get_frame(vsink, NULL, &frame_info, &frame);
		if (res < 0) {
			ULOG_ERRNO("pdraw_vsink_get_frame", -res);
			continue;
		}

		/* Get frame information */
		ULOGI("frame #%d (width=%d height=%d)",
		      i + 1,
		      frame_info.raw.info.resolution.width,
		      frame_info.raw.info.resolution.height);

		/* Get the video metadata */
		res = mbuf_raw_video_frame_get_metadata(frame, &frame_meta);
		if (res < 0 && res != -ENOENT)
			ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		if (frame_meta != NULL) {
			uint8_t battery_percentage;
			vmeta_frame_get_battery_percentage(frame_meta,
							   &battery_percentage);
			ULOGI("metadata: battery_percentage=%d%%",
			      battery_percentage);
		}

		/* Get the frame planes */
		plane_count =
			vdef_get_raw_frame_plane_count(&frame_info.raw.format);
		for (unsigned int k = 0; k < plane_count; k++) {
			const void *plane = NULL;
			size_t plane_len;
			res = mbuf_raw_video_frame_get_plane(
				frame, k, &plane, &plane_len);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_get_plane(%u)",
					   -res,
					   k);
				continue;
			}
			ULOGI("plane[%d]: addr=%p stride=%zu",
			      k,
			      plane,
			      frame_info.raw.plane_stride[k]);
			res = mbuf_raw_video_frame_release_plane(
				frame, k, plane);
			if (res < 0)
				ULOG_ERRNO(
					"mbuf_raw_video_frame_release_plane(%u)",
					-res,
					k);
		}

		vmeta_frame_unref(frame_meta);
		mbuf_raw_video_frame_unref(frame);
	}

	res = pdraw_vsink_stop(vsink);
	if (res < 0)
		ULOG_ERRNO("pdraw_vsink_stop", -res);

	ULOGI("%s", (status == EXIT_SUCCESS) ? "success!" : "failed!");
	exit(status);
}
