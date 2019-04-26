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

#include <pdraw-vsink/pdraw_vsink.h>
#include <video-buffers/vbuf.h>
#include <video-buffers/vbuf_generic.h>


int main(int argc, char **argv)
{
	int status = EXIT_SUCCESS, res, i;
	struct pdraw_vsink *vsink = NULL;
	struct vbuf_cbs vbuf_cbs;
	struct vbuf_buffer *buffer = NULL;

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

	res = vbuf_generic_get_cbs(&vbuf_cbs);
	if (res < 0) {
		ULOG_ERRNO("vbuf_generic_get_cbs", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	res = vbuf_new(0, 0, &vbuf_cbs, NULL, &buffer);
	if (res < 0) {
		ULOG_ERRNO("vbuf_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	for (i = 0; i < 100; i++) {
		struct pdraw_video_frame frame;
		memset(&frame, 0, sizeof(frame));
		res = pdraw_vsink_get_frame(vsink, -1, &frame, buffer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_vsink_get_frame", -res);
			continue;
		}
		ULOGI("got a frame (width=%d height=%d)",
		      frame.yuv.width,
		      frame.yuv.height);
	}

out:
	vbuf_unref(buffer);
	res = pdraw_vsink_stop(vsink);
	if (res < 0)
		ULOG_ERRNO("pdraw_vsink_stop", -res);

	ULOGI("%s", (status == EXIT_SUCCESS) ? "success!" : "failed!");
	exit(status);
}
