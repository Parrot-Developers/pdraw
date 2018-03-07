/**
 * Parrot Drones Awesome Video Viewer Library
 * Broadcom VideoCore 4 EGL renderer
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_renderer_videocoreegl.hpp"
#include "pdraw_session.hpp"

#ifdef USE_VIDEOCOREEGL

#include <string.h>
#include <unistd.h>
#include <time.h>
#define ULOG_TAG libpdraw
#include <ulog.h>

namespace Pdraw {


VideoCoreEglRenderer::VideoCoreEglRenderer(
	Session *session) :
	Gles2Renderer(session)
{
	mDisplay = EGL_NO_DISPLAY;
}


VideoCoreEglRenderer::~VideoCoreEglRenderer(
	void)
{
}


int VideoCoreEglRenderer::open(
	unsigned int windowWidth,
	unsigned int windowHeight,
	int renderX,
	int renderY,
	unsigned int renderWidth,
	unsigned int renderHeight,
	bool hud,
	bool hmdDistorsionCorrection,
	bool headtracking,
	struct egl_display *eglDisplay)
{
	if (eglDisplay == NULL) {
		ULOGE("VideoCoreEglRenderer: invalid EGL display");
		return -EINVAL;
	}
	mDisplay = (EGLDisplay)eglDisplay;

	/* HMD distorsion correction is not supported with VideoCore 4 */
	hmdDistorsionCorrection = false; /* TODO */

	return Gles2Renderer::open(windowWidth, windowHeight,
		renderX, renderY, renderWidth, renderHeight, hud,
		hmdDistorsionCorrection, headtracking, eglDisplay);
}


int VideoCoreEglRenderer::loadVideoFrame(
	const uint8_t *data,
	struct avcdecoder_output_buffer *frame,
	enum gles2_video_color_conversion colorConversion)
{
	int ret;
	ret = mGles2Video->loadFrame(data, frame->plane_offset, frame->stride,
		frame->width, frame->height, colorConversion,
		(struct egl_display *)mDisplay);
	if (ret < 0)
		ULOGE("VideoCoreEglRenderer: failed to load video frame");
	return 0;
}

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREEGL */
