/**
 * Parrot Drones Awesome Video Viewer Library
 * Broadcom VideoCore 4 EGL renderer
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

#define ULOG_TAG pdraw_rndvidvcgl
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_renderer_videocoreegl.hpp"
#include "pdraw_session.hpp"

#ifdef USE_VIDEOCOREEGL

#	include <string.h>
#	include <time.h>
#	include <unistd.h>

namespace Pdraw {


VideoCoreEglRenderer::VideoCoreEglRenderer(
	Session *session,
	Element::Listener *listener,
	IPdraw::IVideoRenderer *renderer,
	IPdraw::IVideoRenderer::Listener *rndListener,
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	struct egl_display *eglDisplay) :
		Gles2Renderer(session,
			      listener,
			      renderer,
			      rndListener,
			      mediaId,
			      nullptr,
			      nullptr,
			      nullptr)
{
	Element::setClassName(__func__);
	mDisplay = EGL_NO_DISPLAY;
	setRawVideoMediaFormatCaps(VDEF_BCM_MMAL_OPAQUE);
	setup(renderPos, params, eglDisplay);
}


VideoCoreEglRenderer::~VideoCoreEglRenderer(void) {}


int VideoCoreEglRenderer::setup(
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	struct egl_display *eglDisplay)
{
	if (eglDisplay == nullptr) {
		PDRAW_LOGE("invalid EGL display");
		return -EINVAL;
	}
	mDisplay = (EGLDisplay)eglDisplay;

	struct pdraw_video_renderer_params _params = *params;
	/* HMD distortion correction is not supported with VideoCore 4 */
	_params.enableHmdDistortionCorrection = 0; /* TODO */

	return Gles2Renderer::setup(renderPos, &_params, eglDisplay);
}


int VideoCoreEglRenderer::loadVideoFrame(const uint8_t *data,
					 RawVideoMedia::Frame *frame)
{
	int ret;

	mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_NONE;

	if (vdef_dim_is_null(&frame->frame.info.resolution) ||
	    vdef_dim_is_null(&frame->frame.info.sar)) {
		PDRAW_LOGE("invalid frame dimensions");
		return -EINVAL;
	}

	ret = mGles2Video->loadFrame(data,
				     nullptr,
				     nullptr,
				     frame->frame.info.resolution.width,
				     frame->frame.info.resolution.height,
				     mColorConversion,
				     (struct egl_display *)mDisplay);
	if (ret < 0)
		PDRAW_LOG_ERRNO("gles2Video->loadFrame", -ret);

	return 0;
}


int VideoCoreEglRenderer::loadExternalVideoFrame(
	const uint8_t *data,
	RawVideoMedia::Frame *frame,
	const struct pdraw_media_info *media_info)
{
	return -ENOSYS;
}


int VideoCoreEglRenderer::renderVideoFrame(RawVideoMedia::Frame *frame,
					   const struct pdraw_rect *renderPos,
					   struct pdraw_rect *contentPos,
					   Eigen::Matrix4f &viewProjMat)
{
	enum gles2_video_yuv_range yuvRange =
		frame->opaqueFrame.fullRange ? GLES2_VIDEO_YUV_FULL_RANGE
					     : GLES2_VIDEO_YUV_LIMITED_RANGE;

	return mGles2Video->renderFrame(&frame->frame.plane_stride,
					frame->frame.info.resolution.height,
					0,
					0,
					frame->frame.info.resolution.width,
					frame->frame.info.resolution.height,
					frame->frame.info.sar.width,
					frame->frame.info.sar.height,
					renderPos,
					contentPos,
					viewProjMat,
					mColorConversion,
					yuvRange,
					&frame->metadata,
					&mParams);
}


int VideoCoreEglRenderer::renderExternalVideoFrame(
	RawVideoMedia::Frame *frame,
	const struct pdraw_rect *renderPos,
	struct pdraw_rect *contentPos,
	Eigen::Matrix4f &viewProjMat)
{
	return -ENOSYS;
}

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREEGL */
