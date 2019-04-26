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

#include "pdraw_renderer_videocoreegl.hpp"
#include "pdraw_session.hpp"

#ifdef USE_VIDEOCOREEGL

#	include <string.h>
#	include <time.h>
#	include <unistd.h>

#	define ULOG_TAG pdraw_rndvidvcgl
#	include <ulog.h>
ULOG_DECLARE_TAG(pdraw_rndvidvcgl);

namespace Pdraw {


VideoCoreEglRenderer::VideoCoreEglRenderer(
	Session *session,
	Element::Listener *listener,
	IPdraw::VideoRendererListener *rndListener) :
		Gles2Renderer(session, listener, rndListener)
{
	Element::mName = "VideoCoreEglRenderer";
	Sink::mName = "VideoCoreEglRenderer";
	mDisplay = EGL_NO_DISPLAY;
	setVideoMediaFormatCaps(VideoMedia::Format::OPAQUE);
	setVideoMediaSubFormatCaps(VideoMedia::OpaqueFormat::MMAL);
}


VideoCoreEglRenderer::~VideoCoreEglRenderer(void) {}


int VideoCoreEglRenderer::setup(
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	struct egl_display *eglDisplay)
{
	if (eglDisplay == NULL) {
		ULOGE("invalid EGL display");
		return -EINVAL;
	}
	mDisplay = (EGLDisplay)eglDisplay;

	struct pdraw_video_renderer_params _params = *params;
	/* HMD distortion correction is not supported with VideoCore 4 */
	_params.enableHmdDistortionCorrection = 0; /* TODO */

	return Gles2Renderer::setup(renderPos, &_params, eglDisplay);
}


int VideoCoreEglRenderer::loadVideoFrame(const uint8_t *data,
					 VideoMedia::Frame *frame)
{
	int ret;

	if (frame->format != VideoMedia::Format::OPAQUE) {
		ULOGE("unsupported frame format");
		return -ENOSYS;
	}
	if (frame->opaqueFrame.format != VideoMedia::OpaqueFormat::MMAL) {
		ULOGE("unsupported frame sub-format");
		return -ENOSYS;
	}

	mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_NONE;

	if ((frame->opaqueFrame.width == 0) ||
	    (frame->opaqueFrame.sarWidth == 0) ||
	    (frame->opaqueFrame.height == 0) ||
	    (frame->opaqueFrame.sarHeight == 0)) {
		ULOGE("invalid frame dimensions");
		return -EINVAL;
	}

	ret = mGles2Video->loadFrame(data,
				     NULL,
				     NULL,
				     frame->opaqueFrame.width,
				     frame->opaqueFrame.height,
				     mColorConversion,
				     (struct egl_display *)mDisplay);
	if (ret < 0)
		ULOG_ERRNO("gles2Video->loadFrame", -ret);

	return 0;
}


int VideoCoreEglRenderer::loadExternalVideoFrame(
	const uint8_t *data,
	VideoMedia::Frame *frame,
	const struct pdraw_session_info *session_info,
	const struct vmeta_session *session_meta)
{
	return -ENOSYS;
}


int VideoCoreEglRenderer::renderVideoFrame(VideoMedia::Frame *frame,
					   const struct pdraw_rect *renderPos,
					   struct pdraw_rect *contentPos,
					   Eigen::Matrix4f &viewProjMat)
{
	enum gles2_video_yuv_range yuvRange =
		frame->opaqueFrame.fullRange ? GLES2_VIDEO_YUV_FULL_RANGE
					     : GLES2_VIDEO_YUV_LIMITED_RANGE;

	return mGles2Video->renderFrame(&frame->opaqueFrame.stride,
					frame->opaqueFrame.height,
					frame->opaqueFrame.cropLeft,
					frame->opaqueFrame.cropTop,
					frame->opaqueFrame.cropWidth,
					frame->opaqueFrame.cropHeight,
					frame->opaqueFrame.sarWidth,
					frame->opaqueFrame.sarHeight,
					renderPos,
					contentPos,
					viewProjMat,
					mColorConversion,
					yuvRange,
					&frame->metadata,
					&mParams);
}


int VideoCoreEglRenderer::renderExternalVideoFrame(
	VideoMedia::Frame *frame,
	const struct pdraw_rect *renderPos,
	struct pdraw_rect *contentPos,
	Eigen::Matrix4f &viewProjMat)
{
	return -ENOSYS;
}

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREEGL */
