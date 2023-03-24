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

#ifndef _PDRAW_RENDERER_VIDEOCOREEGL_HPP_
#define _PDRAW_RENDERER_VIDEOCOREEGL_HPP_

#ifdef USE_VIDEOCOREEGL

#	include "pdraw_gles2_hmd.hpp"
#	include "pdraw_gles2_video.hpp"
#	include "pdraw_renderer_gles2.hpp"

#	include <EGL/egl.h>

namespace Pdraw {


class VideoCoreEglRenderer : public Gles2Renderer {
public:
	VideoCoreEglRenderer(Session *session,
			     Element::Listener *listener,
			     IPdraw::IVideoRenderer *renderer,
			     IPdraw::IVideoRenderer::Listener *rndListener,
			     unsigned int mediaId,
			     const struct pdraw_rect *renderPos,
			     const struct pdraw_video_renderer_params *params,
			     struct egl_display *eglDisplay);

	~VideoCoreEglRenderer(void);

private:
	int setup(const struct pdraw_rect *renderPos,
		  const struct pdraw_video_renderer_params *params,
		  struct egl_display *eglDisplay);

	int loadVideoFrame(const uint8_t *data, RawVideoMedia::Frame *frame);

	int loadExternalVideoFrame(const uint8_t *data,
				   RawVideoMedia::Frame *frame,
				   const struct pdraw_media_info *media_info);

	int renderVideoFrame(RawVideoMedia::Frame *frame,
			     const struct pdraw_rect *renderPos,
			     struct pdraw_rect *contentPos,
			     Eigen::Matrix4f &viewProjMat);

	int renderExternalVideoFrame(RawVideoMedia::Frame *frame,
				     const struct pdraw_rect *renderPos,
				     struct pdraw_rect *contentPos,
				     Eigen::Matrix4f &viewProjMat);

	EGLDisplay mDisplay;
};

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREEGL */

#endif /* !_PDRAW_RENDERER_VIDEOCOREEGL_HPP_ */
