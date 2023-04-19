/**
 * Parrot Drones Awesome Video Viewer Library
 * Video renderer interface
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

#ifndef _PDRAW_RENDERER_VIDEO_HPP_
#define _PDRAW_RENDERER_VIDEO_HPP_

#include "pdraw_element.hpp"

#include <pdraw/pdraw.hpp>

namespace Pdraw {

class VideoRenderer : public SinkElement {
public:
	virtual ~VideoRenderer(void);

	virtual int render(struct pdraw_rect *contentPos,
			   const float *viewMat = nullptr,
			   const float *projMat = nullptr) = 0;

	virtual int resize(const struct pdraw_rect *renderPos) = 0;

	virtual int setMediaId(unsigned int mediaId) = 0;

	virtual unsigned int getMediaId(void) = 0;

	virtual int
	setParams(const struct pdraw_video_renderer_params *params) = 0;

	virtual int getParams(struct pdraw_video_renderer_params *params) = 0;

	virtual void completeStop(void) = 0;

	static VideoRenderer *
	create(Session *session,
	       Element::Listener *listener,
	       IPdraw::IVideoRenderer *renderer,
	       IPdraw::IVideoRenderer::Listener *rndListener,
	       unsigned int mediaId,
	       const struct pdraw_rect *renderPos,
	       const struct pdraw_video_renderer_params *params,
	       struct egl_display *eglDisplay);

protected:
	VideoRenderer(Session *session,
		      Element::Listener *listener,
		      IPdraw::IVideoRenderer *renderer,
		      IPdraw::IVideoRenderer::Listener *rndListener,
		      uint32_t mediaTypeCaps,
		      const struct vdef_raw_format *rawVideoMediaFormatCaps,
		      int rawVideoMediaFormatCapsCount,
		      unsigned int mediaId,
		      const struct pdraw_rect *renderPos,
		      const struct pdraw_video_renderer_params *params,
		      struct egl_display *eglDisplay);

	void removeRendererListener(void);

	void asyncCompleteStop(void);

	IPdraw::IVideoRenderer *mRenderer;
	IPdraw::IVideoRenderer::Listener *mRendererListener;
	pthread_mutex_t mListenerMutex;

private:
	static void idleCompleteStop(void *userdata);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_RENDERER_VIDEO_HPP_ */
