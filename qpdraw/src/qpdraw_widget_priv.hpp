/**
 * Parrot Drones Awesome Video Viewer
 * Qt PDrAW widget
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

#ifndef _QPDRAW_WIDGET_PRIV_HPP_
#define _QPDRAW_WIDGET_PRIV_HPP_

#include <pdraw/pdraw_backend.hpp>
#include <pdraw/qpdraw_widget.hpp>

using namespace Pdraw;

namespace QPdraw {
namespace Internal {


class QPdrawWidgetPriv : public QObject,
			 public IPdraw::IVideoRenderer::Listener {
	Q_OBJECT

public:
	explicit QPdrawWidgetPriv(QPdrawWidget *parent);
	~QPdrawWidgetPriv();

	void start(QPdraw *pdraw,
		   unsigned int mediaId,
		   const struct pdraw_rect *renderPos,
		   const struct pdraw_video_renderer_params *params);
	void stop();
	bool resizeGL(const struct pdraw_rect *renderPos);
	bool paintGL();

private:
	void onVideoRendererMediaAdded(IPdraw *pdraw,
				       IPdraw::IVideoRenderer *renderer,
				       const struct pdraw_media_info *info);

	void onVideoRendererMediaRemoved(IPdraw *pdraw,
					 IPdraw::IVideoRenderer *renderer,
					 const struct pdraw_media_info *info);

	void onVideoRenderReady(IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer);

	int loadVideoTexture(IPdraw *pdraw,
			     IPdraw::IVideoRenderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_media_info *mediaInfo,
			     struct mbuf_raw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen);

	int
	renderVideoOverlay(IPdraw *pdraw,
			   IPdraw::IVideoRenderer *renderer,
			   const struct pdraw_rect *renderPos,
			   const struct pdraw_rect *contentPos,
			   const float *viewMat,
			   const float *projMat,
			   const struct pdraw_media_info *mediaInfo,
			   struct vmeta_frame *frameMeta,
			   const struct pdraw_video_frame_extra *frameExtra);

signals:
	/**
	 * Render ready signal, called by onVideoRenderReady() method,
	 * internally connected to the renderReady() slot, as the widget
	 * update must be done from the Qt GUI thread.
	 */
	void onRenderReady();

private slots:
	/**
	 * Render ready slot, connected to the onRenderReady() signal,
	 * used to trigger the widget update in the Qt GUI thread.
	 */
	void renderReady();

private:
	QPdrawWidget *mParent;
	QPdraw *mPdraw;
	IPdraw::IVideoRenderer *mRenderer;
};

} /* namespace Internal */
} /* namespace QPdraw */

#endif /* !_QPDRAW_WIDGET_PRIV_HPP_ */
