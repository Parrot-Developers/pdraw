/**
 * Parrot Drones Audio and Video Vector
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

#ifndef _QPDRAW_WIDGET_HPP_
#define _QPDRAW_WIDGET_HPP_

#include <QObject>

#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFunctions>

#include <QtWidgets/QOpenGLWidget>

#include <pdraw/pdraw_defs.h>
#include <pdraw/qpdraw.hpp>

namespace QPdraw {

/* Forward declarations */
namespace Internal {
class QPdrawWidgetPriv;
}


class QPdrawWidget : public QOpenGLWidget {
	Q_OBJECT

public:
	explicit QPdrawWidget(QWidget *parent = nullptr);

	~QPdrawWidget();

	/**
	 * Start a QPdraw widget.
	 * This function creates a QPdraw widget on a media of the given media
	 * id; if the media id is zero the first raw media encountered is used.
	 * This function must be called after the object creation prior to
	 * calling any other function.
	 * @param pdraw: information on the media
	 * @param mediaId: identifier of the raw media to render (from a
	 *                 pdraw_media_info structure); if zero the first
	 *                 raw media found is used for rendering
	 */
	void start(QPdraw *pdraw, unsigned int mediaId);

	/**
	 * Start a QPdraw widget with rendering position and params.
	 * This function creates a QPdraw widget on a media of the given media
	 * id; if the media id is zero the first raw media encountered is used.
	 * The rendering position and rendering parameters are also provided.
	 * This function must be called after the object creation prior to
	 * calling any other function.
	 * @param pdraw: information on the media
	 * @param mediaId: identifier of the raw media to render (from a
	 *                 pdraw_media_info structure); if zero the first
	 *                 raw media found is used for rendering
	 * @param renderPos: rendering position and size
	 * @param params: renderer parameters
	 */
	void start(QPdraw *pdraw,
		   unsigned int mediaId,
		   const struct pdraw_rect *renderPos,
		   const struct pdraw_video_renderer_params *params);

	/**
	 * Stop a QPdraw widget.
	 * This function must be called prior to destroying the instance.
	 */
	void stop();

	/**
	 * Set the rendering framerate of a QPdraw widget.
	 * The QPdraw widget uses an internal timer to periodically render
	 * at the requested frequency.
	 * This function can be overridden by any child class, but all
	 * implementations shall call the parent function.
	 * @param framerate: rendering framerate
	 */
	virtual void setFramerate(float framerate);

	/**
	 * Get the widget rendering rectangle.
	 * @return the rendering rectangle
	 */
	QRect getVideoRect() const;

signals:

	/**
	 * Media added signal, called when a media has been added internally to
	 * the renderer. Medias are raw video medias.
	 * @param info: information on the media
	 */
	void mediaAdded(const struct pdraw_media_info info);

	/**
	 * Media removed signal, called when a media has been removed internally
	 * from the renderer. Medias are raw video medias.
	 * @param info: information on the media
	 * @param restart: true if a new media should follow shortly
	 *                 (reconfiguration, resolution change...)
	 */
	void mediaRemoved(const struct pdraw_media_info info, bool restart);

	/**
	 * External texture loading signal. This signal is emitted before the
	 * rendering of the video frame in order to override the frame loading
	 * as a texture. This can be used to transform the video frames outside
	 * of PDrAW before resuming the rendering. This signal is emitted from
	 * the rendering thread. If no implementation of this function is
	 * required by the application, the retVal value must be set to -ENOSYS
	 * (before checking input values, so that implementation can be tested
	 * with all arguments null).
	 * @param textureWidth: texture width in pixels
	 * @param textureHeight: texture height in pixels
	 * @param sessionInfo: session information
	 * @param sessionMeta: session metadata
	 * @param frame: frame information
	 * @param frameUserdata: frame user data buffer
	 * @param frameUserdataLen: frame user data buffer size
	 *                          in bytes
	 * @param retVal: return value to be filled by the application
	 */
	void loadVideoTexture(unsigned int textureWidth,
			      unsigned int textureHeight,
			      const struct pdraw_media_info *mediaInfo,
			      struct mbuf_raw_video_frame *frame,
			      const void *frameUserdata,
			      unsigned int frameUserdataLen,
			      int *retVal);

	/**
	 * Overlay rendering signal. This signal is emitted after the rendering
	 * of the video frame in order to render an application overlay on top
	 * of the video. This signal is emitted from the rendering thread. If no
	 * implementation of this function is required by the application, the
	 * retVal value must be set to -ENOSYS (before checking input values, so
	 * that implementation can be tested with all arguments null).
	 * @param renderPos: rendering position
	 * @param contentPos: video content position
	 * @param viewMat: 4x4 view matrix
	 * @param projMat: 4x4 projection matrix
	 * @param sessionInfo: session information
	 * @param sessionMeta: session metadata
	 * @param frameMeta: frame metadata
	 * @param frameExtra: frame extra information
	 * @param retVal: return value to be filled by the application
	 */
	void
	renderVideoOverlay(const struct pdraw_rect *renderPos,
			   const struct pdraw_rect *contentPos,
			   const float *viewMat,
			   const float *projMat,
			   const struct pdraw_media_info *mediaInfo,
			   struct vmeta_frame *frameMeta,
			   const struct pdraw_video_frame_extra *frameExtra,
			   int *retVal);

protected:
	void initializeGL();

	void resizeGL(int w, int h);

	void paintGL();

private:
	Internal::QPdrawWidgetPriv *mPriv;
};

} /* namespace QPdraw */

#endif /* !_QPDRAW_WIDGET_HPP_ */
