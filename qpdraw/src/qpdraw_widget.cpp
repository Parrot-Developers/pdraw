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

#include "qpdraw_widget_priv.hpp"

#define ULOG_TAG qpdraw_widget
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

namespace QPdraw {
namespace Internal {


QPdrawWidgetPriv::QPdrawWidgetPriv(QPdrawWidget *parent) :
		mParent(parent), mPdraw(nullptr), mRenderer(nullptr)
{
	connect(this,
		&QPdrawWidgetPriv::onRenderReady,
		this,
		&QPdrawWidgetPriv::renderReady,
		Qt::QueuedConnection);
}


QPdrawWidgetPriv::~QPdrawWidgetPriv()
{
	stop();
}


void QPdrawWidgetPriv::start(QPdraw *pdraw,
			     unsigned int mediaId,
			     const struct pdraw_rect *renderPos,
			     const struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_IF(mPdraw != nullptr, EBUSY);

	IPdraw *pdrawInternal =
		reinterpret_cast<IPdraw *>(pdraw->getInternal());
	ULOG_ERRNO_RETURN_IF(pdrawInternal == nullptr, EPROTO);

	int res = pdrawInternal->createVideoRenderer(
		mediaId, renderPos, params, this, &mRenderer);
	if (res == 0)
		mPdraw = pdraw;
}


void QPdrawWidgetPriv::stop()
{
	if (mPdraw == nullptr)
		return;

	delete mRenderer;
	mRenderer = nullptr;
	mPdraw = nullptr;
}


bool QPdrawWidgetPriv::resizeGL(const struct pdraw_rect *renderPos)
{
	if (mPdraw == nullptr)
		return false;

	ULOG_ERRNO_RETURN_VAL_IF(mRenderer == nullptr, EINVAL, false);

	int res = mRenderer->resize(renderPos);
	if (res < 0) {
		ULOG_ERRNO("IVideoRenderer::resize", -res);
		return false;
	}

	return true;
}


bool QPdrawWidgetPriv::paintGL()
{
	if (mPdraw == nullptr)
		return false;

	ULOG_ERRNO_RETURN_VAL_IF(mRenderer == nullptr, EINVAL, false);

	int res = mRenderer->render(nullptr);
	if (res < 0) {
		ULOG_ERRNO("IVideoRenderer::render", -res);
		return false;
	}

	return true;
}


void QPdrawWidgetPriv::onVideoRendererMediaAdded(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	struct pdraw_media_info info_copy = *info;
	emit mParent->mediaAdded(info_copy);
}


void QPdrawWidgetPriv::onVideoRendererMediaRemoved(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	struct pdraw_media_info info_copy = *info;
	emit mParent->mediaRemoved(info_copy);
}


void QPdrawWidgetPriv::onVideoRenderReady(IPdraw *pdraw,
					  IPdraw::IVideoRenderer *renderer)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	emit onRenderReady();
}


int QPdrawWidgetPriv::loadVideoTexture(IPdraw *pdraw,
				       IPdraw::IVideoRenderer *renderer,
				       unsigned int textureWidth,
				       unsigned int textureHeight,
				       const struct pdraw_media_info *mediaInfo,
				       struct mbuf_raw_video_frame *frame,
				       const void *frameUserdata,
				       size_t frameUserdataLen)
{
	int ret = -ENOSYS;

	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	emit mParent->loadVideoTexture(textureWidth,
				       textureHeight,
				       mediaInfo,
				       frame,
				       frameUserdata,
				       (unsigned int)frameUserdataLen,
				       &ret);
	return ret;
}


int QPdrawWidgetPriv::renderVideoOverlay(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_rect *renderPos,
	const struct pdraw_rect *contentPos,
	const float *viewMat,
	const float *projMat,
	const struct pdraw_media_info *mediaInfo,
	struct vmeta_frame *frameMeta,
	const struct pdraw_video_frame_extra *frameExtra)
{
	int ret = -ENOSYS;

	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	emit mParent->renderVideoOverlay(renderPos,
					 contentPos,
					 viewMat,
					 projMat,
					 mediaInfo,
					 frameMeta,
					 frameExtra,
					 &ret);
	return ret;
}


void QPdrawWidgetPriv::renderReady()
{
	mParent->update();
}


} /* namespace Internal */


QPdrawWidget::QPdrawWidget(QWidget *parent) : QOpenGLWidget(parent)
{
	qRegisterMetaType<pdraw_media_info>("pdraw_media_info");

	mPriv = new Internal::QPdrawWidgetPriv(this);

	QSurfaceFormat format;
	format.setSwapBehavior(QSurfaceFormat::TripleBuffer);

	/* Must be called before the widget or its parent
	 * window gets shown */
	setFormat(format);
}


QPdrawWidget::~QPdrawWidget()
{
	delete mPriv;
}


void QPdrawWidget::start(QPdraw *pdraw, unsigned int mediaId)
{
	struct pdraw_rect renderPos = {
		.x = 0,
		.y = 0,
		.width = (unsigned int)(rect().width() * devicePixelRatio()),
		.height = (unsigned int)(rect().height() * devicePixelRatio()),
	};

	struct pdraw_video_renderer_params params = {
		.scheduling_mode =
			PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE,
		.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND,
		.enable_transition_flags = 0xFFFFFFFF,
		.enable_hmd_distortion_correction = 0,
		.hmd_ipd_offset = 0.f,
		.hmd_x_offset = 0.f,
		.hmd_y_offset = 0.f,
		.video_scale_factor = 0.f,
		.enable_overexposure_zebras = 0,
		.overexposure_zebras_threshold = 0,
		.enable_histograms = 0,
		.video_texture_width = 0,
		.video_texture_dar_width = 0,
		.video_texture_dar_height = 0,
	};

	QPdrawWidget::start(pdraw, mediaId, &renderPos, &params);
}


void QPdrawWidget::start(QPdraw *pdraw,
			 unsigned int mediaId,
			 const struct pdraw_rect *renderPos,
			 const struct pdraw_video_renderer_params *params)
{
	makeCurrent();

	mPriv->start(pdraw, mediaId, renderPos, params);

	doneCurrent();
}


void QPdrawWidget::stop()
{
	makeCurrent();

	mPriv->stop();

	doneCurrent();
}


QRect QPdrawWidget::getVideoRect() const
{
	return rect();
}


void QPdrawWidget::initializeGL()
{
	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}


void QPdrawWidget::resizeGL(int w, int h)
{
	Q_UNUSED(w);
	Q_UNUSED(h);

	/* TODO: w and h? */

	struct pdraw_rect renderPos = {
		0,
		0,
		(unsigned int)(rect().width() * devicePixelRatio()),
		(unsigned int)(rect().height() * devicePixelRatio())};

	bool ret = mPriv->resizeGL(&renderPos);
	if (ret)
		return;

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClear(GL_COLOR_BUFFER_BIT);
}


void QPdrawWidget::paintGL()
{
	bool ret = mPriv->paintGL();
	if (ret)
		return;

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClear(GL_COLOR_BUFFER_BIT);
}

} /* namespace QPdraw */
