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

#include "qpdraw_widget_priv.hpp"

#define ULOG_TAG qpdraw_widget
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#include <futils/futils.h>


/* Default rendering framerate of the widget (Hz) */
#define QPDRAW_WIDGET_DEFAULT_FRAMERATE 30.
/* Minimum interval before the next rendering of the widget (msec) */
#define QPDRAW_WIDGET_MIN_RENDER_INTERVAL_MS 5


Q_DECLARE_METATYPE(struct pdraw_media_info);


namespace QPdraw {
namespace Internal {


QPdrawWidgetPriv::QPdrawWidgetPriv(QPdrawWidget *parent) :
		mParent(parent), mPdraw(nullptr), mRenderer(nullptr),
		mTimer(nullptr), mFramerate(QPDRAW_WIDGET_DEFAULT_FRAMERATE),
		mPrevRenderTs(UINT64_MAX), mNextRenderExpectedTs(UINT64_MAX)
{
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
	ULOG_ERRNO_RETURN_IF(mParent == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(mPdraw != nullptr, EBUSY);

	IPdraw *pdrawInternal =
		reinterpret_cast<IPdraw *>(pdraw->getInternal());
	ULOG_ERRNO_RETURN_IF(pdrawInternal == nullptr, EPROTO);

	mTimer = new QTimer();
	ULOG_ERRNO_RETURN_IF(mTimer == nullptr, EPROTO);

	int res = pdrawInternal->createVideoRenderer(
		mediaId, renderPos, params, this, &mRenderer);
	if (res == 0)
		mPdraw = pdraw;

	connect(mTimer, SIGNAL(timeout()), this, SLOT(update()));

	mTimer->setTimerType(Qt::PreciseTimer);
	/* Start rendering ASAP */
	mTimer->start(0);
}


void QPdrawWidgetPriv::update(void)
{
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTimeUs;
	int64_t renderPeriodUs = 1000000. / mFramerate;

	ULOG_ERRNO_RETURN_IF(mTimer == nullptr, EINVAL);

	res = time_get_monotonic(&ts);
	ULOG_ERRNO_RETURN_IF(res < 0, -res);
	res = time_timespec_to_us(&ts, &curTimeUs);
	ULOG_ERRNO_RETURN_IF(res < 0, -res);

	/* First iteration */
	if (mNextRenderExpectedTs == UINT64_MAX)
		mNextRenderExpectedTs = curTimeUs;
	/* First iteration: assume that previous render was as expected */
	if (mPrevRenderTs == UINT64_MAX)
		mPrevRenderTs = curTimeUs - renderPeriodUs;

	int64_t prevIntervalUs = (int64_t)curTimeUs - (int64_t)mPrevRenderTs;
	int64_t renderErrorUs =
		(int64_t)curTimeUs - (int64_t)mNextRenderExpectedTs;
	int64_t nextIntervalMs = (renderPeriodUs - renderErrorUs) / 1000.;

	/* Filter-out negative and too small interval */
	if (nextIntervalMs < QPDRAW_WIDGET_MIN_RENDER_INTERVAL_MS)
		nextIntervalMs = QPDRAW_WIDGET_MIN_RENDER_INTERVAL_MS;

	if (mNextRenderExpectedTs != UINT64_MAX) {
		ULOGD("prevInterval: %.2fms, nextInterval: %" PRId64
		      ", error: %.2fms",
		      (float)prevIntervalUs / 1000.,
		      nextIntervalMs,
		      (float)renderErrorUs / 1000.);
	}

	/* Update widget (will trigger rendering) */
	mParent->update();

	/* Schedule next rendering */
	mTimer->setInterval(nextIntervalMs);

	mNextRenderExpectedTs += renderPeriodUs;
	mPrevRenderTs = curTimeUs;
}


void QPdrawWidgetPriv::stop()
{
	if (mPdraw == nullptr)
		return;

	if (mTimer != nullptr) {
		disconnect(mTimer, SIGNAL(timeout()), 0, 0);
		mTimer->stop();
		delete mTimer;
		mTimer = nullptr;
	}

	if (mRenderer != nullptr) {
		delete mRenderer;
		mRenderer = nullptr;
	}
	mPdraw = nullptr;
}


void QPdrawWidgetPriv::setFramerate(float framerate)
{
	if (mFramerate == framerate)
		return;

	mFramerate = framerate;

	/* Reset values as framerate has changed */
	mPrevRenderTs = UINT64_MAX;
	mNextRenderExpectedTs = UINT64_MAX;

	/* Note: no need to call setInterval, rendering interval
	 * will dynamically be updated when scheduling the next rendering */
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

	/* Reset values as rendering will start */
	mPrevRenderTs = UINT64_MAX;
	mNextRenderExpectedTs = UINT64_MAX;

	struct pdraw_media_info info_copy = *info;
	emit mParent->mediaAdded(info_copy);
}


void QPdrawWidgetPriv::onVideoRendererMediaRemoved(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info,
	bool restart)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);

	struct pdraw_media_info info_copy = *info;
	emit mParent->mediaRemoved(info_copy, restart);
}


void QPdrawWidgetPriv::onVideoRenderReady(IPdraw *pdraw,
					  IPdraw::IVideoRenderer *renderer)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(renderer);
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


} /* namespace Internal */


QPdrawWidget::QPdrawWidget(QWidget *parent) : QOpenGLWidget(parent)
{
	qRegisterMetaType<pdraw_media_info>("pdraw_media_info");

	mPriv = new Internal::QPdrawWidgetPriv(this);

	QSurfaceFormat format;
	format.setSwapBehavior(QSurfaceFormat::TripleBuffer);
	format.setSwapInterval(1);

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
		.enable_auto_normalization = 0,
		.enable_overexposure_zebras = 0,
		.overexposure_zebras_threshold = 0,
		.enable_histograms = 0,
		.enable_simplified_rendering = 0,
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


void QPdrawWidget::setFramerate(float framerate)
{
	mPriv->setFramerate(framerate);
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
