/**
 * Parrot Drones Audio and Video Vector library
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

#define ULOG_TAG pdraw_rndvid
#include <ulog.h>

#include "pdraw_renderer_video.hpp"
#include "pdraw_renderer_video_gl.hpp"
#include "pdraw_session.hpp"

#include <errno.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {

std::unique_ptr<VideoRenderer> VideoRenderer::create(
	[[maybe_unused]] Session *session,
	[[maybe_unused]] Element::Listener *listener,
	[[maybe_unused]] VideoRendererWrapper *wrapper,
	[[maybe_unused]] IPdraw::IVideoRenderer::Listener *rndListener,
	[[maybe_unused]] unsigned int mediaId,
	[[maybe_unused]] const struct pdraw_rect *renderPos,
	[[maybe_unused]] const struct pdraw_video_renderer_params *params)
{
#if defined(PDRAW_USE_GL)
	try {
		return std::make_unique<GlVideoRenderer>(session,
							 listener,
							 wrapper,
							 rndListener,
							 mediaId,
							 renderPos,
							 params);
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to allocate renderer", __func__);
		return nullptr;
	}
#else
	ULOGE("no video renderer implementation found");
	return nullptr;
#endif
}


VideoRenderer::VideoRenderer(
	Session *session,
	Element::Listener *listener,
	VideoRendererWrapper *wrapper,
	IPdraw::IVideoRenderer::Listener *rndListener,
	[[maybe_unused]] uint32_t mediaTypeCaps,
	const struct vdef_raw_format *rawVideoMediaFormatCaps,
	int rawVideoMediaFormatCapsCount,
	[[maybe_unused]] unsigned int mediaId,
	[[maybe_unused]] const struct pdraw_rect *renderPos,
	[[maybe_unused]] const struct pdraw_video_renderer_params *params) :
		SinkElement(session,
			    listener,
			    wrapper,
			    1,
			    nullptr,
			    0,
			    rawVideoMediaFormatCaps,
			    rawVideoMediaFormatCapsCount,
			    nullptr,
			    0),
		mRenderer(wrapper), mRendererListener(rndListener)
{

	mCompleteStopHandler.set([this] { idleCompleteStop(); });
}


VideoRenderer::~VideoRenderer()
{
	/* Make sure listener functions will no longer be called */
	removeRendererListener();

	/* Remove any leftover idle callbacks */
	mSession->getPompLoop()->idleRemove(this);
}


void VideoRenderer::removeRendererListener()
{
	std::scoped_lock lock(mListenerMutex);
	mRendererListener = nullptr;
}


void VideoRenderer::asyncCompleteStop()
{
	int err = mSession->getPompLoop()->idleAdd(&mCompleteStopHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("Loop::idleAdd", -err);
}


/* Call from an idle function on the loop thread */
void VideoRenderer::idleCompleteStop()
{
	completeStop();
}


/* Called on the rendering thread */
VideoRendererWrapper::VideoRendererWrapper(
	Session *session,
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener)
{
	auto impl = Pdraw::VideoRenderer::create(
		session, session, this, listener, mediaId, renderPos, params);
	mRenderer = impl.get();
	mElement = impl.release();
}


/* Called on the rendering thread */
VideoRendererWrapper::~VideoRendererWrapper()
{
	if (isElementStopped())
		return;

	int ret = mRenderer->stop();
	if (ret < 0)
		ULOG_ERRNO("renderer->stop", -ret);
}


/* Called on the rendering thread */
int VideoRendererWrapper::resize(const struct pdraw_rect *renderPos)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->resize(renderPos);
}


/* Called on the rendering thread */
int VideoRendererWrapper::setMediaId(unsigned int mediaId)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->setMediaId(mediaId);
}


/* Called on the rendering thread */
unsigned int VideoRendererWrapper::getMediaId()
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->getMediaId();
}


/* Called on the rendering thread */
int VideoRendererWrapper::setParams(
	const struct pdraw_video_renderer_params *params)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->setParams(params, false);
}


/* Called on the rendering thread */
int VideoRendererWrapper::getParams(struct pdraw_video_renderer_params *params)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->getParams(params);
}


/* Called on the rendering thread */
int VideoRendererWrapper::render(struct pdraw_rect *contentPos,
				 const float *viewMat,
				 const float *projMat)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->render(contentPos, viewMat, projMat);
}

} /* namespace Pdraw */
