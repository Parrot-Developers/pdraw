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

#define ULOG_TAG pdraw_rndvid
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_renderer_video.hpp"
#include "pdraw_renderer_video_gles2.hpp"
#include "pdraw_renderer_video_videocoreegl.hpp"
#include "pdraw_session.hpp"

#include <errno.h>

namespace Pdraw {

VideoRenderer *
VideoRenderer::create(Session *session,
		      Element::Listener *listener,
		      IPdraw::IVideoRenderer *renderer,
		      IPdraw::IVideoRenderer::Listener *rndListener,
		      unsigned int mediaId,
		      const struct pdraw_rect *renderPos,
		      const struct pdraw_video_renderer_params *params,
		      struct egl_display *eglDisplay)
{
#if defined(USE_VIDEOCOREEGL)
	return new VideoCoreEglVideoRenderer(session,
					     listener,
					     renderer,
					     rndListener,
					     mediaId,
					     renderPos,
					     params,
					     eglDisplay);
#elif defined(USE_GLES2)
	return new Gles2VideoRenderer(session,
				      listener,
				      renderer,
				      rndListener,
				      mediaId,
				      renderPos,
				      params,
				      eglDisplay);
#else
	return nullptr;
#endif
}


VideoRenderer::VideoRenderer(
	Session *session,
	Element::Listener *listener,
	IPdraw::IVideoRenderer *renderer,
	IPdraw::IVideoRenderer::Listener *rndListener,
	uint32_t mediaTypeCaps,
	const struct vdef_raw_format *rawVideoMediaFormatCaps,
	int rawVideoMediaFormatCapsCount,
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	struct egl_display *eglDisplay) :
		SinkElement(session,
			    listener,
			    1,
			    nullptr,
			    0,
			    rawVideoMediaFormatCaps,
			    rawVideoMediaFormatCapsCount),
		mRenderer(renderer), mRendererListener(rndListener)
{
	int res;
	pthread_mutexattr_t attr;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		PDRAW_LOG_ERRNO("pthread_mutexattr_init", res);
		return;
	}

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		PDRAW_LOG_ERRNO("pthread_mutexattr_settype", res);
		goto exit;
	}

	res = pthread_mutex_init(&mListenerMutex, &attr);
	if (res != 0) {
		PDRAW_LOG_ERRNO("pthread_mutex_init", res);
		goto exit;
	}

exit:
	pthread_mutexattr_destroy(&attr);
	return;
}


VideoRenderer::~VideoRenderer(void)
{
	/* Remove any leftover idle callbacks */
	int err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	pthread_mutex_destroy(&mListenerMutex);
}


void VideoRenderer::removeRendererListener(void)
{
	pthread_mutex_lock(&mListenerMutex);
	mRendererListener = nullptr;
	pthread_mutex_unlock(&mListenerMutex);
}


void VideoRenderer::asyncCompleteStop(void)
{
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleCompleteStop, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


/**
 * VideoRenderer calls from idle functions on the loop thread
 */

void VideoRenderer::idleCompleteStop(void *userdata)
{
	VideoRenderer *self = reinterpret_cast<VideoRenderer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	self->completeStop();
}

} /* namespace Pdraw */
