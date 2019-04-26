/**
 * Parrot Drones Awesome Video Viewer Library
 * Renderer interface
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

#include "pdraw_renderer.hpp"
#include "pdraw_renderer_gles2.hpp"
#include "pdraw_renderer_videocoreegl.hpp"

#include <errno.h>

#define ULOG_TAG pdraw_renderer
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_renderer);

namespace Pdraw {

Renderer *Renderer::create(Session *session,
			   Element::Listener *listener,
			   IPdraw::VideoRendererListener *rndListener)
{
#if defined(USE_VIDEOCOREEGL)
	return new VideoCoreEglRenderer(session, listener, rndListener);
#elif defined(USE_GLES2)
	return new Gles2Renderer(session, listener, rndListener);
#else
	return NULL;
#endif
}


Renderer::Renderer(Session *session,
		   Element::Listener *listener,
		   IPdraw::VideoRendererListener *rndListener,
		   uint32_t mediaTypeCaps,
		   uint32_t videoMediaFormatCaps,
		   uint32_t videoMediaSubFormatCaps) :
		Element(session, listener),
		Sink(mediaTypeCaps,
		     videoMediaFormatCaps,
		     videoMediaSubFormatCaps),
		mRendererListener(rndListener)
{
	int res;
	pthread_mutexattr_t attr;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		return;
	}

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto exit;
	}

	res = pthread_mutex_init(&mListenerMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		goto exit;
	}

exit:
	pthread_mutexattr_destroy(&attr);
	return;
}


Renderer::~Renderer(void)
{
	pthread_mutex_destroy(&mListenerMutex);
}


void Renderer::removeRendererListener(void)
{
	pthread_mutex_lock(&mListenerMutex);
	mRendererListener = NULL;
	pthread_mutex_unlock(&mListenerMutex);
}

} /* namespace Pdraw */
