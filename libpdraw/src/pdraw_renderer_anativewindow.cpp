/**
 * Parrot Drones Awesome Video Viewer Library
 * Android NDK native window renderer
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_renderer_anativewindow.hpp"

#ifdef USE_ANATIVEWINDOW

#include <unistd.h>
#include <time.h>
#define ULOG_TAG libpdraw
#include <ulog.h>

namespace Pdraw {


ANativeWindowRenderer::ANativeWindowRenderer(
	Session *session) :
	Gles2Renderer(session, false)
{
	int ret;
	mThreadShouldStop = false;
	mWindow = NULL;
	mDisplay = EGL_NO_DISPLAY;
	mContext = EGL_NO_CONTEXT;
	mSurface = EGL_NO_SURFACE;
	mRendererThreadLaunched = false;
	mVideoWidth = 1920; /* TODO */
	mVideoHeight = 1080; /* TODO */

	ret = pthread_create(&mRendererThread, NULL,
		runRendererThread, (void*)this);
	if (ret != 0) {
		ULOGE("ANativeWindowRenderer: renderer thread creation "
			"failed (%d)", ret);
		return;
	}

	mRendererThreadLaunched = true;
}


ANativeWindowRenderer::~ANativeWindowRenderer(
	void)
{
	int ret;

	mThreadShouldStop = true;

	if (mRendererThreadLaunched) {
		ret = pthread_join(mRendererThread, NULL);
		if (ret != 0)
			ULOGE("ANativeWindowRenderer: pthread_join() "
				"failed (%d)", ret);
	}

	ret = setRendererParams(0, 0, 0, 0, 0, 0, false, false, false, NULL);
	if (ret != 0)
		ULOGE("ANativeWindowRenderer: setRendererParams() failed (%d)",
			ret);
}


int ANativeWindowRenderer::setRendererParams(
	int windowWidth,
	int windowHeight,
	int renderX,
	int renderY,
	int renderWidth,
	int renderHeight,
	bool hud,
	bool hmdDistorsionCorrection,
	bool headtracking,
	void *uiHandler)
{
	int ret;
	EGLBoolean eglRet;

	pthread_mutex_lock(&mMutex);

	if (mWindow) {
		mWindowWidth = 0;
		mWindowHeight = 0;

		eglRet = eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);
		if (eglRet == EGL_FALSE)
			ULOGE("ANativeWindowRenderer: eglMakeCurrent() failed");

		if (!uiHandler) {
			ret = Gles2Renderer::setRendererParams_nolock(
				0, 0, 0, 0, 0, 0, false, false, false, NULL);
			if (ret != 0)
				ULOGE("ANativeWindowRenderer: "
					"setRendererParams_nolock() "
					"failed (%d)", ret);
		}

		if (mDisplay != EGL_NO_DISPLAY) {
			eglMakeCurrent(mDisplay, EGL_NO_SURFACE,
				EGL_NO_SURFACE, EGL_NO_CONTEXT);
			if (mContext != EGL_NO_CONTEXT)
				eglDestroyContext(mDisplay, mContext);
			if (mSurface != EGL_NO_SURFACE)
				eglDestroySurface(mDisplay, mSurface);
			eglTerminate(mDisplay);
		}
		mDisplay = EGL_NO_DISPLAY;
		mContext = EGL_NO_CONTEXT;
		mSurface = EGL_NO_SURFACE;
		mWindow = NULL;
	}

	if (!uiHandler) {
		pthread_mutex_unlock(&mMutex);
		return 0;
	}

	/* TODO: better error handling */

	mWindow = (ANativeWindow*)uiHandler;

	/*
	 * Here specify the attributes of the desired configuration.
	 * Below, we select an EGLConfig with at least 8 bits per color
	 * component compatible with on-screen windows
	 */
	const EGLint attribs[] =
	{
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_BLUE_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_RED_SIZE, 8,
		EGL_NONE
	};
	EGLint w, h, format;
	EGLint numConfigs;
	EGLConfig config;
	EGLSurface surface;
	EGLContext context;

	EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

	eglInitialize(display, 0, 0);

	/* Pick the first EGLConfig that matches our criteria */
	eglChooseConfig(display, attribs, &config, 1, &numConfigs);

	/* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
	 * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
	 * As soon as we picked a EGLConfig, we can safely reconfigure the
	 * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
	eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);

	int geoW = mVideoWidth, geoH = mVideoHeight;
	if (hmdDistorsionCorrection) {
		geoW = windowWidth;
		geoH = windowHeight;
	} else {
		if (geoW > windowWidth) {
			geoW = windowWidth;
			geoH = windowWidth * mVideoHeight / mVideoWidth;
		}
		if (geoH > windowHeight) {
			geoW = windowHeight * mVideoWidth / mVideoHeight;
			geoH = windowHeight;
		}
		if (windowWidth * geoH > windowHeight * geoW)
			geoW = geoH * windowWidth / windowHeight;
		else
			geoH = geoW * windowHeight / windowWidth;
	}
	ANativeWindow_setBuffersGeometry(mWindow, geoW, geoH, format);

	surface = eglCreateWindowSurface(display, config, mWindow, NULL);

	const EGLint attrib_list[] =
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	context = eglCreateContext(display, config, NULL, attrib_list);

	eglRet = eglMakeCurrent(display, surface, surface, context);
	if (eglRet == EGL_FALSE) {
		ULOGE("ANativeWindowRenderer: eglMakeCurrent() failed");
		ret = -1;
		goto out;
	}

	eglQuerySurface(display, surface, EGL_WIDTH, &w);
	eglQuerySurface(display, surface, EGL_HEIGHT, &h);

	ret = Gles2Renderer::setRendererParams_nolock(w, h,
		renderX, renderY, renderWidth, renderHeight, hud,
		hmdDistorsionCorrection, headtracking, uiHandler);
	if (ret < 0) {
		ULOGE("ANativeWindowRenderer: failed to set renderer params");
		goto out;
	}

	eglRet = eglMakeCurrent(display, EGL_NO_SURFACE,
		EGL_NO_SURFACE, EGL_NO_CONTEXT);
	if (eglRet == EGL_FALSE) {
		ULOGE("ANativeWindowRenderer: eglMakeCurrent() failed");
		ret = -1;
		goto out;
	}

	mDisplay = display;
	mContext = context;
	mSurface = surface;
	mRunning = true;

	ULOGI("ANativeWindowRenderer: EGL context initialized "
		"(width=%d, height=%d)", w, h);

out:
	pthread_mutex_unlock(&mMutex);

	return ret;
}


int ANativeWindowRenderer::render(
	uint64_t lastRenderTime)
{
	int ret = 0;

	pthread_mutex_lock(&mMutex);

	if (!mRunning) {
		ret = 0;
		goto exit;
	}

	if ((mWindowWidth == 0) || (mWindowHeight == 0)) {
		ret = -1;
		goto exit;
	}

	EGLBoolean eglRet;
	eglRet = eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);
	if (eglRet == EGL_FALSE) {
		ULOGE("ANativeWindowRenderer: eglMakeCurrent() failed");
		ret = -1;
		goto exit;
	}

	ret = Gles2Renderer::render_nolock(lastRenderTime);
	if (ret > 0)
		eglSwapBuffers(mDisplay, mSurface);

	eglMakeCurrent(mDisplay, EGL_NO_SURFACE,
		EGL_NO_SURFACE, EGL_NO_CONTEXT);

	pthread_mutex_unlock(&mMutex);

	return ret;

exit:
	pthread_mutex_unlock(&mMutex);

	usleep(5000); /* TODO */

	return ret;
}


void* ANativeWindowRenderer::runRendererThread(
	void *ptr)
{
	ANativeWindowRenderer *renderer = (ANativeWindowRenderer*)ptr;
	uint64_t lastRenderTime = 0;

	while (!renderer->mThreadShouldStop) {
		renderer->render(lastRenderTime);
		struct timespec t1;
		clock_gettime(CLOCK_MONOTONIC, &t1);
		lastRenderTime =
			(uint64_t)t1.tv_sec * 1000000 +
			(uint64_t)t1.tv_nsec / 1000;
	}

	return NULL;
}

} /* namespace Pdraw */

#endif /* USE_ANATIVEWINDOW */
