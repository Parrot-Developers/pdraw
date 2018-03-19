/**
 * Parrot Drones Awesome Video Viewer Library
 * Broadcom VideoCore 4 EGL renderer
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

#include "pdraw_renderer_videocoreegl.hpp"
#include "pdraw_session.hpp"

#ifdef USE_VIDEOCOREEGL

#include <string.h>
#include <unistd.h>
#include <time.h>
#define ULOG_TAG libpdraw
#include <ulog.h>

namespace Pdraw {


VideoCoreEglRenderer::VideoCoreEglRenderer(
	Session *session) :
	Gles2Renderer(session)
{
	mDisplay = EGL_NO_DISPLAY;
	mContext = EGL_NO_CONTEXT;
	mSurface = EGL_NO_SURFACE;
}


VideoCoreEglRenderer::~VideoCoreEglRenderer(
	void)
{
}


int VideoCoreEglRenderer::setRendererParams(
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
	int ret = 0;

	/* HMD distorsion correction is not supported with VideoCore:
	 * the video decoder outputs frames through the egl_renderer element,
	 * this must be changed to render to FBO */
	hmdDistorsionCorrection = false; /* TODO */

	pthread_mutex_lock(&mMutex);

	ret = Gles2Renderer::setRendererParams_nolock(windowWidth, windowHeight,
		renderX, renderY, renderWidth, renderHeight, hud,
		hmdDistorsionCorrection, headtracking, uiHandler);

	/* TODO: do something better */
	struct uiParams_s {
		EGLDisplay display;
		EGLSurface surface;
		EGLContext context;
	};
	struct uiParams_s *uiParams = (struct uiParams_s *)uiHandler;
	mDisplay = uiParams->display;
	mSurface = uiParams->surface;
	mContext = uiParams->context;

	if (ret > 0)
		mRunning = true;

	pthread_mutex_unlock(&mMutex);

	return ret;
}


int VideoCoreEglRenderer::render(
	uint64_t lastRenderTime)
{
	int ret = 0;
	EGLBoolean eglRet;
	struct vbuf_buffer *buffer = NULL;
	struct avcdecoder_output_buffer *data = NULL;
	int dequeueRet;
	const uint8_t *cdata;
	bool load = false;
	struct timespec t1;
	uint64_t renderTimestamp, renderTimestamp1;

	pthread_mutex_lock(&mMutex);

	if (!mRunning)
		goto exit;
	if ((mWindowWidth == 0) || (mWindowHeight == 0))
		goto exit;
	if ((mRenderWidth == 0) || (mRenderHeight == 0))
		goto exit;

	eglRet = eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);
	if (eglRet == EGL_FALSE) {
		ULOGE("VideoCoreEglRenderer: eglMakeCurrent() failed");
		ret = -1;
		goto out;
	}

	dequeueRet = vbuf_queue_pop(mDecoderOutputBufferQueue,
		0, &buffer);
	while ((dequeueRet == 0) && (buffer)) {
		if (mCurrentBuffer != NULL) {
			int releaseRet = vbuf_unref(
				&mCurrentBuffer);
			if (releaseRet != 0)
				ULOGE("VideoCoreEglRenderer: failed to "
					"release buffer (%d)", releaseRet);
		}
		mCurrentBuffer = buffer;
		load = true;
		dequeueRet = vbuf_queue_pop(mDecoderOutputBufferQueue,
			0, &buffer);
	}
	if ((dequeueRet < 0) && (dequeueRet != -EAGAIN))
		ULOGE("VideoCoreEglRenderer: failed to "
			"get buffer from queue (%d)", dequeueRet);

	if (mCurrentBuffer == NULL)
		goto out;

	cdata = vbuf_get_cdata(mCurrentBuffer);
	data = (struct avcdecoder_output_buffer *)
		vbuf_metadata_get(mCurrentBuffer,
			mDecoder->getMedia(), NULL, NULL);
	if ((cdata == NULL) || (data == NULL)) {
		ULOGE("Gles2Renderer: invalid buffer data");
		ret = -1;
		goto out;
	}

	if (mHmdDistorsionCorrection) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mFbo));
		GLCHK(glViewport(0, 0, mRenderWidth, mRenderHeight));
	}

	if (mGles2Video != NULL) {
		if (load) {
			ret = mGles2Video->loadFrame(cdata,
				data->plane_offset, data->stride,
				data->width, data->height,
				GLES2_VIDEO_COLOR_CONVERSION_NONE,
				(void *)mDisplay);
			if (ret != 0) {
				ULOGE("VideoCoreEglRenderer: failed to "
					"render video frame");
				goto out;
			}
		}
		ret = mGles2Video->renderFrame(data->stride,
			data->width, data->height, data->sarWidth,
			data->sarHeight, mRenderWidth, mRenderHeight,
			mRenderX, mRenderY,
			GLES2_VIDEO_COLOR_CONVERSION_NONE, &data->metadata,
			mHeadtracking, 0);
		if (ret != 0) {
			ULOGE("VideoCoreEglRenderer: failed to "
				"render video frame");
			goto out;
		}
	}

	if (mGles2Hud != NULL) {
		ret = mGles2Hud->renderHud(data->width * data->sarWidth,
			data->height * data->sarHeight,
			(mHmdDistorsionCorrection) ? mRenderWidth / 2 :
			mRenderWidth, mRenderHeight, &data->metadata,
			mHmdDistorsionCorrection, mHeadtracking);
		if (ret != 0)
			ULOGE("VideoCoreEglRenderer: failed to render the HUD");
	}

	if (mHmdDistorsionCorrection) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
		GLCHK(glViewport(mRenderX, mRenderY,
			mRenderWidth, mRenderHeight));

		if (mGles2Hmd) {
			ret = mGles2Hmd->renderHmd(mFboTexture,
				mRenderWidth, mRenderHeight);
			if (ret != 0)
				ULOGE("VideoCoreEglRenderer: failed to "
					"render HMD distorsion correction");
		}
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	renderTimestamp1 = renderTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	eglSwapBuffers(mDisplay, mSurface);

	clock_gettime(CLOCK_MONOTONIC, &t1);
	renderTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

#if 0
	/* TODO: remove debug */
	uint64_t currentTime = 0, duration = 0;
	unsigned int cHrs = 0, cMin = 0, cSec = 0, cMsec = 0;
	unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
	currentTime = mSession->getCurrentTime();
	duration = mSession->getDuration();
	if ((currentTime > 0) && (currentTime != (uint64_t)-1)) {
		pdraw_friendlyTimeFromUs(currentTime,
			&cHrs, &cMin, &cSec, &cMsec);
	}
	if ((duration > 0) && (duration != (uint64_t)-1)) {
		pdraw_friendlyTimeFromUs(duration,
			&dHrs, &dMin, &dSec, &dMsec);
	}

	ULOGI("VideoCoreEglRenderer: %02d:%02d:%02d.%03d / %02d:%02d:%02d.%03d "
		"frame (decoding: %.2fms, rendering: %.2fms, "
		"est. latency: %.2fms) render@%.1ffps",
		cHrs, cMin, cSec, cMsec, dHrs, dMin, dSec, dMsec,
		(float)(data->decoderOutputTimestamp -
			data->demuxOutputTimestamp) / 1000.,
		(float)(renderTimestamp1 -
			data->decoderOutputTimestamp) / 1000.,
		(data->auNtpTimestampLocal != 0) ? (float)(renderTimestamp -
			data->auNtpTimestampLocal) / 1000. : 0.,
		(renderTimestamp - lastRenderTime > 0) ? 1000000. /
			((float)(renderTimestamp - lastRenderTime)) : 0.);
#endif

out:
	pthread_mutex_unlock(&mMutex);

	return ret;

exit:
	pthread_mutex_unlock(&mMutex);

	usleep(5000); /* TODO */

	return ret;
}

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREEGL */
