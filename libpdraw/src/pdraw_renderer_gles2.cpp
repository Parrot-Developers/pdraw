/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 renderer
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

#include "pdraw_renderer_gles2.hpp"
#include "pdraw_settings.hpp"
#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"

#ifdef USE_GLES2

#include <string.h>
#include <unistd.h>
#include <time.h>
#define ULOG_TAG pdraw_rndvidgl
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_rndvidgl);

namespace Pdraw {


Gles2Renderer::Gles2Renderer(
	Session *session)
{
	int ret;
	mRunning = false;
	mSession = session;
	mMedia = NULL;
	mWindowWidth = 0;
	mWindowHeight = 0;
	mQueue = NULL;
	mCurrentBuffer = NULL;
	mGles2Video = NULL;
	mGles2Hud = NULL;
	mGles2HmdFirstTexUnit = 0;
	mGles2VideoFirstTexUnit =
		mGles2HmdFirstTexUnit + Gles2Hmd::getTexUnitCount();
	mGles2HudFirstTexUnit =
		mGles2VideoFirstTexUnit + Gles2Video::getTexUnitCount();
	mHmdDistorsionCorrection = false;
	mGles2Hmd = NULL;
	mFbo = 0;
	mFboTexture = 0;
	mFboRenderBuffer = 0;

	ret = pthread_mutex_init(&mMutex, NULL);
	if (ret < 0)
		ULOG_ERRNO("pthread_mutex_init", -ret);
}


Gles2Renderer::~Gles2Renderer(
	void)
{
	if (mMedia != NULL) {
		int ret = removeInputSource(mMedia);
		if (ret < 0)
			ULOG_ERRNO("removeInputSource", -ret);
	}

	close();

	pthread_mutex_destroy(&mMutex);
}


int Gles2Renderer::open(
	unsigned int windowWidth,
	unsigned int windowHeight,
	int renderX,
	int renderY,
	unsigned int renderWidth,
	unsigned int renderHeight,
	bool hud,
	bool hmdDistorsionCorrection,
	bool headtracking,
	struct egl_display *eglDisplay)
{
	if ((windowWidth == 0) || (windowHeight == 0))
		return -EINVAL;
	if (renderWidth == 0)
		renderWidth = windowWidth - renderX;
	if (renderHeight == 0)
		renderHeight = windowHeight - renderY;
	if ((renderWidth == 0) || (renderHeight == 0))
		return -EINVAL;

	mWindowWidth = windowWidth;
	mWindowHeight = windowHeight;
	mHud = hud;
	mHmdDistorsionCorrection = hmdDistorsionCorrection;
	mHeadtracking = headtracking;

	GLCHK();

	mGles2Video = new Gles2Video(mSession, (VideoMedia*)mMedia,
		mGles2VideoFirstTexUnit);
	if (mGles2Video == NULL) {
		ULOG_ERRNO("failed to create Gles2Video", ENOMEM);
		goto err;
	}

	if (mHud) {
		mGles2Hud = new Gles2Hud(mSession, (VideoMedia*)mMedia,
			mGles2HudFirstTexUnit);
		if (mGles2Hud == NULL) {
			ULOG_ERRNO("failed to create Gles2Hud", ENOMEM);
			goto err;
		}
	}

	GLCHK();
	GLCHK(glViewport(renderX, renderY, renderWidth, renderHeight));
	GLCHK(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	if (mHmdDistorsionCorrection) {
		GLCHK(glGenFramebuffers(1, &mFbo));
		if (mFbo <= 0) {
			ULOGE("failed to create framebuffer");
			goto err;
		}
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mFbo));

		GLCHK(glGenTextures(1, &mFboTexture));
		if (mFboTexture <= 0) {
			ULOGE("failed to create texture");
			goto err;
		}
		GLCHK(glActiveTexture(GL_TEXTURE0 + mGles2HmdFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mFboTexture));
		GLCHK(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
			mWindowWidth / 2, mWindowHeight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, NULL));

		GLCHK(glTexParameteri(GL_TEXTURE_2D,
			GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCHK(glTexParameteri(GL_TEXTURE_2D,
			GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCHK(glTexParameterf(GL_TEXTURE_2D,
			GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCHK(glTexParameterf(GL_TEXTURE_2D,
			GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

		GLCHK(glGenRenderbuffers(1, &mFboRenderBuffer));
		if (mFboRenderBuffer <= 0) {
			ULOGE("failed to create render buffer");
			goto err;
		}
		GLCHK(glBindRenderbuffer(GL_RENDERBUFFER, mFboRenderBuffer));
		GLCHK(glRenderbufferStorage(GL_RENDERBUFFER,
			GL_DEPTH_COMPONENT16, mWindowWidth / 2, mWindowHeight));

		GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
			GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mFboTexture, 0));
		GLCHK(glFramebufferRenderbuffer(GL_FRAMEBUFFER,
			GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
			mFboRenderBuffer));

		GLenum gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (gle != GL_FRAMEBUFFER_COMPLETE) {
			ULOGE("invalid framebuffer status");
			goto err;
		}

		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		GLCHK(glBindTexture(GL_TEXTURE_2D, 0));
		GLCHK(glBindRenderbuffer(GL_RENDERBUFFER, 0));
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, 0));

		if (mSession != NULL) {
			float xdpi = 0., ydpi = 0., deviceMargin = 0.;
			float ipd = 0., scale = 0., panH = 0., panV = 0.;
			enum pdraw_hmd_model hmdModel = PDRAW_HMD_MODEL_UNKNOWN;
			Settings *settings = mSession->getSettings();
			settings->lock();
			settings->getDisplayScreenSettings(
				&xdpi, &ydpi, &deviceMargin);
			settings->getHmdDistorsionCorrectionSettings(
				&hmdModel, &ipd, &scale, &panH, &panV);
			settings->unlock();
			mGles2Hmd = new Gles2Hmd(mGles2HmdFirstTexUnit,
				mWindowWidth, mWindowHeight, hmdModel,
				xdpi, ydpi, deviceMargin, ipd,
				scale, panH, panV);
		} else {
			mGles2Hmd = new Gles2Hmd(mGles2HmdFirstTexUnit,
				mWindowWidth, mWindowHeight);
		}
		if (mGles2Hmd == NULL) {
			ULOGE("failed to create Gles2Hmd context");
			goto err;
		}
	}

	mRunning = true;
	return 0;

err:
	close();
	return -1;
}


int Gles2Renderer::close(
	void)
{
	if ((mGles2Video != NULL) ||
		(mGles2Hud != NULL) || (mGles2Hmd != NULL))
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	if (mGles2Video != NULL) {
		delete mGles2Video;
		mGles2Video = NULL;
	}
	if (mGles2Hud != NULL) {
		delete mGles2Hud;
		mGles2Hud = NULL;
	}
	if (mGles2Hmd != NULL) {
		delete mGles2Hmd;
		mGles2Hmd = NULL;
	}
	if (mFboRenderBuffer > 0) {
		GLCHK(glDeleteRenderbuffers(1, &mFboRenderBuffer));
		mFboRenderBuffer = 0;
	}
	if (mFboTexture > 0) {
		GLCHK(glDeleteTextures(1, &mFboTexture));
		mFboTexture = 0;
	}
	if (mFbo > 0) {
		GLCHK(glDeleteFramebuffers(1, &mFbo));
		mFbo = 0;
	}

	return 0;
}


int Gles2Renderer::addInputSource(
	Media *media)
{
	if (media == NULL)
		return -EINVAL;
	if (mMedia != NULL) {
		ULOGE("multiple input media are not supported");
		return -ENOSYS;
	}
	VideoMedia *vmedia = dynamic_cast<VideoMedia *>(media);
	if (vmedia == NULL) {
		ULOGE("media is not a video media");
		return -EPROTO;
	}

	mQueue = vbuf_queue_new(0, 0);
	if (mQueue == NULL) {
		ULOGE("failed to create queue");
		return -ENOMEM;
	}

	mMedia = media;
	if (mGles2Video)
		mGles2Video->setVideoMedia(vmedia);
	if (mGles2Hud)
		mGles2Hud->setVideoMedia(vmedia);

	return 0;
}


int Gles2Renderer::removeInputSource(
	Media *media)
{
	int ret;

	if (media == NULL)
		return -EINVAL;
	if (media != mMedia) {
		ULOGE("invalid media");
		return -ENOENT;
	}

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(&mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
	}

	if (mQueue != NULL) {
		ret = vbuf_queue_destroy(mQueue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_destroy", -ret);
		mQueue = NULL;
	}

	mMedia = NULL;

	return 0;
}


int Gles2Renderer::getInputSourceQueue(
	Media *media,
	struct vbuf_queue **queue)
{
	if (media == NULL)
		return -EINVAL;
	if (queue == NULL)
		return -EINVAL;
	if (media != mMedia) {
		ULOGE("invalid media");
		return -ENOENT;
	}

	*queue = mQueue;

	return 0;
}


int Gles2Renderer::loadVideoFrame(
	const uint8_t *data,
	struct avcdecoder_output_buffer *frame,
	enum gles2_video_color_conversion colorConversion)
{
	int ret;
	ret = mGles2Video->loadFrame(data, frame->plane_offset, frame->stride,
		frame->width, frame->height, colorConversion);
	if (ret < 0)
		ULOG_ERRNO("gles2Video->loadFrame", -ret);
	return 0;
}


int Gles2Renderer::render(
	int renderX,
	int renderY,
	unsigned int renderWidth,
	unsigned int renderHeight,
	uint64_t timestamp)
{
	int ret = 0;
	struct vbuf_buffer *buffer = NULL;
	const uint8_t *cdata;
	int dequeueRet = 0;
	bool load = false;

	if (!mRunning)
		return 0;

	if (renderWidth == 0)
		renderWidth = mWindowWidth - renderX;
	if (renderHeight == 0)
		renderHeight = mWindowHeight - renderY;
	if ((renderWidth == 0) || (renderHeight == 0))
		return 0;

	if (mQueue == NULL) {
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		return 0;
	}

	dequeueRet = vbuf_queue_pop(mQueue, 0, &buffer);
	while ((dequeueRet == 0) && (buffer != NULL)) {
		if (mCurrentBuffer != NULL) {
			int releaseRet = vbuf_unref(&mCurrentBuffer);
			if (releaseRet < 0)
				ULOG_ERRNO("vbuf_unref", -releaseRet);
		}
		mCurrentBuffer = buffer;
		load = true;
		dequeueRet = vbuf_queue_pop(mQueue, 0, &buffer);
	}
	if ((dequeueRet < 0) && (dequeueRet != -EAGAIN)) {
		ULOG_ERRNO("vbuf_queue_pop", -dequeueRet);
	}

	if (mCurrentBuffer == NULL)
		return 0;

	cdata = vbuf_get_cdata(mCurrentBuffer);
	struct avcdecoder_output_buffer *data =
		(struct avcdecoder_output_buffer *)
		vbuf_metadata_get(mCurrentBuffer, mMedia, NULL, NULL);

	if ((cdata == NULL) || (data == NULL)) {
		ULOGE("invalid buffer data");
		return -EPROTO;
	}

	if (mHmdDistorsionCorrection) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mFbo));
		GLCHK(glViewport(0, 0, renderWidth / 2, renderHeight));
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		GLCHK(glDisable(GL_DITHER));
	} else {
		GLCHK(glViewport(renderX, renderY,
			renderWidth, renderHeight));
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		GLCHK(glDisable(GL_DITHER));
	}

	if (mGles2Video) {
		enum gles2_video_color_conversion colorConversion;
		switch (data->colorFormat) {
		default:
		case AVCDECODER_COLOR_FORMAT_YUV420PLANAR:
			colorConversion =
				GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB;
			break;
		case AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR:
			colorConversion =
				GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB;
			break;
		case AVCDECODER_COLOR_FORMAT_MMAL_OPAQUE:
			colorConversion =
				GLES2_VIDEO_COLOR_CONVERSION_NONE;
			break;
		}

		ret = (load) ? loadVideoFrame(cdata, data, colorConversion) : 0;
		if (ret == 0) {
			ret = mGles2Video->renderFrame(data->stride,
				data->height, data->cropLeft, data->cropTop,
				data->cropWidth, data->cropHeight,
				data->sarWidth, data->sarHeight,
				(mHmdDistorsionCorrection) ? 0 : renderX,
				(mHmdDistorsionCorrection) ? 0 : renderY,
				(mHmdDistorsionCorrection) ? renderWidth / 2 :
				renderWidth, renderHeight,
				colorConversion, &data->metadata, mHeadtracking,
				(mHmdDistorsionCorrection) ? mFbo : 0);
			if (ret < 0)
				ULOG_ERRNO("gles2Video->renderFrame", -ret);
		}
	}

	if (mGles2Hud) {
		ret = mGles2Hud->renderHud(data->width * data->sarWidth,
			data->height * data->sarHeight,
			(mHmdDistorsionCorrection) ? renderWidth / 2 :
			renderWidth, renderHeight, &data->metadata,
			mHmdDistorsionCorrection, mHeadtracking);
		if (ret < 0)
			ULOG_ERRNO("gles2Hud->renderHud", -ret);
	}

	if (mHmdDistorsionCorrection) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
		GLCHK(glViewport(renderX, renderY,
			renderWidth, renderHeight));

		if (mGles2Hmd) {
			ret = mGles2Hmd->renderHmd(mFboTexture,
				renderWidth / 2, renderHeight);
			if (ret < 0)
				ULOG_ERRNO("gles2Hmd->renderHmd", -ret);
		}
	}

#if 0
	/* TODO: remove debug */
	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	uint64_t renderTimestamp, renderTimestamp1;
	renderTimestamp1 = renderTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	uint64_t currentTime = mSession->getCurrentTime();
	uint64_t duration = mSession->getDuration();
	unsigned int cHrs = 0, cMin = 0, cSec = 0, cMsec = 0;
	unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
	if ((currentTime > 0) && (currentTime != (uint64_t)-1)) {
		pdraw_friendlyTimeFromUs(currentTime,
			&cHrs, &cMin, &cSec, &cMsec);
	}
	if ((duration > 0) && (duration != (uint64_t)-1)) {
		pdraw_friendlyTimeFromUs(duration,
			&dHrs, &dMin, &dSec, &dMsec);
	}


	ULOGI("%02d:%02d:%02d.%03d / %02d:%02d:%02d.%03d "
		"frame (decoding: %.2fms, rendering: %.2fms, "
		"est. latency: %.2fms) render@%.1ffps",
		cHrs, cMin, cSec, cMsec, dHrs, dMin, dSec, dMsec,
		(float)(data->decoderOutputTimestamp -
			data->demuxOutputTimestamp) / 1000.,
		(float)(renderTimestamp1 -
			data->decoderOutputTimestamp) / 1000.,
		(data->auNtpTimestampLocal != 0) ? (float)(renderTimestamp -
			data->auNtpTimestampLocal) / 1000. : 0.,
		(renderTimestamp - timestamp > 0) ? 1000000. /
			((float)(renderTimestamp - timestamp)) : 0.);
#endif

	return 0;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
