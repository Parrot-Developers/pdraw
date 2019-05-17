/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 renderer
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

#include "pdraw_renderer_gles2.hpp"
#include "pdraw_session.hpp"
#include "pdraw_settings.hpp"

#ifdef USE_GLES2

#	include <string.h>
#	include <time.h>
#	include <unistd.h>

#	define ULOG_TAG pdraw_rndvidgl
#	include <ulog.h>
ULOG_DECLARE_TAG(pdraw_rndvidgl);

namespace Pdraw {


#	define GLES2_RENDERER_DEFAULT_DELAY_MS 33
#	define GLES2_RENDERER_FADE_FROM_BLACK_DURATION 500000
#	define GLES2_RENDERER_FADE_TO_BLACK_DURATION 500000
#	define GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_DURATION 2000000
#	define GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_HOLD 0
#	define GLES2_RENDERER_FADE_TO_BLUR_DURATION 2000000
#	define GLES2_RENDERER_FADE_TO_BLUR_HOLD 500000
#	define GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_DURATION 300000
#	define GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_HOLD 300000


/* Called on the rendering thread */
Gles2Renderer::Gles2Renderer(Session *session,
			     Element::Listener *listener,
			     IPdraw::VideoRendererListener *rndListener) :
		Renderer(session,
			 listener,
			 rndListener,
			 Media::Type::VIDEO,
			 VideoMedia::Format::YUV,
			 VideoMedia::YuvFormat::I420 |
				 VideoMedia::YuvFormat::NV12)
{
	int ret;

	Element::mName = "Gles2Renderer";
	Sink::mName = "Gles2Renderer";
	mRunning = false;
	mRendering = false;
	mRenderMutexCreated = false;
	mRenderCondCreated = false;
	mCurrentBuffer = NULL;
	mLastAddedMedia = NULL;
	mTimer = NULL;
	mGles2Video = NULL;
	mGles2HmdFirstTexUnit = 0;
	mGles2VideoFirstTexUnit =
		mGles2HmdFirstTexUnit + Gles2Hmd::getTexUnitCount();
	mGles2Hmd = NULL;
	mDefaultFbo = 0;
	mHmdFboSize = 0;
	mHmdFbo = 0;
	mHmdFboTexture = 0;
	mHmdFboRenderBuffer = 0;
	mExtLoadFbo = 0;
	mExtLoadFboTexture = 0;
	mX = 0;
	mY = 0;
	mWidth = 0;
	mHeight = 0;
	mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_NONE;
	mPendingTransition = Transition::NONE;
	mCurrentTransition = Transition::NONE;
	mTransitionStartTime = 0;
	mTransitionHoldTime = 0;
	memset(&mParams, 0, sizeof(mParams));
	mExtLoadVideoTexture = false;
	mExtVideoTextureWidth = 0;
	mExtVideoTextureHeight = 0;
	mRenderVideoOverlay = false;
	mFirstFrame = false;

	ret = pthread_mutex_init(&mRenderMutex, NULL);
	if (ret != 0) {
		ULOG_ERRNO("pthread_mutex_init", ret);
		goto err;
	}
	mRenderMutexCreated = true;

	ret = pthread_cond_init(&mRenderCond, NULL);
	if (ret != 0) {
		ULOG_ERRNO("pthread_cond_init", ret);
		goto err;
	}
	mRenderCondCreated = true;

	if (mRendererListener != NULL) {
		ret = mRendererListener->loadVideoTexture(
			NULL, NULL, 0, 0, NULL, NULL, NULL, NULL, 0);
		if (ret != -ENOSYS)
			mExtLoadVideoTexture = true;
	}
	if (mRendererListener != NULL) {
		ret = mRendererListener->renderVideoOverlay(NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL,
							    NULL);
		if (ret != -ENOSYS)
			mRenderVideoOverlay = true;
	}

	/* Create the notification timer */
	mTimer = pomp_timer_new(mSession->getLoop(), timerCb, this);
	if (mTimer == NULL) {
		ULOGE("pomp_timer_new failed");
		goto err;
	}

	/* Post a message on the loop thread */
	setStateAsyncNotify(CREATED);
	return;

err:
	if (mRenderCondCreated) {
		ret = pthread_cond_destroy(&mRenderCond);
		if (ret != 0)
			ULOG_ERRNO("pthread_cond_destroy", ret);
		mRenderCondCreated = false;
	}
	if (mRenderMutexCreated) {
		ret = pthread_mutex_destroy(&mRenderMutex);
		if (ret != 0)
			ULOG_ERRNO("pthread_mutex_destroy", ret);
		mRenderMutexCreated = false;
	}
	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}
}


/* Must be called on the loop thread */
Gles2Renderer::~Gles2Renderer(void)
{
	if (mState == STARTED)
		ULOGW("renderer is still running");

	int ret;
	unsigned int count = getInputMediaCount();
	if (count > 0) {
		ULOGW("not all input media have been removed");
		ret = removeInputMedias();
		if (ret < 0)
			ULOG_ERRNO("removeInputMedias", -ret);
	}

	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}

	if (mGles2Video != NULL) {
		delete mGles2Video;
		mGles2Video = NULL;
	}
	if (mGles2Hmd != NULL) {
		delete mGles2Hmd;
		mGles2Hmd = NULL;
	}
	if (mRenderCondCreated) {
		ret = pthread_cond_destroy(&mRenderCond);
		if (ret != 0)
			ULOG_ERRNO("pthread_cond_destroy", ret);
		mRenderCondCreated = false;
	}
	if (mRenderMutexCreated) {
		ret = pthread_mutex_destroy(&mRenderMutex);
		if (ret != 0)
			ULOG_ERRNO("pthread_mutex_destroy", ret);
		mRenderMutexCreated = false;
	}
}


/* Called on the rendering thread */
int Gles2Renderer::setup(const struct pdraw_rect *renderPos,
			 const struct pdraw_video_renderer_params *params,
			 struct egl_display *eglDisplay)
{
	int ret = 0;

	if (params == NULL)
		return -EINVAL;
	if (renderPos == NULL)
		return -EINVAL;

	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	ret = resize(renderPos);
	if (ret < 0) {
		ULOG_ERRNO("resize", -ret);
		goto out;
	}

	ret = setParams(params);
	if (ret < 0) {
		ULOG_ERRNO("setParams", -ret);
		goto out;
	}

	mGles2Video = new Gles2Video(mSession,
				     (mParams.enable_hmd_distortion_correction)
					     ? mHmdFbo
					     : mDefaultFbo,
				     mGles2VideoFirstTexUnit);
	if (mGles2Video == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("failed to create Gles2Video", -ret);
		goto out;
	}
	mGles2Video->setExtTexture(mExtLoadFboTexture);

out:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::start(void)
{
	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		ULOGE("renderer is not created");
		return -EPROTO;
	}
	/* Post a message on the loop thread */
	setStateAsyncNotify(STARTING);

	mRunning = true;

	/* Post a message on the loop thread */
	setStateAsyncNotify(STARTED);

	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		ULOGE("renderer is not started");
		return -EPROTO;
	}

	removeRendererListener();

	/* Post a message on the loop thread */
	setStateAsyncNotify(STOPPING);
	mRunning = false;

	if ((mGles2Video != NULL) || (mGles2Hmd != NULL))
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	if (mGles2Video != NULL) {
		delete mGles2Video;
		mGles2Video = NULL;
	}

	ret = stopHmd();
	if (ret < 0)
		ULOG_ERRNO("stopHmd", -ret);
	ret = stopExtLoad();
	if (ret < 0)
		ULOG_ERRNO("stopExtLoad", -ret);
	mExtLoadVideoTexture = false;

	/* Post a message on the loop thread */
	mSession->asyncRendererCompleteStop(this);

	return 0;
}


/* Called on the loop thread */
void Gles2Renderer::completeStop(void)
{
	int ret;

	ret = pomp_timer_clear(mTimer);
	if (ret < 0)
		ULOG_ERRNO("pomp_timer_clear", -ret);

	ret = removeInputMedias();
	if (ret < 0)
		ULOG_ERRNO("removeInputMedias", -ret);

	setState(STOPPED);
}


/* Called on the loop thread */
void Gles2Renderer::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	Gles2Renderer *renderer = (Gles2Renderer *)userdata;
	int res = 0;

	if (renderer == NULL) {
		ULOGE("invalid renderer pointer");
		return;
	}

	pthread_mutex_lock(&renderer->mListenerMutex);
	if (renderer->mRendererListener) {
		renderer->mRendererListener->onVideoRenderReady(
			renderer->mSession,
			reinterpret_cast<struct pdraw_video_renderer *>(
				renderer));
		res = pomp_timer_set(renderer->mTimer,
				     GLES2_RENDERER_DEFAULT_DELAY_MS);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_set", -res);
	}
	pthread_mutex_unlock(&renderer->mListenerMutex);
}


/* Called on the loop thread */
void Gles2Renderer::timerCb(struct pomp_timer *timer, void *userdata)
{
	Gles2Renderer *renderer = (Gles2Renderer *)userdata;
	int res;

	if ((!renderer->mRunning) || (renderer->mState != STARTED))
		return;

	pthread_mutex_lock(&renderer->mListenerMutex);
	if (renderer->mRendererListener) {
		renderer->mRendererListener->onVideoRenderReady(
			renderer->mSession,
			reinterpret_cast<struct pdraw_video_renderer *>(
				renderer));
		res = pomp_timer_set(renderer->mTimer,
				     GLES2_RENDERER_DEFAULT_DELAY_MS);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_set", -res);
	}
	pthread_mutex_unlock(&renderer->mListenerMutex);
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelFlush(Channel *channel)
{
	int ret;

	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	ULOGD("flushing input channel");

	/* Synchronize with rendering (avoid using mCurrentBuffer after
	 * unreferencing it) */
	pthread_mutex_lock(&mRenderMutex);
	while (mRendering)
		pthread_cond_wait(&mRenderCond, &mRenderMutex);
	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}
	pthread_mutex_unlock(&mRenderMutex);

	Sink::lock();

	struct vbuf_queue *queue = channel->getQueue();
	if (queue != NULL) {
		ret = vbuf_queue_flush(queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_flush", -ret);
	}

	Sink::unlock();

	ret = channel->flushDone();
	if (ret < 0)
		ULOG_ERRNO("channel->flushDone", -ret);
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelSos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelSos(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS)
		mPendingTransition = Transition::FADE_FROM_BLACK;

	Sink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelEos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelEos(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS)
		mPendingTransition = Transition::FADE_TO_BLACK;

	Sink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelReconfigure(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelReconfigure(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE)
		mPendingTransition = Transition::FADE_TO_BLUR;

	Sink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelTimeout(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelTimeout(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT)
		mPendingTransition = Transition::FADE_TO_BLACK_AND_WHITE;

	Sink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelTimeout(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER)
		mPendingTransition = Transition::FLASH_THEN_BLACK_AND_WHITE;

	Sink::unlock();
}


/* Must be called on the loop thread */
int Gles2Renderer::addInputMedia(Media *media)
{
	struct pomp_loop *loop = NULL;
	struct pomp_evt *evt = NULL;
	int res = 0;

	VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
	if (videoMedia == NULL) {
		ULOGE("invalid video media");
		return -EINVAL;
	}

	Sink::lock();

	mFirstFrame = true;

	res = Sink::addInputMedia(media);
	if (res == -EEXIST) {
		Sink::unlock();
		return res;
	} else if (res < 0) {
		Sink::unlock();
		ULOG_ERRNO("Sink::addInputMedia", -res);
		return res;
	}

	Channel *channel = getInputChannel(media);
	if (channel == NULL) {
		Sink::unlock();
		ULOGE("failed to get channel");
		return -EPROTO;
	}

	channel->setKey(this);
	struct vbuf_queue *queue;
	res = vbuf_queue_new(2, 1, &queue);
	if (res < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_queue_new", -res);
		return res;
	}
	channel->setQueue(queue);

	evt = vbuf_queue_get_evt(queue);
	if (evt == NULL) {
		Sink::unlock();
		ULOGE("bad queue evt");
		res = -ENODEV;
		goto error;
	}

	loop = mSession->getLoop();
	if (loop == NULL) {
		Sink::unlock();
		ULOGE("loop not found");
		res = -ENODEV;
		goto error;
	}

	res = pomp_evt_attach_to_loop(evt, loop, &queueEventCb, this);
	if (res < 0) {
		Sink::unlock();
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	mLastAddedMedia = media;
	if (mGles2Video != NULL)
		mGles2Video->setVideoMedia(videoMedia);
	Sink::unlock();

	return 0;

error:
	removeQueueFdFromPomp(queue);
	return res;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeQueueFdFromPomp(struct vbuf_queue *queue)
{
	int ret;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *evt = NULL;

	loop = mSession->getLoop();
	if (loop == NULL) {
		ULOGE("loop not found");
		return -ENODEV;
	}

	evt = vbuf_queue_get_evt(queue);
	if (evt == NULL) {
		ULOGE("bad queue evt");
		return -ENODEV;
	}

	ret = pomp_evt_detach_from_loop(evt, loop);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_detach_from_loop", -ret);
		return ret;
	}

	return 0;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeInputMedia(Media *media)
{
	int ret;

	/* Synchronize with rendering (avoid using mCurrentBuffer after
	 * unreferencing it) */
	pthread_mutex_lock(&mRenderMutex);
	while (mRendering)
		pthread_cond_wait(&mRenderCond, &mRenderMutex);
	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}
	pthread_mutex_unlock(&mRenderMutex);

	Sink::lock();

	Channel *channel = getInputChannel(media);
	if (channel == NULL) {
		Sink::unlock();
		ULOGE("failed to get channel");
		return -EPROTO;
	}
	/* Keep a reference on the queue to destroy it after removing the
	 * input media (avoids deadlocks when trying to push new frames out
	 * of the AvcDecoder whereas the queue is already destroyed) */
	struct vbuf_queue *queue = channel->getQueue();

	ret = Sink::removeInputMedia(media);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("Sink::removeInputMedia", -ret);
		return ret;
	}

	if (queue != NULL) {
		ret = removeQueueFdFromPomp(queue);
		if (ret < 0)
			ULOG_ERRNO("removeQueueFdFromPomp", -ret);
		ret = vbuf_queue_flush(queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_flush", -ret);
		ret = vbuf_queue_destroy(queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_destroy", -ret);
	}

	if (mLastAddedMedia == media) {
		mLastAddedMedia = NULL;
		if (mGles2Video != NULL)
			mGles2Video->setVideoMedia(NULL);
	}
	Sink::unlock();

	return 0;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeInputMedias(void)
{
	int ret, inputMediaCount, i;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		Media *media = getInputMedia(i);
		if (media == NULL) {
			ULOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		ret = removeInputMedia(media);
		if (ret < 0) {
			ULOG_ERRNO("removeInputMedia", -ret);
			continue;
		}
	}

	mLastAddedMedia = NULL;
	if (mGles2Video != NULL)
		mGles2Video->setVideoMedia(NULL);
	Sink::unlock();

	return 0;
}


/* Called on the rendering thread */
void Gles2Renderer::abortTransition(void)
{
	Sink::lock();
	mCurrentTransition = Transition::NONE;
	mTransitionStartTime = 0;
	mTransitionHoldTime = 0;
	Sink::unlock();

	if (mGles2Video) {
		mGles2Video->setSatCoef(1.0f);
		mGles2Video->setLightCoef(1.0f);
		mGles2Video->setDarkCoef(1.0f);
		mGles2Video->abortTransition();
	}
}


/* Called on the rendering thread */
int Gles2Renderer::doTransition(uint64_t timestamp,
				bool frameReady,
				bool *loadFrame)
{
	if (timestamp == 0)
		return -EINVAL;
	if (loadFrame == NULL)
		return -EINVAL;

	Sink::lock();

	if (mPendingTransition == Transition::NONE)
		goto out;

	if (mCurrentTransition != Transition::NONE) {
		/* Abort previous transition */
		abortTransition();
	}

	mCurrentTransition = mPendingTransition;
	mPendingTransition = Transition::NONE;
	mTransitionStartTime = timestamp;

	switch (mCurrentTransition) {
	case Transition::FADE_FROM_BLACK:
		mTransitionHoldTime = GLES2_RENDERER_FADE_FROM_BLACK_DURATION;
		if (mGles2Video) {
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FADE_FROM_BLACK,
				GLES2_RENDERER_FADE_FROM_BLACK_DURATION,
				false);
		}
		break;
	case Transition::FADE_TO_BLACK:
		mTransitionHoldTime = GLES2_RENDERER_FADE_TO_BLACK_DURATION;
		if (mGles2Video) {
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FADE_TO_BLACK,
				GLES2_RENDERER_FADE_TO_BLACK_DURATION,
				true);
		}
		break;
	case Transition::FADE_TO_BLACK_AND_WHITE:
		mTransitionHoldTime =
			GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_HOLD;
		if (mGles2Video) {
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FADE_TO_BLACK_AND_WHITE,
				GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_DURATION,
				true);
		}
		break;
	case Transition::FADE_TO_BLUR:
		mTransitionHoldTime = GLES2_RENDERER_FADE_TO_BLUR_HOLD;
		if (mGles2Video) {
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FADE_TO_BLUR,
				GLES2_RENDERER_FADE_TO_BLUR_DURATION,
				true);
		}
		break;
	case Transition::FLASH_THEN_BLACK_AND_WHITE:
		mTransitionHoldTime =
			GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_HOLD;
		if (mGles2Video) {
			mGles2Video->setSatCoef(0.0f);
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FADE_FROM_WHITE,
				GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_DURATION,
				true);
		}
		break;
	default:
		abortTransition();
		break;
	}

out:
	if ((frameReady) && (mCurrentTransition != Transition::NONE) &&
	    (timestamp >= mTransitionStartTime + mTransitionHoldTime)) {
		/* End the transition */
		abortTransition();
		*loadFrame = true;
	} else if ((frameReady) &&
		   ((mCurrentTransition == Transition::NONE) ||
		    (mCurrentTransition == Transition::FADE_FROM_BLACK))) {
		*loadFrame = true;
	} else {
		*loadFrame = false;
	}
	Sink::unlock();
	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::loadVideoFrame(const uint8_t *data, VideoMedia::Frame *frame)
{
	int ret;

	if (frame->format != VideoMedia::Format::YUV) {
		ULOGE("unsupported frame format");
		return -ENOSYS;
	}

	switch (frame->yuvFrame.format) {
	case VideoMedia::YuvFormat::I420:
		mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB;
		break;
	case VideoMedia::YuvFormat::NV12:
		mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB;
		break;
	default:
		ULOGE("unsupported frame sub-format");
		return -ENOSYS;
	}

	if ((frame->yuvFrame.width == 0) || (frame->yuvFrame.sarWidth == 0) ||
	    (frame->yuvFrame.height == 0) || (frame->yuvFrame.sarHeight == 0)) {
		ULOGE("invalid frame dimensions");
		return -EINVAL;
	}

	ret = mGles2Video->loadFrame(data,
				     frame->yuvFrame.planeOffset,
				     frame->yuvFrame.planeStride,
				     frame->yuvFrame.width,
				     frame->yuvFrame.height,
				     mColorConversion);
	if (ret < 0)
		ULOG_ERRNO("gles2Video->loadFrame", -ret);

	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::loadExternalVideoFrame(
	const uint8_t *data,
	VideoMedia::Frame *frame,
	const struct pdraw_session_info *session_info,
	const struct vmeta_session *session_meta)
{
	int ret;
	struct pdraw_video_frame meta;
	const void *frameUserdata = vbuf_get_cuserdata(mCurrentBuffer);
	ssize_t frameUserdataLen = vbuf_get_userdata_size(mCurrentBuffer);

	if (frameUserdataLen < 0) {
		ULOG_ERRNO("vbuf_get_userdata_size", (int)-frameUserdataLen);
		return frameUserdataLen;
	}

	if (frame->format != VideoMedia::Format::YUV) {
		ULOGE("unsupported frame format");
		return -ENOSYS;
	}

	if ((frame->yuvFrame.width == 0) || (frame->yuvFrame.sarWidth == 0) ||
	    (frame->yuvFrame.height == 0) || (frame->yuvFrame.sarHeight == 0)) {
		ULOGE("invalid frame dimensions");
		return -EINVAL;
	}

	/* Fill the output frame data */
	memset(&meta, 0, sizeof(meta));
	switch (frame->yuvFrame.format) {
	case VideoMedia::YuvFormat::I420:
		meta.yuv.format = PDRAW_YUV_FORMAT_I420;
		break;
	case VideoMedia::YuvFormat::NV12:
		meta.yuv.format = PDRAW_YUV_FORMAT_NV12;
		break;
	default:
		meta.yuv.format = PDRAW_YUV_FORMAT_UNKNOWN;
		break;
	}
	meta.yuv.plane[0] = data + frame->yuvFrame.planeOffset[0];
	meta.yuv.plane[1] = data + frame->yuvFrame.planeOffset[1];
	meta.yuv.plane[2] = data + frame->yuvFrame.planeOffset[2];
	meta.yuv.stride[0] = frame->yuvFrame.planeStride[0];
	meta.yuv.stride[1] = frame->yuvFrame.planeStride[1];
	meta.yuv.stride[2] = frame->yuvFrame.planeStride[2];
	meta.yuv.width = frame->yuvFrame.width;
	meta.yuv.height = frame->yuvFrame.height;
	meta.yuv.sar_width = frame->yuvFrame.sarWidth;
	meta.yuv.sar_height = frame->yuvFrame.sarHeight;
	meta.yuv.crop_left = frame->yuvFrame.cropLeft;
	meta.yuv.crop_top = frame->yuvFrame.cropTop;
	meta.yuv.crop_width = frame->yuvFrame.cropWidth;
	meta.yuv.crop_height = frame->yuvFrame.cropHeight;
	meta.format = (enum pdraw_video_media_format)frame->format;
	meta.has_errors = (frame->hasErrors) ? 1 : 0;
	meta.is_silent = (frame->isSilent) ? 1 : 0;
	meta.ntp_timestamp = frame->ntpTimestamp;
	meta.ntp_unskewed_timestamp = frame->ntpUnskewedTimestamp;
	meta.ntp_raw_timestamp = frame->ntpRawTimestamp;
	meta.ntp_raw_unskewed_timestamp = frame->ntpRawUnskewedTimestamp;
	meta.play_timestamp = frame->playTimestamp;
	meta.capture_timestamp = frame->captureTimestamp;
	meta.local_timestamp = frame->localTimestamp;
	meta.has_metadata = (frame->hasMetadata) ? 1 : 0;
	meta.metadata = frame->metadata;

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mExtLoadFbo));
	GLCHK(glViewport(0, 0, mExtVideoTextureWidth, mExtVideoTextureHeight));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	/* Texture loading callback function */
	ret = mRendererListener->loadVideoTexture(
		mSession,
		reinterpret_cast<struct pdraw_video_renderer *>(this),
		mExtVideoTextureWidth,
		mExtVideoTextureHeight,
		session_info,
		session_meta,
		&meta,
		frameUserdata,
		(size_t)frameUserdataLen);

	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));
		GLCHK(glViewport(0, 0, mHmdFboSize, mHmdFboSize));
	} else {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
	}

	mColorConversion = GLES2_VIDEO_COLOR_CONVERSION_NONE;

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::renderVideoFrame(VideoMedia::Frame *frame,
				    const struct pdraw_rect *renderPos,
				    struct pdraw_rect *contentPos,
				    Eigen::Matrix4f &viewProjMat)
{
	enum gles2_video_yuv_range yuvRange =
		frame->yuvFrame.fullRange ? GLES2_VIDEO_YUV_FULL_RANGE
					  : GLES2_VIDEO_YUV_LIMITED_RANGE;

	return mGles2Video->renderFrame(frame->yuvFrame.planeStride,
					frame->yuvFrame.height,
					frame->yuvFrame.cropLeft,
					frame->yuvFrame.cropTop,
					frame->yuvFrame.cropWidth,
					frame->yuvFrame.cropHeight,
					frame->yuvFrame.sarWidth,
					frame->yuvFrame.sarHeight,
					renderPos,
					contentPos,
					viewProjMat,
					mColorConversion,
					yuvRange,
					&frame->metadata,
					&mParams);
}


/* Called on the rendering thread */
int Gles2Renderer::renderExternalVideoFrame(VideoMedia::Frame *frame,
					    const struct pdraw_rect *renderPos,
					    struct pdraw_rect *contentPos,
					    Eigen::Matrix4f &viewProjMat)
{
	enum gles2_video_yuv_range yuvRange =
		frame->yuvFrame.fullRange ? GLES2_VIDEO_YUV_FULL_RANGE
					  : GLES2_VIDEO_YUV_LIMITED_RANGE;
	size_t frameStride[3] = {mExtVideoTextureWidth, 0, 0};

	unsigned int sarWidth =
		frame->yuvFrame.cropWidth * mExtVideoTextureHeight;
	unsigned int sarHeight =
		frame->yuvFrame.cropHeight * mExtVideoTextureWidth;

	return mGles2Video->renderFrame(frameStride,
					mExtVideoTextureHeight,
					0,
					0,
					mExtVideoTextureWidth,
					mExtVideoTextureHeight,
					sarWidth,
					sarHeight,
					renderPos,
					contentPos,
					viewProjMat,
					mColorConversion,
					yuvRange,
					&frame->metadata,
					&mParams);
}

int Gles2Renderer::setupExtTexture(const VideoMedia::Frame *frame)
{
	int ret = 0;

	ULOGI("external video texture: source=%ux%u (SAR=%u:%u) "
	      "DAR=%u:%u textureWidth=%u",
	      frame->yuvFrame.width,
	      frame->yuvFrame.height,
	      frame->yuvFrame.sarWidth,
	      frame->yuvFrame.sarHeight,
	      mParams.video_texture_dar_width,
	      mParams.video_texture_dar_height,
	      mParams.video_texture_width);

	/* Compute the external texture size */
	if (mParams.video_texture_dar_width != 0 &&
	    mParams.video_texture_dar_height != 0) {
		/* Custom DAR is provided */
		if (mParams.video_texture_width > 0) {
			/* Custom DAR with custom texture width */
			mExtVideoTextureWidth = mParams.video_texture_width;
			mExtVideoTextureHeight =
				(mParams.video_texture_width *
					 mParams.video_texture_dar_height +
				 mParams.video_texture_dar_width / 2) /
				mParams.video_texture_dar_width;
		} else {
			/* Custom DAR without custom texture width */
			float dar, ar;
			dar = (float)mParams.video_texture_dar_width /
			      (float)mParams.video_texture_dar_height;
			ar = (float)frame->yuvFrame.width /
			     (float)frame->yuvFrame.height;
			/* Take the DAR into account, never decreasing
			 * the dimensions */
			if (dar > ar) {
				mExtVideoTextureWidth =
					(frame->yuvFrame.height *
						 mParams.video_texture_dar_width +
					 mParams.video_texture_dar_height / 2) /
					mParams.video_texture_dar_height;
				mExtVideoTextureHeight = frame->yuvFrame.height;
			} else {
				mExtVideoTextureWidth = frame->yuvFrame.width;
				mExtVideoTextureHeight =
					(frame->yuvFrame.width *
						 mParams.video_texture_dar_height +
					 mParams.video_texture_dar_width / 2) /
					mParams.video_texture_dar_width;
			}
		}
	} else if (mParams.video_texture_width > 0) {
		/* Custom texture width without custom DAR */
		mExtVideoTextureWidth = mParams.video_texture_width;
		mExtVideoTextureHeight =
			(mParams.video_texture_width * frame->yuvFrame.height +
			 frame->yuvFrame.width / 2) /
			frame->yuvFrame.width;

		/* Take the SAR into account */
		mExtVideoTextureHeight =
			(mExtVideoTextureHeight * frame->yuvFrame.sarHeight +
			 frame->yuvFrame.sarWidth / 2) /
			frame->yuvFrame.sarWidth;
	} else {
		/* No custom texture width and no custom DAR */
		float sar = (float)frame->yuvFrame.sarWidth /
			    (float)frame->yuvFrame.sarHeight;
		/* Take the SAR into account, never decreasing the dimensions */
		if (sar < 1.f) {
			mExtVideoTextureWidth = frame->yuvFrame.width;
			mExtVideoTextureHeight =
				(frame->yuvFrame.height *
					 frame->yuvFrame.sarHeight +
				 frame->yuvFrame.sarWidth / 2) /
				frame->yuvFrame.sarWidth;
		} else {
			mExtVideoTextureWidth =
				(frame->yuvFrame.width *
					 frame->yuvFrame.sarWidth +
				 frame->yuvFrame.sarHeight / 2) /
				frame->yuvFrame.sarHeight;
			mExtVideoTextureHeight = frame->yuvFrame.height;
		}
	}

	/* Round up to nearest multiple of 2 */
	mExtVideoTextureWidth = (mExtVideoTextureWidth + 1) & ~1;
	mExtVideoTextureHeight = (mExtVideoTextureHeight + 1) & ~1;

	/* start external Texture */
	if (mExtVideoTextureWidth != 0 && mExtVideoTextureHeight != 0) {
		ret = startExtLoad();
		if (ret < 0) {
			ULOG_ERRNO("startExtLoad", -ret);
			mExtLoadVideoTexture = false;
			mExtVideoTextureWidth = 0;
			mExtVideoTextureHeight = 0;
			mParams.video_texture_width = 0;
			mParams.video_texture_dar_width = 0;
			mParams.video_texture_dar_height = 0;
		}
	} else {
		mExtLoadVideoTexture = false;
		mExtVideoTextureWidth = 0;
		mExtVideoTextureHeight = 0;
		mParams.video_texture_width = 0;
		mParams.video_texture_dar_width = 0;
		mParams.video_texture_dar_height = 0;
		ret = stopExtLoad();
		if (ret < 0)
			ULOG_ERRNO("stopExtLoad", -ret);
	}

	ULOGI("external video texture: size=%ux%u",
	      mExtVideoTextureWidth,
	      mExtVideoTextureHeight);

	return 0;
}


void Gles2Renderer::createProjMatrix(Eigen::Matrix4f &projMat,
				     float aspectRatio,
				     float near,
				     float far)
{
	float w = 1.f;
	float h = aspectRatio;
	float a = (far + near) / (far - near);
	float b = -((2 * far * near) / (far - near));

	projMat << w, 0, 0, 0, 0, h, 0, 0, 0, 0, a, b, 0, 0, 1, 0;
}


/* Called on the rendering thread */
int Gles2Renderer::render(struct pdraw_rect *contentPos,
			  const float *viewMat,
			  const float *projMat)
{
	int ret = 0, err;
	struct vbuf_buffer *buffer = NULL;
	const uint8_t *cdata;
	int dequeueRet = 0;
	struct pdraw_rect renderPos;
	bool load = false, render = false;
	uint64_t curTime = 0;
	struct timespec ts = {0, 0};
	Media *media = NULL;
	InputPort *port = NULL;
	struct vbuf_queue *queue = NULL;
	Eigen::Matrix4f vpMat, vMat, pMat;
	struct pdraw_rect content;
	memset(&content, 0, sizeof(content));

	if (contentPos != NULL)
		*contentPos = content;

	if ((!mRunning) || (mState != STARTED))
		return 0;

	if ((mWidth == 0) || (mHeight == 0))
		return 0;

	pthread_mutex_lock(&mRenderMutex);
	mRendering = true;
	pthread_mutex_unlock(&mRenderMutex);

	Sink::lock();
	if (mLastAddedMedia == NULL) {
		Sink::unlock();
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		goto out;
	}
	media = findInputMedia(mLastAddedMedia);
	if (media == NULL) {
		Sink::unlock();
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		ULOGE("failed to find media");
		ret = -EPROTO;
		goto out;
	}
	port = getInputPort(media);
	if (port == NULL) {
		Sink::unlock();
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		ULOGE("failed to get input port");
		ret = -EPROTO;
		goto out;
	}
	queue = port->channel->getQueue();
	if (queue == NULL) {
		Sink::unlock();
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		ULOGE("failed to get input queue");
		ret = -EPROTO;
		goto out;
	}

	dequeueRet = vbuf_queue_pop(queue, 0, &buffer);
	while ((dequeueRet == 0) && (buffer != NULL)) {
		if (mCurrentBuffer != NULL) {
			int releaseRet = vbuf_unref(mCurrentBuffer);
			if (releaseRet < 0)
				ULOG_ERRNO("vbuf_unref", -releaseRet);
		}
		mCurrentBuffer = buffer;
		load = true;
		buffer = NULL;
		dequeueRet = vbuf_queue_pop(queue, 0, &buffer);
	}
	if ((dequeueRet < 0) && (dequeueRet != -EAGAIN)) {
		ULOG_ERRNO("vbuf_queue_pop", -dequeueRet);
	}

	Sink::unlock();

	if (mCurrentBuffer == NULL)
		goto out;

	cdata = vbuf_get_cdata(mCurrentBuffer);
	if (cdata == NULL) {
		ULOGE("invalid buffer data");
		ret = -EPROTO;
		goto out;
	}

	VideoMedia::Frame *data;
	ret = vbuf_metadata_get(
		mCurrentBuffer, media, NULL, NULL, (uint8_t **)&data);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get", -ret);
		goto out;
	}

	if (mFirstFrame) {
		mFirstFrame = false;
		if (mExtLoadVideoTexture) {
			ret = setupExtTexture(data);
			if (ret < 0)
				ULOG_ERRNO("setupExtTexture", -ret);
		}
	}

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));
		GLCHK(glViewport(0, 0, mHmdFboSize, mHmdFboSize));
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		GLCHK(glDisable(GL_DITHER));
		renderPos.x = 0;
		renderPos.y = 0;
		renderPos.width = mHmdFboSize;
		renderPos.height = mHmdFboSize;
	} else {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));
		GLCHK(glDisable(GL_DITHER));
		renderPos.x = mX;
		renderPos.y = mY;
		renderPos.width = mWidth;
		renderPos.height = mHeight;
	}

	struct pdraw_session_info sessionInfo;
	mSession->getSessionInfo(&sessionInfo);
	struct vmeta_session sessionMeta;
	mSession->getPeerMetadata()->get(&sessionMeta);

	if (viewMat != NULL) {
		unsigned int i, j;
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++)
				vMat(i, j) = viewMat[j * 4 + i];
		}
	} else {
		vMat = Eigen::Matrix4f::Identity();
	}
	if (projMat != NULL) {
		unsigned int i, j;
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++)
				pMat(i, j) = projMat[j * 4 + i];
		}
	} else {
		createProjMatrix(pMat,
				 (float)renderPos.width /
					 (float)renderPos.height,
				 0.1f,
				 100.f);
	}
	vpMat = pMat * vMat;

	if (mGles2Video) {
		err = time_get_monotonic(&ts);
		if (err < 0)
			ULOG_ERRNO("time_get_monotonic", -err);
		err = time_timespec_to_us(&ts, &curTime);
		if (err < 0)
			ULOG_ERRNO("time_timespec_to_us", -err);

		err = doTransition(curTime, load, &load);
		if (err < 0)
			ULOG_ERRNO("doTransition", -err);

		if (load || mExtLoadVideoTexture) {
			if (mExtLoadVideoTexture) {
				err = loadExternalVideoFrame(cdata,
							     data,
							     &sessionInfo,
							     &sessionMeta);
			} else {
				err = loadVideoFrame(cdata, data);
			}
		}
		if (err == 0) {
			if (mExtLoadVideoTexture) {
				err = renderExternalVideoFrame(
					data, &renderPos, &content, vpMat);
				if (err < 0) {
					ULOG_ERRNO(
						"gles2Video->"
						"renderExternalVideoFrame",
						-err);
				} else {
					render = true;
				}
			} else {
				err = renderVideoFrame(
					data, &renderPos, &content, vpMat);
				if (err < 0) {
					ULOG_ERRNO(
						"gles2Video->renderVideoFrame",
						-err);
				} else {
					render = true;
				}
			}
		}
	}

	/* Overlay rendering callback function */
	if (render && mRenderVideoOverlay) {
		struct pdraw_video_frame_extra frameExtra;
		memset(&frameExtra, 0, sizeof(frameExtra));
		frameExtra.play_timestamp = data->playTimestamp;
		if (mGles2Video) {
			mGles2Video->getHistograms(frameExtra.histogram,
						   frameExtra.histogram_len);
		}
		mRendererListener->renderVideoOverlay(
			mSession,
			reinterpret_cast<struct pdraw_video_renderer *>(this),
			&renderPos,
			&content,
			vMat.data(),
			pMat.data(),
			&sessionInfo,
			&sessionMeta,
			&data->metadata,
			&frameExtra);
	}

	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
		GLCHK(glClear(GL_COLOR_BUFFER_BIT));

		if (mGles2Hmd) {
			ret = mGles2Hmd->renderHmd(
				mHmdFboTexture, mHmdFboSize, mHmdFboSize);
			if (ret < 0)
				ULOG_ERRNO("gles2Hmd->renderHmd", -ret);
		}
	}

#	if 0
	/* TODO: remove debug */
	struct timespec ts = {0, 0};
	time_get_monotonic(&ts);
	uint64_t renderTimestamp = 0, renderTimestamp1;
	time_timespec_to_us(&ts, &renderTimestamp);
	renderTimestamp1 = renderTimestamp;

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
#	endif

out:
	pthread_mutex_lock(&mRenderMutex);
	mRendering = false;
	pthread_mutex_unlock(&mRenderMutex);
	pthread_cond_broadcast(&mRenderCond);

	if (contentPos != NULL)
		*contentPos = content;

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::startHmd(void)
{
	int ret = 0;
	GLenum gle;

	int err = stopHmd();
	if (err < 0)
		ULOG_ERRNO("stopHmd", -err);

	mHmdFboSize = (mWidth / 2 > mHeight) ? mWidth / 2 : mHeight;
	GLCHK();

	GLCHK(glGenFramebuffers(1, &mHmdFbo));
	if (mHmdFbo <= 0) {
		ULOGE("failed to create framebuffer");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));

	GLCHK(glGenTextures(1, &mHmdFboTexture));
	if (mHmdFboTexture <= 0) {
		ULOGE("failed to create texture");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glActiveTexture(GL_TEXTURE0 + mGles2HmdFirstTexUnit));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mHmdFboTexture));
	GLCHK(glTexImage2D(GL_TEXTURE_2D,
			   0,
			   GL_RGBA,
			   mHmdFboSize,
			   mHmdFboSize,
			   0,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   NULL));

	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCHK(glGenRenderbuffers(1, &mHmdFboRenderBuffer));
	if (mHmdFboRenderBuffer <= 0) {
		ULOGE("failed to create render buffer");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glBindRenderbuffer(GL_RENDERBUFFER, mHmdFboRenderBuffer));
	GLCHK(glRenderbufferStorage(GL_RENDERBUFFER,
				    GL_DEPTH_COMPONENT16,
				    mHmdFboSize,
				    mHmdFboSize));

	GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
				     GL_COLOR_ATTACHMENT0,
				     GL_TEXTURE_2D,
				     mHmdFboTexture,
				     0));
	GLCHK(glFramebufferRenderbuffer(GL_FRAMEBUFFER,
					GL_DEPTH_ATTACHMENT,
					GL_RENDERBUFFER,
					mHmdFboRenderBuffer));

	gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (gle != GL_FRAMEBUFFER_COMPLETE) {
		ULOGE("invalid framebuffer status");
		ret = -EPROTO;
		goto out;
	}

	GLCHK(glClear(GL_COLOR_BUFFER_BIT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, 0));
	GLCHK(glBindRenderbuffer(GL_RENDERBUFFER, 0));
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));

	mGles2Hmd = new Gles2Hmd(mSession,
				 mGles2HmdFirstTexUnit,
				 mWidth,
				 mHeight,
				 mParams.hmd_ipd_offset,
				 mParams.hmd_x_offset,
				 mParams.hmd_y_offset);
	if (mGles2Hmd == NULL) {
		ULOGE("failed to create Gles2Hmd context");
		ret = -ENOMEM;
		goto out;
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	if (mGles2Video)
		mGles2Video->setDefaultFbo(mHmdFbo);

	return 0;

out:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	if (mGles2Video)
		mGles2Video->setDefaultFbo(mDefaultFbo);

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::stopHmd(void)
{
	GLCHK();

	if (mGles2Hmd != NULL) {
		delete mGles2Hmd;
		mGles2Hmd = NULL;
	}
	if (mHmdFboRenderBuffer > 0) {
		GLCHK(glDeleteRenderbuffers(1, &mHmdFboRenderBuffer));
		mHmdFboRenderBuffer = 0;
	}
	if (mHmdFboTexture > 0) {
		GLCHK(glDeleteTextures(1, &mHmdFboTexture));
		mHmdFboTexture = 0;
	}
	if (mHmdFbo > 0) {
		GLCHK(glDeleteFramebuffers(1, &mHmdFbo));
		mHmdFbo = 0;
	}
	if (mGles2Video)
		mGles2Video->setDefaultFbo(mDefaultFbo);
	mHmdFboSize = 0;

	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::startExtLoad(void)
{
	int ret = 0, err;
	GLenum gle;

	err = stopExtLoad();
	if (err < 0)
		ULOG_ERRNO("stopExtLoad", -err);

	GLCHK();

	GLCHK(glGenFramebuffers(1, &mExtLoadFbo));
	if (mExtLoadFbo <= 0) {
		ULOGE("failed to create framebuffer");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mExtLoadFbo));

	GLCHK(glGenTextures(1, &mExtLoadFboTexture));
	if (mExtLoadFboTexture <= 0) {
		ULOGE("failed to create texture");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glActiveTexture(GL_TEXTURE0));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mExtLoadFboTexture));
	GLCHK(glTexImage2D(GL_TEXTURE_2D,
			   0,
			   GL_RGBA,
			   mExtVideoTextureWidth,
			   mExtVideoTextureHeight,
			   0,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   NULL));

	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
				     GL_COLOR_ATTACHMENT0,
				     GL_TEXTURE_2D,
				     mExtLoadFboTexture,
				     0));

	gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (gle != GL_FRAMEBUFFER_COMPLETE) {
		ULOGE("invalid framebuffer status");
		ret = -EPROTO;
		goto out;
	}

	GLCHK(glClear(GL_COLOR_BUFFER_BIT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, 0));

	if (mGles2Video)
		mGles2Video->setExtTexture(mExtLoadFboTexture);

out:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::stopExtLoad(void)
{
	GLCHK();

	if (mExtLoadFboTexture > 0) {
		GLCHK(glDeleteTextures(1, &mExtLoadFboTexture));
		mExtLoadFboTexture = 0;
	}
	if (mExtLoadFbo > 0) {
		GLCHK(glDeleteFramebuffers(1, &mExtLoadFbo));
		mExtLoadFbo = 0;
	}

	if (mGles2Video)
		mGles2Video->setExtTexture(mExtLoadFboTexture);

	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::resize(const struct pdraw_rect *render_pos)
{
	int ret;

	if (render_pos == NULL)
		return -EINVAL;

	mX = render_pos->x;
	mY = render_pos->y;
	mWidth = render_pos->width;
	mHeight = render_pos->height;

	GLCHK();
	GLCHK(glViewport(mX, mY, mWidth, mHeight));
	GLCHK(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	if (mParams.enable_hmd_distortion_correction) {
		ret = startHmd();
		if (ret < 0)
			ULOG_ERRNO("startHmd", -ret);
	} else {
		ret = stopHmd();
		if (ret < 0)
			ULOG_ERRNO("stopHmd", -ret);
	}

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::setParams(const struct pdraw_video_renderer_params *params)
{
	int ret;

	if (params == NULL)
		return -EINVAL;

	mParams = *params;

	if (mParams.video_scale_factor == 0.f)
		mParams.video_scale_factor = 1.f;

	if (mParams.enable_hmd_distortion_correction) {
		ret = startHmd();
		if (ret < 0)
			ULOG_ERRNO("startHmd", -ret);
	} else {
		ret = stopHmd();
		if (ret < 0)
			ULOG_ERRNO("stopHmd", -ret);
	}

	if (!mExtLoadVideoTexture) {
		mExtVideoTextureWidth = 0;
		mExtVideoTextureHeight = 0;
		mParams.video_texture_width = 0;
		mParams.video_texture_dar_width = 0;
		mParams.video_texture_dar_height = 0;
	}
	mFirstFrame = true;

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::getParams(struct pdraw_video_renderer_params *params)
{
	if (params)
		*params = mParams;
	return 0;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
