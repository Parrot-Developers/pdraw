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

#define ULOG_TAG pdraw_rndvidgl
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_renderer_gles2.hpp"
#include "pdraw_session.hpp"
#include "pdraw_settings.hpp"

#ifdef USE_GLES2

#	include <string.h>
#	include <time.h>
#	include <unistd.h>

#	include <media-buffers/mbuf_ancillary_data.h>

namespace Pdraw {


#	define GLES2_RENDERER_DEFAULT_DELAY_MS 33
#	define GLES2_RENDERER_FADE_FROM_BLACK_DURATION 500000
#	define GLES2_RENDERER_FADE_TO_BLACK_DURATION 500000
#	define GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_DURATION 2000000
#	define GLES2_RENDERER_FADE_TO_BLACK_AND_WHITE_HOLD 50000
#	define GLES2_RENDERER_FADE_TO_BLUR_DURATION 2000000
#	define GLES2_RENDERER_FADE_TO_BLUR_HOLD 500000
#	define GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_DURATION 200000
#	define GLES2_RENDERER_FLASH_THEN_BLACK_AND_WHITE_HOLD 200000
#	define GLES2_RENDERER_WATCHDOG_TIME_S 2


#	define NB_SUPPORTED_FORMATS 4
static struct vdef_raw_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_i420;
	supportedFormats[1] = vdef_nv12;
	supportedFormats[2] = vdef_i420_10_16le;
	supportedFormats[3] = vdef_nv12_10_16le_high;
}


/* Called on the rendering thread */
Gles2Renderer::Gles2Renderer(Session *session,
			     Element::Listener *listener,
			     IPdraw::IVideoRenderer *renderer,
			     IPdraw::IVideoRenderer::Listener *rndListener,
			     unsigned int mediaId,
			     const struct pdraw_rect *renderPos,
			     const struct pdraw_video_renderer_params *params,
			     struct egl_display *eglDisplay) :
		Renderer(session,
			 listener,
			 renderer,
			 rndListener,
			 Media::Type::RAW_VIDEO,
			 nullptr,
			 0,
			 mediaId,
			 renderPos,
			 params,
			 eglDisplay)
{
	int ret;

	Element::setClassName(__func__);
	mMediaId = mediaId;
	mCurrentMediaId = 0;
	mRunning = false;
	mCurrentFrame = nullptr;
	mCurrentFrameData = {};
	mCurrentFrameInfo = {};
	mCurrentFrameMetadata = nullptr;
	mLastAddedMedia = nullptr;
	mMediaInfo = {};
	mMediaInfoSessionMeta = {};
	mTimer = nullptr;
	mGles2Video = nullptr;
	mGles2HmdFirstTexUnit = 0;
	mGles2VideoFirstTexUnit =
		mGles2HmdFirstTexUnit + Gles2Hmd::getTexUnitCount();
	mGles2Hmd = nullptr;
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
	mFrameLoaded = false;
	mFrameLoadedLogged = false;
	mWatchdogTimer = nullptr;
	mWatchdogTriggered = false;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setRawVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	if (mRendererListener != nullptr) {
		ret = mRendererListener->loadVideoTexture(
			nullptr, nullptr, 0, 0, nullptr, nullptr, nullptr, 0);
		if (ret != -ENOSYS)
			mExtLoadVideoTexture = true;
	}
	if (mRendererListener != nullptr) {
		ret = mRendererListener->renderVideoOverlay(nullptr,
							    nullptr,
							    nullptr,
							    nullptr,
							    nullptr,
							    nullptr,
							    nullptr,
							    nullptr,
							    nullptr);
		if (ret != -ENOSYS)
			mRenderVideoOverlay = true;
	}

	if (renderPos != nullptr && params != nullptr) {
		ret = setup(renderPos, params, eglDisplay);
		if (ret != 0)
			return;
	}

	/* Post a message on the loop thread */
	setStateAsyncNotify(CREATED);
	return;
}


/* Must be called on the loop thread */
Gles2Renderer::~Gles2Renderer(void)
{
	int ret;

	if (mState == STARTED)
		PDRAW_LOGW("renderer is still running");

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("not all input media have been removed");
		ret = removeInputMedias();
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedias", -ret);
	}

	if (mCurrentFrameMetadata != nullptr) {
		vmeta_frame_unref(mCurrentFrameMetadata);
		mCurrentFrameMetadata = nullptr;
	}
	if (mCurrentFrame != nullptr) {
		int releaseRet = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (releaseRet < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
					-releaseRet);
		mCurrentFrame = nullptr;
	}

	Media::cleanupMediaInfo(&mMediaInfo);

	if (mTimer != nullptr) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = nullptr;
	}
	if (mWatchdogTimer != nullptr) {
		ret = pomp_timer_clear(mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
		mWatchdogTimer = nullptr;
	}

	if (mGles2Video != nullptr) {
		delete mGles2Video;
		mGles2Video = nullptr;
	}
	if (mGles2Hmd != nullptr) {
		delete mGles2Hmd;
		mGles2Hmd = nullptr;
	}
}


/* Called on the rendering thread */
int Gles2Renderer::setup(const struct pdraw_rect *renderPos,
			 const struct pdraw_video_renderer_params *params,
			 struct egl_display *eglDisplay)
{
	int ret = 0;

	if (params == nullptr)
		return -EINVAL;
	if (renderPos == nullptr)
		return -EINVAL;

	if ((mState != INVALID) && (mState != CREATED)) {
		PDRAW_LOGE("invalid state");
		return -EPROTO;
	}

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	ret = resize(renderPos);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("resize", -ret);
		goto out;
	}

	ret = setParams(params);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("setParams", -ret);
		goto out;
	}

	mGles2Video = new Gles2Video(mSession,
				     (mParams.enable_hmd_distortion_correction)
					     ? mHmdFbo
					     : mDefaultFbo,
				     mGles2VideoFirstTexUnit);
	if (mGles2Video == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("failed to create Gles2Video", -ret);
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
		PDRAW_LOGE("renderer is not created");
		return -EPROTO;
	}
	/* Post a message on the loop thread */
	setStateAsyncNotify(STARTING);

	mRunning = true;

	int ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleStart, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return ret;
}

/* Called on the loop thread by start() */
void Gles2Renderer::idleStart(void *renderer)
{
	Gles2Renderer *self = reinterpret_cast<Gles2Renderer *>(renderer);
	ULOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	int ret;
	if (self->mTimer == nullptr) {
		self->mTimer = pomp_timer_new(
			self->mSession->getLoop(), timerCb, self);
		if (self->mTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto err;
		}
	}
	if (self->mWatchdogTimer == nullptr) {
		self->mWatchdogTimer = pomp_timer_new(
			self->mSession->getLoop(), watchdogTimerCb, self);
		if (self->mWatchdogTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto err;
		}
	}

	if (self->mRendererListener != nullptr) {
		ret = pomp_timer_set(self->mTimer,
				     GLES2_RENDERER_DEFAULT_DELAY_MS);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -ret);
	}

	/* Post a message on the loop thread */
	self->setState(STARTED);
	return;

err:
	if (self->mTimer != nullptr) {
		ret = pomp_timer_clear(self->mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(self->mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
		self->mTimer = nullptr;
	}
	if (self->mWatchdogTimer != nullptr) {
		ret = pomp_timer_clear(self->mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(self->mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
		self->mWatchdogTimer = nullptr;
	}
}


/* Called on the rendering thread */
int Gles2Renderer::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("renderer is not started");
		return -EPROTO;
	}

	removeRendererListener();

	/* Post a message on the loop thread */
	setStateAsyncNotify(STOPPING);

	mRunning = false;

	if (mGles2Video != nullptr) {
		delete mGles2Video;
		mGles2Video = nullptr;
	}

	ret = stopHmd();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stopHmd", -ret);
	ret = stopExtLoad();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stopExtLoad", -ret);
	mExtLoadVideoTexture = false;

	/* Post a message on the loop thread */
	asyncCompleteStop();

	return 0;
}


/* Called on the loop thread */
void Gles2Renderer::completeStop(void)
{
	int ret;

	ret = pomp_timer_clear(mTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);

	ret = removeInputMedias();
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedias", -ret);

	setState(STOPPED);
}


/* Called on the loop thread */
void Gles2Renderer::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	Gles2Renderer *self = (Gles2Renderer *)userdata;
	int res = 0;

	if (self == nullptr) {
		PDRAW_LOGE("invalid renderer pointer");
		return;
	}

	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mRendererListener) {
		self->mRendererListener->onVideoRenderReady(self->mSession,
							    self->mRenderer);
		res = pomp_timer_set(self->mTimer,
				     GLES2_RENDERER_DEFAULT_DELAY_MS);
		if (res < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -res);
	}
	pthread_mutex_unlock(&self->mListenerMutex);
}


/* Called on the loop thread */
void Gles2Renderer::timerCb(struct pomp_timer *timer, void *userdata)
{
	Gles2Renderer *self = (Gles2Renderer *)userdata;
	int res;

	if ((!self->mRunning) || (self->mState != STARTED))
		return;

	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mRendererListener) {
		self->mRendererListener->onVideoRenderReady(self->mSession,
							    self->mRenderer);
	}
	pthread_mutex_unlock(&self->mListenerMutex);

	res = pomp_timer_set(self->mTimer, GLES2_RENDERER_DEFAULT_DELAY_MS);
	if (res < 0)
		PDRAW_LOG_ERRNO("pomp_timer_set", -res);
}


/* Called on the loop thread */
void Gles2Renderer::watchdogTimerCb(struct pomp_timer *timer, void *userdata)
{
	Gles2Renderer *self = (Gles2Renderer *)userdata;

	bool expected = false;
	if (self->mWatchdogTriggered.compare_exchange_strong(expected, true)) {
		PDRAW_LOGW("no new frame for %ds",
			   GLES2_RENDERER_WATCHDOG_TIME_S);
	}
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelFlush(RawChannel *channel)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");

	RawSink::lock();

	struct mbuf_raw_video_frame_queue *queue = channel->getQueue();
	if (queue != nullptr) {
		ret = mbuf_raw_video_frame_queue_flush(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
	}
	if (mCurrentFrame != nullptr) {
		int releaseRet = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (releaseRet < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
					-releaseRet);
		mCurrentFrame = nullptr;
	}

	RawSink::unlock();

	ret = channel->flushDone();
	if (ret < 0)
		PDRAW_LOG_ERRNO("channel->flushDone", -ret);
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelSos(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSink::lock();

	RawSink::onChannelSos(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS)
		mPendingTransition = Transition::FADE_FROM_BLACK;

	RawSink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelEos(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSink::lock();

	RawSink::onChannelEos(channel);
	int ret = pomp_timer_clear(mWatchdogTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS)
		mPendingTransition = Transition::FADE_TO_BLACK;

	RawSink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelReconfigure(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSink::lock();

	RawSink::onChannelReconfigure(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE)
		mPendingTransition = Transition::FADE_TO_BLUR;

	RawSink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelTimeout(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSink::lock();

	RawSink::onChannelTimeout(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT)
		mPendingTransition = Transition::FADE_TO_BLACK_AND_WHITE;

	RawSink::unlock();
}


/* Must be called on the loop thread */
void Gles2Renderer::onChannelPhotoTrigger(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSink::lock();

	RawSink::onChannelPhotoTrigger(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER)
		mPendingTransition = Transition::FLASH_THEN_BLACK_AND_WHITE;

	RawSink::unlock();
}


/* Must be called on the loop thread */
int Gles2Renderer::addInputMedia(RawVideoMedia *media)
{
	struct pomp_loop *loop = nullptr;
	struct pomp_evt *evt = nullptr;
	int res = 0;

	if ((mMediaId != 0) && (mMediaId != media->id))
		return -EPERM;
	if (mLastAddedMedia != nullptr)
		return -EBUSY;

	RawSink::lock();

	mFirstFrame = true;

	res = RawSink::addInputMedia(media);
	if (res == -EEXIST) {
		RawSink::unlock();
		return res;
	} else if (res < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("RawSink::addInputMedia", -res);
		return res;
	}

	RawChannel *channel = getInputChannel(media);
	if (channel == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	channel->setKey(this);
	struct mbuf_raw_video_frame_queue_args args = {};
	args.max_frames = 2;
	struct mbuf_raw_video_frame_queue *queue;
	res = mbuf_raw_video_frame_queue_new_with_args(&args, &queue);
	if (res < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args",
				-res);
		return res;
	}
	channel->setQueue(queue);

	res = mbuf_raw_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
		goto error;
	}

	loop = mSession->getLoop();
	if (loop == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("loop not found");
		res = -ENODEV;
		goto error;
	}

	res = pomp_evt_attach_to_loop(evt, loop, &queueEventCb, this);
	if (res < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	mLastAddedMedia = media;
	mCurrentMediaId = mMediaId;

	media->fillMediaInfo(&mMediaInfo);
	/* Deep copy: copy the session metadata */
	mMediaInfoSessionMeta = *mMediaInfo.session_meta;
	mMediaInfo.session_meta = &mMediaInfoSessionMeta;

	RawSink::unlock();

	if (mRendererListener) {
		mRendererListener->onVideoRendererMediaAdded(
			mSession, mRenderer, &mMediaInfo);
	}

	return 0;

error:
	removeQueueFdFromPomp(queue);
	return res;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeQueueFdFromPomp(
	struct mbuf_raw_video_frame_queue *queue)
{
	int ret;
	struct pomp_loop *loop = nullptr;
	struct pomp_evt *evt = nullptr;

	loop = mSession->getLoop();
	if (loop == nullptr) {
		PDRAW_LOGE("loop not found");
		return -ENODEV;
	}

	ret = mbuf_raw_video_frame_queue_get_event(queue, &evt);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -ret);
		return ret;
	}

	ret = pomp_evt_detach_from_loop(evt, loop);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -ret);
		return ret;
	}

	return 0;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeInputMedia(RawVideoMedia *media)
{
	int ret;

	RawSink::lock();

	if (mLastAddedMedia == media) {
		mLastAddedMedia = nullptr;
		mCurrentMediaId = 0;
		if (mRendererListener) {
			mRendererListener->onVideoRendererMediaRemoved(
				mSession, mRenderer, &mMediaInfo);
		}

		if (mCurrentFrame != nullptr) {
			int releaseRet =
				mbuf_raw_video_frame_unref(mCurrentFrame);
			if (releaseRet < 0)
				PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
						-releaseRet);
			mCurrentFrame = nullptr;
		}

		Media::cleanupMediaInfo(&mMediaInfo);
	}


	RawChannel *channel = getInputChannel(media);
	if (channel == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	/* Keep a reference on the queue to destroy it after removing the
	 * input media (avoids deadlocks when trying to push new frames out
	 * of the VideoDecoder whereas the queue is already destroyed) */
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue();

	ret = RawSink::removeInputMedia(media);
	if (ret < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("RawSink::removeInputMedia", -ret);
		return ret;
	}

	RawSink::unlock();

	if (queue != nullptr) {
		ret = removeQueueFdFromPomp(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeQueueFdFromPomp", -ret);
		ret = mbuf_raw_video_frame_queue_flush(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
		ret = mbuf_raw_video_frame_queue_destroy(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-ret);
	}

	return 0;
}


/* Must be called on the loop thread */
int Gles2Renderer::removeInputMedias(void)
{
	int ret, inputMediaCount, i;

	RawSink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		RawVideoMedia *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		ret = removeInputMedia(media);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
			continue;
		}
	}

	mLastAddedMedia = nullptr;
	mCurrentMediaId = 0;
	RawSink::unlock();

	return 0;
}


void Gles2Renderer::idleRenewMedia(void *userdata)
{
	Gles2Renderer *self = reinterpret_cast<Gles2Renderer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	if (self->mLastAddedMedia != nullptr)
		self->removeInputMedia(self->mLastAddedMedia);
	self->mSession->addMediaToRenderer(self->mMediaId, self);
}


/* Called on the rendering thread */
void Gles2Renderer::abortTransition(void)
{
	RawSink::lock();
	mCurrentTransition = Transition::NONE;
	mTransitionStartTime = 0;
	mTransitionHoldTime = 0;
	RawSink::unlock();

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
	if (loadFrame == nullptr)
		return -EINVAL;

	RawSink::lock();

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
			mGles2Video->startTransition(
				GLES2_VIDEO_TRANSITION_FLASH,
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
		    (mCurrentTransition == Transition::FADE_FROM_BLACK) ||
		    (mCurrentTransition ==
		     Transition::FLASH_THEN_BLACK_AND_WHITE))) {
		*loadFrame = true;
	} else {
		*loadFrame = false;
	}
	RawSink::unlock();
	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::loadVideoFrame(struct mbuf_raw_video_frame *mbufFrame,
				  RawVideoMedia::Frame *frame)
{
	int ret;
	unsigned int planeCount = 0;
	const void *planes[VDEF_RAW_MAX_PLANE_COUNT] = {};

	if (vdef_dim_is_null(&mCurrentFrameInfo.info.resolution) ||
	    vdef_dim_is_null(&mCurrentFrameInfo.info.sar)) {
		PDRAW_LOGE("invalid frame dimensions");
		ret = -EINVAL;
		goto out;
	}

	planeCount = vdef_get_raw_frame_plane_count(&mCurrentFrameInfo.format);
	for (unsigned int i = 0; i < planeCount; i++) {
		size_t dummyPlaneLen;
		ret = mbuf_raw_video_frame_get_plane(
			mbufFrame, i, &planes[i], &dummyPlaneLen);
		if (ret < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -ret, i);
			goto out;
		}
	}

	ret = mGles2Video->loadFrame((const uint8_t **)planes,
				     mCurrentFrameInfo.plane_stride,
				     &mCurrentFrameInfo.format,
				     &mCurrentFrameInfo.info);
	if (ret < 0)
		PDRAW_LOG_ERRNO("gles2Video->loadFrame", -ret);

out:
	for (unsigned int i = 0; i < planeCount; i++) {
		if (planes[i] == nullptr)
			continue;
		int err = mbuf_raw_video_frame_release_plane(
			mbufFrame, i, planes[i]);
		if (err < 0)
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_release_plane(%u)",
				-err,
				i);
	}

	mFrameLoaded = (ret == 0);
	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::loadExternalVideoFrame(
	struct mbuf_raw_video_frame *mbufFrame,
	RawVideoMedia::Frame *frame,
	const struct pdraw_media_info *mediaInfo)
{
	int ret;
	struct mbuf_ancillary_data *userData = nullptr;
	size_t frameUserdataLen;
	const void *frameUserdata = nullptr;

	ret = mbuf_raw_video_frame_get_ancillary_data(
		mbufFrame, MBUF_ANCILLARY_KEY_USERDATA_SEI, &userData);
	if (ret == -ENOENT) {
		/* No userdata */
		frameUserdata = nullptr;
		frameUserdataLen = 0;
	} else if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-ret);
		goto out;
	} else {
		frameUserdata = mbuf_ancillary_data_get_buffer(
			userData, &frameUserdataLen);
	}

	if (vdef_dim_is_null(&mCurrentFrameInfo.info.resolution) ||
	    vdef_dim_is_null(&mCurrentFrameInfo.info.sar)) {
		PDRAW_LOGE("invalid frame dimensions");
		ret = -EINVAL;
		goto out;
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mExtLoadFbo));
	GLCHK(glViewport(0, 0, mExtVideoTextureWidth, mExtVideoTextureHeight));

	/* Texture loading callback function */
	ret = mRendererListener->loadVideoTexture(mSession,
						  mRenderer,
						  mExtVideoTextureWidth,
						  mExtVideoTextureHeight,
						  mediaInfo,
						  mbufFrame,
						  frameUserdata,
						  (size_t)frameUserdataLen);

	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));
		GLCHK(glViewport(0, 0, mHmdFboSize, mHmdFboSize));
	} else {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
	}

out:
	if (userData)
		mbuf_ancillary_data_unref(userData);
	mFrameLoaded = (ret == 0);
	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::renderVideoFrame(const struct pdraw_rect *renderPos,
				    struct pdraw_rect *contentPos,
				    Eigen::Matrix4f &viewProjMat)
{
	struct vdef_rect crop = {
		.left = 0,
		.top = 0,
		.width = mCurrentFrameInfo.info.resolution.width,
		.height = mCurrentFrameInfo.info.resolution.height,
	};

	return mGles2Video->renderFrame(renderPos,
					contentPos,
					viewProjMat,
					mCurrentFrameInfo.plane_stride,
					&mCurrentFrameInfo.format,
					&mCurrentFrameInfo.info,
					&crop,
					mCurrentFrameMetadata,
					&mParams);
}


/* Called on the rendering thread */
int Gles2Renderer::renderExternalVideoFrame(const struct pdraw_rect *renderPos,
					    struct pdraw_rect *contentPos,
					    Eigen::Matrix4f &viewProjMat)
{
	struct vdef_frame_info info = mCurrentFrameInfo.info;
	struct vdef_rect crop = {
		.left = 0,
		.top = 0,
		.width = mExtVideoTextureWidth,
		.height = mExtVideoTextureHeight,
	};
	size_t frameStride[3] = {mExtVideoTextureWidth, 0, 0};

	info.sar.width = mCurrentFrameInfo.info.resolution.width *
			 mExtVideoTextureHeight;
	info.sar.height = mCurrentFrameInfo.info.resolution.height *
			  mExtVideoTextureWidth;

	return mGles2Video->renderFrame(renderPos,
					contentPos,
					viewProjMat,
					frameStride,
					&vdef_rgb,
					&info,
					&crop,
					mCurrentFrameMetadata,
					&mParams);
}


int Gles2Renderer::setupExtTexture(const struct vdef_raw_frame *frameInfo,
				   const RawVideoMedia::Frame *frame)
{
	int ret = 0;

	PDRAW_LOGI(
		"external video texture: source=%ux%u (SAR=%u:%u) "
		"DAR=%u:%u textureWidth=%u",
		frameInfo->info.resolution.width,
		frameInfo->info.resolution.height,
		frameInfo->info.sar.width,
		frameInfo->info.sar.height,
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
			ar = (float)frameInfo->info.resolution.width /
			     (float)frameInfo->info.resolution.height;
			/* Take the DAR into account, never decreasing
			 * the dimensions */
			if (dar > ar) {
				mExtVideoTextureWidth =
					(frameInfo->info.resolution.height *
						 mParams.video_texture_dar_width +
					 mParams.video_texture_dar_height / 2) /
					mParams.video_texture_dar_height;
				mExtVideoTextureHeight =
					frameInfo->info.resolution.height;
			} else {
				mExtVideoTextureWidth =
					frameInfo->info.resolution.width;
				mExtVideoTextureHeight =
					(frameInfo->info.resolution.width *
						 mParams.video_texture_dar_height +
					 mParams.video_texture_dar_width / 2) /
					mParams.video_texture_dar_width;
			}
		}
	} else if (mParams.video_texture_width > 0) {
		/* Custom texture width without custom DAR */
		mExtVideoTextureWidth = mParams.video_texture_width;
		mExtVideoTextureHeight =
			(mParams.video_texture_width *
				 frameInfo->info.resolution.height +
			 frameInfo->info.resolution.width / 2) /
			frameInfo->info.resolution.width;

		/* Take the SAR into account */
		mExtVideoTextureHeight =
			(mExtVideoTextureHeight * frameInfo->info.sar.height +
			 frameInfo->info.sar.width / 2) /
			frameInfo->info.sar.width;
	} else {
		/* No custom texture width and no custom DAR */
		float sar = (float)frameInfo->info.sar.width /
			    (float)frameInfo->info.sar.height;
		/* Take the SAR into account, never decreasing the dimensions */
		if (sar < 1.f) {
			mExtVideoTextureWidth =
				frameInfo->info.resolution.width;
			mExtVideoTextureHeight =
				(frameInfo->info.resolution.height *
					 frameInfo->info.sar.height +
				 frameInfo->info.sar.width / 2) /
				frameInfo->info.sar.width;
		} else {
			mExtVideoTextureWidth =
				(frameInfo->info.resolution.width *
					 frameInfo->info.sar.width +
				 frameInfo->info.sar.height / 2) /
				frameInfo->info.sar.height;
			mExtVideoTextureHeight =
				frameInfo->info.resolution.height;
		}
	}

	/* Round up to nearest multiple of 2 */
	mExtVideoTextureWidth = (mExtVideoTextureWidth + 1) & ~1;
	mExtVideoTextureHeight = (mExtVideoTextureHeight + 1) & ~1;

	/* start external Texture */
	if (mExtVideoTextureWidth != 0 && mExtVideoTextureHeight != 0) {
		ret = startExtLoad();
		if (ret < 0) {
			PDRAW_LOG_ERRNO("startExtLoad", -ret);
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
			PDRAW_LOG_ERRNO("stopExtLoad", -ret);
	}

	PDRAW_LOGI("external video texture: size=%ux%u",
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
	int err;
	struct mbuf_raw_video_frame *frame = nullptr;
	int dequeueRet = 0;
	struct pdraw_rect renderPos;
	bool load = false, render = false;
	uint64_t curTime = 0;
	struct timespec ts = {0, 0};
	struct pdraw_media_info *mediaInfoPtr = nullptr;
	InputPort *port = nullptr;
	struct mbuf_raw_video_frame_queue *queue = nullptr;
	Eigen::Matrix4f vpMat, vMat, pMat;
	struct pdraw_rect content;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	RawVideoMedia::Frame *data;

	memset(&content, 0, sizeof(content));

	if (contentPos != nullptr)
		*contentPos = content;

	if ((!mRunning) || (mState != STARTED))
		return 0;

	if ((mWidth == 0) || (mHeight == 0))
		return 0;

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));
		GLCHK(glViewport(0, 0, mHmdFboSize, mHmdFboSize));
		GLCHK(glDisable(GL_DITHER));
		renderPos.x = 0;
		renderPos.y = 0;
		renderPos.width = mHmdFboSize;
		renderPos.height = mHmdFboSize;
	} else {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
		GLCHK(glDisable(GL_DITHER));
		renderPos.x = mX;
		renderPos.y = mY;
		renderPos.width = mWidth;
		renderPos.height = mHeight;
	}

	if (viewMat != nullptr) {
		unsigned int i, j;
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++)
				vMat(i, j) = viewMat[j * 4 + i];
		}
	} else {
		vMat = Eigen::Matrix4f::Identity();
	}
	if (projMat != nullptr) {
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

	RawSink::lock();
	if (mLastAddedMedia == nullptr) {
		/* No current media */
		goto skip_dequeue;
	}

	Media::cleanupMediaInfo(&mMediaInfo);
	mLastAddedMedia->fillMediaInfo(&mMediaInfo);
	/* Deep copy: copy the session metadata */
	mMediaInfoSessionMeta = *mMediaInfo.session_meta;
	mMediaInfo.session_meta = &mMediaInfoSessionMeta;
	mediaInfoPtr = &mMediaInfo;

	port = getInputPort(mLastAddedMedia);
	if (port == nullptr) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("failed to get input port", EPROTO);
		goto skip_dequeue;
	}
	queue = port->channel->getQueue();
	if (queue == nullptr) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("failed to get input queue", EPROTO);
		goto skip_dequeue;
	}

	/* Get a new frame to render */
	dequeueRet = mbuf_raw_video_frame_queue_pop(queue, &frame);
	while ((dequeueRet == 0) && (frame != nullptr)) {
		if (mCurrentFrame != nullptr) {
			int releaseRet =
				mbuf_raw_video_frame_unref(mCurrentFrame);
			if (releaseRet < 0)
				PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
						-releaseRet);
		}
		mCurrentFrame = frame;
		load = true;
		frame = nullptr;
		dequeueRet = mbuf_raw_video_frame_queue_pop(queue, &frame);
		/* We have a new frame */
		bool expected = true;
		if (mWatchdogTriggered.compare_exchange_strong(expected,
							       false)) {
			PDRAW_LOGI("new frame to render");
		}
		int err = pomp_timer_set(mWatchdogTimer,
					 1000 * GLES2_RENDERER_WATCHDOG_TIME_S);
		if (err != 0) {
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
	}
	if ((dequeueRet < 0) && (dequeueRet != -EAGAIN)) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_pop", -dequeueRet);
	}

	if (mCurrentFrame == nullptr) {
		/* No new frame to load */
		goto skip_dequeue;
	}

	if (mCurrentFrameMetadata) {
		vmeta_frame_unref(mCurrentFrameMetadata);
		mCurrentFrameMetadata = nullptr;
	}

	err = mbuf_raw_video_frame_get_ancillary_data(
		mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&ancillaryData);
	if (err < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-err);
		goto skip_render;
	}
	data = (RawVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, NULL);
	if (data == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("invalid ancillary data pointer");
		goto skip_render;
	}
	mCurrentFrameData = *data;
	mbuf_ancillary_data_unref(ancillaryData);

	err = mbuf_raw_video_frame_get_frame_info(mCurrentFrame,
						  &mCurrentFrameInfo);
	if (err < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -err);
		goto skip_render;
	}

	err = mbuf_raw_video_frame_get_metadata(mCurrentFrame,
						&mCurrentFrameMetadata);
	if (err < 0 && err != -ENOENT) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -err);
		goto skip_render;
	}

	if (mFirstFrame) {
		mFirstFrame = false;
		if (mExtLoadVideoTexture) {
			err = setupExtTexture(&mCurrentFrameInfo,
					      &mCurrentFrameData);
			if (err < 0)
				PDRAW_LOG_ERRNO("setupExtTexture", -err);
		}
	}

skip_dequeue:
	if (mGles2Video == nullptr) {
		RawSink::unlock();
		goto skip_render;
	}

	err = time_get_monotonic(&ts);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_get_monotonic", -err);
	err = time_timespec_to_us(&ts, &curTime);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_timespec_to_us", -err);

	err = doTransition(curTime, load, &load);
	if (err < 0) {
		PDRAW_LOG_ERRNO("doTransition", -err);
		goto skip_render;
	}

	if (mCurrentFrame != nullptr) {
		if (mExtLoadVideoTexture) {
			err = loadExternalVideoFrame(mCurrentFrame,
						     &mCurrentFrameData,
						     mediaInfoPtr);
			if (err < 0)
				goto skip_render;
		} else if (load) {
			err = loadVideoFrame(mCurrentFrame, &mCurrentFrameData);
			if (err < 0)
				goto skip_render;
		}
	}

	RawSink::unlock();

	/* Actual rendering */
	if (mFrameLoaded) {
		if (mExtLoadVideoTexture) {
			err = renderExternalVideoFrame(
				&renderPos, &content, vpMat);
			if (err < 0) {
				PDRAW_LOG_ERRNO(
					"gles2Video->"
					"renderExternalVideoFrame",
					-err);
			} else {
				render = true;
			}
		} else {
			err = renderVideoFrame(&renderPos, &content, vpMat);
			if (err < 0) {
				PDRAW_LOG_ERRNO("gles2Video->renderVideoFrame",
						-err);
			} else {
				render = true;
			}
		}
	} else {
		err = mGles2Video->clear(vpMat);
		if (err < 0)
			PDRAW_LOG_ERRNO("gles2Video->clear", -err);
	}
	if (mFrameLoaded != mFrameLoadedLogged) {
		mFrameLoadedLogged = mFrameLoaded;
		PDRAW_LOGI("render state: %s",
			   mFrameLoaded ? "video" : "black");
	}

skip_render:
	/* Overlay rendering callback function */
	if (mRenderVideoOverlay) {
		if (render) {
			struct pdraw_video_frame_extra frameExtra;
			memset(&frameExtra, 0, sizeof(frameExtra));
			frameExtra.play_timestamp =
				mCurrentFrameData.playTimestamp;
			if (mGles2Video) {
				mGles2Video->getHistograms(
					frameExtra.histogram,
					frameExtra.histogram_len);
			}
			mRendererListener->renderVideoOverlay(
				mSession,
				mRenderer,
				&renderPos,
				&content,
				vMat.data(),
				pMat.data(),
				mediaInfoPtr,
				mCurrentFrameMetadata,
				&frameExtra);
		} else {
			mRendererListener->renderVideoOverlay(mSession,
							      mRenderer,
							      &renderPos,
							      &content,
							      vMat.data(),
							      pMat.data(),
							      mediaInfoPtr,
							      nullptr,
							      nullptr);
		}
	}

	/* HMD rendering */
	if (mParams.enable_hmd_distortion_correction) {
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
		GLCHK(glViewport(mX, mY, mWidth, mHeight));

		if (mGles2Hmd) {
			err = mGles2Hmd->renderHmd(
				mHmdFboTexture, mHmdFboSize, mHmdFboSize);
			if (err < 0)
				PDRAW_LOG_ERRNO("gles2Hmd->renderHmd", -err);
		}
	}

	if (contentPos != nullptr)
		*contentPos = content;

	return 0;
}


/* Called on the rendering thread */
int Gles2Renderer::startHmd(void)
{
	int ret = 0;
	GLenum gle;

	int err = stopHmd();
	if (err < 0)
		PDRAW_LOG_ERRNO("stopHmd", -err);

	mHmdFboSize = (mWidth / 2 > mHeight) ? mWidth / 2 : mHeight;
	GLCHK();

	GLCHK(glGenFramebuffers(1, &mHmdFbo));
	if (mHmdFbo <= 0) {
		PDRAW_LOGE("failed to create framebuffer");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHmdFbo));

	GLCHK(glGenTextures(1, &mHmdFboTexture));
	if (mHmdFboTexture <= 0) {
		PDRAW_LOGE("failed to create texture");
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
			   nullptr));

	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCHK(glGenRenderbuffers(1, &mHmdFboRenderBuffer));
	if (mHmdFboRenderBuffer <= 0) {
		PDRAW_LOGE("failed to create render buffer");
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
		PDRAW_LOGE("invalid framebuffer status");
		ret = -EPROTO;
		goto out;
	}

	GLCHK(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
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
	if (mGles2Hmd == nullptr) {
		PDRAW_LOGE("failed to create Gles2Hmd context");
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

	if (mGles2Hmd != nullptr) {
		delete mGles2Hmd;
		mGles2Hmd = nullptr;
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
		PDRAW_LOG_ERRNO("stopExtLoad", -err);

	GLCHK();

	GLCHK(glGenFramebuffers(1, &mExtLoadFbo));
	if (mExtLoadFbo <= 0) {
		PDRAW_LOGE("failed to create framebuffer");
		ret = -EPROTO;
		goto out;
	}
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mExtLoadFbo));

	GLCHK(glGenTextures(1, &mExtLoadFboTexture));
	if (mExtLoadFboTexture <= 0) {
		PDRAW_LOGE("failed to create texture");
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
			   nullptr));

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
		PDRAW_LOGE("invalid framebuffer status");
		ret = -EPROTO;
		goto out;
	}

	GLCHK(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
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
int Gles2Renderer::resize(const struct pdraw_rect *renderPos)
{
	int ret;

	if (renderPos == nullptr)
		return -EINVAL;

	mX = renderPos->x;
	mY = renderPos->y;
	mWidth = renderPos->width;
	mHeight = renderPos->height;

	GLCHK();
	GLCHK(glViewport(mX, mY, mWidth, mHeight));

	if (mParams.enable_hmd_distortion_correction) {
		ret = startHmd();
		if (ret < 0)
			PDRAW_LOG_ERRNO("startHmd", -ret);
	} else {
		ret = stopHmd();
		if (ret < 0)
			PDRAW_LOG_ERRNO("stopHmd", -ret);
	}

	return ret;
}


/* Called on the rendering thread */
int Gles2Renderer::setMediaId(unsigned int mediaId)
{
	if (mediaId == mMediaId)
		return 0;

	mMediaId = mediaId;
	int ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleRenewMedia, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return 0;
}


/* Called on the rendering thread */
unsigned int Gles2Renderer::getMediaId(void)
{
	return mCurrentMediaId;
}


/* Called on the rendering thread */
int Gles2Renderer::setParams(const struct pdraw_video_renderer_params *params)
{
	int ret;

	if (params == nullptr)
		return -EINVAL;

	mParams = *params;

	if (mParams.video_scale_factor == 0.f)
		mParams.video_scale_factor = 1.f;

	if (mParams.enable_hmd_distortion_correction) {
		ret = startHmd();
		if (ret < 0)
			PDRAW_LOG_ERRNO("startHmd", -ret);
	} else {
		ret = stopHmd();
		if (ret < 0)
			PDRAW_LOG_ERRNO("stopHmd", -ret);
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
