/**
 * Parrot Drones Audio and Video Vector library
 * OpenGL video renderer
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

#include "pdraw_renderer_video_gl.hpp"
#include "pdraw_session.hpp"
#include "pdraw_settings.hpp"

#ifdef PDRAW_USE_GL

#	include <string.h>
#	include <time.h>
#	include <unistd.h>

#	include <media-buffers/mbuf_ancillary_data.h>
#	include <video-streaming/vstrm.h>

namespace Pdraw {

#	define GL_RENDERER_ANCILLARY_DATA_KEY_INPUT_TIME                      \
		"pdraw.gl_video_renderer.input_time"
#	define GL_RENDERER_QUEUE_MAX_FRAMES 5

#	define GL_RENDERER_DEFAULT_DELAY_MS 33
#	define GL_RENDERER_FADE_FROM_BLACK_DURATION 500000
#	define GL_RENDERER_FADE_TO_BLACK_DURATION 500000
#	define GL_RENDERER_FADE_TO_BLACK_AND_WHITE_DURATION 2000000
#	define GL_RENDERER_FADE_TO_BLACK_AND_WHITE_HOLD 50000
#	define GL_RENDERER_FADE_TO_BLUR_DURATION 2000000
#	define GL_RENDERER_FADE_TO_BLUR_HOLD 500000
#	define GL_RENDERER_FLASH_THEN_BLACK_AND_WHITE_DURATION 200000
#	define GL_RENDERER_FLASH_THEN_BLACK_AND_WHITE_HOLD 200000
#	define GL_RENDERER_WATCHDOG_TIME_S 2
#	define GL_RENDERER_VIDEO_PRES_STATS_TIME_MS 200
#	define GL_RENDERER_SCHED_ADAPTIVE_EPSILON_PERCENT 5
#	define GL_RENDERER_SCHED_ADAPTIVE_INIT_BUF_DELAY_PERCENT 50

#	define NB_SUPPORTED_FORMATS 8
static struct vdef_raw_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_i420;
	supportedFormats[1] = vdef_nv12;
	supportedFormats[2] = vdef_nv21;
	supportedFormats[3] = vdef_i420_10_16le;
	supportedFormats[4] = vdef_nv12_10_16le_high;
	supportedFormats[5] = vdef_gray;
	supportedFormats[6] = vdef_raw16;
	supportedFormats[7] = vdef_raw32;
}


/* Called on the rendering thread */
GlVideoRenderer::GlVideoRenderer(
	Session *session,
	Element::Listener *listener,
	VideoRendererWrapper *wrapper,
	IPdraw::IVideoRenderer::Listener *rndListener,
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params) :
		VideoRenderer(session,
			      listener,
			      wrapper,
			      rndListener,
			      Media::Type::RAW_VIDEO,
			      nullptr,
			      0,
			      mediaId,
			      renderPos,
			      params)
{
	int ret;

	Element::setClassName(__func__);
	mTargetPrimaryMediaId = mediaId;
	mPrimaryMediaId = 0;
	mRunning = false;
	mCurrentFrame = nullptr;
	mCurrentFrameData = {};
	mCurrentFrameInfo = {};
	mCurrentFrameMetadata = nullptr;
	mPrimaryMedia = nullptr;
	mMediaInfo = {};
	mMediaInfoSessionMeta = {};
	mTimer = nullptr;
	mGlVideo = nullptr;
	mGlVideoFirstTexUnit = 0;
	mDefaultFbo = 0;
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
	mLastLoadTimestamp = UINT64_MAX;
	mLastRenderTimestamp = UINT64_MAX;
	mAvgRenderRate = 0.f;
	mLastFrameTimestamp = UINT64_MAX;
	mRenderReadyScheduled = false;
	mPendingRestart = false;
	mVideoPresStatsTimer = nullptr;
	mSchedLastInputTimestamp = UINT64_MAX;
	mSchedLastOutputTimestamp = UINT64_MAX;
	mSchedInitialBuffering = false;
	mFrameLoadedLogged = false;
	mWatchdogTimer = nullptr;
	mWatchdogTriggered = false;
	mEos = false;
	mAncillaryKey = "";
	mPendingResize = false;
	mMbStatusOverlay = false;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setRawVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	if (mRendererListener != nullptr) {
		ret = mRendererListener->loadVideoTexture(
			nullptr, nullptr, 0, 0, nullptr, nullptr, nullptr, 0);
		if (ret != -ENOSYS)
			mExtLoadVideoTexture = true;
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
		ret = setup(renderPos, params);
		if (ret != 0)
			return;
	}

	/* Post a message on the loop thread */
	setStateAsyncNotify(CREATED);
	return;
}


/* Must be called on the loop thread */
GlVideoRenderer::~GlVideoRenderer(void)
{
	int ret;

	if (mState == STARTED)
		PDRAW_LOGW("renderer is still running");

	mExtLoadVideoTexture = false;
	mRenderVideoOverlay = false;

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
	if (mVideoPresStatsTimer != nullptr) {
		ret = pomp_timer_clear(mVideoPresStatsTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mVideoPresStatsTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
		mVideoPresStatsTimer = nullptr;
	}

	if (mGlVideo != nullptr) {
		delete mGlVideo;
		mGlVideo = nullptr;
	}
}


/* Called on the rendering thread */
int GlVideoRenderer::setup(const struct pdraw_rect *renderPos,
			   const struct pdraw_video_renderer_params *params)
{
	int ret = 0;
	char *key = nullptr;

	if (params == nullptr)
		return -EINVAL;
	if (renderPos == nullptr)
		return -EINVAL;

	if ((mState != INVALID) && (mState != CREATED)) {
		PDRAW_LOGE("invalid state");
		return -EPROTO;
	}

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	ret = mbuf_ancillary_data_build_key(
		GL_RENDERER_ANCILLARY_DATA_KEY_INPUT_TIME,
		(uintptr_t)this,
		&key);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_build_key", -ret);
		goto out;
	}
	mAncillaryKey = key;

	if (renderPos->width != 0 && renderPos->height != 0) {
		ret = resize(renderPos);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("resize", -ret);
			goto out;
		}
	}

	ret = setParams(params, true);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("setParams", -ret);
		goto out;
	}

	mGlVideo = new GlVideo(mSession,
			       mDefaultFbo,
			       mGlVideoFirstTexUnit,
			       mParams.enable_simplified_rendering);
	if (mGlVideo == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("failed to create GlVideo", -ret);
		goto out;
	}
	mGlVideo->setExtTexture(mExtLoadFboTexture);

out:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	free(key);
	return ret;
}


/* Called on the rendering thread */
int GlVideoRenderer::start(void)
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
void GlVideoRenderer::idleStart(void *renderer)
{
	GlVideoRenderer *self = reinterpret_cast<GlVideoRenderer *>(renderer);
	ULOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	int err;

	if (self->mState != STARTING) {
		PDRAW_LOGE("renderer is not starting");
		return;
	}

	if (self->mTimer == nullptr) {
		self->mTimer = pomp_timer_new(
			self->mSession->getLoop(), timerCb, self);
		if (self->mTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}
	if (self->mWatchdogTimer == nullptr) {
		self->mWatchdogTimer = pomp_timer_new(
			self->mSession->getLoop(), watchdogTimerCb, self);
		if (self->mWatchdogTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}
	if (self->mVideoPresStatsTimer == nullptr) {
		self->mVideoPresStatsTimer = pomp_timer_new(
			self->mSession->getLoop(), videoPresStatsTimerCb, self);
		if (self->mVideoPresStatsTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}

	pthread_mutex_lock(&self->mListenerMutex);
	if ((self->mRendererListener != nullptr) &&
	    (self->mParams.scheduling_mode !=
	     PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE)) {
		err = pomp_timer_set(self->mTimer,
				     GL_RENDERER_DEFAULT_DELAY_MS);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}
	pthread_mutex_unlock(&self->mListenerMutex);

	/* Post a message on the loop thread */
	self->setState(STARTED);
	return;

error:
	if (self->mTimer != nullptr) {
		err = pomp_timer_clear(self->mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mTimer = nullptr;
	}
	if (self->mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mWatchdogTimer = nullptr;
	}
	if (self->mVideoPresStatsTimer != nullptr) {
		err = pomp_timer_clear(self->mVideoPresStatsTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mVideoPresStatsTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mVideoPresStatsTimer = nullptr;
	}
}


/* Called on the rendering thread */
int GlVideoRenderer::stop(void)
{
	int err;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;

	/* Post a message on the loop thread */
	setStateAsyncNotify(STOPPING);

	/* Make sure listener functions will no longer be called.
	 * Note: remaining medias will be removed in the completeStop function
	 * on the loop thread;
	 * the IVideoRenderer::Listener::onVideoRendererMediaRemoved function
	 * will not be called. */
	removeRendererListener();

	mRunning = false;

	/* Flush the remaining frames */
	Sink::lock();
	struct mbuf_raw_video_frame_queue *queue = getPrimaryMediaQueue();
	if (queue != nullptr) {
		err = mbuf_raw_video_frame_queue_flush(queue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-err);
		}
	}
	if (mCurrentFrame != nullptr) {
		err = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}
	Sink::unlock();

	mExtLoadVideoTexture = false;
	mRenderVideoOverlay = false;

	if (mGlVideo != nullptr) {
		delete mGlVideo;
		mGlVideo = nullptr;
	}

	err = stopExtLoad();
	if (err < 0)
		PDRAW_LOG_ERRNO("stopExtLoad", -err);

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	/* Post a message on the loop thread */
	asyncCompleteStop();

	return 0;
}


/* Called on the loop thread */
void GlVideoRenderer::completeStop(void)
{
	int ret;

	if (mState == STOPPED)
		return;

	if (mTimer != nullptr) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	}
	if (mWatchdogTimer != nullptr) {
		ret = pomp_timer_clear(mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	}
	if (mVideoPresStatsTimer != nullptr) {
		ret = pomp_timer_clear(mVideoPresStatsTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	}

	ret = removeInputMedias();
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedias", -ret);

	setState(STOPPED);
}


/* Called on the loop thread */
void GlVideoRenderer::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	GlVideoRenderer *self = (GlVideoRenderer *)userdata;
	int err = 0;
	uint32_t delayMs;
	uint64_t delayUs = 0;
	bool processAnyway = false;

	if (self == nullptr) {
		PDRAW_LOGE("invalid renderer pointer");
		return;
	}

	if ((!self->mRunning) || (self->mState != STARTED))
		return;

	self->Sink::lock();

	if (self->mPrimaryMedia == nullptr) {
		/* No current media */
		self->Sink::unlock();
		return;
	}

	struct mbuf_raw_video_frame_queue *queue = self->getPrimaryMediaQueue();
	if (queue == nullptr) {
		self->Sink::unlock();
		return;
	}

	uint64_t curTime = 0;
	struct timespec ts = {0, 0};
	err = time_get_monotonic(&ts);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_get_monotonic", -err);
	err = time_timespec_to_us(&ts, &curTime);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_timespec_to_us", -err);

	err = self->getNextFrameDelay(queue,
				      curTime,
				      false,
				      nullptr,
				      &processAnyway,
				      &delayUs,
				      nullptr,
				      nullptr,
				      nullptr,
				      0);
	if (err < 0) {
		PDRAW_LOG_ERRNO("getNextFrameDelay", -err);
		self->Sink::unlock();
		return;
	}

	self->Sink::unlock();

	bool setRenderReadyScheduled = false;
	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mRendererListener &&
	    (!self->mRenderReadyScheduled || processAnyway)) {
		delayMs = (delayUs + 500) / 1000;
		if (delayMs == 0) {
			/* Signal "render ready" now and schedule a renderer
			 * keepalive timer */
			self->mRendererListener->onVideoRenderReady(
				self->mSession, self->mRenderer);
			delayMs = self->getPrimaryMediaFrameIntervalMs();
		} else {
			/* Only true when the timer is set for a delayed frame
			 * and not the renderer keepalive timer */
			setRenderReadyScheduled = true;
		}
		if (self->mParams.scheduling_mode !=
		    PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE) {
			err = pomp_timer_set(self->mTimer, delayMs);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
			else if (setRenderReadyScheduled)
				self->mRenderReadyScheduled = true;
		}
	}
	pthread_mutex_unlock(&self->mListenerMutex);
}


/* Called on the loop thread */
void GlVideoRenderer::timerCb(struct pomp_timer *timer, void *userdata)
{
	GlVideoRenderer *self = (GlVideoRenderer *)userdata;
	int err;
	bool sinkLocked = false;
	struct mbuf_raw_video_frame_queue *queue = nullptr;
	struct timespec ts = {0, 0};
	uint32_t delayMs;
	uint64_t curTime = 0;
	uint64_t delayUs = 0;
	bool setRenderReadyScheduled = false;

	if (self == nullptr) {
		PDRAW_LOGE("invalid renderer pointer");
		return;
	}

	if ((!self->mRunning) || (self->mState != STARTED))
		goto out;

	self->Sink::lock();
	sinkLocked = true;

	if (self->mPrimaryMedia == nullptr) {
		/* No current media */
		goto out;
	}

	queue = self->getPrimaryMediaQueue();
	if (queue == nullptr)
		goto out;

	err = time_get_monotonic(&ts);
	if (err < 0) {
		PDRAW_LOG_ERRNO("time_get_monotonic", -err);
		goto out;
	}
	err = time_timespec_to_us(&ts, &curTime);
	if (err < 0) {
		PDRAW_LOG_ERRNO("time_timespec_to_us", -err);
		goto out;
	}

	err = self->getNextFrameDelay(queue,
				      curTime,
				      false,
				      nullptr,
				      nullptr,
				      &delayUs,
				      nullptr,
				      nullptr,
				      nullptr,
				      0);
	if ((err < 0) && (err != -EAGAIN)) {
		PDRAW_LOG_ERRNO("getNextFrameDelay", -err);
		goto out;
	}

	self->Sink::unlock();
	sinkLocked = false;

	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mRendererListener) {
		delayMs = (delayUs + 500) / 1000;
		if (delayMs > 0) {
			/* Only true when the timer is set for a delayed frame
			 * and not the renderer keepalive timer */
			setRenderReadyScheduled = true;
		}
		if ((err == -EAGAIN) || (delayMs == 0)) {
			self->mRendererListener->onVideoRenderReady(
				self->mSession, self->mRenderer);
			delayMs = self->getPrimaryMediaFrameIntervalMs();
		}
		if (self->mParams.scheduling_mode !=
		    PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE) {
			err = pomp_timer_set(self->mTimer, delayMs);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
			else if (setRenderReadyScheduled)
				self->mRenderReadyScheduled = true;
		}
	}
	pthread_mutex_unlock(&self->mListenerMutex);

out:
	if (!setRenderReadyScheduled)
		self->mRenderReadyScheduled = false;
	if (sinkLocked)
		self->Sink::unlock();
}


/* Called on the loop thread */
void GlVideoRenderer::watchdogTimerCb(struct pomp_timer *timer, void *userdata)
{
	GlVideoRenderer *self = (GlVideoRenderer *)userdata;

	if ((!self->mRunning) || (self->mState != STARTED))
		return;

	bool expected = false;
	if (self->mWatchdogTriggered.compare_exchange_strong(expected, true)) {
		PDRAW_LOGW("no new frame for %ds", GL_RENDERER_WATCHDOG_TIME_S);
	}
}


/* Called on the loop thread */
void GlVideoRenderer::videoPresStatsTimerCb(struct pomp_timer *timer,
					    void *userdata)
{
	GlVideoRenderer *self = (GlVideoRenderer *)userdata;

	if ((!self->mRunning) || (self->mState != STARTED))
		return;

	self->Sink::lock();
	if (self->mPrimaryMedia == nullptr) {
		/* No current media */
		self->Sink::unlock();
		return;
	}

	Channel *channel = self->getInputChannel(self->mPrimaryMedia);
	if (channel == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("failed to get input port", EPROTO);
		return;
	}

	int err = channel->sendVideoPresStats(&self->mVideoPresStats);
	if (err < 0)
		PDRAW_LOG_ERRNO("channel->sendVideoPresStats", -err);

	self->Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelFlush(Channel *channel)
{
	int ret;

	RawVideoChannel *c = dynamic_cast<RawVideoChannel *>(channel);
	if (c == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");

	Sink::lock();

	struct mbuf_raw_video_frame_queue *queue = c->getQueue(this);
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

	Sink::unlock();

	ret = c->asyncFlushDone();
	if (ret < 0)
		PDRAW_LOG_ERRNO("Channel::asyncFlushDone", -ret);
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelSos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mEos = false;

	Sink::lock();

	Sink::onChannelSos(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS)
		mPendingTransition = Transition::FADE_FROM_BLACK;

	Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelEos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mEos = true;

	Sink::lock();

	Sink::onChannelEos(channel);
	int ret = pomp_timer_clear(mWatchdogTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	ret = pomp_timer_clear(mVideoPresStatsTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS)
		mPendingTransition = Transition::FADE_TO_BLACK;

	Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelReconfigure(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();

	Sink::onChannelReconfigure(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE)
		mPendingTransition = Transition::FADE_TO_BLUR;

	Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelResolutionChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();

	/* Low priority transition, do not trigger transition if another
	 * transition is ongoing */
	Sink::onChannelResolutionChange(channel);
	if (mParams.enable_transition_flags &
		    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT &&
	    mCurrentTransition == Transition::NONE)
		mPendingTransition = Transition::FADE_TO_BLACK_AND_WHITE;

	Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelFramerateChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();

	/* Low priority transition, do not trigger transition if another
	 * transition is ongoing */
	Sink::onChannelFramerateChange(channel);
	if (mParams.enable_transition_flags &
		    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT &&
	    mCurrentTransition == Transition::NONE)
		mPendingTransition = Transition::FADE_TO_BLACK_AND_WHITE;

	Sink::unlock();
}


/* Must be called on the loop thread */
void GlVideoRenderer::onChannelTimeout(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
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
void GlVideoRenderer::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	Sink::onChannelPhotoTrigger(channel);
	if (mParams.enable_transition_flags &
	    PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER)
		mPendingTransition = Transition::FLASH_THEN_BLACK_AND_WHITE;

	Sink::unlock();
}


bool GlVideoRenderer::queueFilter(struct mbuf_raw_video_frame *frame,
				  void *userdata)
{
	int err;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};

	GlVideoRenderer *self = (GlVideoRenderer *)userdata;

	/* Set the input time ancillary data to the frame */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	err = mbuf_raw_video_frame_add_ancillary_buffer(
		frame, self->mAncillaryKey.c_str(), &ts_us, sizeof(ts_us));
	if (err < 0)
		ULOGW_ERRNO(-err, "mbuf_raw_video_frame_add_ancillary_buffer");

	return true;
}


uint64_t GlVideoRenderer::getFrameU64(struct mbuf_raw_video_frame *frame,
				      const char *key)
{
	int err;
	struct mbuf_ancillary_data *data;
	uint64_t val = 0;
	const void *raw_data;
	size_t len;

	err = mbuf_raw_video_frame_get_ancillary_data(frame, key, &data);
	if (err < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(val))
		goto out;
	memcpy(&val, raw_data, sizeof(val));

out:
	mbuf_ancillary_data_unref(data);
	return val;
}


unsigned int GlVideoRenderer::getPrimaryMediaFrameIntervalMs(void)
{
	if (vdef_frac_is_null(&mMediaInfo.video.raw.info.framerate))
		return GL_RENDERER_DEFAULT_DELAY_MS;

	return (1000 * mMediaInfo.video.raw.info.framerate.den +
		mMediaInfo.video.raw.info.framerate.num / 2) /
	       mMediaInfo.video.raw.info.framerate.num;
}


struct mbuf_raw_video_frame_queue *GlVideoRenderer::getPrimaryMediaQueue(void)
{
	Sink::lock();
	if (mPrimaryMedia == nullptr) {
		Sink::unlock();
		return nullptr;
	}
	RawVideoChannel *channel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(mPrimaryMedia));
	if (channel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
		Sink::unlock();
		return nullptr;
	}
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		PDRAW_LOGE("failed to get input queue");
		Sink::unlock();
		return nullptr;
	}
	Sink::unlock();
	return queue;
}


/* Must be called on the loop thread */
int GlVideoRenderer::addInputMedia(Media *media)
{
	struct pomp_loop *loop = nullptr;
	struct pomp_evt *evt = nullptr;
	struct pdraw_media_info mediaInfoCopy = {};
	struct vmeta_session sessionMetaCopy = {};
	int res = 0;

	/* Only accept raw video media */
	RawVideoMedia *m = dynamic_cast<RawVideoMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("unsupported input media");
		return -ENOSYS;
	}

	if ((mTargetPrimaryMediaId != 0) && (mTargetPrimaryMediaId != m->id))
		return -EPERM;
	if (mPrimaryMedia != nullptr)
		return -EBUSY;
	if ((!mRunning) || (mState != STARTED))
		return -EAGAIN;

	Sink::lock();

	mFirstFrame = true;

	/* Reset the initial buffering */
	mSchedLastInputTimestamp = UINT64_MAX;
	mSchedLastOutputTimestamp = UINT64_MAX;
	mSchedInitialBuffering = true;
	PDRAW_LOGD("INITIAL BUFFERING: start");

	res = Sink::addInputMedia(m);
	if (res == -EEXIST) {
		Sink::unlock();
		return res;
	} else if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		return res;
	}

	RawVideoChannel *channel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(m));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	struct mbuf_raw_video_frame_queue_args args = {};
	args.filter = &queueFilter;
	args.filter_userdata = this;
	args.max_frames = GL_RENDERER_QUEUE_MAX_FRAMES;
	struct mbuf_raw_video_frame_queue *queue;
	res = mbuf_raw_video_frame_queue_new_with_args(&args, &queue);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args",
				-res);
		return res;
	}
	channel->setQueue(this, queue);

	res = mbuf_raw_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
		goto error;
	}

	loop = mSession->getLoop();
	if (loop == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("loop not found");
		res = -ENODEV;
		goto error;
	}

	res = pomp_evt_attach_to_loop(evt, loop, &queueEventCb, this);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	mPrimaryMedia = m;
	mPrimaryMediaId = mTargetPrimaryMediaId;

	m->fillMediaInfo(&mMediaInfo);
	/* Deep copy: copy the session metadata */
	mMediaInfoSessionMeta = *mMediaInfo.video.session_meta;
	mMediaInfo.video.session_meta = &mMediaInfoSessionMeta;

	/* Another deep copy only used by the listener (must be unlocked). */
	m->fillMediaInfo(&mediaInfoCopy);
	sessionMetaCopy = *mediaInfoCopy.video.session_meta;
	mediaInfoCopy.video.session_meta = &sessionMetaCopy;
	/* TODO: discuss about whether the onVideoRendererMediaAdded listener
	 * might be called with Sink mutex locked to avoid local copy. */

	Sink::unlock();

	pthread_mutex_lock(&mListenerMutex);
	if (mRendererListener) {
		mRendererListener->onVideoRendererMediaAdded(
			mSession, mRenderer, &mediaInfoCopy);
	}
	pthread_mutex_unlock(&mListenerMutex);

	Media::cleanupMediaInfo(&mediaInfoCopy);

	return 0;

error:
	removeQueueFdFromPomp(queue);
	return res;
}


/* Must be called on the loop thread */
int GlVideoRenderer::removeQueueFdFromPomp(
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
int GlVideoRenderer::removeInputMedia(Media *media)
{
	int ret;

	Sink::lock();

	if (mPrimaryMedia == media) {
		mPrimaryMedia = nullptr;
		mPrimaryMediaId = 0;
		pthread_mutex_lock(&mListenerMutex);
		if (mRendererListener) {
			mRendererListener->onVideoRendererMediaRemoved(
				mSession,
				mRenderer,
				&mMediaInfo,
				mPendingRestart);
		}
		pthread_mutex_unlock(&mListenerMutex);

		if (mCurrentFrame != nullptr) {
			int releaseRet =
				mbuf_raw_video_frame_unref(mCurrentFrame);
			if (releaseRet < 0)
				PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
						-releaseRet);
			mCurrentFrame = nullptr;
		}

		Media::cleanupMediaInfo(&mMediaInfo);
		mPendingRestart = false;
	}


	RawVideoChannel *channel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	/* Keep a reference on the queue to destroy it after removing the
	 * input media (avoids deadlocks when trying to push new frames out
	 * of the VideoDecoder whereas the queue is already destroyed) */
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);

	if (queue != nullptr) {
		/* Disconnect and flush queue before removing input media, as
		 * all memories from a pool owned by the upstream element can be
		 * released once the channel is unlinked */
		ret = removeQueueFdFromPomp(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeQueueFdFromPomp", -ret);
		ret = mbuf_raw_video_frame_queue_flush(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
	}

	ret = Sink::removeInputMedia(media);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -ret);
		return ret;
	}

	Sink::unlock();

	if (queue != nullptr) {
		ret = mbuf_raw_video_frame_queue_destroy(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-ret);
	}

	return 0;
}


/* Must be called on the loop thread */
int GlVideoRenderer::removeInputMedias(void)
{
	int ret, inputMediaCount, i;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		RawVideoMedia *media =
			dynamic_cast<RawVideoMedia *>(getInputMedia(i));
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

	mPrimaryMedia = nullptr;
	mPrimaryMediaId = 0;
	Sink::unlock();

	return 0;
}


void GlVideoRenderer::idleRenewMedia(void *userdata)
{
	GlVideoRenderer *self = reinterpret_cast<GlVideoRenderer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	if (self->mPrimaryMedia != nullptr)
		self->removeInputMedia(self->mPrimaryMedia);
	self->mSession->addMediaToVideoRenderer(self->mTargetPrimaryMediaId,
						self);
}


/* Called on the rendering thread */
void GlVideoRenderer::abortTransition(void)
{
	Sink::lock();
	mCurrentTransition = Transition::NONE;
	mTransitionStartTime = 0;
	mTransitionHoldTime = 0;
	Sink::unlock();

	if (mGlVideo) {
		mGlVideo->setSatCoef(1.0f);
		mGlVideo->setLightCoef(1.0f);
		mGlVideo->setDarkCoef(1.0f);
		mGlVideo->abortTransition();
	}
}


/* Called on the rendering thread */
int GlVideoRenderer::doTransition(uint64_t timestamp,
				  bool frameReady,
				  bool *loadFrame)
{
	if (timestamp == 0)
		return -EINVAL;
	if (loadFrame == nullptr)
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
		mTransitionHoldTime = GL_RENDERER_FADE_FROM_BLACK_DURATION;
		if (mGlVideo) {
			mGlVideo->startTransition(
				GL_VIDEO_TRANSITION_FADE_FROM_BLACK,
				GL_RENDERER_FADE_FROM_BLACK_DURATION,
				false);
		}
		break;
	case Transition::FADE_TO_BLACK:
		mTransitionHoldTime = GL_RENDERER_FADE_TO_BLACK_DURATION;
		if (mGlVideo) {
			mGlVideo->startTransition(
				GL_VIDEO_TRANSITION_FADE_TO_BLACK,
				GL_RENDERER_FADE_TO_BLACK_DURATION,
				true);
		}
		break;
	case Transition::FADE_TO_BLACK_AND_WHITE:
		mTransitionHoldTime = GL_RENDERER_FADE_TO_BLACK_AND_WHITE_HOLD;
		if (mGlVideo) {
			mGlVideo->startTransition(
				GL_VIDEO_TRANSITION_FADE_TO_BLACK_AND_WHITE,
				GL_RENDERER_FADE_TO_BLACK_AND_WHITE_DURATION,
				true);
		}
		break;
	case Transition::FADE_TO_BLUR:
		mTransitionHoldTime = GL_RENDERER_FADE_TO_BLUR_HOLD;
		if (mGlVideo) {
			mGlVideo->startTransition(
				GL_VIDEO_TRANSITION_FADE_TO_BLUR,
				GL_RENDERER_FADE_TO_BLUR_DURATION,
				true);
		}
		break;
	case Transition::FLASH_THEN_BLACK_AND_WHITE:
		mTransitionHoldTime =
			GL_RENDERER_FLASH_THEN_BLACK_AND_WHITE_HOLD;
		if (mGlVideo) {
			mGlVideo->startTransition(
				GL_VIDEO_TRANSITION_FLASH,
				GL_RENDERER_FLASH_THEN_BLACK_AND_WHITE_DURATION,
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
	Sink::unlock();
	return 0;
}


void GlVideoRenderer::setNormalization(void)
{
	int err;
	float minVal, maxVal, contrast, brightness;

	if (!mParams.enable_auto_normalization)
		goto reset;

	/* Only the normalization of thermal video with available min/max
	 * values is supported */

	if (mMediaInfoSessionMeta.camera_spectrum !=
	    VMETA_CAMERA_SPECTRUM_THERMAL)
		goto reset;

	if (mCurrentFrameMetadata == nullptr ||
	    mCurrentFrameMetadata->type != VMETA_FRAME_TYPE_PROTO)
		goto reset;

	const Vmeta__TimedMetadata *tm;
	err = vmeta_frame_proto_get_unpacked(mCurrentFrameMetadata, &tm);
	if (err < 0)
		goto reset;

	if (tm->thermal == nullptr || tm->thermal->min == nullptr ||
	    tm->thermal->max == nullptr) {
		vmeta_frame_proto_release_unpacked(mCurrentFrameMetadata, tm);
		goto reset;
	}

	minVal = (float)tm->thermal->min->value;
	maxVal = (float)tm->thermal->max->value;
	if (minVal >= maxVal) {
		vmeta_frame_proto_release_unpacked(mCurrentFrameMetadata, tm);
		goto reset;
	}

	contrast = (1 << mCurrentFrameInfo.format.pix_size) / (maxVal - minVal);
	brightness = -minVal / (1 << mCurrentFrameInfo.format.pix_size);

	mGlVideo->setBrightnessCoef(brightness);
	mGlVideo->setContrastCoef(contrast);
	vmeta_frame_proto_release_unpacked(mCurrentFrameMetadata, tm);
	return;

reset:
	mGlVideo->setBrightnessCoef(0.0f);
	mGlVideo->setContrastCoef(1.0f);
}


/* Called on the rendering thread */
int GlVideoRenderer::loadVideoFrame(struct mbuf_raw_video_frame *frame)
{
	int ret;
	unsigned int planeCount = 0;
	const void *planes[VDEF_RAW_MAX_PLANE_COUNT] = {};
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	uint8_t *mbStatus = nullptr;

	if (vdef_dim_is_null(&mCurrentFrameInfo.info.resolution) ||
	    vdef_dim_is_null(&mCurrentFrameInfo.info.sar)) {
		PDRAW_LOGE("invalid frame dimensions");
		ret = -EINVAL;
		goto out;
	}

	if (mMbStatusOverlay) {
		ret = mbuf_raw_video_frame_get_ancillary_data(
			frame, VSTRM_ANCILLARY_KEY_MB_STATUS, &ancillaryData);
		if (ret == 0) {
			mbStatus = (uint8_t *)mbuf_ancillary_data_get_buffer(
				ancillaryData, nullptr);
		}
	}

	planeCount = vdef_get_raw_frame_plane_count(&mCurrentFrameInfo.format);
	for (unsigned int i = 0; i < planeCount; i++) {
		size_t dummyPlaneLen;
		ret = mbuf_raw_video_frame_get_plane(
			frame, i, &planes[i], &dummyPlaneLen);
		if (ret < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -ret, i);
			goto out;
		}
	}

	ret = mGlVideo->loadFrame((const uint8_t **)planes,
				  mCurrentFrameInfo.plane_stride,
				  &mCurrentFrameInfo.format,
				  &mCurrentFrameInfo.info,
				  mbStatus);
	if (ret < 0)
		PDRAW_LOG_ERRNO("gles2Video->loadFrame", -ret);

	setNormalization();

out:
	for (unsigned int i = 0; i < planeCount; i++) {
		if (planes[i] == nullptr)
			continue;
		int err =
			mbuf_raw_video_frame_release_plane(frame, i, planes[i]);
		if (err < 0)
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_release_plane(%u)",
				-err,
				i);
	}
	if (ancillaryData != nullptr) {
		int err = mbuf_ancillary_data_unref(ancillaryData);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -err);
	}

	mFrameLoaded = (ret == 0);
	return ret;
}


/* Called on the rendering thread */
int GlVideoRenderer::loadExternalVideoFrame(
	struct mbuf_raw_video_frame *frame,
	const struct pdraw_media_info *mediaInfo)
{
	int ret;
	struct mbuf_ancillary_data *userData = nullptr;
	size_t frameUserdataLen;
	const void *frameUserdata = nullptr;

	ret = mbuf_raw_video_frame_get_ancillary_data(
		frame, MBUF_ANCILLARY_KEY_USERDATA_SEI, &userData);
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
	pthread_mutex_lock(&mListenerMutex);
	if (mRendererListener != nullptr) {
		ret = mRendererListener->loadVideoTexture(
			mSession,
			mRenderer,
			mExtVideoTextureWidth,
			mExtVideoTextureHeight,
			mediaInfo,
			frame,
			frameUserdata,
			(size_t)frameUserdataLen);
	}
	pthread_mutex_unlock(&mListenerMutex);

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(mX, mY, mWidth, mHeight));

out:
	if (userData)
		mbuf_ancillary_data_unref(userData);
	mFrameLoaded = (ret == 0);
	return ret;
}


/* Called on the rendering thread */
int GlVideoRenderer::renderVideoFrame(const struct pdraw_rect *renderPos,
				      struct pdraw_rect *contentPos,
				      Eigen::Matrix4f &viewProjMat)
{
	struct vdef_rect crop = {
		.left = 0,
		.top = 0,
		.width = mCurrentFrameInfo.info.resolution.width,
		.height = mCurrentFrameInfo.info.resolution.height,
	};

	return mGlVideo->renderFrame(renderPos,
				     contentPos,
				     viewProjMat,
				     mCurrentFrameInfo.plane_stride,
				     &mCurrentFrameInfo.format,
				     &mCurrentFrameInfo.info,
				     &crop,
				     &mParams);
}


/* Called on the rendering thread */
int GlVideoRenderer::renderExternalVideoFrame(
	const struct pdraw_rect *renderPos,
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

	return mGlVideo->renderFrame(renderPos,
				     contentPos,
				     viewProjMat,
				     frameStride,
				     &vdef_rgb,
				     &info,
				     &crop,
				     &mParams);
}


int GlVideoRenderer::setupExtTexture(const struct vdef_raw_frame *frameInfo)
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


void GlVideoRenderer::createProjMatrix(Eigen::Matrix4f &projMat,
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


int GlVideoRenderer::getNextFrameDelay(mbuf_raw_video_frame_queue *queue,
				       uint64_t curTime,
				       bool allowDrop,
				       bool *shouldBreak,
				       bool *processAnyway,
				       uint64_t *delayUs,
				       int64_t *compensationUs,
				       int64_t *timingErrorUs,
				       uint64_t *frameTsUs,
				       int logLevel)
{
	int ret;
	uint32_t delay;
	struct mbuf_raw_video_frame *frame = nullptr;
	struct vdef_raw_frame frameInfo = {};
	uint64_t frameTs = 0;

	int count = mbuf_raw_video_frame_queue_get_count(queue);
	if ((mParams.scheduling_mode ==
	     PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE) &&
	    allowDrop && (count >= GL_RENDERER_QUEUE_MAX_FRAMES)) {
		/* The queue is full, discard the next frame to catch up */
		ret = mbuf_raw_video_frame_queue_pop(queue, &frame);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_pop", -ret);
		} else {
			if (logLevel) {
				(void)mbuf_raw_video_frame_get_frame_info(
					frame, &frameInfo);
				_PDRAW_LOG_INT(logLevel,
					       "QUEUE FULL (queue_count=%u) "
					       "frame #%u is discarded",
					       count,
					       frameInfo.info.index);
			}
			(void)mbuf_raw_video_frame_unref(frame);
			frame = nullptr;
			count--;
		}
	}

	ret = mbuf_raw_video_frame_queue_peek(queue, &frame);
	if (ret < 0) {
		if (ret != -EAGAIN) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_peek",
					-ret);
		}
		return ret;
	}

	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		mbuf_raw_video_frame_unref(frame);
		return ret;
	}
	frameTs = frameInfo.info.timescale
			  ? (frameInfo.info.timestamp * 1000000 +
			     frameInfo.info.timescale / 2) /
				    frameInfo.info.timescale
			  : 0;

	uint64_t inputTime = UINT64_MAX;
	inputTime = getFrameU64(frame, mAncillaryKey.c_str());

	mbuf_raw_video_frame_unref(frame);
	frame = nullptr;

	uint32_t outputDelay =
		(inputTime != UINT64_MAX) ? curTime - inputTime : 0;
	int64_t timingError = 0, addedTimingError = 0;
	int64_t compensation, epsilon = 0;
	if (mSchedLastInputTimestamp != UINT64_MAX) {
		epsilon = (GL_RENDERER_SCHED_ADAPTIVE_EPSILON_PERCENT *
				   (frameTs - mSchedLastInputTimestamp) +
			   50) /
			  100;
	}
	if (mSchedLastInputTimestamp != UINT64_MAX &&
	    mSchedLastOutputTimestamp != UINT64_MAX) {
		timingError =
			(int64_t)frameTs - (int64_t)mSchedLastInputTimestamp -
			(int64_t)curTime + (int64_t)mSchedLastOutputTimestamp;
	}
	if (mSchedInitialBuffering && mSchedLastInputTimestamp != UINT64_MAX) {
		addedTimingError =
			(GL_RENDERER_SCHED_ADAPTIVE_INIT_BUF_DELAY_PERCENT *
				 (frameTs - mSchedLastInputTimestamp) +
			 50) /
			100;
	}

	bool _shouldBreak = false;
	bool _processAnyway = false;
	switch (mParams.scheduling_mode) {
	default:
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP:
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE:
		/* Render the frame ASAP */
		delay = 0;
		compensation = 0;
		break;
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE:
		if ((count > GL_RENDERER_QUEUE_MAX_FRAMES / 2) &&
		    (mSchedInitialBuffering)) {
			/* Half the queue must be filled (rounded up) to exit
			 * the initial buffering */
			PDRAW_LOGD("INITIAL BUFFERING: end (queue_count=%u)",
				   count);
			mSchedInitialBuffering = false;
		}
		if (mSchedInitialBuffering) {
			/* During the initial buffering, add more timing error
			 * to delay frames and allow filling the queue while
			 * still rendering frames */
			if (logLevel) {
				_PDRAW_LOG_INT(
					logLevel,
					"INITIAL BUFFERING: (queue_count=%u) "
					"added_timing_error=%.2fms",
					count,
					(float)addedTimingError / 1000.);
			}
			timingError += addedTimingError;
		}
		if (timingError > epsilon) {
			/* The frame is early */
			if (count < GL_RENDERER_QUEUE_MAX_FRAMES - 1) {
				/* Process the frame later; break */
				delay = timingError;
				compensation = 0;
				_shouldBreak = true;
				if (logLevel) {
					_PDRAW_LOG_INT(
						logLevel,
						"[EARLY]  frame #%u is %.2fms "
						"early (delay=%.2fms, "
						"queue_count=%u)",
						frameInfo.info.index,
						(float)timingError / 1000.,
						(float)outputDelay / 1000.,
						count);
				}
			} else {
				/* Process the frame now anyway; do not take the
				 * timing error into account as compensation in
				 * mSchedLastOutputTimestamp */
				if (logLevel) {
					_PDRAW_LOG_INT(
						logLevel,
						"[EARLY]  frame #%u is %.2fms "
						"early (delay=%.2fms, "
						"queue_count=%u) "
						"=> PROCESS ANYWAY",
						frameInfo.info.index,
						(float)timingError / 1000.,
						(float)outputDelay / 1000.,
						count);
				}
				_processAnyway = true;
				compensation = 0;
				delay = 0;
			}
		} else if (timingError < -epsilon) {
			/* The frame is late */
			if (logLevel) {
				_PDRAW_LOG_INT(logLevel,
					       "[LATE]   frame #%u is %.2fms "
					       "late (delay=%.2fms, "
					       "queue_count=%u)",
					       frameInfo.info.index,
					       (float)timingError / -1000.,
					       (float)outputDelay / 1000.,
					       count);
			}
			delay = 0;
			compensation = timingError;
		} else {
			/* The frame is on time */
			if (logLevel) {
				_PDRAW_LOG_INT(logLevel,
					       "[ONTIME] frame #%u is on time "
					       "(delay=%.2fms, queue_count=%u)",
					       frameInfo.info.index,
					       (float)outputDelay / 1000.,
					       count);
			}
			delay = 0;
			compensation = timingError;
		}
		break;
	}

	if (shouldBreak != nullptr)
		*shouldBreak = _shouldBreak;
	if (processAnyway != nullptr)
		*processAnyway = _processAnyway;
	if (delayUs != nullptr)
		*delayUs = delay;
	if (compensationUs != nullptr)
		*compensationUs = compensation;
	if (timingErrorUs != nullptr)
		*timingErrorUs = timingError;
	if (frameTsUs != nullptr)
		*frameTsUs = frameTs;

	return 0;
}


int GlVideoRenderer::scheduleFrame(uint64_t curTime,
				   bool *load,
				   int64_t *compensationUs)
{
	int ret = 0, err;
	bool _load = false;
	struct mbuf_raw_video_frame *frame = nullptr;
	int count;
	bool shouldBreak;
	uint64_t frameTs;
	int64_t compensation;

	struct mbuf_raw_video_frame_queue *queue = getPrimaryMediaQueue();
	if (queue == nullptr) {
		ret = -EPROTO;
		goto out;
	}

	count = mbuf_raw_video_frame_queue_get_count(queue);
	if (count == 0 && !mSchedInitialBuffering) {
		/* Empty queue, reset to initial buffering (the shortage of
		 * frames can be because of a seek or pause in the playback) */
		ret = -EAGAIN;
		mSchedLastInputTimestamp = UINT64_MAX;
		mSchedLastOutputTimestamp = UINT64_MAX;
		mSchedInitialBuffering = true;
		PDRAW_LOGD("INITIAL BUFFERING: start");
		goto out;
	}

	/* Get a new frame to render */
	do {
		shouldBreak = false;
		frameTs = 0;
		compensation = 0;

		err = getNextFrameDelay(queue,
					curTime,
					true,
					&shouldBreak,
					nullptr,
					nullptr,
					&compensation,
					nullptr,
					&frameTs,
					0);
		if (err < 0) {
			if (err != -EAGAIN)
				PDRAW_LOG_ERRNO("getNextFrameDelay", -err);
			break;
		}

		if (shouldBreak)
			break;

		err = mbuf_raw_video_frame_queue_pop(queue, &frame);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_pop", -err);
			break;
		}

		if (mCurrentFrame != nullptr)
			(void)mbuf_raw_video_frame_unref(mCurrentFrame);
		mCurrentFrame = frame;
		_load = true;
		frame = nullptr;
		mSchedLastOutputTimestamp = curTime + compensation;
		mSchedLastInputTimestamp = frameTs;
		if (compensationUs != nullptr)
			*compensationUs = compensation;
	} while (true);

out:
	if ((ret == 0) && (load != nullptr))
		*load = _load;
	return ret;
}


/* Called on the loop thread */
void GlVideoRenderer::onChannelSessionMetaUpdate(Channel *channel)
{
	int inputMediaCount, i;
	Sink::onChannelSessionMetaUpdate(channel);

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	for (i = inputMediaCount - 1; i >= 0; i--) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGE("getInputMedia");
			continue;
		}
		RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);
		if (rawMedia == nullptr)
			continue;
		if (getInputChannel(media) != channel)
			continue;
		if (mPrimaryMedia != rawMedia)
			continue;
		/* Media found */
		mMediaInfoSessionMeta = rawMedia->sessionMeta;
		mMediaInfo.video.session_meta = &mMediaInfoSessionMeta;
	}

	Sink::unlock();
}


/* Called on the rendering thread */
int GlVideoRenderer::render(struct pdraw_rect *contentPos,
			    const float *viewMat,
			    const float *projMat)
{
	int err;
	struct pdraw_rect renderPos;
	bool load = false, render = false;
	uint64_t curTime = 0, inputTime, delay = 0;
	int64_t compensation = 0;
	struct timespec ts = {0, 0};
	struct pdraw_media_info *mediaInfoPtr = nullptr;
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

	err = time_get_monotonic(&ts);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_get_monotonic", -err);
	err = time_timespec_to_us(&ts, &curTime);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_timespec_to_us", -err);

	/* Compute the rendering framerate as a sliding average of the
	 * rendering timestamp delta */
	uint64_t renderDelta = 0;
	if (mLastRenderTimestamp != UINT64_MAX)
		renderDelta = curTime - mLastRenderTimestamp;
	mLastRenderTimestamp = curTime;
	mAvgRenderRate = 0.9f * mAvgRenderRate +
			 (renderDelta ? 0.1f * 1000000 / renderDelta : 0.f);

	GLCHK(glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFbo));

	if (mPendingResize) {
		GLCHK();
		GLCHK(glViewport(mX, mY, mWidth, mHeight));
		mPendingResize = false;
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(mX, mY, mWidth, mHeight));
	GLCHK(glDisable(GL_DITHER));
	renderPos.x = mX;
	renderPos.y = mY;
	renderPos.width = mWidth;
	renderPos.height = mHeight;

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

	Sink::lock();
	if (mPrimaryMedia == nullptr) {
		/* No current media */
		goto skip_dequeue;
	}

	Media::cleanupMediaInfo(&mMediaInfo);
	mPrimaryMedia->fillMediaInfo(&mMediaInfo);
	/* Deep copy: copy the session metadata */
	mMediaInfoSessionMeta = *mMediaInfo.video.session_meta;
	mMediaInfo.video.session_meta = &mMediaInfoSessionMeta;
	mediaInfoPtr = &mMediaInfo;

	err = scheduleFrame(curTime, &load, &compensation);
	if (err < 0) {
		Sink::unlock();
		goto skip_dequeue;
	}

	if (load && !mEos) {
		/* We have a new frame */
		bool expected = true;
		if (mWatchdogTriggered.compare_exchange_strong(expected,
							       false)) {
			PDRAW_LOGI("new frame to render");
		}
		int err = pomp_timer_set(mWatchdogTimer,
					 1000 * GL_RENDERER_WATCHDOG_TIME_S);
		if (err != 0) {
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
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
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-err);
		goto skip_render;
	}
	data = (RawVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, nullptr);
	if (data == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid ancillary data pointer");
		goto skip_render;
	}
	mCurrentFrameData = *data;
	mbuf_ancillary_data_unref(ancillaryData);

	err = mbuf_raw_video_frame_get_frame_info(mCurrentFrame,
						  &mCurrentFrameInfo);
	if (err < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -err);
		goto skip_render;
	}

	err = mbuf_raw_video_frame_get_metadata(mCurrentFrame,
						&mCurrentFrameMetadata);
	if (err < 0 && err != -ENOENT) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -err);
		goto skip_render;
	}

	inputTime = getFrameU64(mCurrentFrame, mAncillaryKey.c_str());
	delay = (inputTime != 0) ? curTime - inputTime : 0;

	if (mFirstFrame) {
		mFirstFrame = false;
		int err = pomp_timer_set_periodic(
			mVideoPresStatsTimer,
			GL_RENDERER_VIDEO_PRES_STATS_TIME_MS,
			GL_RENDERER_VIDEO_PRES_STATS_TIME_MS);
		if (err != 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		if (mExtLoadVideoTexture) {
			err = setupExtTexture(&mCurrentFrameInfo);
			if (err < 0)
				PDRAW_LOG_ERRNO("setupExtTexture", -err);
		}
	}

skip_dequeue:
	if (mGlVideo == nullptr) {
		Sink::unlock();
		goto skip_render;
	}

	err = doTransition(curTime, load, &load);
	if (err < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("doTransition", -err);
		goto skip_render;
	}

	if (mCurrentFrame != nullptr) {
		if (mExtLoadVideoTexture) {
			err = loadExternalVideoFrame(mCurrentFrame,
						     mediaInfoPtr);
			if (err < 0) {
				Sink::unlock();
				goto skip_render;
			}
		} else if (load) {
			err = loadVideoFrame(mCurrentFrame);
			if (err < 0) {
				Sink::unlock();
				goto skip_render;
			}
		}
	}

	if (mCurrentFrame != nullptr && load) {
		/* First rendering of the current frame */

		int queueCount = 0;
		struct mbuf_raw_video_frame_queue *queue =
			getPrimaryMediaQueue();
		if (queue != nullptr) {
			queueCount =
				mbuf_raw_video_frame_queue_get_count(queue);
		}

		mCurrentFrameData.renderTimestamp = curTime;
		uint64_t timestampDelta = 0;
		if ((mCurrentFrameData.ntpRawTimestamp != 0) &&
		    (mLastFrameTimestamp != UINT64_MAX)) {
			timestampDelta = mCurrentFrameData.ntpRawTimestamp -
					 mLastFrameTimestamp;
		}
		uint64_t loadDelta = 0;
		if ((mCurrentFrameData.renderTimestamp != 0) &&
		    (mLastLoadTimestamp != UINT64_MAX)) {
			loadDelta = mCurrentFrameData.renderTimestamp -
				    mLastLoadTimestamp;
		}
		int64_t timingError = 0;
		if ((timestampDelta != 0) && (loadDelta != 0)) {
			timingError =
				(int64_t)timestampDelta - (int64_t)loadDelta;
		}
		uint64_t timingErrorAbs =
			(timingError < 0) ? -timingError : timingError;
		uint64_t estLatency = 0;
		if ((mCurrentFrameData.renderTimestamp != 0) &&
		    (mCurrentFrameData.localTimestamp != 0) &&
		    (mCurrentFrameData.renderTimestamp >
		     mCurrentFrameData.localTimestamp)) {
			estLatency = mCurrentFrameData.renderTimestamp -
				     mCurrentFrameData.localTimestamp;
		}
		uint64_t playerLatency = 0;
		if ((mCurrentFrameData.renderTimestamp != 0) &&
		    (mCurrentFrameData.recvStartTimestamp != 0) &&
		    (mCurrentFrameData.renderTimestamp >
		     mCurrentFrameData.recvStartTimestamp)) {
			playerLatency = mCurrentFrameData.renderTimestamp -
					mCurrentFrameData.recvStartTimestamp;
		}
		PDRAW_LOGD(
			"frame #%u loaded est_total_latency=%.2fms "
			"player_latency=%.2fms load_interval=%.2fms "
			"render_delay=%.2fms timing_error=%.2fms "
			"avg_render_rate=%.2ffps queue_count=%u",
			mCurrentFrameInfo.info.index,
			(float)estLatency / 1000.,
			(float)playerLatency / 1000.,
			(float)loadDelta / 1000.,
			(float)delay / 1000.,
			(float)timingError / 1000.,
			mAvgRenderRate,
			queueCount);
		mVideoPresStats.timestamp = mCurrentFrameData.captureTimestamp;
		mVideoPresStats.presentationFrameCount++;
		mVideoPresStats.presentationTimestampDeltaIntegral +=
			timestampDelta;
		mVideoPresStats.presentationTimestampDeltaIntegralSq +=
			timestampDelta * timestampDelta;
		mVideoPresStats.presentationTimingErrorIntegral +=
			timingErrorAbs;
		mVideoPresStats.presentationTimingErrorIntegralSq +=
			timingErrorAbs * timingErrorAbs;
		mVideoPresStats.presentationEstimatedLatencyIntegral +=
			estLatency;
		mVideoPresStats.presentationEstimatedLatencyIntegralSq +=
			estLatency * estLatency;
		mVideoPresStats.playerLatencyIntegral += playerLatency;
		mVideoPresStats.playerLatencyIntegralSq +=
			playerLatency * playerLatency;
		mVideoPresStats.estimatedLatencyPrecisionIntegral +=
			mCurrentFrameData.localTimestampPrecision;
		mLastLoadTimestamp =
			mCurrentFrameData.renderTimestamp + compensation;
		mLastFrameTimestamp = mCurrentFrameData.ntpRawTimestamp;
	}

	Sink::unlock();

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
		err = mGlVideo->clear(vpMat);
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
		struct pdraw_video_frame_extra frameExtra = {};
		struct vmeta_frame *frameMetaPtr = nullptr;
		const struct pdraw_video_frame_extra *frameExtraPtr = nullptr;
		if (render) {
			frameExtra.info = mCurrentFrameInfo.info;
			frameExtra.play_timestamp =
				mCurrentFrameData.playTimestamp;
			if (mGlVideo) {
				mGlVideo->getHistograms(
					frameExtra.histogram,
					frameExtra.histogram_len);
			}
			frameMetaPtr = mCurrentFrameMetadata;
			frameExtraPtr = &frameExtra;
		}
		pthread_mutex_lock(&mListenerMutex);
		if (mRendererListener != nullptr) {
			mRendererListener->renderVideoOverlay(mSession,
							      mRenderer,
							      &renderPos,
							      &content,
							      vMat.data(),
							      pMat.data(),
							      mediaInfoPtr,
							      frameMetaPtr,
							      frameExtraPtr);
		}
		pthread_mutex_unlock(&mListenerMutex);
	}

	if (contentPos != nullptr)
		*contentPos = content;

	return 0;
}


/* Called on the rendering thread */
int GlVideoRenderer::startExtLoad(void)
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

	if (mGlVideo)
		mGlVideo->setExtTexture(mExtLoadFboTexture);

out:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	return ret;
}


/* Called on the rendering thread */
int GlVideoRenderer::stopExtLoad(void)
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

	if (mGlVideo)
		mGlVideo->setExtTexture(mExtLoadFboTexture);

	return 0;
}


/* Called on the rendering thread */
int GlVideoRenderer::resize(const struct pdraw_rect *renderPos)
{
	if (renderPos == nullptr)
		return -EINVAL;

	if (renderPos->width == 0 && renderPos->height == 0)
		return -EPROTO;

	mX = renderPos->x;
	mY = renderPos->y;
	mWidth = renderPos->width;
	mHeight = renderPos->height;
	mPendingResize = true;

	return 0;
}


/* Called on the rendering thread */
int GlVideoRenderer::setMediaId(unsigned int mediaId)
{
	if (mediaId == mTargetPrimaryMediaId)
		return 0;

	mTargetPrimaryMediaId = mediaId;
	int ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleRenewMedia, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return 0;
}


/* Called on the rendering thread */
unsigned int GlVideoRenderer::getMediaId(void)
{
	return mPrimaryMediaId;
}


/* Called on the rendering thread */
int GlVideoRenderer::setParams(const struct pdraw_video_renderer_params *params,
			       bool force)
{
	int err;

	if (params == nullptr)
		return -EINVAL;

	/* This parameter cannot be dynamically changed */
	int enable_simplified_rendering = mParams.enable_simplified_rendering;

	mParams = *params;

	if (!force) {
		mParams.enable_simplified_rendering =
			enable_simplified_rendering;
	}

	uint32_t dbgFlags = 0;
	const char *envDbgFlags = getenv(PDRAW_VIDEO_RENDERER_DBG_FLAGS);
	if (envDbgFlags != nullptr) {
		char *endptr = nullptr;
		errno = 0;
		long parsedint = strtol(envDbgFlags, &endptr, 10);
		if (envDbgFlags[0] == '\0' || endptr[0] != '\0' ||
		    parsedint < 0 || errno != 0) {
			err = -errno;
			PDRAW_LOG_ERRNO("strtol: %s", -err, envDbgFlags);
		} else {
			dbgFlags = parsedint;
		}
	}

	mMbStatusOverlay =
		dbgFlags & PDRAW_VIDEO_RENDERER_DBG_FLAG_MB_STATUS_OVERLAY;

	if (!mExtLoadVideoTexture) {
		mExtVideoTextureWidth = 0;
		mExtVideoTextureHeight = 0;
		mParams.video_texture_width = 0;
		mParams.video_texture_dar_width = 0;
		mParams.video_texture_dar_height = 0;
	}
	mFirstFrame = true;

	return 0;
}


/* Called on the rendering thread */
int GlVideoRenderer::getParams(struct pdraw_video_renderer_params *params)
{
	if (params)
		*params = mParams;
	return 0;
}

} /* namespace Pdraw */

#endif /* PDRAW_USE_GL */
