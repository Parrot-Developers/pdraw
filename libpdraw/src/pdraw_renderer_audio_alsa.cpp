/**
 * Parrot Drones Audio and Video Vector library
 * ALSA audio renderer
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

#define ULOG_TAG pdraw_rndaudioalsa
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_renderer_audio_alsa.hpp"
#include "pdraw_settings.hpp"

#ifdef PDRAW_USE_ALSA

namespace Pdraw {

#	define ALSA_RENDERER_DEFAULT_DELAY_MS 33
#	define ALSA_RENDERER_WATCHDOG_TIME_S 2
#	define ALSA_RENDERER_ANCILLARY_DATA_KEY_INPUT_TIME                    \
		"pdraw.alsa_renderer.input_time"
#	define ALSA_RENDERER_QUEUE_MAX_FRAMES 5
#	define ALSA_RENDERER_WATCHDOG_TIME_S 2
#	define ALSA_RENDERER_MIN_FRAMES_START 5

#	define NB_SUPPORTED_FORMATS 24
static struct adef_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;

static void initializeSupportedFormats(void)
{
	size_t i = 0;

	supportedFormats[i++] = adef_pcm_16b_8000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_8000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_11025hz_mono;
	supportedFormats[i++] = adef_pcm_16b_11025hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_12000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_12000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_16000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_16000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_22050hz_mono;
	supportedFormats[i++] = adef_pcm_16b_22050hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_24000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_24000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_32000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_32000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_44100hz_mono;
	supportedFormats[i++] = adef_pcm_16b_44100hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_48000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_48000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_64000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_64000hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_88200hz_mono;
	supportedFormats[i++] = adef_pcm_16b_88200hz_stereo;
	supportedFormats[i++] = adef_pcm_16b_96000hz_mono;
	supportedFormats[i++] = adef_pcm_16b_96000hz_stereo;
}


AlsaAudioRenderer::AlsaAudioRenderer(
	Session *session,
	Element::Listener *listener,
	AudioRendererWrapper *wrapper,
	IPdraw::IAudioRenderer::Listener *rndListener,
	uint32_t mediaTypeCaps,
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params) :
		AudioRenderer(session,
			      listener,
			      wrapper,
			      rndListener,
			      mediaTypeCaps,
			      nullptr,
			      0,
			      mediaId,
			      params),
		mMediaId(mediaId), mCurrentMediaId(0), mRunning(false),
		mLastAddedMedia(nullptr), mMediaInfo({}), mAlsaReady(false),
		mAddress(""), mHandle(nullptr), mHwParams(nullptr),
		mSwParams(nullptr), mFrameSize(0),
		mSampleCount(ALSA_AUDIO_DEFAULT_SAMPLE_COUNT),
		mWatchdogTimer(nullptr), mWatchdogTriggered(false), mEos(false)
{
	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setAudioMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	Element::setClassName(__func__);

	mParams = *params;
	if (params->address != nullptr) {
		mAddress = params->address;
		mParams.address = mAddress.c_str();
	} else {
		mParams.address = nullptr;
	}

	setState(CREATED);
}


AlsaAudioRenderer::~AlsaAudioRenderer(void)
{
	int err;

	if (mState == STARTED)
		PDRAW_LOGW("renderer is still running");

	/* Make sure listener function will no longer be called */
	removeRendererListener();

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("not all input media have been removed");
		err = removeInputMedias();
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedias", -err);
	}

	Media::cleanupMediaInfo(&mMediaInfo);

	if (mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mWatchdogTimer = nullptr;
	}
}


void AlsaAudioRenderer::watchdogTimerCb(struct pomp_timer *timer,
					void *userdata)
{
	AlsaAudioRenderer *self = (AlsaAudioRenderer *)userdata;

	if ((!self->mRunning) || (self->mState != STARTED))
		return;

	bool expected = false;
	if (self->mWatchdogTriggered.compare_exchange_strong(expected, true)) {
		PDRAW_LOGW("no new frame for %ds",
			   ALSA_RENDERER_WATCHDOG_TIME_S);
	}
}


int AlsaAudioRenderer::startAlsa(void)
{
	int ret;
	snd_pcm_format_t format;

	if (mAlsaReady)
		return -EALREADY;

	/* Check parameters */
	if (!adef_is_format_valid(&mMediaInfo.audio.format)) {
		ret = -EINVAL;
		PDRAW_LOGE("invalid audio format: " ADEF_FORMAT_TO_STR_FMT,
			   ADEF_FORMAT_TO_STR_ARG(&mMediaInfo.audio.format));
		goto error;
	}

	format = AlsaAudio::adefFormatToAlsa(&mMediaInfo.audio.format);
	if (format == SND_PCM_FORMAT_UNKNOWN) {
		ret = -EINVAL;
		PDRAW_LOGE("unsupported audio format: " ADEF_FORMAT_TO_STR_FMT,
			   ADEF_FORMAT_TO_STR_ARG(&mMediaInfo.audio.format));
		goto error;
	}

	mFrameSize = snd_pcm_format_width(format) / 8 *
		     mMediaInfo.audio.format.channel_count;

	if (mHandle == nullptr) {
		ret = snd_pcm_open(&mHandle,
				   mParams.address,
				   SND_PCM_STREAM_PLAYBACK,
				   SND_PCM_NONBLOCK);
		if (ret < 0) {
			PDRAW_LOGE("snd_pcm_open:'%s'(%s)",
				   mParams.address,
				   snd_strerror(ret));
			goto error;
		}
	}

	if (mHwParams == nullptr) {
		ret = snd_pcm_hw_params_malloc(&mHwParams);
		if (ret < 0) {
			PDRAW_LOGE("snd_pcm_hw_params_malloc(%s)",
				   snd_strerror(ret));
			goto error;
		}
	}

	ret = snd_pcm_hw_params_any(mHandle, mHwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_any(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_access(
		mHandle,
		mHwParams,
		mMediaInfo.audio.format.pcm.interleaved
			? SND_PCM_ACCESS_RW_INTERLEAVED
			: SND_PCM_ACCESS_RW_NONINTERLEAVED);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_access(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_format(mHandle, mHwParams, format);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_format(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_channels(
		mHandle, mHwParams, mMediaInfo.audio.format.channel_count);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_channels(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_rate(
		mHandle, mHwParams, mMediaInfo.audio.format.sample_rate, 0);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_rate(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_period_size(
		mHandle, mHwParams, mSampleCount, 0);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_period_size(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params(mHandle, mHwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params(%s)", snd_strerror(ret));
		goto error;
	}

	if (mSwParams == nullptr) {
		ret = snd_pcm_sw_params_malloc(&mSwParams);
		if (ret < 0) {
			PDRAW_LOGE("snd_pcm_sw_params_malloc(%s)",
				   snd_strerror(ret));
			goto error;
		}
	}

	ret = snd_pcm_sw_params_current(mHandle, mSwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_sw_params_current(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_sw_params_set_start_threshold(
		mHandle,
		mSwParams,
		mSampleCount * ALSA_RENDERER_MIN_FRAMES_START);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_sw_params_set_start_threshold(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_sw_params(mHandle, mSwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_sw_params(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_sw_params_set_avail_min(
		mHandle,
		mSwParams,
		mSampleCount * ALSA_RENDERER_MIN_FRAMES_START);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_sw_params_set_avail_min(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_sw_params(mHandle, mSwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_sw_params(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_prepare(mHandle);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_prepare(%s)", snd_strerror(ret));
		goto error;
	}

	mAlsaReady = true;

	return 0;

error:
	(void)stopAlsa();
	return ret;
}


int AlsaAudioRenderer::stopAlsa(void)
{
	int ret;

	if (!mAlsaReady)
		return 0;

	if (mHandle != nullptr) {
		ret = snd_pcm_drop(mHandle);
		if (ret < 0)
			PDRAW_LOG_ERRNO("snd_pcm_drop", -ret);
		ret = snd_pcm_close(mHandle);
		if (ret < 0)
			PDRAW_LOG_ERRNO("snd_pcm_close", -ret);
		mHandle = nullptr;
	}

	if (mHwParams != nullptr) {
		snd_pcm_hw_params_free(mHwParams);
		mHwParams = nullptr;
	}

	if (mSwParams != nullptr) {
		snd_pcm_sw_params_free(mSwParams);
		mSwParams = nullptr;
	}

	mAlsaReady = false;

	return 0;
}


void AlsaAudioRenderer::onChannelFlush(Channel *channel)
{
	int err;

	AudioChannel *c = dynamic_cast<AudioChannel *>(channel);
	if (c == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");

	Sink::lock();

	struct mbuf_audio_frame_queue *queue = c->getQueue(this);
	if (queue != nullptr) {
		err = mbuf_audio_frame_queue_flush(queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
	}

	Sink::unlock();

	err = c->asyncFlushDone();
	if (err < 0)
		PDRAW_LOG_ERRNO("Channel::asyncFlushDone", -err);
}


void AlsaAudioRenderer::onChannelSos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mEos = false;

	Sink::onChannelSos(channel);
}


void AlsaAudioRenderer::onChannelEos(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mEos = true;

	Sink::onChannelEos(channel);
	int ret = pomp_timer_clear(mWatchdogTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
}


void AlsaAudioRenderer::idleStart(void *renderer)
{
	AlsaAudioRenderer *self =
		reinterpret_cast<AlsaAudioRenderer *>(renderer);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	int err;

	if (self->mState != STARTING) {
		PDRAW_LOGE("renderer is not starting");
		return;
	}

	if (self->mWatchdogTimer == nullptr) {
		self->mWatchdogTimer = pomp_timer_new(
			self->mSession->getLoop(), watchdogTimerCb, self);
		if (self->mWatchdogTimer == nullptr) {
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}

	self->setState(STARTED);
	return;

error:
	if (self->mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mWatchdogTimer = nullptr;
	}
}


int AlsaAudioRenderer::start(void)
{

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("renderer is not created");
		return -EPROTO;
	}

	setState(STARTING);

	mRunning = true;

	int ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleStart, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return ret;
}


struct mbuf_audio_frame_queue *AlsaAudioRenderer::getLastAddedMediaQueue(void)
{
	Sink::lock();
	struct mbuf_audio_frame_queue *queue = nullptr;
	AudioChannel *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(mLastAddedMedia));
	if (channel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
		Sink::unlock();
		return nullptr;
	}
	queue = channel->getQueue(this);
	if (queue == nullptr) {
		PDRAW_LOGE("failed to get input queue");
		Sink::unlock();
		return nullptr;
	}
	Sink::unlock();
	return queue;
}


int AlsaAudioRenderer::stop(void)
{
	int err = 0;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;

	setState(STOPPING);

	mRunning = false;

	/* Flush the remaining frames */
	Sink::lock();

	struct mbuf_audio_frame_queue *queue = getLastAddedMediaQueue();
	if (queue != nullptr) {
		err = mbuf_audio_frame_queue_flush(queue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
		}
	}
	Sink::unlock();

	removeRendererListener();

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	/* Post a message on the loop thread */
	asyncCompleteStop();

	return 0;
}


int AlsaAudioRenderer::setMediaId(unsigned int mediaId)
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


unsigned int AlsaAudioRenderer::getMediaId(void) const
{
	return mMediaId;
}


int AlsaAudioRenderer::setParams(
	const struct pdraw_audio_renderer_params *params)
{
	if (params == nullptr)
		return -EINVAL;

	/* Address cannot change */
	if (mParams.address != nullptr && params->address != nullptr &&
	    strcmp(mParams.address, params->address) != 0)
		return -EPROTO;

	mParams = *params;
	if (!mAddress.empty())
		mParams.address = mAddress.c_str();
	else
		mParams.address = nullptr;

	return 0;
}


int AlsaAudioRenderer::getParams(struct pdraw_audio_renderer_params *params)
{
	if (params)
		*params = mParams;
	return 0;
}


bool AlsaAudioRenderer::queueFilter(struct mbuf_audio_frame *frame,
				    void *userdata)
{
	int err;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};

	/* Set the input time ancillary data to the frame */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_audio_frame_add_ancillary_buffer(
		frame,
		ALSA_RENDERER_ANCILLARY_DATA_KEY_INPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0)
		ULOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -err);

	return true;
}


int AlsaAudioRenderer::removeQueueFdFromPomp(
	struct mbuf_audio_frame_queue *queue)
{
	int ret;
	struct pomp_loop *loop = nullptr;
	struct pomp_evt *evt = nullptr;

	loop = mSession->getLoop();
	if (loop == nullptr) {
		PDRAW_LOGE("loop not found");
		return -ENODEV;
	}

	ret = mbuf_audio_frame_queue_get_event(queue, &evt);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -ret);
		return ret;
	}

	ret = pomp_evt_detach_from_loop(evt, loop);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -ret);
		return ret;
	}

	return 0;
}


int AlsaAudioRenderer::addInputMedia(Media *media)
{
	struct pomp_loop *loop = nullptr;
	struct pomp_evt *evt = nullptr;
	int res = 0, err = 0;

	/* Only accept raw audio media */
	AudioMedia *m = dynamic_cast<AudioMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("unsupported input media");
		return -ENOSYS;
	}

	if ((mMediaId != 0) && (mMediaId != m->id))
		return -EPERM;
	if (mLastAddedMedia != nullptr)
		return -EBUSY;
	if ((!mRunning) || (mState != STARTED))
		return -EAGAIN;

	Sink::lock();

	res = Sink::addInputMedia(m);
	if (res == -EEXIST) {
		Sink::unlock();
		return res;
	} else if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		return res;
	}

	AudioChannel *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(m));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	struct mbuf_audio_frame_queue_args args = {};
	args.filter = &queueFilter;
	args.filter_userdata = this;
	args.max_frames = ALSA_RENDERER_QUEUE_MAX_FRAMES;
	struct mbuf_audio_frame_queue *queue;
	res = mbuf_audio_frame_queue_new_with_args(&args, &queue);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_new_with_args", -res);
		return res;
	}
	channel->setQueue(this, queue);

	err = mbuf_audio_frame_queue_get_event(queue, &evt);
	if (err != 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -err);
		goto error;
	}

	loop = mSession->getLoop();
	if (loop == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("loop not found");
		res = -ENODEV;
		goto error;
	}

	err = pomp_evt_attach_to_loop(evt, loop, renderCb, this);
	if (err != 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -err);
		goto error;
	}

	mLastAddedMedia = m;
	mCurrentMediaId = mMediaId;

	m->fillMediaInfo(&mMediaInfo);

	Sink::unlock();

	res = startAlsa();
	if (res < 0) {
		PDRAW_LOG_ERRNO("startAlsa", -res);
		goto error;
	}

	pthread_mutex_lock(&mListenerMutex);
	if (mRendererListener) {
		mRendererListener->onAudioRendererMediaAdded(
			mSession, mRenderer, &mMediaInfo);
	}
	pthread_mutex_unlock(&mListenerMutex);

	return 0;

error:
	removeQueueFdFromPomp(queue);
	return res;
}


int AlsaAudioRenderer::removeInputMedia(Media *media)
{
	int ret, err;

	Sink::lock();

	if (mLastAddedMedia == media) {
		mLastAddedMedia = nullptr;
		mCurrentMediaId = 0;
		pthread_mutex_lock(&mListenerMutex);
		if (mRendererListener) {
			mRendererListener->onAudioRendererMediaRemoved(
				mSession, mRenderer, &mMediaInfo);
		}
		pthread_mutex_unlock(&mListenerMutex);

		Media::cleanupMediaInfo(&mMediaInfo);
	}

	AudioChannel *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	/* Keep a reference on the queue to destroy it after removing the
	 * input media (avoids deadlocks when trying to push new frames out
	 * of the AudioDecoder whereas the queue is already destroyed) */
	struct mbuf_audio_frame_queue *queue = channel->getQueue(this);

	ret = Sink::removeInputMedia(media);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -ret);
		return ret;
	}

	Sink::unlock();

	if (queue != nullptr) {
		err = removeQueueFdFromPomp(queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueFdFromPomp", -err);
		err = mbuf_audio_frame_queue_flush(queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
		err = mbuf_audio_frame_queue_destroy(queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
	}

	err = stopAlsa();
	if (err < 0)
		PDRAW_LOG_ERRNO("stopAlsa", -err);

	return 0;
}


int AlsaAudioRenderer::removeInputMedias(void)
{
	int ret, inputMediaCount, i;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		AudioMedia *media =
			dynamic_cast<AudioMedia *>(getInputMedia(i));
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
	Sink::unlock();

	return 0;
}


void AlsaAudioRenderer::idleRenewMedia(void *userdata)
{
	AlsaAudioRenderer *self =
		reinterpret_cast<AlsaAudioRenderer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	if (self->mLastAddedMedia != nullptr)
		self->removeInputMedia(self->mLastAddedMedia);
	self->mSession->addMediaToAudioRenderer(self->mMediaId, self);
}


void AlsaAudioRenderer::completeStop(void)
{
	int ret;

	if (mState == STOPPED)
		return;

	if (mWatchdogTimer != nullptr) {
		ret = pomp_timer_clear(mWatchdogTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	}

	ret = removeInputMedias();
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedias", -ret);

	setState(STOPPED);
}


void AlsaAudioRenderer::renderCb(pomp_evt *event, void *userdata)
{
	AlsaAudioRenderer *self = static_cast<AlsaAudioRenderer *>(userdata);
	/* We have a new frame */

	if (!self->mEos) {
		bool expected = true;
		if (self->mWatchdogTriggered.compare_exchange_strong(expected,
								     false)) {
			PDRAW_LOGI("new frame to render");
		}
		int err = pomp_timer_set(self->mWatchdogTimer,
					 1000 * ALSA_RENDERER_WATCHDOG_TIME_S);
		if (err != 0) {
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
	}
	self->render();
}


int AlsaAudioRenderer::render()
{
	struct mbuf_audio_frame *frame = nullptr;
	struct mbuf_audio_frame_queue *queue = nullptr;
	int ret = 0, err;
	int count = 0;
	const void *data = nullptr;
	size_t len;

	if (!mAlsaReady) {
		PDRAW_LOGE("not ready");
		return -EPROTO;
	}

	Sink::lock();

	queue = getLastAddedMediaQueue();
	if (queue == nullptr) {
		ret = -EPROTO;
		PDRAW_LOGE("getLastAddedMediaQueue");
		goto out;
	}

	count = mbuf_audio_frame_queue_get_count(queue);
	if (count < 1) {
		ret = -EAGAIN;
		PDRAW_LOGW("no frame in queue");
		goto out;
	}

	ret = mbuf_audio_frame_queue_pop(queue, &frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_pop", -ret);
		goto out;
	}

	ret = mbuf_audio_frame_get_buffer(frame, &data, &len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_buffer", -ret);
		goto out;
	}

	if (len != mFrameSize * mSampleCount) {
		ret = -EPROTO;
		PDRAW_LOG_ERRNO(
			"invalid frame size (%zu), expecting "
			"frame size (%zu) * sample count (%zu)",
			-ret,
			len,
			mFrameSize,
			mSampleCount);
		goto out;
	}

	while (true) {
		if (mMediaInfo.audio.format.pcm.interleaved) {
			ret = snd_pcm_writei(mHandle, data, mSampleCount);
			if (ret < 0) {
				if (ret == -EPIPE) {
					snd_pcm_prepare(mHandle);
					continue;
				}
				PDRAW_LOG_ERRNO("snd_pcm_writei", -ret);
				break;
			}
		} else {
			ret = -EPROTO;
			PDRAW_LOGE("non-interleaved format is unsupported");
			goto out;
		}
		break;
	}

out:
	if (frame != nullptr) {
		if (data != nullptr) {
			err = mbuf_audio_frame_release_buffer(frame, data);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbuf_audio_frame_buffer",
						-err);
		}
		err = mbuf_audio_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
	}

	Sink::unlock();

	return ret;
}

} /* namespace Pdraw */

#endif /* PDRAW_USE_ALSA */
