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

#include "pdraw_renderer_audio_alsa.hpp"
#include "pdraw_settings.hpp"

#include <array>

ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef PDRAW_USE_ALSA

namespace Pdraw {

constexpr size_t ALSA_RENDERER_DEFAULT_DELAY_MS = 33;
constexpr size_t ALSA_RENDERER_WATCHDOG_TIME_S = 2;
constexpr const char *ALSA_RENDERER_ANCILLARY_DATA_KEY_INPUT_TIME =
	"pdraw.alsa_renderer.input_time";
constexpr size_t ALSA_RENDERER_QUEUE_MAX_FRAMES = 5;
constexpr size_t ALSA_RENDERER_MIN_FRAMES_START = 5;

static const std::array<struct adef_format, 24> &getSupportedFormats()
{
	static const std::array<struct adef_format, 24> formats = {{
		adef_pcm_16b_8000hz_mono,  adef_pcm_16b_8000hz_stereo,
		adef_pcm_16b_11025hz_mono, adef_pcm_16b_11025hz_stereo,
		adef_pcm_16b_12000hz_mono, adef_pcm_16b_12000hz_stereo,
		adef_pcm_16b_16000hz_mono, adef_pcm_16b_16000hz_stereo,
		adef_pcm_16b_22050hz_mono, adef_pcm_16b_22050hz_stereo,
		adef_pcm_16b_24000hz_mono, adef_pcm_16b_24000hz_stereo,
		adef_pcm_16b_32000hz_mono, adef_pcm_16b_32000hz_stereo,
		adef_pcm_16b_44100hz_mono, adef_pcm_16b_44100hz_stereo,
		adef_pcm_16b_48000hz_mono, adef_pcm_16b_48000hz_stereo,
		adef_pcm_16b_64000hz_mono, adef_pcm_16b_64000hz_stereo,
		adef_pcm_16b_88200hz_mono, adef_pcm_16b_88200hz_stereo,
		adef_pcm_16b_96000hz_mono, adef_pcm_16b_96000hz_stereo,
	}};
	return formats;
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
		mMediaId(mediaId), mParams(*params)
{
	setAudioMediaFormatCaps(getSupportedFormats().data(),
				static_cast<int>(getSupportedFormats().size()));

	Element::setClassName(__func__);

	if (params->address != nullptr) {
		mAddress = params->address;
		mParams.address = mAddress.c_str();
	} else {
		mParams.address = nullptr;
	}

	mWatchdogTimerHandler.set([this] { onWatchdogTimer(); });
	mIdleStartHandler.set([this] { idleStart(); });
	mIdleDrainHandler.set([this] { idleDrain(); });
	mIdleRenewMediaHandler.set([this] { idleRenewMedia(); });

	setState(State::CREATED);
}


AlsaAudioRenderer::~AlsaAudioRenderer()
{
	int err;

	if (mState == State::STARTED)
		PDRAW_LOGW("renderer is still running");

	/* Make sure listener function will no longer be called */
	removeRendererListener();

	/* Remove any leftover idle callbacks */
	err = mSession->getPompLoop()->idleRemove(this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("not all input media have been removed");
		err = removeInputMedias();
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedias", -err);
	}

	Media::cleanupMediaInfo(&mMediaInfo);

	mWatchdogTimer.reset();
}


void AlsaAudioRenderer::onWatchdogTimer()
{
	if ((!mRunning) || (mState != State::STARTED))
		return;

	bool expected = false;
	if (mWatchdogTriggered.compare_exchange_strong(expected, true)) {
		PDRAW_LOGW("no new frame for %zus",
			   ALSA_RENDERER_WATCHDOG_TIME_S);
	}
}


int AlsaAudioRenderer::startAlsa()
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


int AlsaAudioRenderer::stopAlsa()
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

	auto *c = dynamic_cast<AudioChannel *>(channel);
	ULOG_ERRNO_RETURN_IF(c == nullptr, EINVAL);

	PDRAW_LOGD("flushing input channel");

	Sink::lock();

	setFlushingState(FlushingState::FLUSHING);

	mbuf::Queue *queue = c->getQueue(this);
	if (queue != nullptr) {
		err = queue->flush();
		if (err < 0)
			PDRAW_LOG_ERRNO("queue::flush", -err);
	}

	setFlushingState(FlushingState::FLUSHED);

	Sink::unlock();

	err = c->asyncFlushDone();
	if (err < 0)
		PDRAW_LOG_ERRNO("Channel::asyncFlushDone", -err);
}


void AlsaAudioRenderer::onChannelDrain(Channel *channel)
{
	int err;
	const mbuf::Queue *queue = nullptr;
	auto *c = dynamic_cast<AudioChannel *>(channel);
	ULOG_ERRNO_RETURN_IF(c == nullptr, EINVAL);

	PDRAW_LOGD("flushing input channel");

	Sink::lock();

	setFlushingState(FlushingState::FLUSHING, false);

	/* Don't flush the queue */
	Sink::unlock();

	queue = c->getQueue(this);
	if ((queue != nullptr) && (queue->getCount() > 0))
		return;

	setFlushingState(FlushingState::FLUSHED);

	err = c->asyncDrainDone();
	if (err < 0)
		PDRAW_LOG_ERRNO("Channel::asyncFlushDone", -err);
}


void AlsaAudioRenderer::onChannelSos(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	mEos = false;

	Sink::onChannelSos(channel);
}


void AlsaAudioRenderer::onChannelEos(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	mEos = true;

	Sink::onChannelEos(channel);
	if (mWatchdogTimer != nullptr)
		(void)mWatchdogTimer->clear();
}


void AlsaAudioRenderer::idleStart()
{
	if (mState != State::STARTING) {
		PDRAW_LOGE("renderer is not starting");
		return;
	}

	if (mWatchdogTimer == nullptr) {
		try {
			mWatchdogTimer = std::make_unique<pomp::Timer>(
				mSession->getPompLoop(),
				&mWatchdogTimerHandler);
		} catch (const std::bad_alloc &) {
			PDRAW_LOGE("pomp::Timer allocation failed");
			goto error;
		}
	}

	setState(State::STARTED);
	return;

error:
	mWatchdogTimer.reset();
}


/* Called on the loop thread */
void AlsaAudioRenderer::idleDrain()
{
	int err = 0;
	AudioChannel *channel = nullptr;

	lock();
	if (mLastAddedMedia == nullptr) {
		unlock();
		return;
	}

	channel =
		dynamic_cast<AudioChannel *>(getInputChannel(mLastAddedMedia));
	if (channel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
		unlock();
		return;
	}

	err = channel->asyncDrainDone();
	if (err < 0)
		PDRAW_LOG_ERRNO("Channel::asyncFlushDone", -err);

	unlock();
	setFlushingState(FlushingState::FLUSHED);
}


int AlsaAudioRenderer::start()
{

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("renderer is not created");
		return -EPROTO;
	}

	setState(State::STARTING);

	mRunning = true;

	int ret = mSession->getPompLoop()->idleAdd(&mIdleStartHandler, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -ret);

	return ret;
}


mbuf::Queue *AlsaAudioRenderer::getLastAddedMediaQueue()
{
	Sink::lock();
	mbuf::Queue *queue = nullptr;
	const auto *channel =
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


int AlsaAudioRenderer::stop()
{
	int err = 0;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;

	setState(State::STOPPING);

	mRunning = false;

	/* Flush the remaining frames */
	Sink::lock();

	mbuf::Queue *queue = getLastAddedMediaQueue();
	if (queue != nullptr) {
		err = queue->flush();
		if (err < 0) {
			PDRAW_LOG_ERRNO("queue::flush", -err);
		}
	}
	Sink::unlock();

	removeRendererListener();

	/* Remove any leftover idle callbacks */
	err = mSession->getPompLoop()->idleRemove(this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	/* Post a message on the loop thread */
	asyncCompleteStop();

	return 0;
}


int AlsaAudioRenderer::setMediaId(unsigned int mediaId)
{
	if (mediaId == mMediaId)
		return 0;

	mMediaId = mediaId;
	int ret =
		mSession->getPompLoop()->idleAdd(&mIdleRenewMediaHandler, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -ret);

	return 0;
}


unsigned int AlsaAudioRenderer::getMediaId() const
{
	return mCurrentMediaId;
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
				    [[maybe_unused]] void *userdata)
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


int AlsaAudioRenderer::addInputMedia(Media *media)
{
	int res = 0;

	/* Only accept raw audio media */
	auto *m = dynamic_cast<AudioMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("unsupported input media");
		return -ENOSYS;
	}

	if ((mMediaId != 0) && (mMediaId != m->getId()))
		return -EPERM;
	if (mLastAddedMedia != nullptr)
		return -EBUSY;
	if ((!mRunning) || (mState != State::STARTED))
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

	auto *channel = dynamic_cast<AudioChannel *>(getInputChannel(m));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	struct mbuf_audio_frame_queue_args args = {};
	args.filter = &queueFilter;
	args.filter_userdata = this;
	args.max_frames = ALSA_RENDERER_QUEUE_MAX_FRAMES;
	try {
		mInputQueue = mbuf::Queue::createWithArgs(&args);
	} catch (const std::bad_alloc &) {
		Sink::unlock();
		PDRAW_LOGE("queue allocation failed");
		return -ENOMEM;
	}
	channel->setQueue(this, mInputQueue.get());

	res = mInputQueue->attachToLoop(mSession->getLoop(), renderCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("queue::attachToLoop", -res);
		goto error;
	}

	mLastAddedMedia = m;
	mCurrentMediaId = m->getId();

	m->fillMediaInfo(&mMediaInfo);

	Sink::unlock();

	res = startAlsa();
	if (res < 0) {
		PDRAW_LOG_ERRNO("startAlsa", -res);
		goto error;
	}

	{
		std::scoped_lock lock(mListenerMutex);
		if (mRendererListener) {
			mRendererListener->onAudioRendererMediaAdded(
				mSession, mRenderer, &mMediaInfo);
		}
	}

	return 0;

error:
	removeInputMedia(media);
	return res;
}


int AlsaAudioRenderer::removeInputMedia(Media *media)
{
	int ret;
	int err;

	Sink::lock();

	if (mLastAddedMedia == media) {
		mLastAddedMedia = nullptr;
		mCurrentMediaId = 0;
		{
			std::scoped_lock lock(mListenerMutex);
			if (mRendererListener) {
				mRendererListener->onAudioRendererMediaRemoved(
					mSession, mRenderer, &mMediaInfo);
			}
		}

		Media::cleanupMediaInfo(&mMediaInfo);
	}

	const auto *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	ret = Sink::removeInputMedia(media);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -ret);
		return ret;
	}

	Sink::unlock();

	if (mInputQueue) {
		err = mInputQueue->detachFromLoop(mSession->getLoop());
		if (err < 0)
			PDRAW_LOG_ERRNO("queue::detachFromLoop", -err);
		err = mInputQueue->flush();
		if (err < 0)
			PDRAW_LOG_ERRNO("queue::flush", -err);
		mInputQueue.reset();
	}

	err = stopAlsa();
	if (err < 0)
		PDRAW_LOG_ERRNO("stopAlsa", -err);

	return 0;
}


int AlsaAudioRenderer::removeInputMedias()
{
	int ret;
	int inputMediaCount;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (int i = inputMediaCount - 1; i >= 0; i--) {
		auto *media = dynamic_cast<AudioMedia *>(getInputMedia(i));
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


void AlsaAudioRenderer::idleRenewMedia()
{
	if (mLastAddedMedia != nullptr)
		removeInputMedia(mLastAddedMedia);
	mSession->addMediaToAudioRenderer(mMediaId, this);
}


void AlsaAudioRenderer::completeStop()
{
	int ret;

	if (mState == State::STOPPED)
		return;

	if (mWatchdogTimer != nullptr)
		(void)mWatchdogTimer->clear();

	ret = removeInputMedias();
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedias", -ret);

	setState(State::STOPPED);
}


void AlsaAudioRenderer::renderCb([[maybe_unused]] struct pomp_evt *event,
				 void *userdata)
{

	auto *self = static_cast<AlsaAudioRenderer *>(userdata);
	/* We have a new frame */

	if (!self->mEos) {
		bool expected = true;
		if (self->mWatchdogTriggered.compare_exchange_strong(expected,
								     false)) {
			PDRAW_LOGI("new frame to render");
		}
		int err = self->mWatchdogTimer->set(
			1000 * ALSA_RENDERER_WATCHDOG_TIME_S);
		if (err != 0)
			PDRAW_LOG_ERRNO("pomp::Timer::set", -err);
	}
	self->render();
}


int AlsaAudioRenderer::render()
{
	struct mbuf_audio_frame *frame = nullptr;
	mbuf::Queue *queue = nullptr;
	int ret = 0;
	int err;
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

	count = queue->getCount();
	if (count < 1) {
		err = mSession->getPompLoop()->idleAdd(&mIdleDrainHandler,
						       this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
		ret = -EAGAIN;
		PDRAW_LOGW("no frame in queue");
		goto out;
	}

	ret = queue->popFrame(&frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("queue::popFrame", -ret);
		goto out;
	}

	/* Don't clobber a pending drain (FLUSHING): unconditionally resetting
	 * to UNFLUSHED here would make asyncDrainDone() never fire. */
	if (getFlushingState() != FlushingState::FLUSHING) {
		setFlushingState(FlushingState::UNFLUSHED);
	} else if (queue->getCount() == 0) {
		err = mSession->getPompLoop()->idleAdd(&mIdleDrainHandler,
						       this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
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
			snd_pcm_sframes_t frames =
				snd_pcm_writei(mHandle, data, mSampleCount);
			if (frames < 0) {
				if (frames == -EPIPE) {
					snd_pcm_prepare(mHandle);
					continue;
				}
				ret = static_cast<int>(frames);
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
