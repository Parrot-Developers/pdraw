/**
 * Parrot Drones Audio and Video Vector library
 * ALSA source
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

#define ULOG_TAG pdraw_alsa_source
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <time.h>

#include "pdraw_alsa_source.hpp"
#include "pdraw_session.hpp"

#ifdef PDRAW_USE_ALSA
constexpr const char *PDRAW_ALSA_SOURCE_ANCILLARY_KEY_INPUT_TIME =
	"pdraw.alsasource.input_time";

constexpr size_t DEFAULT_IN_POOL_SIZE = 100;
#endif

namespace Pdraw {


#ifdef PDRAW_USE_ALSA

/* TODO: Channel::DownstreamEvent::TIMEOUT? */

AlsaSource::AlsaSource(Session *session,
		       Element::Listener *elementListener,
		       Source::Listener *sourceListener,
		       IPdraw::IAlsaSource::Listener *listener,
		       AlsaSourceWrapper *wrapper,
		       const struct pdraw_alsa_source_params *params) :
		SourceElement(session,
			      elementListener,
			      wrapper,
			      1,
			      sourceListener),
		mAlsaSource(wrapper), mAlsaSourceListener(listener)
{
	Element::setClassName(__func__);

	mParams = *params;
	if (params->address != nullptr) {
		mAddress = params->address;
		mParams.address = mAddress.c_str();
	} else {
		mParams.address = nullptr;
	}
	if (mParams.sample_count == 0)
		mParams.sample_count = ALSA_AUDIO_DEFAULT_SAMPLE_COUNT;

	setState(State::CREATED);
}


AlsaSource::~AlsaSource()
{
	int err;

	/* Make sure listener functions will no longer be called */
	mAlsaSourceListener = nullptr;

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (mHandle != nullptr) {
		err = snd_pcm_close(mHandle);
		if (err < 0)
			PDRAW_LOG_ERRNO("snd_pcm_close", -err);
	}

	if (mHwParams != nullptr)
		snd_pcm_hw_params_free(mHwParams);

	if (mState == State::STARTED)
		PDRAW_LOGW("ALSA source is still running");

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int AlsaSource::start()
{
	int ret;
	snd_pcm_format_t format;

	if ((mState == State::STARTED) || (mState == State::STARTING))
		return 0;
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(State::STARTING);

	if (mTimer == nullptr) {
		mTimer = pomp_timer_new(mSession->getLoop(), timerCb, this);
		if (mTimer == nullptr) {
			ret = -ENOMEM;
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}

	/* Check parameters */
	if (!adef_is_format_valid(&mParams.audio.format)) {
		ret = -EINVAL;
		PDRAW_LOGE("invalid audio format: " ADEF_FORMAT_TO_STR_FMT,
			   ADEF_FORMAT_TO_STR_ARG(&mParams.audio.format));
		goto error;
	}

	format = AlsaAudio::adefFormatToAlsa(&mParams.audio.format);
	if (format == SND_PCM_FORMAT_UNKNOWN) {
		ret = -EINVAL;
		PDRAW_LOGE("unsupported audio format: " ADEF_FORMAT_TO_STR_FMT,
			   ADEF_FORMAT_TO_STR_ARG(&mParams.audio.format));
		goto error;
	}

	mFrameSize = snd_pcm_format_width(format) / 8 *
		     mParams.audio.format.channel_count;

	if (mHandle == nullptr) {
		ret = snd_pcm_open(&mHandle,
				   mParams.address,
				   SND_PCM_STREAM_CAPTURE,
				   SND_PCM_NONBLOCK);
		if (ret < 0) {
			PDRAW_LOGE("snd_pcm_open'%s'(%s)",
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
		mParams.audio.format.pcm.interleaved
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

	ret = snd_pcm_hw_params_set_rate(
		mHandle, mHwParams, mParams.audio.format.sample_rate, 0);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_rate(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params_set_period_size(
		mHandle, mHwParams, mParams.sample_count, 0);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("snd_pcm_hw_params_set_period_size", -ret);
		goto error;
	}

	ret = snd_pcm_hw_params_set_channels(
		mHandle, mHwParams, mParams.audio.format.channel_count);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params_set_channels(%s)",
			   snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_hw_params(mHandle, mHwParams);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_hw_params(%s)", snd_strerror(ret));
		goto error;
	}

	ret = snd_pcm_prepare(mHandle);
	if (ret < 0) {
		PDRAW_LOGE("snd_pcm_prepare(%s)", snd_strerror(ret));
		goto error;
	}

	setState(State::STARTED);

	ret = setupMedia();
	if (ret < 0) {
		PDRAW_LOG_ERRNO("setupMedia", -ret);
		goto error;
	}

	return 0;

error:
	(void)stop();
	return ret;
}


int AlsaSource::stop()
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if ((mState != State::STARTED) && (mState != State::STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	setState(State::STOPPING);

	mReady = false;

	/* Make sure listener functions will no longer be called */
	mAlsaSourceListener = nullptr;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	if (mRunning)
		mRunning = false;

	if (mHandle != nullptr) {
		int err = snd_pcm_drop(mHandle);
		if (err < 0)
			PDRAW_LOG_ERRNO("snd_pcm_drop", -err);
		err = snd_pcm_close(mHandle);
		if (err < 0)
			PDRAW_LOG_ERRNO("snd_pcm_close", -err);
		mHandle = nullptr;
	}

	if (mHwParams != nullptr) {
		snd_pcm_hw_params_free(mHwParams);
		mHwParams = nullptr;
	}

	/* Flush everything */
	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int AlsaSource::tryStop()
{
	int pendingCount;

	if (mState != State::STOPPING)
		return 0;

	pendingCount = teardownChannels();

	if (pendingCount == 0)
		completeStop();

	return 0;
}


void AlsaSource::completeStop()
{
	int err;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr)
		goto exit;

	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	if (mPool != nullptr) {
		err = mbuf_pool_destroy(mPool);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_pool_destroy", -err);
	}

	(void)destroyMedia();

exit:
	Source::unlock();

	if (mTimer != nullptr) {
		err = pomp_timer_clear(mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mTimer = nullptr;
	}

	setState(State::STOPPED);
}


void AlsaSource::playResponse()
{
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPlayResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void AlsaSource::pauseResponse()
{
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPauseResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void AlsaSource::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING) {
		completeStop();
		return;
	}

	if (mOutputMediaChanging) {
		Source::lock();

		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
		if (outputChannelCount > 0) {
			Source::unlock();
			return;
		}

		Source::unlock();

		(void)destroyMedia();
		mOutputMediaChanging = false;
		(void)setupMedia();
	}
}


void AlsaSource::idleCompleteFlush(void *userdata)
{
	auto *self = static_cast<AlsaSource *>(userdata);
	self->completeFlush();
}


int AlsaSource::flush(bool discard)
{
	int ret;
	int err;
	bool channelFound = false;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("alsa source is already flushed, nothing to do");
		ret = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleCompleteFlush, this, this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (mFlushDiscard)
				err = outputChannel->flush();
			else
				err = outputChannel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO(
					"channel->%s (channel index=%u)",
					-err,
					mFlushDiscard ? "flush" : "drain",
					i);
			} else {
				channelFound = true;
			}
		}
	}
	Source::unlock();

	if (!channelFound)
		completeFlush();

	return 0;
}


void AlsaSource::completeFlush()
{
	bool pending = false;

	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			const Channel *outputChannel =
				getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (outputChannel->isFlushPending() ||
			    outputChannel->isDrainPending()) {
				pending = true;
				break;
			}
		}
	}
	Source::unlock();

	if (pending)
		return;

	setFlushingState(FlushingState::FLUSHED);

	if ((mState != State::STOPPING) && (!mOutputMediaChanging))
		return;

	int pendingCount = teardownChannels();
	if (pendingCount == 0) {
		if (mState == State::STOPPING) {
			completeStop();
		} else if (mOutputMediaChanging) {
			(void)destroyMedia();
			mOutputMediaChanging = false;
			(void)setupMedia();
		}
	}
}


void AlsaSource::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


void AlsaSource::onChannelDrained(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel drained media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();


	if (mPausePending) {
		pauseResponse();
	}
}


bool AlsaSource::isReadyToPlay() const
{
	return mReady;
}


bool AlsaSource::isPaused() const
{
	return !mRunning;
}


int AlsaSource::play()
{
	int ret;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	if (!mReady) {
		PDRAW_LOGE("%s: not ready to play", __func__);
		return -EPROTO;
	}

	if (mRunning)
		return 0;

	ret = snd_pcm_start(mHandle);
	if (ret < 0) {
		PDRAW_LOG_ERRNO(
			"snd_pcm_start failed (%s)", -ret, snd_strerror(ret));
		return ret;
	}

	if (mTimer != nullptr) {
		uint32_t timerFreqMs =
			(1000000 / mParams.audio.format.sample_rate *
			 mParams.sample_count) /
			1000;
		int err = pomp_timer_set_periodic(
			mTimer, timerFreqMs, timerFreqMs);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}

	mRunning = true;
	return 0;
}


int AlsaSource::pause()
{
	int ret;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	if (mPausePending)
		return -EALREADY;

	if (mReady && mRunning) {
		if (mTimer != nullptr) {
			int err = pomp_timer_clear(mTimer);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		}

		ret = snd_pcm_drop(mHandle);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("snd_pcm_drop failed (%s)",
					-ret,
					snd_strerror(ret));
			return ret;
		}
		mRunning = false;
	}

	/* Drain */
	ret = drain();
	if (ret < 0 && ret != -EALREADY) {
		PDRAW_LOG_ERRNO("drain", -ret);
		return ret;
	}

	mPausePending = true;

	return 0;
}


int AlsaSource::getCapabilities(const std::string &address,
				struct pdraw_alsa_source_caps *caps)
{
	int ret;
	unsigned int val;
	snd_pcm_t *handle;
	snd_pcm_hw_params_t *hwParams = nullptr;
	struct pdraw_alsa_source_caps tmpCaps = {};

	if (caps == nullptr)
		return -EINVAL;

	ret = snd_pcm_open(&handle,
			   address.c_str(),
			   SND_PCM_STREAM_CAPTURE,
			   SND_PCM_NONBLOCK);
	if (ret < 0) {
		ULOGE("snd_pcm_open:'%s'(%s)",
		      address.c_str(),
		      snd_strerror(ret));
		goto out;
	}

	ret = snd_pcm_hw_params_malloc(&hwParams);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_malloc(%s)", snd_strerror(ret));
		goto out;
	}

	ret = snd_pcm_hw_params_any(handle, hwParams);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_any(%s)", snd_strerror(ret));
		goto out;
	}

	ret = snd_pcm_hw_params_get_channels_min(hwParams, &val);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_get_channels_min(%s)",
		      snd_strerror(ret));
		goto out;
	}
	tmpCaps.channel_count.min =
		static_cast<uint8_t>(MIN(val, (unsigned int)UINT8_MAX));

	ret = snd_pcm_hw_params_get_channels_max(hwParams, &val);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_get_channels_max(%s)",
		      snd_strerror(ret));
		goto out;
	}
	tmpCaps.channel_count.max =
		static_cast<uint8_t>(MIN(val, (unsigned int)UINT8_MAX));

	ret = snd_pcm_hw_params_get_rate_min(hwParams, &val, nullptr);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_get_rate_min(%s)", snd_strerror(ret));
		goto out;
	}
	tmpCaps.sample_rate.min =
		static_cast<uint16_t>(MIN(val, (unsigned int)UINT16_MAX));

	ret = snd_pcm_hw_params_get_rate_max(hwParams, &val, nullptr);
	if (ret < 0) {
		ULOGE("snd_pcm_hw_params_get_rate_max(%s)", snd_strerror(ret));
		goto out;
	}
	tmpCaps.sample_rate.max =
		static_cast<uint16_t>(MIN(val, (unsigned int)UINT16_MAX));

	*caps = tmpCaps;
	ret = 0;

out:
	if (handle != nullptr) {
		int err = snd_pcm_close(handle);
		if (err < 0)
			ULOG_ERRNO("snd_pcm_close", -err);
		handle = nullptr;
	}

	if (hwParams != nullptr) {
		snd_pcm_hw_params_free(hwParams);
		hwParams = nullptr;
	}
	return ret;
}


int AlsaSource::processFrame(struct mbuf_mem *mem, size_t len)
{
	int ret;
	int err;
	struct mbuf_audio_frame *mbufFrame = nullptr;
	struct adef_frame frameInfo = {};
	AudioMedia::Frame out_meta{};
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	unsigned int outputChannelCount;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (mFirstFrame) {
		mFirstFrame = false;
		err = sendDownstreamEvent(mOutputMedia.get(),
					  Channel::DownstreamEvent::SOS);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}

	if (mFrameIndex == 0) {
		/* Save the first frame timestamp as the base offset */
		time_get_monotonic(&ts);
		time_timespec_to_us(&ts, &curTime);

		mFirstTimestamp = curTime;
	}

	mCurTimestamp += 1000000ULL * mParams.sample_count /
			 mOutputMedia->format.sample_rate;

	frameInfo.format = mOutputMedia->format;
	frameInfo.info.timestamp = mCurTimestamp;
	frameInfo.info.timescale = mTimescale;
	frameInfo.info.capture_timestamp = mCurTimestamp + mFirstTimestamp;
	frameInfo.info.index = mFrameIndex++;

	/* Check the frame timestamp */
	if ((mLastTimestamp != UINT64_MAX) &&
	    (frameInfo.info.timestamp <= mLastTimestamp)) {
		PDRAW_LOGE("%s: non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   __func__,
			   frameInfo.info.timestamp,
			   mLastTimestamp);
		ret = -EINVAL;
		goto out;
	}

	/* Save frame timestamp as last_timestamp */
	mLastTimestamp = frameInfo.info.timestamp;

	/* Create the raw video frame */
	ret = mbuf_audio_frame_new(&frameInfo, &mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_new", -ret);
		goto out;
	}
	ret = mbuf_audio_frame_set_buffer(mbufFrame, mem, 0, len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_set_buffer", -ret);
		goto out;
	}

	/* Set the ancillary data */
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	err = mbuf_audio_frame_add_ancillary_buffer(
		mbufFrame,
		PDRAW_ALSA_SOURCE_ANCILLARY_KEY_INPUT_TIME,
		&curTime,
		sizeof(curTime));
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -err);
	}

	out_meta.ntpTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpUnskewedTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpRawTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpRawUnskewedTimestamp = frameInfo.info.capture_timestamp;
	out_meta.demuxOutputTimestamp = curTime;
	out_meta.playTimestamp = frameInfo.info.capture_timestamp;
	out_meta.captureTimestamp = frameInfo.info.capture_timestamp;
	out_meta.localTimestamp = frameInfo.info.capture_timestamp;

	ret = mbuf_audio_frame_add_ancillary_buffer(
		mbufFrame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		goto out;
	}

	/* Notify that a frame is ready */
	if (mAlsaSourceListener != nullptr) {
		mAlsaSourceListener->alsaSourceFrameReady(
			mSession, getAlsaSource(), mbufFrame);
	}

	/* Finalize the frame */
	ret = mbuf_audio_frame_finalize(mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_finalize", -ret);
		goto out;
	}

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia.get(), i);
		auto *channel = dynamic_cast<AudioChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(mbufFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
		else
			setFlushingState(FlushingState::UNFLUSHED);
	}

out:
	Source::unlock();

	if (mbufFrame != nullptr) {
		err = mbuf_audio_frame_unref(mbufFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
	}

	return ret;
}


int AlsaSource::setupMedia()
{
	if (mState == State::STOPPED)
		return 0;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMediaChanging) {
		PDRAW_LOGI("%s: output media is already pending change",
			   __func__);
		Source::unlock();
		return 0;
	}

	if (mOutputMedia != nullptr) {
		mOutputMediaChanging = true;
		int ret = flush();
		if (ret < 0 && ret != -EALREADY)
			PDRAW_LOG_ERRNO("flush", -ret);
		else
			ret = 0;
		Source::unlock();
		return ret;
	}

	Source::unlock();

	return createMedia();
}


/**
 * Alsa source listener calls from idle functions
 */
void AlsaSource::callOnMediaAdded(void *userdata)
{
	auto *self = static_cast<AlsaSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGW("%s: output media not found", __func__);
		return;
	}

	if (self->Source::mListener) {
		self->Source::mListener->onOutputMediaAdded(
			self, self->mOutputMedia.get(), self->getAlsaSource());
	}
}


/* Listener call from an idle function */
void AlsaSource::callPlayResponse(void *userdata)
{
	auto *self = static_cast<AlsaSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mAlsaSourceListener != nullptr) {
		self->mAlsaSourceListener->alsaSourcePlayResponse(
			self->mSession, self->getAlsaSource());
	}
}


/* Listener call from an idle function */
void AlsaSource::callPauseResponse(void *userdata)
{
	auto *self = static_cast<AlsaSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	self->mPausePending = false;

	if (self->mAlsaSourceListener != nullptr) {
		self->mAlsaSourceListener->alsaSourcePauseResponse(
			self->mSession, self->getAlsaSource());
	}
}


int AlsaSource::createMedia()
{
	int ret;
	int err;
	std::string path;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMedia != nullptr) {
		Source::unlock();
		PDRAW_LOGE("already existing output media");
		return -EALREADY;
	}

	try {
		mOutputMedia = make_unique<AudioMedia>(mSession);
	} catch (const std::bad_alloc &) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	path = Element::getName() + "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = mParams.audio.format;

	Source::unlock();

	PDRAW_LOGI("output media created");

	if (Source::mListener) {
		/* Call the onMediaAdded listener function from a fresh
		 * callstack as a direct call could ultimately be blocking
		 * in a pdraw-backend application calling another pdraw-backend
		 * function from the onMediaAdded listener function */
		err = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), callOnMediaAdded, this, this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

	if (!mReady) {
		mReady = true;
		if (mAlsaSourceListener != nullptr) {
			mAlsaSourceListener->alsaSourceReadyToPlay(
				mSession,
				getAlsaSource(),
				mReady,
				PDRAW_ALSA_SOURCE_EOS_REASON_NONE);
		}
	}

	return 0;
}


int AlsaSource::destroyMedia()
{
	int ret;

	if ((mState != State::STARTED) && (mState != State::STOPPING) &&
	    (mState != State::STOPPED)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGW("output media already destroyed");
		return 0;
	}

	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia.get(), getAlsaSource());
	}
	ret = removeOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
		return ret;
	}

	mOutputMedia.reset();

	Source::unlock();

	PDRAW_LOGI("output media destroyed");

	return 0;
}


int AlsaSource::teardownChannels()
{
	unsigned int pendingCount = 0;

	Source::lock();

	if (mOutputMedia == nullptr) {
		Source::unlock();
		return 0;
	}

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */

	int outputChannelCount = getOutputChannelCount(mOutputMedia.get());

	for (int i = outputChannelCount - 1; i >= 0; i--) {
		Channel *channel = getOutputChannel(mOutputMedia.get(), i);
		if (channel == nullptr) {
			PDRAW_LOGW("failed to get channel at index %d", i);
			continue;
		}
		int err = channel->teardown();
		if (err < 0)
			PDRAW_LOG_ERRNO("channel::teardown", -err);
		else
			pendingCount++;
	}

	Source::unlock();

	return pendingCount;
}


const char *AlsaSource::getSourceName() const
{
	if (!mAddress.empty())
		return mAddress.c_str();
	else
		return "<NULL>";
}


int AlsaSource::readFrame()
{
	int ret;
	int err;
	struct mbuf_mem *mem = nullptr;
	void *data;
	size_t cap;
	ssize_t ret1;

	/* Create mPool if needed */
	if (mPool == nullptr) {
		ret = mbuf_pool_new(mbuf_mem_generic_impl,
				    mFrameSize * mParams.sample_count,
				    DEFAULT_IN_POOL_SIZE,
				    MBUF_POOL_NO_GROW,
				    0,
				    getName().c_str(),
				    &mPool);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_pool_new", -ret);
			goto unref;
		}
	}

	ret1 = snd_pcm_avail(mHandle);
	if (ret1 < 0) {
		ret = static_cast<int>(ret1);
		if (ret == -EPIPE)
			goto recover_xrun;
		PDRAW_LOG_ERRNO(
			"snd_pcm_avail failed (%s)", -ret, snd_strerror(ret));
		goto unref;
	}

	/* Not enough samples available, retry later. */
	if (ret1 < (int)mParams.sample_count) {
		ret = -EAGAIN;
		goto unref;
	}

	/* Copy the frame */
	ret = mbuf_pool_get(mPool, &mem);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_pool_get", -ret);
		goto unref;
	}
	ret = mbuf_mem_get_data(mem, &data, &cap);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto unref;
	}
	if (cap < mFrameSize * mParams.sample_count) {
		ret = -ENOBUFS;
		PDRAW_LOGE("insufficient buffer space");
		goto unref;
	}

	if (mParams.audio.format.pcm.interleaved) {
		ret1 = snd_pcm_readi(mHandle, data, mParams.sample_count);
		if (ret1 != mParams.sample_count) {
			if (ret1 < 0) {
				ret = static_cast<int>(ret1);
				if (ret == -EPIPE)
					goto recover_xrun;
				PDRAW_LOG_ERRNO("snd_pcm_readi failed (%s)",
						-ret,
						snd_strerror(ret));
				goto unref;
			} else {
				/* TODO: handle ret1 > 0 but != sample_count */
			}
		}
	} else {
		ret = -ENOSYS;
		PDRAW_LOGE("non-interleaved format is unsupported");
		goto unref;
	}

	ret = processFrame(mem, cap);
	if (ret < 0)
		goto unref;

	ret = 0;

	goto unref;

recover_xrun:
	PDRAW_LOGW("overrun occured");

	err = snd_pcm_recover(mHandle, ret, 1);
	if (err == 0) {
		/*  Recovered successfully */
		ret = snd_pcm_start(mHandle);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("snd_pcm_start failed (%s)",
					-ret,
					snd_strerror(ret));
			goto unref;
		}
		ret = -EAGAIN;
	} else {
		PDRAW_LOG_ERRNO(
			"cannot recover from overrun: "
			"(snd_pcm_recover failed (%s))",
			-err,
			snd_strerror(err));
	}

unref:
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
	}

	return ret;
}


/* Called on the loop thread */
void AlsaSource::timerCb(struct pomp_timer *timer, void *userdata)
{
	PDRAW_UNUSED(timer);

	int err;
	auto *self = static_cast<AlsaSource *>(userdata);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}
	if (!self->mReady) {
		PDRAW_LOGE("%s: timer called while not ready", __func__);
		return;
	}
	if (!self->mRunning) {
		PDRAW_LOGE("%s: timer called while not running", __func__);
		return;
	}

	while ((err = self->readFrame()) != -EAGAIN) {
		if (err < 0) {
			PDRAW_LOG_ERRNO("readFrame", -err);
			goto unrecoverable_error;
		}
	}

	return;

unrecoverable_error:
	self->mReady = false;

	if (self->mTimer != nullptr) {
		err = pomp_timer_clear(self->mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	if (self->mAlsaSourceListener != nullptr) {
		self->mAlsaSourceListener->alsaSourceReadyToPlay(
			self->mSession,
			self->getAlsaSource(),
			self->mReady,
			PDRAW_ALSA_SOURCE_EOS_REASON_UNRECOVERABLE_ERROR);
	}
}

#endif /* PDRAW_USE_ALSA */


AlsaSourceWrapper::AlsaSourceWrapper(
	Session *session,
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener)
#ifdef PDRAW_USE_ALSA
		:
		ElementWrapper(new Pdraw::AlsaSource(session,
						     session,
						     session,
						     listener,
						     this,
						     params)),
		mSource(static_cast<Pdraw::AlsaSource *>(mElement))
#else
		:
		ElementWrapper(nullptr)
#endif
{
#ifndef PDRAW_USE_ALSA
	PDRAW_UNUSED(session);
	PDRAW_UNUSED(params);
	PDRAW_UNUSED(listener);

	ULOGE("no ALSA source implementation found");
#endif
}


AlsaSourceWrapper::~AlsaSourceWrapper()
{
#ifdef PDRAW_USE_ALSA
	if (isElementStopped())
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
#endif
}


bool AlsaSourceWrapper::isReadyToPlay()
{
#ifdef PDRAW_USE_ALSA
	if (isElementStopped())
		return false;
	return mSource->isReadyToPlay();
#else
	return false;
#endif
}


bool AlsaSourceWrapper::isPaused()
{
#ifdef PDRAW_USE_ALSA
	if (isElementStopped())
		return false;
	return mSource->isPaused();
#else
	return false;
#endif
}


int AlsaSourceWrapper::play()
{
#ifdef PDRAW_USE_ALSA
	if (isElementStopped())
		return -EPROTO;
	return mSource->play();
#else
	return -ENOSYS;
#endif
}


int AlsaSourceWrapper::pause()
{
#ifdef PDRAW_USE_ALSA
	if (isElementStopped())
		return -EPROTO;
	return mSource->pause();
#else
	return -ENOSYS;
#endif
}

} /* namespace Pdraw */
