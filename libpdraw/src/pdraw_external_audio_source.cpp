/**
 * Parrot Drones Audio and Video Vector library
 * Application external audio source
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

#define ULOG_TAG pdraw_external_audio_source
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_external_audio_source.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#define PDRAW_EXT_AUDIO_SOURCE_ANCILLARY_KEY_INPUT_TIME                        \
	"pdraw.audiosource.input_time"

namespace Pdraw {


ExternalAudioSource::ExternalAudioSource(
	Session *session,
	Element::Listener *elementListener,
	Source::Listener *sourceListener,
	IPdraw::IAudioSource::Listener *listener,
	AudioSourceWrapper *wrapper,
	const struct pdraw_audio_source_params *params) :
		SourceElement(session,
			      elementListener,
			      wrapper,
			      1,
			      sourceListener),
		mAudioSource(wrapper), mAudioSourceListener(listener),
		mParams(*params), mFrameQueue(nullptr), mOutputMedia(nullptr),
		mLastTimestamp(UINT64_MAX)
{
	Element::setClassName(__func__);

	setState(State::CREATED);
}


ExternalAudioSource::~ExternalAudioSource(void)
{
	int err;

	if (mState == State::STARTED)
		PDRAW_LOGW("audio source is still running");

	/* Make sure listener functions will no longer be called */
	mAudioSourceListener = nullptr;

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (mFrameQueue != nullptr) {
		err = mbuf_audio_frame_queue_flush(mFrameQueue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
		}
		err = removeQueueEvtFromLoop(mFrameQueue, mSession->getLoop());
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
		err = mbuf_audio_frame_queue_destroy(mFrameQueue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
		}
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int ExternalAudioSource::start(void)
{
	int ret;
	struct pomp_evt *evt = nullptr;
	std::string path;

	if ((mState == State::STARTED) || (mState == State::STARTING))
		return 0;
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(State::STARTING);

	mbuf_audio_frame_queue_args args = {
		.filter = &ExternalAudioSource::inputFilter,
		.filter_userdata = this,
		.max_frames = 0,
	};
	ret = mbuf_audio_frame_queue_new_with_args(&args, &mFrameQueue);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_new_with_args", -ret);
		goto error;
	}

	ret = mbuf_audio_frame_queue_get_event(mFrameQueue, &evt);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(
		evt, mSession->getLoop(), &queueEventCb, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	setState(State::STARTED);

	Source::lock();

	mOutputMedia = new AudioMedia(mSession);
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		ret = -ENOMEM;
		goto error;
	}
	path = Element::getName() + "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		goto error;
	}

	mOutputMedia->format = mParams.audio.format;
	mOutputMedia->playbackType = mParams.playback_type;
	mOutputMedia->duration = mParams.duration;

	Source::unlock();

	if (Source::mListener) {
		/* Call the onMediaAdded listener function from a fresh
		 * callstack as a direct call could ultimately be blocking
		 * in a pdraw-backend application calling another pdraw-backend
		 * function from the onMediaAdded listener function */
		ret = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), callOnMediaAdded, this, this);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
			goto error;
		}
	}

	return 0;

error:
	(void)stop();
	return ret;
}


int ExternalAudioSource::stop(void)
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	setState(State::STOPPING);

	/* Make sure listener functions will no longer be called */
	mAudioSourceListener = nullptr;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	/* Flush everything */
	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


void ExternalAudioSource::idleCompleteFlush(void *userdata)
{
	ExternalAudioSource *self = (ExternalAudioSource *)userdata;
	self->completeFlush();
}


int ExternalAudioSource::tryStop(void)
{
	int ret, err;
	int completeStopPendingCount;

	if (mState != State::STOPPING)
		return 0;

	if (mFrameQueue != nullptr) {
		err = removeQueueEvtFromLoop(mFrameQueue, mSession->getLoop());
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
		err = mbuf_audio_frame_queue_destroy(mFrameQueue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
		mFrameQueue = nullptr;
	}

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	completeStopPendingCount = 0;
	Source::lock();
	if (mOutputMedia != nullptr) {
		int outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (int i = outputChannelCount - 1; i >= 0; i--) {
			Channel *channel = getOutputChannel(mOutputMedia, i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->teardown", -ret);
			else
				completeStopPendingCount++;
		}
	}
	Source::unlock();

	if (completeStopPendingCount == 0)
		completeStop();

	return 0;
}


void ExternalAudioSource::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr)
		goto exit;

	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia, getAudioSource());
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = nullptr;
	}

exit:
	Source::unlock();

	setState(State::STOPPED);
}


void ExternalAudioSource::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING)
		completeStop();
}


int ExternalAudioSource::removeQueueEvtFromLoop(
	struct mbuf_audio_frame_queue *queue,
	struct pomp_loop *loop)
{
	int res;
	struct pomp_evt *evt = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(queue == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);

	res = mbuf_audio_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -res);
		return res;
	}

	if (!pomp_evt_is_attached(evt, loop))
		return 0;

	res = pomp_evt_detach_from_loop(evt, loop);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -res);
		return res;
	}

	return 0;
}


int ExternalAudioSource::flush(bool discard)
{
	int ret, err;
	bool channelFound = false;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("audio source is already %s, nothing to do",
			   discard ? "flushed" : "drained");
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

	if ((mFrameQueue != nullptr) && !discard) {
		/* Drain the queue */
		err = process();
		if (err < 0)
			PDRAW_LOG_ERRNO("process", -err);
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia);
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
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

	if ((mFrameQueue != nullptr) && mFlushDiscard) {
		/* Flush the queue */
		err = mbuf_audio_frame_queue_flush(mFrameQueue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
	}

	if (!channelFound)
		completeFlush();

	return 0;
}


void ExternalAudioSource::completeFlush(void)
{
	bool pending = false;
	int err = 0;

	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia);
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *outputChannel =
				getOutputChannel(mOutputMedia, i);
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

	if (mState != State::STOPPING) {
		/* Signal to the application that flushing is done */
		err = pomp_loop_idle_add_with_cookie(mSession->getLoop(),
						     callAudioSourceFlushed,
						     this,
						     this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	} else {
		setFlushingState(FlushingState::FLUSHED);
	}

	tryStop();
}


void ExternalAudioSource::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
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


void ExternalAudioSource::onChannelDrained(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel drained media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


int ExternalAudioSource::process(void)
{
	int ret, err;
	struct mbuf_audio_frame *frame = nullptr;

	if (mFrameQueue == nullptr) {
		ret = -ENODEV;
		PDRAW_LOG_ERRNO("%s: invalid queue", -ret, __func__);
		return ret;
	}

	do {
		ret = mbuf_audio_frame_queue_pop(mFrameQueue, &frame);
		if (ret < 0) {
			if (ret != -EAGAIN) {
				PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_pop",
						-ret);
			}
			continue;
		}

		(void)processFrame(frame);

		err = mbuf_audio_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
	} while (ret == 0);

	return 0;
}


void ExternalAudioSource::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	ExternalAudioSource *self =
		reinterpret_cast<ExternalAudioSource *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}
	if (self->mFrameQueue == nullptr) {
		PDRAW_LOGE("%s: invalid queue", __func__);
		return;
	}
	if (self->getFlushingState() == FlushingState::FLUSHING) {
		PDRAW_LOGI("%s: flush pending, discarding queue event",
			   __func__);
		return;
	}

	(void)self->process();
}


bool ExternalAudioSource::inputFilter(struct mbuf_audio_frame *frame,
				      void *userdata)
{
	ExternalAudioSource *self =
		reinterpret_cast<ExternalAudioSource *>(userdata);
	int ret, err;
	bool accept = true;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};
	struct adef_frame frame_info = {};

	PDRAW_LOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, false);
	PDRAW_LOG_ERRNO_RETURN_VAL_IF(frame == nullptr, EINVAL, false);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return false;
	}

	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		accept = false;
		goto out;
	}

	ret = mbuf_audio_frame_get_frame_info(frame, &frame_info);
	if (ret < 0) {
		accept = false;
		goto out;
	}

	/* Check the frame format */
	if (!adef_format_intersect(
		    &frame_info.format, &self->mOutputMedia->format, 1)) {
		PDRAW_LOGE(
			"%s: unsupported format:"
			" " ADEF_FORMAT_TO_STR_FMT,
			__func__,
			ADEF_FORMAT_TO_STR_ARG(&frame_info.format));
		accept = false;
		goto out;
	}

	/* Check the frame timestamp */
	if (frame_info.info.timestamp <= self->mLastTimestamp &&
	    self->mLastTimestamp != UINT64_MAX) {
		PDRAW_LOGE("%s: non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   __func__,
			   frame_info.info.timestamp,
			   self->mLastTimestamp);
		accept = false;
		goto out;
	}

	/* Save frame timestamp as last_timestamp */
	self->mLastTimestamp = frame_info.info.timestamp;

	/* Set the input time ancillary data */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_audio_frame_add_ancillary_buffer(
		frame,
		PDRAW_EXT_AUDIO_SOURCE_ANCILLARY_KEY_INPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -err);
	}

out:
	self->Source::unlock();

	return accept;
}


int ExternalAudioSource::processFrame(struct mbuf_audio_frame *frame)
{
	int ret, err;
	struct adef_frame info = {};
	AudioMedia::Frame out_meta = {};
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		ret = -EPROTO;
		goto out;
	}

	ret = mbuf_audio_frame_get_frame_info(frame, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		goto out;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	out_meta.ntpTimestamp =
		(info.info.timestamp * 1000000 + info.info.timescale / 2) /
		info.info.timescale;
	out_meta.ntpUnskewedTimestamp = out_meta.ntpTimestamp;
	out_meta.ntpRawTimestamp = out_meta.ntpTimestamp;
	out_meta.ntpRawUnskewedTimestamp = out_meta.ntpTimestamp;
	out_meta.demuxOutputTimestamp = curTime;
	out_meta.playTimestamp = info.info.capture_timestamp;
	out_meta.captureTimestamp = info.info.capture_timestamp;
	out_meta.localTimestamp = info.info.capture_timestamp;

	ret = mbuf_audio_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0 && ret != -EEXIST) {
		/* Ancillary buffer may already exist; ignore -EEXIST */
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		goto out;
	}

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia, i);
		AudioChannel *channel = dynamic_cast<AudioChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
		else
			setFlushingState(FlushingState::UNFLUSHED);
	}

out:
	Source::unlock();

	return ret;
}


/* Listener call from an idle function */
void ExternalAudioSource::callOnMediaAdded(void *userdata)
{
	ExternalAudioSource *self =
		reinterpret_cast<ExternalAudioSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGW("%s: output media not found", __func__);
		return;
	}

	if (self->Source::mListener) {
		self->Source::mListener->onOutputMediaAdded(
			self, self->mOutputMedia, self->getAudioSource());
	}
}


/* Listener call from an idle function */
void ExternalAudioSource::callAudioSourceFlushed(void *userdata)
{
	ExternalAudioSource *self =
		reinterpret_cast<ExternalAudioSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	self->setFlushingState(FlushingState::FLUSHED);

	if (self->mAudioSourceListener != nullptr) {
		if (self->mFlushDiscard) {
			self->mAudioSourceListener->onAudioSourceFlushed(
				self->mSession, self->getAudioSource());
		} else {
			self->mAudioSourceListener->onAudioSourceDrained(
				self->mSession, self->getAudioSource());
		}
	}
}


AudioSourceWrapper::AudioSourceWrapper(
	Session *session,
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener)
{
	mElement = mSource = new Pdraw::ExternalAudioSource(
		session, session, session, listener, this, params);
}


AudioSourceWrapper::~AudioSourceWrapper(void)
{
	if (isElementStopped())
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("ExternalAudioSource::stop", -ret);
}


struct mbuf_audio_frame_queue *AudioSourceWrapper::getQueue(void)
{
	if (isElementStopped())
		return nullptr;
	return mSource->getQueue();
}


int AudioSourceWrapper::flush(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSource->flush();
}


int AudioSourceWrapper::drain(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSource->drain();
}

} /* namespace Pdraw */
