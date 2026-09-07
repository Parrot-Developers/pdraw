/**
 * Parrot Drones Audio and Video Vector library
 * Video encoder element
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

#define ULOG_TAG pdraw_aenc
#include <ulog.h>

#include "pdraw_encoder_audio.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


const struct aenc_cbs AudioEncoder::mEncoderCbs = {
	.frame_output = &AudioEncoder::frameOutputCb,
	.flush = &AudioEncoder::flushCb,
	.stop = &AudioEncoder::stopCb,
	.pre_release = &AudioEncoder::framePreReleaseCb,
};


AudioEncoder::AudioEncoder(Session *session,
			   Element::Listener *elementListener,
			   Source::Listener *sourceListener,
			   IPdraw::IAudioEncoder::Listener *listener,
			   AudioEncoderWrapper *wrapper,
			   const struct aenc_config *params) :
		FilterElement(session,
			      elementListener,
			      wrapper,
			      1,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      1,
			      sourceListener),
		mEncoder(wrapper), mEncoderListener(listener)
{
	int err;
	const struct adef_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	mCompleteFlushHandler.set([this] { idleCompleteFlush(); });

	/* Supported input formats */
	supportedInputFormatsCount = aenc_get_supported_input_formats(
		AENC_ENCODER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0)
		PDRAW_LOG_ERRNO("aenc_get_supported_input_formats",
				-supportedInputFormatsCount);
	else
		setAudioMediaFormatCaps(supportedInputFormats,
					supportedInputFormatsCount);

	if (params != nullptr) {
		/* Encoder params deep copy */
		aenc_config *rawConfig = nullptr;
		err = aenc_config_copy(params, &rawConfig);
		if (err < 0) {
			PDRAW_LOG_ERRNO("aenc_config_copy", -err);
		} else {
			mEncoderConfig.reset(rawConfig);
			if (mEncoderConfig->name != nullptr)
				mEncoderName =
					std::string(mEncoderConfig->name);
			if (mEncoderConfig->device != nullptr)
				mEncoderDevice =
					std::string(mEncoderConfig->device);
		}
	}

	setState(State::CREATED);
}


AudioEncoder::~AudioEncoder()
{
	int err;

	if (mState != State::STOPPED)
		PDRAW_LOGW("encoder is still running");

	/* Make sure listener functions will no longer be called */
	removeEncoderListener();

	/* Remove any leftover idle callbacks */
	mSession->getPompLoop()->idleRemove(this);

	if (mAenc != nullptr) {
		err = aenc_destroy(mAenc);
		if (err < 0)
			PDRAW_LOG_ERRNO("aenc_destroy", -err);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int AudioEncoder::start()
{
	int ret = 0;
	int err;
	Media *media = nullptr;
	const InputPort *port = nullptr;
	Channel *c = nullptr;
	AudioChannel *channel = nullptr;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: encoder is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media count");
		ret = -EPROTO;
		goto error;
	}
	media = getInputMedia(0);
	if (media == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		ret = -EPROTO;
		goto error;
	}
	mInputMedia = dynamic_cast<AudioMedia *>(media);
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		ret = -EPROTO;
		goto error;
	}
	port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		ret = -EPROTO;
		goto error;
	}

	/* Initialize the encoder */
	if (mEncoderConfig) {
		/* The configuration was provided through the constructor;
		 * simply override the input config */
		mEncoderConfig->input.format = mInputMedia->format;
		if (mEncoderConfig->implem == AENC_ENCODER_IMPLEM_AUTO &&
		    mEncoderConfig->encoding != ADEF_ENCODING_UNKNOWN) {
			/* If AUTO implem was provided with an encoding,
			 * auto select encoder implem by encoding */
			mEncoderConfig->implem =
				aenc_get_auto_implem_by_encoding(
					mEncoderConfig->encoding);
			if (mEncoderConfig->implem ==
			    AENC_ENCODER_IMPLEM_AUTO) {
				Sink::unlock();
				ret = -ENOENT;
				PDRAW_LOG_ERRNO(
					"aenc_get_auto_implem_by_encoding",
					-ret);
				goto error;
			}
		}
	} else {
		mEncoderConfig = make_c_struct<AencConfigPtr>();
		if (!mEncoderConfig) {
			Sink::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("calloc", -ret);
			goto error;
		}
		mEncoderConfig->implem = AENC_ENCODER_IMPLEM_AUTO;
		mEncoderConfig->encoding = ADEF_ENCODING_AAC_LC; /* TODO */
		if (mEncoderConfig->implem == AENC_ENCODER_IMPLEM_AUTO &&
		    mEncoderConfig->encoding != ADEF_ENCODING_UNKNOWN) {
			/* If AUTO implem was provided with an encoding,
			 * auto select encoder implem by encoding */
			mEncoderConfig->implem =
				aenc_get_auto_implem_by_encoding(
					mEncoderConfig->encoding);
			if (mEncoderConfig->implem ==
			    AENC_ENCODER_IMPLEM_AUTO) {
				Sink::unlock();
				ret = -ENOENT;
				PDRAW_LOG_ERRNO(
					"aenc_get_auto_implem_by_encoding",
					-ret);
				goto error;
			}
		}
		mEncoderConfig->input.format = mInputMedia->format;
		mEncoderConfig->aac_lc.max_bitrate = 128000; /* TODO */
	}
	ret = aenc_new(mSession->getLoop(),
		       mEncoderConfig.get(),
		       &mEncoderCbs,
		       this,
		       &mAenc);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("aenc_new", -ret);
		goto error;
	}

	/* Setup the input port */
	c = port->channel.get();
	channel = dynamic_cast<AudioChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		ret = -EPROTO;
		goto error;
	}
	mInputBufferQueue = mbuf::Queue::wrapExisting(
		aenc_get_input_buffer_queue(mAenc), false);
	channel->setQueue(this, mInputBufferQueue.get());
	mInputBufferPool = aenc_get_input_buffer_pool(mAenc);
	channel->setPool(this, mInputBufferPool);

	Sink::unlock();

	setState(State::STARTED);

	return 0;

error:
	if (mInputMedia != nullptr) {
		/* mInputMedia must be removed synchronously to avoid holding a
		 * reference to a media that can be destroyed at any moment by
		 * the Source element */
		err = removeInputMedia(mInputMedia);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -err);
		else
			mInputMedia = nullptr;
	}
	err = stop();
	if (err < 0)
		PDRAW_LOG_ERRNO("stop", -err);

	return ret;
}


int AudioEncoder::stop()
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState == State::CREATED) {
		/* Skip flush/drain for unstarted elements to prevent them from
		 * getting stuck in CREATED and blocking session stop */
		setState(State::STOPPED);
		return 0;
	}
	if ((mState != State::STARTING) && (mState != State::STARTED)) {
		PDRAW_LOGE("%s: encoder is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);
	mAencStopPending = true;

	/* Make sure listener functions will no longer be called */
	removeEncoderListener();

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


int AudioEncoder::flush(bool discard)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		if (mInputBufferQueue != nullptr) {
			ret = mInputBufferQueue->getCount();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("queue::getCount", -ret);
				return ret;
			} else if (ret > 0) {
				setFlushingState(FlushingState::UNFLUSHED);
				break;
			}
		}
		PDRAW_LOGD("encoder is already %s, nothing to do",
			   discard ? "flushed" : "drained");
		ret = mSession->getPompLoop()->idleAdd(&mCompleteFlushHandler,
						       this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("Loop::idleAdd", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	Source::lock();
	if (mOutputMedia != nullptr) {
		if (mFlushDiscard) {
			/* Flush the output channels (async) */
			outputChannelCount =
				getOutputChannelCount(mOutputMedia.get());
			for (unsigned int i = 0; i < outputChannelCount; i++) {
				outputChannel =
					getOutputChannel(mOutputMedia.get(), i);
				if (outputChannel == nullptr) {
					PDRAW_LOGW(
						"failed to get output channel "
						"at index %d",
						i);
					continue;
				}
				err = outputChannel->flush();
				if (err < 0 && err != -EALREADY) {
					PDRAW_LOG_ERRNO(
						"channel->flush "
						"(channel index=%u)",
						-err,
						i);
				}
			}
		} else {
			/* Drain event is called once flush is complete */
			mOutputChannelDrainRequired = true;
		}
	}
	Source::unlock();

	/* Flush the encoder (async)
	 * (the input channel queue is flushed by aenc) */
	if (mAenc != nullptr) {
		if (!mAencFlushPending) {
			ret = aenc_flush(mAenc, mFlushDiscard);
			if (ret < 0)
				PDRAW_LOG_ERRNO("aenc_flush", -ret);
			else
				mAencFlushPending = true;
		}
	} else {
		completeFlush();
	}

	return ret;
}


void AudioEncoder::completeFlush()
{
	int ret;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;
	bool pending = false;

	if (mAencFlushPending)
		return;

	/* Drain the output channels (async) */
	Source::lock();
	if (!mFlushDiscard && mOutputChannelDrainRequired &&
	    mOutputMedia != nullptr) {
		mOutputChannelDrainRequired = false;
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			err = outputChannel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO(
					"channel->drain (channel index=%u)",
					-err,
					i);
			}
		}
	}
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
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

	Sink::lock();
	if ((mInputMedia != nullptr) && mInputChannelFlushPending) {
		mInputChannelFlushPending = false;
		Channel *inputChannel = getInputChannel(mInputMedia);
		if (inputChannel == nullptr) {
			PDRAW_LOGE("failed to get input channel");
		} else {
			if (mFlushDiscard)
				ret = inputChannel->flushDone();
			else
				ret = inputChannel->drainDone();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("channel->%s",
						-ret,
						mFlushDiscard ? "flushDone"
							      : "drainDone");
			}
		}
	}
	Sink::unlock();

	tryStop();
}


void AudioEncoder::idleCompleteFlush()
{
	completeFlush();
}


int AudioEncoder::tryStop()
{
	int ret;
	int outputChannelCount = 0;

	if (mState != State::STOPPING)
		return 0;

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		auto *channel = dynamic_cast<AudioChannel *>(
			getInputChannel(mInputMedia));
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			channel->setQueue(this, nullptr);
			channel->setPool(this, nullptr);
		}
		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = nullptr;
	}
	Sink::unlock();

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	Source::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());

		for (int i = outputChannelCount - 1; i >= 0; i--) {
			Channel *channel =
				getOutputChannel(mOutputMedia.get(), i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->teardown", -ret);
		}
	}
	Source::unlock();

	/* Stop the encoder
	 * tryStop() can be re-entered while STOPPING; guard against issuing
	 * aenc_stop() more than once (see VideoScaler/vscale_libyuv). */
	if (mAenc != nullptr) {
		if (!mAencStopIssued) {
			mAencStopIssued = true;
			ret = aenc_stop(mAenc);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("aenc_stop", -ret);
				return ret;
			}
		}
	} else {
		mAencStopPending = false;
		completeStop();
	}

	return 0;
}


void AudioEncoder::completeStop()
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia.get(), getAudioEncoder());
	}
	ret = removeOutputPort(mOutputMedia.get());
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		mOutputMedia.reset();
	}

	Source::unlock();

exit:
	if ((!mAencStopPending) && (mOutputMedia == nullptr))
		setState(State::STOPPED);
}


int AudioEncoder::createOutputMedia(
	const struct adef_frame *frame_info,
	[[maybe_unused]] const AudioMedia::Frame &frame)
{

	int ret;

	Source::lock();

	try {
		mOutputMedia = std::make_unique<AudioMedia>(mSession);
	} catch (const std::bad_alloc &) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frame_info->format;
	mOutputMedia->copyPropertiesFrom(mInputMedia);

	if (mOutputMedia->format.encoding == ADEF_ENCODING_AAC_LC) {
		size_t ascSize = 0;
		ret = aenc_get_aac_asc(mAenc, nullptr, &ascSize);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("aenc_get_aac_asc", -ret);
			return ret;
		}

		try {
			std::vector<uint8_t> ascBuf(ascSize);

			ret = aenc_get_aac_asc(mAenc, ascBuf.data(), &ascSize);
			if (ret < 0) {
				Source::unlock();
				PDRAW_LOG_ERRNO("aenc_get_aac_asc", -ret);
				return ret;
			}
			ret = mOutputMedia->setAacAsc(ascBuf.data(),
						      ascBuf.size());
			if (ret < 0) {
				Source::unlock();
				PDRAW_LOG_ERRNO("media->setAacAsc", -ret);
				return ret;
			}
		} catch (const std::bad_alloc &) {
			Source::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("std::vector allocation failed", -ret);
			return ret;
		}
	} else {
		Source::unlock();
		PDRAW_LOGE("unsupported encoding");
		return -EINVAL;
	}

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia.get(), getAudioEncoder());

	return 0;
}


void AudioEncoder::removeEncoderListener()
{
	std::scoped_lock lock(mListenerMutex);
	mEncoderListener = nullptr;
}


void AudioEncoder::onAudioChannelQueue(AudioChannel *channel,
				       struct mbuf_audio_frame *frame)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(frame == nullptr, EINVAL);

	if (mState != State::STARTED) {
		PDRAW_LOGE("frame input: encoder is not started");
		return;
	}
	if (mAencFlushPending || mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	if (mInputBufferQueue == nullptr ||
	    !channel->hasQueue(mInputBufferQueue.get())) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}

	Sink::onAudioChannelQueue(channel, frame);
	setFlushingState(FlushingState::UNFLUSHED);
	Sink::unlock();
}


void AudioEncoder::onChannelFlush(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void AudioEncoder::onChannelDrain(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("draining input channel");
	mInputChannelFlushPending = true;

	int ret = drain();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("drain", -ret);
}


void AudioEncoder::onChannelFlushed(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

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


void AudioEncoder::onChannelDrained(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

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
}


void AudioEncoder::onChannelTeardown(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


void AudioEncoder::onChannelUnlink(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING)
		completeStop();
}


void AudioEncoder::frameOutputCb([[maybe_unused]] struct aenc_encoder *enc,
				 int status,
				 struct mbuf_audio_frame *out_frame,
				 void *userdata)
{

	int ret;
	auto *self = static_cast<AudioEncoder *>(userdata);
	struct adef_frame info;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const AudioMedia::Frame *in_meta;
	AudioMedia::Frame out_meta{};
	unsigned int outputChannelCount;

	if (status != 0) {
		PDRAW_LOGE("encoder error: %d(%s)", -status, strerror(-status));
		return;
	}

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_IF(out_frame == nullptr, EINVAL);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("frame output: encoder is not started");
		return;
	}
	if (self->mFlushDiscard &&
	    (self->mAencFlushPending || self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}
	self->Sink::unlock();

	ret = mbuf_audio_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_audio_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&ancillaryData);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_ancillary_data:pdraw_in",
				-ret);
		return;
	}

	in_meta = static_cast<const AudioMedia::Frame *>(
		mbuf_ancillary_data_get_buffer(ancillaryData, nullptr));
	out_meta.ntpTimestamp = in_meta->ntpTimestamp;
	out_meta.ntpUnskewedTimestamp = in_meta->ntpUnskewedTimestamp;
	out_meta.ntpRawTimestamp = in_meta->ntpRawTimestamp;
	out_meta.ntpRawUnskewedTimestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta.playTimestamp = in_meta->playTimestamp;
	out_meta.captureTimestamp = in_meta->captureTimestamp;
	out_meta.localTimestamp = in_meta->localTimestamp;
	out_meta.localTimestampPrecision = in_meta->localTimestampPrecision;
	out_meta.recvStartTimestamp = in_meta->recvStartTimestamp;
	out_meta.recvEndTimestamp = in_meta->recvEndTimestamp;
	out_meta.demuxOutputTimestamp = in_meta->demuxOutputTimestamp;
	out_meta.encoderOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, AENC_ANCILLARY_KEY_OUTPUT_TIME);
	ret = mbuf_ancillary_data_unref(ancillaryData);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -ret);

	/* Remove the PDrAW input ancillary data */
	ret = mbuf_audio_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_audio_frame_remove_ancillary_data", -ret);

	ret = mbuf_audio_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		return;
	}

	{
		std::scoped_lock lock(self->mListenerMutex);
		if (self->mEncoderListener != nullptr) {
			self->mEncoderListener->audioEncoderFrameOutput(
				self->mSession,
				self->getAudioEncoder(),
				out_frame);
		}
	}

	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	outputChannelCount =
		self->getOutputChannelCount(self->mOutputMedia.get());
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c =
			self->getOutputChannel(self->mOutputMedia.get(), i);
		auto *channel = dynamic_cast<AudioChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		ret = channel->queue(out_frame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->queue", -ret);
	}

	self->Source::unlock();
}


void AudioEncoder::flushCb([[maybe_unused]] struct aenc_encoder *enc,
			   void *userdata)
{

	auto *self = static_cast<AudioEncoder *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("encoder is flushed");
	self->mAencFlushPending = false;

	self->completeFlush();
}


void AudioEncoder::stopCb([[maybe_unused]] struct aenc_encoder *enc,
			  void *userdata)
{

	auto *self = static_cast<AudioEncoder *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("encoder is stopped");
	self->mAencStopPending = false;
	self->completeStop();
}


void AudioEncoder::framePreReleaseCb(struct mbuf_audio_frame *frame,
				     void *userdata)
{
	auto *self = static_cast<AudioEncoder *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	std::scoped_lock lock(self->mListenerMutex);
	if (self->mEncoderListener != nullptr) {
		self->mEncoderListener->audioEncoderFramePreRelease(
			self->mSession, self->getAudioEncoder(), frame);
	}
}


AudioEncoderWrapper::AudioEncoderWrapper(
	Session *session,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener) :
		ElementWrapper(new Pdraw::AudioEncoder(session,
						       session,
						       session,
						       listener,
						       this,
						       params)),
		mEncoder(static_cast<Pdraw::AudioEncoder *>(mElement))
{
}


AudioEncoderWrapper::~AudioEncoderWrapper()
{
	if (isElementStopped())
		return;
	int ret = mEncoder->stop();
	if (ret < 0)
		ULOG_ERRNO("AudioEncoder::stop", -ret);
}

} /* namespace Pdraw */
