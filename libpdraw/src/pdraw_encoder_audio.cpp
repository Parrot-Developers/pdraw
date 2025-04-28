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
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_encoder_audio.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

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
		mEncoder(wrapper), mEncoderListener(listener),
		mInputMedia(nullptr), mOutputMedia(nullptr),
		mInputBufferPool(nullptr), mInputBufferQueue(nullptr),
		mEncoderConfig(nullptr), mAenc(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mAencFlushPending(false),
		mAencStopPending(false)
{
	int err;
	pthread_mutexattr_t attr;
	bool attrCreated = false;
	const struct adef_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	err = pthread_mutexattr_init(&attr);
	if (err != 0) {
		PDRAW_LOG_ERRNO("pthread_mutexattr_init", err);
		goto skip_mutex;
	}
	attrCreated = true;

	err = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (err != 0) {
		PDRAW_LOG_ERRNO("pthread_mutexattr_settype", err);
		goto skip_mutex;
	}

	err = pthread_mutex_init(&mListenerMutex, &attr);
	if (err != 0) {
		PDRAW_LOG_ERRNO("pthread_mutex_init", err);
		goto skip_mutex;
	}

skip_mutex:
	if (attrCreated)
		pthread_mutexattr_destroy(&attr);

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
		int err = aenc_config_copy(params, &mEncoderConfig);
		if (err < 0) {
			PDRAW_LOG_ERRNO("aenc_config_copy", -err);
		} else {
			if (mEncoderConfig->name != nullptr)
				mEncoderName =
					std::string(mEncoderConfig->name);
			if (mEncoderConfig->device != nullptr)
				mEncoderDevice =
					std::string(mEncoderConfig->device);
		}
	}

	setState(CREATED);
}


AudioEncoder::~AudioEncoder(void)
{
	int err;

	if (mState != STOPPED)
		PDRAW_LOGW("encoder is still running");

	/* Make sure listener functions will no longer be called */
	removeEncoderListener();

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (mAenc != nullptr) {
		err = aenc_destroy(mAenc);
		if (err < 0)
			PDRAW_LOG_ERRNO("aenc_destroy", -err);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");

	err = aenc_config_free(mEncoderConfig);
	if (err < 0)
		PDRAW_LOG_ERRNO("aenc_config_free", -err);

	pthread_mutex_destroy(&mListenerMutex);
}


int AudioEncoder::start(void)
{
	int ret = 0, err;
	Media *media = nullptr;
	InputPort *port = nullptr;
	Channel *c = nullptr;
	AudioChannel *channel = nullptr;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: encoder is not created", __func__);
		return -EPROTO;
	}
	setState(STARTING);

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
	if (mEncoderConfig != nullptr) {
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
		mEncoderConfig = (struct aenc_config *)calloc(
			1, sizeof(*mEncoderConfig));
		if (mEncoderConfig == nullptr) {
			Sink::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("malloc", -ret);
			goto error;
		}
		mEncoderConfig->implem = AENC_ENCODER_IMPLEM_AUTO;
		mEncoderConfig->encoding = ADEF_ENCODING_AAC_LC; /* TODO */
		mEncoderConfig->input.format = mInputMedia->format;
	}
	ret = aenc_new(mSession->getLoop(),
		       mEncoderConfig,
		       &mEncoderCbs,
		       this,
		       &mAenc);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("aenc_new", -ret);
		goto error;
	}

	/* Setup the input port */
	c = port->channel;
	channel = dynamic_cast<AudioChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		ret = -EPROTO;
		goto error;
	}
	mInputBufferQueue = aenc_get_input_buffer_queue(mAenc);
	channel->setQueue(this, mInputBufferQueue);
	mInputBufferPool = aenc_get_input_buffer_pool(mAenc);
	channel->setPool(this, mInputBufferPool);

	Sink::unlock();

	setState(STARTED);

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


int AudioEncoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTING) && (mState != STARTED)) {
		PDRAW_LOGE("%s: encoder is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);
	mAencStopPending = true;

	/* Make sure listener functions will no longer be called */
	removeEncoderListener();

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int AudioEncoder::flush(void)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;

	if (mIsFlushed) {
		PDRAW_LOGD("encoder is already flushed, nothing to do");
		ret = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleCompleteFlush, this, this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		return ret;
	}

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			err = outputChannel->flush();
			if (err < 0) {
				PDRAW_LOG_ERRNO(
					"channel->flush (channel index=%u)",
					-err,
					i);
			}
		}
	}
	Source::unlock();

	/* Flush the encoder (async)
	 * (the input channel queue is flushed by aenc) */
	if (mAenc != nullptr) {
		if (!mAencFlushPending) {
			ret = aenc_flush(mAenc, 1);
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


void AudioEncoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;
	bool pending = false;

	if (mAencFlushPending)
		return;

	Source::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (outputChannel->isFlushPending()) {
				pending = true;
				break;
			}
		}
	}
	Source::unlock();

	if (pending)
		return;

	Sink::lock();
	if (mInputMedia != nullptr) {
		mIsFlushed = true;
		if (mInputChannelFlushPending) {
			mInputChannelFlushPending = false;
			AudioChannel *inputChannel =
				dynamic_cast<AudioChannel *>(
					getInputChannel(mInputMedia));
			if (inputChannel == nullptr) {
				PDRAW_LOGE("failed to get input channel");
			} else {
				ret = inputChannel->flushDone();
				if (ret < 0)
					PDRAW_LOG_ERRNO("channel->flushDone",
							-ret);
			}
		}
	}
	Sink::unlock();

	tryStop();
}


void AudioEncoder::idleCompleteFlush(void *userdata)
{
	AudioEncoder *self = (AudioEncoder *)userdata;
	self->completeFlush();
}


int AudioEncoder::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;

	if (mState != STOPPING)
		return 0;

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		AudioChannel *channel = dynamic_cast<AudioChannel *>(
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
		outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (i = outputChannelCount - 1; i >= 0; i--) {
			Channel *channel = getOutputChannel(mOutputMedia, i);
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

	/* Stop the encoder */
	if (mAenc != nullptr) {
		ret = aenc_stop(mAenc);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("aenc_stop", -ret);
			return ret;
		}
	} else {
		mAencStopPending = false;
		completeStop();
	}

	return 0;
}


void AudioEncoder::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia, getAudioEncoder());
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = nullptr;
	}

	Source::unlock();

exit:
	if ((!mAencStopPending) && (mOutputMedia == nullptr))
		setState(STOPPED);
}


int AudioEncoder::createOutputMedia(struct adef_frame *frame_info,
				    AudioMedia::Frame &frame)
{
	int ret;

	Source::lock();

	mOutputMedia = new AudioMedia(mSession);
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frame_info->format;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	switch (mOutputMedia->format.encoding) {
	case ADEF_ENCODING_AAC_LC: {
		uint8_t *asc;
		size_t ascSize = 0;
		ret = aenc_get_aac_asc(mAenc, nullptr, &ascSize);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("aenc_get_aac_asc", -ret);
			return ret;
		}
		asc = (uint8_t *)malloc(ascSize);
		if (asc == nullptr) {
			Source::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("malloc:asc", -ret);
			return ret;
		}
		ret = aenc_get_aac_asc(mAenc, asc, &ascSize);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("aenc_get_aac_asc", -ret);
			free(asc);
			return ret;
		}
		ret = mOutputMedia->setAacAsc(asc, ascSize);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("media->setAacAsc", -ret);
			free(asc);
			return ret;
		}
		free(asc);
		break;
	}
	default:
		Source::unlock();
		PDRAW_LOGE("unsupported encoding");
		return -EINVAL;
	}

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia, getAudioEncoder());

	return 0;
}


void AudioEncoder::removeEncoderListener(void)
{
	pthread_mutex_lock(&mListenerMutex);
	mEncoderListener = nullptr;
	pthread_mutex_unlock(&mListenerMutex);
}


void AudioEncoder::onAudioChannelQueue(AudioChannel *channel,
				       struct mbuf_audio_frame *frame)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		PDRAW_LOG_ERRNO("frame", EINVAL);
		return;
	}
	if (mState != STARTED) {
		PDRAW_LOGE("frame input: encoder is not started");
		return;
	}
	if ((mAencFlushPending) || (mInputChannelFlushPending)) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	struct mbuf_audio_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}
	if (queue != mInputBufferQueue) {
		Sink::unlock();
		PDRAW_LOGE("invalid input buffer queue");
		return;
	}

	Sink::onAudioChannelQueue(channel, frame);
	mIsFlushed = false;
	Sink::unlock();
}


void AudioEncoder::onChannelFlush(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void AudioEncoder::onChannelFlushed(Channel *channel)
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


void AudioEncoder::onChannelTeardown(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


void AudioEncoder::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING)
		completeStop();
}


void AudioEncoder::frameOutputCb(struct aenc_encoder *enc,
				 int status,
				 struct mbuf_audio_frame *out_frame,
				 void *userdata)
{
	int ret;
	AudioEncoder *self = (AudioEncoder *)userdata;
	struct adef_frame info;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	AudioMedia::Frame *in_meta;
	AudioMedia::Frame out_meta;
	unsigned int outputChannelCount, i;

	if (status != 0) {
		PDRAW_LOGE("encoder error: %d(%s)", -status, strerror(-status));
		return;
	}

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}
	if (out_frame == nullptr) {
		PDRAW_LOG_ERRNO("out_frame", EINVAL);
		return;
	}
	if (self->mState != STARTED) {
		PDRAW_LOGE("frame output: encoder is not started");
		return;
	}
	if ((self->mAencFlushPending) || (self->mInputChannelFlushPending)) {
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

	in_meta = (AudioMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, nullptr);
	memset(&out_meta, 0, sizeof(out_meta));
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

	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mEncoderListener != nullptr) {
		self->mEncoderListener->audioEncoderFrameOutput(
			self->mSession, self->getAudioEncoder(), out_frame);
	}
	pthread_mutex_unlock(&self->mListenerMutex);

	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	outputChannelCount = self->getOutputChannelCount(self->mOutputMedia);
	for (i = 0; i < outputChannelCount; i++) {
		Channel *c = self->getOutputChannel(self->mOutputMedia, i);
		AudioChannel *channel = dynamic_cast<AudioChannel *>(c);
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


void AudioEncoder::flushCb(struct aenc_encoder *enc, void *userdata)
{
	AudioEncoder *self = (AudioEncoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("encoder is flushed");
	self->mAencFlushPending = false;

	self->completeFlush();
}


void AudioEncoder::stopCb(struct aenc_encoder *enc, void *userdata)
{
	AudioEncoder *self = (AudioEncoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("encoder is stopped");
	self->mAencStopPending = false;
	self->completeStop();
}


void AudioEncoder::framePreReleaseCb(struct mbuf_audio_frame *frame,
				     void *userdata)
{
	AudioEncoder *self = (AudioEncoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	pthread_mutex_lock(&self->mListenerMutex);
	if (self->mEncoderListener != nullptr) {
		self->mEncoderListener->audioEncoderFramePreRelease(
			self->mSession, self->getAudioEncoder(), frame);
	}
	pthread_mutex_unlock(&self->mListenerMutex);
}


AudioEncoderWrapper::AudioEncoderWrapper(
	Session *session,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener)
{
	mElement = mEncoder = new Pdraw::AudioEncoder(
		session, session, session, listener, this, params);
}


AudioEncoderWrapper::~AudioEncoderWrapper(void)
{
	if (mEncoder == nullptr)
		return;
	int ret = mEncoder->stop();
	if (ret < 0)
		ULOG_ERRNO("AudioEncoder::stop", -ret);
}

} /* namespace Pdraw */
