/**
 * Parrot Drones Audio and Video Vector library
 * Audio decoder element
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

#define ULOG_TAG pdraw_adec
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_decoder_audio.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

namespace Pdraw {


const struct adec_cbs AudioDecoder::mDecoderCbs = {
	.frame_output = &AudioDecoder::frameOutputCb,
	.flush = &AudioDecoder::flushCb,
	.stop = &AudioDecoder::stopCb,
};


AudioDecoder::AudioDecoder(Session *session,
			   Element::Listener *elementListener,
			   Source::Listener *sourceListener) :
		FilterElement(session,
			      elementListener,
			      nullptr,
			      1,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      1,
			      sourceListener),
		mInputMedia(nullptr), mOutputMedia(nullptr),
		mInputBufferPool(nullptr), mInputBufferQueue(nullptr),
		mAdec(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mAdecFlushPending(false),
		mAdecStopPending(false)
{
	const struct adef_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	/* Supported input formats */
	supportedInputFormatsCount = adec_get_supported_input_formats(
		ADEC_DECODER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0) {
		PDRAW_LOG_ERRNO("adec_get_supported_input_formats",
				-supportedInputFormatsCount);
	} else {
		setAudioMediaFormatCaps(supportedInputFormats,
					supportedInputFormatsCount);
	}

	setState(CREATED);
}


AudioDecoder::~AudioDecoder(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		PDRAW_LOGW("decoder is still running");

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	if (mAdec != nullptr) {
		ret = adec_destroy(mAdec);
		if (ret < 0)
			PDRAW_LOG_ERRNO("adec_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int AudioDecoder::start(void)
{
	int ret = 0, err;
	InputPort *port = nullptr;
	struct adec_config cfg = {};
	Channel *c = nullptr;
	AudioChannel *channel = nullptr;
	const uint8_t *asc = nullptr;
	size_t ascSize = 0;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: decoder is not created", __func__);
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
	mInputMedia = dynamic_cast<AudioMedia *>(getInputMedia(0));
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

	/* Initialize the decoder */
	cfg.implem = ADEC_DECODER_IMPLEM_AUTO;
	cfg.encoding = mInputMedia->format.encoding;
	ret = adec_new(mSession->getLoop(), &cfg, &mDecoderCbs, this, &mAdec);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("adec_new", -ret);
		goto error;
	}

	/* Configure the decoder */
	switch (mInputMedia->format.encoding) {
	case ADEF_ENCODING_AAC_LC:
		/* Configure the decoder */
		ret = mInputMedia->getAacAsc(&asc, &ascSize);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("media->getAacAsc", -ret);
			goto error;
		}
		ret = adec_set_aac_asc(mAdec,
				       asc,
				       ascSize,
				       mInputMedia->format.aac.data_format);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("adec_set_aac_asc", -ret);
			goto error;
		}
		break;
	default:
		Sink::unlock();
		PDRAW_LOGE("unsupported input media encoding (%s)",
			   adef_encoding_to_str(mInputMedia->format.encoding));
		ret = -EPROTO;
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
	mInputBufferQueue = adec_get_input_buffer_queue(mAdec);
	channel->setQueue(this, mInputBufferQueue);
	mInputBufferPool = adec_get_input_buffer_pool(mAdec);
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


int AudioDecoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTING) && (mState != STARTED)) {
		PDRAW_LOGE("%s: decoder is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);
	mAdecStopPending = true;

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


int AudioDecoder::flush(void)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;

	if (mIsFlushed) {
		PDRAW_LOGD("decoder is already flushed, nothing to do");
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

	/* Flush the decoder (async)
	 * (the input channel queue is flushed by adec) */
	if (mAdec != nullptr) {
		if (!mAdecFlushPending) {
			ret = adec_flush(mAdec, 1);
			if (ret < 0)
				PDRAW_LOG_ERRNO("adec_flush", -ret);
			else
				mAdecFlushPending = true;
		}
	} else {
		completeFlush();
	}

	return ret;
}


void AudioDecoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;
	bool pending = false;

	if (mAdecFlushPending)
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


void AudioDecoder::idleCompleteFlush(void *userdata)
{
	AudioDecoder *self = (AudioDecoder *)userdata;
	self->completeFlush();
}


int AudioDecoder::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;

	if (mState != STOPPING)
		return 0;

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

	/* Stop the decoder */
	if (mAdec != nullptr) {
		ret = adec_stop(mAdec);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("adec_stop", -ret);
			return ret;
		}
	}
	/* Else, delay completeStop() after removing the input port */

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

	if (mAdec == nullptr) {
		mAdecStopPending = false;
		completeStop();
	}

	return 0;
}


void AudioDecoder::completeStop(void)
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
			this, mOutputMedia, nullptr);
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
	if ((!mAdecStopPending) && (mOutputMedia == nullptr))
		setState(STOPPED);
}


int AudioDecoder::createOutputMedia(const struct adef_frame *frameInfo,
				    const AudioMedia::Frame &frame)
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

	mOutputMedia->format = frameInfo->format;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia, nullptr);

	return 0;
}


void AudioDecoder::onAudioChannelQueue(AudioChannel *channel,
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
		PDRAW_LOGE("frame input: decoder is not started");
		return;
	}
	if ((mAdecFlushPending) || (mInputChannelFlushPending)) {
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


void AudioDecoder::onChannelFlush(Channel *channel)
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


void AudioDecoder::onChannelFlushed(Channel *channel)
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


void AudioDecoder::onChannelTeardown(Channel *channel)
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


void AudioDecoder::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING)
		completeStop();
}


void AudioDecoder::frameOutputCb(struct adec_decoder *dec,
				 int status,
				 struct mbuf_audio_frame *out_frame,
				 void *userdata)
{
	int ret;
	AudioDecoder *self = (AudioDecoder *)userdata;
	struct adef_frame info;
	struct mbuf_ancillary_data *ancillaryData;
	AudioMedia::Frame *in_meta;
	AudioMedia::Frame out_meta;
	unsigned int outputChannelCount, i;

	if (status != 0) {
		PDRAW_LOGE("decoder error (%s)", strerror(-status));
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
		PDRAW_LOGE("frame output: decoder is not started");
		return;
	}
	if ((self->mAdecFlushPending) || (self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_audio_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_audio_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->Sink::unlock();
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
	out_meta.decoderOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, ADEC_ANCILLARY_KEY_OUTPUT_TIME);
	ret = mbuf_ancillary_data_unref(ancillaryData);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -ret);

	/* Remove the PDrAW input ancillary data */
	ret = mbuf_audio_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_audio_frame_remove_ancillary_data", -ret);

	self->Sink::unlock();
	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		/* Create the output media now that the format is known */
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	ret = mbuf_audio_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		self->Source::unlock();
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		return;
	}

	/* Push the frame (unless it is silent) */
	if (true) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			Channel *c =
				self->getOutputChannel(self->mOutputMedia, i);
			AudioChannel *channel = dynamic_cast<AudioChannel *>(c);
			if (channel == nullptr) {
				PDRAW_LOGE("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->queue(out_frame);
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	}

	self->Source::unlock();
}


void AudioDecoder::flushCb(struct adec_decoder *dec, void *userdata)
{
	AudioDecoder *self = (AudioDecoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("decoder is flushed");
	self->mAdecFlushPending = false;

	self->completeFlush();
}


void AudioDecoder::stopCb(struct adec_decoder *dec, void *userdata)
{
	AudioDecoder *self = (AudioDecoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("decoder is stopped");
	self->mAdecStopPending = false;
	self->completeStop();
}

} /* namespace Pdraw */
