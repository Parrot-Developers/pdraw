/**
 * Parrot Drones Awesome Video Viewer Library
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

#define ULOG_TAG pdraw_venc
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_encoder_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

namespace Pdraw {


const struct venc_cbs VideoEncoder::mEncoderCbs = {
	.frame_output = &VideoEncoder::frameOutputCb,
	.flush = &VideoEncoder::flushCb,
	.stop = &VideoEncoder::stopCb,
	.pre_release = nullptr,
};


VideoEncoder::VideoEncoder(Session *session,
			   Element::Listener *elementListener,
			   Source::Listener *sourceListener) :
		FilterElement(session,
			      elementListener,
			      1,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      1,
			      sourceListener),
		mInputMedia(nullptr), mOutputMedia(nullptr),
		mInputBufferPool(nullptr), mInputBufferQueue(nullptr),
		mVenc(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mVencFlushPending(false),
		mVencStopPending(false)
{
	const struct vdef_raw_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	/* Supported input formats */
	supportedInputFormatsCount = venc_get_supported_input_formats(
		VENC_ENCODER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0)
		PDRAW_LOG_ERRNO("venc_get_supported_input_formats",
				-supportedInputFormatsCount);
	else
		setRawVideoMediaFormatCaps(supportedInputFormats,
					   supportedInputFormatsCount);

	setState(CREATED);
}


VideoEncoder::~VideoEncoder(void)
{
	int ret;

	if (mState != STOPPED)
		PDRAW_LOGW("encoder is still running");

	if (mVenc) {
		ret = venc_destroy(mVenc);
		if (ret < 0)
			PDRAW_LOG_ERRNO("venc_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int VideoEncoder::start(void)
{
	int ret = 0;

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
		return -EPROTO;
	}
	Media *media = getInputMedia(0);
	if (media == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	mInputMedia = dynamic_cast<RawVideoMedia *>(media);
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		return -EPROTO;
	}

	/* Initialize the encoder */
	struct venc_config cfg = {};
	cfg.implem = VENC_ENCODER_IMPLEM_AUTO;
	cfg.encoding = VDEF_ENCODING_H264; /* TODO */
	cfg.input.format = mInputMedia->format;
	cfg.input.info = mInputMedia->info;
	if ((cfg.input.info.framerate.num == 0) ||
	    (cfg.input.info.framerate.den == 0)) {
		/* TODO: this is an ugly workaround,
		 * mInputMedia should have a valid framerate */
		cfg.input.info.framerate.num = 30;
		cfg.input.info.framerate.den = 1;
	}
	cfg.h264.max_bitrate = 10000000; /* TODO */
	/* TODO: Default to AVCC, see how to handle the choice later */
	cfg.output.preferred_format = VDEF_CODED_DATA_FORMAT_AVCC;
	ret = venc_new(mSession->getLoop(), &cfg, &mEncoderCbs, this, &mVenc);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("venc_new", -ret);
		return ret;
	}

	/* Setup the input port */
	Channel *c = port->channel;
	RawVideoChannel *channel = dynamic_cast<RawVideoChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		return -EPROTO;
	}
	mInputBufferQueue = venc_get_input_buffer_queue(mVenc);
	channel->setQueue(this, mInputBufferQueue);
	mInputBufferPool = venc_get_input_buffer_pool(mVenc);
	channel->setPool(this, mInputBufferPool);

	Sink::unlock();

	setState(STARTED);

	return 0;
}


int VideoEncoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("%s: encoder is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);
	mVencStopPending = true;

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VideoEncoder::flush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;

	if (mIsFlushed) {
		PDRAW_LOGD("encoder is already flushed, nothing to do");
		completeFlush();
		return 0;
	}

	mVencFlushPending = true;

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
			ret = outputChannel->flush();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->flush", -ret);
		}
	}
	Source::unlock();

	/* Flush the encoder (async)
	 * (the input channel queue is flushed by venc) */
	ret = venc_flush(mVenc, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("venc_flush", -ret);

	return ret;
}


void VideoEncoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;
	bool pending = false;

	if (mVencFlushPending)
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
			RawVideoChannel *inputChannel =
				dynamic_cast<RawVideoChannel *>(
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


int VideoEncoder::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;

	if (mState != STOPPING)
		return 0;

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		RawVideoChannel *channel = dynamic_cast<RawVideoChannel *>(
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
	ret = venc_stop(mVenc);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("venc_stop", -ret);
		return ret;
	}

	return 0;
}


void VideoEncoder::completeStop(void)
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
		Source::mListener->onOutputMediaRemoved(this, mOutputMedia);
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
	if ((!mVencStopPending) && (mOutputMedia == nullptr))
		setState(STOPPED);
}


int VideoEncoder::createOutputMedia(struct vdef_coded_frame *frame_info,
				    CodedVideoMedia::Frame &frame)
{
	int ret;

	Source::lock();

	mOutputMedia = new CodedVideoMedia(mSession);
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
	vdef_frame_to_format_info(&frame_info->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	uint8_t *sps, *pps;
	size_t spsSize = 0, ppsSize = 0;
	ret = venc_get_h264_ps(mVenc, nullptr, &spsSize, nullptr, &ppsSize);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("venc_get_h264_ps", -ret);
		return ret;
	}
	sps = (uint8_t *)malloc(spsSize);
	if (sps == nullptr) {
		Source::unlock();
		PDRAW_LOG_ERRNO("malloc:sps", ENOMEM);
		return ret;
	}
	pps = (uint8_t *)malloc(ppsSize);
	if (pps == nullptr) {
		Source::unlock();
		PDRAW_LOG_ERRNO("malloc:pps", ENOMEM);
		free(sps);
		return ret;
	}
	ret = venc_get_h264_ps(mVenc, sps, &spsSize, pps, &ppsSize);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("venc_get_h264_ps", -ret);
		free(sps);
		free(pps);
		return ret;
	}
	ret = mOutputMedia->setPs(nullptr, 0, sps, spsSize, pps, ppsSize);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("media->setPs", -ret);
		free(sps);
		free(pps);
		return ret;
	}
	free(sps);
	free(pps);

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(this, mOutputMedia);

	return 0;
}


void VideoEncoder::onRawVideoChannelQueue(RawVideoChannel *channel,
					  struct mbuf_raw_video_frame *frame)
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
	if ((mVencFlushPending) || (mInputChannelFlushPending)) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);
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

	Sink::onRawVideoChannelQueue(channel, frame);
	mIsFlushed = false;
	Sink::unlock();
}


void VideoEncoder::onChannelFlush(Channel *channel)
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


void VideoEncoder::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


void VideoEncoder::onChannelTeardown(Channel *channel)
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


void VideoEncoder::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING)
		completeStop();
}


void VideoEncoder::frameOutputCb(struct venc_encoder *enc,
				 int status,
				 struct mbuf_coded_video_frame *out_frame,
				 void *userdata)
{
	int ret;
	VideoEncoder *self = (VideoEncoder *)userdata;
	struct vdef_coded_frame info;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	RawVideoMedia::Frame *in_meta;
	CodedVideoMedia::Frame out_meta;
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
	if ((self->mVencFlushPending) || (self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_coded_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_coded_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO(
			"mbuf_coded_video_frame_get_ancillary_data:pdraw_in",
			-ret);
		return;
	}

	in_meta = (RawVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, NULL);
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
		out_frame, VENC_ANCILLARY_KEY_OUTPUT_TIME);
	out_meta.isRef = (info.type != VDEF_CODED_FRAME_TYPE_P_NON_REF);
	out_meta.isSync = ((info.type == VDEF_CODED_FRAME_TYPE_IDR) ||
			   (info.type == VDEF_CODED_FRAME_TYPE_P_IR_START));
	ret = mbuf_ancillary_data_unref(ancillaryData);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -ret);

	/* Remove the PDrAW input ancillary data */
	ret = mbuf_coded_video_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_remove_ancillary_data",
				-ret);

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	self->Sink::unlock();
	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			Channel *c =
				self->getOutputChannel(self->mOutputMedia, i);
			CodedVideoChannel *channel =
				dynamic_cast<CodedVideoChannel *>(c);
			if (channel == nullptr) {
				PDRAW_LOGE("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->queue(out_frame);
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	} else {
		PDRAW_LOGD("silent frame (ignored)");
	}

	self->Source::unlock();
}


void VideoEncoder::flushCb(struct venc_encoder *enc, void *userdata)
{
	VideoEncoder *self = (VideoEncoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("encoder is flushed");
	self->mVencFlushPending = false;

	self->completeFlush();
}


void VideoEncoder::stopCb(struct venc_encoder *enc, void *userdata)
{
	VideoEncoder *self = (VideoEncoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("encoder is stopped");
	self->mVencStopPending = false;
	self->completeStop();
}

} /* namespace Pdraw */
