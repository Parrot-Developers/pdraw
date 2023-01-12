/**
 * Parrot Drones Awesome Video Viewer Library
 * Video decoder element
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

#define ULOG_TAG pdraw_vdec
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_decoder_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"
#if BUILD_LIBVIDEO_DECODE_MEDIACODEC
#	include <video-decode/vdec_mediacodec.h>
#endif

#include <time.h>
#include <unistd.h>

#include <vector>

namespace Pdraw {


const struct vdec_cbs VideoDecoder::mDecoderCbs = {
	.frame_output = &VideoDecoder::frameOutputCb,
	.flush = &VideoDecoder::flushCb,
	.stop = &VideoDecoder::stopCb,
};


VideoDecoder::VideoDecoder(Session *session,
			   Element::Listener *elementListener,
			   RawSource::Listener *sourceListener) :
		CodedToRawFilterElement(session,
					elementListener,
					1,
					nullptr,
					0,
					1,
					sourceListener),
		mInputMedia(nullptr), mOutputMedia(nullptr),
		mInputBufferPool(nullptr), mInputBufferQueue(nullptr),
		mVdec(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mResyncPending(false),
		mVdecFlushPending(false), mVdecStopPending(false),
		mCompleteStopPendingCount(0)
{
	const struct vdef_coded_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	/* Supported input formats */
	supportedInputFormatsCount = vdec_get_supported_input_formats(
		VDEC_DECODER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0) {
		PDRAW_LOG_ERRNO("vdec_get_supported_input_formats",
				-supportedInputFormatsCount);
	} else {
		setCodedVideoMediaFormatCaps(supportedInputFormats,
					     supportedInputFormatsCount);
	}

	setState(CREATED);
}


VideoDecoder::~VideoDecoder(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		PDRAW_LOGW("decoder is still running");

	if (mVdec) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vdec_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int VideoDecoder::start(void)
{
	int ret = 0;
	enum vdef_coded_data_format fmt = VDEF_CODED_DATA_FORMAT_UNKNOWN;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("decoder is not created");
		return -EPROTO;
	}
	setState(STARTING);

	/* Get the input media and port */
	CodedSink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid input media count");
		return -EPROTO;
	}
	mInputMedia = getInputMedia(0);
	if (mInputMedia == nullptr) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port = getInputPort(mInputMedia);
	if (port == nullptr) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid input port");
		return -EPROTO;
	}

	fmt = mInputMedia->format.data_format;
	if (fmt == VDEF_CODED_DATA_FORMAT_UNKNOWN) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid input data format");
		return -EPROTO;
	}

	/* Initialize the decoder */
	struct vdec_config cfg = {};
	cfg.implem = VDEC_DECODER_IMPLEM_AUTO;
	cfg.encoding = mInputMedia->format.encoding;
	cfg.low_delay =
		(mInputMedia->playbackType == PDRAW_PLAYBACK_TYPE_LIVE) ? 1 : 0;
#ifdef BCM_VIDEOCORE
	cfg.preferred_output_format = VDEF_BCM_MMAL_OPAQUE;
#endif /* BCM_VIDEOCORE */
	cfg.android_jvm = mSession->getAndroidJvm();
	cfg.gen_grey_idr = 1;
#if BUILD_LIBVIDEO_DECODE_MEDIACODEC
	struct vdec_config_mediacodec mediacodec_cfg = {};
	if (vdec_get_auto_implem() == VDEC_DECODER_IMPLEM_MEDIACODEC) {
		mediacodec_cfg.implem = VDEC_DECODER_IMPLEM_MEDIACODEC;
		mediacodec_cfg.fake_frame_num = true;
		cfg.implem_cfg = (struct vdec_config_impl *)&mediacodec_cfg;
	}
#endif
	ret = vdec_new(mSession->getLoop(), &cfg, &mDecoderCbs, this, &mVdec);
	if (ret < 0) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("vdec_new", -ret);
		return ret;
	}

	/* Configure the decoder */
	const uint8_t *vps = nullptr, *sps = nullptr, *pps = nullptr;
	size_t vpsSize = 0, spsSize = 0, ppsSize = 0;
	ret = mInputMedia->getPs(
		&vps, &vpsSize, &sps, &spsSize, &pps, &ppsSize);
	if (ret < 0) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("media->getPs", -ret);
		return ret;
	}

	uint32_t start;
	size_t prefix_size = (fmt == VDEF_CODED_DATA_FORMAT_RAW_NALU) ? 0 : 4;

	uint8_t *vpsBuffer = nullptr;
	if (mInputMedia->format.encoding == VDEF_ENCODING_H265) {
		vpsBuffer = (uint8_t *)malloc(prefix_size + vpsSize);
		if (vpsBuffer == nullptr) {
			CodedSink::unlock();
			PDRAW_LOG_ERRNO("malloc:VPS", ENOMEM);
			return -ENOMEM;
		}

		if (fmt != VDEF_CODED_DATA_FORMAT_RAW_NALU) {
			start = (fmt == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
					? htonl(0x00000001)
					: htonl(vpsSize);
			memcpy(vpsBuffer, &start, sizeof(start));
		}
		memcpy(vpsBuffer + prefix_size, vps, vpsSize);
	}

	uint8_t *spsBuffer = (uint8_t *)malloc(prefix_size + spsSize);
	if (spsBuffer == nullptr) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("malloc:SPS", ENOMEM);
		free(vpsBuffer);
		return -ENOMEM;
	}

	if (fmt != VDEF_CODED_DATA_FORMAT_RAW_NALU) {
		start = (fmt == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
				? htonl(0x00000001)
				: htonl(spsSize);
		memcpy(spsBuffer, &start, sizeof(start));
	}
	memcpy(spsBuffer + prefix_size, sps, spsSize);

	uint8_t *ppsBuffer = (uint8_t *)malloc(prefix_size + ppsSize);
	if (ppsBuffer == nullptr) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("malloc:PPS", ENOMEM);
		free(vpsBuffer);
		free(spsBuffer);
		return -ENOMEM;
	}

	if (fmt != VDEF_CODED_DATA_FORMAT_RAW_NALU) {
		start = (fmt == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
				? htonl(0x00000001)
				: htonl(ppsSize);
		memcpy(ppsBuffer, &start, sizeof(start));
	}
	memcpy(ppsBuffer + prefix_size, pps, ppsSize);

	switch (mInputMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		ret = vdec_set_h264_ps(mVdec,
				       spsBuffer,
				       spsSize + prefix_size,
				       ppsBuffer,
				       ppsSize + prefix_size,
				       &mInputMedia->format);
		if (ret < 0) {
			CodedSink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_h264_ps", -ret);
			free(vpsBuffer);
			free(spsBuffer);
			free(ppsBuffer);
			return ret;
		}
		break;
	case VDEF_ENCODING_H265:
		ret = vdec_set_h265_ps(mVdec,
				       vpsBuffer,
				       vpsSize + prefix_size,
				       spsBuffer,
				       spsSize + prefix_size,
				       ppsBuffer,
				       ppsSize + prefix_size,
				       &mInputMedia->format);
		if (ret < 0) {
			CodedSink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_h265_ps", -ret);
			free(vpsBuffer);
			free(spsBuffer);
			free(ppsBuffer);
			return ret;
		}
		break;
	default:
		CodedSink::unlock();
		PDRAW_LOGE("unsupported input media encoding (%s)",
			   vdef_encoding_to_str(mInputMedia->format.encoding));
		free(vpsBuffer);
		free(spsBuffer);
		free(ppsBuffer);
		return -EPROTO;
	}

	free(vpsBuffer);
	free(spsBuffer);
	free(ppsBuffer);

	/* Setup the input port */
	port->channel->setKey(this);
	mInputBufferQueue = vdec_get_input_buffer_queue(mVdec);
	port->channel->setQueue(mInputBufferQueue);
	mInputBufferPool = vdec_get_input_buffer_pool(mVdec);
	port->channel->setPool(mInputBufferPool);

	CodedSink::unlock();

	setState(STARTED);

	return 0;
}


int VideoDecoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("decoder is not started");
		return -EPROTO;
	}
	setState(STOPPING);
	mVdecStopPending = true;

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VideoDecoder::flush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	RawChannel *outputChannel;

	if (mIsFlushed) {
		PDRAW_LOGD("decoder is already flushed, nothing to do");
		completeFlush();
		return 0;
	}

	mVdecFlushPending = true;

	/* Flush the output channels (async) */
	RawSource::lock();
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
	RawSource::unlock();

	/* Flush the decoder (async)
	 * (the input channel queue is flushed by vdec) */
	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vdec_flush", -ret);

	return ret;
}


void VideoDecoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	RawChannel *outputChannel;
	bool pending = false;

	if (mVdecFlushPending)
		return;

	RawSource::lock();
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
	RawSource::unlock();

	if (pending)
		return;

	CodedSink::lock();
	if (mInputMedia != nullptr) {
		mIsFlushed = true;
		if (mInputChannelFlushPending) {
			mInputChannelFlushPending = false;
			CodedChannel *inputChannel =
				getInputChannel(mInputMedia);
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
	CodedSink::unlock();

	completeResync();

	tryStop();
}


int VideoDecoder::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;

	if (mState != STOPPING)
		return 0;

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	RawSource::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (i = outputChannelCount - 1; i >= 0; i--) {
			RawChannel *channel = getOutputChannel(mOutputMedia, i);
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
	RawSource::unlock();

	/* Stop the decoder */
	ret = vdec_stop(mVdec);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vdec_stop", -ret);
		return ret;
	}

	/* Remove the input port */
	CodedSink::lock();
	if (mInputMedia != nullptr) {
		CodedChannel *channel = getInputChannel(mInputMedia);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			channel->setQueue(nullptr);
			channel->setPool(nullptr);
		}

		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = nullptr;
	}
	CodedSink::unlock();

	return 0;
}


void VideoDecoder::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	mCompleteStopPendingCount--;

	RawSource::lock();
	if (mOutputMedia == nullptr) {
		RawSource::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		RawSource::unlock();
		return;
	}

	/* Remove the output port */
	if (RawSource::mListener) {
		RawSource::mListener->onOutputMediaRemoved(this, mOutputMedia);
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = nullptr;
	}

	RawSource::unlock();

exit:
	if ((!mVdecStopPending) && (mCompleteStopPendingCount == 0))
		setState(STOPPED);
}


void VideoDecoder::resync(void)
{
	int ret;

	CodedSink::lock();

	if (mResyncPending) {
		CodedSink::unlock();
		PDRAW_LOGD(
			"%s: decoder is already synchronizing, nothing to do",
			__func__);
		return;
	}

	if (mIsFlushed) {
		CodedSink::unlock();
		PDRAW_LOGD("%s: decoder is already flushed, nothing to do",
			   __func__);
		return;
	}

	mResyncPending = true;

	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vdec_flush", -ret);

	CodedSink::unlock();
}


void VideoDecoder::completeResync(void)
{
	int ret;

	CodedSink::lock();

	if (!mResyncPending) {
		CodedSink::unlock();
		return;
	}

	CodedChannel *inputChannel = getInputChannel(mInputMedia);
	if (inputChannel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
	} else {
		ret = inputChannel->resync();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->resync", -ret);
	}

	mResyncPending = false;
	CodedSink::unlock();
}


int VideoDecoder::createOutputMedia(struct vdef_raw_frame *frameInfo,
				    RawVideoMedia::Frame &frame)
{
	int ret;

	RawSource::lock();

	mOutputMedia = new RawVideoMedia(mSession);
	if (mOutputMedia == nullptr) {
		RawSource::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		RawSource::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frameInfo->format;
	vdef_frame_to_format_info(&frameInfo->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	RawSource::unlock();

	if (RawSource::mListener)
		RawSource::mListener->onOutputMediaAdded(this, mOutputMedia);

	return 0;
}


void VideoDecoder::onChannelQueue(CodedChannel *channel,
				  struct mbuf_coded_video_frame *frame)
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
		PDRAW_LOGE("decoder is not started");
		return;
	}
	if ((mVdecFlushPending) || (mInputChannelFlushPending)) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	CodedSink::lock();
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue();
	if (queue == nullptr) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}
	if (queue != mInputBufferQueue) {
		CodedSink::unlock();
		PDRAW_LOGE("invalid input buffer queue");
		return;
	}

	CodedSink::onChannelQueue(channel, frame);
	mIsFlushed = false;
	CodedSink::unlock();
}


void VideoDecoder::onChannelFlush(CodedChannel *channel)
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


void VideoDecoder::onChannelFlushed(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel key=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getKey());

	completeFlush();
}


void VideoDecoder::onChannelTeardown(CodedChannel *channel)
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


void VideoDecoder::onChannelUnlink(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSource::onChannelUnlink(channel);

	if (mState == STOPPING) {
		mCompleteStopPendingCount++;
		completeStop();
	}
}


void VideoDecoder::frameOutputCb(struct vdec_decoder *dec,
				 int status,
				 struct mbuf_raw_video_frame *out_frame,
				 void *userdata)
{
	int ret;
	VideoDecoder *self = (VideoDecoder *)userdata;
	struct vdef_raw_frame info;
	struct mbuf_ancillary_data *ancillaryData;
	CodedVideoMedia::Frame *in_meta;
	RawVideoMedia::Frame out_meta;
	unsigned int outputChannelCount, i;
	RawChannel *channel;

	if (status != 0) {
		PDRAW_LOGE("decoder error %d(%s), resync required",
			   -status,
			   strerror(-status));
		self->resync();
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
		PDRAW_LOGE("decoder is not started");
		return;
	}
	if ((self->mVdecFlushPending) || (self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->CodedSink::lock();
	if (self->mInputMedia == nullptr) {
		self->CodedSink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		self->CodedSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_raw_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->CodedSink::unlock();
		PDRAW_LOG_ERRNO(
			"mbuf_raw_video_frame_get_ancillary_data:pdraw_in",
			-ret);
		return;
	}
	in_meta = (CodedVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
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
	out_meta.decoderOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, VDEC_ANCILLARY_KEY_OUTPUT_TIME);
	ret = mbuf_ancillary_data_unref(ancillaryData);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -ret);

	/* Remove the PDrAW input ancillary data */
	ret = mbuf_raw_video_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_remove_ancillary_data",
				-ret);

	self->CodedSink::unlock();
	self->RawSource::lock();

	if (self->mOutputMedia == nullptr) {
		/* Create the output media now that the format is known */
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->RawSource::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	} else {
		/* TODO: This should be generic for every filter element */
		/* Update the output media metadata */
		self->mOutputMedia->sessionMeta =
			self->mInputMedia->sessionMeta;
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		self->RawSource::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			channel = self->getOutputChannel(self->mOutputMedia, i);
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

	self->RawSource::unlock();
}


void VideoDecoder::flushCb(struct vdec_decoder *dec, void *userdata)
{
	VideoDecoder *self = (VideoDecoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("decoder is flushed");
	self->mVdecFlushPending = false;

	self->completeFlush();
}


void VideoDecoder::stopCb(struct vdec_decoder *dec, void *userdata)
{
	VideoDecoder *self = (VideoDecoder *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("decoder is stopped");
	self->mVdecStopPending = false;

	self->mCompleteStopPendingCount++;
	self->completeStop();
}

} /* namespace Pdraw */
