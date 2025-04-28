/**
 * Parrot Drones Audio and Video Vector library
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
		mVdec(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mResyncPending(false),
		mVdecFlushPending(false), mVdecStopPending(false)
{
	const struct vdef_coded_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	/* Supported input formats */
	supportedInputFormatsCount =
		vdec_get_all_supported_input_formats(&supportedInputFormats);
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

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	if (mVdec != nullptr) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vdec_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


ssize_t VideoDecoder::preparePsBuffer(const uint8_t *ps,
				      size_t psSize,
				      enum vdef_coded_data_format fmt,
				      uint8_t **ret)
{
	size_t prefixSize = (fmt == VDEF_CODED_DATA_FORMAT_RAW_NALU) ? 0 : 4;
	uint32_t start;

	uint8_t *psBuffer = (uint8_t *)malloc(prefixSize + psSize);
	if (psBuffer == nullptr) {
		ULOG_ERRNO("malloc", ENOMEM);
		return -ENOMEM;
	}

	if (fmt != VDEF_CODED_DATA_FORMAT_RAW_NALU) {
		start = (fmt == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
				? htonl(0x00000001)
				: htonl(psSize);
		memcpy(psBuffer, &start, sizeof(start));
	}
	memcpy(psBuffer + prefixSize, ps, psSize);

	*ret = psBuffer;
	return prefixSize + psSize;
}


int VideoDecoder::start(void)
{
	int ret = 0, err;
	enum vdef_coded_data_format fmt = VDEF_CODED_DATA_FORMAT_UNKNOWN;
	const uint8_t *vps = nullptr, *sps = nullptr, *pps = nullptr;
	size_t vpsSize = 0, spsSize = 0, ppsSize = 0;
	uint8_t *vpsBuffer = nullptr, *spsBuffer = nullptr,
		*ppsBuffer = nullptr;
	ssize_t ret1;
	InputPort *port = nullptr;
	struct vdec_config cfg = {};
	Channel *c = nullptr;
	CodedVideoChannel *channel = nullptr;

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
	mInputMedia = dynamic_cast<CodedVideoMedia *>(getInputMedia(0));
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

	fmt = mInputMedia->format.data_format;
	if (fmt == VDEF_CODED_DATA_FORMAT_UNKNOWN) {
		Sink::unlock();
		PDRAW_LOGE("invalid input data format");
		ret = -EPROTO;
		goto error;
	}

	/* Initialize the decoder */
	cfg.implem = vdec_get_auto_implem_by_coded_format(&mInputMedia->format);
	if (cfg.implem == VDEC_DECODER_IMPLEM_AUTO) {
		Sink::unlock();
		char *fmt = vdef_coded_format_to_str(&mInputMedia->format);
		PDRAW_LOGE("no implementation found for format %s", fmt);
		free(fmt);
		ret = -EPROTO;
		goto error;
	}
	cfg.encoding = mInputMedia->format.encoding;
	cfg.low_delay =
		(mInputMedia->playbackType == PDRAW_PLAYBACK_TYPE_LIVE) ? 1 : 0;
	cfg.gen_grey_idr = 1;
	ret = vdec_new(mSession->getLoop(), &cfg, &mDecoderCbs, this, &mVdec);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("vdec_new", -ret);
		goto error;
	}

	/* Configure the decoder */
	switch (mInputMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		ret = mInputMedia->getPs(
			nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("media->getPs", -ret);
			goto error;
		}

		ret1 = preparePsBuffer(sps, spsSize, fmt, &spsBuffer);
		if (ret1 < 0) {
			Sink::unlock();
			ret = ret1;
			PDRAW_LOG_ERRNO("preparePsBuffer:SPS", -ret);
			goto error;
			;
		}
		spsSize = ret1;

		ret1 = preparePsBuffer(pps, ppsSize, fmt, &ppsBuffer);
		if (ret1 < 0) {
			Sink::unlock();
			ret = ret1;
			PDRAW_LOG_ERRNO("preparePsBuffer:PPS", -ret);
			free(spsBuffer);
			goto error;
		}
		ppsSize = ret1;

		ret = vdec_set_h264_ps(mVdec,
				       spsBuffer,
				       spsSize,
				       ppsBuffer,
				       ppsSize,
				       &mInputMedia->format);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_h264_ps", -ret);
			free(spsBuffer);
			free(ppsBuffer);
			goto error;
		}

		free(spsBuffer);
		free(ppsBuffer);
		break;

	case VDEF_ENCODING_H265:
		ret = mInputMedia->getPs(
			&vps, &vpsSize, &sps, &spsSize, &pps, &ppsSize);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("media->getPs", -ret);
			goto error;
		}

		ret1 = preparePsBuffer(vps, vpsSize, fmt, &vpsBuffer);
		if (ret1 < 0) {
			Sink::unlock();
			ret = ret1;
			PDRAW_LOG_ERRNO("preparePsBuffer:VPS", -ret);
			goto error;
		}
		vpsSize = ret1;

		ret1 = preparePsBuffer(sps, spsSize, fmt, &spsBuffer);
		if (ret1 < 0) {
			Sink::unlock();
			ret = ret1;
			PDRAW_LOG_ERRNO("preparePsBuffer:SPS", -ret);
			free(vpsBuffer);
			goto error;
		}
		spsSize = ret1;

		ret1 = preparePsBuffer(pps, ppsSize, fmt, &ppsBuffer);
		if (ret1 < 0) {
			Sink::unlock();
			ret = ret1;
			PDRAW_LOG_ERRNO("preparePsBuffer:PPS", -ret);
			free(vpsBuffer);
			free(spsBuffer);
			goto error;
		}
		ppsSize = ret1;

		ret = vdec_set_h265_ps(mVdec,
				       vpsBuffer,
				       vpsSize,
				       spsBuffer,
				       spsSize,
				       ppsBuffer,
				       ppsSize,
				       &mInputMedia->format);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_h265_ps", -ret);
			free(vpsBuffer);
			free(spsBuffer);
			free(ppsBuffer);
			goto error;
		}

		free(vpsBuffer);
		free(spsBuffer);
		free(ppsBuffer);
		break;

	case VDEF_ENCODING_JPEG:
		ret = vdec_set_jpeg_params(mVdec, &mInputMedia->info);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_jpeg_params", -ret);
			goto error;
		}
		break;

	default:
		Sink::unlock();
		PDRAW_LOGE("unsupported input media encoding (%s)",
			   vdef_encoding_to_str(mInputMedia->format.encoding));
		ret = -EPROTO;
		goto error;
	}

	/* Setup the input port */
	c = port->channel;
	channel = dynamic_cast<CodedVideoChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		ret = -EPROTO;
		goto error;
	}
	mInputBufferQueue = vdec_get_input_buffer_queue(mVdec);
	channel->setQueue(this, mInputBufferQueue);
	mInputBufferPool = vdec_get_input_buffer_pool(mVdec);
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


int VideoDecoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTING) && (mState != STARTED)) {
		PDRAW_LOGE("%s: decoder is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);
	mVdecStopPending = true;

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


int VideoDecoder::flush(void)
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
	 * (the input channel queue is flushed by vdec) */
	if (mVdec != nullptr) {
		if (!mVdecFlushPending) {
			ret = vdec_flush(mVdec, 1);
			if (ret < 0)
				PDRAW_LOG_ERRNO("vdec_flush", -ret);
			else
				mVdecFlushPending = true;
		}
	} else {
		completeFlush();
	}

	return ret;
}


void VideoDecoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;
	bool pending = false;

	if (mVdecFlushPending)
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
			CodedVideoChannel *inputChannel =
				dynamic_cast<CodedVideoChannel *>(
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

	completeResync();

	tryStop();
}


void VideoDecoder::idleCompleteFlush(void *userdata)
{
	VideoDecoder *self = (VideoDecoder *)userdata;
	self->completeFlush();
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
	if (mVdec != nullptr) {
		ret = vdec_stop(mVdec);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("vdec_stop", -ret);
			return ret;
		}
	}
	/* Else, delay completeStop() after removing the input port */

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		CodedVideoChannel *channel = dynamic_cast<CodedVideoChannel *>(
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

	if (mVdec == nullptr) {
		mVdecStopPending = false;
		completeStop();
	}

	return 0;
}


void VideoDecoder::completeStop(void)
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
	if ((!mVdecStopPending) && (mOutputMedia == nullptr))
		setState(STOPPED);
}


void VideoDecoder::resync(void)
{
	int ret;

	Sink::lock();

	if (mResyncPending) {
		Sink::unlock();
		PDRAW_LOGD(
			"%s: decoder is already synchronizing, nothing to do",
			__func__);
		return;
	}

	if (mIsFlushed) {
		Sink::unlock();
		PDRAW_LOGD("%s: decoder is already flushed, nothing to do",
			   __func__);
		return;
	}

	mResyncPending = true;

	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vdec_flush", -ret);
	else
		mVdecFlushPending = true;

	Sink::unlock();
}


void VideoDecoder::completeResync(void)
{
	int ret;

	Sink::lock();

	if (!mResyncPending) {
		Sink::unlock();
		return;
	}

	CodedVideoChannel *inputChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(mInputMedia));
	if (inputChannel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
	} else {
		ret = inputChannel->resync();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->resync", -ret);
	}

	mResyncPending = false;
	Sink::unlock();
}


int VideoDecoder::createOutputMedia(const struct vdef_raw_frame *frameInfo,
				    const RawVideoMedia::Frame &frame)
{
	int ret;

	Source::lock();

	mOutputMedia = new RawVideoMedia(mSession);
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
	vdef_frame_to_format_info(&frameInfo->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia, nullptr);

	return 0;
}


void VideoDecoder::onCodedVideoChannelQueue(
	CodedVideoChannel *channel,
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
		PDRAW_LOGE("frame input: decoder is not started");
		return;
	}
	if ((mVdecFlushPending) || (mInputChannelFlushPending)) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue(this);
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

	Sink::onCodedVideoChannelQueue(channel, frame);
	mIsFlushed = false;
	Sink::unlock();
}


void VideoDecoder::onChannelFlush(Channel *channel)
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


void VideoDecoder::onChannelFlushed(Channel *channel)
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


void VideoDecoder::onChannelTeardown(Channel *channel)
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


void VideoDecoder::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING)
		completeStop();
}


void VideoDecoder::onChannelSessionMetaUpdate(Channel *channel)
{
	struct vmeta_session tmpSessionMeta;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("%s: input media not found", __func__);
		return;
	}
	tmpSessionMeta = mInputMedia->sessionMeta;
	Sink::unlock();

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	mOutputMedia->sessionMeta = tmpSessionMeta;
	Source::unlock();

	PDRAW_LOGD("updating session metadata");

	FilterElement::onChannelSessionMetaUpdate(channel);
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
		PDRAW_LOGE("frame output: decoder is not started");
		return;
	}
	if ((self->mVdecFlushPending) || (self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_raw_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO(
			"mbuf_raw_video_frame_get_ancillary_data:pdraw_in",
			-ret);
		return;
	}
	in_meta = (CodedVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
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
		self->Source::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			Channel *c =
				self->getOutputChannel(self->mOutputMedia, i);
			RawVideoChannel *channel =
				dynamic_cast<RawVideoChannel *>(c);
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
	self->completeStop();
}

} /* namespace Pdraw */
