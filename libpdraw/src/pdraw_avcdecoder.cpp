/**
 * Parrot Drones Awesome Video Viewer Library
 * H.264/AVC decoder interface
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

#include "pdraw_avcdecoder.hpp"
#include "pdraw_session.hpp"

#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>

#include <vector>

#define ULOG_TAG pdraw_decavc
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_decavc);

namespace Pdraw {


const struct vdec_cbs AvcDecoder::mDecoderCbs = {
	.frame_output = &AvcDecoder::frameOutputCb,
	.flush = &AvcDecoder::flushCb,
	.stop = &AvcDecoder::stopCb,
};


AvcDecoder::AvcDecoder(Session *session,
		       Element::Listener *elementListener,
		       Source::Listener *sourceListener) :
		Element(session, elementListener),
		Sink(Media::Type::VIDEO,
		     VideoMedia::Format::H264,
		     VideoMedia::H264BitstreamFormat::H264_UNKNOWN),
		Source(sourceListener)
{
	int ret;
	struct vdec_config cfg;
	uint32_t supported_input_format;

	Element::mName = "AvcDecoder";
	Sink::mName = "AvcDecoder";
	Source::mName = "AvcDecoder";
	mInputMedia = NULL;
	mOutputMedia = NULL;
	mInputBufferPool = NULL;
	mInputBufferQueue = NULL;
	mVdec = NULL;
	mFrameIndex = 0;
	mIsFlushed = true;
	mIsSync = true;
	mInputChannelFlushPending = false;
	mResyncPending = false;
	mVdecFlushPending = false;
	mVdecStopPending = false;
	mCompleteStopPendingCount = 0;
	mLastTimestamp = UINT64_MAX;

	/* Decide the input bitstream format */
	supported_input_format =
		vdec_get_supported_input_format(VDEC_DECODER_IMPLEM_AUTO);
	if (supported_input_format & VDEC_INPUT_FORMAT_BYTE_STREAM) {
		mVdecInputFormat = VDEC_INPUT_FORMAT_BYTE_STREAM;
		mInputFormat = VideoMedia::H264BitstreamFormat::BYTE_STREAM;
		setVideoMediaSubFormatCaps(
			VideoMedia::H264BitstreamFormat::BYTE_STREAM);
	} else if (supported_input_format & VDEC_INPUT_FORMAT_AVCC) {
		mVdecInputFormat = VDEC_INPUT_FORMAT_AVCC;
		mInputFormat = VideoMedia::H264BitstreamFormat::AVCC;
		setVideoMediaSubFormatCaps(
			VideoMedia::H264BitstreamFormat::AVCC);
	} else {
		mVdecInputFormat = (enum vdec_input_format)0;
		mInputFormat = VideoMedia::H264BitstreamFormat::H264_UNKNOWN;
		ULOGE("unsuppoted input format");
		goto error;
	}

	/* Initialize the decoder */
	memset(&cfg, 0, sizeof(cfg));
	cfg.implem = VDEC_DECODER_IMPLEM_AUTO;
	cfg.encoding = VDEC_ENCODING_H264;
	cfg.low_delay =
		(mSession->getSessionType() == PDRAW_SESSION_TYPE_LIVE) ? 1 : 0;
#ifdef BCM_VIDEOCORE
	cfg.preferred_output_format = VDEC_OUTPUT_FORMAT_MMAL_OPAQUE;
#endif /* BCM_VIDEOCORE */
	cfg.android_jvm = mSession->getAndroidJvm();
	ret = vdec_new(&cfg, &mDecoderCbs, this, &mVdec);
	if (ret < 0) {
		ULOG_ERRNO("vdec_new", -ret);
		goto error;
	}

	setState(CREATED);
	return;

error:
	if (mVdec) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			ULOG_ERRNO("vdec_destroy", -ret);
		mVdec = NULL;
	}
}


AvcDecoder::~AvcDecoder(void)
{
	int ret;

	if (mState != STOPPED)
		ULOGW("decoder is still running");

	if (mVdec) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			ULOG_ERRNO("vdec_destroy", -ret);
		mVdec = NULL;
		mInputBufferQueue = NULL;
	}

	if (mOutputMedia != NULL)
		ULOGW("output media was not properly removed");
}


int AvcDecoder::start(void)
{
	int ret = 0;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		ULOGE("decoder is not created");
		return -EPROTO;
	}
	setState(STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		ULOGE("invalid input media count");
		return -EPROTO;
	}
	Media *media = getInputMedia(0);
	if (media == NULL) {
		Sink::unlock();
		ULOGE("invalid input media");
		return -EPROTO;
	}
	mInputMedia = dynamic_cast<VideoMedia *>(media);
	if (mInputMedia == NULL) {
		Sink::unlock();
		ULOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port = getInputPort(mInputMedia);
	if (port == NULL) {
		Sink::unlock();
		ULOGE("invalid input port");
		return -EPROTO;
	}

	/* Configure the decoder */
	uint8_t *sps = NULL, *pps = NULL;
	size_t spsSize = 0, ppsSize = 0;
	ret = mInputMedia->getSpsPps(&sps, &spsSize, &pps, &ppsSize);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("media->getSpsPps", -ret);
		return ret;
	}

	uint32_t start;
	uint8_t *spsBuffer = (uint8_t *)malloc(spsSize + 4);
	if (spsBuffer == NULL) {
		Sink::unlock();
		ULOG_ERRNO("malloc:SPS", ENOMEM);
		return -ENOMEM;
	}

	start = (mVdecInputFormat == VDEC_INPUT_FORMAT_BYTE_STREAM)
			? htonl(0x00000001)
			: htonl(spsSize);
	*((uint32_t *)spsBuffer) = start;
	memcpy(spsBuffer + 4, sps, spsSize);

	uint8_t *ppsBuffer = (uint8_t *)malloc(ppsSize + 4);
	if (ppsBuffer == NULL) {
		Sink::unlock();
		ULOG_ERRNO("malloc:PPS", ENOMEM);
		free(spsBuffer);
		return -ENOMEM;
	}

	start = (mVdecInputFormat == VDEC_INPUT_FORMAT_BYTE_STREAM)
			? htonl(0x00000001)
			: htonl(ppsSize);
	*((uint32_t *)ppsBuffer) = start;
	memcpy(ppsBuffer + 4, pps, ppsSize);

	ret = vdec_set_sps_pps(mVdec,
			       spsBuffer,
			       spsSize + 4,
			       ppsBuffer,
			       ppsSize + 4,
			       mVdecInputFormat);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("vdec_set_sps_pps", -ret);
		free(spsBuffer);
		free(ppsBuffer);
		return ret;
	}

	free(spsBuffer);
	free(ppsBuffer);

	/* Setup the input port */
	port->channel->setKey(this);
	mInputBufferQueue = vdec_get_input_buffer_queue(mVdec);
	port->channel->setQueue(mInputBufferQueue);
	mInputBufferPool = vdec_get_input_buffer_pool(mVdec);
	port->channel->setPool(mInputBufferPool);

	/* Create the output port */
	Source::lock();
	mOutputMedia = new VideoMedia(mSession);
	if (mOutputMedia == NULL) {
		Source::unlock();
		Sink::unlock();
		ULOGE("media allocation failed");
		return -ENOMEM;
	}
	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		Sink::unlock();
		ULOG_ERRNO("addOutputPort", -ret);
		return ret;
	}
#ifdef BCM_VIDEOCORE
	mOutputMedia->format = VideoMedia::Format::OPAQUE;
	mOutputMedia->opaqueFormat = VideoMedia::OpaqueFormat::MMAL;
#else /* BCM_VIDEOCORE */
	mOutputMedia->format = VideoMedia::Format::YUV;
#endif /* BCM_VIDEOCORE */
	mOutputMedia->width = mInputMedia->width;
	mOutputMedia->height = mInputMedia->height;
	mOutputMedia->cropLeft = mInputMedia->cropLeft;
	mOutputMedia->cropWidth = mInputMedia->cropWidth;
	mOutputMedia->cropTop = mInputMedia->cropTop;
	mOutputMedia->cropHeight = mInputMedia->cropHeight;
	mOutputMedia->sarWidth = mInputMedia->sarWidth;
	mOutputMedia->sarHeight = mInputMedia->sarHeight;
	mOutputMedia->hfov = mInputMedia->hfov;
	mOutputMedia->vfov = mInputMedia->vfov;
	mOutputMedia->fullRange = mInputMedia->fullRange;

	Source::unlock();
	Sink::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(this, mOutputMedia);

	setState(STARTED);

	return 0;
}


int AvcDecoder::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		ULOGE("decoder is not started");
		return -EPROTO;
	}
	setState(STOPPING);
	mVdecStopPending = true;

	/* Abort waiting on the input buffer pool */
	if (mInputBufferPool) {
		ret = vbuf_pool_abort(mInputBufferPool);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_pool_abort:input", -ret);
			return ret;
		}
	}

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int AvcDecoder::flush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;

	if (mIsFlushed) {
		ULOGD("decoder is already flushed, nothing to do");
		completeFlush();
		return 0;
	}

	mVdecFlushPending = true;

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != NULL) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == NULL) {
				ULOGW("failed to get output channel "
				      "at index %d",
				      i);
				continue;
			}
			ret = outputChannel->flush();
			if (ret < 0)
				ULOG_ERRNO("channel->flush", -ret);
		}
	}
	Source::unlock();

	/* Flush the decoder (async)
	 * (the input channel queue is flushed by vdec) */
	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		ULOG_ERRNO("vdec_flush", -ret);

	return ret;
}


void AvcDecoder::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	Channel *outputChannel;
	bool pending = false;

	if (mVdecFlushPending)
		return;

	Source::lock();
	if (mOutputMedia != NULL) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == NULL) {
				ULOGW("failed to get output channel "
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
	if (mInputMedia != NULL) {
		mIsFlushed = true;
		if (mInputChannelFlushPending) {
			mInputChannelFlushPending = false;
			Channel *inputChannel = getInputChannel(mInputMedia);
			if (inputChannel == NULL) {
				ULOGE("failed to get input channel");
			} else {
				ret = inputChannel->flushDone();
				if (ret < 0)
					ULOG_ERRNO("channel->flushDone", -ret);
			}
		}
	}
	Sink::unlock();

	completeResync();

	tryStop();
}


int AvcDecoder::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;
	Channel *channel;

	if (mState != STOPPING)
		return 0;

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != NULL) {
		channel = getInputChannel(mInputMedia);
		if (channel == NULL) {
			ULOGE("failed to get channel");
		} else {
			channel->setQueue(NULL);
			channel->setPool(NULL);
		}

		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			ULOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = NULL;
	}
	Sink::unlock();

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	Source::lock();
	if (mOutputMedia != NULL) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (i = outputChannelCount - 1; i >= 0; i--) {
			channel = getOutputChannel(mOutputMedia, i);
			if (channel == NULL) {
				ULOGW("failed to get channel at index %d", i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				ULOG_ERRNO("channel->teardown", -ret);
		}
	}
	Source::unlock();

	/* Stop the decoder */
	ret = vdec_stop(mVdec);
	if (ret < 0) {
		ULOG_ERRNO("vdec_stop", -ret);
		return ret;
	}

	return 0;
}


void AvcDecoder::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	mCompleteStopPendingCount--;

	Source::lock();
	if (mOutputMedia == NULL) {
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
		ULOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = NULL;
	}

	Source::unlock();

exit:
	if ((!mVdecStopPending) && (mCompleteStopPendingCount == 0))
		setState(STOPPED);
}


void AvcDecoder::resync(void)
{
	int ret;

	Sink::lock();

	if (mIsSync) {
		Sink::unlock();
		ULOGD("decoder is already synchronized, nothing to do");
		return;
	}

	mResyncPending = true;

	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		ULOG_ERRNO("vdec_flush", -ret);

	Sink::unlock();
}


void AvcDecoder::completeResync(void)
{
	int ret;

	Sink::lock();

	if (!mResyncPending) {
		Sink::unlock();
		return;
	}

	Channel *inputChannel = getInputChannel(mInputMedia);
	if (inputChannel == NULL) {
		ULOGE("failed to get input channel");
	} else {
		ret = inputChannel->resync();
		if (ret < 0)
			ULOG_ERRNO("channel->resync", -ret);
	}

	mResyncPending = false;
	mIsSync = true;
	Sink::unlock();
}


void AvcDecoder::onChannelQueue(Channel *channel, vbuf_buffer *buf)
{
	VideoMedia::Frame *in_meta;
	struct vdec_input_metadata *vdec_meta;
	unsigned int level = 0;

	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}
	if (buf == NULL) {
		ULOG_ERRNO("buf", EINVAL);
		return;
	}
	if (mState != STARTED) {
		ULOGE("decoder is not started");
		return;
	}
	if ((mVdecFlushPending) || (mInputChannelFlushPending)) {
		ULOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	struct vbuf_queue *queue = channel->getQueue();
	if (queue == NULL) {
		Sink::unlock();
		ULOGE("invalid queue");
		return;
	}
	if (queue != mInputBufferQueue) {
		Sink::unlock();
		ULOGE("invalid input buffer queue");
		return;
	}

	/* Set the vdec metadata from the PDrAW metadata */
	int ret = vbuf_metadata_get(
		buf, mInputMedia, &level, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_get", -ret);
		return;
	}
	if (in_meta->format != VideoMedia::Format::H264) {
		Sink::unlock();
		ULOGE("unsupported input format");
		return;
	}
	ret = vbuf_metadata_add(buf,
				mVdec,
				level + 1,
				sizeof(*vdec_meta),
				(uint8_t **)&vdec_meta);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_add", -ret);
		return;
	}
	vdec_meta->timestamp = in_meta->ntpRawTimestamp;
	/* Decoding timestamps for the decoder must be monotonic; avoid
	 * duplicate or rollback timestamps by faking the timestamp as
	 * the last timestamp + 1 */
	if ((mLastTimestamp != UINT64_MAX) &&
	    (vdec_meta->timestamp == mLastTimestamp)) {
		ULOGW("duplicate timestamp (%" PRIu64 "), incrementing",
		      mLastTimestamp);
		vdec_meta->timestamp = mLastTimestamp + 1;
	} else if ((mLastTimestamp != UINT64_MAX) &&
		   (vdec_meta->timestamp < mLastTimestamp)) {
		ULOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
		      ", incrementing",
		      mLastTimestamp,
		      vdec_meta->timestamp);
		vdec_meta->timestamp = mLastTimestamp + 1;
	}
	mLastTimestamp = vdec_meta->timestamp;
	vdec_meta->index = mFrameIndex++;
	switch (in_meta->h264Frame.format) {
	case VideoMedia::H264BitstreamFormat::BYTE_STREAM:
		vdec_meta->format = VDEC_INPUT_FORMAT_BYTE_STREAM;
		break;
	case VideoMedia::H264BitstreamFormat::AVCC:
		vdec_meta->format = VDEC_INPUT_FORMAT_AVCC;
		break;
	default:
		Sink::unlock();
		ULOG_ERRNO("in_meta->h264Frame.format", ENOSYS);
		return;
	}
	vdec_meta->complete = in_meta->h264Frame.isComplete;
	vdec_meta->errors = in_meta->hasErrors;
	vdec_meta->ref = in_meta->h264Frame.isRef;
	vdec_meta->silent = in_meta->isSilent;
	struct timespec ts = {0, 0};
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &vdec_meta->input_time);

	Sink::onChannelQueue(channel, buf);
	mIsFlushed = false;
	mIsSync = in_meta->h264Frame.isSync;
	Sink::unlock();
}


void AvcDecoder::onChannelFlush(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	ULOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);
}


void AvcDecoder::onChannelFlushed(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}
	ULOGD("'%s': channel flushed media id=%d type=%s (channel key=%p)",
	      Source::mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* Post a message on the loop thread */
	mSession->asyncAvcDecoderCompleteFlush(this);
}


void AvcDecoder::onChannelTeardown(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	ULOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		ULOG_ERRNO("stop", -ret);
}


void AvcDecoder::onChannelUnlink(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING) {
		/* Post a message on the loop thread */
		mCompleteStopPendingCount++;
		mSession->asyncAvcDecoderCompleteStop(this);
	}
}


void AvcDecoder::onChannelSos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelSos(channel);

	Source::lock();
	if (mOutputMedia != NULL) {
		int ret = Source::sendDownstreamEvent(
			mOutputMedia, Channel::DownstreamEvent::SOS);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void AvcDecoder::onChannelEos(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelEos(channel);

	Source::lock();
	if (mOutputMedia != NULL) {
		int ret = Source::sendDownstreamEvent(
			mOutputMedia, Channel::DownstreamEvent::EOS);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void AvcDecoder::onChannelReconfigure(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelReconfigure(channel);

	Source::lock();
	if (mOutputMedia != NULL) {
		int ret = Source::sendDownstreamEvent(
			mOutputMedia, Channel::DownstreamEvent::RECONFIGURE);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void AvcDecoder::onChannelTimeout(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelTimeout(channel);

	Source::lock();
	if (mOutputMedia != NULL) {
		int ret = Source::sendDownstreamEvent(
			mOutputMedia, Channel::DownstreamEvent::TIMEOUT);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void AvcDecoder::onChannelPhotoTrigger(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::onChannelPhotoTrigger(channel);

	Source::lock();
	if (mOutputMedia != NULL) {
		int ret = Source::sendDownstreamEvent(
			mOutputMedia, Channel::DownstreamEvent::PHOTO_TRIGGER);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
	}
	Source::unlock();
}


void AvcDecoder::frameOutputCb(struct vdec_decoder *dec,
			       int status,
			       struct vbuf_buffer *out_buf,
			       void *userdata)
{
	int ret;
	AvcDecoder *decoder = (AvcDecoder *)userdata;
	struct vdec_output_metadata *vdec_meta;
	VideoMedia::Frame *in_meta;
	VideoMedia::Frame _out_meta;
	VideoMedia::Frame *out_meta;
	unsigned int level = 0;
	unsigned int outputChannelCount, i;
	Channel *channel;

	if (status != 0) {
		ULOGE("decoder error, resync required");
		decoder->mSession->asyncAvcDecoderResync(decoder);
		return;
	}

	if (userdata == NULL) {
		ULOG_ERRNO("userdata", EINVAL);
		return;
	}
	if (out_buf == NULL) {
		ULOG_ERRNO("out_buf", EINVAL);
		return;
	}
	if (decoder->mState != STARTED) {
		ULOGE("decoder is not started");
		return;
	}
	if ((decoder->mVdecFlushPending) ||
	    (decoder->mInputChannelFlushPending)) {
		ULOGI("frame output: flush pending, discard frame");
		return;
	}

	decoder->Sink::lock();
	if (decoder->mInputMedia == NULL) {
		decoder->Sink::unlock();
		ULOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	/* Set the PDrAW metadata from the vdec metadata */
	ret = vbuf_metadata_get(
		out_buf, decoder->mVdec, NULL, NULL, (uint8_t **)&vdec_meta);
	if (ret < 0) {
		decoder->Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_get:vdec", -ret);
		return;
	}
	ret = vbuf_metadata_get(out_buf,
				decoder->mInputMedia,
				&level,
				NULL,
				(uint8_t **)&in_meta);
	if (ret < 0) {
		decoder->Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_get:pdraw_in", -ret);
		return;
	}
	memset(&_out_meta, 0, sizeof(_out_meta));
	if (vdec_meta->format == VDEC_OUTPUT_FORMAT_MMAL_OPAQUE) {
		_out_meta.format = VideoMedia::Format::OPAQUE;
		_out_meta.opaqueFrame.format = VideoMedia::OpaqueFormat::MMAL;
		_out_meta.opaqueFrame.stride = vdec_meta->plane_stride[0];
		_out_meta.opaqueFrame.width = vdec_meta->width;
		_out_meta.opaqueFrame.height = vdec_meta->height;
		_out_meta.opaqueFrame.sarWidth = vdec_meta->sar_width;
		_out_meta.opaqueFrame.sarHeight = vdec_meta->sar_height;
		_out_meta.opaqueFrame.cropLeft = vdec_meta->crop_left;
		_out_meta.opaqueFrame.cropTop = vdec_meta->crop_top;
		_out_meta.opaqueFrame.cropWidth = vdec_meta->crop_width;
		_out_meta.opaqueFrame.cropHeight = vdec_meta->crop_height;
		_out_meta.opaqueFrame.fullRange = vdec_meta->full_range;
	} else {
		_out_meta.format = VideoMedia::Format::YUV;
		switch (vdec_meta->format) {
		case VDEC_OUTPUT_FORMAT_I420:
			_out_meta.yuvFrame.format = VideoMedia::YuvFormat::I420;
			break;
		case VDEC_OUTPUT_FORMAT_NV12:
			_out_meta.yuvFrame.format = VideoMedia::YuvFormat::NV12;
			break;
		default:
			_out_meta.yuvFrame.format =
				VideoMedia::YuvFormat::YUV_UNKNOWN;
			break;
		}
		_out_meta.yuvFrame.planeOffset[0] = vdec_meta->plane_offset[0];
		_out_meta.yuvFrame.planeOffset[1] = vdec_meta->plane_offset[1];
		_out_meta.yuvFrame.planeOffset[2] = vdec_meta->plane_offset[2];
		_out_meta.yuvFrame.planeStride[0] = vdec_meta->plane_stride[0];
		_out_meta.yuvFrame.planeStride[1] = vdec_meta->plane_stride[1];
		_out_meta.yuvFrame.planeStride[2] = vdec_meta->plane_stride[2];
		_out_meta.yuvFrame.width = vdec_meta->width;
		_out_meta.yuvFrame.height = vdec_meta->height;
		_out_meta.yuvFrame.sarWidth = vdec_meta->sar_width;
		_out_meta.yuvFrame.sarHeight = vdec_meta->sar_height;
		_out_meta.yuvFrame.cropLeft = vdec_meta->crop_left;
		_out_meta.yuvFrame.cropTop = vdec_meta->crop_top;
		_out_meta.yuvFrame.cropWidth = vdec_meta->crop_width;
		_out_meta.yuvFrame.cropHeight = vdec_meta->crop_height;
		_out_meta.yuvFrame.fullRange = vdec_meta->full_range;
	}
	_out_meta.hasErrors = vdec_meta->errors;
	_out_meta.isSilent = vdec_meta->silent;
	_out_meta.ntpTimestamp = in_meta->ntpTimestamp;
	_out_meta.ntpUnskewedTimestamp = in_meta->ntpUnskewedTimestamp;
	_out_meta.ntpRawTimestamp = in_meta->ntpRawTimestamp;
	_out_meta.ntpRawUnskewedTimestamp = in_meta->ntpRawUnskewedTimestamp;
	_out_meta.playTimestamp = in_meta->playTimestamp;
	_out_meta.captureTimestamp = in_meta->captureTimestamp;
	_out_meta.localTimestamp = in_meta->localTimestamp;
	_out_meta.demuxOutputTimestamp = in_meta->demuxOutputTimestamp;
	_out_meta.decoderOutputTimestamp = vdec_meta->output_time;

	/* Frame metadata */
	if (in_meta->hasMetadata) {
		memcpy(&_out_meta.metadata,
		       &in_meta->metadata,
		       sizeof(struct vmeta_frame));
		_out_meta.hasMetadata = true;
	} else {
		_out_meta.hasMetadata = false;
	}

#if 0
	/* TODO: remove debug */
	ULOGI("frame #%u dequeue=%.2fms decode=%.2fms",
		vdec_meta->index,
		(float)(vdec_meta->dequeue_time -
			vdec_meta->input_time) / 1000.,
		(float)(vdec_meta->output_time -
			vdec_meta->dequeue_time) / 1000.);
#endif

	/* Remove the vdec metadata */
	ret = vbuf_metadata_remove(out_buf, decoder->mVdec);
	if (ret < 0)
		ULOG_ERRNO("vbuf_metadata_remove", -ret);

	/* Remove the PDrAW input metadata */
	ret = vbuf_metadata_remove(out_buf, decoder->mInputMedia);
	if (ret < 0)
		ULOG_ERRNO("vbuf_metadata_remove", -ret);

	decoder->Sink::unlock();
	decoder->Source::lock();

	if (decoder->mOutputMedia == NULL) {
		decoder->Source::unlock();
		ULOG_ERRNO("invalid output media", EPROTO);
		return;
	}

	/* Add the PDrAW output metadata */
	ret = vbuf_metadata_add(out_buf,
				decoder->mOutputMedia,
				level,
				sizeof(*out_meta),
				(uint8_t **)&out_meta);
	if (ret < 0) {
		decoder->Source::unlock();
		ULOG_ERRNO("vbuf_metadata_add", -ret);
		return;
	}
	memcpy(out_meta, &_out_meta, sizeof(*out_meta));

	/* Push the frame (unless it is silent) */
	if (!_out_meta.isSilent) {
		outputChannelCount =
			decoder->getOutputChannelCount(decoder->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			channel = decoder->getOutputChannel(
				decoder->mOutputMedia, i);
			if (channel == NULL) {
				ULOGE("failed to get channel at index %d", i);
				continue;
			}
			ret = channel->queue(out_buf);
			if (ret < 0)
				ULOG_ERRNO("channel->queue", -ret);
		}
	} else {
		ULOGD("silent frame (ignored)");
	}

	decoder->Source::unlock();
}


void AvcDecoder::flushCb(struct vdec_decoder *dec, void *userdata)
{
	AvcDecoder *decoder = (AvcDecoder *)userdata;

	if (userdata == NULL) {
		ULOG_ERRNO("userdata", EINVAL);
		return;
	}

	ULOGD("decoder is flushed");
	decoder->mVdecFlushPending = false;

	/* Post a message on the loop thread */
	decoder->mSession->asyncAvcDecoderCompleteFlush(decoder);
}


void AvcDecoder::stopCb(struct vdec_decoder *dec, void *userdata)
{
	AvcDecoder *decoder = (AvcDecoder *)userdata;

	if (userdata == NULL) {
		ULOG_ERRNO("userdata", EINVAL);
		return;
	}

	ULOGD("decoder is stopped");
	decoder->mVdecStopPending = false;

	/* Post a message on the loop thread */
	decoder->mCompleteStopPendingCount++;
	decoder->mSession->asyncAvcDecoderCompleteStop(decoder);
}

} /* namespace Pdraw */
