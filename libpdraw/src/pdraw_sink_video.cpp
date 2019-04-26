/**
 * Parrot Drones Awesome Video Viewer Library
 * Application video sink
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

#include "pdraw_sink_video.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#define ULOG_TAG pdraw_sinkvid
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_sinkvid);

namespace Pdraw {


VideoSink::VideoSink(Session *session,
		     unsigned int requiredFormat,
		     Element::Listener *elementListener) :
		Element(session, elementListener),
		Sink(Media::Type::VIDEO,
		     VideoMedia::Format::YUV | VideoMedia::Format::H264,
		     VideoMedia::YuvFormat::I420 | VideoMedia::YuvFormat::NV12 |
			     VideoMedia::H264BitstreamFormat::BYTE_STREAM |
			     VideoMedia::H264BitstreamFormat::AVCC)
{
	Element::mName = "VideoSink";
	Sink::mName = "VideoSink";
	mVideoSinkListener = NULL;
	memset(&mParams, 0, sizeof(mParams));
	mInputMedia = NULL;
	mInputBufferQueue = NULL;
	mIsFlushed = true;
	mInputChannelFlushPending = false;
	mTearingDown = false;
	switch (requiredFormat) {
	case 0:
		break;
	case PDRAW_H264_FORMAT_BYTE_STREAM:
		setVideoMediaSubFormatCaps(
			VideoMedia::H264BitstreamFormat::BYTE_STREAM);
		break;
	case PDRAW_H264_FORMAT_AVCC:
		setVideoMediaSubFormatCaps(
			VideoMedia::H264BitstreamFormat::AVCC);
		break;
	default:
		ULOGW("unsupported required format: %d", requiredFormat);
	}

	setState(CREATED);
}


VideoSink::~VideoSink(void)
{
	int ret;

	if (mState == STARTED)
		ULOGW("video sink is still running");

	/* Flush and destroy the queue */
	if (mInputBufferQueue != NULL) {
		ret = vbuf_queue_flush(mInputBufferQueue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_flush", -ret);
		ret = vbuf_queue_destroy(mInputBufferQueue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_destroy", -ret);
		mInputBufferQueue = NULL;
	}
}


int VideoSink::setup(IPdraw::VideoSinkListener *listener,
		     const struct pdraw_video_sink_params *params)
{
	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	Element::lock();
	mVideoSinkListener = listener;
	mParams = *params;
	Element::unlock();

	return 0;
}


int VideoSink::start(void)
{
	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		ULOGE("video sink is not started");
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

	/* Create the queue */
	int res = vbuf_queue_new(mParams.queue_max_count,
				 mParams.queue_drop_when_full,
				 &mInputBufferQueue);
	if (res < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_queue_new", -res);
		return res;
	}

	/* Setup the input port */
	port->channel->setKey(this);
	port->channel->setQueue(mInputBufferQueue);

	Sink::unlock();

	setState(STARTED);

	return 0;
}


int VideoSink::stop(void)
{
	int ret;
	Channel *channel = NULL;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		ULOGE("video sink is not started");
		return -EPROTO;
	}
	setState(STOPPING);

	Sink::lock();

	if (mInputMedia == NULL) {
		Sink::unlock();
		setState(STOPPED);
		return 0;
	}

	channel = getInputChannel(mInputMedia);
	if (channel == NULL) {
		Sink::unlock();
		ULOGE("failed to get channel");
		return -EPROTO;
	}

	Sink::unlock();

	ret = channelTeardown(channel);
	if (ret < 0)
		ULOG_ERRNO("channelTeardown", -ret);

	return 0;
}


int VideoSink::resync(void)
{
	int ret;
	Channel *channel = NULL;

	if (mState != STARTED) {
		ULOGE("video sink is not started");
		return -EPROTO;
	}

	Sink::lock();

	ret = flush();
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("flush", -ret);
		return ret;
	}

	channel = getInputChannel(mInputMedia);
	if (channel == NULL) {
		Sink::unlock();
		ULOGE("failed to get channel");
		return -EPROTO;
	}

	ret = channel->resync();
	if (ret < 0)
		ULOG_ERRNO("channel->resync", -ret);

	Sink::unlock();

	return ret;
}


int VideoSink::flush(void)
{
	int ret;

	if (mIsFlushed) {
		ULOGD("video sink is already flushed, nothing to do");
		ret = flushDone();
		if (ret < 0)
			ULOG_ERRNO("flushDone", -ret);
		return ret;
	}

	/* Signal the application for flushing */
	ret = mSession->flushVideoSink(this);
	if (ret < 0)
		ULOG_ERRNO("session->flushVideoSink", -ret);

	return ret;
}


int VideoSink::flushDone(void)
{
	int ret;

	Sink::lock();

	if (mInputMedia == NULL)
		goto exit;

	if (mInputChannelFlushPending) {
		Channel *channel = getInputChannel(mInputMedia);
		if (channel == NULL) {
			ULOGE("failed to get channel");
		} else {
			mIsFlushed = true;
			mInputChannelFlushPending = false;
			ret = channel->flushDone();
			if (ret < 0)
				ULOG_ERRNO("channel->flushDone", -ret);
		}
	}

exit:
	Sink::unlock();

	if (mState == STOPPING)
		setState(STOPPED);

	return 0;
}


void VideoSink::onChannelQueue(Channel *channel, vbuf_buffer *buf)
{
	int ret;
	VideoMedia::Frame *in_meta;
	struct pdraw_video_frame *out_meta;
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
		ULOGE("video sink is not started");
		return;
	}
	if (mInputChannelFlushPending) {
		ULOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	if (mInputMedia == NULL) {
		Sink::unlock();
		ULOGE("invalid input media");
		return;
	}
	if ((mInputMedia->format != VideoMedia::Format::YUV) &&
	    (mInputMedia->format != VideoMedia::Format::H264)) {
		Sink::unlock();
		ULOGE("invalid input media format");
		return;
	}
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

	/* Set the output metadata from the PDrAW metadata */
	ret = vbuf_metadata_get(
		buf, mInputMedia, &level, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_get", -ret);
		return;
	}
	if ((in_meta->format != VideoMedia::Format::YUV) &&
	    (in_meta->format != VideoMedia::Format::H264)) {
		Sink::unlock();
		ULOGE("unsupported input format");
		return;
	}
	if ((in_meta->yuvFrame.format & getVideoMediaSubFormatCaps()) == 0) {
		Sink::unlock();
		ULOGE("unsupported input sub-format");
		return;
	}
	ret = vbuf_metadata_add(
		buf, this, level + 1, sizeof(*out_meta), (uint8_t **)&out_meta);
	if (ret < 0) {
		Sink::unlock();
		ULOG_ERRNO("vbuf_metadata_add", -ret);
		return;
	}
	if (in_meta->format == VideoMedia::Format::YUV) {
		switch (in_meta->yuvFrame.format) {
		case VideoMedia::YuvFormat::I420:
			out_meta->yuv.format = PDRAW_YUV_FORMAT_I420;
			break;
		case VideoMedia::YuvFormat::NV12:
			out_meta->yuv.format = PDRAW_YUV_FORMAT_NV12;
			break;
		default:
			out_meta->yuv.format = PDRAW_YUV_FORMAT_UNKNOWN;
			break;
		}
		const uint8_t *base = vbuf_get_cdata(buf);
		out_meta->yuv.plane[0] =
			base + in_meta->yuvFrame.planeOffset[0];
		out_meta->yuv.plane[1] =
			base + in_meta->yuvFrame.planeOffset[1];
		out_meta->yuv.plane[2] =
			base + in_meta->yuvFrame.planeOffset[2];
		out_meta->yuv.stride[0] = in_meta->yuvFrame.planeStride[0];
		out_meta->yuv.stride[1] = in_meta->yuvFrame.planeStride[1];
		out_meta->yuv.stride[2] = in_meta->yuvFrame.planeStride[2];
		out_meta->yuv.width = in_meta->yuvFrame.width;
		out_meta->yuv.height = in_meta->yuvFrame.height;
		out_meta->yuv.sar_width = in_meta->yuvFrame.sarWidth;
		out_meta->yuv.sar_height = in_meta->yuvFrame.sarHeight;
		out_meta->yuv.crop_left = in_meta->yuvFrame.cropLeft;
		out_meta->yuv.crop_top = in_meta->yuvFrame.cropTop;
		out_meta->yuv.crop_width = in_meta->yuvFrame.cropWidth;
		out_meta->yuv.crop_height = in_meta->yuvFrame.cropHeight;
		out_meta->yuv.full_range = in_meta->yuvFrame.fullRange;
	} else if (in_meta->format == VideoMedia::Format::H264) {
		switch (in_meta->h264Frame.format) {
		case VideoMedia::H264BitstreamFormat::BYTE_STREAM:
			out_meta->h264.format = PDRAW_H264_FORMAT_BYTE_STREAM;
			break;
		case VideoMedia::H264BitstreamFormat::AVCC:
			out_meta->h264.format = PDRAW_H264_FORMAT_AVCC;
			break;
		default:
			out_meta->h264.format = PDRAW_H264_FORMAT_UNKNOWN;
			break;
		}
		out_meta->h264.is_complete =
			(in_meta->h264Frame.isComplete) ? 1 : 0;
		out_meta->h264.is_sync = (in_meta->h264Frame.isSync) ? 1 : 0;
		out_meta->h264.is_ref = (in_meta->h264Frame.isRef) ? 1 : 0;
	}
	out_meta->format = (enum pdraw_video_media_format)in_meta->format;
	out_meta->has_errors = (in_meta->hasErrors) ? 1 : 0;
	out_meta->is_silent = (in_meta->isSilent) ? 1 : 0;
	out_meta->ntp_timestamp = in_meta->ntpTimestamp;
	out_meta->ntp_unskewed_timestamp = in_meta->ntpUnskewedTimestamp;
	out_meta->ntp_raw_timestamp = in_meta->ntpRawTimestamp;
	out_meta->ntp_raw_unskewed_timestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta->play_timestamp = in_meta->playTimestamp;
	out_meta->capture_timestamp = in_meta->captureTimestamp;
	out_meta->local_timestamp = in_meta->localTimestamp;
	out_meta->has_metadata = (in_meta->hasMetadata) ? 1 : 0;
	out_meta->metadata = in_meta->metadata;

	Sink::onChannelQueue(channel, buf);
	mIsFlushed = false;
	Sink::unlock();
}


void VideoSink::onChannelFlush(Channel *channel)
{
	int ret;

	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	ULOGD("flushing input channel");
	mInputChannelFlushPending = true;

	ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);
}


void VideoSink::onChannelTeardown(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	ULOGD("tearing down input channel");

	int ret = channelTeardown(channel);
	if (ret < 0)
		ULOG_ERRNO("channelTeardown", -ret);
}


int VideoSink::channelTeardown(Channel *channel)
{
	int ret;

	if (channel == NULL)
		return -EINVAL;

	Sink::lock();

	if (mInputMedia == NULL) {
		/* The channel is already torn down, nothing more to do */
		Sink::unlock();
		return 0;
	}

	if (mTearingDown) {
		/* The teardown may already be in progress but mInputMedia
		 * is not yet set to NULL.
		 * Eg. removeInputMedia() utimately calls the app's
		 * mediaRemoved() callback, which can call the VideoSink
		 * stop() function, which calls channelTeardown() again. */
		Sink::unlock();
		return 0;
	}
	mTearingDown = true;

	/* Remove the input port */
	channel->setQueue(NULL);

	ret = removeInputMedia(mInputMedia);
	if (ret < 0)
		ULOG_ERRNO("removeInputMedia", -ret);
	else
		mInputMedia = NULL;

	mTearingDown = false;
	Sink::unlock();

	ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);

	return ret;
}

} /* namespace Pdraw */
