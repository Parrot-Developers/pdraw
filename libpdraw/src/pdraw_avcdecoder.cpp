/**
 * Parrot Drones Awesome Video Viewer Library
 * H.264/AVC decoder interface
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_avcdecoder.hpp"
#include <unistd.h>
#include <time.h>
#define ULOG_TAG pdraw_decavc
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_decavc);
#include <video-buffers/vbuf_generic.h>
#include <vector>

namespace Pdraw {


AvcDecoder::AvcDecoder(
	VideoMedia *media)
{
	int ret;
	struct vdec_config cfg;
	struct vdec_cbs cbs;
	uint32_t supported_input_format;

	mConfigured = false;
	mMedia = (Media*)media;
	mInputBufferPool = NULL;
	mInputBufferPoolAllocated = false;
	mInputBufferQueue = NULL;
	mVdec = NULL;
	mFrameIndex = 0;

	supported_input_format = vdec_get_supported_input_format(
		VDEC_DECODER_IMPLEM_AUTO);
	if (supported_input_format & VDEC_INPUT_FORMAT_BYTE_STREAM) {
		mInputFormat = VDEC_INPUT_FORMAT_BYTE_STREAM;
	} else if (supported_input_format & VDEC_INPUT_FORMAT_AVCC) {
		mInputFormat = VDEC_INPUT_FORMAT_AVCC;
	} else {
		ULOGE("unsuppoted input format");
		goto error;
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.implem = VDEC_DECODER_IMPLEM_AUTO;
	cfg.encoding = VDEC_ENCODING_H264;
	cfg.low_delay = 0; /* TODO */
#ifdef BCM_VIDEOCORE
	cfg.preferred_output_format = VDEC_OUTPUT_FORMAT_MMAL_OPAQUE;
#endif /* BCM_VIDEOCORE */
	memset(&cbs, 0, sizeof(cbs));
	cbs.frame_output = &frameOutputCb;
	cbs.flush = &flushCb;
	cbs.stop = &stopCb;
	ret = vdec_new(&cfg, &cbs, this, &mVdec);
	if (ret < 0) {
		ULOG_ERRNO("vdec_new", -ret);
		goto error;
	}

	return;

error:
	if (mVdec) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			ULOG_ERRNO("vdec_destroy", -ret);
		mVdec = NULL;
	}
}


AvcDecoder::~AvcDecoder(
	void)
{
	int ret;

	if (mVdec) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			ULOG_ERRNO("vdec_destroy", -ret);
		mVdec = NULL;
		mInputBufferQueue = NULL;
	}

	if ((mInputBufferPool != NULL) && (mInputBufferPoolAllocated)) {
		ret = vbuf_pool_destroy(mInputBufferPool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_destroy:input", -ret);
		mInputBufferPool = NULL;
	}
	mInputBufferPool = NULL;

	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();
	while (q != mOutputBufferQueues.end()) {
		vbuf_queue_destroy(*q);
		q++;
	}
}


uint32_t AvcDecoder::getInputBitstreamFormatCaps(
	void)
{
	switch (mInputFormat) {
	case VDEC_INPUT_FORMAT_BYTE_STREAM:
		return AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM;
	case VDEC_INPUT_FORMAT_AVCC:
		return AVCDECODER_BITSTREAM_FORMAT_AVCC;
	default:
		return AVCDECODER_BITSTREAM_FORMAT_UNKNOWN;
	}
}


int AvcDecoder::open(
	uint32_t inputBitstreamFormat,
	const uint8_t *pSps,
	unsigned int spsSize,
	const uint8_t *pPps,
	unsigned int ppsSize)
{
	int ret;
	struct vbuf_cbs cbs;
	unsigned int width = 0, height = 0;

	if (mConfigured) {
		ULOGE("decoder is already configured");
		return -EPROTO;
	}
	if ((inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) &&
		(inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_AVCC)) {
		ULOGE("unsupported input bitstream format");
		return -EINVAL;
	}

	ret = vdec_set_sps_pps(mVdec, pSps, spsSize, pPps, ppsSize,
		mInputFormat);
	if (ret < 0) {
		ULOG_ERRNO("vdec_set_sps_pps", -ret);
		return ret;
	}

	ret = vdec_get_video_dimensions(mVdec, &width, &height,
		NULL, NULL, NULL, NULL, NULL, NULL);
	if (ret < 0) {
		ULOG_ERRNO("vdec_get_video_dimensions", -ret);
		return ret;
	}

	mInputBufferPool = vdec_get_input_buffer_pool(mVdec);
	mInputBufferQueue = vdec_get_input_buffer_queue(mVdec);

	if (mInputBufferPool == NULL) {
		ret = vbuf_generic_get_cbs(&cbs);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_generic_get_cbs", -ret);
			return ret;
		}

		/* Input buffers pool allocation */
		mInputBufferPool = vbuf_pool_new(
			AVCDECODER_INPUT_BUFFER_COUNT,
			width * height * 3 / 4, 0,
			&cbs); /* TODO: number of buffers and buffers size */
		if (mInputBufferPool == NULL) {
			ULOG_ERRNO("vbuf_pool_new:input", ENOMEM);
			return -ENOMEM;
		}
		mInputBufferPoolAllocated = true;
	}

	mConfigured = true;
	ULOGI("decoder is configured");

	return 0;
}


int AvcDecoder::getInputBuffer(
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (buffer == NULL)
		return -EINVAL;
	if (!mConfigured) {
		ULOGE("decoder is not configured");
		return -EPROTO;
	}
	if (mInputBufferPool == NULL) {
		ULOGE("invalid input buffer pool");
		return -EPROTO;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
	if (ret < 0) {
		if ((ret != -EAGAIN) && (ret != -ETIMEDOUT))
			ULOG_ERRNO("vbuf_pool_get:input", -ret);
		return ret;
	}
	*buffer = buf;

	return 0;
}


int AvcDecoder::queueInputBuffer(
	struct vbuf_buffer *buffer)
{
	int ret;
	struct avcdecoder_input_buffer *in_meta;
	struct vdec_input_metadata *vdec_meta;
	unsigned int level = 0;

	if (buffer == NULL)
		return -EINVAL;
	if (!mConfigured) {
		ULOGE("decoder is not configured");
		return -EPROTO;
	}
	if (mInputBufferQueue == NULL) {
		ULOGE("invalid input buffer queue");
		return -EPROTO;
	}

	in_meta = (struct avcdecoder_input_buffer *)vbuf_metadata_get(buffer,
		mMedia, &level, NULL);
	if (in_meta == NULL) {
		ULOG_ERRNO("vbuf_metadata_get", EPROTO);
		return -EPROTO;
	}
	vdec_meta = (struct vdec_input_metadata *)vbuf_metadata_add(buffer,
		mVdec, level + 1, sizeof(*vdec_meta));
	if (vdec_meta == NULL) {
		ULOG_ERRNO("vbuf_metadata_add", ENOMEM);
		return -ENOMEM;
	}
	vdec_meta->timestamp = in_meta->auNtpTimestampRaw;
	vdec_meta->index = mFrameIndex++;
	vdec_meta->format = mInputFormat;
	vdec_meta->complete = in_meta->isComplete;
	vdec_meta->errors = in_meta->hasErrors;
	vdec_meta->ref = in_meta->isRef;
	vdec_meta->silent = in_meta->isSilent;
	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	vdec_meta->input_time =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	ret = vbuf_queue_push(mInputBufferQueue, buffer);
	if (ret < 0)
		ULOG_ERRNO("vbuf_queue_push:input", -ret);

	return 0;
}


struct vbuf_queue *AvcDecoder::addOutputQueue(
	void)
{
	struct vbuf_queue *q = vbuf_queue_new(0, 0);
	if (q == NULL) {
		ULOG_ERRNO("vbuf_queue_new:output", ENOMEM);
		return NULL;
	}

	mOutputBufferQueues.push_back(q);
	return q;
}


int AvcDecoder::removeOutputQueue(
	struct vbuf_queue *queue)
{
	if (!queue)
		return -EINVAL;

	bool found = false;
	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			mOutputBufferQueues.erase(q);
			int ret = vbuf_queue_destroy(*q);
			if (ret < 0)
				ULOG_ERRNO("vbuf_queue_destroy:output", -ret);
			found = true;
			break;
		}
		q++;
	}

	return (found) ? 0 : -ENOENT;
}


bool AvcDecoder::isOutputQueueValid(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOG_ERRNO("queue", EINVAL);
		return false;
	}

	bool found = false;
	std::vector<struct vbuf_queue*>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			found = true;
			break;
		}
		q++;
	}

	return found;
}


int AvcDecoder::flush(
	void)
{
	int ret;

	if (!mConfigured) {
		ULOGE("decoder is not configured");
		return -EPROTO;
	}

	/* Flush the output queues */
	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();
	while (q != mOutputBufferQueues.end()) {
		ret = vbuf_queue_flush(*q);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_flush:output", -ret);
		q++;
	}

	/* TODO: flush the downstream elements */

	/* Flush the decoder */
	ret = vdec_flush(mVdec, 1);
	if (ret < 0) {
		ULOG_ERRNO("vdec_flush", -ret);
		return ret;
	}

	return 0;
}


int AvcDecoder::close(
	void)
{
	int ret;

	if (!mConfigured) {
		ULOGE("decoder is not configured");
		return -EPROTO;
	}

	mConfigured = false;

	if (mInputBufferPool) {
		ret = vbuf_pool_abort(mInputBufferPool);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_pool_abort:input", -ret);
			return ret;
		}
	}

	ret = vdec_stop(mVdec);
	if (ret < 0) {
		ULOG_ERRNO("vdec_stop", -ret);
		return ret;
	}

	return 0;
}


void AvcDecoder::frameOutputCb(
	struct vbuf_buffer *out_buf,
	void *userdata)
{
	int ret;
	AvcDecoder *decoder = (AvcDecoder *)userdata;
	struct vdec_output_metadata *vdec_meta;
	struct avcdecoder_input_buffer *in_meta;
	struct avcdecoder_output_buffer _out_meta;
	struct avcdecoder_output_buffer *out_meta;
	unsigned int level = 0;

	if (userdata == NULL) {
		ULOG_ERRNO("userdata", EINVAL);
		return;
	}
	if (out_buf == NULL) {
		ULOG_ERRNO("out_buf", EINVAL);
		return;
	}

	vdec_meta = (struct vdec_output_metadata *)
		vbuf_metadata_get(out_buf, decoder->mVdec, NULL, NULL);
	in_meta = (struct avcdecoder_input_buffer *)
		vbuf_metadata_get(out_buf, decoder->mMedia, &level, NULL);
	memset(&_out_meta, 0, sizeof(_out_meta));
	_out_meta.plane_offset[0] = vdec_meta->plane_offset[0];
	_out_meta.plane_offset[1] = vdec_meta->plane_offset[1];
	_out_meta.plane_offset[2] = vdec_meta->plane_offset[2];
	_out_meta.stride[0] = vdec_meta->stride[0];
	_out_meta.stride[1] = vdec_meta->stride[1];
	_out_meta.stride[2] = vdec_meta->stride[2];
	_out_meta.width = vdec_meta->width;
	_out_meta.height = vdec_meta->height;
	_out_meta.sarWidth = vdec_meta->sar_width;
	_out_meta.sarHeight = vdec_meta->sar_height;
	_out_meta.cropLeft = vdec_meta->crop_left;
	_out_meta.cropTop = vdec_meta->crop_top;
	_out_meta.cropWidth = vdec_meta->crop_width;
	_out_meta.cropHeight = vdec_meta->crop_height;
	switch (vdec_meta->format) {
	case VDEC_OUTPUT_FORMAT_I420:
		_out_meta.colorFormat =
			AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
		break;
	case VDEC_OUTPUT_FORMAT_NV12:
		_out_meta.colorFormat =
			AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR;
		break;
	case VDEC_OUTPUT_FORMAT_MMAL_OPAQUE:
		_out_meta.colorFormat =
			AVCDECODER_COLOR_FORMAT_MMAL_OPAQUE;
		break;
	default:
		_out_meta.colorFormat =
			AVCDECODER_COLOR_FORMAT_UNKNOWN;
		break;
	}
	_out_meta.isComplete = in_meta->isComplete;
	_out_meta.hasErrors = vdec_meta->errors;
	_out_meta.isRef = in_meta->isRef;
	_out_meta.isSilent = vdec_meta->silent;
	_out_meta.auNtpTimestamp =
		in_meta->auNtpTimestamp;
	_out_meta.auNtpTimestampRaw =
		in_meta->auNtpTimestampRaw;
	_out_meta.auNtpTimestampLocal =
		in_meta->auNtpTimestampLocal;
	_out_meta.demuxOutputTimestamp =
		in_meta->demuxOutputTimestamp;
	_out_meta.decoderOutputTimestamp = vdec_meta->output_time;

	/* Frame metadata */
	if (in_meta->hasMetadata) {
		memcpy(&_out_meta.metadata,
			&in_meta->metadata,
			sizeof(struct vmeta_frame_v2));
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

	/* Remove the pdraw input metadata */
	ret = vbuf_metadata_remove(out_buf, decoder->mMedia);
	if (ret < 0)
		ULOG_ERRNO("vbuf_metadata_remove", -ret);

	/* Add the pdraw output metadata */
	out_meta = (struct avcdecoder_output_buffer *)
		vbuf_metadata_add(out_buf, decoder->mMedia,
		level, sizeof(*out_meta));
	if (out_meta == NULL) {
		ULOG_ERRNO("vbuf_metadata_add", ENOMEM);
		return;
	}
	memcpy(out_meta, &_out_meta, sizeof(*out_meta));

	/* Push the frame */
	if (!_out_meta.isSilent) {
		std::vector<struct vbuf_queue *>::iterator q =
			decoder->mOutputBufferQueues.begin();
		while (q != decoder->mOutputBufferQueues.end()) {
			ret = vbuf_queue_push(*q, out_buf);
			if (ret < 0)
				ULOG_ERRNO("vbuf_queue_push:output", -ret);
			q++;
		}
	} else {
		ULOGD("silent frame (ignored)");
	}
}


void AvcDecoder::flushCb(
	void *userdata)
{
	ULOGI("decoder is flushed");

	/* TODO: signal the upstream elements */
}


void AvcDecoder::stopCb(
	void *userdata)
{
	ULOGI("decoder is stopped");

	/* TODO: signal the upstream elements */
}

} /* namespace Pdraw */
