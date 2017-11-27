/**
 * Parrot Drones Awesome Video Viewer Library
 * FFmpeg H.264/AVC video decoder
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

#include "pdraw_avcdecoder_ffmpeg.hpp"

#ifdef USE_FFMPEG

#include <unistd.h>
#include <time.h>
#include <video-buffers/vbuf_avframe.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <vector>

namespace Pdraw {


FfmpegAvcDecoder::FfmpegAvcDecoder(
	VideoMedia *media)
{
	AVCodec *codec = NULL;
	int ret;

	mConfigured = false;
	mOutputColorFormat = AVCDECODER_COLOR_FORMAT_UNKNOWN;
	mMedia = (Media*)media;
	mInputBufferPool = NULL;
	mInputBufferQueue = NULL;
	mOutputBufferPool = NULL;
	mThreadShouldStop = false;
	mDecoderThreadLaunched = false;
	mCodecCtx = NULL;
	mFrameWidth = 0;
	mFrameHeight = 0;

	avcodec_register_all();
	av_log_set_level(AV_LOG_VERBOSE);
	if (codec == NULL) {
		codec = avcodec_find_decoder_by_name("h264_cuvid");
		if (codec != NULL) {
			ULOGI("FFmpeg: using NVDec (cuvid) "
				"hardware acceleration");
			mOutputColorFormat =
				AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR;
		}
	}
	if (codec == NULL) {
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);
		if (codec != NULL) {
			ULOGI("FFmpeg: using CPU H.264 decoding");
			mOutputColorFormat =
				AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
		}
	}
	if (codec == NULL) {
		ULOGE("FFmpeg: codec not found");
		goto err;
	}

	mCodecCtx = avcodec_alloc_context3(codec);
	if (mCodecCtx == NULL) {
		ULOGE("FFmpeg: failed to allocate codec context");
		goto err;
	}

	mCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
	mCodecCtx->skip_frame = AVDISCARD_DEFAULT;
	mCodecCtx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
	mCodecCtx->skip_loop_filter = AVDISCARD_DEFAULT;
	mCodecCtx->workaround_bugs = FF_BUG_AUTODETECT;
	mCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
	mCodecCtx->codec_id = AV_CODEC_ID_H264;
	mCodecCtx->skip_idct = AVDISCARD_DEFAULT;

	ret = avcodec_open2(mCodecCtx, codec, NULL);
	if (ret < 0) {
		ULOGE("FFmpeg: failed to open codec (%d)", ret);
		goto err;
	}

	av_init_packet(&mPacket);

	ret = pthread_create(&mDecoderThread, NULL,
		runDecoderThread, (void *)this);
	if (ret != 0) {
		ULOGE("FFmpeg: decoder thread creation failed (%d)", ret);
		goto err;
	} else {
		mDecoderThreadLaunched = true;
	}

	return;

err:
	if (mCodecCtx != NULL) {
		avcodec_free_context(&mCodecCtx);
		mCodecCtx = NULL;
	}
}


FfmpegAvcDecoder::~FfmpegAvcDecoder(
	void)
{
	mThreadShouldStop = true;
	if (mDecoderThreadLaunched) {
		int thErr = pthread_join(mDecoderThread, NULL);
		if (thErr != 0)
			ULOGE("FFmpeg: pthread_join() failed (%d)", thErr);
	}

	if (mInputBufferQueue != NULL)
		vbuf_queue_destroy(mInputBufferQueue);
	if (mInputBufferPool != NULL)
		vbuf_pool_destroy(mInputBufferPool);
	if (mOutputBufferPool != NULL)
		vbuf_pool_destroy(mOutputBufferPool);

	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();
	while (q != mOutputBufferQueues.end()) {
		vbuf_queue_destroy(*q);
		q++;
	}

	avcodec_free_context(&mCodecCtx);
}


int FfmpegAvcDecoder::configure(
	uint32_t inputBitstreamFormat,
	const uint8_t *pSps,
	unsigned int spsSize,
	const uint8_t *pPps,
	unsigned int ppsSize)
{
	int ret = 0;
	struct vbuf_cbs cbs;

	if (mConfigured) {
		ULOGE("FFmpeg: decoder is already configured");
		return -1;
	}
	if (inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) {
		ULOGE("FFmpeg: unsupported input bitstream format");
		return -1;
	}

	/* Nothing to decode here for FFmpeg:
	 * SPS/PPS will be decoded with the first picture */

	ret = vbuf_generic_get_cbs(&cbs);
	if (ret != 0) {
		ULOGE("FFmpeg: failed to get "
			"input buffers allocation callbacks");
		return -1;
	}

	/* Input buffers pool allocation */
	mInputBufferPool = vbuf_pool_new(
		AVCDECODER_FFMPEG_INPUT_BUFFER_COUNT,
		AVCDECODER_FFMPEG_INPUT_BUFFER_SIZE,
		sizeof(struct avcdecoder_input_buffer), 0,
		&cbs); /* TODO: number of buffers and buffers size */
	if (mInputBufferPool == NULL) {
		ULOGE("FFmpeg: failed to allocate decoder "
			"input buffers pool");
		return -1;
	}

	/* Input buffers queue allocation */
	mInputBufferQueue = vbuf_queue_new();
	if (mInputBufferQueue == NULL) {
		ULOGE("FFmpeg: failed to allocate decoder "
			"input buffers queue");
		return -1;
	}

	ret = vbuf_avframe_get_cbs(&cbs);
	if (ret != 0) {
		ULOGE("FFmpeg: failed to get "
			"output buffers allocation callbacks");
		return -1;
	}

	/* Output buffers pool allocation */
	mOutputBufferPool = vbuf_pool_new(
		AVCDECODER_FFMPEG_OUTPUT_BUFFER_COUNT, 0,
		sizeof(struct avcdecoder_output_buffer), 0,
		&cbs); /* TODO: number of buffers */
	if (mOutputBufferPool == NULL) {
		ULOGE("FFmpeg: failed to allocate decoder "
			"output buffers pool");
		return -1;
	}

	mConfigured = true;
	ULOGI("FFmpeg: decoder is configured");

	return 0;
}


int FfmpegAvcDecoder::getInputBuffer(
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (buffer == NULL) {
		ULOGE("FFmpeg: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}
	if (mInputBufferPool == NULL) {
		ULOGE("FFmpeg: input buffer pool has not been created");
		return -1;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
	if ((ret == 0) && (buf != NULL)) {
		vbuf_set_size(buf, 0);
		*buffer = buf;
	} else {
		ULOGD("FFmpeg: failed to get an input buffer (%d)", ret);
		return -2;
	}

	return 0;
}


int FfmpegAvcDecoder::queueInputBuffer(
	struct vbuf_buffer *buffer)
{
	if (buffer == NULL) {
		ULOGE("FFmpeg: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}
	if (mInputBufferQueue == NULL) {
		ULOGE("FFmpeg: input queue has not been created");
		return -1;
	}

	int ret = vbuf_queue_push(mInputBufferQueue, buffer);
	if (ret != 0)
		ULOGE("FFmpeg: failed to push the buffer into the input queue");

	return 0;
}


struct vbuf_queue *FfmpegAvcDecoder::addOutputQueue(
	void)
{
	struct vbuf_queue *q = vbuf_queue_new();
	if (q == NULL) {
		ULOGE("FFmpeg: queue allocation failed");
		return NULL;
	}

	mOutputBufferQueues.push_back(q);
	return q;
}


int FfmpegAvcDecoder::removeOutputQueue(
	struct vbuf_queue *queue)
{
	if (!queue) {
		ULOGE("FFmpeg: invalid queue pointer");
		return -1;
	}

	bool found = false;
	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			mOutputBufferQueues.erase(q);
			int ret = vbuf_queue_destroy(*q);
			if (ret != 0)
				ULOGE("FFmpeg: vbuf_queue_destroy() "
					"failed (%d)", ret);
			found = true;
			break;
		}
		q++;
	}

	return (found) ? 0 : -1;
}


bool FfmpegAvcDecoder::isOutputQueueValid(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("FFmpeg: invalid queue pointer");
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


int FfmpegAvcDecoder::dequeueOutputBuffer(
	struct vbuf_queue *queue,
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (queue == NULL) {
		ULOGE("FFmpeg: invalid queue pointer");
		return -1;
	}
	if (buffer == NULL) {
		ULOGE("FFmpeg: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}
	if (!isOutputQueueValid(queue)) {
		ULOGE("FFmpeg: invalid output queue");
		return -1;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_queue_pop(queue, (blocking) ? -1 : 0, &buf);
	if ((ret != 0) || (buf == NULL)) {
		if (ret != -EAGAIN) {
			ULOGW("FFmpeg: failed to dequeue "
				"an output buffer (%d)", ret);
			return -1;
		} else {
			return -2;
		}
	}

	struct avcdecoder_output_buffer *data =
		(struct avcdecoder_output_buffer *)vbuf_get_metadata_ptr(buf);
	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	data->decoderOutputTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
	*buffer = buf;

	return 0;
}


int FfmpegAvcDecoder::releaseOutputBuffer(
	struct vbuf_buffer **buffer)
{
	if (buffer == NULL) {
		ULOGE("FFmpeg: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}

	vbuf_unref(buffer);

	return 0;
}


int FfmpegAvcDecoder::stop(
	void)
{
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}

	mThreadShouldStop = true;
	mConfigured = false;

	if (mInputBufferPool)
		vbuf_pool_abort(mInputBufferPool);
	if (mOutputBufferPool)
		vbuf_pool_abort(mOutputBufferPool);
	if (mInputBufferQueue)
		vbuf_queue_abort(mInputBufferQueue);

	return 0;
}


void* FfmpegAvcDecoder::runDecoderThread(
	void *ptr)
{
	FfmpegAvcDecoder *decoder = (FfmpegAvcDecoder*)ptr;

	while (!decoder->mThreadShouldStop) {
		if (!decoder->mConfigured) {
			usleep(5000); /* TODO: do something better (cond?) */
			continue;
		}

		struct vbuf_buffer *inputBuffer = NULL;
		struct vbuf_buffer *outputBuffer = NULL;
		int ret;

		ret = vbuf_queue_pop(
			decoder->mInputBufferQueue, -1, &inputBuffer);
		if ((ret != 0) || (inputBuffer == NULL)) {
			if (!decoder->mThreadShouldStop) {
				ULOGW("FFmpeg: failed to dequeue "
					"an input buffer (%d)", ret);
				usleep(5000);
			}
			continue;
		}

		ret = vbuf_pool_get(
			decoder->mOutputBufferPool, 0, &outputBuffer);
		if ((ret != 0) || (outputBuffer == NULL)) {
			ULOGW("FFmpeg: failed to get "
				"an output buffer (%d)", ret);
			vbuf_unref(&inputBuffer);
			continue;
		}

		ret = decoder->decode(inputBuffer, outputBuffer);

		/* Push the frame */
		if (ret == 1) {
			std::vector<struct vbuf_queue *>::iterator q =
				decoder->mOutputBufferQueues.begin();
			while (q != decoder->mOutputBufferQueues.end()) {
				vbuf_queue_push(*q, outputBuffer);
				q++;
			}
		} else if (ret == 0) {
			ULOGI("FFmpeg: silent frame (ignored)");
		}

		vbuf_unref(&outputBuffer);
		vbuf_unref(&inputBuffer);
	}

	return NULL;
}


int FfmpegAvcDecoder::decode(
	struct vbuf_buffer *inputBuffer,
	struct vbuf_buffer *outputBuffer)
{
	if (!mConfigured) {
		ULOGE("FFmpeg: decoder is not configured");
		return -1;
	}

	int frameFinished = false;
	struct avcdecoder_input_buffer *inputMeta =
		(struct avcdecoder_input_buffer *)
		vbuf_get_metadata_ptr(inputBuffer);
	struct avcdecoder_output_buffer *outputMeta =
		(struct avcdecoder_output_buffer *)
		vbuf_get_metadata_ptr(outputBuffer);
	AVFrame *frame = vbuf_avframe_get_frame(outputBuffer);
	if ((inputMeta == NULL) || (outputMeta == NULL) || (frame == NULL)) {
		ULOGE("FFmpeg: invalid input or output buffer");
		return -1;
	}

	mPacket.data = vbuf_get_ptr(inputBuffer);
	mPacket.size = vbuf_get_size(inputBuffer);

	avcodec_decode_video2(mCodecCtx, frame, &frameFinished, &mPacket);

	if (!frameFinished) {
		ULOGI("FFmpeg: failed to decode frame");
		return -1;
	}

	if ((mFrameWidth != (uint32_t)mCodecCtx->width) ||
		(mFrameHeight != (uint32_t)mCodecCtx->height)) {
		mFrameWidth = mCodecCtx->width;
		mFrameHeight = mCodecCtx->height;
		mSarWidth = (mCodecCtx->sample_aspect_ratio.num > 0) ?
			mCodecCtx->sample_aspect_ratio.num : 1;
		mSarHeight = (mCodecCtx->sample_aspect_ratio.den > 0) ?
			mCodecCtx->sample_aspect_ratio.den : 1;
	}

	memset(outputMeta, 0, sizeof(*outputMeta));

	/* Pixel data */
	outputMeta->colorFormat = mOutputColorFormat;
	if (mOutputColorFormat == AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR) {
		outputMeta->plane[0] = frame->data[0];
		outputMeta->plane[1] = frame->data[1];
		outputMeta->stride[0] = frame->linesize[0];
		outputMeta->stride[1] = frame->linesize[1];
	} else if (mOutputColorFormat == AVCDECODER_COLOR_FORMAT_YUV420PLANAR) {
		outputMeta->plane[0] = frame->data[0];
		outputMeta->plane[1] = frame->data[1];
		outputMeta->plane[2] = frame->data[2];
		outputMeta->stride[0] = frame->linesize[0];
		outputMeta->stride[1] = frame->linesize[1];
		outputMeta->stride[2] = frame->linesize[2];
	}
	outputMeta->width = mFrameWidth;
	outputMeta->height = mFrameHeight;
	outputMeta->sarWidth = mSarWidth;
	outputMeta->sarHeight = mSarHeight;

	outputMeta->isComplete = inputMeta->isComplete;
	outputMeta->hasErrors = inputMeta->hasErrors;
	outputMeta->isRef = inputMeta->isRef;
	outputMeta->isSilent = inputMeta->isSilent;
	outputMeta->auNtpTimestamp = inputMeta->auNtpTimestamp;
	outputMeta->auNtpTimestampRaw = inputMeta->auNtpTimestampRaw;
	outputMeta->auNtpTimestampLocal = inputMeta->auNtpTimestampLocal;
	outputMeta->demuxOutputTimestamp = inputMeta->demuxOutputTimestamp;

	/* Frame metadata */
	if (inputMeta->hasMetadata) {
		memcpy(&outputMeta->metadata, &inputMeta->metadata,
			sizeof(struct vmeta_frame_v2));
		outputMeta->hasMetadata = true;
	} else {
		outputMeta->hasMetadata = false;
	}

	/* User data */
	unsigned int userDataSize = vbuf_get_userdata_size(inputBuffer);
	uint8_t *userData = vbuf_get_userdata_ptr(inputBuffer);
	if ((userData) && (userDataSize > 0)) {
		int ret = vbuf_set_userdata_capacity(
			outputBuffer, userDataSize);
		if (ret < (signed)userDataSize) {
			ULOGE("FFmpeg: failed to realloc user data buffer");
		} else {
			uint8_t *dstBuf = vbuf_get_userdata_ptr(outputBuffer);
			memcpy(dstBuf, userData, userDataSize);
			vbuf_set_userdata_size(outputBuffer, userDataSize);
		}
	} else {
		vbuf_set_userdata_size(outputBuffer, 0);
	}

	return (outputMeta->isSilent) ? 0 : 1;
}

} /* namespace Pdraw */

#endif /* USE_FFMPEG */
