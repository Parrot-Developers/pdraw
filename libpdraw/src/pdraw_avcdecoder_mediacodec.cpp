/**
 * Parrot Drones Awesome Video Viewer Library
 * Android MediaCodec H.264/AVC video decoder
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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "pdraw_avcdecoder_mediacodec.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_session.hpp"

#ifdef USE_MEDIACODEC

#include <unistd.h>
#include <time.h>
#include <video-buffers/vbuf_mediacodec.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <vector>

namespace Pdraw {


#define AVCDECODER_MEDIACODEC_MIME_TYPE "video/avc"


MediaCodecAvcDecoder::MediaCodecAvcDecoder(
	VideoMedia *media)
{
	int ret;

	mConfigured = false;
	mMedia = (Media*)media;
	mInputBufferPool = NULL;
	mInputBufferQueue = NULL;
	mOutputBufferPool = NULL;
	mThreadShouldStop = false;
	mOutputPollThreadLaunched = false;
	mWidth = 0;
	mHeight = 0;
	mCropLeft = 0;
	mCropRight = 0;
	mCropTop = 0;
	mCropBottom = 0;
	mCroppedWidth = 0;
	mCroppedHeight = 0;
	mSarWidth = 0;
	mSarHeight = 0;
	mMcw = NULL;
	mCodec = NULL;

	void *jniEnv = NULL;
	if (mMedia) {
		Session *session = mMedia->getSession();
		if (session)
			jniEnv = session->getJniEnv();
	}

	mMcw = mcw_new((mcw_jnienv *)jniEnv, MCW_IMPLEMENTATION_AUTO);
	if (mMcw == NULL) {
		ULOGE("MediaCodec: mcw_new() failed");
		goto err;
	}

	mCodec = mMcw->mediacodec.create_decoder_by_type(
		AVCDECODER_MEDIACODEC_MIME_TYPE);
	if (mCodec == NULL) {
		ULOGE("MediaCodec: create_decoder_by_type() failed");
		goto err;
	}

	ret = pthread_create(&mOutputPollThread, NULL,
		runOutputPollThread, (void*)this);
	if (ret != 0)
		ULOGE("MediaCodec: output poll thread "
			"creation failed (%d)", ret);
	else
		mOutputPollThreadLaunched = true;

	return;

err:
	if (mCodec) {
		mMcw->mediacodec.ddelete(mCodec);
		mCodec = NULL;
	}
	if (mMcw) {
		mcw_destroy(mMcw);
		mMcw = NULL;
	}
}


MediaCodecAvcDecoder::~MediaCodecAvcDecoder(
	void)
{
	mThreadShouldStop = true;
	if (mOutputPollThreadLaunched) {
		int ret = pthread_join(mOutputPollThread, NULL);
		if (ret != 0)
			ULOGE("MediaCodec: pthread_join() failed (%d)", ret);
	}

	if (mMcw != NULL) {
		enum mcw_media_status err = mMcw->mediacodec.ddelete(mCodec);
		if (err != MCW_MEDIA_STATUS_OK)
			ULOGE("MediaCodec: mediacodec.delete() "
				"failed (%d)", err);
		mcw_destroy(mMcw);
	}

	if (mInputBufferQueue)
		vbuf_queue_destroy(mInputBufferQueue);
	if (mInputBufferPool)
		vbuf_pool_destroy(mInputBufferPool);
	if (mOutputBufferPool)
		vbuf_pool_destroy(mOutputBufferPool);

	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();
	while (q != mOutputBufferQueues.end()) {
		vbuf_queue_destroy(*q);
		q++;
	}
}


int MediaCodecAvcDecoder::open(
	uint32_t inputBitstreamFormat,
	const uint8_t *pSps,
	unsigned int spsSize,
	const uint8_t *pPps,
	unsigned int ppsSize)
{
	int ret = 0;
	enum mcw_media_status err;
	struct mcw_mediaformat *format = NULL;
	struct vbuf_cbs cbs;

	if (mConfigured) {
		ULOGE("MediaCodec: decoder is already configured");
		return -1;
	}
	if (mMcw == NULL) {
		ULOGE("MediaCodec: invalid mediacodec wrapper");
		return -1;
	}
	if ((pSps == NULL) || (spsSize <= 4) ||
		(pPps == NULL) || (ppsSize <= 4)) {
		ULOGE("MediaCodec: invalid SPS/PPS");
		return -1;
	}
	if (inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) {
		ULOGE("MediaCodec: unsupported input bitstream format");
		return -1;
	}

	format = mMcw->mediaformat.nnew();
	if (format == NULL) {
		ULOGE("MediaCodec: mediaformat.new() failed");
		ret = -1;
		goto out;
	}

	mMcw->mediaformat.set_string(format,
		mMcw->mediaformat.KEY_MIME, AVCDECODER_MEDIACODEC_MIME_TYPE);
	mMcw->mediaformat.set_int32(format,
		mMcw->mediaformat.KEY_WIDTH, 480); /* TODO */
	mMcw->mediaformat.set_int32(format,
		mMcw->mediaformat.KEY_HEIGHT, 360); /* TODO */
	mMcw->mediaformat.set_int32(format,
		mMcw->mediaformat.KEY_MAX_INPUT_SIZE,
		1920 * 1088 * 3 / 4); /* TODO */
	mMcw->mediaformat.set_buffer(format,
		"csd-0", (void *)pSps, spsSize);
	mMcw->mediaformat.set_buffer(format,
		"csd-1", (void *)pPps, ppsSize);

	/* TODO: get the renderer uiHandler */
	err = mMcw->mediacodec.configure(mCodec, format,
		NULL /* (ANativeWindow *)mUiHandler */, NULL, 0);
	if (err != MCW_MEDIA_STATUS_OK) {
		ULOGE("MediaCodec: mediacodec.configure() "
			"failed (%d)", err);
		ret = -1;
		goto out;
	}

	ret = vbuf_mediacodec_get_input_cbs(mMcw, mCodec, &cbs);
	if (ret != 0) {
		ULOGE("AMediaCodec: failed to get "
			"input buffers allocation callbacks");
		ret = -1;
		goto out;
	}

	/* Input buffers pool allocation */
	mInputBufferPool = vbuf_pool_new(
		AVCDECODER_MEDIACODEC_INPUT_BUFFER_COUNT, 0,
		sizeof(struct avcdecoder_input_buffer), 0,
		&cbs); /* TODO: number of buffers */
	if (mInputBufferPool == NULL) {
		ULOGE("MediaCodec: failed to allocate "
			"decoder input buffers pool");
		ret = -1;
		goto out;
	}

	/* Input buffers queue allocation */
	mInputBufferQueue = vbuf_queue_new();
	if (mInputBufferQueue == NULL) {
		ULOGE("MediaCodec: failed to allocate "
			"decoder input buffers queue");
		ret = -1;
		goto out;
	}

	ret = vbuf_mediacodec_get_output_cbs(mMcw, mCodec, &cbs);
	if (ret != 0) {
		ULOGE("AMediaCodec: failed to get "
			"output buffers allocation callbacks");
		ret = -1;
		goto out;
	}

	/* Output buffers pool allocation */
	mOutputBufferPool = vbuf_pool_new(
		AVCDECODER_MEDIACODEC_OUTPUT_BUFFER_COUNT, 0,
		sizeof(struct avcdecoder_output_buffer), 0,
		&cbs); /* TODO: number of buffers */
	if (mOutputBufferPool == NULL) {
		ULOGE("MediaCodec: failed to allocate "
			"decoder output buffers pool");
		ret = -1;
		goto out;
	}

	err = mMcw->mediacodec.start(mCodec);
	if (err != MCW_MEDIA_STATUS_OK) {
		ULOGE("MediaCodec: mediacodec.start() "
			"failed (%d)", err);
		ret = -1;
		goto out;
	}

	((VideoMedia *)mMedia)->getDimensions(&mWidth, &mHeight,
		&mCropLeft, &mCropRight, &mCropTop, &mCropBottom,
		&mCroppedWidth, &mCroppedHeight, &mSarWidth, &mSarHeight);

	mConfigured = true;
	ULOGI("MediaCodec: decoder is configured");

out:
	if (format != NULL)
		mMcw->mediaformat.ddelete(format);

	return ret;
}


int MediaCodecAvcDecoder::close(
	void)
{
	if (!mConfigured) {
		ULOGE("MediaCodec: decoder is not configured");
		return -1;
	}

	mThreadShouldStop = true;
	mConfigured = false;

	if (mMcw != NULL) {
		enum mcw_media_status err = mMcw->mediacodec.stop(mCodec);
		if (err != MCW_MEDIA_STATUS_OK)
			ULOGE("MediaCodec: mediacodec.stop() failed (%d)", err);
	}

	/* TODO */
	if (mInputBufferPool != NULL)
		vbuf_pool_abort(mInputBufferPool);
	if (mOutputBufferPool != NULL)
		vbuf_pool_abort(mOutputBufferPool);
	if (mInputBufferQueue != NULL)
		vbuf_queue_abort(mInputBufferQueue);

	return 0;
}


int MediaCodecAvcDecoder::getInputBuffer(
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (buffer == NULL) {
		ULOGE("MediaCodec: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("MediaCodec: decoder is not configured");
		return -1;
	}
	if (mMcw == NULL) {
		ULOGE("MediaCodec: invalid mediacodec wrapper");
		return -1;
	}
	if (mInputBufferPool == NULL) {
		ULOGE("MediaCodec: input buffer pool has not been created");
		return -1;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
	if ((ret != 0) || (buf == NULL)) {
		ULOGW("MediaCodec: failed to get an input buffer (%d)", ret);
		return -2;
	}

	*buffer = buf;

	return 0;
}


int MediaCodecAvcDecoder::queueInputBuffer(
	struct vbuf_buffer *buffer)
{
	if (buffer == NULL) {
		ULOGE("MediaCodec: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("MediaCodec: decoder is not configured");
		return -1;
	}
	if (mMcw == NULL) {
		ULOGE("MediaCodec: invalid mediacodec wrapper");
		return -1;
	}
	if (mInputBufferQueue == NULL) {
		ULOGE("MediaCodec: input queue has not been created");
		return -1;
	}

	uint64_t ts = 0;
	struct avcdecoder_input_buffer *data =
		(struct avcdecoder_input_buffer *)
		vbuf_get_metadata_ptr(buffer);
	if (data)
		ts = data->auNtpTimestampRaw;

	vbuf_mediacodec_set_info(buffer, 0, ts, 0);
	vbuf_queue_push(mInputBufferQueue, buffer);

	return 0;
}


struct vbuf_queue *MediaCodecAvcDecoder::addOutputQueue(
	void)
{
	struct vbuf_queue *q = vbuf_queue_new();
	if (q == NULL) {
		ULOGE("MediaCodec: queue allocation failed");
		return NULL;
	}

	mOutputBufferQueues.push_back(q);
	return q;
}


int MediaCodecAvcDecoder::removeOutputQueue(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("MediaCodec: invalid queue pointer");
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
				ULOGE("videoCoreOmx: vbuf_queue_destroy() "
					"failed (%d)", ret);
			found = true;
			break;
		}
		q++;
	}

	return (found) ? 0 : -1;
}


bool MediaCodecAvcDecoder::isOutputQueueValid(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("MediaCodec: invalid queue pointer");
		return false;
	}

	bool found = false;
	std::vector<struct vbuf_queue *>::iterator q =
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


int MediaCodecAvcDecoder::dequeueOutputBuffer(
	struct vbuf_queue *queue,
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (queue == NULL) {
		ULOGE("MediaCodec: invalid queue pointer");
		return -1;
	}
	if (buffer == NULL) {
		ULOGE("MediaCodec: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("MediaCodec: decoder is not configured");
		return -1;
	}
	if (!isOutputQueueValid(queue)) {
		ULOGE("MediaCodec: invalid output queue");
		return -1;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_queue_pop(queue, (blocking) ? -1 : 0, &buf);
	if ((ret != 0) || (buf == NULL)) {
		if (ret != -EAGAIN) {
			ULOGW("MediaCodec: failed to dequeue "
				"an output buffer (%d)", ret);
			return -1;
		} else {
			return -2;
		}
	}

	*buffer = buf;

	return 0;
}


int MediaCodecAvcDecoder::releaseOutputBuffer(
	struct vbuf_buffer **buffer)
{
	if ((buffer == NULL) || (*buffer == NULL)) {
		ULOGE("MediaCodec: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("MediaCodec: decoder is not configured");
		return -1;
	}
	if (mMcw == NULL) {
		ULOGE("MediaCodec: invalid mediacodec wrapper");
		return -1;
	}

	vbuf_mediacodec_set_render_time(*buffer, 0);
	vbuf_unref(buffer);

	return 0;
}


int MediaCodecAvcDecoder::pollDecoderOutput(
	void)
{
	int ret = 0, _ret;
	struct vbuf_buffer *inputBuffer = NULL, *outputBuffer = NULL;
	struct avcdecoder_input_buffer *inputMeta = NULL;
	struct avcdecoder_output_buffer *outputMeta = NULL;

	_ret = vbuf_pool_get(mOutputBufferPool, 5, &outputBuffer);
	while ((_ret == 0) && (outputBuffer)) {
		int32_t colorFormat = MCW_COLOR_FORMAT_YUV420_SEMIPLANAR;
		size_t bufIdx = 0;
		off_t offset = 0;
		uint64_t timestamp = 0;
		uint32_t flags = 0;
		uint8_t *pBuf = vbuf_get_ptr(outputBuffer);

		vbuf_mediacodec_get_info(outputBuffer, &bufIdx,
			&offset, &timestamp, &flags);

		/* TODO: do this only on INFO_OUTPUT_FORMAT_CHANGED */
		struct mcw_mediaformat *format =
			mMcw->mediacodec.get_output_format(mCodec, bufIdx);
		if (format) {
			mMcw->mediaformat.get_int32(format,
				mMcw->mediaformat.KEY_COLOR_FORMAT,
				&colorFormat);
			mMcw->mediaformat.ddelete(format);
		}

		struct vbuf_buffer *b;
		while ((_ret = vbuf_queue_peek(
			mInputBufferQueue, 0, &b)) == 0) {
			struct avcdecoder_input_buffer *d =
				(struct avcdecoder_input_buffer *)
				vbuf_get_metadata_ptr(b);

			if (timestamp == d->auNtpTimestampRaw) {
				vbuf_queue_pop(mInputBufferQueue,
					0, &inputBuffer);
				inputMeta = d;
				break;
			} else {
				_ret = vbuf_queue_pop(mInputBufferQueue,
					0, &b);
				if ((_ret == 0) && (b))
					vbuf_unref(&b);
				ULOGD("MediaCodec: discarded input buffer with "
					"TS %" PRIu64 " (expected %" PRIu64 ")",
					d->auNtpTimestampRaw, timestamp);
			}
		}

		if ((inputBuffer == NULL) || (inputMeta == NULL)) {
			ULOGW("MediaCodec: failed to find buffer "
				"for TS %" PRIu64, timestamp);
			vbuf_unref(&outputBuffer);
			ret = -1;
			break;
		}

		outputMeta = (struct avcdecoder_output_buffer *)
			vbuf_get_metadata_ptr(outputBuffer);
		if (outputMeta == NULL) {
			ULOGW("MediaCodec: invalid output buffer data");
			vbuf_unref(&outputBuffer);
			vbuf_unref(&inputBuffer);
			ret = -1;
			break;
		}

		struct timespec t1;
		clock_gettime(CLOCK_MONOTONIC, &t1);
		memset(outputMeta, 0, sizeof(*outputMeta));

		/* Pixel data */
		outputMeta->width = mCroppedWidth;
		outputMeta->height = mCroppedHeight;
		outputMeta->sarWidth = mSarWidth;
		outputMeta->sarHeight = mSarHeight;
		switch (colorFormat) {
		case MCW_COLOR_FORMAT_YUV420_PLANAR:
			outputMeta->colorFormat =
				AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
			outputMeta->plane[0] =
				pBuf + mCropTop * mWidth + mCropLeft;
			outputMeta->plane[1] =
				pBuf + mWidth * mHeight +
				mCropTop / 2 * mWidth / 2 + mCropLeft / 2;
			outputMeta->plane[2] =
				pBuf + mWidth * mHeight * 5 / 4 +
				mCropTop / 2 * mWidth / 2 + mCropLeft / 2;
			outputMeta->stride[0] = mWidth;
			outputMeta->stride[1] = mWidth / 2;
			outputMeta->stride[2] = mWidth / 2;
			break;
		case MCW_COLOR_FORMAT_YUV420_SEMIPLANAR:
			outputMeta->colorFormat =
				AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR;
			outputMeta->plane[0] =
				pBuf + mCropTop * mWidth + mCropLeft;
			outputMeta->plane[1] =
				pBuf + mWidth * mHeight +
				mCropTop / 2 * mWidth + mCropLeft;
			outputMeta->plane[2] =
				pBuf + mWidth * mHeight +
				mCropTop / 2 * mWidth + mCropLeft;
			outputMeta->stride[0] = mWidth;
			outputMeta->stride[1] = mWidth;
			outputMeta->stride[2] = mWidth;
			break;
		default:
			outputMeta->colorFormat =
				AVCDECODER_COLOR_FORMAT_UNKNOWN;
			outputMeta->plane[0] =
				pBuf + mCropTop * mWidth + mCropLeft;
			outputMeta->plane[1] =
				pBuf + mCropTop * mWidth + mCropLeft;
			outputMeta->plane[2] =
				pBuf + mCropTop * mWidth + mCropLeft;
			outputMeta->stride[0] = mWidth;
			outputMeta->stride[1] = mWidth;
			outputMeta->stride[2] = mWidth;
			break;
		}
		outputMeta->isComplete = inputMeta->isComplete;
		outputMeta->hasErrors = inputMeta->hasErrors;
		outputMeta->isRef = inputMeta->isRef;
		outputMeta->isSilent = inputMeta->isSilent;
		outputMeta->auNtpTimestamp =
			inputMeta->auNtpTimestamp;
		outputMeta->auNtpTimestampRaw =
			inputMeta->auNtpTimestampRaw;
		outputMeta->auNtpTimestampLocal =
			inputMeta->auNtpTimestampLocal;
		outputMeta->demuxOutputTimestamp =
			inputMeta->demuxOutputTimestamp;
		outputMeta->decoderOutputTimestamp =
			(uint64_t)t1.tv_sec * 1000000 +
			(uint64_t)t1.tv_nsec / 1000;

		/* Frame metadata */
		if (inputMeta->hasMetadata) {
			memcpy(&outputMeta->metadata,
				&inputMeta->metadata,
				sizeof(struct vmeta_frame_v2));
			outputMeta->hasMetadata = true;
		} else {
			outputMeta->hasMetadata = false;
		}

		/* User data */
		unsigned int userDataSize =
			vbuf_get_userdata_size(inputBuffer);
		uint8_t *userData = vbuf_get_userdata_ptr(inputBuffer);
		if ((userData) && (userDataSize > 0)) {
			int ret = vbuf_set_userdata_capacity(
				outputBuffer, userDataSize);
			if (ret < (signed)userDataSize) {
				ULOGE("MediaCodec: failed to "
					"realloc user data buffer");
			} else {
				void *dstBuf = vbuf_get_userdata_ptr(
					outputBuffer);
				memcpy(dstBuf, userData, userDataSize);
				vbuf_set_userdata_size(
					outputBuffer, userDataSize);
			}
		} else {
			vbuf_set_userdata_size(outputBuffer, 0);
		}

		/* Push the frame */
		if (!outputMeta->isSilent) {
			std::vector<struct vbuf_queue *>::iterator q =
				mOutputBufferQueues.begin();
			while (q != mOutputBufferQueues.end()) {
				vbuf_queue_push(*q, outputBuffer);
				q++;
			}
		} else {
			ULOGI("MediaCodec: silent frame (ignored)");
		}

		vbuf_unref(&outputBuffer);
		vbuf_unref(&inputBuffer);

		_ret = vbuf_pool_get(mOutputBufferPool, 0, &outputBuffer);
	}

	return ret;
}


void* MediaCodecAvcDecoder::runOutputPollThread(
	void *ptr)
{
	MediaCodecAvcDecoder *decoder = (MediaCodecAvcDecoder *)ptr;

	while (!decoder->mThreadShouldStop) {
		if ((decoder->mConfigured) && (decoder->mMcw != NULL)) {
			int ret = decoder->pollDecoderOutput();
			if (ret != 0)
				ULOGE("MediaCodec: pollDecoderOutput() "
					"failed (%d)", ret);
		} else {
			usleep(5000); /* TODO: do something better (cond?) */
		}
	}

	return NULL;
}

} /* namespace Pdraw */

#endif /* USE_MEDIACODEC */
