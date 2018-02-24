/**
 * Parrot Drones Awesome Video Viewer Library
 * Apple VideoToolbox H.264/AVC video decoder
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

#include "pdraw_avcdecoder_videotoolbox.hpp"
#include "pdraw_media_video.hpp"

#ifdef USE_VIDEOTOOLBOX

#include <unistd.h>
#include <time.h>
#include <TargetConditionals.h>
#include <video-buffers/vbuf_cvbuffer.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <vector>

namespace Pdraw {


VideoToolboxAvcDecoder::VideoToolboxAvcDecoder(
	VideoMedia *media)
{
	mConfigured = false;
	mOutputColorFormat = AVCDECODER_COLOR_FORMAT_UNKNOWN;
	mMedia = (Media *)media;
	mInputBufferPool = NULL;
	mInputBufferQueue = NULL;
	mOutputBufferPool = NULL;
	mFormatDescRef = NULL;
	mDecompressRef = NULL;
}


VideoToolboxAvcDecoder::~VideoToolboxAvcDecoder(
	void)
{
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

	if (mDecompressRef) {
		VTDecompressionSessionInvalidate(mDecompressRef);
		CFRelease(mDecompressRef);
	}

	if (mFormatDescRef)
		CFRelease(mFormatDescRef);
}


int VideoToolboxAvcDecoder::open(
	uint32_t inputBitstreamFormat,
	const uint8_t *pSps,
	unsigned int spsSize,
	const uint8_t *pPps,
	unsigned int ppsSize)
{
	int ret = 0;
	struct vbuf_cbs cbs;
	OSStatus osstatus;
	const uint8_t *ps[2] = {
		pSps + 4,
		pPps + 4,
	};
	size_t ps_size[2] = {
		spsSize - 4,
		ppsSize - 4,
	};

	if (mConfigured) {
		ULOGE("VideoToolbox: decoder is already configured");
		return -1;
	}
	if ((!pSps) || (spsSize <= 4) || (!pPps) || (ppsSize <= 4)) {
		ULOGE("VideoToolbox: invalid SPS/PPS");
		return -1;
	}
	if (inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_AVCC) {
		ULOGE("VideoToolbox: unsupported input bitstream format");
		return -1;
	}

	osstatus = CMVideoFormatDescriptionCreateFromH264ParameterSets(
		kCFAllocatorDefault, 2, ps, ps_size, 4, &mFormatDescRef);
	if (osstatus != noErr) {
		ULOGE("VideoToolbox: CMVideoFormatDescriptionCreateFrom"
			"H264ParameterSets() failed (%d)", (int)osstatus);
		return -1;
	}

	VTDecompressionOutputCallbackRecord cb;
	cb.decompressionOutputCallback = frameOutputCb;
	cb.decompressionOutputRefCon = this;

	osstatus = VTDecompressionSessionCreate(
		kCFAllocatorDefault, mFormatDescRef,
		NULL, NULL, &cb, &mDecompressRef);
	if (osstatus != noErr) {
		ULOGE("VideoToolbox: VTDecompressionSessionCreate() "
			"failed (%d)", (int)osstatus);
		return -1;
	}

	ret = vbuf_generic_get_cbs(&cbs);
	if (ret != 0) {
		ULOGE("VideoToolbox: failed to get "
			"input buffers allocation callbacks");
		return -1;
	}

	/* Input buffers pool allocation */
	/* TODO: number of buffers and buffers size */
	mInputBufferPool = vbuf_pool_new(
		AVCDECODER_VIDEOTOOLBOX_INPUT_BUFFER_COUNT,
		AVCDECODER_VIDEOTOOLBOX_INPUT_BUFFER_SIZE,
		sizeof(struct avcdecoder_input_buffer), 0, &cbs);
	if (mInputBufferPool == NULL) {
		ULOGE("VideoToolbox: failed to allocate decoder "
			"input buffers pool");
		return -1;
	}

	/* Input buffers queue allocation */
	mInputBufferQueue = vbuf_queue_new();
	if (mInputBufferQueue == NULL) {
		ULOGE("VideoToolbox: failed to allocate decoder "
			"input buffers queue");
		return -1;
	}

	ret = vbuf_cvbuffer_get_cbs(&cbs);
	if (ret != 0) {
		ULOGE("VideoToolbox: failed to get "
			"output buffers allocation callbacks");
		return -1;
	}

	/* Output buffers pool allocation */
	/* TODO: number of buffers */
	mOutputBufferPool = vbuf_pool_new(
		AVCDECODER_VIDEOTOOLBOX_OUTPUT_BUFFER_COUNT, 0,
		sizeof(struct avcdecoder_output_buffer), 0, &cbs);
	if (mOutputBufferPool == NULL) {
		ULOGE("VideoToolbox: failed to allocate decoder "
			"output buffers pool");
		return -1;
	}

	((VideoMedia *)mMedia)->getDimensions(&mWidth, &mHeight,
		&mCropLeft, &mCropRight, &mCropTop, &mCropBottom,
		&mCroppedWidth, &mCroppedHeight, &mSarWidth, &mSarHeight);

	mConfigured = true;
	ULOGI("VideoToolbox: decoder is configured");

	return 0;
}


int VideoToolboxAvcDecoder::close(
	void)
{
	if (!mConfigured) {
		ULOGE("VideoToolbox: decoder is not configured");
		return -1;
	}

	mConfigured = false;

	if (mInputBufferPool)
		vbuf_pool_abort(mInputBufferPool);
	if (mOutputBufferPool)
		vbuf_pool_abort(mOutputBufferPool);
	if (mInputBufferQueue)
		vbuf_queue_abort(mInputBufferQueue);

	return 0;
}


int VideoToolboxAvcDecoder::getInputBuffer(
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (buffer == NULL) {
		ULOGE("VideoToolbox: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("VideoToolbox: decoder is not configured");
		return -1;
	}
	if (mInputBufferPool == NULL) {
		ULOGE("VideoToolbox: input buffer pool has not been created");
		return -1;
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
	if ((ret != 0) || (buf == NULL)) {
		ULOGD("VideoToolbox: failed to get an input buffer (%d)", ret);
		return -2;
	}

	vbuf_set_size(buf, 0);
	*buffer = buf;

	return 0;
}


int VideoToolboxAvcDecoder::queueInputBuffer(
	struct vbuf_buffer *buffer)
{
	CMBlockBufferRef blockBufferRef = NULL;
	CMSampleBufferRef sampleBufferRef = NULL;
	CFArrayRef attachments = NULL;
	CFMutableDictionaryRef dict = NULL;
	int ret = 0;

	if (buffer == NULL) {
		ULOGE("VideoToolbox: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("VideoToolbox: decoder is not configured");
		return -1;
	}
	if (mInputBufferQueue == NULL) {
		ULOGE("VideoToolbox: input queue has not been created");
		return -1;
	}

	uint64_t timestamp = 0;
	struct avcdecoder_input_buffer *data =
		(struct avcdecoder_input_buffer *)vbuf_get_metadata_ptr(buffer);
	if (data)
		timestamp = data->auNtpTimestampRaw;
	OSStatus osstatus;
	CMSampleTimingInfo timingInfo[1] = { {
		.duration = kCMTimeInvalid,
		.presentationTimeStamp = CMTimeMake(timestamp, 1000000),
		.decodeTimeStamp = kCMTimeInvalid
	} };
	VTDecodeFrameFlags decodeFlags = 0;
	VTDecodeInfoFlags infoFlags = 0;

	osstatus = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault,
		vbuf_get_ptr(buffer), vbuf_get_size(buffer),
		kCFAllocatorNull, NULL, 0, vbuf_get_size(buffer),
		0, &blockBufferRef);
	if (osstatus != noErr) {
		ULOGE("VideoToolbox: CMBlockBufferCreateWithMemoryBlock()"
			" failed (%d)", (int)osstatus);
		ret = -1;
		goto out;
	}

	osstatus = CMSampleBufferCreate(kCFAllocatorDefault,
		blockBufferRef, TRUE, NULL, NULL, mFormatDescRef,
		1, 1, timingInfo, 0, NULL, &sampleBufferRef);
	if (osstatus != noErr) {
		ULOGE("VideoToolbox: CMSampleBufferCreate()"
			" failed (%d)", (int)osstatus);
		ret = -1;
		goto out;
	}

	attachments = CMSampleBufferGetSampleAttachmentsArray(
		sampleBufferRef, TRUE);
	dict = (CFMutableDictionaryRef)CFArrayGetValueAtIndex(attachments, 0);
	CFDictionarySetValue(dict,
		kCMSampleAttachmentKey_DisplayImmediately, kCFBooleanTrue);
#if TARGET_OS_IPHONE
	CFDictionarySetValue(dict,
		kCVPixelBufferOpenGLESCompatibilityKey, kCFBooleanTrue);
#endif

	decodeFlags |= kVTDecodeFrame_EnableAsynchronousDecompression;
	osstatus = VTDecompressionSessionDecodeFrame(mDecompressRef,
		sampleBufferRef, decodeFlags, buffer, &infoFlags);
	if (osstatus != noErr) {
		ULOGE("VideoToolbox: VTDecompressionSessionDecodeFrame()"
			" failed (%d)", (int)osstatus);
		ret = -1;
		goto out;
	}

	ret = vbuf_queue_push(mInputBufferQueue, buffer);
	if (ret != 0)
		ULOGE("VideoToolbox: failed to push the buffer "
			"into the input queue");

out:
	if (blockBufferRef != NULL)
		CFRelease(blockBufferRef);
	if (sampleBufferRef != NULL)
		CFRelease(sampleBufferRef);

	return 0;
}


struct vbuf_queue *VideoToolboxAvcDecoder::addOutputQueue(
	void)
{
	struct vbuf_queue *q = vbuf_queue_new();
	if (q == NULL) {
		ULOGE("VideoToolbox: queue allocation failed");
		return NULL;
	}

	mOutputBufferQueues.push_back(q);
	return q;
}


int VideoToolboxAvcDecoder::removeOutputQueue(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("VideoToolbox: invalid queue pointer");
		return -1;
	}

	bool found = false;
	std::vector<struct vbuf_queue*>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			mOutputBufferQueues.erase(q);
			int ret = vbuf_queue_destroy(*q);
			if (ret != 0)
				ULOGE("VideoToolbox: vbuf_queue_destroy()"
					" failed (%d)", ret);
			found = true;
			break;
		}
		q++;
	}

	return (found) ? 0 : -1;
}


bool VideoToolboxAvcDecoder::isOutputQueueValid(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("VideoToolbox: invalid queue pointer");
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


void VideoToolboxAvcDecoder::frameOutputCb(
	void *decompressionOutputRefCon,
	void *sourceFrameRefCon,
	OSStatus status,
	VTDecodeInfoFlags infoFlags,
	CVImageBufferRef imageBuffer,
	CMTime presentationTimeStamp,
	CMTime presentationDuration)
{
	VideoToolboxAvcDecoder *decoder =
		(VideoToolboxAvcDecoder*)decompressionOutputRefCon;
	struct vbuf_buffer *buffer = (struct vbuf_buffer *)sourceFrameRefCon;
	struct vbuf_buffer *inputBuffer = NULL;
	struct vbuf_buffer *outputBuffer = NULL;
	struct avcdecoder_input_buffer *inputMeta = NULL;
	struct avcdecoder_output_buffer *outputMeta = NULL;
	CVPixelBufferRef pixelBuffer = imageBuffer;
	OSType pixelFormat;
	uint8_t *userData;
	size_t userDataSize;
	struct timespec t1;
	int ret;

	if (!decoder->mConfigured) {
		return;
	}
	if (imageBuffer == NULL) {
		ULOGE("VideoToolbox: invalid buffer");
		return;
	}

	ret = vbuf_queue_pop(decoder->mInputBufferQueue, -1, &inputBuffer);
	if ((ret != 0) || (inputBuffer == NULL)) {
		ULOGE("VideoToolbox: failed to dequeue "
			"an input buffer (%d)", ret);
		goto out;
	}

	if (status != noErr) {
		ULOGE("VideoToolbox: decoder error %d", (int)status);
		goto out;
	}

	if (inputBuffer != buffer) {
		ULOGE("VideoToolbox: buffer mismatch (%p vs. %p)",
			inputBuffer, buffer);
		goto out;
	}

	inputMeta = (struct avcdecoder_input_buffer *)vbuf_get_metadata_ptr(
		inputBuffer);
	if (inputMeta == NULL) {
		ULOGE("VideoToolbox: invalid input buffer metadata");
		goto out;
	}

	ret = vbuf_pool_get(decoder->mOutputBufferPool, 0, &outputBuffer);
	if ((ret != 0) || (outputBuffer == NULL)) {
		ULOGE("VideoToolbox: failed to get an output buffer (%d)", ret);
		goto out;
	}

	outputMeta = (struct avcdecoder_output_buffer *)vbuf_get_metadata_ptr(
		outputBuffer);
	if (outputMeta == NULL) {
		ULOGE("VideoToolbox: invalid output buffer metadata");
		goto out;
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	memset(outputMeta, 0, sizeof(*outputMeta));

	ret = vbuf_cvbuffer_set_buffer(outputBuffer,
		(struct _CVBufferRef *)imageBuffer, 1, 1);
	if (ret != 0) {
		ULOGE("VideoToolbox: vbuf_cvbuffer_set_buffer() failed (%d)",
			ret);
		goto out;
	}

	/* Pixel data */
	pixelFormat = CVPixelBufferGetPixelFormatType(pixelBuffer);
	outputMeta->width = decoder->mCroppedWidth;
	outputMeta->height = decoder->mCroppedHeight;
	outputMeta->sarWidth = decoder->mSarWidth;
	outputMeta->sarHeight = decoder->mSarHeight;
	switch (pixelFormat) {
	case kCVPixelFormatType_420YpCbCr8Planar:
	case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
		outputMeta->colorFormat =
			AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
		outputMeta->plane[0] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
		outputMeta->plane[1] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
		outputMeta->plane[2] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 2);
		outputMeta->stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
		outputMeta->stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
		outputMeta->stride[2] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 2);
		break;
	case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
	case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
		outputMeta->colorFormat =
			AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR;
		outputMeta->plane[0] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
		outputMeta->plane[1] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
		outputMeta->plane[2] = NULL;
		outputMeta->stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
		outputMeta->stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
		outputMeta->stride[2] = 0;
		break;
	default:
		outputMeta->colorFormat =
			AVCDECODER_COLOR_FORMAT_UNKNOWN;
		outputMeta->plane[0] = (uint8_t *)
			CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
		outputMeta->plane[1] = NULL;
		outputMeta->plane[2] = NULL;
		outputMeta->stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
		outputMeta->stride[1] = 0;
		outputMeta->stride[2] = 0;
		break;
	}
	outputMeta->isComplete = inputMeta->isComplete;
	outputMeta->hasErrors = inputMeta->hasErrors;
	outputMeta->isRef = inputMeta->isRef;
	outputMeta->isSilent = inputMeta->isSilent;
	outputMeta->auNtpTimestamp = inputMeta->auNtpTimestamp;
	outputMeta->auNtpTimestampRaw = inputMeta->auNtpTimestampRaw;
	outputMeta->auNtpTimestampLocal = inputMeta->auNtpTimestampLocal;
	outputMeta->demuxOutputTimestamp = inputMeta->demuxOutputTimestamp;
	outputMeta->decoderOutputTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	/* Frame metadata */
	if (inputMeta->hasMetadata) {
		memcpy(&outputMeta->metadata, &inputMeta->metadata,
			sizeof(outputMeta->metadata));
		outputMeta->hasMetadata = true;
	} else {
		outputMeta->hasMetadata = false;
	}

	/* User data */
	userData = vbuf_get_userdata_ptr(inputBuffer);
	userDataSize = vbuf_get_userdata_size(inputBuffer);
	if ((userData) && (userDataSize > 0)) {
		ret = vbuf_set_userdata_capacity(outputBuffer, userDataSize);
		if (ret < (ssize_t)userDataSize) {
			ULOGE("VideoToolbox: failed to realloc "
				"user data buffer");
		} else {
			uint8_t *dstBuf = vbuf_get_userdata_ptr(outputBuffer);
			memcpy(dstBuf, userData, userDataSize);
			vbuf_set_userdata_size(outputBuffer, userDataSize);
		}
	} else {
		vbuf_set_userdata_size(outputBuffer, 0);
	}

	/* Push the frame */
	if (!outputMeta->isSilent) {
		std::vector<struct vbuf_queue *>::iterator q =
			decoder->mOutputBufferQueues.begin();
		while (q != decoder->mOutputBufferQueues.end()) {
			vbuf_queue_push(*q, outputBuffer);
			q++;
		}
	} else {
		ULOGI("VideoToolbox: silent frame (ignored)");
	}

out:
	if (inputBuffer)
		vbuf_unref(&inputBuffer);
	if (outputBuffer)
		vbuf_unref(&outputBuffer);
}

} /* namespace Pdraw */

#endif /* USE_VIDEOTOOLBOX */
