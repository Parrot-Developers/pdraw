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

#ifndef _PDRAW_AVCDECODER_MEDIACODEC_HPP_
#define _PDRAW_AVCDECODER_MEDIACODEC_HPP_

#ifdef USE_MEDIACODEC

#include <pthread.h>
#include <mediacodec-wrapper/libmcw.h>
#include <vector>
#include "pdraw_avcdecoder.hpp"

namespace Pdraw {


#define AVCDECODER_MEDIACODEC_INPUT_BUFFER_COUNT	(20)
#define AVCDECODER_MEDIACODEC_OUTPUT_BUFFER_COUNT	(20)


class MediaCodecAvcDecoder : public AvcDecoder {
public:
	MediaCodecAvcDecoder(
		VideoMedia *media);

	~MediaCodecAvcDecoder(
		void);

	uint32_t getInputBitstreamFormatCaps(
		void) {
		return AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM;
	}

	int configure(
		uint32_t inputBitstreamFormat,
		const uint8_t *pSps,
		unsigned int spsSize,
		const uint8_t *pPps,
		unsigned int ppsSize);

	bool isConfigured(
		void) {
		return mConfigured;
	}

	int getInputBuffer(
		struct vbuf_buffer **buffer,
		bool blocking);

	int queueInputBuffer(
		struct vbuf_buffer *buffer);

	struct vbuf_queue *addOutputQueue(
		void);

	int removeOutputQueue(
		struct vbuf_queue *queue);

	int dequeueOutputBuffer(
		struct vbuf_queue *queue,
		struct vbuf_buffer **buffer,
		bool blocking);

	int releaseOutputBuffer(
		struct vbuf_buffer **buffer);

	int stop(
		void);

	Media *getMedia(
		void) {
		return mMedia;
	}

	VideoMedia *getVideoMedia(
		void) {
		return (VideoMedia *)mMedia;
	}

private:
	bool isOutputQueueValid(
		struct vbuf_queue *queue);

	int pollDecoderOutput(
		void);

	static void* runOutputPollThread(
		void *ptr);

	struct mcw *mMcw;
	struct mcw_mediacodec *mCodec;
	struct vbuf_pool *mInputBufferPool;
	struct vbuf_queue *mInputBufferQueue;
	struct vbuf_pool *mOutputBufferPool;
	std::vector<struct vbuf_queue*> mOutputBufferQueues;
	pthread_t mOutputPollThread;
	bool mOutputPollThreadLaunched;
	bool mThreadShouldStop;
	unsigned int mWidth;
	unsigned int mHeight;
	unsigned int mCropLeft;
	unsigned int mCropRight;
	unsigned int mCropTop;
	unsigned int mCropBottom;
	unsigned int mCroppedWidth;
	unsigned int mCroppedHeight;
	unsigned int mSarWidth;
	unsigned int mSarHeight;
};

} /* namespace Pdraw */

#endif /* USE_MEDIACODEC */

#endif /* !_PDRAW_AVCDECODER_MEDIACODEC_HPP_ */
