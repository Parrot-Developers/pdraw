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

#ifndef _PDRAW_AVCDECODER_HPP_
#define _PDRAW_AVCDECODER_HPP_

#include <inttypes.h>
#include <video-buffers/vbuf.h>
#include <video-decode/vdec.h>
#include "pdraw_decoder.hpp"
#include "pdraw_metadata_videoframe.hpp"

namespace Pdraw {


#define AVCDECODER_BITSTREAM_FORMAT_UNKNOWN		(0)
#define AVCDECODER_BITSTREAM_FORMAT_RAW_NALU		(1 << 0)
#define AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM		(1 << 1)
#define AVCDECODER_BITSTREAM_FORMAT_AVCC		(1 << 2)

#define AVCDECODER_COLOR_FORMAT_UNKNOWN			(0)
#define AVCDECODER_COLOR_FORMAT_YUV420PLANAR		(1 << 0)
#define AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR	(1 << 1)
#define AVCDECODER_COLOR_FORMAT_MMAL_OPAQUE		(1 << 2)

#define AVCDECODER_INPUT_BUFFER_COUNT			(10)


struct avcdecoder_input_source {
	struct vbuf_queue *queue;
	struct vbuf_pool *pool;
	int (*queue_buffer)(
		struct vbuf_queue *queue,
		struct vbuf_buffer *buffer,
		void *userdata);
	void *userdata;
};


struct avcdecoder_input_buffer {
	bool isComplete;
	bool hasErrors;
	bool isRef;
	bool isSilent;
	uint64_t auNtpTimestamp;
	uint64_t auNtpTimestampRaw;
	uint64_t auNtpTimestampLocal;
	bool hasMetadata;
	struct vmeta_frame_v2 metadata;
	uint64_t demuxOutputTimestamp;
};


struct avcdecoder_output_buffer {
	size_t plane_offset[3];
	size_t stride[3];
	unsigned int width;
	unsigned int height;
	unsigned int sarWidth;
	unsigned int sarHeight;
	unsigned int cropLeft;
	unsigned int cropTop;
	unsigned int cropWidth;
	unsigned int cropHeight;
	uint32_t colorFormat;
	bool isComplete;
	bool hasErrors;
	bool isRef;
	bool isSilent;
	uint64_t auNtpTimestamp;
	uint64_t auNtpTimestampRaw;
	uint64_t auNtpTimestampLocal;
	bool hasMetadata;
	struct vmeta_frame_v2 metadata;
	uint64_t demuxOutputTimestamp;
	uint64_t decoderOutputTimestamp;
};


class VideoMedia;


class AvcDecoder : public Decoder {
public:
	AvcDecoder(
		VideoMedia *media);

	~AvcDecoder(
		void);

	uint32_t getInputBitstreamFormatCaps(
		void);

	int open(
		uint32_t inputBitstreamFormat,
		const uint8_t *pSps,
		unsigned int spsSize,
		const uint8_t *pPps,
		unsigned int ppsSize);

	bool isConfigured(
		void) {
		return mConfigured;
	}

	int flush(
		void);

	int close(
		void);

	int getInputSource(
		Media *media,
		struct avcdecoder_input_source *src);

	int addOutputSink(
		Media *media,
		struct vbuf_queue *queue);

	int removeOutputSink(
		Media *media,
		struct vbuf_queue *queue);

	Media *getMedia(
		void) {
		return mMedia;
	}

	VideoMedia *getVideoMedia(
		void) {
		return (VideoMedia *)mMedia;
	}

private:
	static int queueBufferCb(
		struct vbuf_queue *queue,
		struct vbuf_buffer *buffer,
		void *userdata);

	static void frameOutputCb(
		struct vbuf_buffer *out_buf,
		void *userdata);

	static void flushCb(
		void *userdata);

	static void stopCb(
		void *userdata);

	struct vbuf_pool *mInputBufferPool;
	bool mInputBufferPoolAllocated;
	struct vbuf_queue *mInputBufferQueue;
	std::vector<struct vbuf_queue*> mOutputBufferQueues;
	struct vdec_decoder *mVdec;
	unsigned int mFrameIndex;
	enum vdec_input_format mInputFormat;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_AVCDECODER_HPP_ */
