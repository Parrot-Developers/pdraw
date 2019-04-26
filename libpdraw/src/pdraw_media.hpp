/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline media
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

#ifndef _PDRAW_MEDIA_HPP_
#define _PDRAW_MEDIA_HPP_

#include <inttypes.h>
#include <pthread.h>

#include <string>

#include <pdraw/pdraw_defs.h>

namespace Pdraw {

class Session;

class Media {
public:
	enum Type {
		UNKNOWN = 0,
		VIDEO = (1 << 0),
		AUDIO = (1 << 1),
	};

	Media(Session *session, Type t);

	virtual ~Media(void) {}

	static const char *getMediaTypeStr(Type val);

	Type type;
	unsigned int id;

private:
	Session *mSession;
	static pthread_mutex_t mMutex;
	static unsigned int mIdCounter;
};


class VideoMedia : public Media {
public:
	enum Format {
		FORMAT_UNKNOWN = 0,
		YUV = (1 << 0),
		H264 = (1 << 1),
		OPAQUE = (1 << 2),
	};

	enum SubFormat {
		SUBFORMAT_UNKNOWN = 0,
		YUV_I420 = (1 << 0),
		YUV_NV12 = (1 << 1),
		OPAQUE_MMAL = (1 << 2),
		H264_BYTE_STREAM = (1 << 3),
		H264_AVCC = (1 << 4),
	};

	enum YuvFormat {
		YUV_UNKNOWN = SubFormat::SUBFORMAT_UNKNOWN,
		I420 = SubFormat::YUV_I420,
		NV12 = SubFormat::YUV_NV12,
	};

	enum OpaqueFormat {
		OPAQUE_UNKNOWN = SubFormat::SUBFORMAT_UNKNOWN,
		MMAL = SubFormat::OPAQUE_MMAL,
	};

	enum H264BitstreamFormat {
		H264_UNKNOWN = SubFormat::SUBFORMAT_UNKNOWN,
		BYTE_STREAM = SubFormat::H264_BYTE_STREAM,
		AVCC = SubFormat::H264_AVCC,
	};

	struct YuvFrame {
		YuvFormat format;
		size_t planeOffset[3];
		size_t planeStride[3];
		unsigned int width;
		unsigned int height;
		unsigned int sarWidth;
		unsigned int sarHeight;
		unsigned int cropLeft;
		unsigned int cropTop;
		unsigned int cropWidth;
		unsigned int cropHeight;
		bool fullRange;
	};

	struct OpaqueFrame {
		OpaqueFormat format;
		size_t stride;
		unsigned int width;
		unsigned int height;
		unsigned int sarWidth;
		unsigned int sarHeight;
		unsigned int cropLeft;
		unsigned int cropTop;
		unsigned int cropWidth;
		unsigned int cropHeight;
		bool fullRange;
	};

	struct H264Frame {
		H264BitstreamFormat format;
		bool isComplete;
		bool isSync;
		bool isRef;
	};

	struct Frame {
		Format format;
		union {
			struct YuvFrame yuvFrame;
			struct OpaqueFrame opaqueFrame;
			struct H264Frame h264Frame;
		};
		bool hasErrors;
		bool isSilent;
		uint64_t ntpTimestamp;
		uint64_t ntpUnskewedTimestamp;
		uint64_t ntpRawTimestamp;
		uint64_t ntpRawUnskewedTimestamp;
		uint64_t playTimestamp;
		uint64_t captureTimestamp;
		uint64_t localTimestamp;
		bool hasMetadata;
		struct vmeta_frame metadata;
		uint64_t demuxOutputTimestamp;
		uint64_t decoderOutputTimestamp;
	};

	VideoMedia(Session *session);

	virtual ~VideoMedia(void);

	int getSpsPps(uint8_t **sps,
		      size_t *spsSize,
		      uint8_t **pps,
		      size_t *ppsSize);

	int setSpsPps(const uint8_t *sps,
		      size_t spsSize,
		      const uint8_t *pps,
		      size_t ppsSize);

	static const char *getVideoFormatStr(Format val);

	static const char *getVideoSubFormatStr(SubFormat val);

	Format format;
	union {
		SubFormat subFormat;
		YuvFormat yuvFormat;
		OpaqueFormat opaqueFormat;
		H264BitstreamFormat h264BitstreamFormat;
	};
	unsigned int width;
	unsigned int height;
	unsigned int cropLeft;
	unsigned int cropWidth;
	unsigned int cropTop;
	unsigned int cropHeight;
	unsigned int sarWidth;
	unsigned int sarHeight;
	float hfov;
	float vfov;
	bool fullRange;

private:
	uint8_t *mSps;
	size_t mSpsSize;
	uint8_t *mPps;
	size_t mPpsSize;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MEDIA_HPP_ */
