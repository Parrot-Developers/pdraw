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

#include <atomic>
#include <string>

#include <pdraw/pdraw_defs.h>

/**
 * mbuf ancillary data key for CodedVideoMedia::Frame objects
 */
#define PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME "pdraw.coded_video_media.frame"

/**
 * mbuf ancillary data key for RawVideoMedia::Frame objects
 */
#define PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME "pdraw.raw_video_media.frame"

namespace Pdraw {

class Session;

class Media {
public:
	enum Type {
		UNKNOWN = 0,
		RAW_VIDEO = (1 << 0),
		CODED_VIDEO = (1 << 1),
		RAW_AUDIO = (1 << 2),
		CODED_AUDIO = (1 << 3),
	};

	Media(Session *session, Type t);

	virtual ~Media(void) {}

	std::string &getName(void);

	std::string &getPath(void);

	void setPath(std::string &name);

	void setPath(const char *name);

	static const char *getMediaTypeStr(Type val);

	virtual void fillMediaInfo(struct pdraw_media_info *minfo) = 0;

	static void cleanupMediaInfo(struct pdraw_media_info *minfo);

	Type type;
	unsigned int id;
	struct vmeta_session sessionMeta;
	enum pdraw_playback_type playbackType;
	uint64_t duration;

protected:
	void setClassName(std::string &name);

	void setClassName(const char *name);

private:
	Session *mSession;
	std::string mName;
	std::string mPath;
	static std::atomic<unsigned int> mIdCounter;
};


class RawVideoMedia : public Media {
public:
	struct Frame {
		uint64_t ntpTimestamp;
		uint64_t ntpUnskewedTimestamp;
		uint64_t ntpRawTimestamp;
		uint64_t ntpRawUnskewedTimestamp;
		uint64_t playTimestamp;
		uint64_t captureTimestamp;
		uint64_t localTimestamp;
		uint32_t localTimestampPrecision;
		uint64_t recvStartTimestamp;
		uint64_t recvEndTimestamp;
		uint64_t demuxOutputTimestamp;
		uint64_t decoderOutputTimestamp;
		uint64_t scalerOutputTimestamp;
		uint64_t renderTimestamp;
	};

	RawVideoMedia(Session *session);

	~RawVideoMedia(void);

	virtual void fillMediaInfo(struct pdraw_media_info *minfo);

	struct vdef_raw_format format;
	struct vdef_format_info info;
};


class CodedVideoMedia : public Media {
public:
	struct CodedFrame {
	};

	struct Frame {
		bool isSync;
		bool isRef;
		uint64_t ntpTimestamp;
		uint64_t ntpUnskewedTimestamp;
		uint64_t ntpRawTimestamp;
		uint64_t ntpRawUnskewedTimestamp;
		uint64_t playTimestamp;
		uint64_t captureTimestamp;
		uint64_t localTimestamp;
		uint32_t localTimestampPrecision;
		uint64_t recvStartTimestamp;
		uint64_t recvEndTimestamp;
		uint64_t demuxOutputTimestamp;
		uint64_t encoderOutputTimestamp;
	};

	CodedVideoMedia(Session *session);

	~CodedVideoMedia(void);

	int getPs(const uint8_t **vps,
		  size_t *vpsSize,
		  const uint8_t **sps,
		  size_t *spsSize,
		  const uint8_t **pps,
		  size_t *ppsSize);

	int setPs(const uint8_t *vps,
		  size_t vpsSize,
		  const uint8_t *sps,
		  size_t spsSize,
		  const uint8_t *pps,
		  size_t ppsSize);

	virtual void fillMediaInfo(struct pdraw_media_info *minfo);

	struct vdef_coded_format format;
	struct vdef_format_info info;

private:
	uint8_t *mVps;
	size_t mVpsSize;
	uint8_t *mSps;
	size_t mSpsSize;
	uint8_t *mPps;
	size_t mPpsSize;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MEDIA_HPP_ */
