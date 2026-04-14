/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include <inttypes.h>
#include <pthread.h>

#include <atomic>
#include <climits>
#include <string>
#include <vector>

#include <pdraw/pdraw_defs.h>

/* mbuf ancillary data key for CodedVideoMedia::Frame objects */
constexpr const char *PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME =
	"pdraw.coded_video_media.frame";

/* mbuf ancillary data key for RawVideoMedia::Frame objects */
constexpr const char *PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME =
	"pdraw.raw_video_media.frame";

/* mbuf ancillary data key for AudioMedia::Frame objects */
constexpr const char *PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME =
	"pdraw.audio_media.frame";

namespace Pdraw {

class Session;

class Media {
public:
	enum class Type {
		UNKNOWN = 0,
		RAW_VIDEO = (1 << 0),
		CODED_VIDEO = (1 << 1),
		AUDIO = (1 << 2),
	};

	Media(Session *session, Type t);

	virtual ~Media() = default;

	const std::string &getName() const;

	const std::string &getPath() const;

	void setPath(const std::string &name);

	void setPath(const char *name);

	void setTearingDown()
	{
		mTearingDown = true;
	}

	bool isTearingDown() const
	{
		return mTearingDown;
	}

	static const char *getMediaTypeStr(Type val);

	virtual void fillMediaInfo(struct pdraw_media_info *minfo) = 0;

	static void cleanupMediaInfo(struct pdraw_media_info *minfo);

	Type type = Type::UNKNOWN;
	unsigned int id = UINT_MAX;
	enum pdraw_playback_type playbackType = PDRAW_PLAYBACK_TYPE_UNKNOWN;
	uint64_t duration = 0;

protected:
	void setClassName(const std::string &name);

	void setClassName(const char *name);

private:
	Session *mSession = nullptr;
	bool mTearingDown = false;
	std::string mName{};
	std::string mPath{};
	static std::atomic<unsigned int> mIdCounter;
};


class RawVideoMedia : public Media {
public:
	struct Frame {
		uint64_t ntpTimestamp = 0;
		uint64_t ntpUnskewedTimestamp = 0;
		uint64_t ntpRawTimestamp = 0;
		uint64_t ntpRawUnskewedTimestamp = 0;
		uint64_t playTimestamp = 0;
		uint64_t captureTimestamp = 0;
		uint64_t localTimestamp = 0;
		uint32_t localTimestampPrecision = 0;
		uint64_t recvStartTimestamp = 0;
		uint64_t recvEndTimestamp = 0;
		uint64_t demuxOutputTimestamp = 0;
		uint64_t decoderOutputTimestamp = 0;
		uint64_t scalerOutputTimestamp = 0;
		uint64_t renderTimestamp = 0;
	};

	explicit RawVideoMedia(Session *session);

	~RawVideoMedia() override = default;

	void fillMediaInfo(struct pdraw_media_info *minfo) override;

	struct vdef_raw_format format {
	};
	struct vdef_format_info info {
	};
	struct vmeta_session sessionMeta {
	};
};


class CodedVideoMedia : public Media {
public:
	struct Frame {
		bool isSync = false;
		bool isRef = false;
		uint64_t ntpTimestamp = 0;
		uint64_t ntpUnskewedTimestamp = 0;
		uint64_t ntpRawTimestamp = 0;
		uint64_t ntpRawUnskewedTimestamp = 0;
		uint64_t playTimestamp = 0;
		uint64_t captureTimestamp = 0;
		uint64_t localTimestamp = 0;
		uint32_t localTimestampPrecision = 0;
		uint64_t recvStartTimestamp = 0;
		uint64_t recvEndTimestamp = 0;
		uint64_t demuxOutputTimestamp = 0;
		uint64_t encoderOutputTimestamp = 0;
	};

	explicit CodedVideoMedia(Session *session);

	~CodedVideoMedia() override = default;

	int getPs(const uint8_t **vps,
		  size_t *vpsSize,
		  const uint8_t **sps,
		  size_t *spsSize,
		  const uint8_t **pps,
		  size_t *ppsSize) const;

	int setPs(const uint8_t *vps,
		  size_t vpsSize,
		  const uint8_t *sps,
		  size_t spsSize,
		  const uint8_t *pps,
		  size_t ppsSize);

	void fillMediaInfo(struct pdraw_media_info *minfo) override;

	struct vdef_coded_format format {
	};
	struct vdef_format_info info {
	};
	struct vmeta_session sessionMeta {
	};

private:
	std::vector<uint8_t> mVps{};
	std::vector<uint8_t> mSps{};
	std::vector<uint8_t> mPps{};
};


class AudioMedia : public Media {
public:
	struct Frame {
		uint64_t ntpTimestamp = 0;
		uint64_t ntpUnskewedTimestamp = 0;
		uint64_t ntpRawTimestamp = 0;
		uint64_t ntpRawUnskewedTimestamp = 0;
		uint64_t playTimestamp = 0;
		uint64_t captureTimestamp = 0;
		uint64_t localTimestamp = 0;
		uint32_t localTimestampPrecision = 0;
		uint64_t recvStartTimestamp = 0;
		uint64_t recvEndTimestamp = 0;
		uint64_t demuxOutputTimestamp = 0;
		uint64_t encoderOutputTimestamp = 0;
		uint64_t decoderOutputTimestamp = 0;
	};

	explicit AudioMedia(Session *session);

	~AudioMedia() override = default;

	int getAacAsc(const uint8_t **asc, size_t *ascSize) const;

	int setAacAsc(const uint8_t *asc, size_t ascSize);

	void fillMediaInfo(struct pdraw_media_info *minfo) override;

	struct adef_format format {
	};

private:
	std::vector<uint8_t> mAacAsc{};
};

} /* namespace Pdraw */
