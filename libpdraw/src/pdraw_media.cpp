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

#define ULOG_TAG pdraw_media
#include <ulog.h>

#include "pdraw_media.hpp"

#include <algorithm>

#include <h264/h264.h>
#include <h265/h265.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


std::atomic<unsigned int> Media::mIdCounter(0);


Media::Media([[maybe_unused]] Session *session, Type t) :
		mType(t), mId(++mIdCounter),
		mName(std::string(__func__) + "#" + std::to_string(mId))

{
}


const std::string &Media::getName() const
{
	return mName;
}


void Media::setClassName(const std::string &name)
{
	mName = name + "#" + std::to_string(mId);
}


void Media::setClassName(const char *name)
{
	mName = std::string(name) + "#" + std::to_string(mId);
}


const std::string &Media::getPath() const
{
	return mPath;
}


void Media::setPath(std::string_view path)
{
	mPath = path;
}


void Media::setPath(const char *path)
{
	mPath = path;
}


const char *Media::getMediaTypeStr(Type val)
{
	switch (val) {
	case Media::Type::UNKNOWN:
		return "UNKNOWN";
	case Media::Type::RAW_VIDEO:
		return "RAW_VIDEO";
	case Media::Type::CODED_VIDEO:
		return "CODED_VIDEO";
	case Media::Type::AUDIO:
		return "AUDIO";
	default:
		return nullptr;
	}
}


void Media::cleanupMediaInfo(struct pdraw_media_info *minfo)
{
	free(const_cast<char *>(minfo->name));
	minfo->name = nullptr;
	free(const_cast<char *>(minfo->path));
	minfo->path = nullptr;
}


RawVideoMedia::RawVideoMedia(Session *session) : Media(session, Type::RAW_VIDEO)
{
	Media::setClassName(__func__);
}


void RawVideoMedia::fillMediaInfo(struct pdraw_media_info *minfo)
{
	if (!minfo)
		return;

	*minfo = {};

	minfo->type = PDRAW_MEDIA_TYPE_VIDEO;
	minfo->id = mId;
	minfo->name = strdup(getName().c_str());
	minfo->path = strdup(getPath().c_str());
	minfo->playback_type = mPlaybackType;
	minfo->duration = mDuration;
	minfo->video.format = VDEF_FRAME_TYPE_RAW;
	minfo->video.session_meta = &sessionMeta;
	minfo->video.raw.format = format;
	minfo->video.raw.info = info;
}


CodedVideoMedia::CodedVideoMedia(Session *session) :
		Media(session, Type::CODED_VIDEO)
{
	Media::setClassName(__func__);
}


int CodedVideoMedia::getPs(const uint8_t **vps,
			   size_t *vpsSize,
			   const uint8_t **sps,
			   size_t *spsSize,
			   const uint8_t **pps,
			   size_t *ppsSize) const
{
	enum vdef_encoding encoding = format.encoding;
	if ((encoding != VDEF_ENCODING_H264) &&
	    (encoding != VDEF_ENCODING_H265))
		return -EPROTO;

	if (encoding == VDEF_ENCODING_H265) {
		/* VPS is for H.265 only */
		if (vps)
			*vps = mVps.data();
		if (vpsSize)
			*vpsSize = mVps.size();
	}
	if (sps)
		*sps = mSps.data();
	if (spsSize)
		*spsSize = mSps.size();
	if (pps)
		*pps = mPps.data();
	if (ppsSize)
		*ppsSize = mPps.size();

	return 0;
}


int CodedVideoMedia::setPs(const uint8_t *vps,
			   size_t vpsSize,
			   const uint8_t *sps,
			   size_t spsSize,
			   const uint8_t *pps,
			   size_t ppsSize)
{
	int ret;
	struct h264_info h264Info;
	struct h265_info h265Info;
	enum vdef_encoding encoding = format.encoding;

	if ((encoding != VDEF_ENCODING_H264) &&
	    (encoding != VDEF_ENCODING_H265))
		return -EPROTO;
	if ((encoding == VDEF_ENCODING_H265) &&
	    ((vps == nullptr) || (vpsSize == 0)))
		return -EINVAL;
	if ((sps == nullptr) || (spsSize == 0))
		return -EINVAL;
	if ((pps == nullptr) || (ppsSize == 0))
		return -EINVAL;

	try {
		mVps.clear();
		if (encoding == VDEF_ENCODING_H265)
			mVps = std::vector<uint8_t>(vps, vps + vpsSize);

		mSps.clear();
		mSps = std::vector<uint8_t>(sps, sps + spsSize);

		mPps.clear();
		mPps = std::vector<uint8_t>(pps, pps + ppsSize);

		if (encoding == VDEF_ENCODING_H264) {
			ret = h264_get_info(
				sps, spsSize, pps, ppsSize, &h264Info);
			if (ret < 0) {
				ULOG_ERRNO("h264_get_info", -ret);
				return ret;
			}

			info.bit_depth = h264Info.bit_depth_luma;
			info.full_range = h264Info.full_range;
			info.color_primaries = vdef_color_primaries_from_h264(
				h264Info.colour_primaries);
			info.transfer_function =
				vdef_transfer_function_from_h264(
					h264Info.transfer_characteristics);
			info.matrix_coefs = vdef_matrix_coefs_from_h264(
				h264Info.matrix_coefficients);
			info.resolution.width = h264Info.crop_width;
			info.resolution.height = h264Info.crop_height;
			info.sar.width = h264Info.sar_width;
			info.sar.height = h264Info.sar_height;
			info.framerate.num = h264Info.framerate_num;
			info.framerate.den = h264Info.framerate_den;
		} else if (encoding == VDEF_ENCODING_H265) {
			ret = h265_get_info(vps,
					    vpsSize,
					    sps,
					    spsSize,
					    pps,
					    ppsSize,
					    &h265Info);
			if (ret < 0) {
				ULOG_ERRNO("h265_get_info", -ret);
				return ret;
			}

			info.bit_depth = h265Info.bit_depth_luma;
			info.full_range = h265Info.full_range;
			info.color_primaries = vdef_color_primaries_from_h265(
				h265Info.colour_primaries);
			info.transfer_function =
				vdef_transfer_function_from_h265(
					h265Info.transfer_characteristics);
			info.matrix_coefs = vdef_matrix_coefs_from_h265(
				h265Info.matrix_coefficients);
			info.resolution.width = h265Info.crop_width;
			info.resolution.height = h265Info.crop_height;
			info.sar.width = h265Info.sar_width;
			info.sar.height = h265Info.sar_height;
			info.framerate.num = h265Info.framerate_num;
			info.framerate.den = h265Info.framerate_den;
		}

		return 0;
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		ULOG_ERRNO("std::vector allocation failed", -ret);
		mVps.clear();
		mSps.clear();
		mPps.clear();
		return ret;
	}
}


void CodedVideoMedia::fillMediaInfo(struct pdraw_media_info *minfo)
{
	size_t cplen;

	if (!minfo)
		return;

	*minfo = {};

	minfo->type = PDRAW_MEDIA_TYPE_VIDEO;
	minfo->id = mId;
	minfo->name = strdup(getName().c_str());
	minfo->path = strdup(getPath().c_str());
	minfo->playback_type = mPlaybackType;
	minfo->duration = mDuration;
	minfo->video.format = VDEF_FRAME_TYPE_CODED;
	minfo->video.session_meta = &sessionMeta;
	minfo->video.coded.format = format;
	minfo->video.coded.info = info;
	switch (format.encoding) {
	case VDEF_ENCODING_H264:
		if (sizeof(minfo->video.coded.h264.sps) < mSps.size())
			ULOGW("%s: truncated SPS", __func__);
		cplen = std::min(mSps.size(),
				 sizeof(minfo->video.coded.h264.sps));
		memcpy(minfo->video.coded.h264.sps, mSps.data(), cplen);
		minfo->video.coded.h264.spslen = cplen;
		if (sizeof(minfo->video.coded.h264.pps) < mPps.size())
			ULOGW("%s: truncated PPS", __func__);
		cplen = std::min(mPps.size(),
				 sizeof(minfo->video.coded.h264.pps));
		memcpy(minfo->video.coded.h264.pps, mPps.data(), cplen);
		minfo->video.coded.h264.ppslen = cplen;
		break;
	case VDEF_ENCODING_H265:
		if (sizeof(minfo->video.coded.h265.vps) < mVps.size())
			ULOGW("%s: truncated VPS", __func__);
		cplen = std::min(mVps.size(),
				 sizeof(minfo->video.coded.h265.vps));
		memcpy(minfo->video.coded.h265.vps, mVps.data(), cplen);
		minfo->video.coded.h265.vpslen = cplen;
		if (sizeof(minfo->video.coded.h265.sps) < mSps.size())
			ULOGW("%s: truncated SPS", __func__);
		cplen = std::min(mSps.size(),
				 sizeof(minfo->video.coded.h265.sps));
		memcpy(minfo->video.coded.h265.sps, mSps.data(), cplen);
		minfo->video.coded.h265.spslen = cplen;
		if (sizeof(minfo->video.coded.h265.pps) < mPps.size())
			ULOGW("%s: truncated PPS", __func__);
		cplen = std::min(mPps.size(),
				 sizeof(minfo->video.coded.h265.pps));
		memcpy(minfo->video.coded.h265.pps, mPps.data(), cplen);
		minfo->video.coded.h265.ppslen = cplen;
		break;
	default:
		break;
	}
}


AudioMedia::AudioMedia(Session *session) : Media(session, Type::AUDIO)
{
	Media::setClassName(__func__);
}


int AudioMedia::getAacAsc(const uint8_t **asc, size_t *ascSize) const
{
	enum adef_encoding encoding = format.encoding;
	if (encoding != ADEF_ENCODING_AAC_LC)
		return -EPROTO;

	if (asc)
		*asc = mAacAsc.data();
	if (ascSize)
		*ascSize = mAacAsc.size();

	return 0;
}


int AudioMedia::setAacAsc(const uint8_t *asc, size_t ascSize)
{
	int ret;
	enum adef_encoding encoding = format.encoding;

	if (encoding != ADEF_ENCODING_AAC_LC)
		return -EPROTO;
	if ((encoding == ADEF_ENCODING_AAC_LC) &&
	    ((asc == nullptr) || (ascSize == 0)))
		return -EINVAL;

	try {
		mAacAsc.clear();
		mAacAsc = std::vector<uint8_t>(asc, asc + ascSize);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		ULOG_ERRNO("std::vector allocation failed", -ret);
		mAacAsc.clear();
		return ret;
	}
	return 0;
}


void AudioMedia::fillMediaInfo(struct pdraw_media_info *minfo)
{
	size_t cplen;

	if (!minfo)
		return;

	*minfo = {};

	minfo->type = PDRAW_MEDIA_TYPE_AUDIO;
	minfo->id = mId;
	minfo->name = strdup(getName().c_str());
	minfo->path = strdup(getPath().c_str());
	minfo->playback_type = mPlaybackType;
	minfo->duration = mDuration;
	minfo->audio.format = format;
	if (format.encoding == ADEF_ENCODING_AAC_LC) {
		if (sizeof(minfo->audio.aac_lc.asc) < mAacAsc.size())
			ULOGW("%s: truncated ASC", __func__);
		cplen = std::min(mAacAsc.size(),
				 sizeof(minfo->audio.aac_lc.asc));
		memcpy(minfo->audio.aac_lc.asc, mAacAsc.data(), cplen);
		minfo->audio.aac_lc.asclen = cplen;
	}
}

} /* namespace Pdraw */
