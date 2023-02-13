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

#define ULOG_TAG pdraw_media
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_media.hpp"

#include <algorithm>

#include <h264/h264.h>
#include <h265/h265.h>

namespace Pdraw {


std::atomic<unsigned int> Media::mIdCounter(0);


Media::Media(Session *session, Type t) :
		type(t), sessionMeta({}),
		playbackType(PDRAW_PLAYBACK_TYPE_UNKNOWN), duration(0),
		mSession(session)

{
	id = ++mIdCounter;
	mName = std::string(__func__) + "#" + std::to_string(id);
}


std::string &Media::getName(void)
{
	return mName;
}


void Media::setClassName(std::string &name)
{
	mName = name + "#" + std::to_string(id);
}


void Media::setClassName(const char *name)
{
	mName = std::string(name) + "#" + std::to_string(id);
}


std::string &Media::getPath(void)
{
	return mPath;
}


void Media::setPath(std::string &path)
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
	case UNKNOWN:
		return "UNKNOWN";
	case RAW_VIDEO:
		return "RAW_VIDEO";
	case CODED_VIDEO:
		return "CODED_VIDEO";
	case RAW_AUDIO:
		return "RAW_AUDIO";
	case CODED_AUDIO:
		return "CODED_AUDIO";
	default:
		return nullptr;
	}
}


void Media::cleanupMediaInfo(struct pdraw_media_info *minfo)
{
	free((void *)minfo->name);
	minfo->name = nullptr;
	free((void *)minfo->path);
	minfo->path = nullptr;
}


RawVideoMedia::RawVideoMedia(Session *session) : Media(session, RAW_VIDEO)
{
	Media::setClassName(__func__);
	format = {};
	info = {};
}


RawVideoMedia::~RawVideoMedia(void)
{
	return;
}


void RawVideoMedia::fillMediaInfo(struct pdraw_media_info *minfo)
{
	if (!minfo)
		return;

	*minfo = {};

	minfo->type = PDRAW_MEDIA_TYPE_VIDEO;
	minfo->id = id;
	minfo->name = strdup(getName().c_str());
	minfo->path = strdup(getPath().c_str());
	minfo->playback_type = playbackType;
	minfo->duration = duration;
	minfo->session_meta = &sessionMeta;
	minfo->video.format = VDEF_FRAME_TYPE_RAW;
	minfo->video.type = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA;
	minfo->video.raw.format = format;
	minfo->video.raw.info = info;
}


CodedVideoMedia::CodedVideoMedia(Session *session) : Media(session, CODED_VIDEO)
{
	Media::setClassName(__func__);
	mVps = nullptr;
	mVpsSize = 0;
	mSps = nullptr;
	mSpsSize = 0;
	mPps = nullptr;
	mPpsSize = 0;

	format = {};
	info = {};
}


CodedVideoMedia::~CodedVideoMedia(void)
{
	free(mVps);
	free(mSps);
	free(mPps);
}


int CodedVideoMedia::getPs(const uint8_t **vps,
			   size_t *vpsSize,
			   const uint8_t **sps,
			   size_t *spsSize,
			   const uint8_t **pps,
			   size_t *ppsSize)
{
	enum vdef_encoding encoding = format.encoding;
	if ((encoding != VDEF_ENCODING_H264) &&
	    (encoding != VDEF_ENCODING_H265))
		return -EPROTO;

	if (encoding == VDEF_ENCODING_H265) {
		/* VPS is for H.265 only */
		if (vps)
			*vps = mVps;
		if (vpsSize)
			*vpsSize = mVpsSize;
	}
	if (sps)
		*sps = mSps;
	if (spsSize)
		*spsSize = mSpsSize;
	if (pps)
		*pps = mPps;
	if (ppsSize)
		*ppsSize = mPpsSize;

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

	free(mVps);
	mVps = nullptr;
	mVpsSize = 0;
	if (encoding == VDEF_ENCODING_H265) {
		mVps = (uint8_t *)malloc(vpsSize);
		if (mVps == nullptr) {
			ret = -ENOMEM;
			ULOG_ERRNO("malloc", -ret);
			goto error;
		}
		mVpsSize = vpsSize;
		memcpy(mVps, vps, mVpsSize);
	}

	free(mSps);
	mSpsSize = 0;
	mSps = (uint8_t *)malloc(spsSize);
	if (mSps == nullptr) {
		ret = -ENOMEM;
		ULOG_ERRNO("malloc", -ret);
		goto error;
	}
	mSpsSize = spsSize;
	memcpy(mSps, sps, mSpsSize);

	free(mPps);
	mPpsSize = 0;
	mPps = (uint8_t *)malloc(ppsSize);
	if (mPps == nullptr) {
		ret = -ENOMEM;
		ULOG_ERRNO("malloc", -ret);
		goto error;
	}
	mPpsSize = ppsSize;
	memcpy(mPps, pps, mPpsSize);

	switch (encoding) {
	case VDEF_ENCODING_H264:
		ret = h264_get_info(sps, spsSize, pps, ppsSize, &h264Info);
		if (ret < 0) {
			ULOG_ERRNO("h264_get_info", -ret);
			goto error;
		}

		info.bit_depth = h264Info.bit_depth_luma;
		info.full_range = h264Info.full_range;
		info.color_primaries = vdef_color_primaries_from_h264(
			h264Info.colour_primaries);
		info.transfer_function = vdef_transfer_function_from_h264(
			h264Info.transfer_characteristics);
		info.matrix_coefs = vdef_matrix_coefs_from_h264(
			h264Info.matrix_coefficients);
		info.resolution.width = h264Info.crop_width;
		info.resolution.height = h264Info.crop_height;
		info.sar.width = h264Info.sar_width;
		info.sar.height = h264Info.sar_height;
		info.framerate.num = h264Info.framerate_num;
		info.framerate.den = h264Info.framerate_den;
		break;

	case VDEF_ENCODING_H265:
		ret = h265_get_info(
			vps, vpsSize, sps, spsSize, pps, ppsSize, &h265Info);
		if (ret < 0) {
			ULOG_ERRNO("h265_get_info", -ret);
			goto error;
		}

		info.bit_depth = h265Info.bit_depth_luma;
		info.full_range = h265Info.full_range;
		info.color_primaries = vdef_color_primaries_from_h265(
			h265Info.colour_primaries);
		info.transfer_function = vdef_transfer_function_from_h265(
			h265Info.transfer_characteristics);
		info.matrix_coefs = vdef_matrix_coefs_from_h265(
			h265Info.matrix_coefficients);
		info.resolution.width = h265Info.crop_width;
		info.resolution.height = h265Info.crop_height;
		info.sar.width = h265Info.sar_width;
		info.sar.height = h265Info.sar_height;
		info.framerate.num = h265Info.framerate_num;
		info.framerate.den = h265Info.framerate_den;
		break;
	default:
		break;
	}

	return 0;

error:
	free(mVps);
	free(mSps);
	free(mPps);
	mVps = nullptr;
	mSps = nullptr;
	mPps = nullptr;
	mVpsSize = 0;
	mSpsSize = 0;
	mPpsSize = 0;
	return ret;
}


void CodedVideoMedia::fillMediaInfo(struct pdraw_media_info *minfo)
{
	size_t cplen;

	if (!minfo)
		return;

	*minfo = {};

	minfo->type = PDRAW_MEDIA_TYPE_VIDEO;
	minfo->id = id;
	minfo->name = strdup(getName().c_str());
	minfo->path = strdup(getPath().c_str());
	minfo->playback_type = playbackType;
	minfo->duration = duration;
	minfo->session_meta = &sessionMeta;
	minfo->video.format = VDEF_FRAME_TYPE_CODED;
	minfo->video.type = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA;
	minfo->video.coded.format = format;
	minfo->video.coded.info = info;
	switch (format.encoding) {
	case VDEF_ENCODING_H264:
		if (sizeof(minfo->video.coded.h264.sps) < mSpsSize)
			ULOGW("%s: truncated SPS", __func__);
		cplen = std::min(mSpsSize, sizeof(minfo->video.coded.h264.sps));
		memcpy(minfo->video.coded.h264.sps, mSps, cplen);
		minfo->video.coded.h264.spslen = cplen;
		if (sizeof(minfo->video.coded.h264.pps) < mPpsSize)
			ULOGW("%s: truncated PPS", __func__);
		cplen = std::min(mPpsSize, sizeof(minfo->video.coded.h264.pps));
		memcpy(minfo->video.coded.h264.pps, mPps, cplen);
		minfo->video.coded.h264.ppslen = cplen;
		break;
	case VDEF_ENCODING_H265:
		if (sizeof(minfo->video.coded.h265.vps) < mVpsSize)
			ULOGW("%s: truncated VPS", __func__);
		cplen = std::min(mVpsSize, sizeof(minfo->video.coded.h265.vps));
		memcpy(minfo->video.coded.h265.vps, mVps, cplen);
		minfo->video.coded.h265.vpslen = cplen;
		if (sizeof(minfo->video.coded.h265.sps) < mSpsSize)
			ULOGW("%s: truncated SPS", __func__);
		cplen = std::min(mSpsSize, sizeof(minfo->video.coded.h265.sps));
		memcpy(minfo->video.coded.h265.sps, mSps, cplen);
		minfo->video.coded.h265.spslen = cplen;
		if (sizeof(minfo->video.coded.h265.pps) < mPpsSize)
			ULOGW("%s: truncated PPS", __func__);
		cplen = std::min(mPpsSize, sizeof(minfo->video.coded.h265.pps));
		memcpy(minfo->video.coded.h265.pps, mPps, cplen);
		minfo->video.coded.h265.ppslen = cplen;
		break;
	default:
		break;
	}
}

} /* namespace Pdraw */
