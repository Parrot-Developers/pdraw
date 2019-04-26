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

#include "pdraw_media.hpp"

#include <h264/h264.h>
#define ULOG_TAG pdraw_media
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_media);

namespace Pdraw {


pthread_mutex_t Media::mMutex = PTHREAD_MUTEX_INITIALIZER;
unsigned int Media::mIdCounter = 0;


#define UTILS_H264_EXTENDED_SAR 255

static const unsigned int pdraw_h264Sar[17][2] = {
	{1, 1},
	{1, 1},
	{12, 11},
	{10, 11},
	{16, 11},
	{40, 33},
	{24, 11},
	{20, 11},
	{32, 11},
	{80, 33},
	{18, 11},
	{15, 11},
	{64, 33},
	{160, 99},
	{4, 3},
	{3, 2},
	{2, 1},
};


Media::Media(Session *session, Type t)
{
	mSession = session;
	type = t;

	pthread_mutex_lock(&mMutex);
	id = ++mIdCounter;
	pthread_mutex_unlock(&mMutex);
}


const char *Media::getMediaTypeStr(Type val)
{
	switch (val) {
	case UNKNOWN:
		return "UNKNOWN";
	case VIDEO:
		return "VIDEO";
	case AUDIO:
		return "AUDIO";
	default:
		return NULL;
	}
}


VideoMedia::VideoMedia(Session *session) : Media(session, VIDEO)
{
	mSps = NULL;
	mSpsSize = 0;
	mPps = NULL;
	mPpsSize = 0;

	format = FORMAT_UNKNOWN;
	subFormat = SUBFORMAT_UNKNOWN;
	width = 0;
	height = 0;
	cropLeft = 0;
	cropWidth = 0;
	cropTop = 0;
	cropHeight = 0;
	sarWidth = 1;
	sarHeight = 1;
	hfov = 0.;
	vfov = 0.;
	fullRange = false;
}


VideoMedia::~VideoMedia(void)
{
	free(mSps);
	free(mPps);
}


int VideoMedia::getSpsPps(uint8_t **sps,
			  size_t *spsSize,
			  uint8_t **pps,
			  size_t *ppsSize)
{
	if (format != H264)
		return -EPROTO;
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


int VideoMedia::setSpsPps(const uint8_t *sps,
			  size_t spsSize,
			  const uint8_t *pps,
			  size_t ppsSize)
{
	int ret;
	struct h264_sps *_sps = NULL;

	if (format != H264)
		return -EPROTO;
	if ((sps == NULL) || (spsSize == 0))
		return -EINVAL;
	if ((pps == NULL) || (ppsSize == 0))
		return -EINVAL;

	free(mSps);
	mSpsSize = 0;
	mSps = (uint8_t *)malloc(spsSize);
	if (mSps == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	mSpsSize = spsSize;
	memcpy(mSps, sps, mSpsSize);

	free(mPps);
	mPpsSize = 0;
	mPps = (uint8_t *)malloc(ppsSize);
	if (mPps == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	mPpsSize = ppsSize;
	memcpy(mPps, pps, mPpsSize);

	_sps = (h264_sps *)calloc(1, sizeof(*_sps));
	if (_sps == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		ret = -ENOMEM;
		goto exit;
	}

	ret = h264_parse_sps(sps, spsSize, _sps);
	if (ret < 0) {
		ULOG_ERRNO("h264_parse_sps", -ret);
		goto exit;
	}

	struct h264_sps_derived sps_derived;
	ret = h264_get_sps_derived(_sps, &sps_derived);
	if (ret < 0) {
		ULOG_ERRNO("h264_get_sps_derived", -ret);
		goto exit;
	}

	width = sps_derived.PicWidthInSamplesLuma;
	height = sps_derived.FrameHeightInMbs * 16;
	cropLeft = 0;
	cropWidth = width;
	cropTop = 0;
	cropHeight = height;
	if (_sps->frame_cropping_flag) {
		cropLeft = _sps->frame_crop_left_offset * sps_derived.CropUnitX;
		cropWidth = width - _sps->frame_crop_right_offset *
					    sps_derived.CropUnitX;
		cropTop = _sps->frame_crop_top_offset * sps_derived.CropUnitY;
		cropHeight = height - _sps->frame_crop_bottom_offset *
					      sps_derived.CropUnitY;
	}

	if (_sps->vui_parameters_present_flag)
		fullRange = _sps->vui.video_full_range_flag;

	sarWidth = 1;
	sarHeight = 1;
	if (_sps->vui.aspect_ratio_info_present_flag) {
		if (_sps->vui.aspect_ratio_idc == UTILS_H264_EXTENDED_SAR) {
			sarWidth = _sps->vui.sar_width;
			sarHeight = _sps->vui.sar_height;
		} else if (_sps->vui.aspect_ratio_idc <= 16) {
			sarWidth = pdraw_h264Sar[_sps->vui.aspect_ratio_idc][0];
			sarHeight =
				pdraw_h264Sar[_sps->vui.aspect_ratio_idc][1];
		}
	}

	ret = 0;

exit:
	free(_sps);
	if (ret < 0) {
		free(mSps);
		free(mPps);
		mSps = NULL;
		mPps = NULL;
		mSpsSize = 0;
		mPpsSize = 0;
	}

	return ret;
}


const char *VideoMedia::getVideoFormatStr(Format val)
{
	switch (val) {
	case FORMAT_UNKNOWN:
		return "UNKNOWN";
	case YUV:
		return "YUV";
	case H264:
		return "H264";
	case OPAQUE:
		return "OPAQUE";
	default:
		return NULL;
	}
}


const char *VideoMedia::getVideoSubFormatStr(SubFormat val)
{
	switch (val) {
	case SubFormat::SUBFORMAT_UNKNOWN:
		return "UNKNOWN";
	case SubFormat::YUV_I420:
		return "I420";
	case SubFormat::YUV_NV12:
		return "NV12";
	case SubFormat::OPAQUE_MMAL:
		return "MMAL";
	case SubFormat::H264_BYTE_STREAM:
		return "BYTE_STREAM";
	case SubFormat::H264_AVCC:
		return "AVCC";
	default:
		return NULL;
	}
}

} /* namespace Pdraw */
