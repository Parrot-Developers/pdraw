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
	struct h264_info h264Info;

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
		goto error;
	}
	mSpsSize = spsSize;
	memcpy(mSps, sps, mSpsSize);

	free(mPps);
	mPpsSize = 0;
	mPps = (uint8_t *)malloc(ppsSize);
	if (mPps == NULL) {
		ret = -ENOMEM;
		goto error;
	}
	mPpsSize = ppsSize;
	memcpy(mPps, pps, mPpsSize);

	ret = h264_get_info(sps, spsSize, pps, ppsSize, &h264Info);
	if (ret < 0) {
		ULOG_ERRNO("h264_get_info", -ret);
		goto error;
	}

	width = h264Info.width;
	height = h264Info.height;
	cropLeft = h264Info.crop_left;
	cropWidth = h264Info.crop_width;
	cropTop = h264Info.crop_top;
	cropHeight = h264Info.crop_height;
	sarWidth = h264Info.sar_width;
	sarHeight = h264Info.sar_height;
	fullRange = h264Info.full_range;

	return 0;

error:
	free(mSps);
	free(mPps);
	mSps = NULL;
	mPps = NULL;
	mSpsSize = 0;
	mPpsSize = 0;
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
