/**
 * Parrot Drones Awesome Video Viewer Library
 * Video media
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

#include "pdraw_media_video.hpp"
#include "pdraw_avcdecoder.hpp"
#include <math.h>
#include <string.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <vector>

namespace Pdraw {


VideoMedia::VideoMedia(
	Session *session,
	enum elementary_stream_type esType,
	unsigned int id)
{
	mSession = session;
	mEsType = esType;
	mId = id;
	mVideoType = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA;
	mWidth = mHeight = 0;
	mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
	mSarWidth = mSarHeight = 0;
	mHfov = mVfov = 0.;
	mDemux = NULL;
	mDemuxEsIndex = -1;
	mDecoder = NULL;
}


VideoMedia::VideoMedia(
	Session *session,
	enum elementary_stream_type esType,
	unsigned int id,
	Demuxer *demux,
	int demuxEsIndex)
{
	mSession = session;
	mEsType = esType;
	mId = id;
	mVideoType = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA;
	mWidth = mHeight = 0;
	mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
	mSarWidth = mSarHeight = 0;
	mHfov = mVfov = 0.;
	mDemux = demux;
	mDemuxEsIndex = demuxEsIndex;
	mDecoder = NULL;
}


VideoMedia::~VideoMedia(
	void)
{
	std::vector<VideoFrameFilter *>::iterator p =
		mVideoFrameFilters.begin();
	while (p != mVideoFrameFilters.end()) {
		delete *p;
		p++;
	}

	if (mDecoder)
		disableDecoder();
}


void VideoMedia::getDimensions(
	unsigned int *width,
	unsigned int *height,
	unsigned int *cropLeft,
	unsigned int *cropRight,
	unsigned int *cropTop,
	unsigned int *cropBottom,
	unsigned int *croppedWidth,
	unsigned int *croppedHeight,
	unsigned int *sarWidth,
	unsigned int *sarHeight)
{
	if (width)
		*width = mWidth;
	if (height)
		*height = mHeight;
	if (cropLeft)
		*cropLeft = mCropLeft;
	if (cropRight)
		*cropRight = mCropRight;
	if (cropTop)
		*cropTop = mCropTop;
	if (cropBottom)
		*cropBottom = mCropBottom;
	if (croppedWidth)
		*croppedWidth = mWidth - mCropLeft - mCropRight;
	if (croppedHeight)
		*croppedHeight = mHeight - mCropTop - mCropBottom;
	if (sarWidth)
		*sarWidth = mSarWidth;
	if (sarHeight)
		*sarHeight = mSarHeight;
}


void VideoMedia::setDimensions(
	unsigned int width,
	unsigned int height,
	unsigned int cropLeft,
	unsigned int cropRight,
	unsigned int cropTop,
	unsigned int cropBottom,
	unsigned int sarWidth,
	unsigned int sarHeight)
{
	mWidth = width;
	mHeight = height;
	mCropLeft = cropLeft;
	mCropRight = cropRight;
	mCropTop = cropTop;
	mCropBottom = cropBottom;
	mSarWidth = sarWidth;
	mSarHeight = sarHeight;
}


void VideoMedia::getFov(
	float *hfov,
	float *vfov)
{
	if (hfov)
		*hfov = mHfov;
	if (vfov)
		*vfov = mVfov;
}


void VideoMedia::setFov(
	float hfov,
	float vfov)
{
	mHfov = hfov;
	mVfov = vfov;
}


int VideoMedia::enableDecoder(
	void)
{
	if (mDecoder != NULL) {
		ULOGE("VideoMedia: decoder is already enabled");
		return -1;
	}

	mDecoder = AvcDecoder::create(this);
	if (mDecoder == NULL) {
		ULOGE("VideoMedia: failed to create AVC decoder");
		return -1;
	}

	if ((mDemuxEsIndex != -1) && (mDemux != NULL)) {
		int ret = mDemux->setElementaryStreamDecoder(
			mDemuxEsIndex, mDecoder);
		if (ret != 0) {
			ULOGE("VideoMedia: setElementaryStreamDecoder() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


int VideoMedia::disableDecoder(
	void)
{
	if (mDecoder == NULL) {
		ULOGE("VideoMedia: decoder is not enabled");
		return -1;
	}

	int ret = ((AvcDecoder*)mDecoder)->stop();
	if (ret != 0) {
		ULOGE("VideoMedia: failed to stop AVC decoder (%d)", ret);
		return -1;
	}

	delete mDecoder;
	mDecoder = NULL;

	return 0;
}


VideoFrameFilter *VideoMedia::addVideoFrameFilter(
	bool frameByFrame)
{
	if (mDecoder == NULL) {
		ULOGE("VideoMedia: decoder is not enabled");
		return NULL;
	}

	VideoFrameFilter *p = new VideoFrameFilter(
		this, (AvcDecoder*)mDecoder, frameByFrame);
	if (p == NULL) {
		ULOGE("VideoMedia: video frame filter allocation failed");
		return NULL;
	}

	mVideoFrameFilters.push_back(p);
	return p;
}


VideoFrameFilter *VideoMedia::addVideoFrameFilter(
	pdraw_video_frame_filter_callback_t cb,
	void *userPtr,
	bool frameByFrame)
{
	if (mDecoder == NULL) {
		ULOGE("VideoMedia: decoder is not enabled");
		return NULL;
	}
	if (cb == NULL) {
		ULOGE("VideoMedia: invalid callback function");
		return NULL;
	}

	VideoFrameFilter *p = new VideoFrameFilter(
		this, (AvcDecoder*)mDecoder, cb, userPtr, frameByFrame);
	if (p == NULL) {
		ULOGE("VideoMedia: video frame filter allocation failed");
		return NULL;
	}

	mVideoFrameFilters.push_back(p);
	return p;
}


int VideoMedia::removeVideoFrameFilter(
	VideoFrameFilter *filter)
{
	if (filter == NULL) {
		ULOGE("VideoMedia: invalid video frame filter pointer");
		return -1;
	}

	bool found = false;
	std::vector<VideoFrameFilter*>::iterator p = mVideoFrameFilters.begin();

	while (p != mVideoFrameFilters.end()) {
		if (*p == filter) {
			mVideoFrameFilters.erase(p);
			delete *p;
			found = true;
			break;
		}
		p++;
	}

	return (found) ? 0 : -1;
}


bool VideoMedia::isVideoFrameFilterValid(
	VideoFrameFilter *filter)
{
	if (filter == NULL) {
		ULOGE("VideoMedia: invalid video frame filter pointer");
		return false;
	}

	bool found = false;
	std::vector<VideoFrameFilter*>::iterator p = mVideoFrameFilters.begin();

	while (p != mVideoFrameFilters.end()) {
		if (*p == filter) {
			found = true;
			break;
		}
		p++;
	}

	return found;
}

} /* namespace Pdraw */
