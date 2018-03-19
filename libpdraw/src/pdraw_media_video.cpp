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
#define ULOG_TAG pdraw_mediavideo
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_mediavideo);
#include <vector>

namespace Pdraw {


VideoMedia::VideoMedia(
	Session *session,
	enum elementary_stream_type esType,
	unsigned int id) :
	VideoMedia(session, esType, id, NULL, -1)
{
}


VideoMedia::VideoMedia(
	Session *session,
	enum elementary_stream_type esType,
	unsigned int id,
	Demuxer *demux,
	int demuxEsIndex)
{
	int res;
	pthread_mutexattr_t attr;
	bool mutex_created = false, attr_created = false;

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

	res = pthread_mutexattr_init(&attr);
	if (res < 0) {
		ULOG_ERRNO("pthread_mutexattr_init", -res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res < 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", -res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res < 0) {
		ULOG_ERRNO("pthread_mutex_init", -res);
		goto error;
	}
	mutex_created = true;

	pthread_mutexattr_destroy(&attr);
	return;

error:
	if (mutex_created)
		pthread_mutex_destroy(&mMutex);
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
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

	pthread_mutex_destroy(&mMutex);
}


void VideoMedia::lock(
	void)
{
	pthread_mutex_lock(&mMutex);
}


void VideoMedia::unlock(
	void)
{
	pthread_mutex_unlock(&mMutex);
}


unsigned int VideoMedia::getId(
	void) {
	pthread_mutex_lock(&mMutex);
	unsigned int ret = mId;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


enum pdraw_video_type VideoMedia::getVideoType(
	void) {
	pthread_mutex_lock(&mMutex);
	enum pdraw_video_type ret = mVideoType;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void VideoMedia::setVideoType(
	enum pdraw_video_type type) {
	pthread_mutex_lock(&mMutex);
	mVideoType = type;
	pthread_mutex_unlock(&mMutex);
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
	pthread_mutex_lock(&mMutex);
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
	pthread_mutex_unlock(&mMutex);
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
	pthread_mutex_lock(&mMutex);
	mWidth = width;
	mHeight = height;
	mCropLeft = cropLeft;
	mCropRight = cropRight;
	mCropTop = cropTop;
	mCropBottom = cropBottom;
	mSarWidth = sarWidth;
	mSarHeight = sarHeight;
	pthread_mutex_unlock(&mMutex);
}


void VideoMedia::getFov(
	float *hfov,
	float *vfov)
{
	pthread_mutex_lock(&mMutex);
	if (hfov)
		*hfov = mHfov;
	if (vfov)
		*vfov = mVfov;
	pthread_mutex_unlock(&mMutex);
}


void VideoMedia::setFov(
	float hfov,
	float vfov)
{
	pthread_mutex_lock(&mMutex);
	mHfov = hfov;
	mVfov = vfov;
	pthread_mutex_unlock(&mMutex);
}


int VideoMedia::enableDecoder(
	void)
{
	pthread_mutex_lock(&mMutex);

	if (mDecoder != NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("decoder is already enabled");
		return -EPROTO;
	}

	mDecoder = new AvcDecoder(this);
	if (mDecoder == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("failed to create AVC decoder");
		return -ENOMEM;
	}

	if ((mDemuxEsIndex != -1) && (mDemux != NULL)) {
		int ret = mDemux->setElementaryStreamDecoder(
			mDemuxEsIndex, mDecoder);
		if (ret < 0) {
			pthread_mutex_unlock(&mMutex);
			ULOG_ERRNO("demuxer->setElementaryStreamDecoder", -ret);
			return ret;
		}
	}

	pthread_mutex_unlock(&mMutex);
	return 0;
}


int VideoMedia::disableDecoder(
	void)
{
	pthread_mutex_lock(&mMutex);

	if (mDecoder == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("decoder is not enabled");
		return -EPROTO;
	}

	int ret = ((AvcDecoder*)mDecoder)->close();
	if (ret < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("decoder->close", -ret);
		return ret;
	}

	delete mDecoder;
	mDecoder = NULL;

	pthread_mutex_unlock(&mMutex);
	return 0;
}


Session *VideoMedia::getSession(
	void) {
	pthread_mutex_lock(&mMutex);
	Session *ret = mSession;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Decoder *VideoMedia::getDecoder(
	void) {
	pthread_mutex_lock(&mMutex);
	Decoder *ret = mDecoder;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


VideoFrameFilter *VideoMedia::addVideoFrameFilter(
	bool frameByFrame)
{
	pthread_mutex_lock(&mMutex);

	if (mDecoder == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("decoder is not enabled", EPROTO);
		return NULL;
	}

	VideoFrameFilter *p = new VideoFrameFilter(
		this, (AvcDecoder*)mDecoder, frameByFrame);
	if (p == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("video frame filter creation failed", ENOMEM);
		return NULL;
	}

	mVideoFrameFilters.push_back(p);
	pthread_mutex_unlock(&mMutex);
	return p;
}


VideoFrameFilter *VideoMedia::addVideoFrameFilter(
	pdraw_video_frame_filter_callback_t cb,
	void *userPtr,
	bool frameByFrame)
{
	pthread_mutex_lock(&mMutex);

	if (mDecoder == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("decoder is not enabled", EPROTO);
		return NULL;
	}
	if (cb == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("invalid callback function", EINVAL);
		return NULL;
	}

	VideoFrameFilter *p = new VideoFrameFilter(
		this, (AvcDecoder*)mDecoder, cb, userPtr, frameByFrame);
	if (p == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("video frame filter creation failed", ENOMEM);
		return NULL;
	}

	mVideoFrameFilters.push_back(p);
	pthread_mutex_unlock(&mMutex);
	return p;
}


int VideoMedia::removeVideoFrameFilter(
	VideoFrameFilter *filter)
{
	pthread_mutex_lock(&mMutex);

	if (filter == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("invalid video frame filter pointer");
		return -EINVAL;
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

	pthread_mutex_unlock(&mMutex);
	return (found) ? 0 : -ENOENT;
}


bool VideoMedia::isVideoFrameFilterValid(
	VideoFrameFilter *filter)
{
	pthread_mutex_lock(&mMutex);

	if (filter == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("invalid video frame filter pointer", EINVAL);
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

	pthread_mutex_unlock(&mMutex);
	return found;
}

} /* namespace Pdraw */
