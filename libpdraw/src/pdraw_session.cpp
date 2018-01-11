/**
 * Parrot Drones Awesome Video Viewer Library
 * Session
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

#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_demuxer_stream.hpp"
#include "pdraw_demuxer_record.hpp"
#include <math.h>
#include <string.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <algorithm>
#include <string>
#include <vector>

namespace Pdraw {


Session::Session(
	Settings *settings)
{
	int ret;

	mSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
	mSettings = settings;
	mDemuxer = NULL;
	mRenderer = NULL;
	mMediaIdCounter = 0;
	mThreadShouldStop = false;
	mLoop = NULL;
	mLoopThreadLaunched = false;

	mLoop = pomp_loop_new();
	if (mLoop == NULL) {
		ULOGE("Session: pomp_loop_new() failed");
		goto err;
	}

	ret = pthread_create(&mLoopThread, NULL, runLoopThread, (void *)this);
	if (ret != 0) {
		ULOGE("Session: loop thread creation failed (%d)", ret);
		goto err;
	}

	mLoopThreadLaunched = true;

	return;

err:
	if (mLoop != NULL) {
		pomp_loop_destroy(mLoop);
		mLoop = NULL;
	}
}


Session::~Session(
	void)
{
	int ret;

	if (mDemuxer != NULL) {
		int ret = mDemuxer->stop();
		if (ret != 0)
			ULOGE("Session: failed to stop demuxer");
		else
			delete mDemuxer;
	}

	if (mRenderer != NULL)
		delete mRenderer;

	std::vector<Media*>::iterator m = mMedias.begin();
	while (m != mMedias.end()) {
		delete *m;
		m++;
	}

	mThreadShouldStop = true;
	if (mLoop)
		pomp_loop_wakeup(mLoop);

	if (mLoopThreadLaunched) {
		ret = pthread_join(mLoopThread, NULL);
		if (ret != 0)
			ULOGE("Session: pthread_join() failed (%d)", ret);
	}

	if (mLoop) {
		ret = pomp_loop_destroy(mLoop);
		if (ret != 0)
			ULOGE("Session: pomp_loop_destroy() failed");
	}
}


int Session::open(
	const std::string &url)
{
	return open(url, "");
}


int Session::open(
	const std::string &url,
	const std::string &ifaceAddr)
{
	int ret = 0;

	std::string ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
	if ((url.front() == '/') && (ext == ".mp4")) {
		mSessionType = PDRAW_SESSION_TYPE_RECORD;
		mDemuxer = new RecordDemuxer(this);
		if (mDemuxer == NULL) {
			ULOGE("Session: failed to alloc demuxer");
			ret = -1;
		} else {
			ret = mDemuxer->configure(url);
			if (ret != 0) {
				ULOGE("Session: failed to configure demuxer");
				delete mDemuxer;
				mDemuxer = NULL;
				ret = -1;
			}
		}
	} else if (((url.front() == '/') && (ext == ".sdp")) ||
		((url.substr(0, 7) == "http://") && (ext == ".sdp")) ||
		(url.substr(0, 7) == "rtsp://")) {
		mSessionType = PDRAW_SESSION_TYPE_STREAM;
		mDemuxer = new StreamDemuxer(this);
		if (mDemuxer == NULL) {
			ULOGE("Session: failed to alloc demuxer");
			ret = -1;
		} else {
			ret = ((StreamDemuxer*)mDemuxer)->configure(
				url, ifaceAddr);
			if (ret != 0) {
				ULOGE("Session: failed to configure demuxer");
				delete mDemuxer;
				mDemuxer = NULL;
				ret = -1;
			}
		}
	} else {
		ULOGE("Session: unsupported URL");
		ret = -1;
	}

	return (ret == 0) ? addMediaFromDemuxer() : ret;
}


int Session::open(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	int ret = 0;

	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	mDemuxer = new StreamDemuxer(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		return -1;
	}

	ret = ((StreamDemuxer*)mDemuxer)->configure(localAddr,
		localStreamPort, localControlPort, remoteAddr,
		remoteStreamPort, remoteControlPort, ifaceAddr);
	if (ret != 0) {
		ULOGE("Session: failed to configure demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		return -1;
	}

	return addMediaFromDemuxer();
}


int Session::openSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	int ret = 0;

	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	mDemuxer = new StreamDemuxer(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		return -1;
	}

	ret = ((StreamDemuxer*)mDemuxer)->configureWithSdp(sdp, ifaceAddr);
	if (ret != 0) {
		ULOGE("Session: failed to configure demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		return -1;
	}

	return addMediaFromDemuxer();
}


int Session::addMediaFromDemuxer(
	void)
{
	int esCount = 0;

	esCount = mDemuxer->getElementaryStreamCount();
	if (esCount < 0) {
		ULOGE("Session: getElementaryStreamCount() failed (%d)",
			esCount);
		return -1;
	}

	int i;
	for (i = 0; i < esCount; i++) {
		enum elementary_stream_type esType =
			mDemuxer->getElementaryStreamType(i);
		if (esType < 0) {
			ULOGE("Session: getElementaryStreamType() failed (%d)",
				esType);
			continue;
		}

		Media *m = addMedia(esType, mDemuxer, i);
		if (!m)
			ULOGE("Session: media creation failed");
	}

	return 0;
}


Media *Session::addMedia(
	enum elementary_stream_type esType)
{
	Media *m = NULL;
	switch (esType) {
	case ELEMENTARY_STREAM_TYPE_UNKNOWN:
	default:
		break;
	case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
		m = new VideoMedia(this, esType, mMediaIdCounter++);
		m->enableDecoder();
		break;
	}

	if (m == NULL) {
		ULOGE("Session: media creation failed");
		return NULL;
	}

	mMedias.push_back(m);

	return m;
}


Media *Session::addMedia(
	enum elementary_stream_type esType,
	Demuxer *demuxer,
	int demuxEsIndex)
{
	Media *m = NULL;
	switch (esType) {
	case ELEMENTARY_STREAM_TYPE_UNKNOWN:
	default:
		break;
	case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
		m = new VideoMedia(this, esType, mMediaIdCounter++,
			demuxer, demuxEsIndex);
		if (demuxer != NULL) {
			unsigned int width, height, sarWidth, sarHeight;
			unsigned int cropLeft, cropRight, cropTop, cropBottom;
			float hfov, vfov;
			demuxer->getElementaryStreamVideoDimensions(
				demuxEsIndex, &width, &height,
				&cropLeft, &cropRight, &cropTop, &cropBottom,
				&sarWidth, &sarHeight);
			((VideoMedia*)m)->setDimensions(width, height,
				cropLeft, cropRight, cropTop, cropBottom,
				sarWidth, sarHeight);
			demuxer->getElementaryStreamVideoFov(
				demuxEsIndex, &hfov, &vfov);
			((VideoMedia*)m)->setFov(hfov, vfov);
		}
		m->enableDecoder();
		break;
	}

	if (m == NULL) {
		ULOGE("Session: media creation failed");
		return NULL;
	}

	mMedias.push_back(m);

	return m;
}


int Session::removeMedia(
	Media *media)
{
	bool found = false;
	std::vector<Media*>::iterator m = mMedias.begin();

	while (m != mMedias.end()) {
		if (*m == media) {
			mMedias.erase(m);
			delete *m;
			found = true;
			break;
		}
		m++;
	}

	return (found) ? 0 : -1;
}


int Session::removeMedia(
	unsigned int index)
{
	if (index >= mMedias.size())
		return -1;

	Media *m = mMedias.at(index);
	mMedias.erase(mMedias.begin() + index);
	delete m;

	return 0;
}


unsigned int Session::getMediaCount(
	void)
{
	return mMedias.size();
}


Media *Session::getMedia(
	unsigned int index)
{
	if (index >= mMedias.size())
		return NULL;

	return mMedias.at(index);
}


Media *Session::getMediaById(
	unsigned int id)
{
	std::vector<Media*>::iterator m = mMedias.begin();

	while (m != mMedias.end()) {
		if ((*m)->getId() == id)
			return *m;
		m++;
	}

	ULOGE("Session: unable to find media by id");
	return NULL;
}


int Session::enableRenderer(
	void)
{
	int ret = 0;

	if (mRenderer != NULL) {
		ULOGE("Session: renderer is already enabled");
		return -1;
	}

	mRenderer = Renderer::create(this);
	if (mRenderer == NULL) {
		ULOGE("Session: failed to alloc renderer");
		return -1;
	}

	std::vector<Media*>::iterator m;

	for (m = mMedias.begin(); m < mMedias.end(); m++) {
		if ((*m)->getType() == PDRAW_MEDIA_TYPE_VIDEO) {
			ret = mRenderer->addAvcDecoder(
				(AvcDecoder*)((*m)->getDecoder()));
			if (ret != 0)
				ULOGE("Session: failed add decoder to renderer");
		}
	}

	return ret;
}


int Session::disableRenderer(
	void)
{
	int ret = 0;

	if (mRenderer == NULL) {
		ULOGE("Session: renderer is not enabled");
		return -1;
	}

	std::vector<Media*>::iterator m;

	for (m = mMedias.begin(); m < mMedias.end(); m++) {
		if ((*m)->getType() == PDRAW_MEDIA_TYPE_VIDEO) {
			ret = mRenderer->removeAvcDecoder(
				(AvcDecoder*)((*m)->getDecoder()));
			if (ret != 0)
				ULOGE("Session: failed remove decoder from renderer");
		}
	}

	delete mRenderer;
	mRenderer = NULL;

	return ret;
}


uint64_t Session::getDuration(
	void)
{
	return (mDemuxer) ? mDemuxer->getDuration() : 0;
}


uint64_t Session::getCurrentTime(
	void)
{
	return (mDemuxer) ? mDemuxer->getCurrentTime() : 0;
}


void Session::getCameraOrientationForHeadtracking(
	float *pan,
	float *tilt)
{
	Eigen::Quaternionf headQuat = mSelfMetadata.getDebiasedHeadOrientation();
	vmeta_quaternion quat;
	quat.w = headQuat.w();
	quat.x = headQuat.x();
	quat.y = headQuat.y();
	quat.z = headQuat.z();
	vmeta_euler euler;
	pdraw_quat2euler(&quat, &euler);

	if (pan)
		*pan = euler.yaw;
	if (tilt)
		*tilt = euler.pitch;
}


void *Session::runLoopThread(
	void *ptr)
{
	Session *session = (Session *)ptr;

	while (!session->mThreadShouldStop)
		pomp_loop_wait_and_process(session->mLoop, -1);

	return NULL;
}

} /* namespace Pdraw */
