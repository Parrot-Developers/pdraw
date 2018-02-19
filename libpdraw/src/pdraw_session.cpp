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
ULOG_DECLARE_TAG(libpdraw);
#include <algorithm>
#include <string>
#include <vector>

namespace Pdraw {


IPdraw *createPdraw(
	void)
{
	return new Session;
}


Session::Session(
	void)
{
	int ret;

	mSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
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
		int ret = mDemuxer->close();
		if (ret != 0)
			ULOGE("Session: failed to close demuxer");
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


int Session::close(
	void)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->close();
		if (ret != 0) {
			ULOGE("Failed to close demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::play(
	float speed)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->play(speed);
		if (ret != 0) {
			ULOGE("Failed to start demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::pause(
	void)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->pause();
		if (ret != 0) {
			ULOGE("Failed to pause demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


bool Session::isPaused(
	void)
{
	if (mDemuxer != NULL)
		return mDemuxer->isPaused();
	else
		return false;
}


int Session::previousFrame(
	void)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->previous();
		if (ret != 0) {
			ULOGE("Failed to go to previous frame in the demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::nextFrame(
	void)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->next();
		if (ret != 0) {
			ULOGE("Failed to go to next frame in the demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::seekTo(
	uint64_t timestamp,
	bool exact)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->seekTo(timestamp, exact);
		if (ret != 0) {
			ULOGE("Failed to seek with demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::seekForward(
	uint64_t delta,
	bool exact)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->seekForward(delta, exact);
		if (ret != 0) {
			ULOGE("Failed to seek with demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
}


int Session::seekBack(
	uint64_t delta,
	bool exact)
{
	if (mDemuxer != NULL) {
		int ret = mDemuxer->seekBack(delta, exact);
		if (ret != 0) {
			ULOGE("Failed to seek with demuxer");
			return -1;
		}
	} else {
		ULOGE("Invalid demuxer");
		return -1;
	}

	return 0;
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


int Session::startRenderer(
	int windowWidth,
	int windowHeight,
	int renderX,
	int renderY,
	int renderWidth,
	int renderHeight,
	bool hmdDistorsionCorrection,
	bool headtracking,
	void *uiHandler)
{
	if (mRenderer == NULL) {
		int ret = enableRenderer();
		if (ret != 0)
			ULOGE("Failed to enable renderer");
	}

	if (mRenderer != NULL) {
		return mRenderer->setRendererParams(
			windowWidth, windowHeight, renderX, renderY,
			renderWidth, renderHeight, hmdDistorsionCorrection,
			headtracking, uiHandler);
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}
}


int Session::stopRenderer(
	void)
{
	if (mRenderer != NULL) {
		int ret = disableRenderer();
		if (ret != 0) {
			ULOGE("Failed to disable renderer");
			return -1;
		}
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}

	return 0;
}


int Session::render(
	uint64_t lastRenderTime)
{
	if (mRenderer != NULL) {
		return mRenderer->render(lastRenderTime);
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}
}


std::string& Session::getSelfFriendlyName(
	void)
{
	return mSelfMetadata.getFriendlyName();
}


void Session::setSelfFriendlyName(
	const std::string &friendlyName)
{
	mSelfMetadata.setFriendlyName(friendlyName);
}


std::string& Session::getSelfSerialNumber(
	void)
{
	return mSelfMetadata.getSerialNumber();
}


void Session::setSelfSerialNumber(
	const std::string &serialNumber)
{
	mSelfMetadata.setSerialNumber(serialNumber);
}


std::string& Session::getSelfSoftwareVersion(
	void)
{
	return mSelfMetadata.getSoftwareVersion();
}


void Session::setSelfSoftwareVersion(
	const std::string &softwareVersion)
{
	mSelfMetadata.setSoftwareVersion(softwareVersion);
}


bool Session::isSelfPilot(
	void)
{
	return mSelfMetadata.isPilot();
}


void Session::setSelfPilot(
	bool isPilot)
{
	mSelfMetadata.setPilot(isPilot);
}


void Session::getSelfLocation(
	struct vmeta_location *loc)
{
	mSelfMetadata.getLocation(loc);
}


void Session::setSelfLocation(
	const struct vmeta_location *loc)
{
	mSelfMetadata.setLocation(loc);
}


int Session::getControllerBatteryLevel(
	void)
{
	return mSelfMetadata.getControllerBatteryLevel();
}


void Session::setControllerBatteryLevel(
	int batteryLevel)
{
	mSelfMetadata.setControllerBatteryLevel(batteryLevel);
}


void Session::getSelfControllerOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getControllerOrientation(quat);
}


void Session::getSelfControllerOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getControllerOrientation(euler);
}


void Session::setSelfControllerOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setControllerOrientation(quat);
}


void Session::setSelfControllerOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setControllerOrientation(euler);
}


void Session::getSelfHeadOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getHeadOrientation(quat);
}


void Session::getSelfHeadOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getHeadOrientation(euler);
}


void Session::setSelfHeadOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setHeadOrientation(quat);
}


void Session::setSelfHeadOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setHeadOrientation(euler);
}


void Session::getSelfHeadRefOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getHeadRefOrientation(quat);
}


void Session::getSelfHeadRefOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getHeadRefOrientation(euler);
}


void Session::setSelfHeadRefOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setHeadRefOrientation(quat);
}


void Session::setSelfHeadRefOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setHeadRefOrientation(euler);
}


void Session::resetSelfHeadRefOrientation(
	void)
{
	mSelfMetadata.resetHeadRefOrientation();
}


std::string& Session::getPeerFriendlyName(
	void)
{
	return mPeerMetadata.getFriendlyName();
}


std::string& Session::getPeerMaker(
	void)
{
	return mPeerMetadata.getMaker();
}


std::string& Session::getPeerModel(
	void)
{
	return mPeerMetadata.getModel();
}


std::string& Session::getPeerModelId(
	void)
{
	return mPeerMetadata.getModelId();
}


enum pdraw_drone_model Session::getPeerDroneModel(
	void)
{
	return mPeerMetadata.getDroneModel();
}


std::string& Session::getPeerSerialNumber(
	void)
{
	return mPeerMetadata.getSerialNumber();
}


std::string& Session::getPeerSoftwareVersion(
	void)
{
	return mPeerMetadata.getSoftwareVersion();
}


std::string& Session::getPeerBuildId(
	void)
{
	return mPeerMetadata.getBuildId();
}


std::string& Session::getPeerTitle(
	void)
{
	return mPeerMetadata.getTitle();
}


std::string& Session::getPeerComment(
	void)
{
	return mPeerMetadata.getComment();
}


std::string& Session::getPeerCopyright(
	void)
{
	return mPeerMetadata.getCopyright();
}


std::string& Session::getPeerRunDate(
	void)
{
	return mPeerMetadata.getRunDate();
}


std::string& Session::getPeerRunUuid(
	void)
{
	return mPeerMetadata.getRunUuid();
}


std::string& Session::getPeerMediaDate(
	void)
{
	return mPeerMetadata.getMediaDate();
}


void Session::getPeerTakeoffLocation(
	struct vmeta_location *loc)
{
	mPeerMetadata.getTakeoffLocation(loc);
}


void Session::setPeerTakeoffLocation(
	const struct vmeta_location *loc)
{
	mPeerMetadata.setTakeoffLocation(loc);
}


void Session::getPeerHomeLocation(
	struct vmeta_location *loc)
{
	mPeerMetadata.getHomeLocation(loc);
}


void Session::setPeerHomeLocation(
	const struct vmeta_location *loc)
{
	mPeerMetadata.setHomeLocation(loc);
}


uint64_t Session::getPeerRecordingDuration(
	void)
{
	return mPeerMetadata.getRecordingDuration();
}


void Session::setPeerRecordingDuration(
	uint64_t duration)
{
	mPeerMetadata.setRecordingDuration(duration);
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


int Session::getMediaCount(
	void)
{
	return mMedias.size();
}


int Session::getMediaInfo(
	unsigned int index,
	struct pdraw_media_info *info)
{
	if (info == NULL) {
		ULOGE("Invalid info struct");
		return -1;
	}

	Media *media = getMedia(index);

	if (media == NULL) {
		ULOGE("Invalid media index");
		return -1;
	}

	switch (media->getType()) {
	case PDRAW_MEDIA_TYPE_VIDEO:
		info->type = PDRAW_MEDIA_TYPE_VIDEO;
		info->info.video.type = ((VideoMedia*)media)->getVideoType();
		((VideoMedia*)media)->getDimensions(
			&info->info.video.width,
			&info->info.video.height,
			&info->info.video.cropLeft,
			&info->info.video.cropRight,
			&info->info.video.cropTop,
			&info->info.video.cropBottom,
			&info->info.video.croppedWidth,
			&info->info.video.croppedHeight,
			&info->info.video.sarWidth,
			&info->info.video.sarHeight);
		((VideoMedia*)media)->getFov(
			&info->info.video.horizontalFov,
			&info->info.video.verticalFov);
		break;
	default:
		info->type = PDRAW_MEDIA_TYPE_UNKNOWN;
		break;
	}
	info->id = media->getId();

	return 0;
}


void *Session::addVideoFrameFilterCallback(
	unsigned int mediaId,
	pdraw_video_frame_filter_callback_t cb,
	void *userPtr)
{
	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		ULOGE("Invalid media id");
		return NULL;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		ULOGE("Invalid media type");
		return NULL;
	}

	VideoFrameFilter *filter =
		((VideoMedia*)media)->addVideoFrameFilter(cb, userPtr);
	if (filter == NULL) {
		ULOGE("Failed to create video frame filter");
		return NULL;
	}

	return (void *)filter;
}


int Session::removeVideoFrameFilterCallback(
	unsigned int mediaId,
	void *filterCtx)
{
	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		ULOGE("Invalid media id");
		return -1;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		ULOGE("Invalid media type");
		return -1;
	}

	if (filterCtx == NULL) {
		ULOGE("Invalid context pointer");
		return -1;
	}

	VideoFrameFilter *filter = (VideoFrameFilter*)filterCtx;
	return ((VideoMedia*)media)->removeVideoFrameFilter(filter);
}


void *Session::addVideoFrameProducer(
	unsigned int mediaId,
	bool frameByFrame)
{
	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		ULOGE("Invalid media id");
		return NULL;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		ULOGE("Invalid media type");
		return NULL;
	}

	VideoFrameFilter *filter =
		((VideoMedia*)media)->addVideoFrameFilter(frameByFrame);
	if (filter == NULL) {
		ULOGE("Failed to create video frame filter");
		return NULL;
	}

	return (void*)filter;
}


int Session::removeVideoFrameProducer(
	void *producerCtx)
{
	if (producerCtx == NULL) {
		ULOGE("Invalid context pointer");
		return -1;
	}

	VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;
	VideoMedia *media = filter->getVideoMedia();

	if (media == NULL) {
		ULOGE("Invalid media");
		return -1;
	}

	return media->removeVideoFrameFilter(filter);
}


int Session::getProducerLastFrame(
	void *producerCtx,
	struct pdraw_video_frame *frame,
	int timeout)
{
	if (producerCtx == NULL) {
		ULOGE("Invalid context pointer");
		return -1;
	}
	if (frame == NULL) {
		ULOGE("Invalid frame structure pointer");
		return -1;
	}

	VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;

	return filter->getLastFrame(frame, timeout);
}


float Session::getControllerRadarAngleSetting(
	void)
{
	return mSettings.getControllerRadarAngle();
}


void Session::setControllerRadarAngleSetting(
	float angle)
{
	mSettings.setControllerRadarAngle(angle);
}


void Session::getDisplayScreenSettings(
	float *xdpi,
	float *ydpi,
	float *deviceMargin)
{
	mSettings.getDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void Session::setDisplayScreenSettings(
	float xdpi,
	float ydpi,
	float deviceMargin)
{
	mSettings.setDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void Session::getHmdDistorsionCorrectionSettings(
	enum pdraw_hmd_model *hmdModel,
	float *ipd,
	float *scale,
	float *panH,
	float *panV)
{
	mSettings.getHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
}


void Session::setHmdDistorsionCorrectionSettings(
	enum pdraw_hmd_model hmdModel,
	float ipd,
	float scale,
	float panH,
	float panV)
{
	mSettings.setHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
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


void *Session::runLoopThread(
	void *ptr)
{
	Session *session = (Session *)ptr;

	while (!session->mThreadShouldStop)
		pomp_loop_wait_and_process(session->mLoop, -1);

	return NULL;
}

} /* namespace Pdraw */
