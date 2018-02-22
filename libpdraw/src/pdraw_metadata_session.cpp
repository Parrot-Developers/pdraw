/**
 * Parrot Drones Awesome Video Viewer Library
 * Session metadata
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

#include "pdraw_metadata_session.hpp"
#include <math.h>
#include <string.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <string>

#ifdef __APPLE__
#ifndef sincosf
#define sincosf __sincosf
#endif
#endif

namespace Pdraw {


SessionSelfMetadata::SessionSelfMetadata(
	void)
{
	int res;
	pthread_mutexattr_t attr;
	bool mutex_created = false, attr_created = false;

	mIsPilot = true;
	mLocation.valid = 0;
	mControllerBatteryLevel = 256;
	mControllerQuat = Eigen::Quaternionf(1., 0., 0., 0.);
	mPrevControllerQuat = Eigen::Quaternionf(1., 0., 0., 0.);
	mHeadQuat = Eigen::Quaternionf(1., 0., 0., 0.);
	mHeadRefQuat = mHeadQuat;
	mIsControllerValid = false;
	mIsHeadValid = false;
	mIsHeadRefValid = false;
	mHeadPsiSpeed = 0;
	mLastHeadPsiTimestamp = 0;
	mControllerQuatRef = Eigen::Quaternionf(1., 0., 0., 0.);
	mLastControllerQuatTimestamp = 0;
	mPrevControllerQuatTimestamp = 0;
	mTracking = false;

	res = pthread_mutexattr_init(&attr);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutexattr_init", -res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutexattr_settype", -res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutex_init", -res);
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


SessionSelfMetadata::~SessionSelfMetadata(
	void)
{
	pthread_mutex_destroy(&mMutex);
}


void SessionSelfMetadata::lock(
	void)
{
	pthread_mutex_lock(&mMutex);
}


void SessionSelfMetadata::unlock(
	void)
{
	pthread_mutex_unlock(&mMutex);
}


std::string SessionSelfMetadata::getFriendlyName(
	void)
{
	pthread_mutex_lock(&mMutex);
	std::string ret = mFriendlyName;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setFriendlyName(
	const std::string& friendlyName)
{
	pthread_mutex_lock(&mMutex);
	mFriendlyName = friendlyName;
	pthread_mutex_unlock(&mMutex);
}


std::string SessionSelfMetadata::getSerialNumber(
	void)
{
	pthread_mutex_lock(&mMutex);
	std::string ret = mSerialNumber;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setSerialNumber(
	const std::string& serialNumber)
{
	pthread_mutex_lock(&mMutex);
	mSerialNumber = serialNumber;
	pthread_mutex_unlock(&mMutex);
}


std::string SessionSelfMetadata::getSoftwareVersion(
	void)
{
	pthread_mutex_lock(&mMutex);
	std::string ret = mSoftwareVersion;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setSoftwareVersion(
	const std::string& softwareVersion)
{
	pthread_mutex_lock(&mMutex);
	mSoftwareVersion = softwareVersion;
	pthread_mutex_unlock(&mMutex);
}


bool SessionSelfMetadata::isPilot(
	void)
{
	pthread_mutex_lock(&mMutex);
	bool ret = mIsPilot;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setPilot(
	bool isPilot)
{
	pthread_mutex_lock(&mMutex);
	mIsPilot = isPilot;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::getLocation(
	struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(loc, &mLocation, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setLocation(
	const struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(&mLocation, loc, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


int SessionSelfMetadata::getControllerBatteryLevel(
	void)
{
	pthread_mutex_lock(&mMutex);
	int ret = mControllerBatteryLevel;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setControllerBatteryLevel(
	int batteryLevel)
{
	pthread_mutex_lock(&mMutex);
	mControllerBatteryLevel = batteryLevel;
	pthread_mutex_unlock(&mMutex);
}


bool SessionSelfMetadata::getControllerOrientation(
	struct vmeta_quaternion *quat)
{
	if (!quat)
		return false;

	pthread_mutex_lock(&mMutex);
	quat->w = mControllerQuat.w();
	quat->x = mControllerQuat.x();
	quat->y = mControllerQuat.y();
	quat->z = mControllerQuat.z();
	bool valid = mIsControllerValid;
	pthread_mutex_unlock(&mMutex);

	return valid;
}


bool SessionSelfMetadata::getControllerOrientation(
	struct vmeta_euler *euler)
{
	if (!euler)
		return false;

	struct vmeta_quaternion quat;
	/* No lock */
	bool ret = getControllerOrientation(&quat);
	pdraw_quat2euler(&quat, euler);
	return ret;
}


void SessionSelfMetadata::setControllerOrientation(
	Eigen::Quaternionf &quat)
{
	pthread_mutex_lock(&mMutex);

	mPrevControllerQuat = mControllerQuat;
	mPrevControllerQuatTimestamp = mLastControllerQuatTimestamp;

	mControllerQuat = quat;
	mIsControllerValid = true;

	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	uint64_t timestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	if (mLastControllerQuatTimestamp == 0) {
		mPrevControllerQuat = mControllerQuat;
		mControllerQuatRef = mControllerQuat;
		mPrevControllerQuatTimestamp = timestamp;
		mLastControllerQuatTimestamp = timestamp;
	} else {
		mLastControllerQuatTimestamp = timestamp;
	}

	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setControllerOrientation(
	const struct vmeta_quaternion *quat)
{
	if (!quat)
		return;

	Eigen::Quaternionf _quat = Eigen::Quaternionf(
		quat->w, quat->x, quat->y, quat->z);
	/* No lock */
	setControllerOrientation(_quat);
}


void SessionSelfMetadata::setControllerOrientation(
	const struct vmeta_euler *euler)
{
	if (!euler)
		return;

	struct vmeta_quaternion quat;
	pdraw_euler2quat(euler, &quat);
	/* No lock */
	setControllerOrientation(&quat);
}


bool SessionSelfMetadata::getHeadOrientation(
	struct vmeta_quaternion *quat)
{
	if (!quat)
		return false;

	pthread_mutex_lock(&mMutex);
	quat->w = mHeadQuat.w();
	quat->x = mHeadQuat.x();
	quat->y = mHeadQuat.y();
	quat->z = mHeadQuat.z();
	bool valid = mIsHeadValid;
	pthread_mutex_unlock(&mMutex);

	return valid;
}


bool SessionSelfMetadata::getHeadOrientation(
	struct vmeta_euler *euler)
{
	if (!euler)
		return false;

	struct vmeta_quaternion quat;
	/* No lock */
	bool ret = getHeadOrientation(&quat);
	pdraw_quat2euler(&quat, euler);
	return ret;
}


Eigen::Quaternionf SessionSelfMetadata::getDebiasedHeadOrientation(
	void)
{
	pthread_mutex_lock(&mMutex);

	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	uint64_t curTime =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	Eigen::Quaternionf qControllerNed, qController;
	Eigen::Quaternionf qControllerPsi, qHeadRef, qHead, qDebiasedHead;
	struct vmeta_euler controllerOrientation;
	struct vmeta_quaternion quat;
	float w = 0., z = 0.;

	float alpha = 1.;
	if (mLastControllerQuatTimestamp - mPrevControllerQuatTimestamp > 0) {
		alpha = (float)(curTime - mLastControllerQuatTimestamp) /
			(float)(mLastControllerQuatTimestamp -
				mPrevControllerQuatTimestamp);
	}
	if (alpha > 1.)
		alpha = 1.;
	qControllerNed = mControllerQuat.slerp(alpha, mPrevControllerQuat);

	if (mHeadPsiSpeed < HEAD_PSI_SPEED_THRES) {
		if (mTracking) {
			qController =
				mControllerQuatRef.conjugate() * qControllerNed;
			quat.w = qController.w();
			quat.x = qController.x();
			quat.y = qController.y();
			quat.z = qController.z();
			pdraw_quat2euler(&quat, &controllerOrientation);

			sincosf(controllerOrientation.psi / 2., &z, &w);
			qControllerPsi = Eigen::Quaternionf(w, 0., 0., z);
			qHeadRef = qControllerPsi * mHeadRefQuat;
			mHeadRefQuat = qHeadRef;
		}

		mTracking = false;
		mControllerQuatRef = qControllerNed;
	} else {
		mTracking = true;
	}

	qController = mControllerQuatRef.conjugate() * qControllerNed;
	quat.w = qController.w();
	quat.x = qController.x();
	quat.y = qController.y();
	quat.z = qController.z();
	pdraw_quat2euler(&quat, &controllerOrientation);

	sincosf(-controllerOrientation.psi / 2., &z, &w);
	qControllerPsi = Eigen::Quaternionf(w, 0., 0., z);
	qHead = qControllerPsi * mHeadQuat;

	qDebiasedHead = mHeadRefQuat.conjugate() * qHead;

	pthread_mutex_unlock(&mMutex);

	return qDebiasedHead;
}


void SessionSelfMetadata::setHeadOrientation(
	Eigen::Quaternionf &quat)
{
	pthread_mutex_lock(&mMutex);

	struct vmeta_quaternion headQuat, prevHeadQuat;
	struct vmeta_euler headOrientation, prevHeadOrientation;

	prevHeadQuat.w = mHeadQuat.w();
	prevHeadQuat.x = mHeadQuat.x();
	prevHeadQuat.y = mHeadQuat.y();
	prevHeadQuat.z = mHeadQuat.z();
	pdraw_quat2euler(&prevHeadQuat, &prevHeadOrientation);

	mHeadQuat = quat;
	mIsHeadValid = true;

	headQuat.w = mHeadQuat.w();
	headQuat.x = mHeadQuat.x();
	headQuat.y = mHeadQuat.y();
	headQuat.z = mHeadQuat.z();
	pdraw_quat2euler(&headQuat, &headOrientation);

	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	uint64_t timestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	if (mLastHeadPsiTimestamp == 0) {
		mHeadPsiSpeed = 0;
	} else {
		uint64_t tsDiff = timestamp - mLastHeadPsiTimestamp;
		if (tsDiff > 0) {
			mHeadPsiSpeed = fabs(pdraw_wrapToPi(
				headOrientation.psi - prevHeadOrientation.psi) *
				1000000. / (float)tsDiff);
		}
	}

	mLastHeadPsiTimestamp = timestamp;

	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setHeadOrientation(
	const struct vmeta_quaternion *quat)
{
	if (!quat)
		return;

	Eigen::Quaternionf _quat = Eigen::Quaternionf(
		quat->w, quat->x, quat->y, quat->z);
	/* No lock */
	setHeadOrientation(_quat);
}


void SessionSelfMetadata::setHeadOrientation(
	const struct vmeta_euler *euler)
{
	if (!euler)
		return;

	struct vmeta_quaternion quat;
	pdraw_euler2quat(euler, &quat);
	/* No lock */
	setHeadOrientation(&quat);
}


bool SessionSelfMetadata::getHeadRefOrientation(
	struct vmeta_quaternion *quat)
{
	if (!quat)
		return false;

	pthread_mutex_lock(&mMutex);
	quat->w = mHeadRefQuat.w();
	quat->x = mHeadRefQuat.x();
	quat->y = mHeadRefQuat.y();
	quat->z = mHeadRefQuat.z();
	bool valid = mIsHeadRefValid;
	pthread_mutex_unlock(&mMutex);

	return valid;
}


bool SessionSelfMetadata::getHeadRefOrientation(
	struct vmeta_euler *euler)
{
	if (!euler)
		return false;

	struct vmeta_quaternion quat;
	/* No lock */
	bool ret = getHeadRefOrientation(&quat);
	pdraw_quat2euler(&quat, euler);
	return ret;
}


void SessionSelfMetadata::setHeadRefOrientation(
	const struct vmeta_quaternion *quat)
{
	if (!quat)
		return;

	pthread_mutex_lock(&mMutex);

	mHeadRefQuat = Eigen::Quaternionf(quat->w, quat->x, quat->y, quat->z);
	mIsHeadRefValid = true;

	mLastHeadPsiTimestamp = 0;
	mHeadPsiSpeed = 0;
	mLastControllerQuatTimestamp = 0;
	mControllerQuat = mHeadRefQuat;

	setControllerOrientation(mControllerQuat);

	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setHeadRefOrientation(
	const struct vmeta_euler *euler)
{
	if (!euler)
		return;

	struct vmeta_quaternion quat;
	pdraw_euler2quat(euler, &quat);
	/* No lock */
	setHeadRefOrientation(&quat);
}


void SessionSelfMetadata::resetHeadRefOrientation(
	void)
{
	pthread_mutex_lock(&mMutex);

	mHeadRefQuat = mHeadQuat;
	mIsHeadRefValid = true;

	mLastHeadPsiTimestamp = 0;
	mHeadPsiSpeed = 0;
	mLastControllerQuatTimestamp = 0;
	mControllerQuat = mHeadRefQuat;

	setControllerOrientation(mControllerQuat);

	pthread_mutex_unlock(&mMutex);
}


SessionPeerMetadata::SessionPeerMetadata(
	void)
{
	int res;
	pthread_mutexattr_t attr;
	bool mutex_created = false, attr_created = false;

	mTakeoffLocation.valid = 0;
	mHomeLocation.valid = 0;
	mDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;
	mRecordingStartTime = 0;

	res = pthread_mutexattr_init(&attr);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutexattr_init", -res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutexattr_settype", -res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res < 0) {
		ULOG_ERRNO("SessionMetadata: pthread_mutex_init", -res);
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


SessionPeerMetadata::~SessionPeerMetadata(
	void)
{
	pthread_mutex_destroy(&mMutex);
}


void SessionPeerMetadata::lock(
	void)
{
	pthread_mutex_lock(&mMutex);
}


void SessionPeerMetadata::unlock(
	void)
{
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::set(
	const struct vmeta_session *meta)
{
	if (!meta)
		return;

	pthread_mutex_lock(&mMutex);

	setFriendlyName(std::string(meta->friendly_name));
	setMaker(std::string(meta->maker));
	setModel(std::string(meta->model));
	setModelId(std::string(meta->model_id));
	setSerialNumber(std::string(meta->serial_number));
	setSoftwareVersion(std::string(meta->software_version));
	setBuildId(std::string(meta->build_id));
	setTitle(std::string(meta->title));
	setComment(std::string(meta->comment));
	setCopyright(std::string(meta->copyright));
	if (meta->media_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date, sizeof(date),
			meta->media_date, meta->media_date_gmtoff);
		if (ret > 0)
			setRunDate(std::string(date));
	}
	if (meta->run_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date, sizeof(date),
			meta->run_date, meta->run_date_gmtoff);
		if (ret > 0)
			setRunDate(std::string(date));
	}
	setRunUuid(std::string(meta->run_id));
	if (meta->takeoff_loc.valid) {
		struct vmeta_location loc;
		loc.valid = meta->takeoff_loc.valid;
		loc.latitude = meta->takeoff_loc.latitude;
		loc.longitude = meta->takeoff_loc.longitude;
		loc.altitude = meta->takeoff_loc.altitude;
		loc.svCount = meta->takeoff_loc.svCount;
		setTakeoffLocation(&loc);
	}

	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::setFriendlyName(
	const std::string& friendlyName)
{
	pthread_mutex_lock(&mMutex);

	mFriendlyName = friendlyName;
	if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN) {
		if (!mFriendlyName.compare("Parrot Bebop"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
		else if (!mFriendlyName.compare("Parrot Bebop 2"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
		else if (!mFriendlyName.compare("Parrot Disco"))
			mDroneModel = PDRAW_DRONE_MODEL_DISCO;
	}

	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::setModel(
	const std::string& model)
{
	pthread_mutex_lock(&mMutex);

	mModel = model;
	if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN) {
		if (!mModel.compare("Bebop"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
		else if (!mModel.compare("Bebop 2"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
		else if (!mModel.compare("Disco"))
			mDroneModel = PDRAW_DRONE_MODEL_DISCO;
	}

	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::setModelId(
	const std::string& modelId)
{
	pthread_mutex_lock(&mMutex);

	mModelId = modelId;
	if (!mModelId.compare("0901"))
		mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
	else if (!mModelId.compare("090c"))
		mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
	else if (!mModelId.compare("090e"))
		mDroneModel = PDRAW_DRONE_MODEL_DISCO;

	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::getTakeoffLocation(
	struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(loc, &mTakeoffLocation, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::setTakeoffLocation(
	const struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(&mTakeoffLocation, loc, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::getHomeLocation(
	struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(loc, &mHomeLocation, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::setHomeLocation(
	const struct vmeta_location *loc)
{
	if (!loc)
		return;

	pthread_mutex_lock(&mMutex);
	memcpy(&mHomeLocation, loc, sizeof(*loc));
	pthread_mutex_unlock(&mMutex);
}


uint64_t SessionPeerMetadata::getRecordingDuration(
	void)
{
	uint64_t ret;
	pthread_mutex_lock(&mMutex);

	if (mRecordingStartTime) {
		struct timespec t1;
		clock_gettime(CLOCK_MONOTONIC, &t1);
		uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 +
			(uint64_t)t1.tv_nsec / 1000;
		ret = (curTime > mRecordingStartTime) ?
			curTime - mRecordingStartTime : 0;
	} else {
		ret = 0;
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionPeerMetadata::setRecordingDuration(
	uint64_t duration)
{
	pthread_mutex_lock(&mMutex);

	if (duration == 0) {
		mRecordingStartTime = 0;
	} else {
		struct timespec t1;
		clock_gettime(CLOCK_MONOTONIC, &t1);
		uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 +
			(uint64_t)t1.tv_nsec / 1000;
		mRecordingStartTime =
			(curTime > duration) ? curTime - duration : 0;
	}

	pthread_mutex_unlock(&mMutex);
}

} /* namespace Pdraw */
