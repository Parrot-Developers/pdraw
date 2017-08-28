/**
 * @file pdraw_metadata_session.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - session metadata
 * @date 05/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <string.h>

#include "pdraw_metadata_session.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


SessionSelfMetadata::SessionSelfMetadata()
{
    mIsPilot = true;
    mLocation.valid = 0;
    mControllerBatteryLevel = 256;
    mControllerQuat = { 1, 0, 0, 0 };
    mPrevControllerQuat = { 1, 0, 0, 0 };
    mHeadQuat = { 1, 0, 0, 0 };
    mHeadRefQuat = mHeadQuat;
    mIsControllerValid = false;
    mIsHeadValid = false;
    mIsHeadRefValid = false;
    mHeadPsiSpeed = 0;
    mLastHeadPsiTimestamp = 0;
    mControllerQuatRef = { 1, 0, 0, 0 };
    mLastControllerQuatTimestamp = 0;
    mPrevControllerQuatTimestamp = 0;
    mTracking = false;
}


SessionSelfMetadata::~SessionSelfMetadata()
{

}


void SessionSelfMetadata::getLocation(struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mLocation, sizeof(*loc));
}


void SessionSelfMetadata::setLocation(const struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(&mLocation, loc, sizeof(*loc));
}


bool SessionSelfMetadata::getControllerOrientation(struct vmeta_quaternion *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mControllerQuat, sizeof(*quat));
    return mIsControllerValid;
}


bool SessionSelfMetadata::getControllerOrientation(struct vmeta_euler *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mControllerQuat, euler);
    return mIsControllerValid;
}


void SessionSelfMetadata::setControllerOrientation(const struct vmeta_quaternion *quat)
{
    if (!quat)
        return;

    mPrevControllerQuat = mControllerQuat;
    mPrevControllerQuatTimestamp = mLastControllerQuatTimestamp;

    memcpy(&mControllerQuat, quat, sizeof(*quat));
    mIsControllerValid = true;

    struct timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    uint64_t timestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    if (mLastControllerQuatTimestamp == 0)
    {
        mPrevControllerQuat = mControllerQuat;
        mControllerQuatRef = mControllerQuat;
        mPrevControllerQuatTimestamp = mLastControllerQuatTimestamp = timestamp;
    }
    else
    {
        mLastControllerQuatTimestamp = timestamp;
    }
}


void SessionSelfMetadata::setControllerOrientation(const struct vmeta_euler *euler)
{
    if (!euler)
        return;
    struct vmeta_quaternion quat;
    pdraw_euler2quat(euler, &quat);
    setControllerOrientation(&quat);
}


bool SessionSelfMetadata::getHeadOrientation(struct vmeta_quaternion *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mHeadQuat, sizeof(*quat));
    return mIsHeadValid;
}


bool SessionSelfMetadata::getHeadOrientation(struct vmeta_euler *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mHeadQuat, euler);
    return mIsHeadValid;
}


void SessionSelfMetadata::getDebiasedHeadOrientation(struct vmeta_quaternion *quat)
{
    if (!quat)
        return;

    struct timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    struct vmeta_quaternion qControllerNed, qControllerRefConj, qController;
    struct vmeta_quaternion qControllerPsi, qHeadRef, qHead, qHeadRefConj;
    struct vmeta_euler controllerOrientation;

    float alpha = 1.;
    if (mLastControllerQuatTimestamp - mPrevControllerQuatTimestamp > 0)
        alpha = (float)(curTime - mLastControllerQuatTimestamp) / (float)(mLastControllerQuatTimestamp - mPrevControllerQuatTimestamp);
    if (alpha > 1.)
        alpha = 1.;
    //TODO: slerp
    qControllerNed.w = mPrevControllerQuat.w * (1. - alpha) + mControllerQuat.w * alpha;
    qControllerNed.x = mPrevControllerQuat.x * (1. - alpha) + mControllerQuat.x * alpha;
    qControllerNed.y = mPrevControllerQuat.y * (1. - alpha) + mControllerQuat.y * alpha;
    qControllerNed.z = mPrevControllerQuat.z * (1. - alpha) + mControllerQuat.z * alpha;

    if (mHeadPsiSpeed < PDRAW_HEAD_PSI_SPEED_THRES)
    {
        if (mTracking)
        {
            pdraw_quatConj(&mControllerQuatRef, &qControllerRefConj);
            pdraw_quatMult(&qControllerRefConj, &qControllerNed, &qController);
            pdraw_quat2euler(&qController, &controllerOrientation);

            qControllerPsi.x = qControllerPsi.y = 0.;
            sincosf(controllerOrientation.psi / 2., &qControllerPsi.z, &qControllerPsi.w);
            pdraw_quatMult(&qControllerPsi, &mHeadRefQuat, &qHeadRef);
            mHeadRefQuat = qHeadRef;
        }

        mTracking = false;
        mControllerQuatRef = qControllerNed;
    }
    else
    {
        mTracking = true;
    }

    pdraw_quatConj(&mControllerQuatRef, &qControllerRefConj);
    pdraw_quatMult(&qControllerRefConj, &qControllerNed, &qController);
    pdraw_quat2euler(&qController, &controllerOrientation);

    qControllerPsi.x = qControllerPsi.y = 0.;
    sincosf(-controllerOrientation.psi / 2., &qControllerPsi.z, &qControllerPsi.w);
    pdraw_quatMult(&qControllerPsi, &mHeadQuat, &qHead);

    pdraw_quatConj(&mHeadRefQuat, &qHeadRefConj);
    pdraw_quatMult(&qHeadRefConj, &qHead, quat);
}


void SessionSelfMetadata::getDebiasedHeadOrientation(struct vmeta_euler *euler)
{
    if (!euler)
        return;
    struct vmeta_quaternion quat;
    getDebiasedHeadOrientation(&quat);
    pdraw_quat2euler(&quat, euler);
}


void SessionSelfMetadata::setHeadOrientation(const struct vmeta_quaternion *quat)
{
    if (!quat)
        return;

    struct vmeta_euler euler, prevEuler;
    pdraw_quat2euler(&mHeadQuat, &prevEuler);
    pdraw_quat2euler(quat, &euler);

    memcpy(&mHeadQuat, quat, sizeof(*quat));
    mIsHeadValid = true;

    struct timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    uint64_t timestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    if (mLastHeadPsiTimestamp == 0)
    {
        mHeadPsiSpeed = 0;
    }
    else
    {
        uint64_t tsDiff = timestamp - mLastHeadPsiTimestamp;
        if (tsDiff > 0)
            mHeadPsiSpeed = fabs(pdraw_wrapToPi(euler.psi - prevEuler.psi) * 1000000. / (float)tsDiff);
    }

    mLastHeadPsiTimestamp = timestamp;
}


void SessionSelfMetadata::setHeadOrientation(const struct vmeta_euler *euler)
{
    if (!euler)
        return;
    struct vmeta_quaternion quat;
    pdraw_euler2quat(euler, &quat);
    setHeadOrientation(&quat);
}


bool SessionSelfMetadata::getHeadRefOrientation(struct vmeta_quaternion *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mHeadRefQuat, sizeof(*quat));
    return mIsHeadRefValid;
}


bool SessionSelfMetadata::getHeadRefOrientation(struct vmeta_euler *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mHeadRefQuat, euler);
    return mIsHeadRefValid;
}


void SessionSelfMetadata::setHeadRefOrientation(const struct vmeta_quaternion *quat)
{
    if (!quat)
        return;
    memcpy(&mHeadRefQuat, quat, sizeof(*quat));
    mIsHeadRefValid = true;

    mLastHeadPsiTimestamp = 0;
    mHeadPsiSpeed = 0;
    mLastControllerQuatTimestamp = 0;
    memcpy(&mControllerQuat, quat, sizeof(*quat));
    setControllerOrientation(quat);
}


void SessionSelfMetadata::setHeadRefOrientation(const struct vmeta_euler *euler)
{
    if (!euler)
        return;
    struct vmeta_quaternion quat;
    pdraw_euler2quat(euler, &quat);
    setHeadRefOrientation(&quat);
}


void SessionSelfMetadata::resetHeadRefOrientation()
{
    memcpy(&mHeadRefQuat, &mHeadQuat, sizeof(mHeadQuat));
    mIsHeadRefValid = true;

    mLastHeadPsiTimestamp = 0;
    mHeadPsiSpeed = 0;
    mLastControllerQuatTimestamp = 0;
    memcpy(&mControllerQuat, &mHeadQuat, sizeof(mHeadQuat));
    setControllerOrientation(&mHeadQuat);
}


SessionPeerMetadata::SessionPeerMetadata()
{
    mTakeoffLocation.valid = 0;
    mHomeLocation.valid = 0;
    mDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;
    mRecordingStartTime = 0;
}


SessionPeerMetadata::~SessionPeerMetadata()
{

}


void SessionPeerMetadata::set(struct vmeta_session *meta)
{
    if (!meta)
        return;

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
    if (meta->media_date != 0)
    {
        char date[VMETA_SESSION_DATE_MAX_LEN];
        ssize_t ret = vmeta_session_date_write(date, sizeof(date),
            meta->media_date, meta->media_date_gmtoff);
        if (ret > 0)
            setRunDate(std::string(date));
    }
    if (meta->run_date != 0)
    {
        char date[VMETA_SESSION_DATE_MAX_LEN];
        ssize_t ret = vmeta_session_date_write(date, sizeof(date),
            meta->run_date, meta->run_date_gmtoff);
        if (ret > 0)
            setRunDate(std::string(date));
    }
    setRunUuid(std::string(meta->run_id));
    if (meta->takeoff_loc.valid)
    {
        struct vmeta_location loc;
        loc.valid = meta->takeoff_loc.valid;
        loc.latitude = meta->takeoff_loc.latitude;
        loc.longitude = meta->takeoff_loc.longitude;
        loc.altitude = meta->takeoff_loc.altitude;
        loc.svCount = meta->takeoff_loc.svCount;
        setTakeoffLocation(&loc);
    }
}


void SessionPeerMetadata::setFriendlyName(const std::string& friendlyName)
{
    mFriendlyName = friendlyName;
    if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN)
    {
        if (!mFriendlyName.compare("Parrot Bebop"))
            mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
        else if (!mFriendlyName.compare("Parrot Bebop 2"))
            mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
        else if (!mFriendlyName.compare("Parrot Disco"))
            mDroneModel = PDRAW_DRONE_MODEL_DISCO;
    }
}


void SessionPeerMetadata::setModel(const std::string& model)
{
    mModel = model;
    if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN)
    {
        if (!mModel.compare("Bebop"))
            mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
        else if (!mModel.compare("Bebop 2"))
            mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
        else if (!mModel.compare("Disco"))
            mDroneModel = PDRAW_DRONE_MODEL_DISCO;
    }
}


void SessionPeerMetadata::setModelId(const std::string& modelId)
{
    mModelId = modelId;
    if (!mModelId.compare("0901"))
        mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
    else if (!mModelId.compare("090c"))
        mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
    else if (!mModelId.compare("090e"))
        mDroneModel = PDRAW_DRONE_MODEL_DISCO;
}


void SessionPeerMetadata::getTakeoffLocation(struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mTakeoffLocation, sizeof(*loc));
}


void SessionPeerMetadata::setTakeoffLocation(const struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(&mTakeoffLocation, loc, sizeof(*loc));
}


void SessionPeerMetadata::getHomeLocation(struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mHomeLocation, sizeof(*loc));
}


void SessionPeerMetadata::setHomeLocation(const struct vmeta_location *loc)
{
    if (!loc)
        return;
    memcpy(&mHomeLocation, loc, sizeof(*loc));
}


uint64_t SessionPeerMetadata::getRecordingDuration(void)
{
    if (mRecordingStartTime)
    {
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
        return (curTime > mRecordingStartTime) ? curTime - mRecordingStartTime : 0;
    }
    else
    {
        return 0;
    }
}


void SessionPeerMetadata::setRecordingDuration(uint64_t duration)
{
    if (duration == 0)
    {
        mRecordingStartTime = 0;
    }
    else
    {
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
        mRecordingStartTime = (curTime > duration) ? curTime - duration : 0;
    }
}

}
