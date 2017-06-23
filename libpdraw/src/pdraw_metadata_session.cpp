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
    mLocation.isValid = 0;
    mControllerBatteryLevel = 256;
    mControllerQuat = { 1, 0, 0, 0 };
    mHeadQuat = { 1, 0, 0, 0 };
    mHeadRefQuat = mHeadQuat;
    mIsControllerValid = false;
    mIsHeadValid = false;
    mIsHeadRefValid = false;
}


SessionSelfMetadata::~SessionSelfMetadata()
{

}


void SessionSelfMetadata::getLocation(location_t *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mLocation, sizeof(*loc));
}


void SessionSelfMetadata::setLocation(const location_t *loc)
{
    if (!loc)
        return;
    memcpy(&mLocation, loc, sizeof(*loc));
}


bool SessionSelfMetadata::getControllerOrientation(quaternion_t *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mControllerQuat, sizeof(*quat));
    return mIsControllerValid;
}


bool SessionSelfMetadata::getControllerOrientation(euler_t *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mControllerQuat, euler);
    return mIsControllerValid;
}


void SessionSelfMetadata::setControllerOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mControllerQuat, quat, sizeof(*quat));
    mIsControllerValid = true;
}


void SessionSelfMetadata::setControllerOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mControllerQuat);
    mIsControllerValid = true;
}


bool SessionSelfMetadata::getHeadOrientation(quaternion_t *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mHeadQuat, sizeof(*quat));
    return mIsHeadValid;
}


bool SessionSelfMetadata::getHeadOrientation(euler_t *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mHeadQuat, euler);
    return mIsHeadValid;
}


void SessionSelfMetadata::setHeadOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mHeadQuat, quat, sizeof(*quat));
    mIsHeadValid = true;
}


void SessionSelfMetadata::setHeadOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mHeadQuat);
    mIsHeadValid = true;
}


bool SessionSelfMetadata::getHeadRefOrientation(quaternion_t *quat)
{
    if (!quat)
        return false;
    memcpy(quat, &mHeadRefQuat, sizeof(*quat));
    return mIsHeadRefValid;
}


bool SessionSelfMetadata::getHeadRefOrientation(euler_t *euler)
{
    if (!euler)
        return false;
    pdraw_quat2euler(&mHeadRefQuat, euler);
    return mIsHeadRefValid;
}


void SessionSelfMetadata::setHeadRefOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mHeadRefQuat, quat, sizeof(*quat));
    mIsHeadRefValid = true;
}


void SessionSelfMetadata::setHeadRefOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mHeadRefQuat);
    mIsHeadRefValid = true;
}


SessionPeerMetadata::SessionPeerMetadata()
{
    mTakeoffLocation.isValid = 0;
    mHomeLocation.isValid = 0;
    mDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;
}


SessionPeerMetadata::~SessionPeerMetadata()
{

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


void SessionPeerMetadata::getTakeoffLocation(location_t *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mTakeoffLocation, sizeof(*loc));
}


void SessionPeerMetadata::setTakeoffLocation(const location_t *loc)
{
    if (!loc)
        return;
    memcpy(&mTakeoffLocation, loc, sizeof(*loc));
}


void SessionPeerMetadata::getHomeLocation(location_t *loc)
{
    if (!loc)
        return;
    memcpy(loc, &mHomeLocation, sizeof(*loc));
}


void SessionPeerMetadata::setHomeLocation(const location_t *loc)
{
    if (!loc)
        return;
    memcpy(&mHomeLocation, loc, sizeof(*loc));
}

}
