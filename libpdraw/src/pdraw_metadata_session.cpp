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
    mControllerQuat = { 1, 0, 0, 0 };
    mHeadQuat = { 1, 0, 0, 0 };
    mHeadRefQuat = mHeadQuat;
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


void SessionSelfMetadata::getControllerOrientation(quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(quat, &mControllerQuat, sizeof(*quat));
}


void SessionSelfMetadata::getControllerOrientation(euler_t *euler)
{
    if (!euler)
        return;
    pdraw_quat2euler(&mControllerQuat, euler);
}


void SessionSelfMetadata::setControllerOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mControllerQuat, quat, sizeof(*quat));
}


void SessionSelfMetadata::setControllerOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mControllerQuat);
}


void SessionSelfMetadata::getHeadOrientation(quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(quat, &mHeadQuat, sizeof(*quat));
}


void SessionSelfMetadata::getHeadOrientation(euler_t *euler)
{
    if (!euler)
        return;
    pdraw_quat2euler(&mHeadQuat, euler);
}


void SessionSelfMetadata::setHeadOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mHeadQuat, quat, sizeof(*quat));
}


void SessionSelfMetadata::setHeadOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mHeadQuat);
}


void SessionSelfMetadata::getHeadRefOrientation(quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(quat, &mHeadRefQuat, sizeof(*quat));
}


void SessionSelfMetadata::getHeadRefOrientation(euler_t *euler)
{
    if (!euler)
        return;
    pdraw_quat2euler(&mHeadRefQuat, euler);
}


void SessionSelfMetadata::setHeadRefOrientation(const quaternion_t *quat)
{
    if (!quat)
        return;
    memcpy(&mHeadRefQuat, quat, sizeof(*quat));
}


void SessionSelfMetadata::setHeadRefOrientation(const euler_t *euler)
{
    if (!euler)
        return;
    pdraw_euler2quat(euler, &mHeadRefQuat);
}


SessionPeerMetadata::SessionPeerMetadata()
{
    mTakeoffLocation.isValid = 0;
    mHomeLocation.isValid = 0;
}


SessionPeerMetadata::~SessionPeerMetadata()
{

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
