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
    mLocation.isValid = 0;
    mIsPilot = true;
}


SessionSelfMetadata::~SessionSelfMetadata()
{

}


void SessionSelfMetadata::getLocation(location_t *loc)
{
    if (!loc)
        return;
    loc->isValid = mLocation.isValid;
    loc->latitude = mLocation.latitude;
    loc->longitude = mLocation.longitude;
    loc->altitude = mLocation.altitude;
    loc->svCount = mLocation.svCount;
}


void SessionSelfMetadata::setLocation(const location_t *loc)
{
    if (!loc)
        return;
    mLocation.isValid = loc->isValid;
    mLocation.latitude = loc->latitude;
    mLocation.longitude = loc->longitude;
    mLocation.altitude = loc->altitude;
    mLocation.svCount = loc->svCount;
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
    loc->isValid = mTakeoffLocation.isValid;
    loc->latitude = mTakeoffLocation.latitude;
    loc->longitude = mTakeoffLocation.longitude;
    loc->altitude = mTakeoffLocation.altitude;
    loc->svCount = mTakeoffLocation.svCount;
}


void SessionPeerMetadata::setTakeoffLocation(const location_t *loc)
{
    if (!loc)
        return;
    mTakeoffLocation.isValid = loc->isValid;
    mTakeoffLocation.latitude = loc->latitude;
    mTakeoffLocation.longitude = loc->longitude;
    mTakeoffLocation.altitude = loc->altitude;
    mTakeoffLocation.svCount = loc->svCount;
}


void SessionPeerMetadata::getHomeLocation(location_t *loc)
{
    if (!loc)
        return;
    loc->isValid = mHomeLocation.isValid;
    loc->latitude = mHomeLocation.latitude;
    loc->longitude = mHomeLocation.longitude;
    loc->altitude = mHomeLocation.altitude;
    loc->svCount = mHomeLocation.svCount;
}


void SessionPeerMetadata::setHomeLocation(const location_t *loc)
{
    if (!loc)
        return;
    mHomeLocation.isValid = loc->isValid;
    mHomeLocation.latitude = loc->latitude;
    mHomeLocation.longitude = loc->longitude;
    mHomeLocation.altitude = loc->altitude;
    mHomeLocation.svCount = loc->svCount;
}

}
