/**
 * @file pdraw_metadata.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - metadata
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

#include <video-metadata/vmeta.h>

#include "pdraw_metadata.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


Metadata::Metadata()
{

}


Metadata::~Metadata()
{

}


string Metadata::getTitle(void)
{
    return mTitle;
}


void Metadata::setTitle(const string& title)
{
    mTitle = title;
}


string Metadata::getMaker(void)
{
    return mMaker;
}


void Metadata::setMaker(const string& maker)
{
    mMaker = maker;
}


string Metadata::getModel(void)
{
    return mModel;
}


void Metadata::setModel(const string& model)
{
    mModel = model;
}


string Metadata::getVersion(void)
{
    return mVersion;
}


void Metadata::setVersion(const string& version)
{
    mVersion = version;
}


string Metadata::getSerial(void)
{
    return mSerial;
}


void Metadata::setSerial(const string& serial)
{
    mSerial = serial;
}


string Metadata::getCopyright(void)
{
    return mCopyright;
}


void Metadata::setCopyright(const string& copyright)
{
    mCopyright = copyright;
}


string Metadata::getComment(void)
{
    return mComment;
}


void Metadata::setComment(const string& comment)
{
    mComment = comment;
}


string Metadata::getMediaDate(void)
{
    return mMediaDate;
}


void Metadata::setMediaDate(const string& mediaDate)
{
    mMediaDate = mediaDate;
}


string Metadata::getRunDate(void)
{
    return mRunDate;
}


void Metadata::setRunDate(const string& runDate)
{
    mRunDate = runDate;
}


string Metadata::getRunUuid(void)
{
    return mRunUuid;
}


void Metadata::setRunUuid(const string& runUuid)
{
    mRunUuid = runUuid;
}


void Metadata::getTakeoffLocation(location_t *takeoff)
{
    if (!takeoff)
        return;
    takeoff->isValid = mTakeoffLocation.isValid;
    takeoff->latitude = mTakeoffLocation.latitude;
    takeoff->longitude = mTakeoffLocation.longitude;
    takeoff->altitude = mTakeoffLocation.altitude;
    takeoff->svCount = mTakeoffLocation.svCount;
}


void Metadata::setTakeoffLocation(const location_t *takeoff)
{
    if (!takeoff)
        return;
    mTakeoffLocation.isValid = takeoff->isValid;
    mTakeoffLocation.latitude = takeoff->latitude;
    mTakeoffLocation.longitude = takeoff->longitude;
    mTakeoffLocation.altitude = takeoff->altitude;
    mTakeoffLocation.svCount = takeoff->svCount;
}


void Metadata::getHomeLocation(location_t *home)
{
    if (!home)
        return;
    home->isValid = mHomeLocation.isValid;
    home->latitude = mHomeLocation.latitude;
    home->longitude = mHomeLocation.longitude;
    home->altitude = mHomeLocation.altitude;
    home->svCount = mHomeLocation.svCount;
}


void Metadata::setHomeLocation(const location_t *home)
{
    if (!home)
        return;
    mHomeLocation.isValid = home->isValid;
    mHomeLocation.latitude = home->latitude;
    mHomeLocation.longitude = home->longitude;
    mHomeLocation.altitude = home->altitude;
    mHomeLocation.svCount = home->svCount;
}


void Metadata::getPilotLocation(location_t *pilot)
{
    if (!pilot)
        return;
    pilot->isValid = mPilotLocation.isValid;
    pilot->latitude = mPilotLocation.latitude;
    pilot->longitude = mPilotLocation.longitude;
    pilot->altitude = mPilotLocation.altitude;
    pilot->svCount = mPilotLocation.svCount;
}


void Metadata::setPilotLocation(const location_t *pilot)
{
    if (!pilot)
        return;
    mPilotLocation.isValid = pilot->isValid;
    mPilotLocation.latitude = pilot->latitude;
    mPilotLocation.longitude = pilot->longitude;
    mPilotLocation.altitude = pilot->altitude;
    mPilotLocation.svCount = pilot->svCount;
}


void Metadata::getPictureFov(float *pictureHFov, float *pictureVFov)
{
    if (pictureHFov) *pictureHFov = mPictureHFov;
    if (pictureVFov) *pictureVFov = mPictureVFov;
}


void Metadata::setPictureFov(float pictureHFov, float pictureVFov)
{
    mPictureHFov = pictureHFov;
    mPictureVFov = pictureVFov;
}


static void mapFrameMetadataV1rec(const struct vmeta_v1_recording *meta, frame_metadata_t *metadata)
{
    metadata->groundDistance = meta->altitude;
    metadata->location.isValid = (meta->location.valid) ? true : false;
    if (meta->location.valid)
    {
        metadata->location.latitude = meta->location.latitude;
        metadata->location.longitude = meta->location.longitude;
        metadata->location.altitude = meta->location.altitude;
        metadata->location.svCount = meta->location.svCount;
    }
    //TODO speed xyz to NED?
    metadata->groundSpeed.north = meta->speed.x;
    metadata->groundSpeed.east = meta->speed.y;
    metadata->groundSpeed.down = meta->speed.z;
    metadata->airSpeed = -1.;
    metadata->droneAttitude.phi = meta->droneAttitude.roll;
    metadata->droneAttitude.theta = meta->droneAttitude.pitch;
    metadata->droneAttitude.psi = meta->droneAttitude.yaw;
    pdraw_euler2quat(&metadata->droneAttitude, &metadata->droneQuat);
    metadata->frameQuat.w = meta->frameQuat.w;
    metadata->frameQuat.x = meta->frameQuat.x;
    metadata->frameQuat.y = meta->frameQuat.y;
    metadata->frameQuat.z = meta->frameQuat.z;
    pdraw_quat2euler(&metadata->frameQuat, &metadata->frameOrientation);
    metadata->cameraPan = meta->cameraPan;
    metadata->cameraTilt = meta->cameraTilt;
    metadata->exposureTime = meta->exposureTime;
    metadata->gain = meta->gain;
    metadata->flyingState = meta->state;
    metadata->binning = meta->binning;
    metadata->pilotingMode = meta->mode;
    metadata->animation = meta->animation;
    metadata->wifiRssi = meta->wifiRssi;
    metadata->batteryPercentage = meta->batteryPercentage;
}


static void mapFrameMetadataV1strmext(const struct vmeta_v1_streaming_extended *meta, frame_metadata_t *metadata)
{
    metadata->groundDistance = meta->altitude;
    metadata->location.isValid = (meta->location.valid) ? true : false;
    if (meta->location.valid)
    {
        metadata->location.latitude = meta->location.latitude;
        metadata->location.longitude = meta->location.longitude;
        metadata->location.altitude = meta->location.altitude;
        metadata->location.svCount = meta->location.svCount;
    }
    //TODO speed xyz to NED?
    metadata->groundSpeed.north = meta->speed.x;
    metadata->groundSpeed.east = meta->speed.y;
    metadata->groundSpeed.down = meta->speed.z;
    metadata->airSpeed = -1.;
    metadata->droneAttitude.phi = meta->droneAttitude.roll;
    metadata->droneAttitude.theta = meta->droneAttitude.pitch;
    metadata->droneAttitude.psi = meta->droneAttitude.yaw;
    pdraw_euler2quat(&metadata->droneAttitude, &metadata->droneQuat);
    metadata->frameQuat.w = meta->frameQuat.w;
    metadata->frameQuat.x = meta->frameQuat.x;
    metadata->frameQuat.y = meta->frameQuat.y;
    metadata->frameQuat.z = meta->frameQuat.z;
    pdraw_quat2euler(&metadata->frameQuat, &metadata->frameOrientation);
    metadata->cameraPan = meta->cameraPan;
    metadata->cameraTilt = meta->cameraTilt;
    metadata->exposureTime = meta->exposureTime;
    metadata->gain = meta->gain;
    metadata->flyingState = meta->state;
    metadata->binning = meta->binning;
    metadata->pilotingMode = meta->mode;
    metadata->animation = meta->animation;
    metadata->wifiRssi = meta->wifiRssi;
    metadata->batteryPercentage = meta->batteryPercentage;
}


static void mapFrameMetadataV1strmbasic(const struct vmeta_v1_streaming_basic *meta, frame_metadata_t *metadata)
{
    metadata->location.isValid = false;
    metadata->droneAttitude.phi = meta->droneAttitude.roll;
    metadata->droneAttitude.theta = meta->droneAttitude.pitch;
    metadata->droneAttitude.psi = meta->droneAttitude.yaw;
    pdraw_euler2quat(&metadata->droneAttitude, &metadata->droneQuat);
    metadata->frameQuat.w = meta->frameQuat.w;
    metadata->frameQuat.x = meta->frameQuat.x;
    metadata->frameQuat.y = meta->frameQuat.y;
    metadata->frameQuat.z = meta->frameQuat.z;
    pdraw_quat2euler(&metadata->frameQuat, &metadata->frameOrientation);
    metadata->cameraPan = meta->cameraPan;
    metadata->cameraTilt = meta->cameraTilt;
    metadata->exposureTime = meta->exposureTime;
    metadata->gain = meta->gain;
    metadata->wifiRssi = meta->wifiRssi;
    metadata->batteryPercentage = meta->batteryPercentage;
}


static void mapFrameMetadataV2(const struct vmeta_v2 *meta, frame_metadata_t *metadata)
{
    metadata->groundDistance = meta->base.groundDistance;
    metadata->location.isValid = (meta->base.location.valid) ? true : false;
    if (meta->base.location.valid)
    {
        metadata->location.latitude = meta->base.location.latitude;
        metadata->location.longitude = meta->base.location.longitude;
        metadata->location.altitude = meta->base.location.altitude;
        metadata->location.svCount = meta->base.location.svCount;
    }
    metadata->groundSpeed.north = meta->base.speed.north;
    metadata->groundSpeed.east = meta->base.speed.east;
    metadata->groundSpeed.down = meta->base.speed.down;
    metadata->airSpeed = meta->base.airSpeed;
    metadata->droneQuat.w = meta->base.droneQuat.w;
    metadata->droneQuat.x = meta->base.droneQuat.x;
    metadata->droneQuat.y = meta->base.droneQuat.y;
    metadata->droneQuat.z = meta->base.droneQuat.z;
    pdraw_quat2euler(&metadata->droneQuat, &metadata->droneAttitude);
    metadata->frameQuat.w = meta->base.frameQuat.w;
    metadata->frameQuat.x = meta->base.frameQuat.x;
    metadata->frameQuat.y = meta->base.frameQuat.y;
    metadata->frameQuat.z = meta->base.frameQuat.z;
    pdraw_quat2euler(&metadata->frameQuat, &metadata->frameOrientation);
    metadata->cameraPan = meta->base.cameraPan;
    metadata->cameraTilt = meta->base.cameraTilt;
    metadata->exposureTime = meta->base.exposureTime;
    metadata->gain = meta->base.gain;
    metadata->flyingState = meta->base.state;
    metadata->binning = meta->base.binning;
    metadata->pilotingMode = meta->base.mode;
    metadata->animation = meta->base.animation;
    metadata->wifiRssi = meta->base.wifiRssi;
    metadata->batteryPercentage = meta->base.batteryPercentage;
}


bool Metadata::decodeFrameMetadata(const void *metadataBuffer, unsigned int metadataSize,
                                   frame_metadata_source_t source, const char *mimeType, frame_metadata_t *metadata)
{
    bool ret = false;
    struct pomp_buffer *buf;
    size_t pos = 0;
    int err;

    if ((!metadataBuffer) || (!metadataSize) || (!metadata))
    {
        return false;
    }

    memset(metadata, 0, sizeof(frame_metadata_t));

    buf = pomp_buffer_new_with_data(metadataBuffer, metadataSize);
    if (!buf)
    {
        ULOGE("Metadata: buffer allocation failed");
        return false;
    }

    if (source == FRAME_METADATA_SOURCE_RECORDING)
    {
        struct vmeta_recording rmeta;
        err = vmeta_recording_read(buf, &pos, &rmeta, mimeType);
        if (err != 0)
        {
            ULOGE("Metadata: vmeta_recording_read() failed (%d: '%s')", err, strerror(err));
        }
        else
        {
            if (rmeta.type == VMETA_RECORDING_TYPE_V2)
            {
                mapFrameMetadataV2(&rmeta.v2, metadata);
                ret = true;
            }
            else if (rmeta.type == VMETA_RECORDING_TYPE_V1)
            {
                mapFrameMetadataV1rec(&rmeta.v1, metadata);
                ret = true;
            }
            else
            {
                ULOGW("Metadata: invalid metadata type %d", rmeta.type);
            }
        }
    }
    else
    {
        struct vmeta_streaming smeta;
        err = vmeta_streaming_read(buf, &pos, &smeta);
        if (err != 0)
        {
            ULOGE("Metadata: vmeta_streaming_read() failed (%d: '%s')", err, strerror(err));
        }
        else
        {
            if (smeta.type == VMETA_STREAMING_TYPE_V2)
            {
                mapFrameMetadataV2(&smeta.v2, metadata);
                ret = true;
            }
            else if (smeta.type == VMETA_STREAMING_TYPE_V1_EXTENDED)
            {
                mapFrameMetadataV1strmext(&smeta.v1_extended, metadata);
                ret = true;
            }
            else if (smeta.type == VMETA_STREAMING_TYPE_V1_BASIC)
            {
                mapFrameMetadataV1strmbasic(&smeta.v1_basic, metadata);
                ret = true;
            }
            else
            {
                ULOGW("Metadata: invalid metadata type %d", smeta.type);
            }
        }
    }

    pomp_buffer_unref(buf);

    return ret;
}

}
