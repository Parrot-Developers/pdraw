/**
 * @file pdraw_metadata_videoframe.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - video frame metadata
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

#include "pdraw_metadata_videoframe.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


static void mapFrameMetadataV1rec(const struct vmeta_frame_v1_recording *meta, struct pdraw_video_frame_metadata *metadata)
{
    metadata->groundDistance = meta->altitude;
    metadata->location.isValid = (meta->location.valid) ? 1 : 0;
    if (meta->location.valid)
    {
        metadata->location.latitude = meta->location.latitude;
        metadata->location.longitude = meta->location.longitude;
        metadata->location.altitude = meta->location.altitude;
        metadata->location.svCount = meta->location.svCount;
    }
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
    switch (meta->state)
    {
    default:
    case VMETA_FLYING_STATE_LANDED:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDED;
        break;
    case VMETA_FLYING_STATE_TAKINGOFF:
        metadata->flyingState = PDRAW_FLYING_STATE_TAKINGOFF;
        break;
    case VMETA_FLYING_STATE_HOVERING:
        metadata->flyingState = PDRAW_FLYING_STATE_HOVERING;
        break;
    case VMETA_FLYING_STATE_FLYING:
        metadata->flyingState = PDRAW_FLYING_STATE_FLYING;
        break;
    case VMETA_FLYING_STATE_LANDING:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDING;
        break;
    case VMETA_FLYING_STATE_EMERGENCY:
        metadata->flyingState = PDRAW_FLYING_STATE_EMERGENCY;
        break;
    }
    metadata->binning = meta->binning;
    switch (meta->mode)
    {
    default:
    case VMETA_PILOTING_MODE_MANUAL:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_MANUAL;
        break;
    case VMETA_PILOTING_MODE_RETURN_HOME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_RETURN_HOME;
        break;
    case VMETA_PILOTING_MODE_FLIGHT_PLAN:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FLIGHT_PLAN;
        break;
    case VMETA_PILOTING_MODE_FOLLOW_ME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FOLLOW_ME;
        break;
    }
    metadata->animation = meta->animation;
    metadata->wifiRssi = meta->wifiRssi;
    metadata->batteryPercentage = meta->batteryPercentage;
}


static void mapFrameMetadataV1strmext(const struct vmeta_frame_v1_streaming_extended *meta, struct pdraw_video_frame_metadata *metadata)
{
    metadata->groundDistance = meta->altitude;
    metadata->location.isValid = (meta->location.valid) ? 1 : 0;
    if (meta->location.valid)
    {
        metadata->location.latitude = meta->location.latitude;
        metadata->location.longitude = meta->location.longitude;
        metadata->location.altitude = meta->location.altitude;
        metadata->location.svCount = meta->location.svCount;
    }
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
    switch (meta->state)
    {
    default:
    case VMETA_FLYING_STATE_LANDED:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDED;
        break;
    case VMETA_FLYING_STATE_TAKINGOFF:
        metadata->flyingState = PDRAW_FLYING_STATE_TAKINGOFF;
        break;
    case VMETA_FLYING_STATE_HOVERING:
        metadata->flyingState = PDRAW_FLYING_STATE_HOVERING;
        break;
    case VMETA_FLYING_STATE_FLYING:
        metadata->flyingState = PDRAW_FLYING_STATE_FLYING;
        break;
    case VMETA_FLYING_STATE_LANDING:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDING;
        break;
    case VMETA_FLYING_STATE_EMERGENCY:
        metadata->flyingState = PDRAW_FLYING_STATE_EMERGENCY;
        break;
    }
    metadata->binning = meta->binning;
    switch (meta->mode)
    {
    default:
    case VMETA_PILOTING_MODE_MANUAL:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_MANUAL;
        break;
    case VMETA_PILOTING_MODE_RETURN_HOME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_RETURN_HOME;
        break;
    case VMETA_PILOTING_MODE_FLIGHT_PLAN:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FLIGHT_PLAN;
        break;
    case VMETA_PILOTING_MODE_FOLLOW_ME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FOLLOW_ME;
        break;
    }
    metadata->animation = meta->animation;
    metadata->wifiRssi = meta->wifiRssi;
    metadata->batteryPercentage = meta->batteryPercentage;
}


static void mapFrameMetadataV1strmbasic(const struct vmeta_frame_v1_streaming_basic *meta, struct pdraw_video_frame_metadata *metadata)
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


static void mapFrameMetadataV2(const struct vmeta_frame_v2 *meta, struct pdraw_video_frame_metadata *metadata)
{
    metadata->groundDistance = meta->base.groundDistance;
    metadata->location.isValid = (meta->base.location.valid) ? 1 : 0;
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
    switch (meta->base.state)
    {
    default:
    case VMETA_FLYING_STATE_LANDED:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDED;
        break;
    case VMETA_FLYING_STATE_TAKINGOFF:
        metadata->flyingState = PDRAW_FLYING_STATE_TAKINGOFF;
        break;
    case VMETA_FLYING_STATE_HOVERING:
        metadata->flyingState = PDRAW_FLYING_STATE_HOVERING;
        break;
    case VMETA_FLYING_STATE_FLYING:
        metadata->flyingState = PDRAW_FLYING_STATE_FLYING;
        break;
    case VMETA_FLYING_STATE_LANDING:
        metadata->flyingState = PDRAW_FLYING_STATE_LANDING;
        break;
    case VMETA_FLYING_STATE_EMERGENCY:
        metadata->flyingState = PDRAW_FLYING_STATE_EMERGENCY;
        break;
    }
    metadata->binning = meta->base.binning;
    switch (meta->base.mode)
    {
    default:
    case VMETA_PILOTING_MODE_MANUAL:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_MANUAL;
        break;
    case VMETA_PILOTING_MODE_RETURN_HOME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_RETURN_HOME;
        break;
    case VMETA_PILOTING_MODE_FLIGHT_PLAN:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FLIGHT_PLAN;
        break;
    case VMETA_PILOTING_MODE_FOLLOW_ME:
        metadata->pilotingMode = PDRAW_PILOTING_MODE_FOLLOW_ME;
        break;
    }
    metadata->animation = meta->base.animation;
    metadata->wifiRssi = meta->base.wifiRssi;
    metadata->batteryPercentage = meta->base.batteryPercentage;
    if (meta->has_timestamp)
    {
        metadata->frameTimestamp = meta->timestamp.frameTimestamp;
    }
    if (meta->has_followme)
    {
        metadata->followMeEnabled = meta->followme.enabled;
        metadata->followMeMode = meta->followme.mode;
        metadata->followMeAngleLocked = meta->followme.angleLocked;
        switch (meta->followme.animation)
        {
        default:
        case VMETA_FOLLOWME_ANIM_NONE:
            metadata->followMeAnimation = PDRAW_FOLLOWME_ANIM_NONE;
            break;
        case VMETA_FOLLOWME_ANIM_ORBIT:
            metadata->followMeAnimation = PDRAW_FOLLOWME_ANIM_ORBIT;
            break;
        case VMETA_FOLLOWME_ANIM_BOOMERANG:
            metadata->followMeAnimation = PDRAW_FOLLOWME_ANIM_BOOMERANG;
            break;
        case VMETA_FOLLOWME_ANIM_PARABOLA:
            metadata->followMeAnimation = PDRAW_FOLLOWME_ANIM_PARABOLA;
            break;
        case VMETA_FOLLOWME_ANIM_ZENITH:
            metadata->followMeAnimation = PDRAW_FOLLOWME_ANIM_ZENITH;
            break;
        }
        metadata->followMeTargetLocation.isValid = (meta->followme.target.valid) ? 1 : 0;
        if (meta->followme.target.valid)
        {
            metadata->followMeTargetLocation.latitude = meta->followme.target.latitude;
            metadata->followMeTargetLocation.longitude = meta->followme.target.longitude;
            metadata->followMeTargetLocation.altitude = meta->followme.target.altitude;
            metadata->followMeTargetLocation.svCount = meta->followme.target.svCount;
        }
    }
}


bool VideoFrameMetadata::decodeMetadata(const void *metadataBuffer, unsigned int metadataSize,
                                        video_frame_metadata_source_t source, const char *mimeType, struct pdraw_video_frame_metadata *metadata)
{
    bool ret = false;
    struct vmeta_buffer buf;
    int err;
    struct vmeta_frame meta;

    if ((!metadataBuffer) || (!metadataSize) || (!metadata))
    {
        return false;
    }

    memset(metadata, 0, sizeof(struct pdraw_video_frame_metadata));
    vmeta_buffer_set_cdata(&buf, (const uint8_t *)metadataBuffer, metadataSize, 0);

    if (source == FRAME_METADATA_SOURCE_STREAMING)
    {
        mimeType = NULL;
    }

    err = vmeta_frame_read(&buf, &meta, mimeType);
    if (err != 0)
    {
        ULOGE("VideoFrameMetadata: vmeta_frame_read() failed (%d: '%s')", err, strerror(err));
    }
    else
    {
        switch (meta.type)
        {
        case VMETA_FRAME_TYPE_V2:
            mapFrameMetadataV2(&meta.v2, metadata);
            ret = true;
            break;
        case VMETA_FRAME_TYPE_V1_RECORDING:
            mapFrameMetadataV1rec(&meta.v1_rec, metadata);
            ret = true;
            break;
        case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
            mapFrameMetadataV1strmext(&meta.v1_strm_ext, metadata);
            ret = true;
            break;
        case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
            mapFrameMetadataV1strmbasic(&meta.v1_strm_basic, metadata);
            ret = true;
            break;
        default:
            ULOGW("VideoFrameMetadata: invalid metadata type %d", meta.type);
            break;
        }
    }

    return ret;
}

}
