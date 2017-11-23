/**
 * Parrot Drones Awesome Video Viewer Library
 * Video frame metadata
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

#include <math.h>
#include <string.h>

#include "pdraw_metadata_videoframe.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


static void mapFrameMetadataV1rec(const struct vmeta_frame_v1_recording *in, struct vmeta_frame_v2 *out)
{
    memset(out, 0, sizeof(struct vmeta_frame_v2));
    out->base.groundDistance = in->altitude;
    out->base.location = in->location;
    /* TODO: speed frame should be converted to NED */
    out->base.speed.north = in->speed.x;
    out->base.speed.east = in->speed.y;
    out->base.speed.down = in->speed.z;
    out->base.airSpeed = -1.;
    pdraw_euler2quat(&in->droneAttitude, &out->base.droneQuat);
    out->base.frameQuat = in->frameQuat;
    out->base.cameraPan = in->cameraPan;
    out->base.cameraTilt = in->cameraTilt;
    out->base.exposureTime = in->exposureTime;
    out->base.gain = in->gain;
    out->base.state = in->state;
    out->base.binning = in->binning;
    out->base.mode = in->mode;
    out->base.animation = in->animation;
    out->base.wifiRssi = in->wifiRssi;
    out->base.batteryPercentage = in->batteryPercentage;
}


static void mapFrameMetadataV1strmext(const struct vmeta_frame_v1_streaming_extended *in, struct vmeta_frame_v2 *out)
{
    memset(out, 0, sizeof(struct vmeta_frame_v2));
    out->base.groundDistance = in->altitude;
    out->base.location = in->location;
    /* TODO: speed frame should be converted to NED */
    out->base.speed.north = in->speed.x;
    out->base.speed.east = in->speed.y;
    out->base.speed.down = in->speed.z;
    out->base.airSpeed = -1.;
    pdraw_euler2quat(&in->droneAttitude, &out->base.droneQuat);
    out->base.frameQuat = in->frameQuat;
    out->base.cameraPan = in->cameraPan;
    out->base.cameraTilt = in->cameraTilt;
    out->base.exposureTime = in->exposureTime;
    out->base.gain = in->gain;
    out->base.state = in->state;
    out->base.binning = in->binning;
    out->base.mode = in->mode;
    out->base.animation = in->animation;
    out->base.wifiRssi = in->wifiRssi;
    out->base.batteryPercentage = in->batteryPercentage;
}


static void mapFrameMetadataV1strmbasic(const struct vmeta_frame_v1_streaming_basic *in, struct vmeta_frame_v2 *out)
{
    memset(out, 0, sizeof(struct vmeta_frame_v2));
    out->base.location.valid = false;
    pdraw_euler2quat(&in->droneAttitude, &out->base.droneQuat);
    out->base.frameQuat = in->frameQuat;
    out->base.cameraPan = in->cameraPan;
    out->base.cameraTilt = in->cameraTilt;
    out->base.exposureTime = in->exposureTime;
    out->base.gain = in->gain;
    out->base.wifiRssi = in->wifiRssi;
    out->base.batteryPercentage = in->batteryPercentage;
}


static void mapFrameMetadataV2(const struct vmeta_frame_v2 *in, struct vmeta_frame_v2 *out)
{
    memcpy(out, in, sizeof(*out));
}


bool VideoFrameMetadata::decodeMetadata(const void *metadataBuffer, unsigned int metadataSize,
                                        video_frame_metadata_source_t source, const char *mimeType, struct vmeta_frame_v2 *metadata)
{
    bool ret = false;
    struct vmeta_buffer buf;
    int err;
    struct vmeta_frame meta;

    if ((!metadataBuffer) || (!metadataSize) || (!metadata))
    {
        return false;
    }

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
