/**
 * @file pdraw_defs.h
 * @brief Parrot Drones Awesome Video Viewer Library - common definitions
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

#ifndef _PDRAW_DEFS_H_
#define _PDRAW_DEFS_H_

#include <inttypes.h>


typedef enum
{
    PDRAW_DRONE_MODEL_UNKNOWN = 0,
    PDRAW_DRONE_MODEL_BEBOP,
    PDRAW_DRONE_MODEL_BEBOP2,
    PDRAW_DRONE_MODEL_DISCO,

} pdraw_drone_model_t;


typedef enum
{
    PDRAW_SESSION_TYPE_UNKNOWN = 0,
    PDRAW_SESSION_TYPE_STREAM,
    PDRAW_SESSION_TYPE_RECORD,

} pdraw_session_type_t;


typedef enum
{
    PDRAW_MEDIA_TYPE_UNKNOWN = 0,
    PDRAW_MEDIA_TYPE_VIDEO,

} pdraw_media_type_t;


typedef enum
{
    PDRAW_COLOR_FORMAT_UNKNOWN = 0,
    PDRAW_COLOR_FORMAT_YUV420PLANAR,
    PDRAW_COLOR_FORMAT_YUV420SEMIPLANAR,

} pdraw_color_format_t;


typedef enum
{
    PDRAW_VIDEO_TYPE_DEFAULT_CAMERA = 0,
    PDRAW_VIDEO_TYPE_FRONT_CAMERA = 0,
    PDRAW_VIDEO_TYPE_VERTICAL_CAMERA,

} pdraw_video_type_t;


typedef enum
{
    PDRAW_FLYING_STATE_LANDED = 0,
    PDRAW_FLYING_STATE_TAKINGOFF,
    PDRAW_FLYING_STATE_HOVERING,
    PDRAW_FLYING_STATE_FLYING,
    PDRAW_FLYING_STATE_LANDING,
    PDRAW_FLYING_STATE_EMERGENCY,

} pdraw_flying_state_t;


typedef enum
{
    PDRAW_PILOTING_MODE_MANUAL = 0,
    PDRAW_PILOTING_MODE_RETURN_HOME,
    PDRAW_PILOTING_MODE_FLIGHT_PLAN,
    PDRAW_PILOTING_MODE_FOLLOW_ME,

} pdraw_piloting_mode_t;


typedef enum
{
    PDRAW_FOLLOWME_ANIM_NONE = 0,
    PDRAW_FOLLOWME_ANIM_ORBIT,
    PDRAW_FOLLOWME_ANIM_BOOMERANG,
    PDRAW_FOLLOWME_ANIM_PARABOLA,
    PDRAW_FOLLOWME_ANIM_ZENITH,

} pdraw_followme_anim_t;


typedef struct
{
    pdraw_video_type_t type;
    unsigned int width;
    unsigned int height;
    unsigned int cropLeft;
    unsigned int cropRight;
    unsigned int cropTop;
    unsigned int cropBottom;
    unsigned int croppedWidth;
    unsigned int croppedHeight;
    unsigned int sarWidth;
    unsigned int sarHeight;
    float horizontalFov;
    float verticalFov;

} pdraw_video_info_t;

typedef union
{
  pdraw_video_info_t video;
} pdraw_media_union_t;

typedef struct
{
    pdraw_media_type_t type;
    unsigned int id;
    pdraw_media_union_t info;

} pdraw_media_info_t;


typedef struct
{
    int isValid;
    double latitude;
    double longitude;
    double altitude;
    uint8_t svCount;

} pdraw_location_t;


typedef struct
{
    float w;
    float x;
    float y;
    float z;

} pdraw_quaternion_t;


typedef struct
{
    float phi;      // roll
    float theta;    // pitch
    float psi;      // yaw

} pdraw_euler_t;


typedef struct
{
    float north;
    float east;
    float down;

} pdraw_speed_t;


typedef struct
{
    pdraw_location_t location;
    float groundDistance;
    pdraw_speed_t groundSpeed;
    float airSpeed;
    pdraw_quaternion_t droneQuat;
    pdraw_euler_t droneAttitude;
    pdraw_quaternion_t frameQuat;
    pdraw_euler_t frameOrientation;
    float cameraPan;
    float cameraTilt;
    float exposureTime;
    int gain;
    pdraw_flying_state_t flyingState;
    int binning;
    pdraw_piloting_mode_t pilotingMode;
    int animation;
    int wifiRssi;
    int batteryPercentage;
    uint64_t frameTimestamp;
    int followMeEnabled;
    int followMeMode;
    int followMeAngleLocked;
    pdraw_followme_anim_t followMeAnimation;
    pdraw_location_t followMeTargetLocation;

} pdraw_video_frame_metadata_t;


typedef struct
{
    pdraw_color_format_t colorFormat;
    uint8_t *plane[3];
    unsigned int stride[3];
    unsigned int width;
    unsigned int height;
    unsigned int sarWidth;
    unsigned int sarHeight;
    int isComplete;
    int hasErrors;
    int isRef;
    uint64_t auNtpTimestamp;
    uint64_t auNtpTimestampRaw;
    uint64_t auNtpTimestampLocal;
    int hasMetadata;
    pdraw_video_frame_metadata_t metadata;

} pdraw_video_frame_t;


typedef void (*pdraw_video_frame_filter_callback_t)(void *filterCtx, const pdraw_video_frame_t *frame, void *userPtr);


#endif /* !_PDRAW_DEFS_H_ */
