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


enum pdraw_drone_model {
	PDRAW_DRONE_MODEL_UNKNOWN = 0,
	PDRAW_DRONE_MODEL_BEBOP,
	PDRAW_DRONE_MODEL_BEBOP2,
	PDRAW_DRONE_MODEL_DISCO,
};


enum pdraw_hmd_model {
	PDRAW_HMD_MODEL_UNKNOWN = 0,
	PDRAW_HMD_MODEL_COCKPIT_GLASSES = 0,
};


enum pdraw_session_type {
	PDRAW_SESSION_TYPE_UNKNOWN = 0,
	PDRAW_SESSION_TYPE_STREAM,
	PDRAW_SESSION_TYPE_RECORD,
};


enum pdraw_media_type {
	PDRAW_MEDIA_TYPE_UNKNOWN = 0,
	PDRAW_MEDIA_TYPE_VIDEO,
};


enum pdraw_color_format {
	PDRAW_COLOR_FORMAT_UNKNOWN = 0,
	PDRAW_COLOR_FORMAT_YUV420PLANAR,
	PDRAW_COLOR_FORMAT_YUV420SEMIPLANAR,
};


enum pdraw_video_type {
	PDRAW_VIDEO_TYPE_DEFAULT_CAMERA = 0,
	PDRAW_VIDEO_TYPE_FRONT_CAMERA = 0,
	PDRAW_VIDEO_TYPE_VERTICAL_CAMERA,
};


enum pdraw_flying_state {
	PDRAW_FLYING_STATE_LANDED = 0,
	PDRAW_FLYING_STATE_TAKINGOFF,
	PDRAW_FLYING_STATE_HOVERING,
	PDRAW_FLYING_STATE_FLYING,
	PDRAW_FLYING_STATE_LANDING,
	PDRAW_FLYING_STATE_EMERGENCY,
};


enum pdraw_piloting_mode {
	PDRAW_PILOTING_MODE_MANUAL = 0,
	PDRAW_PILOTING_MODE_RETURN_HOME,
	PDRAW_PILOTING_MODE_FLIGHT_PLAN,
	PDRAW_PILOTING_MODE_FOLLOW_ME,
};


enum pdraw_followme_anim {
	PDRAW_FOLLOWME_ANIM_NONE = 0,
	PDRAW_FOLLOWME_ANIM_ORBIT,
	PDRAW_FOLLOWME_ANIM_BOOMERANG,
	PDRAW_FOLLOWME_ANIM_PARABOLA,
	PDRAW_FOLLOWME_ANIM_ZENITH,
};


struct pdraw_video_info {
	enum pdraw_video_type type;
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
};


struct pdraw_media_info {
	enum pdraw_media_type type;
	unsigned int id;
	union {
		struct pdraw_video_info video;
	} info;
};


struct pdraw_location {
	int isValid;
	double latitude;
	double longitude;
	double altitude;
	uint8_t svCount;
};


struct pdraw_quaternion {
	float w;
	float x;
	float y;
	float z;
};


struct pdraw_euler {
	union {
		float roll;
		float phi;
	};
	union {
		float pitch;
		float theta;
	};
	union {
		float yaw;
		float psi;
	};
};


struct pdraw_speed {
	float north;
	float east;
	float down;
};


struct pdraw_video_frame_metadata {
	struct pdraw_location location;
	float groundDistance;
	struct pdraw_speed groundSpeed;
	float airSpeed;
	struct pdraw_quaternion droneQuat;
	struct pdraw_euler droneAttitude;
	struct pdraw_quaternion frameQuat;
	struct pdraw_euler frameOrientation;
	float cameraPan;
	float cameraTilt;
	float exposureTime;
	int gain;
	enum pdraw_flying_state flyingState;
	int binning;
	enum pdraw_piloting_mode pilotingMode;
	int animation;
	int wifiRssi;
	int batteryPercentage;
	uint64_t frameTimestamp;
	int followMeEnabled;
	int followMeMode;
	int followMeAngleLocked;
	enum pdraw_followme_anim followMeAnimation;
	struct pdraw_location followMeTargetLocation;
};


struct pdraw_video_frame {
	enum pdraw_color_format colorFormat;
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
	struct pdraw_video_frame_metadata metadata;
	uint8_t *userData;
	unsigned int userDataSize;
};


typedef void (*pdraw_video_frame_filter_callback_t)(
	void *filterCtx,
	const struct pdraw_video_frame *frame,
	void *userPtr);


#endif /* !_PDRAW_DEFS_H_ */
