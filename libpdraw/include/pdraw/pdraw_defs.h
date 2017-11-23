/**
 * Parrot Drones Awesome Video Viewer Library
 * Common definitions
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

#ifndef _PDRAW_DEFS_H_
#define _PDRAW_DEFS_H_

#include <inttypes.h>
#include <video-metadata/vmeta.h>


#define PDRAW_PLAY_SPEED_MAX        1000.f


enum pdraw_drone_model {
	PDRAW_DRONE_MODEL_UNKNOWN = 0,
	PDRAW_DRONE_MODEL_BEBOP,
	PDRAW_DRONE_MODEL_BEBOP2,
	PDRAW_DRONE_MODEL_DISCO,
};


enum pdraw_hmd_model {
	PDRAW_HMD_MODEL_UNKNOWN = 0,
	PDRAW_HMD_MODEL_COCKPITGLASSES = 0,
	PDRAW_HMD_MODEL_COCKPITGLASSES_2,
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


union pdraw_media_info_union {
	struct pdraw_video_info video;
};


struct pdraw_media_info {
	enum pdraw_media_type type;
	unsigned int id;
	union pdraw_media_info_union info;
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
	struct vmeta_frame_v2 metadata;
	uint8_t *userData;
	unsigned int userDataSize;
};


typedef void (*pdraw_video_frame_filter_callback_t)(
	void *filterCtx,
	const struct pdraw_video_frame *frame,
	void *userPtr);


#endif /* !_PDRAW_DEFS_H_ */
