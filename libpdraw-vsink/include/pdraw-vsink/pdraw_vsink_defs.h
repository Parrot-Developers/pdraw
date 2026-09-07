/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library - type definitions
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/* To be used for all public API */
#ifdef PDRAW_VSINK_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_VSINK_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_VSINK_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_VSINK_API_EXPORTS */
#	define PDRAW_VSINK_API
#endif /* !PDRAW_VSINK_API_EXPORTS */

#include <pdraw/pdraw_defs.h>


enum pdraw_vsink_video_media_type {
	PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW = 0,
	PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED,
};


struct pdraw_vsink_frame {
	/* Type of the video media */
	enum pdraw_vsink_video_media_type type;
	union {
		/* If type is PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW */
		struct mbuf_raw_video_frame *raw;
		/* If type is PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED */
		struct mbuf_coded_video_frame *coded;
	};
};


/* Instance parameters */
struct pdraw_vsink_params {
	/* Network URL or local file (mandatory) */
	const char *url;

	/* Playback synchronization mode.
	 * - PDRAW_PLAYBACK_MODE_REALTIME: Frames are delivered following the
	 * original stream timestamps (synchronized with the clock).
	 * - PDRAW_PLAYBACK_MODE_OFFLINE: Frames are delivered as fast as the
	 * demuxing and decoding pipeline allows, ignoring real-time pacing. */
	enum pdraw_playback_mode playback_mode;

	/* Camera type to select (optional; set to VMETA_CAMERA_TYPE_UNKNOWN to
	 * select the default camera) */
	enum vmeta_camera_type camera_type;

	/* Type of the video media */
	enum pdraw_vsink_video_media_type video_media_type;
};
