/**
 * Parrot Drones Awesome Video Viewer Library
 * Utilities
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

#ifndef _PDRAW_UTILS_HPP_
#define _PDRAW_UTILS_HPP_

#include <inttypes.h>
#include <string.h>

#include <pdraw/pdraw_defs.h>

#include <Eigen/Eigen>


#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


void pdraw_gaussianDistribution(float *samples,
				unsigned int sampleCount,
				float sigma);


void pdraw_friendlyTimeFromUs(uint64_t time,
			      unsigned int *hrs,
			      unsigned int *min,
			      unsigned int *sec,
			      unsigned int *msec);


const char *pdraw_droneModelStr(enum pdraw_drone_model val);


const char *pdraw_hmdModelStr(enum pdraw_hmd_model val);


const char *pdraw_pipelineModeStr(enum pdraw_pipeline_mode val);


const char *pdraw_sessionTypeStr(enum pdraw_session_type val);


const char *pdraw_mediaTypeStr(enum pdraw_media_type val);


const char *pdraw_videoMediaFormatStr(enum pdraw_video_media_format val);


const char *pdraw_yuvFormatStr(enum pdraw_yuv_format val);


const char *pdraw_h264FormatStr(enum pdraw_h264_format val);


const char *pdraw_videoTypeStr(enum pdraw_video_type val);


const char *pdraw_histogramChannelStr(enum pdraw_histogram_channel val);


const char *
pdraw_videoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val);


const char *pdraw_videoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val);


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *md,
			      struct json_object *jobj);


int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *mdata,
				 char *output,
				 unsigned int len);


int pdraw_packYUVFrame(const struct pdraw_video_frame *in_frame,
		       struct pdraw_video_frame *out_frame,
		       struct vbuf_buffer *out_buf);


static inline char *xstrdup(const char *s)
{
	return s == NULL ? NULL : strdup(s);
}


static inline int xstrcmp(const char *s1, const char *s2)
{
	if (s1 == NULL && s2 == NULL)
		return 0;
	else if (s1 == NULL)
		return -1;
	else if (s2 == NULL)
		return 1;
	return strcmp(s1, s2);
}

#endif /* !_PDRAW_UTILS_HPP_ */
