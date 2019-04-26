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

#include "pdraw_utils.hpp"

#include <math.h>
#include <string.h>

#ifdef BUILD_JSON
#	include <json-c/json.h>
#endif
#define ULOG_TAG pdraw_utils
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_utils);


/* Approximation without the Simphson integration;
 * see http://dev.theomader.com/gaussian-kernel-calculator/ */
void pdraw_gaussianDistribution(float *samples,
				unsigned int sampleCount,
				float sigma)
{
	unsigned int i;
	float a, x, g, start, step, sum;

	if (samples == NULL)
		return;

	if (sampleCount == 0)
		return;
	if (!(sampleCount & 1))
		sampleCount--;
	if (sampleCount == 0)
		return;

	a = 1.f / (sqrtf(2.f * M_PI) * sigma);
	start = -(float)sampleCount / 2.f;
	step = (sampleCount > 1)
		       ? (float)sampleCount / ((float)sampleCount - 1.f)
		       : 0.f;

	/* Compute coefs */
	for (i = 0, x = start, sum = 0.f; i < (sampleCount + 1) / 2;
	     i++, x += step) {
		g = expf(-x * x / (2.f * sigma * sigma)) * a;
		sum += (i == sampleCount / 2) ? g : g * 2.f;
		/* Array is symmetric so write to one side of the array only */
		*(samples + i) = g;
	}

	/* Normalization */
	for (i = 0; i < (sampleCount + 1) / 2; i++) {
		g = *(samples + i) / sum;
		/* Array is symmetric so write to both sides of the array
		 * (the middle value is written twice but it doesn't matter) */
		*(samples + i) = g;
		*(samples + sampleCount - 1 - i) = g;
	}
}


void pdraw_friendlyTimeFromUs(uint64_t time,
			      unsigned int *hrs,
			      unsigned int *min,
			      unsigned int *sec,
			      unsigned int *msec)
{
	unsigned int _hrs =
		(unsigned int)((time + 500) / 1000 / 60 / 60) / 1000;
	unsigned int _min =
		(unsigned int)((time + 500) / 1000 / 60 - _hrs * 60000) / 1000;
	unsigned int _sec = (unsigned int)((time + 500) / 1000 -
					   _hrs * 60 * 60000 - _min * 60000) /
			    1000;
	unsigned int _msec =
		(unsigned int)((time + 500) / 1000 - _hrs * 60 * 60000 -
			       _min * 60000 - _sec * 1000);
	if (hrs)
		*hrs = _hrs;
	if (min)
		*min = _min;
	if (sec)
		*sec = _sec;
	if (msec)
		*msec = _msec;
}


const char *pdraw_droneModelStr(enum pdraw_drone_model val)
{
	switch (val) {
	default:
	case PDRAW_DRONE_MODEL_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_DRONE_MODEL_BEBOP:
		return "BEBOP";
	case PDRAW_DRONE_MODEL_BEBOP2:
		return "BEBOP2";
	case PDRAW_DRONE_MODEL_DISCO:
		return "DISCO";
	case PDRAW_DRONE_MODEL_BLUEGRASS:
		return "BLUEGRASS";
	case PDRAW_DRONE_MODEL_ANAFI:
		return "ANAFI";
	case PDRAW_DRONE_MODEL_ANAFI_THERMAL:
		return "ANAFI_THERMAL";
	}
}


const char *pdraw_hmdModelStr(enum pdraw_hmd_model val)
{
	switch (val) {
	default:
	case PDRAW_HMD_MODEL_COCKPITGLASSES:
		return "COCKPITGLASSES";
	case PDRAW_HMD_MODEL_COCKPITGLASSES_2:
		return "COCKPITGLASSES_2";
	}
}


const char *pdraw_pipelineModeStr(enum pdraw_pipeline_mode val)
{
	switch (val) {
	default:
	case PDRAW_PIPELINE_MODE_DECODE_ALL:
		return "DECODE_ALL";
	case PDRAW_PIPELINE_MODE_DECODE_NONE:
		return "DECODE_NONE";
	}
}


const char *pdraw_sessionTypeStr(enum pdraw_session_type val)
{
	switch (val) {
	default:
	case PDRAW_SESSION_TYPE_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_SESSION_TYPE_LIVE:
		return "LIVE";
	case PDRAW_SESSION_TYPE_REPLAY:
		return "REPLAY";
	}
}


const char *pdraw_mediaTypeStr(enum pdraw_media_type val)
{
	switch (val) {
	default:
	case PDRAW_MEDIA_TYPE_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_MEDIA_TYPE_VIDEO:
		return "VIDEO";
	}
}


const char *pdraw_videoMediaFormatStr(enum pdraw_video_media_format val)
{
	switch (val) {
	default:
	case PDRAW_VIDEO_MEDIA_FORMAT_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_VIDEO_MEDIA_FORMAT_YUV:
		return "YUV";
	case PDRAW_VIDEO_MEDIA_FORMAT_H264:
		return "H264";
	case PDRAW_VIDEO_MEDIA_FORMAT_OPAQUE:
		return "OPAQUE";
	}
}


const char *pdraw_yuvFormatStr(enum pdraw_yuv_format val)
{
	switch (val) {
	default:
	case PDRAW_YUV_FORMAT_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_YUV_FORMAT_I420:
		return "I420";
	case PDRAW_YUV_FORMAT_NV12:
		return "NV12";
	}
}


const char *pdraw_h264FormatStr(enum pdraw_h264_format val)
{
	switch (val) {
	default:
	case PDRAW_H264_FORMAT_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_H264_FORMAT_BYTE_STREAM:
		return "BYTE_STREAM";
	case PDRAW_H264_FORMAT_AVCC:
		return "AVCC";
	}
}


const char *pdraw_videoTypeStr(enum pdraw_video_type val)
{
	switch (val) {
	default:
	case PDRAW_VIDEO_TYPE_DEFAULT_CAMERA:
		return "DEFAULT_CAMERA";
	}
}


const char *pdraw_histogramChannelStr(enum pdraw_histogram_channel val)
{
	switch (val) {
	default:
		return "UNKNOWN";
	case PDRAW_HISTOGRAM_CHANNEL_RED:
		return "RED";
	case PDRAW_HISTOGRAM_CHANNEL_GREEN:
		return "GREEN";
	case PDRAW_HISTOGRAM_CHANNEL_BLUE:
		return "BLUE";
	case PDRAW_HISTOGRAM_CHANNEL_LUMA:
		return "LUMA";
	}
}


const char *
pdraw_videoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val)
{
	switch (val) {
	default:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT:
		return "FIT";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_CROP:
		return "CROP";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP:
		return "FIT_PAD_BLUR_CROP";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND:
		return "FIT_PAD_BLUR_EXTEND";
	}
}


const char *pdraw_videoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val)
{
	switch (val) {
	default:
		return "NONE";
	case PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS:
		return "SOS";
	case PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS:
		return "EOS";
	case PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE:
		return "RECONFIGURE";
	case PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT:
		return "TIMEOUT";
	case PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER:
		return "PHOTO_TRIGGER";
	}
}


#ifdef BUILD_JSON
static void jsonFillInYuvInfo(struct json_object *jobj,
			      const struct pdraw_video_yuv_frame *yuv)
{
	json_object_object_add(
		jobj, "format", json_object_new_int(yuv->format));
	json_object_object_add(jobj, "width", json_object_new_int(yuv->width));
	json_object_object_add(
		jobj, "height", json_object_new_int(yuv->height));
	json_object_object_add(
		jobj, "sar_width", json_object_new_int(yuv->sar_width));
	json_object_object_add(
		jobj, "sar_height", json_object_new_int(yuv->sar_height));
	json_object_object_add(
		jobj, "crop_left", json_object_new_int(yuv->crop_left));
	json_object_object_add(
		jobj, "crop_top", json_object_new_int(yuv->crop_top));
	json_object_object_add(
		jobj, "crop_width", json_object_new_int(yuv->crop_width));
	json_object_object_add(
		jobj, "crop_height", json_object_new_int(yuv->crop_height));
}


static void jsonFillInH264Info(struct json_object *jobj,
			       const struct pdraw_video_h264_frame *h264)
{
	json_object_object_add(
		jobj, "format", json_object_new_int(h264->format));
	json_object_object_add(
		jobj, "is_complete", json_object_new_int(h264->is_complete));
	json_object_object_add(
		jobj, "is_sync", json_object_new_int(h264->is_sync));
	json_object_object_add(
		jobj, "is_ref", json_object_new_int(h264->is_ref));
}


int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *md,
				 char *output,
				 unsigned int len)
{
	if (!md || !output)
		return -EINVAL;

	const char *jstr;
	struct json_object *jobj = json_object_new_object();
	int ret = pdraw_frameMetadataToJson(md, jobj);
	if (ret < 0)
		goto out;

	jstr = json_object_to_json_string(jobj);
	if (strlen(jstr) + 1 > len) {
		ret = -ENOBUFS;
		goto out;
	}
	strcpy(output, jstr);

out:
	json_object_put(jobj);
	return ret;
}


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *md,
			      struct json_object *jobj)
{
	if (!md || !jobj)
		return -EINVAL;

	int ret = 0;
	struct json_object *jobj_yuv = NULL;
	struct json_object *jobj_h264 = NULL;
	struct json_object *jobj_vmeta = NULL;

	switch (md->format) {
	case PDRAW_VIDEO_MEDIA_FORMAT_YUV:
		json_object_object_add(
			jobj, "format", json_object_new_string("yuv"));
		jobj_yuv = json_object_new_object();
		jsonFillInYuvInfo(jobj_yuv, &md->yuv);
		json_object_object_add(jobj, "yuv", jobj_yuv);
		break;

	case PDRAW_VIDEO_MEDIA_FORMAT_H264:
		json_object_object_add(
			jobj, "format", json_object_new_string("h264"));
		jobj_h264 = json_object_new_object();
		jsonFillInH264Info(jobj_h264, &md->h264);
		json_object_object_add(jobj, "h264", jobj_h264);
		break;

	default:
		ULOGW("Format %d not supported", md->format);
		break;
	}

	json_object_object_add(
		jobj, "has_errors", json_object_new_int(md->has_errors));
	json_object_object_add(
		jobj, "is_silent", json_object_new_int(md->is_silent));
	json_object_object_add(jobj,
			       "ntp_timestamp",
			       json_object_new_int64(md->ntp_timestamp));
	json_object_object_add(
		jobj,
		"ntp_unskewed_timestamp",
		json_object_new_int64(md->ntp_unskewed_timestamp));
	json_object_object_add(jobj,
			       "ntp_raw_timestamp",
			       json_object_new_int64(md->ntp_raw_timestamp));
	json_object_object_add(
		jobj,
		"ntp_raw_unskewed_timestamp",
		json_object_new_int64(md->ntp_raw_unskewed_timestamp));
	json_object_object_add(jobj,
			       "play_timestamp",
			       json_object_new_int64(md->play_timestamp));
	json_object_object_add(jobj,
			       "capture_timestamp",
			       json_object_new_int64(md->capture_timestamp));
	json_object_object_add(jobj,
			       "local_timestamp",
			       json_object_new_int64(md->local_timestamp));
	if (md->has_metadata) {
		jobj_vmeta = json_object_new_object();
		int ret = vmeta_frame_to_json(&md->metadata, jobj_vmeta);
		if (ret < 0) {
			if (jobj_vmeta != NULL)
				json_object_put(jobj_vmeta);
			return ret;
		}
		json_object_object_add(jobj, "metadata", jobj_vmeta);
	}

	return ret;
}


#else /* BUILD_JSON undefined */

int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *,
				 char *,
				 unsigned int)
{
	ULOGW("%s not implemented", __func__);
	return -ENOSYS;
}


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *,
			      struct json_object *)
{
	ULOGW("%s not implemented", __func__);
	return -ENOSYS;
}

#endif /* BUILD_JSON */
