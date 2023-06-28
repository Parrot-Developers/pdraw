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

#define ULOG_TAG pdraw_utils
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_utils.hpp"

#include <math.h>
#include <string.h>

#ifdef BUILD_JSON
#	include <json-c/json.h>
#endif

extern "C" {
const char *PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME = "pdraw.video.frame";
}

/* Approximation without the Simphson integration;
 * see http://dev.theomader.com/gaussian-kernel-calculator/ */
void pdraw_gaussianDistribution(float *samples,
				unsigned int sampleCount,
				float sigma)
{
	unsigned int i;
	float a, x, g, start, step, sum;

	if (samples == nullptr)
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


const char *pdraw_playbackTypeStr(enum pdraw_playback_type val)
{
	switch (val) {
	default:
	case PDRAW_PLAYBACK_TYPE_UNKNOWN:
		return "UNKNOWN";
	case PDRAW_PLAYBACK_TYPE_LIVE:
		return "LIVE";
	case PDRAW_PLAYBACK_TYPE_REPLAY:
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


const char *pdraw_videoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val)
{
	switch (val) {
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP:
		return "ASAP";
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE:
		return "ADAPTIVE";
	case PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_MAX:
	default:
		return "UNKNOWN";
	}
}


const char *
pdraw_videoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val)
{
	switch (val) {
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT:
		return "FIT";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_CROP:
		return "CROP";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP:
		return "FIT_PAD_BLUR_CROP";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND:
		return "FIT_PAD_BLUR_EXTEND";
	case PDRAW_VIDEO_RENDERER_FILL_MODE_MAX:
	default:
		return "UNKNOWN";
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


const char *pdraw_vipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val)
{
	switch (val) {
	case PDRAW_VIPC_SOURCE_EOS_REASON_NONE:
		return "NONE";
	case PDRAW_VIPC_SOURCE_EOS_REASON_RESTART:
		return "RESTART";
	case PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION:
		return "CONFIGURATION";
	case PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT:
		return "TIMEOUT";
	default:
		return "UNKNOWN";
	}
}


#ifdef BUILD_JSON
static void jsonFillRawVideoInfo(struct json_object *jobj,
				 const struct vdef_raw_frame *frame)
{
	int ret;
	struct json_object *jobj_frame = json_object_new_object();
	if (jobj_frame == nullptr) {
		ULOG_ERRNO("json_object_new_object", ENOMEM);
		return;
	}
	struct json_object *jobj_info = json_object_new_object();
	if (jobj_info == nullptr) {
		ULOG_ERRNO("json_object_new_object", ENOMEM);
		json_object_put(jobj_frame);
		return;
	}

	json_object_object_add(jobj_frame,
			       "timestamp",
			       json_object_new_int64(frame->info.timestamp));
	json_object_object_add(jobj_frame,
			       "timescale",
			       json_object_new_int(frame->info.timescale));
	json_object_object_add(
		jobj_frame, "index", json_object_new_int(frame->info.index));
	char *fmt = nullptr;
	ret = asprintf(&fmt,
		       VDEF_RAW_FORMAT_TO_STR_FMT,
		       VDEF_RAW_FORMAT_TO_STR_ARG(&frame->format));
	if (fmt != nullptr) {
		json_object_object_add(
			jobj_frame, "format", json_object_new_string(fmt));
		free(fmt);
	}

	json_object_object_add(jobj_info,
			       "full_range",
			       json_object_new_boolean(frame->info.full_range));
	json_object_object_add(
		jobj_info,
		"color_primaries",
		json_object_new_string(vdef_color_primaries_to_str(
			frame->info.color_primaries)));
	json_object_object_add(
		jobj_info,
		"transfer_function",
		json_object_new_string(vdef_transfer_function_to_str(
			frame->info.transfer_function)));
	json_object_object_add(jobj_info,
			       "matrix_coefs",
			       json_object_new_string(vdef_matrix_coefs_to_str(
				       frame->info.matrix_coefs)));
	json_object_object_add(
		jobj_info,
		"width",
		json_object_new_int(frame->info.resolution.width));
	json_object_object_add(
		jobj_info,
		"height",
		json_object_new_int(frame->info.resolution.height));
	json_object_object_add(jobj_info,
			       "sar_width",
			       json_object_new_int(frame->info.sar.width));
	json_object_object_add(jobj_info,
			       "sar_height",
			       json_object_new_int(frame->info.sar.height));

	json_object_object_add(jobj_frame, "info", jobj_info);
	json_object_object_add(jobj, "frame", jobj_frame);
}


static void jsonFillCodedVideoInfo(struct json_object *jobj,
				   const struct vdef_coded_frame *frame)
{
	int ret;
	struct json_object *jobj_frame = json_object_new_object();
	if (jobj_frame == nullptr) {
		ULOG_ERRNO("json_object_new_object", ENOMEM);
		return;
	}
	struct json_object *jobj_info = json_object_new_object();
	if (jobj_info == nullptr) {
		ULOG_ERRNO("json_object_new_object", ENOMEM);
		json_object_put(jobj_frame);
		return;
	}

	json_object_object_add(jobj_frame,
			       "timestamp",
			       json_object_new_int64(frame->info.timestamp));
	json_object_object_add(jobj_frame,
			       "timescale",
			       json_object_new_int(frame->info.timescale));
	json_object_object_add(
		jobj_frame, "index", json_object_new_int(frame->info.index));
	char *fmt = nullptr;
	ret = asprintf(&fmt,
		       VDEF_CODED_FORMAT_TO_STR_FMT,
		       VDEF_CODED_FORMAT_TO_STR_ARG(&frame->format));
	if (fmt != nullptr) {
		json_object_object_add(
			jobj_frame, "format", json_object_new_string(fmt));
		free(fmt);
	}

	json_object_object_add(jobj_info,
			       "full_range",
			       json_object_new_boolean(frame->info.full_range));
	json_object_object_add(
		jobj_info,
		"color_primaries",
		json_object_new_string(vdef_color_primaries_to_str(
			frame->info.color_primaries)));
	json_object_object_add(
		jobj_info,
		"transfer_function",
		json_object_new_string(vdef_transfer_function_to_str(
			frame->info.transfer_function)));
	json_object_object_add(jobj_info,
			       "matrix_coefs",
			       json_object_new_string(vdef_matrix_coefs_to_str(
				       frame->info.matrix_coefs)));
	json_object_object_add(
		jobj_info,
		"width",
		json_object_new_int(frame->info.resolution.width));
	json_object_object_add(
		jobj_info,
		"height",
		json_object_new_int(frame->info.resolution.height));
	json_object_object_add(jobj_info,
			       "sar_width",
			       json_object_new_int(frame->info.sar.width));
	json_object_object_add(jobj_info,
			       "sar_height",
			       json_object_new_int(frame->info.sar.height));

	json_object_object_add(jobj_frame, "info", jobj_info);
	json_object_object_add(jobj, "frame", jobj_frame);
}


int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *frame,
				 struct vmeta_frame *metadata,
				 char *output,
				 unsigned int len)
{
	if (!frame || !output)
		return -EINVAL;

	const char *jstr;
	struct json_object *jobj = json_object_new_object();
	if (jobj == nullptr)
		return -ENOMEM;
	int ret = pdraw_frameMetadataToJson(frame, metadata, jobj);
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


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *frame,
			      struct vmeta_frame *metadata,
			      struct json_object *jobj)
{
	if (!frame || !jobj)
		return -EINVAL;

	int ret = 0;
	struct json_object *jobj_sub = nullptr;

	json_object_object_add(
		jobj,
		"format",
		json_object_new_string(vdef_frame_type_to_str(frame->format)));
	switch (frame->format) {
	case VDEF_FRAME_TYPE_RAW:
		jobj_sub = json_object_new_object();
		if (jobj_sub == nullptr)
			return -ENOMEM;
		jsonFillRawVideoInfo(jobj_sub, &frame->raw);
		json_object_object_add(jobj, "raw", jobj_sub);
		json_object_object_add(
			jobj,
			"has_errors",
			json_object_new_int(!!(frame->raw.info.flags &
					       VDEF_FRAME_FLAG_VISUAL_ERROR)));
		json_object_object_add(
			jobj,
			"is_silent",
			json_object_new_int(!!(frame->raw.info.flags &
					       VDEF_FRAME_FLAG_SILENT)));
		break;

	case VDEF_FRAME_TYPE_CODED:
		jobj_sub = json_object_new_object();
		if (jobj_sub == nullptr)
			return -ENOMEM;
		jsonFillCodedVideoInfo(jobj_sub, &frame->coded);
		json_object_object_add(jobj, "coded", jobj_sub);
		json_object_object_add(
			jobj,
			"has_errors",
			json_object_new_int(!!(frame->coded.info.flags &
					       VDEF_FRAME_FLAG_VISUAL_ERROR)));
		json_object_object_add(
			jobj,
			"is_silent",
			json_object_new_int(!!(frame->coded.info.flags &
					       VDEF_FRAME_FLAG_SILENT)));
		break;

	default:
		ULOGW("unknown frame format: %d(%s)",
		      frame->format,
		      vdef_frame_type_to_str(frame->format));
		break;
	}
	json_object_object_add(
		jobj, "is_sync", json_object_new_int(frame->is_sync));
	json_object_object_add(
		jobj, "is_ref", json_object_new_int(frame->is_ref));
	json_object_object_add(jobj,
			       "ntp_timestamp",
			       json_object_new_int64(frame->ntp_timestamp));
	json_object_object_add(
		jobj,
		"ntp_unskewed_timestamp",
		json_object_new_int64(frame->ntp_unskewed_timestamp));
	json_object_object_add(jobj,
			       "ntp_raw_timestamp",
			       json_object_new_int64(frame->ntp_raw_timestamp));
	json_object_object_add(
		jobj,
		"ntp_raw_unskewed_timestamp",
		json_object_new_int64(frame->ntp_raw_unskewed_timestamp));
	json_object_object_add(jobj,
			       "play_timestamp",
			       json_object_new_int64(frame->play_timestamp));
	json_object_object_add(jobj,
			       "capture_timestamp",
			       json_object_new_int64(frame->capture_timestamp));
	json_object_object_add(jobj,
			       "local_timestamp",
			       json_object_new_int64(frame->local_timestamp));
	if (metadata) {
		jobj_sub = json_object_new_object();
		if (jobj_sub == nullptr)
			return -ENOMEM;
		int ret = vmeta_frame_to_json(metadata, jobj_sub);
		if (ret < 0) {
			if (jobj_sub != nullptr)
				json_object_put(jobj_sub);
			return ret;
		}
		json_object_object_add(jobj, "metadata", jobj_sub);
	}

	return ret;
}


#else /* BUILD_JSON undefined */

int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *frame,
				 struct vmeta_frame *metadata,
				 char *output,
				 unsigned int len)
{
	ULOGW("%s not implemented", __func__);
	return -ENOSYS;
}


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *frame,
			      struct vmeta_frame *metadata,
			      struct json_object *jobj)
{
	ULOGW("%s not implemented", __func__);
	return -ENOSYS;
}

#endif /* BUILD_JSON */


uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_coded_video_frame *frame,
					 const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_coded_video_frame_get_ancillary_data(frame, key, &data);
	if (res < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(ts))
		goto out;
	memcpy(&ts, raw_data, sizeof(ts));

out:
	mbuf_ancillary_data_unref(data);
	return ts;
}


uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_raw_video_frame *frame,
					 const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_raw_video_frame_get_ancillary_data(frame, key, &data);
	if (res < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(ts))
		goto out;
	memcpy(&ts, raw_data, sizeof(ts));

out:
	mbuf_ancillary_data_unref(data);
	return ts;
}


struct pdraw_media_info *pdraw_mediaInfoDup(const struct pdraw_media_info *src)
{
	struct pdraw_media_info *dst;

	ULOG_ERRNO_RETURN_VAL_IF(src == NULL, EINVAL, NULL);

	dst = (pdraw_media_info *)malloc(sizeof(*src));
	if (dst == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	*dst = *src;

	dst->name = NULL;
	dst->path = NULL;

	dst->name = strdup(src->name);
	if (dst->name == NULL) {
		ULOG_ERRNO("strdup", ENOMEM);
		goto failure;
	}
	dst->path = strdup(src->path);
	if (dst->path == NULL) {
		ULOG_ERRNO("strdup", ENOMEM);
		goto failure;
	}

	return dst;

failure:
	free((void *)dst->name);
	free((void *)dst->path);
	free(dst);
	return NULL;
}


void pdraw_mediaInfoFree(struct pdraw_media_info *media_info)
{
	if (media_info == NULL)
		return;

	free((void *)media_info->name);
	free((void *)media_info->path);
	free(media_info);
}


namespace Pdraw {

std::atomic<unsigned int> Loggable::mIdCounter(0);

Loggable::Loggable()
{
	mName = std::string(__func__) + "#" + std::to_string(++mIdCounter);
	self = this;
}

void Loggable::setName(std::string &name)
{
	mName = name;
}

void Loggable::setName(const char *name)
{
	mName = std::string(name);
}

} // namespace Pdraw
