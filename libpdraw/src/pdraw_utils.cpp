/**
 * Parrot Drones Audio and Video Vector library
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
const char *PDRAW_ANCILLARY_DATA_KEY_AUDIOFRAME = "pdraw.audio.frame";
const char *PDRAW_VIDEO_RENDERER_DBG_FLAGS = "PDRAW_VIDEO_RENDERER_DBG_FLAGS";
}


static const std::map<enum pdraw_demuxer_autodecoding_mode, const char *>
	pdraw_demuxer_autodecoding_mode_map{
		{PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL, "DECODE_ALL"},
		{PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE, "DECODE_NONE"},
	};


static const std::map<enum pdraw_playback_type, const char *>
	pdraw_playback_type_map{
		{PDRAW_PLAYBACK_TYPE_UNKNOWN, "UNKNOWN"},
		{PDRAW_PLAYBACK_TYPE_LIVE, "LIVE"},
		{PDRAW_PLAYBACK_TYPE_REPLAY, "REPLAY"},
	};


static const std::map<enum pdraw_media_type, const char *> pdraw_media_type_map{
	{PDRAW_MEDIA_TYPE_UNKNOWN, "UNKNOWN"},
	{PDRAW_MEDIA_TYPE_VIDEO, "VIDEO"},
	{PDRAW_MEDIA_TYPE_AUDIO, "AUDIO"},
};


static const std::map<enum pdraw_muxer_connection_state, const char *>
	pdraw_muxer_connection_state_map{
		{PDRAW_MUXER_CONNECTION_STATE_UNKNOWN, "UNKNOWN"},
		{PDRAW_MUXER_CONNECTION_STATE_DISCONNECTED, "DISCONNECTED"},
		{PDRAW_MUXER_CONNECTION_STATE_CONNECTING, "CONNECTING"},
		{PDRAW_MUXER_CONNECTION_STATE_CONNECTED, "CONNECTED"},
	};


static const std::map<enum pdraw_muxer_disconnection_reason, const char *>
	pdraw_muxer_disconnection_reason_map{
		{PDRAW_MUXER_DISCONNECTION_REASON_UNKNOWN, "UNKNOWN"},
		{PDRAW_MUXER_DISCONNECTION_REASON_CLIENT_REQUEST,
		 "CLIENT_REQUEST"},
		{PDRAW_MUXER_DISCONNECTION_REASON_SERVER_REQUEST,
		 "SERVER_REQUEST"},
		{PDRAW_MUXER_DISCONNECTION_REASON_NETWORK_ERROR,
		 "NETWORK_ERROR"},
		{PDRAW_MUXER_DISCONNECTION_REASON_REFUSED, "REFUSED"},
		{PDRAW_MUXER_DISCONNECTION_REASON_ALREADY_IN_USE,
		 "ALREADY_IN_USE"},
		{PDRAW_MUXER_DISCONNECTION_REASON_TIMEOUT, "TIMEOUT"},
		{PDRAW_MUXER_DISCONNECTION_REASON_INTERNAL_ERROR,
		 "INTERNAL_ERROR"},
	};


static const std::map<enum pdraw_video_type, const char *> pdraw_video_type_map{
	{PDRAW_VIDEO_TYPE_DEFAULT_CAMERA, "DEFAULT_CAMERA"},
};


static const std::map<enum pdraw_histogram_channel, const char *>
	pdraw_histogram_channel_map{
		{PDRAW_HISTOGRAM_CHANNEL_RED, "RED"},
		{PDRAW_HISTOGRAM_CHANNEL_GREEN, "GREEN"},
		{PDRAW_HISTOGRAM_CHANNEL_BLUE, "BLUE"},
		{PDRAW_HISTOGRAM_CHANNEL_LUMA, "LUMA"},
	};

static const std::map<enum pdraw_video_renderer_scheduling_mode, const char *>
	pdraw_video_renderer_scheduling_mode_map{
		{PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP, "ASAP"},
		{PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE,
		 "ASAP_SIGNAL_ONCE"},
		{PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE, "ADAPTIVE"},
	};


static const std::map<enum pdraw_video_renderer_fill_mode, const char *>
	pdraw_video_renderer_fill_mode_map{
		{PDRAW_VIDEO_RENDERER_FILL_MODE_FIT, "FIT"},
		{PDRAW_VIDEO_RENDERER_FILL_MODE_CROP, "CROP"},
		{PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP,
		 "FIT_PAD_BLUR_CROP"},
		{PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND,
		 "FIT_PAD_BLUR_EXTEND"},
	};

static const std::map<enum pdraw_video_renderer_transition_flag, const char *>
	pdraw_video_renderer_transition_flag_map{
		{PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS, "SOS"},
		{PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS, "EOS"},
		{PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE,
		 "RECONFIGURE"},
		{PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT, "TIMEOUT"},
		{PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER,
		 "PHOTO_TRIGGER"},
	};


static const std::map<enum pdraw_vipc_source_eos_reason, const char *>
	pdraw_vipc_source_eos_reason_map{
		{PDRAW_VIPC_SOURCE_EOS_REASON_NONE, "NONE"},
		{PDRAW_VIPC_SOURCE_EOS_REASON_RESTART, "RESTART"},
		{PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION, "CONFIGURATION"},
		{PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT, "TIMEOUT"},
	};


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


const char *
pdraw_demuxerAutodecodingModeStr(enum pdraw_demuxer_autodecoding_mode val)
{
	auto search = pdraw_demuxer_autodecoding_mode_map.find(val);

	if (search != pdraw_demuxer_autodecoding_mode_map.end())
		return search->second;

	ULOGW("invalid demuxer autodecoding mode: %d", val);
	return "INVALID";
}


enum pdraw_demuxer_autodecoding_mode
pdraw_demuxerAutodecodingModeFromStr(const char *val)
{
	enum pdraw_demuxer_autodecoding_mode ret =
		pdraw_demuxer_autodecoding_mode_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_demuxer_autodecoding_mode_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_playbackTypeStr(enum pdraw_playback_type val)
{
	auto search = pdraw_playback_type_map.find(val);

	if (search != pdraw_playback_type_map.end())
		return search->second;

	ULOGW("invalid pdraw_playback_type: %d", val);
	return "INVALID";
}


enum pdraw_playback_type pdraw_playbackTypeFromStr(const char *val)
{
	enum pdraw_playback_type ret = pdraw_playback_type_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_playback_type_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_mediaTypeStr(enum pdraw_media_type val)
{
	auto search = pdraw_media_type_map.find(val);

	if (search != pdraw_media_type_map.end())
		return search->second;


	ULOGW("invalid pdraw_media_type: %d", val);
	return "INVALID";
}


enum pdraw_media_type pdraw_mediaTypeFromStr(const char *val)
{
	enum pdraw_media_type ret = pdraw_media_type_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_media_type_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_muxerConnectionStateStr(enum pdraw_muxer_connection_state val)
{
	auto search = pdraw_muxer_connection_state_map.find(val);

	if (search != pdraw_muxer_connection_state_map.end())
		return search->second;

	ULOGW("invalid pdraw_muxer_connection_state: %d", val);
	return "INVALID";
}


const char *
pdraw_muxerDisconnectionReasonStr(enum pdraw_muxer_disconnection_reason val)
{
	auto search = pdraw_muxer_disconnection_reason_map.find(val);

	if (search != pdraw_muxer_disconnection_reason_map.end())
		return search->second;

	ULOGW("invalid pdraw_muxer_disconnection_reason: %d", val);
	return "INVALID";
}


const char *pdraw_videoTypeStr(enum pdraw_video_type val)
{
	auto search = pdraw_video_type_map.find(val);

	if (search != pdraw_video_type_map.end())
		return search->second;

	ULOGW("invalid pdraw_video_type: %d", val);
	return "INVALID";
}


enum pdraw_video_type pdraw_videoTypeFromStr(const char *val)
{
	enum pdraw_video_type ret = pdraw_video_type_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_video_type_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_histogramChannelStr(enum pdraw_histogram_channel val)
{
	auto search = pdraw_histogram_channel_map.find(val);

	if (search != pdraw_histogram_channel_map.end())
		return search->second;

	ULOGW("invalid pdraw_histogram_channel: %d", val);
	return "INVALID";
}


enum pdraw_histogram_channel pdraw_histogramChannelFromStr(const char *val)
{
	enum pdraw_histogram_channel ret =
		pdraw_histogram_channel_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_histogram_channel_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_videoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val)
{
	auto search = pdraw_video_renderer_scheduling_mode_map.find(val);

	if (search != pdraw_video_renderer_scheduling_mode_map.end())
		return search->second;

	ULOGW("invalid pdraw_video_renderer_scheduling_mode: %d", val);
	return "INVALID";
}


enum pdraw_video_renderer_scheduling_mode
pdraw_videoRendererSchedulingModeFromStr(const char *val)
{
	enum pdraw_video_renderer_scheduling_mode ret =
		pdraw_video_renderer_scheduling_mode_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_video_renderer_scheduling_mode_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *
pdraw_videoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val)
{
	auto search = pdraw_video_renderer_fill_mode_map.find(val);

	if (search != pdraw_video_renderer_fill_mode_map.end())
		return search->second;

	ULOGW("invalid pdraw_video_renderer_fill_mode: %d", val);
	return "INVALID";
}


enum pdraw_video_renderer_fill_mode
pdraw_videoRendererFillModeFromStr(const char *val)
{
	enum pdraw_video_renderer_fill_mode ret =
		pdraw_video_renderer_fill_mode_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_video_renderer_fill_mode_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_videoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val)
{
	auto search = pdraw_video_renderer_transition_flag_map.find(val);

	if (search != pdraw_video_renderer_transition_flag_map.end())
		return search->second;

	ULOGW("invalid pdraw_video_renderer_transition_flag: %d", val);
	return "INVALID";
}


enum pdraw_video_renderer_transition_flag
pdraw_videoRendererTransitionFlagFromStr(const char *val)
{
	enum pdraw_video_renderer_transition_flag ret =
		pdraw_video_renderer_transition_flag_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_video_renderer_transition_flag_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
}


const char *pdraw_vipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val)
{
	auto search = pdraw_vipc_source_eos_reason_map.find(val);

	if (search != pdraw_vipc_source_eos_reason_map.end())
		return search->second;

	ULOGW("invalid pdraw_vipc_source_eos_reason: %d", val);
	return "INVALID";
}


enum pdraw_vipc_source_eos_reason
pdraw_vipcSourceEosReasonFromStr(const char *val)
{
	enum pdraw_vipc_source_eos_reason ret =
		pdraw_vipc_source_eos_reason_map.begin()->first;

	ULOG_ERRNO_RETURN_VAL_IF(val == nullptr, EINVAL, ret);

	for (auto i : pdraw_vipc_source_eos_reason_map) {
		if (strcmp(i.second, val) == 0)
			return i.first;
	}
	ULOGW("invalid input: %s", val);
	return ret;
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


uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_audio_frame *frame,
					 const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_audio_frame_get_ancillary_data(frame, key, &data);
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

	ULOG_ERRNO_RETURN_VAL_IF(src == nullptr, EINVAL, nullptr);

	dst = (pdraw_media_info *)malloc(sizeof(*src));
	if (dst == nullptr) {
		ULOG_ERRNO("calloc", ENOMEM);
		return nullptr;
	}
	*dst = *src;

	dst->name = nullptr;
	dst->path = nullptr;

	dst->name = strdup(src->name);
	if (dst->name == nullptr) {
		ULOG_ERRNO("strdup", ENOMEM);
		goto failure;
	}
	dst->path = strdup(src->path);
	if (dst->path == nullptr) {
		ULOG_ERRNO("strdup", ENOMEM);
		goto failure;
	}

	return dst;

failure:
	free((void *)dst->name);
	free((void *)dst->path);
	free(dst);
	return nullptr;
}


void pdraw_mediaInfoFree(struct pdraw_media_info *media_info)
{
	if (media_info == nullptr)
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
