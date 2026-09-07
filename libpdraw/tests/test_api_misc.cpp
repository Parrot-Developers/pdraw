/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Misc helper API input-validation (Tier A)
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "pdraw/pdraw.h"
#include "pdraw/pdraw.hpp"
#include "test_common.h"

#include <video-metadata/vmeta_frame.h>

#include <json-c/json.h>

using namespace Pdraw;


/* ── Free / helper functions ─────────────────────────────────────── */

static void testMuxerParamsFreeNull()
{
	pdraw_muxer_params_free(nullptr);
	CU_PASS("no crash");
}


static void testDemuxerMediaListFreeNull()
{
	pdraw_demuxer_media_list_free(nullptr, 0);
	CU_PASS("no crash");
}


/* Each xxxStrInvalid() test below passes an enum value absent from the
 * function's lookup table (pdraw_xxx_map in pdraw_utils.cpp); besides not
 * crashing, the documented fallback is the "INVALID" literal (never
 * nullptr), asserted explicitly here rather than just checking for "no
 * crash". */

static void testDemuxerAutodecodingModeStrInvalid()
{
	const char *s = pdrawDemuxerAutodecodingModeStr(
		(enum pdraw_demuxer_autodecoding_mode) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testPlaybackTypeStrInvalid()
{
	const char *s = pdrawPlaybackTypeStr((enum pdraw_playback_type) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testMediaTypeStrInvalid()
{
	const char *s = pdrawMediaTypeStr((enum pdraw_media_type) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testMuxerConnectionStateStrInvalid()
{
	const char *s = pdraw_muxer_connection_state_str(
		(enum pdraw_muxer_connection_state) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testMuxerDisconnectionReasonStrInvalid()
{
	const char *s = pdraw_muxer_disconnection_reason_str(
		(enum pdraw_muxer_disconnection_reason) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testMuxerRtspTransportStrInvalid()
{
	const char *s = pdraw_muxer_rtsp_transport_str(
		(enum pdraw_muxer_rtsp_transport) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testVideoRendererFillModeStrInvalid()
{
	const char *s = pdraw_video_renderer_fill_mode_str(
		(enum pdraw_video_renderer_fill_mode) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testVipcSourceEosReasonStrInvalid()
{
	const char *s = pdraw_vipc_source_eos_reason_str(
		(enum pdraw_vipc_source_eos_reason) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testHistogramChannelStrInvalid()
{
	const char *s =
		pdraw_histogram_channel_str((enum pdraw_histogram_channel) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testVideoRendererSchedulingModeStrInvalid()
{
	const char *s = pdraw_video_renderer_scheduling_mode_str(
		(enum pdraw_video_renderer_scheduling_mode) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testVideoRendererTransitionFlagStrInvalid()
{
	const char *s = pdraw_video_renderer_transition_flag_str(
		(enum pdraw_video_renderer_transition_flag) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testFromStrNullIsSafe()
{
	/* Every *_from_str() wrapper below is a thin strcmp() lookup table;
	 * grouped here (rather than one test per function, unlike the
	 * enum-to-string tests above) since they all exercise the exact
	 * same property: a null input string must not crash. */
	(void)pdraw_demuxer_autodecoding_mode_from_str(nullptr);
	(void)pdraw_playback_type_from_str(nullptr);
	(void)pdraw_media_type_from_str(nullptr);
	(void)pdraw_histogram_channel_from_str(nullptr);
	(void)pdraw_muxer_rtsp_transport_from_str(nullptr);
	(void)pdraw_video_renderer_fill_mode_from_str(nullptr);
	(void)pdraw_video_renderer_scheduling_mode_from_str(nullptr);
	(void)pdraw_video_renderer_transition_flag_from_str(nullptr);
	(void)pdraw_vipc_source_eos_reason_from_str(nullptr);
	CU_PASS("no crash");
}


/* ── FromStr() with an unrecognized (non-null) string ──────────────────────
 * testFromStrNullIsSafe above only exercises the "val == nullptr" early
 * return (ULOG_ERRNO_RETURN_VAL_IF in pdraw_utils.cpp); a non-null string
 * that matches no table entry instead falls through the whole strcmp()
 * loop and hits the ULOGW("invalid input: %s", val) branch just below it --
 * a distinct line, not reached by the null case. Each *_from_str() then
 * falls back to the lookup table's first (lowest-value) entry; asserted
 * explicitly below rather than just checking for "no crash", since that
 * fallback value is part of the documented contract callers rely on. ─────── */

static void testDemuxerAutodecodingModeFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_demuxer_autodecoding_mode_from_str("BOGUS"),
			PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL);
}


static void testPlaybackTypeFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_playback_type_from_str("BOGUS"),
			PDRAW_PLAYBACK_TYPE_UNKNOWN);
}


static void testMediaTypeFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_media_type_from_str("BOGUS"),
			PDRAW_MEDIA_TYPE_UNKNOWN);
}


static void testMuxerRtspTransportFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_muxer_rtsp_transport_from_str("BOGUS"),
			PDRAW_MUXER_RTSP_TRANSPORT_UDP);
}


static void testHistogramChannelFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_histogram_channel_from_str("BOGUS"),
			PDRAW_HISTOGRAM_CHANNEL_RED);
}


static void testVideoRendererSchedulingModeFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_video_renderer_scheduling_mode_from_str("BOGUS"),
			PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP);
}


static void testVideoRendererFillModeFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_video_renderer_fill_mode_from_str("BOGUS"),
			PDRAW_VIDEO_RENDERER_FILL_MODE_FIT);
}


static void testVideoRendererTransitionFlagFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_video_renderer_transition_flag_from_str("BOGUS"),
			PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS);
}


static void testVipcSourceEosReasonFromStrInvalidString()
{
	CU_ASSERT_EQUAL(pdraw_vipc_source_eos_reason_from_str("BOGUS"),
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
}


/* ── Str/FromStr round trips (C++ API, pdraw.hpp) ─────────────────────────
 * Every test above only exercises the C API (pdraw.h) entry points
 * (pdraw_xxx_str()/pdraw_xxx_from_str()). The parallel C++ namespace entry
 * points (pdrawXxxStr()/pdrawXxxFromStr() in pdraw.hpp) are separate thin
 * wrapper functions in pdraw_session.cpp with their own lines, entirely
 * uncovered by the above -- calling the C versions does not exercise them.
 * Each test below does a real round trip (Str(valid enum) ->
 * FromStr(that string) == original enum), which additionally exercises the
 * "found in table" branch that the StrInvalid/FromStrNullIsSafe tests above
 * never reach (those only ever pass invalid/null input). ────────────────── */

static void testCxxDemuxerAutodecodingModeRoundTrips()
{
	const char *s = pdrawDemuxerAutodecodingModeStr(
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawDemuxerAutodecodingModeFromStr(s),
			PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL);
}


static void testCxxPlaybackTypeRoundTrips()
{
	const char *s = pdrawPlaybackTypeStr(PDRAW_PLAYBACK_TYPE_LIVE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawPlaybackTypeFromStr(s), PDRAW_PLAYBACK_TYPE_LIVE);
}


static void testCxxMediaTypeRoundTrips()
{
	const char *s = pdrawMediaTypeStr(PDRAW_MEDIA_TYPE_VIDEO);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawMediaTypeFromStr(s), PDRAW_MEDIA_TYPE_VIDEO);
}


static void testCxxHistogramChannelRoundTrips()
{
	const char *s = pdrawHistogramChannelStr(PDRAW_HISTOGRAM_CHANNEL_RED);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawHistogramChannelFromStr(s),
			PDRAW_HISTOGRAM_CHANNEL_RED);
}


static void testCxxVideoRendererSchedulingModeRoundTrips()
{
	const char *s = pdrawVideoRendererSchedulingModeStr(
		PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawVideoRendererSchedulingModeFromStr(s),
			PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP);
}


static void testCxxVideoRendererFillModeRoundTrips()
{
	const char *s = pdrawVideoRendererFillModeStr(
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawVideoRendererFillModeFromStr(s),
			PDRAW_VIDEO_RENDERER_FILL_MODE_FIT);
}


static void testCxxVideoRendererTransitionFlagRoundTrips()
{
	const char *s = pdrawVideoRendererTransitionFlagStr(
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawVideoRendererTransitionFlagFromStr(s),
			PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS);
}


static void testCxxVipcSourceEosReasonRoundTrips()
{
	const char *s =
		pdrawVipcSourceEosReasonStr(PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_EQUAL(pdrawVipcSourceEosReasonFromStr(s),
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
}


/* ── Struct Dup/Free round trips (C++ API, pdraw.hpp) ─────────────────────
 * The FreeNull tests further above only exercise the "src == nullptr" early
 * return of each Free(); Dup() itself, and the real deep-copy/free logic
 * for a populated struct, were entirely uncovered. Each test below builds a
 * struct with heap-independent (string-literal) fields, confirms Dup()
 * produces an EQUAL but DIFFERENT-POINTER copy (proving a genuine deep
 * copy via strdup, not a shallow struct copy that would alias the
 * literals), then frees the copy -- exercising Free()'s real free() calls
 * too, not just its null-check. ──────────────────────────────────────────── */

static void testCxxMediaInfoDupRoundTrips()
{
	struct pdraw_media_info src = {};
	src.type = PDRAW_MEDIA_TYPE_VIDEO;
	src.id = 42;
	src.name = "test-media";
	src.path = "test-path";

	struct pdraw_media_info *dup = pdrawMediaInfoDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_EQUAL(dup->id, src.id);
	CU_ASSERT_STRING_EQUAL(dup->name, src.name);
	CU_ASSERT_STRING_EQUAL(dup->path, src.path);
	CU_ASSERT_PTR_NOT_EQUAL(dup->name, src.name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->path, src.path);

	pdrawMediaInfoFree(dup);
}


static void testCxxVipcSourceParamsDupRoundTrips()
{
	struct pdraw_vipc_source_params src = {};
	src.address = "127.0.0.1:1234";
	src.friendly_name = "test-vipc";
	src.backend_name = "test-backend";

	struct pdraw_vipc_source_params *dup = pdrawVipcSourceParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->address, src.address);
	CU_ASSERT_STRING_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_STRING_EQUAL(dup->backend_name, src.backend_name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->address, src.address);

	pdrawVipcSourceParamsFree(dup);
}


static void testCxxMuxerParamsDupRoundTrips()
{
	struct pdraw_muxer_params src = {};
	src.recovery.tables_file = "test-tables.bin";

	struct pdraw_muxer_params *dup = pdrawMuxerParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->recovery.tables_file,
			       src.recovery.tables_file);
	CU_ASSERT_PTR_NOT_EQUAL(dup->recovery.tables_file,
				src.recovery.tables_file);

	pdrawMuxerParamsFree(dup);
}


static void testCxxMuxerMediaParamsDupRoundTrips()
{
	struct pdraw_muxer_media_params src = {};
	src.track_name = "test-track";
	src.is_default = true;

	struct pdraw_muxer_media_params *dup = pdrawMuxerMediaParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_TRUE(dup->is_default);

	pdrawMuxerMediaParamsFree(dup);
}


static void testCxxDemuxerMediaListFreeWithRealContent()
{
	/* pdrawDemuxerMediaListFree() frees each entry's .name/.uri via
	 * free(), so unlike testDemuxerMediaListFreeNull above (which only
	 * exercises the "media_list == nullptr" early return), these must
	 * be genuinely heap-allocated. */
	struct pdraw_demuxer_media *list =
		(struct pdraw_demuxer_media *)calloc(2, sizeof(*list));
	CU_ASSERT_PTR_NOT_NULL_FATAL(list);
	list[0].name = strdup("track0");
	list[0].uri = strdup("uri0");
	list[1].name = strdup("track1");
	list[1].uri = nullptr; /* uri is optional: must tolerate null too */

	pdrawDemuxerMediaListFree(list, 2);
	CU_PASS("no crash");
}


/* ── Video frame -> JSON conversion (C++ API, pdraw.hpp) ──────────────────
 * pdrawVideoFrameToJson()/ToJsonStr() are compiled unconditionally (see
 * pdraw_utils.cpp) but only ever produce real content when BUILD_JSON is
 * defined (json-c actually linked in); otherwise they return -ENOSYS
 * without touching jobj/buf at all -- confirmed enabled in this build
 * before writing this as a strict (ret == 0, content non-empty) test,
 * same precaution as for the optional x265/turbojpeg video encoder
 * backends earlier in this suite. ───────────────────────────────────────── */

static void testCxxVideoFrameToJsonRoundTrips()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_RAW;
	frame.raw.format = vdef_i420;
	frame.raw.info.resolution.width = 64;
	frame.raw.info.resolution.height = 64;

	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	int ret = pdrawVideoFrameToJson(&frame, nullptr, jobj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	const char *jstr = json_object_to_json_string(jobj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jstr);
	/* The whole point: real content was written, not an empty "{}". */
	CU_ASSERT_TRUE(strstr(jstr, "format") != nullptr);

	json_object_put(jobj);
}


static void testCxxVideoFrameToJsonStrRoundTrips()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_RAW;
	frame.raw.format = vdef_i420;
	frame.raw.info.resolution.width = 64;
	frame.raw.info.resolution.height = 64;

	/* pdraw_frameMetadataToJsonStr() (pdraw_utils.cpp) returns -ENOBUFS if
	 * the serialized JSON doesn't fit: jsonFillRawVideoInfo() dumps many
	 * more fields than just format/resolution (sar, framerate, dynamic
	 * range, plane strides/sizes...), comfortably exceeding 512 bytes --
	 * confirmed the hard way (a smaller buffer here made this test fail
	 * with ret == -ENOBUFS while the equivalent ToJson test, which has
	 * no such fixed-size constraint, passed). */
	char buf[4096] = {};
	int ret = pdrawVideoFrameToJsonStr(&frame, nullptr, buf, sizeof(buf));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(strstr(buf, "format") != nullptr);
}


/* jsonFillCodedVideoInfo() (pdraw_utils.cpp) was entirely untested: every
 * frame-to-JSON test above uses VDEF_FRAME_TYPE_RAW, so
 * pdraw_frameMetadataToJson()'s VDEF_FRAME_TYPE_CODED branch (the only
 * caller of jsonFillCodedVideoInfo()) never ran. */
static void testCxxVideoFrameToJsonCodedRoundTrips()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_CODED;
	frame.coded.format = vdef_h264_avcc;
	frame.coded.info.resolution.width = 64;
	frame.coded.info.resolution.height = 64;

	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	int ret = pdrawVideoFrameToJson(&frame, nullptr, jobj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	const char *jstr = json_object_to_json_string(jobj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jstr);
	CU_ASSERT_TRUE(strstr(jstr, "\"coded\"") != nullptr);
	/* jsonFillCodedVideoInfo() writes format as "<encoding>/<data_format>"
	 * via VDEF_CODED_FORMAT_TO_STR_FMT (vdefs.h); vdef_encoding_to_str()
	 * returns "H264" for VDEF_ENCODING_H264 (vdefs.c). */
	CU_ASSERT_TRUE(strstr(jstr, "H264") != nullptr);

	json_object_put(jobj);
}


/* pdraw_frameMetadataToJson()'s `if (metadata)` branch (pdraw_utils.cpp),
 * which calls vmeta_frame_to_json(), was never exercised: every other test
 * in this file passes metadata=nullptr. A synthetic vmeta_frame is built
 * here (rather than reading one from a real recording) so this Tier A test
 * stays fixture-free and deterministic. */
static void testCxxVideoFrameToJsonWithVmetaIncludesMetadata()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_RAW;
	frame.raw.format = vdef_i420;
	frame.raw.info.resolution.width = 64;
	frame.raw.info.resolution.height = 64;

	struct vmeta_frame *meta = nullptr;
	int ret = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &meta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta);
	meta->v3.base.location.valid = 1;
	meta->v3.base.location.latitude = 48.879124;
	meta->v3.base.location.longitude = 2.367456;

	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	ret = pdrawVideoFrameToJson(&frame, meta, jobj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct json_object *jmeta = nullptr;
	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(jobj, "metadata", &jmeta));
	const char *jstr = json_object_to_json_string(jobj);
	CU_ASSERT_TRUE(strstr(jstr, "latitude") != nullptr);

	json_object_put(jobj);
	vmeta_frame_unref(meta);
}


/* ── C API (pdraw_wrapper.cpp) pass-through coverage ───────────────────────
 * Every function below has a C++ (pdraw.hpp, tested above) or pdraw_utils.cpp
 * (tested in test_utils.cpp) sibling exercising the exact same underlying
 * logic -- but pdraw_wrapper.cpp's own one-line C API bodies (the actual
 * pdraw.h entry points) were never called from any test, so they showed 0%
 * coverage despite the logic they forward to being fully tested elsewhere.
 * Each test below is a minimal call through the real C entry point. ────────
 */

static void testCApiDemuxerAutodecodingModeStrInvalid()
{
	const char *s = pdraw_demuxer_autodecoding_mode_str(
		(enum pdraw_demuxer_autodecoding_mode) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testCApiPlaybackTypeStrInvalid()
{
	const char *s = pdraw_playback_type_str((enum pdraw_playback_type) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testCApiMediaTypeStrInvalid()
{
	const char *s = pdraw_media_type_str((enum pdraw_media_type) - 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(s);
	CU_ASSERT_STRING_EQUAL(s, "INVALID");
}


static void testCApiMediaInfoDupRoundTrips()
{
	struct pdraw_media_info src = {};
	src.type = PDRAW_MEDIA_TYPE_VIDEO;
	src.id = 42;
	src.name = "test-media";
	src.path = "test-path";

	struct pdraw_media_info *dup = pdraw_media_info_dup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_EQUAL(dup->id, src.id);
	CU_ASSERT_STRING_EQUAL(dup->name, src.name);
	CU_ASSERT_STRING_EQUAL(dup->path, src.path);
	CU_ASSERT_PTR_NOT_EQUAL(dup->name, src.name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->path, src.path);

	pdraw_media_info_free(dup);
}


static void testCApiVipcSourceParamsDupRoundTrips()
{
	struct pdraw_vipc_source_params src = {};
	src.address = "127.0.0.1:1234";
	src.friendly_name = "test-vipc";
	src.backend_name = "test-backend";

	struct pdraw_vipc_source_params *dup =
		pdraw_vipc_source_params_dup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->address, src.address);
	CU_ASSERT_STRING_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_STRING_EQUAL(dup->backend_name, src.backend_name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->address, src.address);

	pdraw_vipc_source_params_free(dup);
}


static void testCApiMuxerParamsDupRoundTrips()
{
	struct pdraw_muxer_params src = {};
	src.recovery.tables_file = "test-tables.bin";

	struct pdraw_muxer_params *dup = pdraw_muxer_params_dup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->recovery.tables_file,
			       src.recovery.tables_file);
	CU_ASSERT_PTR_NOT_EQUAL(dup->recovery.tables_file,
				src.recovery.tables_file);

	pdraw_muxer_params_free(dup);
}


static void testCApiMuxerMediaParamsDupRoundTrips()
{
	struct pdraw_muxer_media_params src = {};
	src.track_name = "test-track";
	src.is_default = true;

	struct pdraw_muxer_media_params *dup =
		pdraw_muxer_media_params_dup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_STRING_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_TRUE(dup->is_default);

	pdraw_muxer_media_params_free(dup);
}


static void testCApiVideoFrameToJsonRoundTrips()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_RAW;
	frame.raw.format = vdef_i420;
	frame.raw.info.resolution.width = 64;
	frame.raw.info.resolution.height = 64;

	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	int ret = pdraw_video_frame_to_json(&frame, nullptr, jobj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	const char *jstr = json_object_to_json_string(jobj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jstr);
	CU_ASSERT_TRUE(strstr(jstr, "format") != nullptr);

	json_object_put(jobj);
}


static void testCApiVideoFrameToJsonStrRoundTrips()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_RAW;
	frame.raw.format = vdef_i420;
	frame.raw.info.resolution.width = 64;
	frame.raw.info.resolution.height = 64;

	char buf[4096] = {};
	int ret = pdraw_video_frame_to_json_str(
		&frame, nullptr, buf, sizeof(buf));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(strstr(buf, "format") != nullptr);
}


/* Unlike pdrawAlsaSourceGetCapabilities() (pdraw.hpp, tested in
 * test_api_alsa_source.cpp), the C API entry point null-checks both
 * arguments itself (pdraw_wrapper.cpp) before ever reaching
 * AlsaSource::getCapabilities() -- so these two guards return -EINVAL
 * unconditionally, regardless of whether CONFIG_PDRAW_USE_ALSA is compiled
 * in (no CU_ASSERT_ALSA_NULL_GUARD needed here, unlike the Cxx version). */
static void testCApiAlsaSourceGetCapabilitiesNullAddress()
{
	struct pdraw_alsa_source_caps caps = {};
	int ret = pdraw_alsa_source_get_capabilities(nullptr, &caps);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiAlsaSourceGetCapabilitiesNullCaps()
{
	int ret = pdraw_alsa_source_get_capabilities("hw:0,0", nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiAlsaSourceGetCapabilitiesInvalidAddress()
{
	struct pdraw_alsa_source_caps caps = {};
	int ret = pdraw_alsa_source_get_capabilities(
		"this-alsa-device-does-not-exist", &caps);
	/* -ENOSYS without ALSA support, a real (negative) ALSA errno with it --
	 * never 0. */
	CU_ASSERT_FATAL(ret < 0);
}


CU_TestInfo g_pdraw_test_api_misc[] = {
	{FN("testMuxerParamsFreeNull"), testMuxerParamsFreeNull},
	{FN("testDemuxerMediaListFreeNull"), testDemuxerMediaListFreeNull},
	{FN("testDemuxerAutodecodingModeStrInvalid"),
	 testDemuxerAutodecodingModeStrInvalid},
	{FN("testPlaybackTypeStrInvalid"), testPlaybackTypeStrInvalid},
	{FN("testMediaTypeStrInvalid"), testMediaTypeStrInvalid},
	{FN("testMuxerConnectionStateStrInvalid"),
	 testMuxerConnectionStateStrInvalid},
	{FN("testMuxerDisconnectionReasonStrInvalid"),
	 testMuxerDisconnectionReasonStrInvalid},
	{FN("testMuxerRtspTransportStrInvalid"),
	 testMuxerRtspTransportStrInvalid},
	{FN("testVideoRendererFillModeStrInvalid"),
	 testVideoRendererFillModeStrInvalid},
	{FN("testVipcSourceEosReasonStrInvalid"),
	 testVipcSourceEosReasonStrInvalid},
	{FN("testHistogramChannelStrInvalid"), testHistogramChannelStrInvalid},
	{FN("testVideoRendererSchedulingModeStrInvalid"),
	 testVideoRendererSchedulingModeStrInvalid},
	{FN("testVideoRendererTransitionFlagStrInvalid"),
	 testVideoRendererTransitionFlagStrInvalid},
	{FN("testFromStrNullIsSafe"), testFromStrNullIsSafe},
	{FN("testDemuxerAutodecodingModeFromStrInvalidString"),
	 testDemuxerAutodecodingModeFromStrInvalidString},
	{FN("testPlaybackTypeFromStrInvalidString"),
	 testPlaybackTypeFromStrInvalidString},
	{FN("testMediaTypeFromStrInvalidString"),
	 testMediaTypeFromStrInvalidString},
	{FN("testMuxerRtspTransportFromStrInvalidString"),
	 testMuxerRtspTransportFromStrInvalidString},
	{FN("testHistogramChannelFromStrInvalidString"),
	 testHistogramChannelFromStrInvalidString},
	{FN("testVideoRendererSchedulingModeFromStrInvalidString"),
	 testVideoRendererSchedulingModeFromStrInvalidString},
	{FN("testVideoRendererFillModeFromStrInvalidString"),
	 testVideoRendererFillModeFromStrInvalidString},
	{FN("testVideoRendererTransitionFlagFromStrInvalidString"),
	 testVideoRendererTransitionFlagFromStrInvalidString},
	{FN("testVipcSourceEosReasonFromStrInvalidString"),
	 testVipcSourceEosReasonFromStrInvalidString},
	{FN("testCxxDemuxerAutodecodingModeRoundTrips"),
	 testCxxDemuxerAutodecodingModeRoundTrips},
	{FN("testCxxPlaybackTypeRoundTrips"), testCxxPlaybackTypeRoundTrips},
	{FN("testCxxMediaTypeRoundTrips"), testCxxMediaTypeRoundTrips},
	{FN("testCxxHistogramChannelRoundTrips"),
	 testCxxHistogramChannelRoundTrips},
	{FN("testCxxVideoRendererSchedulingModeRoundTrips"),
	 testCxxVideoRendererSchedulingModeRoundTrips},
	{FN("testCxxVideoRendererFillModeRoundTrips"),
	 testCxxVideoRendererFillModeRoundTrips},
	{FN("testCxxVideoRendererTransitionFlagRoundTrips"),
	 testCxxVideoRendererTransitionFlagRoundTrips},
	{FN("testCxxVipcSourceEosReasonRoundTrips"),
	 testCxxVipcSourceEosReasonRoundTrips},
	{FN("testCxxMediaInfoDupRoundTrips"), testCxxMediaInfoDupRoundTrips},
	{FN("testCxxVipcSourceParamsDupRoundTrips"),
	 testCxxVipcSourceParamsDupRoundTrips},
	{FN("testCxxMuxerParamsDupRoundTrips"),
	 testCxxMuxerParamsDupRoundTrips},
	{FN("testCxxMuxerMediaParamsDupRoundTrips"),
	 testCxxMuxerMediaParamsDupRoundTrips},
	{FN("testCxxDemuxerMediaListFreeWithRealContent"),
	 testCxxDemuxerMediaListFreeWithRealContent},
	{FN("testCxxVideoFrameToJsonRoundTrips"),
	 testCxxVideoFrameToJsonRoundTrips},
	{FN("testCxxVideoFrameToJsonStrRoundTrips"),
	 testCxxVideoFrameToJsonStrRoundTrips},
	{FN("testCxxVideoFrameToJsonCodedRoundTrips"),
	 testCxxVideoFrameToJsonCodedRoundTrips},
	{FN("testCxxVideoFrameToJsonWithVmetaIncludesMetadata"),
	 testCxxVideoFrameToJsonWithVmetaIncludesMetadata},
	{FN("testCApiDemuxerAutodecodingModeStrInvalid"),
	 testCApiDemuxerAutodecodingModeStrInvalid},
	{FN("testCApiPlaybackTypeStrInvalid"), testCApiPlaybackTypeStrInvalid},
	{FN("testCApiMediaTypeStrInvalid"), testCApiMediaTypeStrInvalid},
	{FN("testCApiMediaInfoDupRoundTrips"), testCApiMediaInfoDupRoundTrips},
	{FN("testCApiVipcSourceParamsDupRoundTrips"),
	 testCApiVipcSourceParamsDupRoundTrips},
	{FN("testCApiMuxerParamsDupRoundTrips"),
	 testCApiMuxerParamsDupRoundTrips},
	{FN("testCApiMuxerMediaParamsDupRoundTrips"),
	 testCApiMuxerMediaParamsDupRoundTrips},
	{FN("testCApiVideoFrameToJsonRoundTrips"),
	 testCApiVideoFrameToJsonRoundTrips},
	{FN("testCApiVideoFrameToJsonStrRoundTrips"),
	 testCApiVideoFrameToJsonStrRoundTrips},
	{FN("testCApiAlsaSourceGetCapabilitiesNullAddress"),
	 testCApiAlsaSourceGetCapabilitiesNullAddress},
	{FN("testCApiAlsaSourceGetCapabilitiesNullCaps"),
	 testCApiAlsaSourceGetCapabilitiesNullCaps},
	{FN("testCApiAlsaSourceGetCapabilitiesInvalidAddress"),
	 testCApiAlsaSourceGetCapabilitiesInvalidAddress},
	CU_TEST_INFO_NULL,
};
