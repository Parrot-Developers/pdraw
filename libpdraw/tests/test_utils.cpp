/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — pdraw_utils (Tier A)
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

#define ULOG_TAG pdraw_test_utils
#include "pdraw_utils.hpp"
#include "test_common.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <json-c/json.h>

ULOG_DECLARE_TAG(ULOG_TAG);


/* ── pdraw_gcd ───────────────────────────────────────────────────────── */

static void testGcdBasic()
{
	CU_ASSERT_EQUAL(pdraw_gcd(12, 8), 4u);
	CU_ASSERT_EQUAL(pdraw_gcd(100, 75), 25u);
}


static void testGcdZero()
{
	/* gcd(a, 0) == a by the Euclidean convention */
	CU_ASSERT_EQUAL(pdraw_gcd(7, 0), 7u);
	CU_ASSERT_EQUAL(pdraw_gcd(0, 5), 5u);
}


static void testGcdEqual()
{
	CU_ASSERT_EQUAL(pdraw_gcd(9, 9), 9u);
}


static void testGcdCoprime()
{
	CU_ASSERT_EQUAL(pdraw_gcd(7, 13), 1u);
}


/* ── pdraw_friendlyTimeFromUs ─────────────────────────────────────────── */

static void testFriendlyTimeFromUsSeconds()
{
	unsigned int h, m, s, ms;

	/* 90 500 000 µs = 1 min 30 s 500 ms */
	pdraw_friendlyTimeFromUs(90500000ULL, &h, &m, &s, &ms);
	CU_ASSERT_EQUAL(h, 0u);
	CU_ASSERT_EQUAL(m, 1u);
	CU_ASSERT_EQUAL(s, 30u);
	CU_ASSERT_EQUAL(ms, 500u);
}


static void testFriendlyTimeFromUsHoursMinutes()
{
	unsigned int h, m, s, ms;

	/* 3 661 001 000 µs = 1h 1m 1s 1ms */
	pdraw_friendlyTimeFromUs(3661001000ULL, &h, &m, &s, &ms);
	CU_ASSERT_EQUAL(h, 1u);
	CU_ASSERT_EQUAL(m, 1u);
	CU_ASSERT_EQUAL(s, 1u);
	CU_ASSERT_EQUAL(ms, 1u);
}


static void testFriendlyTimeFromUsZero()
{
	unsigned int h, m, s, ms;

	pdraw_friendlyTimeFromUs(0, &h, &m, &s, &ms);
	CU_ASSERT_EQUAL(h, 0u);
	CU_ASSERT_EQUAL(m, 0u);
	CU_ASSERT_EQUAL(s, 0u);
	CU_ASSERT_EQUAL(ms, 0u);

	/* Passing NULL for any output pointer must not crash. */
	pdraw_friendlyTimeFromUs(1000000ULL, nullptr, &m, &s, &ms);
	CU_ASSERT_EQUAL(m, 0u);
	CU_ASSERT_EQUAL(s, 1u);
}


/* ── pdraw_gaussianDistribution ──────────────────────────────────────── */

static void testGaussianDistributionPeak()
{
	/* With sigma=1, the peak at index 0 (normalised centre) should be
	 * strictly positive and larger than the tail value. */
	float samples[5] = {0};
	pdraw_gaussianDistribution(samples, 5, 1.0f);

	CU_ASSERT(samples[2] > 0.0f);
	CU_ASSERT(samples[2] > samples[0]);
	CU_ASSERT(samples[2] > samples[4]);
}


static void testGaussianDistributionSymmetry()
{
	float samples[7] = {0};
	pdraw_gaussianDistribution(samples, 7, 1.5f);

	/* Distribution should be symmetric around the centre index. */
	float tol = 1e-5f;
	CU_ASSERT(fabsf(samples[0] - samples[6]) < tol);
	CU_ASSERT(fabsf(samples[1] - samples[5]) < tol);
	CU_ASSERT(fabsf(samples[2] - samples[4]) < tol);
}

static void testGaussianDistributionNullSamplesIsNoop()
{
	/* samples == nullptr must return immediately without touching
	 * any memory (pdraw_utils.cpp line 173-174). */
	pdraw_gaussianDistribution(nullptr, 5, 1.0f);
	CU_PASS("nullptr samples did not crash");
}


static void testGaussianDistributionZeroCountIsNoop()
{
	/* sampleCount == 0 must return immediately (pdraw_utils.cpp
	 * lines 176-177). The buffer must remain all-zeros. */
	float samples[4] = {0.5f, 0.5f, 0.5f, 0.5f};
	pdraw_gaussianDistribution(samples, 0, 1.0f);

	/* Buffer must be untouched. */
	CU_ASSERT_EQUAL(samples[0], 0.5f);
	CU_ASSERT_EQUAL(samples[1], 0.5f);
}


static void testGaussianDistributionSingleSample()
{
	/* With a single sample, the distribution contains only one value,
	 * which must be normalised to 1. */
	float samples[1] = {0.0f};

	pdraw_gaussianDistribution(samples, 1, 1.0f);

	CU_ASSERT_EQUAL(samples[0], 1.0f);
}


static void testGaussianDistributionEvenCountDecremented()
{
	/* Even sampleCount is silently decremented to the next lower odd
	 * value before computing (pdraw_utils.cpp lines 178-179).
	 * Passing sampleCount=6 must produce the same result as passing
	 * sampleCount=5 (both use 5 effective samples). */
	float samplesEven[6] = {0};
	float samplesOdd[5] = {0};

	pdraw_gaussianDistribution(samplesEven, 6, 1.0f);
	pdraw_gaussianDistribution(samplesOdd, 5, 1.0f);

	float tol = 1e-6f;
	for (int i = 0; i < 5; i++)
		CU_ASSERT(fabsf(samplesEven[i] - samplesOdd[i]) < tol);

	/* The 6th element (index 5) must remain zero — it was never written
	 * because only 5 effective samples were computed. */
	CU_ASSERT_EQUAL(samplesEven[5], 0.0f);
}


/* ── pdraw_mediaInfoDup / pdraw_mediaInfoFree ─────────────────────────── */

static void testMediaInfoDupFreeRoundtrip()
{
	struct pdraw_media_info src = {};
	src.type = PDRAW_MEDIA_TYPE_VIDEO;
	src.id = 42;
	src.name = "test_stream";
	src.path = "/live/0";

	struct pdraw_media_info *dup = pdraw_mediaInfoDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);

	CU_ASSERT_EQUAL(dup->type, src.type);
	CU_ASSERT_EQUAL(dup->id, src.id);
	CU_ASSERT_PTR_NOT_EQUAL(dup->name, src.name);
	CU_ASSERT_STRING_EQUAL(dup->name, src.name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->path, src.path);
	CU_ASSERT_STRING_EQUAL(dup->path, src.path);

	pdraw_mediaInfoFree(dup);
}


/* ── pdraw_muxerParamsDup / pdraw_muxerParamsFree ────────────────────── */

static void testMuxerParamsDupFreeRoundtrip()
{
	struct pdraw_muxer_params src = {};
	src.recovery.tables_file = "recovery.bin";
	src.recovery.sync_period_ms = 500;

	struct pdraw_muxer_params *dup = pdraw_muxerParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);

	/* String must be a deep copy. */
	CU_ASSERT_PTR_NOT_EQUAL(dup->recovery.tables_file,
				src.recovery.tables_file);
	CU_ASSERT_STRING_EQUAL(dup->recovery.tables_file,
			       src.recovery.tables_file);
	CU_ASSERT_EQUAL(dup->recovery.sync_period_ms,
			src.recovery.sync_period_ms);

	pdraw_muxerParamsFree(dup);
}


static void testMuxerMediaParamsDupFreeRoundtrip()
{
	struct pdraw_muxer_media_params src = {};
	src.track_name = "VideoTrack";
	src.timescale = 90000;

	struct pdraw_muxer_media_params *dup = pdraw_muxerMediaParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);

	CU_ASSERT_PTR_NOT_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_STRING_EQUAL(dup->track_name, src.track_name);
	CU_ASSERT_EQUAL(dup->timescale, src.timescale);

	pdraw_muxerMediaParamsFree(dup);
}


static void testMuxerParamsDupNullTablesFile()
{
	/* tables_file == nullptr: no strdup branch taken; must not crash. */
	struct pdraw_muxer_params src = {};
	src.recovery.sync_period_ms = 1000;

	struct pdraw_muxer_params *dup = pdraw_muxerParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_PTR_NULL(dup->recovery.tables_file);
	CU_ASSERT_EQUAL(dup->recovery.sync_period_ms,
			src.recovery.sync_period_ms);

	pdraw_muxerParamsFree(dup);
}


static void testMuxerParamsDupNullReturnsNull()
{
	CU_ASSERT_PTR_NULL(pdraw_mediaInfoDup(nullptr));
	CU_ASSERT_PTR_NULL(pdraw_muxerParamsDup(nullptr));
	CU_ASSERT_PTR_NULL(pdraw_muxerMediaParamsDup(nullptr));
	CU_ASSERT_PTR_NULL(pdraw_vipcSourceParamsDup(nullptr));
}


static void testMuxerParamsFreeNull()
{
	/* All *Free helpers must tolerate nullptr. */
	pdraw_muxerParamsFree(nullptr);
	pdraw_muxerMediaParamsFree(nullptr);
	pdraw_vipcSourceParamsFree(nullptr);
	pdraw_mediaInfoFree(nullptr);
	CU_PASS("null-pointer free calls did not crash");
}


static void testVipcSourceParamsDupFreeRoundtrip()
{
	struct pdraw_vipc_source_params src = {};
	src.address = "vipc://camera";
	src.friendly_name = "FrontCamera";
	src.frame_count = 3;

	struct pdraw_vipc_source_params *dup = pdraw_vipcSourceParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);

	CU_ASSERT_PTR_NOT_EQUAL(dup->address, src.address);
	CU_ASSERT_STRING_EQUAL(dup->address, src.address);
	CU_ASSERT_PTR_NOT_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_STRING_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_PTR_NULL(dup->backend_name); /* was nullptr → no dup */
	CU_ASSERT_EQUAL(dup->frame_count, src.frame_count);

	pdraw_vipcSourceParamsFree(dup);
}


static void testMediaInfoDupNullFields()
{
	/* name and path nullptr: xstrdup(nullptr) must return nullptr, not
	 * crash */
	struct pdraw_media_info src = {};
	src.type = PDRAW_MEDIA_TYPE_VIDEO;
	src.id = 7;

	struct pdraw_media_info *dup = pdraw_mediaInfoDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_EQUAL(dup->type, src.type);
	CU_ASSERT_EQUAL(dup->id, src.id);
	CU_ASSERT_PTR_NULL(dup->name);
	CU_ASSERT_PTR_NULL(dup->path);

	pdraw_mediaInfoFree(dup);
}


static void testVipcSourceParamsDupAllStrings()
{
	/* All three string fields non-null: covers the backend_name branch */
	struct pdraw_vipc_source_params src = {};
	src.address = "vipc://main";
	src.friendly_name = "MainCam";
	src.backend_name = "hisi";
	src.frame_count = 5;

	struct pdraw_vipc_source_params *dup = pdraw_vipcSourceParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);

	CU_ASSERT_PTR_NOT_EQUAL(dup->address, src.address);
	CU_ASSERT_STRING_EQUAL(dup->address, src.address);
	CU_ASSERT_PTR_NOT_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_STRING_EQUAL(dup->friendly_name, src.friendly_name);
	CU_ASSERT_PTR_NOT_EQUAL(dup->backend_name, src.backend_name);
	CU_ASSERT_STRING_EQUAL(dup->backend_name, src.backend_name);
	CU_ASSERT_EQUAL(dup->frame_count, src.frame_count);

	pdraw_vipcSourceParamsFree(dup);
}


static void testMuxerMediaParamsDupNullTrackName()
{
	/* track_name nullptr: no strdup branch taken; scalar fields copied */
	struct pdraw_muxer_media_params src = {};
	src.timescale = 44100;

	struct pdraw_muxer_media_params *dup = pdraw_muxerMediaParamsDup(&src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dup);
	CU_ASSERT_PTR_NULL(dup->track_name);
	CU_ASSERT_EQUAL(dup->timescale, src.timescale);

	pdraw_muxerMediaParamsFree(dup);
}


static void testDemuxerMediaListFreePopulated()
{
	/* Build a 2-element list as the library would: malloc + strdup fields
	 */
	struct pdraw_demuxer_media *list =
		static_cast<struct pdraw_demuxer_media *>(
			calloc(2, sizeof(*list)));
	CU_ASSERT_PTR_NOT_NULL_FATAL(list);

	list[0].name = strdup("Video");
	list[0].uri = strdup("rtsp://host/video");
	list[1].name = strdup("Audio");
	list[1].uri = nullptr; /* uri is optional */

	pdraw_demuxerMediaListFree(list, 2);
	CU_PASS("no crash");
}


/* ── pdraw_frameMetadataToJson: null args / unknown format ────────────────
 * test_api_misc.cpp already covers this function's RAW/CODED branches (via
 * the C++ wrapper pdrawVideoFrameToJson()) and the `if (metadata)` branch,
 * but never: the `if (!frame || !jobj) return -EINVAL;` early return, nor
 * the `default:` switch branch taken when frame->format is neither
 * VDEF_FRAME_TYPE_RAW nor VDEF_FRAME_TYPE_CODED (e.g. the zero value
 * VDEF_FRAME_TYPE_UNKNOWN). That default branch only logs a warning
 * (ULOGW) and falls through: no "raw"/"coded" sub-object and no
 * has_errors/is_silent are added, but format/is_sync/is_ref/the four
 * ntp-, play-, capture- and local timestamps are still written and the
 * function still returns 0, not an error -- confirmed by reading
 * pdraw_utils.cpp, asserted explicitly below rather than assumed. */

static void testFrameMetadataToJsonNullArgs()
{
	struct pdraw_video_frame frame = {};
	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	CU_ASSERT_EQUAL(pdraw_frameMetadataToJson(nullptr, nullptr, jobj),
			-EINVAL);
	CU_ASSERT_EQUAL(pdraw_frameMetadataToJson(&frame, nullptr, nullptr),
			-EINVAL);
	CU_ASSERT_EQUAL(pdraw_frameMetadataToJson(nullptr, nullptr, nullptr),
			-EINVAL);

	json_object_put(jobj);
}


static void testFrameMetadataToJsonUnknownFormat()
{
	struct pdraw_video_frame frame = {};
	frame.format = VDEF_FRAME_TYPE_UNKNOWN;
	frame.is_sync = 1;

	struct json_object *jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	int ret = pdraw_frameMetadataToJson(&frame, nullptr, jobj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	const char *jstr = json_object_to_json_string(jobj);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jstr);
	CU_ASSERT_TRUE(strstr(jstr, "\"UNKNOWN\"") != nullptr);
	/* Neither branch's sub-object should be present. */
	CU_ASSERT_TRUE(strstr(jstr, "\"raw\"") == nullptr);
	CU_ASSERT_TRUE(strstr(jstr, "\"coded\"") == nullptr);
	CU_ASSERT_TRUE(strstr(jstr, "has_errors") == nullptr);
	CU_ASSERT_TRUE(strstr(jstr, "is_silent") == nullptr);
	/* But the unconditional fields are still written. */
	CU_ASSERT_TRUE(strstr(jstr, "is_sync") != nullptr);
	CU_ASSERT_TRUE(strstr(jstr, "ntp_timestamp") != nullptr);

	json_object_put(jobj);
}


/* ── pdraw_getTimestampFromMbufFrame: no ancillary data ────────────────────
 * None of the 3 overloads (raw/coded/audio video frame) had any test
 * (confirmed by grep across tests/): the "no ancillary data under this key"
 * path -- mbuf_xxx_frame_get_ancillary_data() returning -ENOENT -- is
 * silent (no ULOG* call) and simply returns 0 from `if (res < 0) return 0;`.
 * Each test below builds the minimal valid frame for its type: get_
 * ancillary_data() doesn't require the frame to be finalized (confirmed by
 * reading mbuf_base_frame_get_ancillary_data(), which only walks the
 * frame's ancillary_data list under a mutex), so no plane/NALU/buffer setup
 * is needed here, only a frame constructed with a valid format. */

static void testGetTimestampFromMbufRawFrameNoAncillaryData()
{
	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_i420;

	struct mbuf_raw_video_frame *frame = nullptr;
	int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_EQUAL(pdraw_getTimestampFromMbufFrame(frame, "no.such.key"),
			0);

	mbuf_raw_video_frame_unref(frame);
}


static void testGetTimestampFromMbufCodedFrameNoAncillaryData()
{
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_EQUAL(pdraw_getTimestampFromMbufFrame(frame, "no.such.key"),
			0);

	mbuf_coded_video_frame_unref(frame);
}


static void testGetTimestampFromMbufAudioFrameNoAncillaryData()
{
	struct adef_frame frameInfo = {};
	frameInfo.format = adef_pcm_16b_44100hz_mono;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_EQUAL(pdraw_getTimestampFromMbufFrame(frame, "no.such.key"),
			0);

	mbuf_audio_frame_unref(frame);
}


/* ── Enum string round-trips ─────────────────────────────────────────── */

static void testEnumStringRoundtripMediaType()
{
	static const enum pdraw_media_type kValues[] = {
		PDRAW_MEDIA_TYPE_UNKNOWN,
		PDRAW_MEDIA_TYPE_VIDEO,
		PDRAW_MEDIA_TYPE_AUDIO,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_mediaTypeStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_mediaTypeFromStr(str), kValues[i]);
	}
}


static void testEnumStringRoundtripPlaybackType()
{
	static const enum pdraw_playback_type kValues[] = {
		PDRAW_PLAYBACK_TYPE_UNKNOWN,
		PDRAW_PLAYBACK_TYPE_LIVE,
		PDRAW_PLAYBACK_TYPE_REPLAY,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_playbackTypeStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_playbackTypeFromStr(str), kValues[i]);
	}
}


static void testEnumStringRoundtripHistogramChannel()
{
	static const enum pdraw_histogram_channel kValues[] = {
		PDRAW_HISTOGRAM_CHANNEL_RED,
		PDRAW_HISTOGRAM_CHANNEL_GREEN,
		PDRAW_HISTOGRAM_CHANNEL_BLUE,
		PDRAW_HISTOGRAM_CHANNEL_LUMA,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_histogramChannelStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_histogramChannelFromStr(str), kValues[i]);
	}
}


static void testEnumStringRoundtripDemuxerAutodecodingMode()
{
	static const enum pdraw_demuxer_autodecoding_mode kValues[] = {
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL,
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_demuxerAutodecodingModeStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_demuxerAutodecodingModeFromStr(str),
				kValues[i]);
	}
}


static void testEnumStringRoundtripMuxerRtspTransport()
{
	static const enum pdraw_muxer_rtsp_transport kValues[] = {
		PDRAW_MUXER_RTSP_TRANSPORT_UDP,
		PDRAW_MUXER_RTSP_TRANSPORT_TCP,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_muxerRtpTransportStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_muxerRtpTransportFromStr(str),
				kValues[i]);
	}
}


/* pdraw_muxerConnectionStateStr() has no FromStr counterpart (unlike the
 * other enum Str functions above) -- only the enum -> string direction
 * exists in pdraw_utils.hpp/.cpp, so this checks the mapped string value
 * directly instead of a roundtrip, plus the "INVALID" fallback for a
 * value absent from pdraw_muxer_connection_state_map. */
static void testEnumStringMuxerConnectionState()
{
	static const struct {
		enum pdraw_muxer_connection_state val;
		const char *str;
	} kValues[] = {
		{PDRAW_MUXER_CONNECTION_STATE_UNKNOWN, "UNKNOWN"},
		{PDRAW_MUXER_CONNECTION_STATE_DISCONNECTED, "DISCONNECTED"},
		{PDRAW_MUXER_CONNECTION_STATE_CONNECTING, "CONNECTING"},
		{PDRAW_MUXER_CONNECTION_STATE_CONNECTED, "CONNECTED"},
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_muxerConnectionStateStr(kValues[i].val);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_STRING_EQUAL(str, kValues[i].str);
	}

	CU_ASSERT_STRING_EQUAL(
		pdraw_muxerConnectionStateStr(
			static_cast<enum pdraw_muxer_connection_state>(999)),
		"INVALID");
}


/* Same rationale as testEnumStringMuxerConnectionState() above:
 * pdraw_muxerDisconnectionReasonStr() has no FromStr counterpart either. */
static void testEnumStringMuxerDisconnectionReason()
{
	static const struct {
		enum pdraw_muxer_disconnection_reason val;
		const char *str;
	} kValues[] = {
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
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str =
			pdraw_muxerDisconnectionReasonStr(kValues[i].val);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_STRING_EQUAL(str, kValues[i].str);
	}

	CU_ASSERT_STRING_EQUAL(
		pdraw_muxerDisconnectionReasonStr(
			static_cast<enum pdraw_muxer_disconnection_reason>(
				999)),
		"INVALID");
}


static void testEnumStringRoundtripVideoRendererFillMode()
{
	static const enum pdraw_video_renderer_fill_mode kValues[] = {
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT,
		PDRAW_VIDEO_RENDERER_FILL_MODE_CROP,
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP,
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_videoRendererFillModeStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_videoRendererFillModeFromStr(str),
				kValues[i]);
	}
}


static void testEnumStringRoundtripVipcSourceEosReason()
{
	static const enum pdraw_vipc_source_eos_reason kValues[] = {
		PDRAW_VIPC_SOURCE_EOS_REASON_NONE,
		PDRAW_VIPC_SOURCE_EOS_REASON_RESTART,
		PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION,
		PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT,
	};
	for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); i++) {
		const char *str = pdraw_vipcSourceEosReasonStr(kValues[i]);
		CU_ASSERT_PTR_NOT_NULL(str);
		CU_ASSERT_EQUAL(pdraw_vipcSourceEosReasonFromStr(str),
				kValues[i]);
	}
}


CU_TestInfo g_pdraw_test_utils[] = {
	{FN("testGcdBasic"), testGcdBasic},
	{FN("testGcdZero"), testGcdZero},
	{FN("testGcdEqual"), testGcdEqual},
	{FN("testGcdCoprime"), testGcdCoprime},
	{FN("testFriendlyTimeFromUsSeconds"), testFriendlyTimeFromUsSeconds},
	{FN("testFriendlyTimeFromUsHoursMinutes"),
	 testFriendlyTimeFromUsHoursMinutes},
	{FN("testFriendlyTimeFromUsZero"), testFriendlyTimeFromUsZero},
	{FN("testGaussianDistributionPeak"), testGaussianDistributionPeak},
	{FN("testGaussianDistributionSymmetry"),
	 testGaussianDistributionSymmetry},
	{FN("testGaussianDistributionNullSamplesIsNoop"),
	 testGaussianDistributionNullSamplesIsNoop},
	{FN("testGaussianDistributionZeroCountIsNoop"),
	 testGaussianDistributionZeroCountIsNoop},
	{FN("testGaussianDistributionSingleSample"),
	 testGaussianDistributionSingleSample},
	{FN("testGaussianDistributionEvenCountDecremented"),
	 testGaussianDistributionEvenCountDecremented},
	{FN("testMediaInfoDupFreeRoundtrip"), testMediaInfoDupFreeRoundtrip},
	{FN("testMediaInfoDupNullFields"), testMediaInfoDupNullFields},
	{FN("testMuxerParamsDupFreeRoundtrip"),
	 testMuxerParamsDupFreeRoundtrip},
	{FN("testMuxerParamsDupNullTablesFile"),
	 testMuxerParamsDupNullTablesFile},
	{FN("testMuxerParamsDupNullReturnsNull"),
	 testMuxerParamsDupNullReturnsNull},
	{FN("testMuxerParamsFreeNull"), testMuxerParamsFreeNull},
	{FN("testMuxerMediaParamsDupFreeRoundtrip"),
	 testMuxerMediaParamsDupFreeRoundtrip},
	{FN("testMuxerMediaParamsDupNullTrackName"),
	 testMuxerMediaParamsDupNullTrackName},
	{FN("testVipcSourceParamsDupFreeRoundtrip"),
	 testVipcSourceParamsDupFreeRoundtrip},
	{FN("testVipcSourceParamsDupAllStrings"),
	 testVipcSourceParamsDupAllStrings},
	{FN("testDemuxerMediaListFreePopulated"),
	 testDemuxerMediaListFreePopulated},
	{FN("testFrameMetadataToJsonNullArgs"),
	 testFrameMetadataToJsonNullArgs},
	{FN("testFrameMetadataToJsonUnknownFormat"),
	 testFrameMetadataToJsonUnknownFormat},
	{FN("testGetTimestampFromMbufRawFrameNoAncillaryData"),
	 testGetTimestampFromMbufRawFrameNoAncillaryData},
	{FN("testGetTimestampFromMbufCodedFrameNoAncillaryData"),
	 testGetTimestampFromMbufCodedFrameNoAncillaryData},
	{FN("testGetTimestampFromMbufAudioFrameNoAncillaryData"),
	 testGetTimestampFromMbufAudioFrameNoAncillaryData},
	{FN("testEnumStringRoundtripMediaType"),
	 testEnumStringRoundtripMediaType},
	{FN("testEnumStringRoundtripPlaybackType"),
	 testEnumStringRoundtripPlaybackType},
	{FN("testEnumStringRoundtripHistogramChannel"),
	 testEnumStringRoundtripHistogramChannel},
	{FN("testEnumStringRoundtripDemuxerAutodecodingMode"),
	 testEnumStringRoundtripDemuxerAutodecodingMode},
	{FN("testEnumStringRoundtripMuxerRtspTransport"),
	 testEnumStringRoundtripMuxerRtspTransport},
	{FN("testEnumStringMuxerConnectionState"),
	 testEnumStringMuxerConnectionState},
	{FN("testEnumStringMuxerDisconnectionReason"),
	 testEnumStringMuxerDisconnectionReason},
	{FN("testEnumStringRoundtripVideoRendererFillMode"),
	 testEnumStringRoundtripVideoRendererFillMode},
	{FN("testEnumStringRoundtripVipcSourceEosReason"),
	 testEnumStringRoundtripVipcSourceEosReason},
	CU_TEST_INFO_NULL,
};
