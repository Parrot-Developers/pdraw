/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Demuxer API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_demuxer

#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-metadata/vmeta_frame.h>

#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

/* Needed for pdraw_demuxerMediaListFree(), used to free getMediaList()
 * output (not part of the public pdraw.h, only of the internal utils). Must
 * be included here, BEFORE the `#define private public` block below:
 * pdraw_element.hpp itself includes pdraw_utils.hpp, which drags in Eigen ->
 * <complex> -> <sstream>. If that chain got its first parse while
 * `private`/`protected` are redefined to `public`, standard library classes
 * (e.g. std::basic_stringbuf) would get parsed with corrupted access
 * specifiers, which the compiler rejects as a redeclaration mismatch the
 * next time any TU includes <sstream> normally. Including pdraw_utils.hpp
 * here first lets that whole chain parse normally; its header guards then
 * make the (still-needed, for RecordDemuxer/Element) pdraw_utils.hpp
 * re-inclusion below a no-op, so the macro only ever touches Pdraw's own
 * classes. */
#include "pdraw_utils.hpp"

/* Only needed to reach RecordDemuxer's internal per-track media objects
 * (DemuxerCodedVideoMedia/DemuxerRawVideoMedia/DemuxerAudioMedia) and their
 * private mCurrentFrame/mCurrentMem members, in order to directly exercise
 * the unref-on-flush/stop/teardown branches that a normal demux run never
 * hits (processSample() always nulls them out again before returning), plus
 * Element's own protected setState()/mReadyToPlay/etc. used by the
 * RecordDemuxer precondition-guard tests. Same trick as
 * test_pipeline_demuxer_stream.cpp uses for StreamDemuxer -- and, like that
 * file, this block MUST come before test_api_common.hpp/test_fixtures.hpp:
 * those transitively pull in pdraw_element.hpp too, and its include guard
 * would make a later #include here a no-op, leaving the real (non-macroed)
 * `protected`/`private` keywords in effect. */
#define private public
#define protected public
#include "pdraw_demuxer_record.hpp"
#include "pdraw_element.hpp"
#undef protected
#undef private

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── MP4/M4A fixtures (real files, from the NAS assets share) ───────────── */
/* @see PDRAW_GET_ASSET_PATH / PDRAW_TEST_ASSETS_ROOT in test_api_common.hpp.
 * Add more relative_path entries here to broaden coverage later (e.g.
 * different resolutions, multi-track, H.265). */

enum {
	ASSET_VIDEO_H264 = 0,
	ASSET_AUDIO_AAC = 1,
	/* Unlike champs_240p30_h264.mp4 above, this stream-sharing recording
	 * carries real Parrot session metadata (drone model/serial/
	 * friendly_name) and real per-frame vmeta (GPS/attitude), so it is
	 * used below to exercise RecordDemuxer::fetchSessionMetadata() and
	 * DemuxerCodedVideoMedia::processSample()'s vmeta_frame_read() path
	 * with actual non-empty values, which the other fixtures never do. */
	ASSET_VIDEO_H264_WITH_METADATA = 2,
	/* H.265, HDR, with MDCV/CLL SEI messages: exercises
	 * DemuxerCodedVideoMedia::h265MdcvSeiCb()/h265CllSeiCb()
	 * (pdraw_demuxer_record_coded_video_media.cpp), previously 0% covered
	 * since no other fixture is H.265 nor carries HDR SEI. */
	ASSET_VIDEO_H265_HDR_TC = 3,
	/* H.265 with a SEI time code message: exercises
	 * DemuxerCodedVideoMedia::h265TimeCodeSeiCb(), previously 0% covered.
	 * Kept as a separate fixture from ASSET_VIDEO_H265_HDR_TC above (only
	 * used for the MDCV/CLL test) since it targets a different SEI
	 * message. */
	ASSET_VIDEO_H265_PIC_TIMING_SEI = 4,
	/* Real multitrack recording: 2 coded video tracks + 1 raw (thermal)
	 * video track + 1 audio track, the coded and raw video tracks all
	 * carrying real per-track session metadata and real per-frame vmeta.
	 * Every fixture above carries exactly one video track (coded XOR
	 * raw): this is the first one used in this file with more than 2
	 * tracks total, exercising RecordDemuxer::processSelectedMedias()
	 * with a selection wider than the usual single "1 video + 1 audio"
	 * default, and DemuxerRawVideoMedia::processSample()'s
	 * vmeta_frame_read() path (pdraw_demuxer_record_raw_video_media.cpp)
	 * with real (non-synthetic) data for the first time --
	 * testCxxRawVideoTrackRoundtripsThroughDemuxerSink below mux/demuxes
	 * a synthetic raw8 track that carries no metadata at all. */
	ASSET_VIDEO_MULTITRACK_THERMAL = 5,
	/* Structurally valid MP4 (ftyp/moov present) with 0 trak boxes --
	 * NOT a 0-byte file, which would instead fail synchronously inside
	 * mp4_demux_open() (-ENODATA) and never reach completeStart()'s
	 * "no media track" branch (pdraw_demuxer_record.cpp:276-280) this
	 * fixture targets. */
	ASSET_VIDEO_EMPTY = 6,
	/* Legacy "regis" recording: 1 coded (AVC) track + 1 raw (i420) track,
	 * both carrying the legacy string session metadata key
	 * "com.parrot.regis.first_timestamp" (value 93086972837 on this
	 * fixture). This is the only fixture in this file carrying that key,
	 * and therefore the only way to exercise the strtol() parsing block
	 * in DemuxerCodedVideoMedia::setupMedia()
	 * (pdraw_demuxer_record_coded_video_media.cpp) and
	 * DemuxerRawVideoMedia::setupMedia()
	 * (pdraw_demuxer_record_raw_video_media.cpp), previously 0% covered.
	 * Every other coded/raw fixture above predates this legacy key. */
	ASSET_VIDEO_REGIS_RAW_CODED = 7,
	/* Same multitrack layout as ASSET_VIDEO_MULTITRACK_THERMAL above (2
	 * coded video tracks + 1 raw thermal video track + 1 audio track),
	 * but ~7s long / ~59 video frames instead of ~2s / 15 -- needed
	 * because DEMUXER_RECORD_{CODED,RAW}_VIDEO_MEDIA_OUTPUT_BUFFER_COUNT
	 * is 30 (pdraw_demuxer_record.hpp): the "very short" fixture's 15
	 * video frames per track can never exhaust a 30-buffer pool, so it
	 * can't exercise the "failed to get an input buffer" PDRAW_LOGW()
	 * branch in DemuxerCodedVideoMedia/DemuxerRawVideoMedia::
	 * processSample(). This fixture's audio track alone would already be
	 * enough for DemuxerAudioMedia's own 60-buffer pool (80 frames on the
	 * very-short fixture), but a single fixture covering all 3 branches
	 * at once is simpler than juggling two. */
	ASSET_VIDEO_MULTITRACK_THERMAL_LONGER = 8,
	/* Same 6-track layout as ASSET_VIDEO_MULTITRACK_THERMAL (2 coded video
	 * tracks + 1 raw thermal video track + 1 audio track), but with the
	 * LAST sample's declared size (in the stsz box) of tracks 1
	 * (DefaultVideo, coded), 3 (DefaultAudio) and 4 (ThermalVideo, raw)
	 * inflated to ~1e6/1e6/1e9 bytes -- far above what any of the 3
	 * processSample() implementations' fixed-capacity output buffer could
	 * ever hold (4 KB for audio, width*height*3/4 for coded video, the
	 * exact computed raw frame size for raw video). No sample data or
	 * byte offsets are touched, only those 3 stsz entries, generated with
	 * a one-off script (not part of this repo) that rewrites just those
	 * 4-byte fields in a copy of ASSET_VIDEO_MULTITRACK_THERMAL. Since
	 * it's the LAST sample of each track that's corrupted, every sample
	 * before it is delivered untouched (no stco/stsc offset is disturbed),
	 * and this exercises the "mp4_demux_get_track_sample" PDRAW_LOG_ERRNO()
	 * branch (distinct from the pool-exhaustion PDRAW_LOGW() branch above)
	 * in all 3 Demuxer{Audio,CodedVideo,RawVideo}Media::processSample():
	 * the "get a sample size" probe call (index 0) still succeeds since it
	 * only reads the (corrupted) stsz table already parsed in memory, but
	 * the "get a sample" call (index 1) then fails with -ENOBUFS since the
	 * declared size no longer fits the output buffer. Track sample counts
	 * on the underlying ASSET_VIDEO_MULTITRACK_THERMAL fixture: 15 for
	 * both coded video tracks, 80 for audio, 15 for raw -- so with only
	 * the very last sample of each corrupted, the tests below expect
	 * exactly 14/79/14 delivered frames. */
	ASSET_VIDEO_MULTITRACK_THERMAL_STSZ_CORRUPT = 9,
	/* A literal 0-byte file: unlike ASSET_VIDEO_EMPTY above (structurally
	 * valid MP4, 0 trak boxes), this has no ftyp/moov box at all, so
	 * mp4_demux_open() itself fails synchronously -- exercises
	 * RecordDemuxer::start()'s "Create the MP4 demuxer" failure branch
	 * (pdraw_demuxer_record.cpp:209-213), previously 0% covered since
	 * every other fixture in this file at least opens successfully. */
	ASSET_VIDEO_ZERO_BYTE = 10,
	/* Single raw-video-over-METADATA-track fixture, mime_format ==
	 * "video/raw;format=i420" (no "resolution=" CSV param) -- covers the
	 * "resolutionStr" sscanf() fallback in DemuxerRawVideoMedia::
	 * setupMedia() (pdraw_demuxer_record_raw_video_media.cpp:194-206),
	 * previously 0% covered: every other fixture with a "video/raw;..."
	 * mime_format (including ASSET_VIDEO_REGIS_RAW_CODED) carries a full
	 * "resolution=" CSV param that vdef_format_info_from_csv() would
	 * otherwise overwrite. Since that function only ever merges fields
	 * present in the CSV string and never resets absent ones (confirmed by
	 * reading libvideo-defs/src/vdefs.c), the resolution set here from the
	 * legacy "com.parrot.regis.resolution" (214x120) string survives
	 * untouched. The "com.parrot.regis.format" key on this track is
	 * "yvu420", which is not recognized by either legacy format branch
	 * ("raw32"/"grey") -- irrelevant here since the mime_format's
	 * "format=i420" already takes precedence for the format itself. */
	ASSET_VIDEO_REGIS_RAW_LEGACY_META_RESOLUTION_FALLBACK = 11,
	/* Same single raw-video-over-METADATA-track layout, but mime_format is
	 * the bare "video/raw" (no trailing ";", no CSV params at all) and
	 * "com.parrot.regis.format" is "raw32" -- covers the pure legacy path
	 * end-to-end: both format ("raw32" -> vdef_raw32) and resolution
	 * (214x120) come exclusively from the "com.parrot.regis.*" string
	 * metadata, since the bare mime_format fails the
	 * strncmp(..., VDEF_RAW_MIME_TYPE ";", ...) prefix check (no ";") and
	 * never enters the CSV-parsing branch. Previously 0% covered: no other
	 * fixture in this file exercises the "raw32"/"grey" branches at
	 * pdraw_demuxer_record_raw_video_media.cpp:187-193. */
	ASSET_VIDEO_REGIS_RAW_LEGACY_META_RAW32 = 12,
	/* Same layout, mime_format is also bare "video/raw" but
	 * "com.parrot.regis.format" is "yvu420" -- not recognized by any
	 * format branch (neither "raw32"/"grey" nor the CSV path, which is
	 * skipped entirely for the same reason as above). unknownFormat stays
	 * true, so DemuxerRawVideoMedia::setupMedia() returns -ENOSYS and
	 * RecordDemuxer::processSelectedMedias() never pushes this track's
	 * wrapper into RecordDemuxer::mMedias (pdraw_demuxer_record.cpp:
	 * 1149-1156) -- the whole demuxer still opens/reports ready-to-play
	 * normally (processSelectedMedias()'s return value is discarded by its
	 * only caller, RecordDemuxer::completeStart()), but this one track is
	 * silently never wired into the pipeline. Previously untested via a
	 * real fixture/open flow (only exercised in isolation, via a
	 * hand-crafted mp4_track_info, by
	 * testCxxDemuxerRawVideoMediaSetupMediaErrorPaths). */
	ASSET_VIDEO_REGIS_RAW_LEGACY_META_UNSUPPORTED_FORMAT = 13,
	/* Same layout, mime_format is bare "video/raw" (skips the CSV path) and
	 * "com.parrot.regis.format" is the valid "raw32", but
	 * "com.parrot.regis.resolution" is the garbage string "bad_resolution"
	 * and "com.parrot.regis.first_timestamp" is the garbage string
	 * "bad_first_ts" -- covers two previously-untested branches at once:
	 * the strtol() failure path for first_timestamp
	 * (pdraw_demuxer_record_raw_video_media.cpp:169-173: values[i][0] is
	 * not '\0' and endptr does land past the parsed digits, but since none
	 * of the string is a valid number endptr[0] != '\0', so mFirstTs is
	 * left untouched and only a PDRAW_LOG_ERRNO() is emitted -- ret is
	 * overwritten by later calls in setupMedia() before being used,
	 * so this branch has no assertable side effect beyond the log), and
	 * the sscanf() failure path for resolution (:199-202: ret != 2, so
	 * info.resolution.{width,height} are forced to 0). Unlike
	 * ASSET_VIDEO_REGIS_RAW_LEGACY_META_UNSUPPORTED_FORMAT above, format is
	 * valid here (unknownFormat stays false), so setupMedia() instead
	 * returns -ENOSYS from the *next* check, "invalid raw video media
	 * resolution" (:233-236, previously 0% covered) -- the track is still
	 * never pushed into RecordDemuxer::mMedias and the demuxer still opens
	 * normally, same observable behavior as the unsupported-format case but
	 * through a distinct code path. */
	ASSET_VIDEO_REGIS_RAW_LEGACY_META_BAD_VALUES = 14,
	/* Single coded (AVC) video track with valid SPS/PPS, carrying the same
	 * "com.parrot.regis.*" static metadata as
	 * ASSET_VIDEO_REGIS_RAW_LEGACY_META_BAD_VALUES above (resolution=
	 * "bad_resolution", format="raw32", first_timestamp="bad_first_ts") --
	 * covers the strtol() failure path for first_timestamp in
	 * DemuxerCodedVideoMedia::setupMedia()
	 * (pdraw_demuxer_record_coded_video_media.cpp:216-227), a separate copy
	 * of the same parsing logic already covered for raw video, previously
	 * untested for the coded path. Unlike the raw fixture, "com.parrot.
	 * regis.resolution"/"format" are not read at all by
	 * DemuxerCodedVideoMedia::setupMedia() (resolution comes from the AVC
	 * decoder config instead), so they're inert here -- kept only because
	 * it's the same source track reused as-is. Since the SPS/PPS are valid,
	 * setupMedia() still succeeds despite the bad first_timestamp: the
	 * strtol() failure only skips setting mFirstTs (left at its
	 * UINT64_MAX default) and logs, it doesn't reject the track. */
	ASSET_VIDEO_REGIS_CODED_LEGACY_META_BAD_VALUES = 15,
};

static const struct {
	const char *relative_path;
} s_assets_tests_pdraw_demux[] = {
	{"Tests/anafi/4k/video_recording/champs_240p30_h264.mp4"},
	{"Tests/miscellaneous/audio.m4a"},
	{"Tests/anafi3/classic/stream_sharing/stream_rec_240p.MP4"},
	{"Tests/miscellaneous/h265_240p_hdr_tc.mp4"},
	{"Tests/miscellaneous/h265_240p_pic_timing_sei.mp4"},
	{"Tests/anafi3/classic/video_recording/1080p30_very_short_thermal.mp4"},
	{"Tests/miscellaneous/empty.mp4"},
	{"Tests/miscellaneous/regis_120p30_raw_coded.mp4"},
	{"Tests/anafi3/classic/video_recording/1080p30_short_thermal.MP4"},
	{"Tests/anafi3/classic/video_recording/1080p30_very_short_thermal_stsz_corrupt.MP4"},
	{"Tests/miscellaneous/empty_file.mp4"},
	{"Tests/miscellaneous/regis_120p30_raw_legacy_meta_resolution_fallback.mp4"},
	{"Tests/miscellaneous/regis_120p30_raw_legacy_meta_raw32.mp4"},
	{"Tests/miscellaneous/regis_120p30_raw_legacy_meta_unsupported_format.mp4"},
	{"Tests/miscellaneous/regis_120p30_raw_legacy_meta_bad_values.mp4"},
	{"Tests/miscellaneous/regis_120p30_coded_legacy_meta_bad_values.mp4"},
};


/* Demuxer listener that records the async events needed to drive the
 * open/ready-to-play/close state machine from a test, since the
 * corresponding pdraw callbacks are dispatched via idle handlers on the
 * pomp loop rather than called synchronously. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class RecordingDemuxerListener : public IPdraw::IDemuxer::Listener {
public:
	void demuxerOpenResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status) override
	{
		mOpenStatus = status;
		mGotOpenResponse = true;
	}

	void demuxerCloseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int status) override
	{
		mCloseStatus = status;
		mGotCloseResponse = true;
	}

	void onDemuxerUnrecoverableError(IPdraw * /*p*/,
					 IPdraw::IDemuxer * /*d*/) override
	{
		mGotUnrecoverableError = true;
	}

	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*m*/,
			       size_t /*c*/,
			       uint32_t /*sel*/) override
	{
		/* -ENOSYS: not implemented, choose the default medias. */
		return -ENOSYS;
	}

	void demuxerReadyToPlay(IPdraw * /*p*/,
				IPdraw::IDemuxer * /*d*/,
				bool ready) override
	{
		mReady = ready;
		mGotReadyToPlay = true;
	}

	void onDemuxerEndOfRange(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 uint64_t /*ts*/) override
	{
	}

	void demuxerPlayResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		mPlayStatus = status;
		mPlayTimestamp = timestamp;
		mPlaySpeed = speed;
		mGotPlayResponse = true;
	}

	void demuxerPauseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int status,
				  uint64_t timestamp) override
	{
		mPauseStatus = status;
		mPauseTimestamp = timestamp;
		mGotPauseResponse = true;
	}

	void demuxerSeekResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		mSeekStatus = status;
		mSeekTimestamp = timestamp;
		mSeekSpeed = speed;
		mGotSeekResponse = true;
	}

	bool mGotOpenResponse = false;
	int mOpenStatus = 0;
	bool mGotReadyToPlay = false;
	bool mReady = false;
	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
	bool mGotUnrecoverableError = false;
	bool mGotPlayResponse = false;
	int mPlayStatus = 0;
	uint64_t mPlayTimestamp = 0;
	float mPlaySpeed = 0.f;
	bool mGotPauseResponse = false;
	int mPauseStatus = 0;
	uint64_t mPauseTimestamp = 0;
	bool mGotSeekResponse = false;
	int mSeekStatus = 0;
	uint64_t mSeekTimestamp = 0;
	float mSeekSpeed = 0.f;
};


/* Open the given fixture file and pump the loop until the demuxer reports
 * ready-to-play. Shared by the behavioral tests below to avoid repeating
 * the same open/params/pump boilerplate in each of them. */
static IPdraw::IDemuxer *
openDemuxerAndWaitReady(IPdraw *session,
			RecordingDemuxerListener *listener,
			size_t assetIndex = ASSET_VIDEO_H264)
{
	char path[512];
	PDRAW_GET_ASSET_PATH(path, assetIndex, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady = g_test_loop->pumpUntil(
		[listener]() { return listener->mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(listener->mReady);

	return obj;
}

/* Close and destroy a demuxer opened with openDemuxerAndWaitReady(),
 * waiting for the close response so the underlying mp4 demux context is
 * fully released before the next test runs. */
static void closeAndDestroyDemuxer(IPdraw::IDemuxer *obj,
				   RecordingDemuxerListener *listener)
{
	int ret = obj->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = g_test_loop->pumpUntil(
		[listener]() { return listener->mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);
	auto objOwner = std::unique_ptr<IPdraw::IDemuxer>(obj);
}


/* ── C API (pdraw_demuxer_*) ─────────────────────────────────────── */

static void testCApiNewFromUrl()
{
	struct pdraw_demuxer_params params = {};
	struct pdraw_demuxer *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_demuxer_new_from_url(nullptr,
					 "file:///dev/null",
					 &params,
					 &g_stub_demuxer_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullUrl */
	ret = pdraw_demuxer_new_from_url(g_test_pdraw_c,
					 nullptr,
					 &params,
					 &g_stub_demuxer_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* EmptyUrl */
	ret = pdraw_demuxer_new_from_url(g_test_pdraw_c,
					 "",
					 &params,
					 &g_stub_demuxer_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_demuxer_new_from_url(g_test_pdraw_c,
					 "file:///dev/null",
					 nullptr,
					 &g_stub_demuxer_cbs,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_demuxer_new_from_url(g_test_pdraw_c,
					 "file:///dev/null",
					 &params,
					 nullptr,
					 nullptr,
					 &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_demuxer_new_from_url(g_test_pdraw_c,
					 "file:///dev/null",
					 &params,
					 &g_stub_demuxer_cbs,
					 nullptr,
					 nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiClose()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_demuxer_close(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullDemuxer */
	ret = pdraw_demuxer_close(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_demuxer_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullDemuxer */
	ret = pdraw_demuxer_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiSingleStreamLifecycle()
{
	struct pdraw_demuxer_params params = {};
	struct pdraw_demuxer_cbs cbs = {};
	struct pdraw_demuxer *obj = nullptr;
	int ret;

	ret = pdraw_demuxer_new_single_stream(g_test_pdraw_c,
					      "127.0.0.1",
					      0,
					      0,
					      "127.0.0.1",
					      0,
					      0,
					      &params,
					      &cbs,
					      nullptr,
					      &obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(obj);

	if (obj != nullptr) {
		ret = pdraw_demuxer_destroy(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


static void testCApiPlay()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_demuxer_play(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullDemuxer */
	ret = pdraw_demuxer_play(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiPause()
{
	int ret = pdraw_demuxer_pause(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiSeek()
{
	int ret = pdraw_demuxer_seek(g_test_pdraw_c, nullptr, 0, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiIsReadyToPlay()
{
	/* Both null pdraw and null demuxer must return 0 (false / error) */
	CU_ASSERT_EQUAL(pdraw_demuxer_is_ready_to_play(nullptr, nullptr), 0);
	CU_ASSERT_EQUAL(pdraw_demuxer_is_ready_to_play(g_test_pdraw_c, nullptr),
			0);
}


static void testCApiIsPaused()
{
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(nullptr, nullptr), 0);
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(g_test_pdraw_c, nullptr), 0);
}


static void testCApiGetDuration()
{
	CU_ASSERT_EQUAL(pdraw_demuxer_get_duration(nullptr, nullptr), 0ULL);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_duration(g_test_pdraw_c, nullptr),
			0ULL);
}


static void testCApiGetCurrentTime()
{
	CU_ASSERT_EQUAL(pdraw_demuxer_get_current_time(nullptr, nullptr), 0ULL);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_current_time(g_test_pdraw_c, nullptr),
			0ULL);
}


static void testCApiGetSingleStreamLocalPorts()
{
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_stream_port(
				nullptr, nullptr),
			0);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_stream_port(
				g_test_pdraw_c, nullptr),
			0);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_control_port(
				nullptr, nullptr),
			0);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_control_port(
				g_test_pdraw_c, nullptr),
			0);
}


/* ── C API — forwarders exercised on a real, ready demuxer handle ────────
 * Every testCApi* test above only ever passes a null pdraw/demuxer: it
 * proves the ULOG_ERRNO_RETURN_ERR_IF() guards work, but never reaches the
 * one-line forward to the C++ Demuxer (pdraw_wrapper.cpp) underneath --
 * that forwarding line for get_media_list()/select_media()/seek*()/
 * next_frame()/previous_frame()/play_with_speed()/get_chapter_list()/
 * get_single_stream_local_*_port() is only ever exercised through
 * IPdraw::IDemuxer directly, by the testCxx* tests further below (confirmed
 * via gcov: those pdraw_demuxer_*() C entry points showed 0 hits before
 * this test). This test does not re-verify the underlying behavior --
 * already covered there, with tighter assertions -- it only drives each C
 * wrapper once through a real struct pdraw_demuxer *, using the same
 * open/play/pause/seek sequencing already proven safe by
 * testCxxPlayPauseLifecycle/testCxxSeekRelative/testCxxPreviousNextFrame. */

struct DemuxerCApiUserdata {
	bool gotOpenResp = false;
	int openStatus = 0;
	bool gotReadyToPlay = false;
	bool ready = false;
	bool gotPlayResp = false;
	int playStatus = 0;
	bool gotPauseResp = false;
	int pauseStatus = 0;
	bool gotSeekResp = false;
	int seekStatus = 0;
	uint64_t seekTimestamp = 0;
	bool gotCloseResp = false;
	int closeStatus = 0;
	bool gotUnrecoverableError = false;
};

static void testCApiMethodsValid()
{
	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	DemuxerCApiUserdata ud;
	struct pdraw_demuxer_cbs cbs = {};
	cbs.open_resp = [](struct pdraw *,
			   struct pdraw_demuxer *,
			   int status,
			   void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->openStatus = status;
		u->gotOpenResp = true;
	};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_demuxer *,
			       int ready,
			       void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->ready = (ready != 0);
		u->gotReadyToPlay = true;
	};
	cbs.close_resp = [](struct pdraw *,
			    struct pdraw_demuxer *,
			    int status,
			    void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->closeStatus = status;
		u->gotCloseResp = true;
	};
	cbs.unrecoverable_error = [](struct pdraw *,
				     struct pdraw_demuxer *,
				     void *userdata) {
		((DemuxerCApiUserdata *)userdata)->gotUnrecoverableError = true;
	};
	cbs.play_resp = [](struct pdraw *,
			   struct pdraw_demuxer *,
			   int status,
			   uint64_t,
			   float,
			   void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->playStatus = status;
		u->gotPlayResp = true;
	};
	cbs.pause_resp = [](struct pdraw *,
			    struct pdraw_demuxer *,
			    int status,
			    uint64_t,
			    void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->pauseStatus = status;
		u->gotPauseResp = true;
	};
	cbs.seek_resp = [](struct pdraw *,
			   struct pdraw_demuxer *,
			   int status,
			   uint64_t timestamp,
			   float,
			   void *userdata) {
		auto *u = (DemuxerCApiUserdata *)userdata;
		u->seekStatus = status;
		u->seekTimestamp = timestamp;
		u->gotSeekResp = true;
	};

	struct pdraw_demuxer *obj = nullptr;
	int ret = pdraw_demuxer_new_from_url(
		g_test_pdraw_c, path, &params, &cbs, &ud, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady =
		g_test_loop->pumpUntil([&ud]() { return ud.gotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(ud.ready);

	/* is_ready_to_play() / is_paused() / get_duration() /
	 * get_current_time(): testCApiIsReadyToPlay & co above only ever pass
	 * a null demuxer, so the real "return d->xxx();" line never ran. */
	CU_ASSERT_EQUAL(pdraw_demuxer_is_ready_to_play(g_test_pdraw_c, obj), 1);
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(g_test_pdraw_c, obj), 1);
	uint64_t duration = pdraw_demuxer_get_duration(g_test_pdraw_c, obj);
	CU_ASSERT_FATAL(duration > 0);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_current_time(g_test_pdraw_c, obj),
			0ULL);

	/* A local file (RecordDemuxer) is never a single/RTP stream: both
	 * ports must be 0, same contract as
	 * testCxxGetSingleStreamPortsAreZeroForLocalFile above. */
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_stream_port(
				g_test_pdraw_c, obj),
			0);
	CU_ASSERT_EQUAL(pdraw_demuxer_get_single_stream_local_control_port(
				g_test_pdraw_c, obj),
			0);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = pdraw_demuxer_get_media_list(
		g_test_pdraw_c, obj, &mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(mediaCount > 0);
	CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	/* Re-selecting the default medias, same as
	 * testCxxSelectMediaAfterReady's first assertion. */
	ret = pdraw_demuxer_select_media(g_test_pdraw_c, obj, 0);
	CU_ASSERT_EQUAL(ret, 0);

	/* getChapterList() is never exercised anywhere else in this file
	 * (C or C++): champs_240p30_h264.mp4 has no chapters, so
	 * RecordDemuxer::getChapterList() (pdraw_demuxer_record.cpp) takes
	 * its count==0 branch and returns -ENOENT with a null list -- still a
	 * real pass through the wrapper's forwarding line. */
	struct pdraw_chapter *chapterList = nullptr;
	size_t chapterCount = 0;
	ret = pdraw_demuxer_get_chapter_list(
		g_test_pdraw_c, obj, &chapterList, &chapterCount);
	CU_ASSERT_TRUE(ret == 0 || ret == -ENOENT);
	if (ret == 0) {
		for (size_t i = 0; i < chapterCount; i++)
			free(const_cast<char *>(chapterList[i].name));
		free(chapterList);
	}

	/* play_with_speed(): never called through the C API anywhere else
	 * (only obj->play(speed) in testCxxPlayBackwardWithNegativeSpeed). */
	ret = pdraw_demuxer_play_with_speed(g_test_pdraw_c, obj, 1.0f);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay =
		g_test_loop->pumpUntil([&ud]() { return ud.gotPlayResp; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL(ud.playStatus, 0);
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(g_test_pdraw_c, obj), 0);

	ret = pdraw_demuxer_pause(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause =
		g_test_loop->pumpUntil([&ud]() { return ud.gotPauseResp; });
	CU_ASSERT_TRUE_FATAL(gotPause);
	CU_ASSERT_EQUAL(ud.pauseStatus, 0);
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(g_test_pdraw_c, obj), 1);

	/* next_frame() / previous_frame(): both require the demuxer to
	 * already be paused, same precondition as testCxxPreviousNextFrame. */
	ud.gotSeekResp = false;
	ret = pdraw_demuxer_next_frame(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotNext =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotNext);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);

	ud.gotSeekResp = false;
	ret = pdraw_demuxer_previous_frame(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPrevious =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotPrevious);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);

	/* seek_to() / seek_forward() / seek_back() / seek(): none of these
	 * are ever called through the C API elsewhere -- only via
	 * IPdraw::IDemuxer in testCxxSeekToMidpoint/testCxxSeekRelative
	 * above. */
	ud.gotSeekResp = false;
	ret = pdraw_demuxer_seek_to(g_test_pdraw_c, obj, duration / 2, 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeekTo =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotSeekTo);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);
	uint64_t midTs = ud.seekTimestamp;

	ud.gotSeekResp = false;
	ret = pdraw_demuxer_seek_forward(g_test_pdraw_c, obj, duration / 8, 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotForward =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotForward);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);
	CU_ASSERT(ud.seekTimestamp >= midTs);

	ud.gotSeekResp = false;
	ret = pdraw_demuxer_seek_back(g_test_pdraw_c, obj, duration / 8, 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotBack =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotBack);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);

	ud.gotSeekResp = false;
	ret = pdraw_demuxer_seek(
		g_test_pdraw_c, obj, -static_cast<int64_t>(duration / 8), 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeek =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotSeek);
	CU_ASSERT_EQUAL(ud.seekStatus, 0);

	ret = pdraw_demuxer_close(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose =
		g_test_loop->pumpUntil([&ud]() { return ud.gotCloseResp; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(ud.closeStatus, 0);
	CU_ASSERT_FALSE(ud.gotUnrecoverableError);

	ret = pdraw_demuxer_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


/* ── C++ API (IPdraw::createDemuxer) ────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_demuxer_params params = {};
	IPdraw::IDemuxer *obj = nullptr;
	int ret;

	/* EmptyUrl */
	ret = session->createDemuxer(
		"", &params, &g_stub_demuxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullListener */
	ret = session->createDemuxer(
		"file:///dev/null", &params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = session->createDemuxer(
		"file:///dev/null", &params, &g_stub_demuxer_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* ── C++ API — behavioral tests on a real MP4 file ──────────────────────── */

static void testCxxOpenLocalFileReachesReadyToPlay()
{
	IPdraw *session = g_test_session->get();
	char path[512];
	PDRAW_GET_ASSET_PATH(path, 0, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);
	auto objOwner = std::unique_ptr<IPdraw::IDemuxer>(obj);

	bool gotOpen = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; });
	CU_ASSERT_TRUE_FATAL(gotOpen);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	bool gotReady = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE(listener.mReady);

	/* Behavioral checks now that the demuxer has actually parsed a real
	 * MP4 file, as opposed to the null-guard checks above. */
	CU_ASSERT_TRUE(obj->isReadyToPlay());
	/* play() was never called: RecordDemuxer starts paused (mRunning
	 * defaults to false), isPaused() only turns false once play() runs. */
	CU_ASSERT_TRUE(obj->isPaused());
	CU_ASSERT(obj->getDuration() > 0);
	CU_ASSERT_EQUAL(obj->getCurrentTime(), 0ULL);

	ret = obj->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotCloseResponse; });
	CU_ASSERT_TRUE_FATAL(gotClose);
	CU_ASSERT_EQUAL(listener.mCloseStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	objOwner.reset();
}


static void testCxxGetMediaListAfterOpen()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret = obj->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount > 0);
	CU_ASSERT_PTR_NOT_NULL(mediaList);
	CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);

	/* At least one selected media should be a video track marked as the
	 * default choice (auto-selected since demuxerSelectMedia() returned
	 * -ENOSYS above). */
	bool foundDefaultVideo = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if ((mediaList[i].type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    mediaList[i].is_default &&
		    (selectedMedias & (1U << mediaList[i].media_id)))
			foundDefaultVideo = true;
	}
	CU_ASSERT_TRUE(foundDefaultVideo);

	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxOpenAudioFileReachesReadyToPlay()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj =
		openDemuxerAndWaitReady(session, &listener, ASSET_AUDIO_AAC);

	/* Same behavioral contract as the video fixture: ready but paused,
	 * with a positive duration. */
	CU_ASSERT_TRUE(obj->isReadyToPlay());
	CU_ASSERT_TRUE(obj->isPaused());
	CU_ASSERT(obj->getDuration() > 0);
	CU_ASSERT_EQUAL(obj->getCurrentTime(), 0ULL);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret = obj->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount > 0);
	CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);

	/* An audio-only file must expose an audio (not video) default
	 * media, exercising the audio demuxing path (AAC-LC/ASC parsing)
	 * that the H.264 fixture above never touches. */
	bool foundDefaultAudio = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if ((mediaList[i].type == PDRAW_MEDIA_TYPE_AUDIO) &&
		    mediaList[i].is_default &&
		    (selectedMedias & (1U << mediaList[i].media_id)))
			foundDefaultAudio = true;
	}
	CU_ASSERT_TRUE(foundDefaultAudio);

	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxGetSingleStreamPortsAreZeroForLocalFile()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	/* A local file (RecordDemuxer) is not a StreamDemuxerNet: both ports
	 * must report 0, as documented for "no open on a mux channel". */
	CU_ASSERT_EQUAL(obj->getSingleStreamLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(obj->getSingleStreamLocalControlPort(), 0);

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxSelectMediaAfterReady()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	/* Re-selecting the default medias must succeed. */
	CU_ASSERT_EQUAL(obj->selectMedia(0), 0);

	/* A bitfield that matches no existing media_id: the demuxer ends up
	 * with an empty selection, which is reported as -ENOENT (not
	 * -EINVAL as one might expect from the API doc: there is no bitfield
	 * validation, only a check that at least one media matched). */
	CU_ASSERT_EQUAL(obj->selectMedia(1U << 30), -ENOENT);

	closeAndDestroyDemuxer(obj, &listener);
}


/* demuxerSelectMedia() returning -ECANCELED is never exercised anywhere else
 * in this file: every other listener (RecordingDemuxerListener included)
 * returns -ENOSYS, "not implemented, choose the default medias". -ECANCELED
 * is a distinct, intentional path in RecordDemuxer::start()
 * (pdraw_demuxer_record.cpp): unlike any other negative return (treated as
 * a real selection error, reported via onDemuxerUnrecoverableError()),
 * -ECANCELED means the application deliberately cancelled the open, so
 * "ready=false" must NOT be escalated to an unrecoverable error -- confirmed
 * by reading the exit: block's `if (!ready && (ret != -ECANCELED))` guard,
 * which exists specifically to skip onUnrecoverableError() in this case. */
static void testCxxSelectMediaCancelled()
{
	class CancellingDemuxerListener : public RecordingDemuxerListener {
	public:
		int demuxerSelectMedia(IPdraw * /*p*/,
				       IPdraw::IDemuxer * /*d*/,
				       const struct pdraw_demuxer_media * /*m*/,
				       size_t /*c*/,
				       uint32_t /*sel*/) override
		{
			return -ECANCELED;
		}
	};

	IPdraw *session = g_test_session->get();
	CancellingDemuxerListener listener;

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotOpen = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; });
	CU_ASSERT_TRUE_FATAL(gotOpen);
	/* openResponse() forwards the cancellation status as-is. */
	CU_ASSERT_EQUAL(listener.mOpenStatus, -ECANCELED);

	/* readyToPlay(false) is called internally, but Demuxer::readyToPlay()
	 * (pdraw_demuxer.cpp) only schedules the demuxerReadyToPlay() callback
	 * "on changes in value" -- mReadyToPlay already defaults to false, so
	 * this particular call is a no-op and the callback never fires (found
	 * the hard way: waiting on listener.mGotReadyToPlay here timed out).
	 * isReadyToPlay() is checked synchronously instead, which is safe
	 * since both openResponse() and readyToPlay() run synchronously
	 * inside RecordDemuxer::start()'s exit: block, before this idle
	 * callback for openResponse even fires. */
	CU_ASSERT_FALSE(listener.mGotReadyToPlay);
	CU_ASSERT_FALSE(obj->isReadyToPlay());

	/* The whole point of this test: per RecordDemuxer::start()'s exit:
	 * block, any other ret<0 with ready=false calls onUnrecoverableError()
	 * -- but the -ECANCELED case is carved out specifically so an
	 * application-cancelled selection is never reported as an error. */
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	closeAndDestroyDemuxer(obj, &listener);
}


/* Covers RecordDemuxer::completeStart()'s "no media track" branch
 * (pdraw_demuxer_record.cpp:276-280): mediasCount stays 0 when the MP4 file
 * has zero tracks recognized by isMediaTrack() (video/audio/raw-video mime).
 * Uses ASSET_VIDEO_EMPTY (Tests/miscellaneous/empty.mp4): a structurally
 * valid MP4 (ftyp/moov present) with 0 trak boxes -- confirmed by reading
 * mp4_demux_open()/mp4_tracks_build() (libmp4) that a trackless-but-valid
 * container opens successfully (track_count == 0), unlike a literal 0-byte
 * file which fails synchronously inside mp4_demux_open() itself (-ENODATA,
 * long before completeStart() ever runs).
 *
 * IMPORTANT finding while writing this test: completeStart()'s exit: block
 * (pdraw_demuxer_record.cpp:376-391) only calls openResponse()/
 * readyToPlay()/onUnrecoverableError() when ret == 0 or ret == -ECANCELED
 * (line 377). For ret == -ENOENT (this branch), execution falls to the
 * `else { setState(State::CREATED); }` at line 385/386 -- no callback at
 * all fires, contradicting pdraw.hpp's documented demuxerSelectMedia()
 * contract ("demuxerOpenResponse() ... or onDemuxerUnrecoverableError() ...
 * otherwise" -- worded for the sibling branch below, but the same exit:
 * block is shared). This looks like a genuine gap in the production code,
 * not a test-design issue; flagged here rather than silently worked around,
 * since fixing it is a production behavior change beyond "add test
 * coverage". As a result there is no positive application-visible signal
 * to assert for this branch: this test characterizes the current silent-
 * failure behavior (no callback ever fires) plus the one indirect signal
 * that does exist -- a later close() returns -EPROTO, since mState never
 * left State::CREATED and RecordDemuxer::stop() requires STARTED/STARTING. */
static void testCxxOpenFileWithNoMediaTracksNeverBecomesReady()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_EMPTY, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	/* Bounded wait: completeStart() runs on the very next idle callback,
	 * so this is ample -- nothing legitimate to wait longer for, since
	 * (per the finding above) no callback will ever fire. */
	(void)g_test_loop->pumpUntil([]() { return false; }, 1000);

	CU_ASSERT_FALSE(listener.mGotOpenResponse);
	CU_ASSERT_FALSE(listener.mGotReadyToPlay);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);
	CU_ASSERT_FALSE(obj->isReadyToPlay());

	/* Not closeAndDestroyDemuxer(): that helper asserts close()==0 and
	 * waits for mGotCloseResponse, neither of which will ever hold here. */
	ret = obj->close();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* Direct destruction is safe regardless of the failed close():
	 * ~DemuxerWrapper() (pdraw_demuxer.cpp) calls stop() unconditionally
	 * again and forces mElementStopped=true either way. */
	auto objOwner = std::unique_ptr<IPdraw::IDemuxer>(obj);
}


/* Covers RecordDemuxer::completeStart()'s "application failed to select a
 * media" branch (pdraw_demuxer_record.cpp:358-364): any demuxerSelectMedia()
 * return value other than a bitmask (>=0), -ENOSYS, or -ECANCELED. Same
 * finding as testCxxOpenFileWithNoMediaTracksNeverBecomesReady above applies
 * here: this branch doesn't set `ready = false` either (unlike its
 * -ECANCELED sibling just above it), but since ret stays a real negative
 * errno (not 0/-ECANCELED), the exit: block's `if` at line 377 is false
 * regardless -- no openResponse()/readyToPlay()/onUnrecoverableError() ever
 * fires, despite pdraw.hpp's documented contract saying one of the two
 * should. Same characterization-test shape: assert absence of callbacks,
 * then assert close() returns -EPROTO. */
static void testCxxSelectMediaHardErrorNeverBecomesReady()
{
	class FailingSelectMediaDemuxerListener
			: public RecordingDemuxerListener {
	public:
		int demuxerSelectMedia(IPdraw * /*p*/,
				       IPdraw::IDemuxer * /*d*/,
				       const struct pdraw_demuxer_media * /*m*/,
				       size_t /*c*/,
				       uint32_t /*sel*/) override
		{
			return -EINVAL;
		}
	};

	IPdraw *session = g_test_session->get();
	FailingSelectMediaDemuxerListener listener;

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	(void)g_test_loop->pumpUntil([]() { return false; }, 1000);

	CU_ASSERT_FALSE(listener.mGotOpenResponse);
	CU_ASSERT_FALSE(listener.mGotReadyToPlay);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);
	CU_ASSERT_FALSE(obj->isReadyToPlay());

	ret = obj->close();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	auto objOwner = std::unique_ptr<IPdraw::IDemuxer>(obj);
}


/* Covers RecordDemuxer::start()'s "Create the MP4 demuxer" failure branch
 * (pdraw_demuxer_record.cpp:209-213): mp4_demux_open() fails synchronously
 * on a literal 0-byte file (no ftyp/moov box at all to parse), unlike
 * ASSET_VIDEO_EMPTY above (structurally valid, just 0 tracks), which reaches
 * completeStart() just fine and fails there instead, much later. Every other
 * fixture in this file opens successfully, so this branch was previously 0%
 * covered.
 *
 * Session::createDemuxer() (pdraw_session.cpp) calls Element::start()
 * synchronously and, when it returns < 0, propagates that return value
 * directly WITHOUT ever setting *retObj -- so unlike every other test in
 * this file, there is no IPdraw::IDemuxer* to close/destroy afterward. The
 * local DemuxerWrapper instance inside createDemuxer() is destroyed on the
 * spot instead: its destructor (~DemuxerWrapper(), pdraw_demuxer.cpp) calls
 * Demuxer::stop() unconditionally, which is safe here since
 * RecordDemuxer::stop() explicitly allows State::STARTING -- exactly the
 * state start() left the element in right before failing. The underlying
 * Element stays owned by Session::mElements (pushed there before start() was
 * called) and is reaped later like any other stopped element; nothing more
 * for the test to do. */
static void testCxxRecordDemuxerOpenZeroByteFileFailsSynchronously()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_ZERO_BYTE, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_TRUE_FATAL(ret < 0);
	CU_ASSERT_PTR_NULL_FATAL(obj);

	/* No callback of any kind can ever fire: openResponse()/readyToPlay()/
	 * onUnrecoverableError() are only ever reached from completeStart(),
	 * which never gets a chance to run (start() failed before scheduling
	 * its idle callback). Bounded wait, same rationale as
	 * testCxxOpenFileWithNoMediaTracksNeverBecomesReady above. */
	(void)g_test_loop->pumpUntil([]() { return false; }, 1000);
	CU_ASSERT_FALSE(listener.mGotOpenResponse);
	CU_ASSERT_FALSE(listener.mGotReadyToPlay);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);
}


/* Covers RecordDemuxer::selectMedia(uint32_t)'s own "not started" guard
 * (pdraw_demuxer_record.cpp:1202-1205, `!mCalledOpenResp -> -EPROTO`),
 * previously 0% covered: the only other selectMedia() test
 * (testCxxSelectMediaAfterReady) always calls it after
 * openDemuxerAndWaitReady(), by which point completeStart() has already run
 * and called openResponse() (which sets mCalledOpenResp = true).
 *
 * Session::createDemuxer() calls RecordDemuxer::start() synchronously, but
 * start() only *schedules* completeStart() via an idle handler -- it does
 * not run inline. So calling selectMedia() immediately after createDemuxer()
 * returns, before ever pumping the loop, deterministically still finds
 * mCalledOpenResp == false (confirmed by asserting mGotOpenResponse is still
 * false right beforehand). */
static void testCxxRecordDemuxerSelectMediaBeforeOpenResponseGuard()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	CU_ASSERT_FALSE_FATAL(listener.mGotOpenResponse);
	CU_ASSERT_EQUAL(obj->selectMedia(0), -EPROTO);

	bool gotReady = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxPlayPauseLifecycle()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	/* Not playing yet: RecordDemuxer starts paused until play() runs. */
	CU_ASSERT_TRUE(obj->isPaused());

	int ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL(listener.mPlayStatus, 0);
	CU_ASSERT_FALSE(obj->isPaused());

	/* pause() is only safe to call once the play() response has cleared
	 * the pending command (otherwise RecordDemuxer::play() returns
	 * -EBUSY); pumpUntil() above already waited for that. */
	ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);
	CU_ASSERT_EQUAL(listener.mPauseStatus, 0);
	CU_ASSERT_TRUE(obj->isPaused());

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxSeekToMidpoint()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	uint64_t duration = obj->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	int ret = obj->seekTo(duration / 2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeek = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotSeek);
	CU_ASSERT_EQUAL(listener.mSeekStatus, 0);

	/* seekTo() clamps to [0, duration]; exact=false additionally snaps
	 * to a preceding sync sample, so only the range can be asserted. */
	CU_ASSERT(listener.mSeekTimestamp <= duration);
	CU_ASSERT(obj->getCurrentTime() <= duration);

	closeAndDestroyDemuxer(obj, &listener);
}


/* seek()/seekForward()/seekBack() are never exercised anywhere else in this
 * file -- only seekTo() is, above. All three ultimately reach
 * RecordDemuxer::seek(int64_t delta, bool exact) (pdraw_demuxer_record.cpp),
 * seekForward()/seekBack() just pass a signed version of their unsigned
 * delta through. */
static void testCxxSeekRelative()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	uint64_t duration = obj->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	/* Start from a known midpoint so seekBack() below has room to move
	 * backward without clamping at 0 (which would make the "did it
	 * actually move backward" check below meaningless). */
	int ret = obj->seekTo(duration / 2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeekTo = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotSeekTo);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);
	uint64_t midTs = listener.mSeekTimestamp;

	listener.mGotSeekResponse = false;
	ret = obj->seekForward(duration / 8);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotForward = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotForward);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);
	CU_ASSERT(listener.mSeekTimestamp >= midTs);
	CU_ASSERT(listener.mSeekTimestamp <= duration);
	uint64_t forwardTs = listener.mSeekTimestamp;

	listener.mGotSeekResponse = false;
	ret = obj->seekBack(duration / 8);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotBack = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotBack);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);
	CU_ASSERT(listener.mSeekTimestamp <= forwardTs);

	/* seek() with a negative delta behaves like seekBack(); with a
	 * positive delta, like seekForward(). */
	listener.mGotSeekResponse = false;
	ret = obj->seek(-static_cast<int64_t>(duration / 8));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeekNeg = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotSeekNeg);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);

	listener.mGotSeekResponse = false;
	ret = obj->seek(static_cast<int64_t>(duration / 8));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeekPos = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotSeekPos);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);

	closeAndDestroyDemuxer(obj, &listener);
}


/* previousFrame()/nextFrame() are never exercised anywhere else in this
 * file. Both require the demuxer to already be paused (see their doc
 * comments in pdraw.hpp: "If the playback is not currently paused an error
 * is returned"), hence the play()-then-pause() dance below, matching
 * testCxxPlayPauseLifecycle's proven sequencing. nextFrame() first (from
 * the start of the file, guaranteed to have a next frame), then
 * previousFrame() (back to where we started, guaranteed to have a previous
 * one) -- calling previousFrame() first, from frame 0, would risk hitting
 * an edge case (no previous frame at all) unrelated to what this test
 * wants to check. */
static void testCxxPreviousNextFrame()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	int ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);

	ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);
	CU_ASSERT_EQUAL_FATAL(listener.mPauseStatus, 0);
	CU_ASSERT_TRUE_FATAL(obj->isPaused());

	listener.mGotSeekResponse = false;
	ret = obj->nextFrame();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotNext = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotNext);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);

	listener.mGotSeekResponse = false;
	ret = obj->previousFrame();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPrevious = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotPrevious);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);

	closeAndDestroyDemuxer(obj, &listener);
}


/* play() is never called anywhere else in this file with a speed other than
 * the default (1.0, forward). RecordDemuxer::DemuxerMedia::onTimer()'s
 * "speed < 0. => play backward" branch (pdraw_demuxer_record.cpp) -- which
 * periodically seeks back to earlier sync samples instead of scheduling the
 * next sample forward -- is therefore never exercised: confirmed via gcov,
 * that whole branch showed 0 executions. Starting from the midpoint (rather
 * than frame 0) gives backward playback real room to move before hitting
 * the start of the file. */
static void testCxxPlayBackwardWithNegativeSpeed()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	uint64_t duration = obj->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	int ret = obj->seekTo(duration / 2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeek = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotSeekResponse; });
	CU_ASSERT_TRUE_FATAL(gotSeek);
	CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);
	uint64_t midTs = listener.mSeekTimestamp;

	ret = obj->play(-1.0f);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);
	CU_ASSERT_EQUAL(listener.mPlaySpeed, -1.0f);
	CU_ASSERT_FALSE(obj->isPaused());

	/* Let backward playback run long enough for the current position to
	 * move strictly before the starting midpoint -- the strongest
	 * available proof that onTimer()'s backward branch actually ran
	 * (rather than just that play() accepted a negative speed). */
	bool positionDecreased = g_test_loop->pumpUntil(
		[&]() { return obj->getCurrentTime() < midTs; }, 10000);
	CU_ASSERT_TRUE_FATAL(positionDecreased);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);
	CU_ASSERT_EQUAL_FATAL(listener.mPauseStatus, 0);

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::play()'s (and thus pause()'s -- DemuxerWrapper::pause() is
 * literally `return play(0.);`, the exact same code path) own "not started"
 * (pdraw_demuxer_record.cpp:763-765) and "not ready to play" (767-769)
 * guards, previously 0% covered: every existing play()/pause() test in this
 * file only calls them after openDemuxerAndWaitReady(), which already
 * guarantees both preconditions. Element::setState()/mReadyToPlay are forced
 * directly (as in test_pipeline_demuxer_stream.cpp's analogous StreamDemuxer
 * guard tests): DemuxerWrapper::play()/pause() have their own, separate
 * isElementStopped() guard ahead of the forwarding call, but that flag
 * (mElementStopped) is only ever flipped by the wrapper's own state-change
 * bookkeeping, not by directly poking the underlying element's mState, so
 * forcing mState here still reaches RecordDemuxer::play()'s own check. */
static void testCxxRecordDemuxerPlayGuards()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	/* Not started. */
	rd->setState(Element::State::STOPPING);
	CU_ASSERT_EQUAL(obj->play(), -EPROTO);
	CU_ASSERT_EQUAL(obj->pause(), -EPROTO);
	rd->setState(Element::State::STARTED);

	/* Not ready to play. */
	rd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(obj->play(), -EPROTO);
	CU_ASSERT_EQUAL(obj->pause(), -EPROTO);
	rd->mReadyToPlay = true;

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::play()'s own pending-command switch
 * (pdraw_demuxer_record.cpp:772-790), previously 0% covered: every existing
 * play()/pause() test in this file pumps the loop to completion between
 * calls, so a second command always finds getPendingCommand() == NONE by the
 * time it runs. Issuing a second command back-to-back with no pump in
 * between exercises every previously-uncovered outcome:
 *   - PAUSE_NEXT pending, PAUSE requested again (speed == 0) -> -EALREADY
 *   - PAUSE_NEXT pending, PLAY requested (speed != 0)        -> -EBUSY
 *   - PLAY pending, PLAY requested again (speed != 0)        -> -EALREADY
 *   - PLAY pending, PAUSE requested (speed == 0)             -> -EBUSY
 *   - PAUSE pending, PAUSE requested again (speed == 0)      -> -EALREADY
 *   - PAUSE pending, PLAY requested (speed != 0)             -> -EBUSY
 *   - any other pending command (SEEK, forced directly since a real seek()
 *     completes synchronously fast in this fixture) -> -EBUSY via the
 *     `default:` case, since SEEK has no explicit case in this switch.
 * A fresh demuxer has mWasRunningOnce == false, so its first pause() sets
 * Command::PAUSE_NEXT rather than Command::PAUSE (see the comment on
 * mWasRunningOnce in pdraw_demuxer_record.hpp) -- exercising that case
 * requires pausing before ever playing, so it comes first below. */
static void testCxxRecordDemuxerPlayPendingCommandGuard()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	/* PAUSE_NEXT pending -> PAUSE again (-EALREADY) -> PLAY (-EBUSY). */
	CU_ASSERT_FALSE_FATAL(rd->mWasRunningOnce);
	int ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(obj->pause(), -EALREADY);
	CU_ASSERT_EQUAL(obj->play(), -EBUSY);
	bool gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);

	/* PLAY pending -> PLAY again (-EALREADY) -> PAUSE (-EBUSY). */
	listener.mGotPlayResponse = false;
	ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(obj->play(), -EALREADY);
	CU_ASSERT_EQUAL(obj->pause(), -EBUSY);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);

	/* PAUSE pending (mWasRunningOnce is now true, since play() ran above)
	 * -> PAUSE again (-EALREADY) -> PLAY (-EBUSY). */
	CU_ASSERT_TRUE_FATAL(rd->mWasRunningOnce);
	listener.mGotPauseResponse = false;
	ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(obj->pause(), -EALREADY);
	CU_ASSERT_EQUAL(obj->play(), -EBUSY);
	gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);

	/* Any other pending command (forced directly, not reachable through
	 * play()/pause() themselves) -> -EBUSY via `default:`. */
	rd->setPendingCommand(Demuxer::Command::SEEK);
	CU_ASSERT_EQUAL(obj->play(), -EBUSY);
	CU_ASSERT_EQUAL(obj->pause(), -EBUSY);
	rd->clearPendingCommand();

	closeAndDestroyDemuxer(obj, &listener);
}


/* previousFrame()/nextFrame()'s (RecordDemuxer::previous()/next(),
 * pdraw_demuxer_record.cpp) shared "not started" (838-840/875-877), "not
 * ready to play" (842-844/879-881) and "not paused" (846-848/883-885) guards,
 * plus their pending-command guards (851-856 for previous(); 888-902 for
 * next()), previously 0% covered: the only other test exercising these
 * (testCxxPreviousNextFrame) always calls them from an already
 * paused/frame-by-frame, no-pending-command state. mFrameByFrame defaults to
 * false on a freshly opened demuxer, so "not paused" is hit for free below
 * with no extra forcing needed, right after restoring the "not ready"
 * precondition.
 *
 * next()'s own extra allowance -- Command::PAUSE_NEXT pending AND
 * !mWasRunningOnce falls through to a `break` (i.e. proceeds normally)
 * instead of `-EBUSY` -- is deliberately NOT exercised here: reaching it
 * needs a demuxer that starts directly in frame-by-frame mode before any
 * play()/pause() has ever run (the "start in pause" feature), which
 * openDemuxerAndWaitReady()'s params don't request, and forcing it via raw
 * field writes would leave a real, artificially-triggered seek in flight
 * with no natural precondition backing it up. Left as a known gap (see
 * TEST_PROGRESS.md). */
static void testCxxRecordDemuxerPreviousNextGuards()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	/* Not started. */
	rd->setState(Element::State::STOPPING);
	CU_ASSERT_EQUAL(obj->previousFrame(), -EPROTO);
	CU_ASSERT_EQUAL(obj->nextFrame(), -EPROTO);
	rd->setState(Element::State::STARTED);

	/* Not ready to play. */
	rd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(obj->previousFrame(), -EPROTO);
	CU_ASSERT_EQUAL(obj->nextFrame(), -EPROTO);
	rd->mReadyToPlay = true;

	/* Not paused: mFrameByFrame is still false here, its default value on
	 * a freshly opened demuxer that has never played/paused. */
	CU_ASSERT_FALSE_FATAL(rd->mFrameByFrame);
	CU_ASSERT_EQUAL(obj->previousFrame(), -EPROTO);
	CU_ASSERT_EQUAL(obj->nextFrame(), -EPROTO);

	/* A command is pending (forced directly) -> -EBUSY for both. */
	rd->mFrameByFrame = true;
	rd->setPendingCommand(Demuxer::Command::SEEK);
	CU_ASSERT_EQUAL(obj->previousFrame(), -EBUSY);
	CU_ASSERT_EQUAL(obj->nextFrame(), -EBUSY);
	rd->clearPendingCommand();
	rd->mFrameByFrame = false;

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::seek()'s own "not started" (922-924), "not ready to play"
 * (926-928) and pending-command (931-936) guards, previously 0% covered:
 * unlike StreamDemuxer::seek() (which delegates everything to seekTo()),
 * RecordDemuxer::seek() has its own independent copy of all three checks, so
 * they need their own coverage here rather than being implied by
 * seekTo()'s. */
static void testCxxRecordDemuxerSeekGuards()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	/* Not started. */
	rd->setState(Element::State::STOPPING);
	CU_ASSERT_EQUAL(obj->seek(1000, false), -EPROTO);
	rd->setState(Element::State::STARTED);

	/* Not ready to play. */
	rd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(obj->seek(1000, false), -EPROTO);
	rd->mReadyToPlay = true;

	/* A command is pending (forced directly) -> -EBUSY. */
	rd->setPendingCommand(Demuxer::Command::SEEK);
	CU_ASSERT_EQUAL(obj->seek(1000, false), -EBUSY);
	rd->clearPendingCommand();

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::seekTo()'s own "not started" (955-957), "not ready to play"
 * (959-961) and pending-command (964-969) guards, previously 0% covered:
 * testCxxSeekToMidpoint only ever calls seekTo() on an already
 * started/ready/idle demuxer. */
static void testCxxRecordDemuxerSeekToGuards()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	uint64_t duration = obj->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	/* Not started. */
	rd->setState(Element::State::STOPPING);
	CU_ASSERT_EQUAL(obj->seekTo(duration / 2), -EPROTO);
	rd->setState(Element::State::STARTED);

	/* Not ready to play. */
	rd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(obj->seekTo(duration / 2), -EPROTO);
	rd->mReadyToPlay = true;

	/* A command is pending (forced directly) -> -EBUSY. */
	rd->setPendingCommand(Demuxer::Command::SEEK);
	CU_ASSERT_EQUAL(obj->seekTo(duration / 2), -EBUSY);
	rd->clearPendingCommand();

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::isPaused()'s "not started" guard (pdraw_demuxer_record.cpp:
 * 825-827), previously 0% covered: every other isPaused() call in this file
 * runs on an already-started demuxer. isPaused() itself just returns false
 * in this case (no error code to check -- bool API), so the only observable
 * proof is the return value dropping to false regardless of the real
 * play/pause state underneath. */
static void testCxxRecordDemuxerIsPausedNotStartedGuard()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	/* Paused (the default state) while started -- baseline. */
	CU_ASSERT_TRUE(obj->isPaused());

	rd->setState(Element::State::STOPPING);
	CU_ASSERT_FALSE(obj->isPaused());
	rd->setState(Element::State::STARTED);

	closeAndDestroyDemuxer(obj, &listener);
}


/* RecordDemuxer::getChapterList()'s "not started" guard
 * (pdraw_demuxer_record.cpp:1005-1007), previously 0% covered:
 * testCApiMethodsValid only exercises the count==0 -> -ENOENT branch, on an
 * already-started demuxer. */
static void testCxxRecordDemuxerGetChapterListNotStartedGuard()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(obj);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	struct pdraw_chapter *chapterList = nullptr;
	size_t chapterCount = 0;

	rd->setState(Element::State::STOPPING);
	int ret = obj->getChapterList(&chapterList, &chapterCount);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	rd->setState(Element::State::STARTED);

	closeAndDestroyDemuxer(obj, &listener);
}


/* Shared by the 4 tests below: exercises RecordDemuxer::DemuxerMedia::
 * onTimer()'s "catch-up" while loop (pdraw_demuxer_record.cpp) -- entered
 * whenever the demuxer can't keep up with the requested playback schedule
 * (`newDuration - error < 0`) and has to seek ahead/behind to the next/
 * previous sync sample, dropping the samples in between. Forward: roughly
 * lines 1779-1848 (`mp4_demux_get_track_next_sample_time_after()`).
 * Backward: roughly lines 1720-1778 (`..._prev_sample_time_before()`).
 * Both were previously at 0% (per gcov communicated by the user).
 *
 * How each speed regime is made to actually enter the loop:
 * - Fast (|speed| well above 1, but below PDRAW_PLAY_SPEED_MAX=1000 --
 *   at/above MAX takes an entirely different, always-duration=0 fast path
 *   that never reaches this loop at all): the per-sample real-time budget
 *   (`duration = sample_interval / speed`) shrinks enough (a few hundred
 *   µs at 50x for a 30fps track) that ordinary pomp-loop dispatch +
 *   processSample()'s own CPU cost (real file I/O, NALU parsing, mbuf
 *   alloc) alone reliably exceeds it -- no artificial stall needed. This
 *   is the SAME mechanism TEST_PROGRESS.md already documents happening BY
 *   ACCIDENT at 1x speed on a loaded machine (for a different, unrelated
 *   test that deliberately steered away from it via PDRAW_PLAY_SPEED_MAX);
 *   here it's leaned into on purpose, at a speed chosen specifically to
 *   make it a near-certainty rather than a load-dependent maybe.
 * - Slow (|speed| < 1): `duration` grows instead, so a deliberate stall of
 *   the test thread (sleep_for, no loop pumping meanwhile -- same
 *   technique as testCxxAlsaSourceCapturedFrameRecoversFromOverrun in
 *   test_pipeline_alsa_source.cpp) is used to guarantee real elapsed time
 *   exceeds `duration` on the very next tick, deterministically and
 *   independent of host load.
 *
 * Observable proof used (no private-member access -- getCurrentTime() is
 * already public API): a catch-up seek makes obj->getCurrentTime() (==
 * RecordDemuxer::mCurrentTime, set from the REAL mp4-timeline dts of
 * whatever sample was actually read, in DemuxerCodedVideoMedia::
 * processSample()) jump discontinuously by far more than one nominal
 * frame interval between two consecutive polls -- unlike the frame's own
 * re-derived nominal timestamp field, which always advances by exactly
 * one nominal interval regardless of any seek and would never show a gap.
 * Measured as the largest single-poll-to-poll delta seen over a bounded
 * window (not "current value vs. a distant start value", which would
 * also trip on ordinary accumulated playback progress over enough real
 * time -- a false positive this design avoids).
 *
 * Assumption flagged as unverified (fixture content not inspectable from
 * this sandbox): champs_240p30_h264.mp4 (ASSET_VIDEO_H264, the only
 * fixture with a confirmed >2s duration) is assumed to have more than one
 * sync (key) sample, spaced meaningfully apart (a real "video_recording"
 * clip, plausibly ~1-2s GOP) -- if it turned out to be single-GOP
 * (intra-only), the catch-up loop would still be *entered* (covering the
 * lines targeted here) but would immediately `break` without finding a
 * later/earlier sync sample, and the "big jump" proof below might not
 * materialize meaningfully. */
static void runOnTimerCatchUpTest(float speed, bool stallToForceCatchUp)
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);

	uint64_t duration = obj->getDuration();
	CU_ASSERT_FATAL(duration > 2000000ULL);

	/* Backward playback needs room to move: start from the midpoint,
	 * same as testCxxPlayBackwardWithNegativeSpeed above. Forward
	 * playback starts from 0 (the default position after open). */
	if (speed < 0.f) {
		int seekRet = obj->seekTo(duration / 2);
		CU_ASSERT_EQUAL_FATAL(seekRet, 0);
		bool gotSeek = g_test_loop->pumpUntil(
			[&listener]() { return listener.mGotSeekResponse; });
		CU_ASSERT_TRUE_FATAL(gotSeek);
		CU_ASSERT_EQUAL_FATAL(listener.mSeekStatus, 0);
	}

	/* For backward speeds: pin firstTs to the seek target (duration/2)
	 * rather than reading getCurrentTime(), because the demuxer only
	 * updates mCurrentTime from processSample(), not from seekTo() -- so
	 * getCurrentTime() returns 0 both before AND after seekTo() until the
	 * first sample is output.  If firstTs==0==prevTs and the backward run
	 * ends at position 0 (demuxer reached the beginning), the step
	 * formula (now < prevTs) ? prevTs-now : 0 gives (0<0)?0:0 == 0 for
	 * every iteration, so maxStep never exceeds the threshold.
	 * Using duration/2 as the baseline guarantees the first iteration
	 * sees a large backward jump (0 < duration/2 → step == duration/2
	 * >> threshold) regardless of how fast the demuxer ran.
	 * For forward speeds getCurrentTime() is 0 (natural start position)
	 * and the forward step formula (now > prevTs) works correctly from
	 * there. */
	uint64_t firstTs =
		(speed < 0.f) ? (duration / 2) : obj->getCurrentTime();

	int ret = obj->play(speed);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);

	/* Let at least one real sample flow first so mLastSampleOutputTime/
	 * mLastSampleDuration are non-zero -- onTimer() forces error=0 on
	 * the very first tick after play()/seek otherwise
	 * (pdraw_demuxer_record.cpp:1700-1701), which would silently absorb
	 * a stall applied too early.
	 *
	 * Deliberately NOT _FATAL below (unlike the rest of this function):
	 * a real run showed the original threshold below was miscalibrated
	 * (see kBigJumpThresholdUs), and a _FATAL failure here would abort
	 * the function before reaching pause()/closeAndDestroyDemuxer() at
	 * the bottom -- leaking a still-playing RecordDemuxer that keeps
	 * firing onTimer() in the background on the *shared* g_test_session
	 * loop for the rest of the run. That leak was confirmed to be the
	 * actual root cause of 2 *other* tests failing in the same run
	 * (including a pre-existing, unrelated test) after this one first
	 * failed on a bad threshold -- one miscalibration cascaded into 3
	 * failures purely because cleanup never ran. Using non-fatal
	 * assertions here guarantees cleanup always runs regardless of
	 * whether the timing-based proof below succeeds. */
	/* Slow speeds (|speed|<1) need proportionally longer bounds: the
	 * backward branch's non-catch-up duration is itself based on the
	 * distance to the previous SYNC sample (sample.prev_sync_dts, not
	 * the previous individual sample -- pdraw_demuxer_record.cpp), which
	 * divided by a small |speed| can legitimately take several real
	 * seconds even with no catch-up involved. */
	bool slowSpeed = std::fabs(speed) < 1.f;
	/* Backward playback seeks in GOP-sized steps (~1s); the first step
	 * after play() can take up to one GOP interval of real time before
	 * the timer fires, so give it the same generous window as slow-speed
	 * forward. */
	bool needsMoreTime = slowSpeed || (speed < 0.f);
	uint32_t movedTimeoutMs = needsMoreTime ? 20000 : 5000;

	bool moved = g_test_loop->pumpUntil(
		[&]() { return obj->getCurrentTime() != firstTs; },
		movedTimeoutMs);
	CU_ASSERT_TRUE(moved);

	if (stallToForceCatchUp) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	/* Poll getCurrentTime() at a steady cadence and track the largest
	 * single-poll-to-poll jump seen, in the expected direction, over a
	 * bounded window. Deliberately starts from `firstTs` (NOT a fresh
	 * obj->getCurrentTime() read here): a real run showed the catch-up
	 * jump can already have happened by the time the `moved` wait above
	 * is satisfied (the very first tick can itself be the catch-up
	 * tick), so re-reading the current value here as the baseline threw
	 * the jump away before it could ever be measured -- starting from
	 * `firstTs` guarantees the first measured step captures however far
	 * things had already moved by the time `moved` became true.
	 *
	 * Threshold recalibrated from an actual run: at slow speed (0.1x),
	 * the real catch-up correction observed was a single seek forward
	 * of ~85ms (log: "unable to keep up with playback timings, seek
	 * forward 42.67 ms" followed by an actual landing ~85333us ahead of
	 * the start), NOT the >=1s jump originally assumed -- this fixture's
	 * sync samples are much more closely spaced than assumed. 50ms stays
	 * safely below that observed correction while remaining well above
	 * the nominal ~21-33ms per-frame advance seen in the same run (so a
	 * normal, non-catch-up tick can't spuriously trip it). */
	static const uint64_t kBigJumpThresholdUs = 50000ULL; /* 50ms */
	uint64_t prevTs = firstTs;
	uint64_t maxStep = 0;
	int iterations = needsMoreTime ? 1000 : 300;
	for (int i = 0; i < iterations && maxStep <= kBigJumpThresholdUs; i++) {
		g_test_loop->runOnce();
		uint64_t now = obj->getCurrentTime();
		uint64_t step = (speed < 0.f)
					? ((now < prevTs) ? (prevTs - now) : 0)
					: ((now > prevTs) ? (now - prevTs) : 0);
		if (step > maxStep)
			maxStep = step;
		prevTs = now;
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	CU_ASSERT_TRUE(maxStep > kBigJumpThresholdUs);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	ret = obj->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; });
	CU_ASSERT_TRUE_FATAL(gotPause);

	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxRecordDemuxerCatchUpSeekForwardFastSpeed()
{
	runOnTimerCatchUpTest(50.0f, /*stallToForceCatchUp=*/false);
}


static void testCxxRecordDemuxerCatchUpSeekForwardSlowSpeed()
{
	runOnTimerCatchUpTest(0.1f, /*stallToForceCatchUp=*/true);
}


static void testCxxRecordDemuxerCatchUpSeekBackwardFastSpeed()
{
	/* At -50x the GOP-step duration (~20ms) is large enough that
	 * timing error alone never drives the catch-up loop into action;
	 * force it with an artificial stall, same as the slow-speed case. */
	runOnTimerCatchUpTest(-50.0f, /*stallToForceCatchUp=*/true);
}


static void testCxxRecordDemuxerCatchUpSeekBackwardSlowSpeed()
{
	runOnTimerCatchUpTest(-0.1f, /*stallToForceCatchUp=*/true);
}


/* ── C++ API — real session/frame metadata (stream_rec_240p.MP4 fixture) ──
 * champs_240p30_h264.mp4 (used by every test above) has no useful embedded
 * vmeta, so RecordDemuxer::fetchSessionMetadata() and
 * DemuxerCodedVideoMedia::processSample()'s per-frame vmeta_frame_read()
 * path (pdraw_demuxer_record_coded_video_media.cpp) were never exercised
 * with real data. stream_rec_240p.MP4 is a stream-sharing recording that
 * does carry both, so the two tests below open it instead. ─────────────── */

static void testCxxGetMediaListSessionMetadataFromRealRecording()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(
		session, &listener, ASSET_VIDEO_H264_WITH_METADATA);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret = obj->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(mediaCount > 0);

	bool foundVideoWithSessionMeta = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		if (mediaList[i].video.session_meta.friendly_name[0] != '\0')
			foundVideoWithSessionMeta = true;
	}
	CU_ASSERT_TRUE(foundVideoWithSessionMeta);

	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	closeAndDestroyDemuxer(obj, &listener);
}


/* Minimal coded video sink listener, just enough to drain the queue on
 * flush/drain so the pipeline never stalls; per-file-local copy of the same
 * shape as QueueDrainingCodedVideoSinkListener in
 * test_pipeline_sourcesink_coded.cpp (anonymous namespace: see this file's
 * top-of-file comment on avoiding ODR violations across sibling test TUs). */
class DrainingCodedVideoSinkListener
		: public IPdraw::ICodedVideoSink::Listener {
public:
	/* Captures the coded video format info handed out at attach time (see
	 * ExternalCodedVideoSink::addInputMedia()): a fresh snapshot of
	 * CodedVideoMedia::info, taken at the moment THIS sink attaches --
	 * used by testCxxCodedVideoSinkExposesH265MdcvAndCllInfo() below to
	 * observe mdcv/cll set by an earlier sink's already-flowed frames. */
	void
	onCodedVideoSinkMediaAdded(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink * /*sk*/,
				   const struct pdraw_media_info *info) override
	{
		if (info != nullptr)
			mLastCodedInfo = info->video.coded.info;
		mGotMediaAdded = true;
	}

	void onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
					  IPdraw::ICodedVideoSink * /*sk*/,
					  const struct pdraw_media_info * /*i*/,
					  bool /*restart*/) override
	{
	}

	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		discardQueue();
		sk->queueFlushed();
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		discardQueue();
		sk->queueDrained();
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	struct mbuf_coded_video_frame_queue *mQueue = nullptr;
	bool mGotMediaAdded = false;
	struct vdef_format_info mLastCodedInfo = {};

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_coded_video_frame_unref(f);
	}
};


/* mediaId=0 below relies on RecordDemuxer::DemuxerCodedVideoMedia::
 * setupMedia() (pdraw_demuxer_record_coded_video_media.cpp) always creating
 * its AVCC-format CodedVideoMedia (codedMedias[0]) before its byte-stream one
 * (codedMedias[1]): ExternalCodedVideoSink::addInputMedia() accepts the
 * first offered coded media unconditionally when mTargetMediaId == 0 and
 * rejects any further one with -EBUSY (see
 * Session::PipelineFactory::addAllMediaToCodedVideoSink(), which offers
 * both in creation order once the sink reaches STARTED) -- so this
 * deterministically binds to the AVCC track, never the byte-stream one. */
static void testCxxCodedVideoSinkReceivesRealFrameMetadata()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(
		session, &listener, ASSET_VIDEO_H264_WITH_METADATA);

	struct pdraw_video_sink_params sinkParams = {};
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	int ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);

	ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);

	static const size_t kMinFrames = 5;
	std::vector<struct mbuf_coded_video_frame *> frames;
	bool gotFrames = g_test_loop->pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(
				       sinkListener.mQueue, &f) == 0)
				frames.push_back(f);
			return frames.size() >= kMinFrames;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	/* At least one of the first frames must carry real per-frame vmeta,
	 * proving DemuxerCodedVideoMedia::processSample() actually read and
	 * attached it via mbuf_coded_video_frame_set_metadata(). */
	bool foundFrameMeta = false;
	for (auto *f : frames) {
		struct vmeta_frame *meta = nullptr;
		int metaRet = mbuf_coded_video_frame_get_metadata(f, &meta);
		if (metaRet == 0 && meta != nullptr) {
			if (meta->type != VMETA_FRAME_TYPE_NONE)
				foundFrameMeta = true;
			vmeta_frame_unref(meta);
		}
		mbuf_coded_video_frame_unref(f);
	}
	CU_ASSERT_TRUE(foundFrameMeta);

	sinkOwner.reset();
	closeAndDestroyDemuxer(obj, &listener);
}


/* ── C++ API — H.265 SEI (h265_240p_hdr_tc.mp4 / h265_240p_pic_timing_sei.mp4
 * fixtures) ───────────────────────────────────────────────────────────────
 * mediaId=0 below relies on the same codedMedias[0]-is-created-first
 * guarantee documented above for
 * testCxxCodedVideoSinkReceivesRealFrameMetadata() (setupMedia() in
 * pdraw_demuxer_record_coded_video_media.cpp always builds the "packetized"
 * format first: codedMedias[0]->format = vdef_h265_hvcc, codedMedias[1]->format
 * = vdef_h265_byte_stream), so it deterministically binds to the HVCC-format
 * track. ─────────────────────────────────────── */

static void testCxxCodedVideoSinkReceivesH265TimeCodeCaptureTimestamp()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(
		session, &listener, ASSET_VIDEO_H265_PIC_TIMING_SEI);

	struct pdraw_video_sink_params sinkParams = {};
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	int ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);

	ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);

	static const size_t kMinFrames = 5;
	std::vector<struct mbuf_coded_video_frame *> frames;
	bool gotFrames = g_test_loop->pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(
				       sinkListener.mQueue, &f) == 0)
				frames.push_back(f);
			return frames.size() >= kMinFrames;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	/* h265TimeCodeSeiCb() sets mCurrentFrameCaptureTs from the SEI time
	 * code SEI message (payload type 136); unlike MDCV/CLL below, it is
	 * explicitly re-synced into frameInfo.info.capture_timestamp for the
	 * SAME frame right before mbuf_coded_video_frame_set_frame_info() is
	 * called (see DemuxerCodedVideoMedia::processSample()), so it IS
	 * observable per frame. The only other way capture_timestamp could
	 * end up non-zero (mFirstTs from a "com.parrot.regis.first_timestamp"
	 * track metadata key, a legacy raw-video-track marker) does not apply
	 * to a normal H.265 video track such as this fixture's.
	 * NOTE: struct h265_ctx_cbs (libh265) only exposes a sei_time_code
	 * hook for H.265, not sei_pic_timing (unlike H.264, which does wire
	 * up .sei_pic_timing = h264PicTimingSeiCb just above in this file) --
	 * pic_timing SEI messages (payload type 1) are not parsed/hooked at
	 * all here. This test therefore relies on
	 * h265_240p_pic_timing_sei.mp4 actually carrying a *time code* SEI
	 * message despite its name; if it only carries a pic_timing SEI (no
	 * time_code), h265TimeCodeSeiCb() never runs and this assertion will
	 * fail. */
	bool foundCaptureTs = false;
	for (auto *f : frames) {
		struct vdef_coded_frame info = {};
		int infoRet = mbuf_coded_video_frame_get_frame_info(f, &info);
		if (infoRet == 0 && info.info.capture_timestamp != 0)
			foundCaptureTs = true;
		mbuf_coded_video_frame_unref(f);
	}
	CU_ASSERT_TRUE(foundCaptureTs);

	sinkOwner.reset();
	closeAndDestroyDemuxer(obj, &listener);
}


static void testCxxCodedVideoSinkExposesH265MdcvAndCllInfo()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(
		session, &listener, ASSET_VIDEO_H265_HDR_TC);

	/* First sink: attaches immediately (mediaId=0), before any sample has
	 * been read, so CodedVideoMedia::info.mdcv/cll are still all-zero at
	 * that point -- h265MdcvSeiCb()/h265CllSeiCb() only run once
	 * processSample() starts parsing SEI NALUs, i.e. after play(). Its
	 * job here is just to pump the pipeline so those callbacks actually
	 * run at least once. */
	struct pdraw_video_sink_params sinkParams = {};
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	int ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);

	ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener.mPlayStatus, 0);

	static const size_t kMinFrames = 5;
	std::vector<struct mbuf_coded_video_frame *> frames;
	bool gotFrames = g_test_loop->pumpUntil(
		[&]() {
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(
				       sinkListener.mQueue, &f) == 0)
				frames.push_back(f);
			return frames.size() >= kMinFrames;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);
	for (auto *f : frames)
		mbuf_coded_video_frame_unref(f);

	/* CodedVideoMedia::info.mdcv/cll, updated in place by
	 * h265MdcvSeiCb()/h265CllSeiCb(), is only ever copied out into a
	 * pdraw_media_info snapshot at ExternalCodedVideoSink::
	 * addInputMedia() time -- i.e. once, when a sink first attaches
	 * (Session::PipelineFactory::addAllMediaToCodedVideoSink(), triggered
	 * once by the sink's own STARTED transition, itself called
	 * synchronously from Session::createCodedVideoSink()). It is never
	 * copied into per-frame info (struct vdef_frame_info has no mdcv/cll
	 * fields at all, unlike struct vdef_format_info which does), so it
	 * cannot be observed through the frames popped above. A second,
	 * later-created sink attaching to the SAME already-existing media
	 * gets its OWN fresh addInputMedia() call -- and by now several
	 * frames' worth of SEI have already updated CodedVideoMedia::info in
	 * place, so its snapshot is up to date. */
	DrainingCodedVideoSinkListener sinkListener2;
	IPdraw::ICodedVideoSink *sink2 = nullptr;
	ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener2, &sink2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink2);
	auto sink2Owner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink2);
	CU_ASSERT_TRUE_FATAL(sinkListener2.mGotMediaAdded);

	CU_ASSERT(sinkListener2.mLastCodedInfo.mdcv
			  .max_display_mastering_luminance > 0.f);
	CU_ASSERT(sinkListener2.mLastCodedInfo.cll.max_cll > 0u);

	sink2Owner.reset();
	sinkOwner.reset();
	closeAndDestroyDemuxer(obj, &listener);
}


/* ── C++ API — raw video track demuxing (self-contained fixture) ────────
 * DemuxerRawVideoMedia::processSample() (pdraw_demuxer_record_raw_video_
 * media.cpp) was 0% covered before this test: no NAS asset here has a raw
 * video track, and test_pipeline_muxer_record.cpp's
 * testCxxMuxerRecordsRawVideoTrack deliberately stops at confirming the
 * produced file is valid/re-openable (see its own comment) -- that suite's job
 * is the muxer, not the demuxer. This test picks up where that one leaves off:
 * mux a tiny synthetic raw8 MP4 (same IsobmffRecordMuxer raw8/raw16-only
 * constraint, see getSupportedRawFormats() in pdraw_muxer_record_isobmff.cpp),
 * then demux it back for real via an IRawVideoSink to actually exercise
 * processSample().
 *
 * Uses its own local TestPompLoop/TestSession, unlike every other test in
 * this file (which share g_test_session/g_test_loop): g_test_session's
 * listener is fixed to nullptr for the whole suite (see
 * pdraw_test_api_init() in test_fixtures.cpp), so onMediaAdded() -- needed
 * to learn the freshly created raw video source's own media id before it
 * can be muxed -- would never fire on it. */

class RawVideoTrackMediaListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override
	{
		/* Two distinct medias get added over the lifetime of this
		 * test: the source's own output media (phase 1, muxing) and
		 * the demuxer's output media (phase 2, re-reading the file).
		 * Both must be tracked by elementUserData identity -- info->id
		 * is a pipeline-wide Media::mId, unrelated to the MP4 track
		 * id reported by getMediaList()'s media_id field (that one is
		 * only meaningful for the demuxerSelectMedia() callback's
		 * selection bitmask, see pdraw_defs.h's doc comment on
		 * pdraw_demuxer_media::media_id -- using it to create a sink
		 * is what silently broke this test the first time: wrong id
		 * namespace, ExternalRawVideoSink::addInputMedia() returned
		 * -EPERM, which PipelineFactory::addMediaToRawVideoSink()
		 * treats as a harmless no-op, so no input channel was ever
		 * created and no frame ever had anywhere to go -- no crash,
		 * no error log, just an eternally empty sink queue). */
		if (elementUserData == mSource) {
			mMediaId = info->id;
			mGotMediaAdded = true;
		} else if (elementUserData == mDemuxer) {
			mDemuxerMediaId = info->id;
			mGotDemuxerMediaAdded = true;
		}
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	unsigned int mMediaId = 0;
	void *mDemuxer = nullptr;
	bool mGotDemuxerMediaAdded = false;
	unsigned int mDemuxerMediaId = 0;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};

class RecordingMuxerListener : public IPdraw::IMuxer::Listener {
public:
	void onMuxerConnectionStateChanged(
		IPdraw * /*p*/,
		IPdraw::IMuxer * /*m*/,
		enum pdraw_muxer_connection_state /*cs*/,
		enum pdraw_muxer_disconnection_reason /*dr*/) override
	{
	}
	void onMuxerMediaReady(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*path*/,
			       const struct iovec * /*iov*/,
			       int /*cnt*/) override
	{
	}
	void onMuxerMediaSaved(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*path*/) override
	{
	}
	void onMuxerUnrecoverableError(IPdraw * /*p*/,
				       IPdraw::IMuxer * /*m*/,
				       int /*s*/) override
	{
		mGotUnrecoverableError = true;
	}
	void muxerCloseResponse(IPdraw * /*p*/,
				IPdraw::IMuxer * /*m*/,
				int status) override
	{
		mCloseStatus = status;
		mGotCloseResponse = true;
	}

	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
	bool mGotUnrecoverableError = false;
};


static void testCxxRawVideoTrackRoundtripsThroughDemuxerSink()
{
	TestPompLoop loop;
	RawVideoTrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* ── Phase 1: mux a tiny synthetic raw8 MP4. ── */

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;
	struct vdef_dim resolution = {kWidth, kHeight};
	size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
	int ret = vdef_calc_raw_frame_size(&vdef_raw8,
					   &resolution,
					   planeStride,
					   nullptr,
					   nullptr,
					   nullptr,
					   planeSize,
					   nullptr);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_raw8;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_api_demuxer_raw_video");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_api_demuxer_raw_video.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	RecordingMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);

	struct pdraw_muxer_media_params muxerMediaParams = {};
	muxerMediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &muxerMediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	/* At least 2 frames: a single-sample track has no delta between
	 * timestamps to derive a duration from, so libmp4 writes it with
	 * duration == 0 -- which then crashes mp4_demux_seek() with a
	 * division by zero (SIGFPE) as soon as playback tries to seek to
	 * the next sample (found the hard way: this test originally pushed
	 * only 1 frame). Not a raw-video-specific issue (mp4_demux_seek()
	 * divides by tk->duration for any track), and out of scope to fix
	 * here (libmp4, not pdraw) -- just avoid the degenerate case, which
	 * doesn't reflect any real recording anyway. */
	static const unsigned int kFrameCount = 3;
	for (unsigned int i = 0; i < kFrameCount; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		frameInfo.plane_stride[0] = planeStride[0];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(planeSize[0], &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0x42, capacity);
		ret = mbuf_raw_video_frame_set_plane(
			frame, 0, mem, 0, planeSize[0]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotMuxedFrame = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.raw_video_frames >= kFrameCount);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMuxedFrame);

	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotCloseResponse; });
	CU_ASSERT_TRUE_FATAL(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);
	muxerOwner.reset();
	sourceOwner.reset();

	/* ── Phase 2: demux it back for real, via an IRawVideoSink -- this is
	 * the part that actually exercises processSample(). ── */

	struct pdraw_demuxer_params demuxParams = {};
	demuxParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	demuxParams.playback_mode = PDRAW_PLAYBACK_MODE_OFFLINE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	ret = session->createDemuxer(
		kOutPath, &demuxParams, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);
	mediaListener.mDemuxer = demuxer;

	/* Wait for both: ready-to-play, and the demuxer's own onMediaAdded()
	 * (fired from DemuxerRawVideoMedia::setupMedia(), which is what
	 * hands out the real pipeline Media::mId needed below -- NOT the
	 * MP4 track id from getMediaList(), see the comment on
	 * RawVideoTrackMediaListener::onMediaAdded() above). */
	bool gotReady = loop.pumpUntil([&]() {
		return demuxListener.mGotReadyToPlay &&
		       mediaListener.mGotDemuxerMediaAdded;
	});
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(demuxListener.mReady);

	/* Sanity check only (matches test_pipeline_muxer_record.cpp's
	 * testCxxMuxerRecordsRawVideoTrack reopen check): confirms the
	 * track is discoverable. The media_id from this list must NOT be
	 * used to create the sink below -- see the comment on
	 * RawVideoTrackMediaListener::onMediaAdded(). */
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(mediaCount >= 1);
	bool foundVideoTrack = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type == PDRAW_MEDIA_TYPE_VIDEO)
			foundVideoTrack = true;
	}
	pdraw_demuxerMediaListFree(mediaList, mediaCount);
	CU_ASSERT_TRUE(foundVideoTrack);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(mediaListener.mDemuxerMediaId,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	std::vector<struct mbuf_raw_video_frame *> outputFrames;
	bool gotOutputFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(outQueue, &f) ==
			       0)
				outputFrames.push_back(f);
			return outputFrames.size() >= kFrameCount;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotOutputFrames);
	CU_ASSERT_EQUAL_FATAL(outputFrames.size(), kFrameCount);

	/* The whole point: a real raw video frame, matching what was muxed,
	 * came back out through DemuxerRawVideoMedia::processSample(). */
	struct vdef_raw_frame outInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(outputFrames[0], &outInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(vdef_raw_format_cmp(&outInfo.format, &vdef_raw8));
	CU_ASSERT_EQUAL(outInfo.info.resolution.width, kWidth);
	CU_ASSERT_EQUAL(outInfo.info.resolution.height, kHeight);

	for (auto *f : outputFrames)
		mbuf_raw_video_frame_unref(f);

	sinkOwner.reset();

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDemuxClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotDemuxClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* ── C++ API — multitrack record demuxing + metadata (1080p30_very_short_
 * thermal.mp4 fixture, ASSET_VIDEO_MULTITRACK_THERMAL) ──────────────────
 * See the fixture's own comment above for what makes it distinct from every
 * other asset in this file: 2 coded video tracks + 1 raw (thermal) video
 * track + 1 audio track, all 3 video tracks carrying real metadata. ────── */

/* Session-wide listener that records every onMediaAdded() event by id/type/
 * video format. Needed (rather than the simpler bool-returning listeners
 * above) because selecting every track of this fixture at once creates
 * several pipeline medias of the same PDRAW_MEDIA_TYPE_VIDEO type in a
 * single call: 2 CodedVideoMedia per coded MP4 track (RecordDemuxer::
 * DemuxerCodedVideoMedia::setupMedia() always creates both the packetized
 * and byte-stream format variants, see the comment on
 * testCxxCodedVideoSinkReceivesRealFrameMetadata above) plus 1 RawVideoMedia
 * for the thermal track -- distinguishable from each other only via
 * info->video.format (CODED vs RAW), not via getMediaList()'s media type
 * (both report PDRAW_MEDIA_TYPE_VIDEO).
 * g_test_session's listener is fixed to nullptr (see the comment on
 * RawVideoTrackMediaListener above), so this builds its own private
 * TestPompLoop/TestSession, same rationale as that class. */
class MultitrackMediaListener : public IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		enum pdraw_media_type type;
		enum vdef_frame_type videoFormat;
	};

	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*elementUserData*/) override
	{
		Added a = {};
		a.id = info->id;
		a.type = info->type;
		if (info->type == PDRAW_MEDIA_TYPE_VIDEO)
			a.videoFormat = info->video.format;
		mAdded.push_back(a);
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	size_t countVideo(enum vdef_frame_type format) const
	{
		size_t n = 0;
		for (const auto &a : mAdded) {
			if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
			    (a.videoFormat == format))
				n++;
		}
		return n;
	}

	size_t countAudio() const
	{
		size_t n = 0;
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_AUDIO)
				n++;
		}
		return n;
	}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Simplest of the 3 tests below: no need to select any non-default media,
 * getMediaList() alone (shared g_test_session/g_test_loop, like every other
 * test in this file) is enough to prove both the multitrack listing and the
 * per-track metadata reading. */
static void testCxxGetMediaListMultitrackSessionMetadata()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(
		session, &listener, ASSET_VIDEO_MULTITRACK_THERMAL);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret = obj->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);

	/* 2 coded video tracks + 1 raw video track + 1 audio track: the first
	 * fixture in this file with more than 2 tracks total. */
	CU_ASSERT_EQUAL(mediaCount, 4u);

	size_t videoCount = 0, audioCount = 0, videoWithMetaCount = 0;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type == PDRAW_MEDIA_TYPE_VIDEO) {
			videoCount++;
			if (mediaList[i].video.session_meta.friendly_name[0] !=
			    '\0')
				videoWithMetaCount++;
		} else if (mediaList[i].type == PDRAW_MEDIA_TYPE_AUDIO) {
			audioCount++;
		}
	}
	CU_ASSERT_EQUAL(videoCount, 3u);
	CU_ASSERT_EQUAL(audioCount, 1u);

	/* All 3 video tracks (both coded ones and the raw one) carry real
	 * session metadata: RecordDemuxer::completeStart() calls
	 * fetchSessionMetadata() for every PDRAW_MEDIA_TYPE_VIDEO entry in
	 * the list the same way regardless of whether the underlying track
	 * is coded or raw (pdraw_demuxer_record.cpp). */
	CU_ASSERT_EQUAL(videoWithMetaCount, 3u);

	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	closeAndDestroyDemuxer(obj, &listener);
}


/* Shared by the 2 tests below: opens the fixture, then explicitly selects
 * EVERY track reported by getMediaList() (as opposed to every other test in
 * this file, which only ever plays with the demuxer's default selection) and
 * waits for all the resulting pipeline medias to be set up. */
static IPdraw::IDemuxer *openMultitrackRecordingAndSelectAllTracks(
	IPdraw *session,
	TestPompLoop *loop,
	RecordingDemuxerListener *demuxListener,
	MultitrackMediaListener *mediaListener)
{
	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, demuxListener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady = loop->pumpUntil(
		[demuxListener]() { return demuxListener->mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = obj->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 4u);
	CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = obj->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* 2 coded MP4 tracks x 2 format variants each (always both created,
	 * see the comment on MultitrackMediaListener above) + 1 raw video
	 * track + 1 audio track = 6 pipeline medias in total. */
	bool gotAllMedias = loop->pumpUntil(
		[mediaListener]() { return mediaListener->mAdded.size() >= 6; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotAllMedias);

	return obj;
}


/* The core "multitrack demuxing" test: proves RecordDemuxer::
 * processSelectedMedias() correctly sets up ALL selected tracks
 * simultaneously (2 coded video + 1 raw video + 1 audio), not just the
 * usual single default video + default audio pair every other test in this
 * file selects. */
static void testCxxSelectAllTracksSetsUpCodedRawAndAudioMedias()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	CU_ASSERT_EQUAL(mediaListener.mAdded.size(), 6u);
	CU_ASSERT_EQUAL(mediaListener.countVideo(VDEF_FRAME_TYPE_CODED), 4u);
	CU_ASSERT_EQUAL(mediaListener.countVideo(VDEF_FRAME_TYPE_RAW), 1u);
	CU_ASSERT_EQUAL(mediaListener.countAudio(), 1u);

	int ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* The core "metadata reading" test for this fixture: attaches an
 * IRawVideoSink directly to the demuxer's raw (thermal) video track --
 * unlike testCxxRawVideoTrackRoundtripsThroughDemuxerSink above, which
 * mux/demuxes a synthetic raw8 track carrying no metadata at all -- to prove
 * DemuxerRawVideoMedia::processSample()'s vmeta_frame_read() path
 * (pdraw_demuxer_record_raw_video_media.cpp) actually attaches real per-frame
 * vmeta read from a genuine MP4 raw video track. */
static void
testCxxRawVideoTrackFromMultitrackRecordingReceivesRealFrameMetadata()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	const MultitrackMediaListener::Added *rawVideo = nullptr;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_RAW)) {
			rawVideo = &a;
			break;
		}
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideo);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	int ret = session->createRawVideoSink(rawVideo->id,
					      &sinkParams,
					      &g_stub_raw_video_sink_listener,
					      &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *queue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	static const size_t kMinFrames = 3;
	std::vector<struct mbuf_raw_video_frame *> frames;
	bool gotFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_raw_video_frame *f = nullptr;
			while (mbuf_raw_video_frame_queue_pop(queue, &f) == 0)
				frames.push_back(f);
			return frames.size() >= kMinFrames;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	bool foundFrameMeta = false;
	for (auto *f : frames) {
		struct vmeta_frame *meta = nullptr;
		int metaRet = mbuf_raw_video_frame_get_metadata(f, &meta);
		if (metaRet == 0 && meta != nullptr) {
			if (meta->type != VMETA_FRAME_TYPE_NONE)
				foundFrameMeta = true;
			vmeta_frame_unref(meta);
		}
		mbuf_raw_video_frame_unref(f);
	}
	CU_ASSERT_TRUE(foundFrameMeta);

	sinkOwner.reset();

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the "failed to get an input buffer" PDRAW_LOGW() branch, present
 * identically in RecordDemuxer::DemuxerCodedVideoMedia/DemuxerRawVideoMedia/
 * DemuxerAudioMedia::processSample() (pdraw_demuxer_record_coded_video_media.
 * cpp:526, pdraw_demuxer_record_raw_video_media.cpp:363,
 * pdraw_demuxer_record_audio_media.cpp:280): previously 0% covered, since no
 * other fixture in this file is ever played back with a downstream sink that
 * doesn't drain its queue. Split into 3 tests below (one per media type),
 * each opening its OWN demuxer instance and attaching exactly ONE real sink
 * -- see the "single sink only" note below for why, discovered the hard way
 * from a real run of an earlier, single-test version of this that attached
 * all 3 sinks at once.
 *
 * Mechanism: Source::createOutputPortMemoryPool() gives each track a
 * MBUF_POOL_NO_GROW pool of a fixed size (DEMUXER_RECORD_*_OUTPUT_BUFFER_
 * COUNT, pdraw_demuxer_record.hpp: 30 for coded/raw video, 60 for audio). A
 * buffer only returns to the pool once nothing downstream still references
 * it. A real sink is attached below (ICodedVideoSink/IRawVideoSink/
 * IAudioSink, default params -> queue_max_count = 0, i.e. "never drop"),
 * using the stub listeners from test_api_common.hpp, and its queue is
 * deliberately never popped during playback, so every produced frame
 * permanently holds its buffer. Once the fixed pool is exhausted,
 * getOutputMemory()/getCodedVideoOutputMemory() starts failing and
 * processSample() takes the "failed to get an input buffer" branch on every
 * subsequent sample for that track, forever (the pool can never regain a
 * buffer since this test never releases one).
 *
 * This fixture (1080p30_short_thermal.MP4, ASSET_VIDEO_MULTITRACK_THERMAL_
 * LONGER, ~7s / ~59 video frames) exists solely for these tests: the sibling
 * ASSET_VIDEO_MULTITRACK_THERMAL only has 15 frames per video track, never
 * enough to exhaust a 30-buffer pool (see the comment on the asset enum
 * above).
 *
 * Importantly, the warning is only logged when mDemuxer->mPlaybackMode !=
 * PDRAW_PLAYBACK_MODE_OFFLINE, so -- unlike testCxxRawVideoTrackRoundtrips
 * ThroughDemuxerSink above -- these tests must NOT set playback_mode to
 * OFFLINE, or the branch would still execute (still 100% line coverage) but
 * would no longer match what a real live/replay playback actually logs.
 *
 * **Single sink only, confirmed by a real failing run**: RecordDemuxer::
 * flush() (triggered by ANY track's pool exhaustion, via DemuxerMedia::
 * onTimer()'s waitFlush handling) loops over *every* selected track
 * (RecordDemuxer::mMedias) and flushes/drains each one's own channels --
 * i.e. one track's exhaustion cascades a Channel::flush() onto every OTHER
 * sinked track too. Channel::flush() itself never touches a sink's queue
 * (it only invokes the sink's onChannelFlush() callback), but the built-in
 * Export{Coded,Raw}Video/AudioSink implementations respond to it by setting
 * an internal "flush pending" flag that makes them silently DISCARD every
 * subsequent incoming frame until the application calls queueFlushed() --
 * which the stub listeners here never do. A first real run of a single test
 * attaching all 3 sinks at once showed exactly this: the audio track (60
 * frames, but this fixture's audio samples are paced almost back-to-back,
 * so its pool exhausted within milliseconds of play()) hit the warning
 * first, its cascading flush put the coded/raw video sinks into permanent
 * "flush pending" mode before either had reached anywhere near 30 frames of
 * their own, and both then discarded every later frame forever -- so their
 * counts never reached 30 and the test timed out. Attaching only ONE real
 * sink per demuxer instance sidesteps this entirely: every OTHER track in
 * the fixture (the always-auto-selected DefaultVideo+DefaultAudio pair, plus
 * whichever one of these 3 tests isn't currently exercising a given track)
 * is either not selected at all, or selected-but-unsinked -- and an unsinked
 * track's single held buffer is unref'd at the very start of its own next
 * processSample() call, so it never accumulates/exhausts its own pool and
 * never triggers a cascading flush of its own. */
static void testCxxDemuxerAudioWarnsAndSkipsSampleOnOutputBufferPoolExhaustion()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_LONGER,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* No selectMedia() call: rely on the demuxer's default auto-selection
	 * (DefaultVideo + DefaultAudio, confirmed by a real run's log) -- 2
	 * coded video format variants + 1 audio media = 3 pipeline medias. */
	bool gotDefaultMedias = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 3; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDefaultMedias);
	CU_ASSERT_EQUAL(mediaListener.countAudio(), 1u);

	unsigned int audioMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if (a.type == PDRAW_MEDIA_TYPE_AUDIO)
			audioMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(audioMediaId != UINT_MAX);

	IPdraw::IAudioSink *audioSink = nullptr;
	ret = session->createAudioSink(
		audioMediaId, &g_stub_audio_sink_listener, &audioSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto audioSinkOwner = std::unique_ptr<IPdraw::IAudioSink>(audioSink);
	struct mbuf_audio_frame_queue *audioQueue = audioSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(audioQueue);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* The queue is never popped below: every produced frame keeps
	 * holding its buffer, so the fixed-size 60-buffer audio pool fills
	 * up and then can never grow again -- this is what proves "failed to
	 * get an input buffer" was hit and kept being hit for the rest of
	 * the fixture's ~7s duration. Once the count has reached the pool
	 * size, it is permanently stable (see the comment above), so
	 * asserting exact equality right after the predicate fires is not
	 * racy. */
	bool gotFullPool = loop.pumpUntil(
		[&]() {
			return mbuf_audio_frame_queue_get_count(audioQueue) >=
			       60;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotFullPool);
	CU_ASSERT_EQUAL(mbuf_audio_frame_queue_get_count(audioQueue), 60);

	/* Reaching 60 above only proves the pool's 60 buffers all got checked
	 * out -- it does NOT by itself prove the "failed to get an input
	 * buffer" branch (pdraw_demuxer_record_audio_media.cpp:271-272)
	 * actually fired: pumpUntil() returns the instant its predicate
	 * becomes true, i.e. right when the 60th frame lands, which can be
	 * before the next sample delivery that would call processSample()
	 * again against the now-exhausted pool (confirmed by a real run
	 * showing 0 hits on that line despite the assertion above passing,
	 * same false assumption already corrected for the coded/raw video
	 * variants below). Keep pumping a bit longer (with an always-false
	 * predicate, so this only relies on the timeout) so at least one more
	 * sample delivery actually runs and hits that branch; the queue count
	 * must stay at exactly 60 throughout, since the pool cannot hand out
	 * any more buffers no matter how many more samples are attempted. */
	bool stayedFull = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedFull);
	CU_ASSERT_EQUAL(mbuf_audio_frame_queue_get_count(audioQueue), 60);

	/* Same fix as the coded video pool-exhaustion test below (see its
	 * comment for the full rationale): close() must be called BEFORE
	 * resetting the sink (destroying it first delivers UNLINK while the
	 * demuxer is still STARTED, too early, so close()'s own flush()
	 * no-ops with -EALREADY on the already-FLUSHING channel), and the
	 * pool-exhaustion-triggered flush (FLUSHING(discard=1) in a real
	 * run's log) is never acknowledged by the stub listener's
	 * onAudioSinkFlush() (a no-op, unlike a real application), so
	 * queueFlushed() must be called explicitly before the sink is
	 * destroyed -- otherwise the sink can never reach STOPPED and
	 * Session::stop() waits forever on it (confirmed by a real run:
	 * close() and/or stop() time out). */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = audioSink->queueFlushed();
	CU_ASSERT_EQUAL(ret, 0);
	audioSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same mechanism/fixture as testCxxDemuxerAudioWarnsAndSkipsSampleOnOutput
 * BufferPoolExhaustion above (see its comment for the full rationale, in
 * particular why only one real sink is attached here), but for
 * DemuxerCodedVideoMedia::processSample() (pdraw_demuxer_record_coded_video_
 * media.cpp:526) and its 30-buffer pool. DefaultVideo is auto-selected by
 * default (no selectMedia() call needed, same as the audio test above);
 * DefaultAudio is auto-selected alongside it but left unsinked, which is
 * harmless (see the "single sink only" note above).
 *
 * Same reasoning as audio applies here too (pumpUntil([count>=30]) returns
 * the INSTANT the 30th frame lands, before the next delivery that would
 * actually try, and fail, to get a 31st buffer from the now-exhausted pool
 * has run), for this ~30 fps video track delivering one frame per onTimer()
 * tick spaced ~33 ms apart. Reaching 30 alone therefore does NOT prove the
 * "failed to get an input buffer" branch fired -- confirmed by a real run
 * showing 0 hits on that line despite this assertion passing. The extra
 * short pumpUntil() below, with an always-false predicate (so it only relies
 * on the timeout), keeps driving the loop for a few more ticks after
 * reaching 30 so that branch actually executes at least once. */
static void
testCxxDemuxerCodedVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_LONGER,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* No selectMedia() call: DefaultVideo (2 coded format variants) +
	 * DefaultAudio (1) are auto-selected -- 3 pipeline medias. */
	bool gotDefaultMedias = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 3; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDefaultMedias);
	CU_ASSERT_EQUAL(mediaListener.countVideo(VDEF_FRAME_TYPE_CODED), 2u);

	/* Only need one of the 2 format variants: whichever one we attach a
	 * sink to becomes the "required" format for the track's shared pool
	 * (see Source::getCodedVideoOutputMemory()), so the sibling variant
	 * -- which gets no sink and therefore no output channel -- is simply
	 * skipped by processSample() rather than separately consuming pool
	 * buffers. */
	unsigned int codedMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_CODED) &&
		    (codedMediaId == UINT_MAX))
			codedMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(codedMediaId != UINT_MAX);

	struct pdraw_video_sink_params videoSinkParams = {};
	IPdraw::ICodedVideoSink *codedSink = nullptr;
	ret = session->createCodedVideoSink(codedMediaId,
					    &videoSinkParams,
					    &g_stub_coded_video_sink_listener,
					    &codedSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto codedSinkOwner =
		std::unique_ptr<IPdraw::ICodedVideoSink>(codedSink);
	struct mbuf_coded_video_frame_queue *codedQueue = codedSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedQueue);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* ~59 video frames are available in this fixture, comfortably more
	 * than the 30-buffer pool -- without the exhaustion branch, this
	 * count would keep climbing well past 30. */
	bool gotFullPool = loop.pumpUntil(
		[&]() {
			return mbuf_coded_video_frame_queue_get_count(
				       codedQueue) >= 30;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotFullPool);
	CU_ASSERT_EQUAL(mbuf_coded_video_frame_queue_get_count(codedQueue), 30);

	/* Reaching 30 above only proves the pool's 30 buffers all got
	 * checked out -- it does NOT by itself prove the "failed to get an
	 * input buffer" branch (pdraw_demuxer_record_coded_video_media.cpp:
	 * 526) actually fired: pumpUntil() returns the instant its predicate
	 * becomes true, i.e. right when the 30th frame lands, which is
	 * *before* the ~33 ms-later timer tick that would call processSample()
	 * a 31st time against the now-exhausted pool. Keep pumping a bit
	 * longer (with an always-false predicate, so this only relies on the
	 * timeout) so at least a few more onTimer() ticks actually run and
	 * hit that branch; the queue count must stay at exactly 30 throughout,
	 * since the pool cannot hand out any more buffers no matter how many
	 * more samples are attempted. */
	bool stayedFull = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedFull);
	CU_ASSERT_EQUAL(mbuf_coded_video_frame_queue_get_count(codedQueue), 30);

	/* Unlike the other tests in this file, close() is called BEFORE
	 * resetting the sink here, not after (this alone was needed to fix a
	 * prior hang on gotClose: tearing the sink down first delivers its
	 * UNLINK while the demuxer is still STARTED, too early to complete
	 * teardown, and close()'s own flush() then no-ops with -EALREADY on
	 * the already-FLUSHING channel).
	 *
	 * That fix alone left a SECOND hang, this time on gotStop: the
	 * pool-exhaustion branch forced above leaves the SINK's own channel
	 * settled in FLUSHING with the flush never acknowledged (the stub
	 * listener's onCodedVideoSinkFlush() is a no-op, unlike a real
	 * application which would call queueFlushed() here). Once in that
	 * state, destroying the sink calls its own flush() again during
	 * teardown, which also no-ops with -EALREADY -- so the sink can
	 * never reach STOPPED, and Session::stop() waits forever on it
	 * (confirmed by a real run: close() succeeds, but stop() times out).
	 * Explicitly acknowledging the flush before resetting the sink fixes
	 * this deterministically: queueFlushed() clears the pending flush, so
	 * teardown's flush() call takes the normal (not -EALREADY) path and
	 * self-completes since the listener is about to be torn down anyway. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = codedSink->queueFlushed();
	CU_ASSERT_EQUAL(ret, 0);
	codedSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Regression test for a real production bug reported from device logs: an
 * exact seekTo() issued while the RecordDemuxer is actively playing (i.e.
 * !mFrameByFrame) never completed if it arrived while DemuxerCodedVideoMedia
 * ::processSample() (pdraw_demuxer_record_coded_video_media.cpp:527) was
 * failing to get an output buffer. RecordDemuxer::DemuxerMedia::onTimer()'s
 * "waitFlush" branch (pdraw_demuxer_record.cpp:1677) used to unconditionally
 * clear mPendingSeekExact and stop the per-media timer (waitMs = 0): with
 * mPendingSeekExact already cleared, processSample() could never again raise
 * *didSeek for this seek (it only ever does so for an exact seek while that
 * flag is still set), so completeSeek() -> onMediaSeekComplete() ->
 * seekResponse() was never called -- and since the timer was also dead, no
 * later onTimer() pass ever ran to give it another chance. The only thing
 * that ever resolved the pending SEEK command was Demuxer's own
 * DEMUXER_PENDING_COMMAND_TIMEOUT_MS (3000 ms, pdraw_demuxer.hpp:40)
 * watchdog, i.e. demuxerSeekResponse(-ETIMEDOUT, ...) -- exactly the
 * "pending operation (SEEK) timed out" log line from the field report.
 *
 * Reuses the exact same deterministic output-pool-exhaustion mechanism as
 * testCxxDemuxerCodedVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion
 * above (see its comment for the full rationale: single real sink, queue
 * never popped, playback_mode left at its default -- NOT OFFLINE, since
 * OFFLINE neutralizes the "waitFlush" branch into a plain retry, per
 * pdraw_demuxer_record.cpp:1649) to force processSample() into "waitFlush",
 * then drives play() (to leave frame-by-frame mode, matching the field
 * report) and an exact seekTo() while the pool is still exhausted, pumps
 * once more to force the seek's own attempt to hit "waitFlush" too (this is
 * what actually engages the bug, as opposed to a seek that happens to
 * succeed on its very first try), and only then releases the held buffers
 * so a correctly-behaving build can retry and really complete the seek. */
static void testCxxRecordDemuxerExactSeekCompletesAfterOutputPoolExhaustion()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_LONGER,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	uint64_t duration = demuxer->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	/* No selectMedia() call: DefaultVideo (2 coded format variants) +
	 * DefaultAudio (1) are auto-selected -- 3 pipeline medias. */
	bool gotDefaultMedias = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 3; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDefaultMedias);
	CU_ASSERT_EQUAL(mediaListener.countVideo(VDEF_FRAME_TYPE_CODED), 2u);

	unsigned int codedMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_CODED) &&
		    (codedMediaId == UINT_MAX))
			codedMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(codedMediaId != UINT_MAX);

	struct pdraw_video_sink_params videoSinkParams = {};
	IPdraw::ICodedVideoSink *codedSink = nullptr;
	ret = session->createCodedVideoSink(codedMediaId,
					    &videoSinkParams,
					    &g_stub_coded_video_sink_listener,
					    &codedSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto codedSinkOwner =
		std::unique_ptr<IPdraw::ICodedVideoSink>(codedSink);
	struct mbuf_coded_video_frame_queue *codedQueue = codedSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedQueue);

	/* play() leaves mFrameByFrame == false, matching the field report:
	 * the timeout was only ever observed for an exact seek issued while
	 * actively playing, never for one issued from a scrubbing-while-
	 * paused sequence (frame-by-frame seeks are resolved by a different,
	 * unaffected path in RecordDemuxer::completeFlush()). */
	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);
	CU_ASSERT_FALSE_FATAL(demuxer->isPaused());

	/* Same two-step wait as the sibling pool-exhaustion test above:
	 * reaching count==30 only proves the pool got fully checked out, not
	 * that processSample() has actually retried against the
	 * now-exhausted pool and hit the "waitFlush" branch -- keep pumping
	 * a little longer (relying only on the timeout) to force at least
	 * one more onTimer() tick through that branch before the seek
	 * below. */
	bool gotFullPool = loop.pumpUntil(
		[&]() {
			return mbuf_coded_video_frame_queue_get_count(
				       codedQueue) >= 30;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotFullPool);
	bool stayedFull = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedFull);
	CU_ASSERT_EQUAL_FATAL(
		mbuf_coded_video_frame_queue_get_count(codedQueue), 30);

	demuxListener.mGotSeekResponse = false;
	ret = demuxer->seekTo(duration / 4, /* exact */ true);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Pump with the pool still exhausted, so the seek's own first
	 * onTimer() attempt is forced through "waitFlush" too -- this is
	 * what actually engages the bug (a seek that happens to succeed on
	 * its very first try never goes anywhere near this branch). */
	bool seekHitWaitFlushWindow =
		loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(seekHitWaitFlushWindow);
	CU_ASSERT_FALSE_FATAL(demuxListener.mGotSeekResponse);

	/* Only now release the pressure: popping frees each frame's buffer
	 * back to the pool (same pattern as ExternalCodedVideoSinkListener::
	 * discardQueue() above in this file). Without this the seek could
	 * never complete at all regardless of the fix, which would just be
	 * a different, unrelated failure mode than the one under test. */
	struct mbuf_coded_video_frame *f = nullptr;
	while (mbuf_coded_video_frame_queue_pop(codedQueue, &f) == 0)
		mbuf_coded_video_frame_unref(f);

	/* The discriminating assertion. Before the fix, the "waitFlush" pass
	 * forced above left mPendingSeekExact cleared and the per-media
	 * timer permanently stopped, so nothing -- not even freeing the pool
	 * -- could ever make this seek complete on its own; only Demuxer's
	 * own 3 s watchdog eventually answers it, with status -ETIMEDOUT.
	 * The 10 s budget here is comfortably more than that 3 s, so a still
	 * -broken build fails on the status assertion below rather than on
	 * a generic pumpUntil timeout. */
	bool gotSeek = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotSeekResponse; },
		10000);
	CU_ASSERT_TRUE_FATAL(gotSeek);
	CU_ASSERT_EQUAL(demuxListener.mSeekStatus, 0);
	CU_ASSERT(demuxListener.mSeekTimestamp <= duration);

	/* Same close()-before-sink-teardown / explicit queueFlushed() dance
	 * as the sibling pool-exhaustion test above -- see its comment for
	 * the full rationale. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = codedSink->queueFlushed();
	CU_ASSERT_EQUAL(ret, 0);
	codedSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same mechanism/fixture as testCxxDemuxerAudioWarnsAndSkipsSampleOnOutput
 * BufferPoolExhaustion above (see its comment for the full rationale), but
 * for DemuxerRawVideoMedia::processSample() (pdraw_demuxer_record_raw_video_
 * media.cpp:363) and its 30-buffer pool. Unlike DefaultVideo/DefaultAudio,
 * ThermalVideo (the raw track) is NOT auto-selected by default, so it needs
 * an explicit selectMedia() -- found by name via getMediaList(), since
 * pdraw_demuxer_media doesn't expose a raw/coded distinction at that level
 * (both report PDRAW_MEDIA_TYPE_VIDEO; only the resulting pipeline media's
 * video.format does, via MultitrackMediaListener below).
 *
 * See the coded video test's comment above for why reaching count==30 alone
 * does not prove the "failed to get an input buffer" branch fired: this
 * ~30 fps track also delivers one frame per ~33 ms onTimer() tick, so
 * pumpUntil() returns right at the 30th frame, before the next tick (which
 * would actually hit the exhausted pool) has run -- hence the extra short
 * pumpUntil() below to force that. */
static void
testCxxDemuxerRawVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_LONGER,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 4u);

	int rawTrackMediaId = -1;
	for (size_t i = 0; i < mediaCount; i++) {
		if (strcmp(mediaList[i].name, "ThermalVideo") == 0)
			rawTrackMediaId = mediaList[i].media_id;
	}
	pdraw_demuxerMediaListFree(mediaList, mediaCount);
	CU_ASSERT_TRUE_FATAL(rawTrackMediaId >= 0);

	ret = demuxer->selectMedia(1U << rawTrackMediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRawMedia = loop.pumpUntil(
		[&]() {
			return mediaListener.countVideo(VDEF_FRAME_TYPE_RAW) >=
			       1;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);

	unsigned int rawMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_RAW))
			rawMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(rawMediaId != UINT_MAX);

	struct pdraw_video_sink_params videoSinkParams = {};
	IPdraw::IRawVideoSink *rawSink = nullptr;
	ret = session->createRawVideoSink(rawMediaId,
					  &videoSinkParams,
					  &g_stub_raw_video_sink_listener,
					  &rawSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto rawSinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(rawSink);
	struct mbuf_raw_video_frame_queue *rawQueue = rawSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawQueue);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	bool gotFullPool = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_get_count(rawQueue) >=
			       30;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotFullPool);
	CU_ASSERT_EQUAL(mbuf_raw_video_frame_queue_get_count(rawQueue), 30);

	/* Same reasoning as the coded video test's twin wait above: reaching
	 * 30 only proves the pool's buffers all got checked out, not that the
	 * "failed to get an input buffer" branch (pdraw_demuxer_record_raw_
	 * video_media.cpp:363) actually ran -- pumpUntil() returns the instant
	 * the 30th frame lands, before the next timer tick's processSample()
	 * call would hit the now-exhausted pool. Keep pumping a bit longer so
	 * that call actually happens; the count must stay at exactly 30. */
	bool stayedFull = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedFull);
	CU_ASSERT_EQUAL(mbuf_raw_video_frame_queue_get_count(rawQueue), 30);

	/* close() is called BEFORE resetting the sink here, unlike the other
	 * tests in this file -- see the identical note in the coded video
	 * test above for the full rationale, including the SECOND hang (on
	 * gotStop, not gotClose) this alone doesn't fix: the pool-exhaustion
	 * branch just forced above leaves the sink's own channel settled in
	 * FLUSHING, never acknowledged (the stub listener's onRawVideoSink
	 * Flush() is a no-op, unlike a real app which would call
	 * queueFlushed() here). Destroying the sink in that state makes its
	 * own teardown flush() call also no-op with -EALREADY, so it can
	 * never reach STOPPED and Session::stop() waits on it forever
	 * (confirmed by a real run: close() succeeds, stop() times out).
	 * Explicitly acknowledging the flush before resetting the sink fixes
	 * this deterministically. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = rawSink->queueFlushed();
	CU_ASSERT_EQUAL(ret, 0);
	rawSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the "mp4_demux_get_track_sample" PDRAW_LOG_ERRNO() branch on the
 * "get a sample" call, present identically in RecordDemuxer::
 * DemuxerCodedVideoMedia/DemuxerRawVideoMedia/DemuxerAudioMedia::
 * processSample() (pdraw_demuxer_record_coded_video_media.cpp:570,
 * pdraw_demuxer_record_raw_video_media.cpp:405, pdraw_demuxer_record_audio_
 * media.cpp:322): distinct from the pool-exhaustion PDRAW_LOGW() branch
 * above, and NOT gated by playback mode. Unlike that branch, no real fixture
 * can hit this one on its own -- the 3 processSample() implementations size
 * their fixed-capacity output buffer generously enough (4 KB for audio,
 * width*height*3/4 for coded video, the exact computed frame size for raw
 * video) that a real encoded/raw sample never exceeds it. So
 * ASSET_VIDEO_MULTITRACK_THERMAL_STSZ_CORRUPT deliberately lies about the
 * LAST sample's size in the stsz box of the relevant track (see the asset
 * enum comment above for how/why only the last sample). Split into 3 tests
 * below (one per media type) for the same "single sink only" reason as the
 * pool-exhaustion tests -- not strictly required here since this branch
 * never sets *waitFlush* (so no RecordDemuxer::flush() cascade risk), but
 * kept for consistency with the established per-media-type pattern above.
 *
 * Playback is at PDRAW_PLAY_SPEED_MAX (unlike the pool-exhaustion tests,
 * which need real-time pacing to observe the pool actually filling up):
 * here we just need the fixture to play through to its corrupted last
 * sample as fast as possible. Since exactly one sample per track is
 * corrupted and it's always the LAST one, the sink's delivered frame count
 * can structurally never exceed (real sample count - 1), so waiting for the
 * count to reach that value and then asserting equality is not racy --
 * EXCEPT for audio, whose 79-frame expected count exceeds its own 60-buffer
 * output pool (DEMUXER_RECORD_AUDIO_MEDIA_OUTPUT_BUFFER_COUNT): if left
 * unpopped like the coded/raw video queues below, it would hit the *other*
 * "failed to get an input buffer" branch (pool exhaustion) at frame 61,
 * long before ever reaching the corrupted 80th sample -- confirmed by 2 real
 * runs, the second of which showed that popping into a vector to unref
 * *later* (the pattern used by the coded/raw video tests below, harmless
 * there since their target counts are well under their pool size) is NOT
 * enough on its own: a popped-but-not-yet-unref'd frame still holds its
 * pool buffer, so the pool exhausted exactly the same way. Each popped
 * audio frame must be mbuf_audio_frame_unref()'d immediately (keeping just
 * a running count) for its buffer to actually return to the pool. Both real
 * runs showed the same symptom: track 3's own onTimer() hit *waitFlush*,
 * called RecordDemuxer::flush(), and then failed a follow-up
 * mp4_demux_seek() ("unable to seek in track") -- all self-contained to
 * track 3's own processing, not caused by track 1 (confirmed by reading
 * onTimer()/selectReferenceTrack(): a non-waitFlush *retry* outcome, which
 * is all track 1's corrupted last sample produces, only ever reschedules
 * that same track, it never touches any other track's state). */
static void testCxxDemuxerAudioLogsErrorAndSkipsSampleOnCorruptedSampleSize()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_STSZ_CORRUPT,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* Explicitly select ONLY the audio track, unlike the pool-exhaustion
	 * audio test above (which relies on default auto-selection, leaving
	 * DefaultVideo selected alongside it). Demuxer::selectMedia() clears
	 * and replaces the whole selection rather than adding to it, so this
	 * also deselects DefaultVideo entirely -- makes audio itself the sole
	 * (and therefore reference) media, keeping this test's corrupted-
	 * sample event fully self-contained to the track under test. Not
	 * strictly required to fix the failure below (see its root cause in
	 * the comment on that wait), but avoids DefaultVideo hitting its own
	 * corrupted last sample as unrelated background noise during the
	 * same run. */
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	int audioTrackMediaId = -1;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type == PDRAW_MEDIA_TYPE_AUDIO)
			audioTrackMediaId = mediaList[i].media_id;
	}
	pdraw_demuxerMediaListFree(mediaList, mediaCount);
	CU_ASSERT_TRUE_FATAL(audioTrackMediaId >= 0);

	ret = demuxer->selectMedia(1U << audioTrackMediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotAudioMedia = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.countAudio() >= 1; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotAudioMedia);

	unsigned int audioMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if (a.type == PDRAW_MEDIA_TYPE_AUDIO)
			audioMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(audioMediaId != UINT_MAX);

	IPdraw::IAudioSink *audioSink = nullptr;
	ret = session->createAudioSink(
		audioMediaId, &g_stub_audio_sink_listener, &audioSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto audioSinkOwner = std::unique_ptr<IPdraw::IAudioSink>(audioSink);
	struct mbuf_audio_frame_queue *audioQueue = audioSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(audioQueue);

	ret = demuxer->play(PDRAW_PLAY_SPEED_MAX);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* 80 real audio samples, the very last one corrupted -> 79 delivered.
	 * Unlike the coded/raw video tests below, this can't just watch
	 * mbuf_audio_frame_queue_get_count() without popping (nor even just
	 * pop into a vector to unref later, as tried in an earlier version of
	 * this test): the audio output buffer pool only has DEMUXER_RECORD_
	 * AUDIO_MEDIA_OUTPUT_BUFFER_COUNT (60) buffers, fewer than the 79
	 * frames expected here. A popped-but-not-yet-unref'd frame still
	 * holds its pool buffer -- popping alone only moves the accumulation
	 * from the sink's queue to wherever the popped frames are kept, it
	 * doesn't release anything. Only mbuf_audio_frame_unref() returns the
	 * buffer to the pool, so each frame must be unref'd right away
	 * (keeping just a running count) for the pool to never fill up. */
	size_t deliveredCount = 0;
	bool gotExpectedCount = loop.pumpUntil(
		[&]() {
			struct mbuf_audio_frame *f = nullptr;
			while (mbuf_audio_frame_queue_pop(audioQueue, &f) ==
			       0) {
				mbuf_audio_frame_unref(f);
				deliveredCount++;
			}
			return deliveredCount >= 79;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotExpectedCount);
	CU_ASSERT_EQUAL(deliveredCount, 79u);
	CU_ASSERT_EQUAL(mbuf_audio_frame_queue_get_count(audioQueue), 0);

	/* Reaching 79 above only proves the 79 valid samples were delivered --
	 * it does NOT by itself prove the corrupted 80th sample was ever
	 * attempted and rejected via the "mp4_demux_get_track_sample" ENOBUFS
	 * branch (pdraw_demuxer_record_audio_media.cpp): pumpUntil() returns
	 * the instant its predicate becomes true, i.e. right when the 79th
	 * (valid) frame lands, which is *before* the next timer tick that
	 * would actually attempt the corrupted 80th sample. Confirmed by
	 * temporary instrumentation on the raw video variant below:
	 * processSample() was never even called for the corrupted last sample,
	 * so this assertion alone was a false positive (it would pass
	 * identically if the last sample were not corrupted at all, e.g. on a
	 * 79-sample track). Keep pumping a bit longer (with an always-false
	 * predicate, so this only relies on the timeout) so the corrupted
	 * sample is actually attempted; the delivered count must stay at
	 * exactly 79 throughout, since no more valid samples exist. */
	bool stayedAt79 = loop.pumpUntil(
		[&]() {
			struct mbuf_audio_frame *f = nullptr;
			while (mbuf_audio_frame_queue_pop(audioQueue, &f) ==
			       0) {
				mbuf_audio_frame_unref(f);
				deliveredCount++;
			}
			return false;
		},
		500);
	CU_ASSERT_FALSE(stayedAt79);
	CU_ASSERT_EQUAL(deliveredCount, 79u);

	/* Same class of fix as the pool-exhaustion tests above (see
	 * testCxxDemuxerAudioWarnsAndSkipsSampleOnOutputBufferPoolExhaustion's
	 * comment for the full rationale): the corrupted last sample being
	 * actually attempted now means the track reaches its natural end,
	 * which triggers a DRAIN downstream event (confirmed by a real run's
	 * log: "channel downstream event DRAIN" -> "element flushing state
	 * change to FLUSHING (discard=0)") -- but the stub listener's
	 * onAudioSinkDrain() is a no-op, unlike a real application which
	 * would call queueDrained() here, so the sink can never reach
	 * STOPPED and Session::stop() waits forever on it (confirmed by a
	 * real run: close() succeeds but stop() times out). Also,
	 * queueDrained() must be called (and close() issued) before
	 * resetting the sink -- same reasoning as the pool-exhaustion fix. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = audioSink->queueDrained();
	CU_ASSERT_EQUAL(ret, 0);
	audioSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same mechanism/fixture as testCxxDemuxerAudioLogsErrorAndSkipsSampleOn
 * CorruptedSampleSize above (see its comment for the full rationale), but
 * for DemuxerCodedVideoMedia::processSample() (pdraw_demuxer_record_coded_
 * video_media.cpp:570). DefaultVideo (track 1) is auto-selected by default;
 * its stsz's last (15th) sample is the one corrupted -> 14 delivered. */
static void
testCxxDemuxerCodedVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_STSZ_CORRUPT,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* No selectMedia() call: DefaultVideo (2 coded format variants) +
	 * DefaultAudio (1) are auto-selected -- 3 pipeline medias. */
	bool gotDefaultMedias = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 3; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDefaultMedias);
	CU_ASSERT_EQUAL(mediaListener.countVideo(VDEF_FRAME_TYPE_CODED), 2u);

	unsigned int codedMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_CODED) &&
		    (codedMediaId == UINT_MAX))
			codedMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(codedMediaId != UINT_MAX);

	struct pdraw_video_sink_params videoSinkParams = {};
	IPdraw::ICodedVideoSink *codedSink = nullptr;
	ret = session->createCodedVideoSink(codedMediaId,
					    &videoSinkParams,
					    &g_stub_coded_video_sink_listener,
					    &codedSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto codedSinkOwner =
		std::unique_ptr<IPdraw::ICodedVideoSink>(codedSink);
	struct mbuf_coded_video_frame_queue *codedQueue = codedSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedQueue);

	ret = demuxer->play(PDRAW_PLAY_SPEED_MAX);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* 15 real video frames, the very last one corrupted -> 14 delivered. */
	bool gotExpectedCount = loop.pumpUntil(
		[&]() {
			return mbuf_coded_video_frame_queue_get_count(
				       codedQueue) >= 14;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotExpectedCount);
	CU_ASSERT_EQUAL(mbuf_coded_video_frame_queue_get_count(codedQueue), 14);

	/* Reaching 14 above only proves the 14 valid samples were delivered --
	 * it does NOT by itself prove the corrupted 15th sample was ever
	 * attempted and rejected via the "mp4_demux_get_track_sample" ENOBUFS
	 * branch: pumpUntil() returns the instant its predicate becomes true,
	 * i.e. right when the 14th (valid) frame lands, before the next timer
	 * tick that would actually attempt the corrupted 15th sample.
	 * Confirmed by temporary instrumentation on the raw video variant
	 * below: processSample() was never even called for the corrupted last
	 * sample, so this assertion alone was a false positive (it would pass
	 * identically on a 14-sample track with no corruption at all). Keep
	 * pumping a bit longer (with an always-false predicate, so this only
	 * relies on the timeout) so the corrupted sample is actually
	 * attempted; the count must stay at exactly 14 throughout. */
	bool stayedAt14 = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedAt14);
	CU_ASSERT_EQUAL(mbuf_coded_video_frame_queue_get_count(codedQueue), 14);

	/* Same class of fix as the audio variant above (see its comment for
	 * the full rationale): the corrupted last sample being actually
	 * attempted now means the track reaches its natural end, triggering
	 * a DRAIN downstream event (confirmed by a real run's log: "channel
	 * downstream event DRAIN" -> "element flushing state change to
	 * FLUSHING (discard=0)") that the stub listener's
	 * onCodedVideoSinkDrain() never acknowledges, so the sink can never
	 * reach STOPPED and Session::stop() waits forever on it (confirmed
	 * by a real run: close() succeeds but stop() times out). Also,
	 * queueDrained() must be called (and close() issued) before
	 * resetting the sink -- same reasoning as the pool-exhaustion fix. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = codedSink->queueDrained();
	CU_ASSERT_EQUAL(ret, 0);
	codedSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same mechanism/fixture as testCxxDemuxerAudioLogsErrorAndSkipsSampleOn
 * CorruptedSampleSize above (see its comment for the full rationale), but
 * for DemuxerRawVideoMedia::processSample() (pdraw_demuxer_record_raw_video_
 * media.cpp:405). ThermalVideo (track 4, raw) is NOT auto-selected by
 * default, so it needs an explicit selectMedia() -- found by name via
 * getMediaList(), same as the pool-exhaustion raw video test above. Its
 * stsz's last (15th) sample is the one corrupted -> 14 delivered. */
static void testCxxDemuxerRawVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_STSZ_CORRUPT,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 4u);

	int rawTrackMediaId = -1;
	for (size_t i = 0; i < mediaCount; i++) {
		if (strcmp(mediaList[i].name, "ThermalVideo") == 0)
			rawTrackMediaId = mediaList[i].media_id;
	}
	pdraw_demuxerMediaListFree(mediaList, mediaCount);
	CU_ASSERT_TRUE_FATAL(rawTrackMediaId >= 0);

	ret = demuxer->selectMedia(1U << rawTrackMediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRawMedia = loop.pumpUntil(
		[&]() {
			return mediaListener.countVideo(VDEF_FRAME_TYPE_RAW) >=
			       1;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);

	unsigned int rawMediaId = UINT_MAX;
	for (const auto &a : mediaListener.mAdded) {
		if ((a.type == PDRAW_MEDIA_TYPE_VIDEO) &&
		    (a.videoFormat == VDEF_FRAME_TYPE_RAW))
			rawMediaId = a.id;
	}
	CU_ASSERT_TRUE_FATAL(rawMediaId != UINT_MAX);

	struct pdraw_video_sink_params videoSinkParams = {};
	IPdraw::IRawVideoSink *rawSink = nullptr;
	ret = session->createRawVideoSink(rawMediaId,
					  &videoSinkParams,
					  &g_stub_raw_video_sink_listener,
					  &rawSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto rawSinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(rawSink);
	struct mbuf_raw_video_frame_queue *rawQueue = rawSink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawQueue);

	ret = demuxer->play(PDRAW_PLAY_SPEED_MAX);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* 15 real raw frames, the very last one corrupted -> 14 delivered. */
	bool gotExpectedCount = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_get_count(rawQueue) >=
			       14;
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotExpectedCount);
	CU_ASSERT_EQUAL(mbuf_raw_video_frame_queue_get_count(rawQueue), 14);

	/* Reaching 14 above only proves the 14 valid samples were delivered --
	 * it does NOT by itself prove the corrupted 15th sample was ever
	 * attempted and rejected via the "mp4_demux_get_track_sample" ENOBUFS
	 * branch (pdraw_demuxer_record_raw_video_media.cpp): pumpUntil()
	 * returns the instant its predicate becomes true, i.e. right when the
	 * 14th (valid) frame lands, before the next timer tick that would
	 * actually attempt the corrupted 15th sample. Confirmed by temporary
	 * instrumentation (mSampleIndex logged in processSample()): without
	 * the extra pump below, processSample() was never even called for
	 * the corrupted last sample (index 14) -- this assertion alone was a
	 * false positive, passing identically regardless of whether the last
	 * sample is corrupted or simply doesn't exist. Keep pumping a bit
	 * longer (with an always-false predicate, so this only relies on the
	 * timeout) so the corrupted sample is actually attempted; the count
	 * must stay at exactly 14 throughout. */
	bool stayedAt14 = loop.pumpUntil([]() { return false; }, 500);
	CU_ASSERT_FALSE(stayedAt14);
	CU_ASSERT_EQUAL(mbuf_raw_video_frame_queue_get_count(rawQueue), 14);

	/* Same class of fix as the audio/coded video variants above (see the
	 * audio one's comment for the full rationale): the corrupted last
	 * sample being actually attempted now means the track reaches its
	 * natural end, triggering a DRAIN downstream event that the stub
	 * listener's onRawVideoSinkDrain() never acknowledges, so the sink
	 * can never reach STOPPED and Session::stop() waits forever on it.
	 * Also, queueDrained() must be called (and close() issued) before
	 * resetting the sink -- same reasoning as the pool-exhaustion fix. */
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	ret = rawSink->queueDrained();
	CU_ASSERT_EQUAL(ret, 0);
	rawSinkOwner.reset();
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the mCurrentFrame/mCurrentMem unref branches of flush()/stop()/
 * teardownMedia() in all 3 RecordDemuxer per-track media subclasses
 * (pdraw_demuxer_record_coded_video_media.cpp,
 * pdraw_demuxer_record_raw_video_media.cpp,
 * pdraw_demuxer_record_audio_media.cpp): processSample() always nulls these
 * members out again before returning on every code path, so a real demux
 * run never leaves them non-null by the time flush()/stop()/teardownMedia()
 * can observe them -- these branches are structurally unreachable through
 * normal playback. This test injects fake frame/mem objects directly
 * (bypassing processSample() entirely) via the private-access trick above,
 * then calls each method and checks the members were unref'd/nulled.
 *
 * The demuxer is deliberately never play()'d/previous()'d/next()'d/
 * seekTo()'d: DemuxerMedia::play() is the only thing that arms the
 * per-track pomp::Timer (mTimer->set(1)), and it is only reached from
 * RecordDemuxer::play() or from processSelectedMedias() when mRunning is
 * already true (pdraw_demuxer_record.cpp) -- neither happens just from
 * createDemuxer()+selectMedia(). So onTimer()/processSample() never fires
 * and can't race with / clobber these manual injections. */
static void
testCxxRecordDemuxerMediaUnrefsCurrentFrameAndMemOnFlushStopTeardown()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	bool testedCoded = false, testedRaw = false, testedAudio = false;

	for (auto &m : rd->mMedias) {
		if (auto *cv = dynamic_cast<
			    RecordDemuxer::DemuxerCodedVideoMedia *>(m.get())) {
			struct vdef_coded_frame fi = {};
			fi.format = vdef_h264_byte_stream;

			struct mbuf_coded_video_frame *frame = nullptr;
			int ret = mbuf_coded_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			cv->mCurrentFrame = frame;
			cv->mCurrentMem = mem;
			cv->flush(true);
			CU_ASSERT_PTR_NULL(cv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(cv->mCurrentMem);

			ret = mbuf_coded_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			cv->mCurrentFrame = frame;
			cv->mCurrentMem = mem;
			cv->stop();
			CU_ASSERT_PTR_NULL(cv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(cv->mCurrentMem);

			ret = mbuf_coded_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			cv->mCurrentFrame = frame;
			cv->mCurrentMem = mem;
			cv->teardownMedia();
			CU_ASSERT_PTR_NULL(cv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(cv->mCurrentMem);

			testedCoded = true;
		} else if (auto *rv = dynamic_cast<
				   RecordDemuxer::DemuxerRawVideoMedia *>(
				   m.get())) {
			struct vdef_raw_frame fi = {};
			fi.format = vdef_i420;

			struct mbuf_raw_video_frame *frame = nullptr;
			int ret = mbuf_raw_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			rv->mCurrentFrame = frame;
			rv->mCurrentMem = mem;
			rv->flush(true);
			CU_ASSERT_PTR_NULL(rv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(rv->mCurrentMem);

			ret = mbuf_raw_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			rv->mCurrentFrame = frame;
			rv->mCurrentMem = mem;
			rv->stop();
			CU_ASSERT_PTR_NULL(rv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(rv->mCurrentMem);

			ret = mbuf_raw_video_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			rv->mCurrentFrame = frame;
			rv->mCurrentMem = mem;
			rv->teardownMedia();
			CU_ASSERT_PTR_NULL(rv->mCurrentFrame);
			CU_ASSERT_PTR_NULL(rv->mCurrentMem);

			testedRaw = true;
		} else if (auto *am = dynamic_cast<
				   RecordDemuxer::DemuxerAudioMedia *>(
				   m.get())) {
			struct adef_frame fi = {};
			fi.format = adef_pcm_16b_44100hz_mono;

			struct mbuf_audio_frame *frame = nullptr;
			int ret = mbuf_audio_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			am->mCurrentFrame = frame;
			am->mCurrentMem = mem;
			am->flush(true);
			CU_ASSERT_PTR_NULL(am->mCurrentFrame);
			CU_ASSERT_PTR_NULL(am->mCurrentMem);

			ret = mbuf_audio_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			am->mCurrentFrame = frame;
			am->mCurrentMem = mem;
			am->stop();
			CU_ASSERT_PTR_NULL(am->mCurrentFrame);
			CU_ASSERT_PTR_NULL(am->mCurrentMem);

			ret = mbuf_audio_frame_new(&fi, &frame);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_mem_generic_new(64, &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			am->mCurrentFrame = frame;
			am->mCurrentMem = mem;
			am->teardownMedia();
			CU_ASSERT_PTR_NULL(am->mCurrentFrame);
			CU_ASSERT_PTR_NULL(am->mCurrentMem);

			testedAudio = true;
		}
	}

	CU_ASSERT_TRUE(testedCoded);
	CU_ASSERT_TRUE(testedRaw);
	CU_ASSERT_TRUE(testedAudio);

	int ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the strtol()-based "com.parrot.regis.first_timestamp" parsing block
 * in DemuxerCodedVideoMedia::setupMedia() and DemuxerRawVideoMedia::
 * setupMedia() (pdraw_demuxer_record_{coded,raw}_video_media.cpp): both
 * previously 0% covered since no other fixture in this file carries that
 * legacy string metadata key. Selects both tracks of the fixture (same
 * "select every reported track" approach as
 * openMultitrackRecordingAndSelectAllTracks() above, inlined here since this
 * fixture has a different track layout: 1 coded + 1 raw, no audio), then
 * reads the resulting mFirstTs directly off RecordDemuxer's internal media
 * objects via the same private-access trick used by
 * testCxxRecordDemuxerMediaUnrefsCurrentFrameAndMemOnFlushStopTeardown --
 * there is no public getter for mFirstTs, and its only downstream effect
 * (capture_timestamp on decoded frames) is gated by a per-frame SEI/vmeta
 * fallback that would make a failed parse hard to tell apart from "no
 * timestamp info at all" (see the comment on
 * testCxxCodedVideoSinkReceivesH265TimeCodeCaptureTimestamp above).
 *
 * DemuxerRawVideoMedia::setupMedia() also lets a real
 * sessionMeta.first_frame_capture_ts (read via fetchSessionMetadata())
 * override the regis-parsed mFirstTs -- this fixture's raw track carries no
 * such key (confirmed via mp4info: only com.apple.quicktime.title,
 * com.parrot.regis.first_timestamp and com.parrot.regis.decimation are
 * listed under "static metadata"), so no override happens and both tracks
 * are expected to carry the exact same parsed value. */
static void testCxxRegisFirstTimestampMetadataParsedFromCodedAndRawTracks()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_REGIS_RAW_CODED, s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	/* 1 coded (AVC) track + 1 raw (i420) track. */
	CU_ASSERT_EQUAL_FATAL(mediaCount, 2u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Coded track -> 2 pipeline medias (AVCC + byte-stream, see the
	 * comment on MultitrackMediaListener above); raw track -> 1. */
	bool gotAllMedias = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 3; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotAllMedias);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	static const uint64_t kExpectedFirstTs = 93086972837ULL;
	bool testedCoded = false, testedRaw = false;
	for (auto &m : rd->mMedias) {
		if (auto *cv = dynamic_cast<
			    RecordDemuxer::DemuxerCodedVideoMedia *>(m.get())) {
			CU_ASSERT_EQUAL(cv->mFirstTs, kExpectedFirstTs);
			testedCoded = true;
		} else if (auto *rv = dynamic_cast<
				   RecordDemuxer::DemuxerRawVideoMedia *>(
				   m.get())) {
			CU_ASSERT_EQUAL(rv->mFirstTs, kExpectedFirstTs);
			testedRaw = true;
		}
	}
	CU_ASSERT_TRUE(testedCoded);
	CU_ASSERT_TRUE(testedRaw);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the "resolutionStr" sscanf() fallback branch in DemuxerRawVideoMedia
 * ::setupMedia() (pdraw_demuxer_record_raw_video_media.cpp:194-206): with a
 * mime_format of "video/raw;format=i420" (no "resolution=" CSV param),
 * vdef_format_info_from_csv() only merges the "format" field and leaves
 * info.resolution untouched (confirmed by reading libvideo-defs/src/
 * vdefs.c: that function never resets fields absent from the CSV string), so
 * the resolution parsed just before from the legacy
 * "com.parrot.regis.resolution" string (214x120) survives into the final
 * RawVideoMedia. See ASSET_VIDEO_REGIS_RAW_LEGACY_META_RESOLUTION_FALLBACK
 * above for the full fixture rationale. */
static void testCxxRegisResolutionFallbackWhenMimeFormatOmitsResolution()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path,
		ASSET_VIDEO_REGIS_RAW_LEGACY_META_RESOLUTION_FALLBACK,
		s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 1u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotMedia = loop.pumpUntil([&mediaListener]() {
		return mediaListener.mAdded.size() >= 1;
	});
	CU_ASSERT_TRUE_FATAL(gotMedia);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerRawVideoMedia *rv = nullptr;
	for (auto &m : rd->mMedias) {
		rv = dynamic_cast<RecordDemuxer::DemuxerRawVideoMedia *>(
			m.get());
		if (rv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(rv);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rv->mRawVideoMedia);
	CU_ASSERT_TRUE(
		vdef_raw_format_cmp(&rv->mRawVideoMedia->format, &vdef_i420));
	CU_ASSERT_EQUAL(rv->mRawVideoMedia->info.resolution.width, 214u);
	CU_ASSERT_EQUAL(rv->mRawVideoMedia->info.resolution.height, 120u);
	CU_ASSERT_EQUAL(rv->mFirstTs, 6006963938ULL);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the pure legacy "raw32" format branch in DemuxerRawVideoMedia::
 * setupMedia() (pdraw_demuxer_record_raw_video_media.cpp:187-189),
 * previously 0% covered: mime_format here is the bare "video/raw" (no
 * trailing ";"), which fails the strncmp(..., VDEF_RAW_MIME_TYPE ";", ...)
 * prefix check and never enters the CSV-parsing branch, so both format
 * ("raw32" -> vdef_raw32) and resolution (214x120) come exclusively from the
 * "com.parrot.regis.*" string metadata. See
 * ASSET_VIDEO_REGIS_RAW_LEGACY_META_RAW32 above for the full fixture
 * rationale. */
static void testCxxRegisLegacyRaw32FormatParsedWithoutMimeCsv()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_REGIS_RAW_LEGACY_META_RAW32,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 1u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotMedia = loop.pumpUntil([&mediaListener]() {
		return mediaListener.mAdded.size() >= 1;
	});
	CU_ASSERT_TRUE_FATAL(gotMedia);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerRawVideoMedia *rv = nullptr;
	for (auto &m : rd->mMedias) {
		rv = dynamic_cast<RecordDemuxer::DemuxerRawVideoMedia *>(
			m.get());
		if (rv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(rv);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rv->mRawVideoMedia);
	CU_ASSERT_TRUE(
		vdef_raw_format_cmp(&rv->mRawVideoMedia->format, &vdef_raw32));
	CU_ASSERT_EQUAL(rv->mRawVideoMedia->info.resolution.width, 214u);
	CU_ASSERT_EQUAL(rv->mRawVideoMedia->info.resolution.height, 120u);
	CU_ASSERT_EQUAL(rv->mFirstTs, 6695984654ULL);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the "unknownFormat" rejection path in DemuxerRawVideoMedia::
 * setupMedia() (pdraw_demuxer_record_raw_video_media.cpp:229-232) through a
 * real fixture/open flow, complementing the isolated hand-crafted
 * mp4_track_info calls in testCxxDemuxerRawVideoMediaSetupMediaErrorPaths:
 * this track's mime_format is the bare "video/raw" (skips the CSV path) and
 * its "com.parrot.regis.format" is "yvu420", unrecognized by either legacy
 * branch, so setupMedia() returns -ENOSYS. RecordDemuxer::
 * processSelectedMedias() (pdraw_demuxer_record.cpp:1149-1156) then never
 * pushes this track's wrapper into RecordDemuxer::mMedias, and no
 * onMediaAdded() ever fires for it -- but per RecordDemuxer::completeStart()
 * (pdraw_demuxer_record.cpp:372-384), processSelectedMedias()'s return value
 * is discarded, so the demuxer as a whole still opens and reports
 * ready-to-play normally, and the track still appears in getMediaList()
 * (built earlier, independently of setup success, from isMediaTrack()). */
static void testCxxRegisUnsupportedLegacyFormatTrackNeverAddedButDemuxerOpens()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(
		path,
		ASSET_VIDEO_REGIS_RAW_LEGACY_META_UNSUPPORTED_FORMAT,
		s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_EQUAL(demuxListener.mOpenStatus, 0);
	CU_ASSERT_TRUE(demuxListener.mReady);

	/* The track is still reported: isMediaTrack() only checks for the
	 * presence of the regis keys, not whether their value parses. */
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 1u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Nothing should ever get added for this track: there is no success
	 * event to wait for, so pump a bounded amount of time and then check
	 * that onMediaAdded() never fired. The predicate always returns
	 * false, so this is expected to time out -- that's the point. */
	bool gotMedia = loop.pumpUntil([]() { return false; }, 2000);
	CU_ASSERT_FALSE(gotMedia);
	CU_ASSERT_TRUE(mediaListener.mAdded.empty());

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerRawVideoMedia *rv = nullptr;
	for (auto &m : rd->mMedias) {
		rv = dynamic_cast<RecordDemuxer::DemuxerRawVideoMedia *>(
			m.get());
		if (rv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NULL(rv);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the "invalid raw video media resolution" rejection path in
 * DemuxerRawVideoMedia::setupMedia() (pdraw_demuxer_record_raw_video_media.
 * cpp:233-236), distinct from the "unknownFormat" rejection path covered by
 * testCxxRegisUnsupportedLegacyFormatTrackNeverAddedButDemuxerOpens above:
 * here "com.parrot.regis.format" is the valid "raw32" but
 * "com.parrot.regis.resolution" is the garbage string "bad_resolution",
 * so sscanf() fails and info.resolution.{width,height} are forced to 0
 * (:199-202), which trips the *next* check instead of the format one. Also
 * exercises the strtol() failure branch for "com.parrot.regis.first_timestamp"
 * (:169-173) via the garbage string "bad_first_ts", though that branch has no
 * assertable side effect (see ASSET_VIDEO_REGIS_RAW_LEGACY_META_BAD_VALUES
 * above for why). See that enumerator for the full fixture rationale. */
static void testCxxRegisBadLegacyMetaValuesTrackNeverAddedButDemuxerOpens()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_REGIS_RAW_LEGACY_META_BAD_VALUES,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_EQUAL(demuxListener.mOpenStatus, 0);
	CU_ASSERT_TRUE(demuxListener.mReady);

	/* The track is still reported: isMediaTrack() only checks for the
	 * presence of the regis keys, not whether their value parses. */
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 1u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Nothing should ever get added for this track: there is no success
	 * event to wait for, so pump a bounded amount of time and then check
	 * that onMediaAdded() never fired. The predicate always returns
	 * false, so this is expected to time out -- that's the point. */
	bool gotMedia = loop.pumpUntil([]() { return false; }, 2000);
	CU_ASSERT_FALSE(gotMedia);
	CU_ASSERT_TRUE(mediaListener.mAdded.empty());

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerRawVideoMedia *rv = nullptr;
	for (auto &m : rd->mMedias) {
		rv = dynamic_cast<RecordDemuxer::DemuxerRawVideoMedia *>(
			m.get());
		if (rv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NULL(rv);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the strtol() failure path for "com.parrot.regis.first_timestamp" in
 * DemuxerCodedVideoMedia::setupMedia()
 * (pdraw_demuxer_record_coded_video_media.cpp:216-227) -- a separate copy of
 * the same parsing logic covered for raw video by
 * testCxxRegisBadLegacyMetaValuesTrackNeverAddedButDemuxerOpens above,
 * previously untested for the coded path. Unlike that raw-video test, the
 * bad "com.parrot.regis.resolution"/"format" strings on this fixture are
 * never read by the coded setupMedia() (resolution comes from the AVC
 * decoder config instead), and the SPS/PPS are valid, so setupMedia() still
 * succeeds: the strtol() failure only skips setting mFirstTs, leaving it at
 * its UINT64_MAX default (pdraw_demuxer_record.hpp:342), and the track is
 * added normally. See ASSET_VIDEO_REGIS_CODED_LEGACY_META_BAD_VALUES above
 * for the full fixture rationale. */
static void testCxxRegisCodedBadFirstTimestampKeepsDefaultFirstTs()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_REGIS_CODED_LEGACY_META_BAD_VALUES,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 1u);

	uint32_t allMedias = 0;
	for (size_t i = 0; i < mediaCount; i++)
		allMedias |= (1U << mediaList[i].media_id);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = demuxer->selectMedia(allMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotMedia = loop.pumpUntil([&mediaListener]() {
		return mediaListener.mAdded.size() >= 1;
	});
	CU_ASSERT_TRUE_FATAL(gotMedia);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerCodedVideoMedia *cv = nullptr;
	for (auto &m : rd->mMedias) {
		cv = dynamic_cast<RecordDemuxer::DemuxerCodedVideoMedia *>(
			m.get());
		if (cv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(cv);
	CU_ASSERT_EQUAL(cv->mFirstTs, UINT64_MAX);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Covers the nullptr-guard branches of the 6 H.264/H.265 SEI callbacks in
 * DemuxerCodedVideoMedia (pdraw_demuxer_record_coded_video_media.cpp),
 * which real playback never hits since the h264_reader/h265_reader always
 * pass valid self/ctx/sei and mCurrentFrame is always set while a sample is
 * being processed. Same technique as
 * testCxxVideoDecoderFrameOutputCbCoverage (test_pipeline_decoder_video.cpp):
 * call the static callbacks directly with crafted/null arguments instead of
 * trying to force these edge cases through the full demux pipeline.
 * struct h264_ctx / h265_ctx are opaque outside of libh264/libh265, so a
 * fake non-null pointer is used where a non-null ctx is required -- it is
 * never dereferenced because mCurrentFrame is left nullptr, which makes the
 * guard right before any ctx/sei dereference return first. */
static void testCxxDemuxerCodedVideoSeiCallbacksNullGuardsCoverage()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL_LONGER,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret =
		session->createDemuxer(path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* No selectMedia() call: DefaultVideo (2 coded format variants) +
	 * DefaultAudio (1) are auto-selected -- 3 pipeline medias. */
	bool gotDefaultMedias = loop.pumpUntil([&mediaListener]() {
		return mediaListener.mAdded.size() >= 3;
	});
	CU_ASSERT_TRUE_FATAL(gotDefaultMedias);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerCodedVideoMedia *cv = nullptr;
	for (auto &m : rd->mMedias) {
		cv = dynamic_cast<RecordDemuxer::DemuxerCodedVideoMedia *>(
			m.get());
		if (cv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(cv);
	/* play() was never called: no sample was ever processed, so
	 * mCurrentFrame is still at its default value. */
	CU_ASSERT_PTR_NULL(cv->mCurrentFrame);

	uint8_t buf[4] = {0, 0, 0, 1};
	struct h264_sei_user_data_unregistered h264UserData = {};
	struct h264_sei_pic_timing h264PicTiming = {};
	struct h265_sei_user_data_unregistered h265UserData = {};
	struct h265_sei_time_code h265TimeCode = {};
	struct h265_sei_mastering_display_colour_volume h265Mdcv = {};
	struct h265_sei_content_light_level h265Cll = {};
	auto *fakeH264Ctx = reinterpret_cast<struct h264_ctx *>(0x1);
	auto *fakeH265Ctx = reinterpret_cast<struct h265_ctx *>(0x1);

	/* h264UserDataSeiCb: self / buf / len / sei / mCurrentFrame guards. */
	RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), &h264UserData, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
		nullptr, nullptr, sizeof(buf), &h264UserData, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
		nullptr, buf, 0, &h264UserData, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), nullptr, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), &h264UserData, cv);

	/* h264PicTimingSeiCb: self / ctx / sei / mCurrentFrame guards. */
	RecordDemuxer::DemuxerCodedVideoMedia::h264PicTimingSeiCb(
		fakeH264Ctx, buf, sizeof(buf), &h264PicTiming, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h264PicTimingSeiCb(
		nullptr, buf, sizeof(buf), &h264PicTiming, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h264PicTimingSeiCb(
		fakeH264Ctx, buf, sizeof(buf), nullptr, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h264PicTimingSeiCb(
		fakeH264Ctx, buf, sizeof(buf), &h264PicTiming, cv);

	/* h265UserDataSeiCb: self / buf / len / sei / mCurrentFrame guards. */
	RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
		nullptr, buf, sizeof(buf), &h265UserData, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
		nullptr, nullptr, sizeof(buf), &h265UserData, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
		nullptr, buf, 0, &h265UserData, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
		nullptr, buf, sizeof(buf), nullptr, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
		nullptr, buf, sizeof(buf), &h265UserData, cv);

	/* h265TimeCodeSeiCb: self / ctx / sei / mCurrentFrame guards. */
	RecordDemuxer::DemuxerCodedVideoMedia::h265TimeCodeSeiCb(
		fakeH265Ctx, buf, sizeof(buf), &h265TimeCode, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h265TimeCodeSeiCb(
		nullptr, buf, sizeof(buf), &h265TimeCode, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265TimeCodeSeiCb(
		fakeH265Ctx, buf, sizeof(buf), nullptr, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265TimeCodeSeiCb(
		fakeH265Ctx, buf, sizeof(buf), &h265TimeCode, cv);

	/* h265MdcvSeiCb: self / ctx / sei guards (no mCurrentFrame check --
	 * it writes to rd->mMedias' CodedVideoMedia::info instead). */
	RecordDemuxer::DemuxerCodedVideoMedia::h265MdcvSeiCb(
		fakeH265Ctx, buf, sizeof(buf), &h265Mdcv, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h265MdcvSeiCb(
		nullptr, buf, sizeof(buf), &h265Mdcv, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265MdcvSeiCb(
		fakeH265Ctx, buf, sizeof(buf), nullptr, cv);

	/* h265CllSeiCb: self / ctx / sei guards (same reasoning). */
	RecordDemuxer::DemuxerCodedVideoMedia::h265CllSeiCb(
		fakeH265Ctx, buf, sizeof(buf), &h265Cll, nullptr);
	RecordDemuxer::DemuxerCodedVideoMedia::h265CllSeiCb(
		nullptr, buf, sizeof(buf), &h265Cll, cv);
	RecordDemuxer::DemuxerCodedVideoMedia::h265CllSeiCb(
		fakeH265Ctx, buf, sizeof(buf), nullptr, cv);

	/* None of the calls above should have reached past their guards. */
	CU_ASSERT_PTR_NULL(cv->mCurrentFrame);
	CU_ASSERT_EQUAL(cv->mCurrentFrameCaptureTs, 0u);
	CU_PASS("no crash");

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* ── C API shim tests: select_media and end_of_range ─────────────────────── */

/* Covers PdrawDemuxerListener::demuxerSelectMedia (→ select_media C callback).
 * testCApiMethodsValid() above already exercises
 * play_resp/pause_resp/seek_resp; select_media is absent from it because it
 * fires before ready_to_play during the open phase. Returning 0 from the
 * callback means "use default medias", same effect as -ENOSYS but proves the
 * shim forwarding line actually ran. */
static void testCDemuxerListenerSelectMedia()
{
	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct Ud {
		int selectMediaCallCount = 0;
		bool gotReadyToPlay = false;
		bool gotCloseResp = false;
	} ud;

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	struct pdraw_demuxer_cbs cbs = {};
	cbs.select_media = [](struct pdraw *,
			      struct pdraw_demuxer *,
			      const struct pdraw_demuxer_media *,
			      size_t,
			      uint32_t,
			      void *userdata) -> int {
		static_cast<Ud *>(userdata)->selectMediaCallCount++;
		return 0; /* 0 → use default medias (same effect as -ENOSYS) */
	};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_demuxer *,
			       int /*ready*/,
			       void *userdata) {
		static_cast<Ud *>(userdata)->gotReadyToPlay = true;
	};
	cbs.close_resp = [](struct pdraw *,
			    struct pdraw_demuxer *,
			    int /*status*/,
			    void *userdata) {
		static_cast<Ud *>(userdata)->gotCloseResp = true;
	};

	struct pdraw_demuxer *obj = nullptr;
	int ret = pdraw_demuxer_new_from_url(
		g_test_pdraw_c, path, &params, &cbs, &ud, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady =
		g_test_loop->pumpUntil([&ud]() { return ud.gotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	/* select_media fires during the open phase, before ready_to_play */
	CU_ASSERT_TRUE(ud.selectMediaCallCount >= 1);

	ret = pdraw_demuxer_close(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose =
		g_test_loop->pumpUntil([&ud]() { return ud.gotCloseResp; });
	CU_ASSERT_TRUE(gotClose);
	ret = pdraw_demuxer_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


/* Covers PdrawDemuxerListener::onDemuxerEndOfRange (→ end_of_range C callback).
 * Seeks 2 seconds before end-of-file then plays; RecordDemuxer fires
 * end_of_range and auto-pauses when it runs out of samples to deliver.
 * No decoding is needed (DECODE_NONE): the demuxer advances through samples
 * at maximum speed since there are no downstream channels to throttle it. */
static void testCDemuxerListenerEndOfRange()
{
	char path[512];
	PDRAW_GET_ASSET_PATH(
		path, ASSET_VIDEO_H264, s_assets_tests_pdraw_demux);

	struct Ud {
		bool gotReadyToPlay = false;
		bool gotSeekResp = false;
		bool gotPlayResp = false;
		bool gotEndOfRange = false;
		uint64_t endOfRangeTimestamp = 0;
		bool gotCloseResp = false;
	} ud;

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	struct pdraw_demuxer_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_demuxer *,
			       int /*ready*/,
			       void *userdata) {
		static_cast<Ud *>(userdata)->gotReadyToPlay = true;
	};
	cbs.seek_resp = [](struct pdraw *,
			   struct pdraw_demuxer *,
			   int /*status*/,
			   uint64_t /*ts*/,
			   float /*speed*/,
			   void *userdata) {
		static_cast<Ud *>(userdata)->gotSeekResp = true;
	};
	cbs.play_resp = [](struct pdraw *,
			   struct pdraw_demuxer *,
			   int /*status*/,
			   uint64_t /*ts*/,
			   float /*speed*/,
			   void *userdata) {
		static_cast<Ud *>(userdata)->gotPlayResp = true;
	};
	cbs.end_of_range = [](struct pdraw *,
			      struct pdraw_demuxer *,
			      uint64_t timestamp,
			      void *userdata) {
		auto *u = static_cast<Ud *>(userdata);
		u->endOfRangeTimestamp = timestamp;
		u->gotEndOfRange = true;
	};
	cbs.close_resp = [](struct pdraw *,
			    struct pdraw_demuxer *,
			    int /*status*/,
			    void *userdata) {
		static_cast<Ud *>(userdata)->gotCloseResp = true;
	};

	struct pdraw_demuxer *obj = nullptr;
	int ret = pdraw_demuxer_new_from_url(
		g_test_pdraw_c, path, &params, &cbs, &ud, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady =
		g_test_loop->pumpUntil([&ud]() { return ud.gotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);

	uint64_t duration = pdraw_demuxer_get_duration(g_test_pdraw_c, obj);
	CU_ASSERT_FATAL(duration > 2000000ULL);

	/* Seek 2 s before the end (snaps to nearest IDR; exact=0) */
	ret = pdraw_demuxer_seek_to(
		g_test_pdraw_c, obj, duration - 2000000ULL, 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSeek =
		g_test_loop->pumpUntil([&ud]() { return ud.gotSeekResp; });
	CU_ASSERT_TRUE_FATAL(gotSeek);

	/* Play at max speed: RecordDemuxer sets near-zero inter-frame wait
	 * (~33µs), draining the remaining ~2s of file in a few milliseconds.
	 * This avoids the catch-up seek mechanism that, at real-time speed,
	 * can overshoot the end of file and prevent end_of_range from firing.
	 */
	ret = pdraw_demuxer_play_with_speed(
		g_test_pdraw_c, obj, PDRAW_PLAY_SPEED_MAX);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay =
		g_test_loop->pumpUntil([&ud]() { return ud.gotPlayResp; });
	CU_ASSERT_TRUE_FATAL(gotPlay);

	/* end_of_range → onDemuxerEndOfRange shim. At max speed, the remaining
	 * ~2s of samples drain in milliseconds, so a 5s timeout is generous. */
	bool gotEor = g_test_loop->pumpUntil(
		[&ud]() { return ud.gotEndOfRange; }, 5000);
	CU_ASSERT_TRUE(gotEor);
	CU_ASSERT_TRUE(ud.endOfRangeTimestamp > 0);

	/* RecordDemuxer auto-pauses after end_of_range */
	CU_ASSERT_EQUAL(pdraw_demuxer_is_paused(g_test_pdraw_c, obj), 1);

	ret = pdraw_demuxer_close(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose =
		g_test_loop->pumpUntil([&ud]() { return ud.gotCloseResp; });
	CU_ASSERT_TRUE(gotClose);
	ret = pdraw_demuxer_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


/* Verify that DemuxerWrapper::close() sets mElementStopped synchronously
 * so that every subsequent public method returns -EPROTO (or the documented
 * sentinel value for non-int returns). */
static void testCxxRecordDemuxerWrapperGuardsAfterElementCleared()
{
	IPdraw *session = g_test_session->get();
	RecordingDemuxerListener listener;
	IPdraw::IDemuxer *obj = openDemuxerAndWaitReady(session, &listener);
	auto objOwner = std::unique_ptr<IPdraw::IDemuxer>(obj);

	/* close() sets mElementStopped = true synchronously, then starts an
	 * async stop.  Every subsequent call hits the isElementStopped() guard
	 * before touching the inner Demuxer object. */
	CU_ASSERT_EQUAL(obj->close(), 0);

	/* int-returning guards → -EPROTO */
	CU_ASSERT_EQUAL(obj->close(), -EPROTO);
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	CU_ASSERT_EQUAL(
		obj->getMediaList(&mediaList, &mediaCount, &selectedMedias),
		-EPROTO);
	CU_ASSERT_EQUAL(obj->selectMedia(0), -EPROTO);
	CU_ASSERT_EQUAL(obj->play(), -EPROTO);
	CU_ASSERT_EQUAL(obj->previousFrame(), -EPROTO);
	CU_ASSERT_EQUAL(obj->nextFrame(), -EPROTO);
	CU_ASSERT_EQUAL(obj->seek(0), -EPROTO);
	CU_ASSERT_EQUAL(obj->seekTo(0), -EPROTO);
	struct pdraw_chapter *chapterList = nullptr;
	size_t chapterCount = 0;
	CU_ASSERT_EQUAL(obj->getChapterList(&chapterList, &chapterCount),
			-EPROTO);

	/* Non-int guards → documented sentinel values */
	CU_ASSERT_EQUAL(obj->getSingleStreamLocalStreamPort(), (uint16_t)0);
	CU_ASSERT_EQUAL(obj->getSingleStreamLocalControlPort(), (uint16_t)0);
	CU_ASSERT_FALSE(obj->isReadyToPlay());
	CU_ASSERT_FALSE(obj->isPaused());
	CU_ASSERT_EQUAL(obj->getDuration(), (uint64_t)0);
	CU_ASSERT_EQUAL(obj->getCurrentTime(), (uint64_t)0);

	/* Wait for async close to complete before destroying. */
	bool gotClose = g_test_loop->pumpUntil(
		[&listener]() { return listener.mGotCloseResponse; }, 10000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener.mCloseStatus, 0);
}


static void testCxxDemuxerAudioMediaSetupMediaErrorPaths()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerAudioMedia *am = nullptr;
	for (auto &m : rd->mMedias) {
		am = dynamic_cast<RecordDemuxer::DemuxerAudioMedia *>(m.get());
		if (am != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(am);

	/* 1. Invalid audio_codec != MP4_AUDIO_CODEC_AAC_LC (lines 148-153) */
	struct mp4_track_info badAudioTkinfo = {};
	badAudioTkinfo.audio_codec = MP4_AUDIO_CODEC_UNKNOWN;
	int ret = am->setupMedia(&badAudioTkinfo);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* 2. Invalid track ID -> mp4_demux_get_track_audio_specific_config
	 * failure (lines 142-145) */
	uint32_t origTrackId = am->mTrackId;
	am->mTrackId = 99999;
	struct mp4_track_info validAudioTkinfo = {};
	validAudioTkinfo.audio_codec = MP4_AUDIO_CODEC_AAC_LC;
	ret = am->setupMedia(&validAudioTkinfo);
	CU_ASSERT_TRUE(ret < 0);
	am->mTrackId = origTrackId;

	/* 3. Corrupted ASC -> aac_parse_asc failure (lines 199-204) */
	uint8_t *ascBuf = nullptr;
	unsigned int ascSize = 0;
	ret = mp4_demux_get_track_audio_specific_config(
		rd->mDemux, am->mTrackId, &ascBuf, &ascSize);
	if (ret == 0 && ascBuf != nullptr && ascSize >= 2) {
		uint8_t b0 = ascBuf[0], b1 = ascBuf[1];

		/* 3a. Invalid ASC bytes -> aac_parse_asc fails (lines 199-204)
		 */
		ascBuf[0] = 0xFF;
		ascBuf[1] = 0xFF;
		ret = am->setupMedia(&validAudioTkinfo);
		CU_ASSERT_TRUE(ret < 0);

		/* 3b. AAC Main AOT (ObjectType 1) -> aac_parse_asc succeeds,
		 * but aac_asc_to_adef_format fails (lines 206-212) */
		ascBuf[0] = 0x0A;
		ascBuf[1] = 0x10;
		ret = am->setupMedia(&validAudioTkinfo);
		CU_ASSERT_TRUE(ret < 0);

		ascBuf[0] = b0;
		ascBuf[1] = b1;
	}

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


static void testCxxDemuxerRawVideoMediaSetupMediaErrorPaths()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerRawVideoMedia *rv = nullptr;
	for (auto &m : rd->mMedias) {
		rv = dynamic_cast<RecordDemuxer::DemuxerRawVideoMedia *>(
			m.get());
		if (rv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(rv);

	/* 1. Invalid track ID -> mp4_demux_get_track_metadata_strings failure
	 * (lines 151-156) */
	uint32_t origTrackId = rv->mTrackId;
	rv->mTrackId = 99999;
	struct mp4_track_info tkinfo = {};
	int ret = rv->setupMedia(&tkinfo);
	CU_ASSERT_TRUE(ret < 0);
	rv->mTrackId = origTrackId;

	/* 2. Unknown raw format (unknownFormat == true, lines 229-232) */
	struct mp4_track_info badFormatTkinfo = {};
	badFormatTkinfo.mime_format = "invalid_mime";
	ret = rv->setupMedia(&badFormatTkinfo);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* 3. Invalid resolution (width/height == 0, lines 233-236) */
	struct mp4_track_info badResTkinfo = {};
	badResTkinfo.mime_format = "video/raw;format=I420;width=0;height=0";
	ret = rv->setupMedia(&badResTkinfo);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* 4. vdef_raw_format_from_csv failure (line 217) */
	struct mp4_track_info badFormatCsvTkinfo = {};
	badFormatCsvTkinfo.mime_format =
		"video/raw;format=UNKNOWN_FORMAT;width=640;height=480";
	ret = rv->setupMedia(&badFormatCsvTkinfo);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* 5. vdef_format_info_from_csv failure (line 222) */
	struct mp4_track_info badInfoCsvTkinfo = {};
	badInfoCsvTkinfo.mime_format = "video/raw;format=I420;width=invalid";
	ret = rv->setupMedia(&badInfoCsvTkinfo);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


static void testCxxDemuxerCodedVideoMediaSetupMediaErrorPaths()
{
	TestPompLoop loop;
	MultitrackMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	RecordingDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = openMultitrackRecordingAndSelectAllTracks(
		session, &loop, &demuxListener, &mediaListener);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	DemuxerWrapper *wrapper = static_cast<DemuxerWrapper *>(demuxer);
	RecordDemuxer *rd = static_cast<RecordDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rd);

	RecordDemuxer::DemuxerCodedVideoMedia *cv = nullptr;
	for (auto &m : rd->mMedias) {
		cv = dynamic_cast<RecordDemuxer::DemuxerCodedVideoMedia *>(
			m.get());
		if (cv != nullptr)
			break;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(cv);

	auto clearReaders = [](RecordDemuxer::DemuxerCodedVideoMedia *m) {
		if (m->mH264Reader != nullptr) {
			h264_reader_destroy(m->mH264Reader);
			m->mH264Reader = nullptr;
		}
		if (m->mH265Reader != nullptr) {
			h265_reader_destroy(m->mH265Reader);
			m->mH265Reader = nullptr;
		}
	};

	/* 1. Invalid track ID -> mp4_demux_get_track_metadata_strings failure
	 * (lines 210-213) */
	uint32_t origTrackId = cv->mTrackId;
	cv->mTrackId = 99999;
	struct mp4_track_info tkinfo = {};
	int ret = cv->setupMedia(&tkinfo);
	CU_ASSERT_TRUE(ret < 0);
	cv->mTrackId = origTrackId;

	/* 2. Video track already defined (!mMedias.empty(), lines 288-293 +
	 * goto error h264/h265 reader destroy) */
	clearReaders(cv);
	struct mp4_track_info validCodedTkinfo = {};
	validCodedTkinfo.video_codec = MP4_VIDEO_CODEC_AVC;
	ret = cv->setupMedia(&validCodedTkinfo);
	CU_ASSERT_EQUAL(ret, -EEXIST);

	/* 3. Invalid video_codec in tkinfo (default switch, lines 420-424 +
	 * goto error) */
	clearReaders(cv);
	auto savedMedias = std::move(cv->mMedias);
	cv->mMedias.clear();
	struct mp4_track_info badCodedTkinfo = {};
	badCodedTkinfo.video_codec = static_cast<enum mp4_video_codec>(999);
	ret = cv->setupMedia(&badCodedTkinfo);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	cv->teardownMedia();
	cv->mMedias = std::move(savedMedias);

	/* 4. Corrupted NALU -> h264_reader_parse_nalu / h265_reader_parse_nalu
	 * failure (lines 358-370 & 398-416) */
	struct mp4_video_decoder_config vdc = {};
	ret = mp4_demux_get_track_video_decoder_config(
		rd->mDemux, cv->mTrackId, &vdc);
	if (ret == 0 && vdc.codec == MP4_VIDEO_CODEC_AVC) {
		/* 4a. Corrupted H.264 SPS NALU */
		if (vdc.avc.sps != nullptr && vdc.avc.sps_size >= 2) {
			uint8_t *spsBuf = const_cast<uint8_t *>(vdc.avc.sps);
			uint8_t b0 = spsBuf[0], b1 = spsBuf[1];
			spsBuf[0] = 0x00;
			spsBuf[1] = 0x00;

			clearReaders(cv);
			auto savedMedias2 = std::move(cv->mMedias);
			cv->mMedias.clear();
			struct mp4_track_info avcTkinfo = {};
			avcTkinfo.video_codec = MP4_VIDEO_CODEC_AVC;
			ret = cv->setupMedia(&avcTkinfo);
			CU_ASSERT_TRUE(ret < 0);
			cv->teardownMedia();
			cv->mMedias = std::move(savedMedias2);

			spsBuf[0] = b0;
			spsBuf[1] = b1;
		}

		/* 4b. Corrupted H.264 PPS NALU (lines 365-370) */
		if (vdc.avc.pps != nullptr && vdc.avc.pps_size >= 2) {
			uint8_t *ppsBuf = const_cast<uint8_t *>(vdc.avc.pps);
			uint8_t b0 = ppsBuf[0], b1 = ppsBuf[1];
			ppsBuf[0] = 0x00;
			ppsBuf[1] = 0x00;

			clearReaders(cv);
			auto savedMediasPps = std::move(cv->mMedias);
			cv->mMedias.clear();
			struct mp4_track_info avcTkinfo = {};
			avcTkinfo.video_codec = MP4_VIDEO_CODEC_AVC;
			ret = cv->setupMedia(&avcTkinfo);
			CU_ASSERT_TRUE(ret < 0);
			cv->teardownMedia();
			cv->mMedias = std::move(savedMediasPps);

			ppsBuf[0] = b0;
			ppsBuf[1] = b1;
		}
	}

	/* 5. Corrupted H.265 VPS, SPS, PPS NALUs -> h265_reader_parse_nalu
	 * failure */
	char h265Path[512];
	PDRAW_GET_ASSET_PATH(
		h265Path, ASSET_VIDEO_H265_HDR_TC, s_assets_tests_pdraw_demux);
	struct pdraw_demuxer_params h265Params = {};
	h265Params.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	IPdraw::IDemuxer *h265Demuxer = nullptr;
	RecordingDemuxerListener h265DemuxListener;
	ret = session->createDemuxer(
		h265Path, &h265Params, &h265DemuxListener, &h265Demuxer);
	if (ret == 0 && h265Demuxer != nullptr) {
		auto h265Owner = std::unique_ptr<IPdraw::IDemuxer>(h265Demuxer);
		bool ready = loop.pumpUntil([&h265DemuxListener]() {
			return h265DemuxListener.mGotReadyToPlay;
		});
		if (ready) {
			DemuxerWrapper *h265Wrapper =
				static_cast<DemuxerWrapper *>(h265Demuxer);
			RecordDemuxer *h265Rd = static_cast<RecordDemuxer *>(
				h265Wrapper->getDemuxer());
			if (h265Rd != nullptr) {
				for (auto &m : h265Rd->mMedias) {
					auto *cvHevc = dynamic_cast<
						RecordDemuxer::
							DemuxerCodedVideoMedia
								*>(m.get());
					if (cvHevc == nullptr)
						continue;
					struct mp4_video_decoder_config
						vdcHevc = {};
					ret = mp4_demux_get_track_video_decoder_config(
						h265Rd->mDemux,
						cvHevc->mTrackId,
						&vdcHevc);
					if (ret == 0 &&
					    vdcHevc.codec ==
						    MP4_VIDEO_CODEC_HEVC) {
						/* 5a. Corrupted H.265 VPS NALU
						 * (lines 398-403) */
						if (vdcHevc.hevc.vps !=
							    nullptr &&
						    vdcHevc.hevc.vps_size >=
							    2) {
							uint8_t *vpsBuf = const_cast<
								uint8_t *>(
								vdcHevc.hevc
									.vps);
							uint8_t b0 = vpsBuf[0],
								b1 = vpsBuf[1];
							vpsBuf[0] = 0x00;
							vpsBuf[1] = 0x00;

							clearReaders(cvHevc);
							auto savedMediasH265 = std::
								move(cvHevc->mMedias);
							cvHevc->mMedias.clear();
							struct mp4_track_info
								hevcTkinfo = {};
							hevcTkinfo.video_codec =
								MP4_VIDEO_CODEC_HEVC;
							ret = cvHevc->setupMedia(
								&hevcTkinfo);
							CU_ASSERT_TRUE(ret < 0);
							cvHevc->teardownMedia();
							cvHevc->mMedias = std::move(
								savedMediasH265);

							vpsBuf[0] = b0;
							vpsBuf[1] = b1;
						}

						/* 5b. Corrupted H.265 SPS NALU
						 * (lines 405-410) */
						if (vdcHevc.hevc.sps !=
							    nullptr &&
						    vdcHevc.hevc.sps_size >=
							    2) {
							uint8_t *spsBuf = const_cast<
								uint8_t *>(
								vdcHevc.hevc
									.sps);
							uint8_t sb0 = spsBuf[0],
								sb1 = spsBuf[1];
							spsBuf[0] = 0x00;
							spsBuf[1] = 0x00;

							clearReaders(cvHevc);
							auto savedMediasSps = std::
								move(cvHevc->mMedias);
							cvHevc->mMedias.clear();
							struct mp4_track_info
								hevcTkinfo = {};
							hevcTkinfo.video_codec =
								MP4_VIDEO_CODEC_HEVC;
							ret = cvHevc->setupMedia(
								&hevcTkinfo);
							CU_ASSERT_TRUE(ret < 0);
							cvHevc->teardownMedia();
							cvHevc->mMedias = std::move(
								savedMediasSps);

							spsBuf[0] = sb0;
							spsBuf[1] = sb1;
						}

						/* 5c. Corrupted H.265 PPS NALU
						 * (lines 411-416) */
						if (vdcHevc.hevc.pps !=
							    nullptr &&
						    vdcHevc.hevc.pps_size >=
							    2) {
							uint8_t *ppsBuf = const_cast<
								uint8_t *>(
								vdcHevc.hevc
									.pps);
							uint8_t pb0 = ppsBuf[0],
								pb1 = ppsBuf[1];
							ppsBuf[0] = 0x00;
							ppsBuf[1] = 0x00;

							clearReaders(cvHevc);
							auto savedMediasPps = std::
								move(cvHevc->mMedias);
							cvHevc->mMedias.clear();
							struct mp4_track_info
								hevcTkinfo = {};
							hevcTkinfo.video_codec =
								MP4_VIDEO_CODEC_HEVC;
							ret = cvHevc->setupMedia(
								&hevcTkinfo);
							CU_ASSERT_TRUE(ret < 0);
							cvHevc->teardownMedia();
							cvHevc->mMedias = std::move(
								savedMediasPps);

							ppsBuf[0] = pb0;
							ppsBuf[1] = pb1;
						}
					}
				}
			}
		}
		h265Demuxer->close();
		bool gotH265Close = loop.pumpUntil([&h265DemuxListener]() {
			return h265DemuxListener.mGotCloseResponse;
		});
		CU_ASSERT_TRUE(gotH265Close);
	}

	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	demuxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Regression test for heap-use-after-free in RecordDemuxer teardown when
 * the session is destroyed while an audio-bearing demuxer is still active.
 *
 * Root cause: DemuxerMedia::teardownMedia() tried removeOutputPort() on the
 * audio media, got -EBUSY (AudioDecoder's input channel still attached), then
 * freed the AudioMedia object anyway via mMedias.clear().  Source::~Source()
 * later dereferenced the dangling pointer in mOutputPorts via
 * removeOutputPorts() → onOutputMediaRemoved() → media->fillMediaInfo().
 *
 * Fix: call teardownOutputChannels() before removeOutputPort() in
 * teardownMedia() so the port can always be removed cleanly. */
static void testCxxRecordDemuxerAudioChannelTornDownOnSessionDestroy()
{
	RecordingDemuxerListener demuxListener;
	TestPompLoop loop;
	TestSession session(&loop);

	char path[PATH_MAX];
	PDRAW_GET_ASSET_PATH(path,
			     ASSET_VIDEO_MULTITRACK_THERMAL,
			     s_assets_tests_pdraw_demux);

	struct pdraw_demuxer_params params = {};
	/* DECODE_ALL wires an AudioDecoder onto the audio output port, which is
	 * the channel that caused the EBUSY / dangling-pointer crash: with
	 * DECODE_NONE no audio decoder is created and the bug does not trigger.
	 */
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL;

	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session.get()->createDemuxer(
		path, &params, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(demuxListener.mReady);

	/* Explicitly reset the wrapper (triggers an async stop() on the
	 * RecordDemuxer), then let session and loop go out of scope without
	 * pumping the loop: the demuxer element is still STOPPING and the
	 * AudioDecoder's input channel is still attached.  Without the fix
	 * this crashes with a heap-use-after-free; with the fix it exits
	 * cleanly because teardownOutputChannels() removes the channel before
	 * mMedias.clear() frees the AudioMedia. */
	demuxerOwner.reset();
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_api_demuxer[] = {
	{FN("testCApiNewFromUrl"), testCApiNewFromUrl},
	{FN("testCApiClose"), testCApiClose},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiSingleStreamLifecycle"), testCApiSingleStreamLifecycle},
	{FN("testCApiPlay"), testCApiPlay},
	{FN("testCApiPause"), testCApiPause},
	{FN("testCApiSeek"), testCApiSeek},
	{FN("testCApiIsReadyToPlay"), testCApiIsReadyToPlay},
	{FN("testCApiIsPaused"), testCApiIsPaused},
	{FN("testCApiGetDuration"), testCApiGetDuration},
	{FN("testCApiGetCurrentTime"), testCApiGetCurrentTime},
	{FN("testCApiGetSingleStreamLocalPorts"),
	 testCApiGetSingleStreamLocalPorts},
	{FN("testCApiMethodsValid"), testCApiMethodsValid},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxOpenLocalFileReachesReadyToPlay"),
	 testCxxOpenLocalFileReachesReadyToPlay},
	{FN("testCxxGetMediaListAfterOpen"), testCxxGetMediaListAfterOpen},
	{FN("testCxxOpenAudioFileReachesReadyToPlay"),
	 testCxxOpenAudioFileReachesReadyToPlay},
	{FN("testCxxGetSingleStreamPortsAreZeroForLocalFile"),
	 testCxxGetSingleStreamPortsAreZeroForLocalFile},
	{FN("testCxxSelectMediaAfterReady"), testCxxSelectMediaAfterReady},
	{FN("testCxxSelectMediaCancelled"), testCxxSelectMediaCancelled},
	{FN("testCxxOpenFileWithNoMediaTracksNeverBecomesReady"),
	 testCxxOpenFileWithNoMediaTracksNeverBecomesReady},
	{FN("testCxxSelectMediaHardErrorNeverBecomesReady"),
	 testCxxSelectMediaHardErrorNeverBecomesReady},
	{FN("testCxxRecordDemuxerOpenZeroByteFileFailsSynchronously"),
	 testCxxRecordDemuxerOpenZeroByteFileFailsSynchronously},
	{FN("testCxxRecordDemuxerSelectMediaBeforeOpenResponseGuard"),
	 testCxxRecordDemuxerSelectMediaBeforeOpenResponseGuard},
	{FN("testCxxPlayPauseLifecycle"), testCxxPlayPauseLifecycle},
	{FN("testCxxSeekToMidpoint"), testCxxSeekToMidpoint},
	{FN("testCxxSeekRelative"), testCxxSeekRelative},
	{FN("testCxxPreviousNextFrame"), testCxxPreviousNextFrame},
	{FN("testCxxPlayBackwardWithNegativeSpeed"),
	 testCxxPlayBackwardWithNegativeSpeed},
	{FN("testCxxRecordDemuxerPlayGuards"), testCxxRecordDemuxerPlayGuards},
	{FN("testCxxRecordDemuxerPlayPendingCommandGuard"),
	 testCxxRecordDemuxerPlayPendingCommandGuard},
	{FN("testCxxRecordDemuxerPreviousNextGuards"),
	 testCxxRecordDemuxerPreviousNextGuards},
	{FN("testCxxRecordDemuxerSeekGuards"), testCxxRecordDemuxerSeekGuards},
	{FN("testCxxRecordDemuxerSeekToGuards"),
	 testCxxRecordDemuxerSeekToGuards},
	{FN("testCxxRecordDemuxerIsPausedNotStartedGuard"),
	 testCxxRecordDemuxerIsPausedNotStartedGuard},
	{FN("testCxxRecordDemuxerGetChapterListNotStartedGuard"),
	 testCxxRecordDemuxerGetChapterListNotStartedGuard},
	{FN("testCxxRecordDemuxerCatchUpSeekForwardFastSpeed"),
	 testCxxRecordDemuxerCatchUpSeekForwardFastSpeed},
	{FN("testCxxRecordDemuxerCatchUpSeekForwardSlowSpeed"),
	 testCxxRecordDemuxerCatchUpSeekForwardSlowSpeed},
	{FN("testCxxRecordDemuxerCatchUpSeekBackwardFastSpeed"),
	 testCxxRecordDemuxerCatchUpSeekBackwardFastSpeed},
	{FN("testCxxRecordDemuxerCatchUpSeekBackwardSlowSpeed"),
	 testCxxRecordDemuxerCatchUpSeekBackwardSlowSpeed},
	{FN("testCxxGetMediaListSessionMetadataFromRealRecording"),
	 testCxxGetMediaListSessionMetadataFromRealRecording},
	{FN("testCxxCodedVideoSinkReceivesRealFrameMetadata"),
	 testCxxCodedVideoSinkReceivesRealFrameMetadata},
	{FN("testCxxCodedVideoSinkReceivesH265TimeCodeCaptureTimestamp"),
	 testCxxCodedVideoSinkReceivesH265TimeCodeCaptureTimestamp},
	{FN("testCxxCodedVideoSinkExposesH265MdcvAndCllInfo"),
	 testCxxCodedVideoSinkExposesH265MdcvAndCllInfo},
	{FN("testCxxRawVideoTrackRoundtripsThroughDemuxerSink"),
	 testCxxRawVideoTrackRoundtripsThroughDemuxerSink},
	{FN("testCxxGetMediaListMultitrackSessionMetadata"),
	 testCxxGetMediaListMultitrackSessionMetadata},
	{FN("testCxxSelectAllTracksSetsUpCodedRawAndAudioMedias"),
	 testCxxSelectAllTracksSetsUpCodedRawAndAudioMedias},
	{FN("testCxxRawVideoTrackFromMultitrackRecordingReceivesRealFrameMetadata"),
	 testCxxRawVideoTrackFromMultitrackRecordingReceivesRealFrameMetadata},
	{FN("testCxxDemuxerAudioWarnsAndSkipsSampleOnOutputBufferPoolExhaustion"),
	 testCxxDemuxerAudioWarnsAndSkipsSampleOnOutputBufferPoolExhaustion},
	{FN("testCxxDemuxerCodedVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion"),
	 testCxxDemuxerCodedVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion},
	{FN("testCxxRecordDemuxerExactSeekCompletesAfterOutputPoolExhaustion"),
	 testCxxRecordDemuxerExactSeekCompletesAfterOutputPoolExhaustion},
	{FN("testCxxDemuxerRawVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion"),
	 testCxxDemuxerRawVideoWarnsAndSkipsSampleOnOutputBufferPoolExhaustion},
	{FN("testCxxDemuxerAudioLogsErrorAndSkipsSampleOnCorruptedSampleSize"),
	 testCxxDemuxerAudioLogsErrorAndSkipsSampleOnCorruptedSampleSize},
	{FN("testCxxDemuxerCodedVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize"),
	 testCxxDemuxerCodedVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize},
	{FN("testCxxDemuxerRawVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize"),
	 testCxxDemuxerRawVideoLogsErrorAndSkipsSampleOnCorruptedSampleSize},
	{FN("testCxxRecordDemuxerMediaUnrefsCurrentFrameAndMemOnFlushStopTeardown"),
	 testCxxRecordDemuxerMediaUnrefsCurrentFrameAndMemOnFlushStopTeardown},
	{FN("testCxxRegisFirstTimestampMetadataParsedFromCodedAndRawTracks"),
	 testCxxRegisFirstTimestampMetadataParsedFromCodedAndRawTracks},
	{FN("testCxxRegisResolutionFallbackWhenMimeFormatOmitsResolution"),
	 testCxxRegisResolutionFallbackWhenMimeFormatOmitsResolution},
	{FN("testCxxRegisLegacyRaw32FormatParsedWithoutMimeCsv"),
	 testCxxRegisLegacyRaw32FormatParsedWithoutMimeCsv},
	{FN("testCxxRegisUnsupportedLegacyFormatTrackNeverAddedButDemuxerOpens"),
	 testCxxRegisUnsupportedLegacyFormatTrackNeverAddedButDemuxerOpens},
	{FN("testCxxRegisBadLegacyMetaValuesTrackNeverAddedButDemuxerOpens"),
	 testCxxRegisBadLegacyMetaValuesTrackNeverAddedButDemuxerOpens},
	{FN("testCxxRegisCodedBadFirstTimestampKeepsDefaultFirstTs"),
	 testCxxRegisCodedBadFirstTimestampKeepsDefaultFirstTs},
	{FN("testCxxDemuxerCodedVideoSeiCallbacksNullGuardsCoverage"),
	 testCxxDemuxerCodedVideoSeiCallbacksNullGuardsCoverage},
	{FN("testCDemuxerListenerSelectMedia"),
	 testCDemuxerListenerSelectMedia},
	{FN("testCDemuxerListenerEndOfRange"), testCDemuxerListenerEndOfRange},
	{FN("testCxxRecordDemuxerWrapperGuardsAfterElementCleared"),
	 testCxxRecordDemuxerWrapperGuardsAfterElementCleared},
	{FN("testCxxDemuxerAudioMediaSetupMediaErrorPaths"),
	 testCxxDemuxerAudioMediaSetupMediaErrorPaths},
	{FN("testCxxDemuxerRawVideoMediaSetupMediaErrorPaths"),
	 testCxxDemuxerRawVideoMediaSetupMediaErrorPaths},
	{FN("testCxxDemuxerCodedVideoMediaSetupMediaErrorPaths"),
	 testCxxDemuxerCodedVideoMediaSetupMediaErrorPaths},
	{FN("testCxxRecordDemuxerAudioChannelTornDownOnSessionDestroy"),
	 testCxxRecordDemuxerAudioChannelTornDownOnSessionDestroy},
	CU_TEST_INFO_NULL,
};
