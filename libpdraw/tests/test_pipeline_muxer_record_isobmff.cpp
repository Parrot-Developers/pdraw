/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — ISOBMFF (.mp4) muxer recording, real demuxed coded
 * video/audio tracks and synthetic raw video tracks (Tier B, self-contained
 * fixture)
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

/* Complements test_api_muxer.cpp (which only ever feeds the muxer an unknown
 * mediaId) and test_api_demuxer.cpp (which never records what it demuxes):
 * here the demuxer's own coded video output is fed directly into a record
 * muxer, end to end, and the resulting MP4 file is checked for real content.
 *
 * autodecoding_mode = DECODE_NONE on purpose: a muxer records *coded*
 * (compressed) media, so there is no need to spin up a decoder at all --
 * unlike test_pipeline_decoder_video.cpp, which specifically needs a *decoded*
 * raw media for the scaler/encoder.
 *
 * Like test_pipeline_decoder_video.cpp, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr, see
 * test_fixtures.hpp): observing onMediaAdded() for the demuxer's coded
 * output requires a real session-wide listener, so the test below builds
 * its own private TestPompLoop + TestSession. No suite init/cleanup is
 * registered for this file (see test_main.c: NULL, NULL).
 *
 * Split out of the original test_pipeline_muxer_record.cpp (which grew past
 * 5000 lines) alongside its companion test_pipeline_muxer_record_photo.cpp,
 * which covers the photo (PNG/JFIF/DNG) record muxer instead. */

#define ULOG_TAG pdraw_test_pipeline_muxer_record_isobmff
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

/* Needed for pdraw_demuxerMediaListFree(), used to free getMediaList()
 * output (not part of the public pdraw.h, only of the internal utils). */
#include "pdraw_utils.hpp"


#include <audio-defs/adefs.h>
#include <libmp4.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-metadata/vmeta_frame.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── MP4 fixtures (same NAS assets as test_api_demuxer.cpp) ──────────────── */

enum { ASSET_VIDEO_H264 = 0, ASSET_VIDEO_H265 = 1 };

static constexpr struct {
	const char *relative_path;
} s_assets_pipeline_muxer[] = {
	{"Tests/anafi/4k/video_recording/champs_240p30_h264.mp4"},
	{"Tests/anafi/4k/video_recording/champs_240p30_h265.mp4"},
};


/* Session-wide listener recording the coded video media exposed by the
 * demuxer itself (DECODE_NONE: no decoder is auto-created, so this is the
 * demuxer's own output, available as soon as media selection completes --
 * no need to wait for play() or any actual frame). Only value fields are
 * copied out: the pointers in pdraw_media_info (name, path...) are only
 * valid for the duration of the callback. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class CodedMediaTrackingListener : public IPdraw::Listener {
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
			  void * /*u*/) override
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

	/* First coded (compressed) video media added, or nullptr if none. */
	const Added *findCodedVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_CODED)
				return &a;
		}
		return nullptr;
	}

	/* First audio media added, or nullptr if none. Audio media has no
	 * raw/coded distinction at the pdraw_media_info level (unlike
	 * video): the demuxer's own output is always compressed AAC here. */
	const Added *findAudioMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_AUDIO)
				return &a;
		}
		return nullptr;
	}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Minimal demuxer listener: same rationale/duplication as
 * RecordingDemuxerListener in test_api_demuxer.cpp and
 * PipelineDemuxerListener in test_pipeline_decoder_video.cpp. */
class PipelineDemuxerListener : public IPdraw::IDemuxer::Listener {
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
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
		mPlayStatus = status;
		mGotPlayResponse = true;
	}

	void demuxerPauseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int /*s*/,
				  uint64_t /*ts*/) override
	{
	}

	void demuxerSeekResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int /*s*/,
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
	}

	bool mGotOpenResponse = false;
	int mOpenStatus = 0;
	bool mGotReadyToPlay = false;
	bool mReady = false;
	bool mGotPlayResponse = false;
	int mPlayStatus = 0;
	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
	bool mGotUnrecoverableError = false;
};


/* Muxer listener: tracks the async close response (like demuxer/close
 * elsewhere). unlike every other callback in this suite which runs on the
 * pomp_loop thread, onMuxerUnrecoverableError/muxerCloseResponse are the
 * only ones actually exercised here so std::atomic is somewhat overkill,
 * but kept for consistency/safety with the muxer's internal writer
 * thread (see CLAUDE.md: "RecordMuxer runs on its own internal thread"). */
class PipelineMuxerListener : public IPdraw::IMuxer::Listener {
public:
	void onMuxerConnectionStateChanged(
		IPdraw * /*p*/,
		IPdraw::IMuxer * /*m*/,
		enum pdraw_muxer_connection_state /*cs*/,
		enum pdraw_muxer_disconnection_reason /*dr*/) override
	{
	}

	/* Never fired for a video/ISOBMFF record muxer: per pdraw.hpp this
	 * is only called for photo (JFIF/DNG/PNG) record muxers, once per
	 * output photo file. A video muxer writes continuously into a
	 * single file instead, with no per-chunk "ready" notification --
	 * confirmed empirically (this stayed at 0 through an entire
	 * recording). Kept as a no-op override since it is still part of
	 * the interface. */
	void onMuxerMediaReady(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*mp*/,
			       const struct iovec * /*iov*/,
			       int /*iovcnt*/) override
	{
	}

	void onMuxerMediaSaved(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*mp*/) override
	{
		mGotMediaSaved.store(true);
	}

	void onMuxerUnrecoverableError(IPdraw * /*p*/,
				       IPdraw::IMuxer * /*m*/,
				       int /*status*/) override
	{
		mGotUnrecoverableError.store(true);
	}

	void muxerCloseResponse(IPdraw * /*p*/,
				IPdraw::IMuxer * /*m*/,
				int status) override
	{
		mCloseStatus.store(status);
		mGotCloseResponse.store(true);
	}

	std::atomic<bool> mGotMediaSaved{false};
	std::atomic<bool> mGotUnrecoverableError{false};
	std::atomic<bool> mGotCloseResponse{false};
	std::atomic<int> mCloseStatus{0};
};


static void runMuxerRecordsRealCodedVideo(size_t assetIndex,
					  const char *outSuffix)
{
	char kOutPath[512];
	snprintf(kOutPath,
		 sizeof(kOutPath),
		 "/tmp/pdraw_test_pipeline_muxer%s.mp4",
		 outSuffix);

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path, assetIndex, s_assets_pipeline_muxer);

	struct pdraw_demuxer_params demuxParams = {};
	/* DECODE_NONE: record the coded stream as-is, no decoder needed. */
	demuxParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	demuxParams.playback_mode = PDRAW_PLAYBACK_MODE_OFFLINE;

	PipelineDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(
		path, &demuxParams, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(demuxListener.mReady);

	/* The demuxer's own coded video media is known as soon as it has
	 * selected its default medias, i.e. no later than ready-to-play --
	 * unlike test_pipeline_decoder_video.cpp's *decoded* raw media, which
	 * only appears once the (here nonexistent) decoder outputs a first
	 * frame.
	 *
	 * Copied by value rather than kept as a pointer into mAdded: play()
	 * below pumps the loop, and if that triggers any further
	 * onMediaAdded() (e.g. another track in the same file), the
	 * resulting vector push_back() could reallocate and invalidate a
	 * pointer into it. */
	const CodedMediaTrackingListener::Added *codedVideoPtr =
		mediaListener.findCodedVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedVideoPtr);
	CodedMediaTrackingListener::Added codedVideo = *codedVideoPtr;
	CU_ASSERT_NOT_EQUAL_FATAL(codedVideo.id, 0u);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	/* remove() a possible leftover from a previous run: createMuxer()
	 * itself would also overwrite it, but a clean slate makes the
	 * post-close size check unambiguous. */
	(void)remove(kOutPath);

	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* is_default=true: matches what a real player relies on to
	 * auto-select tracks when re-opening the file (see the equivalent
	 * comment in testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips). */
	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(codedVideo.id, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Offline playback pushes frames as fast as possible; poll getStats()
	 * -- synchronous, and unlike onMuxerMediaReady() (which is only ever
	 * fired for photo/JFIF/DNG/PNG record muxers, never for a video one
	 * that writes continuously into a single file) it reports real muxed
	 * frame counts regardless of muxer type -- for at least one recorded
	 * frame. */
	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= 1);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();

	/* The whole point: an actual, non-trivial MP4 file was written.
	 * (onMuxerMediaSaved's exact timing relative to close() isn't
	 * documented precisely enough to assert on deterministically, so the
	 * file itself -- the ground truth -- is checked instead.) */
	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);
	(void)remove(kOutPath);

	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDemuxClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotDemuxClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner.reset();

	/* IPdraw::~IPdraw() requires stop() + stopResponse() beforehand (see
	 * pdraw.hpp) -- see the identical reasoning in
	 * test_pipeline_decoder_video.cpp's stopSessionAndWait(). */
	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxMuxerRecordsRealCodedVideoH264()
{
	runMuxerRecordsRealCodedVideo(ASSET_VIDEO_H264, ".h264");
}

/* Every other muxer test in this file (this test's H.264 sibling above,
 * testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips below, and the raw
 * video muxer tests further down) uses only H.264 assets -- IsobmffMuxer
 * CodedVideoMedia's H.265/HEVC-specific handling (pdraw_muxer_record_
 * isobmff_media.cpp) was therefore never exercised from the muxer side,
 * mirroring the same gap test_pipeline_decoder_video.cpp's
 * testCxxH265VideoDecodesToRawFrames just closed on the demuxer side. */
static void testCxxMuxerRecordsRealCodedVideoH265()
{
	runMuxerRecordsRealCodedVideo(ASSET_VIDEO_H265, ".h265");
}


/* IsobmffRecordMuxer::addChapter()/internalAddChapter()/addChapters() are
 * never exercised anywhere else in this file -- confirmed via gcov: several
 * IsobmffMuxerMedia getters/setters (setChapters(), getTrackId(),
 * hasChapters(), getMediaTime(), isDefault() in pdraw_muxer_record_isobmff_
 * media.hpp) showed 0 executions, all of them only ever called from
 * addChapters() (pdraw_muxer_record_isobmff.cpp) while wiring up the
 * chapters track's reference to the default coded video track. Verified via
 * IDemuxer::getChapterList() on the reopened file, the strongest available
 * proof (mirrors the "reopen and check via the public API" rationale used
 * throughout this file), rather than just checking addChapter() returns 0. */
static void testCxxMuxerRecordsRealCodedVideoWithChapters()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_chapters.mp4";

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path, ASSET_VIDEO_H264, s_assets_pipeline_muxer);

	struct pdraw_demuxer_params demuxParams = {};
	demuxParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	demuxParams.playback_mode = PDRAW_PLAYBACK_MODE_OFFLINE;

	PipelineDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(
		path, &demuxParams, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(demuxListener.mReady);
	uint64_t duration = demuxer->getDuration();
	CU_ASSERT_FATAL(duration > 0);

	const CodedMediaTrackingListener::Added *codedVideoPtr =
		mediaListener.findCodedVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(codedVideoPtr);
	CodedMediaTrackingListener::Added codedVideo = *codedVideoPtr;
	CU_ASSERT_NOT_EQUAL_FATAL(codedVideo.id, 0u);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	(void)remove(kOutPath);

	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* is_default=true: addChapters() (pdraw_muxer_record_isobmff.cpp)
	 * searches for a default CODED_VIDEO track to attach the chapters
	 * track to, and returns -EAGAIN (silently pending the chapter) if
	 * none is found yet. */
	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(codedVideo.id, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= 1);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	/* First chapter at timestamp 0 -- avoids internalAddChapter()'s
	 * "missing first chapter" auto-insertion, keeping the chapter count
	 * below exactly what this test itself added. */
	ret = muxer->addChapter(0, "Start");
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = muxer->addChapter(duration / 2, "Halfway");
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	auto muxerOwner2 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner2.reset();

	auto demuxerOwner2 = std::unique_ptr<IPdraw::IDemuxer>(demuxer);
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDemuxClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotDemuxClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner2.reset();

	/* The whole point: reopen the file and read the chapters back via the
	 * public API -- the strongest available proof that a real chapters
	 * track was created and wired up, not just that addChapter() itself
	 * returned 0 (which only means the request was successfully queued
	 * onto the writer thread, see IsobmffRecordMuxer::addChapter()). */
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenDemuxerOwner =
		std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);

	struct pdraw_chapter *chapterList = nullptr;
	size_t chapterCount = 0;
	ret = reopenDemuxer->getChapterList(&chapterList, &chapterCount);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(chapterCount, 2u);
	CU_ASSERT_PTR_NOT_NULL_FATAL(chapterList);
	CU_ASSERT_EQUAL(chapterList[0].ts_us, 0ULL);
	CU_ASSERT_STRING_EQUAL(chapterList[0].name, "Start");
	CU_ASSERT_EQUAL(chapterList[1].ts_us, duration / 2);
	CU_ASSERT_STRING_EQUAL(chapterList[1].name, "Halfway");
	for (size_t i = 0; i < chapterCount; i++)
		free(const_cast<char *>(chapterList[i].name));
	free(chapterList);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	CU_ASSERT_EQUAL(reopenListener.mCloseStatus, 0);
	reopenDemuxerOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_multi.mp4";

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	char path[512];
	PDRAW_GET_ASSET_PATH(path, ASSET_VIDEO_H264, s_assets_pipeline_muxer);

	struct pdraw_demuxer_params demuxParams = {};
	demuxParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	demuxParams.playback_mode = PDRAW_PLAYBACK_MODE_OFFLINE;

	PipelineDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(
		path, &demuxParams, &demuxListener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(demuxListener.mReady);

	/* This fixture asset has both a video and an audio track, both
	 * auto-selected by default (confirmed via ulog: "auto-selecting
	 * medias { - 1 (VideoHandler) - 2 (SoundHandler) }"). Copied by
	 * value for the same reallocation-safety reason as
	 * runMuxerRecordsRealCodedVideo above. */
	const CodedMediaTrackingListener::Added *videoPtr =
		mediaListener.findCodedVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(videoPtr);
	CodedMediaTrackingListener::Added video = *videoPtr;
	const CodedMediaTrackingListener::Added *audioPtr =
		mediaListener.findAudioMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(audioPtr);
	CodedMediaTrackingListener::Added audio = *audioPtr;
	CU_ASSERT_NOT_EQUAL_FATAL(video.id, audio.id);

	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mPlayStatus, 0);

	(void)remove(kOutPath);

	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* is_default=true: without it, neither track is marked as a default
	 * media in the written file, so re-opening it below with the same
	 * "-ENOSYS -> choose the default medias" selection logic would find
	 * none and never reach ready-to-play (found the hard way: this is
	 * what a real player also relies on to auto-select tracks). */
	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(video.id, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = muxer->addMedia(audio.id, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Wait until both tracks have at least one real recorded frame
	 * (see getStats() rationale in runMuxerRecordsRealCodedVideo). */
	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= 1) &&
			       (stats.record.audio_frames >= 1);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner3 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner3.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	/* Tear down the source demuxer before re-opening the freshly written
	 * file: no need to keep two demuxers open simultaneously here. */
	auto demuxerOwner3 = std::unique_ptr<IPdraw::IDemuxer>(demuxer);
	ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDemuxClose = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotCloseResponse; });
	CU_ASSERT_TRUE(gotDemuxClose);
	CU_ASSERT_EQUAL(demuxListener.mCloseStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);
	demuxerOwner3.reset();

	/* The strongest proof that a genuinely valid, playable MP4 file was
	 * produced (not just a nonempty blob): re-open it with our own
	 * demuxer and check it behaves like any other recording, with both
	 * tracks present. */
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenDemuxerOwner3 =
		std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);
	CU_ASSERT(reopenDemuxer->getDuration() > 0);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = reopenDemuxer->getMediaList(
		&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount >= 2);
	bool foundReopenedVideo = false;
	bool foundReopenedAudio = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type == PDRAW_MEDIA_TYPE_VIDEO)
			foundReopenedVideo = true;
		else if (mediaList[i].type == PDRAW_MEDIA_TYPE_AUDIO)
			foundReopenedAudio = true;
	}
	CU_ASSERT_TRUE(foundReopenedVideo);
	CU_ASSERT_TRUE(foundReopenedAudio);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	CU_ASSERT_EQUAL(reopenListener.mCloseStatus, 0);
	reopenDemuxerOwner3.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Duplicated from the photo-muxer section of this suite (kept split
 * across files -- see test_pipeline_muxer_record_photo.cpp): this
 * listener is also needed here by the raw-video ISOBMFF tests below.
 * Anonymous-namespace internal linkage avoids any ODR conflict with
 * the identically named class in the sibling file. */
class PhotoSourceMediaListener : public IPdraw::Listener {
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
		if (elementUserData != mSource)
			return;
		mMediaId = info->id;
		mGotMediaAdded = true;
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
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia (pdraw_muxer_record_isobmff_
 * media.cpp) is never exercised anywhere else in this suite: both
 * runMuxerRecordsRealCodedVideo and testCxxMuxerRecordsVideoAndAudio
 * TracksThenRoundtrips only ever record coded video / audio tracks, coming
 * from a real demuxer rather than a synthetic source. This builds a
 * standalone IRawVideoSource feeding an ISOBMFF (.mp4) muxer directly --
 * same "real ICodedVideoSource -> photo muxer" pattern used throughout
 * this file's photo muxer tests, just with a raw video source and an
 * ISOBMFF muxer instead.
 *
 * IsobmffRecordMuxer only accepts vdef_raw8/vdef_raw16 as raw video input
 * (see getSupportedRawFormats() in pdraw_muxer_record_isobmff.cpp) -- a
 * simple single-plane format, unlike planar YUV (e.g. I420, used
 * elsewhere in this test suite for raw video sources), which it rejects
 * outright (confirmed the hard way: addMedia() failed with -ENOSYS,
 * "raw video media format [...] not supported", when this test first used
 * vdef_i420). Correctly strided/sized via vdef_calc_raw_frame_size() so
 * IsobmffMuxerRawVideoMedia::processFrame() reads a self-consistent
 * packed buffer. */
static void testCxxMuxerRecordsRawVideoTrack()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "pdraw_test_pipeline_muxer_raw_video");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_raw_video.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	/* Same rationale as runMuxerRecordsRealCodedVideo: getStats()
	 * reports real muxed frame counts regardless of muxer/media type. */
	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner4 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner4 = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner4.reset();
	sourceOwner4.reset();

	/* Same "ground truth" check as runMuxerRecordsRealCodedVideo. */
	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	/* Same scope as testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips:
	 * this suite's job is only to confirm the produced file is genuinely
	 * valid and re-openable, not to deeply exercise the demuxer's raw
	 * video media handling (DemuxerRawVideoMedia::processSample(), still
	 * untouched at this point) -- that belongs in test_api_demuxer.cpp,
	 * the demuxer-focused suite. */
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenDemuxerOwner4 =
		std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);
	CU_ASSERT(reopenDemuxer->getDuration() > 0);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = reopenDemuxer->getMediaList(
		&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount >= 1);
	bool foundReopenedVideo = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type == PDRAW_MEDIA_TYPE_VIDEO)
			foundReopenedVideo = true;
	}
	CU_ASSERT_TRUE(foundReopenedVideo);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	CU_ASSERT_EQUAL(reopenListener.mCloseStatus, 0);
	reopenDemuxerOwner4.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxMuxerRecordsRawVideoTrackRollbackAndMetadata()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "pdraw_test_pipeline_muxer_raw_video_rollback");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_raw_video_rollback.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	uint64_t timestamps[3] = {0, 1, 2};

	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 10000000;
		frameInfo.info.timestamp = timestamps[i];
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		struct vmeta_frame *metadata = nullptr;
		ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Coded-video sibling of testCxxMuxerRecordsRawVideoTrackRollbackAndMetadata
 * above: same standalone "synthetic source -> ISOBMFF muxer" pattern, but
 * through IsobmffMuxerCodedVideoMedia::processFrame() (pdraw_muxer_record_
 * isobmff_media.cpp:213-391) instead of the raw video sibling.
 *
 * Reproduces the exact same forced timestamp rollback: each frame's
 * info.timescale (10000000) is far higher than the track's mTimescale
 * (DEFAULT_MP4_TIMESCALE == 90000, see pdraw_muxer_record_isobmff.hpp), so
 * mp4_convert_timescale() truncates every one of the small timestamps 0, 1,
 * 2 down to 0. This was confirmed against the analogous raw-video test's
 * real log output:
 *   frame 0: dts=0 (written)
 *   frame 1: computed dts=0 == mLastSampleTs(0) -> "duplicate timestamp (0),
 *            incrementing" -> written at dts=1
 *   frame 2: computed dts=0 < mLastSampleTs(1) -> "timestamp rollback from 1
 *            to 0, incrementing" -> written at dts=2
 * i.e. both the duplicate-timestamp AND the timestamp-rollback branches
 * (pdraw_muxer_record_isobmff_media.cpp:356-367) are exercised by construction,
 * not by chance. Every frame also carries a vmeta_frame (VMETA_FRAME_TYPE_
 * PROTO), same as the raw video sibling, exercising the metadata attach path
 * alongside the rollback handling. */
static void testCxxMuxerRecordsCodedVideoTrackRollbackAndMetadata()
{
	static const uint8_t kH264Sps[] = {0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9,
					   0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
					   0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B,
					   0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
	static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};

	/* Must match the resolution actually encoded in kH264Sps, see
	 * testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample below. */
	const unsigned int kWidth = 1920;
	const unsigned int kHeight = 800;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_h264_avcc;
	sourceParams.video.coded.info.resolution.width = kWidth;
	sourceParams.video.coded.info.resolution.height = kHeight;
	sourceParams.video.coded.info.bit_depth = 8;
	memcpy(sourceParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	sourceParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(sourceParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	sourceParams.video.coded.h264.ppslen = sizeof(kH264Pps);
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_coded_video_rollback");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_coded_video_rollback.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	uint64_t timestamps[3] = {0, 1, 2};

	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_coded_frame frameInfo = {};
		frameInfo.format = vdef_h264_avcc;
		frameInfo.info.timescale = 10000000;
		frameInfo.info.timestamp = timestamps[i];
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

		struct mbuf_coded_video_frame *frame = nullptr;
		ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(sizeof(kH264Pps), &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct vdef_nalu nalu = {};
		nalu.size = sizeof(kH264Pps);
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
		ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		struct vmeta_frame *metadata = nullptr;
		ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_coded_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_coded_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_coded_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Audio sibling of testCxxMuxerRecordsRawVideoTrackRollbackAndMetadata /
 * testCxxMuxerRecordsCodedVideoTrackRollbackAndMetadata above, through
 * IsobmffMuxerAudioMedia::processFrame() (pdraw_muxer_record_isobmff_media.
 * cpp:764-860) -- never exercised anywhere else in this suite on its own
 * (only alongside video, from a real demuxer, in
 * testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips).
 *
 * Unlike the video siblings, an audio track cannot come from a standalone
 * IAudioSource fed straight into the muxer without a real AAC-LC
 * AudioSpecificConfig: IsobmffMuxerAudioMedia::setup() calls
 * mp4_mux_track_set_audio_specific_config(), which rejects asc_size == 0
 * (libmp4/src/mp4_mux.c:1490). Before ExternalAudioSource::start() was fixed
 * to copy audio.aac_lc.asc/asclen into the output AudioMedia (pdraw_external_
 * audio_source.cpp, mirroring the H.264/H.265 SPS/PPS handling in
 * ExternalCodedVideoSource::start()), a synthetic IAudioSource always left
 * that ASC empty -- see testCxxAudioSourceExposesAacLcAsc in
 * test_pipeline_sourcesink_audio.cpp for the regression test covering that
 * fix directly. This test only becomes possible because of it.
 *
 * Same forced-rollback trick as the two video siblings: info.timescale
 * (10000000) is far higher than the track's mTimescale (90000), so the three
 * small timestamps 0, 1, 2 all truncate to a converted dts of 0, driving
 * IsobmffMuxerAudioMedia::processFrame() through the same duplicate-then-
 * rollback increment sequence (pdraw_muxer_record_isobmff_media.cpp:822-833)
 * as the video siblings. IsobmffMuxerAudioMedia::processFrame() has no
 * per-frame metadata path (unlike the video media classes), so this test
 * only covers the rollback handling, not metadata. */
static void testCxxMuxerRecordsAudioTrackRollback()
{
	static const uint8_t kAsc[] = {0x12, 0x08}; /* AAC-LC 44100Hz mono */
	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_audio_rollback.mp4";

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = adef_aac_lc_16b_44100hz_mono_raw;
	memcpy(sourceParams.audio.aac_lc.asc, kAsc, sizeof(kAsc));
	sourceParams.audio.aac_lc.asclen = sizeof(kAsc);

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	uint64_t timestamps[3] = {0, 1, 2};

	for (unsigned int i = 0; i < 3; i++) {
		struct adef_frame frameInfo = {};
		frameInfo.format = adef_aac_lc_16b_44100hz_mono_raw;
		frameInfo.info.timescale = 10000000;
		frameInfo.info.timestamp = timestamps[i];
		frameInfo.info.index = i;

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(sizeof(kAsc), &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0, capacity);

		struct mbuf_audio_frame *frame = nullptr;
		ret = mbuf_audio_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_audio_frame_set_buffer(frame, mem, 0, capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		ret = mbuf_audio_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_audio_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_audio_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.audio_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* RecordMuxer::ensureFreeSpace() (pdraw_muxer_record.cpp) is shared by every
 * record muxer, photo and ISOBMFF alike; test_pipeline_muxer_record_photo.cpp
 * already covers its per-frame failure path once, through the photo (PNG)
 * muxer's saveToDiskIov(). The three tests below repeat the exact same
 * "free_space_limit == current free space" trick (see the comment on
 * testCxxPhotoMuxerReportsUnrecoverableErrorOnFreeSpaceCheckFailure there:
 * the spaceNeeded=0 check at start passes because mFreeSpaceLeft ==
 * free_space_limit is not "<", while the very next check -- for the first
 * frame, with spaceNeeded>0 -- fails deterministically) but through each of
 * the three ISOBMFF media classes' own ensureFreeSpace() call site instead:
 * IsobmffMuxerRawVideoMedia::processFrame() (pdraw_muxer_record_isobmff_
 * media.cpp:630), IsobmffMuxerCodedVideoMedia::processFrame() (:372),
 * IsobmffMuxerAudioMedia::processFrame() (:838).
 *
 * Unlike the photo muxer, IsobmffRecordMuxer::onBeforeAddMuxerMedias()
 * (pdraw_muxer_record_isobmff.cpp:444) calls mp4_mux_open() unconditionally
 * at start, before any frame is ever pushed -- so, unlike the photo test, the
 * output file already exists by the time the first frame's ensureFreeSpace()
 * check fails; only the frame counter staying at 0 distinguishes "recorded"
 * from "rejected" here.
 *
 * kFreeSpaceTestMargin: the exact-current-free-space trick above is exposed
 * to a real race -- ensureFreeSpace() re-reads statvfs() itself, and on a
 * busy/shared filesystem the reported free space can tick down by a few KiB
 * between our measurement and that re-read, which would trip the spaceNeeded
 * =0 start check instead of the intended first-frame one. Subtracting a
 * margin from free_space_limit absorbs that jitter -- but the margin must
 * stay smaller than the pushed frame's byte size, or the first-frame check
 * stops failing too. kFreeSpaceTestPaddedFrameSize exists for that reason:
 * it inflates the (otherwise tiny, a few bytes to a few hundred bytes)
 * synthetic per-track payload used below well above kFreeSpaceTestMargin, so
 * both checks stay deterministic. */
static const size_t kFreeSpaceTestPaddedFrameSize = 256 * 1024;
static const size_t kFreeSpaceTestMargin = 64 * 1024;


static void
testCxxMuxerRawVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* Large enough that the raw frame's byte size comfortably exceeds
	 * kFreeSpaceTestMargin (see comment above). */
	const unsigned int kWidth = 512;
	const unsigned int kHeight = 512;

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "pdraw_test_pipeline_muxer_raw_video_freespace");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_raw_video_freespace.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	struct statvfs vfsStats = {};
	CU_ASSERT_EQUAL_FATAL(statvfs("/tmp", &vfsStats), 0);
	muxerParams.free_space_limit =
		(size_t)vfsStats.f_bavail * vfsStats.f_bsize -
		kFreeSpaceTestMargin;
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_raw8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = kWidth;
	frameInfo.info.resolution.height = kHeight;
	frameInfo.info.bit_depth = 8;
	for (unsigned int p = 0; p < planeCount; p++)
		frameInfo.plane_stride[p] = planeStride[p];

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	for (unsigned int p = 0; p < planeCount; p++) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(planeSize[p], &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0x80, capacity);
		ret = mbuf_raw_video_frame_set_plane(
			frame, p, mem, 0, planeSize[p]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);
	}

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotError);

	struct pdraw_muxer_stats stats = {};
	ret = muxer->getStats(&stats);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(stats.record.raw_video_frames, 0u);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	muxerOwner.reset();
	sourceOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void
testCxxMuxerCodedVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure()
{
	static const uint8_t kH264Sps[] = {0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9,
					   0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
					   0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B,
					   0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
	static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};

	/* Must match the resolution actually encoded in kH264Sps, see
	 * testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample below. */
	const unsigned int kWidth = 1920;
	const unsigned int kHeight = 800;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_h264_avcc;
	sourceParams.video.coded.info.resolution.width = kWidth;
	sourceParams.video.coded.info.resolution.height = kHeight;
	sourceParams.video.coded.info.bit_depth = 8;
	memcpy(sourceParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	sourceParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(sourceParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	sourceParams.video.coded.h264.ppslen = sizeof(kH264Pps);
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_coded_video_freespace");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_coded_video_freespace.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	struct statvfs vfsStats = {};
	CU_ASSERT_EQUAL_FATAL(statvfs("/tmp", &vfsStats), 0);
	muxerParams.free_space_limit =
		(size_t)vfsStats.f_bavail * vfsStats.f_bsize -
		kFreeSpaceTestMargin;
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = kWidth;
	frameInfo.info.resolution.height = kHeight;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Payload content is irrelevant: the frame is rejected by
	 * ensureFreeSpace() before ever being written to disk. Size is
	 * kFreeSpaceTestPaddedFrameSize (not sizeof(kH264Pps)) so it stays
	 * well above kFreeSpaceTestMargin -- see comment above. */
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(kFreeSpaceTestPaddedFrameSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = kFreeSpaceTestPaddedFrameSize;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotError);

	struct pdraw_muxer_stats stats = {};
	ret = muxer->getStats(&stats);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(stats.record.coded_video_frames, 0u);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	muxerOwner.reset();
	sourceOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void
testCxxMuxerAudioTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure()
{
	static const uint8_t kAsc[] = {0x12, 0x08}; /* AAC-LC 44100Hz mono */
	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_audio_freespace.mp4";

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = adef_aac_lc_16b_44100hz_mono_raw;
	memcpy(sourceParams.audio.aac_lc.asc, kAsc, sizeof(kAsc));
	sourceParams.audio.aac_lc.asclen = sizeof(kAsc);

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	struct statvfs vfsStats = {};
	CU_ASSERT_EQUAL_FATAL(statvfs("/tmp", &vfsStats), 0);
	muxerParams.free_space_limit =
		(size_t)vfsStats.f_bavail * vfsStats.f_bsize -
		kFreeSpaceTestMargin;
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	struct adef_frame frameInfo = {};
	frameInfo.format = adef_aac_lc_16b_44100hz_mono_raw;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;

	/* Content is irrelevant (zero-filled below): the frame is rejected by
	 * ensureFreeSpace() before ever being written to disk. Size is
	 * kFreeSpaceTestPaddedFrameSize (not sizeof(kAsc)) so it stays well
	 * above kFreeSpaceTestMargin -- see comment above. */
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(kFreeSpaceTestPaddedFrameSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);

	struct mbuf_audio_frame *frame = nullptr;
	ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_audio_frame_unref(frame);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotError);

	struct pdraw_muxer_stats stats = {};
	ret = muxer->getStats(&stats);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(stats.record.audio_frames, 0u);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	muxerOwner.reset();
	sourceOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* RecordMuxer::internalStart() (pdraw_muxer_record.cpp:811-834) runs its own
 * ensureFreeSpace(0) check before onBeforeAddMuxerMedias() ever calls
 * mp4_mux_open() -- unlike the three per-track tests above, a failure here
 * means Muxer::start() itself fails (pdraw_muxer.cpp:150-152), which
 * Session::createMuxer() (pdraw_session.cpp:601-605) propagates straight back
 * as createMuxer()'s own return value, synchronously, before any media is
 * ever added and before the output file is ever created (MuxerWrapper's
 * constructor releases the Element into Session::mElements immediately, see
 * pdraw_muxer.cpp:807-812, so the still-registered-but-never-started element
 * is safely torn down later by session->stop(), not leaked). A raw video
 * source is used here (as in the simplest per-track test above) purely as a
 * vehicle to reach createMuxer(): no frame is ever pushed, since the muxer
 * never starts.
 *
 * Using an absurdly high free_space_limit (rather than the exact-current-
 * free-space trick used in every other free-space test in this file) fails
 * the spaceNeeded=0 check too, unlike those other tests, where that same
 * check must pass at start for the per-frame check to be the one that fails
 * instead. */
static void testCxxMuxerReportsUnrecoverableErrorOnStartFreeSpaceCheckFailure()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;

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
		 "pdraw_test_pipeline_muxer_start_freespace");

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_start_freespace.mp4";
	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	/* Absurdly high on purpose: fails ensureFreeSpace(0) too (see the
	 * file-level comment above), unlike every other free-space test in
	 * this file, which relies on that particular check passing at start. */
	muxerParams.free_space_limit = (size_t)1024 * 1024 * 1024 * 1024 * 1024;
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, -ENOSPC);
	CU_ASSERT_PTR_NULL_FATAL(muxer);

	/* onMuxerUnrecoverableError() is never reached: the failure is
	 * reported synchronously through createMuxer()'s own return value
	 * instead (see the file-level comment above) -- there is no live
	 * IMuxer to call close() on. */
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	CU_ASSERT_FALSE(muxerListener.mGotCloseResponse.load());

	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_NOT_EQUAL(stat(kOutPath, &st), 0);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Exercises RecordMuxer::onChannelSessionMetaUpdate() (pdraw_muxer_record.cpp:
 * 961-1018), which is never reached by any other test in this file.
 * IRawVideoSource::setSessionMetadata() propagates SESSION_META_UPDATE
 * downstream synchronously, triggering onChannelSessionMetaUpdate on the loop
 * thread; that method posts a SET_METADATA task to the writer thread which
 * calls internalSetMetadata() → track->setSessionMeta() + mMetadataChanged.
 * At close, onInternalStopThread() calls mergeSessionMetadata() which reads
 * the updated track meta and flushes it to the file via
 * vmeta_session_recording_write. SET_METADATA is in the task queue before
 * STOP_THREAD (both processed in FIFO order), so the update is always written.
 * Verified by re-opening the file and checking
 * pdraw_demuxer_media::video.session_meta via getMediaList(). */
static void testCxxMuxerSessionMetaUpdatePropagates()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_session_meta_update.mp4";

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "initial-name");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	auto pushRawFrame = [&](uint64_t timestamp, unsigned int index) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 10000000;
		frameInfo.info.timestamp = timestamp;
		frameInfo.info.index = index;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		int r = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(r, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			r = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(r, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			r = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(r, 0);
			memset(data, 0x80, capacity);
			r = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(r, 0);
			mbuf_mem_unref(mem);
		}

		struct vmeta_frame *metadata = nullptr;
		r = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
		CU_ASSERT_EQUAL_FATAL(r, 0);
		r = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(r, 0);
		vmeta_frame_unref(metadata);

		r = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(r, 0);
		r = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(r, 0);
		mbuf_raw_video_frame_unref(frame);
	};

	pushRawFrame(0, 0);
	pushRawFrame(1, 1);

	bool gotInitialFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 1);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotInitialFrames);

	/* setSessionMetadata() calls sendDownstreamEvent(SESSION_META_UPDATE)
	 * synchronously, posting SET_METADATA to the writer task queue. The
	 * FIFO task queue guarantees SET_METADATA runs before the STOP_THREAD
	 * task posted by close(), so internalSetMetadata() always updates the
	 * track meta before mergeSessionMetadata() reads it at close. */
	struct vmeta_session newMeta = {};
	strncpy(newMeta.friendly_name,
		"updated-name",
		sizeof(newMeta.friendly_name) - 1);
	ret = source->setSessionMetadata(&newMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	pushRawFrame(2, 2);

	bool gotMoreFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMoreFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);

	/* Re-open the file and verify the updated friendly_name was flushed.
	 * fetchSessionMetadata() reads file-level and track-level MP4 metadata
	 * (both written by mergeSessionMetadata() at close) and populates
	 * pdraw_demuxer_media::video.session_meta.
	 * IsobmffMuxerRawVideoMedia uses MP4_TRACK_TYPE_METADATA, which the
	 * demuxer maps to PDRAW_MEDIA_TYPE_VIDEO
	 * (pdraw_demuxer_record.cpp:314), so the video media entry and its
	 * session_meta are always present. */
	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenOwner = std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = reopenDemuxer->getMediaList(
		&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount >= 1);

	bool foundVideoMeta = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		CU_ASSERT_STRING_EQUAL(
			mediaList[i].video.session_meta.friendly_name,
			"updated-name");
		foundVideoMeta = true;
		break;
	}
	CU_ASSERT_TRUE(foundVideoMeta);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	reopenOwner.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Exercises IsobmffMuxerMedia::sessionMetaWriteMediaCb (pdraw_muxer_record_
 * isobmff_media.cpp:178-194). That callback is invoked for each video track
 * from writeRecordingMetadata() → vmeta_session_recording_write() inside
 * mergeSessionMetadata() at close; it calls mp4_mux_add_track_metadata() for
 * every key/value pair produced by vmeta_session_recording_write(). Unlike
 * testCxxMuxerSessionMetaUpdatePropagates (which tests the task-queue update
 * path), this test verifies that the INITIAL session metadata – set at source
 * creation time and propagated via the normal channel session-meta mechanism –
 * is correctly written at track level and survives a close/reopen round-trip.
 * Three distinct fields (friendly_name, maker, model) are verified to confirm
 * that the callback processes all key/value pairs. */
static void testCxxMuxerSessionMetaWriteMediaCbPersistsMultipleFields()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_session_meta_cb.mp4";

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "meta-cb-test");
	snprintf(sourceParams.session_meta.maker,
		 sizeof(sourceParams.session_meta.maker),
		 "Parrot");
	snprintf(sourceParams.session_meta.model,
		 sizeof(sourceParams.session_meta.model),
		 "ANAFI-4K");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		struct vmeta_frame *metadata = nullptr;
		ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	/* Re-open and verify that all three fields set at source creation are
	 * present in the track session metadata written by
	 * sessionMetaWriteMediaCb via mp4_mux_add_track_metadata().
	 * fetchSessionMetadata() (pdraw_demuxer_ record.cpp) reads both
	 * file-level and track-level metadata; checking multiple distinct
	 * fields ensures the callback processed all key/value
	 * pairs produced by vmeta_session_recording_write(). */
	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenOwner = std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = reopenDemuxer->getMediaList(
		&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount >= 1);

	bool foundVideoWithMeta = false;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		CU_ASSERT_STRING_EQUAL(
			mediaList[i].video.session_meta.friendly_name,
			"meta-cb-test");
		CU_ASSERT_STRING_EQUAL(mediaList[i].video.session_meta.maker,
				       "Parrot");
		CU_ASSERT_STRING_EQUAL(mediaList[i].video.session_meta.model,
				       "ANAFI-4K");
		foundVideoWithMeta = true;
		break;
	}
	CU_ASSERT_TRUE(foundVideoWithMeta);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	CU_ASSERT_EQUAL(reopenListener.mCloseStatus, 0);
	reopenOwner.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Builds `frameCount` single-plane raw8 frames and pushes them into
 * `inQueue`. Shared by testCxxMuxerSessionMetaWriteMediaCbDistinctFields
 * SurviveMerge below, once per raw video source. */
static void pushRaw8Frames(struct mbuf_raw_video_frame_queue *inQueue,
			   unsigned int width,
			   unsigned int height,
			   const size_t *planeStride,
			   const size_t *planeSize,
			   unsigned int planeCount,
			   unsigned int frameCount)
{
	for (unsigned int i = 0; i < frameCount; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = width;
		frameInfo.info.resolution.height = height;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		int ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}
}


/* testCxxMuxerSessionMetaWriteMediaCbPersistsMultipleFields above passes a
 * real build's coverage report for writeRecordingMetadata() (pdraw_muxer_
 * record_isobmff_media.cpp:170) but NOT for sessionMetaWriteMediaCb itself
 * (line 178) -- confirmed by a real gcov run. Root cause, found by reading
 * vmeta_session_merge_metadata() (libvideo-metadata/src/vmeta_session.c:4015):
 * it runs in two passes over every video track's session metadata --
 * fill_session_meta_with_identical_values() keeps a field in the merged
 * (file-level) metadata only if EVERY track agrees on it, then
 * erase_session_meta_with_identical_values() ERASES that field from each
 * track's own copy once it made it into the merged result (no point storing
 * it twice). With a single video track, every field trivially "agrees with
 * itself", so it is always hoisted to file level and erased at the track
 * level -- IsobmffRecordMuxer::mergeSessionMetadata() (pdraw_muxer_record_
 * isobmff.cpp:703) then calls track->writeRecordingMetadata() with an
 * all-empty struct, vmeta_session_recording_write() finds nothing to emit,
 * and sessionMetaWriteMediaCb's body never runs.
 *
 * A field only survives at the track level if at least one other video
 * track disagrees with it. This test uses two raw video tracks with
 * distinct friendly_name values: neither is "identical across all tracks",
 * so erase_session_meta_with_identical_values() leaves both alone, and
 * writeRecordingMetadata() actually has non-empty data to write for both
 * tracks. Re-opening the file confirms each track kept its OWN distinct
 * friendly_name (fetchSessionMetadata(), pdraw_demuxer_record.cpp, reads
 * file-level metadata first and track-level metadata second into the same
 * struct -- track-level wins for any field it sets) rather than a single
 * shared file-level value, which would be empty here since the two tracks
 * disagree. */
static void testCxxMuxerSessionMetaWriteMediaCbDistinctFieldsSurviveMerge()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_session_meta_cb_multi.mp4";
	static const char *kNames[2] = {"track-A-meta", "track-B-meta"};

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

	IPdraw::IRawVideoSource *sources[2] = {nullptr, nullptr};

	for (unsigned int s = 0; s < 2; s++) {
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
			 "%s",
			 kNames[s]);

		ret = session->createRawVideoSource(
			&sourceParams,
			&g_stub_raw_video_source_listener,
			&sources[s]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(sources[s]);

		size_t expectedCount = s + 1;
		bool gotMediaAdded = loop.pumpUntil([&mediaListener,
						     expectedCount]() {
			return mediaListener.mAdded.size() >= expectedCount;
		});
		CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	}

	CU_ASSERT_EQUAL_FATAL(mediaListener.mAdded.size(), 2u);
	unsigned int mediaIds[2] = {mediaListener.mAdded[0].id,
				    mediaListener.mAdded[1].id};

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	for (unsigned int s = 0; s < 2; s++) {
		struct pdraw_muxer_media_params mediaParams = {};
		mediaParams.is_default = true;
		ret = muxer->addMedia(mediaIds[s], &mediaParams);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	for (unsigned int s = 0; s < 2; s++) {
		struct mbuf_raw_video_frame_queue *inQueue =
			sources[s]->getQueue();
		CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
		pushRaw8Frames(inQueue,
			       kWidth,
			       kHeight,
			       planeStride,
			       planeSize,
			       planeCount,
			       2);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 4);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	std::vector<std::unique_ptr<IPdraw::IRawVideoSource>> sourceOwners;
	for (unsigned int s = 0; s < 2; s++)
		sourceOwners.push_back(
			std::unique_ptr<IPdraw::IRawVideoSource>(sources[s]));

	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwners.clear();

	PipelineDemuxerListener reopenListener;
	IPdraw::IDemuxer *reopenDemuxer = nullptr;
	struct pdraw_demuxer_params reopenParams = {};
	reopenParams.autodecoding_mode =
		PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;
	ret = session->createDemuxer(
		kOutPath, &reopenParams, &reopenListener, &reopenDemuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reopenDemuxer);
	auto reopenOwner = std::unique_ptr<IPdraw::IDemuxer>(reopenDemuxer);

	bool gotReopenReady = loop.pumpUntil(
		[&reopenListener]() { return reopenListener.mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReopenReady);
	CU_ASSERT_TRUE_FATAL(reopenListener.mReady);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	ret = reopenDemuxer->getMediaList(
		&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT(mediaCount >= 2);

	std::vector<std::string> foundNames;
	for (size_t i = 0; i < mediaCount; i++) {
		if (mediaList[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		foundNames.push_back(
			mediaList[i].video.session_meta.friendly_name);
	}
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	CU_ASSERT_EQUAL(foundNames.size(), 2u);
	bool foundA = false, foundB = false;
	for (const auto &n : foundNames) {
		if (n == kNames[0])
			foundA = true;
		else if (n == kNames[1])
			foundB = true;
	}
	CU_ASSERT_TRUE(foundA);
	CU_ASSERT_TRUE(foundB);

	ret = reopenDemuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotReopenClose = loop.pumpUntil([&reopenListener]() {
		return reopenListener.mGotCloseResponse;
	});
	CU_ASSERT_TRUE(gotReopenClose);
	CU_ASSERT_EQUAL(reopenListener.mCloseStatus, 0);
	reopenOwner.reset();

	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Exercises IsobmffMuxerMedia::addMetadata() (pdraw_muxer_record_isobmff_
 * media.cpp:73-167). That method is called from IsobmffMuxerRawVideoMedia::
 * processFrame() when the first frame carrying a vmeta_frame is encountered;
 * it creates a MP4_TRACK_TYPE_METADATA timed-metadata track with the vmeta
 * MIME type and links it to the video track via mp4_mux_add_ref_to_track().
 * This test verifies the track is present and carries the expected MIME type
 * by opening the output file with mp4_demux and inspecting mp4_track_info. */
static void testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_vmeta_meta_track.mp4";

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "vmeta-track-test");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	/* Push frames with vmeta_frame attached: the first such frame triggers
	 * addMetadata() which creates the timed-metadata track in the MP4 file.
	 */
	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		/* Frame 0 carries no vmeta: mFirstSampleTs (dts≈0) gets set on
		 * frame 0 without metadata. Frame 1 (dts≈3000 at 90kHz) carries
		 * vmeta, triggering addMetadata(); at that point mLastSampleTs
		 * (≈3000) > mFirstSampleTs (0) so addMetadata() writes the
		 * 8-byte VMETA_FRAME_PROTO_EMPTY_COOKIE initial sample —
		 * guaranteeing the vmeta track has at least one sample and is
		 * written to the file. Without this, an empty PROTO frame
		 * produces 0 bytes from vmeta_frame_proto_get_buffer() → no
		 * sample added → libmp4 omits the zero-sample track from the
		 * output file entirely. */
		if (i > 0) {
			struct vmeta_frame *metadata = nullptr;
			ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO,
					      &metadata);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			ret = mbuf_raw_video_frame_set_metadata(frame,
								metadata);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			vmeta_frame_unref(metadata);
		}

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	/* Open the output file with libmp4 directly to verify the vmeta
	 * timed-metadata track was created alongside the raw-video track.
	 * IsobmffMuxerRawVideoMedia itself is MP4_TRACK_TYPE_METADATA with
	 * mime_format "video/raw;..."; the vmeta track from addMetadata() is
	 * a second METADATA track with VMETA_FRAME_PROTO_MIME_TYPE. */
	struct mp4_demux *mp4 = nullptr;
	ret = mp4_demux_open(kOutPath, &mp4);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mp4);

	int trackCount = mp4_demux_get_track_count(mp4);
	/* At minimum: raw-video track + vmeta metadata track */
	CU_ASSERT(trackCount >= 2);

	bool foundVmetaTrack = false;
	for (int i = 0; i < trackCount; i++) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mp4, i, &tk);
		if (ret < 0)
			continue;
		if (tk.type != MP4_TRACK_TYPE_METADATA)
			continue;
		if (tk.mime_format == nullptr)
			continue;
		if (strcmp(tk.mime_format, VMETA_FRAME_PROTO_MIME_TYPE) == 0) {
			foundVmetaTrack = true;
			break;
		}
	}
	CU_ASSERT_TRUE(foundVmetaTrack);

	mp4_demux_close(mp4);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Exercises the "Add a metadata sample to the MP4 muxer" block inside
 * IsobmffMuxerRawVideoMedia::processFrame() (pdraw_muxer_record_isobmff_
 * media.cpp:660-706) -- the real per-frame sample write, as opposed to
 * addMetadata()'s one-time initial empty-cookie sample. The test just above
 * (testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack) only ever attaches an
 * empty VMETA_FRAME_TYPE_PROTO frame, which makes vmeta_frame_proto_get_
 * buffer() return 0 bytes -- so metaContent/metaLen stay null/0 there and
 * this block's mp4_mux_track_add_sample() call on mMetaTrackId is never
 * reached: it only ever proves addMetadata() itself ran.
 *
 * VMETA_FRAME_TYPE_V3 is used here instead (same reasoning as the CODED
 * video sibling test below): vmeta_frame_v3_write() always serializes a
 * fixed-size binary header+fields (168 bytes max) regardless of field
 * content, so metaLen > 0 is guaranteed.
 *
 * Every pushed frame carries metadata (unlike the sibling test above, which
 * deliberately withholds it from frame 0), so: frame 0 triggers
 * addMetadata() (mFirstSampleTs == mLastSampleTs on the very first sample,
 * so addMetadata()'s own initial-sample guard is false -- it does not write
 * a sample itself) and then, still within the same processFrame() call,
 * falls through to the block under test since mMetaTrackId is now non-zero;
 * frames 1 and 2 hit the block directly (mHasMetadata already true). Three
 * frames in => three real per-frame metadata samples out. Verified by
 * reopening the file with mp4_demux directly and checking the vmeta track's
 * sample_count. */
static void testCxxMuxerRawVideoVmetaFrameWritesMetadataSample()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_raw_vmeta_sample.mp4";

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;
	const unsigned int kFrameCount = 3;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "raw-vmeta-sample-test");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < kFrameCount; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		struct vmeta_frame *metadata = nullptr;
		ret = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.raw_video_frames >= kFrameCount);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	/* sample_count on the vmeta track is the strongest available proof
	 * that the per-frame "Add a metadata sample" block ran repeatedly,
	 * not just addMetadata() itself (which does NOT write a sample here,
	 * see the comment above: mFirstSampleTs == mLastSampleTs on frame 0).
	 */
	struct mp4_demux *mp4 = nullptr;
	ret = mp4_demux_open(kOutPath, &mp4);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mp4);

	int trackCount = mp4_demux_get_track_count(mp4);
	CU_ASSERT(trackCount >= 2);

	bool foundVmetaTrack = false;
	for (int i = 0; i < trackCount; i++) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mp4, i, &tk);
		if (ret < 0)
			continue;
		if (tk.type != MP4_TRACK_TYPE_METADATA)
			continue;
		if (tk.mime_format == nullptr)
			continue;
		if (strcmp(tk.mime_format, VMETA_FRAME_V3_MIME_TYPE) != 0)
			continue;
		foundVmetaTrack = true;
		CU_ASSERT(tk.sample_count >= kFrameCount);
		break;
	}
	CU_ASSERT_TRUE(foundVmetaTrack);

	mp4_demux_close(mp4);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Builds a VMETA_FRAME_TYPE_PROTO metadata frame with real (non-default)
 * content -- unlike the empty PROTO frames used by
 * testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack, an empty protobuf
 * message serializes to 0 bytes (default-valued fields are omitted by
 * protobuf wire format), which would make vmeta_frame_proto_get_buffer()
 * return metaLen == 0 in pdraw_muxer_record_isobmff_media.cpp:678-684 and
 * skip the real sample write entirely. Setting a non-default field (here,
 * two thermal spot values) guarantees a non-empty packed buffer. Same
 * writer API (vmeta_frame_proto_get_unpacked_rw/get_thermal/get_thermal_min/
 * get_thermal_max/release_unpacked_rw) as
 * testCxxVideoRendererAutoNormalizationThermal in
 * test_pipeline_renderer_video.cpp. */
static struct vmeta_frame *newNonEmptyProtoVmeta()
{
	struct vmeta_frame *metadata = nullptr;
	int ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	Vmeta__TimedMetadata *tm = nullptr;
	ret = vmeta_frame_proto_get_unpacked_rw(metadata, &tm);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	Vmeta__ThermalMetadata *thermal = vmeta_frame_proto_get_thermal(tm);
	CU_ASSERT_PTR_NOT_NULL_FATAL(thermal);
	Vmeta__ThermalSpot *minSpot =
		vmeta_frame_proto_get_thermal_min(thermal);
	CU_ASSERT_PTR_NOT_NULL_FATAL(minSpot);
	minSpot->value = 100;
	Vmeta__ThermalSpot *maxSpot =
		vmeta_frame_proto_get_thermal_max(thermal);
	CU_ASSERT_PTR_NOT_NULL_FATAL(maxSpot);
	maxSpot->value = 300;
	ret = vmeta_frame_proto_release_unpacked_rw(metadata, tm);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return metadata;
}


/* Same as testCxxMuxerRawVideoVmetaFrameWritesMetadataSample above, but with
 * a non-empty VMETA_FRAME_TYPE_PROTO instead of VMETA_FRAME_TYPE_V3: proves
 * the "else" (PROTO) branch of "if (metadata->type != VMETA_FRAME_TYPE_
 * PROTO) {...} else {...}" (pdraw_muxer_record_isobmff_media.cpp:664-684)
 * also reaches the real sample write with metaLen > 0, and not just the
 * metaLen == 0 empty case already covered by
 * testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack. */
static void testCxxMuxerRawVideoVmetaFrameWritesMetadataSampleProto()
{
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_raw_vmeta_sample_proto.mp4";

	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;
	const unsigned int kFrameCount = 3;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

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
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "raw-vmeta-sample-proto-test");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < kFrameCount; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		struct vmeta_frame *metadata = newNonEmptyProtoVmeta();
		ret = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.raw_video_frames >= kFrameCount);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct mp4_demux *mp4 = nullptr;
	ret = mp4_demux_open(kOutPath, &mp4);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mp4);

	int trackCount = mp4_demux_get_track_count(mp4);
	CU_ASSERT(trackCount >= 2);

	bool foundVmetaTrack = false;
	for (int i = 0; i < trackCount; i++) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mp4, i, &tk);
		if (ret < 0)
			continue;
		if (tk.type != MP4_TRACK_TYPE_METADATA)
			continue;
		if (tk.mime_format == nullptr)
			continue;
		if (strcmp(tk.mime_format, VMETA_FRAME_PROTO_MIME_TYPE) != 0)
			continue;
		foundVmetaTrack = true;
		CU_ASSERT(tk.sample_count >= kFrameCount);
		break;
	}
	CU_ASSERT_TRUE(foundVmetaTrack);

	mp4_demux_close(mp4);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Exercises the "Add a metadata sample to the MP4 muxer" block inside
 * IsobmffMuxerCodedVideoMedia::processFrame() (pdraw_muxer_record_isobmff_
 * media.cpp:408-449) -- the CODED video sibling of the RAW video path
 * covered just above by testCxxMuxerRawVideoVmetaFrameWritesMetadataSample().
 * That block is reached when mbuf_coded_video_frame_get_metadata() succeeds
 * (a vmeta_frame was attached via mbuf_coded_video_frame_set_metadata()) AND
 * mMetaTrackId is non-zero (set by addMetadata(), itself invoked from
 * processFrame() -- pdraw_muxer_record_isobmff_media.cpp:403-407 -- the
 * first time a frame with metadata is seen).
 *
 * VMETA_FRAME_TYPE_V3 is used here, not _PROTO: vmeta_frame_v3_write()
 * (libvideo-metadata/src/vmeta_frame_v3.c:34) always serializes a fixed-size
 * binary header+fields (VMETA_FRAME_V3_MAX_SIZE=168 bytes) regardless of
 * field content, so metaLen > 0 even for a zero-initialized struct --
 * unlike VMETA_FRAME_TYPE_PROTO, which produces 0 bytes when empty (see the
 * comment in testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack above) and
 * would only ever exercise addMetadata()'s one-time initial empty-cookie
 * sample, never this per-frame real sample write.
 *
 * Every pushed frame is marked VDEF_CODED_FRAME_TYPE_IDR (isSync=true, see
 * ExternalCodedVideoSource::processFrame() -- pdraw_external_coded_video_
 * source.cpp:778-779) and carries metadata, so: frame 0 triggers
 * addMetadata() (mFirstSampleTs == mLastSampleTs on the very first sample,
 * so addMetadata()'s own initial-sample guard is false -- it does not write
 * a sample itself) and then, still within the same processFrame() call,
 * falls through to the block under test since mMetaTrackId is now non-zero;
 * frames 1 and 2 hit the block directly (mHasMetadata already true). Three
 * frames in => three real per-frame metadata samples out, independently of
 * addMetadata()'s own sample. Verified by reopening the file with mp4_demux
 * directly and checking the vmeta track's sample_count. */
static void testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample()
{
	static const uint8_t kH264Sps[] = {0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9,
					   0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
					   0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B,
					   0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
	static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_coded_vmeta_sample.mp4";

	/* Must match the resolution actually encoded in kH264Sps: setPs()
	 * (pdraw_media.cpp:184) parses the real SPS/PPS via h264_get_info()
	 * and overwrites info.resolution with the parsed crop_width/height,
	 * ignoring whatever was set in sourceParams.video.coded.info --
	 * inputFilter() then rejects any pushed frame whose own resolution
	 * doesn't match that SPS-derived value. */
	const unsigned int kWidth = 1920;
	const unsigned int kHeight = 800;
	const unsigned int kFrameCount = 3;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_h264_avcc;
	sourceParams.video.coded.info.resolution.width = kWidth;
	sourceParams.video.coded.info.resolution.height = kHeight;
	sourceParams.video.coded.info.bit_depth = 8;
	memcpy(sourceParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	sourceParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(sourceParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	sourceParams.video.coded.h264.ppslen = sizeof(kH264Pps);
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "coded-vmeta-sample-test");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	/* Every frame carries a vmeta_frame: frame 0 triggers addMetadata(),
	 * every frame (0 included, same processFrame() call) then hits the
	 * "Add a metadata sample" block under test. */
	for (unsigned int i = 0; i < kFrameCount; i++) {
		struct vdef_coded_frame frameInfo = {};
		frameInfo.format = vdef_h264_avcc;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

		struct mbuf_coded_video_frame *frame = nullptr;
		ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(sizeof(kH264Pps), &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct vdef_nalu nalu = {};
		nalu.size = sizeof(kH264Pps);
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
		ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		struct vmeta_frame *metadata = nullptr;
		ret = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_coded_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_coded_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_coded_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= kFrameCount);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	/* Open the output file with libmp4 directly: sample_count on the
	 * vmeta track is the strongest available proof that the per-frame
	 * "Add a metadata sample" block ran repeatedly, not just addMetadata()
	 * itself (which does NOT write a sample here, see the comment above:
	 * mFirstSampleTs == mLastSampleTs on frame 0). */
	struct mp4_demux *mp4 = nullptr;
	ret = mp4_demux_open(kOutPath, &mp4);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mp4);

	int trackCount = mp4_demux_get_track_count(mp4);
	CU_ASSERT(trackCount >= 2);

	bool foundVmetaTrack = false;
	for (int i = 0; i < trackCount; i++) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mp4, i, &tk);
		if (ret < 0)
			continue;
		if (tk.type != MP4_TRACK_TYPE_METADATA)
			continue;
		if (tk.mime_format == nullptr)
			continue;
		if (strcmp(tk.mime_format, VMETA_FRAME_V3_MIME_TYPE) != 0)
			continue;
		foundVmetaTrack = true;
		CU_ASSERT(tk.sample_count >= kFrameCount);
		break;
	}
	CU_ASSERT_TRUE(foundVmetaTrack);

	mp4_demux_close(mp4);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same as testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample above, but
 * with a non-empty VMETA_FRAME_TYPE_PROTO instead of VMETA_FRAME_TYPE_V3:
 * proves the "else" (PROTO) branch of "if (metadata->type != VMETA_FRAME_
 * TYPE_PROTO) {...} else {...}" (pdraw_muxer_record_isobmff_media.cpp:
 * 412-432) also reaches the real sample write with metaLen > 0. See
 * newNonEmptyProtoVmeta() above for why a populated (not default-empty)
 * PROTO frame is required. */
static void testCxxMuxerCodedVideoVmetaFrameWritesMetadataSampleProto()
{
	static const uint8_t kH264Sps[] = {0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9,
					   0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
					   0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B,
					   0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
	static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};
	static const char *kOutPath =
		"/tmp/pdraw_test_muxer_coded_vmeta_sample_proto.mp4";

	/* Must match the resolution actually encoded in kH264Sps, see
	 * testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample above. */
	const unsigned int kWidth = 1920;
	const unsigned int kHeight = 800;
	const unsigned int kFrameCount = 3;

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_h264_avcc;
	sourceParams.video.coded.info.resolution.width = kWidth;
	sourceParams.video.coded.info.resolution.height = kHeight;
	sourceParams.video.coded.info.bit_depth = 8;
	memcpy(sourceParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	sourceParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(sourceParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	sourceParams.video.coded.h264.ppslen = sizeof(kH264Pps);
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "coded-vmeta-sample-proto-test");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	(void)remove(kOutPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < kFrameCount; i++) {
		struct vdef_coded_frame frameInfo = {};
		frameInfo.format = vdef_h264_avcc;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = (uint64_t)i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

		struct mbuf_coded_video_frame *frame = nullptr;
		ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(sizeof(kH264Pps), &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct vdef_nalu nalu = {};
		nalu.size = sizeof(kH264Pps);
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
		ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		struct vmeta_frame *metadata = newNonEmptyProtoVmeta();
		ret = mbuf_coded_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		vmeta_frame_unref(metadata);

		ret = mbuf_coded_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_coded_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) &&
			       (stats.record.coded_video_frames >= kFrameCount);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct mp4_demux *mp4 = nullptr;
	ret = mp4_demux_open(kOutPath, &mp4);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mp4);

	int trackCount = mp4_demux_get_track_count(mp4);
	CU_ASSERT(trackCount >= 2);

	bool foundVmetaTrack = false;
	for (int i = 0; i < trackCount; i++) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mp4, i, &tk);
		if (ret < 0)
			continue;
		if (tk.type != MP4_TRACK_TYPE_METADATA)
			continue;
		if (tk.mime_format == nullptr)
			continue;
		if (strcmp(tk.mime_format, VMETA_FRAME_PROTO_MIME_TYPE) != 0)
			continue;
		foundVmetaTrack = true;
		CU_ASSERT(tk.sample_count >= kFrameCount);
		break;
	}
	CU_ASSERT_TRUE(foundVmetaTrack);

	mp4_demux_close(mp4);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* IsobmffRecordMuxer's two periodic writer-thread timers -- mThreadSyncTimer
 * (mRecovery.mSyncPeriodMs, IsobmffRecordMuxer::syncCb() ->
 * internalSync(false)) and mThreadTablesSyncTimer (mTablesSyncPeriodMs,
 * IsobmffRecordMuxer::tablesSyncCb() -> internalSync(true)) -- are only ever
 * created in onWriterLoopInit() when the corresponding period is non-zero
 * *at muxer creation time* (pdraw_muxer_record_isobmff.cpp). Every other
 * muxer test in this file uses a default-constructed pdraw_muxer_params
 * (both periods 0), so neither onWriterLoopInit()'s two "!= 0" branches nor
 * syncCb()/tablesSyncCb() themselves are ever exercised: confirmed via gcov,
 * both showed 0 executions. (testCxxDynParamsRoundtrip in test_api_muxer.cpp
 * only exercises internalSetDynParams()'s *own*, separate timer-creation
 * branch, reached via setDynParams() after creation -- not this one.) */
static void testCxxMuxerRecoverySyncTimersFireWithoutError()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kOutPath =
		"/tmp/pdraw_test_pipeline_muxer_recovery_sync.mp4";
	(void)remove(kOutPath);

	struct pdraw_muxer_params muxerParams = {};
	muxerParams.recovery.sync_period_ms = 50;
	muxerParams.tables_sync_period_ms = 50;
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kOutPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* Let both periodic timers -- running on the muxer's own writer
	 * thread, independent of this test's pomp loop -- fire several times
	 * before any media even exists: mMux is already open at this point
	 * (RecordMuxer::internalStart() creates it via
	 * onBeforeAddMuxerMedias() before spawning the writer thread), so
	 * internalSync() has real work to do. The predicate never turns
	 * true; pumpUntil() is only used here as a wall-clock wait (it still
	 * services this test's own loop instead of blocking outright), long
	 * enough for several 50 ms periods to elapse on the writer thread. */
	(void)loop.pumpUntil([]() { return false; }, 300);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());

	/* Confirm the muxer still works normally afterward: record a real
	 * raw video track through it, same fixture as
	 * testCxxMuxerRecordsRawVideoTrack above. */
	const unsigned int kWidth = 16;
	const unsigned int kHeight = 16;
	struct vdef_dim resolution = {kWidth, kHeight};
	size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
	ret = vdef_calc_raw_frame_size(&vdef_raw8,
				       &resolution,
				       planeStride,
				       nullptr,
				       nullptr,
				       nullptr,
				       planeSize,
				       nullptr);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_raw8);
	CU_ASSERT_EQUAL_FATAL(planeCount, 1u);

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
		 "pdraw_test_pipeline_muxer_recovery_sync");

	IPdraw::IRawVideoSource *source = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	for (unsigned int i = 0; i < 3; i++) {
		struct vdef_raw_frame frameInfo = {};
		frameInfo.format = vdef_raw8;
		frameInfo.info.timescale = 1000000;
		frameInfo.info.timestamp = i * 33333;
		frameInfo.info.index = i;
		frameInfo.info.resolution.width = kWidth;
		frameInfo.info.resolution.height = kHeight;
		frameInfo.info.bit_depth = 8;
		for (unsigned int p = 0; p < planeCount; p++)
			frameInfo.plane_stride[p] = planeStride[p];

		struct mbuf_raw_video_frame *frame = nullptr;
		ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		for (unsigned int p = 0; p < planeCount; p++) {
			struct mbuf_mem *mem = nullptr;
			ret = mbuf_mem_generic_new(planeSize[p], &mem);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			uint8_t *data = nullptr;
			size_t capacity = 0;
			ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			memset(data, 0x80, capacity);
			ret = mbuf_raw_video_frame_set_plane(
				frame, p, mem, 0, planeSize[p]);
			CU_ASSERT_EQUAL_FATAL(ret, 0);
			mbuf_mem_unref(mem);
		}

		ret = mbuf_raw_video_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool gotFrames = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats stats = {};
			int r = muxer->getStats(&stats);
			return (r == 0) && (stats.record.raw_video_frames >= 3);
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	/* One more real-time wait spanning several more periods, with real
	 * media now tracked, before closing -- covers internalSync()'s
	 * mMetadataChanged branch too (session metadata is set as soon as a
	 * media with sessionMeta is added). */
	(void)loop.pumpUntil([]() { return false; }, 120);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	struct stat st = {};
	CU_ASSERT_EQUAL(stat(kOutPath, &st), 0);
	CU_ASSERT(st.st_size > 0);
	(void)remove(kOutPath);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* ── C-wrapper listener callback tests (C API) ── */

struct MuxerCSessionCbState {
	int stopRespCount = 0;
};

static void
muxer_test_session_stop_resp_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<MuxerCSessionCbState *>(ud)->stopRespCount++;
}

static bool make_pdraw_for_muxer_test(TestPompLoop &loop,
				      MuxerCSessionCbState &state,
				      struct pdraw **out)
{
	struct pdraw_cbs cbs = {};
	cbs.stop_resp = muxer_test_session_stop_resp_cb;
	int ret = pdraw_new(loop.raw(), &cbs, &state, out);
	return ret == 0 && *out != nullptr;
}

struct MuxerCloseCbState {
	int closeRespCount = 0;
	int closeRespStatus = -1;
};

static void muxer_close_resp_cb(struct pdraw * /*p*/,
				struct pdraw_muxer * /*m*/,
				int status,
				void *ud)
{
	auto *s = static_cast<MuxerCloseCbState *>(ud);
	s->closeRespStatus = status;
	s->closeRespCount++;
}

/* Verify that pdraw_muxer_close() causes the
 * PdrawMuxerListener::muxerCloseResponse
 * shim in pdraw_wrapper.cpp to fire the C close_resp callback. */
static void testCMuxerListenerCloseResponse()
{
	TestPompLoop loop;
	MuxerCSessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw_for_muxer_test(loop, sessState, &p));

	struct pdraw_muxer_params params = {};
	MuxerCloseCbState muxState;
	struct pdraw_muxer_cbs cbs = {};
	cbs.close_resp = muxer_close_resp_cb;
	struct pdraw_muxer *mux = nullptr;
	int ret = pdraw_muxer_new(p,
				  "/tmp/test_pdraw_muxer_close_resp.mp4",
				  &params,
				  &cbs,
				  &muxState,
				  &mux);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mux);

	ret = pdraw_muxer_close(p, mux);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&muxState]() { return muxState.closeRespCount >= 1; }, 15000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(muxState.closeRespStatus, 0);

	pdraw_muxer_destroy(p, mux);
	pdraw_destroy(p);
	(void)remove("/tmp/test_pdraw_muxer_close_resp.mp4");
}


static void testCxxMuxerDynParams()
{
	/* Covers RecordMuxer::getDynParams() (lines 430, 432, 434) and
	 * RecordMuxer::setDynParams() — the "not ready" branch (lines 406-408)
	 * is hit by pumpUntil predicate calls before the writer thread opens
	 * the MP4 file (mIsMuxerReady false); the normal path (lines 411-424)
	 * is hit once the file is open (predicate returns true). */
	static const char *const kPath = "/tmp/pdraw_test_muxer_dynparams.mp4";

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	(void)remove(kPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);

	struct pdraw_muxer_dyn_params dynParams = {};

	/* Null-pointer guard → -EINVAL (getDynParams line 430). */
	ret = muxer->getDynParams(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Poll until writer thread opens the MP4 file (mIsMuxerReady = true).
	 * Pre-ready calls return -EPROTO (setDynParams lines 406-408); the
	 * successful call returns 0 (lines 411-424). */
	bool muxerReady = loop.pumpUntil(
		[muxer, &dynParams]() {
			return muxer->setDynParams(&dynParams) == 0;
		},
		5000);
	CU_ASSERT_TRUE(muxerReady);

	/* Normal path → 0 (getDynParams lines 432, 434). */
	ret = muxer->getDynParams(&dynParams);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);
	(void)remove(kPath);
}


/* Covers addMuxerMedia lines 659 (trackName = params->track_name) and
 * 668 (finalParams.track_name = trackName.c_str()), plus
 * internalAddMuxerMedia line 703 (cfg.name = params->track_name) in
 * pdraw_muxer_record.cpp — all three are only reached when addMedia() is
 * called with a non-null track_name. The ADD_TRACK task is processed by the
 * writer thread before STOP_THREAD (FIFO queue), so line 703 fires even
 * though no frames are pushed. */
static void testCxxMuxerExplicitTrackName()
{
	static const char *const kPath = "/tmp/pdraw_test_muxer_track_name.mp4";

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_raw8;
	sourceParams.video.raw.info.resolution.width = 16;
	sourceParams.video.raw.info.resolution.height = 16;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia);

	(void)remove(kPath);
	struct pdraw_muxer_params muxerParams = {};
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	mediaParams.track_name = "custom_track";
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL(ret, 0);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner.reset();
	sourceOwner.reset();

	(void)remove(kPath);
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Covers getFreeSpace lines 567-569 (statvfs failure path) and
 * ensureFreeSpace lines 597-598 (getFreeSpace error propagation) in
 * pdraw_muxer_record.cpp.  Achieved by creating a muxer with a non-zero
 * free_space_limit on a path whose parent directory does not exist:
 * internalStart() calls ensureFreeSpace(0) which calls getFreeSpace() on
 * the missing directory; statvfs() returns ENOENT, and createMuxer() fails
 * synchronously before the writer thread is ever started. */
static void testCxxMuxerFreeSpaceCheckOnNonexistentPath()
{
	/* UUID-like component makes collision with a real directory effectively
	 * impossible; no cleanup needed since createMuxer() never opens the
	 * file (it fails in ensureFreeSpace before onBeforeAddMuxerMedias). */
	static const char *const kPath =
		"/tmp/pdraw_9f6c2b7e_nonexistent_dir/test.mp4";

	TestPompLoop loop;
	CodedMediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_muxer_params muxerParams = {};
	muxerParams.free_space_limit = 1; /* non-zero: bypasses the limit==0
					     fast path so getFreeSpace is
					     called */
	PipelineMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT(ret < 0);
	CU_ASSERT_PTR_NULL(muxer);

	stopSessionAndWait(&loop, session, &mediaListener);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_muxer_record_isobmff[] = {
	{FN("testCxxMuxerRecordsRealCodedVideoH264"),
	 testCxxMuxerRecordsRealCodedVideoH264},
	{FN("testCxxMuxerRecordsRealCodedVideoH265"),
	 testCxxMuxerRecordsRealCodedVideoH265},
	{FN("testCxxMuxerRecordsRealCodedVideoWithChapters"),
	 testCxxMuxerRecordsRealCodedVideoWithChapters},
	{FN("testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips"),
	 testCxxMuxerRecordsVideoAndAudioTracksThenRoundtrips},
	{FN("testCxxMuxerRecordsRawVideoTrack"),
	 testCxxMuxerRecordsRawVideoTrack},
	{FN("testCxxMuxerRecordsRawVideoTrackRollbackAndMetadata"),
	 testCxxMuxerRecordsRawVideoTrackRollbackAndMetadata},
	{FN("testCxxMuxerRecordsCodedVideoTrackRollbackAndMetadata"),
	 testCxxMuxerRecordsCodedVideoTrackRollbackAndMetadata},
	{FN("testCxxMuxerRecordsAudioTrackRollback"),
	 testCxxMuxerRecordsAudioTrackRollback},
	{FN("testCxxMuxerRawVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure"),
	 testCxxMuxerRawVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure},
	{FN("testCxxMuxerCodedVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure"),
	 testCxxMuxerCodedVideoTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure},
	{FN("testCxxMuxerAudioTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure"),
	 testCxxMuxerAudioTrackReportsUnrecoverableErrorOnFreeSpaceCheckFailure},
	{FN("testCxxMuxerReportsUnrecoverableErrorOnStartFreeSpaceCheckFailure"),
	 testCxxMuxerReportsUnrecoverableErrorOnStartFreeSpaceCheckFailure},
	{FN("testCxxMuxerSessionMetaUpdatePropagates"),
	 testCxxMuxerSessionMetaUpdatePropagates},
	{FN("testCxxMuxerSessionMetaWriteMediaCbPersistsMultipleFields"),
	 testCxxMuxerSessionMetaWriteMediaCbPersistsMultipleFields},
	{FN("testCxxMuxerSessionMetaWriteMediaCbDistinctFieldsSurviveMerge"),
	 testCxxMuxerSessionMetaWriteMediaCbDistinctFieldsSurviveMerge},
	{FN("testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack"),
	 testCxxMuxerRawVideoVmetaFrameCreatesMetadataTrack},
	{FN("testCxxMuxerRawVideoVmetaFrameWritesMetadataSample"),
	 testCxxMuxerRawVideoVmetaFrameWritesMetadataSample},
	{FN("testCxxMuxerRawVideoVmetaFrameWritesMetadataSampleProto"),
	 testCxxMuxerRawVideoVmetaFrameWritesMetadataSampleProto},
	{FN("testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample"),
	 testCxxMuxerCodedVideoVmetaFrameWritesMetadataSample},
	{FN("testCxxMuxerCodedVideoVmetaFrameWritesMetadataSampleProto"),
	 testCxxMuxerCodedVideoVmetaFrameWritesMetadataSampleProto},
	{FN("testCxxMuxerRecoverySyncTimersFireWithoutError"),
	 testCxxMuxerRecoverySyncTimersFireWithoutError},
	{FN("testCMuxerListenerCloseResponse"),
	 testCMuxerListenerCloseResponse},

	{FN("testCxxMuxerDynParams"), testCxxMuxerDynParams},
	{FN("testCxxMuxerExplicitTrackName"), testCxxMuxerExplicitTrackName},
	{FN("testCxxMuxerFreeSpaceCheckOnNonexistentPath"),
	 testCxxMuxerFreeSpaceCheckOnNonexistentPath},
	CU_TEST_INFO_NULL,
};
