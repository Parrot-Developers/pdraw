/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — photo (PNG/JFIF/DNG) record muxer, one file written
 * per pushed frame (Tier B, self-contained fixture)
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

/* No demuxer/decoder involved at all in this file: each test builds a
 * self-contained hand-built ICodedVideoSource (PNG/JFIF/DNG format) feeding
 * a photo record muxer directly, one file written per pushed frame. No
 * suite init/cleanup is registered for this file (see test_main.c: NULL,
 * NULL).
 *
 * Split out of the original test_pipeline_muxer_record.cpp (which grew past
 * 5000 lines) alongside its companion test_pipeline_muxer_record_isobmff.cpp,
 * which covers the video/audio-track ISOBMFF (.mp4) record muxer instead. */

#define ULOG_TAG pdraw_test_pipeline_muxer_record_photo
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

/* Only needed to reach the protected RecordMuxer::createMedia() overrides
 * directly (see testCxxPhotoMuxer{Jfif,Png}CreateMediaRejectsRawVideo /
 * testCxxPhotoMuxerDngCreateMediaRejectsCodedVideo below) -- same trick as
 * test_pipeline_demuxer_stream_mux.cpp / test_pipeline_demuxer_stream.cpp. */
#define private public
#define protected public
#include "pdraw_muxer.hpp"
#include "pdraw_muxer_record.hpp"
#include "pdraw_muxer_record_dng.hpp"
#include "pdraw_muxer_record_jfif.hpp"
#include "pdraw_muxer_record_media.hpp"
#include "pdraw_muxer_record_photo.hpp"
#include "pdraw_muxer_record_photo_media.hpp"
#include "pdraw_muxer_record_png.hpp"
#undef protected
#undef private

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <atomic>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


namespace {

/* ── Photo (PNG) muxer: the one path where onMuxerMediaReady() actually
 * fires ────────────────────────────────────────────────────────────────
 *
 * Unlike the video/ISOBMFF muxer above (which writes continuously into one
 * file, never calling onMuxerMediaReady()), a photo record muxer
 * (JFIF/DNG/PNG) writes one file per pushed frame and DOES call both
 * onMuxerMediaReady() and onMuxerMediaSaved() for each one (confirmed by
 * reading PhotoRecordMuxer::saveToDiskIov() in
 * pdraw_muxer_record_photo.cpp). PNG is used here specifically because,
 * unlike JFIF (needs libjfif) and DNG (needs libdng-parrot), it has no
 * optional/conditional build dependency (see the unconditional
 * "PngRecordMuxer" branch in MuxerWrapper::MuxerWrapper(),
 * pdraw_muxer.cpp) -- lower risk of "not supported" in an arbitrary build
 * configuration.
 *
 * No demuxer/decoder involved at all: a self-contained, hand-built
 * ICodedVideoSource (format vdef_png) plays the same role
 * test_pipeline_sourcesink_coded.cpp's source did, except its single output
 * channel here goes straight to the muxer's input media (Session::
 * PipelineFactory::addMediaToMuxer() wires source->addOutputChannel()
 * synchronously -- confirmed by reading pdraw_session.cpp -- so there is no
 * source/sink-style "wait for a consumer to exist before pushing frames"
 * ordering concern here: the channel is already wired by the time
 * muxer->addMedia() returns 0).
 *
 * PngMuxerMedia::internalSerialize() (pdraw_muxer_record_png_media.cpp)
 * does a plain byte copy of the packed frame buffer with no PNG parsing
 * or validation at all, so the written file is expected to be byte-for-
 * byte identical to the input image -- checked directly. */

/* The smallest possible valid PNG: an 8-bit RGBA 1x1 transparent pixel,
 * signature + IHDR + IDAT + IEND, 67 bytes total. Content/validity doesn't
 * actually matter to this test (the muxer never parses it), but using a
 * real one is just as easy and more representative. */
static const uint8_t kMinimalPng[] = {
	0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
	0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
	0x08, 0x06, 0x00, 0x00, 0x00, 0x1f, 0x15, 0xc4, 0x89, 0x00, 0x00, 0x00,
	0x0a, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0x00, 0x01, 0x00, 0x00,
	0x05, 0x00, 0x01, 0x0d, 0x0a, 0x2d, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x49,
	0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82,
};


/* Session-wide listener for the hand-built PNG source's own media, same
 * elementUserData-filtering rationale as the source/sink tests'
 * SourceMediaListener (test_pipeline_sourcesink_coded.cpp and siblings). */
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


/* Unlike PipelineMuxerListener above, onMuxerMediaReady() is the whole
 * point here, so it is tracked for real instead of being a documenting
 * no-op. mMutex guards the captured strings: onMuxerMediaReady()'s doc
 * comment does not state which thread calls it (unlike onMuxerMediaSaved(),
 * explicitly documented as "called on the PDrAW loop thread"), so this
 * plays it safe the same way PipelineMuxerListener does with std::atomic. */
class PhotoMuxerListener : public IPdraw::IMuxer::Listener {
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
			       const char *mediaPath,
			       const struct iovec *iov,
			       int iovcnt) override
	{
		size_t total = 0;
		for (int i = 0; i < iovcnt; i++)
			total += iov[i].iov_len;
		{
			std::lock_guard<std::mutex> lock(mMutex);
			mReadyPath = (mediaPath != nullptr) ? mediaPath : "";
			mReadyIovCount = iovcnt;
			mReadyTotalLen = total;
		}
		mGotMediaReady.store(true);
	}

	void onMuxerMediaSaved(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char *mediaPath) override
	{
		{
			std::lock_guard<std::mutex> lock(mMutex);
			mSavedPath = (mediaPath != nullptr) ? mediaPath : "";
		}
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

	std::string readyPath()
	{
		std::lock_guard<std::mutex> lock(mMutex);
		return mReadyPath;
	}

	std::string savedPath()
	{
		std::lock_guard<std::mutex> lock(mMutex);
		return mSavedPath;
	}

	std::mutex mMutex;
	std::string mReadyPath;
	int mReadyIovCount = 0;
	size_t mReadyTotalLen = 0;
	std::string mSavedPath;
	std::atomic<bool> mGotMediaReady{false};
	std::atomic<bool> mGotMediaSaved{false};
	std::atomic<bool> mGotUnrecoverableError{false};
	std::atomic<bool> mGotCloseResponse{false};
	std::atomic<int> mCloseStatus{0};
};


/* Tracks ICodedVideoSource::flush()/drain() completion. Unlike
 * ICodedVideoSink (see test_pipeline_sourcesink_coded.cpp's
 * QueueDrainingCodedVideoSinkListener), a Muxer sink completes flush/drain
 * on its own -- Muxer::onChannelFlush()/onChannelDrain() (pdraw_muxer.cpp)
 * flush their own queue and call asyncCompleteFlush() internally, no
 * application-level queueDrained()-style acknowledgement needed. So only
 * the source side needs a non-stub listener here. */
class DrainTrackingCodedVideoSourceListener
		: public IPdraw::ICodedVideoSource::Listener {
public:
	void
	onCodedVideoSourceFlushed(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*src*/) override
	{
		mGotFlushed = true;
	}

	void
	onCodedVideoSourceDrained(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*src*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


static void testCxxPhotoMuxerWritesOnePngPerFrame()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_png;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* addInputMedia()/addOutputChannel() both happen synchronously inside
	 * addMedia() (see the file-level comment): safe to push a frame right
	 * after this returns, no "wait for wiring" step needed. */
	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Build one coded video frame wrapping the whole PNG buffer as a
	 * single opaque segment (PNG has no NALU concept; the vdef_nalu
	 * h264/h265 union fields are simply left unused/zeroed). */
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_png;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = 1;
	frameInfo.info.resolution.height = 1;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalPng), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalPng));
	memcpy(data, kMinimalPng, sizeof(kMinimalPng));

	struct vdef_nalu nalu = {};
	nalu.size = sizeof(kMinimalPng);
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem); /* the frame holds its own ref via add_nalu */

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);

	/* onMuxerMediaReady() -- the very thing this test exists to exercise
	 * -- must have fired too, with a plausible payload, before the file
	 * was reported saved. */
	CU_ASSERT_TRUE(muxerListener.mGotMediaReady.load());
	CU_ASSERT_EQUAL(muxerListener.mReadyIovCount, 1);
	CU_ASSERT_EQUAL(muxerListener.mReadyTotalLen, sizeof(kMinimalPng));
	CU_ASSERT_STRING_EQUAL(muxerListener.readyPath().c_str(),
			       "/tmp/pdraw_test_pipeline_muxer_photo_0001.png");
	CU_ASSERT_STRING_EQUAL(muxerListener.savedPath().c_str(),
			       "/tmp/pdraw_test_pipeline_muxer_photo_0001.png");

	auto muxerOwner5 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner5 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner5.reset();
	sourceOwner5.reset();

	/* The whole point: PngMuxerMedia::internalSerialize() does a plain
	 * byte copy, so the written file must be byte-for-byte identical to
	 * the pushed PNG buffer. */
	std::string savedPath = muxerListener.savedPath();
	struct stat st = {};
	CU_ASSERT_EQUAL_FATAL(stat(savedPath.c_str(), &st), 0);
	CU_ASSERT_EQUAL_FATAL(static_cast<size_t>(st.st_size),
			      sizeof(kMinimalPng));
	FILE *f = fopen(savedPath.c_str(), "rb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	uint8_t readBack[sizeof(kMinimalPng)];
	size_t readCount = fread(readBack, 1, sizeof(readBack), f);
	fclose(f);
	CU_ASSERT_EQUAL(readCount, sizeof(kMinimalPng));
	CU_ASSERT_EQUAL(memcmp(readBack, kMinimalPng, sizeof(kMinimalPng)), 0);
	(void)remove(savedPath.c_str());

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* PngRecordMuxer::createMedia() (pdraw_muxer_record_png.cpp) only accepts
 * Media::Type::CODED_VIDEO -- any other media type (a raw video media here)
 * falls into the `default:` branch and returns nullptr. Reached directly by
 * calling the protected createMedia() override (exposed via the
 * private/protected-as-public include trick above) instead of going through
 * the async writer-thread path (RecordMuxer::addInputMedia() ->
 * postTask(ADD_TRACK) -> internalAddMuxerMedia()), whose failure is only
 * logged, never surfaced to any IMuxer::Listener callback -- so there would
 * be no observable signal to assert on through the public API alone. No
 * source/frame is ever created: this only exercises the createMedia()
 * rejection path itself. */
static void testCxxPhotoMuxerPngCreateMediaRejectsRawVideo()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_png_reject_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *pngMuxer = static_cast<PngRecordMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(pngMuxer);

	MuxerMediaConfig cfg = {};
	cfg.type = Media::Type::RAW_VIDEO;
	std::unique_ptr<RecordMuxer::MuxerMedia> track =
		pngMuxer->createMedia(cfg);
	CU_ASSERT_PTR_NULL(track.get());

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* PhotoRecordMuxer::PhotoMuxerRawVideoMedia::internalSetupFormat() is a
 * no-op default hook (always returns 0) with no live production caller: the
 * only concrete subclass, DngRecordMuxer::DngMuxerMedia, overrides it (and
 * is only built with BUILD_LIBDNG_PARROT). Exercise the default body
 * directly via a minimal test-only subclass that leaves it unoverridden. */
class MinimalRawVideoMedia : public PhotoRecordMuxer::PhotoMuxerRawVideoMedia {
public:
	using PhotoMuxerRawVideoMedia::PhotoMuxerRawVideoMedia;

protected:
	int setup(const struct pdraw_media_info *mediaInfo,
		  const struct pdraw_muxer_media_params *params) override
	{
		return -ENOSYS;
	}

	int internalSerialize(const uint8_t *buf,
			      size_t len,
			      std::vector<struct iovec> &iov,
			      uint8_t **headerBuf) override
	{
		return -ENOSYS;
	}

	const char *internalGetMimeType() const override
	{
		return "application/octet-stream";
	}
};


static void testCxxPhotoMuxerRawVideoInternalSetupFormatDefaultReturnsZero()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_setupfmt_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *pngMuxer = static_cast<PngRecordMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(pngMuxer);

	MuxerMediaConfig cfg = {};
	cfg.type = Media::Type::RAW_VIDEO;
	MinimalRawVideoMedia media(pngMuxer, cfg);

	struct vdef_raw_frame info = {};
	CU_ASSERT_EQUAL(media.internalSetupFormat(&info), 0);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Verify that PhotoRecordMuxer::getThreadName() returns the expected name. */
static void testCxxPhotoMuxerGetThreadName()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_get_thread_name_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *pngMuxer = static_cast<PngRecordMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(pngMuxer);

	CU_ASSERT_STRING_EQUAL(pngMuxer->PhotoRecordMuxer::getThreadName(),
			       "pdraw_recmx_pho");

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Muxer::addChapter()/Muxer::forceSync() (pdraw_muxer.cpp) are plain -ENOSYS
 * stubs meant to be overridden by muxer subclasses that support them.
 * IsobmffRecordMuxer overrides both, but RecordMuxer (the shared base of
 * every record muxer, including the photo one) only overrides setThumbnail/
 * setFileMetadata/setDynParams/getDynParams/getStats -- addChapter/
 * forceSync fall all the way through to the base Muxer's stub for a photo
 * muxer. No frame ever needs to be pushed: this only exercises the two
 * ENOSYS return paths themselves. */
static void testCxxPhotoMuxerAddChapterAndForceSyncReturnEnosys()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_png;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_enosys");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_enosys_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_EQUAL(muxer->addChapter(0, "chapter"), -ENOSYS);
	CU_ASSERT_EQUAL(muxer->forceSync(), -ENOSYS);

	/* Test dynamic parameters */
	struct pdraw_muxer_dyn_params dynParams = {};
	ret = muxer->getDynParams(&dynParams);
	CU_ASSERT_EQUAL(ret, 0);

	/* NULL and invalid checks */
	CU_ASSERT_EQUAL(muxer->getDynParams(nullptr), -EINVAL);
	CU_ASSERT_EQUAL(muxer->setDynParams(nullptr), -EINVAL);

	/* setDynParams returns 0 synchronously even with invalid pattern as it
	 * posts to thread */
	struct pdraw_muxer_dyn_params invalidDyn = {};
	invalidDyn.file_name_pattern = nullptr;
	CU_ASSERT_EQUAL(muxer->setDynParams(&invalidDyn), 0);

	struct pdraw_muxer_dyn_params setDyn = {};
	setDyn.file_name_pattern = "/tmp/new_pattern_%04u.png";
	setDyn.next_file_index = 42;
	ret = muxer->setDynParams(&setDyn);
	CU_ASSERT_EQUAL(ret, 0);

	/* Run loop to allow setDynParams command to execute asynchronously */
	loop.runOnce();

	struct pdraw_muxer_dyn_params checkDyn = {};
	ret = muxer->getDynParams(&checkDyn);
	CU_ASSERT_EQUAL(ret, 0);

	/* Test thumbnail */
	uint8_t thumbData[4] = {0};
	ret = muxer->setThumbnail(
		PDRAW_MUXER_THUMBNAIL_TYPE_JPEG, thumbData, sizeof(thumbData));
	CU_ASSERT_EQUAL(ret, 0);

	/* Test file metadata (unsupported but returns 0 synchronously when
	 * params are valid) */
	CU_ASSERT_EQUAL(
		muxer->setFileMetadata(nullptr, thumbData, sizeof(thumbData)),
		-EINVAL);

	struct pdraw_muxer_metadata_params metaParams = {};
	metaParams.type = PDRAW_MUXER_METADATA_TYPE_DNG_LSC;
	ret = muxer->setFileMetadata(&metaParams, thumbData, sizeof(thumbData));
	CU_ASSERT_EQUAL(ret, 0);

	/* Run loop to let the unsupported metadata command run */
	loop.runOnce();

	auto muxerOwner6 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner6 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner6.reset();
	sourceOwner6.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* RecordMuxer::ensureFreeSpace() (pdraw_muxer_record.cpp) bypasses its own
 * check entirely when pdraw_muxer_params::free_space_limit == 0 (the value
 * every other test in this file leaves it at); setting it to an absurdly
 * high value here forces the very first ensureFreeSpace() call in
 * PhotoRecordMuxer::saveToDiskIov() to fail with -ENOSPC, which calls
 * Muxer::onUnrecoverableError() (pdraw_muxer.cpp) -- otherwise unreachable
 * from any test in this suite, since every other muxer test only exercises
 * the success path. onUnrecoverableError() dispatches through an idle
 * callback to the IMuxer::Listener, matching the doc comment on
 * onMuxerUnrecoverableError(): "the close() function must be called [...]
 * prior to destroying the muxer" -- so this still ends with close() + wait,
 * like every other muxer test, instead of destroying the muxer directly. */
static void
testCxxPhotoMuxerReportsUnrecoverableErrorOnFreeSpaceCheckFailure(void)
{
	/* This test's whole point is asserting that this exact path is never
	 * created (ensureFreeSpace() failure makes saveToDiskIov() return
	 * before ever reaching writeToFileIov(), see
	 * pdraw_muxer_record_photo.cpp). The name is a fixed literal, not
	 * randomized per run, so a stale file left behind under /tmp by an
	 * unrelated earlier run would make that final assertion pass or fail
	 * for the wrong reason. Clean up defensively before running. */
	static const char *kExpectedPath =
		"/tmp/pdraw_test_pipeline_muxer_photo_freespace_0001.png";
	(void)remove(kExpectedPath);

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_png;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_freespace");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_freespace_%04u.png";
	/* 1 MiB synthetic frame — content irrelevant, muxer never parses it. */
	static const size_t kFrameSize = 1u << 20;
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	/* free_space_limit = current_free − kFrameSize/2.
	 * Startup guard (spaceNeeded=0) passes as long as disk shrinks by
	 * less than 512 KiB between this statvfs() and internalStart()'s
	 * re-measurement — safe on any sane CI. First frame write always
	 * triggers onUnrecoverableError() because:
	 *   mFreeSpaceLeft (≈ current_free) < free_space_limit + kFrameSize
	 *   ⟺ 0 < kFrameSize - kFrameSize/2 = kFrameSize/2  (always true). */
	struct statvfs vfsStats = {};
	CU_ASSERT_EQUAL_FATAL(statvfs("/tmp", &vfsStats), 0);
	muxerParams.free_space_limit =
		(size_t)vfsStats.f_bavail * vfsStats.f_bsize - kFrameSize / 2;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_png;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = 1;
	frameInfo.info.resolution.height = 1;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(kFrameSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = kFrameSize;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError.load();
		},
		20000);
	CU_ASSERT_TRUE_FATAL(gotError);

	/* saveToDiskIov() returns early on ensureFreeSpace() failure, before
	 * ever calling notifyMediaReadyIov(): neither "ready" nor "saved"
	 * must have fired. */
	CU_ASSERT_FALSE(muxerListener.mGotMediaReady.load());
	CU_ASSERT_FALSE(muxerListener.mGotMediaSaved.load());

	auto muxerOwner7 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner7 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	muxerOwner7.reset();
	sourceOwner7.reset();

	/* No file should ever have been created. */
	struct stat st = {};
	CU_ASSERT_NOT_EQUAL(stat(kExpectedPath, &st), 0);

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* ── Muxer as a Sink for ICodedVideoSource::flush()/drain() ──────────────
 * Neither is ever exercised anywhere else in this suite (the demuxer-fed
 * tests have no ICodedVideoSource to call flush()/drain() on). Connecting
 * a source directly to a photo muxer -- exactly like
 * testCxxPhotoMuxerWritesOnePngPerFrame above, minus the frame-content
 * assertions -- and calling source->flush()/drain() exercises
 * Muxer::onChannelFlush()/onChannelDrain() and asyncCompleteFlush()/
 * completeFlush()/callCompleteFlush() (pdraw_muxer.cpp), plus
 * RecordMuxer::onChannelFlush()/onChannelDrain()/internalFlush()
 * (pdraw_muxer_record.cpp), which delegate to the base class either way
 * (confirmed by reading both call paths, sync and writer-thread-async). ── */

static void runPhotoMuxerSourceFlushOrDrain(bool drain)
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_png;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_flushdrain");

	DrainTrackingCodedVideoSourceListener sourceListener;
	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_flushdrain_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_png;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = 1;
	frameInfo.info.resolution.height = 1;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalPng), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalPng));
	memcpy(data, kMinimalPng, sizeof(kMinimalPng));

	struct vdef_nalu nalu = {};
	nalu.size = sizeof(kMinimalPng);
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);
	(void)remove(
		"/tmp/pdraw_test_pipeline_muxer_photo_flushdrain_0001.png");

	if (drain) {
		ret = source->drain();
		CU_ASSERT_EQUAL(ret, 0);
		bool gotDrained = loop.pumpUntil(
			[&sourceListener]() {
				return sourceListener.mGotDrained;
			},
			15000);
		CU_ASSERT_TRUE(gotDrained);
	} else {
		ret = source->flush();
		CU_ASSERT_EQUAL(ret, 0);
		bool gotFlushed = loop.pumpUntil(
			[&sourceListener]() {
				return sourceListener.mGotFlushed;
			},
			15000);
		CU_ASSERT_TRUE(gotFlushed);
	}

	auto muxerOwner8 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner8 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner8.reset();
	sourceOwner8.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxPhotoMuxerSourceFlushCompletesAutomatically()
{
	runPhotoMuxerSourceFlushOrDrain(false);
}


static void testCxxPhotoMuxerSourceDrainCompletesAutomatically()
{
	runPhotoMuxerSourceFlushOrDrain(true);
}


#ifdef BUILD_LIBJFIF

/* ── JFIF (JPEG) muxer: same one-file-per-frame mechanics as PNG above,
 * but NOT a pure byte copy -- JfifRecordMuxer::JfifMuxerMedia::
 * internalSerialize() calls jfif_mux_serialize_iov() (pdraw_muxer_record_
 * jfif_media.cpp / libjfif's jfif_mux.c), which:
 *  - builds a header of just the 2-byte SOI marker (0xFFD8) when no EXIF/
 *    XMP/COM metadata was added (this test adds none),
 *  - strips the input's own leading SOI if present (offset_in = 2),
 *  - appends an EOI marker (0xFFD9) only if the input doesn't already end
 *    with one.
 * With an input that already starts with SOI and ends with EOI, and no
 * metadata added, header + stripped-payload + (no extra EOI) reassembles
 * to be byte-for-byte identical to the original input -- confirmed by
 * reading jfif_mux_serialize_iov()'s implementation, not assumed. The
 * input bytes don't need to be a structurally valid, decodable JPEG at
 * all: this function never parses SOF/DHT/SOS, it only special-cases the
 * first 2 and last 2 bytes. */

static const uint8_t kMinimalJpeg[] = {
	0xff,
	0xd8, /* SOI */
	0x00,
	0x01,
	0x02,
	0x03, /* arbitrary "scan data" */
	0xff,
	0xd9, /* EOI */
};

static void testCxxPhotoMuxerWritesOneJpegPerFrame()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_jpeg_jfif;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_jfif");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_jfif_%04u.jpg";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_jpeg_jfif;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = 1;
	frameInfo.info.resolution.height = 1;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalJpeg), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalJpeg));
	memcpy(data, kMinimalJpeg, sizeof(kMinimalJpeg));

	struct vdef_nalu nalu = {};
	nalu.size = sizeof(kMinimalJpeg);
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);
	CU_ASSERT_TRUE(muxerListener.mGotMediaReady.load());

	auto muxerOwner9 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner9 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner9.reset();
	sourceOwner9.reset();

	/* NOT byte-identical, unlike PNG: create_xmp_payload() (libjfif's
	 * jfif_xmp.c) unconditionally builds and embeds an APP1 XMP segment
	 * (toolkit string + XML boilerplate + padding) for every frame, even
	 * when no custom field was ever added via addXmp() -- it never
	 * returns NULL. So the output is always strictly larger than the
	 * input. Checked structurally instead: SOI/EOI markers at the very
	 * start/end (the payload itself is relocated verbatim, never
	 * re-encoded), the original scan-data bytes still present somewhere
	 * inside, and the configured XMP toolkit string actually embedded
	 * (confirming a real XMP packet was built, not just a size
	 * coincidence). Found the hard way: an earlier version of this test
	 * wrongly assumed a PNG-like byte-identical roundtrip. */
	std::string savedPath = muxerListener.savedPath();
	struct stat st = {};
	CU_ASSERT_EQUAL_FATAL(stat(savedPath.c_str(), &st), 0);
	CU_ASSERT_FATAL(st.st_size > static_cast<off_t>(sizeof(kMinimalJpeg)));

	FILE *f = fopen(savedPath.c_str(), "rb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	std::vector<uint8_t> content(static_cast<size_t>(st.st_size));
	size_t readCount = fread(content.data(), 1, content.size(), f);
	fclose(f);
	CU_ASSERT_EQUAL_FATAL(readCount, content.size());

	CU_ASSERT_FATAL(content.size() >= 2);
	CU_ASSERT_TRUE(content[0] == 0xff && content[1] == 0xd8);
	CU_ASSERT_TRUE(content[content.size() - 2] == 0xff &&
		       content[content.size() - 1] == 0xd9);

	static const uint8_t kScanData[] = {0x00, 0x01, 0x02, 0x03};
	bool foundScanData = std::search(content.begin(),
					 content.end(),
					 std::begin(kScanData),
					 std::end(kScanData)) != content.end();
	CU_ASSERT_TRUE(foundScanData);

	static const char kToolkit[] = "Pdraw_JfifRecordMuxer";
	bool foundToolkit =
		std::search(content.begin(),
			    content.end(),
			    kToolkit,
			    kToolkit + strlen(kToolkit)) != content.end();
	CU_ASSERT_TRUE(foundToolkit);

	(void)remove(savedPath.c_str());

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* JfifRecordMuxer::internalSetThumbnail() (pdraw_muxer_record_jfif.cpp) is
 * never exercised anywhere else in this file. Distinct scan-data bytes from
 * kMinimalJpeg (0xaa/0xbb/0xcc/0xdd instead of 0x00/0x01/0x02/0x03) let the
 * check below tell the embedded thumbnail apart from the frame's own JPEG
 * payload, both structurally present in the same output file. */
static void testCxxPhotoMuxerJfifSetsThumbnail()
{
	static const uint8_t kThumbnailJpeg[] = {
		0xff,
		0xd8, /* SOI */
		0xaa,
		0xbb,
		0xcc,
		0xdd, /* arbitrary "thumbnail data" */
		0xff,
		0xd9, /* EOI */
	};

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_CODED;
	sourceParams.video.coded.format = vdef_jpeg_jfif;
	sourceParams.video.coded.info.resolution.width = 1;
	sourceParams.video.coded.info.resolution.height = 1;
	sourceParams.video.coded.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_jfif_thumb");

	IPdraw::ICodedVideoSource *source = nullptr;
	int ret = session->createCodedVideoSource(
		&sourceParams, &g_stub_coded_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_jfif_thumb_%04u.jpg";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* "for the next frames": must be set before the frame is pushed below.
	 */
	ret = muxer->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
				  kThumbnailJpeg,
				  sizeof(kThumbnailJpeg));
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_jpeg_jfif;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.resolution.width = 1;
	frameInfo.info.resolution.height = 1;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalJpeg), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalJpeg));
	memcpy(data, kMinimalJpeg, sizeof(kMinimalJpeg));

	struct vdef_nalu nalu = {};
	nalu.size = sizeof(kMinimalJpeg);
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);

	auto muxerOwner10 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner10 = std::unique_ptr<IPdraw::ICodedVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner10.reset();
	sourceOwner10.reset();

	/* Structural check only (same rationale as testCxxPhotoMuxerWritesOne
	 * JpegPerFrame): confirms the thumbnail bytes were actually embedded
	 * in the output file, distinguishable from the frame's own payload by
	 * their distinct scan-data marker. */
	std::string savedPath = muxerListener.savedPath();
	struct stat st = {};
	CU_ASSERT_EQUAL_FATAL(stat(savedPath.c_str(), &st), 0);
	FILE *f = fopen(savedPath.c_str(), "rb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	std::vector<uint8_t> content(static_cast<size_t>(st.st_size));
	size_t readCount = fread(content.data(), 1, content.size(), f);
	fclose(f);
	CU_ASSERT_EQUAL_FATAL(readCount, content.size());

	static const uint8_t kThumbnailMarker[] = {0xaa, 0xbb, 0xcc, 0xdd};
	bool foundThumbnail =
		std::search(content.begin(),
			    content.end(),
			    std::begin(kThumbnailMarker),
			    std::end(kThumbnailMarker)) != content.end();
	CU_ASSERT_TRUE(foundThumbnail);

	(void)remove(savedPath.c_str());

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* Same rationale as testCxxPhotoMuxerPngCreateMediaRejectsRawVideo above:
 * JfifRecordMuxer::createMedia() (pdraw_muxer_record_jfif.cpp) only accepts
 * Media::Type::CODED_VIDEO, so a raw video media falls into `default:` and
 * returns nullptr. */
static void testCxxPhotoMuxerJfifCreateMediaRejectsRawVideo()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_jfif_reject_%04u.jpg";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *jfifMuxer = static_cast<JfifRecordMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(jfifMuxer);

	MuxerMediaConfig cfg = {};
	cfg.type = Media::Type::RAW_VIDEO;
	std::unique_ptr<RecordMuxer::MuxerMedia> track =
		jfifMuxer->createMedia(cfg);
	CU_ASSERT_PTR_NULL(track.get());

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop3 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop3);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}

#endif /* BUILD_LIBJFIF */


#ifdef BUILD_LIBDNG_PARROT

/* ── DNG muxer: unlike PNG/JFIF above, this is NOT a byte-preserving
 * passthrough. DngRecordMuxer::DngMuxerMedia::internalSerialize() calls
 * dng_mux_serialize_iov() (libdng-parrot, built on the Adobe DNG SDK),
 * which builds a genuine TIFF/DNG container (IFDs, tags, etc.) around the
 * raw Bayer pixel data -- there is no way to predict its exact output
 * bytes without reimplementing (or linking) the SDK itself. So this test
 * only checks structural plausibility: the file exists, is non-empty, and
 * starts with a valid TIFF magic (little- or big-endian).
 *
 * DNG muxes *raw* video media (Bayer sensor data), not coded video --
 * confirmed by DngRecordMuxer::createMedia() only accepting
 * Media::Type::RAW_VIDEO -- so this uses IRawVideoSource, mirroring
 * test_pipeline_sourcesink_raw.cpp, not ICodedVideoSource. */

static void testCxxPhotoMuxerWritesOneDngPerFrame()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* Minimal 2x2 8-bit Bayer RGGB frame: 1 byte/pixel, stride 2 bytes,
	 * 4 bytes total. */
	const unsigned int kWidth = 2;
	const unsigned int kHeight = 2;
	const unsigned int kStride = 2;
	const uint8_t kMinimalBayer[kStride * kHeight] = {
		0x10, 0x20, 0x30, 0x40};

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_bayer_rggb;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_dng");

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_dng_%04u.dng";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_bayer_rggb;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = kWidth;
	frameInfo.info.resolution.height = kHeight;
	frameInfo.info.bit_depth = 8;
	frameInfo.plane_stride[0] = kStride;

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalBayer), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalBayer));
	memcpy(data, kMinimalBayer, sizeof(kMinimalBayer));

	ret = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, sizeof(kMinimalBayer));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);
	CU_ASSERT_TRUE(muxerListener.mGotMediaReady.load());

	auto muxerOwner11 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner11 = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner11.reset();
	sourceOwner11.reset();

	/* Structural check only (see the file-level comment): a real DNG SDK
	 * builds the actual TIFF/DNG container, not this test. */
	std::string savedPath = muxerListener.savedPath();
	struct stat st = {};
	CU_ASSERT_EQUAL_FATAL(stat(savedPath.c_str(), &st), 0);
	CU_ASSERT_FATAL(st.st_size > 0);
	FILE *f = fopen(savedPath.c_str(), "rb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	uint8_t magic[4] = {};
	size_t readCount = fread(magic, 1, sizeof(magic), f);
	fclose(f);
	CU_ASSERT_EQUAL_FATAL(readCount, sizeof(magic));
	bool isLittleEndianTiff = (magic[0] == 0x49 && magic[1] == 0x49 &&
				   magic[2] == 0x2a && magic[3] == 0x00);
	bool isBigEndianTiff = (magic[0] == 0x4d && magic[1] == 0x4d &&
				magic[2] == 0x00 && magic[3] == 0x2a);
	CU_ASSERT_TRUE(isLittleEndianTiff || isBigEndianTiff);
	(void)remove(savedPath.c_str());

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* DngRecordMuxer::internalSetThumbnail()/internalSetFileMetadata() (pdraw_
 * muxer_record_dng.cpp) are never exercised anywhere else in this file.
 * Verifying the embedded thumbnail/LSC content would require a full TIFF/DNG
 * IFD parser (see the structural-only rationale on
 * testCxxPhotoMuxerWritesOneDngPerFrame above); the point here is coverage
 * of dng_mux_set_thumbnail()/dng_mux_set_lsc() themselves actually
 * succeeding (ret == 0 below, not -ENOSYS/-EINVAL) with real, correctly-
 * shaped inputs, plus no unrecoverable error reported by the writer thread
 * afterward. */
static void testCxxPhotoMuxerDngSetsThumbnailAndLsc()
{
	/* A small, complete JPEG bitstream, per dng_mux_set_thumbnail()'s doc
	 * comment in libdng.h -- content is irrelevant beyond that. */
	static const uint8_t kThumbnailJpeg[] = {
		0xff,
		0xd8, /* SOI */
		0xaa,
		0xbb,
		0xcc,
		0xdd, /* arbitrary "thumbnail data" */
		0xff,
		0xd9, /* EOI */
	};

	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	const unsigned int kWidth = 2;
	const unsigned int kHeight = 2;
	const unsigned int kStride = 2;
	const uint8_t kMinimalBayer[kStride * kHeight] = {
		0x10, 0x20, 0x30, 0x40};

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_bayer_rggb;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_muxer_photo_dng_thumb");

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_dng_thumb_%04u.dng";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	ret = muxer->addMedia(mediaListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* "for the next frames": both must be set before the frame is pushed
	 * below (see dng_mux_set_thumbnail()/dng_mux_set_lsc()'s doc comments
	 * in libdng.h). */
	ret = muxer->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
				  kThumbnailJpeg,
				  sizeof(kThumbnailJpeg));
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Minimal, structurally valid LSC grid: 2x2, 4 channels (typical
	 * Bayer), 32-bit float gains -- dng_mux_set_lsc() only requires size
	 * == width*height*count*sizeof(float), content is irrelevant. */
	const uint32_t kLscWidth = 2, kLscHeight = 2, kLscCount = 4;
	std::vector<float> lscData(kLscWidth * kLscHeight * kLscCount, 1.0f);
	struct pdraw_muxer_metadata_params metaParams = {};
	metaParams.type = PDRAW_MUXER_METADATA_TYPE_DNG_LSC;
	metaParams.dng_lsc.width = kLscWidth;
	metaParams.dng_lsc.height = kLscHeight;
	metaParams.dng_lsc.count = kLscCount;
	metaParams.dng_lsc.format = VDEF_RAW_PIX_ORDER_RGGB;
	ret = muxer->setFileMetadata(
		&metaParams,
		reinterpret_cast<const uint8_t *>(lscData.data()),
		lscData.size() * sizeof(float));
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_bayer_rggb;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.info.index = 0;
	frameInfo.info.resolution.width = kWidth;
	frameInfo.info.resolution.height = kHeight;
	frameInfo.info.bit_depth = 8;
	frameInfo.plane_stride[0] = kStride;

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalBayer), &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= sizeof(kMinimalBayer));
	memcpy(data, kMinimalBayer, sizeof(kMinimalBayer));

	ret = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, sizeof(kMinimalBayer));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	bool gotSaved = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotMediaSaved.load();
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotSaved);

	auto muxerOwner12 = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner12 = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError.load());
	muxerOwner12.reset();
	sourceOwner12.reset();

	std::string savedPath = muxerListener.savedPath();
	struct stat st = {};
	CU_ASSERT_EQUAL_FATAL(stat(savedPath.c_str(), &st), 0);
	CU_ASSERT_FATAL(st.st_size > 0);
	(void)remove(savedPath.c_str());

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop2);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* mapVdefToDngPhase() (pdraw_muxer_record_dng.cpp:68-82) is a free function
 * (not a class member) switching on vdef_raw_pix_order to build the
 * dng_bayer_phase passed to dng_mux_set_lsc(); testCxxPhotoMuxerDngSetsThum
 * bnailAndLsc above only ever exercises it indirectly via setFileMetadata()
 * with VDEF_RAW_PIX_ORDER_RGGB, leaving the BGGR/GRBG/GBRG cases and the
 * default (unknown -> DNG_BAYER_UNKNOWN) branch at 0% coverage. Since it's a
 * free function declared at namespace scope (pdraw_muxer_record_dng.hpp:43,
 * already included above under the private/protected trick, though that
 * trick is irrelevant here -- it only affects class member access), it's
 * reachable directly and synchronously, with no muxer/session/pipeline
 * needed at all. */
static void testCxxPhotoMuxerDngLscFormatMapping()
{
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_RGGB),
			DNG_BAYER_RGGB);
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_BGGR),
			DNG_BAYER_BGGR);
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_GRBG),
			DNG_BAYER_GRBG);
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_GBRG),
			DNG_BAYER_GBRG);
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_UNKNOWN),
			DNG_BAYER_UNKNOWN);
	/* Any other, non-Bayer-aliased order also falls into default:. */
	CU_ASSERT_EQUAL(mapVdefToDngPhase(VDEF_RAW_PIX_ORDER_DABC),
			DNG_BAYER_UNKNOWN);
}


/* Mirror image of testCxxPhotoMuxerPngCreateMediaRejectsRawVideo /
 * testCxxPhotoMuxerJfifCreateMediaRejectsRawVideo: DngRecordMuxer::
 * createMedia() (pdraw_muxer_record_dng.cpp) only accepts
 * Media::Type::RAW_VIDEO (DNG muxes raw Bayer sensor data, not coded video --
 * see the file-level comment above testCxxPhotoMuxerWritesOneDngPerFrame), so
 * a CODED_VIDEO cfg falls into `default:` and returns nullptr. */
static void testCxxPhotoMuxerDngCreateMediaRejectsCodedVideo()
{
	TestPompLoop loop;
	PhotoSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kPattern =
		"/tmp/pdraw_test_pipeline_muxer_photo_dng_reject_%04u.dng";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPattern, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *dngMuxer = static_cast<DngRecordMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(dngMuxer);

	MuxerMediaConfig cfg = {};
	cfg.type = Media::Type::CODED_VIDEO;
	std::unique_ptr<RecordMuxer::MuxerMedia> track =
		dngMuxer->createMedia(cfg);
	CU_ASSERT_PTR_NULL(track.get());

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotMuxerClose = loop.pumpUntil([&muxerListener]() {
		return muxerListener.mGotCloseResponse.load();
	});
	CU_ASSERT_TRUE(gotMuxerClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus.load(), 0);
	muxerOwner.reset();

	ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop3 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop3);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}

#endif /* BUILD_LIBDNG_PARROT */

/* Helpers shared by the photo-muxer C API tests below */

struct PhotoTestSessionState {
	unsigned int lastMediaId = 0;
	int stopRespCount = 0;
};

static void photo_test_media_added_cb(struct pdraw * /*p*/,
				      const struct pdraw_media_info *info,
				      void * /*elem*/,
				      void *ud)
{
	static_cast<PhotoTestSessionState *>(ud)->lastMediaId = info->id;
}

static void
photo_test_stop_resp_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<PhotoTestSessionState *>(ud)->stopRespCount++;
}

static bool make_pdraw_for_photo_test(TestPompLoop &loop,
				      PhotoTestSessionState &state,
				      struct pdraw **out)
{
	struct pdraw_cbs cbs = {};
	cbs.stop_resp = photo_test_stop_resp_cb;
	cbs.media_added = photo_test_media_added_cb;
	int ret = pdraw_new(loop.raw(), &cbs, &state, out);
	return ret == 0 && *out != nullptr;
}

/* flushed/drained are mandatory in pdraw_coded_video_source_cbs */
static void photo_src_flushed_cb(struct pdraw * /*p*/,
				 struct pdraw_coded_video_source * /*s*/,
				 void * /*ud*/)
{
}
static void photo_src_drained_cb(struct pdraw * /*p*/,
				 struct pdraw_coded_video_source * /*s*/,
				 void * /*ud*/)
{
}

struct PhotoMuxerCbState {
	std::atomic<bool> mediaReadyFired{false};
	std::atomic<bool> mediaSavedFired{false};
	std::atomic<bool> unrecoverableErrorFired{false};
	int closeRespCount = 0;
	int closeRespStatus = -1;
	/* guarded by savedPathMutex — media_ready is called from writer thread
	 */
	std::mutex savedPathMutex;
	std::string savedPath;
	size_t readyTotalLen = 0;

	std::string getSavedPath()
	{
		std::lock_guard<std::mutex> lock(savedPathMutex);
		return savedPath;
	}
};

static void photo_muxer_media_ready_cb(struct pdraw * /*p*/,
				       struct pdraw_muxer * /*m*/,
				       const char * /*path*/,
				       const struct iovec *iov,
				       int iovcnt,
				       void *ud)
{
	auto *s = static_cast<PhotoMuxerCbState *>(ud);
	size_t total = 0;
	for (int i = 0; i < iovcnt; i++)
		total += iov[i].iov_len;
	s->readyTotalLen = total;
	s->mediaReadyFired.store(true);
}

static void photo_muxer_media_saved_cb(struct pdraw * /*p*/,
				       struct pdraw_muxer * /*m*/,
				       const char *path,
				       void *ud)
{
	auto *s = static_cast<PhotoMuxerCbState *>(ud);
	{
		std::lock_guard<std::mutex> lock(s->savedPathMutex);
		s->savedPath = path ? path : "";
	}
	s->mediaSavedFired.store(true);
}

static void photo_muxer_unrecoverable_error_cb(struct pdraw * /*p*/,
					       struct pdraw_muxer * /*m*/,
					       int /*status*/,
					       void *ud)
{
	static_cast<PhotoMuxerCbState *>(ud)->unrecoverableErrorFired.store(
		true);
}

static void photo_muxer_close_resp_cb(struct pdraw * /*p*/,
				      struct pdraw_muxer * /*m*/,
				      int status,
				      void *ud)
{
	auto *s = static_cast<PhotoMuxerCbState *>(ud);
	s->closeRespStatus = status;
	s->closeRespCount++;
}

/* Pushes one minimal PNG frame into the coded source's queue.
 * Returns 0 on success, negative errno on failure. */
static int push_png_frame(struct pdraw *p, struct pdraw_coded_video_source *src)
{
	struct vdef_coded_frame fi = {};
	fi.format = vdef_png;
	fi.info.timescale = 1000000;
	fi.info.timestamp = 0;
	fi.info.index = 0;
	fi.info.resolution.width = 1;
	fi.info.resolution.height = 1;
	fi.info.bit_depth = 8;
	fi.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&fi, &frame);
	if (ret < 0)
		return ret;

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kMinimalPng), &mem);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	uint8_t *data = nullptr;
	size_t cap = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &cap);
	if (ret == 0 && cap >= sizeof(kMinimalPng))
		memcpy(data, kMinimalPng, sizeof(kMinimalPng));

	struct vdef_nalu nalu = {};
	nalu.size = sizeof(kMinimalPng);
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	mbuf_mem_unref(mem);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	ret = mbuf_coded_video_frame_finalize(frame);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_source_get_queue(p, src);
	if (q == nullptr) {
		mbuf_coded_video_frame_unref(frame);
		return -EINVAL;
	}

	ret = mbuf_coded_video_frame_queue_push(q, frame);
	mbuf_coded_video_frame_unref(frame);
	return ret;
}


static int pushFrameOfSize(struct pdraw *p,
			   struct pdraw_coded_video_source *src,
			   size_t size)
{
	struct vdef_coded_frame fi = {};
	fi.format = vdef_png;
	fi.info.timescale = 1000000;
	fi.info.timestamp = 0;
	fi.info.index = 0;
	fi.info.resolution.width = 1;
	fi.info.resolution.height = 1;
	fi.info.bit_depth = 8;
	fi.type = VDEF_CODED_FRAME_TYPE_CODED;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&fi, &frame);
	if (ret < 0)
		return ret;

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(size, &mem);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	struct vdef_nalu nalu = {};
	nalu.size = size;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	mbuf_mem_unref(mem);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	ret = mbuf_coded_video_frame_finalize(frame);
	if (ret < 0) {
		mbuf_coded_video_frame_unref(frame);
		return ret;
	}

	struct mbuf_coded_video_frame_queue *q =
		pdraw_coded_video_source_get_queue(p, src);
	if (q == nullptr) {
		mbuf_coded_video_frame_unref(frame);
		return -EINVAL;
	}

	ret = mbuf_coded_video_frame_queue_push(q, frame);
	mbuf_coded_video_frame_unref(frame);
	return ret;
}


/* Creates a PNG-format coded video source and a photo muxer, wires them
 * together via pdraw_muxer_add_media(), and returns handles + media ID.
 * Caller owns both objects and must close/destroy them. */
static bool setup_photo_pipeline(TestPompLoop &loop,
				 struct pdraw *p,
				 PhotoTestSessionState &sessState,
				 const char *muxPattern,
				 const struct pdraw_muxer_params *muxerParams,
				 const struct pdraw_muxer_cbs *muxerCbs,
				 PhotoMuxerCbState *muxState,
				 struct pdraw_coded_video_source **srcOut,
				 struct pdraw_muxer **muxOut)
{
	struct pdraw_video_source_params srcParams = {};
	srcParams.queue_max_count = 0;
	srcParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	srcParams.video.format = VDEF_FRAME_TYPE_CODED;
	srcParams.video.coded.format = vdef_png;
	srcParams.video.coded.info.resolution.width = 1;
	srcParams.video.coded.info.resolution.height = 1;
	srcParams.video.coded.info.bit_depth = 8;

	struct pdraw_coded_video_source_cbs srcCbs = {};
	srcCbs.flushed = photo_src_flushed_cb;
	srcCbs.drained = photo_src_drained_cb;
	int ret = pdraw_coded_video_source_new(
		p, &srcParams, &srcCbs, nullptr, srcOut);
	if (ret != 0 || *srcOut == nullptr)
		return false;

	bool gotMedia = loop.pumpUntil(
		[&sessState]() { return sessState.lastMediaId != 0; }, 5000);
	if (!gotMedia)
		return false;

	ret = pdraw_muxer_new(
		p, muxPattern, muxerParams, muxerCbs, muxState, muxOut);
	if (ret != 0 || *muxOut == nullptr)
		return false;

	struct pdraw_muxer_media_params mediaParams = {};
	ret = pdraw_muxer_add_media(
		p, *muxOut, sessState.lastMediaId, &mediaParams);
	return ret == 0;
}


/* Verify that pushing a PNG frame through the C API photo muxer fires both
 * the media_ready and media_saved shims in pdraw_wrapper.cpp. */
static void testCMuxerListenerMediaReadySaved()
{
	TestPompLoop loop;
	PhotoTestSessionState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw_for_photo_test(loop, sessState, &p));

	static const char *kPattern =
		"/tmp/pdraw_test_c_muxer_media_ready_%04u.png";
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	PhotoMuxerCbState muxState;
	struct pdraw_muxer_cbs muxerCbs = {};
	muxerCbs.media_ready = photo_muxer_media_ready_cb;
	muxerCbs.media_saved = photo_muxer_media_saved_cb;
	muxerCbs.close_resp = photo_muxer_close_resp_cb;

	struct pdraw_coded_video_source *src = nullptr;
	struct pdraw_muxer *mux = nullptr;
	CU_ASSERT_TRUE_FATAL(setup_photo_pipeline(loop,
						  p,
						  sessState,
						  kPattern,
						  &muxerParams,
						  &muxerCbs,
						  &muxState,
						  &src,
						  &mux));

	int ret = push_png_frame(p, src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* media_saved fires on the pomp loop thread after media_ready (writer
	 * thread); pumping until saved guarantees both have run. */
	bool gotSaved = loop.pumpUntil(
		[&muxState]() { return muxState.mediaSavedFired.load(); },
		10000);
	CU_ASSERT_TRUE(gotSaved);
	CU_ASSERT_TRUE(muxState.mediaReadyFired.load());
	CU_ASSERT_EQUAL(muxState.readyTotalLen, sizeof(kMinimalPng));
	CU_ASSERT_STRING_EQUAL(muxState.getSavedPath().c_str(),
			       "/tmp/pdraw_test_c_muxer_media_ready_0001.png");

	ret = pdraw_muxer_close(p, mux);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&muxState]() { return muxState.closeRespCount >= 1; }, 10000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(muxState.closeRespStatus, 0);

	pdraw_muxer_destroy(p, mux);
	pdraw_coded_video_source_destroy(p, src);
	pdraw_destroy(p);
	(void)remove("/tmp/pdraw_test_c_muxer_media_ready_0001.png");
}


/* Verify that a disk-full simulation triggers the unrecoverable_error shim.
 * free_space_limit = current_free − kLargeFrameSize/2.  The startup guard
 * (spaceNeeded=0) passes as long as disk shrinks by less than 512 KiB between
 * this statvfs() and internalStart()'s re-measurement — safe on any CI.
 * The first frame write always triggers onUnrecoverableError() because
 * mFreeSpaceLeft (≈ current_free) < free_space_limit + kLargeFrameSize. */
static void testCMuxerListenerUnrecoverableError()
{
	TestPompLoop loop;
	PhotoTestSessionState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw_for_photo_test(loop, sessState, &p));

	static const char *kPattern =
		"/tmp/pdraw_test_c_muxer_unrecoverable_%04u.png";
	static const size_t kLargeFrameSize = 1u << 20; /* 1 MiB */
	struct statvfs vfsStats = {};
	CU_ASSERT_EQUAL_FATAL(statvfs("/tmp", &vfsStats), 0);
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.initial_file_index = 1;
	muxerParams.free_space_limit =
		(size_t)vfsStats.f_bavail * vfsStats.f_bsize -
		kLargeFrameSize / 2;
	PhotoMuxerCbState muxState;
	struct pdraw_muxer_cbs muxerCbs = {};
	muxerCbs.unrecoverable_error = photo_muxer_unrecoverable_error_cb;
	muxerCbs.close_resp = photo_muxer_close_resp_cb;

	struct pdraw_coded_video_source *src = nullptr;
	struct pdraw_muxer *mux = nullptr;
	CU_ASSERT_TRUE_FATAL(setup_photo_pipeline(loop,
						  p,
						  sessState,
						  kPattern,
						  &muxerParams,
						  &muxerCbs,
						  &muxState,
						  &src,
						  &mux));

	char expected[512];
	snprintf(expected,
		 sizeof(expected),
		 "/tmp/pdraw_test_c_muxer_unrecoverable_0001.png");
	(void)remove(expected);

	for (int i = 0; i < 2000; i++) {
		if (pushFrameOfSize(p, src, kLargeFrameSize) != 0)
			break;
		if (muxState.unrecoverableErrorFired.load())
			break;
		(void)loop.pumpUntil(
			[&muxState]() {
				return muxState.unrecoverableErrorFired.load();
			},
			10);
	}

	bool gotError = loop.pumpUntil(
		[&muxState]() {
			return muxState.unrecoverableErrorFired.load();
		},
		10000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_FALSE(muxState.mediaReadyFired.load());
	CU_ASSERT_FALSE(muxState.mediaSavedFired.load());

	int ret = pdraw_muxer_close(p, mux);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&muxState]() { return muxState.closeRespCount >= 1; }, 10000);
	CU_ASSERT_TRUE(gotClose);

	pdraw_muxer_destroy(p, mux);
	pdraw_coded_video_source_destroy(p, src);
	pdraw_destroy(p);

	(void)remove(expected);
	struct stat st = {};
	CU_ASSERT_NOT_EQUAL(stat(expected, &st), 0);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_muxer_record_photo[] = {
	{FN("testCxxPhotoMuxerWritesOnePngPerFrame"),
	 testCxxPhotoMuxerWritesOnePngPerFrame},
	{FN("testCxxPhotoMuxerPngCreateMediaRejectsRawVideo"),
	 testCxxPhotoMuxerPngCreateMediaRejectsRawVideo},
	{FN("testCxxPhotoMuxerRawVideoInternalSetupFormatDefaultReturnsZero"),
	 testCxxPhotoMuxerRawVideoInternalSetupFormatDefaultReturnsZero},
	{FN("testCxxPhotoMuxerGetThreadName"), testCxxPhotoMuxerGetThreadName},
	{FN("testCxxPhotoMuxerAddChapterAndForceSyncReturnEnosys"),
	 testCxxPhotoMuxerAddChapterAndForceSyncReturnEnosys},
	{FN("testCxxPhotoMuxerReportsUnrecoverableErrorOnFreeSpaceCheckFailure"),
	 testCxxPhotoMuxerReportsUnrecoverableErrorOnFreeSpaceCheckFailure},
	{FN("testCxxPhotoMuxerSourceFlushCompletesAutomatically"),
	 testCxxPhotoMuxerSourceFlushCompletesAutomatically},
	{FN("testCxxPhotoMuxerSourceDrainCompletesAutomatically"),
	 testCxxPhotoMuxerSourceDrainCompletesAutomatically},
#ifdef BUILD_LIBJFIF
	{FN("testCxxPhotoMuxerWritesOneJpegPerFrame"),
	 testCxxPhotoMuxerWritesOneJpegPerFrame},
	{FN("testCxxPhotoMuxerJfifSetsThumbnail"),
	 testCxxPhotoMuxerJfifSetsThumbnail},
	{FN("testCxxPhotoMuxerJfifCreateMediaRejectsRawVideo"),
	 testCxxPhotoMuxerJfifCreateMediaRejectsRawVideo},
#endif
#ifdef BUILD_LIBDNG_PARROT
	{FN("testCxxPhotoMuxerWritesOneDngPerFrame"),
	 testCxxPhotoMuxerWritesOneDngPerFrame},
	{FN("testCxxPhotoMuxerDngSetsThumbnailAndLsc"),
	 testCxxPhotoMuxerDngSetsThumbnailAndLsc},
	{FN("testCxxPhotoMuxerDngLscFormatMapping"),
	 testCxxPhotoMuxerDngLscFormatMapping},
	{FN("testCxxPhotoMuxerDngCreateMediaRejectsCodedVideo"),
	 testCxxPhotoMuxerDngCreateMediaRejectsCodedVideo},
#endif
	{FN("testCMuxerListenerMediaReadySaved"),
	 testCMuxerListenerMediaReadySaved},
	{FN("testCMuxerListenerUnrecoverableError"),
	 testCMuxerListenerUnrecoverableError},
	CU_TEST_INFO_NULL,
};
