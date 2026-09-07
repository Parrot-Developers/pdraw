/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Muxer API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_muxer
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

/* Needed for MuxerWrapper / Muxer white-box tests. */
#include "pdraw_muxer.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_muxer_*) ───────────────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_muxer_params params = {};
	struct pdraw_muxer *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_new(nullptr,
			      "/tmp/test.mp4",
			      &params,
			      &g_stub_muxer_cbs,
			      nullptr,
			      &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullUrl */
	ret = pdraw_muxer_new(g_test_pdraw_c,
			      nullptr,
			      &params,
			      &g_stub_muxer_cbs,
			      nullptr,
			      &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_muxer_new(g_test_pdraw_c,
			      "/tmp/test.mp4",
			      nullptr,
			      &g_stub_muxer_cbs,
			      nullptr,
			      &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_muxer_new(g_test_pdraw_c,
			      "/tmp/test.mp4",
			      &params,
			      nullptr,
			      nullptr,
			      &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_muxer_new(g_test_pdraw_c,
			      "/tmp/test.mp4",
			      &params,
			      &g_stub_muxer_cbs,
			      nullptr,
			      nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiClose()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_close(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_close(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiAddMedia()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_add_media(nullptr, nullptr, 0, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_add_media(g_test_pdraw_c, nullptr, 0, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiGetDynParams()
{
	struct pdraw_muxer_dyn_params dp = {};
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_get_dyn_params(nullptr, nullptr, &dp);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_get_dyn_params(g_test_pdraw_c, nullptr, &dp);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiGetStats()
{
	struct pdraw_muxer_stats stats = {};
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_get_stats(nullptr, nullptr, &stats);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_get_stats(g_test_pdraw_c, nullptr, &stats);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

static void testCApiForceSync()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_muxer_force_sync(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullMuxer */
	ret = pdraw_muxer_force_sync(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* ── C++ API (IPdraw::createMuxer) ──────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_muxer_params params = {};
	IPdraw::IMuxer *obj = nullptr;
	int ret;

	/* EmptyUrl */
	ret = session->createMuxer("", &params, &g_stub_muxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullListener */
	ret = session->createMuxer("/tmp/test.mp4", &params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = session->createMuxer(
		"/tmp/test.mp4", &params, &g_stub_muxer_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* ShortUrl: url.length() < 4 → MuxerWrapper constructor guard
	 * (pdraw_muxer.cpp lines 759-760) */
	ret = session->createMuxer("ab", &params, &g_stub_muxer_listener, &obj);
	CU_ASSERT(ret < 0);
	CU_ASSERT_PTR_NULL(obj);

	/* UnsupportedExtension: else branch after .mp4/.MP4/photo types
	 * (pdraw_muxer.cpp line 799) */
	ret = session->createMuxer("/tmp/pdraw_test_unsupported.xyz",
				   &params,
				   &g_stub_muxer_listener,
				   &obj);
	CU_ASSERT(ret < 0);
	CU_ASSERT_PTR_NULL(obj);
}


/* ── C++ API — behavioral tests on a real record (.mp4) muxer ───────────── */

/* Muxer listener that records the async close response, since (like the
 * demuxer) muxerCloseResponse() is dispatched via an idle handler on the
 * pomp loop rather than called synchronously. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

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
			       const char * /*mp*/,
			       const struct iovec * /*iov*/,
			       int /*iovcnt*/) override
	{
	}

	void onMuxerMediaSaved(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*mp*/) override
	{
	}

	void onMuxerUnrecoverableError(IPdraw * /*p*/,
				       IPdraw::IMuxer * /*m*/,
				       int /*status*/) override
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


/* Create a record (.mp4) muxer writing to /tmp: file creation there is
 * already exercised elsewhere in this suite (session dumpPipeline()) and
 * does not depend on any external hardware, unlike the GL/ALSA renderers. */
static IPdraw::IMuxer *createValidMuxer(IPdraw *session,
					RecordingMuxerListener *listener)
{
	struct pdraw_muxer_params params = {};
	IPdraw::IMuxer *obj = nullptr;
	int ret = session->createMuxer(
		"/tmp/pdraw_test_muxer.mp4", &params, listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);
	return obj;
}

static void closeAndDestroyMuxer(IPdraw::IMuxer *obj,
				 RecordingMuxerListener *listener)
{
	int ret = obj->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = g_test_loop->pumpUntil(
		[listener]() { return listener->mGotCloseResponse; });
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);
	CU_ASSERT_FALSE(listener->mGotUnrecoverableError);
	auto objOwner = std::unique_ptr<IPdraw::IMuxer>(obj);
}

static void testCxxCreateValid()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);
	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxGetStatsInitial()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	struct pdraw_muxer_stats stats = {};
	CU_ASSERT_EQUAL(obj->getStats(&stats), 0);
	CU_ASSERT_EQUAL(stats.type, PDRAW_MUXER_TYPE_RECORD);

	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxForceSyncOnRecordMuxer()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	/* forceSync() is only meaningful (and only returns something other
	 * than -ENOSYS) on a record muxer, which is exactly what this test
	 * creates. */
	CU_ASSERT_EQUAL(obj->forceSync(), 0);

	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxDynParamsRoundtrip()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	struct pdraw_muxer_dyn_params dp = {};
	dp.tables_sync_period_ms = 500;
	CU_ASSERT_EQUAL(obj->setDynParams(&dp), 0);

	/* setDynParams() posts the change asynchronously to the muxer's
	 * writer thread; getDynParams() here only confirms the getter itself
	 * works, not that the just-set value has propagated yet. */
	struct pdraw_muxer_dyn_params outDp = {};
	CU_ASSERT_EQUAL(obj->getDynParams(&outDp), 0);

	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxAddMediaUnknownIdReturnsEnoent()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	/* No media exists anywhere in this session's pipeline. */
	CU_ASSERT_EQUAL(obj->addMedia(999999, nullptr), -ENOENT);

	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxAddChapterOnRecordMuxer()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	/* addChapter() only posts the request to the muxer's internal
	 * writer thread and returns as soon as it's queued; the actual
	 * write (and any failure past this point, e.g. a non-monotonic
	 * timestamp) happens asynchronously and is only logged, never
	 * reported back to the caller. 0 here confirms the request was
	 * accepted, not that a chapter was actually written -- this can be
	 * called before any media is added (chapters queue up and get
	 * flushed once the first video track exists). timestamp=0 avoids a
	 * libmp4 "non-monotonic timestamp" rejection that only kicks in
	 * once a previous chapter had a nonzero timestamp. */
	CU_ASSERT_EQUAL(obj->addChapter(0, "Start"), 0);

	/* name is validated synchronously. */
	CU_ASSERT_EQUAL(obj->addChapter(0, nullptr), -EINVAL);
	CU_ASSERT_EQUAL(obj->addChapter(0, ""), -EINVAL);

	closeAndDestroyMuxer(obj, &listener);
}

static void testCxxSetThumbnailOnRecordMuxer()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	IPdraw::IMuxer *obj = createValidMuxer(session, &listener);

	/* setThumbnail() never parses the image data (mp4_mux_set_file_cover
	 * just memcpy()s it), so dummy bytes are accepted as long as size>0
	 * and the type is a known one -- only type/data/size are validated
	 * synchronously. */
	static const uint8_t kDummyThumbnail[4] = {0xff, 0xd8, 0xff, 0xd9};
	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
					  kDummyThumbnail,
					  sizeof(kDummyThumbnail)),
			0);
	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_PNG,
					  kDummyThumbnail,
					  sizeof(kDummyThumbnail)),
			0);
	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_BMP,
					  kDummyThumbnail,
					  sizeof(kDummyThumbnail)),
			0);

	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_UNKNOWN,
					  kDummyThumbnail,
					  sizeof(kDummyThumbnail)),
			-EINVAL);
	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
					  nullptr,
					  sizeof(kDummyThumbnail)),
			-EINVAL);
	CU_ASSERT_EQUAL(obj->setThumbnail(PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
					  kDummyThumbnail,
					  0),
			-EINVAL);

	/* setThumbnail() only rejects TYPE_UNKNOWN synchronously; it does
	 * not range-check the enum. An out-of-range value is accepted here
	 * and only turns into MP4_METADATA_COVER_TYPE_UNKNOWN later on the
	 * writer thread (thumbnailTypeToCoverType()'s default case), with
	 * no error ever reported back to the caller. */
	CU_ASSERT_EQUAL(
		obj->setThumbnail(
			static_cast<enum pdraw_muxer_thumbnail_type>(999),
			kDummyThumbnail,
			sizeof(kDummyThumbnail)),
		0);

	closeAndDestroyMuxer(obj, &listener);
}


static void
muxer_connection_state_changed_cb(struct pdraw *pdraw,
				  struct pdraw_muxer *muxer,
				  enum pdraw_muxer_connection_state state,
				  enum pdraw_muxer_disconnection_reason reason,
				  void *userdata)
{
}
static void muxer_media_ready_cb(struct pdraw *pdraw,
				 struct pdraw_muxer *muxer,
				 const char *media_path,
				 const struct iovec *iov,
				 int iovcnt,
				 void *userdata)
{
}
static void muxer_media_saved_cb(struct pdraw *pdraw,
				 struct pdraw_muxer *muxer,
				 const char *media_path,
				 void *userdata)
{
}
static void muxer_unrecoverable_error_cb(struct pdraw *pdraw,
					 struct pdraw_muxer *muxer,
					 int status,
					 void *userdata)
{
}
static void muxer_close_response_cb(struct pdraw *pdraw,
				    struct pdraw_muxer *muxer,
				    int status,
				    void *userdata)
{
	bool *gotClose = (bool *)userdata;
	if (gotClose != nullptr)
		*gotClose = true;
}

static const struct pdraw_muxer_cbs g_valid_muxer_cbs = {
	.connection_state_changed = muxer_connection_state_changed_cb,
	.media_ready = muxer_media_ready_cb,
	.media_saved = muxer_media_saved_cb,
	.unrecoverable_error = muxer_unrecoverable_error_cb,
	.close_resp = muxer_close_response_cb,
};

static void testCApiLifecycle()
{
	struct pdraw_muxer_params params = {};
	struct pdraw_muxer *obj = nullptr;
	bool gotClose = false;
	int ret;

	ret = pdraw_muxer_new(g_test_pdraw_c,
			      "/tmp/pdraw_test_capi_muxer.mp4",
			      &params,
			      &g_valid_muxer_cbs,
			      &gotClose,
			      &obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(obj);

	if (obj != nullptr) {
		ret = pdraw_muxer_close(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);

		bool success = g_test_loop->pumpUntil(
			[&gotClose]() { return gotClose; }, 15000);
		CU_ASSERT_TRUE(success);

		ret = pdraw_muxer_destroy(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


/* set_dyn_params()/add_chapter()/set_thumbnail()/set_file_metadata()/
 * get_stats()/force_sync(): all six are only exercised via IPdraw::IMuxer
 * directly (testCxxDynParamsRoundtrip/testCxxAddChapterOnRecordMuxer/
 * testCxxSetThumbnailOnRecordMuxer above, testCxxMuxerRecordSetDynParams
 * GetStats in test_pipeline_muxer_record.cpp) -- never through their
 * pdraw_muxer_*() C wrapper, which showed 0 hits in gcov before this test.
 * Values mirror those already-proven-safe call sites exactly. */
static void testCApiMethodsValid()
{
	struct pdraw_muxer_params params = {};
	struct pdraw_muxer *obj = nullptr;
	bool gotClose = false;
	int ret = pdraw_muxer_new(g_test_pdraw_c,
				  "/tmp/pdraw_test_capi_muxer_methods.mp4",
				  &params,
				  &g_valid_muxer_cbs,
				  &gotClose,
				  &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	struct pdraw_muxer_dyn_params dp = {};
	dp.tables_sync_period_ms = 500;
	ret = pdraw_muxer_set_dyn_params(g_test_pdraw_c, obj, &dp);
	CU_ASSERT_EQUAL(ret, 0);

	/* NullDynParams: previously untested (0 hits in gcov), only reachable
	 * with a live, non-null muxer object. */
	ret = pdraw_muxer_get_dyn_params(g_test_pdraw_c, obj, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* get_stats()/force_sync(): previously untested (0 hits in gcov) --
	 * the null-arg tests above only ever pass a null muxer, so the real
	 * forwarding calls (RecordMuxer::getStats()/IsobmffRecordMuxer::
	 * forceSync()) never ran. Both succeed synchronously on a live
	 * RecordMuxer with its writer thread up (forceSync() posts an async
	 * task but returns 0 as soon as it's queued). */
	struct pdraw_muxer_stats stats = {};
	ret = pdraw_muxer_get_stats(g_test_pdraw_c, obj, &stats);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_muxer_force_sync(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);

	/* name is validated synchronously (see testCxxAddChapterOnRecordMuxer);
	 * timestamp=0 avoids a libmp4 "non-monotonic timestamp" rejection. */
	ret = pdraw_muxer_add_chapter(g_test_pdraw_c, obj, 0, "Start");
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_muxer_add_chapter(g_test_pdraw_c, obj, 0, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	static const uint8_t kDummyThumbnail[4] = {0xff, 0xd8, 0xff, 0xd9};
	ret = pdraw_muxer_set_thumbnail(g_test_pdraw_c,
					obj,
					PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,
					kDummyThumbnail,
					sizeof(kDummyThumbnail));
	CU_ASSERT_EQUAL(ret, 0);

	/* Unsupported by the record muxer but still returns 0 synchronously
	 * once params are valid (see the identical C++ assertion in
	 * test_pipeline_muxer_record.cpp). */
	ret = pdraw_muxer_set_file_metadata(
		g_test_pdraw_c, obj, nullptr, kDummyThumbnail, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	struct pdraw_muxer_metadata_params metaParams = {};
	metaParams.type = PDRAW_MUXER_METADATA_TYPE_DNG_LSC;
	ret = pdraw_muxer_set_file_metadata(g_test_pdraw_c,
					    obj,
					    &metaParams,
					    kDummyThumbnail,
					    sizeof(kDummyThumbnail));
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_muxer_close(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
	bool success = g_test_loop->pumpUntil(
		[&gotClose]() { return gotClose; }, 15000);
	CU_ASSERT_TRUE(success);

	ret = pdraw_muxer_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


/* Session listener that tracks stop response and raw video source media add,
 * used by tests that need a private session they will tear down. */
class MuxerSessionListener : public StubPdrawListener {
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

	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	unsigned int mMediaId = 0;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Covers MuxerWrapper constructor line 110 (mStorageDirPath = ".")
 * reached only when the filename has no directory separator. */
static void testCxxBareFilenameConstructor()
{
	IPdraw *session = g_test_session->get();
	RecordingMuxerListener listener;
	struct pdraw_muxer_params params = {};
	IPdraw::IMuxer *obj = nullptr;
	int ret = session->createMuxer(
		"pdraw_test_bare.mp4", &params, &listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);
	closeAndDestroyMuxer(obj, &listener);
	(void)remove("pdraw_test_bare.mp4");
}


/* Covers MuxerWrapper post-stop guards: after the session is torn down,
 * mElementStopped is true and every IMuxer method returns -EPROTO. */
static void testCxxWrapperGuardsAfterElementCleared()
{
	static const char *const kPath = "/tmp/test_muxer_wrapper_guards.mp4";

	TestPompLoop loop;
	MuxerSessionListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	(void)remove(kPath);
	struct pdraw_muxer_params muxerParams = {};
	RecordingMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret = session->createMuxer(
		kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);

	stopSessionAndWait(&loop, session, &sessionListener);

	/* After session stop, mElementStopped is true; all guards fire. */
	CU_ASSERT_EQUAL(muxer->close(), -EPROTO);
	CU_ASSERT_EQUAL(muxer->setThumbnail(
				PDRAW_MUXER_THUMBNAIL_TYPE_JPEG, nullptr, 0),
			-EPROTO);
	CU_ASSERT_EQUAL(muxer->addChapter(0, "x"), -EPROTO);
	struct pdraw_muxer_metadata_params metaParams = {};
	CU_ASSERT_EQUAL(muxer->setFileMetadata(&metaParams, nullptr, 0),
			-EPROTO);
	struct pdraw_muxer_stats stats = {};
	CU_ASSERT_EQUAL(muxer->getStats(&stats), -EPROTO);
	struct pdraw_muxer_dyn_params dynParams = {};
	CU_ASSERT_EQUAL(muxer->setDynParams(&dynParams), -EPROTO);
	CU_ASSERT_EQUAL(muxer->getDynParams(&dynParams), -EPROTO);
	CU_ASSERT_EQUAL(muxer->forceSync(), -EPROTO);
	struct pdraw_muxer_media_params mediaParams = {};
	CU_ASSERT_EQUAL(muxer->addMedia(0, &mediaParams), -EPROTO);

	(void)remove(kPath);
}


/* Covers Muxer::onChannelTeardown() lines 572, 574, 576, 579, 580:
 * when the only connected source is destroyed while the muxer is STARTED,
 * the channel teardown notification fires Muxer::onChannelTeardown(), which
 * sees empty input ports and calls stop() — the muxer self-closes without
 * an explicit IMuxer::close() call. */
static void testCxxChannelTeardownSelfStops()
{
	static const char *const kPath =
		"/tmp/pdraw_test_teardown_selfstop.mp4";

	TestPompLoop loop;
	MuxerSessionListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
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
	sessionListener.mSource = source;

	bool gotMedia = loop.pumpUntil([&sessionListener]() {
		return sessionListener.mGotMediaAdded;
	});
	CU_ASSERT_TRUE_FATAL(gotMedia);

	(void)remove(kPath);
	struct pdraw_muxer_params muxerParams = {};
	RecordingMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(sessionListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL(ret, 0);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);

	/* Destroy the source handle while connected to the muxer. The source
	 * element stops → channel teardown → Muxer::onChannelTeardown() →
	 * stop() → the muxer closes itself asynchronously. */
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	sourceOwner.reset();

	bool gotClose = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotCloseResponse; },
		10000);
	CU_ASSERT_TRUE(gotClose);

	(void)remove(kPath);
	stopSessionAndWait(&loop, session, &sessionListener);
}


/* Covers Muxer::start() state-machine edge cases:
 *   - line 138: return 0 when already STARTING/STARTED (called right after
 *     createMuxer, before the element has time to stop)
 *   - lines 141, 144: return -EPROTO when state == STOPPED (after close)
 * White-box: inner Muxer* reached via MuxerWrapper::getMuxer(). */
static void testCxxBaseClassEdgeCases()
{
	static const char *const kPath = "/tmp/pdraw_test_muxer_base_edges.mp4";

	TestPompLoop loop;
	MuxerSessionListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
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
	sessionListener.mSource = source;

	bool gotMedia = loop.pumpUntil([&sessionListener]() {
		return sessionListener.mGotMediaAdded;
	});
	CU_ASSERT_TRUE_FATAL(gotMedia);

	(void)remove(kPath);
	struct pdraw_muxer_params muxerParams = {};
	RecordingMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	ret = session->createMuxer(kPath, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* Element is STARTING or STARTED after createMuxer; calling start()
	 * again returns 0 (Muxer::start() line 138). */
	Muxer *impl = static_cast<MuxerWrapper *>(muxer)->getMuxer();
	CU_ASSERT_PTR_NOT_NULL_FATAL(impl);
	ret = impl->start();
	CU_ASSERT_EQUAL(ret, 0);

	struct pdraw_muxer_media_params mediaParams = {};
	mediaParams.is_default = true;
	ret = muxer->addMedia(sessionListener.mMediaId, &mediaParams);
	CU_ASSERT_EQUAL(ret, 0);

	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotCloseResponse; },
		10000);
	CU_ASSERT_TRUE(gotClose);

	/* Element is STOPPED after close; start() returns -EPROTO
	 * (Muxer::start() lines 141, 144). */
	ret = impl->start();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	muxerOwner.reset();
	sourceOwner.reset();

	(void)remove(kPath);
	stopSessionAndWait(&loop, session, &sessionListener);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_api_muxer[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiClose"), testCApiClose},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiAddMedia"), testCApiAddMedia},
	{FN("testCApiGetDynParams"), testCApiGetDynParams},
	{FN("testCApiGetStats"), testCApiGetStats},
	{FN("testCApiForceSync"), testCApiForceSync},
	{FN("testCApiLifecycle"), testCApiLifecycle},
	{FN("testCApiMethodsValid"), testCApiMethodsValid},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxCreateValid"), testCxxCreateValid},
	{FN("testCxxGetStatsInitial"), testCxxGetStatsInitial},
	{FN("testCxxForceSyncOnRecordMuxer"), testCxxForceSyncOnRecordMuxer},
	{FN("testCxxDynParamsRoundtrip"), testCxxDynParamsRoundtrip},
	{FN("testCxxAddMediaUnknownIdReturnsEnoent"),
	 testCxxAddMediaUnknownIdReturnsEnoent},
	{FN("testCxxAddChapterOnRecordMuxer"), testCxxAddChapterOnRecordMuxer},
	{FN("testCxxSetThumbnailOnRecordMuxer"),
	 testCxxSetThumbnailOnRecordMuxer},
	{FN("testCxxBareFilenameConstructor"), testCxxBareFilenameConstructor},
	{FN("testCxxWrapperGuardsAfterElementCleared"),
	 testCxxWrapperGuardsAfterElementCleared},
	{FN("testCxxChannelTeardownSelfStops"),
	 testCxxChannelTeardownSelfStops},
	{FN("testCxxBaseClassEdgeCases"), testCxxBaseClassEdgeCases},
	CU_TEST_INFO_NULL,
};
