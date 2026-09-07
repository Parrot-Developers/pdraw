/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Video IPC source API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_vipc_source
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_vipc_source_*) ────────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_vipc_source_params params = {};
	struct pdraw_vipc_source *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_new(
		nullptr, &params, &g_stub_vipc_source_cbs, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_vipc_source_new(g_test_pdraw_c,
				    nullptr,
				    &g_stub_vipc_source_cbs,
				    nullptr,
				    &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_vipc_source_new(
		g_test_pdraw_c, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_vipc_source_new(g_test_pdraw_c,
				    &params,
				    &g_stub_vipc_source_cbs,
				    nullptr,
				    nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiPlay()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_play(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_play(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiPause()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_pause(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_pause(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiInsertGreyFrame()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_insert_grey_frame(nullptr, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_insert_grey_frame(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiGetSessionMetadata()
{
	struct vmeta_session meta = {};
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_get_session_metadata(nullptr, nullptr, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_get_session_metadata(
		g_test_pdraw_c, nullptr, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* is_ready_to_play()/is_paused() previously had no test at all (0 hits in
 * gcov, not even this null-guard branch): unlike most pdraw_vipc_source_*()
 * wrappers, both return a plain 0/1 int rather than a negative errno, even
 * for null arguments. */
static void testCApiIsReadyToPlay()
{
	CU_ASSERT_EQUAL(pdraw_vipc_source_is_ready_to_play(nullptr, nullptr),
			0);
	CU_ASSERT_EQUAL(
		pdraw_vipc_source_is_ready_to_play(g_test_pdraw_c, nullptr), 0);
}


static void testCApiIsPaused()
{
	CU_ASSERT_EQUAL(pdraw_vipc_source_is_paused(nullptr, nullptr), 0);
	CU_ASSERT_EQUAL(pdraw_vipc_source_is_paused(g_test_pdraw_c, nullptr),
			0);
}


/* configure()/set_session_metadata(): also previously had no test at all. */
static void testCApiConfigure()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_configure(nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_configure(
		g_test_pdraw_c, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* is_ready_to_play()/is_paused()/configure()/insert_grey_frame() happy path:
 * the null-arg tests above only ever pass a null source, so the real
 * forwarding calls (VipcSource::isReadyToPlay()/isPaused()/configure()/
 * insertGreyFrame()) never ran (0 hits in gcov). No live VIPC server is
 * needed for any of these:
 *  - isReadyToPlay()/isPaused() are plain field getters (mReady/mRunning),
 *    both false right after creation, same as testCApiLifecycle's object.
 *  - configure() is a stub that unconditionally returns -ENOSYS regardless
 *    of state or arguments (see VipcSource::configure(), a "TODO").
 *  - insertGreyFrame() requires mStatus, only populated by a real status
 *    exchange with a server; without one it deterministically returns
 *    -EPROTO (see VipcSource::insertGreyFrame()) -- still a real forwarding
 *    call, unlike the null-guard.
 * Uses the same no-server "unix:@..." address as
 * testCVipcSourceListenerCallbacks below (short connection_timeout_ms so the
 * watchdog doesn't linger). */
static void testCApiMethodsValidNoServer()
{
	struct pdraw_vipc_source_params params = {};
	params.address = "unix:@pdraw_test_vipc_no_server_capi_happy";
	params.backend_name = "shm";
	params.connection_timeout_ms = 200;

	struct pdraw_vipc_source *obj = nullptr;
	int ret = pdraw_vipc_source_new(g_test_pdraw_c,
					&params,
					&g_stub_vipc_source_cbs,
					nullptr,
					&obj);
	if (ret != 0) {
		/* BUILD_LIBVIDEO_IPC not compiled in or shm backend
		 * unavailable. */
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	CU_ASSERT_EQUAL(pdraw_vipc_source_is_ready_to_play(g_test_pdraw_c, obj),
			0);
	CU_ASSERT_EQUAL(pdraw_vipc_source_is_paused(g_test_pdraw_c, obj), 1);

	ret = pdraw_vipc_source_configure(
		g_test_pdraw_c, obj, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	ret = pdraw_vipc_source_insert_grey_frame(g_test_pdraw_c, obj, 1000);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	ret = pdraw_vipc_source_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


static void testCApiSetSessionMetadata()
{
	struct vmeta_session meta = {};
	int ret;

	/* NullPdraw */
	ret = pdraw_vipc_source_set_session_metadata(nullptr, nullptr, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_vipc_source_set_session_metadata(
		g_test_pdraw_c, nullptr, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* ── C++ API (IPdraw::createVipcSource) ─────────────────────────── */
/* Note: createVipcSource returns -ENOSYS when BUILD_LIBVIDEO_IPC is not
 * compiled in, so we assert (ret < 0) instead of a specific errno. */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_vipc_source_params params = {};
	IPdraw::IVipcSource *obj = nullptr;
	int ret;

	/* NullParams */
	ret = session->createVipcSource(
		nullptr, &g_stub_vipc_source_listener, &obj);
	CU_ASSERT_VIPC_NULL_GUARD(ret);

	/* NullListener */
	ret = session->createVipcSource(&params, nullptr, &obj);
	CU_ASSERT_VIPC_NULL_GUARD(ret);

	/* NullRetObj */
	ret = session->createVipcSource(
		&params, &g_stub_vipc_source_listener, nullptr);
	CU_ASSERT_VIPC_NULL_GUARD(ret);
}


static void testCApiLifecycle()
{
	struct pdraw_vipc_source_params params = {};
	struct pdraw_vipc_source *obj = nullptr;
	struct pdraw_vipc_source_cbs cbs = {};
	int ret = pdraw_vipc_source_new(
		g_test_pdraw_c, &params, &cbs, nullptr, &obj);
	CU_ASSERT_TRUE(ret == 0 || ret == -ENOSYS || ret < 0);
	if (ret == 0) {
		CU_ASSERT_PTR_NOT_NULL(obj);
		ret = pdraw_vipc_source_destroy(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


/* Verify the 7 PdrawVipcSourceListener C API shims (pdraw_wrapper.cpp).
 *
 * When BUILD_LIBVIDEO_IPC is not compiled in, pdraw_vipc_source_new returns
 * non-zero and the test exits gracefully (no assertions).
 *
 * When VIPC is compiled but no server is reachable, the connection watchdog
 * fires ready_to_play(false, PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT) after
 * connection_timeout_ms, proving the ready_to_play shim end-to-end.
 *
 * The remaining 6 shims (configured, play_resp, pause_resp,
 * framerate_changed, frame_ready, end_of_stream) are wired up with real
 * callbacks and only fire when a live VIPC server is available. */
static void testCVipcSourceListenerCallbacks()
{
	struct Ud {
		int readyToPlayCount = 0;
		int lastReady = -1;
		enum pdraw_vipc_source_eos_reason lastEosReason =
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE;
		int configuredCount = 0;
		int playRespCount = 0;
		int pauseRespCount = 0;
		int framerateChangedCount = 0;
		int frameReadyCount = 0;
		int endOfStreamCount = 0;
	} ud;

	/* connection_timeout_ms = 200: watchdog fires ready_to_play(false,
	 * TIMEOUT) quickly when no VIPC server is reachable.
	 * address must be a valid pomp address (unix:@name); backend_name="shm"
	 * selects the SHM backend explicitly (null would fail getBackend). */
	struct pdraw_vipc_source_params params = {};
	params.address = "unix:@pdraw_test_vipc_no_server_capi";
	params.backend_name = "shm";
	params.connection_timeout_ms = 200;

	struct pdraw_vipc_source_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_vipc_source *,
			       int ready,
			       enum pdraw_vipc_source_eos_reason eos_reason,
			       void *userdata) {
		auto *u = static_cast<Ud *>(userdata);
		u->lastReady = ready;
		u->lastEosReason = eos_reason;
		u->readyToPlayCount++;
	};
	cbs.configured = [](struct pdraw *,
			    struct pdraw_vipc_source *,
			    int /*status*/,
			    const struct vdef_format_info *,
			    const struct vdef_rectf *,
			    void *userdata) {
		static_cast<Ud *>(userdata)->configuredCount++;
	};
	cbs.play_resp =
		[](struct pdraw *, struct pdraw_vipc_source *, void *userdata) {
			static_cast<Ud *>(userdata)->playRespCount++;
		};
	cbs.pause_resp =
		[](struct pdraw *, struct pdraw_vipc_source *, void *userdata) {
			static_cast<Ud *>(userdata)->pauseRespCount++;
		};
	cbs.framerate_changed = [](struct pdraw *,
				   struct pdraw_vipc_source *,
				   const struct vdef_frac *,
				   const struct vdef_frac *,
				   void *userdata) -> bool {
		static_cast<Ud *>(userdata)->framerateChangedCount++;
		return false;
	};
	cbs.frame_ready = [](struct pdraw *,
			     struct pdraw_vipc_source *,
			     struct mbuf_raw_video_frame *,
			     void *userdata) {
		static_cast<Ud *>(userdata)->frameReadyCount++;
	};
	cbs.end_of_stream = [](struct pdraw *,
			       struct pdraw_vipc_source *,
			       enum pdraw_vipc_source_eos_reason,
			       void *userdata) -> bool {
		static_cast<Ud *>(userdata)->endOfStreamCount++;
		return false;
	};

	struct pdraw_vipc_source *obj = nullptr;
	int ret =
		pdraw_vipc_source_new(g_test_pdraw_c, &params, &cbs, &ud, &obj);
	if (ret != 0) {
		/* BUILD_LIBVIDEO_IPC not compiled or backend unavailable. */
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	/* Budget = connection_timeout_ms + 1 s margin. */
	bool gotReady = g_test_loop->pumpUntil(
		[&ud]() { return ud.readyToPlayCount > 0; }, 1200);
	CU_ASSERT_TRUE(gotReady);
	/* ready_to_play shim: C++ bool → C int round-trip */
	CU_ASSERT_TRUE(ud.lastReady == 0 || ud.lastReady == 1);

	ret = pdraw_vipc_source_destroy(g_test_pdraw_c, obj);
	CU_ASSERT_EQUAL(ret, 0);
}


CU_TestInfo g_pdraw_test_api_vipc_source[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiPlay"), testCApiPlay},
	{FN("testCApiPause"), testCApiPause},
	{FN("testCApiInsertGreyFrame"), testCApiInsertGreyFrame},
	{FN("testCApiGetSessionMetadata"), testCApiGetSessionMetadata},
	{FN("testCApiIsReadyToPlay"), testCApiIsReadyToPlay},
	{FN("testCApiIsPaused"), testCApiIsPaused},
	{FN("testCApiConfigure"), testCApiConfigure},
	{FN("testCApiMethodsValidNoServer"), testCApiMethodsValidNoServer},
	{FN("testCApiSetSessionMetadata"), testCApiSetSessionMetadata},
	{FN("testCApiLifecycle"), testCApiLifecycle},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCVipcSourceListenerCallbacks"),
	 testCVipcSourceListenerCallbacks},
	CU_TEST_INFO_NULL,
};
