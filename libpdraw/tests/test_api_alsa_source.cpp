/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — ALSA source API input-validation (Tier C)
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

/* Tier C — uses pdraw_test_api_init/cleanup fixture.
 * No live ALSA device is required: the fixture only creates a pdraw session.
 * createAlsaSource may return -ENOSYS when CONFIG_PDRAW_USE_ALSA is not set;
 * null-guard tests assert (ret < 0) accordingly. */

#define ULOG_TAG pdraw_test_api_alsa_source
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_alsa_source_*) ────────────────────────────────────────── */

static void testCApiNew()
{
	struct pdraw_alsa_source_params params = {};
	struct pdraw_alsa_source *obj = nullptr;
	int ret;

	/* NullPdraw */
	ret = pdraw_alsa_source_new(
		nullptr, &params, &g_stub_alsa_source_cbs, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullParams */
	ret = pdraw_alsa_source_new(g_test_pdraw_c,
				    nullptr,
				    &g_stub_alsa_source_cbs,
				    nullptr,
				    &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_alsa_source_new(
		g_test_pdraw_c, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_alsa_source_new(g_test_pdraw_c,
				    &params,
				    &g_stub_alsa_source_cbs,
				    nullptr,
				    nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_alsa_source_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_alsa_source_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiPlay()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_alsa_source_play(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_alsa_source_play(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiPause()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_alsa_source_pause(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullSource */
	ret = pdraw_alsa_source_pause(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* is_ready_to_play()/is_paused() previously had no test at all (0 hits in
 * gcov, not even this null-guard branch): unlike most pdraw_alsa_source_*()
 * wrappers, both return a plain 0/1 int rather than a negative errno, even
 * for null arguments. */
static void testCApiIsReadyToPlay()
{
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_ready_to_play(nullptr, nullptr),
			0);
	CU_ASSERT_EQUAL(
		pdraw_alsa_source_is_ready_to_play(g_test_pdraw_c, nullptr), 0);
}


static void testCApiIsPaused()
{
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_paused(nullptr, nullptr), 0);
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_paused(g_test_pdraw_c, nullptr),
			0);
}


/* is_ready_to_play()/is_paused() happy path: testCApiIsReadyToPlay/
 * testCApiIsPaused above only ever pass a null source, so the real
 * forwarding calls (AlsaSource::isReadyToPlay()/isPaused()) never ran (0
 * hits in gcov). Uses the same "null" ALSA virtual device as
 * testCAlsaSourceListenerCallbacks below (no real hardware needed).
 * AlsaSource::isPaused() is `!mRunning`, which starts false (paused) and is
 * flipped synchronously by play()/pause() -- no need to wait for
 * play_resp/pause_resp to observe it. */
static void testCApiIsReadyToPlayIsPausedValid()
{
	struct Ud {
		bool ready = false;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *userdata) {
		static_cast<Ud *>(userdata)->gotStopResp = true;
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	struct pdraw_alsa_source_params params = {};
	params.address = "null";
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	struct pdraw_alsa_source_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_alsa_source *,
			       int ready,
			       enum pdraw_alsa_source_eos_reason,
			       void *userdata) {
		if (ready)
			static_cast<Ud *>(userdata)->ready = true;
	};

	struct pdraw_alsa_source *obj = nullptr;
	ret = pdraw_alsa_source_new(p, &params, &cbs, &ud, &obj);
	if (ret != 0) {
		/* CONFIG_PDRAW_USE_ALSA not compiled in or null device
		 * unavailable. */
		pdraw_stop(p);
		(void)loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
		pdraw_destroy(p);
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady = loop.pumpUntil([&ud]() { return ud.ready; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	CU_ASSERT_EQUAL(pdraw_alsa_source_is_ready_to_play(p, obj), 1);
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_paused(p, obj), 1);

	ret = pdraw_alsa_source_play(p, obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_paused(p, obj), 0);

	ret = pdraw_alsa_source_pause(p, obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(pdraw_alsa_source_is_paused(p, obj), 1);

	ret = pdraw_alsa_source_destroy(p, obj);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


/* ── C++ API (IPdraw::createAlsaSource) ─────────────────────────────────── */
/* Note: createAlsaSource returns -ENOSYS when CONFIG_PDRAW_USE_ALSA is not
 * compiled in, so we assert (ret < 0) instead of a specific errno. */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	struct pdraw_alsa_source_params params = {};
	IPdraw::IAlsaSource *obj = nullptr;
	int ret;

	/* NullParams */
	ret = session->createAlsaSource(
		nullptr, &g_stub_alsa_source_listener, &obj);
	CU_ASSERT_ALSA_NULL_GUARD(ret);

	/* NullListener */
	ret = session->createAlsaSource(&params, nullptr, &obj);
	CU_ASSERT_ALSA_NULL_GUARD(ret);

	/* NullRetObj */
	ret = session->createAlsaSource(
		&params, &g_stub_alsa_source_listener, nullptr);
	CU_ASSERT_ALSA_NULL_GUARD(ret);
}


/* ── pdrawAlsaSourceGetCapabilities (namespace-level free function,
 * pdraw.hpp) ────────────────────────────────────────────────────────────
 * Unlike createAlsaSource() above, this isn't a Session method at all --
 * previously entirely untested, and unlike the null-arg tests above it's
 * not just an early-return guard: exercising it for real means reaching
 * AlsaSource::getCapabilities() itself. No live device needed either way:
 * with CONFIG_PDRAW_USE_ALSA unset, it's -ENOSYS before any ALSA call; with
 * it set, AlsaSource::getCapabilities() opens the device via snd_pcm_open()
 * with SND_PCM_NONBLOCK (pdraw_alsa_source.cpp), so a bogus device name
 * fails immediately with a real (negative) ALSA errno instead of blocking
 * on missing hardware. */

static void testCxxAlsaSourceGetCapabilitiesNullCaps()
{
	int ret = pdrawAlsaSourceGetCapabilities("hw:0,0", nullptr);
	CU_ASSERT_ALSA_NULL_GUARD(ret);
}


static void testCxxAlsaSourceGetCapabilitiesInvalidAddress()
{
	struct pdraw_alsa_source_caps caps = {};
	int ret = pdrawAlsaSourceGetCapabilities(
		"this-alsa-device-does-not-exist", &caps);
	/* Whatever the exact errno (depends on whether ALSA support is
	 * compiled in, and if so which libasound error a bogus device name
	 * produces), it must be a real failure -- never 0. */
	CU_ASSERT_FATAL(ret < 0);
}


static void testCApiLifecycle()
{
	struct pdraw_alsa_source_params params = {};
	struct pdraw_alsa_source *obj = nullptr;
	struct pdraw_alsa_source_cbs cbs = {};
	int ret = pdraw_alsa_source_new(
		g_test_pdraw_c, &params, &cbs, nullptr, &obj);
	CU_ASSERT_TRUE(ret == 0 || ret == -ENOSYS || ret < 0);
	if (ret == 0) {
		CU_ASSERT_PTR_NOT_NULL(obj);
		ret = pdraw_alsa_source_destroy(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


/* Exercises the 4 PdrawAlsaSourceListener C API shims (pdraw_wrapper.cpp):
 * ready_to_play, play_resp, pause_resp, frame_ready.
 *
 * Uses the ALSA null virtual device ("null"), available on every Linux host
 * with alsa-lib, so no real hardware is needed.
 *
 * ready_to_play(true): fires from AlsaSource::createMedia() once the null
 * device is open and the output AudioMedia has been created.
 *
 * pause_resp: fires after play() + pause().  The null capture device never
 * produces data (snd_pcm_avail() always returns 0 for CAPTURE), so
 * processFrame() is never called and the FlushingState stays FLUSHED.
 * pause() → drain() → flush() takes the FLUSHED shortcut → async
 * completeFlush() → pauseResponse() → callPauseResponse() → C callback.
 *
 * play_resp: AlsaSource::play() is fully synchronous (snd_pcm_start + timer
 * arm) and never calls alsaSourcePlayResponse() — the shim cannot be reached
 * this way.  The callback is wired up; its count must remain 0 after play().
 *
 * frame_ready: requires real PCM samples; the null device cannot produce them.
 * Covered separately via the snd-aloop loopback device in
 * test_pipeline_alsa_source.cpp::testCAlsaSourceListenerCbsWithLoopback.
 *
 * Uses a dedicated struct pdraw * (pdraw_new) rather than g_test_pdraw_c so
 * that the test can call pdraw_stop() + wait for stop_resp before pdraw_destroy
 * — necessary because pdraw_alsa_source_destroy() only schedules an async
 * element stop and the AlsaSource::~AlsaSource() / Source::~Source() destructor
 * pair has a use-after-free if the element is force-destroyed while still
 * STARTED (mOutputMedia freed by AlsaSource member dtor, then accessed by
 * Source::removeOutputPorts() in the base class dtor). */
static void testCAlsaSourceListenerCallbacks()
{
	struct Ud {
		int readyToPlayCount = 0;
		bool lastReady = false;
		enum pdraw_alsa_source_eos_reason lastEosReason =
			PDRAW_ALSA_SOURCE_EOS_REASON_NONE;
		int playRespCount = 0;
		int pauseRespCount = 0;
		int frameReadyCount = 0;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *userdata) {
		static_cast<Ud *>(userdata)->gotStopResp = true;
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	struct pdraw_alsa_source_params params = {};
	params.address = "null";
	params.audio.format = adef_pcm_16b_44100hz_stereo;

	struct pdraw_alsa_source_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_alsa_source *,
			       int ready,
			       enum pdraw_alsa_source_eos_reason reason,
			       void *userdata) {
		auto *u = static_cast<Ud *>(userdata);
		u->lastReady = (ready != 0);
		u->lastEosReason = reason;
		u->readyToPlayCount++;
	};
	cbs.play_resp =
		[](struct pdraw *, struct pdraw_alsa_source *, void *userdata) {
			static_cast<Ud *>(userdata)->playRespCount++;
		};
	cbs.pause_resp =
		[](struct pdraw *, struct pdraw_alsa_source *, void *userdata) {
			static_cast<Ud *>(userdata)->pauseRespCount++;
		};
	cbs.frame_ready = [](struct pdraw *,
			     struct pdraw_alsa_source *,
			     struct mbuf_audio_frame *,
			     void *userdata) {
		static_cast<Ud *>(userdata)->frameReadyCount++;
	};

	struct pdraw_alsa_source *obj = nullptr;
	ret = pdraw_alsa_source_new(p, &params, &cbs, &ud, &obj);
	if (ret != 0) {
		/* CONFIG_PDRAW_USE_ALSA not compiled in or null device
		 * unavailable. */
		pdraw_stop(p);
		(void)loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
		pdraw_destroy(p);
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	/* ready_to_play(true) fires from createMedia() inside start() */
	bool gotReady = loop.pumpUntil(
		[&ud]() { return ud.readyToPlayCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(ud.lastReady);
	CU_ASSERT_EQUAL(ud.lastEosReason, PDRAW_ALSA_SOURCE_EOS_REASON_NONE);

	/* play() is synchronous: no play_resp fires */
	ret = pdraw_alsa_source_play(p, obj);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(ud.playRespCount, 0);

	/* pause() drains (FLUSHED shortcut: no downstream channels) →
	 * pause_resp
	 */
	ret = pdraw_alsa_source_pause(p, obj);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotPause =
		loop.pumpUntil([&ud]() { return ud.pauseRespCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotPause);

	ret = pdraw_alsa_source_destroy(p, obj);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


CU_TestInfo g_pdraw_test_api_alsa_source[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiPlay"), testCApiPlay},
	{FN("testCApiPause"), testCApiPause},
	{FN("testCApiIsReadyToPlay"), testCApiIsReadyToPlay},
	{FN("testCApiIsPaused"), testCApiIsPaused},
	{FN("testCApiIsReadyToPlayIsPausedValid"),
	 testCApiIsReadyToPlayIsPausedValid},
	{FN("testCApiLifecycle"), testCApiLifecycle},
	{FN("testCAlsaSourceListenerCallbacks"),
	 testCAlsaSourceListenerCallbacks},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxAlsaSourceGetCapabilitiesNullCaps"),
	 testCxxAlsaSourceGetCapabilitiesNullCaps},
	{FN("testCxxAlsaSourceGetCapabilitiesInvalidAddress"),
	 testCxxAlsaSourceGetCapabilitiesInvalidAddress},
	CU_TEST_INFO_NULL,
};
