/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Session API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_session
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_new / pdraw_stop) ─────────────────────────────── */

static void testCApiNew()
{
	struct pdraw *obj = nullptr;
	int ret;

	/* NullLoop */
	ret = pdraw_new(nullptr, &g_stub_pdraw_cbs, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullCbs */
	ret = pdraw_new(g_test_loop->raw(), nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullRetObj */
	ret = pdraw_new(
		g_test_loop->raw(), &g_stub_pdraw_cbs, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiStop()
{
	int ret = pdraw_stop(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiFriendlyNameSetting()
{
	char buf[64];
	int ret;

	/* NullPdraw */
	ret = pdraw_set_friendly_name_setting(nullptr, "X");
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_get_friendly_name_setting(nullptr, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Roundtrip */
	ret = pdraw_set_friendly_name_setting(g_test_pdraw_c, "CAPI-Friendly");
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_get_friendly_name_setting(g_test_pdraw_c, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(buf, "CAPI-Friendly");

	/* NullStr: no write, no error */
	ret = pdraw_get_friendly_name_setting(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, 0);

	/* BufferTooSmall: len must be > strlen(value) */
	ret = pdraw_get_friendly_name_setting(g_test_pdraw_c, buf, 1);
	CU_ASSERT_EQUAL(ret, -ENOBUFS);
}


static void testCApiSerialNumberSetting()
{
	char buf[64];
	int ret;

	/* NullPdraw */
	ret = pdraw_set_serial_number_setting(nullptr, "X");
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_get_serial_number_setting(nullptr, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Roundtrip */
	ret = pdraw_set_serial_number_setting(g_test_pdraw_c, "SN-CAPI-001");
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_get_serial_number_setting(g_test_pdraw_c, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(buf, "SN-CAPI-001");

	/* NullStr: no write, no error */
	ret = pdraw_get_serial_number_setting(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, 0);

	/* BufferTooSmall */
	ret = pdraw_get_serial_number_setting(g_test_pdraw_c, buf, 1);
	CU_ASSERT_EQUAL(ret, -ENOBUFS);
}


static void testCApiSoftwareVersionSetting()
{
	char buf[64];
	int ret;

	/* NullPdraw */
	ret = pdraw_set_software_version_setting(nullptr, "X");
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = pdraw_get_software_version_setting(nullptr, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Roundtrip */
	ret = pdraw_set_software_version_setting(g_test_pdraw_c, "v9.9.9-capi");
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_get_software_version_setting(
		g_test_pdraw_c, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(buf, "v9.9.9-capi");

	/* NullStr: no write, no error */
	ret = pdraw_get_software_version_setting(g_test_pdraw_c, nullptr, 0);
	CU_ASSERT_EQUAL(ret, 0);

	/* BufferTooSmall */
	ret = pdraw_get_software_version_setting(g_test_pdraw_c, buf, 1);
	CU_ASSERT_EQUAL(ret, -ENOBUFS);
}


static void testCApiDumpPipeline()
{
	int ret;

	/* NullPdraw */
	ret = pdraw_dump_pipeline(nullptr, "/tmp/pdraw_test.dot");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* NullFilename */
	ret = pdraw_dump_pipeline(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Valid */
	ret = pdraw_dump_pipeline(g_test_pdraw_c, "/tmp/pdraw_test_c.dot");
	CU_ASSERT_EQUAL(ret, 0);
}


static void testFixtureRunsOnLoopThread()
{
	CU_ASSERT(pthread_equal(pthread_self(), g_test_loop_thread));
}


/* ── C++ API (createPdraw) ──────────────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *obj = nullptr;
	int ret = createPdraw(nullptr, &g_stub_pdraw_listener, &obj);
	auto objOwner = std::unique_ptr<IPdraw>(obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCxxFriendlyNameSetting()
{
	IPdraw *session = g_test_session->get();
	std::string name;

	session->setFriendlyNameSetting("TestFriendly");
	session->getFriendlyNameSetting(&name);
	CU_ASSERT_STRING_EQUAL(name.c_str(), "TestFriendly");

	/* null output pointer must not crash */
	session->getFriendlyNameSetting(nullptr);
}


static void testCxxSerialNumberSetting()
{
	IPdraw *session = g_test_session->get();
	std::string sn;

	session->setSerialNumberSetting("SN-00-TEST");
	session->getSerialNumberSetting(&sn);
	CU_ASSERT_STRING_EQUAL(sn.c_str(), "SN-00-TEST");

	session->getSerialNumberSetting(nullptr);
}


static void testCxxSoftwareVersionSetting()
{
	IPdraw *session = g_test_session->get();
	std::string ver;

	session->setSoftwareVersionSetting("v1.2.3-test");
	session->getSoftwareVersionSetting(&ver);
	CU_ASSERT_STRING_EQUAL(ver.c_str(), "v1.2.3-test");

	session->getSoftwareVersionSetting(nullptr);
}


static void testCxxDumpPipeline()
{
	IPdraw *session = g_test_session->get();

	/* Writing to /tmp must succeed on any POSIX target */
	int ret = session->dumpPipeline("/tmp/pdraw_test_pipeline.dot");
	CU_ASSERT_EQUAL(ret, 0);
}


/* ── C++ API — Session::stop() idempotency (self-contained fixtures) ────
 * Session::stop() has 3 branches for a session that's already stopping or
 * stopped (pdraw_session.cpp:328-343), never exercised by any other test in
 * this whole suite (every other test calls stop() exactly once): calling it
 * again while already STOPPING is a silent no-op (returns 0, does NOT call
 * stopResp() again), but calling it again once already STOPPED explicitly
 * re-invokes stopResp() -- an asymmetry worth pinning down with a real test.
 * Both need their own local session, not g_test_session (shared by every
 * other test in this file, expected to stay alive/STARTED for the whole
 * suite run -- stopping it here would break them). */

/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class CountingPdrawListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mLastStatus = status;
		mStopCount++;
	}
	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*u*/) override
	{
		/* Only ever one media at a time across the tests using this
		 * listener below -- no ambiguity in just keeping the latest. */
		mMediaId = info->id;
	}
	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}
	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	int mStopCount = 0;
	int mLastStatus = -1;
	unsigned int mMediaId = 0;
};

/* Zero-element session: Session::stop() (pdraw_session.cpp:323) finds
 * mElements empty, so the "stopped" bool it initializes to true never
 * flips false -- it reaches STOPPED synchronously within the very first
 * stop() call (no element-async-stop to wait for). stopResp() itself
 * always defers via idleAdd() though, so a pump is still needed to
 * observe the callback. */
static void testCxxStopOnAlreadyStoppedSessionCallsResponseAgain()
{
	TestPompLoop loop;
	CountingPdrawListener listener;
	TestSession testSession(&loop, &listener);
	IPdraw *session = testSession.get();

	int ret = session->stop();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotFirst = loop.pumpUntil(
		[&listener]() { return listener.mStopCount >= 1; });
	CU_ASSERT_TRUE_FATAL(gotFirst);
	CU_ASSERT_EQUAL(listener.mLastStatus, 0);

	/* Session is now STOPPED: this hits the "already stopped" branch
	 * (pdraw_session.cpp:336-343), which -- unlike calling stop() while
	 * still STOPPING, below -- explicitly calls stopResp() again. */
	ret = session->stop();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotSecond = loop.pumpUntil(
		[&listener]() { return listener.mStopCount >= 2; });
	CU_ASSERT_TRUE_FATAL(gotSecond);
	CU_ASSERT_EQUAL(listener.mLastStatus, 0);
}

/* A real element (an ExternalRawVideoSource -- raw video needs no SPS/PPS,
 * unlike coded video's CodedVideoMedia::setPs(), which does real H.264/H.265
 * bitstream parsing and rejects an empty/dummy parameter set outright, found
 * the hard way) needs at least one loop iteration to actually reach STOPPED,
 * so the session sits in STOPPING for a moment -- unlike the zero-element
 * case above, where stop() reaches STOPPED synchronously within the very
 * first call. Calling stop() again immediately, with no pump in between,
 * must hit the "already STOPPING" branch (pdraw_session.cpp:328-334): a
 * silent no-op that returns 0 WITHOUT calling stopResp() again. */
static void testCxxStopWhileAlreadyStoppingIsNoop()
{
	TestPompLoop loop;
	CountingPdrawListener listener;
	TestSession testSession(&loop, &listener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_raw8;
	sourceParams.video.raw.info.resolution.width = 1;
	sourceParams.video.raw.info.resolution.height = 1;
	sourceParams.video.raw.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_api_session_stop_twice");

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	ret = session->stop();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* No pump yet: the session is still STOPPING (the source hasn't had
	 * a chance to finish its own async stop). */
	ret = session->stop();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotStop = loop.pumpUntil(
		[&listener]() { return listener.mStopCount >= 1; }, 20000);
	CU_ASSERT_TRUE_FATAL(gotStop);
	CU_ASSERT_EQUAL(listener.mLastStatus, 0);
	sourceOwner.reset();

	/* The whole point: exactly one stopResponse, not two -- confirms the
	 * second stop() call while already STOPPING really was a no-op. */
	CU_ASSERT_EQUAL(listener.mStopCount, 1);
}


/* ── C++ API — createXXX() refused while STOPPING/STOPPED ───────────────
 * Session::createXXX() guards 8 creation entry points against a session
 * that is no longer accepting new elements (pdraw_session.cpp):
 * createVideoRenderer, both createDemuxer() overloads that go through a
 * real Session:: implementation (the plain url one, which forwards to the
 * url+mux implementation with mux=nullptr -- pdraw_session.cpp:438-444 --
 * and the distinct localAddr/ports one, pdraw_session.cpp:447),
 * createMuxer, createCodedVideoSink, createRawVideoSink, createAudioSink
 * and createAudioRenderer. Each one logs "<kind> creation refused in %s
 * state" and returns -EPROTO the moment mState is STOPPING or STOPPED --
 * BEFORE constructing the underlying element wrapper -- so none of these
 * tests need working params/assets/hardware: any value that clears the
 * preceding null/empty-string checks is enough to reach the guard. None of
 * this was exercised anywhere in the suite before: every other
 * test_api_*.cpp file only ever calls its createXXX() while the shared
 * g_test_session is READY.
 *
 * By contrast, the other 8 createXXX() entry points (createVipcSource,
 * createCodedVideoSource, createRawVideoSource, createAlsaSource,
 * createAudioSource, createVideoEncoder, createVideoScaler,
 * createAudioEncoder) have NO such guard at all in the current code -- they
 * happily construct (or, for the encoder/scaler, search for a matching
 * media and return -ENOENT) regardless of mState. This looks like a real
 * asymmetry/product gap rather than an intentional design choice, but
 * fixing or further characterizing it is a product-side decision left to
 * the user; not covered by tests here. */

/* Drives a self-contained session into STOPPING (session->stop() called,
 * but its one raw video source element -- same technique as
 * testCxxStopWhileAlreadyStoppingIsNoop above -- needs at least one loop
 * iteration to itself reach STOPPED) and, on demand, on into STOPPED.
 * Reused by every testCxx*CreateRefusedByState test below to avoid
 * repeating the same session/source/stop() boilerplate 8 times. */
class CreateRefusedByStateFixture {
public:
	CreateRefusedByStateFixture() : mTestSession(&mLoop, &mListener)
	{
		IPdraw *session = mTestSession.get();

		struct pdraw_video_source_params sourceParams = {};
		sourceParams.queue_max_count = 0;
		sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
		sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
		sourceParams.video.raw.format = vdef_raw8;
		sourceParams.video.raw.info.resolution.width = 1;
		sourceParams.video.raw.info.resolution.height = 1;
		sourceParams.video.raw.info.bit_depth = 8;
		snprintf(sourceParams.session_meta.friendly_name,
			 sizeof(sourceParams.session_meta.friendly_name),
			 "pdraw_test_create_refused_by_state");

		IPdraw::IRawVideoSource *source = nullptr;
		int ret = session->createRawVideoSource(
			&sourceParams,
			&g_stub_raw_video_source_listener,
			&source);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(source);
		mSourceOwner.reset(source);

		/* Enters STOPPING synchronously; the source hasn't had a
		 * chance yet to finish its own async stop, so the session
		 * does NOT reach STOPPED within this call (see
		 * Session::stop(), pdraw_session.cpp:364-370). */
		ret = session->stop();
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	IPdraw *session() const
	{
		return mTestSession.get();
	}

	/* Pumps until the session actually reaches STOPPED. Call between the
	 * STOPPING assertions and the STOPPED ones. */
	void waitStopped()
	{
		bool gotStop = mLoop.pumpUntil(
			[this]() { return mListener.mStopCount >= 1; }, 20000);
		CU_ASSERT_TRUE_FATAL(gotStop);
	}

private:
	TestPompLoop mLoop;
	CountingPdrawListener mListener;
	TestSession mTestSession;
	std::unique_ptr<IPdraw::IRawVideoSource> mSourceOwner;
};


static void testCxxVideoRendererCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_rect renderPos = {0, 0, 640, 480};
	struct pdraw_video_renderer_params params = {};
	IPdraw::IVideoRenderer *obj = nullptr;

	/* STOPPING */
	int ret = fx.session()->createVideoRenderer(
		0, &renderPos, &params, &g_stub_video_renderer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createVideoRenderer(
		0, &renderPos, &params, &g_stub_video_renderer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxDemuxerFromUrlCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_demuxer_params params = {};
	IPdraw::IDemuxer *obj = nullptr;

	/* STOPPING -- goes through the url+mux implementation (mux=nullptr),
	 * see pdraw_session.cpp:438-444. */
	int ret = fx.session()->createDemuxer(
		"file:///dev/null", &params, &g_stub_demuxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createDemuxer(
		"file:///dev/null", &params, &g_stub_demuxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxDemuxerFromLocalAddrCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_demuxer_params params = {};
	IPdraw::IDemuxer *obj = nullptr;

	/* STOPPING -- the distinct localAddr/ports implementation
	 * (pdraw_session.cpp:447), never reached by the url-based overload
	 * tested above. No address/port validation precedes its guard, so
	 * empty strings and port 0 are enough to reach it. */
	int ret = fx.session()->createDemuxer(
		"", 0, 0, "", 0, 0, &params, &g_stub_demuxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createDemuxer(
		"", 0, 0, "", 0, 0, &params, &g_stub_demuxer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxMuxerCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_muxer_params params = {};
	IPdraw::IMuxer *obj = nullptr;

	/* STOPPING */
	int ret = fx.session()->createMuxer(
		"/tmp/pdraw_test_muxer_state_guard.mp4",
		&params,
		&g_stub_muxer_listener,
		&obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createMuxer("/tmp/pdraw_test_muxer_state_guard.mp4",
					&params,
					&g_stub_muxer_listener,
					&obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxCodedVideoSinkCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_video_sink_params params = {};
	IPdraw::ICodedVideoSink *obj = nullptr;

	/* STOPPING */
	int ret = fx.session()->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createCodedVideoSink(
		0, &params, &g_stub_coded_video_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxRawVideoSinkCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_video_sink_params params = {};
	IPdraw::IRawVideoSink *obj = nullptr;

	/* STOPPING */
	int ret = fx.session()->createRawVideoSink(
		0, &params, &g_stub_raw_video_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createRawVideoSink(
		0, &params, &g_stub_raw_video_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxAudioSinkCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	IPdraw::IAudioSink *obj = nullptr;

	/* STOPPING */
	int ret = fx.session()->createAudioSink(
		0, &g_stub_audio_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createAudioSink(
		0, &g_stub_audio_sink_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


static void testCxxAudioRendererCreateRefusedByState()
{
	CreateRefusedByStateFixture fx;
	struct pdraw_audio_renderer_params params = {};
	params.address = "default";
	IPdraw::IAudioRenderer *obj = nullptr;

	/* STOPPING -- the state guard (pdraw_session.cpp:1005) runs before
	 * the ALSA-gated AudioRendererWrapper construction, so this is
	 * -EPROTO unconditionally, unlike testCxxCreateValid in
	 * test_api_renderer_audio.cpp which depends on PDRAW_USE_ALSA. */
	int ret = fx.session()->createAudioRenderer(
		0, &params, &g_stub_audio_renderer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);

	fx.waitStopped();

	/* STOPPED */
	ret = fx.session()->createAudioRenderer(
		0, &params, &g_stub_audio_renderer_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(obj);
}


/* Session::dumpPipeline() writes a DOT graph via two per-element passes
 * (pdraw_session.cpp:1864-1987): the first pass emits one node per element
 * (with its input/output medias), the second emits an edge for every
 * connected sink input media whose source is found. testCxxDumpPipeline
 * above only exercises this against g_test_session, which is not
 * guaranteed to have any elements connected at that point in a real run --
 * with zero/disconnected elements both per-element loop bodies never do
 * anything meaningful, leaving that logic uncovered despite the test
 * "passing". A real, connected Source -> Sink pair exercises both passes
 * for real: the node pass for two elements, and the edge pass (since the
 * sink's one input media resolves back to the source's output media). */
static void testCxxDumpPipelineWithRealElements()
{
	TestPompLoop loop;
	CountingPdrawListener listener;
	TestSession testSession(&loop, &listener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.queue_max_count = 0;
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_raw8;
	sourceParams.video.raw.info.resolution.width = 1;
	sourceParams.video.raw.info.resolution.height = 1;
	sourceParams.video.raw.info.bit_depth = 8;
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_api_session_dump_pipeline");

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	bool gotMediaAdded = loop.pumpUntil(
		[&listener]() { return listener.mMediaId != 0; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(listener.mMediaId,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	static const char *kDotPath =
		"/tmp/pdraw_test_dump_pipeline_real_elements.dot";
	ret = session->dumpPipeline(kDotPath);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Read the file back: with two real, connected elements, the DOT
	 * graph must contain at least one edge ("->", written by the second
	 * pass) -- proof that pass actually ran, not just that the file was
	 * opened and closed with an empty "digraph {}" body. */
	FILE *f = fopen(kDotPath, "r");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	char buf[4096] = {};
	size_t n = fread(buf, 1, sizeof(buf) - 1, f);
	fclose(f);
	CU_ASSERT_FATAL(n > 0);
	CU_ASSERT_TRUE(strstr(buf, "->") != nullptr);

	sinkOwner.reset();
	sourceOwner.reset();
}


/* ── C-wrapper listener callback tests ───────────────────────────────────
 * Self-contained fixtures: each test creates its own TestPompLoop +
 * pdraw_new() with counting C callbacks (no shared g_test_pdraw_c). */

struct SessionCbState {
	int stopRespCount = 0;
	int mediaAddedCount = 0;
	int mediaRemovedCount = 0;
	unsigned int lastMediaId = 0;
};

static void session_stop_resp_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<SessionCbState *>(ud)->stopRespCount++;
}

static void session_media_added_cb(struct pdraw * /*p*/,
				   const struct pdraw_media_info *info,
				   void * /*elem*/,
				   void *ud)
{
	auto *s = static_cast<SessionCbState *>(ud);
	s->mediaAddedCount++;
	s->lastMediaId = info->id;
}

static void session_media_removed_cb(struct pdraw * /*p*/,
				     const struct pdraw_media_info * /*info*/,
				     void * /*elem*/,
				     void *ud)
{
	static_cast<SessionCbState *>(ud)->mediaRemovedCount++;
}

/* Helper: create a pdraw handle with session counting callbacks.
 * Returns true on success. */
static bool
make_pdraw(TestPompLoop &loop, SessionCbState &state, struct pdraw **out)
{
	struct pdraw_cbs cbs = {};
	cbs.stop_resp = session_stop_resp_cb;
	cbs.media_added = session_media_added_cb;
	cbs.media_removed = session_media_removed_cb;
	int ret = pdraw_new(loop.raw(), &cbs, &state, out);
	return ret == 0 && *out != nullptr;
}

struct RawSrcCbState {
	int flushedCount = 0;
	int drainedCount = 0;
};

static void raw_src_flushed_cb(struct pdraw * /*p*/,
			       struct pdraw_raw_video_source * /*s*/,
			       void *ud)
{
	static_cast<RawSrcCbState *>(ud)->flushedCount++;
}

static void raw_src_drained_cb(struct pdraw * /*p*/,
			       struct pdraw_raw_video_source * /*s*/,
			       void *ud)
{
	static_cast<RawSrcCbState *>(ud)->drainedCount++;
}


static void testCSessionListenerStopResp()
{
	TestPompLoop loop;
	SessionCbState state;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, state, &p));

	int ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);

	bool gotStop =
		loop.pumpUntil([&state]() { return state.stopRespCount >= 1; });
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(state.stopRespCount, 1);

	pdraw_destroy(p);
}


/* Exercises the PdrawListener socket_created C API shim (pdraw_wrapper.cpp).
 *
 * The RTSP TCP socket is created (and socket_created fires) BEFORE connect()
 * is called — so pointing to rtsp://127.0.0.1:1 (port 1, immediately
 * connection-refused on any Linux host) is sufficient: no server is needed.
 *
 * Flow: pdraw_demuxer_new_from_url → element start (idle) → StreamDemuxer::
 * open() → rtsp_client_connect() → socket() → onRtspSocketCreated →
 * Session::socketCreated → PdrawListener C shim → socket_created callback.
 * The connection immediately fails (ECONNREFUSED on port 1), firing open_resp
 * with an error; we wait for that before tearing down cleanly. */
static void testCSessionListenerSocketCreated()
{
	struct Ud {
		int socketCreatedCount = 0;
		bool gotOpenResp = false;
		bool gotCloseResp = false;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs cbs = {};
	cbs.stop_resp = [](struct pdraw *, int, void *u) {
		static_cast<Ud *>(u)->gotStopResp = true;
	};
	cbs.socket_created = [](struct pdraw *, int /*fd*/, void *u) {
		static_cast<Ud *>(u)->socketCreatedCount++;
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &cbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	struct pdraw_demuxer_cbs dmxCbs = {};
	dmxCbs.open_resp =
		[](struct pdraw *, struct pdraw_demuxer *, int, void *u) {
			static_cast<Ud *>(u)->gotOpenResp = true;
		};
	dmxCbs.close_resp =
		[](struct pdraw *, struct pdraw_demuxer *, int, void *u) {
			static_cast<Ud *>(u)->gotCloseResp = true;
		};

	struct pdraw_demuxer_params params = {};
	struct pdraw_demuxer *dmx = nullptr;
	ret = pdraw_demuxer_new_from_url(
		p, "rtsp://127.0.0.1:1/test", &params, &dmxCbs, &ud, &dmx);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dmx);

	/* socket_created fires before connect(): at most one loop tick needed
	 */
	bool gotSocket = loop.pumpUntil(
		[&ud]() { return ud.socketCreatedCount >= 1; }, 3000);
	CU_ASSERT_TRUE(gotSocket);

	/* open_resp fires quickly: port 1 → immediate ECONNREFUSED */
	(void)loop.pumpUntil([&ud]() { return ud.gotOpenResp; }, 5000);

	ret = pdraw_demuxer_close(p, dmx);
	CU_ASSERT_EQUAL(ret, 0);
	(void)loop.pumpUntil([&ud]() { return ud.gotCloseResp; }, 5000);

	ret = pdraw_demuxer_destroy(p, dmx);
	CU_ASSERT_EQUAL(ret, 0);

	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


static void testCSessionListenerMediaAddedRemoved()
{
	TestPompLoop loop;
	SessionCbState state;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, state, &p));

	struct pdraw_video_source_params params = {};
	params.video.format = VDEF_FRAME_TYPE_RAW;
	struct pdraw_raw_video_source_cbs srcCbs = {};
	srcCbs.flushed = raw_src_flushed_cb;
	srcCbs.drained = raw_src_drained_cb;
	RawSrcCbState srcState;
	struct pdraw_raw_video_source *src = nullptr;
	int ret = pdraw_raw_video_source_new(
		p, &params, &srcCbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* Session fires onMediaAdded asynchronously via idle handler */
	bool gotAdded = loop.pumpUntil(
		[&state]() { return state.mediaAddedCount >= 1; });
	CU_ASSERT_TRUE(gotAdded);
	CU_ASSERT_NOT_EQUAL(state.lastMediaId, 0u);

	/* Destroying the source: deleteImplAndEraseListenerUnlocked saves the
	 * impl→wrapper pair in pendingRemovedUserdataMap then erases it from
	 * allListeners.  The Session then fires onMediaRemoved asynchronously;
	 * PdrawListener::onMediaRemoved resolves the wrapper via
	 * pendingRemovedUserdataMap (findWrapperForImpl returns nullptr since
	 * the listener was already removed). */
	pdraw_raw_video_source_destroy(p, src);
	src = nullptr;

	bool gotRemoved = loop.pumpUntil(
		[&state]() { return state.mediaRemovedCount >= 1; });
	CU_ASSERT_TRUE(gotRemoved);

	pdraw_destroy(p);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_api_session[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiStop"), testCApiStop},
	{FN("testCApiFriendlyNameSetting"), testCApiFriendlyNameSetting},
	{FN("testCApiSerialNumberSetting"), testCApiSerialNumberSetting},
	{FN("testCApiSoftwareVersionSetting"), testCApiSoftwareVersionSetting},
	{FN("testCApiDumpPipeline"), testCApiDumpPipeline},
	{FN("testFixtureRunsOnLoopThread"), testFixtureRunsOnLoopThread},
	{FN("testCxxCreate"), testCxxCreate},
	{FN("testCxxFriendlyNameSetting"), testCxxFriendlyNameSetting},
	{FN("testCxxSerialNumberSetting"), testCxxSerialNumberSetting},
	{FN("testCxxSoftwareVersionSetting"), testCxxSoftwareVersionSetting},
	{FN("testCxxDumpPipeline"), testCxxDumpPipeline},
	{FN("testCxxStopOnAlreadyStoppedSessionCallsResponseAgain"),
	 testCxxStopOnAlreadyStoppedSessionCallsResponseAgain},
	{FN("testCxxStopWhileAlreadyStoppingIsNoop"),
	 testCxxStopWhileAlreadyStoppingIsNoop},
	{FN("testCxxVideoRendererCreateRefusedByState"),
	 testCxxVideoRendererCreateRefusedByState},
	{FN("testCxxDemuxerFromUrlCreateRefusedByState"),
	 testCxxDemuxerFromUrlCreateRefusedByState},
	{FN("testCxxDemuxerFromLocalAddrCreateRefusedByState"),
	 testCxxDemuxerFromLocalAddrCreateRefusedByState},
	{FN("testCxxMuxerCreateRefusedByState"),
	 testCxxMuxerCreateRefusedByState},
	{FN("testCxxCodedVideoSinkCreateRefusedByState"),
	 testCxxCodedVideoSinkCreateRefusedByState},
	{FN("testCxxRawVideoSinkCreateRefusedByState"),
	 testCxxRawVideoSinkCreateRefusedByState},
	{FN("testCxxAudioSinkCreateRefusedByState"),
	 testCxxAudioSinkCreateRefusedByState},
	{FN("testCxxAudioRendererCreateRefusedByState"),
	 testCxxAudioRendererCreateRefusedByState},
	{FN("testCxxDumpPipelineWithRealElements"),
	 testCxxDumpPipelineWithRealElements},
	{FN("testCSessionListenerStopResp"), testCSessionListenerStopResp},
	{FN("testCSessionListenerSocketCreated"),
	 testCSessionListenerSocketCreated},
	{FN("testCSessionListenerMediaAddedRemoved"),
	 testCSessionListenerMediaAddedRemoved},
	CU_TEST_INFO_NULL,
};
