/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RtspStreamMuxerMux (libmux transport) unit tests
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

/* White-box tests for RtspStreamMuxerMux (libmux/mux_ip_proxy transport
 * variant of the RTSP stream muxer), mirroring the approach and shared mock
 * used by test_pipeline_demuxer_stream_mux.cpp for the read side
 * (StreamDemuxerMux).
 *
 * RtspStreamMuxerMux is instantiated directly (not via
 * Session::createMuxer()) so tests can drive its internals without running a
 * full RTSP signaling handshake against an embedded server -- that handshake
 * is already covered end-to-end for the transport-agnostic base class by
 * test_pipeline_muxer_stream_rtsp_net.cpp. These tests focus exclusively on
 * the parts of pdraw_muxer_stream_rtsp_mux.cpp that only exist for the libmux
 * transport: setMux()/mux_ref()/mux_unref(), VideoMediaMux's mux_ip_proxy
 * lifecycle (prepareSetup/startRtpAvp/stopRtpAvp/proxy callbacks/dataCb/
 * ctrlCb), and VideoMediaMuxTcp (the TCP-interleaved variant that bypasses
 * mux_ip_proxy entirely).
 *
 * tests/mock_libmux.{hpp,cpp} provides struct mux_ctx, struct mux_ip_proxy,
 * and every mux_ and mux_ip_proxy_ symbol pdraw_muxer_stream_rtsp_mux.cpp
 * calls.
 * g_libmux_mock.auto_trigger_proxy_open (default true) makes mux_ip_proxy_new()
 * synchronously invoke the open callback, so prepareSetup()'s -EINPROGRESS
 * async completion is observable immediately without pumping a real mux
 * tunnel. g_libmux_mock.proxy_new_result injects a mux_ip_proxy_new() failure
 * for the error-path test.
 *
 * Refcount note: libmux_mock_new() returns a mux_ctx with refcount 1 (the
 * caller's own reference). RtspStreamMuxerMux::setMux() takes an additional
 * mux_ref() on construction/setMux(non-null) and releases exactly one
 * mux_unref() on destruction/setMux(nullptr) -- it never touches the
 * caller's original reference. Every test below must therefore call
 * mux_unref(mux) itself after the muxer is destroyed, or the mux_ctx leaks
 * (see test_pipeline_demuxer_stream_mux.cpp / TEST_PROGRESS.md for the same
 * pattern on the demuxer side).
 *
 * Test suite
 * ──────────
 * testCxxStreamMuxerMuxCreateAndDestroy
 *   Construction takes its own mux_ref (mMux == mux); destruction releases
 *   it. Exercises the ctor, setMux(valid), and the dtor's setMux(nullptr).
 *
 * testCxxStreamMuxerMuxCreateVideoMediaUdp
 *   createVideoMedia(UDP) returns a VideoMediaMux. Checks getLowerTransport,
 *   getHeaderExt (Parrot link-type "mux" extension), getHeaderExtCount, and
 *   that all port getters return 0 before any setup (proxy == nullptr
 *   branch).
 *
 * testCxxStreamMuxerMuxCreateVideoMediaTcp
 *   createVideoMedia(TCP) returns a VideoMediaMuxTcp. Exercises this class in
 *   full: getLowerTransport, local ports (always 0), remote port
 *   setters/getters roundtrip, prepareSetup() (no-op, returns 0), and
 *   startRtpAvp() (real createSender()).
 *
 * testCxxStreamMuxerMuxPrepareSetupOpensProxies
 *   prepareSetup() with the mock's default auto-open behavior: returns
 *   -EINPROGRESS, both proxies end up marked open, and the proxy != nullptr
 *   branches of the port getters/setRemote*Port setters become exercised.
 *
 * testCxxStreamMuxerMuxPrepareSetupProxyNewFails
 *   g_libmux_mock.proxy_new_result = -ENOMEM forces mux_ip_proxy_new() to
 *   fail on the first call. Exercises prepareSetup()'s error: cleanup path.
 *
 * testCxxStreamMuxerMuxStartStopRtpAvp
 *   After a successful prepareSetup(), startRtpAvp() creates a real sender
 *   and stopRtpAvp() tears everything down (sender, proxies, sockets).
 *   stopRtpAvp() is called twice to confirm it is idempotent.
 *
 * testCxxStreamMuxerMuxStartRtpAvpNoMux
 *   setMux(nullptr) then startRtpAvp() returns -EPROTO without touching the
 *   proxies/sender.
 *
 * testCxxStreamMuxerMuxProxyCallbacks
 *   Directly invokes the static proxy_cbs (open/close/update/failed) with
 *   the stream proxy, the control proxy, and an unrecognized pointer, to
 *   cover every branch of each callback (including the "unknown proxy" log
 *   paths).
 *
 * testCxxStreamMuxerMuxDataAndCtrlCb
 *   After prepareSetup(), invokes the static dataCb/ctrlCb fd handlers
 *   directly: once for POMP_FD_EVENT_OUT (notifyReadyToSend()) and once for
 *   POMP_FD_EVENT_IN on an otherwise-idle socket (tskt_socket_read_pkt()
 *   returns < 0 immediately, exercising the early-return path).
 *
 * testCxxStreamMuxerMuxDisconnectionReasonAbortedIsNetworkError
 *   White-box regression test for the base class's
 *   computeDisconnectionReason()/registerLastRequest(): after
 *   registerLastRequest() with RTSP_CLIENT_REQ_STATUS_ABORTED (mimicking an
 *   OPTIONS/ANNOUNCE/SETUP/RECORD/TEARDOWN request aborted mid-flight, e.g.
 *   by an underlying mux channel failure), computeDisconnectionReason() must
 *   return NETWORK_ERROR, not UNKNOWN. */

#define ULOG_TAG pdraw_test_pipeline_muxer_stream_rtsp_mux

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>

#include "mock_libmux.hpp"
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#define private public
#define protected public
#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_mux.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#undef protected
#undef private

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

namespace {

/* No-op IMuxer::Listener: these tests drive RtspStreamMuxerMux/VideoMediaMux
 * internals directly rather than through the public async API, so none of
 * these callbacks are expected to fire. */
class DummyMuxerListener : public IPdraw::IMuxer::Listener {
public:
	void onMuxerConnectionStateChanged(
		IPdraw * /*p*/,
		IPdraw::IMuxer * /*m*/,
		enum pdraw_muxer_connection_state /*state*/,
		enum pdraw_muxer_disconnection_reason /*reason*/) override
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
	}

	void muxerCloseResponse(IPdraw * /*p*/,
				IPdraw::IMuxer * /*m*/,
				int /*status*/) override
	{
	}
};

/* Construct a RtspStreamMuxerMux directly (bypassing Session::createMuxer()/
 * MuxerWrapper, same convention as StreamDemuxerMux's test), obtains a fresh
 * mock mux_ctx. Caller owns both the returned muxer and the mux_ctx (must
 * mux_unref(mux) after the muxer is destroyed). */
/* remoteHost defaults to a value distinct from the loopback "url" above: mUrl
 * always holds the loopback tunnel URL for the mux transport (see
 * pdraw_muxer_stream_rtsp_mux.cpp's prepareSetup() comment), so a test that
 * left both at 127.0.0.1 could not tell "the real remote_host was used" apart
 * from "the buggy mUrl->getResolvedHost() leaked through". */
static std::unique_ptr<RtspStreamMuxerMux>
makeMuxerMux(Session *session,
	     TestElementListener *elementListener,
	     DummyMuxerListener *muxerListener,
	     struct mux_ctx *mux,
	     const std::string &url = "rtsp://127.0.0.1/live",
	     const std::string &remoteHost = "203.0.113.10")
{
	struct pdraw_muxer_params params = {};
	return std::make_unique<RtspStreamMuxerMux>(session,
						    elementListener,
						    muxerListener,
						    nullptr,
						    url,
						    mux,
						    remoteHost,
						    &params);
}

/* Send a real UDP datagram to 127.0.0.1:dstPort via a throwaway socket
 * (fire-and-forget, no response expected). Same technique as
 * test_pipeline_demuxer_stream_mux.cpp's sendUdpDatagram(): drives
 * dataCb()/ctrlCb() through the real pomp-loop fd-readable dispatch
 * (VideoMediaMux's sockets are created on the same loop as the test's
 * TestPompLoop, via mMuxerMux->mSession->getLoop()), exercising the real
 * tskt_socket_read_pkt() success path instead of only the early-return
 * ("nothing pending") branch. */
static void sendUdpDatagram(uint16_t dstPort, const uint8_t *data, size_t len)
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(dstPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       data,
	       len,
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}

} /* anonymous namespace */


static void testCxxStreamMuxerMuxCreateAndDestroy()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm.get());
	CU_ASSERT_PTR_EQUAL(sm->mMux, mux);

	/* setMux(nullptr): releases the ref, returns false (invalid handle) */
	bool ok = sm->setMux(nullptr);
	CU_ASSERT_FALSE(ok);
	CU_ASSERT_PTR_NULL(sm->mMux);

	/* setMux(mux) again: re-takes the ref, returns true */
	ok = sm->setMux(mux);
	CU_ASSERT_TRUE(ok);
	CU_ASSERT_PTR_EQUAL(sm->mMux, mux);

	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxCreateVideoMediaUdp()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm.get());

	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vmOwner.get());

	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());
	CU_ASSERT_EQUAL(vm->getLowerTransport(), RTSP_LOWER_TRANSPORT_UDP);

	const struct rtsp_header_ext *ext = vm->getHeaderExt();
	CU_ASSERT_PTR_NOT_NULL_FATAL(ext);
	/* key/value are both const char*: RTSP_HEADER_EXT_PARROT_LINK_TYPE is
	 * a string macro (see rtsp/common.h), not an enum -- compare by
	 * content, not raw pointer identity. */
	CU_ASSERT_STRING_EQUAL(ext->key, RTSP_HEADER_EXT_PARROT_LINK_TYPE);
	CU_ASSERT_STRING_EQUAL(ext->value, "mux");
	CU_ASSERT_EQUAL(vm->getHeaderExtCount(), 1u);

	/* No proxy created yet: every port getter takes the proxy == nullptr
	 * branch and returns 0. */
	CU_ASSERT_EQUAL(vm->getLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(vm->getLocalControlPort(), 0);
	CU_ASSERT_EQUAL(vm->getRemoteStreamPort(), 0);
	CU_ASSERT_EQUAL(vm->getRemoteControlPort(), 0);

	/* setRemote*Port() on a null proxy is a documented no-op (guarded by
	 * "if (mStreamProxy != nullptr)"); just confirm it doesn't crash. */
	vm->setRemoteStreamPort(1234);
	vm->setRemoteControlPort(1235);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxCreateVideoMediaTcp()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm.get());

	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_TCP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vmOwner.get());

	auto *vm = static_cast<RtspStreamMuxerMux::VideoMediaMuxTcp *>(
		vmOwner.get());
	CU_ASSERT_EQUAL(vm->getLowerTransport(), RTSP_LOWER_TRANSPORT_TCP);
	CU_ASSERT_EQUAL(vm->getLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(vm->getLocalControlPort(), 0);

	/* VideoMediaMuxTcp deliberately does not override getStreamSocket()/
	 * getControlSocket(): RTP/RTCP flow over the RTSP TCP interleaved
	 * channel, not a raw tskt_socket, so it relies on VideoMedia's base
	 * nullptr default (see pdraw_muxer_stream_rtsp_video_media.hpp). This
	 * is the only concrete transport that hits that base implementation;
	 * VideoMediaNet and VideoMediaMux (UDP) both override it. */
	CU_ASSERT_PTR_NULL(vm->getStreamSocket());
	CU_ASSERT_PTR_NULL(vm->getControlSocket());

	vm->setRemoteStreamPort(5000);
	vm->setRemoteControlPort(5001);
	CU_ASSERT_EQUAL(vm->getRemoteStreamPort(), 5000);
	CU_ASSERT_EQUAL(vm->getRemoteControlPort(), 5001);

	/* prepareSetup() is a synchronous no-op for TCP interleaved */
	CU_ASSERT_EQUAL(vm->prepareSetup(), 0);

	/* startRtpAvp() creates a real vstrm_sender (no mux tunnel involved) */
	int ret = vm->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxPrepareSetupOpensProxies()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();
	g_libmux_mock.auto_trigger_proxy_open = true;

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm.get());

	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	CU_ASSERT_TRUE(vm->mStreamProxyOpened);
	CU_ASSERT_TRUE(vm->mControlProxyOpened);

	/* Regression lock: the proxies must be told the real remote_host
	 * passed in by the caller (mock default "203.0.113.10"), never the
	 * loopback "url" ("127.0.0.1") used for the mux tunnel's own control
	 * channel -- this is exactly the distinction the historical bug got
	 * wrong (see prepareSetup()'s use of mRemoteHost instead of
	 * mUrl->getResolvedHost()). */
	CU_ASSERT_STRING_EQUAL(vm->mStreamProxy->remote_host.c_str(),
			       "203.0.113.10");
	CU_ASSERT_STRING_EQUAL(vm->mControlProxy->remote_host.c_str(),
			       "203.0.113.10");

	/* Both proxies are now non-null: port getters take the proxy != nullptr
	 * branch (mux_ip_proxy_get_peerport/get_remote_port on the mock's
	 * struct mux_ip_proxy, initialized to 0/redirect_port -- see
	 * mock_libmux.cpp's mux_ip_proxy_new()). */
	(void)vm->getLocalStreamPort();
	(void)vm->getLocalControlPort();
	(void)vm->getRemoteStreamPort();
	(void)vm->getRemoteControlPort();

	/* setRemote*Port() now reaches mux_ip_proxy_set_udp_remote() -- confirm
	 * it re-sends the same real remote_host alongside the new port. */
	vm->setRemoteStreamPort(6000);
	CU_ASSERT_EQUAL(vm->getRemoteStreamPort(), 6000);
	CU_ASSERT_STRING_EQUAL(vm->mStreamProxy->remote_host.c_str(),
			       "203.0.113.10");
	vm->setRemoteControlPort(6001);
	CU_ASSERT_EQUAL(vm->getRemoteControlPort(), 6001);
	CU_ASSERT_STRING_EQUAL(vm->mControlProxy->remote_host.c_str(),
			       "203.0.113.10");

	/* Flush the idle finishSetup() handler scheduled once both proxies
	 * opened (a no-op here: mSdpMedia is never set in this white-box
	 * test, same as the demuxer_mux test's equivalent comment). */
	loop.runOnce();

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxPrepareSetupProxyNewFails()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();
	g_libmux_mock.proxy_new_result = -ENOMEM;

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	CU_ASSERT_PTR_NULL(vm->mStreamProxy);
	CU_ASSERT_PTR_NULL(vm->mControlProxy);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxStartStopRtpAvp()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	loop.runOnce();

	ret = vm->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	ret = vm->stopRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NULL(vm->mStreamProxy);
	CU_ASSERT_PTR_NULL(vm->mControlProxy);
	CU_ASSERT_FALSE(vm->mStreamProxyOpened);
	CU_ASSERT_FALSE(vm->mControlProxyOpened);

	/* Idempotent: calling again on already-torn-down state is safe */
	ret = vm->stopRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxStartRtpAvpNoMux()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	/* Drop the muxer's mux handle: startRtpAvp() must refuse to proceed */
	sm->setMux(nullptr);

	int ret = vm->startRtpAvp();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxProxyCallbacks()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	loop.runOnce();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm->mStreamProxy);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm->mControlProxy);

	/* proxyOpenCb: stream / control / unknown branches */
	RtspStreamMuxerMux::VideoMediaMux::proxyOpenCb(
		vm->mStreamProxy, 1111, vm);
	RtspStreamMuxerMux::VideoMediaMux::proxyOpenCb(
		vm->mControlProxy, 2222, vm);
	RtspStreamMuxerMux::VideoMediaMux::proxyOpenCb(nullptr, 0, vm);

	/* proxyCloseCb: stream / control / unknown branches */
	RtspStreamMuxerMux::VideoMediaMux::proxyCloseCb(vm->mStreamProxy, vm);
	CU_ASSERT_FALSE(vm->mStreamProxyOpened);
	RtspStreamMuxerMux::VideoMediaMux::proxyCloseCb(vm->mControlProxy, vm);
	CU_ASSERT_FALSE(vm->mControlProxyOpened);
	RtspStreamMuxerMux::VideoMediaMux::proxyCloseCb(nullptr, vm);

	/* proxyUpdateCb: currently a no-op (TODO in production code) */
	RtspStreamMuxerMux::VideoMediaMux::proxyUpdateCb(vm->mStreamProxy, vm);

	/* proxyFailedCb: stream / control / unknown branches */
	RtspStreamMuxerMux::VideoMediaMux::proxyFailedCb(
		vm->mStreamProxy, -EIO, vm);
	RtspStreamMuxerMux::VideoMediaMux::proxyFailedCb(
		vm->mControlProxy, -EIO, vm);
	RtspStreamMuxerMux::VideoMediaMux::proxyFailedCb(nullptr, -EIO, vm);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


static void testCxxStreamMuxerMuxDataAndCtrlCb()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	loop.runOnce();

	/* POMP_FD_EVENT_OUT: notifyReadyToSend(). No sender exists yet (only
	 * created by startRtpAvp()), so this exercises notifyReadyToSend()'s
	 * own "no sender" guard rather than a real flush -- still real
	 * coverage of dataCb's OUT branch. */
	RtspStreamMuxerMux::VideoMediaMux::dataCb(-1, POMP_FD_EVENT_OUT, vm);

	/* POMP_FD_EVENT_IN on an idle loopback socket: tskt_socket_read_pkt()
	 * returns < 0 immediately (nothing pending), exercising dataCb's/
	 * ctrlCb's early-return read path. */
	RtspStreamMuxerMux::VideoMediaMux::dataCb(-1, POMP_FD_EVENT_IN, vm);
	RtspStreamMuxerMux::VideoMediaMux::ctrlCb(-1, POMP_FD_EVENT_IN, vm);

	/* Safe no-op with a null userdata (PDRAW_LOG_ERRNO_RETURN_IF guard) */
	RtspStreamMuxerMux::VideoMediaMux::dataCb(
		-1, POMP_FD_EVENT_IN, nullptr);
	RtspStreamMuxerMux::VideoMediaMux::ctrlCb(
		-1, POMP_FD_EVENT_IN, nullptr);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


/* Companion to testCxxStreamMuxerMuxDataAndCtrlCb above: that test only ever
 * calls dataCb()/ctrlCb() with nothing pending on the socket, exercising the
 * early-return "res < 0" branch of tskt_socket_read_pkt() only. This test
 * covers the "readlen != 0" branch instead (the loop body: processDataPkt()/
 * processCtrlPkt(), then newRxPkt()+setRxPkt() to allocate the replacement
 * buffer) -- the same gap that was already closed on the demuxer side by
 * testStreamDemuxerMuxDataCbCtrlCbProcessPacket (test_pipeline_demuxer_
 * stream_mux.cpp) and on the Net muxer side by testCxxStreamMuxerStrayUdp
 * DataIgnored / testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats
 * (test_pipeline_muxer_stream_rtsp_net.cpp).
 *
 * getLocalStreamPort()/getLocalControlPort() return the mux_ip_proxy's peer
 * port, which is 0 here (mux_ip_proxy_get_peerport() reads a field
 * mock_libmux.cpp's mux_ip_proxy_new() never sets) -- NOT the actual local
 * UDP port VideoMediaMux's loopback sockets are bound to. To reach the real
 * socket, read mStreamSock/mControlSock directly (private members, exposed
 * by this file's #define private public trick) and query
 * tskt_socket_get_local_port() on them, the same accessor prepareSetup()
 * itself uses to fill in mux_ip_proxy_info.udp_redirect_port.
 *
 * mRxPkt is checked for pointer identity change as proof the "readlen != 0"
 * branch ran (tpkt_unref + replacement only happens on that path). No
 * startRtpAvp() call here: processDataPkt() is a no-op regardless (the
 * muxer side never needs to read RTP data, only drain the socket) and
 * processCtrlPkt() returns 0 immediately when mSender == nullptr, so a
 * garbage packet is safe either way. */
static void testCxxStreamMuxerMuxDataAndCtrlCbProcessPacket()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	int ret = vm->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	loop.runOnce();

	uint16_t streamPort = tskt_socket_get_local_port(vm->mStreamSock);
	uint16_t controlPort = tskt_socket_get_local_port(vm->mControlSock);
	CU_ASSERT_TRUE_FATAL(streamPort > 0);
	CU_ASSERT_TRUE_FATAL(controlPort > 0);

	/* Minimal RTP-looking packet (12-byte header + a few payload bytes);
	 * content doesn't need to be a valid H.264 NAL, only readlen != 0
	 * matters here. */
	static const uint8_t kRtpPkt[] = {
		0x80,
		0x60,
		0x00,
		0x01, /* V=2, PT=96, seq=1 */
		0x00,
		0x01,
		0x5f,
		0x90, /* timestamp */
		0x00,
		0x00,
		0x00,
		0x01, /* ssrc */
		0x65,
		0x00,
		0x00,
		0x00, /* payload */
	};

	struct tpkt_packet *origStreamPkt = vm->mRxPkt;
	sendUdpDatagram(streamPort, kRtpPkt, sizeof(kRtpPkt));
	bool streamProcessed = loop.pumpUntil(
		[vm, origStreamPkt]() { return vm->mRxPkt != origStreamPkt; },
		2000);
	CU_ASSERT_TRUE(streamProcessed);

	/* Minimal RTCP-like packet (8-byte header, no report blocks); again,
	 * only readlen != 0 matters for dataCb/ctrlCb's own coverage. */
	static const uint8_t kRtcpPkt[] = {
		0x80,
		0xc9,
		0x00,
		0x01, /* V=2, PT=200 (SR), length=1 */
		0x00,
		0x00,
		0x00,
		0x01, /* ssrc */
	};

	struct tpkt_packet *origControlPkt = vm->mRxPkt;
	sendUdpDatagram(controlPort, kRtcpPkt, sizeof(kRtcpPkt));
	bool controlProcessed = loop.pumpUntil(
		[vm, origControlPkt]() { return vm->mRxPkt != origControlPkt; },
		2000);
	CU_ASSERT_TRUE(controlProcessed);

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


/* hasMedia()/clearMedia() are concrete (non-virtual) VideoMedia base-class
 * methods, inherited unchanged by VideoMediaMux; no test exercised them
 * directly before (only RtspStreamMuxer::removeInputMedia() calls them in
 * production, see pdraw_muxer_stream_rtsp.cpp). mVideoMedia is private on
 * the base class, reachable here via this file's #define private public
 * trick. A sentinel non-null pointer stands in for a real Media*: hasMedia()
 * only ever compares pointer identity, it never dereferences it (same
 * sentinel idiom as TestRtspIngestServer::setupCb()'s stream_userdata in
 * test_pipeline_muxer_stream_rtsp_net.cpp). */
static void testCxxStreamMuxerMuxHasMediaAndClearMedia()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	std::unique_ptr<RtspStreamMuxer::VideoMedia> vmOwner =
		sm->createVideoMedia(PDRAW_MUXER_RTSP_TRANSPORT_UDP);
	auto *vm =
		static_cast<RtspStreamMuxerMux::VideoMediaMux *>(vmOwner.get());

	auto *fakeMedia = reinterpret_cast<Media *>(1);

	/* Initial state: mVideoMedia == nullptr */
	CU_ASSERT_TRUE(vm->hasMedia(nullptr));
	CU_ASSERT_FALSE(vm->hasMedia(fakeMedia));

	vm->mVideoMedia = fakeMedia;
	CU_ASSERT_TRUE(vm->hasMedia(fakeMedia));
	CU_ASSERT_FALSE(vm->hasMedia(nullptr));

	vm->clearMedia();
	CU_ASSERT_PTR_NULL(vm->mVideoMedia);
	CU_ASSERT_TRUE(vm->hasMedia(nullptr));

	vmOwner.reset();
	sm.reset();
	mux_unref(mux);
}


/* computeDisconnectionReason()/registerLastRequest() live in the
 * transport-agnostic RtspStreamMuxer base class (pdraw_muxer_stream_rtsp.cpp),
 * not in the libmux-specific code this file otherwise focuses on -- but
 * RtspStreamMuxerMux is the cheapest concrete subclass to instantiate for a
 * white-box test (no embedded RTSP server needed, unlike
 * test_pipeline_muxer_stream_rtsp_net.cpp), so it is exercised here directly
 * via the private/protected-public trick instead of driving a real ABORTED
 * request through the RTSP client.
 * Regression test: a request aborted mid-flight (e.g. by an underlying mux
 * channel failure) used to always report
 * PDRAW_MUXER_DISCONNECTION_REASON_UNKNOWN instead of NETWORK_ERROR, because
 * registerLastRequest() stored the raw RTSP response status (always 0 on
 * ABORTED, since no real response is ever received) instead of the error
 * checkReqStatus() actually derives from req_status (-EPROTO for ABORTED). */
static void testCxxStreamMuxerMuxDisconnectionReasonAbortedIsNetworkError()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	TestElementListener elementListener;
	DummyMuxerListener muxerListener;

	auto sm = makeMuxerMux(session, &elementListener, &muxerListener, mux);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm.get());

	/* Simulate having reached CONNECTED, then an OPTIONS request aborted
	 * mid-flight. */
	sm->mHasBeenConnected = true;
	sm->registerLastRequest(0,
				-EPROTO,
				RTSP_CLIENT_REQ_STATUS_ABORTED,
				RtspStreamMuxer::RtspState::CONNECTED);

	CU_ASSERT_FALSE(sm->mDisconnection.lastReqSucceeded);
	CU_ASSERT_EQUAL(sm->computeDisconnectionReason(),
			PDRAW_MUXER_DISCONNECTION_REASON_NETWORK_ERROR);

	sm.reset();
	mux_unref(mux);
}


CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtsp_mux[] = {
	{FN("testCxxStreamMuxerMuxCreateAndDestroy"),
	 testCxxStreamMuxerMuxCreateAndDestroy},
	{FN("testCxxStreamMuxerMuxCreateVideoMediaUdp"),
	 testCxxStreamMuxerMuxCreateVideoMediaUdp},
	{FN("testCxxStreamMuxerMuxCreateVideoMediaTcp"),
	 testCxxStreamMuxerMuxCreateVideoMediaTcp},
	{FN("testCxxStreamMuxerMuxPrepareSetupOpensProxies"),
	 testCxxStreamMuxerMuxPrepareSetupOpensProxies},
	{FN("testCxxStreamMuxerMuxPrepareSetupProxyNewFails"),
	 testCxxStreamMuxerMuxPrepareSetupProxyNewFails},
	{FN("testCxxStreamMuxerMuxStartStopRtpAvp"),
	 testCxxStreamMuxerMuxStartStopRtpAvp},
	{FN("testCxxStreamMuxerMuxStartRtpAvpNoMux"),
	 testCxxStreamMuxerMuxStartRtpAvpNoMux},
	{FN("testCxxStreamMuxerMuxProxyCallbacks"),
	 testCxxStreamMuxerMuxProxyCallbacks},
	{FN("testCxxStreamMuxerMuxDataAndCtrlCb"),
	 testCxxStreamMuxerMuxDataAndCtrlCb},
	{FN("testCxxStreamMuxerMuxDataAndCtrlCbProcessPacket"),
	 testCxxStreamMuxerMuxDataAndCtrlCbProcessPacket},
	{FN("testCxxStreamMuxerMuxHasMediaAndClearMedia"),
	 testCxxStreamMuxerMuxHasMediaAndClearMedia},
	{FN("testCxxStreamMuxerMuxDisconnectionReasonAbortedIsNetworkError"),
	 testCxxStreamMuxerMuxDisconnectionReasonAbortedIsNetworkError},
	CU_TEST_INFO_NULL,
};
