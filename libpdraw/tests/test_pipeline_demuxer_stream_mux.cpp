/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — StreamDemuxerMux unit tests
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

#define ULOG_TAG pdraw_test_pipeline_demuxer_stream_mux

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <complex>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "mock_libmux.hpp"
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#define private public
#define protected public
#include "pdraw_demuxer_stream_mux.hpp"
#undef protected
#undef private

#include <futils/futils.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

namespace {

class DummyDemuxerListener : public IPdraw::IDemuxer::Listener {
public:
	void demuxerOpenResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status) override
	{
	}
	void demuxerCloseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status) override
	{
	}
	void onDemuxerUnrecoverableError(IPdraw *pdraw,
					 IPdraw::IDemuxer *demuxer) override
	{
	}
	int demuxerSelectMedia(IPdraw *pdraw,
			       IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selected) override
	{
		return 0;
	}
	void demuxerReadyToPlay(IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer,
				bool ready) override
	{
	}
	void onDemuxerEndOfRange(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override
	{
	}
	void demuxerPlayResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
	}
	void demuxerPauseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp) override
	{
	}
	void demuxerSeekResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
	}
};

/* Send a real UDP datagram to 127.0.0.1:dstPort via a throwaway socket
 * (fire-and-forget, no response expected). Used to exercise dataCb()/ctrlCb()
 * through the actual pomp-loop fd-readable dispatch (VideoMediaMux's sockets
 * are created on the same loop as the test's TestPompLoop, via
 * mDemuxerMux->mSession->getLoop()), rather than calling the callbacks
 * directly, so the real tskt_socket_read_pkt() success path is exercised. */
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

static void testStreamDemuxerMuxLifecycle()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	std::string url = "rtsp://127.0.0.1/live";
	struct pdraw_demuxer_params params = {};

	TestElementListener elementListener;
	TestSourceListener sourceListener;
	DummyDemuxerListener demuxerListener;

	/* Instantiate StreamDemuxerMux directly */
	std::unique_ptr<StreamDemuxerMux> demuxer(
		new StreamDemuxerMux(session,
				     &elementListener,
				     &sourceListener,
				     nullptr,
				     &demuxerListener,
				     url,
				     mux,
				     &params));

	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer.get());
	CU_ASSERT_PTR_EQUAL(demuxer->mMux, mux);

	/* Test setMux(nullptr) & setMux(same) */
	bool ok = demuxer->setMux(nullptr);
	CU_ASSERT_FALSE(ok);
	CU_ASSERT_PTR_NULL(demuxer->mMux);

	ok = demuxer->setMux(mux);
	CU_ASSERT_TRUE(ok);
	CU_ASSERT_PTR_EQUAL(demuxer->mMux, mux);

	/* Test createVideoMedia */
	std::unique_ptr<StreamDemuxer::VideoMedia> media =
		demuxer->createVideoMedia(RTSP_LOWER_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media.get());

	/* Cast to VideoMediaMux */
	StreamDemuxerMux::VideoMediaMux *mediaMux =
		static_cast<StreamDemuxerMux::VideoMediaMux *>(media.get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaMux);

	/* Test getLowerTransport, getHeaderExt, getHeaderExtCount */
	CU_ASSERT_EQUAL(mediaMux->getLowerTransport(),
			RTSP_LOWER_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL(mediaMux->getHeaderExt());
	CU_ASSERT_EQUAL(mediaMux->getHeaderExtCount(), 1);

	/* Test startRtpAvp for SessionProtocol == RTSP (no-op) */
	demuxer->mSessionProtocol = StreamDemuxer::SessionProtocol::RTSP;
	int ret = mediaMux->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	/* Test stopRtpAvp for SessionProtocol == RTSP */
	ret = mediaMux->stopRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	/* Test startRtpAvp for SessionProtocol != RTSP */
	demuxer->mSessionProtocol = StreamDemuxer::SessionProtocol::NONE;
	ret = mediaMux->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	/* Feed legacy callbacks */
	struct pomp_buffer *buf = pomp_buffer_new(100);
	CU_ASSERT_PTR_NOT_NULL(buf);

	/* Test legacyDataCb & legacyCtrlCb */
	StreamDemuxerMux::VideoMediaMux::legacyDataCb(
		mux,
		MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
		MUX_CHANNEL_DATA,
		buf,
		mediaMux);
	StreamDemuxerMux::VideoMediaMux::legacyCtrlCb(
		mux,
		MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
		MUX_CHANNEL_DATA,
		buf,
		mediaMux);

	/* Test legacy callbacks with null userdata (safe exits) */
	StreamDemuxerMux::VideoMediaMux::legacyDataCb(
		mux,
		MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
		MUX_CHANNEL_DATA,
		buf,
		nullptr);
	StreamDemuxerMux::VideoMediaMux::legacyCtrlCb(
		mux,
		MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
		MUX_CHANNEL_DATA,
		buf,
		nullptr);

	pomp_buffer_unref(buf);

	/* Test stopRtpAvp for SessionProtocol != RTSP */
	ret = mediaMux->stopRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);

	/* Test prepareSetup() in RTSP mode */
	demuxer->mSessionProtocol = StreamDemuxer::SessionProtocol::RTSP;

	ret = mediaMux->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);

	mediaMux->setRemoteStreamPort(5000);
	mediaMux->setRemoteControlPort(5001);

	/* Test ports getters/setters */
	CU_ASSERT_EQUAL(mediaMux->getLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(mediaMux->getLocalControlPort(), 0);
	CU_ASSERT_EQUAL(mediaMux->getRemoteStreamPort(), 5000);
	CU_ASSERT_EQUAL(mediaMux->getRemoteControlPort(), 5001);

	mediaMux->setLocalStreamPort(1234);
	mediaMux->setLocalControlPort(1235);

	/* Test processDataPkt, processCtrlPkt */
	CU_ASSERT_EQUAL(mediaMux->processDataPkt(nullptr), -ENOSYS);
	CU_ASSERT_EQUAL(mediaMux->processCtrlPkt(nullptr), -ENOSYS);

	/* Test sendCtrl with null packet */
	CU_ASSERT_EQUAL(mediaMux->sendCtrl(nullptr, nullptr), -EINVAL);

	/* Call proxy callbacks directly */
	StreamDemuxerMux::VideoMediaMux::proxyOpenCb(
		mediaMux->mStreamProxy, 1234, mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyOpenCb(
		mediaMux->mControlProxy, 1235, mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyOpenCb(
		nullptr, 0, mediaMux); /* Unknown proxy */

	StreamDemuxerMux::VideoMediaMux::proxyCloseCb(mediaMux->mStreamProxy,
						      mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyCloseCb(mediaMux->mControlProxy,
						      mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyCloseCb(
		nullptr, mediaMux); /* Unknown proxy */

	StreamDemuxerMux::VideoMediaMux::proxyUpdateCb(mediaMux->mStreamProxy,
						       mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyFailedCb(
		mediaMux->mStreamProxy, -EIO, mediaMux);
	StreamDemuxerMux::VideoMediaMux::proxyFailedCb(
		mediaMux->mControlProxy, -EIO, mediaMux);

	/* Call dataCb & ctrlCb to trigger tskt_socket_read_pkt error paths */
	StreamDemuxerMux::VideoMediaMux::dataCb(0, 0, mediaMux);
	StreamDemuxerMux::VideoMediaMux::ctrlCb(0, 0, mediaMux);
	StreamDemuxerMux::VideoMediaMux::dataCb(0, 0, nullptr); /* Safe exit */

	/* Run pomp loop to flush idle finishSetup handlers */
	loop.runOnce();

	/* Test cleanup */
	media.reset();
	demuxer.reset();
	mux_unref(mux);
}

static void testStreamDemuxerMuxPrepareSetupFail()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	std::string url = "rtsp://127.0.0.1/live";
	struct pdraw_demuxer_params params = {};

	TestElementListener elementListener;
	TestSourceListener sourceListener;
	DummyDemuxerListener demuxerListener;

	std::unique_ptr<StreamDemuxerMux> demuxer(
		new StreamDemuxerMux(session,
				     &elementListener,
				     &sourceListener,
				     nullptr,
				     &demuxerListener,
				     url,
				     mux,
				     &params));

	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer.get());

	std::unique_ptr<StreamDemuxer::VideoMedia> media =
		demuxer->createVideoMedia(RTSP_LOWER_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media.get());

	StreamDemuxerMux::VideoMediaMux *mediaMux =
		static_cast<StreamDemuxerMux::VideoMediaMux *>(media.get());

	/* Simulate failure in mux_ip_proxy_new */
	g_libmux_mock.proxy_new_result = -ENOMEM;
	int ret = mediaMux->prepareSetup();
	CU_ASSERT_EQUAL(ret, -ENOMEM);

	media.reset();
	demuxer.reset();
	mux_unref(mux);
}

/* Covers the body of dataCb()/ctrlCb() past tskt_socket_read_pkt() success
 * (pdraw_demuxer_stream_mux.cpp:560-663), previously at 0% coverage: only the
 * "no data pending"/nullptr early-return paths were exercised (see
 * testStreamDemuxerMuxLifecycle above), never the "readlen != 0" branch that
 * allocates a replacement mRxPkt (newRxPkt()) and forwards the packet to the
 * vstrm_receiver.
 *
 * The libmux mock's mux_ip_proxy_new() auto-triggers proxyOpenCb()
 * synchronously (g_libmux_mock.auto_trigger_proxy_open, see mock_libmux.cpp),
 * so right after prepareSetup() both proxies are already marked opened and
 * finishSetup() is scheduled as an idle handler -- one loop.runOnce() flushes
 * it (a no-op here since mSdpMedia is never set in this white-box test).
 *
 * mReceiver is created directly via startRtpAvp() (real production code:
 * for SessionProtocol::RTSP it skips the mux_channel_open() calls -- done via
 * prepareSetup()/proxies instead -- and unconditionally calls
 * createReceiver()), giving a real struct vstrm_receiver without needing a full
 * RTSP/SDP handshake.
 *
 * mRtpPaused is forced to false directly (private member, exposed by this
 * file's private/protected trick): play() would be the "normal" way to clear
 * it, but it dereferences mRangeTimer, which is only allocated by
 * VideoMedia::setup(const sdp_media *) -- never called in this white-box
 * test -- so calling play() here would crash on a null unique_ptr.
 *
 * dataCb()/ctrlCb() are never called directly: mStreamSock/mControlSock are
 * created on the same pomp_loop as the test's TestPompLoop (both wrap the
 * same underlying loop, see TestSession), so a real UDP datagram sent to the
 * bound local port is picked up by the loop's normal fd-readable dispatch,
 * exactly like production traffic. mRxPkt is checked for pointer identity
 * change as proof that the "readlen != 0" branch ran (tpkt_unref +
 * replacement only happens on that path). */
static void testStreamDemuxerMuxDataCbCtrlCbProcessPacket()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	std::string url = "rtsp://127.0.0.1/live";
	struct pdraw_demuxer_params params = {};

	TestElementListener elementListener;
	TestSourceListener sourceListener;
	DummyDemuxerListener demuxerListener;

	std::unique_ptr<StreamDemuxerMux> demuxer(
		new StreamDemuxerMux(session,
				     &elementListener,
				     &sourceListener,
				     nullptr,
				     &demuxerListener,
				     url,
				     mux,
				     &params));
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer.get());

	std::unique_ptr<StreamDemuxer::VideoMedia> media =
		demuxer->createVideoMedia(RTSP_LOWER_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media.get());

	StreamDemuxerMux::VideoMediaMux *mediaMux =
		static_cast<StreamDemuxerMux::VideoMediaMux *>(media.get());

	demuxer->mSessionProtocol = StreamDemuxer::SessionProtocol::RTSP;

	/* Create the real UDP sockets (createSockets()); proxies auto-open */
	int ret = mediaMux->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);

	/* Flush the idle finishSetup() handler scheduled by proxyOpenCb() */
	loop.runOnce();

	/* Create a real vstrm_receiver so dataCb/ctrlCb don't take the
	 * "discard, no receiver yet" continue path */
	ret = mediaMux->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaMux->mReceiver);

	/* Bypass isRtpPaused() (defaults to true until a full play() we can't
	 * drive here) so dataCb/ctrlCb take the "process received packet"
	 * branch instead of the short-circuit */
	mediaMux->mRtpPaused = false;

	uint16_t streamPort = mediaMux->mLocalStreamPort;
	uint16_t controlPort = mediaMux->mLocalControlPort;
	CU_ASSERT_TRUE_FATAL(streamPort > 0);
	CU_ASSERT_TRUE_FATAL(controlPort > 0);

	/* Minimal RTP packet (12-byte header + a few payload bytes); content
	 * doesn't need to be a valid H.264 NAL, only readlen != 0 matters here
	 */
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

	struct tpkt_packet *origStreamPkt = mediaMux->mRxPkt;
	sendUdpDatagram(streamPort, kRtpPkt, sizeof(kRtpPkt));
	bool streamProcessed = loop.pumpUntil(
		[mediaMux, origStreamPkt]() {
			return mediaMux->mRxPkt != origStreamPkt;
		},
		2000);
	CU_ASSERT_TRUE(streamProcessed);

	/* Minimal RTCP-like packet (8-byte header, no report blocks); again,
	 * only readlen != 0 matters for dataCb/ctrlCb's own coverage */
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

	struct tpkt_packet *origControlPkt = mediaMux->mRxPkt;
	sendUdpDatagram(controlPort, kRtcpPkt, sizeof(kRtcpPkt));
	bool controlProcessed = loop.pumpUntil(
		[mediaMux, origControlPkt]() {
			return mediaMux->mRxPkt != origControlPkt;
		},
		2000);
	CU_ASSERT_TRUE(controlProcessed);

	media.reset();
	demuxer.reset();
	mux_unref(mux);
}

/* Companion to testStreamDemuxerMuxDataCbCtrlCbProcessPacket: covers the
 * "else" side of the isRtpPaused() ternary in dataCb()/ctrlCb() (res = 0,
 * vstrm_receiver_recv_data/recv_ctrl NOT called), previously unreached since
 * the only prior calls to dataCb/ctrlCb (testStreamDemuxerMuxLifecycle) never
 * got past tskt_socket_read_pkt() at all (no data was ever sent). mRxPkt is
 * still replaced on this path (the replacement happens unconditionally,
 * before the "if (res < 0)" check), so it remains a valid proof the
 * "readlen != 0" branch ran even with RTP paused. */
static void testStreamDemuxerMuxDataCbCtrlCbPausedSkipsProcessing()
{
	libmux_mock_reset();
	struct mux_ctx *mux = libmux_mock_new();

	TestPompLoop loop;
	TestSession testSession(&loop, nullptr);
	Session *session = testSession.get();

	std::string url = "rtsp://127.0.0.1/live";
	struct pdraw_demuxer_params params = {};

	TestElementListener elementListener;
	TestSourceListener sourceListener;
	DummyDemuxerListener demuxerListener;

	std::unique_ptr<StreamDemuxerMux> demuxer(
		new StreamDemuxerMux(session,
				     &elementListener,
				     &sourceListener,
				     nullptr,
				     &demuxerListener,
				     url,
				     mux,
				     &params));
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer.get());

	std::unique_ptr<StreamDemuxer::VideoMedia> media =
		demuxer->createVideoMedia(RTSP_LOWER_TRANSPORT_UDP);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media.get());

	StreamDemuxerMux::VideoMediaMux *mediaMux =
		static_cast<StreamDemuxerMux::VideoMediaMux *>(media.get());

	demuxer->mSessionProtocol = StreamDemuxer::SessionProtocol::RTSP;

	int ret = mediaMux->prepareSetup();
	CU_ASSERT_EQUAL(ret, -EINPROGRESS);
	loop.runOnce();

	ret = mediaMux->startRtpAvp();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaMux->mReceiver);

	/* mRtpPaused defaults to true (see VideoMedia's mRtpPaused member):
	 * left untouched here on purpose to exercise the paused branch */
	CU_ASSERT_TRUE_FATAL(mediaMux->isRtpPaused());

	uint16_t streamPort = mediaMux->mLocalStreamPort;
	uint16_t controlPort = mediaMux->mLocalControlPort;
	CU_ASSERT_TRUE_FATAL(streamPort > 0);
	CU_ASSERT_TRUE_FATAL(controlPort > 0);

	static const uint8_t kRtpPkt[] = {
		0x80,
		0x60,
		0x00,
		0x01,
		0x00,
		0x01,
		0x5f,
		0x90,
		0x00,
		0x00,
		0x00,
		0x01,
		0x65,
		0x00,
		0x00,
		0x00,
	};
	static const uint8_t kRtcpPkt[] = {
		0x80,
		0xc9,
		0x00,
		0x01,
		0x00,
		0x00,
		0x00,
		0x01,
	};

	struct tpkt_packet *origStreamPkt = mediaMux->mRxPkt;
	sendUdpDatagram(streamPort, kRtpPkt, sizeof(kRtpPkt));
	bool streamProcessed = loop.pumpUntil(
		[mediaMux, origStreamPkt]() {
			return mediaMux->mRxPkt != origStreamPkt;
		},
		2000);
	CU_ASSERT_TRUE(streamProcessed);

	struct tpkt_packet *origControlPkt = mediaMux->mRxPkt;
	sendUdpDatagram(controlPort, kRtcpPkt, sizeof(kRtcpPkt));
	bool controlProcessed = loop.pumpUntil(
		[mediaMux, origControlPkt]() {
			return mediaMux->mRxPkt != origControlPkt;
		},
		2000);
	CU_ASSERT_TRUE(controlProcessed);

	media.reset();
	demuxer.reset();
	mux_unref(mux);
}

CU_TestInfo g_pdraw_test_pipeline_demuxer_stream_mux[] = {
	{(char *)"testCxxStreamDemuxerMuxLifecycle",
	 testStreamDemuxerMuxLifecycle},
	{(char *)"testCxxStreamDemuxerMuxPrepareSetupFail",
	 testStreamDemuxerMuxPrepareSetupFail},
	{(char *)"testCxxStreamDemuxerMuxDataCbCtrlCbProcessPacket",
	 testStreamDemuxerMuxDataCbCtrlCbProcessPacket},
	{(char *)"testCxxStreamDemuxerMuxDataCbCtrlCbPausedSkipsProcessing",
	 testStreamDemuxerMuxDataCbCtrlCbPausedSkipsProcessing},
	CU_TEST_INFO_NULL,
};
