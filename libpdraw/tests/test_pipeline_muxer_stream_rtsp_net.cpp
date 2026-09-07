/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RtspStreamMuxerNet RTSP signaling pipeline (Tier B)
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

/* Tests the RtspStreamMuxer RTSP signaling state machine by embedding a
 * minimal RTSP ingest server on the same pomp loop as the PDrAW session.
 *
 * Because both client (RtspStreamMuxer) and server (TestRtspIngestServer)
 * share the single-threaded pomp loop, each pumpUntil() iteration processes
 * events for both sides, driving the full RTSP handshake without threads.
 * No actual RTP/RTCP packets are exchanged; these tests cover exclusively the
 * RTSP signaling paths in pdraw_muxer_stream_rtsp.cpp.
 *
 * A CodedVideoSource with H.264/AVCC format and static SPS/PPS provides the
 * coded media input. The muxer is created BEFORE the source so that when the
 * source's internalStart() fires onOutputMediaAdded synchronously, the session
 * routes the CodedVideoMedia to the already-existing muxer via addInputMedia().
 * This populates the ANNOUNCE SDP and queues a SETUP request before the RTSP
 * handshake completes.
 *
 * The server is adapted from packages/librtsp/tools/rtsp_server_ingest_test.c.
 *
 * Test suite
 * ──────────
 * testCxxStreamMuxerRtspSignalingLifecycle
 *   Single H.264 media, full ingest handshake:
 *   OPTIONS → ANNOUNCE → SETUP → RECORD → TEARDOWN (on close).
 *   Verifies onMuxerConnectionStateChanged(CONNECTED) after SETUP_DONE.
 *
 * testCxxStreamMuxerAnnounceError
 *   Server returns 404 on ANNOUNCE (mAnnounceShouldFail=true).
 *   Exercises onRtspAnnounceResp FAILED → onUnrecoverableError.
 *
 * testCxxStreamMuxerSetupError
 *   ANNOUNCE succeeds; server returns 404 on SETUP.
 *   Exercises onRtspSetupResp FAILED → onUnrecoverableError.
 *
 * testCxxStreamMuxerRecordError
 *   ANNOUNCE + SETUP succeed; server returns 404 on RECORD.
 *   Exercises onRtspRecordResp FAILED → onUnrecoverableError.
 *   Note: onMuxerConnectionStateChanged(CONNECTED) fires at SETUP_DONE,
 *   before the RECORD response arrives, so mGotConnected may be true.
 *
 * testCxxStreamMuxerSetGetDynParams
 *   setDynParams()/getDynParams() nullptr guards, then a real
 *   socket_tx_buffer_size roundtrip once mRtspClient exists (exercises
 *   rtsp_client_set_socket_txbuf_size()).
 *
 * testCxxStreamMuxerGetStats
 *   getStats() nullptr guard; stats.type/rtsp.is_connected reflect real
 *   state before and after the RTSP handshake completes.
 *
 * testCxxStreamMuxerChannelFlushed / testCxxStreamMuxerChannelDrained
 *   source->flush()/drain() drive RtspStreamMuxer::onChannelFlush /
 *   onChannelDrain (the latter also exercises process() on the drain path),
 *   observed via ICodedVideoSource::Listener's onCodedVideoSourceFlushed/
 *   Drained ack.
 *
 * testCxxStreamMuxerForcedTeardown
 *   rtsp_server_force_teardown(path=NULL) pushes a server-initiated TEARDOWN
 *   for the whole session. Exercises onRtspForcedTeardown's content-base
 *   branch (tears down all VideoMedia); the client library then removes its
 *   own session synchronously in the same call stack, which cascades into
 *   onRtspSessionRemoved -> onUnrecoverableError (observable, unlike the
 *   media teardown itself which has no listener-visible effect).
 *
 * testCxxStreamMuxerInterleavedControlDataUpdatesStats
 *   TCP (interleaved) transport: streams one real frame to capture the
 *   sender's actual (random) SSRC off its RTP header, then
 *   rtsp_server_send_interleaved() on the control channel with a matching
 *   hand-built RTCP Receiver Report drives onRtspInterleavedDataCb ->
 *   VideoMedia::processCtrlPkt -> vstrm_sender_recv_ctrl -> receiverReportCb
 *   -> updateStats() -> notifyVideoMediaStatsUpdate(), observed via
 *   getStats(). Also confirms that packet-loss stats are clamped to 0 for
 *   TCP (see updateStats()'s comment on unreliable RTCP loss/RTT over TCP
 *   interleaving). See TEST_PROGRESS.md for why the SSRC can't just be the
 *   0xABCD1234 TestRtspIngestServer::setupCb() replies with, and for how the
 *   interleaved channel numbers (0=stream, 1=control) were determined to be
 *   deterministic for this single-media test.
 *
 * testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats
 *   UDP transport equivalent of the TCP test above: exercises
 *   VideoMedia::ctrlCb (the RTCP socket's pomp_loop fd handler) instead of
 *   onRtspInterleavedDataCb. The real SSRC is captured by binding a raw UDP
 *   socket directly to the server's declared stream port (5004) and
 *   recv()-ing the actual RTP packet, since there's no TestRtspIngestServer
 *   callback for received RTP over UDP (only interleaved_data, TCP-only).
 *
 * testCxxStreamMuxerGoodbyeTriggersUnrecoverableError
 *   A throwaway RTCP RR latches vstrm_sender's peer_ssrc, then an RTCP BYE
 *   naming that same SSRC drives VideoMedia::goodbyeCb ->
 *   Muxer::onUnrecoverableError(-ENETDOWN). UDP transport.
 *
 * testCxxStreamMuxerTcpBackpressureRecoversViaReadyToSend
 *   TCP transport: shrinks the control socket's SO_SNDBUF to 8KiB
 *   (setDynParams), then pushes a single 2 MiB dummy frame (FU-A-fragmented
 *   into hundreds of RTP packets) to force real, sustained backpressure.
 *   Exercises sendPkt()'s -EAGAIN branch and its recovery via
 *   onReadyToSendCb -> notifyReadyToSend() -> processList(nullptr): a short
 *   pumpUntil() must fail to see full delivery (proves it wasn't
 *   instantaneous), a generous one must eventually see it all arrive
 *   (proves the retry path works). See TEST_PROGRESS.md for why
 *   total_packet_count can't be used to observe this directly.
 *
 * testCxxStreamMuxerAnnounceUnauthorizedRetry
 *   Server replies 401 Unauthorized (-EPERM) to the first ANNOUNCE.
 *   Exercises onRtspAnnounceResp's single-retry branch (mAnnounceRetried):
 *   the muxer re-sends ANNOUNCE once and the handshake still reaches
 *   CONNECTED. See TEST_PROGRESS.md for why the already-queued SETUP
 *   request racing the in-flight retry is safe (rtsp_client_setup()'s
 *   -EBUSY guard).
 *
 * testCxxStreamMuxerServerPushedAnnounce
 *   rtsp_server_announce() pushes an unsolicited ANNOUNCE from the server
 *   after the session is established. Exercises onRtspAnnounce, which only
 *   logs the event; the rtsp_client library auto-replies 200 OK, so the
 *   test just confirms the connection survives and keeps working.
 *
 * testCxxStreamMuxerStrayUdpDataIgnored
 *   UDP transport: a plain UDP socket sends a garbage datagram straight to
 *   the muxer's default local RTP stream port. Exercises
 *   VideoMedia::dataCb's read-drain path (processDataPkt() is a no-op) and
 *   confirms a frame pushed afterwards is still sent normally. */

#define ULOG_TAG pdraw_test_pipeline_muxer_stream_rtsp_net

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <rtsp/server.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/resource.h>
#include <sys/socket.h>

#define private public
#define protected public
#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_net.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#undef protected
#undef private

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

/* Port used by the embedded ingest test server. One above the demuxer test
 * server (18554) to allow both suites to coexist if run in parallel. */
static constexpr uint16_t kTestRtspIngestPort = 18555;
static constexpr const char *kTestRtspIngestPath = "live";

/* Minimal H.264 High-Profile Level 4.0 SPS and PPS — same constants as
 * test_pipeline_demuxer_stream.cpp. Raw NAL-unit bytes, no start code. */
static const uint8_t kH264Sps[] = {
	0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9, 0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
	0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B, 0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};


/* ── Anonymous namespace (internal linkage, avoids ODR violations) ────── */
namespace {


/* ── Embedded in-process RTSP ingest server ───────────────────────────── */

/* Minimal RTSP ingest server for signaling tests, bound to the same pomp
 * loop as the PDrAW session under test.
 *
 * Implements the ingest (publisher) side: ANNOUNCE / SETUP / RECORD /
 * PAUSE / TEARDOWN.  DESCRIBE and PLAY return -ENOSYS (ingest servers do
 * not serve playback requests).
 *
 * Failure-injection flags (set before the server is exercised):
 *   mAnnounceShouldFail, mSetupShouldFail, mRecordShouldFail
 *
 * State-tracking flags (for post-test assertions):
 *   mGotAnnounce, mGotSetup, mGotRecord, mGotTeardown */
class TestRtspIngestServer {
public:
	bool mAnnounceShouldFail = false;
	bool mSetupShouldFail = false;
	bool mRecordShouldFail = false;

	/* When set, the first ANNOUNCE is replied to with 401 Unauthorized
	 * (-EPERM); the second (retried) ANNOUNCE gets the normal reply. See
	 * testCxxStreamMuxerAnnounceUnauthorizedRetry. */
	bool mAnnounceUnauthorizedOnce = false;

	bool mGotAnnounce = false;
	bool mGotSetup = false;
	bool mGotRecord = false;
	bool mGotTeardown = false;

	/* Number of ANNOUNCE requests received so far (>1 means the client
	 * retried). */
	int mAnnounceCount = 0;

	/* Captured from setupCb(); needed by rtsp_server_force_teardown(). */
	std::string mSessionId{};

	/* Captured from interleavedDataCb(); the last packet received on the
	 * TCP-interleaved "stream" channel (0), used to read out the RTP
	 * sender's real (self-generated, see VideoMedia::createSender())
	 * SSRC -- see testCxxStreamMuxerInterleavedControlDataUpdatesStats. */
	std::vector<uint8_t> mLastStreamChannelPacket{};

	/* Running total of payload bytes received on the "stream" channel
	 * (0), across every interleaved_data callback -- unlike
	 * mLastStreamChannelPacket (one packet), this tracks cumulative
	 * delivery of a large, possibly FU-A-fragmented frame. See
	 * testCxxStreamMuxerTcpBackpressureRecoversViaReadyToSend. */
	size_t mStreamChannelBytesReceived = 0;

	TestRtspIngestServer(struct pomp_loop *loop, uint16_t port)
	{
		static const struct rtsp_server_cbs s_cbs = {
			.socket_cb = socketCb,
			.describe = describeCb,
			.setup = setupCb,
			.play = playCb,
			.pause = pauseCb,
			.teardown = teardownCb,
			.request_timeout = requestTimeoutCb,
			.announce = announceCb,
			.record = recordCb,
			.interleaved_data = interleavedDataCb,
		};
		int ret = rtsp_server_new(
			nullptr, port, 0, 0, loop, &s_cbs, this, &mServer);
		if (ret < 0) {
			ULOGW("rtsp_server_new failed (%d) — port %u in use?",
			      ret,
			      port);
			mServer = nullptr;
		}
	}

	~TestRtspIngestServer()
	{
		if (mServer) {
			rtsp_server_destroy(mServer);
			mServer = nullptr;
		}
	}

	bool isStarted() const
	{
		return mServer != nullptr;
	}

	struct rtsp_server *raw() const
	{
		return mServer;
	}

private:
	struct rtsp_server *mServer = nullptr;

	static void socketCb(int /*fd*/, void * /*userdata*/) {}

	static void announceCb(struct rtsp_server *server,
			       const char * /*server_address*/,
			       const char * /*path*/,
			       const char * /*sdp*/,
			       const struct rtsp_header_ext * /*ext*/,
			       size_t /*ext_count*/,
			       void *request_ctx,
			       void *userdata)
	{
		auto *self = static_cast<TestRtspIngestServer *>(userdata);
		self->mGotAnnounce = true;
		self->mAnnounceCount++;
		int ret = 0;
		if (self->mAnnounceShouldFail)
			ret = -ENOENT;
		else if (self->mAnnounceUnauthorizedOnce &&
			 self->mAnnounceCount == 1)
			ret = -EPERM;
		rtsp_server_reply_to_announce(
			server, request_ctx, ret, nullptr, 0);
	}

	static void setupCb(struct rtsp_server *server,
			    const char * /*path*/,
			    const char *session_id,
			    const struct rtsp_header_ext * /*ext*/,
			    size_t /*ext_count*/,
			    void *request_ctx,
			    void *media_ctx,
			    enum rtsp_delivery delivery,
			    enum rtsp_lower_transport lower_transport,
			    const char * /*src_address*/,
			    const char * /*dst_address*/,
			    uint16_t /*dst_stream_port*/,
			    uint16_t /*dst_control_port*/,
			    void *userdata)
	{
		auto *self = static_cast<TestRtspIngestServer *>(userdata);
		self->mGotSetup = true;
		self->mSessionId = session_id ? session_id : "";
		int ret = 0;

		if (self->mSetupShouldFail) {
			ret = -ENOENT;
		} else if (delivery != RTSP_DELIVERY_UNICAST) {
			ret = -ENOSYS;
		} else if (lower_transport != RTSP_LOWER_TRANSPORT_UDP &&
			   lower_transport != RTSP_LOWER_TRANSPORT_TCP) {
			ret = -ENOSYS;
		}

		/* stream_userdata must be non-NULL so record_cb can validate
		 * it. Use (void*)1 as a sentinel (same trick as the demuxer
		 * server). */
		rtsp_server_reply_to_setup(server,
					   request_ctx,
					   media_ctx,
					   ret,
					   5004,
					   5005,
					   1,
					   0xABCD1234u,
					   nullptr,
					   0,
					   reinterpret_cast<void *>(1));
	}

	static void recordCb(struct rtsp_server *server,
			     const char * /*session_id*/,
			     const struct rtsp_header_ext * /*ext*/,
			     size_t /*ext_count*/,
			     void *request_ctx,
			     void *media_ctx,
			     const struct rtsp_range *range,
			     void * /*stream_userdata*/,
			     void *userdata)
	{
		auto *self = static_cast<TestRtspIngestServer *>(userdata);
		self->mGotRecord = true;
		int ret = self->mRecordShouldFail ? -ENOENT : 0;
		struct rtsp_range resp_range = {};
		if (range)
			resp_range = *range;
		rtsp_server_reply_to_record(server,
					    request_ctx,
					    media_ctx,
					    ret,
					    &resp_range,
					    nullptr,
					    0);
	}

	static void pauseCb(struct rtsp_server *server,
			    const char * /*session_id*/,
			    const struct rtsp_header_ext * /*ext*/,
			    size_t /*ext_count*/,
			    void *request_ctx,
			    void *media_ctx,
			    const struct rtsp_range *range,
			    void * /*stream_userdata*/,
			    void * /*userdata*/)
	{
		struct rtsp_range resp_range = {};
		if (range)
			resp_range = *range;
		rtsp_server_reply_to_pause(server,
					   request_ctx,
					   media_ctx,
					   0,
					   &resp_range,
					   nullptr,
					   0);
	}

	static void teardownCb(struct rtsp_server *server,
			       const char * /*path*/,
			       const char * /*session_id*/,
			       enum rtsp_server_teardown_reason /*reason*/,
			       const struct rtsp_header_ext * /*ext*/,
			       size_t /*ext_count*/,
			       void *request_ctx,
			       void *media_ctx,
			       void * /*stream_userdata*/,
			       void *userdata)
	{
		auto *self = static_cast<TestRtspIngestServer *>(userdata);
		self->mGotTeardown = true;
		/* request_ctx is NULL for server-initiated teardowns; only
		 * reply when the client sent the request. */
		if (request_ctx)
			rtsp_server_reply_to_teardown(
				server, request_ctx, media_ctx, 0, nullptr, 0);
	}

	static void describeCb(struct rtsp_server *server,
			       const char * /*server_address*/,
			       const char * /*path*/,
			       const struct rtsp_header_ext * /*ext*/,
			       size_t /*ext_count*/,
			       void *request_ctx,
			       void * /*userdata*/)
	{
		/* Ingest servers do not serve DESCRIBE. */
		rtsp_server_reply_to_describe(
			server, request_ctx, -ENOSYS, nullptr, 0, nullptr);
	}

	static void playCb(struct rtsp_server *server,
			   const char * /*session_id*/,
			   const struct rtsp_header_ext * /*ext*/,
			   size_t /*ext_count*/,
			   void *request_ctx,
			   void *media_ctx,
			   const struct rtsp_range *range,
			   float scale,
			   void * /*stream_userdata*/,
			   void * /*userdata*/)
	{
		/* Ingest servers do not serve PLAY. */
		rtsp_server_reply_to_play(server,
					  request_ctx,
					  media_ctx,
					  -ENOSYS,
					  range,
					  scale,
					  0,
					  0,
					  0,
					  0,
					  nullptr,
					  0);
	}

	static void requestTimeoutCb(struct rtsp_server * /*server*/,
				     void * /*request_ctx*/,
				     enum rtsp_method_type /*method*/,
				     void * /*userdata*/)
	{
	}

	static void interleavedDataCb(struct rtsp_server * /*server*/,
				      struct pomp_conn * /*conn*/,
				      uint8_t channel,
				      const void *data,
				      size_t len,
				      void *userdata)
	{
		auto *self = static_cast<TestRtspIngestServer *>(userdata);
		if (channel != 0 /* stream channel, see VideoMedia setup */)
			return;
		const auto *bytes = static_cast<const uint8_t *>(data);
		self->mLastStreamChannelPacket.assign(bytes, bytes + len);
		self->mStreamChannelBytesReceived += len;
	}
};


/* ── Muxer listener ─────────────────────────────────────────────────────── */

class StreamMuxerListener : public IPdraw::IMuxer::Listener {
public:
	void onMuxerConnectionStateChanged(
		IPdraw * /*p*/,
		IPdraw::IMuxer * /*m*/,
		enum pdraw_muxer_connection_state state,
		enum pdraw_muxer_disconnection_reason /*reason*/) override
	{
		mConnectionState = state;
		if (state == PDRAW_MUXER_CONNECTION_STATE_CONNECTED)
			mGotConnected = true;
	}

	/* Never called for a stream muxer; kept as no-ops for completeness. */
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
				       int status) override
	{
		mErrorStatus = status;
		mGotUnrecoverableError = true;
	}

	void muxerCloseResponse(IPdraw * /*p*/,
				IPdraw::IMuxer * /*m*/,
				int status) override
	{
		mCloseStatus = status;
		mGotCloseResponse = true;
	}

	enum pdraw_muxer_connection_state mConnectionState =
		PDRAW_MUXER_CONNECTION_STATE_UNKNOWN;
	bool mGotConnected = false;
	bool mGotUnrecoverableError = false;
	int mErrorStatus = 0;
	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
};


class TestPdrawListener : public StubPdrawListener {
public:
	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*u*/) override
	{
		mMediaId = info->id;
		mGotMediaAdded = true;
	}

	unsigned int mMediaId = 0;
	bool mGotMediaAdded = false;
};


/* Tracks the flush()/drain() acks fired by the muxer's Sink-side onto its
 * input CodedVideoSource, to observe RtspStreamMuxer::onChannelFlush /
 * onChannelDrain (both otherwise invisible from the public API). */
class TrackingCodedVideoSourceListener
		: public IPdraw::ICodedVideoSource::Listener {
public:
	void
	onCodedVideoSourceFlushed(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*s*/) override
	{
		mGotFlushed = true;
	}

	void
	onCodedVideoSourceDrained(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*s*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* ── Shared helpers ─────────────────────────────────────────────────────── */

/* Create a stream muxer then a H.264/AVCC CodedVideoSource for url.
 *
 * The muxer is created FIRST so it is already present in the session's
 * element list when the source's internalStart() fires onOutputMediaAdded
 * synchronously.  The session then calls addInputMedia() on the muxer, which
 * populates the ANNOUNCE SDP and queues a SETUP request — all before the
 * first RTSP round-trip completes.
 *
 * Returns the muxer handle; *outSource receives ownership of the source. */
static IPdraw::IMuxer *
openStreamMuxer(IPdraw *session,
		TestPompLoop &loop,
		const std::string &url,
		StreamMuxerListener *muxerListener,
		IPdraw::ICodedVideoSource **outSource,
		IPdraw::ICodedVideoSource::Listener *srcListener,
		TestPdrawListener *sessionListener,
		enum pdraw_muxer_rtsp_transport transport =
			PDRAW_MUXER_RTSP_TRANSPORT_UDP)
{
	struct pdraw_muxer_params muxerParams = {};
	muxerParams.rtsp_transport = transport;

	IPdraw::IMuxer *muxer = nullptr;
	int ret =
		session->createMuxer(url, &muxerParams, muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* Create source AFTER the muxer so that onOutputMediaAdded (fired
	 * synchronously from internalStart) routes the CodedVideoMedia to the
	 * already-registered muxer via addInputMedia(). */
	struct pdraw_video_source_params srcParams = {};
	srcParams.video.format = VDEF_FRAME_TYPE_CODED;
	srcParams.video.coded.format = vdef_h264_avcc;
	memcpy(srcParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	srcParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(srcParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	srcParams.video.coded.h264.ppslen = sizeof(kH264Pps);

	ret = session->createCodedVideoSource(
		&srcParams, srcListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	if (sessionListener) {
		bool gotMedia = loop.pumpUntil(
			[sessionListener]() {
				return sessionListener->mGotMediaAdded;
			},
			1000);
		CU_ASSERT_TRUE_FATAL(gotMedia);

		ret = muxer->addMedia(sessionListener->mMediaId, nullptr);
		CU_ASSERT_EQUAL(ret, 0);
	}

	return muxer;
}

/* Close the muxer and pump until muxerCloseResponse fires. */
static void closeStreamMuxer(IPdraw::IMuxer *muxer,
			     TestPompLoop &loop,
			     StreamMuxerListener *listener,
			     int timeoutMs = 10000)
{
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	int ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[listener]() { return listener->mGotCloseResponse; },
		timeoutMs);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);
	muxerOwner.reset();
}


/* ── Tests ─────────────────────────────────────────────────────────────── */

/* Full RTSP ingest lifecycle with a single H.264 media:
 *   CONNECT → OPTIONS → ANNOUNCE → SETUP → RECORD
 *   → onMuxerConnectionStateChanged(CONNECTED) (at SETUP_DONE)
 *   → close() → TEARDOWN → DISCONNECT → muxerCloseResponse(0)
 *
 * Verifies the server received ANNOUNCE, SETUP, RECORD, and TEARDOWN. */
static void testCxxStreamMuxerRtspSignalingLifecycle()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Pump until the handshake reaches SETUP_DONE (fires CONNECTED) or an
	 * unrecoverable error cuts it short. */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);

	CU_ASSERT_TRUE(server.mGotAnnounce);
	CU_ASSERT_TRUE(server.mGotSetup);

	/* RECORD is sent right after SETUP_DONE; pump until it reaches the
	 * server (may still be in flight when CONNECTED fires). */
	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	/* close() sends TEARDOWN; muxerCloseResponse fires after DISCONNECT. */
	closeStreamMuxer(muxer, loop, &muxerListener);

	/* TEARDOWN must have reached the server before muxerCloseResponse. */
	CU_ASSERT_TRUE(server.mGotTeardown);

	sourceOwner.reset();
}


/* ANNOUNCE error (C1):
 *   Server returns -ENOENT in announceCb (mAnnounceShouldFail=true).
 *   onRtspAnnounceResp(FAILED) → onUnrecoverableError → listener fires.
 *   After the error, close() must still complete and fire muxerCloseResponse.
 */
static void testCxxStreamMuxerAnnounceError()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mAnnounceShouldFail = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_FALSE(muxerListener.mGotConnected);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* SETUP error (C2):
 *   ANNOUNCE succeeds; server returns -ENOENT in setupCb
 *   (mSetupShouldFail=true).
 *   onRtspSetupResp(FAILED) → onUnrecoverableError → listener fires.
 *   close() must still complete cleanly. */
static void testCxxStreamMuxerSetupError()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mSetupShouldFail = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_FALSE(muxerListener.mGotConnected);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* RECORD error (C3):
 *   ANNOUNCE + SETUP succeed; server returns -ENOENT in recordCb
 *   (mRecordShouldFail=true).
 *   onRtspRecordResp(FAILED) → onUnrecoverableError → listener fires.
 *
 *   Note: onMuxerConnectionStateChanged(CONNECTED) fires at SETUP_DONE
 *   (inside onRtspSetupResp, before the RECORD round-trip completes), so
 *   mGotConnected may already be true when the error arrives.  The test
 *   only asserts that the error eventually fires. */
static void testCxxStreamMuxerRecordError()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mRecordShouldFail = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotError);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


static struct mbuf_coded_video_frame *
makeDummyH264Frame(uint64_t timestamp, unsigned int index, size_t naluSize = 16)
{
	struct vdef_coded_frame frameInfo = {};
	frameInfo.format.encoding = VDEF_ENCODING_H264;
	frameInfo.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(naluSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memset(data, 0, capacity);

	struct vdef_nalu nalu = {};
	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxStreamMuxerRtspStreaming()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	/* Stream 5 frames */
	for (unsigned int i = 0; i < 5; i++) {
		struct mbuf_coded_video_frame *frame =
			makeDummyH264Frame(i * 33333, i);
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		int ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_coded_video_frame_unref(frame);
	}

	/* Pump loop to process the frames and send them */
	auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds(500);
	(void)loop.pumpUntil(
		[deadline]() {
			return std::chrono::steady_clock::now() >= deadline;
		},
		1000);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* setDynParams()/getDynParams() nullptr guards, then a real
 * socket_tx_buffer_size roundtrip once mRtspClient exists (SETUP_DONE),
 * exercising the rtsp_client_set_socket_txbuf_size() branch. */
static void testCxxStreamMuxerSetGetDynParams()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	CU_ASSERT_EQUAL(muxer->setDynParams(nullptr), -EINVAL);
	CU_ASSERT_EQUAL(muxer->getDynParams(nullptr), -EINVAL);

	/* Kept comfortably below common net.core.wmem_max defaults (e.g.
	 * 212992 on Debian/Ubuntu): rtsp_client_set_socket_txbuf_size()
	 * verifies the kernel actually granted the requested size (doubled,
	 * per Linux SO_SNDBUF semantics) and fails with -ENOSYS if the
	 * kernel silently clamped it, which a too-large value would trigger
	 * here. */
	struct pdraw_muxer_dyn_params dynParams = {};
	dynParams.socket_tx_buffer_size = 131072;
	CU_ASSERT_EQUAL(muxer->setDynParams(&dynParams), 0);

	struct pdraw_muxer_dyn_params outDynParams = {};
	CU_ASSERT_EQUAL(muxer->getDynParams(&outDynParams), 0);
	CU_ASSERT_EQUAL(outDynParams.socket_tx_buffer_size, 131072u);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* getStats() nullptr guard; stats.type/rtsp.is_connected reflect real state
 * before and after the RTSP handshake completes. */
static void testCxxStreamMuxerGetStats()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	CU_ASSERT_EQUAL(muxer->getStats(nullptr), -EINVAL);

	struct pdraw_muxer_stats stats = {};
	CU_ASSERT_EQUAL(muxer->getStats(&stats), 0);
	CU_ASSERT_EQUAL(stats.type, PDRAW_MUXER_TYPE_RTSP);
	CU_ASSERT_FALSE(stats.rtsp.is_connected);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	stats = {};
	CU_ASSERT_EQUAL(muxer->getStats(&stats), 0);
	CU_ASSERT_TRUE(stats.rtsp.is_connected);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* source->flush() drives RtspStreamMuxer::onChannelFlush (a thin passthrough
 * to Muxer::onChannelFlush), observed via the flush ack cascading back to
 * the source's listener. */
static void testCxxStreamMuxerChannelFlushed()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	int ret = source->flush();
	CU_ASSERT_EQUAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&srcListener]() { return srcListener.mGotFlushed; }, 5000);
	CU_ASSERT_TRUE(gotFlushed);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* source->drain() (with a frame pending in the queue) drives
 * RtspStreamMuxer::onChannelDrain, which calls process() to flush remaining
 * frames before acking through Muxer::onChannelDrain. */
static void testCxxStreamMuxerChannelDrained()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame *frame = makeDummyH264Frame(0, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	int ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	ret = source->drain();
	CU_ASSERT_EQUAL(ret, 0);

	bool gotDrained = loop.pumpUntil(
		[&srcListener]() { return srcListener.mGotDrained; }, 5000);
	CU_ASSERT_TRUE(gotDrained);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Server-initiated (forced) TEARDOWN for the whole session (path=NULL):
 * rtsp_server_force_teardown() sends a server->client TEARDOWN whose URI is
 * the session's content base. On the client side (rtsp_client.c,
 * teardown_request_process()), this doesn't match any registered per-media
 * path, so `media` stays NULL: RtspStreamMuxer::onRtspForcedTeardown() takes
 * its pathIsContentBase branch and tears down every VideoMedia; then, in the
 * SAME call stack (still inside teardown_request_process(), right after
 * cbs.teardown() returns), the client library removes its own session
 * entirely and calls cbs.session_removed() ==
 * RtspStreamMuxer::onRtspSessionRemoved(), which -- since the muxer isn't
 * already STOPPING -- calls onUnrecoverableError(). This cascade is the only
 * observable proof available here: tearing down a VideoMedia by itself has
 * no listener-visible effect (no removeInputMedia()/media-removed callback
 * on this path, only on the application-initiated removeInputMedia()). */
static void testCxxStreamMuxerForcedTeardown()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);
	CU_ASSERT_FALSE_FATAL(server.mSessionId.empty());

	int ret = rtsp_server_force_teardown(
		server.raw(), server.mSessionId.c_str(), nullptr, nullptr, 0);
	CU_ASSERT_EQUAL(ret, 0);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_TRUE(server.mGotTeardown);

	/* close() must still complete cleanly after the error, same pattern
	 * as testCxxStreamMuxerAnnounceError/SetupError/RecordError. */
	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Hand-build a minimal RTCP Receiver Report packet (RFC 3550 §6.4.2): a
 * fixed 4-byte header (V=2,P=0,RC=1 / PT=201 / length), the reporter's own
 * SSRC, then a single 24-byte report block. LSR/DLSR are left at 0 (unused
 * by this test: the RTD they'd produce isn't asserted). */
static std::vector<uint8_t> buildRtcpReceiverReport(uint32_t reporterSsrc,
						    uint32_t sourceSsrc,
						    uint8_t fraction,
						    uint32_t lost,
						    uint32_t extHighestSeqnum,
						    uint32_t jitter)
{
	std::vector<uint8_t> buf(32, 0);
	buf[0] = 0x81; /* V=2, P=0, RC=1 */
	buf[1] = 201; /* PT=RR */
	uint16_t length = static_cast<uint16_t>(buf.size() / 4 - 1);
	buf[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(length & 0xFF);
	buf[4] = static_cast<uint8_t>((reporterSsrc >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((reporterSsrc >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((reporterSsrc >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(reporterSsrc & 0xFF);
	/* Report block 1 */
	buf[8] = static_cast<uint8_t>((sourceSsrc >> 24) & 0xFF);
	buf[9] = static_cast<uint8_t>((sourceSsrc >> 16) & 0xFF);
	buf[10] = static_cast<uint8_t>((sourceSsrc >> 8) & 0xFF);
	buf[11] = static_cast<uint8_t>(sourceSsrc & 0xFF);
	buf[12] = fraction;
	buf[13] = static_cast<uint8_t>((lost >> 16) & 0xFF);
	buf[14] = static_cast<uint8_t>((lost >> 8) & 0xFF);
	buf[15] = static_cast<uint8_t>(lost & 0xFF);
	buf[16] = static_cast<uint8_t>((extHighestSeqnum >> 24) & 0xFF);
	buf[17] = static_cast<uint8_t>((extHighestSeqnum >> 16) & 0xFF);
	buf[18] = static_cast<uint8_t>((extHighestSeqnum >> 8) & 0xFF);
	buf[19] = static_cast<uint8_t>(extHighestSeqnum & 0xFF);
	buf[20] = static_cast<uint8_t>((jitter >> 24) & 0xFF);
	buf[21] = static_cast<uint8_t>((jitter >> 16) & 0xFF);
	buf[22] = static_cast<uint8_t>((jitter >> 8) & 0xFF);
	buf[23] = static_cast<uint8_t>(jitter & 0xFF);
	/* lsr (2x u16) and dlsr (u32) left at 0 */
	return buf;
}


/* Build a raw RTCP BYE packet (RFC 3550 §6.6, one SSRC, no reason). Same
 * hand-built style as buildRtcpReceiverReport() above; layout confirmed
 * against test_pipeline_demuxer_stream.cpp's sendRtcpBye(). */
static std::vector<uint8_t> buildRtcpGoodbye(uint32_t ssrc)
{
	std::vector<uint8_t> buf(8, 0);
	buf[0] = 0x81; /* V=2, P=0, SC=1 */
	buf[1] = 203; /* PT=BYE */
	uint16_t length = static_cast<uint16_t>(buf.size() / 4 - 1);
	buf[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(length & 0xFF);
	buf[4] = static_cast<uint8_t>((ssrc >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ssrc >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ssrc >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ssrc & 0xFF);
	return buf;
}


/* TCP (interleaved) transport: rtsp_server_send_interleaved() on the control
 * channel drives RtspStreamMuxer::onRtspInterleavedDataCb ->
 * VideoMedia::processCtrlPkt -> vstrm_sender_recv_ctrl -> receiverReportCb
 * -> updateStats() -> notifyVideoMediaStatsUpdate(), observed via getStats().
 *
 * Channel numbers are deterministic here: with client_stream_port=0 and
 * client_control_port=0 (VideoMedia::createSockets() forces both to 0 in TCP
 * mode), librtsp's is_channel_pair_valid() sees rtp==rtcp==0 and treats the
 * pair as unset, so allocate_interleaved_channels() (rtsp_client.c) assigns
 * the next free pair -- {0, 1} for a fresh client with a single media.
 * rtsp_server_reply_to_setup() echoes the request's own interleaved pair
 * back unconditionally for TCP (rtsp_server.c), so the response always
 * matches: stream=channel 0, control=channel 1.
 *
 * The report's SSRC must equal VideoMedia::mSsrc -- NOT the 0xABCD1234
 * TestRtspIngestServer::setupCb() replies with in the SETUP response.
 * onRtspSetupResp() does tentatively apply that value via setSsrc(), but
 * startRtpAvp() -> createSender() runs right after and overwrites it with
 * vstrm_sender_get_ssrc_self() (vstrm_sender.c: `futils_random32(&self->ssrc)`
 * at vstrm_sender_new() time) -- genuinely random, so the server-suggested
 * SSRC is always discarded in practice. There is no public getter for the
 * real value, so this test streams one real frame first (its RTP packet is
 * sent over the TCP stream channel, captured by
 * TestRtspIngestServer::interleavedDataCb()) and reads the SSRC straight out
 * of the RTP header (bytes 8-11, RFC 3550 §5.1) to build a matching RR. */
static void testCxxStreamMuxerInterleavedControlDataUpdatesStats()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener,
						PDRAW_MUXER_RTSP_TRANSPORT_TCP);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame *frame = makeDummyH264Frame(0, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	int ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool gotDataPkt = loop.pumpUntil(
		[&server]() {
			/* >= 12: RFC 3550 fixed RTP header size. */
			return server.mLastStreamChannelPacket.size() >= 12;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotDataPkt);

	const std::vector<uint8_t> &rtpPkt = server.mLastStreamChannelPacket;
	uint32_t realSsrc = (static_cast<uint32_t>(rtpPkt[8]) << 24) |
			    (static_cast<uint32_t>(rtpPkt[9]) << 16) |
			    (static_cast<uint32_t>(rtpPkt[10]) << 8) |
			    static_cast<uint32_t>(rtpPkt[11]);

	constexpr uint8_t kControlChannel = 1;
	constexpr uint32_t kFakeLost = 5;
	std::vector<uint8_t> rr = buildRtcpReceiverReport(
		/* reporterSsrc */ 0x11223344u,
		/* sourceSsrc */ realSsrc,
		/* fraction */ 128,
		/* lost */ kFakeLost,
		/* extHighestSeqnum */ 100,
		/* jitter */ 900);

	ret = rtsp_server_send_interleaved(
		server.raw(), kControlChannel, rr.data(), rr.size());
	CU_ASSERT_EQUAL(ret, 0);

	struct pdraw_muxer_stats stats = {};
	bool gotReport = loop.pumpUntil(
		[muxer, &stats]() {
			return muxer->getStats(&stats) == 0 &&
			       stats.rtsp.receiver_report_count > 0;
		},
		5000);
	CU_ASSERT_TRUE(gotReport);
	CU_ASSERT_EQUAL(stats.rtsp.receiver_report_count, 1u);
	CU_ASSERT_TRUE(stats.rtsp.is_connected);

	/* Packet-loss stats are clamped to 0 for TCP interleaved transport
	 * (see updateStats()'s comment: RTCP loss/RTT are unreliable over
	 * TCP), despite the non-zero `lost`/`fraction` sent above. */
	CU_ASSERT_EQUAL(stats.rtsp.lost_packet_count, 0u);
	CU_ASSERT_EQUAL(stats.rtsp.lost_fraction, 0.f);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* UDP transport: VideoMedia::ctrlCb is the pomp_loop fd handler for the RTCP
 * control socket (mirrors dataCb, but for RTCP instead of RTP). Unlike
 * dataCb's no-op processDataPkt(), ctrlCb's processCtrlPkt() feeds real RTCP
 * packets to vstrm_sender_recv_ctrl() -- this test exercises that same chain
 * (-> receiverReportCb -> updateStats()) as
 * testCxxStreamMuxerInterleavedControlDataUpdatesStats above, but over
 * genuine UDP instead of TCP interleaved.
 *
 * As with the TCP test, the RR's per-report SSRC must equal the sender's
 * real (random) SSRC -- see that test's comment for why. There is no
 * TestRtspIngestServer callback for received RTP over UDP (only
 * interleaved_data, TCP-only), so instead of adding one, a raw UDP socket is
 * bound directly to the server's declared stream port (5004, see
 * TestRtspIngestServer::setupCb()) *before* the frame is pushed, and used to
 * recv() the real RTP packet and read the SSRC out of its header. */
static void testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	/* Fake RTP receiver on the server's declared stream port (5004); bound
	 * before any frame is sent so the real SSRC can be captured. */
	constexpr uint16_t kServerStreamPort = 5004;
	int rtpSock = socket(AF_INET, SOCK_DGRAM, 0);
	CU_ASSERT_TRUE_FATAL(rtpSock >= 0);
	struct sockaddr_in rtpAddr = {};
	rtpAddr.sin_family = AF_INET;
	rtpAddr.sin_port = htons(kServerStreamPort);
	rtpAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	int bindRet = bind(rtpSock,
			   reinterpret_cast<struct sockaddr *>(&rtpAddr),
			   sizeof(rtpAddr));
	CU_ASSERT_EQUAL_FATAL(bindRet, 0);

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_coded_video_frame *frame = makeDummyH264Frame(0, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	int ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	uint8_t rtpBuf[2048];
	ssize_t rtpLen = -1;
	bool gotRtpPacket = loop.pumpUntil(
		[&]() {
			rtpLen = recv(
				rtpSock, rtpBuf, sizeof(rtpBuf), MSG_DONTWAIT);
			return rtpLen >= 12;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotRtpPacket);
	close(rtpSock);

	uint32_t realSsrc = (static_cast<uint32_t>(rtpBuf[8]) << 24) |
			    (static_cast<uint32_t>(rtpBuf[9]) << 16) |
			    (static_cast<uint32_t>(rtpBuf[10]) << 8) |
			    static_cast<uint32_t>(rtpBuf[11]);

	/* Muxer's default local control port, see
	 * MUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT in
	 * pdraw_muxer_stream_rtsp_video_media.cpp (not exposed to tests, so
	 * duplicated here -- same trick as
	 * testCxxStreamMuxerStrayUdpDataIgnored's
	 * kMuxerDefaultLocalStreamPort). */
	constexpr uint16_t kMuxerDefaultLocalControlPort = 55015;
	std::vector<uint8_t> rr = buildRtcpReceiverReport(
		/* reporterSsrc */ 0x11223344u,
		/* sourceSsrc */ realSsrc,
		/* fraction */ 0,
		/* lost */ 0,
		/* extHighestSeqnum */ 100,
		/* jitter */ 500);

	int ctrlSock = socket(AF_INET, SOCK_DGRAM, 0);
	CU_ASSERT_TRUE_FATAL(ctrlSock >= 0);
	struct sockaddr_in ctrlDst = {};
	ctrlDst.sin_family = AF_INET;
	ctrlDst.sin_port = htons(kMuxerDefaultLocalControlPort);
	inet_pton(AF_INET, "127.0.0.1", &ctrlDst.sin_addr);
	sendto(ctrlSock,
	       rr.data(),
	       rr.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&ctrlDst),
	       sizeof(ctrlDst));
	close(ctrlSock);

	struct pdraw_muxer_stats stats = {};
	bool gotReport = loop.pumpUntil(
		[muxer, &stats]() {
			return muxer->getStats(&stats) == 0 &&
			       stats.rtsp.receiver_report_count > 0;
		},
		5000);
	CU_ASSERT_TRUE(gotReport);
	CU_ASSERT_EQUAL(stats.rtsp.receiver_report_count, 1u);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* VideoMedia::goodbyeCb fires when the sender receives an RTCP BYE (RFC 3550
 * §6.6) whose sole SSRC matches self->peer_ssrc (vstrm_sender.c) -- NOT the
 * muxer's own SSRC. peer_ssrc is latched from the *reporter* SSRC field
 * (bytes 4-7) of any previously-received RTCP RR (or SDES), see
 * vstrm_sender_rtcp_receiver_report_cb(): `self->peer_ssrc = rr->ssrc`. So a
 * throwaway RR (its per-report source SSRC is irrelevant here, unlike
 * testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats above) first
 * establishes peer_ssrc, then the BYE naming that same SSRC triggers
 * goodbyeCb -> Muxer::onUnrecoverableError(-ENETDOWN). UDP transport, same
 * default local control port as the RR test above. */
static void testCxxStreamMuxerGoodbyeTriggersUnrecoverableError()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	constexpr uint32_t kPeerSsrc = 0xCAFEBABEu;
	constexpr uint16_t kMuxerDefaultLocalControlPort = 55015;

	int ctrlSock = socket(AF_INET, SOCK_DGRAM, 0);
	CU_ASSERT_TRUE_FATAL(ctrlSock >= 0);
	struct sockaddr_in ctrlDst = {};
	ctrlDst.sin_family = AF_INET;
	ctrlDst.sin_port = htons(kMuxerDefaultLocalControlPort);
	inet_pton(AF_INET, "127.0.0.1", &ctrlDst.sin_addr);

	std::vector<uint8_t> rr = buildRtcpReceiverReport(
		kPeerSsrc, /* sourceSsrc */ 0, 0, 0, 0, 0);
	sendto(ctrlSock,
	       rr.data(),
	       rr.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&ctrlDst),
	       sizeof(ctrlDst));

	/* Let the loop process the RR (latching peer_ssrc) before the BYE. */
	(void)loop.pumpUntil([]() { return false; }, 200);

	std::vector<uint8_t> bye = buildRtcpGoodbye(kPeerSsrc);
	sendto(ctrlSock,
	       bye.data(),
	       bye.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&ctrlDst),
	       sizeof(ctrlDst));
	close(ctrlSock);

	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		5000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT_EQUAL(muxerListener.mErrorStatus, -ENETDOWN);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* TCP transport: forces a real, sustained TCP send backpressure condition on
 * the RTSP control connection (shared with interleaved RTP/RTCP), to
 * exercise VideoMedia::sendPkt()'s -EAGAIN branch and its recovery via
 * RtspStreamMuxer::onReadyToSendCb -> notifyReadyToSend() ->
 * processList(nullptr).
 *
 * There is no public counter for "sendPkt() returned -EAGAIN" (unlike
 * dropped/lost packet stats, which cover different failure modes) --
 * total_packet_count (vstrm_sender's stat) increments at RTP packetization
 * time in vstrm_sender_send_frame(), *before* processList() ever attempts to
 * write anything, so it cannot distinguish "queued" from "actually
 * delivered". Instead, this test observes real bytes arriving at the
 * server's TCP socket (TestRtspIngestServer::mStreamChannelBytesReceived,
 * summed across every interleaved_data callback on the stream channel).
 *
 * To make backpressure deterministic regardless of the machine's default
 * kernel socket buffers (unlike TCP send buffers being possibly huge on
 * CI/build servers we don't control), setDynParams() first shrinks the RTSP
 * client's own control-socket SO_SNDBUF down to a small, known value (same
 * rtsp_client_set_socket_txbuf_size() mechanism as
 * testCxxStreamMuxerSetGetDynParams, just with a much smaller target this
 * time). A single multi-megabyte dummy frame (FU-A-fragmented into many RTP
 * packets by vstrm_sender) then guarantees the shrunk send buffer fills
 * before delivery completes: a FIXED, small number of non-blocking loop
 * iterations must fail to see full delivery (proving it wasn't instantaneous,
 * i.e. -EAGAIN really happened and something had to retry), while a generous
 * pumpUntil() must eventually see it all arrive (proving onReadyToSendCb's
 * retry path actually works).
 *
 * NOTE (found via a real run that flaked on this exact assertion): the first
 * check used to be a short (50ms) pumpUntil() instead of a fixed iteration
 * count. That is a wall-clock race, not a deterministic bound: client and
 * server share this same single-threaded loop with no real network latency,
 * so pumpUntil()'s internal pomp_loop_wait_and_process(mLoop, 20) can return
 * near-instantly whenever a packet is ready and cycle through dozens of
 * write/drain/retry round trips inside a single wall-clock millisecond on a
 * fast enough machine -- exactly what let the whole 2MiB drain inside 50ms
 * here. A FIXED iteration count doesn't have that problem: the amount of
 * data that can possibly move in N loop passes is bounded by the actual
 * kernel SO_SNDBUF capacity per round trip, not by how fast each pass
 * executes, so N stays a safe, CPU-speed-independent upper bound. */
static void testCxxStreamMuxerTcpBackpressureRecoversViaReadyToSend()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener,
						PDRAW_MUXER_RTSP_TRANSPORT_TCP);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	/* Shrink the control socket's SO_SNDBUF down to 8KiB (well below the
	 * 131072 already proven to work in testCxxStreamMuxerSetGetDynParams,
	 * and comfortably above any realistic kernel minimum floor). */
	struct pdraw_muxer_dyn_params dynParams = {};
	dynParams.socket_tx_buffer_size = 8192;
	int ret = muxer->setDynParams(&dynParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	/* 2 MiB single NALU: FU-A-fragmented into hundreds of RTP packets,
	 * many times larger than the shrunk 8KiB send buffer. */
	constexpr size_t kNaluSize = 2 * 1024 * 1024;
	struct mbuf_coded_video_frame *frame =
		makeDummyH264Frame(0, 0, kNaluSize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	/* Must NOT all arrive after a fixed, small number of non-blocking loop
	 * passes: proves delivery wasn't instantaneous, i.e. sendPkt() really
	 * hit -EAGAIN partway through. 10 passes is comfortably enough to get
	 * past the source->sink hop and into the real processList()/sendPkt()
	 * burst, yet nowhere near enough to drain 2MiB through the 8KiB-
	 * shrunk send buffer even under a generous per-pass throughput
	 * assumption (10 x 64KiB = 640KiB << kNaluSize). */
	for (int i = 0; i < 10; i++)
		loop.runOnce();
	CU_ASSERT_FALSE(server.mStreamChannelBytesReceived >= kNaluSize);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);

	/* Must eventually all arrive: proves onReadyToSendCb's retry path
	 * (notifyReadyToSend() -> processList(nullptr)) drains the rest. */
	bool deliveredEventually = loop.pumpUntil(
		[&server]() {
			return server.mStreamChannelBytesReceived >= kNaluSize;
		},
		15000);
	CU_ASSERT_TRUE(deliveredEventually);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Server replies 401 Unauthorized (-EPERM) to the first ANNOUNCE.
 * onRtspAnnounceResp() retries once (mAnnounceRetried) by re-sending the
 * ANNOUNCE; the retry succeeds, and the handshake proceeds normally. This
 * races the (still in-flight) retried ANNOUNCE against the SETUP request
 * that was already queued by addInputMedia(): processRtspRequests() runs
 * immediately after the retry is sent (state is set to ANNOUNCE_DONE right
 * away, before the retry's response arrives), but rtsp_client_setup() itself
 * returns -EBUSY while a request is still pending, so processSetupRequest()
 * defers it (returns 0 without popping the queue) until the retry's response
 * triggers processRtspRequests() again. No dedicated product change needed:
 * the -EBUSY guard in librtsp already makes this safe. */
static void testCxxStreamMuxerAnnounceUnauthorizedRetry()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mAnnounceUnauthorizedOnce = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);
	CU_ASSERT_EQUAL(server.mAnnounceCount, 2);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* rtsp_server_announce() pushes an unsolicited, server-initiated ANNOUNCE to
 * the client once the session is established. RtspStreamMuxer::onRtspAnnounce
 * only logs the event (no state change); the rtsp_client library itself
 * auto-replies 200 OK (see RTSP_METHOD_TYPE_ANNOUNCE in
 * rtsp_client_request_process()), so the only observable effect from the
 * muxer side is that the connection survives and keeps working afterwards. */
static void testCxxStreamMuxerServerPushedAnnounce()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	char pushedUri[] = "live";
	char pushedSdp[] = "v=0\r\n";
	int ret = rtsp_server_announce(
		server.raw(), pushedUri, nullptr, 0, pushedSdp);
	CU_ASSERT_EQUAL(ret, 0);

	/* Let the loop deliver the pushed ANNOUNCE to the client and its
	 * automatic 200 OK reply back to the server. */
	(void)loop.pumpUntil([]() { return false; }, 200);

	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);
	CU_ASSERT_EQUAL(muxerListener.mConnectionState,
			PDRAW_MUXER_CONNECTION_STATE_CONNECTED);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* VideoMedia::dataCb is the pomp_loop fd handler for the UDP "stream" socket
 * used to send RTP (see createSockets()). This socket is bind()-only, never
 * connect()-ed (see socket_impl_read_pkt() in libtransport-socket), so it
 * accepts datagrams from *any* source, not just the negotiated RTSP server.
 * Nothing meaningful is ever expected to arrive there -- processDataPkt() is
 * a no-op -- but the read loop must drain a stray datagram without
 * disrupting the stream. Send one from a plain UDP socket to the muxer's
 * default local stream port (see MUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT in
 * pdraw_muxer_stream_rtsp_video_media.cpp; not exposed to tests otherwise,
 * so duplicated here) and confirm a frame pushed afterwards is still
 * delivered normally. */
static void testCxxStreamMuxerStrayUdpDataIgnored()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE(gotConnected);
	CU_ASSERT_TRUE(muxerListener.mGotConnected);

	bool gotRecord =
		loop.pumpUntil([&server]() { return server.mGotRecord; }, 5000);
	CU_ASSERT_TRUE(gotRecord);

	struct mbuf_coded_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);

	auto pushFrame = [&](unsigned int index) {
		struct mbuf_coded_video_frame *frame =
			makeDummyH264Frame(index * 33333, index);
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		int ret = mbuf_coded_video_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_coded_video_frame_unref(frame);
	};

	pushFrame(0);

	struct pdraw_muxer_stats stats = {};
	bool gotFirstPacket = loop.pumpUntil(
		[muxer, &stats]() {
			return muxer->getStats(&stats) == 0 &&
			       stats.rtsp.total_packet_count > 0;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFirstPacket);
	uint32_t countBefore = stats.rtsp.total_packet_count;

	constexpr uint16_t kMuxerDefaultLocalStreamPort = 55014;
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	CU_ASSERT_TRUE_FATAL(sock >= 0);
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(kMuxerDefaultLocalStreamPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	static const uint8_t garbage[4] = {0xDE, 0xAD, 0xBE, 0xEF};
	sendto(sock,
	       garbage,
	       sizeof(garbage),
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);

	/* Let the loop drain the stray datagram via dataCb() first. */
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);

	pushFrame(1);

	bool gotSecondPacket = loop.pumpUntil(
		[muxer, &stats, countBefore]() {
			return muxer->getStats(&stats) == 0 &&
			       stats.rtsp.total_packet_count > countBefore;
		},
		5000);
	CU_ASSERT_TRUE(gotSecondPacket);
	CU_ASSERT_FALSE(muxerListener.mGotUnrecoverableError);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


} /* anonymous namespace */


static void testCxxStreamMuxerSetThumbnailAndFileMetadataReturnEnosys()
{
	/* Muxer::setThumbnail() and Muxer::setFileMetadata() return -ENOSYS
	 * in the base class (pdraw_muxer.cpp lines 592-611).
	 * RtspStreamMuxer does NOT override either method, so calling them
	 * on a stream muxer exercises those base-class stubs.
	 * (RecordMuxer overrides both, so a record muxer cannot be used here.)
	 */
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* setThumbnail — base Muxer returns -ENOSYS */
	const uint8_t thumb[] = {0xFF, 0xD8, 0xFF}; /* fake JPEG header */
	int ret = muxer->setThumbnail(
		PDRAW_MUXER_THUMBNAIL_TYPE_JPEG, thumb, sizeof(thumb));
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* setFileMetadata — base Muxer returns -ENOSYS */
	struct pdraw_muxer_metadata_params meta = {};
	ret = muxer->setFileMetadata(&meta, nullptr, 0);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Caps this process' fd table so only `extraAllowed` more fds can be
 * allocated, then returns the ORIGINAL limit for the caller to restore via
 * setrlimit() once done. Makes the (extraAllowed+1)-th subsequent socket()/
 * open() call fail with EMFILE -- deterministic and independent of both
 * process privilege (unlike a port <1024, which only fails to bind() for a
 * non-root user) and tskt_impl's own EADDRINUSE-retry-with-ephemeral-port
 * fallback (tskt_impl.c:907-913, "XXX this is crappy behaviour", which
 * silently defeats a port-collision-based approach: bind() failing
 * EADDRINUSE just makes it retry with port 0 instead of returning an
 * error). socket() itself (tskt_impl.c:784) has no such fallback. */
static struct rlimit capFdsAllowingNMore(unsigned int extraAllowed)
{
	struct rlimit origLimit;
	CU_ASSERT_EQUAL_FATAL(getrlimit(RLIMIT_NOFILE, &origLimit), 0);

	int probeFd = open("/dev/null", O_RDONLY);
	CU_ASSERT_TRUE_FATAL(probeFd >= 0);
	close(probeFd);

	struct rlimit tinyLimit = {static_cast<rlim_t>(probeFd) + extraAllowed,
				   origLimit.rlim_max};
	CU_ASSERT_EQUAL_FATAL(setrlimit(RLIMIT_NOFILE, &tinyLimit), 0);

	return origLimit;
}


/* Covers: RtspStreamMuxer::VideoMedia::createSockets() control-socket
 * allocation failure (l.920-932) and the error: cleanup label (l.945-953).
 *
 * Allowing exactly one more fd to be allocated lets the first
 * tskt_socket_new() (stream, l.876) succeed and the second (control,
 * l.920) fail with EMFILE, jump to goto error (l.931), which calls
 * tskt_socket_destroy() on the open stream socket (l.946) and resets both
 * socket pointers to nullptr.
 */
static void testCxxStreamMuxerCreateSocketsControlFails()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotConnected);
	CU_ASSERT_TRUE_FATAL(muxerListener.mGotConnected);

	Pdraw::MuxerWrapper *wrapper =
		static_cast<Pdraw::MuxerWrapper *>(muxer);
	Pdraw::RtspStreamMuxer *sm =
		static_cast<Pdraw::RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm);

	CU_ASSERT_FALSE_FATAL(sm->mVideoMedias.empty());
	Pdraw::RtspStreamMuxerNet::VideoMediaNet *vm =
		static_cast<Pdraw::RtspStreamMuxerNet::VideoMediaNet *>(
			sm->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	if (vm->mStrm.sock != nullptr) {
		tskt_socket_destroy(vm->mStrm.sock);
		vm->mStrm.sock = nullptr;
	}
	if (vm->mCtrl.sock != nullptr) {
		tskt_socket_destroy(vm->mCtrl.sock);
		vm->mCtrl.sock = nullptr;
	}

	vm->mStrm.localPort = 0;
	vm->mCtrl.localPort = 0;

	struct rlimit origLimit = capFdsAllowingNMore(1);

	int res = vm->createSockets();
	CU_ASSERT_NOT_EQUAL(res, 0);
	CU_ASSERT_PTR_NULL(vm->mStrm.sock);
	CU_ASSERT_PTR_NULL(vm->mCtrl.sock);

	CU_ASSERT_EQUAL(setrlimit(RLIMIT_NOFILE, &origLimit), 0);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Covers: RtspStreamMuxer::VideoMedia::createSockets() stream-socket allocation
 * failure (l.876-892).
 *
 * Allowing zero more fds to be allocated makes the very first
 * tskt_socket_new() at line 876 fail with EMFILE, jumping to goto error.
 */
static void testCxxStreamMuxerCreateSocketsStreamFails()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotConnected);
	CU_ASSERT_TRUE_FATAL(muxerListener.mGotConnected);

	Pdraw::MuxerWrapper *wrapper =
		static_cast<Pdraw::MuxerWrapper *>(muxer);
	Pdraw::RtspStreamMuxer *sm =
		static_cast<Pdraw::RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm);

	CU_ASSERT_FALSE_FATAL(sm->mVideoMedias.empty());
	Pdraw::RtspStreamMuxerNet::VideoMediaNet *vm =
		static_cast<Pdraw::RtspStreamMuxerNet::VideoMediaNet *>(
			sm->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	if (vm->mStrm.sock != nullptr) {
		tskt_socket_destroy(vm->mStrm.sock);
		vm->mStrm.sock = nullptr;
	}
	if (vm->mCtrl.sock != nullptr) {
		tskt_socket_destroy(vm->mCtrl.sock);
		vm->mCtrl.sock = nullptr;
	}

	vm->mStrm.localPort = 0;
	vm->mCtrl.localPort = 0;

	struct rlimit origLimit = capFdsAllowingNMore(0);

	int res = vm->createSockets();
	CU_ASSERT_NOT_EQUAL(res, 0);
	CU_ASSERT_PTR_NULL(vm->mStrm.sock);
	CU_ASSERT_PTR_NULL(vm->mCtrl.sock);

	CU_ASSERT_EQUAL(setrlimit(RLIMIT_NOFILE, &origLimit), 0);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


static struct mbuf_coded_video_frame *
makeCodedFrame(enum vdef_coded_data_format dataFormat,
	       const uint8_t *naluData,
	       size_t naluSize)
{
	struct vdef_coded_frame frameInfo = {};
	if (dataFormat == VDEF_CODED_DATA_FORMAT_HVCC)
		frameInfo.format.encoding = VDEF_ENCODING_HEVC;
	else if (dataFormat == VDEF_CODED_DATA_FORMAT_JFIF)
		frameInfo.format.encoding = VDEF_ENCODING_JPEG;
	else
		frameInfo.format.encoding = VDEF_ENCODING_H264;
	frameInfo.format.data_format = dataFormat;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 1000;
	frameInfo.info.capture_timestamp = 1000;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(naluSize, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	void *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memcpy(data, naluData, naluSize);

	struct vdef_nalu nalu = {};
	nalu.size = naluSize;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;

	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	Pdraw::CodedVideoMedia::Frame meta = {};
	meta.isSync = true;

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&meta,
		sizeof(meta));
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


/* Covers: RtspStreamMuxer::VideoMedia::processFrame() data_format switch cases
 * (l.615-636):
 * - VDEF_CODED_DATA_FORMAT_RAW_NALU (start_code_size = 0)
 * - VDEF_CODED_DATA_FORMAT_BYTE_STREAM (h264_get_start_code_length)
 * - VDEF_CODED_DATA_FORMAT_BYTE_STREAM error (invalid start code)
 * - default: unsupported data format (-ENOSYS)
 */
static void testCxxStreamMuxerProcessFrameDataFormats()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotConnected);
	CU_ASSERT_TRUE_FATAL(muxerListener.mGotConnected);

	bool gotRecord = loop.pumpUntil(
		[&server]() { return server.mGotRecord; }, 10000);
	CU_ASSERT_TRUE_FATAL(gotRecord);

	Pdraw::MuxerWrapper *wrapper =
		static_cast<Pdraw::MuxerWrapper *>(muxer);
	Pdraw::RtspStreamMuxer *sm =
		static_cast<Pdraw::RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm);

	/* Ensure recording state is active on client side */
	sm->mRecording = true;

	CU_ASSERT_FALSE_FATAL(sm->mVideoMedias.empty());
	Pdraw::RtspStreamMuxer::VideoMedia *vm = sm->mVideoMedias.front().get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	/* 1. VDEF_CODED_DATA_FORMAT_RAW_NALU -> start_code_size = 0 (l.616) */
	uint8_t rawNalu[16] = {0x65, 0x88, 0x84, 0x00};
	struct mbuf_coded_video_frame *frameRaw = makeCodedFrame(
		VDEF_CODED_DATA_FORMAT_RAW_NALU, rawNalu, sizeof(rawNalu));
	int res = vm->processFrame(frameRaw);
	CU_ASSERT_EQUAL(res, 0);
	mbuf_coded_video_frame_unref(frameRaw);

	/* 2. VDEF_CODED_DATA_FORMAT_BYTE_STREAM with valid 4-byte start code
	 * (l.622) */
	uint8_t byteStreamNalu[16] = {
		0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00};
	struct mbuf_coded_video_frame *frameByteStream =
		makeCodedFrame(VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
			       byteStreamNalu,
			       sizeof(byteStreamNalu));
	res = vm->processFrame(frameByteStream);
	CU_ASSERT_EQUAL(res, 0);
	mbuf_coded_video_frame_unref(frameByteStream);

	/* 3. VDEF_CODED_DATA_FORMAT_BYTE_STREAM with invalid start code (l.625
	 * get_start_code_length error) */
	uint8_t invalidByteStream[16] = {0xFF, 0xFF, 0xFF, 0xFF};
	struct mbuf_coded_video_frame *frameBadByteStream =
		makeCodedFrame(VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
			       invalidByteStream,
			       sizeof(invalidByteStream));
	res = vm->processFrame(frameBadByteStream);
	CU_ASSERT_NOT_EQUAL(res, 0);
	mbuf_coded_video_frame_unref(frameBadByteStream);

	/* 4. default: unsupported data format (l.632) -> returns -ENOSYS (using
	 * JFIF format) */
	struct mbuf_coded_video_frame *frameUnsupported = makeCodedFrame(
		VDEF_CODED_DATA_FORMAT_JFIF, rawNalu, sizeof(rawNalu));
	res = vm->processFrame(frameUnsupported);
	CU_ASSERT_EQUAL(res, -ENOSYS);
	mbuf_coded_video_frame_unref(frameUnsupported);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Covers: RtspStreamMuxer::VideoMedia socket error paths and cleanup:
 * - sendPkt() nullptr guards and NULL socket handling (l.1132, l.1158)
 * - setRemoteStreamPort() and setRemoteControlPort()
 * - startRtpAvp() "only one socket created" bad state (-EPROTO) (l.280-285)
 * - stopRtpAvp() destroying sockets (l.308-323)
 * - sendCtrlCb() error handling with nullptr socket (l.1163)
 */
static void testCxxStreamMuxerSocketErrorsAndCleanup()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotConnected);
	CU_ASSERT_TRUE_FATAL(muxerListener.mGotConnected);

	Pdraw::MuxerWrapper *wrapper =
		static_cast<Pdraw::MuxerWrapper *>(muxer);
	Pdraw::RtspStreamMuxer *sm =
		static_cast<Pdraw::RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm);

	CU_ASSERT_FALSE_FATAL(sm->mVideoMedias.empty());
	Pdraw::RtspStreamMuxerNet::VideoMediaNet *vm =
		static_cast<Pdraw::RtspStreamMuxerNet::VideoMediaNet *>(
			sm->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	/* 1. sendPkt nullptr packet guard (l.1132) -> returns -EINVAL */
	int res = vm->sendPkt(nullptr, 0, vm->mStrm.sock, "strm");
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* 2. sendPkt nullptr socket guard in UDP mode (l.1158) -> returns
	 * -EINVAL */
	uint8_t rawNalu[16] = {0x65, 0x88, 0x84, 0x00};
	struct mbuf_coded_video_frame *frame = makeCodedFrame(
		VDEF_CODED_DATA_FORMAT_RAW_NALU, rawNalu, sizeof(rawNalu));
	struct tpkt_packet *pkt = nullptr;
	struct pomp_buffer *buf = pomp_buffer_new(100);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = tpkt_new_from_buffer(buf, &pkt);
	pomp_buffer_unref(buf);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vm->sendPkt(pkt, 0, nullptr, "strm");
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* 3. Test setRemoteStreamPort / setRemoteControlPort while sockets are
	 * active */
	vm->setRemoteStreamPort(5004);
	vm->setRemoteControlPort(5005);

	/* 4. Destroy control socket, leaving only stream socket -> triggers bad
	 * state in startRtpAvp (l.280) */
	if (vm->mCtrl.sock != nullptr) {
		tskt_socket_destroy(vm->mCtrl.sock);
		vm->mCtrl.sock = nullptr;
	}

	/* sendCtrlCb with nullptr control socket -> returns -EINVAL */
	res = vm->sendCtrlCb(nullptr, pkt, vm);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vm->startRtpAvp();
	CU_ASSERT_EQUAL(res, -EPROTO);
	CU_ASSERT_PTR_NULL(vm->mStrm.sock);
	CU_ASSERT_PTR_NULL(vm->mCtrl.sock);

	/* 5. stopRtpAvp on already stopped sockets is safe */
	res = vm->stopRtpAvp();
	CU_ASSERT_EQUAL(res, 0);

	/* 6. Callback error and early return branches (dataCb / ctrlCb) */
	vm->dataCb(0, 0, nullptr);
	vm->dataCb(0, 0, vm);
	vm->dataCb(0, POMP_FD_EVENT_OUT, vm);
	vm->dataCb(0, POMP_FD_EVENT_IN, vm);

	vm->ctrlCb(0, 0, nullptr);
	vm->ctrlCb(0, 0, vm);
	vm->ctrlCb(0, POMP_FD_EVENT_IN, vm);

	tpkt_unref(pkt);
	mbuf_coded_video_frame_unref(frame);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


/* Covers: RtspStreamMuxer::VideoMedia::processList() socket write error paths
 * (l.766-781):
 * - res == -ENETUNREACH / -ENETDOWN: logs only once and sets mNetdownLogged =
 * true (l.767-774)
 * - res == -EINVAL / other error: logs error and resets mNetdownLogged = false
 * (l.775-778)
 */
static void testCxxStreamMuxerProcessListWriteError()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	TestRtspIngestServer server(loop.raw(), kTestRtspIngestPort);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspIngestPort) + "/" +
			  kTestRtspIngestPath;

	StreamMuxerListener muxerListener;
	StubCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openStreamMuxer(session,
						loop,
						url,
						&muxerListener,
						&source,
						&srcListener,
						&sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotConnected ||
			       muxerListener.mGotUnrecoverableError;
		},
		10000);
	CU_ASSERT_TRUE_FATAL(gotConnected);
	CU_ASSERT_TRUE_FATAL(muxerListener.mGotConnected);

	Pdraw::MuxerWrapper *wrapper =
		static_cast<Pdraw::MuxerWrapper *>(muxer);
	Pdraw::RtspStreamMuxer *sm =
		static_cast<Pdraw::RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sm);

	CU_ASSERT_FALSE_FATAL(sm->mVideoMedias.empty());
	Pdraw::RtspStreamMuxerNet::VideoMediaNet *vm =
		static_cast<Pdraw::RtspStreamMuxerNet::VideoMediaNet *>(
			sm->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	/* 1. Set remote address to an unroutable IP so tskt_socket_write_pkt
	 * returns res < 0 (hitting l.766-780 error logging and jump to next).
	 */
	int res = tskt_socket_set_remote(vm->mStrm.sock, "192.0.2.1", 5004);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	struct tpkt_list *list1 = nullptr;
	res = tpkt_list_new(&list1);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	struct tpkt_packet *pkt1 = nullptr;
	struct pomp_buffer *buf1 = pomp_buffer_new(100);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf1);
	void *bufData1 = nullptr;
	size_t len1 = 0;
	size_t capacity1 = 0;
	res = pomp_buffer_get_data(buf1, &bufData1, &len1, &capacity1);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf1, &pkt1);
	pomp_buffer_unref(buf1);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_len(pkt1, 50);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = tpkt_list_add_last(list1, pkt1);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_unref(pkt1);

	/* First write error: hits l.766-780 (write error -> logged and unref'd
	 * cleanly) */
	res = vm->processList(list1);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_list_destroy(list1);

	/* Second write error: hits l.766-780 again with mNetdownLogged state */
	struct tpkt_list *list2 = nullptr;
	res = tpkt_list_new(&list2);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	struct tpkt_packet *pkt2 = nullptr;
	struct pomp_buffer *buf2 = pomp_buffer_new(100);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf2);
	void *bufData2 = nullptr;
	size_t len2 = 0;
	size_t capacity2 = 0;
	res = pomp_buffer_get_data(buf2, &bufData2, &len2, &capacity2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf2, &pkt2);
	pomp_buffer_unref(buf2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_len(pkt2, 50);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = tpkt_list_add_last(list2, pkt2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_unref(pkt2);

	res = vm->processList(list2);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_list_destroy(list2);

	/* 2. Destroy stream socket so sendPkt returns -EINVAL (hits else branch
	 * l.775-778) */
	if (vm->mStrm.sock != nullptr) {
		tskt_socket_destroy(vm->mStrm.sock);
		vm->mStrm.sock = nullptr;
	}

	struct tpkt_list *list3 = nullptr;
	res = tpkt_list_new(&list3);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	struct tpkt_packet *pkt3 = nullptr;
	struct pomp_buffer *buf3 = pomp_buffer_new(100);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf3);
	res = tpkt_new_from_buffer(buf3, &pkt3);
	pomp_buffer_unref(buf3);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = tpkt_list_add_last(list3, pkt3);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_unref(pkt3);

	res = vm->processList(list3);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_FALSE(vm->mNetdownLogged);
	tpkt_list_destroy(list3);

	closeStreamMuxer(muxer, loop, &muxerListener);
	sourceOwner.reset();
}


static void testCxxStreamMuxerRtspAddInputMediaNull()
{
	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtsp://127.0.0.1:554/live/stream";

	struct pdraw_muxer_params muxerParams = {};
	StreamMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret =
		session->createMuxer(url, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *rtspMuxer = static_cast<RtspStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rtspMuxer);

	/* Call addInputMedia with null media */
	ret = rtspMuxer->addInputMedia(nullptr);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	closeStreamMuxer(muxer, loop, &muxerListener);
}


CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtsp_net[] = {
	{FN("testCxxStreamMuxerRtspSignalingLifecycle"),
	 testCxxStreamMuxerRtspSignalingLifecycle},
	{FN("testCxxStreamMuxerRtspStreaming"),
	 testCxxStreamMuxerRtspStreaming},
	{FN("testCxxStreamMuxerAnnounceError"),
	 testCxxStreamMuxerAnnounceError},
	{FN("testCxxStreamMuxerSetupError"), testCxxStreamMuxerSetupError},
	{FN("testCxxStreamMuxerRecordError"), testCxxStreamMuxerRecordError},
	{FN("testCxxStreamMuxerSetGetDynParams"),
	 testCxxStreamMuxerSetGetDynParams},
	{FN("testCxxStreamMuxerGetStats"), testCxxStreamMuxerGetStats},
	{FN("testCxxStreamMuxerSetThumbnailAndFileMetadataReturnEnosys"),
	 testCxxStreamMuxerSetThumbnailAndFileMetadataReturnEnosys},
	{FN("testCxxStreamMuxerChannelFlushed"),
	 testCxxStreamMuxerChannelFlushed},
	{FN("testCxxStreamMuxerChannelDrained"),
	 testCxxStreamMuxerChannelDrained},
	{FN("testCxxStreamMuxerForcedTeardown"),
	 testCxxStreamMuxerForcedTeardown},
	{FN("testCxxStreamMuxerInterleavedControlDataUpdatesStats"),
	 testCxxStreamMuxerInterleavedControlDataUpdatesStats},
	{FN("testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats"),
	 testCxxStreamMuxerUdpCtrlReceiverReportUpdatesStats},
	{FN("testCxxStreamMuxerGoodbyeTriggersUnrecoverableError"),
	 testCxxStreamMuxerGoodbyeTriggersUnrecoverableError},
	{FN("testCxxStreamMuxerTcpBackpressureRecoversViaReadyToSend"),
	 testCxxStreamMuxerTcpBackpressureRecoversViaReadyToSend},
	{FN("testCxxStreamMuxerAnnounceUnauthorizedRetry"),
	 testCxxStreamMuxerAnnounceUnauthorizedRetry},
	{FN("testCxxStreamMuxerServerPushedAnnounce"),
	 testCxxStreamMuxerServerPushedAnnounce},
	{FN("testCxxStreamMuxerStrayUdpDataIgnored"),
	 testCxxStreamMuxerStrayUdpDataIgnored},
	{FN("testCxxStreamMuxerCreateSocketsControlFails"),
	 testCxxStreamMuxerCreateSocketsControlFails},
	{FN("testCxxStreamMuxerCreateSocketsStreamFails"),
	 testCxxStreamMuxerCreateSocketsStreamFails},
	{FN("testCxxStreamMuxerProcessFrameDataFormats"),
	 testCxxStreamMuxerProcessFrameDataFormats},
	{FN("testCxxStreamMuxerSocketErrorsAndCleanup"),
	 testCxxStreamMuxerSocketErrorsAndCleanup},
	{FN("testCxxStreamMuxerProcessListWriteError"),
	 testCxxStreamMuxerProcessListWriteError},
	{FN("testCxxStreamMuxerRtspAddInputMediaNull"),
	 testCxxStreamMuxerRtspAddInputMediaNull},
	CU_TEST_INFO_NULL,
};
