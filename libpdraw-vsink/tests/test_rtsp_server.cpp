/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw-vsink unit tests -- minimal local RTSP server for real-libpdraw
 * end-to-end tests
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

#include "test_rtsp_server.hpp"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <libsdp.h>

#define ULOG_TAG pdraw_vsink_test_rtsp_server
#include <ulog.h>

ULOG_DECLARE_TAG(ULOG_TAG);

static const char *const kResourcePath = "live";
static const char *const kVideoMediaPath = "stream=0";

/* Source (RTP/RTCP) ports this server declares to the client in its SETUP
 * reply. sendVideoNalu() binds its sending socket to kServerRtpSrcPort
 * explicitly (instead of an OS-assigned ephemeral port) since some RTP
 * receivers filter incoming packets by expected source port -- silently
 * dropping anything that doesn't match what was negotiated at SETUP time,
 * which would otherwise hang pdraw_vsink_start() forever waiting for a
 * media_added that never comes, with no visible error. */
static const uint16_t kServerRtpSrcPort = 5004;
static const uint16_t kServerRtcpSrcPort = 5005;
static const char *const kAudioMediaPath = "stream=1";
static const char *const kVideoFullPath = "live/stream=0";
static const char *const kAudioFullPath = "live/stream=1";

/* Minimal H.264 High-Profile Level 4.0 SPS/PPS and a minimal IDR NAL unit.
 * Reused verbatim (same bytes, same provenance) from
 * packages/pdraw/libpdraw/tests/test_pipeline_demuxer_stream_net.cpp, which
 * sources them from libsdp's own test SDP
 * (packages/libsdp/tools/data/b_medianb.sdp:
 * sprop-parameter-sets=Z2QAKKzZgHgGWwEQAAA+kAALuAjxgxmg,aOl488jw). */
static const uint8_t kH264Sps[] = {
	0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9, 0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
	0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B, 0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};
static const uint8_t kH264Idr[] =
	{0x65, 0x88, 0x84, 0x00, 0x33, 0xC4, 0x86, 0x11};

/* A second, deliberately *different* baseline-profile 64x64 SPS/PPS pair
 * (same provenance/file as above: test_pipeline_demuxer_stream_net.cpp's
 * kH264TinySps/kH264TinyPps), reused verbatim here to simulate a real
 * mid-stream codec/resolution change (see send_video_reconfigured_access_
 * unit()'s doc comment in test_rtsp_server.hpp) -- sending the *same*
 * kH264Sps/kH264Pps again would just look like an ordinary repeated
 * parameter set, not a change. kH264Idr's NAL header (type 5) is reused
 * unchanged for the post-change IDR: the demuxer only reads the NAL
 * type here, never validates that the slice payload actually matches the
 * new SPS's declared resolution. */
static const uint8_t kH264TinySps[] =
	{0x67, 0x42, 0x00, 0x0A, 0xDA, 0x10, 0x99};
static const uint8_t kH264TinyPps[] = {0x68, 0xCE, 0x38, 0x80};

TestRtspServer::TestRtspServer(bool withAudioTrack,
			       enum vmeta_camera_type videoCameraType) :
		mWithAudioTrack(withAudioTrack),
		mVideoCameraType(videoCameraType),
		mPort(static_cast<uint16_t>(10000 + (getpid() % 10000))),
		mLoop(nullptr), mServer(nullptr), mThreadStarted(false),
		mThreadShouldStop(false), mVideoSetupDone(false),
		mPlayDone(false), mVideoRtpPort(0), mSsrc(0x12345678u), mSeq(0),
		mTs(1)
{
	pthread_mutex_init(&mLock, nullptr);
	pthread_cond_init(&mCond, nullptr);

	snprintf(mUrl,
		 sizeof(mUrl),
		 "rtsp://127.0.0.1:%u/%s",
		 mPort,
		 kResourcePath);

	mLoop = pomp_loop_new();
	if (mLoop == nullptr) {
		ULOGE("pomp_loop_new failed");
		return;
	}

	static const struct rtsp_server_cbs kCbs = {
		.socket_cb = &socketCb,
		.describe = &describeCb,
		.setup = &setupCb,
		.play = &playCb,
		.pause = &pauseCb,
		.teardown = &teardownCb,
		.request_timeout = &requestTimeoutCb,
		.announce = nullptr,
		.record = nullptr,
		.interleaved_data = nullptr,
	};

	int res = rtsp_server_new(
		nullptr, mPort, 0, 0, mLoop, &kCbs, this, &mServer);
	if (res < 0) {
		ULOG_ERRNO("rtsp_server_new", -res);
		pomp_loop_destroy(mLoop);
		mLoop = nullptr;
		return;
	}

	res = pthread_create(&mThread, nullptr, &threadFn, this);
	if (res != 0) {
		ULOG_ERRNO("pthread_create", res);
		rtsp_server_destroy(mServer);
		mServer = nullptr;
		pomp_loop_destroy(mLoop);
		mLoop = nullptr;
		return;
	}
	mThreadStarted = true;
}

TestRtspServer::~TestRtspServer()
{
	if (mThreadStarted) {
		mThreadShouldStop = true;
		pomp_loop_wakeup(mLoop);
		pthread_join(mThread, nullptr);
	}
	if (mServer != nullptr)
		rtsp_server_destroy(mServer);
	if (mLoop != nullptr)
		pomp_loop_destroy(mLoop);

	pthread_mutex_destroy(&mLock);
	pthread_cond_destroy(&mCond);
}

void *TestRtspServer::threadFn(void *arg)
{
	static_cast<TestRtspServer *>(arg)->run();
	return nullptr;
}

void TestRtspServer::run()
{
	while (!mThreadShouldStop)
		pomp_loop_wait_and_process(mLoop, -1);
}

int TestRtspServer::waitVideoRtpReady(int timeoutMs)
{
	struct timespec tsTimeout = {};
	clock_gettime(CLOCK_REALTIME, &tsTimeout);
	tsTimeout.tv_sec += timeoutMs / 1000;
	tsTimeout.tv_nsec += (timeoutMs % 1000) * 1000 * 1000;
	if (tsTimeout.tv_nsec >= 1000000000) {
		tsTimeout.tv_sec++;
		tsTimeout.tv_nsec -= 1000000000;
	}

	pthread_mutex_lock(&mLock);
	int res = 0;
	while (!(mVideoSetupDone && mPlayDone) && res == 0)
		res = pthread_cond_timedwait(&mCond, &mLock, &tsTimeout);
	pthread_mutex_unlock(&mLock);

	return (res == 0) ? 0 : -ETIMEDOUT;
}

void TestRtspServer::sendVideoNalu(const uint8_t *payload,
				   size_t len,
				   uint32_t ts,
				   bool marker)
{
	pthread_mutex_lock(&mLock);
	uint16_t port = mVideoRtpPort;
	uint16_t seq = mSeq++;
	pthread_mutex_unlock(&mLock);

	if (port == 0)
		return;

	uint8_t buf[1500];
	if (len + 12 > sizeof(buf))
		return;

	buf[0] = 0x80; /* V=2,P=0,X=0,CC=0 */
	buf[1] =
		static_cast<uint8_t>(96u | (marker ? 0x80u : 0u)); /* M,PT=96 */
	buf[2] = static_cast<uint8_t>((seq >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(seq & 0xFF);
	buf[4] = static_cast<uint8_t>((ts >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ts >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ts >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ts & 0xFF);
	buf[8] = static_cast<uint8_t>((mSsrc >> 24) & 0xFF);
	buf[9] = static_cast<uint8_t>((mSsrc >> 16) & 0xFF);
	buf[10] = static_cast<uint8_t>((mSsrc >> 8) & 0xFF);
	buf[11] = static_cast<uint8_t>(mSsrc & 0xFF);
	memcpy(buf + 12, payload, len);

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;

	int reuse = 1;
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	/* Bind to the exact source port declared to the client in the SETUP
	 * reply (see kServerRtpSrcPort's comment) -- an OS-assigned ephemeral
	 * source port can otherwise cause these packets to be silently
	 * dropped by a receiver that filters on expected source port. */
	struct sockaddr_in src = {};
	src.sin_family = AF_INET;
	src.sin_port = htons(kServerRtpSrcPort);
	src.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(sock, reinterpret_cast<struct sockaddr *>(&src), sizeof(src)) <
	    0) {
		ULOG_ERRNO("bind", errno);
		close(sock);
		return;
	}

	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(port);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       buf,
	       12 + len,
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}

void TestRtspServer::sendVideoIdrAccessUnit()
{
	uint32_t ts;
	pthread_mutex_lock(&mLock);
	ts = mTs;
	/* 90kHz RTP clock, ~30fps spacing between access units. */
	mTs += 3000;
	pthread_mutex_unlock(&mLock);

	sendVideoNalu(kH264Sps, sizeof(kH264Sps), ts, false);
	sendVideoNalu(kH264Pps, sizeof(kH264Pps), ts, false);
	sendVideoNalu(kH264Idr, sizeof(kH264Idr), ts, true);
}

void TestRtspServer::sendVideoReconfiguredAccessUnit()
{
	uint32_t ts;
	pthread_mutex_lock(&mLock);
	ts = mTs;
	mTs += 3000;
	pthread_mutex_unlock(&mLock);

	sendVideoNalu(kH264TinySps, sizeof(kH264TinySps), ts, false);
	sendVideoNalu(kH264TinyPps, sizeof(kH264TinyPps), ts, false);
	sendVideoNalu(kH264Idr, sizeof(kH264Idr), ts, true);
}

void TestRtspServer::socketCb(int fd, void *userdata)
{
	(void)fd;
	(void)userdata;
}

/* Embeds vmeta session metadata into a SDP media's attributes, using the
 * real vmeta_session_streaming_sdp_write() -- the exact wire format
 * libpdraw's own demuxer parses back out via
 * StreamDemuxer::VideoMedia::sessionMetadataFromSdp() /
 * vmeta_session_streaming_sdp_read() on the other end (see
 * pdraw_demuxer_stream.cpp), so this is a real round-trip, not a
 * approximation of one. */
static void vmetaSdpWriteCb(enum vmeta_stream_sdp_type type,
			    const char *value,
			    const char *key,
			    void *userdata)
{
	struct sdp_media *media = static_cast<struct sdp_media *>(userdata);

	if (type == VMETA_STRM_SDP_TYPE_MEDIA_INFO) {
		free(media->media_title);
		media->media_title = strdup(value);
		return;
	}

	struct sdp_attr *attr = nullptr;
	if (sdp_media_attr_add(media, &attr) == 0 && attr != nullptr) {
		attr->key = strdup(key);
		attr->value = strdup(value);
	}
}

static int addVideoMedia(struct sdp_session *session,
			 enum vmeta_camera_type cameraType)
{
	struct sdp_media *media = nullptr;
	int res = sdp_session_media_add(session, &media);
	if (res < 0)
		return res;

	media->type = SDP_MEDIA_TYPE_VIDEO;
	media->media_title = strdup("Video0");
	media->connection_addr = strdup("0.0.0.0");
	media->control_url = strdup(kVideoMediaPath);
	media->payload_type = 96;
	media->encoding_name = strdup("H264");
	media->clock_rate = 90000;

	media->h264_fmtp.valid = 1;
	media->h264_fmtp.packetization_mode = 1;
	media->h264_fmtp.profile_idc = kH264Sps[1];
	media->h264_fmtp.profile_iop = kH264Sps[2];
	media->h264_fmtp.level_idc = kH264Sps[3];
	media->h264_fmtp.sps = static_cast<uint8_t *>(malloc(sizeof(kH264Sps)));
	media->h264_fmtp.pps = static_cast<uint8_t *>(malloc(sizeof(kH264Pps)));
	if (media->h264_fmtp.sps == nullptr || media->h264_fmtp.pps == nullptr)
		return -ENOMEM;
	memcpy(media->h264_fmtp.sps, kH264Sps, sizeof(kH264Sps));
	media->h264_fmtp.sps_size = sizeof(kH264Sps);
	memcpy(media->h264_fmtp.pps, kH264Pps, sizeof(kH264Pps));
	media->h264_fmtp.pps_size = sizeof(kH264Pps);

	if (cameraType != VMETA_CAMERA_TYPE_UNKNOWN) {
		struct vmeta_session meta = {};
		meta.camera_type = cameraType;
		vmeta_session_streaming_sdp_write(
			&meta, 1, &vmetaSdpWriteCb, media);
	}

	return 0;
}

static int addAudioMedia(struct sdp_session *session)
{
	struct sdp_media *media = nullptr;
	int res = sdp_session_media_add(session, &media);
	if (res < 0)
		return res;

	/* No RTP is ever sent for this track; it only needs to exist in the
	 * SDP so libpdraw-vsink's own "non-video media is ignored" logic can
	 * be exercised for real. Dynamic payload type (96-127, RFC 3551),
	 * not PCMU's well-known static type 0: some SDP-writing code paths
	 * treat a payload_type of 0 as "unset" (C's falsy int), which can
	 * silently drop required fields from this media's SDP block. */
	media->type = SDP_MEDIA_TYPE_AUDIO;
	media->media_title = strdup("Audio0");
	media->connection_addr = strdup("0.0.0.0");
	media->control_url = strdup(kAudioMediaPath);
	media->payload_type = 97;
	media->encoding_name = strdup("PCMU");
	media->clock_rate = 8000;

	return 0;
}

void TestRtspServer::describeCb(struct rtsp_server *server,
				const char *serverAddress,
				const char *path,
				const struct rtsp_header_ext *ext,
				size_t extCount,
				void *requestCtx,
				void *userdata)
{
	(void)ext;
	(void)extCount;

	TestRtspServer *self = static_cast<TestRtspServer *>(userdata);
	int res = 0;
	struct sdp_session *session = nullptr;
	char *sdp = nullptr;

	if (serverAddress == nullptr || serverAddress[0] == '\0' || path == nullptr ||
	    strcmp(path, kResourcePath) != 0) {
		res = -ENOENT;
		goto reply;
	}

	session = sdp_session_new();
	if (session == nullptr) {
		res = -ENOMEM;
		goto reply;
	}
	session->session_id = 1;
	session->session_version = 1;
	session->server_addr = strdup(serverAddress);
	session->session_name = strdup("libpdraw-vsink-test");
	session->connection_addr = strdup("0.0.0.0");
	session->control_url = strdup("*");

	res = addVideoMedia(session, self->mVideoCameraType);
	if (res < 0)
		goto reply;

	if (self->mWithAudioTrack) {
		res = addAudioMedia(session);
		if (res < 0)
			goto reply;
	}

	res = sdp_description_write(session, &sdp);

reply:
	rtsp_server_reply_to_describe(server, requestCtx, res, nullptr, 0, sdp);
	if (session != nullptr)
		sdp_session_destroy(session);
	free(sdp);
}

void TestRtspServer::setupCb(struct rtsp_server *server,
			     const char *path,
			     const char *sessionId,
			     const struct rtsp_header_ext *ext,
			     size_t extCount,
			     void *requestCtx,
			     void *mediaCtx,
			     enum rtsp_delivery delivery,
			     enum rtsp_lower_transport lowerTransport,
			     const char *srcAddress,
			     const char *dstAddress,
			     uint16_t dstStreamPort,
			     uint16_t dstControlPort,
			     void *userdata)
{
	(void)ext;
	(void)extCount;
	(void)srcAddress;

	TestRtspServer *self = static_cast<TestRtspServer *>(userdata);
	int res = 0;

	if (path == nullptr || sessionId == nullptr || sessionId[0] == '\0' ||
	    dstAddress == nullptr || dstAddress[0] == '\0' || dstStreamPort == 0 ||
	    dstControlPort == 0) {
		res = -EINVAL;
	} else if (strcmp(path, kVideoFullPath) != 0 &&
		   strcmp(path, kAudioFullPath) != 0) {
		res = -ENOENT;
	} else if (delivery != RTSP_DELIVERY_UNICAST ||
		   lowerTransport != RTSP_LOWER_TRANSPORT_UDP) {
		res = -ENOSYS;
	}

	if (res == 0 && strcmp(path, kVideoFullPath) == 0) {
		pthread_mutex_lock(&self->mLock);
		self->mVideoRtpPort = dstStreamPort;
		self->mVideoSetupDone = true;
		pthread_cond_broadcast(&self->mCond);
		pthread_mutex_unlock(&self->mLock);
	}

	rtsp_server_reply_to_setup(server,
				   requestCtx,
				   mediaCtx,
				   res,
				   kServerRtpSrcPort,
				   kServerRtcpSrcPort,
				   1,
				   self->mSsrc,
				   nullptr,
				   0,
				   reinterpret_cast<void *>(1));
}

void TestRtspServer::playCb(struct rtsp_server *server,
			    const char *sessionId,
			    const struct rtsp_header_ext *ext,
			    size_t extCount,
			    void *requestCtx,
			    void *mediaCtx,
			    const struct rtsp_range *range,
			    float scale,
			    void *streamUserdata,
			    void *userdata)
{
	(void)ext;
	(void)extCount;
	(void)streamUserdata;

	TestRtspServer *self = static_cast<TestRtspServer *>(userdata);
	int res = 0;
	struct rtsp_range respRange = {};

	if (sessionId == nullptr || sessionId[0] == '\0' || range == nullptr) {
		res = -EINVAL;
		goto reply;
	}
	if (range->start.format != RTSP_TIME_FORMAT_NPT) {
		res = -ENOSYS;
		goto reply;
	}
	if (scale == 0.f)
		scale = 1.f;
	respRange = *range;

reply:
	/* Signal readiness *before* replying, not after: the client (real
	 * demuxer) can react to our reply the instant it's sent, and we want
	 * a concurrently woken test thread's sendVideoIdrAccessUnit() to
	 * never have a chance of racing ahead of this flag update. */
	pthread_mutex_lock(&self->mLock);
	self->mPlayDone = true;
	pthread_cond_broadcast(&self->mCond);
	pthread_mutex_unlock(&self->mLock);

	/* seq/rtptime = 0: matches this monorepo's existing convention (see
	 * test_pipeline_demuxer_stream_net.cpp's TestRtspServer::playCb) for
	 * avoiding "silent before play point" filtering on the small,
	 * positive RTP timestamps used by sendVideoIdrAccessUnit(). */
	rtsp_server_reply_to_play(server,
				  requestCtx,
				  mediaCtx,
				  res,
				  &respRange,
				  scale,
				  1,
				  0,
				  1,
				  0,
				  nullptr,
				  0);
}

void TestRtspServer::pauseCb(struct rtsp_server *server,
			     const char *sessionId,
			     const struct rtsp_header_ext *ext,
			     size_t extCount,
			     void *requestCtx,
			     void *mediaCtx,
			     const struct rtsp_range *range,
			     void *streamUserdata,
			     void *userdata)
{
	(void)ext;
	(void)extCount;
	(void)streamUserdata;
	(void)userdata;

	int res = 0;
	struct rtsp_range respRange = {};

	if (sessionId == nullptr || sessionId[0] == '\0' || range == nullptr)
		res = -EINVAL;
	else
		respRange = *range;

	rtsp_server_reply_to_pause(
		server, requestCtx, mediaCtx, res, &respRange, nullptr, 0);
}

void TestRtspServer::teardownCb(struct rtsp_server *server,
				const char *path,
				const char *sessionId,
				enum rtsp_server_teardown_reason reason,
				const struct rtsp_header_ext *ext,
				size_t extCount,
				void *requestCtx,
				void *mediaCtx,
				void *streamUserdata,
				void *userdata)
{
	(void)path;
	(void)reason;
	(void)ext;
	(void)extCount;
	(void)streamUserdata;
	(void)userdata;

	int res = (sessionId == nullptr || sessionId[0] == '\0') ? -EINVAL : 0;

	if (requestCtx != nullptr) {
		rtsp_server_reply_to_teardown(
			server, requestCtx, mediaCtx, res, nullptr, 0);
	}
}

void TestRtspServer::requestTimeoutCb(struct rtsp_server *server,
				      void *requestCtx,
				      enum rtsp_method_type method,
				      void *userdata)
{
	(void)server;
	(void)requestCtx;
	(void)method;
	(void)userdata;
}
