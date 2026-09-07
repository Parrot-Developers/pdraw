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

/*
 * These tests deliberately do NOT mock libpdraw: libpdraw already has its
 * own CUnit suite (tst-libpdraw), so it's trusted here and linked for real.
 * What's being validated is libpdraw-vsink's own logic (state machine,
 * threading/condvar handshake, media-type dispatch, get_frame semantics,
 * flush/drain, teardown), driven through a REAL local pdraw pipeline.
 *
 * libpdraw-vsink's internal pomp_loop/thread (created inside
 * pdraw_vsink_start()) is fully private -- there is no public seam to share
 * it with a test-controlled event source. So this server gets its own
 * independent pomp_loop + thread and talks real RTSP (TCP) + RTP (UDP) to
 * the real demuxer libpdraw-vsink creates, entirely over 127.0.0.1: from
 * libpdraw-vsink's point of view this is indistinguishable from a real
 * server, just local and fully under the test's control (deterministic,
 * no network/NAS dependency).
 *
 * Ported from two existing, working references in this monorepo:
 *   - packages/librtsp/tools/rtsp_server_test.c (DESCRIBE/SETUP/PLAY/
 *     PAUSE/TEARDOWN callback shapes and replies).
 *   - packages/pdraw/libpdraw/tests/test_pipeline_demuxer_stream_net.cpp
 *     (sendRtpNaluPacket() RTP packet layout, and the H.264 SPS/PPS/IDR
 *     byte arrays -- reused verbatim here, not re-derived).
 *
 * Coverage note: unlike a mock, this server can't inject arbitrary internal
 * libpdraw failures (e.g. forcing pdraw_new() itself to fail) -- that is
 * exactly the kind of thing libpdraw's own test suite already covers.
 * What IS realistically coverable for real by controlling what this server
 * serves: a full successful open+play+mediaAdded (SPS/PPS/IDR delivered),
 * and a non-video (audio-only) media being ignored by libpdraw-vsink's own
 * type filter.
 */

#pragma once

#include <stdint.h>

#include <pthread.h>

#include <libpomp.h>
#include <rtsp/server.h>
#include <video-metadata/vmeta.h>
#include <video-metadata/vmeta_session.h>

class TestRtspServer {
public:
	/* withAudioTrack: also advertise a second, audio-only SDP media
	 * (no RTP ever sent for it) -- used to verify that libpdraw-vsink
	 * ignores non-video medias for real.
	 * videoCameraType: if not VMETA_CAMERA_TYPE_UNKNOWN, embeds a real
	 * vmeta "camera_type" attribute (RFC-legal "a=X-*" SDP extension,
	 * via the real vmeta_session_streaming_sdp_write() -- same wire
	 * format libpdraw's own demuxer parses on the other end) into the
	 * video track's SDP, so select_media_cb's camera_type-match branch
	 * (as opposed to its is_default branch) can be exercised for real. */
	explicit TestRtspServer(bool withAudioTrack = false,
				enum vmeta_camera_type videoCameraType =
					VMETA_CAMERA_TYPE_UNKNOWN);
	~TestRtspServer();

	bool isStarted() const
	{
		return mServer != NULL;
	}

	/* URL to pass as pdraw_vsink_params::url. */
	const char *url() const
	{
		return mUrl;
	}

	/* Blocks (bounded by timeoutMs) until the video track's SETUP has
	 * captured its RTP destination port AND the session's PLAY request
	 * has been replied to. Sending RTP before PLAY is confirmed is too
	 * early: the real demuxer's RTP receive path isn't necessarily
	 * listening yet, so packets sent before this point can be silently
	 * dropped, leaving pdraw_vsink_start() blocked forever waiting for a
	 * mediaAdded that never comes.
	 * @return 0 on success, -ETIMEDOUT otherwise. */
	int waitVideoRtpReady(int timeoutMs);

	/* Sends the minimal SPS, PPS, then IDR (marker bit set) sequence --
	 * the smallest access unit that makes a real H.264 demuxer/decoder
	 * report a usable media -- at a freshly incremented RTP timestamp.
	 * Call waitVideoRtpReady() first. Repeated calls simulate
	 * successive frames arriving (each call is a full, self-contained
	 * access unit -- real streams commonly repeat SPS/PPS before every
	 * IDR too, so this is realistic, not just a simplification). */
	void sendVideoIdrAccessUnit();

	/* Sends an access unit with a *different* SPS/PPS than
	 * sendVideoIdrAccessUnit() -- a real mid-stream codec/resolution
	 * change. This is what actually makes libpdraw's demuxer detect new
	 * codec info, tear the channel down and flush the attached sink for
	 * real (see StreamDemuxer::VideoMedia::codecInfoChangedCb() in
	 * pdraw_demuxer_stream.cpp): sending the *same* SPS/PPS again would
	 * just be treated as an ordinary repeated parameter set, not a
	 * change, and would never reach that path. */
	void sendVideoReconfiguredAccessUnit();

private:
	/* Sends one RTP packet (single-NAL mode, RFC 6184 §5.6) carrying a
	 * raw H.264 NAL unit (no start code/size prefix) to the negotiated
	 * video track's RTP port, using and incrementing mSeq. */
	void sendVideoNalu(const uint8_t *payload,
			   size_t len,
			   uint32_t ts,
			   bool marker);

	static void *threadFn(void *arg);
	void run();

	static void socketCb(int fd, void *userdata);
	static void describeCb(struct rtsp_server *server,
			       const char *serverAddress,
			       const char *path,
			       const struct rtsp_header_ext *ext,
			       size_t extCount,
			       void *requestCtx,
			       void *userdata);
	static void setupCb(struct rtsp_server *server,
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
			    void *userdata);
	static void playCb(struct rtsp_server *server,
			   const char *sessionId,
			   const struct rtsp_header_ext *ext,
			   size_t extCount,
			   void *requestCtx,
			   void *mediaCtx,
			   const struct rtsp_range *range,
			   float scale,
			   void *streamUserdata,
			   void *userdata);
	static void pauseCb(struct rtsp_server *server,
			    const char *sessionId,
			    const struct rtsp_header_ext *ext,
			    size_t extCount,
			    void *requestCtx,
			    void *mediaCtx,
			    const struct rtsp_range *range,
			    void *streamUserdata,
			    void *userdata);
	static void teardownCb(struct rtsp_server *server,
			       const char *path,
			       const char *sessionId,
			       enum rtsp_server_teardown_reason reason,
			       const struct rtsp_header_ext *ext,
			       size_t extCount,
			       void *requestCtx,
			       void *mediaCtx,
			       void *streamUserdata,
			       void *userdata);
	static void requestTimeoutCb(struct rtsp_server *server,
				     void *requestCtx,
				     enum rtsp_method_type method,
				     void *userdata);

	bool mWithAudioTrack;
	enum vmeta_camera_type mVideoCameraType;
	uint16_t mPort;
	char mUrl[64];
	struct pomp_loop *mLoop;
	struct rtsp_server *mServer;
	pthread_t mThread;
	bool mThreadStarted;
	volatile bool mThreadShouldStop;

	pthread_mutex_t mLock;
	pthread_cond_t mCond;
	bool mVideoSetupDone;
	bool mPlayDone;
	uint16_t mVideoRtpPort;
	uint32_t mSsrc;

	uint16_t mSeq;
	uint32_t mTs;
};
