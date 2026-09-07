/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — StreamDemuxerNet RTSP signaling pipeline (Tier B)
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

/* Tests the StreamDemuxerNet RTSP signaling state machine by embedding a
 * minimal RTSP server on the same pomp loop as the PDrAW session.
 *
 * Because both client (StreamDemuxerNet) and server (TestRtspServer) share
 * the single-threaded pomp loop, each pumpUntil() iteration processes events
 * for both sides, driving the full RTSP handshake without threads.
 * No actual RTP/RTCP packets are exchanged; these tests cover exclusively the
 * RTSP signaling paths in pdraw_demuxer_stream.cpp / _stream_net.cpp.
 *
 * The server is adapted from packages/librtsp/tools/rtsp_server_test.c.
 *
 * Test suite
 * ──────────
 * testCxxStreamDemuxerRtspSignalingLifecycle
 *   Single-media SDP, auto-selected default track.
 *   OPTIONS → DESCRIBE → SETUP → PLAY → PAUSE → TEARDOWN.
 *   Also exercises getMediaList() (A3) after openResponse.
 *
 * testCxxStreamDemuxerTwoMediaSelectFirst
 *   Two-media SDP; demuxerSelectMedia() callback explicitly selects track 1.
 *   Covers multi-track onNewSdp, callSelectMedia, processSelectedMedias (A2).
 *
 * testCxxStreamDemuxerRtpCodecInfo
 *   SPS/PPS in SDP fmtp → mCodecInfo set at SETUP.  One UDP RTP packet
 *   with matching SSRC triggers codecInfoChangedCb → setupMedia →
 *   onMediaAdded (B1).
 *
 * testCxxStreamDemuxerDescribeError
 *   Server returns 404 on DESCRIBE (wrong resource path).
 *   Covers onRtspDescribeResp FAILED branch → onUnrecoverableError (A1).
 *
 * Groups E (demuxer-level, no attached sink) and F (real attached
 * ExternalCodedVideoSink) further below cover onRtspForcedTeardown,
 * onChannelFlushed/Drained/Unlink/Resync, empty SDP (mediasCount == 0),
 * selectMedia() (-ECANCELED callback and runtime switch), isPaused(),
 * previousFrame()/nextFrame(), seek()/seekTo(), getDuration()/
 * getCurrentTime(), processFrame(), eventCb() and goodbyeCb(). These do
 * send real RTP/RTCP packets (see sendRtpNaluPacket()/sendRtcpBye()/
 * sendRtcpEvent()). See TEST_PROGRESS.md for the full per-test rationale
 * and known limitations (e.g. onChannelVideoPresStats has no public-API
 * trigger and is not covered here).
 *
 * testCxxStreamDemuxerSessionMetaUpdateCascadesToDecoder
 *   autodecoding_mode = DECODE_ALL + a real attached ExternalRawVideoSink
 *   (instead of Group F's ExternalCodedVideoSink): a hand-crafted RTCP SDES
 *   packet (see sendRtcpSdes()) drives sessionMetadataPeerChangedCb() ->
 *   SESSION_META_UPDATE -> VideoDecoder::onChannelSessionMetaUpdate.
 */

#define ULOG_TAG pdraw_test_pipeline_demuxer_stream

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/resource.h>
#include <sys/socket.h>

#include <chrono>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <libsdp.h>
#include <rtsp/server.h>

#include <h264/h264.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>

#define private public
#define protected public

#include "pdraw_demuxer_stream.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_element.hpp"
#include "pdraw_utils.hpp"
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#undef protected
#undef private

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

/* Port used by the embedded test server.
 * High enough to avoid conflicts with well-known services; SO_REUSEADDR on the
 * listening socket allows rebinding immediately after the previous test run. */
static constexpr uint16_t kTestRtspPort = 18554;
static constexpr const char *kTestRtspPath = "live";
static constexpr const char *kTestRtspMedia1Path = "stream=0";
static constexpr const char *kTestRtspMedia2Path = "stream=1";
/* Full control-URL paths received in the server's setup callback. */
static constexpr const char *kTestRtspMedia1FullPath = "live/stream=0";
static constexpr const char *kTestRtspMedia2FullPath = "live/stream=1";

/* SSRC advertised by the test server in SETUP replies — must match the
 * value passed to rtsp_server_reply_to_setup() in setupCb(). */
static constexpr uint32_t kTestRtspSsrc = 0x12345678u;

/* Minimal H.264 High-Profile Level 4.0 SPS and PPS sourced from the
 * libsdp test SDP (tools/data/b_medianb.sdp):
 *   sprop-parameter-sets=Z2QAKKzZgHgGWwEQAAA+kAALuAjxgxmg,aOl488jw
 * These are raw NAL-unit bytes (no start code / size prefix). */
static const uint8_t kH264Sps[] = {
	0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9, 0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
	0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B, 0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};
/* Minimal IDR NAL unit (type 5, NAL ref idc=3): used to trigger
 * asyncCompleteSeek() in recvFrameCb after a seekTo(). The slice payload
 * bytes are not parsed by recvFrameCb — only NAL type is checked. */
static const uint8_t kH264Idr[] =
	{0x65, 0x88, 0x84, 0x00, 0x33, 0xC4, 0x86, 0x11};

/* SEI NAL: User Data Unregistered (type 5).
 * UUID = 16 zero bytes (not the Parrot streaming UUID); data = {0xBE, 0xEF}.
 * Triggers h264UserDataSeiCb (adds ancillary buffer to the current frame). */
static const uint8_t kH264SeiUserDataUnreg[] = {
	0x06, /* NAL header (SEI, nal_unit_type=6) */
	0x05, /* payloadType = user_data_unregistered */
	0x12, /* payloadSize = 18 (16 UUID + 2 data bytes) */
	/* UUID: 16 zero bytes — not the Parrot streaming UUID */
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0xBE,
	0xEF, /* user data */
	0x80, /* rbsp_trailing_bits */
};

/* SEI NAL: Recovery Point (type 6).
 * recovery_frame_cnt=0 (UE→"1"), exact_match_flag=1, broken_link_flag=0,
 * changed_slice_group_idc=0 (u2→"00") → bits "11000" + stop "1" + pad "00"
 * = 0xC4. Triggers h264RecoveryPointSeiCb (clears mWaitForSync). */
static const uint8_t kH264SeiRecoveryPoint[] = {
	0x06, /* NAL header (SEI) */
	0x06, /* payloadType = recovery_point */
	0x01, /* payloadSize = 1 */
	0xC4, /* payload: cnt=0, exact=1, broken=0, changed=0 */
	0x80, /* rbsp_trailing_bits */
};

/* Same SEI as kH264SeiRecoveryPoint, but recovery_frame_cnt=1 instead of 0:
 * UE(1)="010", exact=1->"1", broken=0->"0", changed=0->"00" = "0101000" (7
 * bits) + 1 pad bit = 0x50. h264RecoveryPointSeiCb sets mRecoveryFrameCount
 * = recovery_frame_cnt+1 = 2, so it takes two subsequent ref frames (not
 * just one) to bring StreamDemuxer::VideoMedia::mRecoveryFrameCount back to
 * 0 -- needed to observe the VDEF_FRAME_FLAG_SILENT flag set on one frame
 * and cleared on the next (pdraw_demuxer_stream.cpp:3769-3770, 3795-3796). */
static const uint8_t kH264SeiRecoveryPointCnt1[] = {
	0x06, /* NAL header (SEI) */
	0x06, /* payloadType = recovery_point */
	0x01, /* payloadSize = 1 */
	0x50, /* payload: cnt=1, exact=1, broken=0, changed=0 */
	0x80, /* rbsp_trailing_bits */
};

/* Access Unit Delimiter NAL (type 9, nal_ref_idc=0): not one of the NALU
 * types explicitly handled in processFrame()'s switch (SPS/PPS/SEI/
 * SLICE[_IDR]), so it falls through to the "default" case
 * (pdraw_demuxer_stream.cpp ~line 3730), which is otherwise 0% covered.
 * primary_pic_type=0 ("I"), rbsp_trailing_bits -- payload is not parsed by
 * processFrame(), only the NAL header's type field is read. */
static const uint8_t kH264Aud[] = {
	0x09, /* NAL header (AUD, nal_ref_idc=0) */
	0x10, /* primary_pic_type=0, rbsp_trailing_bits */
};

/* Non-IDR slice NAL (type 1, nal_ref_idc=3): same fake payload bytes as
 * kH264Idr (not parsed by recvFrameCb, only the NAL header is read -- see
 * kH264Idr's comment above), but with type=SLICE instead of SLICE_IDR and a
 * non-zero nal_ref_idc so vstrm_rtp_h264_rx.c sets frame->info.ref=true
 * (vstrm_rtp_h264_rx.c:588: "info.ref = nal_ref_idc != 0"). Used to reach
 * the "discarding frame (wait for sync)" branch (non-IDR while
 * mWaitForSync) and the mRecoveryFrameCount-- branch (a ref frame while
 * mRecoveryFrameCount > 0), both in processFrame(). */
static const uint8_t kH264PSlice[] =
	{0x61, 0x88, 0x84, 0x00, 0x33, 0xC4, 0x86, 0x11};

/* Minimal baseline-profile H.264 SPS/PPS for a 64x64 frame, hand-encoded
 * (exp-golomb) field by field: profile_idc=66 (Baseline, so none of the High
 * profile's extra chroma/bit-depth/scaling-matrix fields apply),
 * level_idc=10, sps_id=0, log2_max_frame_num_minus4=0,
 * pic_order_cnt_type=2 (no extra fields), max_num_ref_frames=1,
 * gaps_in_frame_num_value_allowed_flag=0, pic_width_in_mbs_minus1=3,
 * pic_height_in_map_units_minus1=3 (4x4 macroblocks = 64x64 px),
 * frame_mbs_only_flag=1, direct_8x8_inference_flag=1,
 * frame_cropping_flag=0, vui_parameters_present_flag=0.
 * PPS: pic/seq_parameter_set_id=0, entropy_coding_mode_flag=0 (CAVLC),
 * bottom_field_pic_order_in_frame_present_flag=0, num_slice_groups_minus1=0,
 * num_ref_idx_l0/l1_default_active_minus1=0, weighted_pred_flag=0,
 * weighted_bipred_idc=0, pic_init_qp/qs_minus26=0, chroma_qp_index_offset=0,
 * deblocking_filter_control_present_flag=0, constrained_intra_pred_flag=0,
 * redundant_pic_cnt_present_flag=0.
 * Used by testCxxStreamDemuxerProcessFrameBufferTooSmall to negotiate a
 * media whose output memory pool (createOutputPortMemoryPool(),
 * pdraw_demuxer_stream.cpp:3127-3131) is sized 64*64*3/4 = 3072 bytes --
 * small enough that a single legitimately-sized RTP/UDP NAL unit can
 * exceed it, reaching the otherwise-unreachable (with the file's normal
 * ~1MB, 1920x800 kH264Sps) "input buffer too small" branch at
 * pdraw_demuxer_stream.cpp:3664-3669. */
static const uint8_t kH264TinySps[] =
	{0x67, 0x42, 0x00, 0x0A, 0xDA, 0x10, 0x99};
static const uint8_t kH264TinyPps[] = {0x68, 0xCE, 0x38, 0x80};

/* Same fields as kH264TinySps (baseline profile, 64x64, pic_order_cnt_type=2,
 * max_num_ref_frames=1) but with vui_parameters_present_flag=1 and a VUI
 * carrying: timing_info_present_flag=1 (num_units_in_tick=1, time_scale=
 * 1000000 -- h264_ctx_sei_pic_timing_to_us() returns 0 if either is 0) and
 * pic_struct_present_flag=1 (required for h264_syntax.h's sei_pic_timing()
 * parser to read the clk_ts[] array at all -- with the file's usual kH264Sps,
 * whose VUI has pic_struct_present_flag=0, the parser reads zero bits and
 * h264PicTimingSeiCb only ever sees an all-zero struct, i.e. a capture
 * timestamp of 0). No HRD (nal/vcl_hrd_parameters_present_flag=0), so
 * sei_pic_timing()'s time_offset field defaults to a 24-bit width
 * (h264_syntax.h:662-667). Used with kH264SeiPicTiming below; PPS: reuse
 * kH264TinyPps unchanged (the VUI does not affect PPS fields). */
static const uint8_t kH264PicTimingSps[] = {0x67,
					    0x42,
					    0x00,
					    0x0A,
					    0xDA,
					    0x10,
					    0x9A,
					    0x10,
					    0x00,
					    0x00,
					    0x00,
					    0x10,
					    0x00,
					    0xF4,
					    0x24,
					    0x01,
					    0x40};

/* SEI NAL: Picture Timing (type 1). Requires kH264PicTimingSps's VUI to parse
 * (pic_struct_present_flag=1, no HRD present). Triggers h264PicTimingSeiCb
 * (pdraw_demuxer_stream.cpp:4135-4155), previously 0% covered.
 *
 * Hand bit-packed per h264_syntax.h's sei_pic_timing() (no cpb_removal_delay/
 * dpb_output_delay fields since neither HRD flag is set):
 *   pic_struct=0 (u4) -> num_clock_ts[pic_struct]=1, i.e. one clk_ts[] entry
 *   clk_ts[0]: clock_timestamp_flag=1 (u1), ct_type=0 (u2),
 *     nuit_field_based_flag=0 (u1), counting_type=0 (u5),
 *     full_timestamp_flag=1 (u1), discontinuity_flag=0 (u1),
 *     cnt_dropped_flag=0 (u1), n_frames=4 (u8), seconds_value=3 (u6),
 *     minutes_value=2 (u6), hours_value=1 (u5), time_offset=0 (u24, the
 *     default width used when neither HRD flag is set)
 * = 65 bits total -> 9 payload bytes (the last byte's low 7 bits are unused
 * padding; the parser only ever reads the 65 bits above, so their value is
 * irrelevant). h264_ctx_sei_pic_timing_to_us() then computes:
 *   clock_timestamp = ((1*60+2)*60+3)*time_scale + 4*num_units_in_tick
 *                    = 3723*1000000 + 4*1 = 3723000004
 * and since time_scale == 1000000, converting ticks to microseconds is a
 * no-op, so mCurrentFrameCaptureTs == 3723000004.
 * Trailing 0x80 byte: rbsp_trailing_bits() for the SEI NAL as a whole
 * (7.3.2.3 sei_rbsp), consumed from the outer bitstream once
 * h264_bs_more_rbsp_data() finds no further sei_message() to parse -- same
 * pattern as kH264SeiRecoveryPoint's trailing 0x80 byte above. */
static const uint8_t kH264SeiPicTiming[] = {
	0x06, /* NAL header (SEI, nal_ref_idc=0) */
	0x01, /* payloadType = pic_timing */
	0x09, /* payloadSize = 9 */
	0x08,
	0x04,
	0x04,
	0x0C,
	0x20,
	0x80,
	0x00,
	0x00,
	0x00, /* payload (65 significant bits + 7 padding bits) */
	0x80, /* rbsp_trailing_bits (outer SEI NAL) */
};


/* ── Anonymous namespace (internal linkage, avoids ODR violations) ────── */
namespace {


/* ── Embedded in-process RTSP server ──────────────────────────────────── */

/* Minimal RTSP server for signaling tests, bound to an existing pomp_loop so
 * it shares the same event loop thread as the PDrAW session under test.
 *
 * mediaCount (1 or 2) controls how many H.264 tracks the DESCRIBE response
 * carries.  The second track is used by testCxxStreamDemuxerTwoMediaSelectFirst
 * to exercise the multi-media SDP selection path.
 *
 * Deliberate simplifications vs rtsp_server_test.c:
 *   1. connection_addr = "0.0.0.0" — demuxer ignores it, falls back to URL
 * host.
 *   2. No stream_userdata check in pause/teardown — session-level TEARDOWN can
 *      legitimately carry NULL stream_userdata.
 *   3. Static SSRC (0x12345678) — no randomness needed for signaling tests. */
class TestRtspServer {
public:
	/* Failure-injection flags: set before the server is exercised. */
	bool mSetupShouldFail = false;
	bool mPlayShouldFail = false;
	/* When true the SDP carries "a=range:npt=0-120" so that the demuxer
	 * treats the session as a replay stream (getDuration() > 0). */
	bool mHasDuration = false;
	/* When true, playCb() returns start==stop==120s (both non-zero) to
	 * trigger the idleEndOfRangeNotification idle handler in
	 * onRtspPlayResp (StreamDemuxer::idleEndOfRangeNotification). */
	bool mPlayReturnsEndOfRange = false;
	/* When true, describeCb() returns a valid SDP session with zero
	 * media tracks (onNewSdp's mediasCount == 0 / "empty SDP" branch). */
	bool mEmptySdp = false;
	/* When true, describeCb() includes session_info, tool, and custom
	 * session attributes. */
	bool mWithSdpSessionAttrs = false;
	/* RTSP session_id captured from the first SETUP request: session->
	 * session_id is generated by rtsp_server_session_add() *before*
	 * cbs.setup() is invoked (see rtsp_server_setup() in rtsp_server.c),
	 * so it is already valid by the time setupCb() runs. Needed to call
	 * rtsp_server_force_teardown(). */
	std::string mSessionId;

	TestRtspServer(struct pomp_loop *loop,
		       uint16_t port,
		       int mediaCount = 1) :
			mMediaCount(mediaCount)
	{
		static const struct rtsp_server_cbs s_cbs = {
			.socket_cb = socketCb,
			.describe = describeCb,
			.setup = setupCb,
			.play = playCb,
			.pause = pauseCb,
			.teardown = teardownCb,
			.request_timeout = requestTimeoutCb,
			.announce = nullptr,
			.record = nullptr,
			.interleaved_data = nullptr,
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

	~TestRtspServer()
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
	int mMediaCount;

	/* Helper: append one H.264 video track to the SDP session. */
	static int addMedia(struct sdp_session *session,
			    const char *title,
			    const char *controlUrl)
	{
		struct sdp_media *media = nullptr;
		int ret = sdp_session_media_add(session, &media);
		if (ret < 0)
			return ret;
		media->type = SDP_MEDIA_TYPE_VIDEO;
		media->media_title = strdup(title);
		media->connection_addr = strdup("0.0.0.0");
		media->control_url = strdup(controlUrl);
		media->payload_type = 96;
		media->encoding_name = strdup("H264");
		media->clock_rate = 90000;
		/* Include SPS/PPS in the fmtp line so the demuxer populates
		 * mCodecInfo at createReceiver() time (needed for B-group RTP
		 * tests and also makes the SDP match real-world streams). */
		media->h264_fmtp.valid = 1;
		media->h264_fmtp.packetization_mode = 1;
		/* profile-level-id fields read from the SPS header bytes. */
		media->h264_fmtp.profile_idc = kH264Sps[1]; /* 0x64 = High */
		media->h264_fmtp.profile_iop = kH264Sps[2]; /* 0x00 */
		media->h264_fmtp.level_idc = kH264Sps[3]; /* 0x28 = L4.0 */
		media->h264_fmtp.sps =
			static_cast<uint8_t *>(malloc(sizeof(kH264Sps)));
		media->h264_fmtp.pps =
			static_cast<uint8_t *>(malloc(sizeof(kH264Pps)));
		if (!media->h264_fmtp.sps || !media->h264_fmtp.pps)
			return -ENOMEM;
		memcpy(media->h264_fmtp.sps, kH264Sps, sizeof(kH264Sps));
		media->h264_fmtp.sps_size = sizeof(kH264Sps);
		memcpy(media->h264_fmtp.pps, kH264Pps, sizeof(kH264Pps));
		media->h264_fmtp.pps_size = sizeof(kH264Pps);
		return 0;
	}

	static void socketCb(int /*fd*/, void * /*userdata*/) {}

	static void describeCb(struct rtsp_server *server,
			       const char *server_address,
			       const char *path,
			       const struct rtsp_header_ext * /*ext*/,
			       size_t /*ext_count*/,
			       void *request_ctx,
			       void *userdata)
	{
		auto *self = static_cast<TestRtspServer *>(userdata);
		int ret = 0;
		struct sdp_session *session = nullptr;
		char *sdp = nullptr;

		if (!server_address || !path ||
		    strcmp(path, kTestRtspPath) != 0) {
			ret = -ENOENT;
			goto reply;
		}

		session = sdp_session_new();
		if (!session) {
			ret = -ENOMEM;
			goto reply;
		}
		session->session_id = 1;
		session->session_version = 1;
		session->server_addr = strdup(server_address);
		session->session_name = strdup("TestStream");
		session->connection_addr = strdup("0.0.0.0");
		session->control_url = strdup("*");

		if (self->mWithSdpSessionAttrs) {
			session->session_info = strdup("Test Session Info");
			session->tool = strdup("Test Tool v1.0");
			struct sdp_attr *attr = nullptr;
			if (sdp_session_attr_add(session, &attr) == 0 &&
			    attr != nullptr) {
				attr->key = strdup("com.parrot.test");
				attr->value = strdup("value");
			}
		}

		if (self->mHasDuration) {
			/* Advertise a 120-second replay stream. */
			session->range.start.format = SDP_TIME_FORMAT_NPT;
			session->range.start.npt = {};
			session->range.stop.format = SDP_TIME_FORMAT_NPT;
			session->range.stop.npt.sec = 120;
		}

		if (!self->mEmptySdp) {
			ret = addMedia(session, "Video0", kTestRtspMedia1Path);
			if (ret < 0)
				goto reply;

			if (self->mMediaCount > 1) {
				ret = addMedia(
					session, "Video1", kTestRtspMedia2Path);
				if (ret < 0)
					goto reply;
			}
		}

		ret = sdp_description_write(session, &sdp);

	reply:
		rtsp_server_reply_to_describe(
			server, request_ctx, ret, nullptr, 0, sdp);
		if (session)
			sdp_session_destroy(session);
		free(sdp);
	}

	static void setupCb(struct rtsp_server *server,
			    const char *path,
			    const char *session_id,
			    const struct rtsp_header_ext * /*ext*/,
			    size_t /*ext_count*/,
			    void *request_ctx,
			    void *media_ctx,
			    enum rtsp_delivery delivery,
			    enum rtsp_lower_transport lower_transport,
			    const char *src_address,
			    const char *dst_address,
			    uint16_t dst_stream_port,
			    uint16_t dst_control_port,
			    void *userdata)
	{
		auto *self = static_cast<TestRtspServer *>(userdata);
		int ret = 0;

		if (self && session_id != nullptr)
			self->mSessionId = session_id;

		if (self && self->mSetupShouldFail) {
			ret = -ENOENT;
		} else if (!path || !session_id || !src_address ||
			   !dst_address || dst_stream_port == 0 ||
			   dst_control_port == 0) {
			ret = -EINVAL;
		} else if (strcmp(path, kTestRtspMedia1FullPath) != 0 &&
			   strcmp(path, kTestRtspMedia2FullPath) != 0) {
			ret = -ENOENT;
		} else if (delivery != RTSP_DELIVERY_UNICAST ||
			   (lower_transport != RTSP_LOWER_TRANSPORT_UDP &&
			    lower_transport != RTSP_LOWER_TRANSPORT_TCP)) {
			ret = -ENOSYS;
		}

		/* Use different server-side ports for each track so that two
		 * simultaneous media setups don't advertise the same pair.
		 * No socket is actually bound at these ports (no RTP sent). */
		bool isTrack2 = (strcmp(path, kTestRtspMedia2FullPath) == 0);
		uint16_t streamPort =
			(lower_transport == RTSP_LOWER_TRANSPORT_TCP)
				? (isTrack2 ? 2u : 0u)
				: (isTrack2 ? 5006u : 5004u);
		uint16_t ctrlPort = streamPort + 1u;

		/* stream_userdata = (void*)1 — non-NULL so play/pause/teardown
		 * callbacks don't trip on NULL checks. */
		rtsp_server_reply_to_setup(server,
					   request_ctx,
					   media_ctx,
					   ret,
					   streamPort,
					   ctrlPort,
					   1,
					   0x12345678u,
					   nullptr,
					   0,
					   reinterpret_cast<void *>(1));
	}

	static void playCb(struct rtsp_server *server,
			   const char *session_id,
			   const struct rtsp_header_ext * /*ext*/,
			   size_t /*ext_count*/,
			   void *request_ctx,
			   void *media_ctx,
			   const struct rtsp_range *range,
			   float scale,
			   void * /*stream_userdata*/,
			   void *userdata)
	{
		auto *self = static_cast<TestRtspServer *>(userdata);
		int ret = 0;
		struct rtsp_range resp_range = {};
		uint16_t seq = 0;
		uint32_t ts = 0;

		if (self && self->mPlayShouldFail) {
			ret = -ENOENT;
			goto reply;
		}
		if (!session_id || !range) {
			ret = -EINVAL;
			goto reply;
		}
		if (range->start.format != RTSP_TIME_FORMAT_NPT) {
			ret = -ENOSYS;
			goto reply;
		}
		if (scale == 0.f)
			scale = 1.f;
		resp_range = *range;
		/* Use ts=0 so mPlayNtpTime=0: any RTP frame with a non-zero
		 * timestamp passes the "silent before play point" guard in
		 * recvFrameCb.  Random ts would map to a large mPlayNtpTime
		 * that makes injected IDR frames (small ts) silent, blocking
		 * asyncCompleteSeek. */
		seq = 0;
		ts = 0;

		/* A real server translates an infinity stop to the actual
		 * track end.  Without this, internalPlay() sets
		 * mUpdateTrackDuration=true and onRtspPlayResp receives
		 * stop.infinity=1 → rtsp_time_npt_to_us fails → stop=0 →
		 * mTrackDuration overwritten to 0 (killing the range timer). */
		if (self && self->mHasDuration &&
		    resp_range.stop.npt.infinity) {
			resp_range.stop.npt.infinity = 0;
			resp_range.stop.npt.sec = 120;
			resp_range.stop.npt.usec = 0;
		}

		if (self && self->mPlayReturnsEndOfRange) {
			/* Return start == stop == 120s (both non-zero) to make
			 * onRtspPlayResp trigger idleEndOfRangeNotification. */
			static constexpr uint64_t kEndUs = 120000000u;
			resp_range.start.format = RTSP_TIME_FORMAT_NPT;
			rtsp_time_us_to_npt(kEndUs, &resp_range.start.npt);
			resp_range.stop.format = RTSP_TIME_FORMAT_NPT;
			rtsp_time_us_to_npt(kEndUs, &resp_range.stop.npt);
		}

	reply:
		rtsp_server_reply_to_play(server,
					  request_ctx,
					  media_ctx,
					  ret,
					  &resp_range,
					  scale,
					  1,
					  seq,
					  1,
					  ts,
					  nullptr,
					  0);
	}

	static void pauseCb(struct rtsp_server *server,
			    const char *session_id,
			    const struct rtsp_header_ext * /*ext*/,
			    size_t /*ext_count*/,
			    void *request_ctx,
			    void *media_ctx,
			    const struct rtsp_range *range,
			    void * /*stream_userdata*/,
			    void * /*userdata*/)
	{
		int ret = 0;
		struct rtsp_range resp_range = {};

		if (!session_id || !range)
			ret = -EINVAL;
		else
			resp_range = *range;

		rtsp_server_reply_to_pause(server,
					   request_ctx,
					   media_ctx,
					   ret,
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
			       void * /*userdata*/)
	{
		if (request_ctx)
			rtsp_server_reply_to_teardown(
				server, request_ctx, media_ctx, 0, nullptr, 0);
	}

	static void requestTimeoutCb(struct rtsp_server * /*server*/,
				     void * /*request_ctx*/,
				     enum rtsp_method_type /*method*/,
				     void * /*userdata*/)
	{
	}
};


/* ── Demuxer listeners ─────────────────────────────────────────────────── */

/* Records the async RTSP lifecycle events needed by test assertions. */
class StreamDemuxerListener : public IPdraw::IDemuxer::Listener {
public:
	void demuxerOpenResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status) override
	{
		mOpenStatus = status;
		mGotOpenResponse = true;
	}

	void demuxerCloseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int status) override
	{
		mCloseStatus = status;
		mGotCloseResponse = true;
	}

	void onDemuxerUnrecoverableError(IPdraw * /*p*/,
					 IPdraw::IDemuxer * /*d*/) override
	{
		mGotUnrecoverableError = true;
	}

	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*medias*/,
			       size_t /*count*/,
			       uint32_t /*selected*/) override
	{
		/* -ENOSYS: demuxer auto-selects the single default media.
		 * Only works when the SDP contains exactly 1 media track
		 * (mediasCount == 1 in onNewSdp sets is_default). */
		return -ENOSYS;
	}

	void demuxerReadyToPlay(IPdraw * /*p*/,
				IPdraw::IDemuxer * /*d*/,
				bool ready) override
	{
		mReady = ready;
		mGotReadyToPlay = true;
	}

	void onDemuxerEndOfRange(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 uint64_t /*ts*/) override
	{
	}

	void demuxerPlayResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t /*timestamp*/,
				 float /*speed*/) override
	{
		mPlayStatus = status;
		mGotPlayResponse = true;
	}

	void demuxerPauseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int status,
				  uint64_t /*timestamp*/) override
	{
		mPauseStatus = status;
		mGotPauseResponse = true;
	}

	void demuxerSeekResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		mSeekStatus = status;
		mSeekTimestamp = timestamp;
		mSeekSpeed = speed;
		mGotSeekResponse = true;
	}

	bool mGotOpenResponse = false;
	int mOpenStatus = 0;
	bool mGotReadyToPlay = false;
	bool mReady = false;
	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
	bool mGotUnrecoverableError = false;
	bool mGotPlayResponse = false;
	int mPlayStatus = 0;
	bool mGotPauseResponse = false;
	int mPauseStatus = 0;
	bool mGotSeekResponse = false;
	int mSeekStatus = 0;
	uint64_t mSeekTimestamp = 0;
	float mSeekSpeed = 0.f;
};


/* Variant for two-media SDPs: explicitly selects the first video media by ID,
 * exercising the `res >= 0 → selectedMedias = res` branch in onNewSdp (A2). */
class SelectFirstMediaDemuxerListener : public StreamDemuxerListener {
public:
	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t /*selected*/) override
	{
		/* Return the bitfield for the first video media.
		 * media_id values are 1-based (set as i+1 in onNewSdp). */
		for (size_t i = 0; i < count; i++) {
			if (medias[i].type == PDRAW_MEDIA_TYPE_VIDEO)
				return static_cast<int>(1u
							<< medias[i].media_id);
		}
		return -ENOSYS;
	}
};


/* Session-wide listener that records all onMediaAdded events.
 * Provided here for the future RTP-injection tests (Group B): after SETUP the
 * demuxer eventually calls codecInfoChangedCb, which creates a CodedVideoMedia
 * and fires onMediaAdded — the test needs the resulting pipeline media ID to
 * attach a CodedVideoSink.  Not used by the current signaling-only tests. */
class MediaTrackingListener : public IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		enum pdraw_media_type type;
		enum vdef_frame_type videoFormat;
	};

	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*elementUserData*/) override
	{
		Added a = {};
		a.id = info->id;
		a.type = info->type;
		if (info->type == PDRAW_MEDIA_TYPE_VIDEO)
			a.videoFormat = info->video.format;
		mAdded.push_back(a);
	}

	/* First raw video media added, or nullptr if none yet -- e.g. a
	 * VideoDecoder's own (lazily created) output media, only added once
	 * decoding actually produces a first frame (autodecoding_mode =
	 * DECODE_ALL). Deliberately distinct from the demuxer's own coded
	 * video media (added first, at ready-to-play time). */
	const Added *findRawVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_RAW)
				return &a;
		}
		return nullptr;
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*info*/,
			    void * /*elementUserData*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Minimal coded video sink listener used by the Group F (real-sink) tests
 * below. Must ack flush/drain via queueFlushed()/queueDrained() -- an empty
 * or no-op onCodedVideoSinkFlush/Drain hangs the pipeline forever, since
 * Channel::flushDone()/drainDone() (and therefore
 * StreamDemuxer::onChannelFlushed/onChannelDrained) are only reached through
 * this application-level ack, never automatically. Per-file-local copy of
 * the same shape as DrainingCodedVideoSinkListener in test_api_demuxer.cpp /
 * QueueDrainingCodedVideoSinkListener in test_pipeline_sourcesink_coded.cpp
 * (anonymous namespace: see this file's top-of-file comment on avoiding ODR
 * violations across sibling test TUs). */
class DrainingCodedVideoSinkListener
		: public IPdraw::ICodedVideoSink::Listener {
public:
	void onCodedVideoSinkMediaAdded(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct pdraw_media_info * /*info*/) override
	{
		mGotMediaAdded = true;
	}

	void
	onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
				     IPdraw::ICodedVideoSink * /*sk*/,
				     const struct pdraw_media_info * /*info*/,
				     bool restart) override
	{
		mGotMediaRemoved = true;
		mLastRemovedRestart = restart;
	}

	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		discardQueue();
		mGotFlush = true;
		sk->queueFlushed();
	}

	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink *sk) override
	{
		discardQueue();
		mGotDrain = true;
		sk->queueDrained();
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}

	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_coded_video_frame_unref(f);
	}

	struct mbuf_coded_video_frame_queue *mQueue = nullptr;
	bool mGotMediaAdded = false;
	bool mGotMediaRemoved = false;
	bool mLastRemovedRestart = false;
	bool mGotFlush = false;
	bool mGotDrain = false;
};


/* Raw video sink listener used by the session-metadata test below, attached
 * downstream of the internally auto-created VideoDecoder (autodecoding_mode
 * = DECODE_ALL). Same acking rationale as DrainingCodedVideoSinkListener
 * above: must ack flush/drain for a clean teardown, even though this test's
 * own assertions are about onRawVideoSinkSessionMetaUpdate. */
class TrackingRawVideoSinkListener : public IPdraw::IRawVideoSink::Listener {
public:
	void
	onRawVideoSinkMediaAdded(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/,
				 const struct pdraw_media_info * /*i*/) override
	{
	}

	void onRawVideoSinkMediaRemoved(IPdraw * /*p*/,
					IPdraw::IRawVideoSink * /*sk*/,
					const struct pdraw_media_info * /*i*/,
					bool /*restart*/) override
	{
	}

	void onRawVideoSinkFlush(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		discardQueue();
		sk->queueFlushed();
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		discardQueue();
		sk->queueDrained();
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session *meta) override
	{
		if (meta != nullptr)
			mMeta = *meta;
		mGotSessionMetaUpdate = true;
	}

	struct mbuf_raw_video_frame_queue *mQueue = nullptr;
	struct vmeta_session mMeta = {};
	bool mGotSessionMetaUpdate = false;

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_raw_video_frame *f = nullptr;
		while (mbuf_raw_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_raw_video_frame_unref(f);
	}
};


/* ── Shared helper: open demuxer and pump until openResponse(0) ─────── */

static IPdraw::IDemuxer *
openStreamDemuxer(IPdraw *session,
		  TestPompLoop &loop,
		  const std::string &url,
		  StreamDemuxerListener *listener,
		  int timeoutMs = 10000,
		  enum pdraw_demuxer_autodecoding_mode mode =
			  PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE)
{
	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = mode;

	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(url, &params, listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotOpen = loop.pumpUntil(
		[listener]() { return listener->mGotOpenResponse; }, timeoutMs);
	CU_ASSERT_TRUE_FATAL(gotOpen);

	return demuxer;
}

/* Close the demuxer and pump until demuxerCloseResponse fires. */
static void closeStreamDemuxer(IPdraw::IDemuxer *demuxer,
			       TestPompLoop &loop,
			       StreamDemuxerListener *listener,
			       int timeoutMs = 10000)
{
	auto demuxerOwner = std::unique_ptr<IPdraw::IDemuxer>(demuxer);
	int ret = demuxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[listener]() { return listener->mGotCloseResponse; },
		timeoutMs);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);
	demuxerOwner.reset();
}


/* Send one RTP UDP packet carrying a single H.264 NAL unit (single-NAL
 * unit packet mode, RFC 6184 §5.6).  The 12-byte RTP header is followed
 * by the raw NAL unit bytes (no start-code or size prefix).
 * dataCb() drains all pending datagrams in one while(true) loop so both
 * SPS and PPS packets sent before pumpUntil() are processed together.
 * Set marker=true to set the RTP marker bit (last packet of an access unit). */
static void sendRtpNaluPacket(uint16_t dstPort,
			      uint32_t ssrc,
			      uint16_t seq,
			      uint32_t ts,
			      const uint8_t *payload,
			      size_t payloadLen,
			      bool marker = false)
{
	std::vector<uint8_t> buf(12 + payloadLen);
	buf[0] = 0x80; /* V=2,P=0,X=0,CC=0 */
	buf[1] =
		static_cast<uint8_t>(96 | (marker ? 0x80u : 0u)); /* M, PT=96 */
	buf[2] = static_cast<uint8_t>((seq >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(seq & 0xFF);
	buf[4] = static_cast<uint8_t>((ts >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ts >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ts >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ts & 0xFF);
	buf[8] = static_cast<uint8_t>((ssrc >> 24) & 0xFF);
	buf[9] = static_cast<uint8_t>((ssrc >> 16) & 0xFF);
	buf[10] = static_cast<uint8_t>((ssrc >> 8) & 0xFF);
	buf[11] = static_cast<uint8_t>(ssrc & 0xFF);
	std::copy(payload, payload + payloadLen, buf.begin() + 12);
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(dstPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       buf.data(),
	       buf.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}


/* Send a raw RTCP BYE packet (RFC 3550 §6.6, one SSRC + optional reason) to
 * the demuxer's RTCP control port. Exercises
 * StreamDemuxer::VideoMedia::goodbyeCb(): vstrm_receiver_rtcp_bye_cb()
 * (vstrm_receiver.c) requires bye.sources[0] == the SSRC already tracked by
 * the receiver, i.e. at least one RTP data packet with that SSRC must have
 * been processed first (see sendRtpNaluPacket() above). Header bit layout
 * from librtp's rtcp_pkt.h (RTCP_PKT_VERSION=2, shift=6; BYE=203); built by
 * hand rather than via rtcp_pkt_write_bye() to match this file's existing
 * self-contained style (see sendRtpNaluPacket()). */
static void
sendRtcpBye(uint16_t dstPort, uint32_t ssrc, const std::string &reason)
{
	std::vector<uint8_t> buf(8, 0);
	buf[0] = 0x81; /* V=2, P=0, SC=1 */
	buf[1] = 203; /* PT=BYE */
	buf[4] = static_cast<uint8_t>((ssrc >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ssrc >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ssrc >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ssrc & 0xFF);
	if (!reason.empty()) {
		buf.push_back(static_cast<uint8_t>(reason.size()));
		buf.insert(buf.end(), reason.begin(), reason.end());
		while (buf.size() % 4 != 0)
			buf.push_back(0);
	}
	uint16_t length = static_cast<uint16_t>(buf.size() / 4 - 1);
	buf[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(length & 0xFF);

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(dstPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       buf.data(),
	       buf.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}


/* Send a raw RTCP APP packet (RFC 3550 §6.7) carrying a vstrm "event"
 * sub-message, to the demuxer's RTCP control port. Exercises
 * StreamDemuxer::VideoMedia::eventCb(). Wire format and constants from the
 * *internal* libvideo-streaming headers src/vstrm_rtcp_app.h
 * (VSTRM_RTCP_APP_PACKET_NAME = 0x41525354 = "ARST",
 * VSTRM_RTCP_APP_PACKET_SUBTYPE_EVENT = 3) and src/vstrm_event.h
 * (VSTRM_EVENT_MSG_VERSION = 1, 2-byte payload: version, event id) --
 * inlined here since those headers are not part of the public API; the
 * event id values themselves (e.g. VSTRM_EVENT_RECONFIGURE = 1) are public,
 * see include/video-streaming/vstrm_events.h. */
static void sendRtcpEvent(uint16_t dstPort, uint32_t ssrc, uint8_t vstrmEvent)
{
	static constexpr uint32_t kVstrmRtcpAppName = 0x41525354u; /* "ARST" */
	static constexpr uint8_t kVstrmRtcpAppSubtypeEvent = 3;
	static constexpr uint8_t kVstrmEventMsgVersion = 1;

	std::vector<uint8_t> buf(12, 0);
	buf[0] = static_cast<uint8_t>(0x80 | kVstrmRtcpAppSubtypeEvent);
	buf[1] = 204; /* PT=APP */
	buf[4] = static_cast<uint8_t>((ssrc >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ssrc >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ssrc >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ssrc & 0xFF);
	buf[8] = static_cast<uint8_t>((kVstrmRtcpAppName >> 24) & 0xFF);
	buf[9] = static_cast<uint8_t>((kVstrmRtcpAppName >> 16) & 0xFF);
	buf[10] = static_cast<uint8_t>((kVstrmRtcpAppName >> 8) & 0xFF);
	buf[11] = static_cast<uint8_t>(kVstrmRtcpAppName & 0xFF);
	buf.push_back(kVstrmEventMsgVersion);
	buf.push_back(vstrmEvent);
	while (buf.size() % 4 != 0)
		buf.push_back(0);
	uint16_t length = static_cast<uint16_t>(buf.size() / 4 - 1);
	buf[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(length & 0xFF);

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(dstPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       buf.data(),
	       buf.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}


/* Send a raw RTCP SDES packet (RFC 3550 §6.5, one chunk) carrying the
 * minimal PDrAW/Parrot session metadata to the demuxer's RTCP control port.
 * Exercises StreamDemuxer::VideoMedia::sessionMetadataPeerChangedCb()
 * (vstrm_receiver's session_metadata_peer_changed callback), which only
 * fires once vmeta_session_is_valid() passes: friendly_name non-empty, maker
 * == "Parrot" exactly, model non-empty (vmeta_session.c:4138-4152). Unlike
 * sendRtcpBye() (whose SSRC must match a previously-seen RTP packet's),
 * vstrm_receiver_rtcp_sdes_item_cb() ignores the chunk's SSRC entirely
 * (vstrm_receiver.c:171-180, "UNUSED(ssrc)") -- but session_metadata_peer is
 * memset to zero at the start of *every* incoming RTCP packet
 * (vstrm_receiver_recv_ctrl(), vstrm_receiver.c:1158) and only survives if
 * still valid after parsing that one packet, so all three mandatory items
 * must be present together in this single packet. Item wire format (RFC
 * 3550 §6.5, §6.5.8): type(1B) + len(1B) + data(len bytes); PRIV item data
 * is prefix_len(1B) + prefix + value under that one overall length byte
 * (rtcp_pkt_read_sdes_item(), librtp/src/rtcp_pkt.c:709-756). Built by hand
 * rather than via vstrm_session_metadata_write_rtcp_sdes() to match this
 * file's existing self-contained style (see sendRtcpBye()/sendRtcpEvent()
 * above); PRIV prefixes "maker"/"model" from
 * libvideo-metadata/include/video-metadata/vmeta_session.h
 * (VMETA_STRM_SDES_KEY_MAKER/_MODEL). */
static void sendRtcpSdes(uint16_t dstPort,
			 uint32_t ssrc,
			 const std::string &friendlyName,
			 const std::string &model)
{
	static const std::string kMaker = "Parrot";
	static const std::string kMakerPrefix = "maker";
	static const std::string kModelPrefix = "model";

	std::vector<uint8_t> buf(8, 0);
	buf[0] = 0x81; /* V=2, P=0, SC=1 (one chunk) */
	buf[1] = 202; /* PT=SDES */
	buf[4] = static_cast<uint8_t>((ssrc >> 24) & 0xFF);
	buf[5] = static_cast<uint8_t>((ssrc >> 16) & 0xFF);
	buf[6] = static_cast<uint8_t>((ssrc >> 8) & 0xFF);
	buf[7] = static_cast<uint8_t>(ssrc & 0xFF);

	/* NAME item (type=2): value bytes only, wire length = value size. */
	buf.push_back(2);
	buf.push_back(static_cast<uint8_t>(friendlyName.size()));
	buf.insert(buf.end(), friendlyName.begin(), friendlyName.end());

	/* PRIV items (type=8): data = prefix_len(1B) + prefix + value. */
	buf.push_back(8);
	buf.push_back(
		static_cast<uint8_t>(1 + kMakerPrefix.size() + kMaker.size()));
	buf.push_back(static_cast<uint8_t>(kMakerPrefix.size()));
	buf.insert(buf.end(), kMakerPrefix.begin(), kMakerPrefix.end());
	buf.insert(buf.end(), kMaker.begin(), kMaker.end());

	buf.push_back(8);
	buf.push_back(
		static_cast<uint8_t>(1 + kModelPrefix.size() + model.size()));
	buf.push_back(static_cast<uint8_t>(kModelPrefix.size()));
	buf.insert(buf.end(), kModelPrefix.begin(), kModelPrefix.end());
	buf.insert(buf.end(), model.begin(), model.end());

	buf.push_back(0); /* END marker (RTCP_PKT_SDES_TYPE_END) */
	while (buf.size() % 4 != 0)
		buf.push_back(0);

	uint16_t length = static_cast<uint16_t>(buf.size() / 4 - 1);
	buf[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
	buf[3] = static_cast<uint8_t>(length & 0xFF);

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return;
	struct sockaddr_in dst = {};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(dstPort);
	inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
	sendto(sock,
	       buf.data(),
	       buf.size(),
	       0,
	       reinterpret_cast<struct sockaddr *>(&dst),
	       sizeof(dst));
	close(sock);
}


/* ── Tests ─────────────────────────────────────────────────────────────── */

/* Full RTSP lifecycle with a single auto-selected media:
 *   CONNECT → OPTIONS → DESCRIBE → SETUP → openResponse(0) + readyToPlay
 *   getMediaList() [A3]
 *   PLAY → playResponse(0)
 *   PAUSE → pauseResponse(0)
 *   TEARDOWN → closeResponse(0)
 *
 * Also verifies getSingleStreamLocalStreamPort/ControlPort() > 0 (UDP sockets
 * were bound by StreamDemuxerNet::VideoMediaNet::startRtpAvp). */
static void testCxxStreamDemuxerRtspSignalingLifecycle()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);

	CU_ASSERT_EQUAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	/* A3: getMediaList() after openResponse. With a single-track SDP the
	 * demuxer must report 1 media selected by default. */
	{
		struct pdraw_demuxer_media *mediaList = nullptr;
		size_t mediaCount = 0;
		uint32_t selectedMedias = 0;
		int ret = demuxer->getMediaList(
			&mediaList, &mediaCount, &selectedMedias);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT(mediaCount >= 1);
		CU_ASSERT_NOT_EQUAL(selectedMedias, 0u);
		pdraw_demuxerMediaListFree(mediaList, mediaCount);
	}

	bool gotReady = loop.pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; }, 5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(listener.mReady);
	CU_ASSERT_TRUE(demuxer->isReadyToPlay());

	/* StreamDemuxerNet binds real UDP sockets — ports must be non-zero. */
	CU_ASSERT(demuxer->getSingleStreamLocalStreamPort() > 0);
	CU_ASSERT(demuxer->getSingleStreamLocalControlPort() > 0);

	/* PLAY */
	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(listener.mPlayStatus, 0);

	/* PAUSE — safe after pumpUntil(playResponse): mRunning == true */
	ret = demuxer->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = loop.pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; }, 5000);
	CU_ASSERT_TRUE(gotPause);
	CU_ASSERT_EQUAL(listener.mPauseStatus, 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Two-media SDP + explicit demuxerSelectMedia callback (A2):
 *   Server returns SDP with stream=0 and stream=1.
 *   SelectFirstMediaDemuxerListener selects only stream=0 by media_id bitfield.
 *   This exercises:
 *     - onNewSdp with mediasCount == 2 (neither track is auto-default)
 *     - callSelectMedia → res >= 0 → selectedMedias = res branch
 *     - processSelectedMedias with exactly 1 selected track → single SETUP
 *   After open, getMediaList() must report 2 tracks but only 1 selected. */
static void testCxxStreamDemuxerTwoMediaSelectFirst()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	/* mediaCount=2: DESCRIBE returns stream=0 and stream=1. */
	TestRtspServer server(loop.raw(), kTestRtspPort, 2);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	SelectFirstMediaDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);

	CU_ASSERT_EQUAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	/* getMediaList(): 2 tracks described, but only 1 selected (stream=0).
	 */
	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret =
		demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(mediaCount, 2u);
	/* Exactly one media should be selected. */
	CU_ASSERT_EQUAL(__builtin_popcount(selectedMedias), 1);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	bool gotReady = loop.pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; }, 5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(listener.mReady);

	/* Only 1 SETUP was sent (stream=0) → getSingleStreamLocal*Port > 0. */
	CU_ASSERT(demuxer->getSingleStreamLocalStreamPort() > 0);
	CU_ASSERT(demuxer->getSingleStreamLocalControlPort() > 0);

	/* PLAY → TEARDOWN */
	ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(listener.mPlayStatus, 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* RTP in-band SPS/PPS → onMediaAdded (B1):
 *   After PLAY, two RTP UDP packets carrying H.264 SPS (NAL type 7) then
 *   PPS (NAL type 8) are sent to getSingleStreamLocalStreamPort().
 *
 *   Path exercised:
 *     vstrm_receiver_recv_data (first packet) → init_source → init_seq →
 *     vstrm_rtp_h264_rx_clear (codec_info zeroed)
 *     nalu_complete (SPS) → h264_reader_parse_nalu → sps_cb → sps.valid=true
 *     nalu_complete (PPS) → h264_reader_parse_nalu → pps_cb → pps.valid=true
 *       → pps_received → codec_info_changed (new != zeroed)
 *       → codecInfoChangedCb → setupMedia → onOutputMediaAdded
 *       → IPdraw::Listener::onMediaAdded
 *
 *   dataCb() drains all pending UDP datagrams in one while(true) loop, so
 *   both SPS and PPS are processed in a single pumpUntil() iteration. */
static void testCxxStreamDemuxerRtpCodecInfo()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(demuxListener.mPlayStatus, 0);

	/* Send SPS then PPS as back-to-back UDP datagrams.  dataCb() drains
	 * both in a single while(true) loop pass: SPS sets sps.valid=true;
	 * PPS triggers pps_received → codec_info_changed → setupMedia →
	 * onMediaAdded, all synchronously on the pomp loop thread. */
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kRtpTs = 90000u; /* 1 s @ 90 kHz */
	sendRtpNaluPacket(
		rtpPort, kTestRtspSsrc, 1, kRtpTs, kH264Sps, sizeof(kH264Sps));
	sendRtpNaluPacket(
		rtpPort, kTestRtspSsrc, 2, kRtpTs, kH264Pps, sizeof(kH264Pps));

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE(gotMedia);
	CU_ASSERT(mediaListener.mAdded.size() >= 1);
	if (!mediaListener.mAdded.empty())
		CU_ASSERT_EQUAL(mediaListener.mAdded[0].type,
				PDRAW_MEDIA_TYPE_VIDEO);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* DESCRIBE error (A1): server returns 404 on DESCRIBE (wrong resource path).
 * Exercises onRtspDescribeResp(RTSP_CLIENT_REQ_STATUS_FAILED) →
 *   onUnrecoverableError → openResponse(error) with non-zero status.
 * After the error, close() must still work and fire closeResponse. */
static void testCxxStreamDemuxerDescribeError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	/* Server is running on the standard port — but the URL intentionally
	 * points to "/nonexistent" so the server returns -ENOENT on DESCRIBE.
	 */
	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	/* "nonexistent" path → describeCb's strcmp fails → ret = -ENOENT → 404
	 */
	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/nonexistent";

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(url, &params, &listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	/* onRtspDescribeResp(FAILED) → onUnrecoverableError → openResponse(err)
	 * Since !mCalledOpenResp, demuxerOpenResponse fires (not
	 * onDemuxerUnrecoverableError). Status must be non-zero. */
	bool gotError = loop.pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; }, 10000);
	CU_ASSERT_TRUE(gotError);
	CU_ASSERT(listener.mOpenStatus != 0);

	/* After a DESCRIBE failure, mRtspState == OPTIONS_DONE; close() must
	 * send asyncRtspDisconnect → onRtspConnectionState(DISCONNECTED) →
	 * tryCompleteStop → closeResponse(0). */
	closeStreamDemuxer(demuxer, loop, &listener);
}


/* SETUP error (C2):
 *   Server returns -ENOENT in setupCb (mSetupShouldFail=true).
 *   Exercises onRtspSetupResp FAILED branch → onUnrecoverableError →
 *   openResponse(error) since mCalledOpenResp is still false at SETUP time.
 *   Status must be non-zero; close() must still complete cleanly. */
static void testCxxStreamDemuxerSetupError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mSetupShouldFail = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(url, &params, &listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	/* onRtspSetupResp(FAILED) → onUnrecoverableError → openResponse(err).
	 */
	bool gotOpen = loop.pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; }, 10000);
	CU_ASSERT_TRUE(gotOpen);
	CU_ASSERT(listener.mOpenStatus != 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* PLAY error (C3):
 *   DESCRIBE + SETUP succeed; server returns -ENOENT in playCb
 *   (mPlayShouldFail=true).
 *   Exercises onRtspPlayResp FAILED branch → playResponse(error, ...).
 *   openResponse(0) must have fired before play(); play must report error. */
static void testCxxStreamDemuxerPlayError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mPlayShouldFail = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT(listener.mPlayStatus != 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* PLAY → SEEK (C4):
 *   SDP carries a=range:npt=0-120 so getDuration() > 0 (replay stream).
 *   After PLAY succeeds, seekTo(0) sends a second RTSP PLAY.
 *
 *   seekTo() calls VideoMedia::seek() on every RTP-level VideoMedia, setting
 *   mPendingSeek=true.  onMediaSeekComplete() skips seekResponse while any
 *   VideoMedia is still seeking (anySeeking=true).  mPendingSeek is only
 *   cleared inside recvFrameCb via asyncCompleteSeek() when a non-silent RTP
 *   frame arrives after the seek PLAY response (mSeekingNetwork=false).
 *
 *   So the test must:
 *     1. Send SPS+PPS before play → codec_info_changed → setupMedia (creates
 *        the CodedVideoMedia needed by recvFrameCb's memory allocation path).
 *     2. After seekTo, pump the loop while injecting IDR frames each turn.
 *        The first IDR after the seek PLAY response clears mWaitForSync
 *        (IDR clears it from recvFrameCb) and triggers asyncCompleteSeek →
 *        completeSeek → onMediaSeekComplete(0) → seekResponse → listener.
 *
 *   Path exercised:
 *     seekTo (guards, RTSP PLAY with NPT range)
 *     onRtspPlayResp mSeeking=true → mSeekingNetwork=false
 *     recvFrameCb → asyncCompleteSeek → completeSeek → onMediaSeekComplete
 *     seekResponse → callSeekResponse → demuxerSeekResponse(0, ts, speed) */
static void testCxxStreamDemuxerSeek()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true; /* replay stream: getDuration() > 0 */

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	/* PLAY first — vstrm_receiver only accepts RTP after PLAY, and
	 * seekTo requires mRunning=true. */
	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(demuxListener.mPlayStatus, 0);

	/* Send SPS then PPS so that codecInfoChanged fires and setupMedia
	 * creates a CodedVideoMedia entry in VideoMedia::mVideoMedias.
	 * recvFrameCb needs a non-empty CodedVideoMedia list to allocate output
	 * memory; without it the function returns early and asyncCompleteSeek
	 * is never reached. */
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* seekTo(0): sends RTSP PLAY with range=[0,∞); sets mPendingSeek=true
	 * and mSeekingNetwork=true.  Pump first to let the PLAY response clear
	 * mSeekingNetwork, then inject IDRs until asyncCompleteSeek fires. */
	ret = demuxer->seekTo(0, false);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Pump 200 ms so the RTSP PLAY response arrives and clears
	 * mSeekingNetwork (asyncCompleteSeek requires !mSeekingNetwork). */
	(void)loop.pumpUntil([]() { return false; }, 200);

	/* Send one IDR per 60 ms iteration (> ~30 ms vstrm jitter timer).
	 * Each IDR is released independently by the jitter timer; processFrame
	 * fires asyncCompleteSeek → seekResponse once mSeekingNetwork=false.
	 * Start at seq=3 (immediately after PPS seq=2) to avoid a reorder-
	 * buffer gap that would delay processing. */
	{
		uint32_t idrTs = 180000u;
		uint16_t idrSeq = 3;
		auto deadline = std::chrono::steady_clock::now() +
				std::chrono::milliseconds(2000);
		while (!demuxListener.mGotSeekResponse) {
			if (std::chrono::steady_clock::now() >= deadline)
				break;
			sendRtpNaluPacket(rtpPort,
					  kTestRtspSsrc,
					  idrSeq++,
					  idrTs,
					  kH264Idr,
					  sizeof(kH264Idr),
					  true /* marker */);
			idrTs += 3000;
			(void)loop.pumpUntil([]() { return false; }, 60);
		}
	}

	CU_ASSERT_TRUE(demuxListener.mGotSeekResponse);
	CU_ASSERT_EQUAL(demuxListener.mSeekStatus, 0);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* Wait for demuxerReadyToPlay(true): play()/previousFrame()/nextFrame()/
 * seekTo() all require isReadyToPlay() == true (StreamDemuxer::play() etc.,
 * pdraw_demuxer_stream.cpp), but openStreamDemuxer() only waits for
 * openResponse -- readyToPlay is a separate, later idle-dispatched event
 * (see testCxxStreamDemuxerRtspSignalingLifecycle above, which already
 * waits for it explicitly before calling play()). */
static void waitReadyToPlay(TestPompLoop &loop, StreamDemuxerListener &listener)
{
	bool gotReady = loop.pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; }, 5000);
	CU_ASSERT_TRUE(gotReady);
}


/* Pump the loop for a bounded, fixed duration with no completion predicate
 * to wait on (e.g. letting an async RTSP round-trip or a one-shot RTCP
 * event settle before proceeding). pumpUntil() is [[nodiscard]], so a
 * throwaway `[]() { return false; }` predicate needs its result consumed
 * explicitly. */
static void pumpFor(TestPompLoop &loop, int ms)
{
	bool timedOut = !loop.pumpUntil([]() { return false; }, ms);
	(void)timedOut;
}


/* ── Group E: demuxer-level tests (no attached sink) ──────────────────── */

/* Empty SDP (mediasCount == 0 in onNewSdp, pdraw_demuxer_stream.cpp
 * ~line 2123): server returns a valid SDP session with zero supported media
 * tracks. The demuxer still completes tryCompleteStart() -> openResponse(0)
 * (called from inside the mediasCount==0 branch itself), then
 * readyToPlay(false) at the "stop:" label -- which is a no-op dispatch since
 * mReadyToPlay already defaults to false (Demuxer::readyToPlay() only fires
 * the callback "on changes"), so demuxerReadyToPlay() is never actually
 * invoked here; isReadyToPlay() is checked directly instead. getMediaList()
 * must report zero medias (-ENOENT, mMediaListSize == 0). */
static void testCxxStreamDemuxerEmptySdp()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mEmptySdp = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);

	CU_ASSERT_EQUAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);
	CU_ASSERT_FALSE(demuxer->isReadyToPlay());

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret =
		demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL(ret, -ENOENT);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Variant for the -ECANCELED selectMedia() callback test below: the
 * application cancels media selection unconditionally. */
class CancelSelectMediaDemuxerListener : public StreamDemuxerListener {
public:
	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*medias*/,
			       size_t /*count*/,
			       uint32_t /*selected*/) override
	{
		return -ECANCELED;
	}
};


/* demuxerSelectMedia() returning -ECANCELED: onNewSdp's callSelectMedia()
 * branch (pdraw_demuxer_stream.cpp ~line 2246) sets noError=true and jumps
 * to "stop", where openResponse(res) fires with res == -ECANCELED (since
 * mCalledOpenResp is still false at this point) -- not
 * onDemuxerUnrecoverableError. */
static void testCxxStreamDemuxerSelectMediaCancelled()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	CancelSelectMediaDemuxerListener listener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(url, &params, &listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotOpen = loop.pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; }, 10000);
	CU_ASSERT_TRUE(gotOpen);
	CU_ASSERT_EQUAL(listener.mOpenStatus, -ECANCELED);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Variant for the generic-negative selectMedia() callback test below: the
 * application rejects the selection with an arbitrary error that is neither
 * -ENOSYS nor -ECANCELED. */
class RejectSelectMediaDemuxerListener : public StreamDemuxerListener {
public:
	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*medias*/,
			       size_t /*count*/,
			       uint32_t /*selected*/) override
	{
		return -EIO;
	}
};


/* demuxerSelectMedia() returning a generic negative error (neither -ENOSYS
 * nor -ECANCELED): onNewSdp's callSelectMedia() branch (pdraw_demuxer_stream.
 * cpp ~line 2250-2251, "application failed to select a media") leaves
 * noError == false and jumps to "stop", where `if (!noError)
 * onUnrecoverableError();` runs (line 2286) -- as opposed to -ECANCELED,
 * which sets noError = true and takes the `else if (!mCalledOpenResp)
 * openResponse(res)` branch instead.
 *
 * Demuxer::onUnrecoverableError(int error = -EPROTO) (pdraw_demuxer.cpp) is
 * called here with NO argument, so it uses its default -EPROTO regardless of
 * the actual value returned by the callback (-EIO here): since
 * mCalledOpenResp is still false at this point, it calls openResponse(-EPROTO)
 * directly instead of scheduling onDemuxerUnrecoverableError. So the
 * observable status is -EPROTO, not -EIO -- the key difference from the
 * -ECANCELED test above, which preserves the callback's own return value. */
static void testCxxStreamDemuxerSelectMediaRejected()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE;

	RejectSelectMediaDemuxerListener listener;
	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(url, &params, &listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotOpen = loop.pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; }, 10000);
	CU_ASSERT_TRUE(gotOpen);
	CU_ASSERT_EQUAL(listener.mOpenStatus, -EPROTO);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* isPaused(): reflects mRunning && !mFrameByFrame (inverted). Paused by
 * default (mFrameByFrame starts true); not paused right after play(); paused
 * again after pause(). Also covers isReadyToPlay()'s and isPaused()'s shared
 * "not started" guard (pdraw_demuxer_stream.cpp:2444-2446/2455-2457),
 * previously 0% covered: every existing call to either happens after
 * waitReadyToPlay(), when mState is always STARTED. */
static void testCxxStreamDemuxerIsPaused()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	waitReadyToPlay(loop, listener);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Not started. */
	sd->setState(Pdraw::Element::State::STOPPING);
	CU_ASSERT_FALSE(demuxer->isReadyToPlay());
	CU_ASSERT_FALSE(demuxer->isPaused());
	sd->setState(Pdraw::Element::State::STARTED);

	CU_ASSERT_TRUE(demuxer->isPaused());

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_FALSE(demuxer->isPaused());

	ret = demuxer->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = loop.pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; }, 5000);
	CU_ASSERT_TRUE(gotPause);
	CU_ASSERT_TRUE(demuxer->isPaused());

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* getDuration(): 0 for a live (non-replay) SDP; matches a=range:npt=0-120
 * (120 s, in microseconds) for a replay stream. */
static void testCxxStreamDemuxerDuration()
{
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		CU_ASSERT_EQUAL(demuxer->getDuration(), 0u);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());
		server.mHasDuration = true;

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		CU_ASSERT_EQUAL(demuxer->getDuration(),
				static_cast<uint64_t>(120) * 1000000);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
}


/* selectMedia() to switch the selected media after open: 2-media SDP,
 * initial selection is stream=0/media_id=1 (SelectFirstMediaDemuxerListener
 * picks the first video media), then demuxer->selectMedia() switches to
 * stream=1/media_id=2. Exercises the top-level StreamDemuxer::selectMedia()
 * API, as opposed to the demuxerSelectMedia() application callback path
 * already covered by testCxxStreamDemuxerTwoMediaSelectFirst. */
static void testCxxStreamDemuxerSelectMediaSwitch()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 2);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	SelectFirstMediaDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	struct pdraw_demuxer_media *mediaList = nullptr;
	size_t mediaCount = 0;
	uint32_t selectedMedias = 0;
	int ret =
		demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 2u);
	CU_ASSERT_EQUAL(selectedMedias, 1u << 1);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);
	mediaList = nullptr;

	/* Switch the selection to stream=1 (media_id 2). Demuxer::
	 * selectMedia() updates mSelectedMedias synchronously (before any
	 * RTSP round-trip), so getMediaList() reflects the new selection
	 * immediately below. */
	ret = demuxer->selectMedia(1u << 2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = demuxer->getMediaList(&mediaList, &mediaCount, &selectedMedias);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(mediaCount, 2u);
	CU_ASSERT_EQUAL(selectedMedias, 1u << 2);
	pdraw_demuxerMediaListFree(mediaList, mediaCount);

	/* Let the SETUP (stream=1) / TEARDOWN (stream=0) RTSP round-trip
	 * triggered by the switch settle before closing. */
	pumpFor(loop, 500);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Server-initiated (forced) TEARDOWN: rtsp_server_force_teardown() pushes an
 * out-of-band TEARDOWN to the client, exercising
 * StreamDemuxer::onRtspForcedTeardown(). path=NULL tears down the *whole*
 * RTSP session server-side (not just one media's SETUP entry): confirmed by
 * a real run that the server then removes the session entirely
 * (rtsp_server: "server session ... removed"), which the client library
 * reports back, and the demuxer treats the resulting unexpected loss of its
 * only media as an unrecoverable error -- observable via
 * onDemuxerUnrecoverableError(). (Originally expected the demuxer to keep
 * running silently; a real compiled run showed otherwise -- this is the
 * correct, updated expectation.) */
static void testCxxStreamDemuxerForcedTeardown()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE_FATAL(server.mSessionId.empty());

	int ret = rtsp_server_force_teardown(
		server.raw(), server.mSessionId.c_str(), nullptr, nullptr, 0);
	CU_ASSERT_EQUAL(ret, 0);

	bool gotError = loop.pumpUntil(
		[&listener]() { return listener.mGotUnrecoverableError; },
		5000);
	CU_ASSERT_TRUE(gotError);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* previousFrame()/nextFrame() precondition guards and RTSP dispatch (no
 * wait on the async seek-completion callback, which requires injecting
 * matching RTP frames after the seek PLAY response -- see
 * testCxxStreamDemuxerSeek and TEST_PROGRESS.md for why that path is
 * currently unreliable):
 *   - getDuration() == 0 (live, non-replay) -> -ENOSYS
 *   - with mHasDuration=true: right after open, mFrameByFrame defaults to
 *     true (StreamDemuxer starts "paused"/frame-by-frame), so
 *     previousFrame()/nextFrame() dispatch successfully (ret == 0) without
 *     needing an explicit pause() first
 *   - a second call while the first SEEK command is still pending ->
 *     -EALREADY */
static void testCxxStreamDemuxerPreviousNextGuards()
{
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		waitReadyToPlay(loop, listener);

		CU_ASSERT_EQUAL(demuxer->previousFrame(), -ENOSYS);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -ENOSYS);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());
		server.mHasDuration = true;

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		waitReadyToPlay(loop, listener);

		Pdraw::DemuxerWrapper *wrapper =
			static_cast<Pdraw::DemuxerWrapper *>(demuxer);
		Pdraw::StreamDemuxer *sd = static_cast<Pdraw::StreamDemuxer *>(
			wrapper->getDemuxer());
		CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

		/* Not started (pdraw_demuxer_stream.cpp:2468-2470/2557-2559).
		 */
		sd->setState(Pdraw::Element::State::STOPPING);
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EPROTO);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EPROTO);
		sd->setState(Pdraw::Element::State::STARTED);

		/* Not ready to play (2472-2474/2561-2564). */
		sd->mReadyToPlay = false;
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EPROTO);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EPROTO);
		sd->mReadyToPlay = true;

		/* Not RTSP protocol (2485-2486/2574-2575). */
		sd->mSessionProtocol =
			Pdraw::StreamDemuxer::SessionProtocol::NONE;
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -ENOSYS);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -ENOSYS);
		sd->mSessionProtocol =
			Pdraw::StreamDemuxer::SessionProtocol::RTSP;

		/* RTSP setup not done yet (2488-2489/2577-2578). */
		sd->mRtspState = Pdraw::StreamDemuxer::RtspState::CONNECTED;
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EAGAIN);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EAGAIN);
		sd->mRtspState = Pdraw::StreamDemuxer::RtspState::SETUP_DONE;

		/* A command is pending that isn't SEEK (or, for next(), an
		 * excused PAUSE_NEXT) -> falls to the `default:` -EBUSY
		 * branch (2497-2501/2590-2594), forced directly with
		 * setPendingCommand() since reaching Command::PLAY through a
		 * real play() call would also flip mFrameByFrame to false in
		 * the same call, masking this branch behind the "not paused"
		 * guard tested below instead. */
		sd->setPendingCommand(Pdraw::Demuxer::Command::PLAY);
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EBUSY);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EBUSY);
		sd->clearPendingCommand();

		/* "Demuxer is not paused" i.e. !mFrameByFrame
		 * (2480-2482/2569-2572): play() flips mFrameByFrame to false
		 * synchronously inside internalPlay(), before any RTSP
		 * response is involved, so no pump is needed to observe it
		 * here. */
		int ret = demuxer->play();
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EPROTO);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EPROTO);

		bool gotPlay = loop.pumpUntil(
			[&listener]() { return listener.mGotPlayResponse; },
			5000);
		CU_ASSERT_TRUE_FATAL(gotPlay);

		/* Restore paused/frame-by-frame state. */
		ret = demuxer->pause();
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		bool gotPause = loop.pumpUntil(
			[&listener]() { return listener.mGotPauseResponse; },
			5000);
		CU_ASSERT_TRUE_FATAL(gotPause);

		CU_ASSERT_EQUAL(demuxer->previousFrame(), 0);
		/* A pending SEEK command is now set; second calls to either
		 * are rejected. */
		CU_ASSERT_EQUAL(demuxer->previousFrame(), -EALREADY);
		CU_ASSERT_EQUAL(demuxer->nextFrame(), -EALREADY);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
}


/* seek()/seekTo() precondition guards and RTSP dispatch (see
 * testCxxStreamDemuxerPreviousNextGuards for why this stops short of
 * waiting on the async seek-completion callback):
 *   - getDuration() == 0 -> -ENOSYS for both seek() and seekTo()
 *   - seekTo()'s own "not started" (pdraw_demuxer_stream.cpp:2663-2665), "not
 *     ready to play" (2671-2673), "not RTSP protocol" (2676-2677) and "RTSP
 *     setup not done" (2679-2680) guards: forced directly since every
 *     seekTo() call here otherwise runs over a fully-negotiated RTSP
 *     session, where mState is always STARTED, mReadyToPlay always true,
 *     mSessionProtocol always RTSP and mRtspState always SETUP_DONE.
 *     Element::getState()/setState() are public, so no macro trick is
 *     needed for mState; the other three are private/protected but
 *     reachable via the `#define private public`/`protected public`
 *     convention used elsewhere in this file.
 *   - replay stream: seekTo() dispatches an RTSP PLAY (ret == 0); a second
 *     seekTo() while the first is still pending -> -EBUSY (seekTo() rejects
 *     with -EBUSY for *any* pending command, unlike previousFrame()/
 *     nextFrame() which use -EALREADY, see seekTo()'s
 *     `getPendingCommand() != Command::NONE` check) */
static void testCxxStreamDemuxerSeekToGuards()
{
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		waitReadyToPlay(loop, listener);

		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -ENOSYS);
		CU_ASSERT_EQUAL(demuxer->seek(1000, false), -ENOSYS);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
	{
		TestPompLoop loop;
		TestSession testSession(&loop);
		IPdraw *session = testSession.get();

		TestRtspServer server(loop.raw(), kTestRtspPort, 1);
		CU_ASSERT_TRUE_FATAL(server.isStarted());
		server.mHasDuration = true;

		std::string url = std::string("rtsp://127.0.0.1:") +
				  std::to_string(kTestRtspPort) + "/" +
				  kTestRtspPath;

		StreamDemuxerListener listener;
		IPdraw::IDemuxer *demuxer =
			openStreamDemuxer(session, loop, url, &listener);
		CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
		waitReadyToPlay(loop, listener);

		Pdraw::DemuxerWrapper *wrapper =
			static_cast<Pdraw::DemuxerWrapper *>(demuxer);
		Pdraw::StreamDemuxer *sd = static_cast<Pdraw::StreamDemuxer *>(
			wrapper->getDemuxer());
		CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

		/* Not started. */
		sd->setState(Pdraw::Element::State::STOPPING);
		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -EPROTO);
		sd->setState(Pdraw::Element::State::STARTED);

		/* Not ready to play. */
		sd->mReadyToPlay = false;
		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -EPROTO);
		sd->mReadyToPlay = true;

		/* Not RTSP protocol. */
		sd->mSessionProtocol =
			Pdraw::StreamDemuxer::SessionProtocol::NONE;
		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -ENOSYS);
		sd->mSessionProtocol =
			Pdraw::StreamDemuxer::SessionProtocol::RTSP;

		/* RTSP setup not done yet. */
		sd->mRtspState = Pdraw::StreamDemuxer::RtspState::CONNECTED;
		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -EAGAIN);
		sd->mRtspState = Pdraw::StreamDemuxer::RtspState::SETUP_DONE;

		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), 0);
		CU_ASSERT_EQUAL(demuxer->seekTo(0, false), -EBUSY);

		closeStreamDemuxer(demuxer, loop, &listener);
	}
}


/* seek()'s own "not started" (pdraw_demuxer_stream.cpp:2638-2640) and "not
 * ready to play" (2646-2649) guards, previously 0% covered (its
 * getDuration() == 0 -> -ENOSYS branch is already covered by
 * testCxxStreamDemuxerSeekToGuards). seek() delegates its RTSP
 * protocol/state/pending-command dispatch entirely to seekTo(), so those
 * branches don't need separate coverage here. */
static void testCxxStreamDemuxerSeekGuards()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	waitReadyToPlay(loop, listener);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Not started. */
	sd->setState(Pdraw::Element::State::STOPPING);
	CU_ASSERT_EQUAL(demuxer->seek(1000, false), -EPROTO);
	sd->setState(Pdraw::Element::State::STARTED);

	/* Not ready to play. */
	sd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(demuxer->seek(1000, false), -EPROTO);
	sd->mReadyToPlay = true;

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* play()'s (and thus pause()'s -- DemuxerWrapper::pause() is literally
 * `return play(0.);`, the exact same code path) own "not started"
 * (pdraw_demuxer_stream.cpp:2382-2384) and "not ready to play" (2386-2388)
 * guards, previously 0% covered: every existing play()/pause() test only
 * calls it after waitReadyToPlay(). */
static void testCxxStreamDemuxerPlayGuards()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	waitReadyToPlay(loop, listener);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Not started. */
	sd->setState(Pdraw::Element::State::STOPPING);
	CU_ASSERT_EQUAL(demuxer->play(), -EPROTO);
	CU_ASSERT_EQUAL(demuxer->pause(), -EPROTO);
	sd->setState(Pdraw::Element::State::STARTED);

	/* Not ready to play. */
	sd->mReadyToPlay = false;
	CU_ASSERT_EQUAL(demuxer->play(), -EPROTO);
	CU_ASSERT_EQUAL(demuxer->pause(), -EPROTO);
	sd->mReadyToPlay = true;

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* StreamDemuxer::play()'s own pending-command switch
 * (pdraw_demuxer_stream.cpp:2390-2407), previously untested: every existing
 * play()/pause() test pumps the loop to completion between calls, so a
 * second command always finds getPendingCommand() == NONE by the time it
 * runs. Issuing two commands back-to-back with no pump in between exercises
 * all four previously-uncovered outcomes:
 *   - PLAY pending, PLAY requested again (speed != 0)  -> -EALREADY (2401)
 *   - PLAY pending, PAUSE requested (speed == 0)        -> -EBUSY (2404-2407)
 *   - PAUSE pending, PAUSE requested again (speed == 0) -> -EALREADY (2401)
 *   - PAUSE pending, PLAY requested (speed != 0)        -> -EBUSY (2404-2407)
 */
static void testCxxStreamDemuxerPlayPendingCommandGuard()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	waitReadyToPlay(loop, listener);

	/* PLAY pending -> PLAY again (-EALREADY) -> PAUSE (-EBUSY). */
	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(demuxer->play(), -EALREADY);
	CU_ASSERT_EQUAL(demuxer->pause(), -EBUSY);

	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	/* PAUSE pending -> PAUSE again (-EALREADY) -> PLAY (-EBUSY). */
	ret = demuxer->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(demuxer->pause(), -EALREADY);
	CU_ASSERT_EQUAL(demuxer->play(), -EBUSY);

	bool gotPause = loop.pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotPause);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* StreamDemuxer::play()'s `goto error;` / `error:` cleanup path
 * (pdraw_demuxer_stream.cpp:2426, 2431, 2436-2438), previously 0% covered:
 * only reached when internalPlay()/internalPause() itself returns < 0,
 * which happens when rtsp_client_play()/rtsp_client_pause() fails
 * SYNCHRONOUSLY -- unlike testCxxStreamDemuxerPlayError/
 * RtspPauseAndTeardownError, which exercise an ASYNC error status delivered
 * through the response callback (a completely different code path that
 * never touches this goto). librtsp's rtsp_client_play()/pause()
 * (rtsp_client.c) both reject an empty session_id with -EINVAL before
 * touching the network at all
 * (`ULOG_ERRNO_RETURN_ERR_IF(session_id[0] == '\0', EINVAL)`), so clearing
 * mRtspSessionId right before calling play()/pause() is a simple,
 * deterministic way to force that synchronous failure without needing to
 * fake a connection drop. */
static void testCxxStreamDemuxerPlaySyncDispatchError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	waitReadyToPlay(loop, listener);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	std::string savedSessionId = sd->mRtspSessionId;
	CU_ASSERT_FALSE_FATAL(savedSessionId.empty());
	sd->mRtspSessionId.clear();

	/* play(): internalPlay() -> rtsp_client_play() -> -EINVAL, goto
	 * error, clearPendingCommand(). */
	int ret = demuxer->play();
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_TRUE(sd->getPendingCommand() ==
		       Pdraw::Demuxer::Command::NONE);

	/* pause(): same dispatch failure, this time through internalPause().
	 * mWasRunningOnce is already true (set unconditionally at the top of
	 * internalPlay(), even though the RTSP call above failed), so this
	 * takes the setPendingCommand(PAUSE) branch, not PAUSE_NEXT. */
	ret = demuxer->pause();
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_TRUE(sd->getPendingCommand() ==
		       Pdraw::Demuxer::Command::NONE);

	/* Restore a valid session id so the normal close sequence (which
	 * issues a real RTSP TEARDOWN) works. */
	sd->mRtspSessionId = savedSessionId;

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* RTCP BYE with a reason other than "reconfigure"/"photo_trigger":
 * exercises StreamDemuxer::VideoMedia::goodbyeCb()'s "else" branch (unknown
 * / user-disconnection reason) which calls demuxer->onUnrecoverableError()
 * when exactly one non-tearing-down media is selected -- directly
 * observable via onDemuxerUnrecoverableError() since openResponse has
 * already fired by this point. */
static void testCxxStreamDemuxerGoodbyeUnrecoverableError()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE(gotPlay);

	/* Establish the RTP source (self->source.ssrc) before the BYE:
	 * vstrm_receiver_rtcp_bye_cb() requires bye.sources[0] to match an
	 * already-tracked SSRC. */
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	sendRtcpBye(ctrlPort, kTestRtspSsrc, "test goodbye");

	bool gotError = loop.pumpUntil(
		[&demuxListener]() {
			return demuxListener.mGotUnrecoverableError;
		},
		5000);
	CU_ASSERT_TRUE(gotError);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* ── Group F: tests with a real attached ExternalCodedVideoSink ───────── */

/* Shared setup for the Group F tests below: open, wait ready, play, send
 * SPS+PPS, wait for onMediaAdded, then attach a coded video sink. mediaId=0
 * deterministically binds to the first CodedVideoMedia offered -- the AVCC
 * one -- since StreamDemuxer::VideoMedia::setupMedia() always creates it
 * before the byte-stream one (same guarantee already documented for
 * RecordDemuxer in test_api_demuxer.cpp). Returns the attached sink (caller
 * owns it); *outDemuxer receives the opened demuxer (caller owns it via
 * closeStreamDemuxer()). */
static IPdraw::ICodedVideoSink *
openPlayAndAttachSink(IPdraw *session,
		      TestPompLoop &loop,
		      const std::string &url,
		      StreamDemuxerListener *demuxListener,
		      MediaTrackingListener *mediaListener,
		      DrainingCodedVideoSinkListener *sinkListener,
		      IPdraw::IDemuxer **outDemuxer)
{
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, demuxListener);
	*outDemuxer = demuxer;
	CU_ASSERT_EQUAL_FATAL(demuxListener->mOpenStatus, 0);

	bool gotReady = loop.pumpUntil(
		[demuxListener]() { return demuxListener->mGotReadyToPlay; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[demuxListener]() { return demuxListener->mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[mediaListener]() { return !mediaListener->mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		0, &sinkParams, sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	sinkListener->mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener->mQueue);

	/* Pump until the sink's onCodedVideoSinkMediaAdded fires, which means
	 * the channel is linked and output frames will be queued.  Without this
	 * the link happens asynchronously on the loop: if an RTP frame arrives
	 * before the link completes, processFrame() finds outputChannelCount==0
	 * and silently discards the frame. */
	bool gotSinkMedia = loop.pumpUntil(
		[sinkListener]() { return sinkListener->mGotMediaAdded; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotSinkMedia);

	return sink;
}


/* Deliver one IDR frame to an already-attached sink and return with the
 * sink in FlushingState::UNFLUSHED.  Needed before any flush()/drain()
 * scenario: a sink that has never received a frame stays in its initial
 * FlushingState::FLUSHED, so flush()/drain() auto-completes trivially
 * without invoking the application listener.
 *
 * Design constraints that govern the implementation:
 *
 * 1. Jitter buffer latency: vstrm_receiver holds each RTP frame until a
 *    frame with a HIGHER RTP timestamp arrives (or the ~30ms timer fires).
 *    A single IDR packet therefore never reaches processFrame() on its own.
 *
 * 2. Same-timestamp batching: if every IDR carries the same RTP timestamp
 *    (kIdrTs), the jitter buffer treats all of them as NAL units of the
 *    same frame and flushes the entire batch at once.  That delivers two or
 *    more frames inside a single pomp_loop_wait_and_process() call, which
 *    exhausts the (small) output memory pool: the second frame hits
 *    -EAGAIN → processFrame() calls flush() internally → FlushingState
 *    reset to FLUSHED → drain() later auto-completes with no callback.
 *
 * Solution: use INCREMENTING RTP timestamps (one frame interval = 3000
 * ticks = 33 ms at 90 kHz).  With distinct timestamps the jitter buffer
 * releases each IDR only when the *next* IDR (higher ts) arrives, so at
 * most ONE frame fires per pump iteration.  We destructively pop and unref
 * every delivered frame inside the loop: popping via mbuf_coded_video_
 * frame_queue_pop() does NOT change FlushingState (only the flush/drain
 * state machine does), so the sink stays UNFLUSHED after the first frame
 * arrives regardless of how many subsequent IDRs are sent.
 *
 * Trailing-IDR drain: the last IDR sent is still held in the jitter buffer
 * when the loop exits.  We wait 50 ms (> ~30 ms jitter timer) and pop it,
 * preventing a pool-exhaustion flush during the caller's pumpUntil(). */
static void deliverOneFrameToSink(TestPompLoop &loop,
				  IPdraw::IDemuxer *demuxer,
				  DrainingCodedVideoSinkListener &sinkListener)
{
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kIdrBaseTs = 180000u;
	static constexpr uint32_t kRtpTicksPerFrame =
		3000u; /* 33 ms @ 90 kHz */
	/* Start immediately after SPS (seq=1) and PPS (seq=2) so the jitter
	 * buffer sees no gap.  A gap (e.g. seq=10) causes vstrm_receiver to
	 * hold ALL subsequent packets until the ~30ms gap-recovery timer fires,
	 * then release them all at once — exhausting the output memory pool and
	 * triggering an unintended flush that resets FlushingState to FLUSHED.
	 */
	uint16_t idrSeq = 3;
	uint32_t idrTs = kIdrBaseTs;

	auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds(5000);
	bool gotFrame = false;
	do {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  idrSeq++,
				  idrTs,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		idrTs += kRtpTicksPerFrame;
		pomp_loop_wait_and_process(loop.raw(), 20);
		/* Pop every queued frame immediately to keep the memory pool
		 * below its exhaustion threshold.  Popping does not change
		 * FlushingState, so the sink remains UNFLUSHED once a frame
		 * has been delivered. */
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0) {
			gotFrame = true;
			mbuf_coded_video_frame_unref(f);
		}
	} while (!gotFrame && std::chrono::steady_clock::now() < deadline);

	CU_ASSERT_TRUE_FATAL(gotFrame);

	/* Drain the trailing IDR still held by the jitter buffer. */
	pomp_loop_wait_and_process(loop.raw(), 50);
	{
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			mbuf_coded_video_frame_unref(f);
	}
}


/* onChannelFlushed()/onChannelUnlink() via close(): with a real
 * ExternalCodedVideoSink attached and acking flush via queueFlushed(),
 * close() drives stop() -> flush() -> channel->flush() -> sink acks ->
 * channel->flushDone() -> StreamDemuxer::onChannelFlushed() (mState ==
 * STOPPING, so it also calls channel->teardown() synchronously) ->
 * channel->unlink() -> StreamDemuxer::onChannelUnlink() ->
 * asyncCompleteTeardown() -> completeTeardown() -> tryCompleteStop() ->
 * closeResponse(0). This whole chain is *gated* on onChannelFlushed and
 * onChannelUnlink actually running correctly: completeTeardown() bails out
 * early (leaving mChannelsReadyForStop false forever) unless every output
 * channel has been unlinked -- so closeStreamDemuxer()'s
 * mGotCloseResponse/status==0 assertions are a solid proof that both
 * callbacks executed, not just that the sink's ack was dispatched. Without
 * a sink attached, getOutputChannelCount() == 0 always and neither callback
 * ever runs -- which is why none of the pre-existing tests in this file
 * exercise them. */
static void testCxxStreamDemuxerChannelFlushedAndUnlinkOnClose()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* The sink must have actually received a frame before close(), or
	 * its flush() trivially auto-completes without invoking the
	 * application listener (see deliverOneFrameToSink()). */
	deliverOneFrameToSink(loop, demuxer, sinkListener);

	closeStreamDemuxer(demuxer, loop, &demuxListener);

	CU_ASSERT_TRUE(sinkListener.mGotFlush);

	sinkOwner.reset();
}


/* onChannelDrained() via pause(): StreamDemuxer::VideoMedia::
 * onPauseComplete() (called synchronously from onRtspPauseResp) calls
 * drain(), which calls channel->drain() on every attached output channel;
 * the sink's ack (via queueDrained()) reaches channel->drainDone() ->
 * Source::onChannelUpstreamEvent -> StreamDemuxer::onChannelDrained().
 * Unlike the close()/flush() chain above, demuxerPauseResponse() fires
 * unconditionally right after onPauseComplete() and is *not* gated on the
 * drain ack (see onRtspPauseResp, pdraw_demuxer_stream.cpp ~line 1164), so
 * mGotPauseResponse alone would not prove onChannelDrained ran; and
 * onChannelDrained()'s own bookkeeping (a private per-instance counter) has
 * no public getter either. The only externally-observable signal here is
 * the sink's own onCodedVideoSinkDrain() callback (the downstream half of
 * the round trip) -- this test exercises the onChannelDrained() code path
 * via the sink's ack and proves the downstream dispatch, without an
 * independent black-box proof that onChannelDrained's own bookkeeping
 * completed correctly (see TEST_PROGRESS.md). */
static void testCxxStreamDemuxerChannelDrainedOnPause()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	/* The sink must have actually received a frame before pause(), or
	 * its drain() trivially auto-completes without invoking the
	 * application listener (see deliverOneFrameToSink()). */
	deliverOneFrameToSink(loop, demuxer, sinkListener);

	int ret = demuxer->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPause = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPauseResponse; },
		5000);
	CU_ASSERT_TRUE(gotPause);

	bool gotDrain = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotDrain; }, 5000);
	CU_ASSERT_TRUE(gotDrain);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* onChannelResync() via ICodedVideoSink::resync(): a public API method
 * (pdraw.hpp ICodedVideoSink::resync()) that calls Channel::resync()
 * upstream to Source::onChannelResync() -> StreamDemuxer::onChannelResync()
 * -> VideoMedia::resync() (sets mWaitForSync=true, an internal flag with no
 * public getter). resync() itself returning 0 requires a live input
 * media/channel on the sink side (ExternalCodedVideoSink::resync() looks up
 * both), which is already a meaningful structural check; an IDR frame sent
 * afterwards must still reach the sink, showing mWaitForSync did not
 * permanently block the media. */
static void testCxxStreamDemuxerChannelResync()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	int ret = sink->resync();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Same jitter-buffer drain loop as deliverOneFrameToSink: a single IDR
	 * packet is held by the vstrm_receiver until subsequent packets arrive;
	 * send in a loop until at least one frame pops out of the sink queue.
	 */
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kIdrTs = 180000u;
	uint16_t idrSeq = 10;

	std::vector<struct mbuf_coded_video_frame *> frames;
	auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds(5000);
	do {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  idrSeq++,
				  kIdrTs,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		pomp_loop_wait_and_process(loop.raw(), 20);
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			frames.push_back(f);
	} while (frames.empty() && std::chrono::steady_clock::now() < deadline);

	bool gotFrame = !frames.empty();
	CU_ASSERT_TRUE(gotFrame);
	for (auto *f : frames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* eventCb() with VSTRM_EVENT_RECONFIGURE, sent as a hand-crafted RTCP APP
 * packet on the control port (see sendRtcpEvent()). Directly, this only
 * sets an internal flag (self->mRtcpMediaChangeReceived) and forwards a
 * Channel::DownstreamEvent::RECONFIGURE, which ExternalCodedVideoSink::
 * onChannelReconfigure() turns into mPendingRestart=true
 * (pdraw_external_coded_video_sink.cpp ~line 997) -- itself not publicly
 * observable until the media is actually removed. close() tears the media
 * down (channel->teardown() -> Sink::onChannelTeardown() ->
 * ExternalCodedVideoSink::removeInputMedia(), which passes mPendingRestart
 * straight to the listener), so onCodedVideoSinkMediaRemoved() fires with
 * restart==true -- publicly observable and this test's assertion. */
static void testCxxStreamDemuxerEventReconfigureSetsRestartFlag()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	/* VSTRM_EVENT_RECONFIGURE, see
	 * include/video-streaming/vstrm_events.h. */
	static constexpr uint8_t kVstrmEventReconfigure = 1;
	sendRtcpEvent(ctrlPort, kTestRtspSsrc, kVstrmEventReconfigure);

	/* No dedicated public completion signal for this event; it is
	 * delivered synchronously off a single UDP datagram, so a short
	 * bounded pump is enough to let eventCb()/sendDownstreamEvent() run
	 * before tearing down. */
	pumpFor(loop, 300);

	closeStreamDemuxer(demuxer, loop, &demuxListener);

	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
}


/* processFrame(): full H.264 delivery path with a real attached sink, as
 * opposed to testCxxStreamDemuxerRtpCodecInfo which only exercises
 * codec_info_changed/setupMedia (the SPS/PPS access unit itself is
 * discarded by processFrame() as an "empty frame" since it has 0 slices,
 * see pdraw_demuxer_stream.cpp ~line 3749). Also checks getCurrentTime()
 * advances once the frame is processed (mCurrentTime is updated
 * unconditionally near the top of processFrame(), before the sink-queuing
 * loop) -- checked only *after* confirming a real frame was actually
 * delivered, to avoid depending on undocumented clock/skew internals in
 * isolation. */
static void testCxxStreamDemuxerProcessFrameDeliversToSink()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	CU_ASSERT_EQUAL(demuxer->getCurrentTime(), 0u);

	/* Same jitter-buffer drain loop as deliverOneFrameToSink. */
	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kIdrTs = 180000u;
	uint16_t idrSeq = 10;

	std::vector<struct mbuf_coded_video_frame *> frames;
	auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds(5000);
	do {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  idrSeq++,
				  kIdrTs,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		pomp_loop_wait_and_process(loop.raw(), 20);
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			frames.push_back(f);
	} while (frames.empty() && std::chrono::steady_clock::now() < deadline);

	bool gotFrame = !frames.empty();
	CU_ASSERT_TRUE_FATAL(gotFrame);

	/* The first delivered frame must be an IDR. */
	struct vdef_coded_frame frameInfo = {};
	int ret = mbuf_coded_video_frame_get_frame_info(frames[0], &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	if (ret == 0)
		CU_ASSERT_EQUAL(frameInfo.type, VDEF_CODED_FRAME_TYPE_IDR);

	for (auto *f : frames)
		mbuf_coded_video_frame_unref(f);

	CU_ASSERT_NOT_EQUAL(demuxer->getCurrentTime(), 0u);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* VideoDecoder::onChannelSessionMetaUpdate was entirely uncovered (0% per a
 * real gcov run). VideoDecoder is never exposed through the public API
 * directly (always auto-created by PipelineFactory when autodecoding_mode =
 * DECODE_ALL, same rationale as test_pipeline_decoder_video.cpp), and unlike
 * VideoEncoder/VideoScaler (whose upstream ExternalRawVideoSource has a
 * public setSessionMetadata() to trigger this directly), a demuxer's session
 * metadata can only change via a real network signal: vstrm_receiver parses
 * it from an RTCP SDES packet (see sendRtcpSdes() above), which
 * StreamDemuxer::VideoMedia::sessionMetadataPeerChangedCb()
 * (pdraw_demuxer_stream.cpp:4405) forwards as a SESSION_META_UPDATE
 * downstream event on the demuxer's own coded video media. That event
 * cascades through the internally auto-created VideoDecoder -- exercising
 * its onChannelSessionMetaUpdate (which forwards to its own output channels,
 * see FilterElement::onChannelSessionMetaUpdate in pdraw_element.cpp) -- and
 * reaches the downstream raw video sink (onRawVideoSinkSessionMetaUpdate),
 * proving the whole chain, including the decoder's own handler, actually
 * ran. */
/* A minimal, correctly-strided/sized I420 frame -- same pattern as
 * makeI420Frame() in
 * test_pipeline_scaler_video.cpp/test_pipeline_encoder_video.cpp (duplicated
 * here, not shared, per this suite's TU-private convention). */
static struct mbuf_raw_video_frame *makeI420Frame(uint32_t width,
						  uint32_t height,
						  uint64_t timestamp,
						  unsigned int index)
{
	struct vdef_dim resolution = {width, height};
	size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
	int ret = vdef_calc_raw_frame_size(&vdef_i420,
					   &resolution,
					   planeStride,
					   nullptr,
					   nullptr,
					   nullptr,
					   planeSize,
					   nullptr);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&vdef_i420);
	CU_ASSERT_EQUAL_FATAL(planeCount, 3u);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = vdef_i420;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;
	frameInfo.info.resolution.width = width;
	frameInfo.info.resolution.height = height;
	frameInfo.info.bit_depth = 8;
	for (unsigned int p = 0; p < planeCount; p++)
		frameInfo.plane_stride[p] = planeStride[p];

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	for (unsigned int p = 0; p < planeCount; p++) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(planeSize[p], &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0x80, capacity);
		ret = mbuf_raw_video_frame_set_plane(
			frame, p, mem, 0, planeSize[p]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);
	}

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	return frame;
}


/* kH264Sps/kH264Pps/kH264Idr above are deliberately fake (see kH264Idr's
 * comment: "slice payload bytes are not parsed ... only NAL type is
 * checked") -- fine for Group F's coded-sink-only tests, but NOT decodable
 * by a real decoder. testCxxStreamDemuxerSessionMetaUpdateCascadesToDecoder
 * (DECODE_ALL) needs VideoDecoder to actually succeed at decoding a frame
 * (its raw output media is only created on genuine decode success, see
 * VideoDecoder::createOutputMedia), so genuinely valid bitstream bytes are
 * required. Confirmed the hard way: a first version of this test reused
 * kH264Idr and failed with a real ffmpeg decode error ("top block
 * unavailable for requested intra mode", "error while decoding MB 0 0").
 *
 * Rather than hand-author a valid H.264 bitstream (impractical to get right
 * by hand), this encodes one real frame with the same x264 backend already
 * proven to work in test_pipeline_encoder_video.cpp, and extracts its
 * SPS/PPS/IDR-slice NALUs via mbuf_coded_video_frame_get_nalu() (raw NALU
 * bytes, no start code/length prefix -- exactly what RTP H.264 payloading
 * needs, see sendRtpNaluPacket() above). Runs in its own throwaway
 * TestPompLoop/TestSession, entirely separate from the caller's, so this
 * encoder's own raw/coded media never pollutes the caller's
 * MediaTrackingListener (which would otherwise make findRawVideoMedia()
 * match this encoder's temporary source instead of the real VideoDecoder's
 * output). */
static void encodeSampleH264Nalus(std::vector<uint8_t> *sps,
				  std::vector<uint8_t> *pps,
				  std::vector<uint8_t> *idr)
{
	/* Kept well above hardware decoder minimums (e.g. NVDEC/vdec_ffmpeg's
	 * cuda backend rejects anything below 48px: "Video width 32 not
	 * within range from 48 to 8192") -- the encode side (x264, software)
	 * has no such floor, but the resulting SPS/IDR feed a real decoder
	 * further down this test's pipeline, which may select a HW backend
	 * depending on the machine. */
	constexpr uint32_t kWidth = 64;
	constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawVideoMedia()->id;

	struct venc_config encoderParams = {};
	encoderParams.implem = VENC_ENCODER_IMPLEM_X264;
	encoderParams.encoding = VDEF_ENCODING_H264;
	encoderParams.h264.max_bitrate = 2000000;
	encoderParams.output.preferred_format =
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	/* venc_h264_generate_nalus() (venc_h264.c:1059) only inserts SPS/PPS
	 * into IDR frames when this is set -- otherwise x264's own SPS/PPS
	 * NALUs are silently discarded (add_x264_nalus(), venc_x264.c:367-373,
	 * "we have already inserted these NAL units generated ourselves" --
	 * true only when insert_ps actually ran) and no encoded frame ever
	 * carries them, so LocalEncoderListener below would wait forever. */
	encoderParams.h264.insert_ps = 1;

	class LocalEncoderListener : public IPdraw::IVideoEncoder::Listener {
	public:
		void videoEncoderFrameOutput(
			IPdraw * /*p*/,
			IPdraw::IVideoEncoder * /*e*/,
			struct mbuf_coded_video_frame *frame) override
		{
			if (mGotFrame)
				return;
			int naluCount =
				mbuf_coded_video_frame_get_nalu_count(frame);
			for (int i = 0; i < naluCount; i++) {
				const void *data = nullptr;
				struct vdef_nalu nalu = {};
				int ret = mbuf_coded_video_frame_get_nalu(
					frame, i, &data, &nalu);
				if (ret < 0)
					continue;
				/* venc_x264's add_x264_nalus() always
				 * prefixes each NALU with a 4-byte marker
				 * (the byte-stream start code, since
				 * output.preferred_format ==
				 * VDEF_CODED_DATA_FORMAT_BYTE_STREAM below)
				 * included in nalu.size/data
				 * (venc_x264.c:377-413) -- strip it: RTP
				 * H.264 payloading (sendRtpNaluPacket() above)
				 * needs the bare NALU, no start code/length
				 * prefix. */
				const auto *bytes = (const uint8_t *)data;
				CU_ASSERT_FATAL(nalu.size > 4);
				std::vector<uint8_t> buf(bytes + 4,
							 bytes + nalu.size);
				switch (nalu.h264.type) {
				case H264_NALU_TYPE_SPS:
					*mSps = buf;
					break;
				case H264_NALU_TYPE_PPS:
					*mPps = buf;
					break;
				case H264_NALU_TYPE_SLICE_IDR:
					*mIdr = buf;
					break;
				default:
					break;
				}
				mbuf_coded_video_frame_release_nalu(
					frame, i, data);
			}
			if (!mSps->empty() && !mPps->empty() && !mIdr->empty())
				mGotFrame = true;
		}

		void videoEncoderFramePreRelease(
			IPdraw * /*p*/,
			IPdraw::IVideoEncoder * /*e*/,
			struct mbuf_coded_video_frame * /*f*/) override
		{
		}

		std::vector<uint8_t> *mSps = nullptr;
		std::vector<uint8_t> *mPps = nullptr;
		std::vector<uint8_t> *mIdr = nullptr;
		bool mGotFrame = false;
	} encoderListener;
	encoderListener.mSps = sps;
	encoderListener.mPps = pps;
	encoderListener.mIdr = idr;

	IPdraw::IVideoEncoder *encoder = nullptr;
	ret = session->createVideoEncoder(
		rawMediaId, &encoderParams, &encoderListener, &encoder);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(encoder);
	auto encoderOwner = std::unique_ptr<IPdraw::IVideoEncoder>(encoder);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	for (unsigned int i = 0; i < 4; i++) {
		struct mbuf_raw_video_frame *primer =
			makeI420Frame(kWidth, kHeight, i * 33333, i);
		ret = mbuf_raw_video_frame_queue_push(inQueue, primer);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_raw_video_frame_unref(primer);
	}

	bool gotFrame = loop.pumpUntil(
		[&encoderListener]() { return encoderListener.mGotFrame; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);

	encoderOwner.reset();
	sourceOwner.reset();

	/* Unlike every other test in this file (which only ever destroys a
	 * StreamDemuxerNet cleanly closed via closeStreamDemuxer(), already
	 * fully STOPPED by the time the test function returns), this helper
	 * builds a live ExternalRawVideoSource -> VideoEncoder chain: the
	 * .reset() calls above only *start* each element's async stop()
	 * (flush first, see the "element flushing state change to FLUSHING"
	 * log lines). Without waiting for that to actually finish,
	 * TestSession's destructor (~Session() -> mElements.clear()) tears
	 * down elements still mid-teardown -- confirmed the hard way by a
	 * real ASan heap-use-after-free: Source::~Source() (destroyed after
	 * ExternalRawVideoSource's own mOutputMedia member, per normal C++
	 * member-destruction order) calls removeOutputPorts(), which logs
	 * the (already-destroyed) media's name. session->stop() + waiting
	 * for stopResponse() blocks until every element genuinely reaches
	 * STOPPED first. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL_FATAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE_FATAL(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxStreamDemuxerSessionMetaUpdateCascadesToDecoder()
{
	/* Real, genuinely decodable NALUs -- see encodeSampleH264Nalus()'s
	 * comment for why the shared kH264Sps/kH264Pps/kH264Idr fakes above
	 * cannot be used here. */
	std::vector<uint8_t> realSps, realPps, realIdr;
	encodeSampleH264Nalus(&realSps, &realPps, &realIdr);

	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session,
				  loop,
				  url,
				  &demuxListener,
				  10000,
				  PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  realSps.data(),
			  realSps.size());
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  realPps.data(),
			  realPps.size());

	/* A real decoded frame is required for VideoDecoder to create its own
	 * raw output media (VideoDecoder::createOutputMedia).
	 * The vstrm_receiver jitter buffer (~30ms) needs subsequent packets to
	 * flush; send IDRs in a loop until the decoder produces raw output. */
	static constexpr uint32_t kIdrTs = 180000u;
	uint16_t idrSeq = 10;
	bool gotRawVideo = false;
	{
		auto rawDeadline = std::chrono::steady_clock::now() +
				   std::chrono::milliseconds(15000);
		do {
			sendRtpNaluPacket(rtpPort,
					  kTestRtspSsrc,
					  idrSeq++,
					  kIdrTs,
					  realIdr.data(),
					  realIdr.size(),
					  true /* marker */);
			pomp_loop_wait_and_process(loop.raw(), 20);
			gotRawVideo =
				(mediaListener.findRawVideoMedia() != nullptr);
		} while (!gotRawVideo &&
			 std::chrono::steady_clock::now() < rawDeadline);
	}
	CU_ASSERT_TRUE_FATAL(gotRawVideo);
	const MediaTrackingListener::Added *rawVideoPtr =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideoPtr);
	unsigned int rawMediaId = rawVideoPtr->id;

	struct pdraw_video_sink_params sinkParams = {};
	TrackingRawVideoSinkListener sinkListener;
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(
		rawMediaId, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	sendRtcpSdes(ctrlPort,
		     kTestRtspSsrc,
		     "pdraw_test_stream_session_meta_update",
		     "TestDrone");

	bool gotUpdate = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotSessionMetaUpdate;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotUpdate);
	CU_ASSERT_STRING_EQUAL(sinkListener.mMeta.friendly_name,
			       "pdraw_test_stream_session_meta_update");
	CU_ASSERT_STRING_EQUAL(sinkListener.mMeta.maker, "Parrot");
	CU_ASSERT_STRING_EQUAL(sinkListener.mMeta.model, "TestDrone");

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);

	/* closeStreamDemuxer() only waits for the demuxer's OWN
	 * closeResponse -- sufficient in every other test in this file
	 * (Group F: a coded sink attached directly to the demuxer's output IS
	 * the terminal consumer gating the demuxer's own completeTeardown(),
	 * see testCxxStreamDemuxerChannelFlushedAndUnlinkOnClose's comment).
	 * This test is the first to use DECODE_ALL: the demuxer's closeResponse
	 * fired here while the auto-created VideoDecoder -- one more hop
	 * downstream -- was still mid-teardown ("decoder is still running" /
	 * "output media was not properly removed" in a real run), so
	 * ~TestSession() destroyed it before it reached STOPPED, hitting the
	 * exact same use-after-free pattern as encodeSampleH264Nalus() above
	 * (Source::~Source()'s removeOutputPorts() logging an
	 * already-destroyed media, since VideoDecoder's own mOutputMedia
	 * member is destroyed before FilterElement/Source's base destructor
	 * runs). session->stop() + waiting for stopResponse() (same as
	 * closeAndDestroyDemuxer()+stopSessionAndWait() in
	 * test_pipeline_decoder_video.cpp) ensures every element, not just the
	 * demuxer itself, reaches STOPPED first. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL_FATAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE_FATAL(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


/* ── SEI callbacks: h264UserDataSeiCb + h264RecoveryPointSeiCb ─────────── */

/* Exercises h264UserDataSeiCb (user_data_unregistered, payloadType=5) and
 * h264RecoveryPointSeiCb (payloadType=6) inside processFrame's NAL loop.
 *
 * Flow: SEI NAL packets (M=1) are sent via UDP → vstrm jitter buffer →
 * recvFrameCb → processFrame → h264_reader_parse_nalu → SEI callbacks.
 * SEI-only frames have sliceCount==0, so they hit "goto out" at line 3748
 * BEFORE the mWaitForSync check — both callbacks fire regardless of sync
 * state, and mCurrentFrame is non-null (created before the NAL loop). */
static void testCxxStreamDemuxerH264SeiCallbacks()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(demuxListener.mPlayStatus, 0);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* SEI user_data_unregistered (payloadType=5) → h264UserDataSeiCb.
	 * UUID = 16 zero bytes (not the Parrot streaming UUID), so the callback
	 * adds an ancillary buffer to mCurrentFrame instead of early-returning.
	 */
	static constexpr uint32_t kSeiTs1 = 180000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  3,
			  kSeiTs1,
			  kH264SeiUserDataUnreg,
			  sizeof(kH264SeiUserDataUnreg),
			  true /* marker */);
	/* 150ms: enough for the ~30ms vstrm jitter-buffer timer to fire and
	 * release the SEI frame through recvFrameCb → processFrame. */
	pumpFor(loop, 150);

	/* SEI recovery_point (payloadType=6) → h264RecoveryPointSeiCb.
	 * Different timestamp so the jitter buffer treats this as a new AU.
	 * mWaitForSync is true at this point (no IDR yet), so the callback
	 * executes the mWaitForSync=false branch, adding coverage. */
	static constexpr uint32_t kSeiTs2 = 270000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  4,
			  kSeiTs2,
			  kH264SeiRecoveryPoint,
			  sizeof(kH264SeiRecoveryPoint),
			  true /* marker */);
	pumpFor(loop, 150);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* processFrame(): h264PicTimingSeiCb (pic_timing SEI, payloadType=1) --
 * previously 0% covered (pdraw_demuxer_stream.cpp:4135-4155). Needs its own
 * SPS (kH264PicTimingSps) with VUI pic_struct_present_flag=1 and non-zero
 * num_units_in_tick/time_scale: the file's usual kH264Sps has
 * pic_struct_present_flag=0, so h264_syntax.h's sei_pic_timing() parser would
 * skip the clk_ts[] array entirely and the callback would only ever see an
 * all-zero struct h264_sei_pic_timing (capture timestamp always 0).
 *
 * The SEI alone forms a 0-slice "empty frame" that processFrame() discards
 * before delivery ("Ignore frames with 0 slices", pdraw_demuxer_stream.cpp:
 * 3748-3753), so it must share an access unit with a real slice (kH264Idr) --
 * same pattern as testCxxStreamDemuxerProcessFrameNaluAndSyncBranches's
 * recovery-point SEI + kH264PSlice -- to reach an attached sink where
 * mCurrentFrameCaptureTs (set by the callback, then copied to
 * frameInfo.info.capture_timestamp at pdraw_demuxer_stream.cpp:3791) can be
 * observed externally via mbuf_coded_video_frame_get_frame_info(). */
static void testCxxStreamDemuxerH264PicTimingSei()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Send the dedicated SPS/PPS in-band (NOT the file's usual kH264Sps/
	 * kH264Pps): codecInfoChangedCb -> setupMedia() parses THIS SPS into
	 * mH264Reader's ctx->sps, which is exactly what
	 * h264_ctx_sei_pic_timing_to_us() reads from. */
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264PicTimingSps,
			  sizeof(kH264PicTimingSps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	struct pdraw_video_sink_params sinkParams = {};
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);
	bool gotSinkMedia = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotSinkMedia);

	/* Continue directly from the SPS(seq=1)/PPS(seq=2) sent above --
	 * starting higher opens a sequence-number gap, which vstrm treats as
	 * real packet loss instead of one clean access unit. One access unit:
	 * pic_timing SEI + IDR slice, same RTP timestamp, marker bit only on
	 * the last packet. */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  3,
			  180000u,
			  kH264SeiPicTiming,
			  sizeof(kH264SeiPicTiming));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  4,
			  180000u,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	pumpFor(loop, 200);

	std::vector<struct mbuf_coded_video_frame *> frames;
	{
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			frames.push_back(f);
	}
	CU_ASSERT_EQUAL_FATAL(frames.size(), 1U);

	/* h264PicTimingSeiCb sets mCurrentFrameCaptureTs =
	 * h264_ctx_sei_pic_timing_to_us(ctx, sei); with hours=1, minutes=2,
	 * seconds=3, n_frames=4, num_units_in_tick=1, time_scale=1000000:
	 * clock_timestamp = ((1*60+2)*60+3)*1000000 + 4*1 = 3723000004, and
	 * since time_scale == 1000000 the tick->microsecond conversion is a
	 * no-op -- see kH264SeiPicTiming's comment above for the full
	 * derivation. */
	struct vdef_coded_frame info = {};
	int getRet =
		mbuf_coded_video_frame_get_frame_info(frames.front(), &info);
	CU_ASSERT_EQUAL(getRet, 0);
	CU_ASSERT_EQUAL(info.info.capture_timestamp, 3723000004ULL);

	for (auto *f : frames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* processFrame(): NALU-type switch "default" case, "discarding frame (wait
 * for sync)" branch, and the mRecoveryFrameCount-- / SILENT-flag branches --
 * previously 0% covered (pdraw_demuxer_stream.cpp:3730-3731, 3762-3763,
 * 3769-3770, 3795-3796). Unlike testCxxStreamDemuxerH264SeiCallbacks (which
 * only checks the SEI callbacks' side effects on a demuxer with no sink),
 * this test needs a real attached sink so the popped mbuf_coded_video_frame
 * can be inspected: whether it was delivered at all (discard branch) and its
 * VDEF_FRAME_FLAG_SILENT flag (recovery-count branch). */
static void testCxxStreamDemuxerProcessFrameNaluAndSyncBranches()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Send an access unit (one or more NALUs sharing the same RTP
	 * timestamp, marker bit set only on the last) and, after letting the
	 * ~30ms vstrm jitter-buffer timer release it, pop every frame it
	 * produced into *out. Sequence numbers continue directly from
	 * openPlayAndAttachSink()'s SPS(seq=1)/PPS(seq=2): starting anywhere
	 * higher opens a sequence-number gap, which vstrm's jitter buffer and
	 * H.264 depayloader treat as real packet loss (concealment / "missing
	 * end of frame" handling, see vstrm_rtp_h264_rx.c), corrupting every
	 * assumption this test relies on (a single clean, gap-free AU). */
	uint16_t seq = 3;
	auto sendAuAndPop =
		[&](uint32_t ts,
		    std::initializer_list<std::pair<const uint8_t *, size_t>>
			    nalus,
		    std::vector<struct mbuf_coded_video_frame *> *out) {
			size_t i = 0, n = nalus.size();
			for (const auto &nal : nalus) {
				i++;
				sendRtpNaluPacket(
					rtpPort,
					kTestRtspSsrc,
					seq++,
					ts,
					nal.first,
					nal.second,
					(i == n) /* marker on last */);
			}
			pumpFor(loop, 200);
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(
				       sinkListener.mQueue, &f) == 0)
				out->push_back(f);
		};

	/* 1) "default" NALU-type case (line 3730-3731): an AUD NAL (type 9,
	 * not handled by any explicit switch case) inside the same access
	 * unit as a real IDR slice. sliceCount is still 1 (from the IDR), so
	 * the frame is NOT discarded as "empty" -- proving the default case's
	 * plain `break` doesn't corrupt delivery of the rest of the AU. */
	std::vector<struct mbuf_coded_video_frame *> frames1;
	sendAuAndPop(
		180000u,
		{{kH264Aud, sizeof(kH264Aud)}, {kH264Idr, sizeof(kH264Idr)}},
		&frames1);
	CU_ASSERT_TRUE_FATAL(!frames1.empty());
	for (auto *f : frames1)
		mbuf_coded_video_frame_unref(f);

	/* 2) "discarding frame (wait for sync)" (line 3762-3763): resync()
	 * sets mWaitForSync=true; a non-IDR slice must then be silently
	 * discarded (no frame reaches the sink). */
	int ret = sink->resync();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	std::vector<struct mbuf_coded_video_frame *> discardedFrames;
	sendAuAndPop(183000u,
		     {{kH264PSlice, sizeof(kH264PSlice)}},
		     &discardedFrames);
	CU_ASSERT_TRUE(discardedFrames.empty());

	/* An IDR now clears mWaitForSync (line 3757-3760) and is delivered
	 * normally, confirming the demuxer recovers. */
	std::vector<struct mbuf_coded_video_frame *> resyncFrames;
	sendAuAndPop(186000u, {{kH264Idr, sizeof(kH264Idr)}}, &resyncFrames);
	CU_ASSERT_TRUE_FATAL(!resyncFrames.empty());
	for (auto *f : resyncFrames)
		mbuf_coded_video_frame_unref(f);

	/* 3) mRecoveryFrameCount-- (line 3769-3770) and the SILENT flag it
	 * drives (line 3795-3796): resync() again, then an access unit
	 * carrying a recovery_frame_cnt=1 SEI *and* a ref (nal_ref_idc!=0)
	 * non-IDR slice. h264RecoveryPointSeiCb (called from inside the NAL
	 * loop while parsing the SEI, before this frame's own
	 * mWaitForSync/mRecoveryFrameCount check runs) clears mWaitForSync
	 * and sets mRecoveryFrameCount = recovery_frame_cnt+1 = 2; this same
	 * frame's ref-frame decrement then immediately brings it to 1, still
	 * != 0, so this first frame IS marked SILENT. */
	ret = sink->resync();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	std::vector<struct mbuf_coded_video_frame *> recoveryFrames1;
	sendAuAndPop(
		189000u,
		{{kH264SeiRecoveryPointCnt1, sizeof(kH264SeiRecoveryPointCnt1)},
		 {kH264PSlice, sizeof(kH264PSlice)}},
		&recoveryFrames1);
	CU_ASSERT_TRUE_FATAL(!recoveryFrames1.empty());
	{
		struct vdef_coded_frame info = {};
		int getRet = mbuf_coded_video_frame_get_frame_info(
			recoveryFrames1.front(), &info);
		CU_ASSERT_EQUAL(getRet, 0);
		CU_ASSERT_TRUE(info.info.flags & VDEF_FRAME_FLAG_SILENT);
	}
	for (auto *f : recoveryFrames1)
		mbuf_coded_video_frame_unref(f);

	/* A second ref non-IDR slice decrements mRecoveryFrameCount from 1 to
	 * 0 (line 3769-3770 again, alone in its own frame this time); the
	 * SILENT flag is no longer set on this one. */
	std::vector<struct mbuf_coded_video_frame *> recoveryFrames2;
	sendAuAndPop(192000u,
		     {{kH264PSlice, sizeof(kH264PSlice)}},
		     &recoveryFrames2);
	CU_ASSERT_TRUE_FATAL(!recoveryFrames2.empty());
	{
		struct vdef_coded_frame info = {};
		int getRet = mbuf_coded_video_frame_get_frame_info(
			recoveryFrames2.front(), &info);
		CU_ASSERT_EQUAL(getRet, 0);
		CU_ASSERT_FALSE(info.info.flags & VDEF_FRAME_FLAG_SILENT);
	}
	for (auto *f : recoveryFrames2)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* processFrame(): "input buffer too small" (line 3664-3669), previously 0%
 * covered. The file's usual kH264Sps (1920x800) sizes the output memory pool
 * at ~1.1MB per frame (createOutputPortMemoryPool(), pdraw_demuxer_stream.cpp
 * :3127-3131 -- width*height*3/4), far larger than any single UDP datagram
 * could ever fill, so that branch is unreachable with it. Using
 * kH264TinySps/kH264TinyPps (64x64) instead shrinks the pool to 3072 bytes.
 * A single oversized NALU is NOT how this is reached, though: every RTP
 * packet is read into a fixed DEFAULT_RX_BUFFER_SIZE=1500-byte buffer
 * (pdraw_demuxer_stream_net.cpp:49) and silently truncated to that on
 * receipt, so one huge packet never arrives intact. Instead, several
 * modestly-sized NALUs (each comfortably under 1500 bytes) are sent sharing
 * one RTP timestamp (one access unit, marker bit only on the last): their
 * *combined* size exceeds the 3072-byte budget. Differential proof: a
 * single small NAL is delivered normally; the same content split into 4
 * copies in one access unit is silently dropped. */
static void testCxxStreamDemuxerProcessFrameBufferTooSmall()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Send the tiny 64x64 SPS/PPS in-band (NOT the file's usual
	 * kH264Sps/kH264Pps): codecInfoChangedCb -> setupMedia() sizes the
	 * output pool from THIS negotiated resolution. */
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264TinySps,
			  sizeof(kH264TinySps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	struct pdraw_video_sink_params sinkParams = {};
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::ICodedVideoSink *sink = nullptr;
	ret = session->createCodedVideoSink(
		0, &sinkParams, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);
	sinkListener.mQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListener.mQueue);

	bool gotSinkMedia = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotSinkMedia);

	/* Continue directly from the SPS(seq=1)/PPS(seq=2) sent above --
	 * starting higher opens a sequence-number gap, which vstrm treats as
	 * real packet loss (concealment / "missing end of frame" handling),
	 * not as one clean, gap-free frame. */
	uint16_t seq = 3;
	auto sendOneNaluAndPop =
		[&](uint32_t ts,
		    const uint8_t *nal,
		    size_t len,
		    std::vector<struct mbuf_coded_video_frame *> *out) {
			sendRtpNaluPacket(rtpPort,
					  kTestRtspSsrc,
					  seq++,
					  ts,
					  nal,
					  len,
					  true /* marker */);
			pumpFor(loop, 200);
			struct mbuf_coded_video_frame *f = nullptr;
			while (mbuf_coded_video_frame_queue_pop(
				       sinkListener.mQueue, &f) == 0)
				out->push_back(f);
		};

	/* Baseline: kH264Idr (8 bytes) is trivially under the 3072-byte
	 * budget and is delivered normally. */
	std::vector<struct mbuf_coded_video_frame *> smallFrames;
	sendOneNaluAndPop(180000u, kH264Idr, sizeof(kH264Idr), &smallFrames);
	CU_ASSERT_TRUE_FATAL(!smallFrames.empty());
	for (auto *f : smallFrames)
		mbuf_coded_video_frame_unref(f);

	/* Same NAL content as kH264Idr (so each individual packet stays
	 * comfortably under the ~1500-byte per-packet RX buffer -- a single
	 * oversized packet would just be truncated on receipt, never
	 * reaching processFrame() as a genuinely too-big NALU, see the
	 * comment above), padded to 1000 bytes and repeated across 4 NALUs
	 * sharing one access unit (same RTP timestamp, marker bit only on
	 * the last). Their combined size (~4000 bytes) exceeds the
	 * 3072-byte budget, silently dropped by the "input buffer too
	 * small" branch. */
	std::vector<uint8_t> paddedIdr(kH264Idr, kH264Idr + sizeof(kH264Idr));
	paddedIdr.resize(1000, 0xAA);
	static constexpr int kBigNaluCount = 4;
	for (int i = 0; i < kBigNaluCount; i++) {
		sendRtpNaluPacket(
			rtpPort,
			kTestRtspSsrc,
			seq++,
			183000u,
			paddedIdr.data(),
			paddedIdr.size(),
			(i == kBigNaluCount - 1) /* marker on last */);
	}
	pumpFor(loop, 200);
	std::vector<struct mbuf_coded_video_frame *> bigFrames;
	{
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			bigFrames.push_back(f);
	}
	CU_ASSERT_TRUE(bigFrames.empty());

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* processFrame(): "failed to get an output memory" (line 3633-3639),
 * previously 0% covered. getCodedVideoOutputMemory() (pdraw_source.cpp:
 * 720-829) uses the demuxer's own internal pool whenever no downstream
 * element has installed an "external" pool via Channel::setPool() -- which
 * only decoder/encoder/scaler elements do, never ExternalCodedVideoSink (the
 * public createCodedVideoSink() sink used by every test in this file). That
 * internal pool is created by createOutputPortMemoryPool()
 * (pdraw_demuxer_stream.cpp:3127-3131) with mbuf_pool_new(...,
 * DEMUXER_STREAM_OUTPUT_BUFFER_COUNT=60, MBUF_POOL_NO_GROW, ...)
 * (pdraw_demuxer_stream.hpp:47): a fixed 60-buffer pool that never grows, so
 * mbuf_pool_get() returns -EAGAIN once all 60 are checked out.
 *
 * A buffer stays checked out for as long as its frame is neither popped nor
 * unref'd -- so simply never draining the sink's queue while delivering 60
 * IDR frames leaves all 60 buffers outstanding; the 61st delivery then hits
 * this exact branch.
 *
 * Two side effects make this externally observable without any private
 * member access:
 *   - The failure path calls flush(true) (the default), which calls
 *     resync() (line 3444) and then channel->flush() on every output
 *     channel. DrainingCodedVideoSinkListener::onCodedVideoSinkFlush acks
 *     via queueFlushed() and, in the same callback, discards (pops+unrefs)
 *     every frame currently sitting in the queue -- so mGotFlush flips to
 *     true and the 60 previously-held frames disappear from the queue in
 *     one shot, both directly attributable to this failure path (nothing
 *     else in this test ever calls flush()/resync()).
 *   - flush()'s resync() sets mWaitForSync=true, but since every frame sent
 *     here is an IDR, the very next frame clears it immediately and is
 *     delivered normally, proving the demuxer recovers once buffers are
 *     freed instead of getting stuck. */
static void testCxxStreamDemuxerProcessFrameOutputMemoryExhausted()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Continue directly from openPlayAndAttachSink()'s SPS(seq=1)/PPS
	 * (seq=2): starting a gap causes vstrm to treat it as real packet
	 * loss instead of one clean frame per send (see other tests in this
	 * file for the same rationale). */
	uint16_t seq = 3;
	static constexpr uint32_t kIdrBaseTs = 180000u;
	static constexpr uint32_t kRtpTicksPerFrame = 3000u; /* 33ms @ 90kHz */
	uint32_t idrTs = kIdrBaseTs;

	/* Fill all DEMUXER_STREAM_OUTPUT_BUFFER_COUNT=60 output buffers by
	 * delivering 60 IDR frames one at a time (distinct, increasing RTP
	 * timestamps + marker bit so each is released individually) WITHOUT
	 * popping the sink queue in between. */
	CU_ASSERT_FALSE_FATAL(sinkListener.mGotFlush);
	for (int i = 0; i < 60; i++) {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  seq++,
				  idrTs,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		idrTs += kRtpTicksPerFrame;
		pomp_loop_wait_and_process(loop.raw(), 20);
	}
	CU_ASSERT_FALSE(sinkListener.mGotFlush);

	/* The 61st IDR finds the pool exhausted: getCodedVideoOutputMemory()
	 * returns -EAGAIN, processFrame() hits the "failed to get an output
	 * memory" branch and calls flush(), which the sink acks (discarding
	 * every frame held so far). */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  idrTs,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	idrTs += kRtpTicksPerFrame;
	bool gotFlush = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotFlush; }, 2000);
	CU_ASSERT_TRUE_FATAL(gotFlush);

	{
		struct mbuf_coded_video_frame *f = nullptr;
		CU_ASSERT_NOT_EQUAL(mbuf_coded_video_frame_queue_pop(
					    sinkListener.mQueue, &f),
				    0);
	}

	/* Recovery: with all 60 buffers freed by the flush-triggered
	 * discard, and mWaitForSync cleared by this IDR, a fresh frame is
	 * delivered normally. */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  idrTs,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	pumpFor(loop, 200);
	std::vector<struct mbuf_coded_video_frame *> recoveredFrames;
	{
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListener.mQueue,
							&f) == 0)
			recoveredFrames.push_back(f);
	}
	CU_ASSERT_TRUE_FATAL(!recoveredFrames.empty());
	for (auto *f : recoveredFrames)
		mbuf_coded_video_frame_unref(f);

	sinkOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* processFrame(): format-conversion copy failure (line 3953-3966,
 * previously 0% covered).
 *
 * setupMedia() (pdraw_demuxer_stream.cpp:3022-3171) always creates TWO
 * CodedVideoMedia format variants per VideoMedia -- mVideoMedias[0]
 * (vdef_h264_avcc) and mVideoMedias[1] (vdef_h264_byte_stream) -- and
 * explicitly SHARES the same 60-buffer output pool between them (lines
 * 3137-3145: mediaPort->pool = basePort->pool). A sink's requested format
 * (pdraw_video_sink_params::required_coded_format) selects which of the two
 * it binds to via vdef_coded_format_intersect() in Sink::addInputMedia: a
 * default-initialized (all-zero/UNKNOWN) params binds to the first media
 * with no caps filter (avcc, index 0); requesting vdef_h264_byte_stream
 * explicitly binds a second sink to index 1.
 *
 * With both sinks attached, every delivered frame produces the avcc frame
 * "for free" (a plain ref() on the already-allocated primary buffer, the
 * "else" branch at line 3967-3969) plus a SEPARATE, freshly checked-out
 * buffer for the byte-stream copy (copyCodedVideoOutputFrame() ->
 * Source::getCodedVideoOutputMemory() scoped to just that one media). Both
 * checkouts draw from the same shared pool. Draining sink A's (avcc) queue
 * after every frame keeps its buffer usage at zero, while never draining
 * sink B's (byte-stream) queue lets its copies accumulate one buffer each,
 * permanently. After 59 frames, sink B alone holds 59 of the 60 shared
 * buffers; on the 60th frame the primary avcc capture still succeeds
 * (exactly one buffer remains free), but the subsequent copy for sink B
 * finds the pool now fully exhausted -> -EAGAIN -> copy_ret < 0, hitting
 * this exact branch.
 *
 * Externally observable without any private member access: sink A keeps
 * receiving a fresh frame on every single delivery, including the 60th --
 * this failure is silent to the rest of the pipeline (only a
 * PDRAW_LOG_ERRNO, no flush()) -- while sink B's queue count plateaus at 59
 * and does not grow on the 60th delivery. */
static void testCxxStreamDemuxerProcessFrameFormatCopyFailure()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListenerA;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sinkA = openPlayAndAttachSink(session,
							       loop,
							       url,
							       &demuxListener,
							       &mediaListener,
							       &sinkListenerA,
							       &demuxer);
	auto sinkAOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sinkA);

	/* Second sink, explicitly requesting the byte-stream format so it
	 * binds to mVideoMedias[1] instead of the default avcc [0]. */
	DrainingCodedVideoSinkListener sinkListenerB;
	struct pdraw_video_sink_params sinkParamsB = {};
	sinkParamsB.required_coded_format = vdef_h264_byte_stream;
	IPdraw::ICodedVideoSink *sinkB = nullptr;
	int ret = session->createCodedVideoSink(
		0, &sinkParamsB, &sinkListenerB, &sinkB);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkB);
	auto sinkBOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sinkB);
	sinkListenerB.mQueue = sinkB->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(sinkListenerB.mQueue);
	bool gotSinkBMedia = loop.pumpUntil(
		[&sinkListenerB]() { return sinkListenerB.mGotMediaAdded; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotSinkBMedia);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Continue directly from openPlayAndAttachSink()'s SPS(seq=1)/PPS
	 * (seq=2): starting a gap causes vstrm to treat it as real packet
	 * loss instead of one clean frame per send (see other tests in this
	 * file for the same rationale). */
	uint16_t seq = 3;
	static constexpr uint32_t kIdrBaseTs = 180000u;
	static constexpr uint32_t kRtpTicksPerFrame = 3000u; /* 33ms @ 90kHz */
	uint32_t idrTs = kIdrBaseTs;

	/* Frames 1..59: both the avcc capture and the byte-stream copy
	 * succeed. Sink A is drained after every frame (stays near-empty);
	 * sink B is never drained, so its queue count grows by exactly one
	 * frame each time, reaching 59 undrained buffers. */
	for (int i = 0; i < 59; i++) {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  seq++,
				  idrTs,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		idrTs += kRtpTicksPerFrame;
		pomp_loop_wait_and_process(loop.raw(), 20);

		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListenerA.mQueue,
							&f) == 0)
			mbuf_coded_video_frame_unref(f);
	}
	CU_ASSERT_FALSE_FATAL(sinkListenerA.mGotFlush);
	CU_ASSERT_FALSE_FATAL(sinkListenerB.mGotFlush);
	int countB =
		mbuf_coded_video_frame_queue_get_count(sinkListenerB.mQueue);
	CU_ASSERT_EQUAL_FATAL(countB, 59);

	/* Frame 60: the shared pool now has exactly one free buffer. The
	 * primary avcc capture takes it (sink A still receives this frame
	 * normally), leaving none for the byte-stream copy attempted for
	 * sink B -- hitting copy_ret < 0 (line 3962). */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  idrTs,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	idrTs += kRtpTicksPerFrame;
	pomp_loop_wait_and_process(loop.raw(), 20);

	/* No flush() is ever triggered by this branch: only the sink whose
	 * copy failed silently misses this one frame. */
	CU_ASSERT_FALSE(sinkListenerA.mGotFlush);
	CU_ASSERT_FALSE(sinkListenerB.mGotFlush);

	{
		struct mbuf_coded_video_frame *f = nullptr;
		CU_ASSERT_EQUAL(mbuf_coded_video_frame_queue_pop(
					sinkListenerA.mQueue, &f),
				0);
		if (f != nullptr)
			mbuf_coded_video_frame_unref(f);
	}

	/* Sink B's count did NOT grow past 59: the 60th copy failed and no
	 * new frame was queued for it. */
	countB = mbuf_coded_video_frame_queue_get_count(sinkListenerB.mQueue);
	CU_ASSERT_EQUAL(countB, 59);

	{
		struct mbuf_coded_video_frame *f = nullptr;
		while (mbuf_coded_video_frame_queue_pop(sinkListenerB.mQueue,
							&f) == 0)
			mbuf_coded_video_frame_unref(f);
	}

	sinkAOwner.reset();
	sinkBOwner.reset();
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* recvFrameCb(): mCodecInfoChanging temp-queue overflow, "temp queue is full,
 * dropping frame" (pdraw_demuxer_stream.cpp:4344-4358), previously 0%
 * covered.
 *
 * mCodecInfoChanging is set true by codecInfoChangedCb() (line 4267) only in
 * its "change of output media" branch, i.e. a SECOND in-band SPS/PPS arrives
 * after a first one already set up mVideoMedias (VideoMedia's own inner
 * CodedVideoMedia list, not to be confused with StreamDemuxer::mVideoMedias
 * accessed as sd->mVideoMedias below). It is cleared back to false only from
 * channelUnlink() (line 3529/3534), itself only reachable through
 * codecInfoChangedCb()'s per-output-channel teardown loop (line 4268-4289) --
 * which is a no-op when outputChannelCount==0. Deliberately NOT attaching
 * any coded video sink therefore leaves mCodecInfoChanging permanently true
 * once set, independent of any timing race: a clean, indefinitely-open
 * window to fill mTempQueue from a test.
 *
 * While mCodecInfoChanging is true, recvFrameCb() (line 4344) never calls
 * processFrame() at all -- it only pushes onto mTempQueue (capped at
 * DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE=30, pdraw_demuxer_stream.hpp:53),
 * dropping the oldest entry once full. Verified directly via the
 * `#define private public` access already used throughout this file (no
 * sink/pool interaction needed, since these frames never reach
 * processFrame()). */
static void testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* First SPS/PPS: mVideoMedias is empty, so codecInfoChangedCb() takes
	 * the "new output media" branch (line 4296-4299), calling
	 * setupMedia() directly and leaving mCodecInfoChanging false. */
	static constexpr uint32_t kSetupTs1 = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs1,
			  kH264TinySps,
			  sizeof(kH264TinySps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs1,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());
	auto *vm = sd->mVideoMedias[0].get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);
	CU_ASSERT_FALSE(vm->mCodecInfoChanging);

	/* Second SPS/PPS, a different resolution (this file's usual 1920x800
	 * kH264Sps/kH264Pps): mVideoMedias (VideoMedia's inner list) is now
	 * non-empty, so codecInfoChangedCb() takes the "change of output
	 * media" branch (line 4265-4295), setting mCodecInfoChanging=true.
	 * With no sink attached, its per-channel teardown loop iterates zero
	 * times, so nothing ever calls channelUnlink() to clear the flag. */
	uint16_t seq = 3;
	static constexpr uint32_t kSetupTs2 = 93000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  kSetupTs2,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  kSetupTs2,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotChanging =
		loop.pumpUntil([vm]() { return vm->mCodecInfoChanging; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotChanging);
	CU_ASSERT_TRUE_FATAL(vm->mTempQueue.empty());

	/* Deliver DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE(=30) + 5 distinct IDR
	 * access units (unique RTP timestamps + marker bit on each, so every
	 * one is released by the jitter buffer individually): since
	 * mCodecInfoChanging is stuck true, recvFrameCb() queues every one
	 * into mTempQueue instead of calling processFrame(). Once the queue
	 * holds 30, each further send hits "temp queue is full, dropping
	 * frame", popping/unref'ing the front (oldest) before pushing the new
	 * one -- so the queue size caps at exactly 30 and stays there. */
	static constexpr uint32_t kFrameTs0 = 96000u;
	static constexpr uint32_t kRtpTicksPerFrame = 3000u; /* 33ms @ 90kHz */
	static constexpr int kSendCount = 35;
	for (int i = 0; i < kSendCount; i++) {
		sendRtpNaluPacket(rtpPort,
				  kTestRtspSsrc,
				  seq++,
				  kFrameTs0 + i * kRtpTicksPerFrame,
				  kH264Idr,
				  sizeof(kH264Idr),
				  true /* marker */);
		pomp_loop_wait_and_process(loop.raw(), 20);
	}

	CU_ASSERT_EQUAL(vm->mTempQueue.size(),
			DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* codecInfoChangedCb(): "codec info changed; no change in PS, just resync"
 * (pdraw_demuxer_stream.cpp:4230-4235), previously 0% covered.
 *
 * vstrm_codec_info_cmp() (vstrm_frame.c:249-264) returns true only when two
 * struct vstrm_codec_info are byte-for-byte equal (codec, width, height,
 * SPS, PPS). vstrm's OWN H.264 depayloader already suppresses re-invoking
 * this callback for identical SPS/PPS re-sent while its internal cache is
 * unchanged (pps_received(), vstrm_rtp_h264_rx.c, compares against its own
 * self->codec_info before ever calling back into pdraw) -- simply re-
 * sending the exact same SPS/PPS bytes a second time does nothing by
 * itself. To reach this branch, vstrm's OWN cache must first be wiped
 * (independently of pdraw's own persistent VideoMedia::mCodecInfo, a
 * SEPARATE field only ever touched inside this very function), then the
 * SAME SPS/PPS re-sent so the newly-derived info again compares equal to
 * pdraw's already-cached mCodecInfo.
 *
 * The trigger is RFC 3550 Appendix A.1's "source restarted without telling
 * us" heuristic in vstrm_receiver_update_seq() (vstrm_receiver.c:784-835):
 * two consecutive packets on the SAME SSRC whose sequence numbers jump by
 * udelta in [MAX_DROPOUT=20000, RTP_SEQ_MOD-MAX_MISORDER=45536), the second
 * one landing exactly on the first one's computed "bad_seq". This confirms
 * a restart and calls vstrm_receiver_init_seq(..., keep_ps=false)
 * (vstrm_receiver.c:824), which zeroes vstrm_rtp_h264_rx's own cached
 * sps.valid/pps.valid/codec_info (vstrm_rtp_h264_rx.c:2743-2751). (The
 * restore-from-receiver-cache guard right after, vstrm_receiver.c:777-780,
 * only fires if vstrm_receiver_set_codec_info() was ever called for this
 * SSRC from an out-of-band source -- never true here, since pdraw only
 * calls it with SDP fmtp SPS/PPS, and that whole code path is dead
 * (#if 0'd out in onNewSdp, confirmed while investigating an earlier test
 * in this file).) Two throwaway AUD NALs (kH264Aud, already used elsewhere
 * in this file as an inert "default case" NAL type) carry the seqnum jump;
 * re-sending the SAME kH264TinySps/kH264TinyPps immediately after is then
 * parsed as genuinely new by vstrm's now-blank cache, compares equal to
 * pdraw's still-populated mCodecInfo, and fires codecInfoChangedCb() with
 * !mCodecInfoChanging -- hitting this exact branch.
 *
 * Observed via VideoMedia::mWaitForSync (private member, `#define private
 * public` already in effect in this file). setupMedia() itself calls
 * resync() once during initial setup (line 3148), so mWaitForSync starts
 * true; an IDR is sent first to clear it back to false (confirmed by a real
 * build -- an earlier version of this test wrongly assumed it started
 * false). After that, nothing else in this test calls resync()/flush()
 * except the branch under test, so its transition back to true is directly
 * attributable to it. No sink is attached -- not needed, since this
 * mechanism lives entirely in codecInfoChangedCb(), upstream of
 * processFrame(). */
static void testCxxStreamDemuxerCodecInfoUnchangedResyncOnly()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Normal setup: seq 1 (SPS), seq 2 (PPS). vstrm's internal
	 * source.max_seq ends at 2, and pdraw's VideoMedia::mCodecInfo is
	 * populated with this SPS/PPS's content via the "new output media"
	 * branch. */
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264TinySps,
			  sizeof(kH264TinySps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());
	auto *vm = sd->mVideoMedias[0].get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);
	CU_ASSERT_FALSE_FATAL(vm->mCodecInfoChanging);

	/* setupMedia() itself calls resync() (line 3148) as part of normal
	 * initialization, so mWaitForSync is already true at this point --
	 * confirmed by a real build (this assertion originally expected
	 * false here and failed). An IDR (seq=3) clears it, giving a clean
	 * false baseline before the branch under test. */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  3,
			  91000u,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	bool gotSync =
		loop.pumpUntil([vm]() { return !vm->mWaitForSync; }, 2000);
	CU_ASSERT_TRUE_FATAL(gotSync);

	/* Two throwaway AUD NALs whose sequence numbers jump by 25000
	 * (within [MAX_DROPOUT=20000, RTP_SEQ_MOD-MAX_MISORDER=45536)) from
	 * max_seq=3 (the IDR above), the second landing on the first one's
	 * computed bad_seq: vstrm's RFC 3550 "source restarted" heuristic
	 * fires, wiping its cached SPS/PPS. */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  25003,
			  93000u,
			  kH264Aud,
			  sizeof(kH264Aud),
			  true /* marker */);
	pomp_loop_wait_and_process(loop.raw(), 20);
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  25004,
			  96000u,
			  kH264Aud,
			  sizeof(kH264Aud),
			  true /* marker */);
	pomp_loop_wait_and_process(loop.raw(), 20);

	/* Re-send the SAME SPS/PPS: vstrm re-parses them as genuinely new
	 * (its own cache is blank), the resulting info compares equal to
	 * pdraw's still-populated VideoMedia::mCodecInfo, and
	 * codecInfoChangedCb() takes the "no change in PS, just resync"
	 * branch instead of "change of output media". */
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  25005,
			  99000u,
			  kH264TinySps,
			  sizeof(kH264TinySps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  25006,
			  99000u,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));

	bool gotResync =
		loop.pumpUntil([vm]() { return vm->mWaitForSync; }, 2000);
	CU_ASSERT_TRUE_FATAL(gotResync);
	CU_ASSERT_FALSE(vm->mCodecInfoChanging);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* ── VideoMedia::next() + asyncCompleteSeek (nextFrame completion) ─────── */

/* Exercises VideoMedia::next() and the asyncCompleteSeek → idleCompleteSeek →
 * completeSeek → onMediaSeekComplete → seekResponse path for nextFrame().
 *
 * Key ordering constraint: nextFrame() must be called BEFORE sending SPS/PPS,
 * because VideoMedia::mRtpPaused defaults to true and processDataPkt drops
 * all RTP packets while mRtpPaused=true.  VideoMedia::next() sets it to false.
 *
 * Unlike seekTo(), StreamDemuxer::next() does NOT set mSeekingNetwork, so
 * asyncCompleteSeek fires as soon as the first non-silent IDR arrives.
 *
 * StreamDemuxer::next() sets pendingCommand=SEEK (not PAUSE_NEXT) when the
 * initial command is NONE.  onMediaSeekComplete() therefore routes to
 * demuxerSeekResponse (SEEK command), not demuxerPauseResponse (PAUSE_NEXT).
 * PAUSE_NEXT is only preserved by next() if it was already set by a prior
 * pause() call when !mWasRunningOnce.
 *
 * mFrameByFrame defaults to true (demuxer starts "paused"), so nextFrame()
 * reaches VideoMedia::next() right after readyToPlay without needing an
 * explicit pause() first. */
static void testCxxStreamDemuxerNextFrameCompletion()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* Call nextFrame() BEFORE sending SPS/PPS.  VideoMedia::next() sets
	 * mRtpPaused=false, enabling codec-info packets to be processed by
	 * vstrm.  mSeekingNetwork is NOT set by next() (unlike seekTo()), so
	 * asyncCompleteSeek fires immediately on the first non-silent IDR. */
	int ret = demuxer->nextFrame();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* SPS + PPS are now accepted (mRtpPaused=false): codecInfoChangeCb
	 * → newOutputMedia creates CodedVideoMedia entries. */
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Incrementing-timestamp IDR loop (same pattern as seek test): each
	 * new IDR triggers vstrm to release the previous frame immediately
	 * via timestamp-change, bypassing the ~30ms jitter timer which can
	 * be unreliable when the predicate-based pumpUntil exits early.
	 * asyncCompleteSeek fires on the first non-silent IDR:
	 * (mPendingSeek=true, !mSeekingNetwork [nextFrame never sets it],
	 * !SILENT) → idleCompleteSeek → completeSeek → onMediaSeekComplete
	 * → seekResponse (SEEK command) → demuxerSeekResponse →
	 * mGotSeekResponse=true. */
	{
		uint32_t idrTs = 180000u;
		uint16_t idrSeq = 3;
		auto deadline = std::chrono::steady_clock::now() +
				std::chrono::milliseconds(2000);
		while (!demuxListener.mGotSeekResponse) {
			if (std::chrono::steady_clock::now() >= deadline)
				break;
			sendRtpNaluPacket(rtpPort,
					  kTestRtspSsrc,
					  idrSeq++,
					  idrTs,
					  kH264Idr,
					  sizeof(kH264Idr),
					  true /* marker */);
			idrTs += 3000;
			(void)loop.pumpUntil([]() { return false; }, 60);
		}
	}
	CU_ASSERT_TRUE(demuxListener.mGotSeekResponse);
	CU_ASSERT_EQUAL(demuxListener.mSeekStatus, 0);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* ── onFrameTimeout: fires 2 s after the last RTP frame is processed ───── */

/* Exercises VideoMedia::onFrameTimeout() → sendDownstreamEvent(TIMEOUT).
 *
 * Flow: play → SPS/PPS → gotMedia → one IDR (M=1).  The vstrm jitter-buffer
 * releases the IDR via its own ~30ms timer → recvFrameCb → resetFrameTimer(
 * 2000ms) → processFrame → mLastFrameReceiveTime is set.  With no further
 * frames for 2000ms the frame timer fires → onFrameTimeout checks that
 * 2000 µs have elapsed (always true after 2s) and sends TIMEOUT downstream.
 * No sink is attached, so sendDownstreamEvent iterates an empty channel list
 * harmlessly. */
static void testCxxStreamDemuxerFrameTimeout()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* One IDR (M=1) arms the 2000ms frame timer via recvFrameCb →
	 * resetFrameTimer(true).  The vstrm jitter-buffer holds the frame for
	 * ~30ms then releases it; pumpFor(150) gives ample margin. */
	static constexpr uint32_t kIdrTs = 180000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  10,
			  kIdrTs,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	pumpFor(loop, 150);

	/* 2500ms: the 2000ms frame timer fires → onFrameTimeout(). */
	pumpFor(loop, 2500);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* ── endOfRange tracking listener (tests 4–5) ───────────────────────────── */

class EndOfRangeTrackingListener : public StreamDemuxerListener {
public:
	void onDemuxerEndOfRange(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 uint64_t ts) override
	{
		mEndOfRangeTs = ts;
		mGotEndOfRange = true;
	}

	bool mGotEndOfRange = false;
	uint64_t mEndOfRangeTs = 0;
};


/* ── RTCP APP event: RESOLUTION_CHANGE (event id = 2) ───────────────────── */

/* eventCb() RESOLUTION_CHANGE branch: sends Channel::DownstreamEvent::
 * RESOLUTION_CHANGE downstream and sets mRtcpMediaChangeReceived.
 * ExternalCodedVideoSink::onChannelResolutionChange() sets mPendingRestart=true
 * (pdraw_external_coded_video_sink.cpp), which is observed via
 * onCodedVideoSinkMediaRemoved(restart=true) after close(). */
static void testCxxStreamDemuxerEventResolutionChange()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	static constexpr uint8_t kVstrmEventResolutionChange = 2;
	sendRtcpEvent(ctrlPort, kTestRtspSsrc, kVstrmEventResolutionChange);
	pumpFor(loop, 300);

	closeStreamDemuxer(demuxer, loop, &demuxListener);

	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
}


/* ── RTCP APP event: FRAMERATE_CHANGE (event id = 4) ────────────────────── */

/* eventCb() FRAMERATE_CHANGE branch: identical control flow to
 * RESOLUTION_CHANGE (sets mRtcpMediaChangeReceived, sends FRAMERATE_CHANGE
 * downstream). ExternalCodedVideoSink::onChannelFramerateChange() →
 * mPendingRestart=true, verified via
 * onCodedVideoSinkMediaRemoved(restart=true). */
static void testCxxStreamDemuxerEventFramerateChange()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	static constexpr uint8_t kVstrmEventFramerateChange = 4;
	sendRtcpEvent(ctrlPort, kTestRtspSsrc, kVstrmEventFramerateChange);
	pumpFor(loop, 300);

	closeStreamDemuxer(demuxer, loop, &demuxListener);

	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
}


/* ── RTCP APP event: PHOTO_TRIGGER (event id = 3) ───────────────────────── */

/* eventCb() PHOTO_TRIGGER branch: sends Channel::DownstreamEvent::PHOTO_TRIGGER
 * downstream without setting mRtcpMediaChangeReceived.  Sink's
 * onChannelPhotoTrigger() is a no-op (base Sink::onChannelPhotoTrigger), so
 * mPendingRestart stays false — verified via onCodedVideoSinkMediaRemoved. */
static void testCxxStreamDemuxerEventPhotoTrigger()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	uint16_t ctrlPort = demuxer->getSingleStreamLocalControlPort();
	CU_ASSERT_FATAL(ctrlPort > 0);
	static constexpr uint8_t kVstrmEventPhotoTrigger = 3;
	sendRtcpEvent(ctrlPort, kTestRtspSsrc, kVstrmEventPhotoTrigger);
	pumpFor(loop, 300);

	closeStreamDemuxer(demuxer, loop, &demuxListener);

	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
	/* PHOTO_TRIGGER does not set mPendingRestart in ExternalCodedVideoSink
	 * (only RECONFIGURE / RESOLUTION_CHANGE / FRAMERATE_CHANGE do). */
	CU_ASSERT_FALSE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
}


/* ── idleEndOfRangeNotification via PLAY response start == stop ─────────── */

/* Exercises StreamDemuxer::idleEndOfRangeNotification().
 *
 * onRtspPlayResp checks: if ((start == stop) && start && stop) → idleAdd(
 * mEndOfRangeNotificationHandler) → idleEndOfRangeNotification() →
 * onEndOfRange() → onDemuxerEndOfRange() on listener.
 *
 * Triggered by mPlayReturnsEndOfRange: the server's playCb overrides the
 * echoed range with start == stop == 120s (both non-zero), simulating a
 * server that reports the stream is already at its end point. */
static void testCxxStreamDemuxerIdleEndOfRangeNotification()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true;
	server.mPlayReturnsEndOfRange = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	EndOfRangeTrackingListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	/* Play response triggers idleEndOfRangeNotification on the SAME idle
	 * cycle (idleAdd queued during onRtspPlayResp → fired next pump). */
	bool gotEor = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotEndOfRange; },
		5000);
	CU_ASSERT_TRUE(gotEor);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* ── onRangeTimer: RTP frame near end of track arms the range timer ──────── */

/* Exercises VideoMedia::onRangeTimer() via processFrame().
 *
 * Flow:
 *   mHasDuration=true → mTrackDuration=120s.  mNtpToNptOffset=0 (server
 *   echoes npt=0-120, ts=0 in playCb). IDR at RTP ts=10,755,000 maps to
 *   npt_raw = 10755000 * 1e6 / 90000 = 119,500,000 µs.
 *   remainingPlayTime = 120,000,000 - 119,500,000 = 500,000 µs < 1,000,000 µs
 *   → mRangeTimer->set(delay) with delay = 500000/1000 + 50 = 550 ms.
 *   After 700 ms, onRangeTimer() fires → onEndOfRange() →
 *   onDemuxerEndOfRange() on listener.
 *
 * To drain the jitter buffer:
 *   1. SPS/PPS at ts=90000 are released when IDR1 (ts=10,755,000) arrives.
 *   2. IDR1 is released when IDR2 (ts=10,758,000) arrives.
 *   3. pumpFor(150) gives ample margin for both vstrm timer and processFrame.
 */
static void testCxxStreamDemuxerRangeTimer()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	EndOfRangeTrackingListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);

	bool gotReady = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotReadyToPlay; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* SPS + PPS at ts=90000 (1 s).  Released once IDR1 (higher ts) arrives.
	 * processFrame discards them as empty frames (0 slices). */
	static constexpr uint32_t kSetupTs = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* IDR1 at ts=10,755,000 (≈119.5 s): triggers release of SPS/PPS.
	 * Use seq=3 (consecutive after PPS=2) to avoid jitter-buffer gap hold.
	 */
	static constexpr uint32_t kIdr1Ts = 10755000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  3,
			  kIdr1Ts,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);

	/* IDR2 at ts=10,758,000 (≈119.53 s): triggers release of IDR1.
	 * processFrame(IDR1) → remainingPlayTime≈500ms → mRangeTimer->set(550).
	 */
	static constexpr uint32_t kIdr2Ts = 10758000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  4,
			  kIdr2Ts,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);

	/* 150ms: vstrm jitter buffer drains IDR1 via recvFrameCb → processFrame
	 * arms the range timer (≈550ms). */
	pumpFor(loop, 150);

	/* 1500ms: range timer (≈550ms) fires → onRangeTimer() → idleAdd →
	 * onDemuxerEndOfRange().  Extra margin for idle-callback latency. */
	bool gotEor = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotEndOfRange; },
		1500);
	CU_ASSERT_TRUE(gotEor);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* Exercises StreamDemuxer::onRtspAnnounce().
 *
 * It tests three main aspects of the ANNOUNCE handling logic:
 *  1. Non-matching content_base: ANNOUNCE is ignored and no SDP processing is
 * triggered.
 *  2. Matching content_base with new SDP content: SDP parsing and processing is
 * triggered, updating session configuration (we verify no unrecoverable errors
 * occur).
 *  3. Invalid SDP content: SDP parsing fails cleanly and exits without
 * crashing. */
static void testCxxStreamDemuxerRtspAnnounce()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener, 10000);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Send ANNOUNCE with non-matching content_base -> should be ignored
	 */
	char wrongUri[] = "rtsp://127.0.0.1:18554/wrongpath";
	char pushedSdp[] =
		"v=0\r\n"
		"o=- 1 1 IN IP4 127.0.0.1\r\n"
		"s=TestStream\r\n"
		"c=IN IP4 0.0.0.0\r\n"
		"t=0 0\r\n"
		"a=control:*\r\n"
		"m=video 0 RTP/AVP 96\r\n"
		"a=rtpmap:96 H264/90000\r\n"
		"a=control:stream=0\r\n";
	int ret = rtsp_server_announce(
		server.raw(), wrongUri, nullptr, 0, pushedSdp);
	CU_ASSERT_EQUAL(ret, 0);
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	/* 2. Send ANNOUNCE with matching content_base.
	 * We use the content_base parsed during DESCRIBE (stored in
	 * sd->mContentBase). Due to a limitation/bug in librtsp
	 * client_uri_to_content_base, absolute URIs received via ANNOUNCE are
	 * prepended with the connection base. To make the guard pass, we
	 * temporarily prefix mContentBase to match what the client constructs.
	 */
	std::string originalContentBase = sd->mContentBase;
	std::string hostPort = originalContentBase;
	size_t slashPos = hostPort.find('/', 7); /* skip "rtsp://" */
	if (slashPos != std::string::npos)
		hostPort = hostPort.substr(0, slashPos);
	std::string expectedContentBase = hostPort + "/" + originalContentBase;
	sd->mContentBase = expectedContentBase;

	const char *matchingUri = originalContentBase.c_str();
	char newPushedSdp[] =
		"v=0\r\n"
		"o=- 1 2 IN IP4 127.0.0.1\r\n"
		"s=TestStream2\r\n"
		"i=My Friendly Session Info\r\n"
		"c=IN IP4 0.0.0.0\r\n"
		"t=0 0\r\n"
		"a=tool:My Soft Ver 1.0\r\n"
		"a=X-com-parrot-maker:Parrot\r\n"
		"a=X-com-parrot-model:Anafi\r\n"
		"a=X-com-parrot-serial:123456\r\n"
		"a=control:*\r\n"
		"m=video 0 RTP/AVP 96\r\n"
		"a=rtpmap:96 H264/90000\r\n"
		"a=control:stream=0\r\n";

	ret = rtsp_server_announce(server.raw(),
				   const_cast<char *>(matchingUri),
				   nullptr,
				   0,
				   newPushedSdp);
	CU_ASSERT_EQUAL(ret, 0);
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	/* Verify metadata is parsed and stored correctly */
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.title, "TestStream2");
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.friendly_name,
			       "My Friendly Session Info");
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.software_version,
			       "My Soft Ver 1.0");
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.maker, "Parrot");
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.model, "Anafi");
	CU_ASSERT_STRING_EQUAL(sd->mSessionMetaFromSdp.serial_number, "123456");

	/* 3. Send ANNOUNCE with invalid SDP -> parsing fails cleanly */
	char invalidSdp[] = "invalid_sdp_content\r\n";
	ret = rtsp_server_announce(server.raw(),
				   const_cast<char *>(matchingUri),
				   nullptr,
				   0,
				   invalidSdp);
	CU_ASSERT_EQUAL(ret, 0);
	(void)loop.pumpUntil([]() { return false; }, 200);
	CU_ASSERT_FALSE(demuxListener.mGotUnrecoverableError);

	/* Restore original content base */
	sd->mContentBase = originalContentBase;

	/* 4. Test entering onRtspInterleavedDataCb.
	 * We simulate this by changing the first video media's transport to TCP
	 * interleaved, setting its remote stream port to channel 0, and feeding
	 * a mock RTP packet. We verify the packet was processed by asserting
	 * that vstrm_receiver's received packet count increments. */
	if (!sd->mVideoMedias.empty()) {
		Pdraw::StreamDemuxerNet::VideoMediaNet *vm =
			static_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
				sd->mVideoMedias[0].get());
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm->mReceiver);

		enum rtsp_lower_transport originalTransport =
			vm->mLowerTransport;
		uint16_t originalRemoteStreamPort = vm->mRemoteStreamPort;
		bool originalRtpPaused = vm->mRtpPaused;

		vm->mLowerTransport = RTSP_LOWER_TRANSPORT_TCP;
		vm->mRemoteStreamPort = 0;
		vm->mRtpPaused = false;

		uint8_t rtp_data1[] = {
			0x80,
			0x60,
			0x00,
			0x01, /* RTP header: V=2, PT=96, seq=1 */
			0x00,
			0x00,
			0x00,
			0x00, /* RTP timestamp */
			0x12,
			0x34,
			0x56,
			0x78, /* RTP SSRC */
			0x00,
			0x00,
			0x00,
			0x00 /* Payload */
		};
		uint8_t rtp_data2[] = {
			0x80,
			0x60,
			0x00,
			0x02, /* RTP header: V=2, PT=96, seq=2 */
			0x00,
			0x00,
			0x00,
			0x00, /* RTP timestamp */
			0x12,
			0x34,
			0x56,
			0x78, /* RTP SSRC */
			0x00,
			0x00,
			0x00,
			0x00 /* Payload */
		};
		uint8_t rtp_data3[] = {
			0x80,
			0x60,
			0x00,
			0x03, /* RTP header: V=2, PT=96, seq=3 */
			0x00,
			0x00,
			0x00,
			0x00, /* RTP timestamp */
			0x12,
			0x34,
			0x56,
			0x78, /* RTP SSRC */
			0x00,
			0x00,
			0x00,
			0x00 /* Payload */
		};

		struct vstrm_receiver_stats statsBefore = {};
		int statsRet =
			vstrm_receiver_get_stats(vm->mReceiver, &statsBefore);
		CU_ASSERT_EQUAL(statsRet, 0);

		sd->onRtspInterleavedDataCb(nullptr,
					    0, /* channel 0 */
					    rtp_data1,
					    sizeof(rtp_data1),
					    sd);

		sd->onRtspInterleavedDataCb(nullptr,
					    0, /* channel 0 */
					    rtp_data2,
					    sizeof(rtp_data2),
					    sd);

		sd->onRtspInterleavedDataCb(nullptr,
					    0, /* channel 0 */
					    rtp_data3,
					    sizeof(rtp_data3),
					    sd);

		struct vstrm_receiver_stats statsAfter = {};
		statsRet = vstrm_receiver_get_stats(vm->mReceiver, &statsAfter);
		CU_ASSERT_EQUAL(statsRet, 0);
		CU_ASSERT_TRUE(statsAfter.received_packet_count >
			       statsBefore.received_packet_count);

		/* Restore original transport state */
		vm->mLowerTransport = originalTransport;
		vm->mRemoteStreamPort = originalRemoteStreamPort;
		vm->mRtpPaused = originalRtpPaused;
	}

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerOnChannelVideoPresStats()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Null parameters check */
	struct VideoPresStats stats = {};
	stats.timestamp = 12345678;
	stats.presentationFrameCount = 100;

	sd->onChannelVideoPresStats(nullptr, &stats);
	sd->onChannelVideoPresStats(nullptr, nullptr);

	/* 2. Channel not attached to demuxer output media */
	Pdraw::Channel *dummyChannel =
		reinterpret_cast<Pdraw::Channel *>(0x1234);
	sd->onChannelVideoPresStats(dummyChannel, &stats);

	/* 3. Valid channel connected to output media */
	Pdraw::Channel *outputChannel = nullptr;
	unsigned int mediaCount = sd->getOutputMediaCount();
	for (unsigned int i = 0; i < mediaCount; i++) {
		const Pdraw::Media *media = sd->getOutputMedia(i);
		if (media != nullptr && sd->getOutputChannelCount(media) > 0) {
			outputChannel = sd->getOutputChannel(media, 0);
			break;
		}
	}

	if (outputChannel != nullptr) {
		sd->onChannelVideoPresStats(outputChannel, &stats);
	}

	deliverOneFrameToSink(loop, demuxer, sinkListener);
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerVideoMediaStop()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias[0].get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		/* 1. Test stop() when mCurrentFrame and mCurrentMem are nullptr
		 */
		vm->stop();

		/* 2. Test stop() with non-null mCurrentMem */
		struct mbuf_mem *mem = nullptr;
		int ret = mbuf_mem_generic_new(64, &mem);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL(mem);

		vm->mCurrentMem = mem;
		vm->stop();
		CU_ASSERT_PTR_NULL(vm->mCurrentMem);
	}

	deliverOneFrameToSink(loop, demuxer, sinkListener);
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerVideoMediaOnFrameTimeout()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias[0].get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		Pdraw::Element::State originalState = sd->getState();
		uint64_t originalLastFrameTime = vm->mLastFrameReceiveTime;

		/* 1. State is not STARTED -> returns early */
		sd->setState(Pdraw::Element::State::CREATED);
		vm->onFrameTimeout();

		/* 2. State is STARTED, last frame time is recent (no timeout)
		 */
		sd->setState(Pdraw::Element::State::STARTED);
		struct timespec ts = {0, 0};
		time_get_monotonic(&ts);
		time_timespec_to_us(&ts, &vm->mLastFrameReceiveTime);
		vm->onFrameTimeout();

		/* 3. State is STARTED, last frame time is 0 -> timeout
		 * triggered */
		vm->mLastFrameReceiveTime = 0;
		vm->onFrameTimeout();

		/* Restore original state */
		sd->setState(originalState);
		vm->mLastFrameReceiveTime = originalLastFrameTime;
	}

	deliverOneFrameToSink(loop, demuxer, sinkListener);
	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerCodecInfoChangeOfOutputMedia()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias[0].get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		/* 1. Null parameters check */
		vm->codecInfoChangedCb(nullptr, nullptr, vm);

		/* 2. Unsupported codec check (not H264) */
		struct vstrm_codec_info wrongCodecInfo = {};
		wrongCodecInfo.codec = VSTRM_CODEC_UNKNOWN;
		vm->codecInfoChangedCb(nullptr, &wrongCodecInfo, vm);

		/* 3. Tearing down check */
		vm->mTearingDown = true;
		struct vstrm_codec_info h264Info = vm->mCodecInfo;
		h264Info.codec = VSTRM_CODEC_VIDEO_H264;
		h264Info.h264.width = 1920;
		h264Info.h264.height = 1080;
		vm->codecInfoChangedCb(nullptr, &h264Info, vm);
		vm->mTearingDown = false;

		/* 4. Trigger "change of output media" branch with changed
		 * resolution */
		vm->codecInfoChangedCb(nullptr, &h264Info, vm);
		CU_ASSERT_EQUAL(vm->mCodecInfo.h264.width, 1920);
	}

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* codecInfoChangedCb()'s mCurrentFrame/mCurrentMem cleanup (pdraw_demuxer_
 * stream.cpp:4252-4262) AND setupMedia()'s mTempQueue drain-on-setup (line
 * 3161-3167) -- both previously 0% covered.
 *
 * (a) mCurrentFrame/mCurrentMem non-null cannot be reached organically:
 * every exit path out of processFrame() (pdraw_demuxer_stream.cpp:3610-4028)
 * funnels through the "out:" label (line 4024), which unconditionally
 * unrefs and nulls both mCurrentFrame and mCurrentMem before returning --
 * there is no bare `return` between the point mCurrentFrame is allocated
 * (line 3677) and "out:". Since codec_info_changed and recv_frame are
 * invoked synchronously and sequentially by vstrm (single-threaded pomp
 * loop, no reentrancy), by the time codecInfoChangedCb() runs, any prior
 * processFrame() call has already completed and cleared both members. So
 * this test sets them directly (same established pattern as
 * testCxxStreamDemuxerVideoMediaOnFrameTimeout's `vm->mCurrentMem = mem;`
 * for VideoMedia::stop()), right before a REAL second SPS/PPS pair (over
 * RTP) organically fires codecInfoChangedCb() and finds them non-null.
 *
 * (b) mTempQueue being non-empty when setupMedia() runs again requires
 * mCodecInfoChanging to stay true for a while after being set. Two earlier
 * versions of this test tried to force that window with a SYNTHETIC direct
 * call to codecInfoChangedCb() (same pattern as
 * testCxxStreamDemuxerCodecInfoChangeOfOutputMedia): with a sink attached, a
 * real build showed ExternalCodedVideoSink::channelTeardown() unlinks the
 * media SYNCHRONOUSLY nested inside that same call (pdraw_external_coded_
 * video_sink.cpp:1035-1074, removeInputMedia() is not gated on any app-level
 * flush ack), so mCodecInfoChanging was already back to false before a frame
 * could be sent; dropping the sink fixed that, but the very next real RTP
 * frame sent right after the synthetic call then failed to be queued at all
 * (mTempQueue stayed at size 0 -- root cause not fully pinned down, but
 * vstrm's own internal SPS/PPS/frame-boundary state, entirely unaffected by
 * a direct call to pdraw's own codecInfoChangedCb(), was suspected).
 *
 * To sidestep that uncertainty entirely, this version reuses the exact
 * mechanics of testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow
 * (confirmed working by a real build) verbatim for the mCodecInfoChanging /
 * mTempQueue part: a REAL second SPS/PPS pair sent over RTP (different
 * resolution) organically fires codecInfoChangedCb() through vstrm's own
 * receiver, not a direct call. No sink is attached, so the per-channel
 * teardown loop inside codecInfoChangedCb() iterates zero times,
 * channelUnlink() is never triggered, and mCodecInfoChanging stays stuck
 * true afterward -- same rationale as that test, which also never attaches
 * a sink. A real RTP frame sent while mCodecInfoChanging is stuck true is
 * then queued into mTempQueue by the real recvFrameCb(). teardownMedia() +
 * setupMedia() are then called directly (same established pattern as
 * calling codecInfoChangedCb() directly elsewhere in this file) to
 * reproduce exactly what channelUnlink() would have done once all output
 * channels reached zero -- setupMedia() then drains mTempQueue for real. */
static void testCxxStreamDemuxerCodecInfoChangeDrainsTempQueue()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &demuxListener);
	CU_ASSERT_EQUAL_FATAL(demuxListener.mOpenStatus, 0);
	waitReadyToPlay(loop, demuxListener);

	int ret = demuxer->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&demuxListener]() { return demuxListener.mGotPlayResponse; },
		5000);
	CU_ASSERT_TRUE_FATAL(gotPlay);

	uint16_t rtpPort = demuxer->getSingleStreamLocalStreamPort();
	CU_ASSERT_FATAL(rtpPort > 0);

	/* First SPS/PPS: mVideoMedias (VideoMedia's own inner CodedVideoMedia
	 * list) is empty, so codecInfoChangedCb() takes "new output media"
	 * and calls setupMedia() directly. Deliberately no sink attached. */
	static constexpr uint32_t kSetupTs1 = 90000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  1,
			  kSetupTs1,
			  kH264TinySps,
			  sizeof(kH264TinySps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  2,
			  kSetupTs1,
			  kH264TinyPps,
			  sizeof(kH264TinyPps));
	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() { return !mediaListener.mAdded.empty(); },
		5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());
	auto *vm = sd->mVideoMedias[0].get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);
	CU_ASSERT_FALSE_FATAL(vm->mVideoMedias.empty());
	CU_ASSERT_FALSE(vm->mCodecInfoChanging);

	/* Force mCurrentFrame/mCurrentMem non-null, as if a codec info change
	 * had arrived while processFrame() was mid-flight (see long comment
	 * above for why this cannot happen organically), right before the
	 * real second SPS/PPS below organically fires codecInfoChangedCb(). */
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	vm->mCurrentMem = mem;

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	vm->mCurrentFrame = frame;

	/* Second SPS/PPS, a different resolution (this file's usual 1920x800
	 * kH264Sps/kH264Pps): mVideoMedias (VideoMedia's inner list) is now
	 * non-empty, so codecInfoChangedCb() takes the "change of output
	 * media" branch for real -- same mechanism as
	 * testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow. With no
	 * sink attached, its per-channel teardown loop iterates zero times,
	 * so nothing ever calls channelUnlink() to clear mCodecInfoChanging. */
	uint16_t seq = 3;
	static constexpr uint32_t kSetupTs2 = 93000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  kSetupTs2,
			  kH264Sps,
			  sizeof(kH264Sps));
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  kSetupTs2,
			  kH264Pps,
			  sizeof(kH264Pps));
	bool gotChanging =
		loop.pumpUntil([vm]() { return vm->mCodecInfoChanging; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotChanging);

	/* mCurrentFrame/mCurrentMem cleanup, previously 0% covered. */
	CU_ASSERT_PTR_NULL(vm->mCurrentMem);
	CU_ASSERT_PTR_NULL(vm->mCurrentFrame);
	CU_ASSERT_TRUE(vm->mTempQueue.empty());

	/* A real RTP frame sent now is queued into mTempQueue by the real
	 * recvFrameCb(), since mCodecInfoChanging is (stuck) true -- same
	 * mechanism as testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow.
	 * Not asserting an exact count of 1: vstrm's au_add_nalu() builds a
	 * "current frame" for ANY nalu type, including SPS/PPS themselves, so
	 * the second SPS/PPS above already has its own (degenerate, 0-slice)
	 * frame in progress inside vstrm; this packet's differing timestamp
	 * flushes THAT frame too ("missing end of frame", confirmed by a real
	 * build) into mTempQueue, in addition to this real IDR -- the queue
	 * ends up with 2 entries, not 1. What matters for line 3162-3167's
	 * coverage is only that the queue is non-empty before the drain. */
	static constexpr uint32_t kFrameTs = 96000u;
	sendRtpNaluPacket(rtpPort,
			  kTestRtspSsrc,
			  seq++,
			  kFrameTs,
			  kH264Idr,
			  sizeof(kH264Idr),
			  true /* marker */);
	pumpFor(loop, 200);
	CU_ASSERT_FALSE_FATAL(vm->mTempQueue.empty());

	/* Manually reproduce what channelUnlink() would do once all output
	 * channels reach zero -- teardownMedia() then setupMedia() again --
	 * since there is no channel here to trigger it organically.
	 * setupMedia() then drains mTempQueue for real (pdraw_demuxer_stream.
	 * cpp:3161-3167), previously 0% covered. */
	vm->teardownMedia();
	vm->mCodecInfoChanging = false;
	ret = vm->setupMedia();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(vm->mTempQueue.empty());

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerProcessSelectedMediasNotFound()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Push an invalid selected media with an out-of-bounds index (999) */
	struct pdraw_demuxer_media invalidMedia = {};
	invalidMedia.idx = 999;
	sd->mSelectedMedias.push_back(&invalidMedia);

	/* Call processSelectedMedias() -> triggers PDRAW_LOGE("failed to find
	 * the selected media in the list") and goto stop */
	int ret = sd->processSelectedMedias();
	CU_ASSERT_EQUAL(ret, 0);

	/* Remove invalid media before cleanup */
	sd->mSelectedMedias.pop_back();

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerRtspForcedTeardownSpecificTrack()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias[0].get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		std::string trackPath = sd->mShortContentBase + "/" +
					std::string(vm->getControlUrl());

		/* Trigger forced teardown on server side for a specific track
		 * path */
		int ret = rtsp_server_force_teardown(server.raw(),
						     server.mSessionId.c_str(),
						     trackPath.c_str(),
						     nullptr,
						     0);
		CU_ASSERT_EQUAL(ret, 0);

		bool gotError = loop.pumpUntil(
			[&listener]() {
				return listener.mGotUnrecoverableError;
			},
			5000);
		CU_ASSERT_TRUE(gotError);
	}

	closeStreamDemuxer(demuxer, loop, &listener);
}


static void testCxxStreamDemuxerSelectMediaError()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Call selectMedia with an invalid bitmask (media_id 30 does not exist)
	 */
	int ret = sd->selectMedia(1 << 30);
	CU_ASSERT_EQUAL(ret, -ENOENT);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerWatchdogTimer()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Command::PLAY timeout */
	sd->mPendingCmd = Pdraw::Demuxer::Command::PLAY;
	sd->onWatchdogTimer();
	CU_ASSERT_FALSE(sd->mPlayRespStatusArgs.empty());
	CU_ASSERT_EQUAL(sd->mPlayRespStatusArgs.front(), -ETIMEDOUT);

	/* 2. Command::PAUSE timeout */
	sd->mPendingCmd = Pdraw::Demuxer::Command::PAUSE;
	sd->onWatchdogTimer();
	CU_ASSERT_FALSE(sd->mPauseRespStatusArgs.empty());
	CU_ASSERT_EQUAL(sd->mPauseRespStatusArgs.front(), -ETIMEDOUT);

	/* 3. Command::PAUSE_NEXT timeout */
	sd->mPendingCmd = Pdraw::Demuxer::Command::PAUSE_NEXT;
	sd->onWatchdogTimer();
	CU_ASSERT_FALSE(sd->mPauseRespStatusArgs.empty());

	/* 4. Command::SEEK timeout */
	sd->mPendingCmd = Pdraw::Demuxer::Command::SEEK;
	sd->onWatchdogTimer();
	CU_ASSERT_FALSE(sd->mSeekRespStatusArgs.empty());
	CU_ASSERT_EQUAL(sd->mSeekRespStatusArgs.front(), -ETIMEDOUT);

	/* 5. Default/unsupported command (Command::NONE) timeout */
	sd->mPendingCmd = Pdraw::Demuxer::Command::NONE;
	sd->onWatchdogTimer();
	CU_ASSERT_EQUAL(sd->mPendingCmd, Pdraw::Demuxer::Command::NONE);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerGetChapterListReturnsEnosys()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Test via IPdraw::IDemuxer wrapper */
	struct pdraw_chapter *chapters = nullptr;
	size_t count = 0;
	int ret = demuxer->getChapterList(&chapters, &count);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* 2. Test directly on StreamDemuxer / Demuxer base class */
	ret = sd->getChapterList(&chapters, &count);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerElementGetWrapper()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Test Element::getWrapper() */
	CU_ASSERT_PTR_NOT_NULL(sd->getWrapper());
	CU_ASSERT_EQUAL(sd->getWrapper(),
			static_cast<Pdraw::ElementWrapper *>(wrapper));

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


static void testCxxStreamDemuxerElementSetClassNameString()
{
	MediaTrackingListener mediaListener;
	TestPompLoop loop;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener demuxListener;
	DrainingCodedVideoSinkListener sinkListener;
	IPdraw::IDemuxer *demuxer = nullptr;
	IPdraw::ICodedVideoSink *sink = openPlayAndAttachSink(session,
							      loop,
							      url,
							      &demuxListener,
							      &mediaListener,
							      &sinkListener,
							      &demuxer);
	auto sinkOwner = std::unique_ptr<IPdraw::ICodedVideoSink>(sink);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* Test Element::setClassName(const std::string &name) */
	std::string customClassName = "CustomDemuxerTest";
	sd->setClassName(customClassName);
	std::string expectedName =
		customClassName + "#" + std::to_string(sd->getId());
	CU_ASSERT_EQUAL(sd->getName(), expectedName);

	closeStreamDemuxer(demuxer, loop, &demuxListener);
}


/* Exercises RTSP over TCP transport (RTSP_LOWER_TRANSPORT_TCP) and the
 * StreamDemuxer::onRtspInterleavedDataCb callback
 * (pdraw_demuxer_stream.cpp:420-465) for RTP stream channel 0, RTCP control
 * channel 1, and dropping unknown channel 99. */
static void testCxxStreamDemuxerRtspTcpInterleaved()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	/* Configure VideoMediaNet for TCP transport and set channel ports:
	 * stream channel = 0, control channel = 1 */
	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);
	mediaNet->mLowerTransport = RTSP_LOWER_TRANSPORT_TCP;
	mediaNet->setRemoteStreamPort(0);
	mediaNet->setRemoteControlPort(1);

	/* Prepare sample RTP data packet for stream channel 0 */
	static const uint8_t rtpBuf[12 + 10] = {
		0x80, 96, 0, 1, 0, 0, 0, 10, 0x12, 0x34, 0x56, 0x78};
	Pdraw::StreamDemuxer::onRtspInterleavedDataCb(
		sd->mRtspClient, 0, rtpBuf, sizeof(rtpBuf), sd);

	/* Prepare sample RTCP control packet for control channel 1 */
	static const uint8_t rtcpBuf[8] = {
		0x80, 201, 0, 1, 0x12, 0x34, 0x56, 0x78};
	Pdraw::StreamDemuxer::onRtspInterleavedDataCb(
		sd->mRtspClient, 1, rtcpBuf, sizeof(rtcpBuf), sd);

	/* Test unknown channel 99 -> hits PDRAW_LOGW("dropping pkt from unknown
	 * channel (99)") */
	Pdraw::StreamDemuxer::onRtspInterleavedDataCb(
		sd->mRtspClient, 99, rtpBuf, sizeof(rtpBuf), sd);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises SDP session info, tool, and custom session attribute parsing
 * in StreamDemuxer::onNewSdp (pdraw_demuxer_stream.cpp:190-217). */
static void testCxxStreamDemuxerSdpSessionMetadata()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mWithSdpSessionAttrs = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);
	CU_ASSERT_FALSE(listener.mGotUnrecoverableError);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises play() with custom speed (2.0f) and relative seek() (delta)
 * in StreamDemuxer (pdraw_demuxer_stream.cpp:2378, 2636). */
static void testCxxStreamDemuxerRtspPlaySpeedAndRelativeSeek()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());
	server.mHasDuration = true;

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Play with 2.0f speed */
	int ret = demuxer->play(2.0f);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(sd->mSpeed, 2.0f);

	/* 2. Relative seek +5000000 us (5 sec) */
	listener.mGotPlayResponse = false;
	ret = demuxer->seek(5000000, false);
	CU_ASSERT_EQUAL(ret, 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises RTCP Goodbye reasons: RECONFIGURE, PHOTO_TRIGGER, and USER
 * in VideoMedia::goodbyeCb (pdraw_demuxer_stream.cpp:62-66, 96). */
static void testCxxStreamDemuxerRtcpGoodbyeCustomReasons()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *vm = sd->mVideoMedias.front().get();
	CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

	/* 1. Goodbye reason: configuration change -> sets mRestarting = true */
	vm->goodbyeCb(nullptr, "configuration change", vm);

	/* 2. Goodbye reason: photo trigger */
	vm->goodbyeCb(nullptr, "photo trigger", vm);

	/* 3. Goodbye reason: user disconnection */
	vm->goodbyeCb(nullptr, "user disconnection", vm);

	/* 4. Goodbye reason: unknown -> calls onUnrecoverableError */
	vm->goodbyeCb(nullptr, "unknown reason", vm);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises error status codes and timeout/aborted branches in
 * StreamDemuxer::onRtspPauseResp and onRtspTeardownResp
 * (pdraw_demuxer_stream.cpp:1080-1120, 1220-1250). */
static void testCxxStreamDemuxerRtspPauseAndTeardownError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Test onRtspPauseResp error status branches */
	sd->setPendingCommand(Pdraw::Demuxer::Command::PAUSE);
	sd->onRtspPauseResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_FAILED,
			    -ECONNRESET,
			    nullptr,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	sd->setPendingCommand(Pdraw::Demuxer::Command::PAUSE);
	sd->onRtspPauseResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_ABORTED,
			    0,
			    nullptr,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	sd->setPendingCommand(Pdraw::Demuxer::Command::PAUSE);
	sd->onRtspPauseResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_CANCELED,
			    0,
			    nullptr,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	sd->setPendingCommand(Pdraw::Demuxer::Command::PAUSE);
	sd->onRtspPauseResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			    0,
			    nullptr,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	/* "default" (unexpected/out-of-range status) branch. */
	sd->setPendingCommand(Pdraw::Demuxer::Command::PAUSE);
	sd->onRtspPauseResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    static_cast<enum rtsp_client_req_status>(99),
			    0,
			    nullptr,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	/* 2. Test onRtspTeardownResp error status & wrong session branches */
	sd->onRtspTeardownResp(sd->mRtspClient,
			       "wrong_session_id",
			       RTSP_CLIENT_REQ_STATUS_OK,
			       0,
			       nullptr,
			       0,
			       sd,
			       nullptr);

	sd->onRtspTeardownResp(sd->mRtspClient,
			       sd->mRtspSessionId.c_str(),
			       RTSP_CLIENT_REQ_STATUS_FAILED,
			       -EPIPE,
			       nullptr,
			       0,
			       sd,
			       nullptr);

	sd->onRtspTeardownResp(sd->mRtspClient,
			       sd->mRtspSessionId.c_str(),
			       RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			       0,
			       nullptr,
			       0,
			       sd,
			       nullptr);

	/* CANCELED ("no disconnection") and "default" branches: previously
	 * 0% covered (pdraw_demuxer_stream.cpp:1226-1241). */
	sd->onRtspTeardownResp(sd->mRtspClient,
			       sd->mRtspSessionId.c_str(),
			       RTSP_CLIENT_REQ_STATUS_CANCELED,
			       0,
			       nullptr,
			       0,
			       sd,
			       nullptr);

	sd->onRtspTeardownResp(sd->mRtspClient,
			       sd->mRtspSessionId.c_str(),
			       static_cast<enum rtsp_client_req_status>(99),
			       0,
			       nullptr,
			       0,
			       sd,
			       nullptr);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises the ABORTED/CANCELED/TIMEOUT/unexpected-status branches of
 * StreamDemuxer::onRtspOptionsResp, onRtspDescribeResp, onRtspSetupResp and
 * onRtspPlayResp (pdraw_demuxer_stream.cpp:585-623, 670-715, 756-807,
 * 918-982).
 *
 * Only RTSP_CLIENT_REQ_STATUS_OK (every other test) and, for describe/setup/
 * play, RTSP_CLIENT_REQ_STATUS_FAILED (testCxxStreamDemuxerDescribeError/
 * SetupError/PlayError, via a real server-side failure) were covered before
 * this test. A real rtsp_client never hands the demuxer ABORTED (disconnect
 * while a request is pending), CANCELED (explicit cancel) or an out-of-range
 * req_status in these signaling tests, so all four (including FAILED for
 * options, which had 0% coverage) are driven directly on an already-open
 * StreamDemuxer, mirroring testCxxStreamDemuxerRtspPauseAndTeardownError.
 *
 * NOTE: onRtspDescribeResp's "status == -EPERM -> retry" branch
 * (pdraw_demuxer_stream.cpp:706-710) is deliberately NOT exercised here:
 * it unconditionally falls through to onUnrecoverableError(res) even when
 * the retry fires (res is silently overwritten by sendDescribe()'s return
 * value), which -- on a *fresh* demuxer where mCalledOpenResp is still
 * false -- would call openResponse(0) (success) immediately after sending
 * the retried DESCRIBE, before its response is known. This looks like a
 * latent bug (see TEST_PROGRESS.md); reproducing it here would also leave a
 * real second DESCRIBE in flight against the test server past this test's
 * close(), so it is left for a dedicated follow-up test. */
static void testCxxStreamDemuxerRtspOptionsDescribeSetupPlayError()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. onRtspOptionsResp: entirely untested error path (OPTIONS always
	 * succeeds by the time the demuxer is open in every other test). All
	 * 4 named statuses plus an out-of-range one funnel into
	 * onUnrecoverableError(res); mCalledOpenResp is already true here, so
	 * only the FIRST call actually schedules the idle callback (Demuxer
	 * "report only the first error" guard in onUnrecoverableError) --
	 * later calls still execute (and cover) the switch itself. */
	sd->onRtspOptionsResp(sd->mRtspClient,
			      RTSP_CLIENT_REQ_STATUS_FAILED,
			      -EIO,
			      0,
			      nullptr,
			      0,
			      sd,
			      nullptr);
	sd->onRtspOptionsResp(sd->mRtspClient,
			      RTSP_CLIENT_REQ_STATUS_ABORTED,
			      -EPROTO,
			      0,
			      nullptr,
			      0,
			      sd,
			      nullptr);
	sd->onRtspOptionsResp(sd->mRtspClient,
			      RTSP_CLIENT_REQ_STATUS_CANCELED,
			      -EPROTO,
			      0,
			      nullptr,
			      0,
			      sd,
			      nullptr);
	sd->onRtspOptionsResp(sd->mRtspClient,
			      RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			      -ETIMEDOUT,
			      0,
			      nullptr,
			      0,
			      sd,
			      nullptr);
	sd->onRtspOptionsResp(sd->mRtspClient,
			      static_cast<enum rtsp_client_req_status>(99),
			      -EPROTO,
			      0,
			      nullptr,
			      0,
			      sd,
			      nullptr);

	bool gotUnrecoverable = loop.pumpUntil(
		[&listener]() { return listener.mGotUnrecoverableError; },
		2000);
	CU_ASSERT_TRUE(gotUnrecoverable);

	/* 2. onRtspDescribeResp: FAILED is exercised end-to-end by
	 * testCxxStreamDemuxerDescribeError; cover the remaining 3 cases.
	 * status is never -EPERM here -- see the retry note above. */
	sd->onRtspDescribeResp(sd->mRtspClient,
			       RTSP_CLIENT_REQ_STATUS_ABORTED,
			       -EPROTO,
			       nullptr,
			       nullptr,
			       0,
			       nullptr,
			       sd,
			       nullptr);
	sd->onRtspDescribeResp(sd->mRtspClient,
			       RTSP_CLIENT_REQ_STATUS_CANCELED,
			       -EPROTO,
			       nullptr,
			       nullptr,
			       0,
			       nullptr,
			       sd,
			       nullptr);
	sd->onRtspDescribeResp(sd->mRtspClient,
			       RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			       -ETIMEDOUT,
			       nullptr,
			       nullptr,
			       0,
			       nullptr,
			       sd,
			       nullptr);
	sd->onRtspDescribeResp(sd->mRtspClient,
			       static_cast<enum rtsp_client_req_status>(99),
			       -EPROTO,
			       nullptr,
			       nullptr,
			       0,
			       nullptr,
			       sd,
			       nullptr);

	/* 3. onRtspSetupResp: FAILED is exercised end-to-end by
	 * testCxxStreamDemuxerSetupError; cover the remaining 3 cases.
	 * req_userdata (media) is nullptr: the error branch only reads it
	 * for logging (`media ? ... : ""`), so this is safe. */
	sd->onRtspSetupResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_ABORTED,
			    -EPROTO,
			    0,
			    0,
			    0,
			    0,
			    nullptr,
			    0,
			    sd,
			    nullptr);
	sd->onRtspSetupResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_CANCELED,
			    -EPROTO,
			    0,
			    0,
			    0,
			    0,
			    nullptr,
			    0,
			    sd,
			    nullptr);
	sd->onRtspSetupResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			    -ETIMEDOUT,
			    0,
			    0,
			    0,
			    0,
			    nullptr,
			    0,
			    sd,
			    nullptr);
	sd->onRtspSetupResp(sd->mRtspClient,
			    sd->mRtspSessionId.c_str(),
			    static_cast<enum rtsp_client_req_status>(99),
			    -EPROTO,
			    0,
			    0,
			    0,
			    0,
			    nullptr,
			    0,
			    sd,
			    nullptr);

	/* 4. onRtspPlayResp: FAILED is exercised end-to-end by
	 * testCxxStreamDemuxerPlayError; cover the remaining 3 cases.
	 * mSeeking is false (no seek in progress), so each call takes the
	 * playResponse() branch; set a pending PLAY command first so
	 * Demuxer::callPlayResponse doesn't log a spurious "no pending
	 * command" warning.
	 *
	 * NOTE on the `status` values below: unlike onRtspOptionsResp/
	 * onRtspDescribeResp/onRtspSetupResp (which forward the switch's
	 * *translated* `res` to onUnrecoverableError()), onRtspPlayResp's
	 * error branch forwards the *raw* `status` argument straight to
	 * playResponse()/seekResponse()/pauseResponse()
	 * (pdraw_demuxer_stream.cpp: 968/972/977), not `res`. In real usage
	 * rtsp_client.c's request_complete() only fabricates a dummy response
	 * for ABORTED/CANCELED/TIMEOUT (no real response was ever received),
	 * with status_code left at 0 -- except for TIMEOUT, which is set to
	 * RTSP_STATUS_CODE_REQUEST_TIMEOUT -- before running it through
	 * rtsp_status_to_errno(). status_code=0 doesn't match
	 * RTSP_STATUS_CODE_OK's value (200) so it falls into that function's
	 * `default: return -EPROTO`. So a real ABORTED/CANCELED response
	 * always carries status=-EPROTO here (matching this test), and a real
	 * TIMEOUT one status=-ETIMEDOUT -- passing status=0 instead (as an
	 * earlier draft of this test did) would not reproduce a state librtsp
	 * ever actually produces. One side-effect of this raw-status forward:
	 * for CANCELED specifically, the ULOG_EVT log line above (using `res`)
	 * reports -ECANCELED while the application-visible playResponse()
	 * status is -EPROTO -- a minor logging/reporting mismatch, not a
	 * silent-success bug (see TEST_PROGRESS.md). */
	sd->setPendingCommand(Pdraw::Demuxer::Command::PLAY);
	sd->onRtspPlayResp(sd->mRtspClient,
			   sd->mRtspSessionId.c_str(),
			   RTSP_CLIENT_REQ_STATUS_ABORTED,
			   -EPROTO,
			   nullptr,
			   1.0f,
			   0,
			   0,
			   0,
			   0,
			   nullptr,
			   0,
			   sd,
			   nullptr);
	sd->setPendingCommand(Pdraw::Demuxer::Command::PLAY);
	sd->onRtspPlayResp(sd->mRtspClient,
			   sd->mRtspSessionId.c_str(),
			   RTSP_CLIENT_REQ_STATUS_CANCELED,
			   -EPROTO,
			   nullptr,
			   1.0f,
			   0,
			   0,
			   0,
			   0,
			   nullptr,
			   0,
			   sd,
			   nullptr);
	sd->setPendingCommand(Pdraw::Demuxer::Command::PLAY);
	sd->onRtspPlayResp(sd->mRtspClient,
			   sd->mRtspSessionId.c_str(),
			   RTSP_CLIENT_REQ_STATUS_TIMEOUT,
			   -ETIMEDOUT,
			   nullptr,
			   1.0f,
			   0,
			   0,
			   0,
			   0,
			   nullptr,
			   0,
			   sd,
			   nullptr);
	sd->setPendingCommand(Pdraw::Demuxer::Command::PLAY);
	sd->onRtspPlayResp(sd->mRtspClient,
			   sd->mRtspSessionId.c_str(),
			   static_cast<enum rtsp_client_req_status>(99),
			   -EPROTO,
			   nullptr,
			   1.0f,
			   0,
			   0,
			   0,
			   0,
			   nullptr,
			   0,
			   sd,
			   nullptr);

	bool gotPlayResp = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 2000);
	CU_ASSERT_TRUE(gotPlayResp);
	CU_ASSERT(listener.mPlayStatus != 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises frame-by-frame navigation (next() & previous()) in StreamDemuxer
 * for live streams (-ENOSYS), unpaused state (-EPROTO), and paused state
 * (pdraw_demuxer_stream.cpp:2466, 2555). */
static void testCxxStreamDemuxerFrameByFrameNextPrevious()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	/* 1. Live stream checks (-ENOSYS) */
	server.mHasDuration = false;
	StreamDemuxerListener listener;
	IPdraw::IDemuxer *liveDemuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	int ret = liveDemuxer->previousFrame();
	CU_ASSERT_EQUAL(ret, -ENOSYS);
	ret = liveDemuxer->nextFrame();
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	closeStreamDemuxer(liveDemuxer, loop, &listener);

	/* 2. Replay stream checks (paused success vs running -EPROTO) */
	server.mHasDuration = true;
	listener.mGotOpenResponse = false;
	IPdraw::IDemuxer *replayDemuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(replayDemuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	/* Clear mPendingSeek so seeking completes without waiting for RTP
	 * frames */
	sd->mVideoMedias.front()->mPendingSeek = false;
	(void)loop.pumpUntil(
		[sd]() {
			return sd->getPendingCommand() ==
			       Pdraw::Demuxer::Command::NONE;
		},
		2000);

	/* Pause demuxer so frame-by-frame is active */
	ret = replayDemuxer->pause();
	CU_ASSERT_EQUAL(ret, 0);
	sd->mVideoMedias.front()->mPendingSeek = false;
	(void)loop.pumpUntil(
		[sd]() {
			return sd->getPendingCommand() ==
			       Pdraw::Demuxer::Command::NONE;
		},
		2000);

	/* Clear seeking flag and pending command left by PAUSE_NEXT */
	sd->mSeeking = false;
	sd->clearPendingCommand();

	/* Set mPausePoint to 50ms to exercise start_usec < 0 branch in
	 * previous() */
	sd->mPausePoint = 50000;
	ret = replayDemuxer->previousFrame();
	CU_ASSERT_EQUAL(ret, 0);
	sd->mVideoMedias.front()->mPendingSeek = false;
	(void)loop.pumpUntil(
		[sd]() {
			return sd->getPendingCommand() ==
			       Pdraw::Demuxer::Command::NONE;
		},
		2000);

	sd->mSeeking = false;
	sd->clearPendingCommand();
	ret = replayDemuxer->nextFrame();
	CU_ASSERT_EQUAL(ret, 0);
	sd->mVideoMedias.front()->mPendingSeek = false;
	(void)loop.pumpUntil(
		[sd]() {
			return sd->getPendingCommand() ==
			       Pdraw::Demuxer::Command::NONE;
		},
		2000);

	/* Start playing so demuxer is no longer paused ->
	 * nextFrame()/previousFrame() return -EPROTO */
	ret = replayDemuxer->play(1.0f);
	CU_ASSERT_EQUAL(ret, 0);
	sd->mVideoMedias.front()->mPendingSeek = false;
	(void)loop.pumpUntil(
		[sd]() {
			return sd->getPendingCommand() ==
			       Pdraw::Demuxer::Command::NONE;
		},
		2000);

	ret = replayDemuxer->nextFrame();
	CU_ASSERT_EQUAL(ret, -EPROTO);
	ret = replayDemuxer->previousFrame();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	closeStreamDemuxer(replayDemuxer, loop, &listener);
}


/* Exercises RTSP setup and teardown request queue operations
 * (processRtspRequests, cleanupRtspRequests) in StreamDemuxer
 * (pdraw_demuxer_stream.cpp:220-275). */
static void testCxxStreamDemuxerRtspQueueOperationsAndTeardown()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	/* 1. Push mock setup and teardown requests into queues */
	auto *vm = sd->mVideoMedias.empty() ? nullptr
					    : sd->mVideoMedias.front().get();
	Pdraw::StreamDemuxer::SetupRequest setupReq(
		vm, "stream=0", RTSP_LOWER_TRANSPORT_UDP, 0, 0, nullptr, 0);
	sd->mSetupRequests.push(setupReq);
	sd->mSetupRequestsCount++;

	Pdraw::StreamDemuxer::TeardownRequest teardownReq(vm, "stream=0");
	sd->mTeardownRequests.push(teardownReq);

	CU_ASSERT_FALSE(sd->mSetupRequests.empty());
	CU_ASSERT_FALSE(sd->mTeardownRequests.empty());

	/* 2. Test cleanupRtspRequests clears both queues */
	sd->cleanupRtspRequests();
	CU_ASSERT_TRUE(sd->mSetupRequests.empty());
	CU_ASSERT_EQUAL(sd->mSetupRequestsCount, 0);
	CU_ASSERT_TRUE(sd->mTeardownRequests.empty());

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Exercises StreamDemuxer::getRtspStateStr for every RtspState value. */
static void testCxxStreamDemuxerGetRtspStateStrAllValues()
{
	using RtspState = Pdraw::StreamDemuxer::RtspState;

	struct TestParam {
		RtspState state;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 5> validCases{
		{{RtspState::DISCONNECTED, "DISCONNECTED"},
		 {RtspState::CONNECTED, "CONNECTED"},
		 {RtspState::OPTIONS_DONE, "OPTIONS_DONE"},
		 {RtspState::DESCRIBE_DONE, "DESCRIBE_DONE"},
		 {RtspState::SETUP_DONE, "SETUP_DONE"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(
			Pdraw::StreamDemuxer::getRtspStateStr(test.state),
			test.expectedStr);
	}

	RtspState invalidState = static_cast<RtspState>(-1);
	CU_ASSERT_PTR_NULL(Pdraw::StreamDemuxer::getRtspStateStr(invalidState));
}


static void testCxxStreamDemuxerGetRtspStateStrInvalidReturnsNull()
{
	CU_ASSERT_PTR_NULL(Pdraw::StreamDemuxer::getRtspStateStr(
		static_cast<Pdraw::StreamDemuxer::RtspState>(999)));
}


/* VideoMedia::processCtrlPkt/processDataPkt(nullptr) must both return
 * -EINVAL (pdraw_demuxer_stream.cpp) without dereferencing the packet. */
static void testCxxStreamDemuxerVideoMediaProcessPktNullReturnsEinval()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias.front().get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		CU_ASSERT_EQUAL(vm->processCtrlPkt(nullptr), -EINVAL);
		CU_ASSERT_EQUAL(vm->processDataPkt(nullptr), -EINVAL);
	}

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* VideoMedia::resync(), called after onPlayComplete()/pause(), must set
 * mWaitForSync and reset mRecoveryFrameCount to 0
 * (pdraw_demuxer_stream.cpp). */
static void testCxxStreamDemuxerVideoMediaResyncSetsWaitForSync()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	if (!sd->mVideoMedias.empty()) {
		auto *vm = sd->mVideoMedias.front().get();
		CU_ASSERT_PTR_NOT_NULL_FATAL(vm);

		vm->onPlayComplete();
		vm->pause();
		vm->resync();
		CU_ASSERT_TRUE(vm->mWaitForSync);
		CU_ASSERT_EQUAL(vm->mRecoveryFrameCount, 0);
	}

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers: sendCtrl() null-pkt early return (EINVAL guard, l.249). */
static void testCxxStreamDemuxerVideoMediaSendCtrlNullPkt()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);

	CU_ASSERT_EQUAL(mediaNet->sendCtrl(nullptr, nullptr), -EINVAL);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers the nullptr-guard branches of the 3 H.264 SEI callbacks in
 * StreamDemuxer::VideoMedia (pdraw_demuxer_stream.cpp:4082-4158), which real
 * RTP/H264 delivery never hits: testCxxStreamDemuxerH264SeiCallbacks and
 * testCxxStreamDemuxerH264PicTimingSei above already cover the "success" paths
 * (mCurrentFrame set, sei/ctx valid) by feeding crafted SEI NAL units
 * through the real jitter buffer + h264_reader; the reader itself never
 * calls back with null self/buf/sei, and mCurrentFrame is always non-null
 * while a frame's NAL units are being parsed. Same technique as
 * testCxxDemuxerCodedVideoSeiCallbacksNullGuardsCoverage
 * (test_api_demuxer.cpp) for the record demuxer's equivalent callbacks:
 * call the static callbacks directly with crafted/null arguments instead of
 * the full RTSP/RTP pipeline. struct h264_ctx is opaque outside of libh264
 * (forward-declared only), so a fake non-null pointer is used where a
 * non-null ctx is required -- it is never dereferenced because
 * mCurrentFrame is left nullptr, which makes the guard right before any
 * ctx/sei dereference return first. Unlike the record demuxer's
 * h264UserDataSeiCb, h264RecoveryPointSeiCb has no "sei == nullptr" guard at
 * all (pdraw_demuxer_stream.cpp:4149-4152): only self and mCurrentFrame are
 * checked. */
static void testCxxStreamDemuxerVideoMediaSeiCallbacksNullGuardsCoverage()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);
	/* No play()/RTP delivery: no NAL was ever parsed, so mCurrentFrame is
	 * still at its default value. */
	CU_ASSERT_PTR_NULL(mediaNet->mCurrentFrame);

	uint8_t buf[4] = {0, 0, 0, 1};
	struct h264_sei_user_data_unregistered userData = {};
	struct h264_sei_pic_timing picTiming = {};
	struct h264_sei_recovery_point recoveryPoint = {};
	auto *fakeCtx = reinterpret_cast<struct h264_ctx *>(0x1);

	/* h264UserDataSeiCb: self / buf / len / sei / mCurrentFrame guards. */
	Pdraw::StreamDemuxer::VideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), &userData, nullptr);
	Pdraw::StreamDemuxer::VideoMedia::h264UserDataSeiCb(
		nullptr, nullptr, sizeof(buf), &userData, mediaNet);
	Pdraw::StreamDemuxer::VideoMedia::h264UserDataSeiCb(
		nullptr, buf, 0, &userData, mediaNet);
	Pdraw::StreamDemuxer::VideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), nullptr, mediaNet);
	Pdraw::StreamDemuxer::VideoMedia::h264UserDataSeiCb(
		nullptr, buf, sizeof(buf), &userData, mediaNet);

	/* h264PicTimingSeiCb: self / ctx / sei / mCurrentFrame guards. */
	Pdraw::StreamDemuxer::VideoMedia::h264PicTimingSeiCb(
		fakeCtx, buf, sizeof(buf), &picTiming, nullptr);
	Pdraw::StreamDemuxer::VideoMedia::h264PicTimingSeiCb(
		nullptr, buf, sizeof(buf), &picTiming, mediaNet);
	Pdraw::StreamDemuxer::VideoMedia::h264PicTimingSeiCb(
		fakeCtx, buf, sizeof(buf), nullptr, mediaNet);
	Pdraw::StreamDemuxer::VideoMedia::h264PicTimingSeiCb(
		fakeCtx, buf, sizeof(buf), &picTiming, mediaNet);

	/* h264RecoveryPointSeiCb: self / mCurrentFrame guards only -- no
	 * sei == nullptr check in this callback. */
	Pdraw::StreamDemuxer::VideoMedia::h264RecoveryPointSeiCb(
		fakeCtx, buf, sizeof(buf), &recoveryPoint, nullptr);
	Pdraw::StreamDemuxer::VideoMedia::h264RecoveryPointSeiCb(
		fakeCtx, buf, sizeof(buf), &recoveryPoint, mediaNet);

	/* None of the calls above should have reached past their guards. */
	CU_ASSERT_PTR_NULL(mediaNet->mCurrentFrame);
	CU_ASSERT_EQUAL(mediaNet->mCurrentFrameCaptureTs, 0u);
	CU_PASS("no crash");

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers: sendCtrl() TCP path → rtsp_client_send_interleaved (l.251-262). */
static void testCxxStreamDemuxerVideoMediaSendCtrlTcpInterleaved()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);

	/* Switch to TCP transport; channel index == mRemoteStreamPort (0) */
	mediaNet->mLowerTransport = RTSP_LOWER_TRANSPORT_TCP;
	mediaNet->mRemoteStreamPort = 0;

	/* Re-use the class helper to create a valid tpkt_packet
	 * (mRxBufLen was set to DEFAULT_RX_BUFFER_SIZE by createSockets()). */
	struct tpkt_packet *pkt = mediaNet->newRxPkt();
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);

	/* sendCtrl on the TCP path calls rtsp_client_send_interleaved; the
	 * interleaved channel was not negotiated so the call may fail, but
	 * sendCtrl always returns 0 on this branch. */
	CU_ASSERT_EQUAL(mediaNet->sendCtrl(nullptr, pkt), 0);
	tpkt_unref(pkt);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers: getLocalStreamPort/ControlPort/RemoteStreamPort/RemoteControlPort
 * TCP early-return paths (l.297-298, l.312-313, l.325-327, l.339-341). */
static void testCxxStreamDemuxerGetPortsTcpMode()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);

	/* Switch to TCP so the port getters return mLocal/RemotePort directly
	 */
	mediaNet->mLowerTransport = RTSP_LOWER_TRANSPORT_TCP;
	mediaNet->mLocalStreamPort = 10;
	mediaNet->mLocalControlPort = 11;
	mediaNet->mRemoteStreamPort = 20;
	mediaNet->mRemoteControlPort = 21;

	CU_ASSERT_EQUAL(mediaNet->getLocalStreamPort(), 10);
	CU_ASSERT_EQUAL(mediaNet->getLocalControlPort(), 11);
	CU_ASSERT_EQUAL(mediaNet->getRemoteStreamPort(), 20);
	CU_ASSERT_EQUAL(mediaNet->getRemoteControlPort(), 21);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers: getLocalStreamPort/ControlPort/RemoteStreamPort/RemoteControlPort
 * null-socket EPROTO paths (l.300-303, l.313-316, l.328-331, l.342-345).
 * After the sockets are manually destroyed, destroyReceiver() may attempt
 * to send an RTCP BYE; sendCtrl() returns -EINVAL on null mControlSock,
 * which is safe (the BYE is simply not transmitted). */
static void testCxxStreamDemuxerGetPortsNullSocket()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);

	tskt_socket_destroy(mediaNet->mStreamSock);
	mediaNet->mStreamSock = nullptr;
	CU_ASSERT_EQUAL(mediaNet->getLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(mediaNet->getRemoteStreamPort(), 0);

	tskt_socket_destroy(mediaNet->mControlSock);
	mediaNet->mControlSock = nullptr;
	CU_ASSERT_EQUAL(mediaNet->getLocalControlPort(), 0);
	CU_ASSERT_EQUAL(mediaNet->getRemoteControlPort(), 0);

	closeStreamDemuxer(demuxer, loop, &listener);
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


/* Covers: createSockets() stream-socket allocation failure (l.435-438) and
 * the error: cleanup label (l.478-489). Allowing zero more fds to be
 * allocated makes the very first tskt_socket_new() fail with EMFILE. */
static void testCxxStreamDemuxerCreateSocketsStreamFails()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);
	CU_ASSERT_FALSE_FATAL(sd->mVideoMedias.empty());

	auto *mediaNet = dynamic_cast<Pdraw::StreamDemuxerNet::VideoMediaNet *>(
		sd->mVideoMedias.front().get());
	CU_ASSERT_PTR_NOT_NULL_FATAL(mediaNet);

	/* Tear down existing sockets so createSockets() starts from scratch.
	 * The rx packet is also reset so newRxPkt() runs on the fresh call. */
	tskt_socket_destroy(mediaNet->mStreamSock);
	mediaNet->mStreamSock = nullptr;
	tskt_socket_destroy(mediaNet->mControlSock);
	mediaNet->mControlSock = nullptr;
	tpkt_unref(mediaNet->mRxPkt);
	mediaNet->mRxPkt = nullptr;

	struct rlimit origLimit = capFdsAllowingNMore(0);

	int res = mediaNet->createSockets();
	CU_ASSERT_NOT_EQUAL(res, 0);
	CU_ASSERT_PTR_NULL(mediaNet->mStreamSock);
	CU_ASSERT_PTR_NULL(mediaNet->mControlSock);

	CU_ASSERT_EQUAL(setrlimit(RLIMIT_NOFILE, &origLimit), 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers the ports-based (no-URL, SessionProtocol::NONE) code path end-to-end:
 *  - start() creates a VideoMedia and binds UDP sockets on the default ports
 *    (55004/55005), so getSingleStreamLocalStream/ControlPort() return non-zero
 *  - readyToPlay fires synchronously from start() in NONE mode
 *  - play() dispatches playResponse without waiting for any RTSP exchange
 *    (tests the internalPlay() NONE branch added to fix multicast)
 *  - pause() dispatches pauseResponse likewise
 *    (tests the internalPause() NONE branch)
 *  - close() tears down cleanly via setTearingDown() on all medias
 *    (tests the stop() fix: was calling stopRtpAvp() on front only) */
static void testCxxStreamDemuxerSingleStreamPortsBased()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	StreamDemuxerListener listener;
	struct pdraw_demuxer_params params = {};

	IPdraw::IDemuxer *demuxer = nullptr;
	int ret = session->createDemuxer(
		"", 0, 0, "", 0, 0, &params, &listener, &demuxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(demuxer);

	bool gotOpen = loop.pumpUntil(
		[&listener]() { return listener.mGotOpenResponse; }, 5000);
	CU_ASSERT_TRUE(gotOpen);
	CU_ASSERT_EQUAL(listener.mOpenStatus, 0);

	/* start() created a VideoMedia and bound sockets
	 * -> ports are non-zero */
	CU_ASSERT_NOT_EQUAL(demuxer->getSingleStreamLocalStreamPort(), 0);
	CU_ASSERT_NOT_EQUAL(demuxer->getSingleStreamLocalControlPort(), 0);

	/* readyToPlay(true) is called synchronously from start()
	 * in NONE mode */
	bool gotReady = loop.pumpUntil(
		[&listener]() { return listener.mGotReadyToPlay; }, 5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(listener.mReady);

	/* play() must dispatch playResponse without any RTSP exchange */
	ret = demuxer->play();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotPlay = loop.pumpUntil(
		[&listener]() { return listener.mGotPlayResponse; }, 5000);
	CU_ASSERT_TRUE(gotPlay);
	CU_ASSERT_EQUAL(listener.mPlayStatus, 0);

	/* pause() must dispatch pauseResponse likewise */
	ret = demuxer->pause();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotPause = loop.pumpUntil(
		[&listener]() { return listener.mGotPauseResponse; }, 5000);
	CU_ASSERT_TRUE(gotPause);
	CU_ASSERT_EQUAL(listener.mPauseStatus, 0);

	closeStreamDemuxer(demuxer, loop, &listener);
}


/* Covers: getSingleStreamLocalStreamPort/ControlPort "not started" path
 * (l.117-120) by temporarily storing a non-STARTED element state. */
static void testCxxStreamDemuxerSingleStreamPortsNotStarted()
{
	TestPompLoop loop;
	TestSession testSession(&loop);
	IPdraw *session = testSession.get();

	TestRtspServer server(loop.raw(), kTestRtspPort, 1);
	CU_ASSERT_TRUE_FATAL(server.isStarted());

	std::string url = std::string("rtsp://127.0.0.1:") +
			  std::to_string(kTestRtspPort) + "/" + kTestRtspPath;

	StreamDemuxerListener listener;
	IPdraw::IDemuxer *demuxer =
		openStreamDemuxer(session, loop, url, &listener);
	CU_ASSERT_EQUAL_FATAL(listener.mOpenStatus, 0);

	Pdraw::DemuxerWrapper *wrapper =
		static_cast<Pdraw::DemuxerWrapper *>(demuxer);
	Pdraw::StreamDemuxer *sd =
		static_cast<Pdraw::StreamDemuxer *>(wrapper->getDemuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(sd);

	Element::State savedState = sd->mState.load();
	sd->mState.store(Element::State::CREATED);
	CU_ASSERT_EQUAL(demuxer->getSingleStreamLocalStreamPort(), 0);
	CU_ASSERT_EQUAL(demuxer->getSingleStreamLocalControlPort(), 0);
	sd->mState.store(savedState);

	closeStreamDemuxer(demuxer, loop, &listener);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_demuxer_stream_net[] = {
	{FN("testCxxStreamDemuxerGetRtspStateStrAllValues"),
	 testCxxStreamDemuxerGetRtspStateStrAllValues},
	{FN("testCxxStreamDemuxerGetRtspStateStrInvalidReturnsNull"),
	 testCxxStreamDemuxerGetRtspStateStrInvalidReturnsNull},
	{FN("testCxxStreamDemuxerVideoMediaProcessPktNullReturnsEinval"),
	 testCxxStreamDemuxerVideoMediaProcessPktNullReturnsEinval},
	{FN("testCxxStreamDemuxerVideoMediaResyncSetsWaitForSync"),
	 testCxxStreamDemuxerVideoMediaResyncSetsWaitForSync},
	{FN("testCxxStreamDemuxerRtspQueueOperationsAndTeardown"),
	 testCxxStreamDemuxerRtspQueueOperationsAndTeardown},
	{FN("testCxxStreamDemuxerFrameByFrameNextPrevious"),
	 testCxxStreamDemuxerFrameByFrameNextPrevious},
	{FN("testCxxStreamDemuxerRtspPlaySpeedAndRelativeSeek"),
	 testCxxStreamDemuxerRtspPlaySpeedAndRelativeSeek},
	{FN("testCxxStreamDemuxerRtcpGoodbyeCustomReasons"),
	 testCxxStreamDemuxerRtcpGoodbyeCustomReasons},
	{FN("testCxxStreamDemuxerRtspPauseAndTeardownError"),
	 testCxxStreamDemuxerRtspPauseAndTeardownError},
	{FN("testCxxStreamDemuxerRtspOptionsDescribeSetupPlayError"),
	 testCxxStreamDemuxerRtspOptionsDescribeSetupPlayError},
	{FN("testCxxStreamDemuxerRtspTcpInterleaved"),
	 testCxxStreamDemuxerRtspTcpInterleaved},
	{FN("testCxxStreamDemuxerSdpSessionMetadata"),
	 testCxxStreamDemuxerSdpSessionMetadata},
	{FN("testCxxStreamDemuxerRtspSignalingLifecycle"),
	 testCxxStreamDemuxerRtspSignalingLifecycle},
	{FN("testCxxStreamDemuxerTwoMediaSelectFirst"),
	 testCxxStreamDemuxerTwoMediaSelectFirst},
	{FN("testCxxStreamDemuxerRtpCodecInfo"),
	 testCxxStreamDemuxerRtpCodecInfo},
	{FN("testCxxStreamDemuxerDescribeError"),
	 testCxxStreamDemuxerDescribeError},
	{FN("testCxxStreamDemuxerSetupError"), testCxxStreamDemuxerSetupError},
	{FN("testCxxStreamDemuxerPlayError"), testCxxStreamDemuxerPlayError},
	{FN("testCxxStreamDemuxerSeek"), testCxxStreamDemuxerSeek},
	{FN("testCxxStreamDemuxerEmptySdp"), testCxxStreamDemuxerEmptySdp},
	{FN("testCxxStreamDemuxerSelectMediaCancelled"),
	 testCxxStreamDemuxerSelectMediaCancelled},
	{FN("testCxxStreamDemuxerSelectMediaRejected"),
	 testCxxStreamDemuxerSelectMediaRejected},
	{FN("testCxxStreamDemuxerIsPaused"), testCxxStreamDemuxerIsPaused},
	{FN("testCxxStreamDemuxerDuration"), testCxxStreamDemuxerDuration},
	{FN("testCxxStreamDemuxerSelectMediaSwitch"),
	 testCxxStreamDemuxerSelectMediaSwitch},
	{FN("testCxxStreamDemuxerForcedTeardown"),
	 testCxxStreamDemuxerForcedTeardown},
	{FN("testCxxStreamDemuxerPreviousNextGuards"),
	 testCxxStreamDemuxerPreviousNextGuards},
	{FN("testCxxStreamDemuxerSeekToGuards"),
	 testCxxStreamDemuxerSeekToGuards},
	{FN("testCxxStreamDemuxerSeekGuards"), testCxxStreamDemuxerSeekGuards},
	{FN("testCxxStreamDemuxerPlayGuards"), testCxxStreamDemuxerPlayGuards},
	{FN("testCxxStreamDemuxerPlayPendingCommandGuard"),
	 testCxxStreamDemuxerPlayPendingCommandGuard},
	{FN("testCxxStreamDemuxerPlaySyncDispatchError"),
	 testCxxStreamDemuxerPlaySyncDispatchError},
	{FN("testCxxStreamDemuxerGoodbyeUnrecoverableError"),
	 testCxxStreamDemuxerGoodbyeUnrecoverableError},
	{FN("testCxxStreamDemuxerChannelFlushedAndUnlinkOnClose"),
	 testCxxStreamDemuxerChannelFlushedAndUnlinkOnClose},
	{FN("testCxxStreamDemuxerChannelDrainedOnPause"),
	 testCxxStreamDemuxerChannelDrainedOnPause},
	{FN("testCxxStreamDemuxerChannelResync"),
	 testCxxStreamDemuxerChannelResync},
	{FN("testCxxStreamDemuxerEventReconfigureSetsRestartFlag"),
	 testCxxStreamDemuxerEventReconfigureSetsRestartFlag},
	{FN("testCxxStreamDemuxerProcessFrameDeliversToSink"),
	 testCxxStreamDemuxerProcessFrameDeliversToSink},
	{FN("testCxxStreamDemuxerSessionMetaUpdateCascadesToDecoder"),
	 testCxxStreamDemuxerSessionMetaUpdateCascadesToDecoder},
	{FN("testCxxStreamDemuxerH264SeiCallbacks"),
	 testCxxStreamDemuxerH264SeiCallbacks},
	{FN("testCxxStreamDemuxerH264PicTimingSei"),
	 testCxxStreamDemuxerH264PicTimingSei},
	{FN("testCxxStreamDemuxerProcessFrameNaluAndSyncBranches"),
	 testCxxStreamDemuxerProcessFrameNaluAndSyncBranches},
	{FN("testCxxStreamDemuxerProcessFrameBufferTooSmall"),
	 testCxxStreamDemuxerProcessFrameBufferTooSmall},
	{FN("testCxxStreamDemuxerProcessFrameOutputMemoryExhausted"),
	 testCxxStreamDemuxerProcessFrameOutputMemoryExhausted},
	{FN("testCxxStreamDemuxerProcessFrameFormatCopyFailure"),
	 testCxxStreamDemuxerProcessFrameFormatCopyFailure},
	{FN("testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow"),
	 testCxxStreamDemuxerCodecInfoChangingTempQueueOverflow},
	{FN("testCxxStreamDemuxerCodecInfoUnchangedResyncOnly"),
	 testCxxStreamDemuxerCodecInfoUnchangedResyncOnly},
	{FN("testCxxStreamDemuxerNextFrameCompletion"),
	 testCxxStreamDemuxerNextFrameCompletion},
	{FN("testCxxStreamDemuxerFrameTimeout"),
	 testCxxStreamDemuxerFrameTimeout},
	{FN("testCxxStreamDemuxerEventResolutionChange"),
	 testCxxStreamDemuxerEventResolutionChange},
	{FN("testCxxStreamDemuxerEventFramerateChange"),
	 testCxxStreamDemuxerEventFramerateChange},
	{FN("testCxxStreamDemuxerEventPhotoTrigger"),
	 testCxxStreamDemuxerEventPhotoTrigger},
	{FN("testCxxStreamDemuxerIdleEndOfRangeNotification"),
	 testCxxStreamDemuxerIdleEndOfRangeNotification},
	{FN("testCxxStreamDemuxerRangeTimer"), testCxxStreamDemuxerRangeTimer},
	{FN("testCxxStreamDemuxerRtspAnnounce"),
	 testCxxStreamDemuxerRtspAnnounce},
	{FN("testCxxStreamDemuxerOnChannelVideoPresStats"),
	 testCxxStreamDemuxerOnChannelVideoPresStats},
	{FN("testCxxStreamDemuxerVideoMediaStop"),
	 testCxxStreamDemuxerVideoMediaStop},
	{FN("testCxxStreamDemuxerVideoMediaOnFrameTimeout"),
	 testCxxStreamDemuxerVideoMediaOnFrameTimeout},
	{FN("testCxxStreamDemuxerCodecInfoChangeOfOutputMedia"),
	 testCxxStreamDemuxerCodecInfoChangeOfOutputMedia},
	{FN("testCxxStreamDemuxerCodecInfoChangeDrainsTempQueue"),
	 testCxxStreamDemuxerCodecInfoChangeDrainsTempQueue},
	{FN("testCxxStreamDemuxerProcessSelectedMediasNotFound"),
	 testCxxStreamDemuxerProcessSelectedMediasNotFound},
	{FN("testCxxStreamDemuxerRtspForcedTeardownSpecificTrack"),
	 testCxxStreamDemuxerRtspForcedTeardownSpecificTrack},
	{FN("testCxxStreamDemuxerSelectMediaError"),
	 testCxxStreamDemuxerSelectMediaError},
	{FN("testCxxStreamDemuxerWatchdogTimer"),
	 testCxxStreamDemuxerWatchdogTimer},
	{FN("testCxxStreamDemuxerGetChapterListReturnsEnosys"),
	 testCxxStreamDemuxerGetChapterListReturnsEnosys},
	{FN("testCxxStreamDemuxerElementGetWrapper"),
	 testCxxStreamDemuxerElementGetWrapper},
	{FN("testCxxStreamDemuxerElementSetClassNameString"),
	 testCxxStreamDemuxerElementSetClassNameString},
	{FN("testCxxStreamDemuxerVideoMediaSendCtrlNullPkt"),
	 testCxxStreamDemuxerVideoMediaSendCtrlNullPkt},
	{FN("testCxxStreamDemuxerVideoMediaSeiCallbacksNullGuardsCoverage"),
	 testCxxStreamDemuxerVideoMediaSeiCallbacksNullGuardsCoverage},
	{FN("testCxxStreamDemuxerVideoMediaSendCtrlTcpInterleaved"),
	 testCxxStreamDemuxerVideoMediaSendCtrlTcpInterleaved},
	{FN("testCxxStreamDemuxerGetPortsTcpMode"),
	 testCxxStreamDemuxerGetPortsTcpMode},
	{FN("testCxxStreamDemuxerGetPortsNullSocket"),
	 testCxxStreamDemuxerGetPortsNullSocket},
	{FN("testCxxStreamDemuxerCreateSocketsStreamFails"),
	 testCxxStreamDemuxerCreateSocketsStreamFails},
	{FN("testCxxStreamDemuxerSingleStreamPortsBased"),
	 testCxxStreamDemuxerSingleStreamPortsBased},
	{FN("testCxxStreamDemuxerSingleStreamPortsNotStarted"),
	 testCxxStreamDemuxerSingleStreamPortsNotStarted},
	CU_TEST_INFO_NULL,
};
