/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
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

#ifndef _PDRAW_DEMUXER_STREAM_HPP_
#define _PDRAW_DEMUXER_STREAM_HPP_

#include "pdraw_demuxer.hpp"

#include <string>
#include <vector>

#include <h264/h264.h>
#include <libpomp.h>
#include <libsdp.h>
#include <rtsp/client.h>
#include <video-streaming/vstrm.h>

namespace Pdraw {


#define DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT 55004
#define DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT 55005


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(Session *session,
		      Element::Listener *elementListener,
		      Source::Listener *sourceListener,
		      Demuxer::Listener *demuxerListener);

	virtual ~StreamDemuxer(void);

	int start(void);

	int stop(void);

	int play(float speed = 1.0f);

	bool isPaused(void);

	int previous(void);

	int next(void);

	int seek(int64_t delta, bool exact = false);

	int seekTo(uint64_t timestamp, bool exact = false);

	uint64_t getDuration(void);

	uint64_t getCurrentTime(void);

protected:
	enum SessionProtocol {
		NONE = 0,
		RTSP,
	};

	int startRtsp(const std::string &url);

	int createReceiver(void);

	int destroyReceiver(void);

	virtual int startRtpAvp(void) = 0;

	virtual int stopRtpAvp(void) = 0;

	virtual int sendCtrl(struct vstrm_receiver *stream,
			     struct pomp_buffer *buf) = 0;

	virtual int prepareSetup(uint16_t *streamPort,
				 uint16_t *controlPort,
				 enum rtsp_lower_transport *lowerTransport) = 0;

	std::string mUrl;
	std::string mRtspAddr;
	std::string mRtspPath;
	const char *mContentBase;
	std::string mLocalAddr;
	uint16_t mLocalStreamPort;
	uint16_t mLocalControlPort;
	std::string mRemoteAddr;
	uint16_t mRemoteStreamPort;
	uint16_t mRemoteControlPort;
	struct vstrm_receiver *mReceiver;
	SessionProtocol mSessionProtocol;

private:
	enum RtspState {
		DISCONNECTED = 0,
		CONNECTED,
		OPTIONS_DONE,
		DESCRIBE_DONE,
		SETUP_DONE,
	};

	static const char *getRtspStateStr(StreamDemuxer::RtspState val);

	static void sessionMetadataFromSdp(struct sdp_session *session,
					   struct sdp_media *media,
					   struct vmeta_session *meta);

	int internalPlay(float speed);

	int internalPause(void);

	int flush(void);

	int setupInputMedia(void);

	void completeTeardown(void);

	void onChannelFlushed(Channel *channel);

	void onChannelUnlink(Channel *channel);

	void onChannelResync(Channel *channel);

	int processFrame(struct vstrm_frame *frame);

	int idrFrameSync(void);

	void onNewSdp(const char *content_base, const char *sdp);

	static void onRtspSocketCreated(int fd, void *userdata);

	static void onRtspConnectionState(struct rtsp_client *client,
					  enum rtsp_client_conn_state state,
					  void *userdata);

	static void onRtspSessionRemoved(struct rtsp_client *client,
					 const char *session_id,
					 int status,
					 void *userdata);

	static void onRtspOptionsResp(struct rtsp_client *client,
				      enum rtsp_client_req_status req_status,
				      int status,
				      uint32_t methods,
				      void *userdata,
				      void *req_userdata);

	static void onRtspDescribeResp(struct rtsp_client *client,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const char *content_base,
				       const char *sdp,
				       void *userdata,
				       void *req_userdata);

	static void onRtspSetupResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    uint16_t server_stream_port,
				    uint16_t server_control_port,
				    int ssrc_valid,
				    uint32_t ssrc,
				    void *userdata,
				    void *req_userdata);

	static void onRtspPlayResp(struct rtsp_client *client,
				   const char *session_id,
				   enum rtsp_client_req_status req_status,
				   int status,
				   const struct rtsp_range *range,
				   float scale,
				   int seq_valid,
				   uint16_t seq,
				   int rtptime_valid,
				   uint32_t rtptime,
				   void *userdata,
				   void *req_userdata);

	static void onRtspPauseResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    const struct rtsp_range *range,
				    void *userdata,
				    void *req_userdata);

	static void onRtspTeardownResp(struct rtsp_client *client,
				       const char *session_id,
				       enum rtsp_client_req_status req_status,
				       int status,
				       void *userdata,
				       void *req_userdata);

	static void onRtspAnnounce(struct rtsp_client *client,
				   const char *content_base,
				   const char *sdp,
				   void *userdata);

	static void idleRtspDisconnect(void *userdata);

	static void idleStop(void *userdata);

	static void
	h264UserDataSeiCb(struct h264_ctx *ctx,
			  const uint8_t *buf,
			  size_t len,
			  const struct h264_sei_user_data_unregistered *sei,
			  void *userdata);

	static void h264PicTimingSeiCb(struct h264_ctx *ctx,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_sei_pic_timing *sei,
				       void *userdata);

	static void
	h264RecoveryPointSeiCb(struct h264_ctx *ctx,
			       const uint8_t *buf,
			       size_t len,
			       const struct h264_sei_recovery_point *sei,
			       void *userdata);

	static int sendCtrlCb(struct vstrm_receiver *stream,
			      struct pomp_buffer *buf,
			      void *userdata);

	static void codecInfoChangedCb(struct vstrm_receiver *stream,
				       const struct vstrm_codec_info *info,
				       void *userdata);

	static void recvFrameCb(struct vstrm_receiver *stream,
				struct vstrm_frame *frame,
				void *userdata);

	static void
	sessionMetadataPeerChangedCb(struct vstrm_receiver *stream,
				     const struct vmeta_session *meta,
				     void *userdata);

	static void goodbyeCb(struct vstrm_receiver *stream,
			      const char *reason,
			      void *userdata);

	static void frameTimeoutCb(struct pomp_timer *timer, void *userdata);

	static void rangeTimerCb(struct pomp_timer *timer, void *userdata);

	VideoMedia *mVideoMedia;
	bool mFirstFrame;
	bool mChannelsReadyForStop;
	bool mNetworkReadyForStop;
	struct vbuf_buffer *mCurrentBuffer;
	uint64_t mLastFrameReceiveTime;
	uint64_t mCurrentBufferCaptureTs;
	bool mFlushing;
	bool mIdrFrameSyncPending;
	bool mWaitForSync;
	int mRecoveryFrameCount;
	RtspState mRtspState;
	bool mTearingDown;
	struct pomp_timer *mTimer;
	struct pomp_timer *mRangeTimer;
	struct rtsp_client *mRtspClient;
	const char *mRtspSessionId;
	struct h264_reader *mH264Reader;
	struct vstrm_codec_info mCodecInfo;
	bool mWaitForCodecInfo;
	bool mCodecInfoChanging;
	uint32_t mSsrc;
	bool mRunning;
	uint64_t mStartTime;
	uint64_t mDuration;
	uint64_t mCurrentTime;
	uint64_t mPausePoint;
	int64_t mNtpToNptOffset;
	float mSpeed;
	bool mFrameByFrame;
	bool mEndOfRangeNotified;
	bool mSeeking;
	static const struct rtsp_client_cbs mRtspClientCbs;
	static const struct vstrm_receiver_cbs mReceiverCbs;
	static const struct h264_ctx_cbs mH264Cbs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_STREAM_HPP_ */
