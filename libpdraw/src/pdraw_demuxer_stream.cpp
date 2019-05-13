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

#include "pdraw_demuxer_stream.hpp"
#include "pdraw_session.hpp"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <string>

#include <futils/futils.h>
#define ULOG_TAG pdraw_dmxstrm
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_dmxstrm);

namespace Pdraw {


#define DEMUXER_STREAM_TIMER_INTERVAL_MS 50
#define DEMUXER_STREAM_FRAME_TIMEOUT_US 2000000

#define DEMUXER_STREAM_GOODBYE_REASON_USER "user disconnection"
#define DEMUXER_STREAM_GOODBYE_REASON_RECONFIGURE "configuration change"
#define DEMUXER_STREAM_GOODBYE_REASON_PHOTO_TRIGGER "photo trigger"


const struct rtsp_client_cbs StreamDemuxer::mRtspClientCbs = {
	.socket_cb = &StreamDemuxer::onRtspSocketCreated,
	.connection_state = &StreamDemuxer::onRtspConnectionState,
	.session_removed = &StreamDemuxer::onRtspSessionRemoved,
	.options_resp = &StreamDemuxer::onRtspOptionsResp,
	.describe_resp = &StreamDemuxer::onRtspDescribeResp,
	.setup_resp = &StreamDemuxer::onRtspSetupResp,
	.play_resp = &StreamDemuxer::onRtspPlayResp,
	.pause_resp = &StreamDemuxer::onRtspPauseResp,
	.teardown_resp = &StreamDemuxer::onRtspTeardownResp,
	.announce = &StreamDemuxer::onRtspAnnounce,
};


const struct vstrm_receiver_cbs StreamDemuxer::mReceiverCbs = {
	.send_ctrl = &StreamDemuxer::sendCtrlCb,
	.codec_info_changed = &StreamDemuxer::codecInfoChangedCb,
	.recv_frame = &StreamDemuxer::recvFrameCb,
	.recv_rtp_pkt = NULL,
	.session_metadata_peer_changed =
		&StreamDemuxer::sessionMetadataPeerChangedCb,
	.goodbye = &StreamDemuxer::goodbyeCb,
};


const struct h264_ctx_cbs StreamDemuxer::mH264Cbs = {
	.au_end = NULL,
	.nalu_begin = NULL,
	.nalu_end = NULL,
	.slice = NULL,
	.slice_data_begin = NULL,
	.slice_data_end = NULL,
	.slice_data_mb = NULL,
	.sps = NULL,
	.pps = NULL,
	.aud = NULL,
	.sei = NULL,
	.sei_buffering_period = NULL,
	.sei_pic_timing = &StreamDemuxer::h264PicTimingSeiCb,
	.sei_pan_scan_rect = NULL,
	.sei_filler_payload = NULL,
	.sei_user_data_registered = NULL,
	.sei_user_data_unregistered = &StreamDemuxer::h264UserDataSeiCb,
	.sei_recovery_point = &StreamDemuxer::h264RecoveryPointSeiCb,
};


StreamDemuxer::StreamDemuxer(Session *session,
			     Element::Listener *elementListener,
			     Source::Listener *sourceListener,
			     Demuxer::Listener *demuxerListener) :
		Demuxer(session,
			elementListener,
			sourceListener,
			demuxerListener)
{
	int ret;

	Element::mName = "StreamDemuxer";
	Source::mName = "StreamDemuxer";
	mFirstFrame = true;
	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
	mVideoMedia = NULL;
	mLocalStreamPort = 0;
	mLocalControlPort = 0;
	mRemoteStreamPort = 0;
	mRemoteControlPort = 0;
	mSessionProtocol = NONE;
	mRtspState = DISCONNECTED;
	mRtspClient = NULL;
	mTearingDown = false;
	mTimer = NULL;
	mRangeTimer = NULL;
	mRtspSessionId = NULL;
	mReceiver = NULL;
	mCurrentBuffer = NULL;
	mCurrentBufferCaptureTs = 0;
	mLastFrameReceiveTime = 0;
	mFlushing = false;
	mIdrFrameSyncPending = false;
	mWaitForSync = false;
	mRecoveryFrameCount = 0;
	mStartTime = 0;
	mCurrentTime = 0;
	mDuration = 0;
	mPausePoint = 0;
	mNtpToNptOffset = 0;
	mRunning = false;
	mSpeed = 1.0;
	mFrameByFrame = true;
	mEndOfRangeNotified = false;
	mSsrc = 0;
	mWaitForCodecInfo = true;
	mCodecInfoChanging = false;
	memset(&mCodecInfo, 0, sizeof(mCodecInfo));
	mContentBase = NULL;
	mH264Reader = NULL;
	mSeeking = false;

	std::string userAgent;
	SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();

	/* Create the RTSP client */
	selfMeta->getSoftwareVersion(&userAgent);
	ret = rtsp_client_new(mSession->getLoop(),
			      userAgent.c_str(),
			      &mRtspClientCbs,
			      this,
			      &mRtspClient);
	if (ret < 0) {
		ULOG_ERRNO("rtsp_client_new", -ret);
		goto err;
	}

	/* Create the frame timer */
	mTimer = pomp_timer_new(mSession->getLoop(), &frameTimeoutCb, this);
	if (mTimer == NULL) {
		ULOGE("pomp_timer_new failed");
		goto err;
	}

	/* Create the H.264 reader */
	ret = h264_reader_new(&mH264Cbs, this, &mH264Reader);
	if (ret < 0) {
		ULOG_ERRNO("h264_reader_new", -ret);
		goto err;
	}

	/* Create the end of range timer */
	mRangeTimer = pomp_timer_new(mSession->getLoop(), &rangeTimerCb, this);
	if (mRangeTimer == NULL) {
		ULOGE("pomp_timer_new failed");
		goto err;
	}

	return;

err:
	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}
	if (mRangeTimer != NULL) {
		ret = pomp_timer_clear(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mRangeTimer = NULL;
	}
	if (mH264Reader != NULL) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = NULL;
	}
	if (mRtspClient) {
		ret = rtsp_client_destroy(mRtspClient);
		if (ret < 0)
			ULOG_ERRNO("rtsp_client_destroy", -ret);
		mRtspClient = NULL;
	}
}


StreamDemuxer::~StreamDemuxer(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		ULOGW("demuxer is still running");

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	if (mRtspClient != NULL) {
		ret = rtsp_client_destroy(mRtspClient);
		if (ret < 0)
			ULOG_ERRNO("rtsp_client_destroy", -ret);
		mRtspClient = NULL;
	}

	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}

	if (mRangeTimer != NULL) {
		ret = pomp_timer_clear(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mRangeTimer = NULL;
	}

	if (mH264Reader != NULL) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = NULL;
	}

	if (mVideoMedia != NULL)
		ULOGW("output media was not properly removed");

	ret = pomp_loop_idle_remove(
		mSession->getLoop(), &idleRtspDisconnect, this);
	if (ret < 0)
		ULOG_ERRNO("pomp_loop_idle_remove", -ret);

	ret = pomp_loop_idle_remove(mSession->getLoop(), &idleStop, this);
	if (ret < 0)
		ULOG_ERRNO("pomp_loop_idle_remove", -ret);

	free((void *)mContentBase);
	mContentBase = NULL;

	free((void *)mRtspSessionId);
	mRtspSessionId = NULL;
}


void StreamDemuxer::sessionMetadataFromSdp(struct sdp_session *session,
					   struct sdp_media *media,
					   struct vmeta_session *meta)
{
	int err;
	struct sdp_attr *attr = NULL;

	if (session->session_name != NULL) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_NAME,
			session->session_name,
			NULL,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	if (session->session_info != NULL) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_INFO,
			session->session_info,
			NULL,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	if (session->tool != NULL) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_TOOL,
			session->tool,
			NULL,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	list_walk_entry_forward(&session->attrs, attr, node)
	{
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			attr->value,
			attr->key,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	list_walk_entry_forward(&media->attrs, attr, node)
	{
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
			attr->value,
			attr->key,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
}


void StreamDemuxer::onRtspSocketCreated(int fd, void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	self->mSession->socketCreated(fd);
}


void StreamDemuxer::onRtspConnectionState(struct rtsp_client *client,
					  enum rtsp_client_conn_state state,
					  void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	switch (state) {
	case RTSP_CLIENT_CONN_STATE_DISCONNECTED:
		ULOGI("RTSP disconnected");
		self->mRtspState = DISCONNECTED;
		ULOGD("RTSP state change to %s",
		      getRtspStateStr(self->mRtspState));
		pomp_timer_clear(self->mTimer);
		self->mRunning = false;
		self->mNetworkReadyForStop = true;

		/* If channels are also ready, set the state to stopped */
		if (self->mChannelsReadyForStop) {
			self->setState(STOPPED);
			self->mChannelsReadyForStop = false;
			self->mNetworkReadyForStop = false;
		}
		break;
	case RTSP_CLIENT_CONN_STATE_CONNECTED:
		ULOGI("RTSP connected");
		self->mRtspState = CONNECTED;
		ULOGD("RTSP state change to %s",
		      getRtspStateStr(self->mRtspState));

		res = rtsp_client_options(self->mRtspClient,
					  NULL,
					  RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (res < 0) {
			ULOG_ERRNO("rtsp_client_options", -res);
		}
		break;
	default:
		ULOGW("unhandled RTSP connection state (%d)", state);
		break;
	}
}


void StreamDemuxer::onRtspSessionRemoved(struct rtsp_client *client,
					 const char *session_id,
					 int status,
					 void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		ULOGD("wrong session removed (%s, expected %s)",
		      session_id,
		      self->mRtspSessionId);
		return;
	}

	ULOGI("RTSP session %s removed, status=%d(%s)",
	      session_id,
	      -status,
	      strerror(-status));

	res = pomp_loop_idle_add(self->mSession->getLoop(), &idleStop, self);
	if (res < 0)
		ULOG_ERRNO("pomp_loop_idle_add", -res);
}


void StreamDemuxer::onRtspOptionsResp(struct rtsp_client *client,
				      enum rtsp_client_req_status req_status,
				      int status,
				      uint32_t methods,
				      void *userdata,
				      void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			ULOGE("RTSP options request failed (%d: %s)",
			      status,
			      strerror(-status));
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			ULOGE("RTSP options request aborted");
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			ULOGE("RTSP options request canceled");
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			ULOGE("timeout on RTSP options request");
			break;
		default:
			/* This should not happen */
			ULOGE("unexpected status on options request: %d",
			      req_status);
			break;
		}
		res = pomp_loop_idle_add(
			self->mSession->getLoop(), &idleStop, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		return;
	}

	self->mRtspState = OPTIONS_DONE;
	ULOGD("RTSP state change to %s", getRtspStateStr(self->mRtspState));

	res = rtsp_client_describe(self->mRtspClient,
				   self->mRtspPath.c_str(),
				   NULL,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0)
		ULOG_ERRNO("rtsp_client_describe", -res);
}


void StreamDemuxer::onRtspDescribeResp(struct rtsp_client *client,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const char *content_base,
				       const char *sdp,
				       void *userdata,
				       void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			ULOGE("RTSP describe request failed (%d: %s)",
			      status,
			      strerror(-status));
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			ULOGE("RTSP describe request aborted");
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			ULOGE("RTSP describe request canceled");
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			ULOGE("timeout on RTSP describe request");
			break;
		default:
			/* This should not happen */
			ULOGE("unexpected status on describe request: %d",
			      req_status);
			break;
		}
		res = pomp_loop_idle_add(
			self->mSession->getLoop(), &idleStop, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		return;
	}

	self->onNewSdp(content_base, sdp);
}


void StreamDemuxer::onRtspSetupResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    uint16_t server_stream_port,
				    uint16_t server_control_port,
				    int ssrc_valid,
				    uint32_t ssrc,
				    void *userdata,
				    void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			ULOGE("RTSP setup request failed (%d: %s)",
			      status,
			      strerror(-status));
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			ULOGE("RTSP setup request aborted");
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			ULOGE("RTSP setup request canceled");
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			ULOGE("timeout on RTSP setup request");
			break;
		default:
			/* This should not happen */
			ULOGE("unexpected status on setup request: %d",
			      req_status);
			break;
		}
		res = pomp_loop_idle_add(
			self->mSession->getLoop(), &idleStop, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		return;
	}

	self->mSsrc = (ssrc_valid) ? ssrc : 0;
	self->mRemoteStreamPort = server_stream_port;
	self->mRemoteControlPort = server_control_port;

	free((void *)self->mRtspSessionId);
	self->mRtspSessionId = xstrdup(session_id);

	self->mRtspState = SETUP_DONE;
	ULOGD("RTSP state change to %s", getRtspStateStr(self->mRtspState));

	res = self->startRtpAvp();
	if (res < 0) {
		ULOG_ERRNO("startRtpAvp", -res);
		return;
	}

	/* If the setup is a success, we always need to be in STARTED state */
	self->setState(STARTED);
	self->mDemuxerListener->onReadyToPlay(self, true);
}


void StreamDemuxer::onRtspPlayResp(struct rtsp_client *client,
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
				   void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	uint64_t start, ntptime = 0;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			ULOGE("RTSP play request failed (%d: %s)",
			      status,
			      strerror(-status));
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			ULOGE("RTSP play request aborted");
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			ULOGE("RTSP play request canceled");
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			/* TODO: strategy on RTSP play timeout to be
			 * defined. Retry RTSP play? */
			ULOGE("timeout on RTSP play request");
			break;
		default:
			/* This should not happen */
			ULOGE("unexpected status on play request: %d",
			      req_status);
			break;
		}
		if (self->mSeeking)
			self->mDemuxerListener->seekResp(
				self, status, self->mCurrentTime, self->mSpeed);
		else
			self->mDemuxerListener->playResp(
				self, status, self->mCurrentTime, self->mSpeed);
		self->mSeeking = false;
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		ULOGE("RTSP play response for a wrong session"
		      " (%s instead of %s)",
		      session_id,
		      self->mRtspSessionId);
		return;
	}

	self->mSpeed = (scale != 0.) ? scale : 1.0;
	if (rtptime_valid) {
		ntptime = vstrm_receiver_get_ntp_from_rtp_ts(self->mReceiver,
							     rtptime);
	}
	start = (uint64_t)range->start.npt.sec * 1000000 +
		(uint64_t)range->start.npt.usec;
	self->mNtpToNptOffset =
		(int64_t)(ntptime * self->mSpeed) - (int64_t)start;
	self->mPausePoint = (uint64_t)range->stop.npt.sec * 1000000 +
			    (uint64_t)range->stop.npt.usec;
	if (self->mSeeking)
		self->mDemuxerListener->seekResp(self, 0, start, self->mSpeed);
	else
		self->mDemuxerListener->playResp(self, 0, start, self->mSpeed);
	self->mSeeking = false;
}


void StreamDemuxer::onRtspPauseResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    const struct rtsp_range *range,
				    void *userdata,
				    void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			ULOGE("RTSP pause request failed (%d: %s)",
			      status,
			      strerror(-status));
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			ULOGE("RTSP pause request aborted");
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			ULOGE("RTSP pause request canceled");
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			/* TODO: strategy on RTSP pause timeout to be
			 * defined. Retry RTSP pause? */
			ULOGE("timeout on RTSP pause request");
			break;
		default:
			/* This should not happen */
			ULOGE("unexpected status on pause request: %d",
			      req_status);
			break;
		}
		self->mDemuxerListener->pauseResp(
			self, status, self->mCurrentTime);
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		ULOGE("RTSP pause response for a wrong session"
		      " (%s instead of %s)",
		      session_id,
		      self->mRtspSessionId);
		return;
	}

	self->mPausePoint = (uint64_t)range->start.npt.sec * 1000000 +
			    (uint64_t)range->start.npt.usec;
	self->mDemuxerListener->pauseResp(self, 0, self->mPausePoint);
}


void StreamDemuxer::onRtspTeardownResp(struct rtsp_client *client,
				       const char *session_id,
				       enum rtsp_client_req_status req_status,
				       int status,
				       void *userdata,
				       void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	self->mTearingDown = false;

	switch (req_status) {
	default:
	case RTSP_CLIENT_REQ_STATUS_OK:
		break;
	case RTSP_CLIENT_REQ_STATUS_ABORTED:
		/* Teardown effective by disconnection */
		ULOGW("RTSP teardown request aborted");
		break;
	case RTSP_CLIENT_REQ_STATUS_FAILED:
		/* Force disconnection anyway */
		ULOGE("RTSP teardown request failed (%d: %s)",
		      status,
		      strerror(-status));
		break;
	case RTSP_CLIENT_REQ_STATUS_CANCELED:
		/* No disconnection */
		ULOGI("RTSP teardown request canceled");
		return;
	case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
		/* Force disconnection anyway */
		ULOGE("timeout on RTSP teardown request");
		break;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		ULOGE("RTSP teardown response for a wrong session"
		      " (%s instead of %s)",
		      session_id,
		      self->mRtspSessionId);
		return;
	}

	free((void *)self->mRtspSessionId);
	self->mRtspSessionId = NULL;

	self->mRtspState = OPTIONS_DONE;
	ULOGD("RTSP state change to %s", getRtspStateStr(self->mRtspState));

	res = pomp_loop_idle_add(
		self->mSession->getLoop(), &idleRtspDisconnect, self);
	if (res < 0)
		ULOG_ERRNO("pomp_loop_idle_add", -res);
}


void StreamDemuxer::onRtspAnnounce(struct rtsp_client *client,
				   const char *content_base,
				   const char *sdp,
				   void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (!self->mContentBase ||
	    strcmp(self->mContentBase, content_base) != 0)
		return;

	self->onNewSdp(content_base, sdp);
}


void StreamDemuxer::idleRtspDisconnect(void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	res = rtsp_client_disconnect(self->mRtspClient);
	if (res < 0) {
		ULOG_ERRNO("rtsp_client_disconnect", -res);
		return;
	}
}


void StreamDemuxer::idleStop(void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	res = self->stop();
	if (res < 0) {
		ULOG_ERRNO("stop", -res);
		return;
	}
}


int StreamDemuxer::startRtsp(const std::string &url)
{
	int res;
	std::string::size_type n;

	if (mRtspClient == NULL) {
		ULOG_ERRNO("invalid rtsp client", EPROTO);
		return -EPROTO;
	}

	/* Check that we actually have something after 'rtsp://' */
	if (url.length() < 8)
		return -EINVAL;

	/* Find first '/' in url */
	n = url.find("/", 7);
	if (n == std::string::npos)
		return -EINVAL;
	mRtspAddr = url.substr(0, n);
	mRtspPath = url.substr(n + 1);

	mSessionProtocol = RTSP;
	res = rtsp_client_connect(mRtspClient, mRtspAddr.c_str());
	if (res < 0) {
		ULOG_ERRNO("rtsp_client_connect", -res);
		return res;
	}

	return 0;
}


int StreamDemuxer::start(void)
{
	int res;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		ULOGE("demuxer is not created");
		return -EPROTO;
	}
	setState(STARTING);

	if (mUrl.length() > 0) {
		std::string ext = mUrl.substr(mUrl.length() - 4, 4);
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

		if (mUrl.substr(0, 7) == "rtsp://") {
			res = startRtsp(mUrl);
			if (res < 0) {
				ULOG_ERRNO("startRtsp", -res);
				return res;
			}
		} else {
			ULOGE("unsupported URL");
			return -ENOSYS;
		}

		setState(STARTED);
	} else {
		res = startRtpAvp();
		if (res < 0) {
			ULOG_ERRNO("startRtpAvp", -res);
			return res;
		}
		mDemuxerListener->onReadyToPlay(this, true);

		setState(STARTED);
	}

	return 0;
}


int StreamDemuxer::stop(void)
{
	int ret = 0;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED && mState != STARTING) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}
	setState(STOPPING);
	mDemuxerListener->onReadyToPlay(this, false);

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
	if (mSessionProtocol == RTSP) {
		if (mRtspState == SETUP_DONE) {
			mTearingDown = true;
			ret = rtsp_client_teardown(
				mRtspClient,
				mRtspSessionId,
				NULL,
				RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
			if (ret < 0) {
				ULOG_ERRNO("rtsp_client_teardown", -ret);
				/* disconnect directly the rstp */
				ret = rtsp_client_disconnect(mRtspClient);
				if (ret < 0) {
					ULOG_ERRNO("rtsp_client_disconnect",
						   -ret);
					return ret;
				}
			}
		} else {
			ret = rtsp_client_disconnect(mRtspClient);
			if (ret < 0) {
				ULOG_ERRNO("rtsp_client_disconnect", -ret);
				return ret;
			}
		}
	} else {
		pomp_timer_clear(mTimer);
		mRunning = false;
		mNetworkReadyForStop = true;
		stopRtpAvp();
	}

	Source::lock();

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);

	Source::unlock();

	if (mNetworkReadyForStop && mChannelsReadyForStop) {
		setState(STOPPED);
		mChannelsReadyForStop = false;
		mNetworkReadyForStop = false;
	}

	return ret;
}


int StreamDemuxer::flush(void)
{
	int ret;
	unsigned int outputChannelCount = 0, i;
	Channel *channel;

	if ((mState != STARTED) && (mState != STOPPING)) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	Source::lock();

	mFlushing = true;
	mIdrFrameSyncPending = true;
	mWaitForSync = true;
	mRecoveryFrameCount = 0;

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	if (mVideoMedia != NULL)
		outputChannelCount = getOutputChannelCount(mVideoMedia);

	/* Flush the output channels */
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(mVideoMedia, i);
		if (channel == NULL) {
			ULOGW("failed to get channel at index %d", i);
			continue;
		}
		ret = channel->flush();
		if (ret < 0)
			ULOG_ERRNO("channel->flush", -ret);
	}

	if (outputChannelCount <= 0)
		mChannelsReadyForStop = true;

	Source::unlock();

	return 0;
}


void StreamDemuxer::onChannelFlushed(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::lock();

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		Source::unlock();
		return;
	}
	ULOGD("'%s': channel flushed media id=%d type=%s (channel key=%p)",
	      Source::mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* TODO: do that in a completeFlush() function
	 * when all channels are flushed */
	mFlushing = false;

	if (mIdrFrameSyncPending && (idrFrameSync() == 0))
		mIdrFrameSyncPending = false;

	if (mState == STOPPING)
		channel->teardown();

	Source::unlock();
}


void StreamDemuxer::onChannelUnlink(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel->getKey());
	if (ret < 0)
		ULOG_ERRNO("removeOutputChannel", -ret);

	completeTeardown();
}


void StreamDemuxer::completeTeardown(void)
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();

	if (mVideoMedia == NULL) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mVideoMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener)
		Source::mListener->onOutputMediaRemoved(this, mVideoMedia);
	ret = removeOutputPort(mVideoMedia);
	if (ret < 0) {
		ULOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mVideoMedia;
		mVideoMedia = NULL;
	}

	Source::unlock();

exit:
	if (mState == STOPPING) {
		mChannelsReadyForStop = true;

		/* If network is also ready, set the state to stopped */
		if (mNetworkReadyForStop && mChannelsReadyForStop) {
			setState(STOPPED);
			mChannelsReadyForStop = false;
			mNetworkReadyForStop = false;
		}
	} else if (mCodecInfoChanging) {
		ULOGI("new output media");
		mCodecInfoChanging = false;
		ret = setupInputMedia();
		if (ret < 0) {
			ULOG_ERRNO("setupInputMedia", -ret);
			return;
		}
	}
}


void StreamDemuxer::onChannelResync(Channel *channel)
{
	int ret;

	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::lock();
	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		Source::unlock();
		return;
	}
	ULOGD("'%s': channel resync media id=%d type=%s (channel key=%p)",
	      Source::mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	/* New synchronization is needed */
	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}
	mWaitForSync = true;
	mRecoveryFrameCount = 0;
	mIdrFrameSyncPending = true;
	if (idrFrameSync() == 0)
		mIdrFrameSyncPending = false;
	Source::unlock();
}


void StreamDemuxer::onNewSdp(const char *content_base, const char *sdp)
{
	int res = 0;
	struct sdp_session *session = NULL;
	const char *mediaUrl = NULL;
	char *remoteAddr = NULL;
	uint16_t streamPort, controlPort;
	enum rtsp_lower_transport lowerTransport;
	struct pdraw_demuxer_media *medias = NULL;
	size_t mediasCount = 0, mediaIdx = 0, i;
	bool found = false;
	SessionPeerMetadata *peerMeta = mSession->getPeerMetadata();
	struct vmeta_session meta;
	memset(&meta, 0, sizeof(meta));

	if (mState == STOPPING) {
		ULOGI("new SDP while stopping, ignore it");
		return;
	}

	res = sdp_description_read(sdp, &session);
	if (res < 0) {
		ULOG_ERRNO("sdp_description_read", -res);
		return;
	}
	if (session->deletion)
		ULOGW("sdp refers to a no longer existing session");

	if ((!mContentBase) && (content_base))
		mContentBase = strdup(content_base);

	mLocalStreamPort = 0;
	mLocalControlPort = 0;
	struct sdp_media *media = NULL;
	list_walk_entry_forward(&session->medias, media, node)
	{
		if ((media->type == SDP_MEDIA_TYPE_VIDEO) &&
		    (media->control_url)) {
			if ((media->h264_fmtp.valid) &&
			    (media->h264_fmtp.sps != NULL) &&
			    (media->h264_fmtp.pps != NULL)) {
				memset(&mCodecInfo, 0, sizeof(mCodecInfo));
				mCodecInfo.codec = VSTRM_CODEC_VIDEO_H264;
				if (media->h264_fmtp.sps_size <=
				    sizeof(mCodecInfo.h264.sps)) {
					memcpy(mCodecInfo.h264.sps,
					       media->h264_fmtp.sps,
					       media->h264_fmtp.sps_size);
					mCodecInfo.h264.spslen =
						media->h264_fmtp.sps_size;
				}
				if (media->h264_fmtp.pps_size <=
				    sizeof(mCodecInfo.h264.pps)) {
					memcpy(mCodecInfo.h264.pps,
					       media->h264_fmtp.pps,
					       media->h264_fmtp.pps_size);
					mCodecInfo.h264.ppslen =
						media->h264_fmtp.pps_size;
				}
			}
			struct pdraw_demuxer_media *tmp =
				(struct pdraw_demuxer_media *)realloc(
					medias,
					(mediasCount + 1) * sizeof(*medias));
			if (!tmp)
				goto exit;
			mediasCount++;
			medias = tmp;
			struct pdraw_demuxer_media *current =
				&medias[mediasCount - 1];
			memset(current, 0, sizeof(*current));
			current->media_id = mediasCount;
			current->idx = mediasCount - 1;
			current->name = xstrdup(media->media_title);
			if (current->name == NULL)
				current->name = xstrdup(media->control_url);
			current->uri = xstrdup(media->control_url);
			current->is_default = mediasCount == 1;
		}
	}
	if (mediasCount == 0) {
		/* Empty SDP */
		ULOGI("Empty SDP, no stream");
		/* An empty SDP means that both the server & the URL are good,
		 * but there is currently no streams. In this case, we can set
		 * the demuxer as STARTED here, and wait for an ANNOUCE to get
		 * the media. Otherwise, wait for the setup response before
		 * setting the state */
		setState(STARTED);
		mDemuxerListener->onReadyToPlay(this, false);
		/* Stop the demuxer if a previous setup was done */
		if (mRtspState == SETUP_DONE)
			stopRtpAvp();
		mRtspState = OPTIONS_DONE;
		ULOGD("RTSP state change to %s", getRtspStateStr(mRtspState));
		goto exit;
	}

	res = mDemuxerListener->selectDemuxerMedia(this, medias, mediasCount);
	if (res < 0 && res != -ENOSYS) {
		ULOGE("application failed to select a video media");
		/* Selecting a wrong video media is an error, stop the demuxer
		 * to either report an open response, or an unrecoverable error
		 * to the application */
		res = pomp_loop_idle_add(mSession->getLoop(), &idleStop, this);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		goto exit;
	} else if (res == 0 || res == -ENOSYS) {
		ULOGI("auto-selecting media %d (%s)",
		      medias[0].media_id,
		      medias[0].name);
		mediaUrl = medias[0].uri;
		mediaIdx = medias[0].idx;
	} else {
		size_t j;
		for (j = 0; j < mediasCount; j++) {
			if (medias[j].media_id == res) {
				mediaUrl = medias[j].uri;
				mediaIdx = medias[j].idx;
				break;
			}
		}
		if (!mediaUrl) {
			ULOGE("application requested media %d, "
			      "but it is not a valid video media",
			      res);
			/* Selecting a wrong video track is an error, stop the
			 * demuxer to either report an open response, or an
			 * unrecoverable error to the application */
			res = pomp_loop_idle_add(
				mSession->getLoop(), &idleStop, this);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_idle_add", -res);
			goto exit;
		}
		ULOGI("application selected media %d (%s)",
		      medias[j].media_id,
		      medias[j].name);
	}

	media = NULL;
	i = 0;
	list_walk_entry_forward(&session->medias, media, node)
	{
		if (i == mediaIdx) {
			found = true;
			break;
		}
		i++;
	}
	if (!found) {
		ULOGE("failed to find the selected media in the list");
		res = pomp_loop_idle_add(mSession->getLoop(), &idleStop, this);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		goto exit;
	}

	if (session->connection_addr &&
	    strcmp(session->connection_addr, "0.0.0.0") != 0)
		remoteAddr = strdup(session->connection_addr);
	else if (session->server_addr)
		remoteAddr = strdup(session->server_addr);
	if (remoteAddr == NULL) {
		ULOGE("failed to get server address");
		mDemuxerListener->onReadyToPlay(this, false);
		/* Stop the demuxer if a previous setup was done */
		if (mRtspState == SETUP_DONE)
			stopRtpAvp();
		mRtspState = OPTIONS_DONE;
		ULOGD("RTSP state change to %s", getRtspStateStr(mRtspState));
		goto exit;
	}

	if (session->range.start.format == SDP_TIME_FORMAT_NPT) {
		mDuration = (uint64_t)session->range.stop.npt.sec * 1000000 +
			    (uint64_t)session->range.stop.npt.usec;
	}

	/* Session metadata */
	sessionMetadataFromSdp(session, media, &meta);
	peerMeta->set(&meta);

	mLocalAddr = "0.0.0.0";
	mRemoteAddr = std::string(remoteAddr);

	res = prepareSetup(&streamPort, &controlPort, &lowerTransport);
	if (res != 0) {
		ULOG_ERRNO("prepareSetup", -res);
		mDemuxerListener->onReadyToPlay(this, false);
		/* Stop the demuxer if a previous setup was done */
		if (mRtspState == SETUP_DONE)
			stopRtpAvp();
		mRtspState = OPTIONS_DONE;
		ULOGD("RTSP state change to %s", getRtspStateStr(mRtspState));
		goto exit;
	}

	if (streamPort != 0)
		mLocalStreamPort = streamPort;
	if (controlPort != 0)
		mLocalControlPort = controlPort;

	mRtspState = DESCRIBE_DONE;
	ULOGD("RTSP state change to %s", getRtspStateStr(mRtspState));

	res = rtsp_client_setup(mRtspClient,
				content_base,
				mediaUrl,
				NULL,
				RTSP_DELIVERY_UNICAST,
				lowerTransport,
				mLocalStreamPort,
				mLocalControlPort,
				NULL,
				RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0) {
		ULOG_ERRNO("rtsp_client_setup", -res);
		mDemuxerListener->onReadyToPlay(this, false);
		/* Stop the demuxer if a previous setup was done */
		if (mRtspState == SETUP_DONE)
			stopRtpAvp();
		mRtspState = OPTIONS_DONE;
		ULOGD("RTSP state change to %s", getRtspStateStr(mRtspState));
	}

exit:
	for (i = 0; i < mediasCount; i++) {
		free((char *)medias[i].name);
		free((char *)medias[i].uri);
	}
	free(medias);
	sdp_session_destroy(session);
	free(remoteAddr);
}

int StreamDemuxer::internalPlay(float speed)
{
	mRunning = true;
	mSpeed = speed;
	mFrameByFrame = false;

	if ((mSessionProtocol == RTSP) && (mRtspState == SETUP_DONE)) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.now = 1;
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
		int ret = rtsp_client_play(mRtspClient,
					   mRtspSessionId,
					   &range,
					   scale,
					   NULL,
					   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (ret < 0) {
			ULOG_ERRNO("rtsp_client_play", -ret);
			return ret;
		}
		ret = pomp_timer_clear(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		mEndOfRangeNotified = false;
	}

	return 0;
}


int StreamDemuxer::internalPause(void)
{
	pomp_timer_clear(mTimer);
	mRunning = false;
	mFrameByFrame = true;

	if ((mSessionProtocol == RTSP) && (mRtspState == SETUP_DONE)) {
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		int ret =
			rtsp_client_pause(mRtspClient,
					  mRtspSessionId,
					  &range,
					  NULL,
					  RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (ret < 0) {
			ULOG_ERRNO("rtsp_client_pause", -ret);
			return ret;
		}
		ret = pomp_timer_clear(mRangeTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		mEndOfRangeNotified = false;
	}

	return 0;
}


int StreamDemuxer::play(float speed)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (speed == 0.)
		return internalPause();
	else
		return internalPlay(speed);
}


bool StreamDemuxer::isPaused(void)
{
	if (mState != STARTED) {
		ULOG_ERRNO("demuxer is not started", EPROTO);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int StreamDemuxer::previous(void)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		ULOGE("demuxer is not paused");
		return -EPROTO;
	}

	if (mSessionProtocol != RTSP)
		return -ENOSYS;

	if (mRtspState != SETUP_DONE)
		return -EAGAIN;

#if 0 /* this code is disabled because previousframe is not working yet on the \
       * server                                                                \
       */
      	int ret = 0;
	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = mPausePoint / 1000000;
	range.start.npt.usec = mPausePoint - range.start.npt.sec * 1000000;
	/* TODO: SMPTE timestamps*/
	int32_t start_usec = (int32_t)range.start.npt.usec - 2 * 34000;
	if (start_usec < 0) {
		if (range.start.npt.sec > 0) {
			range.start.npt.sec--;
			range.start.npt.usec = start_usec + 1000000;
		} else {
			range.start.npt.sec = 0;
			range.start.npt.usec = 0;
		}
	} else {
		range.start.npt.usec = start_usec;
	}
	range.stop = range.start;
	range.stop.npt.usec += 4000;
	if (range.stop.npt.usec >= 1000000) {
		range.stop.npt.sec++;
		range.stop.npt.usec -= 1000000;
	}
	ret = rtsp_client_play(mRtspClient,
			       mRtspSessionId,
			       &range,
			       scale,
			       NULL,
			       RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		ULOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	ret = pomp_timer_clear(mRangeTimer);
	if (ret < 0)
		ULOG_ERRNO("pomp_timer_clear", -ret);
	mEndOfRangeNotified = false;

	return 0;
#else
	return -ENOSYS;
#endif
}


int StreamDemuxer::next(void)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		ULOGE("demuxer is not paused");
		return -EPROTO;
	}

	if (mSessionProtocol != RTSP)
		return -ENOSYS;

	if (mRtspState != SETUP_DONE)
		return -EAGAIN;

#if 0 /* this code is disabled because nextframe is not working yet on         \
       * the server                                                            \
       */
	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = mPausePoint / 1000000;
	range.start.npt.usec = mPausePoint - range.start.npt.sec * 1000000;
	range.stop = range.start;
	/* TODO: SMPTE timestamps*/
	range.stop.npt.usec += 1000;
	if (range.stop.npt.usec >= 1000000) {
		range.stop.npt.sec++;
		range.stop.npt.usec -= 1000000;
	}
	int ret = rtsp_client_play(mRtspClient,
				   mRtspSessionId,
				   &range,
				   scale,
				   NULL,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		ULOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	ret = pomp_timer_clear(mRangeTimer);
	if (ret < 0)
		ULOG_ERRNO("pomp_timer_clear", -ret);
	mEndOfRangeNotified = false;

	return 0;
#else
	return -ENOSYS;
#endif
}


int StreamDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	int64_t ts = (int64_t)mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDuration)
		ts = mDuration;

	return seekTo(ts, exact);
}


int StreamDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (mSessionProtocol != RTSP)
		return -ENOSYS;

	if (mRtspState != SETUP_DONE)
		return -EAGAIN;

	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = timestamp / 1000000;
	range.start.npt.usec = timestamp - range.start.npt.sec * 1000000;
	if (mRunning) {
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
	} else {
		/* Only play a single frame if the seek was requested
		 * while in pause */
		range.stop = range.start;
		/* TODO: SMPTE timestamps*/
		range.stop.npt.usec += 1000;
		if (range.stop.npt.usec >= 1000000) {
			range.stop.npt.sec++;
			range.stop.npt.usec -= 1000000;
		}
	}
	int ret = rtsp_client_play(mRtspClient,
				   mRtspSessionId,
				   &range,
				   scale,
				   NULL,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		ULOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	ret = pomp_timer_clear(mRangeTimer);
	if (ret < 0)
		ULOG_ERRNO("pomp_timer_clear", -ret);
	mEndOfRangeNotified = false;

	return 0;
}


uint64_t StreamDemuxer::getDuration(void)
{
	return (mDuration != 0) ? mDuration : (uint64_t)-1;
}


uint64_t StreamDemuxer::getCurrentTime(void)
{
	if (mSessionProtocol == RTSP)
		return mCurrentTime;
	else
		return (mStartTime != 0) ? mCurrentTime - mStartTime : 0;
}


int StreamDemuxer::setupInputMedia(void)
{
	int ret;
	struct vmeta_session *meta;

	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}
	if (mCodecInfo.codec != VSTRM_CODEC_VIDEO_H264) {
		ULOGE("invalid codec info");
		return -EPROTO;
	}

	Source::lock();

	if (mVideoMedia != NULL) {
		Source::unlock();
		ULOGE("media already defined");
		return -EBUSY;
	}

	mVideoMedia = new VideoMedia(mSession);
	if (mVideoMedia == NULL) {
		Source::unlock();
		ULOG_ERRNO("media allocation", ENOMEM);
		return -ENOMEM;
	}
	ret = addOutputPort(mVideoMedia);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("addOutputPort", -ret);
		return ret;
	}
	mVideoMedia->format = VideoMedia::Format::H264;

	SessionPeerMetadata *peerMeta = mSession->getPeerMetadata();
	meta = (struct vmeta_session *)calloc(1, sizeof(*meta));
	if (meta == NULL) {
		Source::unlock();
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		return ret;
	}
	peerMeta->get(meta);
	if (meta->picture_fov.has_horz)
		mVideoMedia->hfov = meta->picture_fov.horz;
	if (meta->picture_fov.has_vert)
		mVideoMedia->vfov = meta->picture_fov.vert;
	free(meta);

	ret = mVideoMedia->setSpsPps(mCodecInfo.h264.sps,
				     mCodecInfo.h264.spslen,
				     mCodecInfo.h264.pps,
				     mCodecInfo.h264.ppslen);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("media->setSpsPps", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(
		mH264Reader, 0, mCodecInfo.h264.sps, mCodecInfo.h264.spslen);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("h264_reader_parse_nalu:sps", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(
		mH264Reader, 0, mCodecInfo.h264.pps, mCodecInfo.h264.ppslen);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("h264_reader_parse_nalu:pps", -ret);
		return ret;
	}

	/* Create the output buffers pool */
	ret = createOutputPortBuffersPool(mVideoMedia,
					  DEMUXER_OUTPUT_BUFFER_COUNT,
					  mVideoMedia->width *
						  mVideoMedia->height * 3 / 4);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("createOutputBuffersPool", -ret);
		return ret;
	}

	/* New synchronization is needed */
	mWaitForSync = true;
	mRecoveryFrameCount = 0;
	mIdrFrameSyncPending = true;

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(this, mVideoMedia);

	/* Try to sync now */
	Source::lock();
	if (idrFrameSync() == 0)
		mIdrFrameSyncPending = false;
	Source::unlock();

	return 0;
}


int StreamDemuxer::createReceiver(void)
{
	SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();
	struct vstrm_receiver_cfg *cfg;
	std::string fn;
	std::string sn;
	std::string sv;
	int ret;

	cfg = (struct vstrm_receiver_cfg *)calloc(1, sizeof(*cfg));
	if (cfg == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		goto error;
	}

	/* Create the stream receiver */
	cfg->loop = mSession->getLoop();
	cfg->flags = VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE |
		     VSTRM_RECEIVER_FLAGS_H264_FAKE_FRAME_NUM |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT;
	selfMeta->getFriendlyName(&fn);
	strncpy(cfg->self_meta.friendly_name,
		fn.c_str(),
		sizeof(cfg->self_meta.friendly_name));
	cfg->self_meta.friendly_name[sizeof(cfg->self_meta.friendly_name) - 1] =
		'\0';
	selfMeta->getSerialNumber(&sn);
	strncpy(cfg->self_meta.serial_number,
		sn.c_str(),
		sizeof(cfg->self_meta.serial_number));
	cfg->self_meta.serial_number[sizeof(cfg->self_meta.serial_number) - 1] =
		'\0';
	selfMeta->getSoftwareVersion(&sv);
	strncpy(cfg->self_meta.software_version,
		sv.c_str(),
		sizeof(cfg->self_meta.software_version));
	cfg->self_meta
		.software_version[sizeof(cfg->self_meta.software_version) - 1] =
		'\0';
	ret = vstrm_receiver_new(cfg, &mReceiverCbs, this, &mReceiver);
	if (ret < 0) {
		mReceiver = NULL;
		ULOG_ERRNO("vstrm_receiver_new", -ret);
		goto error;
	}

	/* Provide the SPS/PPS out of band if available */
	if (mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) {
		ret = vstrm_receiver_set_codec_info(
			mReceiver, &mCodecInfo, mSsrc);
		if (ret < 0)
			ULOG_ERRNO("vstrm_receiver_set_codec_info", -ret);
	}

	free(cfg);
	return 0;

error:
	destroyReceiver();
	free(cfg);
	return ret;
}


int StreamDemuxer::destroyReceiver(void)
{
	int res;
	if (mReceiver != NULL) {
		/* Destroy the receiver */
		res = vstrm_receiver_destroy(mReceiver);
		if (res < 0)
			ULOG_ERRNO("vstrm_receiver_destroy", -res);
		mReceiver = NULL;
	}
	return 0;
}


void StreamDemuxer::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;
	int ret = 0;

	if (demuxer == NULL)
		return;
	if ((buf == NULL) || (len == 0))
		return;
	if (sei == NULL)
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	/* ignore "Parrot Streaming" v1 and v2 user data SEI */
	if ((vstrm_h264_sei_streaming_is_v1(sei->uuid)) ||
	    (vstrm_h264_sei_streaming_is_v2(sei->uuid)))
		return;

	ret = vbuf_set_userdata_capacity(demuxer->mCurrentBuffer, len);
	if (ret < (signed)len) {
		ULOG_ERRNO("vbuf_set_userdata_capacity", -ret);
		return;
	}

	uint8_t *dstBuf = vbuf_get_userdata(demuxer->mCurrentBuffer);
	memcpy(dstBuf, buf, len);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, len);
}


void StreamDemuxer::h264PicTimingSeiCb(struct h264_ctx *ctx,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_sei_pic_timing *sei,
				       void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;

	if (demuxer == NULL)
		return;
	if (ctx == NULL)
		return;
	if (sei == NULL)
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	demuxer->mCurrentBufferCaptureTs =
		h264_ctx_sei_pic_timing_to_us(ctx, sei);
}


void StreamDemuxer::h264RecoveryPointSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_recovery_point *sei,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;

	if (demuxer == NULL)
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	demuxer->Source::lock();
	if (demuxer->mWaitForSync) {
		demuxer->mWaitForSync = false;
		demuxer->mRecoveryFrameCount = sei->recovery_frame_cnt + 1;
	}
	demuxer->Source::unlock();
}


int StreamDemuxer::sendCtrlCb(struct vstrm_receiver *stream,
			      struct pomp_buffer *buf,
			      void *userdata)
{
	if (userdata == NULL)
		return -EINVAL;

	return ((StreamDemuxer *)userdata)->sendCtrl(stream, buf);
}


void StreamDemuxer::codecInfoChangedCb(struct vstrm_receiver *stream,
				       const struct vstrm_codec_info *info,
				       void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;
	int outputChannelCount = 0, i;
	Channel *channel;
	int ret;

	if ((demuxer == NULL) || (info == NULL))
		return;
	if (info->codec != VSTRM_CODEC_VIDEO_H264) {
		ULOG_ERRNO("info->codec", EPROTO);
		return;
	}
	if (demuxer->mState != STARTED) {
		ULOGE("demuxer is not started");
		return;
	}

	ULOGD("codec info changed");
	demuxer->mWaitForCodecInfo = false;

	if ((!demuxer->mCodecInfoChanging) &&
	    (!memcmp(
		    &demuxer->mCodecInfo, info, sizeof(demuxer->mCodecInfo)))) {
		ULOGI("codec info changed; no change in PS, "
		      "just flush and resync");
		ret = demuxer->flush();
		if (ret < 0)
			ULOG_ERRNO("flush", -ret);
		return;
	}
	demuxer->mCodecInfo = *info;

	demuxer->Source::lock();

	if (demuxer->mCurrentBuffer != NULL) {
		ret = vbuf_unref(demuxer->mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		demuxer->mCurrentBuffer = NULL;
	}

	if (demuxer->mVideoMedia != NULL) {
		ULOGI("change of output media");
		demuxer->mCodecInfoChanging = true;
		outputChannelCount =
			demuxer->getOutputChannelCount(demuxer->mVideoMedia);

		/* Teardown the output channels
		 * Note: loop downwards because calling teardown on a channel
		 * may or may not synchronously remove the channel from the
		 * output port */
		for (i = outputChannelCount - 1; i >= 0; i--) {
			channel = demuxer->getOutputChannel(
				demuxer->mVideoMedia, i);
			if (channel == NULL) {
				ULOGW("failed to get channel at index %d", i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				ULOG_ERRNO("channel->teardown", -ret);
		}
	} else {
		ULOGI("new output media");
		demuxer->mCodecInfoChanging = false;
		ret = demuxer->setupInputMedia();
		if (ret < 0) {
			demuxer->Source::unlock();
			ULOG_ERRNO("setupInputMedia", -ret);
			return;
		}
	}

	demuxer->Source::unlock();
}


int StreamDemuxer::processFrame(struct vstrm_frame *frame)
{
	int ret = 0, lock_ret, queue_ret;
	unsigned int outputChannelCount = 0, i;
	Channel *channel;
	VideoMedia::Frame *data = NULL;
	uint32_t flags = 0;
	size_t frameSize = 0, bufSize;
	uint8_t *buf;
	ssize_t res;
	bool isIdr = false, byteStreamRequired = false;
	struct vbuf_pool *pool;
	VideoMedia::H264BitstreamFormat format;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	uint64_t remainingPlayTime = 0;

	Source::lock();

	/* Get an output buffer */
	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}
	ret = getH264OutputBuffer(
		mVideoMedia, &mCurrentBuffer, &byteStreamRequired);
	if ((ret < 0) || (mCurrentBuffer == NULL)) {
		Source::unlock();
		ULOGW("failed to get an input buffer (%d)", ret);
		int flush_ret = flush();
		if (flush_ret < 0)
			ULOG_ERRNO("flush", -flush_ret);
		return ret;
	}
	buf = vbuf_get_data(mCurrentBuffer);
	res = vbuf_get_capacity(mCurrentBuffer);
	if ((buf == NULL) || (res <= 0)) {
		ULOGE("invalid input buffer");
		ret = res;
		goto out;
	}
	bufSize = res;
	mCurrentBufferCaptureTs = 0;

	/* Get the size of the frame */
	flags = (byteStreamRequired)
			? VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE
			: VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE;
	flags |= VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS |
		 VSTRM_FRAME_COPY_FLAGS_FILTER_SEI;
	ret = vstrm_frame_get_size(frame, &frameSize, flags);
	if (ret < 0) {
		ULOG_ERRNO("vstrm_frame_get_size", -ret);
		goto out;
	}
	if (bufSize < frameSize) {
		ULOGW("input buffer too small (%zi vs. %zu",
		      bufSize,
		      frameSize);
		ret = -EPROTO;
		goto out;
	}

	/* Copy the frame */
	/* TODO: avoid copy? */
	ret = vstrm_frame_copy(frame, buf, frameSize, flags);
	if (ret < 0) {
		ULOG_ERRNO("vstrm_frame_copy", -ret);
		goto out;
	}
	vbuf_set_size(mCurrentBuffer, frameSize);

	/* Metadata */
	ret = vbuf_metadata_add(mCurrentBuffer,
				mVideoMedia,
				1,
				sizeof(*data),
				(uint8_t **)&data);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_add", -ret);
		goto out;
	}
	memset(data, 0, sizeof(*data));

	/* IDR and user data */
	vbuf_set_userdata_size(mCurrentBuffer, 0);
	for (i = 0; i < frame->nalu_count; i++) {
		uint8_t naluType = *frame->nalus[i].cdata & 0x1F;
		if (naluType == H264_NALU_TYPE_SLICE_IDR) {
			/* IDR slice */
			isIdr = true;
		} else if (naluType == H264_NALU_TYPE_SEI) {
			/* SEI NAL unit */
			ret = h264_reader_parse_nalu(mH264Reader,
						     0,
						     frame->nalus[i].cdata,
						     frame->nalus[i].len);
			if (ret < 0)
				ULOG_ERRNO("h264_reader_parse_nalu:sei", -ret);
		}
	}

	/* If the frame is an IDR (excluding the generated gray IDR),
	 * sync is complete */
	if ((mWaitForSync) && (isIdr) && (!frame->info.gen_grey_idr)) {
		mWaitForSync = false;
		mRecoveryFrameCount = 0;
	}

	/* If sync is in progress (intra refresh + recovery point received),
	 * decrement recoveryFrameCount for every ref frame until 0 */
	if ((!mWaitForSync) && (mRecoveryFrameCount > 0) && (frame->info.ref))
		mRecoveryFrameCount--;

	data->format = VideoMedia::Format::H264;
	format = (byteStreamRequired)
			 ? VideoMedia::H264BitstreamFormat::BYTE_STREAM
			 : VideoMedia::H264BitstreamFormat::AVCC;
	data->h264Frame.format = format;
	data->h264Frame.isComplete = (frame->info.complete) ? true : false;
	data->hasErrors = (frame->info.error) ? true : false;
	data->h264Frame.isSync = isIdr;
	data->h264Frame.isRef = (frame->info.ref) ? true : false;
	data->isSilent =
		((!mWaitForSync) && (mRecoveryFrameCount == 0)) ? false : true;
	/* TODO: use unskewed timestamps */
	data->ntpTimestamp = frame->timestamps.ntp;
	data->ntpUnskewedTimestamp = frame->timestamps.ntp_unskewed;
	data->ntpRawTimestamp = frame->timestamps.ntp_raw;
	data->ntpRawUnskewedTimestamp = frame->timestamps.ntp_raw_unskewed;
	data->captureTimestamp = mCurrentBufferCaptureTs;
	data->localTimestamp = frame->timestamps.local;

	/* Frame metadata */
	if (frame->metadata.type != VMETA_FRAME_TYPE_NONE) {
		data->hasMetadata = true;
		data->metadata = frame->metadata;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	data->demuxOutputTimestamp = curTime;
	mLastFrameReceiveTime = curTime;

	if (mSessionProtocol == RTSP) {
		mCurrentTime = frame->timestamps.ntp * mSpeed - mNtpToNptOffset;
	} else {
		mCurrentTime = frame->timestamps.ntp;
		if (mStartTime == 0)
			mStartTime = frame->timestamps.ntp;
	}
	data->playTimestamp = mCurrentTime;
	if (mDuration > 0) {
		remainingPlayTime = mDuration - mCurrentTime;
		if (remainingPlayTime < 1000000) {
			/* Less than 1s of play time remaining */

			/* Take the speed into account */
			remainingPlayTime = (mSpeed != 0.f)
						    ? remainingPlayTime / mSpeed
						    : UINT64_MAX;

			if (remainingPlayTime < 1000000) {
				/* Add 50ms margin */
				pomp_timer_set(mRangeTimer,
					       remainingPlayTime / 1000 + 50);
			}
		}
	}

	lock_ret = vbuf_write_lock(mCurrentBuffer);
	if (lock_ret < 0)
		ULOG_ERRNO("vbuf_write_lock", -lock_ret);

	/* Queue the buffer in the output channels */
	outputChannelCount = getOutputChannelCount(mVideoMedia);
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(mVideoMedia, i);
		if (channel == NULL) {
			ULOGW("invalid channel");
			continue;
		}
		pool = channel->getPool();

		if ((!(channel->getVideoMediaSubFormatCaps() & format)) &&
		    (pool == NULL)) {
			ULOGW("incompatible sub-format on channel");
			continue;
		}

		queue_ret = channel->queue(mCurrentBuffer);
		if (queue_ret < 0)
			ULOG_ERRNO("channel->queue", -queue_ret);
	}
	if ((mFirstFrame) && (!data->isSilent)) {
		int evtRet = Source::sendDownstreamEvent(
			mVideoMedia, Channel::DownstreamEvent::SOS);
		if (evtRet < 0)
			ULOG_ERRNO("sendDownstreamEvent", -evtRet);
		mFirstFrame = false;
	}

out:
	vbuf_unref(mCurrentBuffer);
	mCurrentBuffer = NULL;

	Source::unlock();
	return ret;
}


int StreamDemuxer::idrFrameSync(void)
{
	int ret;
	struct vstrm_frame *idr_frame = NULL;

	if (mState != STARTED)
		return -EAGAIN;
	if ((mCodecInfoChanging) || (mFlushing)) {
		return -EAGAIN;
	}

	ret = vstrm_receiver_generate_grey_i_frame(mReceiver, &idr_frame);
	if (ret < 0) {
		ULOG_ERRNO("vstrm_receiver_generate_grey_i_frame", -ret);
		return ret;
	}

	ret = processFrame(idr_frame);
	if ((ret < 0) && (ret != -EAGAIN))
		ULOG_ERRNO("processFrame", -ret);

	vstrm_frame_unref(idr_frame);
	return ret;
}


void StreamDemuxer::recvFrameCb(struct vstrm_receiver *stream,
				struct vstrm_frame *frame,
				void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;
	int ret;

	if ((demuxer == NULL) || (frame == NULL))
		return;
	if (demuxer->mState != STARTED)
		return;

	if (demuxer->mRunning) {
		pomp_timer_set(demuxer->mTimer,
			       DEMUXER_STREAM_TIMER_INTERVAL_MS);
	} else if (demuxer->mSessionProtocol != RTSP) {
		/* just ignore the frames */
		return;
	}

	if ((demuxer->mWaitForCodecInfo) || (demuxer->mCodecInfoChanging) ||
	    (demuxer->mFlushing)) {
		return;
	}

	if (demuxer->mIdrFrameSyncPending) {
		/* Ignore all pending frames and sync */
		ret = demuxer->flush();
		if (ret < 0)
			ULOG_ERRNO("flush", -ret);
		return;
	}

	/* Process the incoming frame */
	ret = demuxer->processFrame(frame);
	if (ret < 0) {
		if (ret != -EAGAIN)
			ULOG_ERRNO("processFrame", -ret);
		return;
	}
}


void StreamDemuxer::sessionMetadataPeerChangedCb(
	struct vstrm_receiver *stream,
	const struct vmeta_session *meta,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;

	if ((demuxer == NULL) || (meta == NULL))
		return;
	if (demuxer->mSession == NULL) {
		ULOGE("invalid session");
		return;
	}

	ULOGD("session metadata changed");

	SessionPeerMetadata *peerMeta = demuxer->mSession->getPeerMetadata();
	peerMeta->set(meta);
	demuxer->Source::lock();
	if (demuxer->mVideoMedia != NULL) {
		if (meta->picture_fov.has_horz)
			demuxer->mVideoMedia->hfov = meta->picture_fov.horz;
		if (meta->picture_fov.has_vert)
			demuxer->mVideoMedia->vfov = meta->picture_fov.vert;
	}
	demuxer->Source::unlock();
}


void StreamDemuxer::goodbyeCb(struct vstrm_receiver *stream,
			      const char *reason,
			      void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;
	int ret;
	Channel::DownstreamEvent event;
	bool sendEvent = false;

	if (demuxer == NULL)
		return;

	ULOGI("received RTCP goodbye%s%s",
	      reason ? ", reason: " : "",
	      reason ? reason : "");

	if (demuxer->mState != STARTED)
		return;

	pomp_timer_clear(demuxer->mTimer);

	/* Wait for new codec info */
	demuxer->mWaitForCodecInfo = true;

	if (reason != NULL) {
		if (strcmp(reason, DEMUXER_STREAM_GOODBYE_REASON_RECONFIGURE) ==
		    0) {
			event = Channel::DownstreamEvent::RECONFIGURE;
			sendEvent = true;
		} else if (
			strcmp(reason,
			       DEMUXER_STREAM_GOODBYE_REASON_PHOTO_TRIGGER) ==
			0) {
			event = Channel::DownstreamEvent::PHOTO_TRIGGER;
			sendEvent = true;
		} else {
			event = Channel::DownstreamEvent::EOS;
			sendEvent = true;
			demuxer->mFirstFrame = true;
			if (demuxer->mSessionProtocol == RTSP &&
			    (strcmp(reason,
				    DEMUXER_STREAM_GOODBYE_REASON_USER) ||
			     !demuxer->mTearingDown)) {
				/* We either received an unknown RTCP goodbye
				 * packet, or an unexpected (not initiated by a
				 * teardown) user_disconnection packet. Stop the
				 * demuxer to notify the application of an
				 * unrecoverable error */
				ret = pomp_loop_idle_add(
					demuxer->mSession->getLoop(),
					&idleStop,
					demuxer);
				if (ret < 0)
					ULOG_ERRNO("pomp_loop_idle_add", -ret);
			}
		}

		if (sendEvent) {
			demuxer->Source::lock();
			if (demuxer->mVideoMedia != NULL) {
				ret = demuxer->Source::sendDownstreamEvent(
					demuxer->mVideoMedia, event);
				if (ret < 0)
					ULOG_ERRNO("sendDownstreamEvent", -ret);
			}
			demuxer->Source::unlock();
		}
	}
}


void StreamDemuxer::frameTimeoutCb(struct pomp_timer *timer, void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer *)userdata;
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	if (demuxer == NULL)
		return;

	if (demuxer->mState != STARTED)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0)
		ULOG_ERRNO("time_timespec_to_us", -res);

	demuxer->Source::lock();
	if ((demuxer->mVideoMedia != NULL) &&
	    (curTime > demuxer->mLastFrameReceiveTime +
			       DEMUXER_STREAM_FRAME_TIMEOUT_US)) {
		ULOGI("frame reception timeout");
		res = demuxer->Source::sendDownstreamEvent(
			demuxer->mVideoMedia,
			Channel::DownstreamEvent::TIMEOUT);
		if (res < 0)
			ULOG_ERRNO("sendDownstreamEvent", -res);
	}
	demuxer->Source::unlock();
}


void StreamDemuxer::rangeTimerCb(struct pomp_timer *timer, void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	if (!self->mEndOfRangeNotified) {
		ULOGI("end of range reached");
		res = self->Source::sendDownstreamEvent(
			self->mVideoMedia, Channel::DownstreamEvent::EOS);
		if (res < 0)
			ULOG_ERRNO("sendDownstreamEvent", -res);
		self->mDemuxerListener->onEndOfRange(self, self->mCurrentTime);
		self->mEndOfRangeNotified = true;
	}
}


const char *StreamDemuxer::getRtspStateStr(StreamDemuxer::RtspState val)
{
	switch (val) {
	case StreamDemuxer::RtspState::DISCONNECTED:
		return "DISCONNECTED";
	case StreamDemuxer::RtspState::CONNECTED:
		return "CONNECTED";
	case StreamDemuxer::RtspState::OPTIONS_DONE:
		return "OPTIONS_DONE";
	case StreamDemuxer::RtspState::DESCRIBE_DONE:
		return "DESCRIBE_DONE";
	case StreamDemuxer::RtspState::SETUP_DONE:
		return "SETUP_DONE";
	default:
		return NULL;
	}
}

} /* namespace Pdraw */
