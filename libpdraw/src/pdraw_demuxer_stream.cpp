/**
 * Parrot Drones Audio and Video Vector library
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

#define ULOG_TAG pdraw_dmxstrm
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_demuxer_stream.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

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
#include <media-buffers/mbuf_ancillary_data.h>
#include <rtp/rtp.h>

namespace Pdraw {


#define DEMUXER_STREAM_FRAME_TIMEOUT_MS 2000

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
	.teardown = &StreamDemuxer::onRtspForcedTeardown,
};


const struct vstrm_receiver_cbs StreamDemuxer::VideoMedia::mReceiverCbs = {
	.send_ctrl = &StreamDemuxer::VideoMedia::sendCtrlCb,
	.codec_info_changed = &StreamDemuxer::VideoMedia::codecInfoChangedCb,
	.recv_frame = &StreamDemuxer::VideoMedia::recvFrameCb,
	.recv_rtp_pkt = nullptr,
	.session_metadata_peer_changed =
		&StreamDemuxer::VideoMedia::sessionMetadataPeerChangedCb,
	.event = &StreamDemuxer::VideoMedia::eventCb,
	.goodbye = &StreamDemuxer::VideoMedia::goodbyeCb,
};


const struct h264_ctx_cbs StreamDemuxer::VideoMedia::mH264Cbs = {
	.au_end = nullptr,
	.nalu_begin = nullptr,
	.nalu_end = nullptr,
	.slice = nullptr,
	.slice_data_begin = nullptr,
	.slice_data_end = nullptr,
	.slice_data_mb = nullptr,
	.sps = nullptr,
	.pps = nullptr,
	.aud = nullptr,
	.sei = nullptr,
	.sei_buffering_period = nullptr,
	.sei_pic_timing = &StreamDemuxer::VideoMedia::h264PicTimingSeiCb,
	.sei_pan_scan_rect = nullptr,
	.sei_filler_payload = nullptr,
	.sei_user_data_registered = nullptr,
	.sei_user_data_unregistered =
		&StreamDemuxer::VideoMedia::h264UserDataSeiCb,
	.sei_recovery_point =
		&StreamDemuxer::VideoMedia::h264RecoveryPointSeiCb,
};


StreamDemuxer::StreamDemuxer(Session *session,
			     Element::Listener *elementListener,
			     Source::Listener *sourceListener,
			     DemuxerWrapper *wrapper,
			     IPdraw::IDemuxer::Listener *demuxerListener,
			     const struct pdraw_demuxer_params *params) :
		Demuxer(session,
			elementListener,
			sourceListener,
			wrapper,
			demuxerListener,
			params),
		mServerPort(0), mContentBase(nullptr), mSessionProtocol(NONE),
		mSetupRequestsCount(0), mTeardownRequestsCount(0),
		mSdpSession(nullptr), mSessionMetaFromSdp({}),
		mChannelsReadyForStop(false), mNetworkReadyForStop(false),
		mRtspState(DISCONNECTED), mRtspClient(nullptr),
		mRtspSessionId(nullptr), mRunning(false), mFlushing(false),
		mDestroyMediasAfterFlush(false), mFlushChannelCount(0),
		mStartTime(0), mDuration(0), mTrackDuration(0),
		mUpdateTrackDuration(false), mCurrentTime(0), mPausePoint(0),
		mNtpToNptOffset(0), mRtpClockRate(0), mSpeed(1.f),
		mFrameByFrame(true), mEndOfRangeNotified(false), mSeeking(false)
{
	Element::setClassName(__func__);
}


StreamDemuxer::~StreamDemuxer(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		PDRAW_LOGW("demuxer is still running");

	if (mSdpSession != nullptr)
		sdp_session_destroy(mSdpSession);

	cleanupRtspRequests();

	destroyAllVideoMedias();

	if (mRtspClient != nullptr) {
		ret = rtsp_client_destroy(mRtspClient);
		if (ret < 0)
			PDRAW_LOG_ERRNO("rtsp_client_destroy", -ret);
	}

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	free((void *)mContentBase);
	free((void *)mRtspSessionId);
}


void StreamDemuxer::sessionMetadataFromSdp(const struct sdp_session *session,
					   struct vmeta_session *meta)
{
	int err;
	struct sdp_attr *attr = nullptr;

	memset(meta, 0, sizeof(*meta));

	if (session->session_name != nullptr) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_NAME,
			session->session_name,
			nullptr,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	if (session->session_info != nullptr) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_INFO,
			session->session_info,
			nullptr,
			meta);
		if (err < 0)
			ULOG_ERRNO("vmeta_session_streaming_sdp_read", -err);
	}
	if (session->tool != nullptr) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_SESSION_TOOL,
			session->tool,
			nullptr,
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
}


int StreamDemuxer::processRtspRequests(void)
{
	int res;

	if (!mSetupRequests.empty()) {
		res = processSetupRequest();
		if (res < 0) {
			if (res != -EBUSY)
				onUnrecoverableError(res);
			return res;
		}
	}

	if (!mTeardownRequests.empty() && (mSetupRequestsCount == 0)) {
		res = processTeardownRequest();
		if (res < 0) {
			if (res != -EBUSY)
				onUnrecoverableError(res);
			return res;
		}
	}

	return 0;
}


void StreamDemuxer::cleanupRtspRequests(void)
{
	while (!mSetupRequests.empty()) {
		SetupRequest req = mSetupRequests.front();
		mSetupRequests.pop();
		mSetupRequestsCount--;
		free(req.controlUrl);
	}

	while (!mTeardownRequests.empty()) {
		TeardownRequest req = mTeardownRequests.front();
		mTeardownRequests.pop();
		mTeardownRequestsCount--;
		free(req.controlUrl);
	}
}


int StreamDemuxer::processSetupRequest(void)
{
	int res;

	if (mSetupRequests.empty()) {
		if (mSetupRequestsCount > 0) {
			/* Returning -EBUSY here to notify that there are still
			 * setup requests in progress (for example asynchronous
			 * requests from the StreamDemuxerMux subclass that are
			 * not yet in the queue) */
			return -EBUSY;
		} else {
			/* All setup requests have been processed; nothing
			 * more to do */
			return 0;
		}
	}

	/* Process the next setup request */
	SetupRequest req = mSetupRequests.front();

	res = rtsp_client_setup(mRtspClient,
				mContentBase,
				req.controlUrl,
				mRtspSessionId,
				RTSP_DELIVERY_UNICAST,
				req.lowerTransport,
				req.localStreamPort,
				req.localControlPort,
				req.headerExt,
				req.headerExtCount,
				req.media,
				RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res == -EBUSY) {
		/* Another setup request is already in progress, the current
		 * setup request will be processed later (i.e. chained in the
		 * setup response callback function) */
		return 0;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_setup", -res);
	} else {
		/* Success: returning -EBUSY here to notify that there are
		 * still setup requests in progress */
		res = -EBUSY;
	}

	mSetupRequests.pop();
	mSetupRequestsCount--;
	free(req.controlUrl);
	return res;
}


int StreamDemuxer::processTeardownRequest(void)
{
	int res;

	if (mTeardownRequests.empty()) {
		if (mTeardownRequestsCount > 0) {
			/* Returning -EBUSY here to notify that there are still
			 * teardown requests in progress (for example
			 * asynchronous requests from the StreamDemuxerMux
			 * subclass that are not yet in the queue) */
			return -EBUSY;
		} else {
			/* All teardown requests have been processed; nothing
			 * more to do */
			return 0;
		}
	}

	/* Process the next teardown request */
	TeardownRequest req = mTeardownRequests.front();

	res = rtsp_client_teardown(mRtspClient,
				   req.controlUrl,
				   mRtspSessionId,
				   nullptr,
				   0,
				   req.media,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res == -EBUSY) {
		/* Another teardown request is already in progress, the current
		 * teardown request will be processed later (i.e. chained in the
		 * teardown response callback function) */
		return 0;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_teardown", -res);
	} else {
		/* Success: returning -EBUSY here to notify that there are
		 * still teardown requests in progress */
		res = -EBUSY;
	}

	mTeardownRequests.pop();
	mTeardownRequestsCount--;
	free(req.controlUrl);
	return res;
}


void StreamDemuxer::teardownVideoMedia(StreamDemuxer::VideoMedia *media)
{
	PDRAW_LOG_ERRNO_RETURN_IF(media == nullptr, EINVAL);

	if (media->isTearingDown())
		return;

	media->sendDownstreamEvent(Channel::DownstreamEvent::EOS);
	media->setTearingDown();
	media->flush(true);
	asyncCompleteTeardown();
}


void StreamDemuxer::teardownAllVideoMedias(void)
{
	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->isTearingDown())
			continue;
		teardownVideoMedia((*p));
	}
}


void StreamDemuxer::destroyAllVideoMedias(void)
{
	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++)
		delete *p;
	mVideoMedias.clear();
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
	int err = 0;
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	PDRAW_LOGI("RTSP client %s", rtsp_client_conn_state_str(state));

	switch (state) {
	case RTSP_CLIENT_CONN_STATE_DISCONNECTED:
		self->setRtspState(DISCONNECTED);
		for (auto p = self->mVideoMedias.begin();
		     p != self->mVideoMedias.end();
		     p++) {
			if ((*p)->isTearingDown())
				continue;
			(*p)->resetFrameTimer(false);
		}
		self->mRunning = false;
		self->mNetworkReadyForStop = true;

		if (self->mState == STOPPING)
			self->tryCompleteStop();
		break;
	case RTSP_CLIENT_CONN_STATE_CONNECTED:
		/* If previous RTSP state is not DISCONNECTED, do not
		 * send OPTIONS request */
		if (self->mRtspState != DISCONNECTED)
			break;
		self->setRtspState(CONNECTED);

		err = rtsp_client_options(self->mRtspClient,
					  nullptr,
					  0,
					  nullptr,
					  RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (err < 0)
			PDRAW_LOG_ERRNO("rtsp_client_options", -err);
		break;
	case RTSP_CLIENT_CONN_STATE_CONNECTING:
	case RTSP_CLIENT_CONN_STATE_DISCONNECTING:
		if (self->mState == STOPPING)
			self->asyncRtspDisconnect();
		break;
	default:
		PDRAW_LOGW("unhandled RTSP connection state: (%d: %s)",
			   state,
			   rtsp_client_conn_state_str(state));
		break;
	}
}


void StreamDemuxer::onRtspSessionRemoved(struct rtsp_client *client,
					 const char *session_id,
					 int status,
					 void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		PDRAW_LOGD("wrong session removed (%s, expected %s)",
			   session_id,
			   self->mRtspSessionId);
		return;
	} else {
		free((void *)self->mRtspSessionId);
		self->mRtspSessionId = nullptr;
	}

	ULOG_EVT("STREAM",
		 "event='client_session_removed';element='%s';"
		 "status=%d;status_str='%s';session='%s';res='%s'",
		 self->getCName(),
		 status,
		 strerror(-status),
		 session_id ? session_id : "",
		 self->mRtspPath.c_str());

	self->teardownAllVideoMedias();

	if (self->mRtspState != DISCONNECTED)
		self->setRtspState(OPTIONS_DONE);

	if (self->mState == STOPPING) {
		if (self->mRtspState != DISCONNECTED)
			self->asyncRtspDisconnect();
	} else {
		/* We used to call stop() here; this is no longer the case
		 * because now a demuxer must not stop itself; we call
		 * unrecoverableError instead, and it is the application's
		 * responsibility to stop and destroy the demuxer and create a
		 * new one. */
		self->onUnrecoverableError();
	}
}


void StreamDemuxer::onRtspOptionsResp(struct rtsp_client *client,
				      enum rtsp_client_req_status req_status,
				      int status,
				      uint32_t methods,
				      const struct rtsp_header_ext *ext,
				      size_t ext_count,
				      void *userdata,
				      void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			PDRAW_LOGE("RTSP options request failed (%d: %s)",
				   status,
				   strerror(-status));
			res = status;
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			PDRAW_LOGE("RTSP options request aborted");
			res = -EPROTO;
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			PDRAW_LOGE("RTSP options request canceled");
			res = -ECANCELED;
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			PDRAW_LOGE("timeout on RTSP options request");
			res = -ETIMEDOUT;
			break;
		default:
			/* This should not happen */
			PDRAW_LOGE("unexpected status on options request: %d",
				   req_status);
			res = -EPROTO;
			break;
		}

		ULOG_EVT("STREAM",
			 "event='client_options_resp';element='%s';"
			 "status=%d;status_str='%s';res='%s'",
			 self->getCName(),
			 res,
			 strerror(-res),
			 self->mRtspPath.c_str());

		self->onUnrecoverableError(res);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_options_resp';element='%s';"
		 "status=%d;status_str='%s';res='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 self->mRtspPath.c_str());

	self->setRtspState(OPTIONS_DONE);

	res = rtsp_client_describe(self->mRtspClient,
				   self->mRtspPath.c_str(),
				   nullptr,
				   0,
				   nullptr,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0)
		PDRAW_LOG_ERRNO("rtsp_client_describe", -res);
}


void StreamDemuxer::onRtspDescribeResp(struct rtsp_client *client,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const char *content_base,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       const char *sdp,
				       void *userdata,
				       void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			PDRAW_LOGE("RTSP describe request failed (%d: %s)",
				   status,
				   strerror(-status));
			res = status;
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			PDRAW_LOGE("RTSP describe request aborted");
			res = -EPROTO;
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			PDRAW_LOGE("RTSP describe request canceled");
			res = -ECANCELED;
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			PDRAW_LOGE("timeout on RTSP describe request");
			res = -ETIMEDOUT;
			break;
		default:
			/* This should not happen */
			PDRAW_LOGE("unexpected status on describe request: %d",
				   req_status);
			res = -EPROTO;
			break;
		}

		ULOG_EVT("STREAM",
			 "event='client_describe_resp';element='%s';"
			 "status=%d;status_str='%s';res='%s'",
			 self->getCName(),
			 res,
			 strerror(-res),
			 self->mRtspPath.c_str());

		self->onUnrecoverableError(res);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_describe_resp';element='%s';"
		 "status=%d;status_str='%s';res='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 self->mRtspPath.c_str());

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
				    const struct rtsp_header_ext *ext,
				    size_t ext_count,
				    void *userdata,
				    void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	VideoMedia *media = reinterpret_cast<VideoMedia *>(req_userdata);
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			PDRAW_LOGE("RTSP setup request failed (%d: %s)",
				   status,
				   strerror(-status));
			res = status;
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			PDRAW_LOGE("RTSP setup request aborted");
			res = -EPROTO;
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			PDRAW_LOGE("RTSP setup request canceled");
			res = -ECANCELED;
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			PDRAW_LOGE("timeout on RTSP setup request");
			res = -ETIMEDOUT;
			break;
		default:
			/* This should not happen */
			PDRAW_LOGE("unexpected status on setup request: %d",
				   req_status);
			res = -EPROTO;
			break;
		}

		ULOG_EVT("STREAM",
			 "event='client_setup_resp';element='%s';"
			 "status=%d;status_str='%s';session='%s'%s%s%s;"
			 "res='%s';media='%s';src='%s:%" PRIu16 ",%" PRIu16
			 "';dst='%s:%" PRIu16 ",%" PRIu16 "'",
			 self->getCName(),
			 res,
			 strerror(-res),
			 session_id ? session_id : "",
			 proxy_session ? ";proxy_session='" : "",
			 proxy_session ? proxy_session : "",
			 proxy_session ? "'" : "",
			 self->mRtspPath.c_str(),
			 media ? media->getControlUrl() : "",
			 self->mRemoteAddr.c_str(),
			 (uint16_t)0,
			 (uint16_t)0,
			 self->mLocalAddr.c_str(),
			 (uint16_t)0,
			 (uint16_t)0);

		self->onUnrecoverableError(res);
		return;
	}

	free((void *)self->mRtspSessionId);
	self->mRtspSessionId = xstrdup(session_id);

	if (media != nullptr) {
		media->setSsrc((ssrc_valid) ? ssrc : 0);
		media->setRemoteStreamPort(server_stream_port);
		media->setRemoteControlPort(server_control_port);

		res = media->startRtpAvp();
		if (res < 0)
			PDRAW_LOG_ERRNO("startRtpAvp", -res);
	}

	ULOG_EVT("STREAM",
		 "event='client_setup_resp';element='%s';"
		 "status=%d;status_str='%s';session='%s'%s%s%s;"
		 "res='%s';media='%s';src='%s:%" PRIu16 ",%" PRIu16
		 "';dst='%s:%" PRIu16 ",%" PRIu16 "'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 session_id ? session_id : "",
		 proxy_session ? ";proxy_session='" : "",
		 proxy_session ? proxy_session : "",
		 proxy_session ? "'" : "",
		 self->mRtspPath.c_str(),
		 media ? media->getControlUrl() : "",
		 self->mRemoteAddr.c_str(),
		 server_stream_port,
		 server_control_port,
		 self->mLocalAddr.c_str(),
		 media ? media->getLocalStreamPort() : (uint16_t)0,
		 media ? media->getLocalControlPort() : (uint16_t)0);

	if (res < 0) {
		self->onUnrecoverableError(res);
		return;
	}

	self->setRtspState(SETUP_DONE);

	if (!self->mCalledOpenResp)
		self->openResponse(0);

	self->readyToPlay(true);

	if (self->mRunning) {
		/* Play newly setup medias */
		self->internalPlay(self->mSpeed);
	}

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
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
				   const struct rtsp_header_ext *ext,
				   size_t ext_count,
				   void *userdata,
				   void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	uint64_t start = 0, stop = 0, ntptime = 0;
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}
	if (range != nullptr) {
		if (range->start.format == RTSP_TIME_FORMAT_NPT)
			rtsp_time_npt_to_us(&range->start.npt, &start);
		if (range->stop.format == RTSP_TIME_FORMAT_NPT) {
			rtsp_time_npt_to_us(&range->stop.npt, &stop);
			if (self->mUpdateTrackDuration) {
				/* RTSP server may update track duration, if
				 * it was requested to play the file to the end.
				 */
				self->mUpdateTrackDuration = false;
				self->mTrackDuration = stop;
			}
		}
	}

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			PDRAW_LOGE("RTSP play request failed (%d: %s)",
				   status,
				   strerror(-status));
			res = status;
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			PDRAW_LOGE("RTSP play request aborted");
			res = -EPROTO;
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			PDRAW_LOGE("RTSP play request canceled");
			res = -ECANCELED;
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			/* TODO: strategy on RTSP play timeout to be
			 * defined. Retry RTSP play? */
			PDRAW_LOGE("timeout on RTSP play request");
			res = -ETIMEDOUT;
			break;
		default:
			/* This should not happen */
			PDRAW_LOGE("unexpected status on play request: %d",
				   req_status);
			res = -EPROTO;
			break;
		}

		ULOG_EVT("STREAM",
			 "event='client_play_resp';element='%s';"
			 "status=%d;status_str='%s';session='%s'%s%s%s;"
			 "res='%s';start_ts=%" PRIu64 ";stop_ts=%" PRIu64
			 ";rtp_ts=%" PRIu32 ";seq=%" PRIu16,
			 self->getCName(),
			 res,
			 strerror(-res),
			 session_id ? session_id : "",
			 proxy_session ? ";proxy_session='" : "",
			 proxy_session ? proxy_session : "",
			 proxy_session ? "'" : "",
			 self->mRtspPath.c_str(),
			 start,
			 stop,
			 rtptime,
			 seq);

		if (self->mSeeking)
			self->seekResponse(
				status, self->mCurrentTime, self->mSpeed);
		else
			self->playResponse(
				status, self->mCurrentTime, self->mSpeed);
		self->mSeeking = false;
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		PDRAW_LOGE(
			"RTSP play response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_play_resp';element='%s';"
		 "status=%d;status_str='%s';session='%s'%s%s%s;"
		 "res='%s';start_ts=%" PRIu64 ";stop_ts=%" PRIu64
		 ";rtp_ts=%" PRIu32 ";seq=%" PRIu16,
		 self->getCName(),
		 res,
		 strerror(-res),
		 session_id ? session_id : "",
		 proxy_session ? ";proxy_session='" : "",
		 proxy_session ? proxy_session : "",
		 proxy_session ? "'" : "",
		 self->mRtspPath.c_str(),
		 start,
		 stop,
		 rtptime,
		 seq);

	self->mSpeed = (scale != 0.) ? scale : 1.0;
	if ((rtptime_valid) && (self->mRtpClockRate != 0)) {
		ntptime = rtp_timestamp_to_us(rtptime, self->mRtpClockRate);
	}
	self->mNtpToNptOffset =
		(int64_t)(ntptime * self->mSpeed) - (int64_t)start;
	self->mPausePoint = stop;
	if (self->mSeeking)
		self->seekResponse(0, start, self->mSpeed);
	else
		self->playResponse(0, start, self->mSpeed);
	self->mSeeking = false;

	if ((start == stop) && start && stop) {
		/* The end of range is reached */
		int res = pomp_loop_idle_add_with_cookie(
			self->mSession->getLoop(),
			&idleEndOfRangeNotification,
			self,
			self);
		if (res < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
		self->mSeeking = false;
	}

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspPauseResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    const struct rtsp_range *range,
				    const struct rtsp_header_ext *ext,
				    size_t ext_count,
				    void *userdata,
				    void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	uint64_t start = 0;
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}
	if ((range != nullptr) && (range->start.format == RTSP_TIME_FORMAT_NPT))
		rtsp_time_npt_to_us(&range->start.npt, &start);

	if (req_status != RTSP_CLIENT_REQ_STATUS_OK) {
		switch (req_status) {
		case RTSP_CLIENT_REQ_STATUS_FAILED:
			PDRAW_LOGE("RTSP pause request failed (%d: %s)",
				   status,
				   strerror(-status));
			res = status;
			break;
		case RTSP_CLIENT_REQ_STATUS_ABORTED:
			PDRAW_LOGE("RTSP pause request aborted");
			res = -EPROTO;
			break;
		case RTSP_CLIENT_REQ_STATUS_CANCELED:
			PDRAW_LOGE("RTSP pause request canceled");
			res = -ECANCELED;
			break;
		case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
			/* TODO: strategy on RTSP pause timeout to be
			 * defined. Retry RTSP pause? */
			PDRAW_LOGE("timeout on RTSP pause request");
			res = -ETIMEDOUT;
			break;
		default:
			/* This should not happen */
			PDRAW_LOGE("unexpected status on pause request: %d",
				   req_status);
			res = -EPROTO;
			break;
		}

		ULOG_EVT("STREAM",
			 "event='client_pause_resp';element='%s';"
			 "status=%d;status_str='%s';session='%s'%s%s%s;"
			 "res='%s';ts=%" PRIu64,
			 self->getCName(),
			 res,
			 strerror(-res),
			 session_id ? session_id : "",
			 proxy_session ? ";proxy_session='" : "",
			 proxy_session ? proxy_session : "",
			 proxy_session ? "'" : "",
			 self->mRtspPath.c_str(),
			 start);

		self->pauseResponse(status, self->mCurrentTime);
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		PDRAW_LOGE(
			"RTSP pause response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_pause_resp';element='%s';"
		 "status=%d;status_str='%s';session='%s'%s%s%s;"
		 "res='%s';ts=%" PRIu64,
		 self->getCName(),
		 res,
		 strerror(-res),
		 session_id ? session_id : "",
		 proxy_session ? ";proxy_session='" : "",
		 proxy_session ? proxy_session : "",
		 proxy_session ? "'" : "",
		 self->mRtspPath.c_str(),
		 start);

	self->mPausePoint = start;
	self->pauseResponse(0, self->mPausePoint);

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspTeardownResp(struct rtsp_client *client,
				       const char *session_id,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       void *userdata,
				       void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	VideoMedia *media = reinterpret_cast<VideoMedia *>(req_userdata);
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}

	if (std::find(self->mVideoMedias.begin(),
		      self->mVideoMedias.end(),
		      media) == self->mVideoMedias.end()) {
		/* Media not found */
		media = nullptr;
	}

	switch (req_status) {
	case RTSP_CLIENT_REQ_STATUS_OK:
		break;
	case RTSP_CLIENT_REQ_STATUS_ABORTED:
		/* Teardown effective by disconnection */
		PDRAW_LOGW("RTSP teardown request aborted");
		res = -EPROTO;
		break;
	case RTSP_CLIENT_REQ_STATUS_FAILED:
		/* Force disconnection anyway */
		PDRAW_LOGE("RTSP teardown request failed (%d: %s)",
			   status,
			   strerror(-status));
		res = status;
		break;
	case RTSP_CLIENT_REQ_STATUS_CANCELED:
		/* No disconnection */
		PDRAW_LOGI("RTSP teardown request canceled");
		res = -ECANCELED;
		break;
	case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
		/* Force disconnection anyway */
		PDRAW_LOGE("timeout on RTSP teardown request");
		res = -ETIMEDOUT;
		break;
	default:
		/* This should not happen */
		PDRAW_LOGE("unexpected status on teardown request: %d",
			   req_status);
		res = -EPROTO;
		break;
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		PDRAW_LOGE(
			"RTSP teardown response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_teardown_resp';element='%s';"
		 "status=%d;status_str='%s';session='%s'%s%s%s;"
		 "res='%s';media='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 session_id ? session_id : "",
		 proxy_session ? ";proxy_session='" : "",
		 proxy_session ? proxy_session : "",
		 proxy_session ? "'" : "",
		 self->mRtspPath.c_str(),
		 media ? media->getControlUrl() : "?");

	if (media != nullptr)
		self->teardownVideoMedia(media);

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspAnnounce(struct rtsp_client *client,
				   const char *content_base,
				   const struct rtsp_header_ext *ext,
				   size_t ext_count,
				   const char *sdp,
				   void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (!self->mContentBase ||
	    strcmp(self->mContentBase, content_base) != 0)
		return;

	const char *resource = nullptr;
	if (strlen(content_base) > 7) {
		const char *p = strchr(content_base + 7, '/');
		if (p)
			resource = p + 1;
	}
	ULOG_EVT("STREAM",
		 "event='client_announce';element='%s';res='%s'",
		 self->getCName(),
		 resource ? resource : "");

	self->onNewSdp(content_base, sdp);
}


void StreamDemuxer::onRtspForcedTeardown(struct rtsp_client *client,
					 const char *path,
					 const char *session_id,
					 const struct rtsp_header_ext *ext,
					 size_t ext_count,
					 void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	VideoMedia *media = nullptr;
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}

	if (xstrcmp(session_id, self->mRtspSessionId) != 0) {
		PDRAW_LOGE(
			"RTSP forced teardown for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId);
		return;
	}

	for (auto p = self->mVideoMedias.begin(); p != self->mVideoMedias.end();
	     p++) {
		std::string mediaPath = std::string(self->mContentBase) + "/" +
					std::string((*p)->getControlUrl());
		if ((*p)->isTearingDown())
			continue;
		if (xstrcmp(mediaPath.c_str(), path) == 0) {
			media = (*p);
			break;
		}
	}

	ULOG_EVT("STREAM",
		 "event='forced_teardown';element='%s';"
		 "status=%d;status_str='%s';session='%s'%s%s%s;"
		 "res='%s';media='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 session_id ? session_id : "",
		 proxy_session ? ";proxy_session='" : "",
		 proxy_session ? proxy_session : "",
		 proxy_session ? "'" : "",
		 self->mRtspPath.c_str(),
		 media ? media->getControlUrl() : "?");

	if (media != nullptr)
		self->teardownVideoMedia(media);
}


void StreamDemuxer::asyncRtspDisconnect(void)
{
	int err = pomp_loop_idle_add_with_cookie(
		this->mSession->getLoop(), idleRtspDisconnect, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void StreamDemuxer::idleRtspDisconnect(void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	if (self->mRtspState == DISCONNECTED)
		return;

	res = rtsp_client_disconnect(self->mRtspClient);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_disconnect", -res);
		return;
	}
}


int StreamDemuxer::startRtsp(const std::string &url)
{
	int res;
	std::string::size_type n;

	if (mRtspClient != nullptr) {
		res = -EBUSY;
		PDRAW_LOG_ERRNO("mRtspClient", -res);
		return res;
	}

	/* Check that we actually have something after 'rtsp://' */
	if (url.length() < 8)
		return -EINVAL;

	/* Find first '/' in url */
	n = url.find("/", 7);
	if (n == std::string::npos)
		return -EINVAL;
	mServerAddr = url.substr(7, n - 7);
	mRtspAddr = url.substr(0, n);
	mRtspPath = url.substr(n + 1);

	/* Find first ':' in mServerAddr */
	n = mServerAddr.find(":");
	if (n != std::string::npos) {
		mServerIpAddr = mServerAddr.substr(0, n);
		mServerPort = std::stoi(mServerAddr.substr(n + 1));
	} else {
		mServerIpAddr = mServerAddr;
		mServerPort = 0; /* 0 means defaut */
	}

	mSessionProtocol = RTSP;

	std::string userAgent;
	mSession->getSettings()->getSoftwareVersion(&userAgent);

	/* Create the RTSP client */
	res = rtsp_client_new(mSession->getLoop(),
			      userAgent.c_str(),
			      &mRtspClientCbs,
			      this,
			      &mRtspClient);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_new", -res);
		return res;
	}

	res = rtsp_client_connect(mRtspClient, mRtspAddr.c_str());
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_connect", -res);
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
		PDRAW_LOGE("%s: demuxer is not created", __func__);
		return -EPROTO;
	}
	setState(STARTING);

	if (mUrl.length() > 0) {
		std::string ext = mUrl.substr(mUrl.length() - 4, 4);
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

		if (mUrl.substr(0, 7) == "rtsp://") {
			res = startRtsp(mUrl);
			if (res < 0) {
				PDRAW_LOG_ERRNO("startRtsp", -res);
				return res;
			}
		} else {
			PDRAW_LOGE("unsupported URL");
			return -ENOSYS;
		}

		/* Do not call openResponse now, it will be called when
		 * receiving a response to the RTSP SETUP request. */
		tryCompleteStart(false);
	} else {
		/* TODO create VideoMedia and setup */
		if (mVideoMedias.size() >= 1) {
			res = mVideoMedias.front()->startRtpAvp();
			if (res < 0) {
				PDRAW_LOG_ERRNO("startRtpAvp", -res);
				return res;
			}
		}

		tryCompleteStart();
		readyToPlay(true);
	}

	return 0;
}


int StreamDemuxer::stop(void)
{
	int ret = 0;
	bool disconnect = false;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED && mState != STARTING) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);

	/* Note: the demuxer listener is not cleared here to allow calling
	 * the IDemuxer::Listener::demuxerCloseResponse listener function when
	 * the IDemuxer::close function was called; clearing the listener when
	 * deleting the API object is done by calling
	 * Demuxer::clearDemuxerListener in the API object destructor prior
	 * to calling Demuxer::stop */

	readyToPlay(false);

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++)
		(*p)->sendDownstreamEvent(Channel::DownstreamEvent::EOS);

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
	if (mSessionProtocol == RTSP) {
		if (mRtspState == SETUP_DONE) {
			for (auto p = mVideoMedias.begin();
			     p != mVideoMedias.end();
			     p++) {
				(*p)->teardown();
				if (mUnrecoverableError) {
					disconnect = true;
					break;
				}
			}
		} else {
			disconnect = true;
		}
		if (disconnect) {
			ret = rtsp_client_disconnect(mRtspClient);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("rtsp_client_disconnect", -ret);
				return ret;
			}
		}
	} else {
		mRunning = false;
		mNetworkReadyForStop = true;
		if (mVideoMedias.size() >= 1)
			mVideoMedias.front()->stopRtpAvp();
	}

	Source::lock();

	ret = flush();
	if ((ret < 0) && (ret != -EALREADY))
		PDRAW_LOG_ERRNO("flush", -ret);

	Source::unlock();

	tryCompleteStop();

	return ret;
}


void StreamDemuxer::tryCompleteStop(void)
{
	if (mState != STOPPING)
		return;

	if (!mNetworkReadyForStop || !mChannelsReadyForStop)
		return;

	destroyAllVideoMedias();

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
	closeResponse(0);
	setStateAsyncNotify(STOPPED);
}


int StreamDemuxer::flush(void)
{
	return flush(false);
}


int StreamDemuxer::flush(bool destroyMedias)
{
	if ((mState != STARTED) && (mState != STOPPING)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	Source::lock();

	mDestroyMediasAfterFlush = destroyMedias;

	if (mFlushing) {
		Source::unlock();
		return -EALREADY;
	}

	mFlushChannelCount = 0;

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++)
		(*p)->flush();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGW("failed to get media at index %d", i);
			continue;
		}

		mFlushChannelCount += getOutputChannelCount(media);
	}

	if (mFlushChannelCount == 0) {
		mChannelsReadyForStop = true;
		mFlushing = false;
		mDestroyMediasAfterFlush = false;
	}

	Source::unlock();

	return 0;
}


void StreamDemuxer::onChannelFlushed(Channel *channel)
{
	bool destroyMedia = false;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::lock();

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->hasMedia(media)) {
			(*p)->channelFlushed(channel);
			destroyMedia = (*p)->getDestroyAfterFlush();
			break;
		}
	}

	if (mState == STOPPING || mDestroyMediasAfterFlush || destroyMedia) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mFlushChannelCount == 0) {
		mFlushing = false;
		mDestroyMediasAfterFlush = false;
	}

	Source::unlock();
}


void StreamDemuxer::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}

	int ret = removeOutputChannel(media, channel);
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeOutputChannel", -ret);

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->hasMedia(media)) {
			(*p)->channelUnlink(channel);
			if ((*p)->getMediaCount() == 0 &&
			    (*p)->isTearingDown()) {
				delete *p;
				mVideoMedias.erase(p);
			}
			break;
		}
	}

	asyncCompleteTeardown();
}


void StreamDemuxer::asyncCompleteTeardown(void)
{
	int err = pomp_loop_idle_add_with_cookie(
		this->mSession->getLoop(), idleCompleteTeardown, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void StreamDemuxer::idleCompleteTeardown(void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	self->completeTeardown();
}


void StreamDemuxer::completeTeardown(void)
{
	Source::lock();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		Media *media = getOutputMedia(i);
		if (media && getOutputChannelCount(media) > 0) {
			Source::unlock();
			return;
		}
	}

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if (!(*p)->isTearingDown()) {
			Source::unlock();
			return;
		}
	}

	destroyAllVideoMedias();

	Source::unlock();

	if (mState == STOPPING) {
		mChannelsReadyForStop = true;

		/* If network is also ready, set the state to stopped */
		tryCompleteStop();
#if 0 /* TODO: this should be re-enabled on a per-media basis */
	} else if (mCodecInfoChanging) {
		PDRAW_LOGI("new output media");
		mCodecInfoChanging = false;
		ret = setupInputMedia();
		if (ret < 0) {
			PDRAW_LOG_ERRNO("setupInputMedia", -ret);
			return;
		}
#endif
	}
}


void StreamDemuxer::onChannelResync(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::lock();

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}
	PDRAW_LOGD("'%s': channel resync media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if (!(*p)->isTearingDown() && (*p)->hasMedia(media)) {
			(*p)->resync();
			Source::unlock();
			return;
		}
	}

	Source::unlock();
}


void StreamDemuxer::onChannelVideoPresStats(Channel *channel,
					    VideoPresStats *stats)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}
	if (stats == nullptr) {
		PDRAW_LOG_ERRNO("stats", EINVAL);
		return;
	}

	Source::lock();

	Source::onChannelVideoPresStats(channel, stats);

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if (!(*p)->isTearingDown() && (*p)->hasMedia(media)) {
			(*p)->channelSendVideoPresStats(channel, stats);
			break;
		}
	}

	Source::unlock();
}


int StreamDemuxer::processSelectedMedias(void)
{
	int res = 0;
	bool noError = false;
	struct sdp_media *media = nullptr;

	for (auto m = mSelectedMedias.begin(); m != mSelectedMedias.end();
	     m++) {
		bool found = false;
		media = nullptr;
		int idx = 0;
		list_walk_entry_forward(&mSdpSession->medias, media, node)
		{
			if (idx == (*m)->idx) {
				found = true;
				break;
			}
			idx++;
		}
		if (!found) {
			PDRAW_LOGE(
				"failed to find the selected "
				"media in the list");
			goto stop;
		}
		found = false;
		for (auto p = mVideoMedias.begin(); p != mVideoMedias.end();
		     p++) {
			if ((*p)->isTearingDown())
				continue;
			if (xstrcmp((*p)->getControlUrl(),
				    media->control_url) == 0) {
				PDRAW_LOGI("media '%s' is already set up",
					   media->control_url);
				found = true;
				break;
			}
		}
		if (!found) {
			StreamDemuxer::VideoMedia *videoMedia =
				createVideoMedia();
			res = videoMedia->setup(media);
			if (res < 0) {
				PDRAW_LOG_ERRNO("VideoMedia::setup", -res);
				delete videoMedia;
				goto stop;
			}
			PDRAW_LOGI("media '%s' not set up, setting up",
				   media->control_url);
			mVideoMedias.push_back(videoMedia);
		}
	}
	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->isTearingDown())
			continue;
		bool found = false;
		for (auto m = mSelectedMedias.begin();
		     m != mSelectedMedias.end();
		     m++) {
			std::string name1 = (*m)->uri;
			std::string name2 = (*p)->getControlUrl();
			if (name1 == name2) {
				found = true;
				break;
			}
		}
		if (!found) {
			PDRAW_LOGI(
				"media '%s' not selected anymore, "
				"tear it down",
				(*p)->getControlUrl());
			(*p)->teardown();
		}
	}

	res = 0;

	goto exit;

stop:
	if (!noError)
		onUnrecoverableError();
	readyToPlay(false);
	if (mRtspState == SETUP_DONE) {
		for (auto p = mVideoMedias.begin(); p != mVideoMedias.end();
		     p++)
			(*p)->stopRtpAvp();
	}
	setRtspState(OPTIONS_DONE);

exit:
	return res;
}


void StreamDemuxer::onNewSdp(const char *content_base, const char *sdp)
{
	int res = 0;
	struct sdp_session *session = nullptr;
	char *remoteAddr = nullptr;
	size_t mediasCount = 0, i;
	bool noError = false;
	struct sdp_media *media = nullptr;
	uint32_t selectedMedias = 0;
	struct pdraw_demuxer_media *newMediaList = NULL;
	size_t newMediaListSize = 0;
	std::vector<struct pdraw_demuxer_media *> newDefaultMedias;

	if (mState == STOPPING) {
		PDRAW_LOGI("new SDP while stopping, ignore it");
		return;
	}

	res = sdp_description_read(sdp, &session);
	if (res < 0) {
		PDRAW_LOG_ERRNO("sdp_description_read", -res);
		return;
	}

	if (mSdpSession != nullptr) {
		res = sdp_session_compare(session, mSdpSession);
		if (res == 0) {
			PDRAW_LOGI("sdp is identical, ignore it");
			goto exit;
		}
		sdp_session_destroy(mSdpSession);
	}
	mSdpSession = session;

	if (mSdpSession->deletion)
		PDRAW_LOGW("sdp refers to a no longer existing session");

	if ((!mContentBase) && (content_base))
		mContentBase = strdup(content_base);

	/* Session-level metadata */
	sessionMetadataFromSdp(mSdpSession, &mSessionMetaFromSdp);

	list_walk_entry_forward(&mSdpSession->medias, media, node)
	{
		if (media->type != SDP_MEDIA_TYPE_VIDEO || !media->control_url)
			continue;
		mediasCount++;
	}

	newMediaList = (struct pdraw_demuxer_media *)calloc(
		mediasCount, sizeof(*newMediaList));
	if (newMediaList == nullptr) {
		PDRAW_LOGE("calloc");
		goto exit;
	}
	newMediaListSize = mediasCount;

	i = 0;
	list_walk_entry_forward(&mSdpSession->medias, media, node)
	{
		if (media->type != SDP_MEDIA_TYPE_VIDEO || !media->control_url)
			continue;

#if 0
		/* TODO: this is wrong with multistream: the codec info
		 * and clock rate will be from the last media in the
		 * list instead of the chosen media */
		if ((media->h264_fmtp.valid) &&
		    (media->h264_fmtp.sps != nullptr) &&
		    (media->h264_fmtp.pps != nullptr)) {
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
#endif
		mRtpClockRate = media->clock_rate;

		struct pdraw_demuxer_media *current = &newMediaList[i];
		current->media_id = i + 1;
		current->idx = i;
		switch (media->type) {
		case SDP_MEDIA_TYPE_VIDEO:
			current->type = PDRAW_MEDIA_TYPE_VIDEO;
			break;
		case SDP_MEDIA_TYPE_AUDIO:
			current->type = PDRAW_MEDIA_TYPE_AUDIO;
			break;
		default:
			current->type = PDRAW_MEDIA_TYPE_UNKNOWN;
			break;
		}
		current->name = xstrdup(media->media_title);
		if (current->name == nullptr)
			current->name = xstrdup(media->control_url);
		current->uri = xstrdup(media->control_url);
		StreamDemuxer::VideoMedia::sessionMetadataFromSdp(
			media,
			&mSessionMetaFromSdp,
			&current->video.session_meta);
		current->is_default = current->video.session_meta.default_media;
		if (current->is_default)
			newDefaultMedias.push_back(current);
		i++;
	}

	if (mediasCount == 1 && newDefaultMedias.empty()) {
		newMediaList[0].is_default = 1;
		newDefaultMedias.push_back(&newMediaList[0]);
	} else if (mediasCount == 0) {
		/* Empty SDP */
		PDRAW_LOGI("empty SDP, no stream");
		/* An empty SDP means that both the server & the URL are good,
		 * but there is currently no streams. In this case, we can set
		 * the demuxer as STARTED here, and wait for an ANNOUCE to get
		 * the media. Otherwise, wait for the setup response before
		 * setting the state. If we have any media, remove them */
		flush(true);
		tryCompleteStart();
		noError = true;
		goto stop;
	} else if (newDefaultMedias.empty()) {
		/* If no default media, check for a media with camera_type ==
		 * VMETA_CAMERA_TYPE_FRONT, and flag it as the default media */
		for (i = 0; i < mediasCount; i++) {
			struct pdraw_demuxer_media *current = &newMediaList[i];
			if (current->video.session_meta.camera_type !=
			    VMETA_CAMERA_TYPE_FRONT)
				continue;
			current->is_default = 1;
			newDefaultMedias.push_back(current);
		}
	}

	res = updateMediaList(newMediaList,
			      newMediaListSize,
			      newDefaultMedias,
			      &selectedMedias);
	if (res < 0) {
		PDRAW_LOG_ERRNO("updateMediaList", -res);
		goto stop;
	}

	newMediaList = nullptr;
	newMediaListSize = 0;
	newDefaultMedias.clear();

	res = callSelectMedia(selectedMedias);
	if (res >= 0) {
		selectedMedias = res;
	} else if (res == -ENOSYS) {
		selectedMedias = 0;
	} else if (res == -ECANCELED) {
		PDRAW_LOGI("application cancelled the media selection");
		tryCompleteStart();
		noError = true;
		goto stop;
	} else if (res < 0) {
		PDRAW_LOGE("application failed to select a media");
		/* Selecting a wrong media is an error, stop the demuxer
		 * to either report an open response, or an unrecoverable error
		 * to the application */
		goto stop;
	}

	res = Demuxer::selectMedia(selectedMedias);
	if (res < 0)
		goto stop;

	if (mSdpSession->connection_addr &&
	    strcmp(mSdpSession->connection_addr, "0.0.0.0") != 0)
		remoteAddr = strdup(mSdpSession->connection_addr);
	else if (mSdpSession->server_addr &&
		 strcmp(mSdpSession->server_addr, "0.0.0.0") != 0)
		remoteAddr = strdup(mSdpSession->server_addr);
	else if (mServerIpAddr.length() > 0)
		remoteAddr = strdup(mServerIpAddr.c_str());
	if (remoteAddr == nullptr) {
		PDRAW_LOGE("failed to get server address");
		goto stop;
	}

	if (mSdpSession->range.start.format == SDP_TIME_FORMAT_NPT) {
		mDuration =
			(uint64_t)mSdpSession->range.stop.npt.sec * 1000000 +
			(uint64_t)mSdpSession->range.stop.npt.usec;
		mTrackDuration = mDuration;
	}

	if (mRtspState != SETUP_DONE)
		setRtspState(DESCRIBE_DONE);

	mLocalAddr = "0.0.0.0";
	mRemoteAddr = std::string(remoteAddr);

	processSelectedMedias();

	goto exit;

stop:
	if (!noError)
		onUnrecoverableError();
	readyToPlay(false);
	if (mRtspState == SETUP_DONE) {
		for (auto p = mVideoMedias.begin(); p != mVideoMedias.end();
		     p++)
			(*p)->stopRtpAvp();
	}
	setRtspState(OPTIONS_DONE);

exit:
	for (size_t i = 0; i < newMediaListSize; i++) {
		free((void *)newMediaList[i].name);
		free((void *)newMediaList[i].uri);
	}
	free(newMediaList);
	free(remoteAddr);
}


void StreamDemuxer::tryCompleteStart(bool callOpenResp)
{
	if (mState != STARTING)
		return;

	setState(STARTED);

	if (callOpenResp && !mCalledOpenResp)
		openResponse(0);
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
		mUpdateTrackDuration = true;
		int ret = rtsp_client_play(mRtspClient,
					   mRtspSessionId,
					   &range,
					   scale,
					   nullptr,
					   0,
					   nullptr,
					   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("rtsp_client_play", -ret);
			return ret;
		}
		mEndOfRangeNotified = false;
	}

	return 0;
}


int StreamDemuxer::internalPause(void)
{
	mRunning = false;
	mFrameByFrame = true;

	if ((mSessionProtocol == RTSP) && (mRtspState == SETUP_DONE)) {
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		int ret =
			rtsp_client_pause(mRtspClient,
					  mRtspSessionId,
					  &range,
					  nullptr,
					  0,
					  nullptr,
					  RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("rtsp_client_pause", -ret);
			return ret;
		}
		mEndOfRangeNotified = false;
	}

	return 0;
}


int StreamDemuxer::play(float speed)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->isTearingDown())
			continue;
		if (speed != 0.)
			(*p)->play();
		else
			(*p)->pause();
	}

	if (speed == 0.)
		return internalPause();
	else
		return internalPlay(speed);
}


bool StreamDemuxer::isReadyToPlay(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	return mReadyToPlay;
}


bool StreamDemuxer::isPaused(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int StreamDemuxer::previous(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
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
			       nullptr,
			       RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	ret = pomp_timer_clear(mRangeTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	mEndOfRangeNotified = false;

	return 0;
#else
	return -ENOSYS;
#endif
}


int StreamDemuxer::next(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
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
				   nullptr,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	ret = pomp_timer_clear(mRangeTimer);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
	mEndOfRangeNotified = false;

	return 0;
#else
	return -ENOSYS;
#endif
}


int StreamDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	int64_t ts = (int64_t)mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mTrackDuration)
		ts = mTrackDuration;

	return seekTo(ts, exact);
}


int StreamDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
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
	mUpdateTrackDuration = range.stop.npt.infinity ? true : false;
	int ret = rtsp_client_play(mRtspClient,
				   mRtspSessionId,
				   &range,
				   scale,
				   nullptr,
				   0,
				   nullptr,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_play", -ret);
		return ret;
	}
	mSeeking = true;
	mEndOfRangeNotified = false;
	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->isTearingDown())
			continue;
		(*p)->play();
	}

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


void StreamDemuxer::setRtspState(StreamDemuxer::RtspState state)
{
	StreamDemuxer::RtspState curState = mRtspState;

	if (state != curState) {
		mRtspState = state;
		PDRAW_LOGI("RTSP state change to %s",
			   getRtspStateStr(mRtspState));
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
		return nullptr;
	}
}


int StreamDemuxer::selectMedia(uint32_t selectedMedias)
{
	int ret = 0;

	ret = Demuxer::selectMedia(selectedMedias);
	if (ret < 0)
		goto stop;

	if (mRtspState != SETUP_DONE)
		setRtspState(DESCRIBE_DONE);

	processSelectedMedias();

	return 0;

stop:
	readyToPlay(false);
	if (mRtspState == SETUP_DONE) {
		for (auto p = mVideoMedias.begin(); p != mVideoMedias.end();
		     p++)
			(*p)->stopRtpAvp();
	}
	setRtspState(OPTIONS_DONE);

	return ret;
}


StreamDemuxer::VideoMedia::VideoMedia(StreamDemuxer *demuxer) :
		mDemuxer(demuxer), mReceiver(nullptr), mLocalStreamPort(0),
		mLocalControlPort(0), mRemoteStreamPort(0),
		mRemoteControlPort(0), mVideoMedias(nullptr), mNbVideoMedias(0),
		mSdpMedia(nullptr), mH264Reader(nullptr), mFrameTimer(nullptr),
		mRangeTimer(nullptr), mSsrc(0), mFlushing(false),
		mDestroyAfterFlush(false), mPendingTearDown(false),
		mTearingDown(false), mFlushChannelCount(0), mFirstFrame(true),
		mLastFrameReceiveTime(0), mFrameIndex(0), mCodecInfo({}),
		mWaitForCodecInfo(false), mCodecInfoChanging(false),
		mWaitForSync(false), mRecoveryFrameCount(0),
		mCurrentFrame(nullptr), mCurrentMem(nullptr),
		mCurrentMemOffset(0), mCurrentFrameCaptureTs(0),
		mSessionMetaFromSdp({})
{
	std::string name = demuxer->getName() + "#VideoMedia";
	Loggable::setName(name);
}


StreamDemuxer::VideoMedia::~VideoMedia(void)
{
	int ret;

	teardownMedia();

	if (mSdpMedia != nullptr)
		sdp_media_destroy(mSdpMedia);

	/* Clear any pending frames (received while the previous media
	 * was being torn down */
	while (!mTempQueue.empty()) {
		struct vstrm_frame *frame = mTempQueue.front();
		mTempQueue.pop();
		vstrm_frame_unref(frame);
	}

	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
	}

	if (mFrameTimer != nullptr) {
		resetFrameTimer(false);
		ret = pomp_timer_destroy(mFrameTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
	}

	if (mRangeTimer != nullptr) {
		ret = pomp_timer_clear(mRangeTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mRangeTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
	}
}


bool StreamDemuxer::VideoMedia::hasMedia(Media *media)
{
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		if (mVideoMedias[i] == media)
			return true;
	}
	return false;
}


int StreamDemuxer::VideoMedia::setup(const struct sdp_media *media)
{
	int ret;

	std::string name =
		mDemuxer->getName() +
		((media != nullptr) ? "#" + std::string(media->control_url)
				    : "#NULL");
	Loggable::setName(name);

	/* Create the frame timer */
	mFrameTimer = pomp_timer_new(
		mDemuxer->mSession->getLoop(), &frameTimeoutCb, this);
	if (mFrameTimer == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -ret);
		return ret;
	}

	/* Create the end of range timer */
	mRangeTimer = pomp_timer_new(
		mDemuxer->mSession->getLoop(), &rangeTimerCb, this);
	if (mRangeTimer == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -ret);
		return ret;
	}

	if (media != nullptr && mDemuxer->mSessionProtocol == RTSP) {
		sdp_media_destroy(mSdpMedia);
		mSdpMedia = sdp_media_new();
		if (mSdpMedia == nullptr) {
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("sdp_media_new", -ret);
			return ret;
		}
		ret = sdp_media_copy(media, mSdpMedia);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("sdp_media_copy", -ret);
			return ret;
		}
	}

	mDemuxer->mSetupRequestsCount++;
	ret = prepareSetup();
	if (ret == -EINPROGRESS) {
		/* Nothing to do, subclass will call finishSetup when needed */
		return 0;
	} else if (ret != 0) {
		PDRAW_LOG_ERRNO("prepareSetup", -ret);
		return ret;
	}
	finishSetup();
	return 0;
}


void StreamDemuxer::VideoMedia::finishSetup(void)
{
	if (mSdpMedia != nullptr) {
		/* Media-level metadata */
		sessionMetadataFromSdp(mSdpMedia,
				       &mDemuxer->mSessionMetaFromSdp,
				       &mSessionMetaFromSdp);

		SetupRequest req = {
			.media = this,
			.controlUrl = strdup(mSdpMedia->control_url),
			.lowerTransport = getLowerTransport(),
			.localStreamPort = getLocalStreamPort(),
			.localControlPort = getLocalControlPort(),
			.headerExt = getHeaderExt(),
			.headerExtCount = getHeaderExtCount(),
		};
		mDemuxer->mSetupRequests.push(req);

		(void)mDemuxer->processRtspRequests();
	}
}


int StreamDemuxer::VideoMedia::teardown(void)
{
	if (mTearingDown)
		return -EALREADY;

	if (mPendingTearDown)
		return -EBUSY;

	mPendingTearDown = true;

	finishTeardown();
	return 0;
}


void StreamDemuxer::VideoMedia::finishTeardown(void)
{
	if (getControlUrl() == nullptr)
		return;

	/* Media-level metadata */
	TeardownRequest req = {
		.media = this,
		.controlUrl = strdup(getControlUrl()),
	};
	mDemuxer->mTeardownRequests.push(req);
	mDemuxer->mTeardownRequestsCount++;

	(void)mDemuxer->processRtspRequests();
}


void StreamDemuxer::VideoMedia::setTearingDown(void)
{
	mPendingTearDown = false;
	mTearingDown = true;
	/* Needed to close sockets */
	stopRtpAvp();
	/* TODO: remove TEARDOWN request from the queue? */
}


int StreamDemuxer::VideoMedia::setupMedia(void)
{
	int ret;
	Source::OutputPort *basePort, *mediaPort;

	if (mDemuxer->mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	/* Note: H.265 streaming is not supported */
	if (mCodecInfo.codec != VSTRM_CODEC_VIDEO_H264) {
		PDRAW_LOGE("invalid codec info");
		return -EPROTO;
	}

	mDemuxer->Source::lock();

	if (mNbVideoMedias > 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media already defined");
		return -EBUSY;
	}

	mNbVideoMedias = 2;
	mVideoMedias = (CodedVideoMedia **)calloc(mNbVideoMedias,
						  sizeof(*mVideoMedias));
	if (mVideoMedias == nullptr) {
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media allocation failed");
		return -ENOMEM;
	}
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		mVideoMedias[i] = new CodedVideoMedia(mDemuxer->mSession);
		if (mVideoMedias[i] == nullptr) {
			mDemuxer->Source::unlock();
			PDRAW_LOGE("media allocation failed");
			return -ENOMEM;
		}
		switch (i) {
		case 0:
			mVideoMedias[i]->format = vdef_h264_avcc;
			break;
		case 1:
			mVideoMedias[i]->format = vdef_h264_byte_stream;
			break;
		default:
			break;
		}
		ret = mDemuxer->addOutputPort(mVideoMedias[i],
					      mDemuxer->getDemuxer());
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("addOutputPort", -ret);
			return ret;
		}
		std::string path = mDemuxer->Element::getName() + "$" +
				   mVideoMedias[i]->getName();
		mVideoMedias[i]->setPath(path);
		ret = mVideoMedias[i]->setPs(nullptr,
					     0,
					     mCodecInfo.h264.sps,
					     mCodecInfo.h264.spslen,
					     mCodecInfo.h264.pps,
					     mCodecInfo.h264.ppslen);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("media->setPs", -ret);
			return ret;
		}
		mVideoMedias[i]->sessionMeta = mSessionMetaFromSdp;
		mVideoMedias[i]->playbackType =
			PDRAW_PLAYBACK_TYPE_LIVE; /* TODO: live/replay */
		mVideoMedias[i]->duration = mDemuxer->mDuration;
	}

	/* Create the H.264 reader
	 * Note: H.265 streaming is not supported */
	ret = h264_reader_new(&mH264Cbs, this, &mH264Reader);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_reader_new", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(
		mH264Reader, 0, mCodecInfo.h264.sps, mCodecInfo.h264.spslen);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("h264_reader_parse_nalu:sps", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(
		mH264Reader, 0, mCodecInfo.h264.pps, mCodecInfo.h264.ppslen);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("h264_reader_parse_nalu:pps", -ret);
		return ret;
	}

	/* Create the output buffers pool on the last media. As the medias will
	 * be destroyed in creation order, this ensures that the media which
	 * owns the buffers pool will be the last destroyed */
	ret = mDemuxer->createOutputPortMemoryPool(
		mVideoMedias[1],
		DEMUXER_STREAM_OUTPUT_BUFFER_COUNT,
		mVideoMedias[1]->info.resolution.width *
			mVideoMedias[1]->info.resolution.height * 3 / 4);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		return ret;
	}
	/* Make the pool shared between all medias */
	basePort = mDemuxer->getOutputPort(mVideoMedias[1]);
	mediaPort = mDemuxer->getOutputPort(mVideoMedias[0]);
	if (basePort == nullptr || mediaPort == nullptr) {
		PDRAW_LOGW("unable to share memory pool between medias");
	} else {
		mediaPort->pool = basePort->pool;
		mediaPort->sharedPool = true;
	}

	/* New synchronization is needed */
	resync();

	mDemuxer->Source::unlock();

	if (mDemuxer->Source::mListener) {
		for (unsigned int i = 0; i < mNbVideoMedias; i++)
			mDemuxer->Source::mListener->onOutputMediaAdded(
				mDemuxer,
				mVideoMedias[i],
				mDemuxer->getDemuxer());
	}

	/* Process any pending frames (received while the previous media
	 * was being torn down */
	while (!mTempQueue.empty()) {
		struct vstrm_frame *frame = mTempQueue.front();
		mTempQueue.pop();
		ret = processFrame(frame);
		if ((ret < 0) && (ret != -EAGAIN))
			PDRAW_LOG_ERRNO("processFrame", -ret);
		vstrm_frame_unref(frame);
	}

	return 0;
}


void StreamDemuxer::VideoMedia::teardownMedia(void)
{
	/* Destroy the H.264 reader */
	if (mH264Reader != nullptr) {
		int ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = nullptr;
	}

	/* Remove the output ports */
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		if (mDemuxer->Source::mListener) {
			mDemuxer->Source::mListener->onOutputMediaRemoved(
				mDemuxer,
				mVideoMedias[i],
				mDemuxer->getDemuxer());
		}
		int ret = mDemuxer->removeOutputPort(mVideoMedias[i]);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("removeOutputPort", -ret);
		} else {
			delete mVideoMedias[i];
		}
	}
	free(mVideoMedias);
	mVideoMedias = nullptr;
	mNbVideoMedias = 0;
}


int StreamDemuxer::VideoMedia::createReceiver(void)
{
	struct vstrm_receiver_cfg *cfg;
	std::string fn;
	std::string sn;
	std::string sv;
	int ret;

	cfg = (struct vstrm_receiver_cfg *)calloc(1, sizeof(*cfg));
	if (cfg == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto error;
	}

	/* Create the stream receiver */
	cfg->loop = mDemuxer->mSession->getLoop();
	cfg->flags = VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE |
		     VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_FRAME |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT;
	mDemuxer->mSession->getSettings()->getFriendlyName(&fn);
	strncpy(cfg->self_meta.friendly_name,
		fn.c_str(),
		sizeof(cfg->self_meta.friendly_name));
	cfg->self_meta.friendly_name[sizeof(cfg->self_meta.friendly_name) - 1] =
		'\0';
	mDemuxer->mSession->getSettings()->getSerialNumber(&sn);
	strncpy(cfg->self_meta.serial_number,
		sn.c_str(),
		sizeof(cfg->self_meta.serial_number));
	cfg->self_meta.serial_number[sizeof(cfg->self_meta.serial_number) - 1] =
		'\0';
	mDemuxer->mSession->getSettings()->getSoftwareVersion(&sv);
	strncpy(cfg->self_meta.software_version,
		sv.c_str(),
		sizeof(cfg->self_meta.software_version));
	cfg->self_meta
		.software_version[sizeof(cfg->self_meta.software_version) - 1] =
		'\0';
	ret = vstrm_receiver_new(cfg, &mReceiverCbs, this, &mReceiver);
	if (ret < 0) {
		mReceiver = nullptr;
		PDRAW_LOG_ERRNO("vstrm_receiver_new", -ret);
		goto error;
	}

	/* Provide the SPS/PPS out of band if available */
	if (mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) {
		ret = vstrm_receiver_set_codec_info(
			mReceiver, &mCodecInfo, mSsrc);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vstrm_receiver_set_codec_info", -ret);
	}

	free(cfg);
	return 0;

error:
	destroyReceiver();
	free(cfg);
	return ret;
}


int StreamDemuxer::VideoMedia::destroyReceiver(void)
{
	int res;
	if (mReceiver != nullptr) {
		/* Destroy the receiver */
		res = vstrm_receiver_destroy(mReceiver);
		if (res < 0)
			PDRAW_LOG_ERRNO("vstrm_receiver_destroy", -res);
		mReceiver = nullptr;
	}
	return 0;
}


void StreamDemuxer::VideoMedia::play(void)
{
	int err;
	/* Rearm the frame timer only when running */
	resetFrameTimer(mDemuxer->mRunning);
	err = pomp_timer_clear(mRangeTimer);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
}


void StreamDemuxer::VideoMedia::pause(void)
{
	resetFrameTimer(false);

	vstrm_receiver_clear(mReceiver);
	/* A flush is needed to discard all pending frames that will be
	 * processed when re-playing later on, leading to issues such as large
	 * rendering timing error. */
	flush();
}


void StreamDemuxer::VideoMedia::resync(void)
{
	mWaitForSync = true;
	mRecoveryFrameCount = 0;
}


void StreamDemuxer::VideoMedia::stop(void)
{
	resetFrameTimer(false);

	mDemuxer->Source::lock();

	if (mCurrentFrame != nullptr) {
		int err = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		int err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}

	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		if (mVideoMedias[i] != nullptr)
			mVideoMedias[i]->setTearingDown();
	}

	mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::flush(bool destroy)
{
	mDemuxer->Source::lock();

	mFlushing = true;
	mFlushChannelCount = 0;
	mDestroyAfterFlush = destroy;

	if (mCurrentFrame != nullptr) {
		int err = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		int err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}

	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(mVideoMedias[i]);
		mFlushChannelCount += outputChannelCount;

		/* Flush the output channels */
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			Channel *channel =
				mDemuxer->getOutputChannel(mVideoMedias[i], j);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   j);
				continue;
			}
			int err = channel->flush();
			if (err < 0)
				PDRAW_LOG_ERRNO("channel->flush", -err);
		}
	}

	mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::channelFlushed(Channel *channel)
{
	mFlushChannelCount--;
	if (mFlushChannelCount <= 0)
		mFlushing = false;
}


void StreamDemuxer::VideoMedia::channelUnlink(Channel *channel)
{
	mDemuxer->Source::lock();

	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(mVideoMedias[i]);
		if (outputChannelCount > 0) {
			mDemuxer->Source::unlock();
			return;
		}
	}

	mDemuxer->Source::unlock();

	if (mCodecInfoChanging) {
		teardownMedia();
		PDRAW_LOGI("new output media");
		mCodecInfoChanging = false;
		int ret = setupMedia();
		if (ret < 0) {
			PDRAW_LOG_ERRNO("setupMedia", -ret);
			return;
		}
	} else if (mTearingDown) {
		teardownMedia();
	}
}


void StreamDemuxer::VideoMedia::resetFrameTimer(bool rearm)
{
	int err;
	if (rearm) {
		err = pomp_timer_set(mFrameTimer,
				     DEMUXER_STREAM_FRAME_TIMEOUT_MS);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", err);
	} else {
		err = pomp_timer_clear(mFrameTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}
}


void StreamDemuxer::VideoMedia::sendDownstreamEvent(
	Channel::DownstreamEvent event)
{
	switch (event) {
	case Channel::DownstreamEvent::SOS:
		mFirstFrame = false;
		break;
	case Channel::DownstreamEvent::EOS:
		/* Always clear frame timer on EOS */
		resetFrameTimer(false);
		mFirstFrame = true;
		break;
	default:
		break;
	}
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		int res = mDemuxer->Source::sendDownstreamEvent(mVideoMedias[i],
								event);
		if (res < 0)
			PDRAW_LOG_ERRNO("Source::sendDownstreamEvent", -res);
	}
}


int StreamDemuxer::VideoMedia::processFrame(struct vstrm_frame *frame)
{
	int ret = 0, err;
	unsigned int outputChannelCount = 0;
	CodedVideoMedia::Frame data = {};
	uint32_t flags = 0;
	size_t frameSize = 0, bufSize;
	uint8_t *buf;
	bool isIdr = false;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	uint64_t remainingPlayTime = 0;
	unsigned int requiredMediaIndex;
	CodedVideoMedia *requiredMedia;
	struct vdef_coded_frame frameInfo = {};
	struct mbuf_coded_video_frame *outputFrame = nullptr;
	unsigned int sliceCount = 0;
	int64_t clock_delta = 0;
	uint32_t precision = UINT32_MAX;
	struct vstrm_video_stats_dyn *videoStatsDyn = nullptr;
	struct mbuf_ancillary_data_cbs cbs = {};

	mDemuxer->Source::lock();

	/* Get an output memory */
	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}
	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}
	std::vector<CodedVideoMedia *> codedMedias;
	for (size_t i = 0; i < mNbVideoMedias; i++)
		codedMedias.push_back(mVideoMedias[i]);
	ret = mDemuxer->getCodedVideoOutputMemory(
		codedMedias, &mCurrentMem, &requiredMediaIndex);
	if ((ret < 0) || (mCurrentMem == nullptr)) {
		mDemuxer->Source::unlock();
		PDRAW_LOGW("failed to get an output memory (%d)", ret);
		flush();
		return ret;
	}
	requiredMedia = mVideoMedias[requiredMediaIndex];
	ret = mbuf_mem_get_data(mCurrentMem, (void **)&buf, &bufSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	mCurrentFrameCaptureTs = 0;
	mCurrentMemOffset = 0;

	/* Get the size of the frame */
	flags = (requiredMedia->format.data_format ==
		 VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
			? VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE
			: VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE;
	flags |= VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS;
	ret = vstrm_frame_get_size(frame, &frameSize, flags);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_frame_get_size", -ret);
		goto out;
	}
	if (bufSize < frameSize) {
		PDRAW_LOGW("input buffer too small (%zu vs. %zu",
			   bufSize,
			   frameSize);
		ret = -EPROTO;
		goto out;
	}

	vdef_format_to_frame_info(&requiredMedia->info, &frameInfo.info);
	frameInfo.info.timestamp = frame->timestamps.ntp_raw;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.index = mFrameIndex++;
	frameInfo.format = requiredMedia->format;
	ret = mbuf_coded_video_frame_new(&frameInfo, &mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}

	/* Copy the frame */
	/* TODO: avoid copy? */
	ret = vstrm_frame_copy(frame, buf, frameSize, flags);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_frame_copy", -ret);
		goto out;
	}

	frameInfo.type = VDEF_CODED_FRAME_TYPE_I;
	for (uint32_t i = 0; i < frame->nalu_count; i++) {
		enum h264_nalu_type naluType;
		enum h264_slice_type sliceType = H264_SLICE_TYPE_UNKNOWN;
		naluType = (enum h264_nalu_type)(*frame->nalus[i].cdata & 0x1F);
		switch (naluType) {
		/* Ignored NALUs */
		case H264_NALU_TYPE_SPS:
		case H264_NALU_TYPE_PPS:
			/* Ignore SPS/PPS */
			continue;
		case H264_NALU_TYPE_SEI:
			/* SEI NAL unit */
			ret = h264_reader_parse_nalu(mH264Reader,
						     0,
						     frame->nalus[i].cdata,
						     frame->nalus[i].len);
			if (ret < 0)
				PDRAW_LOG_ERRNO("h264_reader_parse_nalu:sei",
						-ret);
			sliceType = H264_SLICE_TYPE_UNKNOWN;
			break;
		/* Slices */
		case H264_NALU_TYPE_SLICE_IDR:
			/* IDR slice */
			isIdr = true;
			frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;
			/* fallthrough */
		case H264_NALU_TYPE_SLICE:
			sliceType = /* TODO */ H264_SLICE_TYPE_UNKNOWN;
			/* TODO (coverity complains that the code can
			 * never be reached)
			 * if (sliceType == H264_SLICE_TYPE_P)
			 *   frameInfo.type = VDEF_CODED_FRAME_TYPE_P; */
			sliceCount++;
			break;
		/* Keep all other NALUs */
		default:
			break;
		}
		struct vdef_nalu nalu = {};
		nalu.size = frame->nalus[i].len + 4;
		nalu.h264.type = naluType;
		nalu.h264.slice_type = sliceType;
		/* TODO: h264.slice_mb_count */
		ret = mbuf_coded_video_frame_add_nalu(
			mCurrentFrame, mCurrentMem, mCurrentMemOffset, &nalu);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_nalu",
					-ret);
			goto out;
		}
		mCurrentMemOffset += nalu.size;
	}

	/* Ignore frames with 0 slices */
	if (sliceCount == 0) {
		ret = 0;
		PDRAW_LOGI("%s: empty frame, ignored", __func__);
		goto out;
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
	data.isSync = isIdr;
	data.isRef = frame->info.ref;
	/* TODO: use unskewed timestamps */
	data.ntpTimestamp = frame->timestamps.ntp;
	data.ntpUnskewedTimestamp = frame->timestamps.ntp_unskewed;
	data.ntpRawTimestamp = frame->timestamps.ntp_raw;
	data.ntpRawUnskewedTimestamp = frame->timestamps.ntp_raw_unskewed;
	data.captureTimestamp = mCurrentFrameCaptureTs;
	err = vstrm_receiver_get_clock_delta(
		mReceiver, &clock_delta, &precision);
	if (err < 0) {
		if (err != -EAGAIN)
			PDRAW_LOG_ERRNO("vstrm_receiver_get_clock_delta", -err);
	} else {
		data.localTimestamp = data.captureTimestamp - clock_delta;
		data.localTimestampPrecision = precision;
	}
	data.recvStartTimestamp = frame->timestamps.recv_start;
	data.recvEndTimestamp = frame->timestamps.recv_end;

	frameInfo.info.capture_timestamp = mCurrentFrameCaptureTs;

	if (frame->info.error || !frame->info.complete)
		frameInfo.info.flags |= VDEF_FRAME_FLAG_VISUAL_ERROR;
	if (mWaitForSync || mRecoveryFrameCount != 0)
		frameInfo.info.flags |= VDEF_FRAME_FLAG_SILENT;
	if (frame->info.uses_ltr)
		frameInfo.info.flags |= VDEF_FRAME_FLAG_USES_LTR;

	/* Frame metadata */
	if (frame->metadata) {
		ret = mbuf_coded_video_frame_set_metadata(mCurrentFrame,
							  frame->metadata);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_set_metadata",
					-ret);
			goto out;
		}
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	data.demuxOutputTimestamp = curTime;
	mLastFrameReceiveTime = curTime;

	if (mDemuxer->mSessionProtocol == RTSP) {
		mDemuxer->mCurrentTime =
			frame->timestamps.ntp * mDemuxer->mSpeed -
			mDemuxer->mNtpToNptOffset;
	} else {
		mDemuxer->mCurrentTime = frame->timestamps.ntp;
		if (mDemuxer->mStartTime == 0)
			mDemuxer->mStartTime = frame->timestamps.ntp;
	}
	data.playTimestamp = mDemuxer->mCurrentTime;
	if (mDemuxer->mTrackDuration > 0) {
		remainingPlayTime =
			mDemuxer->mTrackDuration - mDemuxer->mCurrentTime;
		if (remainingPlayTime < 1000000) {
			/* Less than 1s of play time remaining */

			/* Take the speed into account */
			remainingPlayTime =
				(mDemuxer->mSpeed != 0.f)
					? remainingPlayTime / mDemuxer->mSpeed
					: UINT64_MAX;

			if (remainingPlayTime < 1000000) {
				/* Add 50ms margin */
				pomp_timer_set(mRangeTimer,
					       remainingPlayTime / 1000 + 50);
			}
		}
	}

	/* Update the frame info */
	ret = mbuf_coded_video_frame_set_frame_info(mCurrentFrame, &frameInfo);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_set_frame_info", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&data,
		sizeof(data));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	/* Attach the video stats and MB status */
	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		mCurrentFrame,
		VSTRM_ANCILLARY_KEY_VIDEO_STATS,
		&frame->video_stats,
		sizeof(frame->video_stats));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}
	videoStatsDyn = (struct vstrm_video_stats_dyn *)calloc(
		1, sizeof(*videoStatsDyn));
	if (videoStatsDyn == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	ret = vstrm_video_stats_dyn_copy(videoStatsDyn,
					 &frame->video_stats_dyn);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_video_stats_dyn_copy", -ret);
		goto out;
	}
	cbs = (struct mbuf_ancillary_data_cbs){
		.cleaner = &videoStatsDynCleaner,
		.cleaner_userdata = videoStatsDyn,
	};
	ret = mbuf_coded_video_frame_add_ancillary_buffer_with_cbs(
		mCurrentFrame,
		VSTRM_ANCILLARY_KEY_VIDEO_STATS_DYN,
		&videoStatsDyn,
		sizeof(videoStatsDyn),
		&cbs);
	if (ret < 0) {
		PDRAW_LOG_ERRNO(
			"mbuf_coded_video_frame_add_ancillary_buffer_with_cbs",
			-ret);
		goto out;
	}
	/* videoStatsDyn will be freed by the videoStatsDynCleaner */
	videoStatsDyn = nullptr;
	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		mCurrentFrame,
		VSTRM_ANCILLARY_KEY_MB_STATUS,
		frame->info.mb_status,
		frame->info.mb_total * sizeof(uint8_t));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_finalize(mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

	/* Queue the buffer in the output channels */
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		outputChannelCount =
			mDemuxer->getOutputChannelCount(mVideoMedias[i]);
		if (outputChannelCount == 0)
			continue;
		if (outputFrame != nullptr)
			mbuf_coded_video_frame_unref(outputFrame);
		outputFrame = mCurrentFrame;
		if (!vdef_coded_format_cmp(&requiredMedia->format,
					   &mVideoMedias[i]->format)) {
			/* The format is different, we need to pick another
			 * frame */
			int copy_ret = mDemuxer->copyCodedVideoOutputFrame(
				requiredMedia,
				mCurrentFrame,
				mVideoMedias[i],
				&outputFrame);
			if (copy_ret < 0) {
				PDRAW_LOG_ERRNO("copyOutputFrame", -copy_ret);
				outputFrame = nullptr;
				continue;
			}
		} else {
			mbuf_coded_video_frame_ref(outputFrame);
		}
		ret = mbuf_coded_video_frame_get_frame_info(outputFrame,
							    &frameInfo);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
					-ret);
			goto out;
		}
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			const struct vdef_coded_format *caps;
			int capsCount;

			Channel *c =
				mDemuxer->getOutputChannel(mVideoMedias[i], j);
			CodedVideoChannel *channel =
				dynamic_cast<CodedVideoChannel *>(c);
			if (channel == nullptr) {
				PDRAW_LOGW("invalid channel");
				continue;
			}

			capsCount =
				channel->getCodedVideoMediaFormatCaps(&caps);
			if (capsCount < 0) {
				PDRAW_LOGW("invalid channel (no caps)");
				continue;
			}

			if (!vdef_coded_format_intersect(
				    &frameInfo.format, caps, capsCount)) {
				PDRAW_LOGW(
					"incompatible coded video format "
					"on channel");
				continue;
			}

			int queue_ret = channel->queue(outputFrame);
			if (queue_ret < 0)
				PDRAW_LOG_ERRNO("channel->queue", -queue_ret);
		}
	}
	if (outputFrame != nullptr)
		mbuf_coded_video_frame_unref(outputFrame);
	if ((mFirstFrame) &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
	}

out:
	mbuf_mem_unref(mCurrentMem);
	mCurrentMem = nullptr;
	mbuf_coded_video_frame_unref(mCurrentFrame);
	mCurrentFrame = nullptr;
	free(videoStatsDyn);

	mDemuxer->Source::unlock();
	return ret;
}


void StreamDemuxer::VideoMedia::channelSendVideoPresStats(Channel *channel,
							  VideoPresStats *stats)
{
	vstrm_video_stats vstrm_stats = {};

	vstrm_stats.version = VSTRM_VIDEO_STATS_VERSION_2;
	vstrm_stats.timestamp = stats->timestamp;
	vstrm_stats.v2.presentation_frame_count = stats->presentationFrameCount;
	vstrm_stats.v2.presentation_timestamp_delta_integral =
		stats->presentationTimestampDeltaIntegral;
	vstrm_stats.v2.presentation_timestamp_delta_integral_sq =
		stats->presentationTimestampDeltaIntegralSq;
	vstrm_stats.v2.presentation_timing_error_integral =
		stats->presentationTimingErrorIntegral;
	vstrm_stats.v2.presentation_timing_error_integral_sq =
		stats->presentationTimingErrorIntegralSq;
	vstrm_stats.v2.presentation_estimated_latency_integral =
		stats->presentationEstimatedLatencyIntegral;
	vstrm_stats.v2.presentation_estimated_latency_integral_sq =
		stats->presentationEstimatedLatencyIntegralSq;
	vstrm_stats.v2.player_latency_integral = stats->playerLatencyIntegral;
	vstrm_stats.v2.player_latency_integral_sq =
		stats->playerLatencyIntegralSq;
	vstrm_stats.v2.estimated_latency_precision_integral =
		stats->estimatedLatencyPrecisionIntegral;

	int err = vstrm_receiver_set_video_stats(mReceiver, &vstrm_stats);
	if (err < 0)
		PDRAW_LOG_ERRNO("vstrm_receiver_set_video_stats", -err);
}


void StreamDemuxer::VideoMedia::sessionMetadataFromSdp(
	const struct sdp_media *media,
	const struct vmeta_session *sessionMeta,
	struct vmeta_session *meta)
{
	int err;
	struct sdp_attr *attr = nullptr;

	*meta = *sessionMeta;

	if (media->media_title != nullptr) {
		err = vmeta_session_streaming_sdp_read(
			VMETA_STRM_SDP_TYPE_MEDIA_INFO,
			media->media_title,
			nullptr,
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


void StreamDemuxer::VideoMedia::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	int ret = 0;

	if (self == nullptr)
		return;
	if ((buf == nullptr) || (len == 0))
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	/* Ignore "Parrot Streaming" user data SEI */
	if (vstrm_h264_is_sei_streaming(sei->uuid))
		return;

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		self->mCurrentFrame, MBUF_ANCILLARY_KEY_USERDATA_SEI, buf, len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}
}


void StreamDemuxer::VideoMedia::h264PicTimingSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_pic_timing *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	self->mCurrentFrameCaptureTs = h264_ctx_sei_pic_timing_to_us(ctx, sei);
}


void StreamDemuxer::VideoMedia::h264RecoveryPointSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_recovery_point *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	if (self->mWaitForSync) {
		self->mWaitForSync = false;
		self->mRecoveryFrameCount = sei->recovery_frame_cnt + 1;
	}
}


int StreamDemuxer::VideoMedia::sendCtrlCb(struct vstrm_receiver *stream,
					  struct tpkt_packet *pkt,
					  void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return -EINVAL;

	return self->sendCtrl(stream, pkt);
}


void StreamDemuxer::VideoMedia::codecInfoChangedCb(
	struct vstrm_receiver *stream,
	const struct vstrm_codec_info *info,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	int outputChannelCount = 0;
	Channel *channel;
	int ret;

	if ((self == nullptr) || (info == nullptr))
		return;
	if (info->codec != VSTRM_CODEC_VIDEO_H264) {
		PDRAW_LOG_ERRNO("info->codec", EPROTO);
		return;
	}

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return;
	}

	PDRAW_LOGD("codec info changed");
	self->mWaitForCodecInfo = false;

	if ((!self->mCodecInfoChanging) &&
	    (!memcmp(&self->mCodecInfo, info, sizeof(self->mCodecInfo)))) {
		PDRAW_LOGI("codec info changed; no change in PS, just resync");
		self->resync();
		return;
	}
	self->mCodecInfo = *info;

	demuxer->Source::lock();

	if (self->mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(self->mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
		self->mCurrentFrame = nullptr;
	}
	if (self->mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(self->mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		self->mCurrentMem = nullptr;
	}

	if (self->mNbVideoMedias > 0) {
		PDRAW_LOGI("change of output media");
		self->mCodecInfoChanging = true;
		for (unsigned int i = 0; i < self->mNbVideoMedias; i++) {
			outputChannelCount = demuxer->getOutputChannelCount(
				self->mVideoMedias[i]);

			/* Teardown the output channels
			 * Note: loop downwards because calling teardown on a
			 * channel may or may not synchronously remove the
			 * channel from the output port */
			for (int j = outputChannelCount - 1; j >= 0; j--) {
				channel = demuxer->getOutputChannel(
					self->mVideoMedias[i], j);
				if (channel == nullptr) {
					PDRAW_LOGW(
						"failed to get channel "
						"at index %d",
						j);
					continue;
				}
				ret = channel->teardown();
				if (ret < 0)
					PDRAW_LOG_ERRNO("channel->teardown",
							-ret);
			}
		}
	} else {
		PDRAW_LOGI("new output media");
		self->mCodecInfoChanging = false;
		ret = self->setupMedia();
		if (ret < 0) {
			demuxer->Source::unlock();
			PDRAW_LOG_ERRNO("setupMedia", -ret);
			return;
		}
	}

	demuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::recvFrameCb(struct vstrm_receiver *stream,
					    struct vstrm_frame *frame,
					    void *userdata)
{
	int err;
	VideoMedia *self = (VideoMedia *)userdata;

	if ((self == nullptr) || (frame == nullptr))
		return;

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED)
		return;

	if (demuxer->mRunning) {
		self->resetFrameTimer(true);
	} else if (demuxer->mSessionProtocol != RTSP) {
		/* just ignore the frames */
		return;
	}

	if (self->mTearingDown) {
		/* Media is tearing down, ignore frames */
		return;
	}

	if (self->mCodecInfoChanging) {
		/* Keep at most DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE frames in the
		 * tempQueue, discard the oldest frame. */
		if (self->mTempQueue.size() >=
		    DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE) {
			struct vstrm_frame *tempFrame =
				self->mTempQueue.front();
			PDRAW_LOGW("temp queue is full, dropping frame");
			self->mTempQueue.pop();
			vstrm_frame_unref(tempFrame);
		}
		/* Queue the frame to process later */
		vstrm_frame_ref(frame);
		self->mTempQueue.push(frame);
		return;
	}

	if ((self->mWaitForCodecInfo) || (self->mFlushing))
		return;

	/* Process the incoming frame */
	err = self->processFrame(frame);
	if (err < 0) {
		if (err != -EAGAIN)
			PDRAW_LOG_ERRNO("processFrame", -err);
		return;
	}
}


void StreamDemuxer::VideoMedia::sessionMetadataPeerChangedCb(
	struct vstrm_receiver *stream,
	const struct vmeta_session *meta,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if ((self == nullptr) || (meta == nullptr))
		return;

	PDRAW_LOGD("session metadata changed");

	self->mDemuxer->Source::lock();

	for (unsigned int i = 0; i < self->mNbVideoMedias; i++) {
		self->mVideoMedias[i]->sessionMeta = *meta;
		/* FIXME: to be discussed */
		self->mSessionMetaFromSdp = *meta;
		int err = self->mDemuxer->sendDownstreamEvent(
			self->mVideoMedias[i],
			Channel::DownstreamEvent::SESSION_META_UPDATE);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}

	self->mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::eventCb(struct vstrm_receiver *stream,
					enum vstrm_event event,
					void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	Channel::DownstreamEvent evt;
	bool sendEvent = false;

	if (self == nullptr)
		return;

	PDRAW_LOGI("received custom RTCP event '%s'",
		   vstrm_event_to_str(event));

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED)
		return;

	switch (event) {
	case VSTRM_EVENT_RECONFIGURE:
		evt = Channel::DownstreamEvent::RECONFIGURE;
		sendEvent = true;
		break;
	case VSTRM_EVENT_RESOLUTION_CHANGE:
		evt = Channel::DownstreamEvent::RESOLUTION_CHANGE;
		sendEvent = true;
		break;
	case VSTRM_EVENT_FRAMERATE_CHANGE:
		evt = Channel::DownstreamEvent::FRAMERATE_CHANGE;
		sendEvent = true;
		break;
	case VSTRM_EVENT_PHOTO_TRIGGER:
		evt = Channel::DownstreamEvent::PHOTO_TRIGGER;
		sendEvent = true;
		break;
	default:
		break;
	}

	if (sendEvent) {
		demuxer->Source::lock();
		self->sendDownstreamEvent(evt);
		demuxer->Source::unlock();
	}
}


void StreamDemuxer::VideoMedia::goodbyeCb(struct vstrm_receiver *stream,
					  const char *reason,
					  void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	Channel::DownstreamEvent event;
	bool sendEvent = false;

	if (self == nullptr)
		return;

	PDRAW_LOGI("received RTCP goodbye%s%s",
		   reason ? ", reason: " : "",
		   reason ? reason : "");

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED)
		return;

	self->resetFrameTimer(false);

	/* Wait for new codec info */
	self->mWaitForCodecInfo = true;

	if (reason != nullptr) {
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
			if (demuxer->mSessionProtocol == RTSP &&
			    (strcmp(reason,
				    DEMUXER_STREAM_GOODBYE_REASON_USER) ||
			     (demuxer->mState != STOPPING &&
			      demuxer->mState != STOPPED))) {
				/* We either received an unknown RTCP goodbye
				 * packet, or an unexpected (not initiated by a
				 * teardown) user_disconnection packet. Notify
				 * the application of an unrecoverable error
				 * only when a single media is selected. */
				unsigned int count = 0;
				for (auto p = demuxer->mVideoMedias.begin();
				     p != demuxer->mVideoMedias.end();
				     p++) {
					if ((*p)->isTearingDown())
						continue;
					count++;
				}
				if (count == 1)
					demuxer->onUnrecoverableError();
			}
		}

		if (sendEvent) {
			demuxer->Source::lock();
			self->sendDownstreamEvent(event);
			demuxer->Source::unlock();
		}
	}
}


void StreamDemuxer::VideoMedia::frameTimeoutCb(struct pomp_timer *timer,
					       void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	if (self == nullptr)
		return;

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		PDRAW_LOG_ERRNO("time_get_monotonic", -res);
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0)
		PDRAW_LOG_ERRNO("time_timespec_to_us", -res);

	demuxer->Source::lock();
	if (curTime >
	    self->mLastFrameReceiveTime + DEMUXER_STREAM_FRAME_TIMEOUT_MS) {
		self->sendDownstreamEvent(Channel::DownstreamEvent::TIMEOUT);
	}
	demuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::rangeTimerCb(struct pomp_timer *timer,
					     void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return;

	StreamDemuxer *demuxer = self->mDemuxer;

	if (!demuxer->mEndOfRangeNotified) {
		PDRAW_LOGI("end of range reached");
		self->sendDownstreamEvent(Channel::DownstreamEvent::EOS);
		demuxer->onEndOfRange(demuxer->mCurrentTime);
		demuxer->mEndOfRangeNotified = true;
	}
}


void StreamDemuxer::idleEndOfRangeNotification(void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (self == nullptr)
		return;

	if (!self->mEndOfRangeNotified) {
		auto p = self->mVideoMedias.begin();
		while (p != self->mVideoMedias.end()) {
			(*p)->sendDownstreamEvent(
				Channel::DownstreamEvent::EOS);
			p++;
		}

		PDRAW_LOGI("end of range reached");
		self->onEndOfRange(self->mCurrentTime);
		self->mEndOfRangeNotified = true;
	}
}


void StreamDemuxer::videoStatsDynCleaner(struct mbuf_ancillary_data *data,
					 void *userdata)
{
	struct vstrm_video_stats_dyn *dyn =
		reinterpret_cast<struct vstrm_video_stats_dyn *>(userdata);
	if (dyn == nullptr)
		return;
	vstrm_video_stats_dyn_clear(dyn);
	free(dyn);
}

} /* namespace Pdraw */
