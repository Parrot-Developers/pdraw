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
#include <memory>
#include <string>

#include <futils/futils.h>
#include <media-buffers/mbuf_ancillary_data.h>
#include <rtp/rtp.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


constexpr size_t DEMUXER_STREAM_FRAME_TIMEOUT_MS = 2000;

constexpr const char *DEMUXER_STREAM_GOODBYE_REASON_USER = "user disconnection";
constexpr const char *DEMUXER_STREAM_GOODBYE_REASON_RECONFIGURE =
	"configuration change";
constexpr const char *DEMUXER_STREAM_GOODBYE_REASON_PHOTO_TRIGGER =
	"photo trigger";


const struct rtsp_client_cbs StreamDemuxer::mRtspClientCbs = {
	.socket_cb = &StreamDemuxer::onRtspSocketCreated,
	.ready_to_send_cb = nullptr,
	.interleaved_data_cb = &StreamDemuxer::onRtspInterleavedDataCb,
	.connection_state = &StreamDemuxer::onRtspConnectionState,
	.session_removed = &StreamDemuxer::onRtspSessionRemoved,
	.options_resp = &StreamDemuxer::onRtspOptionsResp,
	.describe_resp = &StreamDemuxer::onRtspDescribeResp,
	.announce_resp = nullptr,
	.setup_resp = &StreamDemuxer::onRtspSetupResp,
	.play_resp = &StreamDemuxer::onRtspPlayResp,
	.pause_resp = &StreamDemuxer::onRtspPauseResp,
	.record_resp = nullptr,
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
			params)
{
	Element::setClassName(__func__);
	mRtspDisconnectHandler.set([this] { idleRtspDisconnect(); });
	mEndOfRangeNotificationHandler.set(
		[this] { idleEndOfRangeNotification(); });
	mCompleteTeardownHandler.set([this] { idleCompleteTeardown(); });
}


StreamDemuxer::~StreamDemuxer()
{
	int ret;

	if (mState != State::STOPPED && mState != State::CREATED)
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
	ret = mSession->getPompLoop()->idleRemove(this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -ret);
}


void StreamDemuxer::sessionMetadataFromSdp(const struct sdp_session *session,
					   struct vmeta_session *meta)
{
	int err;
	const struct sdp_attr *attr = nullptr;

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


int StreamDemuxer::processRtspRequests()
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


void StreamDemuxer::cleanupRtspRequests()
{
	while (!mSetupRequests.empty()) {
		SetupRequest req = mSetupRequests.front();
		mSetupRequests.pop();
		mSetupRequestsCount--;
	}

	while (!mTeardownRequests.empty()) {
		TeardownRequest req = mTeardownRequests.front();
		mTeardownRequests.pop();
		mTeardownRequestsCount--;
	}
}


int StreamDemuxer::processSetupRequest()
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
				mContentBase.c_str(),
				req.controlUrl.c_str(),
				mRtspSessionId.c_str(),
				RTSP_DELIVERY_UNICAST,
				req.lowerTransport,
				req.localStreamPort,
				req.localControlPort,
				RTSP_TRANSPORT_METHOD_UNKNOWN,
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
	return res;
}


int StreamDemuxer::processTeardownRequest()
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
				   req.controlUrl.c_str(),
				   mRtspSessionId.c_str(),
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
	return res;
}


void StreamDemuxer::teardownVideoMedia(StreamDemuxer::VideoMedia *media)
{
	PDRAW_LOG_ERRNO_RETURN_IF(media == nullptr, EINVAL);

	if (media->isTearingDown())
		return;

	media->sendDownstreamEvent(Channel::DownstreamEvent::EOS);
	media->setTearingDown();
	media->setDestroyAfterFlush(true);
	media->flush();
	asyncCompleteTeardown();
}


void StreamDemuxer::teardownAllVideoMedias()
{
	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		teardownVideoMedia(m.get());
	}
}


void StreamDemuxer::destroyAllVideoMedias()
{
	mVideoMedias.clear();
}


void StreamDemuxer::onRtspSocketCreated(int fd, void *userdata)
{
	auto *self = static_cast<StreamDemuxer *>(userdata);

	const struct rtsp_url *url =
		rtsp_client_get_remote_url(self->mRtspClient);
	if (url != nullptr) {
		const char *resolved_host = rtsp_url_get_resolved_host(url);
		if (resolved_host != nullptr &&
		    !self->mUrl->hasResolvedHost()) {
			self->mUrl->setResolvedHost(resolved_host);
			self->mRemoteAddr = resolved_host;
			ULOGW("set remote addr to %s",
			      self->mRemoteAddr.c_str());
		}
	}

	self->mSession->socketCreated(fd);
}


void StreamDemuxer::onRtspInterleavedDataCb(
	[[maybe_unused]] struct rtsp_client *client,
	uint8_t channel,
	const uint8_t *data,
	size_t len,
	void *userdata)
{
	int res;
	const auto *self = static_cast<StreamDemuxer *>(userdata);
	struct pomp_buffer *buf = nullptr;
	struct tpkt_packet *pkt = nullptr;
	bool found = false;

	buf = pomp_buffer_new_with_data(data, len);
	if (buf == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_buffer_new_with_data", -res);
		return;
	}

	res = tpkt_new_from_buffer(buf, &pkt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_new_with_data", -res);
		pomp_buffer_unref(buf);
		return;
	}

	for (const auto &m : self->mVideoMedias) {
		if (m->getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP)
			continue;
		if (channel == m->getRemoteStreamPort()) {
			found = true;
			res = m->processDataPkt(pkt);
			if (res < 0)
				PDRAW_LOG_ERRNO("processDataPkt", -res);
			break;
		}
		if (channel == m->getRemoteControlPort()) {
			found = true;
			res = m->processCtrlPkt(pkt);
			if (res < 0)
				PDRAW_LOG_ERRNO("processCtrlPkt", -res);
			break;
		}
	}

	if (!found)
		PDRAW_LOGW("dropping pkt from unknown channel (%d)", channel);

	tpkt_unref(pkt);
	pomp_buffer_unref(buf);
}


void StreamDemuxer::onRtspConnectionState(
	[[maybe_unused]] struct rtsp_client *client,
	enum rtsp_client_conn_state state,
	void *userdata)
{

	int err = 0;
	auto *self = static_cast<StreamDemuxer *>(userdata);

	/* Reset flag */
	self->mDescribeRetried = false;

	PDRAW_LOGI("RTSP client %s", rtsp_client_conn_state_str(state));

	switch (state) {
	case RTSP_CLIENT_CONN_STATE_DISCONNECTED:
		self->setRtspState(RtspState::DISCONNECTED);
		for (const auto &m : self->mVideoMedias) {
			if (m->isTearingDown())
				continue;
			m->resetFrameTimer(false);
		}
		self->mRunning = false;
		self->mNetworkReadyForStop = true;

		if (self->mState == State::STOPPING)
			self->tryCompleteStop();
		break;
	case RTSP_CLIENT_CONN_STATE_CONNECTED:
		/* If previous RTSP state is not DISCONNECTED, do not
		 * send OPTIONS request */
		if (self->mRtspState != RtspState::DISCONNECTED)
			break;
		self->setRtspState(RtspState::CONNECTED);

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
		if (self->mState == State::STOPPING)
			self->asyncRtspDisconnect();
		break;
	default:
		PDRAW_LOGW("unhandled RTSP connection state: (%d: %s)",
			   state,
			   rtsp_client_conn_state_str(state));
		break;
	}
}


void StreamDemuxer::onRtspSessionRemoved(
	[[maybe_unused]] struct rtsp_client *client,
	const char *session_id,
	int status,
	void *userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);

	if (xstrcmp(session_id, self->mRtspSessionId.c_str()) != 0) {
		PDRAW_LOGD("wrong session removed (%s, expected %s)",
			   session_id,
			   self->mRtspSessionId.c_str());
		return;
	} else {
		self->mRtspSessionId.clear();
	}

	ULOG_EVT("STREAM",
		 "event='client_session_removed';element='%s';"
		 "status=%d;status_str='%s';session='%s';res='%s'",
		 self->getCName(),
		 status,
		 strerror(-status),
		 session_id ? session_id : "",
		 self->mUrl->getStreamName().c_str());

	self->teardownAllVideoMedias();

	if (self->mRtspState != RtspState::DISCONNECTED)
		self->setRtspState(RtspState::OPTIONS_DONE);

	if (self->mState == State::STOPPING) {
		if (self->mRtspState != RtspState::DISCONNECTED)
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


void StreamDemuxer::onRtspOptionsResp(
	[[maybe_unused]] struct rtsp_client *client,
	enum rtsp_client_req_status req_status,
	int status,
	[[maybe_unused]] uint32_t methods,
	[[maybe_unused]] const struct rtsp_header_ext *ext,
	[[maybe_unused]] size_t ext_count,
	void *userdata,
	[[maybe_unused]] void *req_userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
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
			 self->mUrl->getStreamName().c_str());

		self->onUnrecoverableError(res);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_options_resp';element='%s';"
		 "status=%d;status_str='%s';res='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 self->mUrl->getStreamName().c_str());

	self->setRtspState(RtspState::OPTIONS_DONE);

	res = self->sendDescribe();
	if (res < 0)
		PDRAW_LOG_ERRNO("sendDescribe", -res);
}


int StreamDemuxer::sendDescribe()
{
	int res = rtsp_client_describe(mRtspClient,
				       mUrl->getStreamName().c_str(),
				       nullptr,
				       0,
				       nullptr,
				       RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0)
		PDRAW_LOG_ERRNO("rtsp_client_describe", -res);
	return res;
}


void StreamDemuxer::onRtspDescribeResp(
	[[maybe_unused]] struct rtsp_client *client,
	enum rtsp_client_req_status req_status,
	int status,
	const char *content_base,
	[[maybe_unused]] const struct rtsp_header_ext *ext,
	[[maybe_unused]] size_t ext_count,
	const char *sdp,
	void *userdata,
	[[maybe_unused]] void *req_userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
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
			 self->mUrl->getStreamName().c_str());

		if (status == -EPERM && !self->mDescribeRetried) {
			res = self->sendDescribe();
			if (res < 0)
				PDRAW_LOG_ERRNO("sendDescribe", -res);
			self->mDescribeRetried = true;
		}

		self->onUnrecoverableError(res);
		return;
	}

	ULOG_EVT("STREAM",
		 "event='client_describe_resp';element='%s';"
		 "status=%d;status_str='%s';res='%s'",
		 self->getCName(),
		 res,
		 strerror(-res),
		 self->mUrl->getStreamName().c_str());

	self->onNewSdp(content_base, sdp);
}


void StreamDemuxer::onRtspSetupResp([[maybe_unused]] struct rtsp_client *client,
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

	auto *self = static_cast<StreamDemuxer *>(userdata);
	auto *media = static_cast<VideoMedia *>(req_userdata);
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
			 self->mUrl->getStreamName().c_str(),
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

	self->mRtspSessionId = session_id;

	if (media != nullptr) {
		media->setSsrc(ssrc_valid ? ssrc : 0);
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
		 self->mUrl->getStreamName().c_str(),
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

	/* Wait for all SETUP requests to be processed before changing
	 * state/openResp/readyToPlay */
	if (self->mSetupRequests.empty()) {
		self->setRtspState(RtspState::SETUP_DONE);

		if (!self->mCalledOpenResp)
			self->openResponse(0);

		self->readyToPlay(true);
	}

	if (self->mRunning && media != nullptr) {
		/* Play newly setup medias */
		media->play();
		self->internalPlay(self->mSpeed);
	}

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspPlayResp([[maybe_unused]] struct rtsp_client *client,
				   const char *session_id,
				   enum rtsp_client_req_status req_status,
				   int status,
				   const struct rtsp_range *range,
				   float scale,
				   [[maybe_unused]] int seq_valid,
				   uint16_t seq,
				   int rtptime_valid,
				   uint32_t rtptime,
				   const struct rtsp_header_ext *ext,
				   size_t ext_count,
				   void *userdata,
				   [[maybe_unused]] void *req_userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
	uint64_t start = 0;
	uint64_t stop = 0;
	uint64_t ntptime = 0;
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
			 self->mUrl->getStreamName().c_str(),
			 start,
			 stop,
			 rtptime,
			 seq);

		if (self->mSeeking) {
			if (self->getPendingCommand() == Command::PAUSE_NEXT) {
				self->pauseResponse(status, self->mCurrentTime);
			} else {
				/* In case of error, call seekResponse
				 * immediately */
				self->seekResponse(status,
						   self->mCurrentTime,
						   self->mSpeed);
			}
		} else {
			self->playResponse(
				status, self->mCurrentTime, self->mSpeed);
		}
		self->mSeeking = false;
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId.c_str()) != 0) {
		PDRAW_LOGE(
			"RTSP play response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId.c_str());
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
		 self->mUrl->getStreamName().c_str(),
		 start,
		 stop,
		 rtptime,
		 seq);

	for (const auto &m : self->mVideoMedias) {
		if (m->isTearingDown())
			continue;
		m->onPlayComplete();
	}

	self->mSpeed = (scale != 0.) ? scale : 1.0;
	if (rtptime_valid && (self->mRtpClockRate != 0)) {
		ntptime = rtp_timestamp_to_us(rtptime, self->mRtpClockRate);
	}
	self->mPlayNtpTime = ntptime;
	self->mNtpToNptOffset =
		static_cast<int64_t>(static_cast<double>(ntptime) *
				     static_cast<double>(self->mSpeed)) -
		static_cast<int64_t>(start);
	self->mPausePoint = stop;
	if (self->mSeeking) {
		self->mSeekResponse = status;
		self->mSeekingNetwork = false;
		self->onMediaSeekComplete(status);
		/* seekResponse is async */
	} else {
		/* Play can be internal, in that case don't call playResp */
		if (self->getPendingCommand() == Command::PLAY)
			self->playResponse(0, start, self->mSpeed);
	}

	if ((start == stop) && start && stop) {
		/* The end of range is reached */
		res = self->mSession->getPompLoop()->idleAdd(
			&self->mEndOfRangeNotificationHandler, self);
		if (res < 0)
			PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -res);
	}

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspPauseResp([[maybe_unused]] struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    const struct rtsp_range *range,
				    const struct rtsp_header_ext *ext,
				    size_t ext_count,
				    void *userdata,
				    [[maybe_unused]] void *req_userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
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
			 self->mUrl->getStreamName().c_str(),
			 start);

		self->pauseResponse(status, self->mCurrentTime);
		return;
	}

	if (xstrcmp(session_id, self->mRtspSessionId.c_str()) != 0) {
		PDRAW_LOGE(
			"RTSP pause response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId.c_str());
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
		 self->mUrl->getStreamName().c_str(),
		 start);

	for (const auto &m : self->mVideoMedias) {
		if (m->isTearingDown())
			continue;
		m->onPauseComplete();
	}

	self->mPausePoint = start;

	if ((self->getPendingCommand() == Command::PAUSE_NEXT) &&
	    (self->getDuration() != 0)) {
		self->next();
	} else {
		self->pauseResponse(0, self->mPausePoint);
	}

	res = self->processRtspRequests();
	if (res < 0) {
		if (res != -EBUSY)
			self->onUnrecoverableError(res);
		return;
	}
}


void StreamDemuxer::onRtspTeardownResp(
	[[maybe_unused]] struct rtsp_client *client,
	const char *session_id,
	enum rtsp_client_req_status req_status,
	int status,
	const struct rtsp_header_ext *ext,
	size_t ext_count,
	void *userdata,
	void *req_userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
	auto *media = static_cast<VideoMedia *>(req_userdata);
	int res = 0;
	const char *proxy_session = nullptr;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}

	auto it = std::find_if(self->mVideoMedias.begin(),
			       self->mVideoMedias.end(),
			       [media](const std::unique_ptr<VideoMedia> &m) {
				       return m.get() == media;
			       });

	if (it == self->mVideoMedias.end()) {
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

	if (xstrcmp(session_id, self->mRtspSessionId.c_str()) != 0) {
		PDRAW_LOGE(
			"RTSP teardown response for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId.c_str());
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
		 self->mUrl->getStreamName().c_str(),
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


void StreamDemuxer::onRtspAnnounce(
	[[maybe_unused]] struct rtsp_client *client,
	const char *content_base,
	[[maybe_unused]] const struct rtsp_header_ext *ext,
	[[maybe_unused]] size_t ext_count,
	const char *sdp,
	void *userdata)
{
	ULOG_ERRNO_RETURN_IF(content_base == nullptr, EINVAL);

	auto *self = static_cast<StreamDemuxer *>(userdata);

	if (self->mContentBase.empty() || self->mContentBase != content_base)
		return;

	const char *resource = nullptr;
	size_t len = strlen(content_base);
	if (len > 7) {
		const char *p = strchr(content_base + 7, '/');
		if (p && *(p + 1) != '\0')
			resource = p + 1;
	}
	ULOG_EVT("STREAM",
		 "event='client_announce';element='%s';res='%s'",
		 self->getCName(),
		 resource ? resource : "");

	self->onNewSdp(content_base, sdp);
}


void StreamDemuxer::onRtspForcedTeardown(
	[[maybe_unused]] struct rtsp_client *client,
	const char *path,
	const char *session_id,
	const struct rtsp_header_ext *ext,
	size_t ext_count,
	void *userdata)
{

	auto *self = static_cast<StreamDemuxer *>(userdata);
	VideoMedia *media = nullptr;
	int res = 0;
	const char *proxy_session = nullptr;
	bool pathIsContentBase = false;

	for (size_t i = 0; i < ext_count; i++) {
		if (strcasecmp(ext[i].key,
			       RTSP_HEADER_EXT_PARROT_PROXY_SESSION) == 0) {
			proxy_session = ext[i].value;
			break;
		}
	}

	if (xstrcmp(session_id, self->mRtspSessionId.c_str()) != 0) {
		PDRAW_LOGE(
			"RTSP forced teardown for a wrong session"
			" (%s instead of %s)",
			session_id,
			self->mRtspSessionId.c_str());
		return;
	}

	if (xstrcmp(path, self->mShortContentBase.c_str()) == 0) {
		pathIsContentBase = true;
	} else {
		for (const auto &m : self->mVideoMedias) {
			std::string mediaPath = self->mShortContentBase + "/" +
						std::string(m->getControlUrl());
			if (m->isTearingDown())
				continue;
			if (xstrcmp(path, mediaPath.c_str()) == 0) {
				media = m.get();
				break;
			}
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
		 self->mUrl->getStreamName().c_str(),
		 media ? media->getControlUrl()
		       : (pathIsContentBase ? "all" : "?"));

	if (pathIsContentBase) {
		for (auto p = self->mVideoMedias.begin();
		     p != self->mVideoMedias.end();
		     p++) {
			if ((*p)->isTearingDown())
				continue;
			self->teardownVideoMedia(p->get());
		}
	} else if (media != nullptr) {
		self->teardownVideoMedia(media);
	}
}


void StreamDemuxer::asyncRtspDisconnect()
{
	int err =
		mSession->getPompLoop()->idleAdd(&mRtspDisconnectHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void StreamDemuxer::idleRtspDisconnect()
{
	int res;

	if (mRtspState == RtspState::DISCONNECTED)
		return;

	res = rtsp_client_disconnect(mRtspClient);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_disconnect", -res);
		return;
	}
}


int StreamDemuxer::startRtsp()
{
	int res;

	if (mRtspClient != nullptr) {
		res = -EBUSY;
		PDRAW_LOG_ERRNO("mRtspClient", -res);
		return res;
	}

	/* Ensure URL is valid */
	if (mUrl == nullptr)
		return -EINVAL;

	mSessionProtocol = SessionProtocol::RTSP;

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

	res = rtsp_client_set_socket_class_selector(mRtspClient,
						    IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW_ERRNO("rtsp_client_set_socket_class_selector", -res);

	/* TODO: set:
	 * rtsp_client_set_socket_rxbuf_size
	 * rtsp_client_set_socket_txbuf_size
	 * (useful in TCP/interleaved mode)
	 */

	res = rtsp_client_connect(mRtspClient,
				  mUrl->getBaseUrlWithAuth().c_str());
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_connect", -res);
		return res;
	}

	return 0;
}


int StreamDemuxer::start()
{
	int res;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: demuxer is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	if (mUrl != nullptr) {
		switch (mUrl->getScheme()) {
		case RTSP_URL_SCHEME_TCP:
		case RTSP_URL_SCHEME_TCP_TLS:
			res = startRtsp();
			if (res < 0) {
				PDRAW_LOG_ERRNO("startRtsp", -res);
				return res;
			}
			break;
		case RTSP_URL_SCHEME_UDP:
		default:
			PDRAW_LOGE("unsupported URL scheme: %s",
				   rtsp_url_scheme_str(mUrl->getScheme()));
			return -ENOSYS;
		}

		/* Do not call openResponse now, it will be called when
		 * receiving a response to the RTSP SETUP request. */
		tryCompleteStart(false);
	} else {
		std::unique_ptr<VideoMedia> noUrlMedia =
			createVideoMedia(RTSP_LOWER_TRANSPORT_UDP);
		if (noUrlMedia == nullptr) {
			PDRAW_LOGE("failed to create VideoMedia");
			return -ENOMEM;
		}

		res = noUrlMedia->setup(nullptr);
		if (res < 0) {
			PDRAW_LOG_ERRNO("VideoMedia::setup", -res);
			return res;
		}

		mVideoMedias.push_back(std::move(noUrlMedia));

		res = mVideoMedias.front()->startRtpAvp();
		if (res < 0) {
			PDRAW_LOG_ERRNO("startRtpAvp", -res);
			return res;
		}

		tryCompleteStart();
		readyToPlay(true);
	}

	return 0;
}


int StreamDemuxer::stopNetwork()
{
	int ret;
	bool disconnect = false;

	if (mSessionProtocol != SessionProtocol::RTSP) {
		mRunning = false;
		mNetworkReadyForStop = true;
		for (const auto &m : mVideoMedias)
			m->setTearingDown();
		return 0;
	}

	if (mRtspState == RtspState::SETUP_DONE) {
		for (const auto &m : mVideoMedias) {
			m->teardown();
			if (mUnrecoverableError) {
				disconnect = true;
				break;
			}
		}
	} else if (mRtspState != RtspState::DISCONNECTED) {
		disconnect = true;
	} else {
		mNetworkReadyForStop = true;
	}

	if (!disconnect)
		return 0;

	ret = rtsp_client_disconnect(mRtspClient);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_disconnect", -ret);
		return ret;
	}

	return 0;
}


int StreamDemuxer::stop()
{
	int ret = 0;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState != State::STARTED && mState != State::STARTING) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);

	/* Note: the demuxer listener is not cleared here to allow calling
	 * the IDemuxer::Listener::demuxerCloseResponse listener function when
	 * the IDemuxer::close function was called; clearing the listener when
	 * deleting the API object is done by calling
	 * Demuxer::clearDemuxerListener in the API object destructor prior
	 * to calling Demuxer::stop */

	readyToPlay(false);

	for (const auto &m : mVideoMedias)
		m->sendDownstreamEvent(Channel::DownstreamEvent::EOS);

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;

	ret = stopNetwork();
	if (ret < 0)
		return ret;

	Source::lock();

	mDestroyMediasAfterFlush = false;
	ret = flush();
	if ((ret < 0) && (ret != -EALREADY))
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	Source::unlock();

	tryCompleteStop();

	return ret;
}


void StreamDemuxer::tryCompleteStop()
{
	if (mState != State::STOPPING)
		return;

	if (!mNetworkReadyForStop || !mChannelsReadyForStop)
		return;

	destroyAllVideoMedias();

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
	closeResponse(0);
	setStateAsyncNotify(State::STOPPED);
}


int StreamDemuxer::flush(bool discard)
{
	if ((mState != State::STARTED) && (mState != State::STOPPING)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("demuxer is already flushed, nothing to do");
		/* No need to call complete flush */
		break;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	Source::lock();

	mFlushChannelCount = 0;

	for (const auto &m : mVideoMedias) {
		m->setDestroyAfterFlush(false);
		if (discard)
			m->flush();
		else
			m->drain();
	}

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		const Media *media = getOutputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGW("failed to get media at index %d", i);
			continue;
		}

		mFlushChannelCount += getOutputChannelCount(media);
	}

	if (mFlushChannelCount == 0) {
		mChannelsReadyForStop = true;
		setFlushingState(FlushingState::FLUSHED);
		mDestroyMediasAfterFlush = false;

		if (mSeeking)
			onMediaSeekComplete(0);
	}

	Source::unlock();

	return 0;
}


void StreamDemuxer::onChannelFlushed(Channel *channel)
{
	bool destroyMedia = false;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::lock();

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}
	PDRAW_LOGI("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	for (const auto &m : mVideoMedias) {
		if (m->hasMedia(media)) {
			m->channelFlushed(channel);
			destroyMedia = m->getDestroyAfterFlush();
			break;
		}
	}

	if (mState == State::STOPPING || mDestroyMediasAfterFlush ||
	    destroyMedia) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mFlushChannelCount == 0) {
		setFlushingState(FlushingState::FLUSHED);
		mDestroyMediasAfterFlush = false;
	}

	Source::unlock();
}


void StreamDemuxer::onChannelDrained(Channel *channel)
{
	bool destroyMedia = false;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::lock();

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}
	PDRAW_LOGI("'%s': channel drained media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	for (auto p = mVideoMedias.begin(); p != mVideoMedias.end(); p++) {
		if ((*p)->hasMedia(media)) {
			(*p)->channelDrained(channel);
			destroyMedia = (*p)->getDestroyAfterFlush();
			break;
		}
	}

	if (mState == State::STOPPING || mDestroyMediasAfterFlush ||
	    destroyMedia) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mFlushChannelCount == 0) {
		setFlushingState(FlushingState::FLUSHED);
		mDestroyMediasAfterFlush = false;

		if (mSeeking)
			onMediaSeekComplete(0);
	}

	Source::unlock();
}


void StreamDemuxer::onChannelUnlink(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
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
				mVideoMedias.erase(p);
			}
			break;
		}
	}

	asyncCompleteTeardown();
}


void StreamDemuxer::asyncCompleteTeardown()
{
	int err = mSession->getPompLoop()->idleAdd(&mCompleteTeardownHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void StreamDemuxer::idleCompleteTeardown()
{
	completeTeardown();
}


void StreamDemuxer::completeTeardown()
{
	Source::lock();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		const Media *media = getOutputMedia(i);
		if (media && getOutputChannelCount(media) > 0) {
			Source::unlock();
			return;
		}
	}

	for (const auto &m : mVideoMedias) {
		if (!m->isTearingDown()) {
			Source::unlock();
			return;
		}
	}

	destroyAllVideoMedias();

	Source::unlock();

	if (mState == State::STOPPING) {
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
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::lock();

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}
	PDRAW_LOGD("'%s': channel resync media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	for (const auto &m : mVideoMedias) {
		if (!m->isTearingDown() && m->hasMedia(media)) {
			m->resync();
			Source::unlock();
			return;
		}
	}

	Source::unlock();
}


void StreamDemuxer::onChannelVideoPresStats(Channel *channel,
					    VideoPresStats *stats)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(stats == nullptr, EINVAL);

	Source::lock();

	Source::onChannelVideoPresStats(channel, stats);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		Source::unlock();
		return;
	}

	for (const auto &m : mVideoMedias) {
		if (!m->isTearingDown() && m->hasMedia(media)) {
			m->channelSendVideoPresStats(channel, stats);
			break;
		}
	}

	Source::unlock();
}


int StreamDemuxer::processSelectedMedias()
{
	int res = 0;
	bool noError = false;
	struct sdp_media *media = nullptr;

	for (auto s : mSelectedMedias) {
		bool found = false;
		media = nullptr;
		int idx = 0;
		list_walk_entry_forward(&mSdpSession->medias, media, node)
		{
			if (idx == s->idx) {
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
		for (const auto &m : mVideoMedias) {
			if (m->isTearingDown())
				continue;
			if (xstrcmp(m->getControlUrl(), media->control_url) ==
			    0) {
				PDRAW_LOGI("media '%s' is already set up",
					   media->control_url);
				found = true;
				break;
			}
		}
		if (!found) {
			std::unique_ptr<VideoMedia> videoMedia;
			enum rtsp_lower_transport transport =
				mUrl->getScheme() == RTSP_URL_SCHEME_TCP_TLS
					? RTSP_LOWER_TRANSPORT_TCP
					: RTSP_LOWER_TRANSPORT_UDP;
			try {
				videoMedia = createVideoMedia(transport);
			} catch (const std::bad_alloc &) {
				res = -ENOMEM;
				PDRAW_LOG_ERRNO("createVideoMedia", -res);
				goto stop;
			}
			res = videoMedia->setup(media);
			if (res < 0) {
				PDRAW_LOG_ERRNO("VideoMedia::setup", -res);
				goto stop;
			}
			PDRAW_LOGI("media '%s' not set up, setting up",
				   media->control_url);
			mVideoMedias.push_back(std::move(videoMedia));
		}
	}
	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		bool found = false;
		for (auto s : mSelectedMedias) {
			std::string name1 = s->uri;
			std::string name2 = m->getControlUrl();
			if (name1 == name2) {
				found = true;
				break;
			}
		}
		if (!found) {
			PDRAW_LOGI(
				"media '%s' not selected anymore, "
				"tear it down",
				m->getControlUrl());
			m->teardown();
		}
	}

	res = 0;

	goto exit;

stop:
	if (!noError)
		onUnrecoverableError();
	readyToPlay(false);
	if (mRtspState == RtspState::SETUP_DONE) {
		for (const auto &m : mVideoMedias)
			m->stopRtpAvp();
	}
	setRtspState(RtspState::OPTIONS_DONE);

exit:
	return res;
}


static inline bool shouldUseSdpAddress(const RtspUrl *url,
				       const char *sdpAddress)
{
	if (!sdpAddress || strcmp(sdpAddress, "0.0.0.0") == 0)
		return false;
	return std::string(sdpAddress) != url->getHost();
}


static std::string selectRemoteAddress(const RtspUrl *mUrl,
				       const struct sdp_session *sdpSession)
{
	const char *selectedAddr = nullptr;

	/* Use remote addr from SDP is valid */
	if (shouldUseSdpAddress(mUrl, sdpSession->connection_addr))
		selectedAddr = sdpSession->connection_addr;
	else if (shouldUseSdpAddress(mUrl, sdpSession->server_addr))
		selectedAddr = sdpSession->server_addr;

	if (selectedAddr != nullptr)
		return selectedAddr;

	/* Else, use remote addr from resolved host */
	const std::string &resolvedHost = mUrl->getResolvedHost();
	return resolvedHost.empty() ? std::string() : resolvedHost;
}


static inline bool isSdpMediaSupported(enum rtsp_url_scheme scheme,
				       const struct sdp_media *media)
{
	if (media->type != SDP_MEDIA_TYPE_VIDEO || !media->control_url)
		return false;

	switch (media->rtp_profile) {
	case SDP_MEDIA_RTP_PROFILE_AVP:
		return true;
	case SDP_MEDIA_RTP_PROFILE_SAVP:
		/* Only accept SAVP when in TCP_TLS */
		return (scheme == RTSP_URL_SCHEME_TCP_TLS);
	case SDP_MEDIA_RTP_PROFILE_AVPF:
	case SDP_MEDIA_RTP_PROFILE_SAVPF:
	default:
		return false;
	}
}


void StreamDemuxer::onNewSdp(const char *content_base, const char *sdp)
{
	int res = 0;
	struct sdp_session *session = nullptr;
	std::string remoteAddr;
	size_t mediasCount = 0;
	unsigned int i;
	bool noError = false;
	const struct sdp_media *media = nullptr;
	uint32_t selectedMedias = 0;
	struct pdraw_demuxer_media *newMediaList = nullptr;
	size_t newMediaListSize = 0;
	std::vector<struct pdraw_demuxer_media *> newDefaultMedias;

	if (mState == State::STOPPING) {
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

	if (mContentBase.empty() && content_base) {
		struct rtsp_url *_url = nullptr;
		res = rtsp_url_parse(content_base, &_url);
		if (res < 0) {
			PDRAW_LOG_ERRNO("rtsp_url_parse", -res);
			goto exit;
		}
		mContentBase = content_base;
		mShortContentBase = rtsp_url_get_path(_url);
		rtsp_url_free(_url);
	}

	/* Session-level metadata */
	sessionMetadataFromSdp(mSdpSession, &mSessionMetaFromSdp);

	list_walk_entry_forward(&mSdpSession->medias, media, node)
	{
		if (!isSdpMediaSupported(mUrl->getScheme(), media))
			continue;
		mediasCount++;
	}

	if (mediasCount == 0) {
		/* Empty SDP */
		PDRAW_LOGI("empty SDP, no stream");
		/* An empty SDP means that both the server & the URL are good,
		 * but there is currently no streams. In this case, we can set
		 * the demuxer as State::STARTED here, and wait for an ANNOUCE
		 * to get the media. Otherwise, wait for the setup response
		 * before setting the state. If we have any media, remove them
		 */
		mDestroyMediasAfterFlush = true;
		clearMediaList();
		flush();
		tryCompleteStart();
		noError = true;
		goto stop;
	}

	newMediaList = static_cast<struct pdraw_demuxer_media *>(
		calloc(mediasCount, sizeof(*newMediaList)));
	if (newMediaList == nullptr) {
		PDRAW_LOGE("calloc");
		goto exit;
	}
	newMediaListSize = mediasCount;

	i = 0;
	list_walk_entry_forward(&mSdpSession->medias, media, node)
	{
		if (!isSdpMediaSupported(mUrl->getScheme(), media) ||
		    (i >= mediasCount))
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

	remoteAddr = selectRemoteAddress(mUrl.get(), mSdpSession);
	if (remoteAddr.empty()) {
		PDRAW_LOGE("failed to select remote address");
		goto stop;
	}

	if (mSdpSession->range.start.format == SDP_TIME_FORMAT_NPT) {
		mDuration = mSdpSession->range.stop.npt.sec * 1000000 +
			    (uint64_t)mSdpSession->range.stop.npt.usec;
		mTrackDuration = mDuration;
	}

	if (mRtspState != RtspState::SETUP_DONE)
		setRtspState(RtspState::DESCRIBE_DONE);

	mLocalAddr = "0.0.0.0";
	mRemoteAddr = remoteAddr;

	processSelectedMedias();

	goto exit;

stop:
	if (!noError)
		onUnrecoverableError();
	else if (!mCalledOpenResp)
		openResponse(res);

	readyToPlay(false);
	if (mRtspState == RtspState::SETUP_DONE) {
		for (const auto &m : mVideoMedias)
			m->stopRtpAvp();
	}
	setRtspState(RtspState::OPTIONS_DONE);

exit:
	pdraw_demuxerMediaListFree(newMediaList, newMediaListSize);
}


void StreamDemuxer::tryCompleteStart(bool callOpenResp)
{
	if (mState != State::STARTING)
		return;

	setState(State::STARTED);

	if (callOpenResp && !mCalledOpenResp)
		openResponse(0);
}


int StreamDemuxer::internalPlay(float speed)
{
	mRunning = true;
	mWasRunningOnce = true;
	mSpeed = speed;
	mFrameByFrame = false;

	if ((mSessionProtocol == SessionProtocol::RTSP) &&
	    (mRtspState == RtspState::SETUP_DONE)) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.now = 1;
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
		mUpdateTrackDuration = true;
		int ret = rtsp_client_play(mRtspClient,
					   mRtspSessionId.c_str(),
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
	} else if (mSessionProtocol == SessionProtocol::NONE) {
		for (const auto &m : mVideoMedias)
			m->onPlayComplete();
		if (getPendingCommand() == Command::PLAY)
			playResponse(0, getCurrentTime(), mSpeed);
	}

	return 0;
}


int StreamDemuxer::internalPause()
{
	mRunning = false;
	mFrameByFrame = true;

	if ((mSessionProtocol == SessionProtocol::RTSP) &&
	    (mRtspState == RtspState::SETUP_DONE)) {
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		int ret =
			rtsp_client_pause(mRtspClient,
					  mRtspSessionId.c_str(),
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
	} else if ((mSessionProtocol == SessionProtocol::NONE) &&
		   (getPendingCommand() == Command::PAUSE ||
		    getPendingCommand() == Command::PAUSE_NEXT)) {
		pauseResponse(0, getCurrentTime());
	}

	return 0;
}


int StreamDemuxer::play(float speed)
{
	int ret;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	switch (getPendingCommand()) {
	case Command::NONE:
		/* OK */
		break;
	case Command::PLAY:
	case Command::PAUSE:
	case Command::PAUSE_NEXT:
		if (((getPendingCommand() == Command::PLAY) && (speed != 0.)) ||
		    ((getPendingCommand() == Command::PAUSE ||
		      getPendingCommand() == Command::PAUSE_NEXT) &&
		     (speed == 0.)))
			return -EALREADY;
		[[fallthrough]];
	default:
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		if (speed != 0.)
			m->play();
		else
			m->pause();
	}

	if (speed == 0.) {
		if (!mWasRunningOnce)
			setPendingCommand(Command::PAUSE_NEXT);
		else
			setPendingCommand(Command::PAUSE);
		ret = internalPause();
		if (ret < 0)
			goto error;
	} else {
		setPendingCommand(Command::PLAY);
		ret = internalPlay(speed);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	clearPendingCommand();
	return ret;
}


bool StreamDemuxer::isReadyToPlay() const
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	return mReadyToPlay;
}


bool StreamDemuxer::isPaused() const
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int StreamDemuxer::previous()
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (getDuration() == 0) {
		PDRAW_LOGE("%s: not a replay pipeline", __func__);
		return -ENOSYS;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	if (mSessionProtocol != SessionProtocol::RTSP)
		return -ENOSYS;

	if (mRtspState != RtspState::SETUP_DONE)
		return -EAGAIN;

	switch (getPendingCommand()) {
	case Command::NONE:
		/* OK */
		break;
	case Command::SEEK:
		return -EALREADY;
	default:
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	int ret = 0;
	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = mPausePoint / 1000000;
	range.start.npt.usec = static_cast<uint32_t>(mPausePoint % 1000000ULL);
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
			       mRtspSessionId.c_str(),
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

	setPendingCommand(Command::SEEK);

	for (const auto &m : mVideoMedias)
		m->previous();

	mEndOfRangeNotified = false;

	return 0;
}


int StreamDemuxer::next()
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (getDuration() == 0) {
		PDRAW_LOGE("%s: not a replay pipeline", __func__);
		return -ENOSYS;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	if (mSessionProtocol != SessionProtocol::RTSP)
		return -ENOSYS;

	if (mRtspState != RtspState::SETUP_DONE)
		return -EAGAIN;

	switch (getPendingCommand()) {
	case Command::NONE:
		/* OK */
		break;
	case Command::PAUSE_NEXT:
		if (!mWasRunningOnce)
			break;
		[[fallthrough]];
	case Command::SEEK:
		return -EALREADY;
	default:
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = mPausePoint / 1000000;
	range.start.npt.usec = static_cast<uint32_t>(mPausePoint % 1000000ULL);
	range.stop = range.start;
	/* TODO: SMPTE timestamps*/
	range.stop.npt.usec += 1000;
	if (range.stop.npt.usec >= 1000000) {
		range.stop.npt.sec++;
		range.stop.npt.usec -= 1000000;
	}
	int ret = rtsp_client_play(mRtspClient,
				   mRtspSessionId.c_str(),
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

	if (getPendingCommand() != Command::PAUSE_NEXT)
		setPendingCommand(Command::SEEK);

	for (const auto &m : mVideoMedias)
		m->next();

	mEndOfRangeNotified = false;

	return 0;
}


int StreamDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (getDuration() == 0) {
		PDRAW_LOGE("%s: not a replay pipeline", __func__);
		return -ENOSYS;
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
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (getDuration() == 0) {
		PDRAW_LOGE("%s: not a replay pipeline", __func__);
		return -ENOSYS;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	if (mSessionProtocol != SessionProtocol::RTSP)
		return -ENOSYS;

	if (mRtspState != RtspState::SETUP_DONE)
		return -EAGAIN;

	if (getPendingCommand() != Command::NONE) {
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	float scale = mSpeed;
	struct rtsp_range range;
	memset(&range, 0, sizeof(range));
	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.sec = timestamp / 1000000;
	range.start.npt.usec = static_cast<uint32_t>(timestamp % 1000000ULL);
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
				   mRtspSessionId.c_str(),
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
	mSeekingNetwork = true;
	mEndOfRangeNotified = false;
	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		m->seek();
	}

	setPendingCommand(Command::SEEK);

	return 0;
}


uint64_t StreamDemuxer::getDuration() const
{
	return mDuration;
}


uint64_t StreamDemuxer::getCurrentTime() const
{
	if (mSessionProtocol == SessionProtocol::RTSP)
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


void StreamDemuxer::onMediaSeekComplete(int seekResponse)
{
	bool anySeeking = false;
	if ((seekResponse != 0) && (mSeekResponse == 0))
		mSeekResponse = seekResponse;
	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		anySeeking |= m->isSeeking();
	}
	if (anySeeking || mSeekingNetwork)
		return;
	/* In frame-by-frame, drain is needed */
	if (!mRunning && mFrameByFrame &&
	    (getFlushingState() != FlushingState::FLUSHED)) {
		drain();
		return;
	}
	if (getPendingCommand() == Command::PAUSE_NEXT)
		this->pauseResponse(mSeekResponse, mCurrentTime);
	else
		this->seekResponse(mSeekResponse, mCurrentTime, mSpeed);
	mSeeking = false;
	mSeekResponse = 0;
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

	if (mRtspState != RtspState::SETUP_DONE)
		setRtspState(RtspState::DESCRIBE_DONE);

	processSelectedMedias();

	return 0;

stop:
	readyToPlay(false);
	if (mRtspState == RtspState::SETUP_DONE) {
		for (const auto &m : mVideoMedias)
			m->stopRtpAvp();
	}
	setRtspState(RtspState::OPTIONS_DONE);

	return ret;
}


StreamDemuxer::VideoMedia::VideoMedia(StreamDemuxer *demuxer) :
		mDemuxer(demuxer)
{
	mFrameTimerHandler.set([this] { onFrameTimeout(); });
	mRangeTimerHandler.set([this] { onRangeTimer(); });
	mCompleteSeekHandler.set([this] { idleCompleteSeek(); });

	std::string name = demuxer->getName() + "#VideoMedia";
	Loggable::setName(name);
}


StreamDemuxer::VideoMedia::~VideoMedia()
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

	if (mFrameTimer) {
		resetFrameTimer(false);
		mFrameTimer.reset();
	}

	mRangeTimer.reset();
}


bool StreamDemuxer::VideoMedia::hasMedia(const Media *media) const
{
	return std::any_of(mVideoMedias.begin(),
			   mVideoMedias.end(),
			   [media](const std::unique_ptr<CodedVideoMedia> &m) {
				   return m.get() == media;
			   });
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
	try {
		mFrameTimer = std::make_unique<pomp::Timer>(
			mDemuxer->mSession->getPompLoop(), &mFrameTimerHandler);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp::Timer", -ret);
		return ret;
	}

	/* Create the end of range timer */
	try {
		mRangeTimer = std::make_unique<pomp::Timer>(
			mDemuxer->mSession->getPompLoop(), &mRangeTimerHandler);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp::Timer", -ret);
		return ret;
	}

	if (media != nullptr &&
	    mDemuxer->mSessionProtocol == SessionProtocol::RTSP) {
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
		mDemuxer->mSetupRequestsCount++;
	}
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


void StreamDemuxer::VideoMedia::finishSetup()
{
	if (mSdpMedia != nullptr) {
		/* Media-level metadata */
		sessionMetadataFromSdp(mSdpMedia,
				       &mDemuxer->mSessionMetaFromSdp,
				       &mSessionMetaFromSdp);

		SetupRequest req(this,
				 std::string(mSdpMedia->control_url
						     ? mSdpMedia->control_url
						     : ""),
				 getLowerTransport(),
				 getLocalStreamPort(),
				 getLocalControlPort(),
				 getHeaderExt(),
				 getHeaderExtCount());
		mDemuxer->mSetupRequests.push(req);

		(void)mDemuxer->processRtspRequests();
	}
}


int StreamDemuxer::VideoMedia::teardown()
{
	if (mTearingDown)
		return -EALREADY;

	if (mPendingTearDown)
		return -EBUSY;

	mPendingTearDown = true;

	finishTeardown();
	return 0;
}


void StreamDemuxer::VideoMedia::finishTeardown()
{
	if (getControlUrl() == nullptr)
		return;

	/* Media-level metadata */
	TeardownRequest req(
		this,
		std::string(getControlUrl() != nullptr ? getControlUrl() : ""));
	mDemuxer->mTeardownRequests.push(req);
	mDemuxer->mTeardownRequestsCount++;

	(void)mDemuxer->processRtspRequests();
}


void StreamDemuxer::VideoMedia::setTearingDown()
{
	mPendingTearDown = false;
	mTearingDown = true;
	/* Needed to close sockets */
	stopRtpAvp();
	/* TODO: remove TEARDOWN request from the queue? */
}


int StreamDemuxer::VideoMedia::setupMedia()
{
	int ret;
	Source::OutputPort *basePort;
	Source::OutputPort *mediaPort;

	if (mDemuxer->mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	/* Note: H.265 streaming is not supported */
	if (mCodecInfo.codec != VSTRM_CODEC_VIDEO_H264) {
		PDRAW_LOGE("invalid codec info");
		return -EPROTO;
	}

	mDemuxer->Source::lock();

	if (!mVideoMedias.empty()) {
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media already defined");
		return -EBUSY;
	}

	size_t nbVideoMedias = 2;

	mVideoMedias.resize(nbVideoMedias);

	for (unsigned int i = 0; i < mVideoMedias.size(); i++) {
		std::unique_ptr<CodedVideoMedia> media;
		try {
			media = std::make_unique<CodedVideoMedia>(
				mDemuxer->mSession);
		} catch (const std::bad_alloc &) {
			mDemuxer->Source::unlock();
			PDRAW_LOGE("media allocation failed");
			return -ENOMEM;
		}

		switch (i) {
		case 0:
			media->format = vdef_h264_avcc;
			break;
		case 1:
			media->format = vdef_h264_byte_stream;
			break;
		default:
			break;
		}
		ret = mDemuxer->addOutputPort(media.get(),
					      mDemuxer->getDemuxer());
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("addOutputPort", -ret);
			return ret;
		}
		std::string path =
			mDemuxer->Element::getName() + "$" + media->getName();
		media->setPath(path);
		ret = media->setPs(nullptr,
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
		media->sessionMeta = mSessionMetaFromSdp;
		media->setPlaybackType(
			PDRAW_PLAYBACK_TYPE_LIVE); /* TODO: live/replay */
		media->setDuration(mDemuxer->mDuration);

		mVideoMedias[i] = std::move(media);
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
		mVideoMedias[1].get(),
		DEMUXER_STREAM_OUTPUT_BUFFER_COUNT,
		mVideoMedias[1]->info.resolution.width *
			mVideoMedias[1]->info.resolution.height * 3 / 4);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		return ret;
	}
	/* Make the pool shared between all medias */
	basePort = mDemuxer->getOutputPort(mVideoMedias[1].get());
	mediaPort = mDemuxer->getOutputPort(mVideoMedias[0].get());
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
		for (const auto &m : mVideoMedias) {
			mDemuxer->Source::mListener->onOutputMediaAdded(
				mDemuxer, m.get(), mDemuxer->getDemuxer());
		}
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


void StreamDemuxer::VideoMedia::teardownMedia()
{
	/* Destroy the H.264 reader */
	if (mH264Reader != nullptr) {
		int ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = nullptr;
	}

	/* Remove the output ports */
	for (const auto &m : mVideoMedias) {
		if (mDemuxer->Source::mListener) {
			mDemuxer->Source::mListener->onOutputMediaRemoved(
				mDemuxer, m.get(), mDemuxer->getDemuxer());
		}
		int ret = mDemuxer->removeOutputPort(m.get());
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	}
	mVideoMedias.clear();
}


void StreamDemuxer::VideoMedia::asyncCompleteSeek()
{
	if (mAsyncCompleteSeekCalled)
		return;

	int err = mDemuxer->mSession->getPompLoop()->idleAdd(
		&mCompleteSeekHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
	else
		mAsyncCompleteSeekCalled = true;
}


void StreamDemuxer::VideoMedia::idleCompleteSeek()
{
	completeSeek();
}


void StreamDemuxer::VideoMedia::completeSeek()
{
	mPendingSeek = false;
	mAsyncCompleteSeekCalled = false;
	mDemuxer->onMediaSeekComplete(mSeekResponse);
}


void StreamDemuxer::VideoMedia::completeFlush()
{
	mFlushChannelCount--;
	if (mFlushChannelCount <= 0)
		mFlushing = false;
}


int StreamDemuxer::VideoMedia::createReceiver()
{
	std::unique_ptr<struct vstrm_receiver_cfg> cfg;
	std::string tmp;
	int ret;

	try {
		cfg = std::make_unique<struct vstrm_receiver_cfg>();
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("std::make_unique", -ret);
		return ret;
	}

	memset(cfg.get(), 0, sizeof(*cfg.get()));

	/* Create the stream receiver */
	cfg->loop = mDemuxer->mSession->getLoop();
	cfg->flags = VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE |
		     VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_FRAME |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP |
		     VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT;

	auto safeCopy = [](char *dest,
			   size_t size,
			   const std::string &src,
			   const char *field) {
		if (size == 0)
			return;
		errno = 0;
		int res = snprintf(dest, size, "%s", src.c_str());
		if (res < 0) {
			ULOG_ERRNO("snprintf error for %s", errno, field);
		} else if ((size_t)res >= size) {
			ULOGE("field '%s' truncated: '%s' -> '%s'",
			      field,
			      src.c_str(),
			      dest);
		}
	};

	mDemuxer->mSession->getSettings()->getFriendlyName(&tmp);
	safeCopy(cfg->self_meta.friendly_name,
		 sizeof(cfg->self_meta.friendly_name),
		 tmp,
		 "friendly_name");

	mDemuxer->mSession->getSettings()->getSerialNumber(&tmp);
	safeCopy(cfg->self_meta.serial_number,
		 sizeof(cfg->self_meta.serial_number),
		 tmp,
		 "serial_number");

	mDemuxer->mSession->getSettings()->getSoftwareVersion(&tmp);
	safeCopy(cfg->self_meta.software_version,
		 sizeof(cfg->self_meta.software_version),
		 tmp,
		 "software_version");

	ret = vstrm_receiver_new(cfg.get(), &mReceiverCbs, this, &mReceiver);
	if (ret < 0) {
		mReceiver = nullptr;
		PDRAW_LOG_ERRNO("vstrm_receiver_new", -ret);
		destroyReceiver();
		return ret;
	}

	/* Provide the SPS/PPS out of band if available */
	if (mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) {
		ret = vstrm_receiver_set_codec_info(
			mReceiver, &mCodecInfo, mSsrc);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vstrm_receiver_set_codec_info", -ret);
	}

	return 0;
}


int StreamDemuxer::VideoMedia::destroyReceiver()
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


void StreamDemuxer::VideoMedia::play()
{
	int err;

	mRtpPaused = false;

	/* Rearm the frame timer only when running */
	resetFrameTimer(mDemuxer->mRunning);
	err = mRangeTimer->clear();
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Timer::clear", -err);
}


void StreamDemuxer::VideoMedia::onPlayComplete() const
{
	/* Nothing to do here */
}


void StreamDemuxer::VideoMedia::pause() const
{
	/* Nothing to do here */
}


void StreamDemuxer::VideoMedia::onPauseComplete()
{
	resetFrameTimer(false);

	mRtpPaused = true;

	vstrm_receiver_clear(mReceiver);
	/* A flush is needed to discard all pending frames that will be
	 * processed when re-playing later on, leading to issues such as large
	 * rendering timing error. */
	setDestroyAfterFlush(false);
	drain();
}


void StreamDemuxer::VideoMedia::resync()
{
	mWaitForSync = true;
	mRecoveryFrameCount = 0;
}


void StreamDemuxer::VideoMedia::seek()
{
	mPendingSeek = true;
	mAsyncCompleteSeekCalled = false;
	mSeekResponse = 0;
	play();
}


void StreamDemuxer::VideoMedia::previous()
{
	int err;
	mRtpPaused = false;
	mPendingSeek = true;
	err = mRangeTimer->clear();
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Timer::clear", -err);
}


void StreamDemuxer::VideoMedia::next()
{
	int err;
	mRtpPaused = false;
	mPendingSeek = true;
	err = mRangeTimer->clear();
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Timer::clear", -err);
}


void StreamDemuxer::VideoMedia::stop()
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

	for (const auto &m : mVideoMedias)
		m->setTearingDown();

	mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::flush(bool discard)
{
	int err;

	mDemuxer->Source::lock();

	mFlushing = true;
	mFlushDiscard = discard;
	mFlushChannelCount = 0;

	/* New synchronization is needed */
	resync();

	if (mCurrentFrame != nullptr) {
		err = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}

	for (const auto &m : mVideoMedias) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(m.get());
		mFlushChannelCount += outputChannelCount;

		/* Flush the output channels */
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *channel =
				mDemuxer->getOutputChannel(m.get(), i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			if (mFlushDiscard)
				err = channel->flush();
			else
				err = channel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO("channel->%s",
						-err,
						mFlushDiscard ? "flush"
							      : "drain");
			}
		}
	}

	if (mFlushChannelCount <= 0)
		mFlushing = false;

	mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::channelFlushed(
	[[maybe_unused]] const Channel *channel)
{

	completeFlush();
}


void StreamDemuxer::VideoMedia::channelDrained(
	[[maybe_unused]] const Channel *channel)
{

	completeFlush();
}


void StreamDemuxer::VideoMedia::channelUnlink(
	[[maybe_unused]] const Channel *channel)
{

	mDemuxer->Source::lock();

	for (const auto &m : mVideoMedias) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(m.get());
		if (outputChannelCount > 0) {
			mDemuxer->Source::unlock();
			return;
		}
	}

	mDemuxer->Source::unlock();

	/* Abort any new media creation once tearing down is requested */
	if (mPendingTearDown || mTearingDown)
		mCodecInfoChanging = false;

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
		err = mFrameTimer->set(DEMUXER_STREAM_FRAME_TIMEOUT_MS);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Timer::set", err);
	} else {
		err = mFrameTimer->clear();
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Timer::clear", -err);
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
	for (const auto &m : mVideoMedias) {
		int res = mDemuxer->Source::sendDownstreamEvent(m.get(), event);
		if (res < 0)
			PDRAW_LOG_ERRNO("Source::sendDownstreamEvent", -res);
	}
}


int StreamDemuxer::VideoMedia::processFrame(struct vstrm_frame *frame)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount = 0;
	CodedVideoMedia::Frame data = {};
	uint32_t flags = 0;
	size_t frameSize = 0;
	size_t bufSize;
	uint8_t *buf;
	bool isIdr = false;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	uint64_t remainingPlayTime = 0;
	unsigned int requiredMediaIndex;
	const CodedVideoMedia *requiredMedia;
	struct vdef_coded_frame frameInfo = {};
	struct mbuf_coded_video_frame *outputFrame = nullptr;
	unsigned int sliceCount = 0;
	int64_t clock_delta = 0;
	uint32_t precision = UINT32_MAX;
	uint64_t ntpTimestamp = 0;
	unique_c_ptr<vstrm_video_stats_dyn> videoStatsDyn;
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

	for (const auto &m : mVideoMedias) {
		if (m != nullptr)
			codedMedias.push_back(m.get());
	}

	ret = mDemuxer->getCodedVideoOutputMemory(
		codedMedias, &mCurrentMem, &requiredMediaIndex);
	if ((ret < 0) || (mCurrentMem == nullptr)) {
		mDemuxer->Source::unlock();
		PDRAW_LOGW("failed to get an output memory (%d)", ret);
		setDestroyAfterFlush(false);
		flush();
		return ret;
	}
	requiredMedia = mVideoMedias[requiredMediaIndex].get();
	{
		void *rawBuf = nullptr;
		ret = mbuf_mem_get_data(mCurrentMem, &rawBuf, &bufSize);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
			goto out;
		}
		buf = static_cast<uint8_t *>(rawBuf);
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
		naluType = extractNaluType<h264_nalu_type>(
			frame->nalus[i].cdata[0], 0x1F);
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
			[[fallthrough]];
		case H264_NALU_TYPE_SLICE:
			sliceType = /* TODO */
				H264_SLICE_TYPE_UNKNOWN;
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
	if (mWaitForSync) {
		if (isIdr && (!frame->info.gen_grey_idr)) {
			mWaitForSync = false;
			mRecoveryFrameCount = 0;
		} else {
			PDRAW_LOGD("discarding frame (wait for sync)");
			goto out;
		}
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
	/* Silent frames before next play point */
	if (frame->timestamps.ntp_raw < mDemuxer->mPlayNtpTime)
		frameInfo.info.flags |= VDEF_FRAME_FLAG_SILENT;

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

	/* The ntp timestamp is unset until the first sender report is
	 * received. Fall back to the ntp_raw timestamp */
	ntpTimestamp = (frame->timestamps.ntp != 0) ? frame->timestamps.ntp
						    : frame->timestamps.ntp_raw;

	if (mDemuxer->mSessionProtocol == SessionProtocol::RTSP) {
		int64_t nptTime =
			static_cast<int64_t>(
				static_cast<double>(ntpTimestamp) *
				static_cast<double>(mDemuxer->mSpeed)) -
			mDemuxer->mNtpToNptOffset;
		mDemuxer->mCurrentTime = (nptTime >= 0) ? (uint64_t)nptTime : 0;
	} else {
		/* Use ntp_raw (monotonic) to avoid a domain jump when the first
		 * RTCP Sender Report arrives and ntp switches from 0 to an
		 * absolute NTP-epoch value (~126 years in µs). */
		mDemuxer->mCurrentTime = frame->timestamps.ntp_raw;
		if (mDemuxer->mStartTime == 0)
			mDemuxer->mStartTime = frame->timestamps.ntp_raw;
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
					? static_cast<uint64_t>(
						  static_cast<double>(
							  remainingPlayTime) /
						  static_cast<double>(
							  mDemuxer->mSpeed))
					: UINT64_MAX;
			if (remainingPlayTime < 1000000) {
				/* Add 50ms margin */
				uint32_t delay = (static_cast<uint32_t>(
							  remainingPlayTime) /
						  1000) +
						 50;
				mRangeTimer->set(delay);
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
	videoStatsDyn = make_c_struct<unique_c_ptr<vstrm_video_stats_dyn>>();
	if (!videoStatsDyn) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	ret = vstrm_video_stats_dyn_copy(videoStatsDyn.get(),
					 &frame->video_stats_dyn);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_video_stats_dyn_copy", -ret);
		goto out;
	}
	{
		/* rawDyn is extracted before release() for the cbs and buffer
		 */
		auto *rawDyn = videoStatsDyn.get();
		cbs = (struct mbuf_ancillary_data_cbs){
			.cleaner = &videoStatsDynCleaner,
			.cleaner_userdata = rawDyn,
		};
		ret = mbuf_coded_video_frame_add_ancillary_buffer_with_cbs(
			mCurrentFrame,
			VSTRM_ANCILLARY_KEY_VIDEO_STATS_DYN,
			&rawDyn,
			sizeof(rawDyn),
			&cbs);
		if (ret < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_coded_video_frame_add_ancillary_buffer"
				"_with_cbs",
				-ret);
			goto out;
		}
	}
	/* ownership transferred to videoStatsDynCleaner */
	videoStatsDyn.release();
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
	for (const auto &m : mVideoMedias) {
		outputChannelCount = mDemuxer->getOutputChannelCount(m.get());
		if (outputChannelCount == 0)
			continue;
		if (outputFrame != nullptr)
			mbuf_coded_video_frame_unref(outputFrame);
		outputFrame = mCurrentFrame;
		if (!vdef_coded_format_cmp(&requiredMedia->format,
					   &m->format)) {
			/* The format is different, we need to pick another
			 * frame */
			int copy_ret = mDemuxer->copyCodedVideoOutputFrame(
				requiredMedia,
				mCurrentFrame,
				m.get(),
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
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			const struct vdef_coded_format *caps;
			int capsCount;

			Channel *c = mDemuxer->getOutputChannel(m.get(), i);
			auto *channel = dynamic_cast<CodedVideoChannel *>(c);
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
			if (queue_ret < 0) {
				PDRAW_LOG_ERRNO("channel->queue", -queue_ret);
			} else {
				mDemuxer->setFlushingState(
					FlushingState::UNFLUSHED);
			}
		}
	}
	if (outputFrame != nullptr)
		mbuf_coded_video_frame_unref(outputFrame);
	if (mFirstFrame && (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
	}

	if (mPendingSeek && !mDemuxer->mSeekingNetwork &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		mSeekResponse = 0;
		asyncCompleteSeek();
	}

out:
	mbuf_mem_unref(mCurrentMem);
	mCurrentMem = nullptr;
	mbuf_coded_video_frame_unref(mCurrentFrame);
	mCurrentFrame = nullptr;

	mDemuxer->Source::unlock();
	return ret;
}


void StreamDemuxer::VideoMedia::channelSendVideoPresStats(
	[[maybe_unused]] const Channel *channel,
	const VideoPresStats *stats)
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
	const struct sdp_attr *attr = nullptr;

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
	[[maybe_unused]] struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);
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
	[[maybe_unused]] const uint8_t *buf,
	[[maybe_unused]] size_t len,
	const struct h264_sei_pic_timing *sei,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);

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
	[[maybe_unused]] struct h264_ctx *ctx,
	[[maybe_unused]] const uint8_t *buf,
	[[maybe_unused]] size_t len,
	const struct h264_sei_recovery_point *sei,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);

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

	auto *self = static_cast<VideoMedia *>(userdata);

	if (self == nullptr)
		return -EINVAL;

	return self->sendCtrl(stream, pkt);
}


void StreamDemuxer::VideoMedia::codecInfoChangedCb(
	[[maybe_unused]] struct vstrm_receiver *stream,
	const struct vstrm_codec_info *info,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);
	int outputChannelCount = 0;
	Channel *channel;
	Channel::DownstreamEvent evt;
	bool sendEvt = false;
	int ret;

	if ((self == nullptr) || (info == nullptr))
		return;
	if (info->codec != VSTRM_CODEC_VIDEO_H264) {
		PDRAW_LOG_ERRNO("info->codec", EPROTO);
		return;
	}

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return;
	}

	if (self->mTearingDown || self->mPendingTearDown) {
		PDRAW_LOGI(
			"ignoring codec info change on a tearing down media");
		return;
	}

	PDRAW_LOGD("codec info changed");
	self->mWaitForCodecInfo = false;

	if ((!self->mCodecInfoChanging) &&
	    vstrm_codec_info_cmp(&self->mCodecInfo, info)) {
		PDRAW_LOGI("codec info changed; no change in PS, just resync");
		self->resync();
		return;
	}
	if ((self->mCodecInfo.codec == info->codec) &&
	    (self->mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) &&
	    (self->mCodecInfo.h264.width == info->h264.width) &&
	    (self->mCodecInfo.h264.height == info->h264.height)) {
		/* TODO: also check framerate */
		sendEvt = (!self->mRtcpMediaChangeReceived);
		evt = Channel::DownstreamEvent::RECONFIGURE;
	} else {
		sendEvt = (!self->mRtcpMediaChangeReceived);
		evt = Channel::DownstreamEvent::RESOLUTION_CHANGE;
	}
	self->mRtcpMediaChangeReceived = false;
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

	if (!self->mVideoMedias.empty()) {
		PDRAW_LOGI("change of output media");
		self->mCodecInfoChanging = true;
		for (const auto &m : self->mVideoMedias) {
			outputChannelCount =
				demuxer->getOutputChannelCount(m.get());
			/* Teardown the output channels
			 * Note: loop downwards because calling teardown on a
			 * channel may or may not synchronously remove the
			 * channel from the output port */
			for (int i = outputChannelCount - 1; i >= 0; i--) {
				channel = demuxer->getOutputChannel(m.get(), i);
				if (channel == nullptr) {
					PDRAW_LOGW(
						"failed to get channel "
						"at index %d",
						i);
					continue;
				}
				ret = channel->teardown();
				if (ret < 0)
					PDRAW_LOG_ERRNO("channel->teardown",
							-ret);
			}
		}
		if (sendEvt) {
			PDRAW_LOGW(
				"sending %s event (RTCP event probably lost)",
				Channel::getDownstreamEventStr(evt));
			self->sendDownstreamEvent(evt);
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


void StreamDemuxer::VideoMedia::recvFrameCb(
	[[maybe_unused]] struct vstrm_receiver *stream,
	struct vstrm_frame *frame,
	void *userdata)
{

	int err;
	auto *self = static_cast<VideoMedia *>(userdata);

	if ((self == nullptr) || (frame == nullptr))
		return;

	const StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != State::STARTED) {
		PDRAW_LOGW("%s: demuxer is not started", __func__);
		return;
	}

	if (demuxer->mRunning) {
		self->resetFrameTimer(true);
	} else if (demuxer->mSessionProtocol != SessionProtocol::RTSP) {
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
	[[maybe_unused]] struct vstrm_receiver *stream,
	const struct vmeta_session *meta,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);

	if ((self == nullptr) || (meta == nullptr))
		return;

	PDRAW_LOGD("session metadata changed");

	self->mDemuxer->Source::lock();

	for (const auto &m : self->mVideoMedias) {
		m->sessionMeta = *meta;
		/* FIXME: to be discussed */
		self->mSessionMetaFromSdp = *meta;
		int err = self->mDemuxer->sendDownstreamEvent(
			m.get(), Channel::DownstreamEvent::SESSION_META_UPDATE);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}

	self->mDemuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::eventCb(
	[[maybe_unused]] struct vstrm_receiver *stream,
	enum vstrm_event event,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);
	Channel::DownstreamEvent evt;
	bool sendEvent = false;

	if (self == nullptr)
		return;

	PDRAW_LOGI("received custom RTCP event '%s'",
		   vstrm_event_to_str(event));

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != State::STARTED)
		return;

	switch (event) {
	case VSTRM_EVENT_RECONFIGURE:
		evt = Channel::DownstreamEvent::RECONFIGURE;
		sendEvent = true;
		self->mRtcpMediaChangeReceived = true;
		break;
	case VSTRM_EVENT_RESOLUTION_CHANGE:
		evt = Channel::DownstreamEvent::RESOLUTION_CHANGE;
		sendEvent = true;
		self->mRtcpMediaChangeReceived = true;
		break;
	case VSTRM_EVENT_FRAMERATE_CHANGE:
		evt = Channel::DownstreamEvent::FRAMERATE_CHANGE;
		sendEvent = true;
		self->mRtcpMediaChangeReceived = true;
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


void StreamDemuxer::VideoMedia::goodbyeCb(
	[[maybe_unused]] struct vstrm_receiver *stream,
	const char *reason,
	void *userdata)
{

	auto *self = static_cast<VideoMedia *>(userdata);
	Channel::DownstreamEvent event;
	bool sendEvent = false;

	if (self == nullptr)
		return;

	PDRAW_LOGI("received RTCP goodbye%s%s",
		   reason ? ", reason: " : "",
		   reason ? reason : "");

	StreamDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != State::STARTED)
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
			if (demuxer->mSessionProtocol ==
				    SessionProtocol::RTSP &&
			    (strcmp(reason,
				    DEMUXER_STREAM_GOODBYE_REASON_USER) ||
			     (demuxer->mState != State::STOPPING &&
			      demuxer->mState != State::STOPPED))) {
				/* We either received an unknown RTCP goodbye
				 * packet, or an unexpected (not initiated by a
				 * teardown) user_disconnection packet. Notify
				 * the application of an unrecoverable error
				 * only when a single media is selected. */
				unsigned int count = 0;
				for (const auto &m : demuxer->mVideoMedias) {
					if (m->isTearingDown())
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


void StreamDemuxer::VideoMedia::onFrameTimeout()
{
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	StreamDemuxer *demuxer = mDemuxer;

	if (demuxer->mState != State::STARTED)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		PDRAW_LOG_ERRNO("time_get_monotonic", -res);
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0)
		PDRAW_LOG_ERRNO("time_timespec_to_us", -res);

	demuxer->Source::lock();
	if (curTime > mLastFrameReceiveTime + DEMUXER_STREAM_FRAME_TIMEOUT_MS) {
		sendDownstreamEvent(Channel::DownstreamEvent::TIMEOUT);
	}
	demuxer->Source::unlock();
}


void StreamDemuxer::VideoMedia::onRangeTimer()
{
	StreamDemuxer *demuxer = mDemuxer;

	if (!demuxer->mEndOfRangeNotified) {
		PDRAW_LOGI("end of range reached");
		if (!demuxer->mFrameByFrame)
			sendDownstreamEvent(Channel::DownstreamEvent::EOS);
		demuxer->onEndOfRange(demuxer->mCurrentTime);
		demuxer->mEndOfRangeNotified = true;
	}
}


void StreamDemuxer::idleEndOfRangeNotification()
{
	if (!mEndOfRangeNotified) {
		if (!mFrameByFrame) {
			for (const auto &m : mVideoMedias)
				m->sendDownstreamEvent(
					Channel::DownstreamEvent::EOS);
		}
		PDRAW_LOGI("end of range reached");
		onEndOfRange(mCurrentTime);
		mEndOfRangeNotified = true;
	}
}


void StreamDemuxer::videoStatsDynCleaner(
	[[maybe_unused]] struct mbuf_ancillary_data *data,
	void *userdata)
{

	auto dyn = unique_c_ptr<vstrm_video_stats_dyn>(
		static_cast<vstrm_video_stats_dyn *>(userdata));
	if (!dyn)
		return;
	vstrm_video_stats_dyn_clear(dyn.get());
}

} /* namespace Pdraw */
