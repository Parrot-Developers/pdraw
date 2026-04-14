/**
 * Parrot Drones Audio and Video Vector library
 * RTSP stream muxer
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

#define ULOG_TAG pdraw_rtspmuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <array>
#include <media-buffers/mbuf_mem_generic.h>


#define LOG_STREAM_EVT(event_var, self, res, fmt, ...)                         \
	ULOG_EVT("STREAM",                                                     \
		 "event='%s';element='%s';"                                    \
		 "status=%d;status_str='%s'" fmt,                              \
		 event_var,                                                    \
		 (self)->getCName(),                                           \
		 res,                                                          \
		 strerror(-(res)),                                             \
		 __VA_ARGS__)


constexpr const char *CONTROL_URL = "streamid=0";


namespace Pdraw {


constexpr size_t NB_SUPPORTED_FORMATS = 1;
static std::array<vdef_coded_format, NB_SUPPORTED_FORMATS> supportedFormats;
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats()
{
	supportedFormats[0] = vdef_h264_avcc;
}


const struct rtsp_client_cbs RtspStreamMuxer::mRtspClientCbs = {
	.socket_cb = &RtspStreamMuxer::onRtspSocketCreated,
	.ready_to_send_cb = &RtspStreamMuxer::onReadyToSendCb,
	.interleaved_data_cb = &RtspStreamMuxer::onRtspInterleavedDataCb,
	.connection_state = &RtspStreamMuxer::onRtspConnectionState,
	.session_removed = &RtspStreamMuxer::onRtspSessionRemoved,
	.options_resp = &RtspStreamMuxer::onRtspOptionsResp,
	.describe_resp = nullptr,
	.announce_resp = &RtspStreamMuxer::onRtspAnnounceResp,
	.setup_resp = &RtspStreamMuxer::onRtspSetupResp,
	.play_resp = nullptr,
	.pause_resp = nullptr,
	.record_resp = &RtspStreamMuxer::onRtspRecordResp,
	.teardown_resp = &RtspStreamMuxer::onRtspTeardownResp,
	.announce = &RtspStreamMuxer::onRtspAnnounce,
	.teardown = &RtspStreamMuxer::onRtspForcedTeardown,
};


/* codecheck_ignore[COMPLEX_MACRO] */
#define MAP_ENUM_CLASS_CASE(_enum, _prefix, _name1, _name2)                    \
	case _enum::_name1:                                                    \
		return _prefix##_name2


RtspStreamMuxer::RtspStreamMuxer(Session *session,
				 Element::Listener *elementListener,
				 IPdraw::IMuxer::Listener *listener,
				 MuxerWrapper *wrapper,
				 const std::string &url,
				 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, wrapper, params),
		mUrl(RtspUrl::create(url))
{
	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);
	setCodedVideoMediaFormatCaps(supportedFormats.data(),
				     supportedFormats.size());

	mStats.type = PDRAW_MUXER_TYPE_RTSP;
}


RtspStreamMuxer::~RtspStreamMuxer()
{
	int err;

	(void)internalStop();

	if (mSdpSession != nullptr)
		sdp_session_destroy(mSdpSession);

	cleanupRtspRequests();

	if (mRtspClient != nullptr) {
		err = rtsp_client_destroy(mRtspClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("rtsp_client_destroy", -err);
	}
}


/* Must be called on the loop thread */
int RtspStreamMuxer::addInputMedia(
	Media *media,
	const struct pdraw_muxer_media_params *params)
{
	int res;

	/* Only accept coded video media */
	auto *m = dynamic_cast<CodedVideoMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -ENOSYS;
	}

	auto videoMedia = make_unique<VideoMedia>(this, mParams.rtsp_transport);
	res = videoMedia->setup(CONTROL_URL, media);
	if (res < 0) {
		PDRAW_LOG_ERRNO("VideoMedia::setup", -res);
		return res;
	}

	/* Update session name */
	free(mSdpSession->session_name);
	mSdpSession->session_name = strdup(videoMedia->getCName());

	res = Muxer::addInputMedia(m, params);
	if (res < 0)
		return res;

	mVideoMedias.push_back(std::move(videoMedia));
	return 0;
}


int RtspStreamMuxer::removeInputMedia(Media *media)
{
	for (auto &m : mVideoMedias) {
		if (m->hasMedia(media)) {
			teardownVideoMedia(m.get());
			m->clearMedia();
		}
	}

	return Muxer::removeInputMedia(media);
}


int RtspStreamMuxer::setDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	uint32_t newSize = dyn_params->socket_tx_buffer_size;

	if (newSize != 0 && mRtspClient != nullptr) {
		int ret =
			rtsp_client_set_socket_txbuf_size(mRtspClient, newSize);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("rtsp_client_set_socket_txbuf_size",
					-ret);
			return ret;
		}
	}

	if (newSize != 0)
		mSocketTxBufferSize = newSize;

	return 0;
}


int RtspStreamMuxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	*dyn_params = {};

	dyn_params->socket_tx_buffer_size = mSocketTxBufferSize;

	return 0;
}


int RtspStreamMuxer::getStats(struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	*stats = mStats;

	return 0;
}


int RtspStreamMuxer::record()
{
	int res = 0;
	struct rtsp_range range = {};

	range.start.format = RTSP_TIME_FORMAT_NPT;
	range.start.npt.now = 1;
	range.stop.format = RTSP_TIME_FORMAT_NPT;
	range.stop.npt.infinity = 1;

	res = rtsp_client_record(
		mRtspClient,
		mRtspSessionId.empty() ? nullptr : mRtspSessionId.c_str(),
		&range,
		nullptr,
		0,
		nullptr,
		RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_record", -res);
		return res;
	}

	return 0;
}


int RtspStreamMuxer::internalStart()
{
	int res;

	if (!mUrl) {
		PDRAW_LOGE("invalid RTSP URL");
		return -EINVAL;
	}

	mSdpSession = sdp_session_new();
	if (mSdpSession == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("sdp_session_new", -res);
		return res;
	}

	mSdpSession->session_id = futils_randomr64();
	mSdpSession->session_version = 1;
	/* Will be overriden once the client is connected */
	mSdpSession->server_addr = strdup("0.0.0.0");
	/* Will be overriden once when media is added */
	mSdpSession->session_name = strdup("Empty session");
	mSdpSession->connection_addr = strdup(mLocalHost.c_str());
	mSdpSession->control_url = strdup("*");
	mSdpSession->start_mode = SDP_START_MODE_SENDONLY;
	mSdpSession->tool = strdup("PDrAW RTSP StreamMuxer");

	std::string userAgent{};
	mSession->getSettings()->getSoftwareVersion(&userAgent);
	if (userAgent.empty())
		mSession->getSettings()->getFriendlyName(&userAgent);

	/* Create the RTSP client and connect */
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

	if (mSocketTxBufferSize != 0) {
		res = rtsp_client_set_socket_txbuf_size(mRtspClient,
							mSocketTxBufferSize);
		if (res < 0)
			PDRAW_LOGW_ERRNO("rtsp_client_set_socket_txbuf_size",
					 -res);
	}

	res = rtsp_client_connect(mRtspClient,
				  mUrl->getBaseUrlWithAuth().c_str());
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_connect", -res);
		return res;
	}

	return 0;
}


void RtspStreamMuxer::asyncCompleteTeardown()
{
	int err = pomp_loop_idle_add_with_cookie(
		this->mSession->getLoop(), idleCompleteTeardown, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void RtspStreamMuxer::idleCompleteTeardown(void *userdata)
{
	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	self->completeTeardown();
}


void RtspStreamMuxer::teardownVideoMedia(RtspStreamMuxer::VideoMedia *media)
{
	PDRAW_LOG_ERRNO_RETURN_IF(media == nullptr, EINVAL);

	if (media->isTearingDown())
		return;

	media->teardown();
	media->setTearingDown();
	media->flush();
	asyncCompleteTeardown();
}


void RtspStreamMuxer::teardownAllVideoMedias()
{
	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;
		teardownVideoMedia(m.get());
	}
}


void RtspStreamMuxer::notifyVideoMediaStatsUpdate(
	const RtspStreamMuxer::VideoMedia *media)
{
	/* Reset stats before aggregation */
	mStats.rtsp.receiver_report_count = 0;
	mStats.rtsp.total_packet_count = 0;
	mStats.rtsp.dropped_packet_count = 0;
	mStats.rtsp.lost_packet_count = 0;
	mStats.rtsp.rtd = UINT32_MAX;

	/* Temporary variables to identify the representative (main) stream */
	uint32_t maxPackets = 0;
	uint32_t representativeJitter = 0;
	float representativeLostFraction = 0.f;
	uint32_t representativeRtd = UINT32_MAX;

	for (const auto &m : mVideoMedias) {
		if (m->isTearingDown())
			continue;

		const struct VideoMediaStats &stats = m->getStats();

		/* Sum cumulative counters for the whole RTSP session */
		mStats.rtsp.receiver_report_count += stats.receiverReportCount;
		mStats.rtsp.total_packet_count += stats.pktCountTotal;
		mStats.rtsp.dropped_packet_count += stats.pktCountDropped;
		mStats.rtsp.lost_packet_count += stats.pktCountLost;

		/* Identify the stream with the highest throughput.
		 * This stream is the most sensitive to network congestion
		 * and best represents the user experience. */
		if (stats.pktCountTotal >= maxPackets) {
			maxPackets = stats.pktCountTotal;
			representativeJitter = stats.jitter;
			representativeLostFraction = stats.lostFraction;
			/* Prioritize RTD from the main stream if available */
			if (stats.rtd != UINT32_MAX)
				representativeRtd = stats.rtd;
		}

		/* RTD Fallback: if the main stream hasn't reported an RTD
		 * yet, use any valid RTD reported by another active media. */
		if (representativeRtd == UINT32_MAX && stats.rtd != UINT32_MAX)
			representativeRtd = stats.rtd;
	}

	/* Finalize quality indicators using the representative stream data */
	mStats.rtsp.jitter = representativeJitter;
	mStats.rtsp.lost_fraction = representativeLostFraction;
	mStats.rtsp.rtd = representativeRtd;
}


void RtspStreamMuxer::destroyAllVideoMedias()
{
	mVideoMedias.clear();
}


void RtspStreamMuxer::completeTeardown()
{
	destroyAllVideoMedias();

	if (mState == State::STOPPING) {
		mChannelsReadyForStop = true;
		/* If network is also ready, set the state to stopped */
		tryCompleteStop();
	}
}


void RtspStreamMuxer::tryCompleteStop()
{
	if (mState != State::STOPPING)
		return;

	if (!mNetworkReadyForStop || !mChannelsReadyForStop)
		return;

	destroyAllVideoMedias();

	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;

	mReadyToStop = true;

	Muxer::completeStop();
}


int RtspStreamMuxer::internalStop()
{
	int ret;
	bool disconnect = false;

	/* Stop is pending, idleCompleteStop will be called when the writer
	 * thread exits */
	if (mPendingStop)
		return 0;

	/* Free the RTSP client */
	mChannelsReadyForStop = false;
	mNetworkReadyForStop = false;
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
	if (disconnect) {
		ret = rtsp_client_disconnect(mRtspClient);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("rtsp_client_disconnect", -ret);
			return ret;
		}
	}

	ret = flush();
	if ((ret < 0) && (ret != -EALREADY))
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* Stop will be async */
	mReadyToStop = false;
	mPendingStop = true;

	tryCompleteStop();

	return 0;
}


int RtspStreamMuxer::flush(bool discard)
{
	for (const auto &m : mVideoMedias)
		m->flush(discard);

	return 0;
}


int RtspStreamMuxer::process()
{
	if (mState != State::STARTED)
		return 0;

	Sink::lock();
	for (auto &media : mVideoMedias)
		media->process();
	Sink::unlock();

	return 0;
}


void RtspStreamMuxer::onChannelFlush(Channel *channel)
{
	Muxer::onChannelFlush(channel);
}


void RtspStreamMuxer::onChannelDrain(Channel *channel)
{
	int err;

	/* Process all remaining frames on the channel */
	err = process();
	if (err < 0)
		PDRAW_LOG_ERRNO("process", -err);

	Muxer::onChannelDrain(channel);
}


void RtspStreamMuxer::setRtspState(RtspStreamMuxer::RtspState state)
{
	if (state == mRtspState)
		return;

	mRtspState = state;
	PDRAW_LOGI("RTSP state change to %s", getRtspStateStr(mRtspState));

	mStats.rtsp.is_connected = (mRtspState == RtspState::SETUP_DONE);
	mHasBeenConnected |= (mRtspState == RtspState::CONNECTED);

	/* Notify connection state changed */
	onConnectionStateChanged(
		rtspStateToMuxerConnectionState(mRtspState),
		PDRAW_MUXER_DISCONNECTION_REASON_UNKNOWN); /* TODO */
}


/* codecheck_ignore[COMPLEX_MACRO] */
#define MAP_ENUM_CASE(_prefix1, _prefix2, _name)                               \
	case _prefix1##_name:                                                  \
		return _prefix2##_name

/* codecheck_ignore[COMPLEX_MACRO] */
#define ENUM_CLASS_CASE(_enum, _name)                                          \
	case _enum::_name:                                                     \
		return #_name


const char *RtspStreamMuxer::getRtspStateStr(RtspStreamMuxer::RtspState val)
{
	/* clang-format off */
	switch (val) {
	ENUM_CLASS_CASE(RtspState, DISCONNECTED);
	ENUM_CLASS_CASE(RtspState, CONNECTED);
	ENUM_CLASS_CASE(RtspState, OPTIONS_DONE);
	ENUM_CLASS_CASE(RtspState, ANNOUNCE_DONE);
	ENUM_CLASS_CASE(RtspState, SETUP_DONE);
	default:
		return "UNKNOWN";
	}
	/* clang-format on */
}


enum pdraw_muxer_connection_state
RtspStreamMuxer::rtspStateToMuxerConnectionState(RtspStreamMuxer::RtspState val)
{
	/* clang-format off */
	switch (val) {
	MAP_ENUM_CLASS_CASE(RtspState,
			    PDRAW_MUXER_CONNECTION_STATE_,
			    DISCONNECTED,
			    DISCONNECTED);
	MAP_ENUM_CLASS_CASE(RtspState,
			    PDRAW_MUXER_CONNECTION_STATE_,
			    CONNECTED,
			    CONNECTING);
	MAP_ENUM_CLASS_CASE(RtspState,
			    PDRAW_MUXER_CONNECTION_STATE_,
			    OPTIONS_DONE,
			    CONNECTING);
	MAP_ENUM_CLASS_CASE(RtspState,
			    PDRAW_MUXER_CONNECTION_STATE_,
			    ANNOUNCE_DONE,
			    CONNECTING);
	MAP_ENUM_CLASS_CASE(RtspState,
			    PDRAW_MUXER_CONNECTION_STATE_,
			    SETUP_DONE,
			    CONNECTED);

	default: return PDRAW_MUXER_CONNECTION_STATE_UNKNOWN;
	}
	/* clang-format on */
}


enum pdraw_muxer_rtsp_transport
RtspStreamMuxer::rtspLowerTransportToPdrawMuxerRtspTransport(
	enum rtsp_lower_transport rtspTransport)
{
	/* clang-format off */
	switch (rtspTransport) {
	MAP_ENUM_CASE(RTSP_LOWER_TRANSPORT_,
		      PDRAW_MUXER_RTSP_TRANSPORT_,
		      UDP);
	MAP_ENUM_CASE(RTSP_LOWER_TRANSPORT_,
		      PDRAW_MUXER_RTSP_TRANSPORT_,
		      TCP);

	default: return PDRAW_MUXER_RTSP_TRANSPORT_UDP;
	}
	/* clang-format on */
}


enum rtsp_lower_transport
RtspStreamMuxer::pdrawMuxerRtspTransportToRtspLowerTransport(
	enum pdraw_muxer_rtsp_transport muxerTransport)
{
	/* clang-format off */
	switch (muxerTransport) {
	MAP_ENUM_CASE(PDRAW_MUXER_RTSP_TRANSPORT_,
		      RTSP_LOWER_TRANSPORT_,
		      UDP);
	MAP_ENUM_CASE(PDRAW_MUXER_RTSP_TRANSPORT_,
		      RTSP_LOWER_TRANSPORT_,
		      TCP);

	default: return RTSP_LOWER_TRANSPORT_UDP;
	}
	/* clang-format on */
}


int RtspStreamMuxer::processRtspRequests()
{
	int res;

	/* Not ready */
	if (mRtspState != RtspState::ANNOUNCE_DONE &&
	    mRtspState != RtspState::SETUP_DONE)
		return 0;

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


void RtspStreamMuxer::cleanupRtspRequests()
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


int RtspStreamMuxer::processSetupRequest()
{
	int res;

	if (mSetupRequests.empty()) {
		if (mSetupRequestsCount > 0) {
			/* Returning -EBUSY here to notify that there
			 * are still setup requests in progress (for
			 * example asynchronous requests from the
			 * RtspStreamMuxer subclass that are
			 * not yet in the queue) */
			return -EBUSY;
		} else {
			/* All setup requests have been processed;
			 * nothing more to do */
			return 0;
		}
	}

	/* Process the next setup request */
	SetupRequest req = mSetupRequests.front();

	res = rtsp_client_setup(mRtspClient,
				mUrl->getUrl().c_str(),
				req.controlUrl.c_str(),
				mRtspSessionId.empty() ? nullptr
						       : mRtspSessionId.c_str(),
				RTSP_DELIVERY_UNICAST,
				req.lowerTransport,
				req.localStreamPort,
				req.localControlPort,
				RTSP_TRANSPORT_METHOD_RECORD,
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


int RtspStreamMuxer::processTeardownRequest()
{
	int res;

	if (mTeardownRequests.empty()) {
		if (mTeardownRequestsCount > 0) {
			/* Returning -EBUSY here to notify that there are still
			 * teardown requests in progress (for example
			 * asynchronous requests from the RtspStreamMuxer
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

	res = rtsp_client_teardown(
		mRtspClient,
		req.controlUrl.c_str(),
		mRtspSessionId.empty() ? nullptr : mRtspSessionId.c_str(),
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


void RtspStreamMuxer::onRtspSocketCreated(int fd, void *userdata)
{
	auto *self = static_cast<RtspStreamMuxer *>(userdata);

	const struct rtsp_url *url =
		rtsp_client_get_remote_url(self->mRtspClient);
	if (url != nullptr) {
		const char *resolved_host = rtsp_url_get_resolved_host(url);
		if (resolved_host != nullptr) {
			self->mUrl->setResolvedHost(resolved_host);
			/* Update server addr */
			free(self->mSdpSession->server_addr);
			self->mSdpSession->server_addr =
				strdup(self->mUrl->getResolvedHost().c_str());
		}
	}

	self->mSession->socketCreated(fd);
}


void RtspStreamMuxer::onReadyToSendCb(struct rtsp_client *client,
				      void *userdata)
{
	auto *self = static_cast<RtspStreamMuxer *>(userdata);

	for (const auto &m : self->mVideoMedias)
		m->notifyReadyToSend();
}


void RtspStreamMuxer::onRtspInterleavedDataCb(struct rtsp_client *client,
					      uint8_t channel,
					      const uint8_t *data,
					      size_t len,
					      void *userdata)
{
	int res;
	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	struct pomp_buffer *buf = nullptr;
	struct tpkt_packet *pkt = nullptr;
	bool found = false;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

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

	res = time_get_monotonic(&ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_get_monotonic", -res);
		pomp_buffer_unref(buf);
		return;
	}
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_timespec_to_us", -res);
		pomp_buffer_unref(buf);
		return;
	}
	res = tpkt_set_timestamp(pkt, curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_set_timestamp", -res);
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


void RtspStreamMuxer::asyncRtspDisconnect()
{
	int err = pomp_loop_idle_add_with_cookie(
		this->mSession->getLoop(), idleRtspDisconnect, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void RtspStreamMuxer::idleRtspDisconnect(void *userdata)
{
	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	int res;

	if (self->mRtspState == RtspState::DISCONNECTED)
		return;

	res = rtsp_client_disconnect(self->mRtspClient);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtsp_client_disconnect", -res);
		return;
	}
}


void RtspStreamMuxer::onRtspConnectionState(struct rtsp_client *client,
					    enum rtsp_client_conn_state state,
					    void *userdata)
{
	PDRAW_UNUSED(client);

	int err = 0;
	auto *self = static_cast<RtspStreamMuxer *>(userdata);

	/* Reset flag */
	self->mAnnounceRetried = false;
	self->mRtspConnectionState = state;

	PDRAW_LOGI("RTSP client %s", rtsp_client_conn_state_str(state));

	switch (state) {
	case RTSP_CLIENT_CONN_STATE_DISCONNECTED:
		self->setRtspState(RtspState::DISCONNECTED);
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

		err = self->sendOptions();
		if (err < 0)
			PDRAW_LOG_ERRNO("sendOptions", -err);
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


void RtspStreamMuxer::onRtspSessionRemoved(struct rtsp_client *client,
					   const char *session_id,
					   int status,
					   void *userdata)
{
	PDRAW_UNUSED(client);

	constexpr const char *OP = "session remove";
	constexpr const char *OP_EVT = "client_session_removed";

	auto *self = static_cast<RtspStreamMuxer *>(userdata);

	if (!self->checkSessionId(session_id, OP))
		return;

	self->mRtspSessionId = {};

	self->logEventSession(OP_EVT, status, session_id);

	self->teardownAllVideoMedias();

	self->mRecording = false;

	if (self->mRtspState != RtspState::DISCONNECTED)
		self->setRtspState(RtspState::OPTIONS_DONE);

	if (self->mState == State::STOPPING) {
		if (self->mRtspState != RtspState::DISCONNECTED)
			self->asyncRtspDisconnect();
	} else {
		/* We used to call stop() here; this is no longer the
		 * case because now a demuxer must not stop itself; we
		 * call unrecoverableError instead, and it is the
		 * application's responsibility to stop and destroy the
		 * demuxer and create a new one. */
		self->onUnrecoverableError(-EPROTO);
	}
}


int RtspStreamMuxer::sendOptions()
{
	int res = rtsp_client_options(mRtspClient,
				      nullptr,
				      0,
				      nullptr,
				      RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0)
		PDRAW_LOG_ERRNO("rtsp_client_options", -res);
	return res;
}


int RtspStreamMuxer::sendAnnounce()
{
	int res;
	char *rawSdp = nullptr;

	res = sdp_description_write(mSdpSession, &rawSdp);
	if (res < 0) {
		PDRAW_LOG_ERRNO("sdp_description_write", -res);
		return res;
	}
	std::unique_ptr<char, decltype(&free)> sdp(rawSdp, &free);

	res = rtsp_client_announce(mRtspClient,
				   mUrl->getStreamName().c_str(),
				   sdp.get(),
				   nullptr,
				   0,
				   nullptr,
				   RTSP_CLIENT_DEFAULT_RESP_TIMEOUT_MS);
	if (res < 0)
		PDRAW_LOG_ERRNO("rtsp_client_announce", -res);
	return res;
}


bool RtspStreamMuxer::checkSessionId(const char *sessionId, const char *op)
{
	if (sessionId == nullptr) {
		PDRAW_LOGE("empty session id");
		return false;
	}
	if (mRtspSessionId.compare(sessionId) != 0) {
		PDRAW_LOGE(
			"RTSP %s for a wrong session"
			" (%s instead of %s)",
			op,
			sessionId,
			mRtspSessionId.c_str());
		return false;
	}
	return true;
}


int RtspStreamMuxer::checkReqStatus(int status,
				    enum rtsp_client_req_status req_status,
				    const char *op)
{
	int ret;

	if (req_status == RTSP_CLIENT_REQ_STATUS_OK)
		return 0;

	switch (req_status) {
	case RTSP_CLIENT_REQ_STATUS_FAILED:
		PDRAW_LOGE("RTSP %s request failed (%d: %s)",
			   op,
			   status,
			   strerror(-status));
		ret = status;
		break;
	case RTSP_CLIENT_REQ_STATUS_ABORTED:
		PDRAW_LOGE("RTSP %s request aborted", op);
		ret = -EPROTO;
		break;
	case RTSP_CLIENT_REQ_STATUS_CANCELED:
		PDRAW_LOGE("RTSP %s request canceled", op);
		ret = -ECANCELED;
		break;
	case RTSP_CLIENT_REQ_STATUS_TIMEOUT:
		PDRAW_LOGE("timeout on RTSP %s request", op);
		ret = -ETIMEDOUT;
		break;
	default:
		/* This should not happen */
		PDRAW_LOGE(
			"unexpected status on %s request: %d", op, req_status);
		ret = -EPROTO;
		break;
	}

	return ret;
}


void RtspStreamMuxer::onRtspOptionsResp(struct rtsp_client *client,
					enum rtsp_client_req_status req_status,
					int status,
					uint32_t methods,
					const struct rtsp_header_ext *ext,
					size_t ext_count,
					void *userdata,
					void *req_userdata)
{
	PDRAW_UNUSED(client);
	PDRAW_UNUSED(methods);
	PDRAW_UNUSED(ext);
	PDRAW_UNUSED(ext_count);
	PDRAW_UNUSED(req_userdata);

	constexpr const char *OP = "options";
	constexpr const char *OP_EVT = "client_options_resp";

	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	int res = 0;

	res = self->checkReqStatus(status, req_status, OP);
	if (res < 0) {
		self->logEvent(OP_EVT, res);
		self->onUnrecoverableError(res);
		return;
	}

	self->logEvent(OP_EVT, res);
	self->setRtspState(RtspState::OPTIONS_DONE);

	res = self->sendAnnounce();
	if (res < 0)
		PDRAW_LOG_ERRNO("sendAnnounce", -res);
}


void RtspStreamMuxer::onRtspAnnounceResp(struct rtsp_client *client,
					 enum rtsp_client_req_status req_status,
					 int status,
					 const struct rtsp_header_ext *ext,
					 size_t ext_count,
					 void *userdata,
					 void *req_userdata)
{
	PDRAW_UNUSED(client);
	PDRAW_UNUSED(ext);
	PDRAW_UNUSED(ext_count);
	PDRAW_UNUSED(req_userdata);

	constexpr const char *OP = "announce";
	constexpr const char *OP_EVT = "client_announce_resp";

	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	int res = 0;

	res = self->checkReqStatus(status, req_status, OP);
	if (res < 0) {
		self->logEvent(OP_EVT, res);
		if ((status == -EPERM) && !self->mAnnounceRetried) {
			res = self->sendAnnounce();
			if (res < 0)
				PDRAW_LOG_ERRNO("sendAnnounce", -res);
			self->mAnnounceRetried = true;
		} else {
			self->onUnrecoverableError(res);
			return;
		}
	}

	self->logEvent(OP_EVT, res);
	self->setRtspState(RtspState::ANNOUNCE_DONE);
	self->processRtspRequests();
}


void RtspStreamMuxer::onRtspSetupResp(struct rtsp_client *client,
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
	PDRAW_UNUSED(client);
	PDRAW_UNUSED(ext);
	PDRAW_UNUSED(ext_count);
	PDRAW_UNUSED(req_userdata);

	constexpr const char *OP = "setup";
	constexpr const char *OP_EVT = "client_setup_resp";

	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	auto *media = static_cast<VideoMedia *>(req_userdata);
	int res = 0;

	res = self->checkReqStatus(status, req_status, OP);
	if (res < 0) {
		self->logEventSetupResp(
			OP_EVT, status, session_id, media, 0, 0, false);
		self->onUnrecoverableError(res);
		return;
	}

	self->mRtspSessionId = std::string(session_id);

	if (media != nullptr) {
		media->setSsrc(ssrc_valid ? ssrc : 0);
		media->setRemoteStreamPort(server_stream_port);
		media->setRemoteControlPort(server_control_port);

		res = media->startRtpAvp();
		if (res < 0)
			PDRAW_LOG_ERRNO("startRtpAvp", -res);
	}

	self->logEventSetupResp(OP_EVT,
				status,
				session_id,
				media,
				server_stream_port,
				server_control_port,
				true);

	if (res < 0) {
		self->onUnrecoverableError(res);
		return;
	}

	self->setRtspState(RtspState::SETUP_DONE);
	/* FIXME handle several medias */
	self->record();
	self->processRtspRequests();
}


void RtspStreamMuxer::onRtspRecordResp(struct rtsp_client *client,
				       const char *session_id,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       void *userdata,
				       void *req_userdata)
{
	constexpr const char *OP = "record";
	constexpr const char *OP_EVT = "client_record_resp";

	int res = 0;
	auto *self = static_cast<RtspStreamMuxer *>(userdata);

	res = self->checkReqStatus(status, req_status, OP);
	if (res < 0) {
		self->logEventSession(OP_EVT, res, session_id);
		self->onUnrecoverableError(res);
		return;
	}

	if (!self->checkSessionId(session_id, OP))
		return;

	self->logEventSession(OP_EVT, res, session_id);
	self->mRecording = true;
	self->processRtspRequests();
}


void RtspStreamMuxer::onRtspTeardownResp(struct rtsp_client *client,
					 const char *session_id,
					 enum rtsp_client_req_status req_status,
					 int status,
					 const struct rtsp_header_ext *ext,
					 size_t ext_count,
					 void *userdata,
					 void *req_userdata)
{
	constexpr const char *OP = "teardown";
	constexpr const char *OP_EVT = "client_teardown_resp";

	int res = 0;
	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	auto *media = static_cast<VideoMedia *>(req_userdata);

	auto it = std::find_if(self->mVideoMedias.begin(),
			       self->mVideoMedias.end(),
			       [media](const std::unique_ptr<VideoMedia> &m) {
				       return m.get() == media;
			       });

	if (it == self->mVideoMedias.end())
		media = nullptr;

	res = self->checkReqStatus(status, req_status, OP);

	if (!self->checkSessionId(session_id, OP))
		return;

	self->logEventMedia(OP_EVT, res, session_id, media);
	if (media != nullptr)
		self->teardownVideoMedia(media);
	self->processRtspRequests();
}


void RtspStreamMuxer::onRtspAnnounce(struct rtsp_client *client,
				     const char *content_base,
				     const struct rtsp_header_ext *ext,
				     size_t ext_count,
				     const char *sdp,
				     void *userdata)
{
	PDRAW_UNUSED(client);
	PDRAW_UNUSED(content_base);
	PDRAW_UNUSED(ext);
	PDRAW_UNUSED(ext_count);
	PDRAW_UNUSED(sdp);

	constexpr const char *OP_EVT = "client_announce";

	const auto *self = static_cast<RtspStreamMuxer *>(userdata);

	ULOG_EVT("STREAM", "event='%s';element='%s'", OP_EVT, self->getCName());
}


void RtspStreamMuxer::onRtspForcedTeardown(struct rtsp_client *client,
					   const char *path,
					   const char *session_id,
					   const struct rtsp_header_ext *ext,
					   size_t ext_count,
					   void *userdata)
{
	PDRAW_UNUSED(client);
	PDRAW_UNUSED(ext);
	PDRAW_UNUSED(ext_count);

	constexpr const char *OP = "forced teardown";
	constexpr const char *OP_EVT = "forced_teardown";

	auto *self = static_cast<RtspStreamMuxer *>(userdata);
	VideoMedia *media = nullptr;
	int res = 0;
	bool pathIsContentBase = false;

	if (!self->checkSessionId(session_id, OP))
		return;

	if (xstrcmp(path, self->mUrl->getStreamName().c_str()) == 0) {
		pathIsContentBase = true;
	} else {
		for (const auto &m : self->mVideoMedias) {
			std::string mediaPath = self->mUrl->getStreamName() +
						"/" + m->getControlUrl();
			if (m->isTearingDown())
				continue;
			if (xstrcmp(path, mediaPath.c_str()) == 0) {
				media = m.get();
				break;
			}
		}
	}

	self->logEventMedia(OP_EVT, res, session_id, media, pathIsContentBase);

	self->mRecording = false;

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


void RtspStreamMuxer::logEvent(const char *eventName, int res) const
{
	LOG_STREAM_EVT(eventName,
		       this,
		       res,
		       ";res='%s'",
		       mUrl->getStreamName().c_str());
}


void RtspStreamMuxer::logEventSession(const char *eventName,
				      int res,
				      const char *sessionId) const
{
	LOG_STREAM_EVT(eventName,
		       this,
		       res,
		       ";session='%s';res='%s'",
		       sessionId ? sessionId : "",
		       mUrl->getStreamName().c_str());
}


void RtspStreamMuxer::logEventMedia(const char *eventName,
				    int res,
				    const char *sessionId,
				    RtspStreamMuxer::VideoMedia *media,
				    bool pathIsContentBase) const
{
	LOG_STREAM_EVT(eventName,
		       this,
		       res,
		       ";session='%s';res='%s';media='%s'",
		       sessionId ? sessionId : "?",
		       mUrl->getStreamName().c_str(),
		       media ? media->getCControlUrl()
			     : (pathIsContentBase ? "all" : "?"));
}


void RtspStreamMuxer::logEventSetupResp(const char *eventName,
					int res,
					const char *sessionId,
					RtspStreamMuxer::VideoMedia *media,
					uint16_t srcStrmPort,
					uint16_t srcCtrlPort,
					bool success) const
{
	uint16_t _srcStrmPort = success ? srcStrmPort : (uint16_t)0;
	uint16_t _srcCtrlPort = success ? srcCtrlPort : (uint16_t)0;
	uint16_t dstStrmPort =
		(success && media) ? media->getLocalStreamPort() : (uint16_t)0;
	uint16_t dstCtrlPort =
		(success && media) ? media->getLocalControlPort() : (uint16_t)0;

	LOG_STREAM_EVT(eventName,
		       this,
		       res,
		       ";session='%s';"
		       "res='%s';media='%s';src='%s:%" PRIu16 ",%" PRIu16
		       "';dst='%s:%" PRIu16 ",%" PRIu16 "'",
		       sessionId ? sessionId : "?",
		       mUrl->getStreamName().c_str(),
		       media ? media->getCControlUrl() : "?",
		       mUrl->getResolvedHost().c_str(),
		       _srcStrmPort,
		       _srcCtrlPort,
		       mLocalHost.c_str(),
		       dstStrmPort,
		       dstCtrlPort);
}

} /* namespace Pdraw */
