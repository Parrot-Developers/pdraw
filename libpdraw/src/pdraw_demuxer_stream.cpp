/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_demuxer_stream.hpp"
#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_log.hpp"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <futils/futils.h>
#include <string>
#include <algorithm>

namespace Pdraw {


#define DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT 55004
#define DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT 55005


StreamDemuxer::StreamDemuxer(
	Session *session)
{
	int ret, mutex_created = 0, cond_created = 0;
	struct rtsp_client_cbs rtspClientCbs = {
		.connection_state = &onRtspConnectionState,
		.options_resp = &onRtspOptionsResp,
		.describe_resp = &onRtspDescribeResp,
		.setup_resp = &onRtspSetupResp,
		.play_resp = &onRtspPlayResp,
		.pause_resp = &onRtspPauseResp,
		.teardown_resp = &onRtspTeardownResp,
	};

	if (session == NULL) {
		ULOGE("StreamDemuxer: invalid session");
		return;
	}

	mSession = session;
	mConfigured = false;
	mRtspRunning = false;
	mRtspClient = NULL;
	mStreamSock = NULL;
	mControlSock = NULL;
	mReceiver = NULL;
	mCurrentBuffer = NULL;
	mDecoder = NULL;
	mDecoderBitstreamFormat = AVCDECODER_BITSTREAM_FORMAT_UNKNOWN;
	mStartTime = 0;
	mCurrentTime = 0;
	mDuration = 0;
	mPausePoint = 0;
	mNtpToNptOffset = 0;
	mWidth = mHeight = 0;
	mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
	mSarWidth = mSarHeight = 0;
	mHfov = mVfov = 0.;
	mRunning = false;
	mSpeed = 1.0;
	mFirstFrame = true;
	mSsrc = 0;
	memset(&mCodecInfo, 0, sizeof(mCodecInfo));

	const char *userAgent = NULL;
	SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();

	userAgent = selfMeta->getSoftwareVersion().c_str();
	ret = rtsp_client_new(mSession->getLoop(), userAgent, &rtspClientCbs,
			this, &mRtspClient);
	if (ret < 0) {
		ULOG_ERRNO("StreamDemuxer: rtsp_client_new() failed", -ret);
		goto err;
	}

	struct h264_ctx_cbs h264_cbs;
	memset(&h264_cbs, 0, sizeof(h264_cbs));
	h264_cbs.userdata = this;
	h264_cbs.sei_user_data_unregistered = &h264UserDataSeiCb;
	ret = h264_reader_new(&h264_cbs, &mH264Reader);
	if (ret < 0) {
		ULOGE("StreamDemuxer: h264_reader_new() failed (%d)", ret);
		goto err;
	}

	ret = pthread_mutex_init(&mDemuxerMutex, NULL);
	if (ret != 0) {
		ULOGE("StreamDemuxer: mutex creation failed (%d)", ret);
		goto err;
	}
	mutex_created = 1;
	ret = pthread_cond_init(&mDemuxerCond, NULL);
	if (ret != 0) {
		ULOGE("StreamDemuxer: cond creation failed (%d)", ret);
		goto err;
	}
	cond_created = 1;

	return;

err:
	if (mutex_created)
		pthread_mutex_destroy(&mDemuxerMutex);
	if (cond_created)
		pthread_cond_destroy(&mDemuxerCond);
	if (mH264Reader != NULL) {
		h264_reader_destroy(mH264Reader);
		mH264Reader = NULL;
	}
	if (mRtspClient) {
		rtsp_client_destroy(mRtspClient);
		mRtspClient = NULL;
	}
}


StreamDemuxer::~StreamDemuxer(
	void)
{
	int ret = close();
	if (ret != 0)
		ULOGE("StreamDemuxer: close() failed (%d)", ret);

	if (mCurrentBuffer != NULL)
		vbuf_unref(&mCurrentBuffer);

	if (mRtspClient != NULL) {
		do {
			ret = rtsp_client_destroy(mRtspClient);
			if ((ret != 0) && (ret != -EBUSY))
				ULOGE("StreamDemuxer: rtsp_client_destroy() "
					"failed");
			usleep(1000);
		} while (ret == -EBUSY);
		mRtspClient = NULL;
	}

	if (mH264Reader != NULL) {
		h264_reader_destroy(mH264Reader);
		mH264Reader = NULL;
	}

	if (mReceiver != NULL) {
		vstrm_receiver_destroy(mReceiver);
		mReceiver = NULL;
	}

	if (mStreamSock != NULL) {
		delete mStreamSock;
		mStreamSock = NULL;
	}

	if (mControlSock != NULL) {
		delete mControlSock;
		mControlSock = NULL;
	}

	pthread_mutex_destroy(&mDemuxerMutex);
	pthread_cond_destroy(&mDemuxerCond);
}


int StreamDemuxer::configureRtpAvp(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();
	struct vstrm_receiver_cfg cfg;
	struct vstrm_receiver_cbs cbs;
	int ret;

	/* Create the sockets */
	mStreamSock = new InetSocket(localAddr, localStreamPort,
		remoteAddr, remoteStreamPort,
		mSession->getLoop(), dataCb, this);
	if (mStreamSock == NULL) {
		ULOGE("StreamDemuxer: failed to create stream socket");
		ret = -EPROTO;
		goto error;
	}
	mControlSock = new InetSocket(localAddr, localControlPort,
		remoteAddr, remoteControlPort,
		mSession->getLoop(), ctrlCb, this);
	if (mControlSock == NULL) {
		ULOGE("StreamDemuxer: failed to create control socket");
		ret = -EPROTO;
		goto error;
	}

	/* Create the stream receiver */
	memset(&cfg, 0, sizeof(cfg));
	cfg.loop = mSession->getLoop();
	cfg.flags = VSTRM_RECEIVER_FLAGS_H264_GEN_SKIPPED_P_SLICE |
			VSTRM_RECEIVER_FLAGS_H264_GEN_GREY_I_FRAME |
			VSTRM_RECEIVER_FLAGS_ENABLE_RTCP |
			VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT |
			VSTRM_RECEIVER_FLAGS_H264_FILTER_SPS_PPS |
			VSTRM_RECEIVER_FLAGS_H264_FILTER_SEI;
	strncpy(cfg.self_meta.friendly_name,
		selfMeta->getFriendlyName().c_str(),
		sizeof(cfg.self_meta.friendly_name));
	strncpy(cfg.self_meta.serial_number,
		selfMeta->getSerialNumber().c_str(),
		sizeof(cfg.self_meta.serial_number));
	strncpy(cfg.self_meta.software_version,
		selfMeta->getSoftwareVersion().c_str(),
		sizeof(cfg.self_meta.software_version));
	memset(&cbs, 0, sizeof(cbs));
	cbs.userdata = this;
	cbs.send_ctrl = &sendCtrlCb;
	cbs.codec_info_changed = &codecInfoChangedCb;
	cbs.recv_frame = &recvFrameCb;
	cbs.session_metadata_peer_changed = sessionMetadataPeerChangedCb;
	ret = vstrm_receiver_new(&cfg, &cbs, &mReceiver);
	if (ret < 0) {
		mReceiver = NULL;
		ULOGE("StreamDemuxer: vstrm_receiver_new() failed (%d)", ret);
		goto error;
	}

	/* Provide the SPS/PPS out of band if available */
	if (mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) {
		ret = vstrm_receiver_set_codec_info(mReceiver,
			&mCodecInfo, mSsrc);
		if (ret < 0) {
			ULOGW("StreamDemuxer: vstrm_receiver_set_codec_info() "
				"failed (%d)", ret);
		}
	}

	return 0;

error:
	if (mReceiver != NULL) {
		vstrm_receiver_destroy(mReceiver);
		mReceiver = NULL;
	}
	if (mStreamSock != NULL) {
		delete mStreamSock;
		mStreamSock = NULL;
	}
	if (mControlSock != NULL) {
		delete mControlSock;
		mControlSock = NULL;
	}

	return ret;
}


int StreamDemuxer::configureSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	int ret;
	char *remoteAddr = NULL;
	int localStreamPort = 0, localControlPort = 0;
	int remoteStreamPort = 0, remoteControlPort = 0;
	struct sdp_session *session = NULL;

	session = sdp_description_read(sdp.c_str());
	if (!session) {
		ULOGE("StreamDemuxer: sdp_description_read() failed");
		return -1;
	}

	struct sdp_media *media = NULL;
	list_walk_entry_forward(&session->medias, media, node) {
		if (media->type == SDP_MEDIA_TYPE_VIDEO) {
			localStreamPort = media->dst_stream_port;
			localControlPort = media->dst_control_port;
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
			break;
		}
	}
	remoteAddr = (strncmp(session->connection_addr, "0.0.0.0", 7)) ?
		strdup(session->connection_addr) :
		strdup(session->server_addr);
	if (!remoteAddr) {
		ULOGE("StreamDemuxer: failed to get server address");
		return -1;
	}
	if (session->range.start.format == SDP_TIME_FORMAT_NPT) {
		mDuration =
			(uint64_t)session->range.stop.npt.sec * 1000000 +
			(uint64_t)session->range.stop.npt.usec;
	}
	sdp_session_destroy(session);

	std::string remote(remoteAddr);
	ret = configureRtpAvp("0.0.0.0", localStreamPort, localControlPort,
		remote, remoteStreamPort, remoteControlPort, ifaceAddr);
	if (ret != 0) {
		ULOGE("StreamDemuxer: configureRtpAvp() failed");
	}

	free(remoteAddr);

	return ret;
}


void StreamDemuxer::onRtspConnectionState(
	struct rtsp_client *client,
	enum rtsp_conn_state state,
	void *userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	switch (state) {
	case RTSP_CONN_STATE_DISCONNECTED:
		ULOGI("StreamDemuxer: RTSP disconnected");
		self->mRtspRunning = false;
		pthread_cond_signal(&self->mDemuxerCond);
		break;
	case RTSP_CONN_STATE_CONNECTING:
		ULOGI("StreamDemuxer: RTSP connecting");
		break;
	case RTSP_CONN_STATE_CONNECTED:
		ULOGI("StreamDemuxer: RTSP connected");

		res = rtsp_client_options(self->mRtspClient, NULL);
		if (res != 0) {
			ULOGE("StreamDemuxer: rtsp_client_options() "
				"failed (%d)", res);
		}
		break;
	case RTSP_CONN_STATE_DISCONNECTING:
		ULOGI("StreamDemuxer: RTSP disconnecting");
		break;
	}
}


void StreamDemuxer::onRtspOptionsResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
	uint32_t methods,
	void *userdata,
	void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: rtsp_client_options() failed");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	res = rtsp_client_describe(self->mRtspClient, NULL);
	if (res < 0)
		ULOGE("StreamDemuxer: rtsp_client_describe() failed (%d)", res);
}


void StreamDemuxer::onRtspDescribeResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
	const char *sdp,
	void *userdata,
	void *req_userdata)
 {
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;
	struct sdp_session *session = NULL;
	char *mediaUrl = NULL, *remoteAddr = NULL;

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: RTSP describe failed (%d: %s)",
			status_code, rtsp_status_str(status_code));
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	session = sdp_description_read(sdp);
	if (session == NULL) {
		ULOGE("StreamDemuxer: sdp_description_read() failed");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	struct sdp_media *media = NULL;
	list_walk_entry_forward(&session->medias, media, node) {
		if ((media->type == SDP_MEDIA_TYPE_VIDEO) &&
			(media->control_url)) {
			mediaUrl = strdup(media->control_url);
			if ((media->h264_fmtp.valid) &&
				(media->h264_fmtp.sps != NULL) &&
				(media->h264_fmtp.pps != NULL)) {
				memset(&self->mCodecInfo, 0,
					sizeof(self->mCodecInfo));
				self->mCodecInfo.codec = VSTRM_CODEC_VIDEO_H264;
				if (media->h264_fmtp.sps_size <=
					sizeof(self->mCodecInfo.h264.sps)) {
					memcpy(self->mCodecInfo.h264.sps,
						media->h264_fmtp.sps,
						media->h264_fmtp.sps_size);
					self->mCodecInfo.h264.spslen =
						media->h264_fmtp.sps_size;
				}
				if (media->h264_fmtp.pps_size <=
					sizeof(self->mCodecInfo.h264.pps)) {
					memcpy(self->mCodecInfo.h264.pps,
						media->h264_fmtp.pps,
						media->h264_fmtp.pps_size);
					self->mCodecInfo.h264.ppslen =
						media->h264_fmtp.pps_size;
				}
			}
			break;
		}
	}
	remoteAddr = (strcmp(session->connection_addr, "0.0.0.0") != 0) ?
		strdup(session->connection_addr) :
		strdup(session->server_addr);
	if (mediaUrl == NULL) {
		ULOGE("StreamDemuxer: failed to get media control URL");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}
	if (remoteAddr == NULL) {
		ULOGE("StreamDemuxer: failed to get server address");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}
	if (session->range.start.format == SDP_TIME_FORMAT_NPT) {
		self->mDuration =
			(uint64_t)session->range.stop.npt.sec * 1000000 +
			(uint64_t)session->range.stop.npt.usec;
	}
	sdp_session_destroy(session);

	res = rtsp_client_setup(self->mRtspClient, mediaUrl,
			RTSP_DELIVERY_UNICAST, RTSP_LOWER_TRANSPORT_UDP,
			DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT,
			DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT, NULL);
	if (res != 0) {
		ULOGE("StreamDemuxer: rtsp_client_setup() failed (%d)", res);
		free(remoteAddr);
		free(mediaUrl);
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	self->mRemoteAddr = std::string(remoteAddr);

	free(mediaUrl);
	free(remoteAddr);
}


void StreamDemuxer::onRtspSetupResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
	int server_stream_port,
	int server_control_port,
	int ssrc_valid,
	uint32_t ssrc,
	void *userdata,
	void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res = 0;

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: RTSP setup failed (%d: %s)",
			status_code, rtsp_status_str(status_code));
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	self->mSsrc = (ssrc_valid) ? ssrc : 0;

	res = self->configureRtpAvp("0.0.0.0",
		DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT,
		DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT,
		self->mRemoteAddr, server_stream_port, server_control_port,
		self->mIfaceAddr);
	if (res != 0) {
		ULOGE("StreamDemuxer: configureRtpAvp() failed");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}

	self->mRtspRunning = true;
	pthread_cond_signal(&self->mDemuxerCond);
}


void StreamDemuxer::onRtspPlayResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
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

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: RTSP play failed (%d: %s)",
			status_code, rtsp_status_str(status_code));
		return;
	}

	self->mSpeed = (scale != 0.) ? scale : 1.0;
	if (rtptime_valid) {
		ntptime = vstrm_receiver_get_ntp_from_rtp_ts(
			self->mReceiver, rtptime);
	}
	start = (uint64_t)range->start.npt.sec * 1000000 +
		(uint64_t)range->start.npt.usec;
	self->mNtpToNptOffset = (int64_t)(ntptime * self->mSpeed) -
		(int64_t)start;
	self->mPausePoint = (uint64_t)range->stop.npt.sec * 1000000 +
		(uint64_t)range->stop.npt.usec;
}


void StreamDemuxer::onRtspPauseResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
	const struct rtsp_range *range,
	void *userdata,
	void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: RTSP pause failed (%d: %s)",
			status_code, rtsp_status_str(status_code));
		return;
	}

	self->mPausePoint = (uint64_t)range->start.npt.sec * 1000000 +
		(uint64_t)range->start.npt.usec;
}


void StreamDemuxer::onRtspTeardownResp(
	struct rtsp_client *client,
	enum rtsp_req_status status,
	int status_code,
	void *userdata,
	void *req_userdata)
{
	StreamDemuxer *self = reinterpret_cast<StreamDemuxer *>(userdata);
	int res;

	if (status != RTSP_REQ_STATUS_OK) {
		ULOGE("StreamDemuxer: RTSP teardown failed (%d: %s)",
			status_code, rtsp_status_str(status_code));
	}

	res = rtsp_client_disconnect(self->mRtspClient);
	if (res != 0) {
		ULOGE("StreamDemuxer: rtsp_client_disconnect() failed");
		pthread_cond_signal(&self->mDemuxerCond);
		return;
	}
}


int StreamDemuxer::configureRtsp(
	const std::string &url,
	const std::string &ifaceAddr)
{
	int res;

	res = rtsp_client_connect(mRtspClient, url.c_str());
	if (res != 0) {
		ULOGE("StreamDemuxer: v() failed (%d)", res);
		return res;
	}

	pthread_mutex_lock(&mDemuxerMutex);
	pthread_cond_wait(&mDemuxerCond, &mDemuxerMutex);
	pthread_mutex_unlock(&mDemuxerMutex);

	return mRtspRunning ? 0 : -EPROTO;
}


int StreamDemuxer::configure(
	const std::string &url)
{
	return configure(url, "");
}


int StreamDemuxer::configure(
	const std::string &url,
	const std::string &ifaceAddr)
{
	int ret;

	if (mConfigured) {
		ULOGE("StreamDemuxer: demuxer is already configured");
		return -1;
	}
	if (mSession == NULL) {
		ULOGE("StreamDemuxer: invalid session");
		return -1;
	}

	std::string ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if (url.substr(0, 7) == "rtsp://") {
		ret = configureRtsp(url, ifaceAddr);
		if (ret != 0) {
			ULOGE("StreamDemuxer: configureRtsp() failed");
			return -1;
		}
	} else if ((url.substr(0, 7) == "http://") && (ext == ".sdp")) {
		/* TODO */
		ULOGE("StreamDemuxer: unsupported URL");
		return -1;
	} else if ((url.front() == '/') && (ext == ".sdp")) {
		struct stat sb;
		FILE *f = NULL;
		char *s = NULL;

		f = fopen(url.c_str(), "r");
		if (f == NULL) {
			ULOGE("StreamDemuxer: failed to open file '%s'",
				url.c_str());
			return -1;
		}

		ret = fstat(fileno(f), &sb);
		if (ret != 0) {
			ULOGE("StreamDemuxer: stat failed on file '%s'",
				url.c_str());
			return -1;
		}

		s = (char *)calloc(1, sb.st_size + 1);
		if (s == NULL) {
			ULOGE("StreamDemuxer: allocation failed");
			return -1;
		}

		ret = fread(s, sb.st_size, 1, f);
		if (ret != 1) {
			ULOGE("StreamDemuxer: failed to read "
				"from the input file");
			return -1;
		}

		s[sb.st_size] = '\0';
		std::string sdp(s);
		ret = configureSdp(sdp, ifaceAddr);
		if (ret != 0) {
			ULOGE("StreamDemuxer: configureSdp() failed");
			return -1;
		}
	} else {
		ULOGE("StreamDemuxer: unsupported URL");
		return -1;
	}

	mConfigured = true;
	ULOGI("StreamDemuxer: demuxer is configured");

	return ret;
}


int StreamDemuxer::configure(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	int ret;

	if (mConfigured) {
		ULOGE("StreamDemuxer: demuxer is already configured");
		return -1;
	}
	if (mSession == NULL) {
		ULOGE("StreamDemuxer: invalid session");
		return -1;
	}

	ret = configureRtpAvp(localAddr,
		localStreamPort, localControlPort, remoteAddr,
		remoteStreamPort, remoteControlPort, ifaceAddr);
	if (ret != 0) {
		ULOGE("StreamDemuxer: configureRtpAvp() failed");
		return -1;
	}

	mConfigured = true;
	ULOGI("StreamDemuxer: demuxer is configured");

	return ret;
}


int StreamDemuxer::configureWithSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	int ret;

	if (mConfigured) {
		ULOGE("StreamDemuxer: demuxer is already configured");
		return -1;
	}
	if (mSession == NULL) {
		ULOGE("StreamDemuxer: invalid session");
		return -1;
	}

	ret = configureSdp(sdp, ifaceAddr);
	if (ret != 0) {
		ULOGE("StreamDemuxer: configureSdp() failed");
		return -1;
	}

	mConfigured = true;
	ULOGI("StreamDemuxer: demuxer is configured");

	return ret;
}


int StreamDemuxer::close(
	void)
{
	int ret = 0;

	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	if ((mRtspClient != NULL) && (mRtspRunning)) {
		ret = rtsp_client_teardown(mRtspClient, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_teardown() failed");
			return -1;
		}

		pthread_mutex_lock(&mDemuxerMutex);
		pthread_cond_wait(&mDemuxerCond, &mDemuxerMutex);
		pthread_mutex_unlock(&mDemuxerMutex);
	}

	mRunning = false;

	if (mReceiver != NULL) {
		/* Destroy the receiver */
		vstrm_receiver_destroy(mReceiver);
		mReceiver = NULL;
	}

	return ret;
}


int StreamDemuxer::getElementaryStreamCount(
	void)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	/* TODO: handle multiple streams */
	return 1;
}


enum elementary_stream_type StreamDemuxer::getElementaryStreamType(
	int esIndex)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return (enum elementary_stream_type)-1;
	}
	if ((esIndex < 0) || (esIndex >= 1)) {
		ULOGE("StreamDemuxer: invalid ES index");
		return (enum elementary_stream_type)-1;
	}

	/* TODO: handle multiple streams */
	return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;
}


int StreamDemuxer::getElementaryStreamVideoDimensions(
	int esIndex,
	unsigned int *width,
	unsigned int *height,
	unsigned int *cropLeft,
	unsigned int *cropRight,
	unsigned int *cropTop,
	unsigned int *cropBottom,
	unsigned int *sarWidth,
	unsigned int *sarHeight)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= 1)) {
		ULOGE("StreamDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	if (width)
		*width = mWidth;
	if (height)
		*height = mHeight;
	if (cropLeft)
		*cropLeft = mCropLeft;
	if (cropRight)
		*cropRight = mCropRight;
	if (cropTop)
		*cropTop = mCropTop;
	if (cropBottom)
		*cropBottom = mCropBottom;
	if (sarWidth)
		*sarWidth = mSarWidth;
	if (sarHeight)
		*sarHeight = mSarHeight;

	return 0;
}


int StreamDemuxer::getElementaryStreamVideoFov(
	int esIndex,
	float *hfov,
	float *vfov)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= 1)) {
		ULOGE("StreamDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	if (hfov)
		*hfov = mHfov;
	if (vfov)
		*vfov = mVfov;

	return 0;
}


int StreamDemuxer::setElementaryStreamDecoder(
	int esIndex,
	Decoder *decoder)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}
	if (decoder == NULL) {
		ULOGE("StreamDemuxer: invalid decoder");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= 1)) {
		ULOGE("StreamDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	mDecoder = (AvcDecoder*)decoder;
	uint32_t formatCaps = mDecoder->getInputBitstreamFormatCaps();
	if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) {
		mDecoderBitstreamFormat =
			AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM;
	} else if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_AVCC) {
		mDecoderBitstreamFormat =
			AVCDECODER_BITSTREAM_FORMAT_AVCC;
	} else {
		ULOGE("StreamDemuxer: unsupported decoder "
			"input bitstream format");
		return -1;
	}

	if ((mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264) &&
		(!mDecoder->isConfigured())) {
		int ret = configureDecoder(this);
		if (ret < 0) {
			ULOGW("StreamDemuxer: configureDecoder() "
				"failed (%d)", ret);
		}
	}

	return 0;
}


int StreamDemuxer::play(
	float speed)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	if (speed == 0.) {
		return pause();
	} else {
		mRunning = true;
		mSpeed = speed;

		if (mRtspRunning) {
			float scale = mSpeed;
			struct rtsp_range range;
			memset(&range, 0, sizeof(range));
			range.start.format = RTSP_TIME_FORMAT_NPT;
			range.start.npt.now = 1;
			range.stop.format = RTSP_TIME_FORMAT_NPT;
			range.stop.npt.infinity = 1;
			int ret = rtsp_client_play(mRtspClient, &range, scale,
					NULL);
			if (ret != 0) {
				ULOGE("StreamDemuxer: rtsp_client_play() "
					"failed (%d)", ret);
				return -1;
			}
		}

		return 0;
	}
}


int StreamDemuxer::pause(
	void)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	mRunning = false;

	if (mRtspRunning) {
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		int ret = rtsp_client_pause(mRtspClient, &range, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_pause() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


bool StreamDemuxer::isPaused(
	void)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return false;
	}

	return !mRunning;
}


int StreamDemuxer::previous(
	void)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	if ((!mRunning) && (mRtspRunning)) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.sec = mPausePoint / 1000000;
		range.start.npt.usec = mPausePoint -
			range.start.npt.sec * 1000000;
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
		int ret = rtsp_client_play(mRtspClient, &range, scale, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_play() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


int StreamDemuxer::next(
	void)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	if ((!mRunning) && (mRtspRunning)) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.sec = mPausePoint / 1000000;
		range.start.npt.usec = mPausePoint -
			range.start.npt.sec * 1000000;
		/* TODO: SMPTE timestamps*/
		range.stop = range.start;
		range.stop.npt.usec += 1000;
		if (range.stop.npt.usec >= 1000000) {
			range.stop.npt.sec++;
			range.stop.npt.usec -= 1000000;
		}
		int ret = rtsp_client_play(mRtspClient, &range, scale, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_play() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


int StreamDemuxer::seekTo(
	uint64_t timestamp,
	bool exact)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	mRunning = true;

	if (mRtspRunning) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.sec = timestamp / 1000000;
		range.start.npt.usec = timestamp -
			range.start.npt.sec * 1000000;
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
		int ret = rtsp_client_play(mRtspClient, &range, scale, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_play() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


int StreamDemuxer::seekForward(
	uint64_t delta,
	bool exact)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	int64_t ts = (int64_t)mCurrentTime + (int64_t)delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDuration)
		ts = mDuration;

	mRunning = true;
	if (mRtspRunning) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.sec = ts / 1000000;
		range.start.npt.usec = ts -
			range.start.npt.sec * 1000000;
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
		int ret = rtsp_client_play(mRtspClient, &range, scale, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_play() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


int StreamDemuxer::seekBack(
	uint64_t delta,
	bool exact)
{
	if (!mConfigured) {
		ULOGE("StreamDemuxer: demuxer is not configured");
		return -1;
	}

	int64_t ts = (int64_t)mCurrentTime - (int64_t)delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDuration)
		ts = mDuration;

	mRunning = true;
	if (mRtspRunning) {
		float scale = mSpeed;
		struct rtsp_range range;
		memset(&range, 0, sizeof(range));
		range.start.format = RTSP_TIME_FORMAT_NPT;
		range.start.npt.sec = ts / 1000000;
		range.start.npt.usec = ts -
			range.start.npt.sec * 1000000;
		range.stop.format = RTSP_TIME_FORMAT_NPT;
		range.stop.npt.infinity = 1;
		int ret = rtsp_client_play(mRtspClient, &range, scale, NULL);
		if (ret != 0) {
			ULOGE("StreamDemuxer: rtsp_client_play() "
				"failed (%d)", ret);
			return -1;
		}
	}

	return 0;
}


uint64_t StreamDemuxer::getDuration(
	void)
{
	return (mDuration != 0) ? mDuration : (uint64_t)-1;
}


uint64_t StreamDemuxer::getCurrentTime(
	void)
{
	if (mRtspRunning)
		return mCurrentTime;
	else
		return (mStartTime != 0) ? mCurrentTime - mStartTime : 0;
}


int StreamDemuxer::configureDecoder(
	StreamDemuxer *demuxer)
{
	uint8_t *sps_buf = NULL, *pps_buf = NULL;
	uint32_t start;
	int ret = 0;

	if (demuxer == NULL) {
		ULOGE("StreamDemuxer: invalid demuxer");
		return -1;
	}
	if (demuxer->mDecoder == NULL) {
		ULOGE("StreamDemuxer: invalid decoder");
		return -1;
	}

	struct vstrm_codec_info *info = &demuxer->mCodecInfo;
	if (info->codec != VSTRM_CODEC_VIDEO_H264) {
		ULOGE("StreamDemuxer: invalid codec");
		return -1;
	}

	ret = pdraw_videoDimensionsFromH264Sps(
		info->h264.sps, info->h264.spslen,
		&demuxer->mWidth, &demuxer->mHeight,
		&demuxer->mCropLeft, &demuxer->mCropRight,
		&demuxer->mCropTop, &demuxer->mCropBottom,
		&demuxer->mSarWidth, &demuxer->mSarHeight);
	if (ret != 0) {
		ULOGW("StreamDemuxer: "
			"pdraw_videoDimensionsFromH264Sps() "
			"failed (%d)", ret);
	} else {
		VideoMedia *vm = demuxer->mDecoder->getVideoMedia();
		if (vm)
			vm->setDimensions(demuxer->mWidth,
				demuxer->mHeight, demuxer->mCropLeft,
				demuxer->mCropRight, demuxer->mCropTop,
				demuxer->mCropBottom,
				demuxer->mSarWidth,
				demuxer->mSarHeight);
	}

	sps_buf = (uint8_t *)malloc(info->h264.spslen + 4);
	if (sps_buf == NULL) {
		ULOGE("RecordDemuxer: SPS buffer allocation failed");
		return -ENOMEM;
	}

	start = (demuxer->mDecoderBitstreamFormat ==
		AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
		htonl(0x00000001) : htonl(info->h264.spslen);
	*((uint32_t*)sps_buf) = start;
	memcpy(sps_buf + 4, info->h264.sps, info->h264.spslen);

	pps_buf = (uint8_t *)malloc(info->h264.ppslen + 4);
	if (pps_buf == NULL) {
		ULOGE("RecordDemuxer: PPS buffer allocation failed");
		free(sps_buf);
		return -ENOMEM;
	}

	start = (demuxer->mDecoderBitstreamFormat ==
		AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
		htonl(0x00000001) : htonl(info->h264.ppslen);
	*((uint32_t*)pps_buf) = start;
	memcpy(pps_buf + 4, info->h264.pps, info->h264.ppslen);

	ret = demuxer->mDecoder->configure(
		demuxer->mDecoderBitstreamFormat,
		sps_buf, info->h264.spslen + 4,
		pps_buf, info->h264.ppslen + 4);
	if (ret != 0) {
		ULOGE("StreamDemuxer: decoder configuration failed (%d)", ret);
	}

	free(sps_buf);
	free(pps_buf);

	return ret;
}


void StreamDemuxer::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	int ret = 0;

	if (demuxer == NULL)
		return;
	if ((buf == NULL) || (len == 0))
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	/* ignore "Parrot Streaming" v1 and v2 user data SEI */
	if ((vstrm_h264_sei_streaming_is_v1(sei->uuid)) ||
		(vstrm_h264_sei_streaming_is_v2(sei->uuid)))
		return;

	ret = vbuf_set_userdata_capacity(demuxer->mCurrentBuffer, len);
	if (ret < (signed)len) {
		ULOGE("StreamDemuxer: failed to realloc user data buffer");
		return;
	}

	uint8_t *dstBuf = vbuf_get_userdata_ptr(demuxer->mCurrentBuffer);
	memcpy(dstBuf, buf, len);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, len);
}


void StreamDemuxer::dataCb(
	int fd,
	uint32_t events,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = { 0, 0 };

	if (demuxer == NULL) {
		ULOGE("StreamDemuxer: invalid callback params");
		return;
	}

	do {
		/* Read data */
		readlen = demuxer->mStreamSock->read();

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				demuxer->mStreamSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0)
				PDRAW_LOG_ERRNO("time_get_monotonic", -res);
			res = vstrm_receiver_recv_data(
				demuxer->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0)
				PDRAW_LOG_ERRNO("vstrm_receiver_recv_ctrl",
					-res);
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


void StreamDemuxer::ctrlCb(
	int fd,
	uint32_t events,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = { 0, 0 };

	if (demuxer == NULL) {
		ULOGE("StreamDemuxer: invalid callback params");
		return;
	}

	do {
		/* Read data */
		readlen = demuxer->mControlSock->read();

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				demuxer->mControlSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0)
				PDRAW_LOG_ERRNO("time_get_monotonic", -res);
			res = vstrm_receiver_recv_ctrl(
				demuxer->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0)
				PDRAW_LOG_ERRNO("vstrm_receiver_recv_ctrl",
					-res);
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


int StreamDemuxer::sendCtrlCb(
	struct vstrm_receiver *stream,
	struct pomp_buffer *buf,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	const void *cdata = NULL;
	size_t len = 0;
	ssize_t writelen = 0;

	if ((demuxer == NULL) || (buf == NULL)) {
		ULOGE("StreamDemuxer: invalid callback params");
		return -EINVAL;
	}

	/* Write data */
	pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
	writelen = demuxer->mControlSock->write(cdata, len);
	return (writelen >= 0) ? 0 : (int)writelen;
}


void StreamDemuxer::codecInfoChangedCb(
	struct vstrm_receiver *stream,
	const struct vstrm_codec_info *info,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	int ret;

	if ((demuxer == NULL) || (info == NULL)) {
		ULOGE("StreamDemuxer: invalid callback params");
		return;
	}
	if (info->codec != VSTRM_CODEC_VIDEO_H264) {
		ULOGE("StreamDemuxer: unsupported codec");
		return;
	}

	ULOGI("StreamDemuxer: received SPS/PPS");
	demuxer->mCodecInfo = *info;

	ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0,
		info->h264.sps, info->h264.spslen);
	if (ret < 0) {
		ULOGW("StreamDemuxer: h264_reader_parse_nalu() "
			"failed (%d)", ret);
		return;
	}

	ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0,
		info->h264.pps, info->h264.ppslen);
	if (ret < 0) {
		ULOGW("StreamDemuxer: h264_reader_parse_nalu() "
			"failed (%d)", ret);
		return;
	}

	if ((demuxer->mDecoder != NULL) &&
		(!demuxer->mDecoder->isConfigured())) {
		ret = configureDecoder(demuxer);
		if (ret < 0)
			ULOGW("StreamDemuxer: configureDecoder() "
				"failed (%d)", ret);
	}
}


void StreamDemuxer::recvFrameCb(
	struct vstrm_receiver *stream,
	struct vstrm_frame *frame,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;
	struct vbuf_buffer *buffer = NULL;
	uint32_t flags = 0, start, i;
	size_t frame_size = 0;
	uint8_t *buf;
	ssize_t res;
	size_t buf_size, out_size = 0;
	int ret;

	if ((demuxer == NULL) || (frame == NULL)) {
		ULOGE("StreamDemuxer: invalid callback params");
		return;
	}
	if (demuxer->mDecoder == NULL) {
		ULOGD("StreamDemuxer: no decoder configured");
		return;
	}
	if (!demuxer->mDecoder->isConfigured()) {
		ULOGD("StreamDemuxer: decoder is not configured");
		return;
	}

	/* Get a decoder input buffer */
	if (demuxer->mCurrentBuffer != NULL)
		buffer = demuxer->mCurrentBuffer;
	else if ((ret = demuxer->mDecoder->getInputBuffer(
		&demuxer->mCurrentBuffer, false)) == 0)
		buffer = demuxer->mCurrentBuffer;
	if (buffer == NULL) {
		ULOGW("StreamDemuxer: failed to get an input buffer (%d)", ret);
		return;
	}

	buf = vbuf_get_ptr(buffer);
	res = vbuf_get_capacity(buffer);
	if ((buf == NULL) || (res <= 0)) {
		ULOGE("StreamDemuxer: invalid input buffer");
		return;
	}
	buf_size = res;

	/* Insert the SPS and PPS (only needed for FFmpeg decoder) */
	if ((demuxer->mFirstFrame) &&
		(demuxer->mCodecInfo.codec == VSTRM_CODEC_VIDEO_H264)) {
		if (demuxer->mCodecInfo.h264.spslen + 4 <= buf_size) {
			start = (demuxer->mDecoderBitstreamFormat ==
				AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
				htonl(0x00000001) :
				htonl(demuxer->mCodecInfo.h264.spslen);
			*((uint32_t *)buf) = start;
			memcpy(buf + 4, demuxer->mCodecInfo.h264.sps,
				demuxer->mCodecInfo.h264.spslen);
			buf += (demuxer->mCodecInfo.h264.spslen + 4);
			buf_size -= (demuxer->mCodecInfo.h264.spslen + 4);
			out_size += (demuxer->mCodecInfo.h264.spslen + 4);
		}
		if (demuxer->mCodecInfo.h264.ppslen + 4 <= buf_size) {
			start = (demuxer->mDecoderBitstreamFormat ==
				AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
				htonl(0x00000001) :
				htonl(demuxer->mCodecInfo.h264.ppslen);
			*((uint32_t *)buf) = start;
			memcpy(buf + 4, demuxer->mCodecInfo.h264.pps,
				demuxer->mCodecInfo.h264.ppslen);
			buf += (demuxer->mCodecInfo.h264.ppslen + 4);
			buf_size -= (demuxer->mCodecInfo.h264.ppslen + 4);
			out_size += (demuxer->mCodecInfo.h264.ppslen + 4);
		}
		demuxer->mFirstFrame = false;
	}

	/* Decoder input format */
	switch(demuxer->mDecoderBitstreamFormat) {
	case AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM:
		flags = VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE;
		break;
	case AVCDECODER_BITSTREAM_FORMAT_AVCC:
		flags = VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE;
		break;
	default:
		ULOGE("StreamDemuxer: unsupported decoder "
			"input bitstream format");
		return;
	}

	/* Get the size of the frame */
	ret = vstrm_frame_get_size(frame, &frame_size, flags);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_frame_get_size", -ret);
		return;
	}

	if ((unsigned)buf_size < frame_size) {
		ULOGW("StreamDemuxer: input buffer too small (%zi vs. %zu",
			buf_size, frame_size);
		return;
	}

	/* Copy the frame */
	/* TODO: avoid copy? */
	ret = vstrm_frame_copy(frame, buf, frame_size, flags);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_frame_copy", -ret);
		return;
	}
	out_size += frame_size;

	struct avcdecoder_input_buffer *data =
		(struct avcdecoder_input_buffer*)vbuf_get_metadata_ptr(buffer);
	struct timespec t1;

	vbuf_set_size(buffer, out_size);
	memset(data, 0, sizeof(*data));
	data->isComplete = (frame->info.complete) ? true : false;
	data->hasErrors = (frame->info.error) ? true : false;
	data->isRef = (frame->info.ref) ? true : false;
	data->isSilent = false; /* TODO */
	data->auNtpTimestamp = frame->timestamp; /* TODO */
	data->auNtpTimestampRaw = frame->timestamp; /* TODO */
	data->auNtpTimestampLocal = frame->timestamp; /* TODO */
	/* TODO: auSyncType */

	/* Metadata */
	if (frame->metadata.type == VMETA_FRAME_TYPE_V2) {
		data->hasMetadata = true;
		data->metadata = frame->metadata.v2;
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	uint64_t curTime =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
	data->demuxOutputTimestamp = curTime;

	/* User data */
	vbuf_set_userdata_size(buffer, 0);
	for (i = 0; i < frame->nalu_count; i++) {
		uint8_t nalu_header = *frame->nalus[i].cdata;
		if ((nalu_header & 0x1F) == 0x06) {
			/* SEI NAL unit */
			ret = h264_reader_parse_nalu(demuxer->mH264Reader,
				0, frame->nalus[i].cdata, frame->nalus[i].len);
			if (ret < 0)
				ULOGW("StreamDemuxer: h264_reader_parse_nalu() "
					"failed (%d)", ret);
			break;
		}
	}

	if (demuxer->mRtspRunning) {
		demuxer->mCurrentTime = frame->timestamp * demuxer->mSpeed -
			demuxer->mNtpToNptOffset;
	} else {
		/* TODO: use auNtpTimestamp */
		demuxer->mCurrentTime = frame->timestamp; /* TODO */
		if (demuxer->mStartTime == 0)
			demuxer->mStartTime = frame->timestamp; /* TODO */
	}

	/* release buffer */
	ret = demuxer->mDecoder->queueInputBuffer(buffer);
	if (ret != 0) {
		ULOGW("StreamDemuxer: failed to release the input buffer (%d)",
			ret);
	} else {
		vbuf_unref(&demuxer->mCurrentBuffer);
		demuxer->mCurrentBuffer = NULL;
	}
}


void StreamDemuxer::sessionMetadataPeerChangedCb(
	struct vstrm_receiver *stream,
	const struct vmeta_session *meta,
	void *userdata)
{
	StreamDemuxer *demuxer = (StreamDemuxer*)userdata;

	if ((demuxer == NULL) || (meta == NULL)) {
		ULOGE("StreamDemuxer: invalid callback params");
		return;
	}
	if (demuxer->mSession == NULL) {
		ULOGE("StreamDemuxer: invalid session");
		return;
	}

	SessionPeerMetadata *peerMeta = demuxer->mSession->getPeerMetadata();
	peerMeta->set(meta);
	if (meta->picture_fov.has_horz)
		demuxer->mHfov = meta->picture_fov.horz;
	if (meta->picture_fov.has_vert)
		demuxer->mVfov = meta->picture_fov.vert;
	if (demuxer->mDecoder) {
		VideoMedia *vm = demuxer->mDecoder->getVideoMedia();
		if (vm)
			vm->setFov(demuxer->mHfov, demuxer->mVfov);
	}
}

} /* namespace Pdraw */
