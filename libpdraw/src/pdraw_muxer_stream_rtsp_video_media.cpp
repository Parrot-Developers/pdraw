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

#define ULOG_TAG pdraw_rtspmuxer_media
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <array>
#include <libmp4.h>
#include <media-buffers/mbuf_mem_generic.h>

namespace Pdraw {

constexpr size_t DEFAULT_RX_BUFFER_SIZE = 1500;
constexpr size_t PDRAW_RTP_TXBUF_SIZE = (1 * 1024 * 1024); /* In Bytes */
constexpr size_t PDRAW_RTP_RXBUF_SIZE = (1 * 1024 * 1024); /* In Bytes */
constexpr size_t MUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT = 55004 + 10;
constexpr size_t MUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT = 55005 + 10;
constexpr size_t RTP_CLOCK_RATE = 90000;
constexpr const char *STREAM_TEARDOWN_REASON = "user disconnection";


const struct vstrm_sender_cbs RtspStreamMuxer::VideoMedia::mSenderCbs = {
	.send_data = &RtspStreamMuxer::VideoMedia::sendDataCb,
	.send_ctrl = &RtspStreamMuxer::VideoMedia::sendCtrlCb,
	.monitor_send_data_ready =
		&RtspStreamMuxer::VideoMedia::monitorSendDataReadyCb,
	.session_metadata_peer_changed = nullptr,
	.receiver_report = &RtspStreamMuxer::VideoMedia::receiverReportCb,
	.video_stats = &RtspStreamMuxer::VideoMedia::videoStatsCb,
	.goodbye = &RtspStreamMuxer::VideoMedia::goodbyeCb,
};


RtspStreamMuxer::VideoMedia::VideoMedia(
	RtspStreamMuxer *muxer,
	enum pdraw_muxer_rtsp_transport transport) :
		mMuxer(muxer),
		mLowerTransport(
			RtspStreamMuxer::
				pdrawMuxerRtspTransportToRtspLowerTransport(
					transport))
{
	std::string name = muxer->getName() + "#VideoMedia";
	Loggable::setName(name);
}


RtspStreamMuxer::VideoMedia::~VideoMedia()
{
	stopRtpAvp();
}


int RtspStreamMuxer::VideoMedia::prepareSetup()
{
	int res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}

	return 0;
}


int RtspStreamMuxer::VideoMedia::createSender()
{
	int ret;
	std::unique_ptr<struct vstrm_sender_cfg> cfg;

	try {
		cfg = make_unique<struct vstrm_sender_cfg>();
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("std::make_unique", -ret);
		return ret;
	}

	memset(cfg.get(), 0, sizeof(*cfg.get()));

	/* Create the stream receiver */
	cfg->loop = mMuxer->mSession->getLoop();
	cfg->flags = 0 | VSTRM_SENDER_FLAGS_ENABLE_RTCP;
	cfg->dyn.target_packet_size = 1400;
	cfg->dyn.max_network_latency_ms[0] = 400;
	cfg->dyn.max_network_latency_ms[1] = 300;
	cfg->dyn.max_network_latency_ms[2] = 200;
	cfg->dyn.max_network_latency_ms[3] = 100;
	cfg->dyn.max_network_latency_ms[4] = 2;
	cfg->dyn.max_network_latency_ms[5] = 2;
	cfg->dyn.max_network_latency_ms[6] = 2;
	cfg->dyn.max_network_latency_ms[7] = 2;

	ret = vstrm_sender_new(cfg.get(), &mSenderCbs, this, &mSender);
	if (ret < 0) {
		mSender = nullptr;
		PDRAW_LOG_ERRNO("vstrm_receiver_new", -ret);
		destroySender();
		return ret;
	}

	uint32_t ssrc = 0;

	/* Retrieve the SPS/PPS out of band if available */
	ret = vstrm_sender_get_ssrc_self(mSender, &ssrc);
	if (ret < 0) {
		ULOG_ERRNO("vstrm_sender_get_ssrc_self", -ret);
		destroySender();
		return ret;
	}

	setSsrc(ssrc);

	return 0;
}


int RtspStreamMuxer::VideoMedia::destroySender()
{
	int res;
	if (mSender != nullptr) {
		res = vstrm_sender_send_goodbye(mSender,
						STREAM_TEARDOWN_REASON);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_send_goodbye", -res);
		res = vstrm_sender_destroy(mSender);
		if (res < 0)
			PDRAW_LOG_ERRNO("vstrm_sender_destroy", -res);
		mSender = nullptr;
	}
	return 0;
}


int RtspStreamMuxer::VideoMedia::setup(const std::string &controlUrl,
				       Media *media)
{
	int ret;

	std::string name = mMuxer->getName() +
			   ((media != nullptr) ? "#" + controlUrl : "#NULL");
	Loggable::setName(name);

	mControlUrl = controlUrl;

	/* Only accept coded video media */
	auto *cvm = dynamic_cast<CodedVideoMedia *>(media);
	if (cvm == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -ENOSYS;
	}

	ret = sdp_session_media_add(mMuxer->mSdpSession, &mSdpMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("sdp_session_media_add", -ret);
		return ret;
	}
	mSdpMedia->type = SDP_MEDIA_TYPE_VIDEO;
	mSdpMedia->control_url = strdup(mControlUrl.c_str());
	mSdpMedia->media_title = strdup(getCName()); /* FIXME */
	mSdpMedia->payload_type = 96;
	mSdpMedia->encoding_name = strdup("H264");
	mSdpMedia->clock_rate = RTP_CLOCK_RATE;

	mSdpMedia->h264_fmtp.valid = 1;
	/* TODO */
	mSdpMedia->h264_fmtp.packetization_mode = 1;

	const uint8_t *sps = nullptr;
	const uint8_t *pps = nullptr;
	size_t spsSize = 0;
	size_t ppsSize = 0;

	ret = cvm->getPs(nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("getPs", -ret);
		return ret;
	}

	mSdpMedia->h264_fmtp.sps =
		static_cast<uint8_t *>(calloc(spsSize, sizeof(uint8_t)));
	mSdpMedia->h264_fmtp.sps_size = static_cast<uint32_t>(spsSize);
	memcpy(mSdpMedia->h264_fmtp.sps, sps, spsSize);

	mSdpMedia->h264_fmtp.pps =
		static_cast<uint8_t *>(calloc(ppsSize, sizeof(uint8_t)));
	mSdpMedia->h264_fmtp.pps_size = static_cast<uint32_t>(ppsSize);
	memcpy(mSdpMedia->h264_fmtp.pps, pps, ppsSize);

	if (spsSize >= 4) {
		/* FIXME better way? */
		uint8_t *spsPtr = mSdpMedia->h264_fmtp.sps;
		uint8_t nal_header = spsPtr[0];
		(void)nal_header;
		mSdpMedia->h264_fmtp.profile_idc = spsPtr[1];
		mSdpMedia->h264_fmtp.profile_iop = spsPtr[2];
		mSdpMedia->h264_fmtp.level_idc = spsPtr[3];
	} else {
		ret = -EPROTO;
		PDRAW_LOG_ERRNO("invalid spsSize: %zu", -ret, spsSize);
		return ret;
	}

	mVideoMedia = media;

	mMuxer->mSetupRequestsCount++;
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


int RtspStreamMuxer::VideoMedia::startRtpAvp()
{
	int res;
	const char *label = getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP
				    ? "Channel"
				    : "Port";

	/* Create sockets only if not created during prepareSetup */
	if (mStrm.sock == nullptr && mCtrl.sock == nullptr &&
	    getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP) {
		res = createSockets();
		if (res != 0) {
			PDRAW_LOG_ERRNO("createSockets", -res);
			goto error;
		}
	} else if ((mStrm.sock != nullptr && mCtrl.sock == nullptr) ||
		   (mStrm.sock == nullptr && mCtrl.sock != nullptr)) {
		PDRAW_LOGE("bad state, only one socket created!");
		res = -EPROTO;
		goto error;
	}

	PDRAW_LOGI("startRtpAvp localStream%s=%d localControl%s=%d",
		   label,
		   mStrm.localPort,
		   label,
		   mCtrl.localPort);

	/* Create the stream sender */
	res = createSender();
	if (res < 0) {
		PDRAW_LOG_ERRNO("createSender", -res);
		goto error;
	}

	return 0;

error:
	stopRtpAvp();
	return res;
}


int RtspStreamMuxer::VideoMedia::stopRtpAvp()
{
	int err;
	PDRAW_LOGD("stopRtpAvp");
	destroySender();
	if (mStrm.sock != nullptr) {
		err = tskt_socket_destroy(mStrm.sock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
		mStrm.sock = nullptr;
	}
	if (mCtrl.sock != nullptr) {
		err = tskt_socket_destroy(mCtrl.sock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
		mCtrl.sock = nullptr;
	}
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
	return 0;
}


void RtspStreamMuxer::VideoMedia::finishSetup()
{
	SetupRequest req = {
		.media = this,
		.controlUrl = getCControlUrl(),
		.lowerTransport = getLowerTransport(),
		.localStreamPort = getLocalStreamPort(),
		.localControlPort = getLocalControlPort(),
		.headerExt = getHeaderExt(),
		.headerExtCount = getHeaderExtCount(),
	};
	mMuxer->mSetupRequests.push(req);

	(void)mMuxer->processRtspRequests();
}


int RtspStreamMuxer::VideoMedia::teardown()
{
	if (mTearingDown)
		return -EALREADY;

	if (mPendingTearDown)
		return -EBUSY;

	mPendingTearDown = true;

	finishTeardown();
	return 0;
}


void RtspStreamMuxer::VideoMedia::setTearingDown()
{
	mPendingTearDown = false;
	mTearingDown = true;
	/* Needed to close sockets */
	stopRtpAvp();
	/* TODO: remove TEARDOWN request from the queue? */
}


void RtspStreamMuxer::VideoMedia::setRemoteStreamPort(uint16_t port)
{
	mStrm.remotePort = port;

	if (mStrm.sock != nullptr) {
		int res = tskt_socket_set_remote(
			mStrm.sock,
			mMuxer->mUrl->getResolvedHost().c_str(),
			mStrm.remotePort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(strm)", -res);
	}
}


void RtspStreamMuxer::VideoMedia::setRemoteControlPort(uint16_t port)
{
	mCtrl.remotePort = port;

	if (mCtrl.sock != nullptr) {
		int res = tskt_socket_set_remote(
			mCtrl.sock,
			mMuxer->mUrl->getResolvedHost().c_str(),
			mCtrl.remotePort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(ctrl)", -res);
	}
}


int RtspStreamMuxer::VideoMedia::process()
{
	int res;
	int err;
	struct mbuf_coded_video_frame *frame;

	auto *channel = dynamic_cast<const CodedVideoChannel *>(
		mMuxer->getInputChannel(mVideoMedia));
	if (channel == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}
	mbuf::Queue *queue = channel->getQueue(mMuxer);
	if (queue == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Channel::getQueue", -res);
		return res;
	}

	/* TODO: This loops drops frames if the processFrame function fails.
	 * This is a problem for most coded streams, so the current behavior
	 * will result in a buggy output stream.
	 * We should instead check what the error was, and decide to either:
	 * - Retry later, which can be achieved by using queue_peek() instead of
	 *   queue_pop(), or
	 * - Discard the frame, flush the queue, and ask the upstream elements
	 *   for a complete resync
	 * The second choice is important to have, because if an error persists,
	 * then this whole element will be stuck on the buggy frame. */
	do {
		res = queue->popFrame(&frame);
		if (res < 0) {
			if (res != -EAGAIN)
				PDRAW_LOG_ERRNO("queue::popFrame", -res);
			continue;
		}

		/* Process each buffer */
		if ((mMuxer->mRtspClient != nullptr) &&
		    (mMuxer->mRtspState == RtspState::SETUP_DONE))
			res = processFrame(frame);

		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	} while (res == 0);

	return 0;
}


/* Note: this structure is copied in each tpkt_packet.
 * Thus, it shall be as light as possible. */
struct tpkt_packet_userdata {
	uint64_t capture_timestamp;
};


void RtspStreamMuxer::VideoMedia::rtpFrameDispose(struct vstrm_frame *vframe)
{
	int res;
	struct mbuf_coded_video_frame *frame = nullptr;
	struct tpkt_packet_userdata *aData = nullptr;

	ULOG_ERRNO_RETURN_IF(vframe == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(vframe->userdata == nullptr, EINVAL);

	frame = static_cast<struct mbuf_coded_video_frame *>(vframe->userdata);
	aData = (struct tpkt_packet_userdata *)vframe->ancillary_data.data;

	for (uint32_t i = 0; i < vframe->nalu_count; i++) {
		int start_code_off = (int)(intptr_t)vframe->nalus[i].userdata;
		res = mbuf_coded_video_frame_release_nalu(
			frame, i, vframe->nalus[i].cdata - start_code_off);
		if (res < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_release_nalu", -res);
	}

	struct vdef_coded_frame frameInfo {
	};
	res = mbuf_coded_video_frame_get_frame_info(frame, &frameInfo);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
	}

	res = mbuf_coded_video_frame_unref(frame);
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_unref", -res);

	if (aData != nullptr)
		free(aData);
}


int RtspStreamMuxer::VideoMedia::processFrame(
	struct mbuf_coded_video_frame *frame)
{
	int res;
	int err;
	struct vstrm_frame *vframe = nullptr;
	struct vstrm_frame_ops ops;
	struct tpkt_packet_userdata *tpkt_userdata = nullptr;
	unsigned int naluCount = 0;
	unsigned int sliceCount = 0;
	struct vdef_coded_frame frameInfo;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const CodedVideoMedia::Frame *meta;
	const void *aData;

	mbuf_coded_video_frame_ref(frame);
	res = mbuf_coded_video_frame_get_frame_info(frame, &frameInfo);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto error;
	}

	res = mbuf_coded_video_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = static_cast<const CodedVideoMedia::Frame *>(aData);

	if (!mMuxer->mRecording)
		goto out;

	if ((!mSynchronized) && (!meta->isSync))
		goto out;
	mSynchronized = true;

	const void *data;
	size_t size;
	res = mbuf_coded_video_frame_get_packed_buffer(frame, &data, &size);
	if (res == 0) {
		mbuf_coded_video_frame_release_packed_buffer(frame, data);
	} else if (res < 0 && res != -EPROTO) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -res);
		goto error;
	}

	memset(&ops, 0, sizeof(ops));
	ops.dispose = &rtpFrameDispose;
	res = vstrm_frame_new(&ops, sizeof(frame), &vframe);
	if (res < 0) {
		ULOG_ERRNO("vstrm_frame_new", -res);
		goto error;
	}

	tpkt_userdata = static_cast<struct tpkt_packet_userdata *>(
		calloc(1, sizeof(struct tpkt_packet_userdata)));

	if (tpkt_userdata == nullptr) {
		res = -errno;
		ULOG_ERRNO("calloc", -res);
		goto error;
	}
	tpkt_userdata->capture_timestamp = frameInfo.info.capture_timestamp;

	/* If the frame capture timestamp is zero, in the case of a replay
	 * media, for instance, fall back to the main frame timestamp which
	 * is the media stream time */
	if (tpkt_userdata->capture_timestamp == 0) {
		/* VDEF_ROUND */
		tpkt_userdata->capture_timestamp = frameInfo.info.timestamp *
						   1000000 /
						   frameInfo.info.timescale;
	}

	vframe->userdata = frame;
	vframe->ancillary_data.data = tpkt_userdata;
	vframe->ancillary_data.size = sizeof(*tpkt_userdata);

	/* Metadata */
	{
		struct vmeta_frame *vmeta = nullptr;
		res = mbuf_coded_video_frame_get_metadata(frame, &vmeta);
		if (res == -ENOENT) {
			/* No metadata, not an error case */
			res = 0;
		} else if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_metadata",
					-res);
			goto out;
		} else {
			vframe->metadata = vmeta;
		}
	}

	vframe->timestamps.ntp =
		(frameInfo.info.timestamp * 1000000) / frameInfo.info.timescale;

	res = mbuf_coded_video_frame_get_nalu_count(frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		goto error;
	}
	naluCount = res;

	for (unsigned int i = 0; i < naluCount; i++) {
		const void *_data;
		struct vdef_nalu nalu;
		int start_code_size;
		res = mbuf_coded_video_frame_get_nalu(frame, i, &_data, &nalu);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_get_nalu", -res);
			goto error;
		}

		/* Compute start code size */
		switch (frameInfo.format.data_format) {
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			start_code_size = 0;
			break;
		case VDEF_CODED_DATA_FORMAT_AVCC:
			start_code_size = 4;
			break;
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM: {
			res = h264_get_start_code_length(
				reinterpret_cast<const uint8_t *>(_data),
				nalu.size);
			if (res < 0) {
				ULOG_ERRNO("get_start_code_length", -res);
				goto error;
			}
			start_code_size = res;
			break;
		}
		default:
			res = -ENOSYS;
			ULOG_ERRNO("unsupported data format", -res);
			goto error;
		}

		struct vstrm_frame_nalu nalu_obj {
		};
		nalu_obj.cdata = (uint8_t *)_data + start_code_size;
		nalu_obj.len = nalu.size - start_code_size;
		nalu_obj.importance = nalu.importance;
		nalu_obj.userdata = (void *)(intptr_t)start_code_size;

		res = vstrm_frame_add_nalu(vframe, &nalu_obj);
		if (res < 0) {
			ULOG_ERRNO("vstrm_frame_add_nalu", -res);
			goto error;
		}

		if ((nalu.h264.type == H264_NALU_TYPE_SLICE_IDR) ||
		    (nalu.h264.type == H264_NALU_TYPE_SLICE))
			sliceCount++;
	}

	/* Do not stream 0-slice frames (frames with discarded slices) */
	if (sliceCount == 0)
		goto out;

	res = vstrm_sender_send_frame(mSender, vframe);
	if (res < 0) {
		ULOG_ERRNO("vstrm_sender_send_frame", -res);
		goto error;
	}

	vstrm_frame_unref(vframe);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	return 0;

out:
	res = 0;

error:
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (vframe != nullptr) {
		vstrm_frame_unref(vframe);
	} else {
		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_unref", -err);
		free(tpkt_userdata);
	}

	return res;
}


void RtspStreamMuxer::VideoMedia::flush(bool discard)
{
	/* Unref frame/mem? */
}


void RtspStreamMuxer::VideoMedia::stop()
{
	/* Unref frame/mem? */
	mVideoMedia->setTearingDown();
}


void RtspStreamMuxer::VideoMedia::finishTeardown()
{
	if (getControlUrl().empty())
		return;

	/* Media-level metadata */
	TeardownRequest req = {
		.media = this,
		.controlUrl = getCControlUrl(),
	};
	mMuxer->mTeardownRequests.push(req);
	mMuxer->mTeardownRequestsCount++;

	(void)mMuxer->processRtspRequests();
}


int RtspStreamMuxer::VideoMedia::createSockets()
{
	int res;
	int err;
	int txBufSize;
	int rxBufSize;
	if (getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP) {
		if (mStrm.localPort == 0)
			mStrm.localPort =
				MUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT;
		if (mCtrl.localPort == 0)
			mCtrl.localPort =
				MUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT;
	} else {
		mStrm.localPort = 0;
		mCtrl.localPort = 0;
	}

	/* Create the rx buffer */
	mRxBufLen = DEFAULT_RX_BUFFER_SIZE;
	if (mRxPkt == nullptr) {
		mRxPkt = newRxPkt();
		if (mRxPkt == nullptr) {
			res = -ENOMEM;
			PDRAW_LOG_ERRNO("newRxPkt", -res);
			goto error;
		}
	}

	/* Socket are not needed in TCP mode */
	if (getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP)
		return 0;

	/* Sockets will be created once address resolution is done */
	if (!mMuxer->mUrl->hasResolvedHost())
		return 0;

	/* Create the sockets */
	res = tskt_socket_new(mMuxer->mLocalHost.c_str(),
			      &mStrm.localPort,
			      mMuxer->mUrl->getResolvedHost().c_str(),
			      mStrm.remotePort,
			      nullptr,
			      mMuxer->mSession->getLoop(),
			      dataCb,
			      this,
			      &mStrm.sock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:stream(%s)(%u->%u)",
				-res,
				mMuxer->mUrl->getResolvedHost().c_str(),
				mStrm.localPort,
				mStrm.remotePort);
		goto error;
	}

	PDRAW_LOGI("created data socket on port: local=%d, remote=%d",
		   mStrm.localPort,
		   mStrm.remotePort);

	txBufSize = PDRAW_RTP_TXBUF_SIZE;
	res = tskt_socket_set_txbuf_size(mStrm.sock, txBufSize);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_set_txbuf_size");
	rxBufSize = PDRAW_RTP_RXBUF_SIZE;
	res = tskt_socket_set_rxbuf_size(mStrm.sock, rxBufSize);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_set_rxbuf_size");

	res = tskt_socket_get_rxbuf_size(mStrm.sock);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_get_rxbuf_size");
	else if (res != 2 * rxBufSize)
		PDRAW_LOGW("failed to set rx buffer size: got %d, expecting %d",
			   res / 2,
			   rxBufSize);

	res = tskt_socket_set_class_selector(mStrm.sock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW("failed to set class selector for stream socket");

	res = tskt_socket_new(mMuxer->mLocalHost.c_str(),
			      &mCtrl.localPort,
			      mMuxer->mUrl->getResolvedHost().c_str(),
			      mCtrl.remotePort,
			      nullptr,
			      mMuxer->mSession->getLoop(),
			      ctrlCb,
			      this,
			      &mCtrl.sock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:control", -res);
		goto error;
	}

	PDRAW_LOGI("created ctrl socket on port: local=%d, remote=%d",
		   mCtrl.localPort,
		   mCtrl.remotePort);

	res = tskt_socket_set_class_selector(mCtrl.sock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW("failed to set class selector for control socket");

	return 0;

error:
	err = tskt_socket_destroy(mStrm.sock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mStrm.sock = nullptr;
	err = tskt_socket_destroy(mCtrl.sock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mCtrl.sock = nullptr;
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
	return res;
}


struct tpkt_packet *RtspStreamMuxer::VideoMedia::newRxPkt()
{
	struct pomp_buffer *buf = pomp_buffer_new(mRxBufLen);
	if (!buf)
		return nullptr;

	struct tpkt_packet *pkt;
	int res = tpkt_new_from_buffer(buf, &pkt);
	pomp_buffer_unref(buf);
	if (res < 0)
		return nullptr;

	return pkt;
}


void RtspStreamMuxer::VideoMedia::setRxPkt(struct tpkt_packet *newPkt)
{
	if (mRxPkt != nullptr)
		tpkt_unref(mRxPkt);
	mRxPkt = newPkt;
}


int RtspStreamMuxer::VideoMedia::notifyReadyToSend()
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mSender == nullptr, EINVAL);

	int res = vstrm_sender_notify_send_data_ready(mSender);
	if (res < 0)
		ULOG_ERRNO("vstrm_sender_notify_send_data_ready", -res);

	return res;
}


int RtspStreamMuxer::VideoMedia::processDataPkt(struct tpkt_packet *pkt)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	/* Nothing to read here */

	return 0;
}


int RtspStreamMuxer::VideoMedia::processCtrlPkt(struct tpkt_packet *pkt)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	int res = vstrm_sender_recv_ctrl(mSender, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("vstrm_sender_recv_ctrl", -res);

	return res;
}


void RtspStreamMuxer::VideoMedia::dataCb(int fd,
					 uint32_t events,
					 void *userdata)
{
	PDRAW_UNUSED(fd);
	PDRAW_UNUSED(events);

	auto *self = static_cast<VideoMedia *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_OUT) != 0) {
		/* Notify sender */
		res = vstrm_sender_notify_send_data_ready(self->mSender);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_notify_send_data_ready", -res);
	}

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mStrm.sock, self->mRxPkt);
		if (res < 0)
			return;

		/* Discard any data received before starting a vstrm_sender */
		if (!self->mSender)
			continue;

		/* Something read? */
		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			return;

		if (readlen == 0) {
			/* TODO: EOF */
			return;
		}

		res = self->processDataPkt(self->mRxPkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("processCtrlPkt", -res);

		/* Allocate new packet for replacement */
		struct tpkt_packet *newPkt = self->newRxPkt();
		if (!newPkt) {
			PDRAW_LOG_ERRNO("newTxPkt", ENOMEM);
			return;
		}
		/* Replace processed packet with new one */
		self->setRxPkt(newPkt);
	}
}


void RtspStreamMuxer::VideoMedia::ctrlCb(int fd,
					 uint32_t events,
					 void *userdata)
{
	PDRAW_UNUSED(fd);
	PDRAW_UNUSED(events);

	auto *self = static_cast<VideoMedia *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mCtrl.sock, self->mRxPkt);
		if (res < 0)
			return;

		/* Discard any data received before starting a vstrm_sender */
		if (!self->mSender)
			continue;

		/* Something read? */
		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			return;

		if (readlen == 0) {
			/* TODO: EOF */
			return;
		}

		res = self->processCtrlPkt(self->mRxPkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("processCtrlPkt", -res);

		/* Allocate new packet for replacement */
		struct tpkt_packet *newPkt = self->newRxPkt();
		if (!newPkt) {
			PDRAW_LOG_ERRNO("newTxPkt", ENOMEM);
			return;
		}
		/* Replace processed packet with new one */
		self->setRxPkt(newPkt);
	}
}


int RtspStreamMuxer::VideoMedia::sendPkt(struct tpkt_packet *pkt,
					 uint16_t channel,
					 struct tskt_socket *sock,
					 const char *logTag)
{
	int err;

	ULOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	if (mMuxer->mRtspConnectionState != RTSP_CLIENT_CONN_STATE_CONNECTED)
		return 0;

	err = updateStats();
	if (err < 0)
		PDRAW_LOG_ERRNO("updateStats", -err);

	if (getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP) {
		const void *data = nullptr;
		size_t len = 0;
		tpkt_get_cdata(pkt, &data, &len, nullptr);

		int ret = rtsp_client_send_interleaved(mMuxer->mRtspClient,
						       (uint8_t)channel,
						       (uint8_t *)data,
						       len);
		if (ret < 0 && ret != -EAGAIN)
			PDRAW_LOG_ERRNO("rtsp_client_send_interleaved(%s)",
					-ret,
					logTag);

		return ret;
	}

	ULOG_ERRNO_RETURN_ERR_IF(sock == nullptr, EINVAL);
	return tskt_socket_write_pkt(sock, pkt);
}


int RtspStreamMuxer::VideoMedia::sendDataCb(struct vstrm_sender *stream,
					    struct tpkt_packet *pkt,
					    bool marker,
					    void *userdata)
{
	const auto self = static_cast<VideoMedia *>(userdata);
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	return self->sendPkt(
		pkt, self->getRemoteStreamPort(), self->mStrm.sock, "strm");
}


int RtspStreamMuxer::VideoMedia::sendCtrlCb(struct vstrm_sender *stream,
					    struct tpkt_packet *pkt,
					    void *userdata)
{
	const auto self = static_cast<VideoMedia *>(userdata);
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	return self->sendPkt(
		pkt, self->getRemoteControlPort(), self->mCtrl.sock, "ctrl");
}


int RtspStreamMuxer::VideoMedia::monitorSendDataReadyCb(
	struct vstrm_sender *stream,
	int enable,
	void *userdata)
{
	const auto self = static_cast<VideoMedia *>(userdata);
	uint32_t events = POMP_FD_EVENT_IN | (enable ? POMP_FD_EVENT_OUT : 0);

	if (self->getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP)
		return 0;

	ULOG_ERRNO_RETURN_ERR_IF(self->mSender == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->mStrm.sock == nullptr, EINVAL);

	int fd = tskt_socket_get_fd(self->mStrm.sock);
	if (fd < 0) {
		ULOG_ERRNO("tskt_socket_get_fd", -fd);
		return fd;
	}

	return pomp_loop_update(self->mMuxer->mSession->getLoop(), fd, events);
}


void RtspStreamMuxer::VideoMedia::videoStatsCb(
	struct vstrm_sender *stream,
	const struct vstrm_video_stats *video_stats,
	const struct vstrm_video_stats_dyn *video_stats_dyn,
	void *userdata)
{
	const auto self = static_cast<VideoMedia *>(userdata);
	PDRAW_LOGI("%s", __func__);
}


int RtspStreamMuxer::VideoMedia::updateStats(
	const struct rtcp_pkt_receiver_report *rr,
	uint32_t rtd)
{
	int ret;
	struct vstrm_sender_stats senderStats = {};

	ULOG_ERRNO_RETURN_ERR_IF(mSender == nullptr, EINVAL);

	ret = vstrm_sender_get_stats(mSender, &senderStats);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_sender_get_stats", -ret);
		return ret;
	}

	mStats.pktCountTotal = senderStats.total_packet_count;
	mStats.pktCountDropped = senderStats.dropped_packet_count;

	if (rr != nullptr) {
		for (size_t i = 0; i < rr->report_count; i++) {
			if (rr->reports[i].ssrc != mSsrc)
				continue;

			/* For RTSP over TCP interleaved, the RTCP packet-loss
			 * and RTT values are not reliable and do not represent
			 * real network conditions. This seems to be a
			 * limitation of using RTCP feedback in TCP-interleaved
			 * workflows. Because TCP already handles congestion and
			 * retransmissions, RTCP feedback should not be used for
			 * bitrate or congestion control in this mode. Wowza
			 * recommends ignoring or clamping RTCP loss value
			 * on the sender side when TCP interleaving is used. */
			if (getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP) {
				mStats.pktCountLost = rr->reports[i].lost;
				mStats.lostFraction =
					static_cast<float>(
						rr->reports[i].fraction) /
					256.0f;
			} else {
				mStats.pktCountLost = 0;
				mStats.lostFraction = 0.f;
			}

			/* Convert jitter from RTP clock units to us. */
			mStats.jitter = static_cast<uint32_t>(
				(static_cast<uint64_t>(rr->reports[i].jitter) *
				 1000000) /
				RTP_CLOCK_RATE);
			mStats.rtd = rtd;
			mStats.receiverReportCount++;
			break;
		}
	}

	/* Notify Muxer */
	mMuxer->notifyVideoMediaStatsUpdate(this);

	return 0;
}


void RtspStreamMuxer::VideoMedia::receiverReportCb(
	struct vstrm_sender *stream,
	const struct rtcp_pkt_receiver_report *rr,
	uint32_t rtd,
	void *userdata)
{
	int err;
	const auto self = static_cast<VideoMedia *>(userdata);

	err = self->updateStats(rr, rtd);
	if (err < 0) {
		PDRAW_LOG_ERRNO("updateStats", -err);
		return;
	}
}


void RtspStreamMuxer::VideoMedia::goodbyeCb(struct vstrm_sender *stream,
					    const char *reason,
					    void *userdata)
{
	auto *self = static_cast<VideoMedia *>(userdata);

	if (self == nullptr)
		return;

	PDRAW_LOGI("received RTCP goodbye%s%s",
		   reason ? ", reason: " : "",
		   reason ? reason : "");

	self->mMuxer->onUnrecoverableError(-ENETDOWN);
}

} /* namespace Pdraw */
