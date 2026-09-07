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

#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <array>
#include <libmp4.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <memory>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {

constexpr size_t RTP_CLOCK_RATE = 90000;
constexpr const char *STREAM_TEARDOWN_REASON = "user disconnection";


const struct vstrm_sender_cbs RtspStreamMuxer::VideoMedia::mSenderCbs = {
	.send_ctrl = &RtspStreamMuxer::VideoMedia::sendCtrlCb,
	.session_metadata_peer_changed = nullptr,
	.receiver_report = &RtspStreamMuxer::VideoMedia::receiverReportCb,
	.video_stats = &RtspStreamMuxer::VideoMedia::videoStatsCb,
	.goodbye = &RtspStreamMuxer::VideoMedia::goodbyeCb,
};


RtspStreamMuxer::VideoMedia::VideoMedia(RtspStreamMuxer *muxer) : mMuxer(muxer)
{
	std::string name = muxer->getName() + "#VideoMedia";
	Loggable::setName(name);
}


RtspStreamMuxer::VideoMedia::~VideoMedia()
{
	/* Safety net: in the normal teardown path stopRtpAvp() has already been
	 * called explicitly (via setTearingDown). Here we only need to destroy
	 * the sender; subclass destructors have already cleaned up their
	 * transport-specific resources (sockets, proxies, etc.). */
	destroySender();

	if (mList != nullptr) {
		int err = tpkt_list_destroy(mList);
		if (err < 0)
			PDRAW_LOG_ERRNO("tpkt_list_destroy", -err);
	}
}


int RtspStreamMuxer::VideoMedia::stopRtpAvp()
{
	PDRAW_LOGD("stopRtpAvp (base: destroying sender only)");
	destroySender();
	return 0;
}


int RtspStreamMuxer::VideoMedia::createSender()
{
	int ret;
	std::unique_ptr<struct vstrm_sender_cfg> cfg;

	try {
		cfg = std::make_unique<struct vstrm_sender_cfg>();
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("std::make_unique", -ret);
		return ret;
	}

	memset(cfg.get(), 0, sizeof(*cfg.get()));

	/* Create the stream sender */
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

	ret = vstrm_sender_get_ssrc_self(mSender, &ssrc);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vstrm_sender_get_ssrc_self", -ret);
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
			PDRAW_LOG_ERRNO("vstrm_sender_send_goodbye", -res);
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
	const auto *cvm = dynamic_cast<CodedVideoMedia *>(media);
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
	if (mSdpMedia->h264_fmtp.sps == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}
	mSdpMedia->h264_fmtp.sps_size = static_cast<uint32_t>(spsSize);
	memcpy(mSdpMedia->h264_fmtp.sps, sps, spsSize);

	mSdpMedia->h264_fmtp.pps =
		static_cast<uint8_t *>(calloc(ppsSize, sizeof(uint8_t)));
	if (mSdpMedia->h264_fmtp.pps == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}
	mSdpMedia->h264_fmtp.pps_size = static_cast<uint32_t>(ppsSize);
	memcpy(mSdpMedia->h264_fmtp.pps, pps, ppsSize);

	if (spsSize >= 4) {
		/* FIXME better way? */
		const uint8_t *spsPtr = mSdpMedia->h264_fmtp.sps;
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

	ret = tpkt_list_new(&mList);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("tpkt_list_new", -ret);
		return ret;
	}

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
	/* Needed to close transport */
	stopRtpAvp();
	/* TODO: remove TEARDOWN request from the queue? */
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


int RtspStreamMuxer::VideoMedia::processFrame(
	struct mbuf_coded_video_frame *frame)
{
	int res;
	int err;
	unique_c_ptr<tpkt_packet_userdata> tpkt_userdata;
	struct tpkt_list *list = nullptr;
	unsigned int naluCount = 0;
	unsigned int sliceCount = 0;
	struct vdef_coded_frame frameInfo;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const CodedVideoMedia::Frame *meta;
	const void *aData;
	struct vmeta_frame *metadata = nullptr;
	bool metadataOwned = false;

	mbuf_coded_video_frame_ref(frame);
	res = mbuf_coded_video_frame_get_frame_info(frame, &frameInfo);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
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
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
				-res);
		goto error;
	}

	tpkt_userdata = make_c_struct<unique_c_ptr<tpkt_packet_userdata>>();
	if (!tpkt_userdata) {
		res = -errno;
		PDRAW_LOG_ERRNO("calloc", -res);
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

	/* Metadata: vstrm_sender_send_frame() below only borrows it (it does
	 * not take ownership), so this reference (if any) must still be
	 * dropped by us once we are done with it */
	res = mbuf_coded_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, not an error case */
		metadata = nullptr;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);
		goto out;
	} else {
		metadataOwned = true;
	}

	res = mbuf_coded_video_frame_get_nalu_count(frame);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		goto error;
	}
	naluCount = res;

	for (unsigned int i = 0; i < naluCount; i++) {
		const void *_data;
		struct vdef_nalu nalu;
		res = mbuf_coded_video_frame_get_nalu(frame, i, &_data, &nalu);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_nalu",
					-res);
			goto error;
		}

		if ((nalu.h264.type == H264_NALU_TYPE_SLICE_IDR) ||
		    (nalu.h264.type == H264_NALU_TYPE_SLICE))
			sliceCount++;

		res = mbuf_coded_video_frame_release_nalu(frame, i, _data);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_release_nalu",
					-res);
			goto error;
		}
	}

	/* Do not stream 0-slice frames (frames with discarded slices) */
	if (sliceCount == 0)
		goto out;

	res = vstrm_sender_send_frame(mSender,
				      frame,
				      metadata,
				      tpkt_userdata.get(),
				      sizeof(*tpkt_userdata),
				      &list);
	if (res < 0) {
		PDRAW_LOG_ERRNO("vstrm_sender_send_frame", -res);
		goto error;
	}

	if (metadataOwned)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	err = mbuf_coded_video_frame_unref(frame);
	if (err < 0)
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);

	res = processList(list);
	if (res < 0 && res != -EAGAIN)
		PDRAW_LOG_ERRNO("processList", -res);
	res = tpkt_list_destroy(list);
	if (res < 0)
		PDRAW_LOG_ERRNO("tpkt_list_destroy", -res);

	return 0;

out:
	res = 0;

error:
	if (metadataOwned)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	err = mbuf_coded_video_frame_unref(frame);
	if (err < 0)
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);

	return res;
}


int RtspStreamMuxer::VideoMedia::processList(struct tpkt_list *newList)
{
	int res = 0;
	struct tpkt_packet *pkt;
	struct timespec ts = {0, 0};
	uint64_t curTimestamp = 0;
	uint64_t expirationTimestamp;

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTimestamp);

	if (newList != nullptr) {
		/* Transfer packets to our own pending packets list */
		pkt = tpkt_list_first(newList);
		while (pkt != nullptr) {
			int err = tpkt_list_remove(newList, pkt);
			if (err < 0) {
				PDRAW_LOG_ERRNO("tpkt_list_remove", -err);
				pkt = tpkt_list_first(newList);
				continue;
			}

			err = tpkt_list_add_last(mList, pkt);
			if (err < 0) {
				PDRAW_LOG_ERRNO("tpkt_list_add_last", -err);
				tpkt_unref(pkt);
				pkt = tpkt_list_first(newList);
				continue;
			}

			err = tpkt_unref(pkt);
			if (err < 0)
				PDRAW_LOG_ERRNO("tpkt_unref", -err);
			pkt = tpkt_list_first(newList);
		}
	}

	/* Send pending packets in the list */
	pkt = tpkt_list_first(mList);
	while (pkt != nullptr) {
		res = tpkt_list_remove(mList, pkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("tpkt_list_remove", -res);

		/* Remove packets on timeout (selective drop) */
		expirationTimestamp = tpkt_get_expiration_timestamp(pkt);
		if ((expirationTimestamp != 0) &&
		    (curTimestamp > expirationTimestamp)) {
			uint64_t delta = curTimestamp - expirationTimestamp;
			uint32_t importance = 0;
			tpkt_get_importance(pkt, &importance);
			PDRAW_LOGD("drop packet: importance=%" PRIu32
				   " (%ums late)",
				   importance,
				   (unsigned int)(delta / 1000));
			goto next;
		}

		res = sendPkt(
			pkt, getRemoteStreamPort(), getStreamSocket(), "strm");
		if (res == -EAGAIN) {
			/* Send socket buffer full, stop processing packets
			 * and retry later */
			int err = tpkt_list_add_first(mList, pkt);
			if (err < 0)
				PDRAW_LOG_ERRNO("tpkt_list_add_first", -err);
			mNetdownLogged = false;
			err = tpkt_unref(pkt);
			if (err < 0)
				ULOG_ERRNO("tpkt_unref", -err);
			break;
		} else if (res < 0) {
			if (res == -ENETUNREACH || res == -ENETDOWN) {
				if (!mNetdownLogged) {
					PDRAW_LOG_ERRNO(
						"tskt_socket_write_pkt "
						"(logged only once)",
						-res);
					mNetdownLogged = true;
				}
			} else {
				PDRAW_LOG_ERRNO("tskt_socket_write_pkt", -res);
				mNetdownLogged = false;
			}
			goto next;
		}
		mNetdownLogged = false;

		/* clang-format off */
next:
		/* clang-format on */
		res = tpkt_unref(pkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("tpkt_unref", -res);

		pkt = tpkt_list_first(mList);
	}

	if (getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP) {
		bool monitorOut = tpkt_list_get_count(mList) != 0;
		uint32_t events =
			POMP_FD_EVENT_IN | (monitorOut ? POMP_FD_EVENT_OUT : 0);
		int fd = tskt_socket_get_fd(getStreamSocket());
		if (fd < 0)
			ULOG_ERRNO("tskt_socket_get_fd", -fd);
		else
			mMuxer->mSession->getPompLoop()->update(fd, events);
	}

	return res;
}


void RtspStreamMuxer::VideoMedia::flush([[maybe_unused]] bool discard) const
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


int RtspStreamMuxer::VideoMedia::notifyReadyToSend()
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mSender == nullptr, EINVAL);

	int res = processList(nullptr);
	if (res < 0 && res != -EAGAIN)
		PDRAW_LOG_ERRNO("processList", -res);

	return res;
}


int RtspStreamMuxer::VideoMedia::processDataPkt(struct tpkt_packet *pkt) const
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	/* Nothing to read here */

	return 0;
}


int RtspStreamMuxer::VideoMedia::processCtrlPkt(struct tpkt_packet *pkt)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	if (mSender == nullptr)
		return 0;

	int res = vstrm_sender_recv_ctrl(mSender, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("vstrm_sender_recv_ctrl", -res);

	return res;
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


int RtspStreamMuxer::VideoMedia::sendCtrlCb(
	[[maybe_unused]] struct vstrm_sender *stream,
	struct tpkt_packet *pkt,
	void *userdata)
{
	const auto self = static_cast<VideoMedia *>(userdata);
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	return self->sendPkt(pkt,
			     self->getRemoteControlPort(),
			     self->getControlSocket(),
			     "ctrl");
}


void RtspStreamMuxer::VideoMedia::videoStatsCb(
	[[maybe_unused]] struct vstrm_sender *stream,
	[[maybe_unused]] const struct vstrm_video_stats *video_stats,
	[[maybe_unused]] const struct vstrm_video_stats_dyn *video_stats_dyn,
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
	[[maybe_unused]] struct vstrm_sender *stream,
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


void RtspStreamMuxer::VideoMedia::goodbyeCb(
	[[maybe_unused]] struct vstrm_sender *stream,
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

void RtspStreamMuxer::VideoMedia::requestResync()
{
	auto *channel = dynamic_cast<CodedVideoChannel *>(
		mMuxer->getInputChannel(mVideoMedia));
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("getInputChannel", ENODEV);
		return;
	}
	int res = channel->resync();
	if (res < 0)
		PDRAW_LOG_ERRNO("channel::resync", -res);
}

} /* namespace Pdraw */
