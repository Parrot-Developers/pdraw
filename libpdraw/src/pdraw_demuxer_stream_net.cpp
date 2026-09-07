/**
 * Parrot Drones Audio and Video Vector library
 * Streaming demuxer - net implementation
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

#define ULOG_TAG pdraw_dmxstrmnet
#include <ulog.h>

#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_session.hpp"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <futils/futils.h>

ULOG_DECLARE_TAG(ULOG_TAG);

constexpr size_t DEFAULT_RX_BUFFER_SIZE = 1500;


namespace Pdraw {


StreamDemuxerNet::StreamDemuxerNet(Session *session,
				   Element::Listener *elementListener,
				   Source::Listener *sourceListener,
				   DemuxerWrapper *wrapper,
				   IPdraw::IDemuxer::Listener *demuxerListener,
				   const std::string &url,
				   const struct pdraw_demuxer_params *params) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      wrapper,
			      demuxerListener,
			      params)
{
	Element::setClassName(__func__);

	mUrl = RtspUrl::create(url);

	setState(State::CREATED);
}


StreamDemuxerNet::StreamDemuxerNet(Session *session,
				   Element::Listener *elementListener,
				   Source::Listener *sourceListener,
				   DemuxerWrapper *wrapper,
				   IPdraw::IDemuxer::Listener *demuxerListener,
				   const std::string &localAddr,
				   uint16_t localStreamPort,
				   uint16_t localControlPort,
				   const std::string &remoteAddr,
				   uint16_t remoteStreamPort,
				   uint16_t remoteControlPort,
				   const struct pdraw_demuxer_params *params) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      wrapper,
			      demuxerListener,
			      params),
		mSingleLocalStreamPort(localStreamPort),
		mSingleLocalControlPort(localControlPort),
		mSingleRemoteStreamPort(remoteStreamPort),
		mSingleRemoteControlPort(remoteControlPort)
{
	Element::setClassName(__func__);

	mLocalAddr = (localAddr.length() > 0) ? localAddr : "0.0.0.0";
	mRemoteAddr = (remoteAddr.length() > 0) ? remoteAddr : "0.0.0.0";

	setState(State::CREATED);
}


StreamDemuxerNet::~StreamDemuxerNet()
{
	return;
}


uint16_t StreamDemuxerNet::getSingleStreamLocalStreamPort()
{
	if (mState != State::STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mVideoMedias.size() != 1) {
		PDRAW_LOG_ERRNO("invalid media count", EPROTO);
		return 0;
	}

	auto *media =
		dynamic_cast<const VideoMediaNet *>(mVideoMedias.front().get());
	if (media == nullptr) {
		PDRAW_LOG_ERRNO("invalid media", EPROTO);
		return 0;
	}

	return media->getLocalStreamPort();
}


uint16_t StreamDemuxerNet::getSingleStreamLocalControlPort()
{
	if (mState != State::STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mVideoMedias.size() != 1) {
		PDRAW_LOG_ERRNO("invalid media count", EPROTO);
		return 0;
	}

	auto *media =
		dynamic_cast<const VideoMediaNet *>(mVideoMedias.front().get());
	if (media == nullptr) {
		PDRAW_LOG_ERRNO("invalid media", EPROTO);
		return 0;
	}

	return media->getLocalControlPort();
}


std::unique_ptr<StreamDemuxer::VideoMedia>
StreamDemuxerNet::createVideoMedia(enum rtsp_lower_transport transport)
{
	return std::make_unique<VideoMediaNet>(this, transport);
}


StreamDemuxerNet::VideoMediaNet::VideoMediaNet(
	StreamDemuxerNet *demuxer,
	enum rtsp_lower_transport transport) :
		VideoMedia(demuxer),
		mDemuxerNet(demuxer), mLowerTransport(transport)
{
}


StreamDemuxerNet::VideoMediaNet::~VideoMediaNet()
{
	stopRtpAvp();
	tpkt_unref(mRxPkt);
}


int StreamDemuxerNet::VideoMediaNet::startRtpAvp()
{
	int res;
	const char *label = getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP
				    ? "Channel"
				    : "Port";

	/* Create sockets only if not created during prepareSetup */
	if (mStreamSock == nullptr && mControlSock == nullptr &&
	    getLowerTransport() != RTSP_LOWER_TRANSPORT_TCP) {
		res = createSockets();
		if (res != 0) {
			PDRAW_LOG_ERRNO("createSockets", -res);
			goto error;
		}
	} else if ((mStreamSock != nullptr && mControlSock == nullptr) ||
		   (mStreamSock == nullptr && mControlSock != nullptr)) {
		PDRAW_LOGE("bad state, only one socket created !");
		res = -EPROTO;
		goto error;
	}

	PDRAW_LOGI("startRtpAvp localStream%s=%d localControl%s=%d",
		   label,
		   mLocalStreamPort,
		   label,
		   mLocalControlPort);

	/* Create the stream receiver */
	res = createReceiver();
	if (res < 0) {
		PDRAW_LOG_ERRNO("createReceiver", -res);
		goto error;
	}

	return 0;

error:
	stopRtpAvp();
	return res;
}


int StreamDemuxerNet::VideoMediaNet::stopRtpAvp()
{
	int err;
	PDRAW_LOGD("stopRtpAvp");
	destroyReceiver();
	err = tskt_socket_destroy(mStreamSock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mStreamSock = nullptr;
	err = tskt_socket_destroy(mControlSock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mControlSock = nullptr;
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
	return 0;
}


int StreamDemuxerNet::VideoMediaNet::sendCtrl(struct vstrm_receiver *stream,
					      struct tpkt_packet *pkt)
{
	int res;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	if (getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP) {
		const void *data = nullptr;
		size_t len = 0;
		tpkt_get_cdata(pkt, &data, &len, nullptr);
		int ret = rtsp_client_send_interleaved(
			mDemuxerNet->mRtspClient,
			(uint8_t)getRemoteStreamPort(),
			(uint8_t *)data,
			len);
		if (ret < 0)
			PDRAW_LOG_ERRNO("rtsp_client_send_interleaved", -ret);
		return 0;
	}

	ULOG_ERRNO_RETURN_ERR_IF(mControlSock == nullptr, EINVAL);

	/* Skip RTCP if no remote control port is configured */
	if (tskt_socket_get_remote_port(mControlSock) == 0)
		return 0;

	/* Write data */
	res = tskt_socket_write_pkt(mControlSock, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


int StreamDemuxerNet::VideoMediaNet::prepareSetup()
{
	int res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}

	return 0;
}


enum rtsp_lower_transport
StreamDemuxerNet::VideoMediaNet::getLowerTransport() const
{
	return mLowerTransport;
}


uint16_t StreamDemuxerNet::VideoMediaNet::getLocalStreamPort() const
{
	if (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
		return mLocalStreamPort;

	if (mStreamSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid stream socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_local_port(mStreamSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getLocalControlPort() const
{
	if (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
		return mLocalControlPort;

	if (mControlSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid control socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_local_port(mControlSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getRemoteStreamPort() const
{
	if (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
		return mRemoteStreamPort;

	if (mStreamSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid stream socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_remote_port(mStreamSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getRemoteControlPort() const
{
	if (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
		return mRemoteControlPort;

	if (mControlSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid control socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_remote_port(mControlSock);
}


void StreamDemuxerNet::VideoMediaNet::setLocalStreamPort(uint16_t port)
{
	mLocalStreamPort = port;
}


void StreamDemuxerNet::VideoMediaNet::setLocalControlPort(uint16_t port)
{
	mLocalControlPort = port;
}


void StreamDemuxerNet::VideoMediaNet::setRemoteStreamPort(uint16_t port)
{
	mRemoteStreamPort = port;
	if (mStreamSock != nullptr) {
		int res =
			tskt_socket_set_remote(mStreamSock,
					       mDemuxerNet->mRemoteAddr.c_str(),
					       mRemoteStreamPort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote", -res);
	}
}


void StreamDemuxerNet::VideoMediaNet::setRemoteControlPort(uint16_t port)
{
	mRemoteControlPort = port;
	if (mControlSock != nullptr) {
		int res =
			tskt_socket_set_remote(mControlSock,
					       mDemuxerNet->mRemoteAddr.c_str(),
					       mRemoteControlPort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote", -res);
	}
}


void StreamDemuxerNet::VideoMediaNet::initStreamPorts()
{
	if (getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP) {
		mLocalStreamPort = 0;
		mLocalControlPort = 0;
		return;
	}

	/* No-URL mode: apply pre-configured ports from demuxer
	 * constructor, falling back to defaults if 0 */
	if (mDemuxerNet->mUrl != nullptr) {
		if (mLocalStreamPort == 0)
			mLocalStreamPort =
				DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT;
		if (mLocalControlPort == 0)
			mLocalControlPort =
				DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT;
		return;
	}

	if (mLocalStreamPort == 0) {
		mLocalStreamPort =
			mDemuxerNet->mSingleLocalStreamPort
				? mDemuxerNet->mSingleLocalStreamPort
				: DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT;
	}

	if (mLocalControlPort == 0) {
		mLocalControlPort =
			mDemuxerNet->mSingleLocalControlPort
				? mDemuxerNet->mSingleLocalControlPort
				: DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT;
	}

	mRemoteStreamPort = mDemuxerNet->mSingleRemoteStreamPort;
	mRemoteControlPort = mDemuxerNet->mSingleRemoteControlPort;
}


int StreamDemuxerNet::VideoMediaNet::createSockets()
{
	int res;
	int err;
	int rxBufSize;

	initStreamPorts();

	/* Create the rx buffer */
	mRxBufLen = DEFAULT_RX_BUFFER_SIZE;
	mRxPkt = newRxPkt();
	if (mRxPkt == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("newRxPkt", -res);
		goto error;
	}

	/* Socket are not needed in TCP mode */
	if (getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP)
		return 0;

	/* In URL mode, sockets will be created once addr resolution is done */
	if (mDemuxerNet->mUrl != nullptr &&
	    !mDemuxerNet->mUrl->hasResolvedHost())
		return 0;

	/* Create the sockets */
	res = tskt_socket_new(mDemuxerNet->mLocalAddr.c_str(),
			      &mLocalStreamPort,
			      mDemuxerNet->mRemoteAddr.c_str(),
			      mRemoteStreamPort,
			      nullptr,
			      mDemuxerNet->mSession->getLoop(),
			      dataCb,
			      this,
			      &mStreamSock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:stream", -res);
		goto error;
	}

	rxBufSize = PDRAW_RTP_RXBUF_SIZE;
	res = tskt_socket_set_rxbuf_size(mStreamSock, rxBufSize);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_set_rxbuf_size");

	res = tskt_socket_get_rxbuf_size(mStreamSock);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_get_rxbuf_size");
	else if (res != 2 * rxBufSize)
		PDRAW_LOGW("failed to set rx buffer size: got %d, expecting %d",
			   res / 2,
			   rxBufSize);

	res = tskt_socket_set_class_selector(mStreamSock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW("failed to set class selector for stream socket");

	res = tskt_socket_new(mDemuxerNet->mLocalAddr.c_str(),
			      &mLocalControlPort,
			      mDemuxerNet->mRemoteAddr.c_str(),
			      mRemoteControlPort,
			      nullptr,
			      mDemuxerNet->mSession->getLoop(),
			      ctrlCb,
			      this,
			      &mControlSock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:control", -res);
		goto error;
	}
	res = tskt_socket_set_class_selector(mControlSock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW("failed to set class selector for control socket");

	return 0;

error:
	err = tskt_socket_destroy(mStreamSock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mStreamSock = nullptr;
	err = tskt_socket_destroy(mControlSock);
	if (err < 0)
		PDRAW_LOG_ERRNO("tskt_socket_destroy", -err);
	mControlSock = nullptr;
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
	return res;
}


struct tpkt_packet *StreamDemuxerNet::VideoMediaNet::newRxPkt()
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


int StreamDemuxerNet::VideoMediaNet::processDataPkt(struct tpkt_packet *pkt)
{
	int ret;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	if (isRtpPaused())
		return 0;

	/* Process received packet */
	ret = vstrm_receiver_recv_data(mReceiver, pkt);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vstrm_receiver_recv_data", -ret);

	return ret;
}


int StreamDemuxerNet::VideoMediaNet::processCtrlPkt(struct tpkt_packet *pkt)
{
	int ret;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	if (isRtpPaused())
		return 0;

	/* Process received packet */
	ret = vstrm_receiver_recv_ctrl(mReceiver, pkt);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vstrm_receiver_recv_ctrl", -ret);

	return 0;
}


void StreamDemuxerNet::VideoMediaNet::dataCb([[maybe_unused]] int fd,
					     [[maybe_unused]] uint32_t events,
					     void *userdata)
{

	auto *self = static_cast<VideoMediaNet *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	while (true) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mStreamSock, self->mRxPkt);
		if (res < 0)
			return;

		/* Discard any data received before starting a vstrm_sender */
		if (!self->mReceiver)
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
		tpkt_unref(self->mRxPkt);
		self->mRxPkt = newPkt;
	}
}


void StreamDemuxerNet::VideoMediaNet::ctrlCb([[maybe_unused]] int fd,
					     [[maybe_unused]] uint32_t events,
					     void *userdata)
{

	auto *self = static_cast<VideoMediaNet *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	while (true) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mControlSock, self->mRxPkt);
		if (res < 0)
			return;

		/* Discard any data received before starting a vstrm_sender */
		if (!self->mReceiver)
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
		tpkt_unref(self->mRxPkt);
		self->mRxPkt = newPkt;
	}
}

} /* namespace Pdraw */
