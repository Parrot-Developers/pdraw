/**
 * Parrot Drones Audio and Video Vector library
 * RTSP stream muxer - network (UDP/TCP) implementation
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

#define ULOG_TAG pdraw_rtspmuxer_net
#include <ulog.h>

#include "pdraw_muxer_stream_rtsp_net.hpp"
#include "pdraw_session.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {

constexpr size_t DEFAULT_RX_BUFFER_SIZE_NET = 1500;
constexpr size_t PDRAW_RTP_TXBUF_SIZE_NET = (1 * 1024 * 1024); /* In Bytes */
constexpr size_t PDRAW_RTP_RXBUF_SIZE_NET = (1 * 1024 * 1024); /* In Bytes */
constexpr size_t MUXER_STREAM_NET_DEFAULT_LOCAL_STREAM_PORT = 55004 + 10;
constexpr size_t MUXER_STREAM_NET_DEFAULT_LOCAL_CONTROL_PORT = 55005 + 10;


/**
 * RtspStreamMuxerNet
 */

RtspStreamMuxerNet::RtspStreamMuxerNet(
	Session *session,
	Element::Listener *elementListener,
	IPdraw::IMuxer::Listener *listener,
	MuxerWrapper *wrapper,
	const std::string &url,
	const struct pdraw_muxer_params *params) :
		RtspStreamMuxer(session,
				elementListener,
				listener,
				wrapper,
				url,
				params)
{
	Element::setClassName(__func__);
}


std::unique_ptr<RtspStreamMuxer::VideoMedia>
RtspStreamMuxerNet::createVideoMedia(enum pdraw_muxer_rtsp_transport transport)
{
	return std::make_unique<VideoMediaNet>(this, transport);
}


/**
 * RtspStreamMuxerNet::VideoMediaNet
 */

RtspStreamMuxerNet::VideoMediaNet::VideoMediaNet(
	RtspStreamMuxerNet *muxer,
	enum pdraw_muxer_rtsp_transport transport) :
		RtspStreamMuxer::VideoMedia(muxer),
		mMuxerNet(muxer),
		mLowerTransport(
			RtspStreamMuxer::
				pdrawMuxerRtspTransportToRtspLowerTransport(
					transport))
{
	std::string name = muxer->getName() + "#VideoMediaNet";
	Loggable::setName(name);
}


RtspStreamMuxerNet::VideoMediaNet::~VideoMediaNet()
{
	/* Clean up transport-specific resources before the base destructor
	 * calls destroySender() as a safety net.
	 * Explicit non-virtual call: derived class is already destroyed. */
	VideoMediaNet::stopRtpAvp();
}


int RtspStreamMuxerNet::VideoMediaNet::startRtpAvp()
{
	int res;
	const char *label = (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
				    ? "Channel"
				    : "Port";

	/* Create sockets only if not created during prepareSetup */
	if (mStrm.sock == nullptr && mCtrl.sock == nullptr &&
	    mLowerTransport != RTSP_LOWER_TRANSPORT_TCP) {
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


int RtspStreamMuxerNet::VideoMediaNet::stopRtpAvp()
{
	int err;
	PDRAW_LOGD("stopRtpAvp (Net)");
	destroySender();
	if (mStrm.sock != nullptr) {
		err = tskt_socket_destroy(mStrm.sock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy(stream)", -err);
		mStrm.sock = nullptr;
	}
	if (mCtrl.sock != nullptr) {
		err = tskt_socket_destroy(mCtrl.sock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy(control)", -err);
		mCtrl.sock = nullptr;
	}
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
	return 0;
}


int RtspStreamMuxerNet::VideoMediaNet::prepareSetup()
{
	int res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}
	return 0;
}


void RtspStreamMuxerNet::VideoMediaNet::setRemoteStreamPort(uint16_t port)
{
	mStrm.remotePort = port;

	if (mStrm.sock != nullptr) {
		int res = tskt_socket_set_remote(
			mStrm.sock,
			mMuxerNet->mUrl->getResolvedHost().c_str(),
			mStrm.remotePort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(strm)", -res);
	}
}


void RtspStreamMuxerNet::VideoMediaNet::setRemoteControlPort(uint16_t port)
{
	mCtrl.remotePort = port;

	if (mCtrl.sock != nullptr) {
		int res = tskt_socket_set_remote(
			mCtrl.sock,
			mMuxerNet->mUrl->getResolvedHost().c_str(),
			mCtrl.remotePort);
		if (res < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(ctrl)", -res);
	}
}


int RtspStreamMuxerNet::VideoMediaNet::createSockets()
{
	int res;
	int err;
	int txBufSize;
	int rxBufSize;

	if (mLowerTransport != RTSP_LOWER_TRANSPORT_TCP) {
		if (mStrm.localPort == 0)
			mStrm.localPort =
				MUXER_STREAM_NET_DEFAULT_LOCAL_STREAM_PORT;
		if (mCtrl.localPort == 0)
			mCtrl.localPort =
				MUXER_STREAM_NET_DEFAULT_LOCAL_CONTROL_PORT;
	} else {
		mStrm.localPort = 0;
		mCtrl.localPort = 0;
	}

	/* Create the rx buffer */
	mRxBufLen = DEFAULT_RX_BUFFER_SIZE_NET;
	if (mRxPkt == nullptr) {
		mRxPkt = newRxPkt();
		if (mRxPkt == nullptr) {
			res = -ENOMEM;
			PDRAW_LOG_ERRNO("newRxPkt", -res);
			goto error;
		}
	}

	/* Sockets are not needed in TCP mode */
	if (mLowerTransport == RTSP_LOWER_TRANSPORT_TCP)
		return 0;

	/* Sockets will be created once address resolution is done */
	if (!mMuxerNet->mUrl->hasResolvedHost())
		return 0;

	/* Create the stream socket */
	res = tskt_socket_new(mMuxerNet->mLocalHost.c_str(),
			      &mStrm.localPort,
			      mMuxerNet->mUrl->getResolvedHost().c_str(),
			      mStrm.remotePort,
			      nullptr,
			      mMuxerNet->mSession->getLoop(),
			      dataCb,
			      this,
			      &mStrm.sock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:stream(%s)(%u->%u)",
				-res,
				mMuxerNet->mUrl->getResolvedHost().c_str(),
				mStrm.localPort,
				mStrm.remotePort);
		goto error;
	}

	PDRAW_LOGI("created data socket on port: local=%d, remote=%d",
		   mStrm.localPort,
		   mStrm.remotePort);

	txBufSize = PDRAW_RTP_TXBUF_SIZE_NET;
	res = tskt_socket_set_txbuf_size(mStrm.sock, txBufSize);
	if (res < 0)
		PDRAW_LOGW("tskt_socket_set_txbuf_size");
	rxBufSize = PDRAW_RTP_RXBUF_SIZE_NET;
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

	/* Create the control socket */
	res = tskt_socket_new(mMuxerNet->mLocalHost.c_str(),
			      &mCtrl.localPort,
			      mMuxerNet->mUrl->getResolvedHost().c_str(),
			      mCtrl.remotePort,
			      nullptr,
			      mMuxerNet->mSession->getLoop(),
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


struct tpkt_packet *RtspStreamMuxerNet::VideoMediaNet::newRxPkt()
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


void RtspStreamMuxerNet::VideoMediaNet::setRxPkt(struct tpkt_packet *newPkt)
{
	if (mRxPkt != nullptr)
		tpkt_unref(mRxPkt);
	mRxPkt = newPkt;
}


void RtspStreamMuxerNet::VideoMediaNet::dataCb([[maybe_unused]] int fd,
					       uint32_t events,
					       void *userdata)
{
	auto *self = static_cast<VideoMediaNet *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_OUT) != 0) {
		/* Notify sender that the socket is writable */
		res = self->notifyReadyToSend();
		if (res < 0)
			ULOG_ERRNO("notifyReadyToSend", -res);
	}

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		res = tskt_socket_read_pkt(self->mStrm.sock, self->mRxPkt);
		if (res < 0)
			return;

		/* Discard any data received before starting a vstrm_sender */
		if (!self->getStreamSocket())
			continue;

		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			return;

		if (readlen == 0)
			return;

		res = self->processDataPkt(self->mRxPkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("processDataPkt", -res);

		struct tpkt_packet *newPkt = self->newRxPkt();
		if (!newPkt) {
			PDRAW_LOG_ERRNO("newRxPkt", ENOMEM);
			return;
		}
		self->setRxPkt(newPkt);
	}
}


void RtspStreamMuxerNet::VideoMediaNet::ctrlCb([[maybe_unused]] int fd,
					       uint32_t events,
					       void *userdata)
{
	auto *self = static_cast<VideoMediaNet *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		res = tskt_socket_read_pkt(self->mCtrl.sock, self->mRxPkt);
		if (res < 0)
			return;

		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			return;

		if (readlen == 0)
			return;

		res = self->processCtrlPkt(self->mRxPkt);
		if (res < 0)
			PDRAW_LOG_ERRNO("processCtrlPkt", -res);

		struct tpkt_packet *newPkt = self->newRxPkt();
		if (!newPkt) {
			PDRAW_LOG_ERRNO("newRxPkt", ENOMEM);
			return;
		}
		self->setRxPkt(newPkt);
	}
}

} /* namespace Pdraw */
