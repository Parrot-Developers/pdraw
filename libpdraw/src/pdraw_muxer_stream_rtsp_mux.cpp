/**
 * Parrot Drones Audio and Video Vector library
 * RTSP stream muxer - libmux implementation
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

#define ULOG_TAG pdraw_rtspmuxer_mux
#include <ulog.h>

#include "pdraw_muxer_stream_rtsp_mux.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"
#include "pdraw_session.hpp"

#ifdef BUILD_LIBMUX
#	include <sys/time.h>
#	include <time.h>
#	include <futils/futils.h>
#	include <libmux-arsdk.h>
#endif

ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef BUILD_LIBMUX

constexpr size_t DEFAULT_RX_BUFFER_SIZE_MUX = 1500;

namespace Pdraw {


const size_t RtspStreamMuxerMux::VideoMediaMux::mHeaderExtCount = 1;

const struct rtsp_header_ext RtspStreamMuxerMux::VideoMediaMux::mHeaderExt = {
	.key = RTSP_HEADER_EXT_PARROT_LINK_TYPE,
	.value = "mux",
};


/**
 * RtspStreamMuxerMux
 */

RtspStreamMuxerMux::RtspStreamMuxerMux(
	Session *session,
	Element::Listener *elementListener,
	IPdraw::IMuxer::Listener *listener,
	MuxerWrapper *wrapper,
	const std::string &url,
	struct mux_ctx *mux,
	const std::string &remoteHost,
	const struct pdraw_muxer_params *params) :
		RtspStreamMuxer(session,
				elementListener,
				listener,
				wrapper,
				url,
				params),
		mRemoteHost(remoteHost)
{
	Element::setClassName(__func__);

	if (!setMux(mux))
		PDRAW_LOGE("invalid mux handle");
}


RtspStreamMuxerMux::~RtspStreamMuxerMux()
{
	setMux(nullptr);
}


bool RtspStreamMuxerMux::setMux(struct mux_ctx *mux)
{
	if (mMux != nullptr)
		mux_unref(mMux);

	mMux = mux;
	if (mMux != nullptr) {
		mux_ref(mMux);
		return true;
	}
	return false;
}


std::unique_ptr<RtspStreamMuxer::VideoMedia>
RtspStreamMuxerMux::createVideoMedia(enum pdraw_muxer_rtsp_transport transport)
{
	if (transport == PDRAW_MUXER_RTSP_TRANSPORT_TCP) {
		/* TCP interleaved: RTP flows over the RTSP TCP socket directly,
		 * no mux_ip_proxy needed. */
		return std::make_unique<VideoMediaMuxTcp>(this);
	}
	/* UDP: loopback sockets bridged to Wowza by mux_ip_proxy on the SC */
	return std::make_unique<VideoMediaMux>(this);
}


/**
 * RtspStreamMuxerMux::VideoMediaMux
 */

RtspStreamMuxerMux::VideoMediaMux::VideoMediaMux(RtspStreamMuxerMux *muxer) :
		RtspStreamMuxer::VideoMedia(muxer), mMuxerMux(muxer)
{
	mCallFinishSetupHandler.set([this]() { finishSetup(); });
}


RtspStreamMuxerMux::VideoMediaMux::~VideoMediaMux()
{
	/* Remove the idle handler before stopping (mirrors demuxer pattern) */
	int err = mMuxerMux->mSession->getPompLoop()->idleRemove(
		&mCallFinishSetupHandler);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	stopRtpAvp();
}


int RtspStreamMuxerMux::VideoMediaMux::startRtpAvp()
{
	int res;

	if (mMuxerMux->mMux == nullptr) {
		PDRAW_LOGE("invalid mux handle");
		return -EPROTO;
	}

	/* Loopback sockets and proxies were opened in prepareSetup() /
	 * proxyOpenCb(). We only need to create the vstrm sender here. */
	PDRAW_LOGI("startRtpAvp (Mux) localStreamPort=%d localControlPort=%d",
		   getLocalStreamPort(),
		   getLocalControlPort());

	res = createSender();
	if (res < 0) {
		PDRAW_LOG_ERRNO("createSender", -res);
		stopRtpAvp();
	}
	return res;
}


int RtspStreamMuxerMux::VideoMediaMux::stopRtpAvp()
{
	int err;
	PDRAW_LOGD("stopRtpAvp (Mux)");

	destroySender();

	if (mStreamProxy != nullptr) {
		err = mux_ip_proxy_destroy(mStreamProxy);
		if (err < 0)
			PDRAW_LOG_ERRNO("mux_ip_proxy_destroy(stream)", -err);
		mStreamProxy = nullptr;
	}
	if (mControlProxy != nullptr) {
		err = mux_ip_proxy_destroy(mControlProxy);
		if (err < 0)
			PDRAW_LOG_ERRNO("mux_ip_proxy_destroy(control)", -err);
		mControlProxy = nullptr;
	}
	/* closeSockets() also unrefs mRxPkt */
	closeSockets();
	mStreamProxyOpened = false;
	mControlProxyOpened = false;
	return 0;
}


uint16_t RtspStreamMuxerMux::VideoMediaMux::getLocalStreamPort() const
{
	if (mStreamProxy == nullptr)
		return 0;
	return mux_ip_proxy_get_peerport(mStreamProxy);
}


uint16_t RtspStreamMuxerMux::VideoMediaMux::getLocalControlPort() const
{
	if (mControlProxy == nullptr)
		return 0;
	return mux_ip_proxy_get_peerport(mControlProxy);
}


uint16_t RtspStreamMuxerMux::VideoMediaMux::getRemoteStreamPort() const
{
	if (mStreamProxy == nullptr)
		return 0;
	return mux_ip_proxy_get_remote_port(mStreamProxy);
}


uint16_t RtspStreamMuxerMux::VideoMediaMux::getRemoteControlPort() const
{
	if (mControlProxy == nullptr)
		return 0;
	return mux_ip_proxy_get_remote_port(mControlProxy);
}


void RtspStreamMuxerMux::VideoMediaMux::setRemoteStreamPort(uint16_t port)
{
	if (mStreamProxy != nullptr)
		mux_ip_proxy_set_udp_remote(
			mStreamProxy, mMuxerMux->mRemoteHost.c_str(), port, -1);
}


void RtspStreamMuxerMux::VideoMediaMux::setRemoteControlPort(uint16_t port)
{
	if (mControlProxy != nullptr)
		mux_ip_proxy_set_udp_remote(mControlProxy,
					    mMuxerMux->mRemoteHost.c_str(),
					    port,
					    -1);
}


const struct rtsp_header_ext *
RtspStreamMuxerMux::VideoMediaMux::getHeaderExt() const
{
	return &mHeaderExt;
}


size_t RtspStreamMuxerMux::VideoMediaMux::getHeaderExtCount() const
{
	return mHeaderExtCount;
}


int RtspStreamMuxerMux::VideoMediaMux::prepareSetup()
{
	/* remote_host: the SC's SLAVE will connect UDP to this address.
	 * This is the real RTSP server host (e.g. Wowza), passed in
	 * explicitly by the caller since mUrl only ever holds the loopback
	 * URL used for the mux-tunneled TCP control channel. */
	const std::string &remoteHost = mMuxerMux->mRemoteHost;
	/* clang-format off */
	struct mux_ip_proxy_info info = {
		.protocol = {
			.transport = MUX_IP_PROXY_TRANSPORT_UDP,
			.application = MUX_IP_PROXY_APPLICATION_NONE,
		},
		.remote_host = remoteHost.c_str(),
		/* remote port will be updated after RTSP SETUP response */
		.remote_port = 0,
		.udp_redirect_port = 0,
	};
	/* clang-format on */
	struct mux_ip_proxy_cbs cbs = {
		.open = proxyOpenCb,
		.close = proxyCloseCb,
		.remote_update = proxyUpdateCb,
		.resolution_failed = proxyFailedCb,
		.userdata = this,
	};
	int res;

	res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}

	info.udp_redirect_port = tskt_socket_get_local_port(mStreamSock);
	res = mux_ip_proxy_new(mMuxerMux->mMux, &info, &cbs, -1, &mStreamProxy);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mux_ip_proxy_new(stream)", -res);
		goto error;
	}

	info.udp_redirect_port = tskt_socket_get_local_port(mControlSock);
	res = mux_ip_proxy_new(
		mMuxerMux->mMux, &info, &cbs, -1, &mControlProxy);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mux_ip_proxy_new(control)", -res);
		goto error;
	}

	/* Setup completes asynchronously once both proxies call back
	 * proxyOpenCb */
	return -EINPROGRESS;

error:
	closeSockets();
	if (mStreamProxy != nullptr) {
		mux_ip_proxy_destroy(mStreamProxy);
		mStreamProxy = nullptr;
	}
	if (mControlProxy != nullptr) {
		mux_ip_proxy_destroy(mControlProxy);
		mControlProxy = nullptr;
	}
	return res;
}


int RtspStreamMuxerMux::VideoMediaMux::createSockets()
{
	int res;

	/* Allocate the rx packet buffer */
	mRxBufLen = DEFAULT_RX_BUFFER_SIZE_MUX;
	mRxPkt = newRxPkt();
	if (mRxPkt == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("newRxPkt", -res);
		goto error;
	}

	/* Create loopback sockets. The mux_ip_proxy will bridge them to the
	 * SkyController over the mux tunnel. */
	res = tskt_socket_new("127.0.0.1",
			      nullptr,
			      "127.0.0.1",
			      0,
			      nullptr,
			      mMuxerMux->mSession->getLoop(),
			      dataCb,
			      this,
			      &mStreamSock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new(stream loopback)", -res);
		goto error;
	}

	res = tskt_socket_new("127.0.0.1",
			      nullptr,
			      "127.0.0.1",
			      0,
			      nullptr,
			      mMuxerMux->mSession->getLoop(),
			      ctrlCb,
			      this,
			      &mControlSock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new(control loopback)", -res);
		goto error;
	}

	return 0;

error:
	closeSockets();
	return res;
}


void RtspStreamMuxerMux::VideoMediaMux::closeSockets()
{
	int err;
	if (mStreamSock != nullptr) {
		err = tskt_socket_destroy(mStreamSock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy(stream)", -err);
		mStreamSock = nullptr;
	}
	if (mControlSock != nullptr) {
		err = tskt_socket_destroy(mControlSock);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_destroy(control)", -err);
		mControlSock = nullptr;
	}
	tpkt_unref(mRxPkt);
	mRxPkt = nullptr;
}


struct tpkt_packet *RtspStreamMuxerMux::VideoMediaMux::newRxPkt()
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


void RtspStreamMuxerMux::VideoMediaMux::setRxPkt(struct tpkt_packet *newPkt)
{
	if (mRxPkt != nullptr)
		tpkt_unref(mRxPkt);
	mRxPkt = newPkt;
}


void RtspStreamMuxerMux::VideoMediaMux::dataCb([[maybe_unused]] int fd,
					       uint32_t events,
					       void *userdata)
{
	auto *self = static_cast<VideoMediaMux *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_OUT) != 0) {
		res = self->notifyReadyToSend();
		if (res < 0)
			ULOG_ERRNO("notifyReadyToSend", -res);
	}

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		res = tskt_socket_read_pkt(self->mStreamSock, self->mRxPkt);
		if (res < 0)
			return;

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


void RtspStreamMuxerMux::VideoMediaMux::ctrlCb([[maybe_unused]] int fd,
					       uint32_t events,
					       void *userdata)
{
	auto *self = static_cast<VideoMediaMux *>(userdata);
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if ((events & POMP_FD_EVENT_IN) == 0)
		return;

	while (true) {
		res = tskt_socket_read_pkt(self->mControlSock, self->mRxPkt);
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


void RtspStreamMuxerMux::VideoMediaMux::proxyOpenCb(struct mux_ip_proxy *proxy,
						    uint16_t localPort,
						    void *userdata)
{
	int err;
	auto *self = static_cast<VideoMediaMux *>(userdata);

	if (proxy == self->mStreamProxy) {
		self->mStreamProxyOpened = true;
		/* Connect the loopback socket to the proxy's local UDP port so
		 * that vstrm can reach the proxy when sending outbound RTP. */
		err = tskt_socket_set_remote(
			self->mStreamSock, "127.0.0.1", localPort);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(stream)", -err);
	} else if (proxy == self->mControlProxy) {
		self->mControlProxyOpened = true;
		err = tskt_socket_set_remote(
			self->mControlSock, "127.0.0.1", localPort);
		if (err < 0)
			PDRAW_LOG_ERRNO("tskt_socket_set_remote(control)",
					-err);
	} else {
		PDRAW_LOGE("unknown proxy opened");
		return;
	}

	if (self->mStreamProxyOpened && self->mControlProxyOpened) {
		pomp::Loop *loop = self->mMuxerMux->mSession->getPompLoop();
		err = loop->idleRemove(&self->mCallFinishSetupHandler);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);
		err = loop->idleAdd(&self->mCallFinishSetupHandler);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
	}
}


void RtspStreamMuxerMux::VideoMediaMux::proxyCloseCb(struct mux_ip_proxy *proxy,
						     void *userdata)
{
	auto *self = static_cast<VideoMediaMux *>(userdata);

	if (proxy == self->mStreamProxy) {
		self->mStreamProxyOpened = false;
	} else if (proxy == self->mControlProxy) {
		self->mControlProxyOpened = false;
	} else {
		PDRAW_LOGE("unknown proxy closed");
	}
}


void RtspStreamMuxerMux::VideoMediaMux::proxyUpdateCb(
	[[maybe_unused]] struct mux_ip_proxy *proxy,
	[[maybe_unused]] void *userdata)
{
	/* TODO ? */
}


void RtspStreamMuxerMux::VideoMediaMux::proxyFailedCb(
	struct mux_ip_proxy *proxy,
	int err,
	void *userdata)
{
	const auto *self = static_cast<VideoMediaMux *>(userdata);
	const char *name = "unknown";
	if (proxy == self->mStreamProxy)
		name = "stream";
	else if (proxy == self->mControlProxy)
		name = "control";
	PDRAW_LOG_ERRNO("%s proxy failed to resolve", -err, name);
}


/**
 * RtspStreamMuxerMux::VideoMediaMuxTcp
 */

RtspStreamMuxerMux::VideoMediaMuxTcp::VideoMediaMuxTcp(
	RtspStreamMuxerMux *muxer) :
		RtspStreamMuxer::VideoMedia(muxer)
{
}


int RtspStreamMuxerMux::VideoMediaMuxTcp::startRtpAvp()
{
	PDRAW_LOGI("startRtpAvp (Mux/TCP interleaved)");
	int res = createSender();
	if (res < 0) {
		PDRAW_LOG_ERRNO("createSender", -res);
		return res;
	}
	return 0;
}

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
