/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer - mux implementation
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

#define ULOG_TAG pdraw_dmxstrmmux
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_session.hpp"

#define DEFAULT_RX_BUFFER_SIZE 1500

#ifdef BUILD_LIBMUX

#	include <sys/time.h>
#	include <time.h>

#	include <futils/futils.h>
#	include <libmux-arsdk.h>

namespace Pdraw {


const size_t StreamDemuxerMux::VideoMediaMux::mHeaderExtCount = 1;


const struct rtsp_header_ext StreamDemuxerMux::VideoMediaMux::mHeaderExt = {
	.key = RTSP_HEADER_EXT_PARROT_LINK_TYPE,
	.value = "mux",
};


StreamDemuxerMux::StreamDemuxerMux(Session *session,
				   Element::Listener *elementListener,
				   Source::Listener *sourceListener,
				   IPdraw::IDemuxer *demuxer,
				   IPdraw::IDemuxer::Listener *demuxerListener,
				   const std::string &url,
				   struct mux_ctx *mux) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      demuxer,
			      demuxerListener),
		mMux(nullptr)
{
	Element::setClassName(__func__);

	if (!setMux(mux))
		PDRAW_LOGE("invalid mux handle");

	mUrl = url;

	setState(CREATED);
}


StreamDemuxerMux::~StreamDemuxerMux(void)
{
	destroyAllVideoMedias();
	setMux(nullptr);
}


StreamDemuxer::VideoMedia *StreamDemuxerMux::createVideoMedia(void)
{
	return (StreamDemuxer::VideoMedia *)new VideoMediaMux(this);
}


bool StreamDemuxerMux::setMux(struct mux_ctx *mux)
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


StreamDemuxerMux::VideoMediaMux::VideoMediaMux(StreamDemuxerMux *demuxer) :
		VideoMedia(demuxer), mDemuxerMux(demuxer), mStreamSock(nullptr),
		mStreamProxy(nullptr), mStreamProxyOpened(false),
		mControlSock(nullptr), mControlProxy(nullptr),
		mControlProxyOpened(false), mRxPkt(nullptr), mRxBufLen(0)
{
}


StreamDemuxerMux::VideoMediaMux::~VideoMediaMux(void)
{
	stopRtpAvp();
	struct pomp_loop *loop = mDemuxerMux->mSession->getLoop();
	pomp_loop_idle_remove(loop, callFinishSetup, this);
}


int StreamDemuxerMux::VideoMediaMux::startRtpAvp(void)
{
	int res;

	if (mDemuxerMux->mMux == nullptr) {
		PDRAW_LOGE("invalid mux handle");
		return -EPROTO;
	}

	if (mDemuxerMux->mSessionProtocol == RTSP) {
		/* Everything should be done in prepareSetup() */
	} else {
		res = mux_channel_open(mDemuxerMux->mMux,
				       MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
				       &legacyDataCb,
				       this);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mux_channel_open", -res);
			goto error;
		}
		res = mux_channel_open(mDemuxerMux->mMux,
				       MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
				       &legacyCtrlCb,
				       this);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mux_channel_open", -res);
			goto error;
		}
	}

	createReceiver();
	return 0;

error:
	stopRtpAvp();
	return res;
}


int StreamDemuxerMux::VideoMediaMux::stopRtpAvp(void)
{
	destroyReceiver();
	if (mDemuxerMux->mMux != nullptr) {
		if (mDemuxerMux->mSessionProtocol == RTSP) {
			closeSockets();
			if (mStreamProxy) {
				mux_ip_proxy_destroy(mStreamProxy);
				mStreamProxy = nullptr;
			}
			if (mControlProxy) {
				mux_ip_proxy_destroy(mControlProxy);
				mControlProxy = nullptr;
			}
		} else {
			mux_channel_close(mDemuxerMux->mMux,
					  MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
			mux_channel_close(mDemuxerMux->mMux,
					  MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
		}
	}
	return 0;
}


int StreamDemuxerMux::VideoMediaMux::sendCtrl(struct vstrm_receiver *stream,
					      struct tpkt_packet *pkt)
{
	int res;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(pkt == nullptr, EINVAL);

	/* Write data */
	res = tskt_socket_write_pkt(mControlSock, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


int StreamDemuxerMux::VideoMediaMux::prepareSetup(void)
{
	/* clang-format off */
	struct mux_ip_proxy_info info = {
		.protocol = {
			.transport = MUX_IP_PROXY_TRANSPORT_UDP,
			.application = MUX_IP_PROXY_APPLICATION_NONE,
		},
		.remote_host = "skycontroller",
		/* remote port will be updated after setup */
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

	int res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}

	info.udp_redirect_port = tskt_socket_get_local_port(mStreamSock);
	res = mux_ip_proxy_new(
		mDemuxerMux->mMux, &info, &cbs, -1, &mStreamProxy);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mux_ip_proxy_new(rtp)", -res);
		goto error;
	}
	info.udp_redirect_port = tskt_socket_get_local_port(mControlSock);
	res = mux_ip_proxy_new(
		mDemuxerMux->mMux, &info, &cbs, -1, &mControlProxy);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mux_ip_proxy_new(rtcp)", -res);
		goto error;
	}

	return -EINPROGRESS;

error:
	closeSockets();
	if (mStreamProxy) {
		mux_ip_proxy_destroy(mStreamProxy);
		mStreamProxy = nullptr;
	}
	if (mControlProxy) {
		mux_ip_proxy_destroy(mControlProxy);
		mControlProxy = nullptr;
	}
	return res;
}


enum rtsp_lower_transport
StreamDemuxerMux::VideoMediaMux::getLowerTransport(void)
{
	return RTSP_LOWER_TRANSPORT_UDP;
}


uint16_t StreamDemuxerMux::VideoMediaMux::getLocalStreamPort(void)
{
	return mux_ip_proxy_get_peerport(mStreamProxy);
}


uint16_t StreamDemuxerMux::VideoMediaMux::getLocalControlPort(void)
{
	return mux_ip_proxy_get_peerport(mControlProxy);
}


uint16_t StreamDemuxerMux::VideoMediaMux::getRemoteStreamPort(void)
{
	return mux_ip_proxy_get_remote_port(mStreamProxy);
}


uint16_t StreamDemuxerMux::VideoMediaMux::getRemoteControlPort(void)
{
	return mux_ip_proxy_get_remote_port(mControlProxy);
}


const struct rtsp_header_ext *
StreamDemuxerMux::VideoMediaMux::getHeaderExt(void)
{
	return &mHeaderExt;
}


size_t StreamDemuxerMux::VideoMediaMux::getHeaderExtCount(void)
{
	return mHeaderExtCount;
}


void StreamDemuxerMux::VideoMediaMux::setLocalStreamPort(uint16_t port)
{
	/* Nothing to do */
}


void StreamDemuxerMux::VideoMediaMux::setLocalControlPort(uint16_t port)
{
	/* Nothing to do */
}


void StreamDemuxerMux::VideoMediaMux::setRemoteStreamPort(uint16_t port)
{
	mux_ip_proxy_set_udp_remote(mStreamProxy, "skycontroller", port, -1);
}


void StreamDemuxerMux::VideoMediaMux::setRemoteControlPort(uint16_t port)
{
	mux_ip_proxy_set_udp_remote(mControlProxy, "skycontroller", port, -1);
}


void StreamDemuxerMux::VideoMediaMux::legacyDataCb(struct mux_ctx *ctx,
						   uint32_t chanid,
						   enum mux_channel_event event,
						   struct pomp_buffer *buf,
						   void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	int res;
	struct tpkt_packet *pkt = nullptr;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	if (self == nullptr)
		return;

	res = tpkt_new_from_buffer(buf, &pkt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_new_from_buffer", -res);
		return;
	}

	res = time_get_monotonic(&ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_get_monotonic", -res);
		goto out;
	}
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_timespec_to_us", -res);
		goto out;
	}
	res = tpkt_set_timestamp(pkt, curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_set_timestamp", -res);
		goto out;
	}

	res = vstrm_receiver_recv_data(self->mReceiver, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("vstrm_receiver_recv_data", -res);

out:
	tpkt_unref(pkt);
}


void StreamDemuxerMux::VideoMediaMux::legacyCtrlCb(struct mux_ctx *ctx,
						   uint32_t chanid,
						   enum mux_channel_event event,
						   struct pomp_buffer *buf,
						   void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	int res;
	struct tpkt_packet *pkt = nullptr;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	if (self == nullptr)
		return;

	res = tpkt_new_from_buffer(buf, &pkt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_new_from_buffer", -res);
		return;
	}

	res = time_get_monotonic(&ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_get_monotonic", -res);
		goto out;
	}
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("time_timespec_to_us", -res);
		goto out;
	}
	res = tpkt_set_timestamp(pkt, curTime);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tpkt_set_timestamp", -res);
		goto out;
	}

	res = vstrm_receiver_recv_ctrl(self->mReceiver, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("vstrm_receiver_recv_ctrl", -res);

out:
	tpkt_unref(pkt);
}


int StreamDemuxerMux::VideoMediaMux::createSockets(void)
{
	int res;
	if (mLocalStreamPort == 0)
		mLocalStreamPort = DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT;
	if (mLocalControlPort == 0)
		mLocalControlPort = DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT;

	/* Create the rx buffer */
	mRxBufLen = DEFAULT_RX_BUFFER_SIZE;
	mRxPkt = newRxPkt();
	if (mRxPkt == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("newRxPkt", -res);
		goto error;
	}

	/* Create the sockets */
	res = tskt_socket_new("127.0.0.1",
			      &mLocalStreamPort,
			      "127.0.0.1",
			      mRemoteStreamPort,
			      nullptr,
			      mDemuxerMux->mSession->getLoop(),
			      dataCb,
			      this,
			      &mStreamSock);
	if (res < 0) {
		PDRAW_LOG_ERRNO("tskt_socket_new:stream", -res);
		goto error;
	}
	res = tskt_socket_set_class_selector(mStreamSock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0)
		PDRAW_LOGW("failed to set class selector for stream socket");

	res = tskt_socket_new("127.0.0.1",
			      &mLocalControlPort,
			      "127.0.0.1",
			      mRemoteControlPort,
			      nullptr,
			      mDemuxerMux->mSession->getLoop(),
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
	closeSockets();
	return res;
}

void StreamDemuxerMux::VideoMediaMux::closeSockets(void)
{
	int err;
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
}


struct tpkt_packet *StreamDemuxerMux::VideoMediaMux::newRxPkt(void)
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


void StreamDemuxerMux::VideoMediaMux::dataCb(int fd,
					     uint32_t events,
					     void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	while (1) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mStreamSock, self->mRxPkt);
		if (res < 0)
			break;

		/* Discard any data received before starting a vstrm_receiver */
		if (!self->mReceiver)
			continue;

		/* Something read? */
		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			break;
		if (readlen != 0) {
			/* Allocate new packet for replacement */
			struct tpkt_packet *newPkt = self->newRxPkt();
			if (!newPkt) {
				PDRAW_LOG_ERRNO("newRxPkt", ENOMEM);
				break;
			}
			/* Process received packet */
			res = vstrm_receiver_recv_data(self->mReceiver,
						       self->mRxPkt);
			/* Replace processed packet with new one */
			tpkt_unref(self->mRxPkt);
			self->mRxPkt = newPkt;
			if (res < 0)
				PDRAW_LOG_ERRNO("vstrm_receiver_recv_data",
						-res);
		} else {
			/* TODO: EOF */
			break;
		}
	}
}


void StreamDemuxerMux::VideoMediaMux::ctrlCb(int fd,
					     uint32_t events,
					     void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	int res;
	size_t readlen = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	while (1) {
		/* Read data */
		res = tskt_socket_read_pkt(self->mControlSock, self->mRxPkt);
		if (res < 0)
			break;

		/* Discard any data received before starting a vstrm_receiver */
		if (!self->mReceiver)
			continue;

		/* Something read? */
		res = tpkt_get_cdata(self->mRxPkt, nullptr, &readlen, nullptr);
		if (res < 0)
			break;
		if (readlen != 0) {
			/* Allocate new packet for replacement */
			struct tpkt_packet *newPkt = self->newRxPkt();
			if (!newPkt) {
				PDRAW_LOG_ERRNO("newRxPkt", ENOMEM);
				break;
			}
			/* Process received packet */
			res = vstrm_receiver_recv_ctrl(self->mReceiver,
						       self->mRxPkt);
			/* Replace processed packet with new one */
			tpkt_unref(self->mRxPkt);
			self->mRxPkt = newPkt;
			if (res < 0)
				PDRAW_LOG_ERRNO("vstrm_receiver_recv_ctrl",
						-res);
		} else {
			/* TODO: EOF */
			break;
		}
	}
}

void StreamDemuxerMux::VideoMediaMux::callFinishSetup(void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	self->finishSetup();
}

void StreamDemuxerMux::VideoMediaMux::proxyOpenCb(struct mux_ip_proxy *proxy,
						  uint16_t localPort,
						  void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	if (proxy == self->mStreamProxy) {
		self->mStreamProxyOpened = true;
	} else if (proxy == self->mControlProxy) {
		self->mControlProxyOpened = true;
	} else {
		PDRAW_LOGE("uknown proxy opened");
		return;
	}

	if (self->mStreamProxyOpened && self->mControlProxyOpened) {
		struct pomp_loop *loop = self->mDemuxerMux->mSession->getLoop();
		pomp_loop_idle_remove(loop, callFinishSetup, self);
		pomp_loop_idle_add(loop, callFinishSetup, self);
	}
}

void StreamDemuxerMux::VideoMediaMux::proxyCloseCb(struct mux_ip_proxy *proxy,
						   void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	if (proxy == self->mStreamProxy) {
		self->mStreamProxyOpened = false;
	} else if (proxy == self->mControlProxy) {
		self->mControlProxyOpened = false;
	} else {
		PDRAW_LOGE("uknown proxy closed");
		return;
	}
}

void StreamDemuxerMux::VideoMediaMux::proxyUpdateCb(struct mux_ip_proxy *proxy,
						    void *userdata)
{
	/* TODO ? */
}

void StreamDemuxerMux::VideoMediaMux::proxyFailedCb(struct mux_ip_proxy *proxy,
						    int err,
						    void *userdata)
{
	VideoMediaMux *self = (VideoMediaMux *)userdata;
	const char *name = "unknown";
	if (proxy == self->mStreamProxy)
		name = "stream";
	else if (proxy == self->mControlProxy)
		name = "control";
	PDRAW_LOG_ERRNO("%s proxy failed to resolve", -err, name);
}

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
