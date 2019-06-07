/**
 * Parrot Drones Awesome Video Viewer Library
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
ULOG_DECLARE_TAG(ULOG_TAG);

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

#define DEFAULT_RX_BUFFER_SIZE 1500


namespace Pdraw {


StreamDemuxerNet::StreamDemuxerNet(Session *session,
				   Element::Listener *elementListener,
				   CodedSource::Listener *sourceListener,
				   IPdraw::IDemuxer *demuxer,
				   IPdraw::IDemuxer::Listener *demuxerListener,
				   const std::string &url) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      demuxer,
			      demuxerListener),
		mSingleLocalStreamPort(0), mSingleLocalControlPort(0),
		mSingleRemoteStreamPort(0), mSingleRemoteControlPort(0)
{
	Element::setClassName(__func__);

	mUrl = url;

	setState(CREATED);
}


StreamDemuxerNet::StreamDemuxerNet(Session *session,
				   Element::Listener *elementListener,
				   CodedSource::Listener *sourceListener,
				   IPdraw::IDemuxer *demuxer,
				   IPdraw::IDemuxer::Listener *demuxerListener,
				   const std::string &localAddr,
				   uint16_t localStreamPort,
				   uint16_t localControlPort,
				   const std::string &remoteAddr,
				   uint16_t remoteStreamPort,
				   uint16_t remoteControlPort) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      demuxer,
			      demuxerListener),
		mSingleLocalStreamPort(localStreamPort),
		mSingleLocalControlPort(localControlPort),
		mSingleRemoteStreamPort(remoteStreamPort),
		mSingleRemoteControlPort(remoteControlPort)
{
	Element::setClassName(__func__);

	mLocalAddr = (localAddr.length() > 0) ? localAddr : "0.0.0.0";
	mRemoteAddr = (remoteAddr.length() > 0) ? remoteAddr : "0.0.0.0";

	setState(CREATED);
}


StreamDemuxerNet::~StreamDemuxerNet(void)
{
	return;
}


uint16_t StreamDemuxerNet::getSingleStreamLocalStreamPort(void)
{
	if (mState != STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mVideoMedias.size() != 1) {
		PDRAW_LOG_ERRNO("invalid media count", EPROTO);
		return 0;
	}

	VideoMediaNet *media =
		dynamic_cast<VideoMediaNet *>(mVideoMedias.front());
	if (media == nullptr) {
		PDRAW_LOG_ERRNO("invalid media", EPROTO);
		return 0;
	}

	return media->getLocalStreamPort();
}


uint16_t StreamDemuxerNet::getSingleStreamLocalControlPort(void)
{
	if (mState != STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mVideoMedias.size() != 1) {
		PDRAW_LOG_ERRNO("invalid media count", EPROTO);
		return 0;
	}

	VideoMediaNet *media =
		dynamic_cast<VideoMediaNet *>(mVideoMedias.front());
	if (media == nullptr) {
		PDRAW_LOG_ERRNO("invalid media", EPROTO);
		return 0;
	}

	return media->getLocalControlPort();
}


StreamDemuxer::VideoMedia *StreamDemuxerNet::createVideoMedia(void)
{
	return (StreamDemuxer::VideoMedia *)new VideoMediaNet(this);
}


StreamDemuxerNet::VideoMediaNet::VideoMediaNet(StreamDemuxerNet *demuxer) :
		VideoMedia(demuxer), mDemuxerNet(demuxer), mStreamSock(nullptr),
		mControlSock(nullptr), mRxPkt(nullptr), mRxBufLen(0)
{
}


StreamDemuxerNet::VideoMediaNet::~VideoMediaNet(void)
{
	stopRtpAvp();
	tpkt_unref(mRxPkt);
}


int StreamDemuxerNet::VideoMediaNet::startRtpAvp(void)
{
	int res;

	/* Create sockets only if not created during prepareSetup */
	if (mStreamSock == nullptr && mControlSock == nullptr) {
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

	PDRAW_LOGD("startRtpAvp localStreamPort=%d localControlPort=%d",
		   mLocalStreamPort,
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


int StreamDemuxerNet::VideoMediaNet::stopRtpAvp(void)
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

	/* Write data */
	res = tskt_socket_write_pkt(mControlSock, pkt);
	if (res < 0)
		PDRAW_LOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


int StreamDemuxerNet::VideoMediaNet::prepareSetup(void)
{
	int res = createSockets();
	if (res != 0) {
		PDRAW_LOG_ERRNO("createSockets", -res);
		return res;
	}

	return 0;
}


enum rtsp_lower_transport
StreamDemuxerNet::VideoMediaNet::getLowerTransport(void)
{
	return RTSP_LOWER_TRANSPORT_UDP;
}


uint16_t StreamDemuxerNet::VideoMediaNet::getLocalStreamPort(void)
{
	if (mStreamSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid stream socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_local_port(mStreamSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getLocalControlPort(void)
{
	if (mControlSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid control socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_local_port(mControlSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getRemoteStreamPort(void)
{
	if (mStreamSock == nullptr) {
		PDRAW_LOG_ERRNO("invalid stream socket", EPROTO);
		return 0;
	}

	return tskt_socket_get_remote_port(mStreamSock);
}


uint16_t StreamDemuxerNet::VideoMediaNet::getRemoteControlPort(void)
{
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


int StreamDemuxerNet::VideoMediaNet::createSockets(void)
{
	int res, err;
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


struct tpkt_packet *StreamDemuxerNet::VideoMediaNet::newRxPkt(void)
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


void StreamDemuxerNet::VideoMediaNet::dataCb(int fd,
					     uint32_t events,
					     void *userdata)
{
	VideoMediaNet *self = (VideoMediaNet *)userdata;
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


void StreamDemuxerNet::VideoMediaNet::ctrlCb(int fd,
					     uint32_t events,
					     void *userdata)
{
	VideoMediaNet *self = (VideoMediaNet *)userdata;
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

} /* namespace Pdraw */
