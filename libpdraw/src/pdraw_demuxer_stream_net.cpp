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
#define ULOG_TAG pdraw_dmxstrmnet
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_dmxstrmnet);

namespace Pdraw {


StreamDemuxerNet::StreamDemuxerNet(Session *session,
				   Element::Listener *elementListener,
				   Source::Listener *sourceListener,
				   Demuxer::Listener *demuxerListener) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      demuxerListener)
{
	Element::mName = "StreamDemuxerNet";
	Source::mName = "StreamDemuxerNet";
	mStreamSock = NULL;
	mControlSock = NULL;

	setState(CREATED);
}


StreamDemuxerNet::~StreamDemuxerNet(void)
{
	if (mState != STOPPED && mState != CREATED)
		ULOGW("demuxer is still running");

	stopRtpAvp();
}


int StreamDemuxerNet::setup(const std::string &url)
{
	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	mUrl = url;

	return 0;
}


int StreamDemuxerNet::setup(const std::string &localAddr,
			    uint16_t localStreamPort,
			    uint16_t localControlPort,
			    const std::string &remoteAddr,
			    uint16_t remoteStreamPort,
			    uint16_t remoteControlPort)
{
	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	mLocalAddr = (localAddr.length() > 0) ? localAddr : "0.0.0.0";
	mLocalStreamPort = localStreamPort;
	mLocalControlPort = localControlPort;
	mRemoteAddr = (remoteAddr.length() > 0) ? remoteAddr : "0.0.0.0";
	mRemoteStreamPort = remoteStreamPort;
	mRemoteControlPort = remoteControlPort;

	return 0;
}


int StreamDemuxerNet::startRtpAvp(void)
{
	int res;

	/* Create sockets only if not created during prepareSetup */
	if (mStreamSock == NULL && mControlSock == NULL) {
		res = createSockets();
		if (res != 0) {
			ULOG_ERRNO("createSockets", -res);
			goto error;
		}
	} else if ((mStreamSock != NULL && mControlSock == NULL) ||
		   (mStreamSock == NULL && mControlSock != NULL)) {
		ULOGE("Bad state, only one socket created !");
		res = -EPROTO;
		goto error;
	}

	ULOGD("startRtpAvp localStreamPort=%d localControlPort=%d",
	      mLocalStreamPort,
	      mLocalControlPort);

	/* Create the stream receiver */
	res = createReceiver();
	if (res < 0) {
		ULOG_ERRNO("createReceiver", -res);
		goto error;
	}

	return 0;

error:
	stopRtpAvp();
	return res;
}


int StreamDemuxerNet::stopRtpAvp(void)
{
	ULOGD("stopRtpAvp");
	destroyReceiver();
	if (mStreamSock != NULL) {
		delete mStreamSock;
		mStreamSock = NULL;
	}
	if (mControlSock != NULL) {
		delete mControlSock;
		mControlSock = NULL;
	}
	return 0;
}


int StreamDemuxerNet::createSockets(void)
{
	int res = 0;
	if (mLocalStreamPort == 0)
		mLocalStreamPort = DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT;
	if (mLocalControlPort == 0)
		mLocalControlPort = DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT;

	/* Create the sockets */
	mStreamSock = new InetSocket(mSession,
				     mLocalAddr,
				     mLocalStreamPort,
				     mRemoteAddr,
				     mRemoteStreamPort,
				     mSession->getLoop(),
				     dataCb,
				     this);
	if (mStreamSock == NULL) {
		ULOGE("failed to create stream socket");
		res = -EPROTO;
		goto error;
	}
	res = mStreamSock->setClass(IPTOS_PREC_FLASHOVERRIDE);
	if (res != 0)
		ULOGW("failed to set IP_TOS for stream socket");

	mControlSock = new InetSocket(mSession,
				      mLocalAddr,
				      mLocalControlPort,
				      mRemoteAddr,
				      mRemoteControlPort,
				      mSession->getLoop(),
				      ctrlCb,
				      this);
	if (mControlSock == NULL) {
		ULOGE("failed to create control socket");
		res = -EPROTO;
		goto error;
	}
	res = mControlSock->setClass(IPTOS_PREC_FLASHOVERRIDE);
	if (res != 0)
		ULOGW("failed to set IP_TOS for control socket");

	return 0;

error:
	if (mStreamSock != NULL) {
		delete mStreamSock;
		mStreamSock = NULL;
	}
	if (mControlSock != NULL) {
		delete mControlSock;
		mControlSock = NULL;
	}
	return res;
}


uint16_t StreamDemuxerNet::getSingleStreamLocalStreamPort(void)
{
	if (mState != STARTED) {
		ULOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mStreamSock == NULL) {
		ULOG_ERRNO("invalid stream socket", EPROTO);
		return 0;
	}

	return mStreamSock->getLocalPort();
}


uint16_t StreamDemuxerNet::getSingleStreamLocalControlPort(void)
{
	if (mState != STARTED) {
		ULOG_ERRNO("demuxer is not started", EPROTO);
		return 0;
	}
	if (mControlSock == NULL) {
		ULOG_ERRNO("invalid control socket", EPROTO);
		return 0;
	}

	return mControlSock->getLocalPort();
}


void StreamDemuxerNet::dataCb(int fd, uint32_t events, void *userdata)
{
	StreamDemuxerNet *self = (StreamDemuxerNet *)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = {0, 0};

	if (self == NULL)
		return;

	do {
		/* Read data */
		readlen = self->mStreamSock->read();

		/* Discard any data received before starting a vstrm_receiver */
		if (!self->mReceiver)
			continue;

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				self->mStreamSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0) {
				ULOG_ERRNO("time_get_monotonic", -res);
			}
			res = vstrm_receiver_recv_data(
				self->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0) {
				ULOG_ERRNO("vstrm_receiver_recv_ctrl", -res);
			}
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


void StreamDemuxerNet::ctrlCb(int fd, uint32_t events, void *userdata)
{
	StreamDemuxerNet *self = (StreamDemuxerNet *)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = {0, 0};

	if (self == NULL)
		return;

	do {
		/* Read data */
		readlen = self->mControlSock->read();

		/* Discard any data received before starting a vstrm_receiver */
		if (!self->mReceiver)
			continue;

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				self->mControlSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0) {
				ULOG_ERRNO("time_get_monotonic", -res);
			}
			res = vstrm_receiver_recv_ctrl(
				self->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0) {
				ULOG_ERRNO("vstrm_receiver_recv_ctrl", -res);
			}
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


int StreamDemuxerNet::sendCtrl(struct vstrm_receiver *stream,
			       struct pomp_buffer *buf)
{
	const void *cdata = NULL;
	size_t len = 0;
	ssize_t writelen = 0;

	if (buf == NULL) {
		ULOGE("invalid buffer");
		return -EINVAL;
	}

	/* Write data */
	pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
	writelen = mControlSock->write(cdata, len);
	return (writelen >= 0) ? 0 : (int)writelen;
}


int StreamDemuxerNet::prepareSetup(uint16_t *streamPort,
				   uint16_t *controlPort,
				   enum rtsp_lower_transport *lowerTransport)
{
	int res;

	if (streamPort == NULL || controlPort == NULL || lowerTransport == NULL)
		return -EINVAL;

	res = createSockets();
	if (res != 0) {
		ULOG_ERRNO("createSockets", -res);
		return res;
	}

	*streamPort = mStreamSock->getLocalPort();
	*controlPort = mControlSock->getLocalPort();
	*lowerTransport = RTSP_LOWER_TRANSPORT_UDP;

	return 0;
}

} /* namespace Pdraw */
