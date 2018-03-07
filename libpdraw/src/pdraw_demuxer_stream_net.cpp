/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer, net implementation
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

#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_session.hpp"
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

namespace Pdraw {


StreamDemuxerNet::StreamDemuxerNet(
	Session *session) :
	StreamDemuxer(session)
{
	mStreamSock = NULL;
	mControlSock = NULL;
}


StreamDemuxerNet::~StreamDemuxerNet(
	void)
{
	if (mStreamSock != NULL) {
		delete mStreamSock;
		mStreamSock = NULL;
	}

	if (mControlSock != NULL) {
		delete mControlSock;
		mControlSock = NULL;
	}
}


int StreamDemuxerNet::open(
	const std::string &url,
	const std::string &ifaceAddr)
{
	int res;

	if (mConfigured) {
		PDRAW_LOGE("StreamDemuxerNet: demuxer is already configured");
		return -EEXIST;
	}

	std::string ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if (url.substr(0, 7) == "rtsp://") {
		res = openRtsp(url, ifaceAddr);
		if (res < 0) {
			PDRAW_LOG_ERRNO("StreamDemuxerNet: openRtsp",
				-res);
			return res;
		}
	} else if ((url.substr(0, 7) == "http://") && (ext == ".sdp")) {
		/* TODO */
		PDRAW_LOGE("StreamDemuxerNet: unsupported URL");
		return -ENOSYS;
	} else if ((url.front() == '/') && (ext == ".sdp")) {
		struct stat sb;
		FILE *f = NULL;
		char *s = NULL;

		f = fopen(url.c_str(), "r");
		if (f == NULL) {
			PDRAW_LOGE("StreamDemuxerNet: failed to open file '%s'",
				url.c_str());
			return -EIO;
		}

		res = fstat(fileno(f), &sb);
		if (res != 0) {
			PDRAW_LOGE("StreamDemuxerNet: stat failed on file '%s'",
				url.c_str());
			return -EIO;
		}

		s = (char *)calloc(1, sb.st_size + 1);
		if (s == NULL) {
			PDRAW_LOGE("StreamDemuxerNet: allocation failed");
			return -ENOMEM;
		}

		res = fread(s, sb.st_size, 1, f);
		if (res != 1) {
			PDRAW_LOGE("StreamDemuxerNet: failed to read "
				"from the input file");
			return -EIO;
		}

		s[sb.st_size] = '\0';
		std::string sdp(s);
		res = openWithSdp(sdp, ifaceAddr);
		if (res < 0) {
			PDRAW_LOG_ERRNO("StreamDemuxerNet: openWithSdp", -res);
			return -res;
		}
	} else {
		PDRAW_LOGE("StreamDemuxerNet: unsupported URL");
		return -ENOSYS;
	}

	mConfigured = true;
	PDRAW_LOGI("StreamDemuxerNet: demuxer is configured");

	return 0;
}


int StreamDemuxerNet::open(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	int res;

	if (mConfigured) {
		PDRAW_LOGE("StreamDemuxerNet: demuxer is already configured");
		return -EEXIST;
	}

	mLocalAddr = localAddr;
	mLocalStreamPort = localStreamPort;
	mLocalControlPort = localControlPort;
	mRemoteAddr = remoteAddr;
	mRemoteStreamPort = remoteStreamPort;
	mRemoteControlPort = remoteControlPort;
	mIfaceAddr = ifaceAddr;

	res = openRtpAvp();
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerNet: openRtpAvp", -res);
		return res;
	}

	mConfigured = true;
	PDRAW_LOGI("StreamDemuxerNet: demuxer is configured");

	return 0;
}


int StreamDemuxerNet::openSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	int res;

	if (mConfigured) {
		PDRAW_LOGE("StreamDemuxerNet: demuxer is already configured");
		return -EEXIST;
	}

	res = openWithSdp(sdp, ifaceAddr);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerNet: openWithSdp", -res);
		return res;
	}

	mConfigured = true;
	PDRAW_LOGI("StreamDemuxerNet: demuxer is configured");

	return 0;
}


int StreamDemuxerNet::openRtpAvp(
	void)
{
	int res;

	/* Create the sockets */
	mStreamSock = new InetSocket(mSession, mLocalAddr, mLocalStreamPort,
		mRemoteAddr, mRemoteStreamPort,
		mSession->getLoop(), dataCb, this);
	if (mStreamSock == NULL) {
		PDRAW_LOGE("StreamDemuxerNet: failed to create stream socket");
		res = -EPROTO;
		goto error;
	}
	mControlSock = new InetSocket(mSession, mLocalAddr, mLocalControlPort,
		mRemoteAddr, mRemoteControlPort,
		mSession->getLoop(), ctrlCb, this);
	if (mControlSock == NULL) {
		PDRAW_LOGE("StreamDemuxerNet: failed to create control socket");
		res = -EPROTO;
		goto error;
	}

	/* Create the stream receiver */
	res = createReceiver();
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: createReceiver", -res);
		goto error;
	}

	return 0;

error:
	destroyReceiver();
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


void StreamDemuxerNet::dataCb(
	int fd,
	uint32_t events,
	void *userdata)
{
	StreamDemuxerNet *self = (StreamDemuxerNet *)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = { 0, 0 };

	if (self == NULL) {
		PDRAW_LOGE("StreamDemuxerNet: invalid callback params");
		return;
	}

	do {
		/* Read data */
		readlen = self->mStreamSock->read();

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				self->mStreamSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0) {
				PDRAW_LOG_ERRNO("StreamDemuxerMux: "
					"time_get_monotonic", -res);
			}
			res = vstrm_receiver_recv_data(
				self->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0) {
				PDRAW_LOG_ERRNO("StreamDemuxerMux: "
					"vstrm_receiver_recv_ctrl", -res);
			}
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


void StreamDemuxerNet::ctrlCb(
	int fd,
	uint32_t events,
	void *userdata)
{
	StreamDemuxerNet *self = (StreamDemuxerNet *)userdata;
	int res = 0;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct timespec ts = { 0, 0 };

	if (self == NULL) {
		PDRAW_LOGE("StreamDemuxerNet: invalid callback params");
		return;
	}

	do {
		/* Read data */
		readlen = self->mControlSock->read();

		/* Something read? */
		if (readlen > 0) {
			/* TODO: avoid copy */
			buf = pomp_buffer_new_with_data(
				self->mControlSock->getRxBuffer(), readlen);
			res = time_get_monotonic(&ts);
			if (res < 0) {
				PDRAW_LOG_ERRNO("StreamDemuxerMux: "
					"time_get_monotonic", -res);
			}
			res = vstrm_receiver_recv_ctrl(
				self->mReceiver, buf, &ts);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0) {
				PDRAW_LOG_ERRNO("StreamDemuxerMux: "
					"vstrm_receiver_recv_ctrl", -res);
			}
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


int StreamDemuxerNet::sendCtrl(
	struct vstrm_receiver *stream,
	struct pomp_buffer *buf)
{
	const void *cdata = NULL;
	size_t len = 0;
	ssize_t writelen = 0;

	if (buf == NULL) {
		PDRAW_LOGE("StreamDemuxerNet: invalid buffer");
		return -EINVAL;
	}

	/* Write data */
	pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
	writelen = mControlSock->write(cdata, len);
	return (writelen >= 0) ? 0 : (int)writelen;
}


} /* namespace Pdraw */
