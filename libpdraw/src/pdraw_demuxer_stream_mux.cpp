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

#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_session.hpp"

#define ULOG_TAG pdraw_dmxstrmmux
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_dmxstrmmux);

#ifdef BUILD_LIBMUX

#	include <sys/time.h>
#	include <time.h>

#	include <futils/futils.h>
#	include <libmux-arsdk.h>

namespace Pdraw {


StreamDemuxerMux::StreamDemuxerMux(Session *session,
				   Element::Listener *elementListener,
				   Source::Listener *sourceListener,
				   Demuxer::Listener *demuxerListener) :
		StreamDemuxer(session,
			      elementListener,
			      sourceListener,
			      demuxerListener)
{
	Element::mName = "StreamDemuxerMux";
	Source::mName = "StreamDemuxerMux";
	mMux = NULL;
	mMuxPort = 0;

	setState(CREATED);
}


StreamDemuxerMux::~StreamDemuxerMux(void)
{
	if (mState != STOPPED && mState != CREATED)
		ULOGW("demuxer is still running");

	stopRtpAvp();

	setMux(NULL);
}


int StreamDemuxerMux::setup(const std::string &url, struct mux_ctx *mux)
{
	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	if (!setMux(mux)) {
		ULOGE("invalid mux handle");
		return -EINVAL;
	}

	mUrl = url;
	return 0;
}


int StreamDemuxerMux::startRtpAvp(void)
{
	int res;

	ULOGD("startRtpAvp");

	if (mMux == NULL) {
		ULOGE("invalid mux handle");
		return -EPROTO;
	}

	if (mSessionProtocol == RTSP) {
		res = mux_channel_open(
			mMux, MUX_ARSDK_CHANNEL_ID_RTP, &rtpCb, this);
		if (res < 0) {
			ULOG_ERRNO("mux_channel_open", -res);
			goto error;
		}
	} else {
		res = mux_channel_open(
			mMux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA, &dataCb, this);
		if (res < 0) {
			ULOG_ERRNO("mux_channel_open", -res);
			goto error;
		}

		res = mux_channel_open(mMux,
				       MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
				       &ctrlCb,
				       this);
		if (res < 0) {
			ULOG_ERRNO("mux_channel_open", -res);
			goto error;
		}
	}

	createReceiver();

	return 0;

error:
	stopRtpAvp();

	return res;
}


int StreamDemuxerMux::stopRtpAvp(void)
{
	ULOGD("stopRtpAvp");
	destroyReceiver();
	if (mMux != NULL) {
		if (mSessionProtocol == RTSP) {
			mux_channel_close(mMux, MUX_ARSDK_CHANNEL_ID_RTP);
		} else {
			mux_channel_close(mMux,
					  MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
			mux_channel_close(mMux,
					  MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
		}
	}
	mMuxPort = 0;
	return 0;
}


void StreamDemuxerMux::dataCb(struct mux_ctx *ctx,
			      uint32_t chanid,
			      enum mux_channel_event event,
			      struct pomp_buffer *buf,
			      void *userdata)
{
	StreamDemuxerMux *self = (StreamDemuxerMux *)userdata;
	int res = 0;
	struct timespec ts = {0, 0};

	if (self == NULL)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);

	res = vstrm_receiver_recv_data(self->mReceiver, buf, &ts);
	if (res < 0)
		ULOG_ERRNO("vstrm_receiver_recv_data", -res);
}


void StreamDemuxerMux::ctrlCb(struct mux_ctx *ctx,
			      uint32_t chanid,
			      enum mux_channel_event event,
			      struct pomp_buffer *buf,
			      void *userdata)
{
	StreamDemuxerMux *self = (StreamDemuxerMux *)userdata;
	int res = 0;
	struct timespec ts = {0, 0};

	if (self == NULL)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);

	res = vstrm_receiver_recv_ctrl(self->mReceiver, buf, &ts);
	if (res < 0)
		ULOG_ERRNO("vstrm_receiver_recv_ctrl", -res);
}


void StreamDemuxerMux::rtpCb(struct mux_ctx *ctx,
			     uint32_t chanid,
			     enum mux_channel_event event,
			     struct pomp_buffer *buf,
			     void *userdata)
{
	StreamDemuxerMux *self = (StreamDemuxerMux *)userdata;
	int res = 0;
	struct timespec ts = {0, 0};
	uint32_t msgId;
	uint16_t port;
	void *data;
	unsigned int dataLen;
	struct pomp_msg *msg = NULL;
	struct pomp_buffer *dataBuf = NULL;

	if (self == NULL)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);

	msg = pomp_msg_new_with_buffer(buf);
	if (msg == NULL) {
		ULOG_ERRNO("pomp_msg_new_with_buffer", ENOMEM);
		return;
	}

	msgId = pomp_msg_get_id(msg);

	switch (msgId) {
	case MUX_ARSDK_MSG_ID_RTP_DATA:
		res = pomp_msg_read(msg,
				    MUX_ARSDK_MSG_FMT_DEC_RTP_DATA,
				    &port,
				    &data,
				    &dataLen);
		break;
	case MUX_ARSDK_MSG_ID_RTCP_DATA:
		res = pomp_msg_read(msg,
				    MUX_ARSDK_MSG_FMT_DEC_RTCP_DATA,
				    &port,
				    &data,
				    &dataLen);
		break;
	default:
		ULOGE("Bad message id: %u", msgId);
		goto exit;
	}

	if (res < 0) {
		ULOG_ERRNO("pomp_msg_read", -res);
		goto exit;
	}

	if (port != self->mMuxPort) {
		ULOGD("Got a RTP/RTCP message for the wrong port");
		goto exit;
	}

	dataBuf = pomp_buffer_new_with_data(data, dataLen);
	if (dataBuf == NULL) {
		ULOG_ERRNO("pomp_buffer_new_with_data", ENOMEM);
		goto exit;
	}

	switch (msgId) {
	case MUX_ARSDK_MSG_ID_RTP_DATA:
		res = vstrm_receiver_recv_data(self->mReceiver, dataBuf, &ts);
		if (res < 0)
			ULOG_ERRNO("vstrm_receiver_recv_data", -res);
		break;
	case MUX_ARSDK_MSG_ID_RTCP_DATA:
		res = vstrm_receiver_recv_ctrl(self->mReceiver, dataBuf, &ts);
		if (res < 0)
			ULOG_ERRNO("vstrm_receiver_recv_ctrl", -res);
		break;
	}

exit:
	if (msg != NULL)
		pomp_msg_destroy(msg);
	if (dataBuf != NULL)
		pomp_buffer_unref(dataBuf);
}


int StreamDemuxerMux::sendCtrl(struct vstrm_receiver *stream,
			       struct pomp_buffer *buf)
{
	void *data;
	size_t len;
	struct pomp_msg *msg = NULL;
	struct pomp_buffer *msgbuf = NULL;
	int ret;

	if (buf == NULL) {
		ULOGE("invalid buffer");
		return -EINVAL;
	}

	if (mSessionProtocol != RTSP)
		return mux_encode(
			mMux, MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL, buf);

	/* For rtsp, create a packet for the current id */
	ret = pomp_buffer_get_data(buf, &data, &len, NULL);
	if (ret < 0)
		goto exit;

	msg = pomp_msg_new();
	if (!msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = pomp_msg_write(msg,
			     MUX_ARSDK_MSG_ID_RTCP_DATA,
			     MUX_ARSDK_MSG_FMT_ENC_RTCP_DATA,
			     mMuxPort,
			     data,
			     (unsigned int)len);
	if (ret < 0)
		goto exit;

	msgbuf = pomp_msg_get_buffer(msg);
	if (!msgbuf) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = mux_encode(mMux, MUX_ARSDK_CHANNEL_ID_RTP, msgbuf);
exit:
	if (msg)
		pomp_msg_destroy(msg);
	return ret;
}


int StreamDemuxerMux::prepareSetup(uint16_t *streamPort,
				   uint16_t *controlPort,
				   enum rtsp_lower_transport *lowerTransport)
{
	if (streamPort == NULL || controlPort == NULL || lowerTransport == NULL)
		return -EINVAL;

	mMuxPort = 1; /* TODO: Dynamic */
	*streamPort = mMuxPort;
	*controlPort = mMuxPort;
	*lowerTransport = RTSP_LOWER_TRANSPORT_MUX;

	return 0;
}


bool StreamDemuxerMux::setMux(struct mux_ctx *mux)
{
	if (mMux != NULL)
		mux_unref(mMux);

	mMux = mux;
	if (mMux != NULL) {
		mux_ref(mMux);
		return true;
	}
	return false;
}

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
