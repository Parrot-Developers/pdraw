/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer, mux implementation
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

#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_session.hpp"
#include "pdraw_log.hpp"

#ifdef BUILD_LIBMUX

#include <time.h>
#include <sys/time.h>
#include <futils/futils.h>
#include <libmux-arsdk.h>

namespace Pdraw {


StreamDemuxerMux::StreamDemuxerMux(
	Session *session) :
	StreamDemuxer(session)
{
	mMux = NULL;
}


StreamDemuxerMux::~StreamDemuxerMux(
	void)
{
	if (mMux != NULL) {
		mux_channel_close(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
		mux_channel_close(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
		mux_unref(mMux);
		mMux = NULL;
	}
}


int StreamDemuxerMux::configure(
	const std::string &url,
	struct mux_ctx *mux)
{
	int res;

	if (mConfigured) {
		PDRAW_LOGE("StreamDemuxer: demuxer is already configured");
		return -EEXIST;
	}
	if (mux == NULL) {
		PDRAW_LOGE("StreamDemuxer: invalid mux handle");
		return -EINVAL;
	}
	mMux = mux;
	/* TODO: RTSP over mux */

	res = configureRtpAvp();
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxer: configureRtpAvp", -res);
		return res;
	}

	mConfigured = true;
	PDRAW_LOGI("StreamDemuxer: demuxer is configured");

	return 0;
}


int StreamDemuxerMux::configureWithSdp(
	const std::string &sdp,
	struct mux_ctx *mux)
{
	/* TODO */
	return -ENOSYS;
}


int StreamDemuxerMux::configureRtpAvp(
	void)
{
	int res = 0;

	if (mMux == NULL) {
		PDRAW_LOGE("StreamDemuxerMux: invalid mux handle");
		return -EPROTO;
	}

	mux_ref(mMux);
	/* TODO: RTSP over mux */

	res = mux_channel_open(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
		&dataCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: mux_channel_open", -res);
		goto error;
	}

	res = mux_channel_open(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
		&ctrlCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: mux_channel_open", -res);
		goto error;
	}

	createReceiver();

	return 0;

error:
	destroyReceiver();
	mux_channel_close(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
	mux_channel_close(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
	mux_unref(mMux);
	mMux = NULL;
	return res;
}


void StreamDemuxerMux::dataCb(
	struct mux_ctx *ctx,
	uint32_t chanid,
	enum mux_channel_event event,
	struct pomp_buffer *buf,
	void *userdata)
{
	StreamDemuxerMux *self = (StreamDemuxerMux *)userdata;
	int res = 0;
	struct timespec ts = { 0, 0 };

	if (self == NULL) {
		PDRAW_LOGE("StreamDemuxerMux: invalid callback params");
		return;
	}

	res = time_get_monotonic(&ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: time_get_monotonic",
			-res);
	}

	res = vstrm_receiver_recv_data(self->mReceiver, buf, &ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: vstrm_receiver_recv_data",
			-res);
	}
}


void StreamDemuxerMux::ctrlCb(
	struct mux_ctx *ctx,
	uint32_t chanid,
	enum mux_channel_event event,
	struct pomp_buffer *buf,
	void *userdata)
{
	StreamDemuxerMux *self = (StreamDemuxerMux*)userdata;
	int res = 0;
	struct timespec ts = { 0, 0 };

	if (self == NULL) {
		PDRAW_LOGE("StreamDemuxerMux: invalid callback params");
		return;
	}

	res = time_get_monotonic(&ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: time_get_monotonic",
			-res);
	}

	res = vstrm_receiver_recv_ctrl(self->mReceiver, buf, &ts);
	if (res < 0) {
		PDRAW_LOG_ERRNO("StreamDemuxerMux: vstrm_receiver_recv_ctrl",
			-res);
	}
}


int StreamDemuxerMux::sendCtrl(
	struct vstrm_receiver *stream,
	struct pomp_buffer *buf)
{
	if (buf == NULL) {
		PDRAW_LOGE("StreamDemuxerMux: invalid buffer");
		return -EINVAL;
	}

	return mux_encode(mMux, MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL, buf);
}


} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
