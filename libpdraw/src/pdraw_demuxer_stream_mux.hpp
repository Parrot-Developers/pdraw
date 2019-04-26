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

#ifndef _PDRAW_DEMUXER_STREAM_MUX_HPP_
#define _PDRAW_DEMUXER_STREAM_MUX_HPP_

#ifdef BUILD_LIBMUX

#	include "pdraw_demuxer_stream.hpp"

#	include <string>

#	include <libmux.h>

namespace Pdraw {

class StreamDemuxerMux : public StreamDemuxer {
public:
	StreamDemuxerMux(Session *session,
			 Element::Listener *elementListener,
			 Source::Listener *sourceListener,
			 Demuxer::Listener *demuxerListener);

	~StreamDemuxerMux(void);

	int setup(const std::string &url, struct mux_ctx *mux);

private:
	int startRtpAvp(void);

	int stopRtpAvp(void);

	static void dataCb(struct mux_ctx *ctx,
			   uint32_t chanid,
			   enum mux_channel_event event,
			   struct pomp_buffer *buf,
			   void *userdata);

	static void ctrlCb(struct mux_ctx *ctx,
			   uint32_t chanid,
			   enum mux_channel_event event,
			   struct pomp_buffer *buf,
			   void *userdata);

	static void rtpCb(struct mux_ctx *ctx,
			  uint32_t chanid,
			  enum mux_channel_event event,
			  struct pomp_buffer *buf,
			  void *userdata);

	int sendCtrl(struct vstrm_receiver *stream, struct pomp_buffer *buf);

	int prepareSetup(uint16_t *streamPort,
			 uint16_t *controlPort,
			 enum rtsp_lower_transport *lowerTransport);

	bool setMux(struct mux_ctx *mux);

	struct mux_ctx *mMux;
	uint16_t mMuxPort;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */

#endif /* !_PDRAW_DEMUXER_STREAM_MUX_HPP_ */
