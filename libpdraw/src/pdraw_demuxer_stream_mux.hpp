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

#	include <atomic>
#	include <string>

#	include <libmux.h>

#	include <transport-packet/tpkt.h>
#	include <transport-socket/tskt.h>

namespace Pdraw {

class StreamDemuxerMux : public StreamDemuxer {
public:
	StreamDemuxerMux(Session *session,
			 Element::Listener *elementListener,
			 CodedSource::Listener *sourceListener,
			 IPdraw::IDemuxer *demuxer,
			 IPdraw::IDemuxer::Listener *demuxerListener,
			 const std::string &url,
			 struct mux_ctx *mux);

	~StreamDemuxerMux(void);

protected:
	VideoMedia *createVideoMedia(void);

private:
	class VideoMediaMux : StreamDemuxer::VideoMedia {
	public:
		VideoMediaMux(StreamDemuxerMux *demuxer);

		~VideoMediaMux(void);

		int startRtpAvp(void);

		int stopRtpAvp(void);

		int sendCtrl(struct vstrm_receiver *stream,
			     struct tpkt_packet *pkt);

		int prepareSetup(void);

		enum rtsp_lower_transport getLowerTransport(void);

		uint16_t getLocalStreamPort(void);

		uint16_t getLocalControlPort(void);

		uint16_t getRemoteStreamPort(void);

		uint16_t getRemoteControlPort(void);

		void setLocalStreamPort(uint16_t port);

		void setLocalControlPort(uint16_t port);

		void setRemoteStreamPort(uint16_t port);

		void setRemoteControlPort(uint16_t port);

	private:
		static void legacyDataCb(struct mux_ctx *ctx,
					 uint32_t chanid,
					 enum mux_channel_event event,
					 struct pomp_buffer *buf,
					 void *userdata);

		static void legacyCtrlCb(struct mux_ctx *ctx,
					 uint32_t chanid,
					 enum mux_channel_event event,
					 struct pomp_buffer *buf,
					 void *userdata);

		int createSockets(void);

		void closeSockets(void);

		struct tpkt_packet *newRxPkt(void);

		static void dataCb(int fd, uint32_t events, void *userdata);

		static void ctrlCb(int fd, uint32_t events, void *userdata);

		static void callFinishSetup(void *userdata);

		static void proxyOpenCb(struct mux_ip_proxy *proxy,
					uint16_t localPort,
					void *userdata);

		static void proxyCloseCb(struct mux_ip_proxy *proxy,
					 void *userdata);

		static void proxyUpdateCb(struct mux_ip_proxy *proxy,
					  void *userdata);

		static void proxyFailedCb(struct mux_ip_proxy *proxy,
					  int err,
					  void *userdata);

		StreamDemuxerMux *mDemuxerMux;
		struct tskt_socket *mStreamSock;
		struct mux_ip_proxy *mStreamProxy;
		bool mStreamProxyOpened;
		struct tskt_socket *mControlSock;
		struct mux_ip_proxy *mControlProxy;
		bool mControlProxyOpened;
		struct tpkt_packet *mRxPkt;
		size_t mRxBufLen;
	};

	bool setMux(struct mux_ctx *mux);

	struct mux_ctx *mMux;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */

#endif /* !_PDRAW_DEMUXER_STREAM_MUX_HPP_ */
