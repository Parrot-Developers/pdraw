/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

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
			 Source::Listener *sourceListener,
			 DemuxerWrapper *wrapper,
			 IPdraw::IDemuxer::Listener *demuxerListener,
			 const std::string &url,
			 struct mux_ctx *mux,
			 const struct pdraw_demuxer_params *params);

	~StreamDemuxerMux() override;

protected:
	std::unique_ptr<VideoMedia>
	createVideoMedia(enum rtsp_lower_transport transport) override;

private:
	class VideoMediaMux : public StreamDemuxer::VideoMedia {
	public:
		explicit VideoMediaMux(StreamDemuxerMux *demuxer);

		~VideoMediaMux() override;

		int startRtpAvp() override;

		int stopRtpAvp() override;

		int sendCtrl(struct vstrm_receiver *stream,
			     struct tpkt_packet *pkt) override;

		int prepareSetup() override;

		enum rtsp_lower_transport getLowerTransport() const override;

		uint16_t getLocalStreamPort() const override;

		uint16_t getLocalControlPort() const override;

		uint16_t getRemoteStreamPort() const override;

		uint16_t getRemoteControlPort() const override;

		const struct rtsp_header_ext *getHeaderExt() const override;

		size_t getHeaderExtCount() const override;

		void setLocalStreamPort(uint16_t port) override;

		void setLocalControlPort(uint16_t port) override;

		void setRemoteStreamPort(uint16_t port) override;

		void setRemoteControlPort(uint16_t port) override;

		int processDataPkt(struct tpkt_packet *pkt) override;

		int processCtrlPkt(struct tpkt_packet *pkt) override;

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

		int createSockets();

		void closeSockets();

		struct tpkt_packet *newRxPkt();

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

		StreamDemuxerMux *mDemuxerMux = nullptr;
		struct tskt_socket *mStreamSock = nullptr;
		struct mux_ip_proxy *mStreamProxy = nullptr;
		bool mStreamProxyOpened = false;
		struct tskt_socket *mControlSock = nullptr;
		struct mux_ip_proxy *mControlProxy = nullptr;
		bool mControlProxyOpened = false;
		struct tpkt_packet *mRxPkt = nullptr;
		size_t mRxBufLen = 0;
		static const struct rtsp_header_ext mHeaderExt;
		static const size_t mHeaderExtCount;
	};

	bool setMux(struct mux_ctx *mux);

	struct mux_ctx *mMux = nullptr;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
