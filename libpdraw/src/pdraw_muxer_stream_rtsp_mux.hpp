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

#pragma once

#ifdef BUILD_LIBMUX

#	include "pdraw_muxer_stream_rtsp.hpp"
#	include "pdraw_muxer_stream_rtsp_video_media.hpp"

#	include <atomic>

#	include <libmux.h>
#	include <transport-packet/tpkt.h>
#	include <transport-socket/tskt.h>

namespace Pdraw {

/* Libmux-transport variant of the RTSP stream muxer.
 * Uses a SkyController mux tunnel (mux_ip_proxy) to carry RTP/RTCP traffic
 * to an RTSP server running on the SkyController.
 * Mirrors the StreamDemuxerMux implementation pattern. */
class RtspStreamMuxerMux : public RtspStreamMuxer {
public:
	RtspStreamMuxerMux(Session *session,
			   Element::Listener *elementListener,
			   IPdraw::IMuxer::Listener *listener,
			   MuxerWrapper *wrapper,
			   const std::string &url,
			   struct mux_ctx *mux,
			   const std::string &remoteHost,
			   const struct pdraw_muxer_params *params);

	~RtspStreamMuxerMux() override;

	std::unique_ptr<RtspStreamMuxer::VideoMedia>
	createVideoMedia(enum pdraw_muxer_rtsp_transport transport) override;

private:
	class VideoMediaMux : public RtspStreamMuxer::VideoMedia {
	public:
		explicit VideoMediaMux(RtspStreamMuxerMux *muxer);

		~VideoMediaMux() override;

		int startRtpAvp() override;

		int stopRtpAvp() override;

		enum rtsp_lower_transport getLowerTransport() const override
		{
			/* Mux transport always uses UDP loopback sockets */
			return RTSP_LOWER_TRANSPORT_UDP;
		}

		uint16_t getLocalStreamPort() const override;

		uint16_t getLocalControlPort() const override;

		uint16_t getRemoteStreamPort() const override;

		uint16_t getRemoteControlPort() const override;

		void setRemoteStreamPort(uint16_t port) override;

		void setRemoteControlPort(uint16_t port) override;

		const struct rtsp_header_ext *getHeaderExt() const override;

		size_t getHeaderExtCount() const override;

	protected:
		struct tskt_socket *getStreamSocket() const override
		{
			return mStreamSock;
		}

		struct tskt_socket *getControlSocket() const override
		{
			return mControlSock;
		}

		int prepareSetup() override;

	private:
		int createSockets();

		void closeSockets();

		struct tpkt_packet *newRxPkt();

		void setRxPkt(struct tpkt_packet *newPkt);

		static void dataCb(int fd, uint32_t events, void *userdata);

		static void ctrlCb(int fd, uint32_t events, void *userdata);

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

		RtspStreamMuxerMux *mMuxerMux = nullptr;
		struct tskt_socket *mStreamSock = nullptr;
		struct mux_ip_proxy *mStreamProxy = nullptr;
		bool mStreamProxyOpened = false;
		struct tskt_socket *mControlSock = nullptr;
		struct mux_ip_proxy *mControlProxy = nullptr;
		bool mControlProxyOpened = false;
		struct tpkt_packet *mRxPkt = nullptr;
		size_t mRxBufLen = 0;
		pomp::Loop::IdleHandlerFunc mCallFinishSetupHandler;
		static const struct rtsp_header_ext mHeaderExt;
		static const size_t mHeaderExtCount;
	};

	/* TCP-interleaved variant: RTP flows over the RTSP TCP socket directly,
	 * bypassing mux_ip_proxy entirely. Used when transport == TCP. */
	class VideoMediaMuxTcp : public RtspStreamMuxer::VideoMedia {
	public:
		explicit VideoMediaMuxTcp(RtspStreamMuxerMux *muxer);

		~VideoMediaMuxTcp() override = default;

		int startRtpAvp() override;

		enum rtsp_lower_transport getLowerTransport() const override
		{
			return RTSP_LOWER_TRANSPORT_TCP;
		}

		uint16_t getLocalStreamPort() const override
		{
			return 0;
		}
		uint16_t getLocalControlPort() const override
		{
			return 0;
		}
		uint16_t getRemoteStreamPort() const override
		{
			return mRemoteStreamPort;
		}
		uint16_t getRemoteControlPort() const override
		{
			return mRemoteControlPort;
		}

		void setRemoteStreamPort(uint16_t port) override
		{
			mRemoteStreamPort = port;
		}
		void setRemoteControlPort(uint16_t port) override
		{
			mRemoteControlPort = port;
		}

	protected:
		int prepareSetup() override
		{
			return 0;
		}

	private:
		uint16_t mRemoteStreamPort = 0;
		uint16_t mRemoteControlPort = 0;
	};

	bool setMux(struct mux_ctx *mux);

	struct mux_ctx *mMux = nullptr;

	/* Real destination host (hostname or IP), as opposed to mUrl which,
	 * for the mux transport, holds the loopback URL used to reach the
	 * TCP control-channel tunnel. Used as the remote_host for the UDP
	 * RTP/RTCP mux_ip_proxy instances. */
	std::string mRemoteHost;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBMUX */
