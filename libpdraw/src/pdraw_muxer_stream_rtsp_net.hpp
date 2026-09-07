/**
 * Parrot Drones Audio and Video Vector library
 * RTSP stream muxer - network (UDP/TCP) implementation
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

#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_video_media.hpp"

namespace Pdraw {

/* Network-transport variant of the RTSP stream muxer.
 * Uses real UDP or TCP-interleaved sockets (tskt_socket / rtsp_client
 * interleaved channel) to carry RTP/RTCP traffic.
 * This class is iso-functional to the original monolithic RtspStreamMuxer. */
class RtspStreamMuxerNet : public RtspStreamMuxer {
public:
	RtspStreamMuxerNet(Session *session,
			   Element::Listener *elementListener,
			   IPdraw::IMuxer::Listener *listener,
			   MuxerWrapper *wrapper,
			   const std::string &url,
			   const struct pdraw_muxer_params *params);

	~RtspStreamMuxerNet() override = default;

	std::unique_ptr<RtspStreamMuxer::VideoMedia>
	createVideoMedia(enum pdraw_muxer_rtsp_transport transport) override;

private:
	class VideoMediaNet : public RtspStreamMuxer::VideoMedia {
	public:
		VideoMediaNet(RtspStreamMuxerNet *muxer,
			      enum pdraw_muxer_rtsp_transport transport);

		~VideoMediaNet() override;

		int startRtpAvp() override;

		int stopRtpAvp() override;

		enum rtsp_lower_transport getLowerTransport() const override
		{
			return mLowerTransport;
		}

		uint16_t getLocalStreamPort() const override
		{
			return mStrm.localPort;
		}

		uint16_t getLocalControlPort() const override
		{
			return mCtrl.localPort;
		}

		uint16_t getRemoteStreamPort() const override
		{
			return mStrm.remotePort;
		}

		uint16_t getRemoteControlPort() const override
		{
			return mCtrl.remotePort;
		}

		void setRemoteStreamPort(uint16_t port) override;

		void setRemoteControlPort(uint16_t port) override;

	protected:
		struct tskt_socket *getStreamSocket() const override
		{
			return mStrm.sock;
		}

		struct tskt_socket *getControlSocket() const override
		{
			return mCtrl.sock;
		}

		int prepareSetup() override;

	private:
		int createSockets();

		struct tpkt_packet *newRxPkt();

		void setRxPkt(struct tpkt_packet *newPkt);

		static void dataCb(int fd, uint32_t events, void *userdata);

		static void ctrlCb(int fd, uint32_t events, void *userdata);

		struct SocketPair {
			struct tskt_socket *sock = nullptr;
			uint16_t localPort = 0;
			uint16_t remotePort = 0;
		};

		/* Concrete-typed pointer so we can access protected members of
		 * RtspStreamMuxerNet (e.g. mSession) through the derived class,
		 * which satisfies the C++ §11.5 protected access rule. */
		RtspStreamMuxerNet *mMuxerNet = nullptr;
		const enum rtsp_lower_transport mLowerTransport =
			RTSP_LOWER_TRANSPORT_UDP;
		SocketPair mStrm{};
		SocketPair mCtrl{};
		struct tpkt_packet *mRxPkt = nullptr;
		size_t mRxBufLen = 0;
	};
};

} /* namespace Pdraw */
