/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_demuxer_stream.hpp"

#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>

#include <string>

namespace Pdraw {

class StreamDemuxerNet : public StreamDemuxer {
public:
	StreamDemuxerNet(Session *session,
			 Element::Listener *elementListener,
			 Source::Listener *sourceListener,
			 DemuxerWrapper *wrapper,
			 IPdraw::IDemuxer::Listener *demuxerListener,
			 const std::string &url,
			 const struct pdraw_demuxer_params *params);

	StreamDemuxerNet(Session *session,
			 Element::Listener *elementListener,
			 Source::Listener *sourceListener,
			 DemuxerWrapper *wrapper,
			 IPdraw::IDemuxer::Listener *demuxerListener,
			 const std::string &localAddr,
			 uint16_t localStreamPort,
			 uint16_t localControlPort,
			 const std::string &remoteAddr,
			 uint16_t remoteStreamPort,
			 uint16_t remoteControlPort,
			 const struct pdraw_demuxer_params *params);

	~StreamDemuxerNet() override;

	uint16_t getSingleStreamLocalStreamPort();

	uint16_t getSingleStreamLocalControlPort();

protected:
	std::unique_ptr<VideoMedia>
	createVideoMedia(enum rtsp_lower_transport transport) override;

private:
	class VideoMediaNet : public StreamDemuxer::VideoMedia {
	public:
		explicit VideoMediaNet(StreamDemuxerNet *demuxer,
				       enum rtsp_lower_transport transport);

		~VideoMediaNet() override;

		int startRtpAvp() override;

		int stopRtpAvp() final;

		int sendCtrl(struct vstrm_receiver *stream,
			     struct tpkt_packet *pkt) override;

		int prepareSetup() override;

		enum rtsp_lower_transport getLowerTransport() const override;

		uint16_t getLocalStreamPort() const override;

		uint16_t getLocalControlPort() const override;

		uint16_t getRemoteStreamPort() const override;

		uint16_t getRemoteControlPort() const override;

		void setLocalStreamPort(uint16_t port) override;

		void setLocalControlPort(uint16_t port) override;

		void setRemoteStreamPort(uint16_t port) override;

		void setRemoteControlPort(uint16_t port) override;

		int processDataPkt(struct tpkt_packet *pkt) override;

		int processCtrlPkt(struct tpkt_packet *pkt) override;

	private:
		void initStreamPorts();

		int createSockets();

		struct tpkt_packet *newRxPkt();

		static void dataCb(int fd, uint32_t events, void *userdata);

		static void ctrlCb(int fd, uint32_t events, void *userdata);

		StreamDemuxerNet *mDemuxerNet = nullptr;
		enum rtsp_lower_transport mLowerTransport =
			RTSP_LOWER_TRANSPORT_UDP;
		struct tskt_socket *mStreamSock = nullptr;
		struct tskt_socket *mControlSock = nullptr;
		struct tpkt_packet *mRxPkt = nullptr;
		size_t mRxBufLen = 0;
	};

	uint16_t mSingleLocalStreamPort = 0;
	uint16_t mSingleLocalControlPort = 0;
	uint16_t mSingleRemoteStreamPort = 0;
	uint16_t mSingleRemoteControlPort = 0;
};

} /* namespace Pdraw */
