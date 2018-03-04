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

#ifndef _PDRAW_DEMUXER_STREAM_NET_HPP_
#define _PDRAW_DEMUXER_STREAM_NET_HPP_

#include "pdraw_demuxer.hpp"
#include "pdraw_demuxer_stream.hpp"
#include "pdraw_socket_inet.hpp"
#include <string>

namespace Pdraw {


class StreamDemuxerNet : public StreamDemuxer {
public:
	StreamDemuxerNet(
		Session *session);

	~StreamDemuxerNet(
		void);

	int open(
		const std::string &url,
		const std::string &ifaceAddr);

	int open(
		const std::string &localAddr,
		uint16_t localStreamPort,
		uint16_t localControlPort,
		const std::string &remoteAddr,
		uint16_t remoteStreamPort,
		uint16_t remoteControlPort,
		const std::string &ifaceAddr);

	int openSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	int getSingleStreamLocalPorts(
		uint16_t *streamPort,
		uint16_t *controlPort);

private:
	int openRtpAvp(
		void);

	static void dataCb(
		int fd,
		uint32_t events,
		void *userdata);

	static void ctrlCb(
		int fd,
		uint32_t events,
		void *userdata);

	int sendCtrl(
		struct vstrm_receiver *stream,
		struct pomp_buffer *buf);

	InetSocket *mStreamSock;
	InetSocket *mControlSock;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_STREAM_NET_HPP_ */
