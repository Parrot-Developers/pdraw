/**
 * Parrot Drones Audio and Video Vector library
 * RTMP stream muxer
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

#include "pdraw_muxer.hpp"

#include <vector>

#include <libsdp.h>
#include <rtsp/client.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>
#include <video-streaming/vstrm.h>

#include "pdraw_muxer_stream_rtsp.hpp"

namespace Pdraw {

class RtspStreamMuxer::VideoMedia : public Loggable {
public:
	VideoMedia(RtspStreamMuxer *muxer,
		   enum pdraw_muxer_rtsp_transport transport);

	~VideoMedia() override;

	bool hasMedia(Media *media) const
	{
		return media == mVideoMedia;
	}

	void clearMedia()
	{
		mVideoMedia = nullptr;
	}

	int startRtpAvp();

	int stopRtpAvp();

	int setup(const std::string &controlUrl, Media *media);

	int process();

	int processFrame(struct mbuf_coded_video_frame *in_frame);

	void flush(bool discard = true);

	inline void drain()
	{
		return flush(false);
	}

	void stop();

	int teardown();

	void setTearingDown();

	void setRemoteStreamPort(uint16_t port);

	void setRemoteControlPort(uint16_t port);

	void setSsrc(uint32_t ssrc)
	{
		mSsrc = ssrc;
	}

	bool isTearingDown() const
	{
		return mTearingDown;
	}

	enum rtsp_lower_transport getLowerTransport() const
	{
		return mLowerTransport;
	}

	const std::string &getControlUrl() const
	{
		return mControlUrl;
	}

	const char *getCControlUrl() const
	{
		return mControlUrl.c_str();
	}

	uint16_t getLocalStreamPort() const
	{
		return mStrm.localPort;
	}

	uint16_t getLocalControlPort() const
	{
		return mCtrl.localPort;
	}

	uint16_t getRemoteStreamPort() const
	{
		return mStrm.remotePort;
	}

	uint16_t getRemoteControlPort() const
	{
		return mCtrl.remotePort;
	}

	const struct rtsp_header_ext *getHeaderExt() const
	{
		return nullptr;
	}

	size_t getHeaderExtCount() const
	{
		return 0;
	}

	const struct VideoMediaStats &getStats() const
	{
		return mStats;
	}

	int notifyReadyToSend();

	int processDataPkt(struct tpkt_packet *pkt);

	int processCtrlPkt(struct tpkt_packet *pkt);

private:
	int createSender();

	int destroySender();

	int createSockets();

	struct tpkt_packet *newRxPkt();

	void setRxPkt(struct tpkt_packet *newPkt);

	int prepareSetup();

	void finishSetup();

	void finishTeardown();

	int sendPkt(struct tpkt_packet *pkt,
		    uint16_t channel,
		    struct tskt_socket *sock,
		    const char *logTag);

	int updateStats(const struct rtcp_pkt_receiver_report *rr = nullptr,
			uint32_t rtd = UINT32_MAX);

	/* tskt cbs */
	static void dataCb(int fd, uint32_t events, void *userdata);

	static void ctrlCb(int fd, uint32_t events, void *userdata);

	/* vstrm_sender cbs */
	static int sendDataCb(struct vstrm_sender *stream,
			      struct tpkt_packet *pkt,
			      bool marker,
			      void *userdata);

	static int sendCtrlCb(struct vstrm_sender *stream,
			      struct tpkt_packet *pkt,
			      void *userdata);

	static int monitorSendDataReadyCb(struct vstrm_sender *stream,
					  int enable,
					  void *userdata);

	static void
	videoStatsCb(struct vstrm_sender *stream,
		     const struct vstrm_video_stats *video_stats,
		     const struct vstrm_video_stats_dyn *video_stats_dyn,
		     void *userdata);

	static void receiverReportCb(struct vstrm_sender *stream,
				     const struct rtcp_pkt_receiver_report *rr,
				     uint32_t rtd,
				     void *userdata);

	static void goodbyeCb(struct vstrm_sender *stream,
			      const char *reason,
			      void *userdata);

	static void rtpFrameDispose(struct vstrm_frame *vframe);

	struct SocketPair {
		struct tskt_socket *sock;
		uint16_t localPort;
		uint16_t remotePort;
	};

	RtspStreamMuxer *mMuxer = nullptr;
	const enum rtsp_lower_transport mLowerTransport =
		RTSP_LOWER_TRANSPORT_UDP;
	struct vstrm_sender *mSender = nullptr;
	std::string mControlUrl{};
	bool mTearingDown = false;
	uint32_t mSsrc = 0;
	struct sdp_media *mSdpMedia = nullptr;
	Media *mVideoMedia = nullptr;
	SocketPair mStrm{};
	SocketPair mCtrl{};
	bool mPendingTearDown = false;
	bool mSynchronized = false;
	struct tpkt_packet *mRxPkt = nullptr;
	size_t mRxBufLen = 0;
	struct VideoMediaStats mStats {
	};
	static const struct vstrm_sender_cbs mSenderCbs;
};

} /* namespace Pdraw */
