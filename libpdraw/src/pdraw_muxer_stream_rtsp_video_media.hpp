/**
 * Parrot Drones Audio and Video Vector library
 * RTSP stream muxer
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

/* Abstract base class for RTSP stream muxer video media.
 * Transport-specific behaviour (UDP sockets vs. libmux proxy) is provided
 * by the concrete subclasses VideoMediaNet and VideoMediaMux, defined in
 * pdraw_muxer_stream_rtsp_net.hpp and pdraw_muxer_stream_rtsp_mux.hpp. */
class RtspStreamMuxer::VideoMedia : public Loggable {
	PDRAW_DISABLE_COPY(VideoMedia)

public:
	explicit VideoMedia(RtspStreamMuxer *muxer);

	~VideoMedia() override;

	bool hasMedia(const Media *media) const
	{
		return media == mVideoMedia;
	}

	void clearMedia()
	{
		mVideoMedia = nullptr;
	}

	/* Pure virtual transport interface (implemented by Net / Mux) */

	virtual int startRtpAvp() = 0;

	virtual int stopRtpAvp();

	virtual enum rtsp_lower_transport getLowerTransport() const = 0;

	virtual uint16_t getLocalStreamPort() const = 0;

	virtual uint16_t getLocalControlPort() const = 0;

	virtual uint16_t getRemoteStreamPort() const = 0;

	virtual uint16_t getRemoteControlPort() const = 0;

	virtual void setRemoteStreamPort(uint16_t port) = 0;

	virtual void setRemoteControlPort(uint16_t port) = 0;

	/* Virtual with default (overridden by Mux to provide the
	 * parrot-link-type: mux header extension) */
	virtual const struct rtsp_header_ext *getHeaderExt() const
	{
		return nullptr;
	}

	virtual size_t getHeaderExtCount() const
	{
		return 0;
	}

	int setup(const std::string &controlUrl, Media *media);

	int process();

	int processFrame(struct mbuf_coded_video_frame *in_frame);

	void flush(bool discard = true) const;

	inline void drain() const
	{
		return flush(false);
	}

	void stop();

	int teardown();

	void setTearingDown();

	void setSsrc(uint32_t ssrc)
	{
		mSsrc = ssrc;
	}

	bool isTearingDown() const
	{
		return mTearingDown;
	}

	const std::string &getControlUrl() const
	{
		return mControlUrl;
	}

	const char *getCControlUrl() const
	{
		return mControlUrl.c_str();
	}

	const struct VideoMediaStats &getStats() const
	{
		return mStats;
	}

	int notifyReadyToSend();

	int processDataPkt(struct tpkt_packet *pkt) const;

	int processCtrlPkt(struct tpkt_packet *pkt);

	/* Send RESYNC upstream to force an IDR from the encoder.
	 * Call when mRecording becomes true so the first frame Wowza sees is
	 * always an IDR (the encoder's original IDR may have been produced
	 * before mRecording was set, leaving only P-frames in the queue). */
	void requestResync();

protected:
	/* Transport socket accessors - override to expose the actual
	 * tskt_socket* used by this transport variant.  processList() and the
	 * sendCtrlCb vstrm_sender callback call these instead of accessing
	 * socket members directly. The nullptr default is relied upon by
	 * VideoMediaMuxTcp (TCP-interleaved: RTP flows over the RTSP TCP
	 * socket, no raw tskt_socket of its own); it is never dereferenced
	 * for that variant since sendPkt()/processList() both check
	 * getLowerTransport() == RTSP_LOWER_TRANSPORT_TCP before touching the
	 * socket. Keep as virtual-with-default, not pure virtual. */
	virtual struct tskt_socket *getStreamSocket() const
	{
		return nullptr;
	}

	virtual struct tskt_socket *getControlSocket() const
	{
		return nullptr;
	}

	/* Called from setup(); must be overridden by each transport variant.
	 * May return -EINPROGRESS for async setup (e.g. Mux proxy open). */
	virtual int prepareSetup() = 0;

	/* Common helpers available to subclasses */
	int createSender();

	int destroySender();

	void finishSetup();

	void finishTeardown();

	int processList(struct tpkt_list *newList);

	int sendPkt(struct tpkt_packet *pkt,
		    uint16_t channel,
		    struct tskt_socket *sock,
		    const char *logTag);

	int updateStats(const struct rtcp_pkt_receiver_report *rr = nullptr,
			uint32_t rtd = UINT32_MAX);

private:
	/* vstrm_sender callback - registered once in createSender().
	 * sendCtrlCb reaches the concrete socket through the virtual
	 * getControlSocket(). Data (RTP) packets are no longer sent through a
	 * vstrm_sender callback: vstrm_sender_send_frame() returns them in a
	 * tpkt_list that processList() sends directly (see processFrame() /
	 * notifyReadyToSend()). */
	static int sendCtrlCb(struct vstrm_sender *stream,
			      struct tpkt_packet *pkt,
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

protected:
	RtspStreamMuxer *mMuxer = nullptr;

private:
	struct vstrm_sender *mSender = nullptr;
	std::string mControlUrl{};
	bool mTearingDown = false;
	uint32_t mSsrc = 0;
	struct sdp_media *mSdpMedia = nullptr;
	Media *mVideoMedia = nullptr;
	bool mPendingTearDown = false;
	bool mSynchronized = false;
	bool mNetdownLogged = false;
	struct tpkt_list *mList = nullptr;
	struct VideoMediaStats mStats {
	};
	static const struct vstrm_sender_cbs mSenderCbs;
};

} /* namespace Pdraw */
