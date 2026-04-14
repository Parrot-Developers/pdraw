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
#include <rtp/rtp.h>
#include <rtsp/client.h>
#include <rtsp/rtsp_url.hpp>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>
#include <video-streaming/vstrm.h>

namespace Pdraw {

class RtspStreamMuxer : public Muxer {
public:
	RtspStreamMuxer(Session *session,
			Element::Listener *elementListener,
			IPdraw::IMuxer::Listener *listener,
			MuxerWrapper *wrapper,
			const std::string &url,
			const struct pdraw_muxer_params *params);

	~RtspStreamMuxer() override;

	int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	};

	int removeInputMedia(Media *media) override;

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

private:
	class VideoMedia;

	struct VideoMediaStats {
		uint32_t receiverReportCount;
		uint32_t pktCountTotal;
		uint32_t pktCountDropped;
		uint32_t pktCountLost;
		float lostFraction;
		uint32_t jitter;
		uint32_t rtd;
	};

	struct SetupRequest {
		VideoMedia *media;
		std::string controlUrl;
		enum rtsp_lower_transport lowerTransport;
		uint16_t localStreamPort;
		uint16_t localControlPort;
		const struct rtsp_header_ext *headerExt;
		size_t headerExtCount;
	};

	struct TeardownRequest {
		VideoMedia *media;
		std::string controlUrl;
	};

	enum class RtspState {
		DISCONNECTED = 0,
		CONNECTED,
		OPTIONS_DONE,
		ANNOUNCE_DONE,
		SETUP_DONE,
	};

	int record();

	int internalStart() override;

	int internalStop() override;

	int sendOptions();

	int sendAnnounce();

	int process() override;

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	void asyncRtspDisconnect();

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void setRtspState(RtspStreamMuxer::RtspState state);

	int processRtspRequests();

	void cleanupRtspRequests();

	int processSetupRequest();

	int processTeardownRequest();

	void asyncCompleteTeardown();

	void tryCompleteStop();

	void completeTeardown();

	void destroyAllVideoMedias();

	static void idleRtspDisconnect(void *userdata);

	static void idleCompleteTeardown(void *userdata);

	void teardownVideoMedia(RtspStreamMuxer::VideoMedia *media);

	void teardownAllVideoMedias();

	void
	notifyVideoMediaStatsUpdate(const RtspStreamMuxer::VideoMedia *media);

	/* Helpers */
	bool checkSessionId(const char *sessionId, const char *op);

	int checkReqStatus(int status,
			   enum rtsp_client_req_status req_status,
			   const char *op);

	static const char *getRtspStateStr(RtspStreamMuxer::RtspState val);

	static enum pdraw_muxer_connection_state
	rtspStateToMuxerConnectionState(RtspStreamMuxer::RtspState val);

	/* rtsp_client callbacks */
	static void onRtspSocketCreated(int fd, void *userdata);

	static void onReadyToSendCb(struct rtsp_client *client, void *userdata);

	static void onRtspInterleavedDataCb(struct rtsp_client *client,
					    uint8_t channel,
					    const uint8_t *data,
					    size_t len,
					    void *userdata);

	static void onRtspConnectionState(struct rtsp_client *client,
					  enum rtsp_client_conn_state state,
					  void *userdata);

	static void onRtspSessionRemoved(struct rtsp_client *client,
					 const char *session_id,
					 int status,
					 void *userdata);

	static void onRtspOptionsResp(struct rtsp_client *client,
				      enum rtsp_client_req_status req_status,
				      int status,
				      uint32_t methods,
				      const struct rtsp_header_ext *ext,
				      size_t ext_count,
				      void *userdata,
				      void *req_userdata);

	static void onRtspAnnounceResp(struct rtsp_client *client,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       void *userdata,
				       void *req_userdata);

	static void onRtspSetupResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    uint16_t server_stream_port,
				    uint16_t server_control_port,
				    int ssrc_valid,
				    uint32_t ssrc,
				    const struct rtsp_header_ext *ext,
				    size_t ext_count,
				    void *userdata,
				    void *req_userdata);

	static void onRtspRecordResp(struct rtsp_client *client,
				     const char *session_id,
				     enum rtsp_client_req_status req_status,
				     int status,
				     const struct rtsp_header_ext *ext,
				     size_t ext_count,
				     void *userdata,
				     void *req_userdata);

	static void onRtspTeardownResp(struct rtsp_client *client,
				       const char *session_id,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       void *userdata,
				       void *req_userdata);

	static void onRtspAnnounce(struct rtsp_client *client,
				   const char *content_base,
				   const struct rtsp_header_ext *ext,
				   size_t ext_count,
				   const char *sdp,
				   void *userdata);

	static void onRtspForcedTeardown(struct rtsp_client *client,
					 const char *path,
					 const char *session_id,
					 const struct rtsp_header_ext *ext,
					 size_t ext_count,
					 void *userdata);

	/* Logging functions */
	void logEvent(const char *eventName, int res) const;

	void logEventSession(const char *eventName,
			     int res,
			     const char *sessionId) const;

	void logEventMedia(const char *eventName,
			   int res,
			   const char *sessionId,
			   RtspStreamMuxer::VideoMedia *media,
			   bool pathIsContentBase = false) const;

	void logEventSetupResp(const char *eventName,
			       int res,
			       const char *sessionId,
			       RtspStreamMuxer::VideoMedia *media,
			       uint16_t _srcStrmPort,
			       uint16_t _srcCtrlPort,
			       bool success) const;

	static enum pdraw_muxer_rtsp_transport
	rtspLowerTransportToPdrawMuxerRtspTransport(
		enum rtsp_lower_transport rtspTransport);

	static enum rtsp_lower_transport
	pdrawMuxerRtspTransportToRtspLowerTransport(
		enum pdraw_muxer_rtsp_transport muxerTransport);

	std::unique_ptr<RtspUrl> mUrl;
	struct rtsp_client *mRtspClient = nullptr;
	size_t mSocketTxBufferSize = 0;
	RtspState mRtspState = RtspState::DISCONNECTED;
	enum rtsp_client_conn_state mRtspConnectionState =
		RTSP_CLIENT_CONN_STATE_DISCONNECTED;
	std::string mRtspSessionId{};
	std::vector<std::unique_ptr<RtspStreamMuxer::VideoMedia>> mVideoMedias;
	std::queue<RtspStreamMuxer::SetupRequest> mSetupRequests{};
	unsigned int mSetupRequestsCount = 0;
	std::queue<RtspStreamMuxer::TeardownRequest> mTeardownRequests{};
	unsigned int mTeardownRequestsCount = 0;
	bool mChannelsReadyForStop = false;
	bool mNetworkReadyForStop = false;
	bool mAnnounceRetried = false;
	bool mRecording = false;
	bool mPendingStop = false;
	const std::string mLocalHost{"0.0.0.0"};
	struct pdraw_muxer_stats mStats {
	};
	struct sdp_session *mSdpSession = nullptr;
	bool mHasBeenConnected = false;

	static const struct rtsp_client_cbs mRtspClientCbs;
};

} /* namespace Pdraw */
