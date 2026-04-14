/**
 * Parrot Drones Audio and Video Vector library
 * Streaming demuxer
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

#include "pdraw_demuxer.hpp"

#include <queue>
#include <string>
#include <vector>

#include <h264/h264.h>
#include <libpomp.h>
#include <libsdp.h>
#include <rtsp/client.h>
#include <rtsp/rtsp_url.hpp>
#include <video-streaming/vstrm.h>

/* Demuxer stream output buffer count has been increased to hold up to 2 seconds
 * of video at 30fps to absorb network jitter */
constexpr size_t DEMUXER_STREAM_OUTPUT_BUFFER_COUNT = 60;

namespace Pdraw {

constexpr size_t DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT = 55004;
constexpr size_t DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT = 55005;
constexpr size_t DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE = 30;
constexpr size_t PDRAW_RTP_RXBUF_SIZE = (1 * 1024 * 1024); /* In Bytes */


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(Session *session,
		      Element::Listener *elementListener,
		      Source::Listener *sourceListener,
		      DemuxerWrapper *wrapper,
		      IPdraw::IDemuxer::Listener *demuxerListener,
		      const struct pdraw_demuxer_params *params);

	~StreamDemuxer() override;

	int selectMedia(uint32_t selectedMedias) override;

	int start() override;

	int stop() override;

	int play(float speed = 1.0f) override;

	bool isReadyToPlay() const override;

	bool isPaused() const override;

	int previous() override;

	int next() override;

	int seek(int64_t delta, bool exact = false) override;

	int seekTo(uint64_t timestamp, bool exact = false) override;

	uint64_t getDuration() const override;

	uint64_t getCurrentTime() const override;

protected:
	enum class SessionProtocol {
		NONE = 0,
		RTSP,
	};

	class VideoMedia : public Loggable {
	public:
		explicit VideoMedia(StreamDemuxer *demuxer);

		~VideoMedia() override;

		bool hasMedia(const Media *media) const;

		unsigned int getMediaCount() const
		{
			return mVideoMedias.size();
		}

		int setup(const struct sdp_media *media);

		int teardown();

		int createReceiver();

		int destroyReceiver();

		virtual int startRtpAvp() = 0;

		virtual int stopRtpAvp() = 0;

		virtual int sendCtrl(struct vstrm_receiver *stream,
				     struct tpkt_packet *pkt) = 0;

		virtual int prepareSetup() = 0;

		virtual enum rtsp_lower_transport getLowerTransport() const = 0;

		virtual uint16_t getLocalStreamPort() const = 0;

		virtual uint16_t getLocalControlPort() const = 0;

		virtual uint16_t getRemoteStreamPort() const = 0;

		virtual uint16_t getRemoteControlPort() const = 0;

		virtual int processDataPkt(struct tpkt_packet *pkt) = 0;

		virtual int processCtrlPkt(struct tpkt_packet *pkt) = 0;

		uint32_t getSsrc() const
		{
			return mSsrc;
		}

		bool getDestroyAfterFlush() const
		{
			return mDestroyAfterFlush;
		}

		const char *getControlUrl() const
		{
			return mSdpMedia->control_url;
		}

		virtual const struct rtsp_header_ext *getHeaderExt() const
		{
			return nullptr;
		}

		virtual size_t getHeaderExtCount() const
		{
			return 0;
		}

		virtual void setLocalStreamPort(uint16_t port) = 0;

		virtual void setLocalControlPort(uint16_t port) = 0;

		virtual void setRemoteStreamPort(uint16_t port) = 0;

		virtual void setRemoteControlPort(uint16_t port) = 0;

		void setSsrc(uint32_t ssrc)
		{
			mSsrc = ssrc;
		}

		void play();

		void onPlayComplete() const;

		void pause() const;

		void onPauseComplete();

		void resync();

		void seek();

		void previous();

		void next();

		void stop();

		void flush(bool discard = true);

		inline void drain()
		{
			flush(false);
		}

		bool isSeeking() const
		{
			return mPendingSeek;
		}

		void setTearingDown();

		bool isTearingDown() const
		{
			return mTearingDown;
		}

		bool isRtpPaused() const
		{
			return mRtpPaused;
		}

		void setDestroyAfterFlush(bool destroy)
		{
			mDestroyAfterFlush = destroy;
		}

		void channelFlushed(const Channel *channel);

		void channelDrained(const Channel *channel);

		void channelUnlink(const Channel *channel);

		void resetFrameTimer(bool rearm);

		void sendDownstreamEvent(Channel::DownstreamEvent event);

		int processFrame(struct vstrm_frame *frame);

		void channelSendVideoPresStats(const Channel *channel,
					       const VideoPresStats *stats);

		static void
		sessionMetadataFromSdp(const struct sdp_media *media,
				       const struct vmeta_session *sessionMeta,
				       struct vmeta_session *meta);


	protected:
		StreamDemuxer *mDemuxer = nullptr;
		struct vstrm_receiver *mReceiver = nullptr;
		uint16_t mLocalStreamPort = 0;
		uint16_t mLocalControlPort = 0;
		uint16_t mRemoteStreamPort = 0;
		uint16_t mRemoteControlPort = 0;

		void finishSetup();

		void finishTeardown();

	private:
		int setupMedia();

		void teardownMedia();

		void asyncCompleteSeek();

		static void idleCompleteSeek(void *userdata);

		void completeSeek();

		void completeFlush();

		static void h264UserDataSeiCb(
			struct h264_ctx *ctx,
			const uint8_t *buf,
			size_t len,
			const struct h264_sei_user_data_unregistered *sei,
			void *userdata);

		static void
		h264PicTimingSeiCb(struct h264_ctx *ctx,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_sei_pic_timing *sei,
				   void *userdata);

		static void h264RecoveryPointSeiCb(
			struct h264_ctx *ctx,
			const uint8_t *buf,
			size_t len,
			const struct h264_sei_recovery_point *sei,
			void *userdata);

		static int sendCtrlCb(struct vstrm_receiver *stream,
				      struct tpkt_packet *pkt,
				      void *userdata);

		static void
		codecInfoChangedCb(struct vstrm_receiver *stream,
				   const struct vstrm_codec_info *info,
				   void *userdata);

		static void recvFrameCb(struct vstrm_receiver *stream,
					struct vstrm_frame *frame,
					void *userdata);

		static void
		sessionMetadataPeerChangedCb(struct vstrm_receiver *stream,
					     const struct vmeta_session *meta,
					     void *userdata);

		static void eventCb(struct vstrm_receiver *stream,
				    enum vstrm_event event,
				    void *userdata);

		static void goodbyeCb(struct vstrm_receiver *stream,
				      const char *reason,
				      void *userdata);

		static void frameTimeoutCb(struct pomp_timer *timer,
					   void *userdata);

		static void rangeTimerCb(struct pomp_timer *timer,
					 void *userdata);

		std::vector<std::unique_ptr<CodedVideoMedia>> mVideoMedias{};
		struct sdp_media *mSdpMedia = nullptr;
		struct h264_reader *mH264Reader = nullptr;
		struct pomp_timer *mFrameTimer = nullptr;
		struct pomp_timer *mRangeTimer = nullptr;
		uint32_t mSsrc = 0;
		bool mFlushing = false;
		bool mFlushDiscard = false;
		bool mPendingSeek = false;
		int mSeekResponse = 0;
		bool mAsyncCompleteSeekCalled = false;
		bool mDestroyAfterFlush = false;
		bool mPendingTearDown = false;
		bool mTearingDown = false;
		unsigned int mFlushChannelCount = 0;
		bool mFirstFrame = true;
		uint64_t mLastFrameReceiveTime = 0;
		unsigned int mFrameIndex = 0;
		struct vstrm_codec_info mCodecInfo {
		};
		bool mWaitForCodecInfo = false;
		bool mCodecInfoChanging = false;
		bool mRtcpMediaChangeReceived = false;
		bool mWaitForSync = false;
		int mRecoveryFrameCount = 0;
		std::queue<struct vstrm_frame *> mTempQueue{};
		struct mbuf_coded_video_frame *mCurrentFrame = nullptr;
		struct mbuf_mem *mCurrentMem = nullptr;
		size_t mCurrentMemOffset = 0;
		uint64_t mCurrentFrameCaptureTs = 0;
		struct vmeta_session mSessionMetaFromSdp {
		};
		bool mRtpPaused = true;
		static const struct vstrm_receiver_cbs mReceiverCbs;
		static const struct h264_ctx_cbs mH264Cbs;
	};

	struct SetupRequest {
		SetupRequest(VideoMedia *m,
			     const std::string &ctrl,
			     rtsp_lower_transport lt,
			     uint16_t streamPort,
			     uint16_t controlPort,
			     const rtsp_header_ext *ext,
			     size_t extCount) :
				media(m),
				controlUrl(ctrl), lowerTransport(lt),
				localStreamPort(streamPort),
				localControlPort(controlPort), headerExt(ext),
				headerExtCount(extCount)
		{
		}

		VideoMedia *media = nullptr;
		std::string controlUrl{};
		enum rtsp_lower_transport lowerTransport =
			RTSP_LOWER_TRANSPORT_UDP;
		uint16_t localStreamPort = 0;
		uint16_t localControlPort = 0;
		const struct rtsp_header_ext *headerExt = nullptr;
		size_t headerExtCount = 0;
	};

	struct TeardownRequest {
		TeardownRequest(VideoMedia *m, const std::string &ctrl) :
				media(m), controlUrl(ctrl)
		{
		}

		VideoMedia *media = nullptr;
		std::string controlUrl{};
	};

	int startRtsp();

	int processRtspRequests();

	void cleanupRtspRequests();

	int processSetupRequest();

	int processTeardownRequest();

	virtual std::unique_ptr<VideoMedia>
	createVideoMedia(enum rtsp_lower_transport transport) = 0;

	void teardownVideoMedia(StreamDemuxer::VideoMedia *media);

	void teardownAllVideoMedias();

	void destroyAllVideoMedias();

	std::unique_ptr<RtspUrl> mUrl;
	struct rtsp_client *mRtspClient = nullptr;
	const char *mContentBase = nullptr;
	const char *mShortContentBase = nullptr;
	std::string mLocalAddr{};
	std::string mRemoteAddr{};
	SessionProtocol mSessionProtocol = SessionProtocol::NONE;
	std::vector<std::unique_ptr<StreamDemuxer::VideoMedia>> mVideoMedias{};
	std::queue<StreamDemuxer::SetupRequest> mSetupRequests{};
	unsigned int mSetupRequestsCount = 0;
	std::queue<StreamDemuxer::TeardownRequest> mTeardownRequests{};
	unsigned int mTeardownRequestsCount = 0;
	struct sdp_session *mSdpSession = nullptr;

private:
	enum class RtspState {
		DISCONNECTED = 0,
		CONNECTED,
		OPTIONS_DONE,
		DESCRIBE_DONE,
		SETUP_DONE,
	};

	int processSelectedMedias();

	int internalPlay(float speed);

	int internalPause();

	int flush(bool discard = true) override;

	void completeTeardown();

	void tryCompleteStart(bool callOpenResp = true);

	void tryCompleteStop();

	void asyncRtspDisconnect();

	void asyncCompleteTeardown();

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	void onChannelResync(Channel *channel) override;

	void onChannelVideoPresStats(Channel *channel,
				     VideoPresStats *stats) override;

	void onNewSdp(const char *content_base, const char *sdp);

	void setRtspState(StreamDemuxer::RtspState state);

	void onMediaSeekComplete(int seekResponse);

	int sendDescribe();

	static const char *getRtspStateStr(StreamDemuxer::RtspState val);

	static void sessionMetadataFromSdp(const struct sdp_session *session,
					   struct vmeta_session *meta);

	static void onRtspSocketCreated(int fd, void *userdata);

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

	static void onRtspDescribeResp(struct rtsp_client *client,
				       enum rtsp_client_req_status req_status,
				       int status,
				       const char *content_base,
				       const struct rtsp_header_ext *ext,
				       size_t ext_count,
				       const char *sdp,
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

	static void onRtspPlayResp(struct rtsp_client *client,
				   const char *session_id,
				   enum rtsp_client_req_status req_status,
				   int status,
				   const struct rtsp_range *range,
				   float scale,
				   int seq_valid,
				   uint16_t seq,
				   int rtptime_valid,
				   uint32_t rtptime,
				   const struct rtsp_header_ext *ext,
				   size_t ext_count,
				   void *userdata,
				   void *req_userdata);

	static void onRtspPauseResp(struct rtsp_client *client,
				    const char *session_id,
				    enum rtsp_client_req_status req_status,
				    int status,
				    const struct rtsp_range *range,
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

	static void idleRtspDisconnect(void *userdata);
	static void idleEndOfRangeNotification(void *userdata);
	static void idleCompleteTeardown(void *userdata);

	static void videoStatsDynCleaner(struct mbuf_ancillary_data *data,
					 void *userdata);

	struct vmeta_session mSessionMetaFromSdp {
	};
	bool mChannelsReadyForStop = false;
	bool mNetworkReadyForStop = false;
	bool mDescribeRetried = false;
	RtspState mRtspState = RtspState::DISCONNECTED;
	const char *mRtspSessionId = nullptr;
	bool mRunning = false;
	/* Whether mRunning has been set to true at least once;
	 * needed to handle the start_in_pause (PAUSE_NEXT) feature */
	bool mWasRunningOnce = false;
	bool mDestroyMediasAfterFlush = false;
	unsigned int mFlushChannelCount = 0;
	uint64_t mStartTime = 0;
	uint64_t mDuration = 0;
	uint64_t mTrackDuration = 0;
	bool mUpdateTrackDuration = false;
	uint64_t mCurrentTime = 0;
	uint64_t mPausePoint = 0;
	uint64_t mPlayNtpTime = 0;
	int64_t mNtpToNptOffset = 0;
	unsigned int mRtpClockRate = 0;
	float mSpeed = 1.f;
	bool mFrameByFrame = true;
	bool mEndOfRangeNotified = false;
	bool mSeeking = false;
	bool mSeekingNetwork = false;
	int mSeekResponse = false;
	static const struct rtsp_client_cbs mRtspClientCbs;
};

} /* namespace Pdraw */
