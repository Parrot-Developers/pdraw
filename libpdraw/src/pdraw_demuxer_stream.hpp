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

#ifndef _PDRAW_DEMUXER_STREAM_HPP_
#define _PDRAW_DEMUXER_STREAM_HPP_

#include "pdraw_demuxer.hpp"

#include <queue>
#include <string>
#include <vector>

#include <h264/h264.h>
#include <libpomp.h>
#include <libsdp.h>
#include <rtsp/client.h>
#include <video-streaming/vstrm.h>

/* Demuxer stream output buffer count has been increased to hold up to 2 seconds
 * of video at 30fps to absorb network jitter */
#define DEMUXER_STREAM_OUTPUT_BUFFER_COUNT (60)

namespace Pdraw {

#define DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT 55004
#define DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT 55005
#define DEMUXER_STREAM_TEMP_QUEUE_MAX_SIZE 30
#define PDRAW_RTP_RXBUF_SIZE (1 * 1024 * 1024) /* In Bytes */


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(Session *session,
		      Element::Listener *elementListener,
		      Source::Listener *sourceListener,
		      DemuxerWrapper *wrapper,
		      IPdraw::IDemuxer::Listener *demuxerListener,
		      const struct pdraw_demuxer_params *params);

	virtual ~StreamDemuxer(void);

	int selectMedia(uint32_t selectedMedias) override;

	int start(void) override;

	int stop(void) override;

	int play(float speed = 1.0f) override;

	bool isReadyToPlay(void) override;

	bool isPaused(void) override;

	int previous(void) override;

	int next(void) override;

	int seek(int64_t delta, bool exact = false) override;

	int seekTo(uint64_t timestamp, bool exact = false) override;

	uint64_t getDuration(void) override;

	uint64_t getCurrentTime(void) override;

protected:
	enum SessionProtocol {
		NONE = 0,
		RTSP,
	};

	class VideoMedia : public Loggable {
	public:
		VideoMedia(StreamDemuxer *demuxer);

		virtual ~VideoMedia(void);

		bool hasMedia(Media *media);

		unsigned int getMediaCount(void) const
		{
			return mNbVideoMedias;
		}

		int setup(const struct sdp_media *media);

		int teardown(void);

		int createReceiver(void);

		int destroyReceiver(void);

		virtual int startRtpAvp(void) = 0;

		virtual int stopRtpAvp(void) = 0;

		virtual int sendCtrl(struct vstrm_receiver *stream,
				     struct tpkt_packet *pkt) = 0;

		virtual int prepareSetup(void) = 0;

		virtual enum rtsp_lower_transport
		getLowerTransport(void) const = 0;

		virtual uint16_t getLocalStreamPort(void) const = 0;

		virtual uint16_t getLocalControlPort(void) const = 0;

		virtual uint16_t getRemoteStreamPort(void) const = 0;

		virtual uint16_t getRemoteControlPort(void) const = 0;

		uint32_t getSsrc(void) const
		{
			return mSsrc;
		}

		bool getDestroyAfterFlush(void) const
		{
			return mDestroyAfterFlush;
		}

		const char *getControlUrl(void) const
		{
			return mSdpMedia->control_url;
		}

		virtual const struct rtsp_header_ext *getHeaderExt(void) const
		{
			return nullptr;
		}

		virtual size_t getHeaderExtCount(void) const
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

		void play(void);

		void pause(void);

		void resync(void);

		void stop(void);

		void flush(bool destroy = false);

		void setTearingDown(void);

		bool isTearingDown(void)
		{
			return mTearingDown;
		}

		void channelFlushed(Channel *channel);

		void channelUnlink(Channel *channel);

		void resetFrameTimer(bool rearm);

		void sendDownstreamEvent(Channel::DownstreamEvent event);

		int processFrame(struct vstrm_frame *frame);

		void channelSendVideoPresStats(Channel *channel,
					       VideoPresStats *stats);

		static void
		sessionMetadataFromSdp(const struct sdp_media *media,
				       const struct vmeta_session *sessionMeta,
				       struct vmeta_session *meta);


	protected:
		StreamDemuxer *mDemuxer;
		struct vstrm_receiver *mReceiver;
		uint16_t mLocalStreamPort;
		uint16_t mLocalControlPort;
		uint16_t mRemoteStreamPort;
		uint16_t mRemoteControlPort;

		void finishSetup(void);

		void finishTeardown(void);

	private:
		int setupMedia(void);

		void teardownMedia(void);

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

		CodedVideoMedia **mVideoMedias;
		unsigned int mNbVideoMedias;
		struct sdp_media *mSdpMedia;
		struct h264_reader *mH264Reader;
		struct pomp_timer *mFrameTimer;
		struct pomp_timer *mRangeTimer;
		uint32_t mSsrc;
		bool mFlushing;
		bool mDestroyAfterFlush;
		bool mPendingTearDown;
		bool mTearingDown;
		unsigned int mFlushChannelCount;
		bool mFirstFrame;
		uint64_t mLastFrameReceiveTime;
		unsigned int mFrameIndex;
		struct vstrm_codec_info mCodecInfo;
		bool mWaitForCodecInfo;
		bool mCodecInfoChanging;
		bool mWaitForSync;
		int mRecoveryFrameCount;
		std::queue<struct vstrm_frame *> mTempQueue;
		struct mbuf_coded_video_frame *mCurrentFrame;
		struct mbuf_mem *mCurrentMem;
		size_t mCurrentMemOffset;
		uint64_t mCurrentFrameCaptureTs;
		struct vmeta_session mSessionMetaFromSdp;
		static const struct vstrm_receiver_cbs mReceiverCbs;
		static const struct h264_ctx_cbs mH264Cbs;
	};

	struct SetupRequest {
		VideoMedia *media;
		char *controlUrl;
		enum rtsp_lower_transport lowerTransport;
		uint16_t localStreamPort;
		uint16_t localControlPort;
		const struct rtsp_header_ext *headerExt;
		size_t headerExtCount;
	};
	struct TeardownRequest {
		VideoMedia *media;
		char *controlUrl;
	};

	int startRtsp(const std::string &url);

	int processRtspRequests(void);

	void cleanupRtspRequests(void);

	int processSetupRequest(void);

	int processTeardownRequest(void);

	virtual VideoMedia *createVideoMedia(void) = 0;

	void teardownVideoMedia(StreamDemuxer::VideoMedia *media);

	void teardownAllVideoMedias(void);

	void destroyAllVideoMedias(void);

	std::string mUrl;
	std::string mServerAddr;
	std::string mServerIpAddr;
	uint16_t mServerPort;
	std::string mRtspAddr;
	std::string mRtspPath;
	const char *mContentBase;
	std::string mLocalAddr;
	std::string mRemoteAddr;
	SessionProtocol mSessionProtocol;
	std::vector<StreamDemuxer::VideoMedia *> mVideoMedias;
	std::queue<StreamDemuxer::SetupRequest> mSetupRequests;
	unsigned int mSetupRequestsCount;
	std::queue<StreamDemuxer::TeardownRequest> mTeardownRequests;
	unsigned int mTeardownRequestsCount;
	struct sdp_session *mSdpSession;

private:
	enum RtspState {
		DISCONNECTED = 0,
		CONNECTED,
		OPTIONS_DONE,
		DESCRIBE_DONE,
		SETUP_DONE,
	};

	int processSelectedMedias(void);

	int internalPlay(float speed);

	int internalPause(void);

	int flush(void) override;

	int flush(bool destroyMedias);

	void completeTeardown(void);

	void tryCompleteStart(bool callOpenResp = true);

	void tryCompleteStop(void);

	void asyncRtspDisconnect(void);

	void asyncCompleteTeardown(void);

	void onChannelFlushed(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	void onChannelResync(Channel *channel) override;

	void onChannelVideoPresStats(Channel *channel,
				     VideoPresStats *stats) override;

	void onNewSdp(const char *content_base, const char *sdp);

	void setRtspState(StreamDemuxer::RtspState state);

	static const char *getRtspStateStr(StreamDemuxer::RtspState val);

	static void sessionMetadataFromSdp(const struct sdp_session *session,
					   struct vmeta_session *meta);

	static void onRtspSocketCreated(int fd, void *userdata);

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

	struct vmeta_session mSessionMetaFromSdp;
	bool mChannelsReadyForStop;
	bool mNetworkReadyForStop;
	RtspState mRtspState;
	struct rtsp_client *mRtspClient;
	const char *mRtspSessionId;
	bool mRunning;
	bool mFlushing;
	bool mDestroyMediasAfterFlush;
	unsigned int mFlushChannelCount;
	uint64_t mStartTime;
	uint64_t mDuration;
	uint64_t mTrackDuration;
	bool mUpdateTrackDuration;
	uint64_t mCurrentTime;
	uint64_t mPausePoint;
	int64_t mNtpToNptOffset;
	unsigned int mRtpClockRate;
	float mSpeed;
	bool mFrameByFrame;
	bool mEndOfRangeNotified;
	bool mSeeking;
	static const struct rtsp_client_cbs mRtspClientCbs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_STREAM_HPP_ */
