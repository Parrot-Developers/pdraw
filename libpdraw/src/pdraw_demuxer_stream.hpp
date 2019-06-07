/**
 * Parrot Drones Awesome Video Viewer Library
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

namespace Pdraw {


#define DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT 55004
#define DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT 55005


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(Session *session,
		      Element::Listener *elementListener,
		      CodedSource::Listener *sourceListener,
		      IPdraw::IDemuxer *demuxer,
		      IPdraw::IDemuxer::Listener *demuxerListener);

	virtual ~StreamDemuxer(void);

	int start(void);

	int stop(void);

	int play(float speed = 1.0f);

	bool isReadyToPlay(void);

	bool isPaused(void);

	int previous(void);

	int next(void);

	int seek(int64_t delta, bool exact = false);

	int seekTo(uint64_t timestamp, bool exact = false);

	uint64_t getDuration(void);

	uint64_t getCurrentTime(void);

protected:
	enum SessionProtocol {
		NONE = 0,
		RTSP,
	};

	class VideoMedia : public Loggable {
	public:
		VideoMedia(StreamDemuxer *demuxer);

		virtual ~VideoMedia(void);

		bool hasMedia(CodedVideoMedia *media);

		int setup(const struct sdp_media *media);

		int createReceiver(void);

		int destroyReceiver(void);

		virtual int startRtpAvp(void) = 0;

		virtual int stopRtpAvp(void) = 0;

		virtual int sendCtrl(struct vstrm_receiver *stream,
				     struct tpkt_packet *pkt) = 0;

		virtual int prepareSetup(void) = 0;

		virtual enum rtsp_lower_transport getLowerTransport(void) = 0;

		virtual uint16_t getLocalStreamPort(void) = 0;

		virtual uint16_t getLocalControlPort(void) = 0;

		virtual uint16_t getRemoteStreamPort(void) = 0;

		virtual uint16_t getRemoteControlPort(void) = 0;

		uint32_t getSsrc(void)
		{
			return mSsrc;
		}

		const char *getControlUrl(void)
		{
			return mSdpMedia->control_url;
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

		void stop(void);

		void flush(void);

		void channelFlushed(CodedChannel *channel);

		void channelUnlink(CodedChannel *channel);

		void sendDownstreamEvent(CodedChannel::DownstreamEvent event);

		int processFrame(struct vstrm_frame *frame);

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
	};

	int startRtsp(const std::string &url);

	int processSetupRequests(void);

	virtual VideoMedia *createVideoMedia(void) = 0;
	void destroyAllVideoMedias(void);

	std::string mUrl;
	std::string mServerAddr;
	std::string mRtspAddr;
	std::string mRtspPath;
	const char *mContentBase;
	std::string mLocalAddr;
	std::string mRemoteAddr;
	SessionProtocol mSessionProtocol;
	std::vector<StreamDemuxer::VideoMedia *> mVideoMedias;
	std::queue<StreamDemuxer::SetupRequest> mSetupRequests;
	unsigned int mSetupRequestsCount;

private:
	enum RtspState {
		DISCONNECTED = 0,
		CONNECTED,
		OPTIONS_DONE,
		DESCRIBE_DONE,
		SETUP_DONE,
	};

	int internalPlay(float speed);

	int internalPause(void);

	int flush(void);

	int flush(bool destroyMedias);

	void completeTeardown(void);

	void onChannelFlushed(CodedChannel *channel);

	void onChannelUnlink(CodedChannel *channel);

	void onChannelResync(CodedChannel *channel);

	void onNewSdp(const char *content_base, const char *sdp);

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

	static void idleRtspDisconnect(void *userdata);

	struct vmeta_session mSessionMetaFromSdp;
	bool mChannelsReadyForStop;
	bool mNetworkReadyForStop;
	RtspState mRtspState;
	bool mTearingDown;
	struct rtsp_client *mRtspClient;
	const char *mRtspSessionId;
	bool mRunning;
	bool mFlushing;
	bool mDestroyMediasAfterFlush;
	unsigned int mFlushChannelCount;
	uint64_t mStartTime;
	uint64_t mDuration;
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
