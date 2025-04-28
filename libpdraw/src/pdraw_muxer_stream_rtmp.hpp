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

#ifndef _PDRAW_MUXER_STREAM_RTMP_HPP_
#define _PDRAW_MUXER_STREAM_RTMP_HPP_

#ifdef BUILD_LIBRTMP

#	include "pdraw_muxer.hpp"

#	include <vector>

#	include <rtmp.h>

namespace Pdraw {


class RtmpStreamMuxer : public Muxer {
public:
	RtmpStreamMuxer(Session *session,
			Element::Listener *elementListener,
			IPdraw::IMuxer::Listener *listener,
			MuxerWrapper *wrapper,
			const std::string &url,
			const struct pdraw_muxer_params *params);

	~RtmpStreamMuxer(void);

	int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	};

	int getStats(struct pdraw_muxer_stats *stats) override;

private:
	enum RtmpState {
		DISCONNECTED = 0,
		CONNECTING,
		CONNECTED,
	};

	int internalStart(void) override;

	int internalStop(void) override;

	int configure(void);

	int scheduleReconnection(void);

	int reconnect(void);

	int process(void) override;

	int processMedia(CodedVideoMedia *media);

	int processFrame(CodedVideoMedia *media,
			 struct mbuf_coded_video_frame *frame);

	void onChannelFlush(Channel *channel) override;

	void setRtmpState(RtmpStreamMuxer::RtmpState state);

	static const char *getRtmpStateStr(RtmpStreamMuxer::RtmpState val);

	static enum pdraw_muxer_connection_state
	rtmpStateToMuxerConnectionState(RtmpStreamMuxer::RtmpState val);

	static void getReconnectionStrategy(
		enum rtmp_client_disconnection_reason disconnectionReason,
		bool *doReconnect,
		int *reconnectionCount);

	static void fakeAudioTimerCb(struct pomp_timer *timer, void *userdata);

	static void onSocketCreated(int fd, void *userdata);

	static void connectionStateCb(
		enum rtmp_client_conn_state state,
		enum rtmp_client_disconnection_reason disconnection_reason,
		void *userdata);

	static void peerBwChangedCb(uint32_t bandwidth, void *userdata);

	static void
	dataUnrefCb(uint8_t *data, void *buffer_userdata, void *userdata);

	static void connectionWatchdogCb(struct pomp_timer *timer,
					 void *userdata);

	static void reconnectionTimerCb(struct pomp_timer *timer,
					void *userdata);

	std::string mUrl;
	struct pomp_timer *mDummyAudioTimer;
	bool mDummyAudioStarted;
	struct rtmp_client *mRtmpClient;
	RtmpState mRtmpState;
	enum rtmp_client_conn_state mRtmpConnectionState;
	enum rtmp_client_disconnection_reason mRtmpDisconnectionReason;
	bool mConfigured;
	bool mSynchronized;
	CodedVideoMedia *mVideoMedia;
	double mDuration;
	int mWidth;
	int mHeight;
	double mFramerate;
	int mAudioSampleRate;
	int mAudioSampleSize;
	uint32_t mDummyAudioTimestamp;
	struct pdraw_muxer_stats mStats;
	std::vector<uint8_t> mVideoAvcc;
	struct pomp_timer *mConnectionWatchdog;
	bool mHasBeenConnected;
	int mReconnectionCount;
	int mReconnectionMaxCount;
	struct pomp_timer *mReconnectionTimer;

	static const struct rtmp_callbacks mRtmpCbs;
	static const uint8_t mDummyAudioSpecificConfig[5];
	static const uint8_t mDummyAudioSample[6];
	static const int mDummyAudioSampleRate;
	static const int mDummyAudioSampleSize;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBRTMP */

#endif /* !_PDRAW_MUXER_STREAM_RTMP_HPP_ */
