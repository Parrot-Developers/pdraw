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

#ifdef BUILD_LIBRTMP

#	include "pdraw_muxer.hpp"

#	include <vector>

#	include <rtmp.h>

namespace Pdraw {


constexpr size_t MUXER_STREAM_RTMP_RECONNECTION_MAX_COUNT = 10;


class RtmpStreamMuxer : public Muxer {
public:
	RtmpStreamMuxer(Session *session,
			Element::Listener *elementListener,
			IPdraw::IMuxer::Listener *listener,
			MuxerWrapper *wrapper,
			const std::string &url,
			const struct pdraw_muxer_params *params);

	~RtmpStreamMuxer() override;

	int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	};

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

private:
	enum class RtmpState {
		DISCONNECTED = 0,
		CONNECTING,
		CONNECTED,
	};

	int internalStart() override;

	int internalStop() final;

	int configure();

	int scheduleReconnection();

	int reconnect();

	int process() override;

	int processMedia(const CodedVideoMedia *media);

	int processFrame(const CodedVideoMedia *media,
			 struct mbuf_coded_video_frame *frame);

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void setRtmpState(RtmpStreamMuxer::RtmpState state);

	static const char *getRtmpStateStr(RtmpStreamMuxer::RtmpState val);

	static enum pdraw_muxer_connection_state
	rtmpStateToMuxerConnectionState(RtmpStreamMuxer::RtmpState val);

	static void getReconnectionStrategy(
		enum rtmp_client_disconnection_reason disconnectionReason,
		bool *doReconnect,
		int *reconnectionCount);

	void onFakeAudioTimer();

	static void onSocketCreated(int fd, void *userdata);

	static void connectionStateCb(
		enum rtmp_client_conn_state state,
		enum rtmp_client_disconnection_reason disconnection_reason,
		void *userdata);

	static void peerBwChangedCb(uint32_t bandwidth, void *userdata);

	static void
	dataUnrefCb(uint8_t *data, void *buffer_userdata, void *userdata);

	void onConnectionWatchdog();

	void onReconnectionTimer();

	std::string mUrl{};
	pomp::Timer::HandlerFunc mDummyAudioTimerHandler;
	std::unique_ptr<pomp::Timer> mDummyAudioTimer;
	bool mDummyAudioStarted = false;
	struct rtmp_client *mRtmpClient = nullptr;
	size_t mSocketTxBufferSize = 0;
	RtmpState mRtmpState = RtmpState::DISCONNECTED;
	enum rtmp_client_conn_state mRtmpConnectionState =
		RTMP_CLIENT_CONN_STATE_DISCONNECTED;
	enum rtmp_client_disconnection_reason mRtmpDisconnectionReason =
		RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;
	bool mConfigured = false;
	bool mSynchronized = false;
	CodedVideoMedia *mVideoMedia = nullptr;
	double mDuration = 0.;
	int mWidth = 0;
	int mHeight = 0;
	double mFramerate = 0.;
	int mAudioSampleRate = 0;
	int mAudioSampleSize = 0;
	uint32_t mDummyAudioTimestamp = 0;
	struct pdraw_muxer_stats mStats {
	};
	std::vector<uint8_t> mVideoAvcc{};
	pomp::Timer::HandlerFunc mConnectionWatchdogHandler;
	std::unique_ptr<pomp::Timer> mConnectionWatchdog;
	bool mHasBeenConnected = false;
	int mReconnectionCount = 0;
	int mReconnectionMaxCount = MUXER_STREAM_RTMP_RECONNECTION_MAX_COUNT;
	pomp::Timer::HandlerFunc mReconnectionTimerHandler;
	std::unique_ptr<pomp::Timer> mReconnectionTimer;

	static const struct rtmp_callbacks mRtmpCbs;
	static const std::array<uint8_t, 5> mDummyAudioSpecificConfig;
	static const std::array<uint8_t, 6> mDummyAudioSample;
	static const int mDummyAudioSampleRate;
	static const int mDummyAudioSampleSize;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBRTMP */
