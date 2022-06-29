/**
 * Parrot Drones Awesome Video Viewer Library
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
			const std::string &url);

	~RtmpStreamMuxer(void);

	int addInputMedia(CodedVideoMedia *media) override;

private:
	int internalStart(void) override;

	int internalStop(void) override;

	int configure(void);

	int process(void) override;

	int processMedia(CodedVideoMedia *media);

	int processFrame(CodedVideoMedia *media,
			 struct mbuf_coded_video_frame *frame);

	static void fakeAudioTimerCb(struct pomp_timer *timer, void *userdata);

	static void onSocketCreated(int fd, void *userdata);

	static void connectionStateCb(enum rtmp_connection_state state,
				      void *userdata);

	static void peerBwChangedCb(uint32_t bandwidth, void *userdata);

	static void
	dataUnrefCb(uint8_t *data, void *buffer_userdata, void *userdata);

	std::string mUrl;
	struct pomp_timer *mDummyAudioTimer;
	bool mDummyAudioStarted;
	struct rtmp_client *mRtmpClient;
	enum rtmp_connection_state mRtmpConnectionState;
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
	std::vector<uint8_t> mVideoAvcc;

	static const struct rtmp_callbacks mRtmpCbs;
	static const uint8_t mDummyAudioSpecificConfig[5];
	static const uint8_t mDummyAudioSample[6];
	static const int mDummyAudioSampleRate;
	static const int mDummyAudioSampleSize;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBRTMP */

#endif /* !_PDRAW_MUXER_STREAM_RTMP_HPP_ */
