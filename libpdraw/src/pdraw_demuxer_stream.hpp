/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer
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

#ifndef _PDRAW_DEMUXER_STREAM_HPP_
#define _PDRAW_DEMUXER_STREAM_HPP_

#include "pdraw_demuxer.hpp"
#include "pdraw_avcdecoder.hpp"
#include <pthread.h>
#include <libARStream2/arstream2_stream_receiver.h>
#include <libpomp.h>
#include <librtsp.h>
#include <libsdp.h>
#include <string>

namespace Pdraw {


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(
		Session *session);

	~StreamDemuxer(
		void);

	enum demuxer_type getType(
		void) {
		return DEMUXER_TYPE_STREAM;
	}

	bool isConfigured(
		void) {
		return mConfigured;
	}

	int configure(
		const std::string &url);

	int configure(
		const std::string &url,
		const std::string &ifaceAddr);

	int configure(
		const std::string &srcAddr,
		const std::string &ifaceAddr,
		int srcStreamPort,
		int srcControlPort,
		int dstStreamPort,
		int dstControlPort,
		int qosMode);

	int configure(
		void *muxContext);

	int configureWithSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	int getElementaryStreamCount(
		void);

	enum elementary_stream_type getElementaryStreamType(
		int esIndex);

	int getElementaryStreamVideoDimensions(
		int esIndex,
		unsigned int *width,
		unsigned int *height,
		unsigned int *cropLeft,
		unsigned int *cropRight,
		unsigned int *cropTop,
		unsigned int *cropBottom,
		unsigned int *sarWidth,
		unsigned int *sarHeight);

	int getElementaryStreamVideoFov(
		int esIndex,
		float *hfov,
		float *vfov);

	int setElementaryStreamDecoder(
		int esIndex,
		Decoder *decoder);

	int play(
		float speed = 1.0f);

	int pause(
		void);

	bool isPaused(
		void);

	int previous(
		void);

	int next(
		void);

	int stop(
		void);

	int seekTo(
		uint64_t timestamp,
		bool exact = false);

	int seekForward(
		uint64_t delta,
		bool exact = false);

	int seekBack(
		uint64_t delta,
		bool exact = false);

	int startRecorder(
		const std::string &fileName);

	int stopRecorder(
		void);

	int startResender(
		const std::string &dstAddr,
		const std::string &ifaceAddr,
		int srcStreamPort,
		int srcControlPort,
		int dstStreamPort,
		int dstControlPort);

	int stopResender(
		void);

	uint64_t getDuration(
		void) {
		return (uint64_t)-1;
	}

	uint64_t getCurrentTime(
		void);

	Session *getSession(
		void) {
		return mSession;
	}

private:
	static void fetchSessionMetadata(
		StreamDemuxer *demuxer);

	int configureRtpAvp(
		const char *srcAddr,
		const char *mcastIfaceAddr,
		int srcStreamPort,
		int srcControlPort,
		int dstStreamPort,
		int dstControlPort);

	int configureSdp(
		const char *sdp,
		const char *mcastIfaceAddr);

	int configureRtsp(
		const char *url,
		const char *mcastIfaceAddr);

	static eARSTREAM2_ERROR spsPpsCallback(
		uint8_t *spsBuffer,
		int spsSize,
		uint8_t *ppsBuffer,
		int ppsSize,
		void *userPtr);

	static eARSTREAM2_ERROR getAuBufferCallback(
		uint8_t **auBuffer,
		int *auBufferSize,
		void **auBufferUserPtr,
		void *userPtr);

	static eARSTREAM2_ERROR auReadyCallback(
		uint8_t *auBuffer,
		int auSize,
		ARSTREAM2_StreamReceiver_AuReadyCallbackTimestamps_t *auTimestamps,
		eARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE auSyncType,
		ARSTREAM2_StreamReceiver_AuReadyCallbackMetadata_t *auMetadata,
		void *auBufferUserPtr,
		void *userPtr);

	static void* runLoopThread(
		void *ptr);

	uint32_t mCurrentAuSize;
	int mMaxPacketSize;
	int mQosMode;
	AvcDecoder *mDecoder;
	uint32_t mDecoderBitstreamFormat;
	struct vbuf_buffer *mCurrentBuffer;
	struct pomp_loop *mLoop;
	pthread_t mLoopThread;
	bool mLoopThreadLaunched;
	bool mThreadShouldStop;
	bool mRtspRunning;
	struct rtsp_client *mRtspClient;
	pthread_t mStreamNetworkThread;
	bool mStreamNetworkThreadLaunched;
	pthread_t mStreamOutputThread;
	bool mStreamOutputThreadLaunched;
	int mRunning;
	ARSTREAM2_StreamReceiver_Handle mStreamReceiver;
	ARSTREAM2_StreamReceiver_ResenderHandle mResender;
	uint64_t mStartTime;
	uint64_t mCurrentTime;
	uint64_t mLastSessionMetadataFetchTime;
	unsigned int mWidth;
	unsigned int mHeight;
	unsigned int mCropLeft;
	unsigned int mCropRight;
	unsigned int mCropTop;
	unsigned int mCropBottom;
	unsigned int mSarWidth;
	unsigned int mSarHeight;
	float mHfov;
	float mVfov;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_STREAM_HPP_ */
