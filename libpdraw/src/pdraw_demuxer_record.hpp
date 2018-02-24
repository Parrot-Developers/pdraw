/**
 * Parrot Drones Awesome Video Viewer Library
 * Recording demuxer
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

#ifndef _PDRAW_DEMUXER_RECORD_HPP_
#define _PDRAW_DEMUXER_RECORD_HPP_

#include "pdraw_demuxer.hpp"
#include "pdraw_avcdecoder.hpp"
#include <libmp4.h>
#include <h264/h264.h>
#include <libpomp.h>
#include <string>

namespace Pdraw {


class RecordDemuxer : public Demuxer {
public:
	RecordDemuxer(
		Session *session);

	~RecordDemuxer(
		void);

	enum demuxer_type getType(
		void) {
		return DEMUXER_TYPE_RECORD;
	}

	int configure(
		const std::string &fileName);

	bool isConfigured(
		void) {
		return mConfigured;
	}

	int close(
		void);

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
		int esIndex, Decoder *decoder);

	int play(
		float speed = 1.0f);

	bool isPaused(
		void);

	int previous(
		void);

	int next(
		void);

	int seek(
		int64_t delta,
		bool exact = false);

	int seekTo(
		uint64_t timestamp,
		bool exact = false);

	uint64_t getDuration(
		void) {
		return mDuration;
	}

	uint64_t getCurrentTime(
		void) {
		return mCurrentTime;
	}

	Session *getSession(
		void) {
		return mSession;
	}

private:
	int fetchVideoDimensions(
		void);

	int fetchSessionMetadata(
		void);

	static int getAvcDecoderConfig(
		RecordDemuxer *demuxer,
		uint8_t **pSps,
		size_t *spsSize,
		uint8_t **pPps,
		size_t *ppsSize);

	static void h264UserDataSeiCb(
		struct h264_ctx *ctx,
		const uint8_t *buf,
		size_t len,
		const struct h264_sei_user_data_unregistered *sei,
		void *userdata);

	static void timerCb(
		struct pomp_timer *timer,
		void *userdata);

	std::string mFileName;
	AvcDecoder *mDecoder;
	uint32_t mDecoderBitstreamFormat;
	pthread_mutex_t mDemuxerMutex;
	bool mRunning;
	bool mFrameByFrame;
	struct mp4_demux *mDemux;
	struct pomp_timer *mTimer;
	struct h264_reader *mH264Reader;
	uint64_t mDuration;
	uint64_t mCurrentTime;
	int mVideoTrackCount;
	unsigned int mVideoTrackId;
	char *mMetadataMimeType;
	bool mFirstFrame;
	size_t mMetadataBufferSize;
	uint8_t *mMetadataBuffer;
	int64_t mAvgOutputInterval;
	uint64_t mLastFrameOutputTime;
	int64_t mLastFrameDuration;
	int64_t mLastOutputError;
	int64_t mPendingSeekTs;
	bool mPendingSeekExact;
	bool mPendingSeekToPrevSample;
	struct vbuf_buffer *mCurrentBuffer;
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
	float mSpeed;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_RECORD_HPP_ */
