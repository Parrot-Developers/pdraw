/**
 * Parrot Drones Awesome Video Viewer Library
 * Recording demuxer
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

#ifndef _PDRAW_DEMUXER_RECORD_HPP_
#define _PDRAW_DEMUXER_RECORD_HPP_

#include "pdraw_demuxer.hpp"

#include <string>
#include <vector>

#include <h264/h264.h>
#include <h265/h265.h>
#include <libmp4.h>
#include <libpomp.h>

namespace Pdraw {

class RecordDemuxer : public Demuxer {
public:
	RecordDemuxer(Session *session,
		      Element::Listener *elementListener,
		      CodedSource::Listener *sourceListener,
		      IPdraw::IDemuxer *demuxer,
		      IPdraw::IDemuxer::Listener *demuxerListener,
		      const std::string &fileName);

	~RecordDemuxer(void);

	int start(void);

	int stop(void);

	int play(float speed = 1.0f);

	bool isReadyToPlay(void);

	bool isPaused(void);

	int previous(void);

	int next(void);

	int seek(int64_t delta, bool exact = false);

	int seekTo(uint64_t timestamp, bool exact = false);

	uint64_t getDuration(void)
	{
		return mDuration;
	}

	uint64_t getCurrentTime(void)
	{
		return mCurrentTime;
	}

private:
	class VideoMedia : public Loggable {
	public:
		VideoMedia(RecordDemuxer *demuxer);

		~VideoMedia(void);

		int setup(struct mp4_track_info *tkinfo);

		void play(void);

		void previous(void);

		void next(void);

		void seek(int64_t delta, bool exact);

		void seekTo(uint64_t timestamp, bool exact);

		void stop(void);

	private:
		void sendDownstreamEvent(CodedChannel::DownstreamEvent event);

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

		static void h265UserDataSeiCb(
			struct h265_ctx *ctx,
			const uint8_t *buf,
			size_t len,
			const struct h265_sei_user_data_unregistered *sei,
			void *userdata);

		static void
		h265TimeCodeSeiCb(struct h265_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h265_sei_time_code *sei,
				  void *userdata);

		static void h265MdcvSeiCb(
			struct h265_ctx *ctx,
			const uint8_t *buf,
			size_t len,
			const struct h265_sei_mastering_display_colour_volume
				*sei,
			void *userdata);

		static void
		h265CllSeiCb(struct h265_ctx *ctx,
			     const uint8_t *buf,
			     size_t len,
			     const struct h265_sei_content_light_level *sei,
			     void *userdata);

		static void timerCb(struct pomp_timer *timer, void *userdata);

		RecordDemuxer *mDemuxer;
		bool mFirstFrame;
		struct pomp_timer *mTimer;
		struct h264_reader *mH264Reader;
		struct h265_reader *mH265Reader;
		CodedVideoMedia **mVideoMedias;
		unsigned int mNbVideoMedias;
		unsigned int mVideoTrackId;
		char *mMetadataMimeType;
		size_t mMetadataBufferSize;
		uint8_t *mMetadataBuffer;
		uint32_t mTimescale;
		int64_t mAvgOutputInterval;
		uint64_t mLastFrameOutputTime;
		int64_t mLastFrameDuration;
		int64_t mLastOutputError;
		int64_t mPendingSeekTs;
		bool mPendingSeekExact;
		bool mPendingSeekToPrevSample;
		bool mPendingSeekToNextSample;
		int mSeekResponse;
		struct mbuf_coded_video_frame *mCurrentFrame;
		struct mbuf_mem *mCurrentMem;
		uint64_t mCurrentFrameCaptureTs;
		uint64_t mDecodingTs;
		uint64_t mDecodingTsInc;
		unsigned int mFrameIndex;
		static const struct h264_ctx_cbs mH264ReaderCbs;
		static const struct h265_ctx_cbs mH265ReaderCbs;
	};

	int fetchSessionMetadata(unsigned int trackId,
				 struct vmeta_session *meta);

	int flush(void);

	void completeFlush(void);

	void completeTeardown(void);

	void onChannelFlushed(CodedChannel *channel);

	void onChannelUnlink(CodedChannel *channel);

	std::string mFileName;
	bool mRunning;
	bool mFrameByFrame;
	struct mp4_demux *mDemux;
	std::vector<RecordDemuxer::VideoMedia *> mVideoMedias;
	uint64_t mDuration;
	uint64_t mCurrentTime;
	float mSpeed;
	int mChannelsFlushing;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_RECORD_HPP_ */
