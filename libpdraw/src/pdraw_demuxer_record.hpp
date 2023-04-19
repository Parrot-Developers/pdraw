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
		      Source::Listener *sourceListener,
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
	class DemuxerMedia : public Loggable {
	public:
		DemuxerMedia(RecordDemuxer *demuxer);

		virtual ~DemuxerMedia(void);

		int setup(struct mp4_track_info *tkinfo);

		void play(void);

		void previous(void);

		void next(void);

		void seek(int64_t delta, bool exact);

		void seekTo(uint64_t timestamp, bool exact);

		virtual void stop(void);

	protected:
		virtual int setupMedia(struct mp4_track_info *tkinfo) = 0;


		virtual int processSample(struct mp4_track_sample *sample,
					  bool *silent,
					  bool *retry,
					  bool *didSeek,
					  bool *waitFlush) = 0;

		virtual void
		sendDownstreamEvent(Channel::DownstreamEvent event) = 0;

		RecordDemuxer *mDemuxer;
		unsigned int mTrackId;
		Pdraw::Media::Type mMediaType;
		bool mFirstSample;
		unsigned int mSampleIndex;
		char *mMetadataMimeType;
		size_t mMetadataBufferSize;
		uint8_t *mMetadataBuffer;
		uint32_t mTimescale;
		int64_t mAvgOutputInterval;
		uint64_t mLastSampleOutputTime;
		int64_t mLastSampleDuration;
		int64_t mLastOutputError;
		int64_t mPendingSeekTs;
		bool mPendingSeekExact;
		bool mPendingSeekToPrevSample;
		bool mPendingSeekToNextSample;
		int mSeekResponse;

	private:
		static void timerCb(struct pomp_timer *timer, void *userdata);

		struct pomp_timer *mTimer;
	};

	class DemuxerCodedVideoMedia : public DemuxerMedia {
	public:
		DemuxerCodedVideoMedia(RecordDemuxer *demuxer);

		~DemuxerCodedVideoMedia(void);

		void stop(void) override;

	private:
		int setupMedia(struct mp4_track_info *tkinfo) override;


		int processSample(struct mp4_track_sample *sample,
				  bool *silent,
				  bool *retry,
				  bool *didSeek,
				  bool *waitFlush) override;

		void
		sendDownstreamEvent(Channel::DownstreamEvent event) override;

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

		struct h264_reader *mH264Reader;
		struct h265_reader *mH265Reader;
		CodedVideoMedia **mCodedVideoMedias;
		unsigned int mNbCodedVideoMedias;
		struct mbuf_coded_video_frame *mCurrentFrame;
		struct mbuf_mem *mCurrentMem;
		uint64_t mCurrentFrameCaptureTs;
		uint64_t mDecodingTs;
		uint64_t mDecodingTsInc;
		uint64_t mFirstTs;
		static const struct h264_ctx_cbs mH264ReaderCbs;
		static const struct h265_ctx_cbs mH265ReaderCbs;
	};

	class DemuxerRawVideoMedia : public DemuxerMedia {
	public:
		DemuxerRawVideoMedia(RecordDemuxer *demuxer);

		~DemuxerRawVideoMedia(void);

		void stop(void) override;

	private:
		int setupMedia(struct mp4_track_info *tkinfo) override;


		int processSample(struct mp4_track_sample *sample,
				  bool *silent,
				  bool *retry,
				  bool *didSeek,
				  bool *waitFlush) override;

		void
		sendDownstreamEvent(Channel::DownstreamEvent event) override;

		RawVideoMedia *mRawVideoMedia;
		struct mbuf_raw_video_frame *mCurrentFrame;
		struct mbuf_mem *mCurrentMem;
		uint64_t mCurrentFrameCaptureTs;
		uint64_t mDecodingTs;
		uint64_t mDecodingTsInc;
		uint64_t mFirstTs;
	};

	int fetchSessionMetadata(unsigned int trackId,
				 struct vmeta_session *meta);

	int flush(void);

	void completeFlush(void);

	void completeTeardown(void);

	void onChannelFlushed(Channel *channel);

	void onChannelUnlink(Channel *channel);

	static bool isMediaTrack(struct mp4_track_info *tkinfo,
				 char **keys,
				 char **values,
				 int count);

	std::string mFileName;
	bool mRunning;
	bool mFrameByFrame;
	struct mp4_demux *mDemux;
	std::vector<RecordDemuxer::DemuxerMedia *> mMedias;
	uint64_t mDuration;
	uint64_t mCurrentTime;
	float mSpeed;
	int mChannelsFlushing;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_RECORD_HPP_ */
