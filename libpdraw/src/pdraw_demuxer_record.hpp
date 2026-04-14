/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_demuxer.hpp"

#include <climits>
#include <memory>
#include <string>
#include <vector>

#include <aac/aac.h>
#include <h264/h264.h>
#include <h265/h265.h>
#include <libmp4.h>
#include <libpomp.h>

constexpr size_t DEMUXER_RECORD_CODED_VIDEO_MEDIA_OUTPUT_BUFFER_COUNT = 30;
constexpr size_t DEMUXER_RECORD_RAW_VIDEO_MEDIA_OUTPUT_BUFFER_COUNT = 30;
constexpr size_t DEMUXER_RECORD_AUDIO_MEDIA_OUTPUT_BUFFER_COUNT = 60;

namespace Pdraw {

class RecordDemuxer : public Demuxer {
public:
	RecordDemuxer(Session *session,
		      Element::Listener *elementListener,
		      Source::Listener *sourceListener,
		      DemuxerWrapper *wrapper,
		      IPdraw::IDemuxer::Listener *demuxerListener,
		      const std::string &fileName,
		      const struct pdraw_demuxer_params *params);

	~RecordDemuxer() override;

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

	int getChapterList(struct pdraw_chapter **chapterList,
			   size_t *chapterCount) override;

	uint64_t getDuration() const override
	{
		return mDuration;
	}

	uint64_t getCurrentTime() const override
	{
		return mCurrentTime;
	}

private:
	class DemuxerMedia : public Loggable {
	public:
		explicit DemuxerMedia(RecordDemuxer *demuxer);

		~DemuxerMedia() override;

		bool hasMedia(const Media *media) const;

		Media *getMedia(unsigned int index) const;

		unsigned int getMediaCount() const
		{
			return mMedias.size();
		}

		int setup(const struct mp4_track_info *tkinfo);

		void play();

		void previous();

		void next();

		void seek(int64_t delta, bool exact);

		void seekTo(uint64_t timestamp, bool exact);

		virtual void flush(bool discard = true);

		inline virtual void drain()
		{
			flush(false);
		}

		virtual void stop();

		bool isTrackEnabled() const
		{
			return mTrackEnabled;
		}

		unsigned int getTrackId() const
		{
			return mTrackId;
		}

		const std::string &getTrackName() const
		{
			return mTrackName;
		}

		Pdraw::Media::Type getMediaType() const
		{
			return mMediaType;
		}

		void setReference(bool isReference)
		{
			mIsReference = isReference;
		}

		bool isReference() const
		{
			return mIsReference;
		}

		void setRunning(bool running)
		{
			mRunning = running;
		}

		bool isRunning() const
		{
			return mRunning;
		}

		bool isSeeking() const
		{
			return mPendingSeek;
		}

		bool isPendingPlay() const
		{
			return mPendingPlay;
		}

		bool isPendingPause() const
		{
			return mPendingPause;
		}

		void setTearingDown()
		{
			mTearingDown = true;
		}

		bool isTearingDown() const
		{
			return mTearingDown;
		}

		void setDestroyAfterFlush(bool destroy)
		{
			mDestroyAfterFlush = destroy;
		}

		bool getDestroyAfterFlush() const
		{
			return mDestroyAfterFlush;
		}

		void sendDownstreamEvent(Channel::DownstreamEvent event);

		void channelFlushed(const Channel *channel);

		void channelDrained(const Channel *channel);

		void channelUnlink(const Channel *channel);

	protected:
		virtual int setupMedia(const struct mp4_track_info *tkinfo) = 0;

		virtual void teardownMedia();

		virtual int processSample(struct mp4_track_sample *sample,
					  bool *silent,
					  bool *retry,
					  bool *didSeek,
					  bool *waitFlush) = 0;

		void completeSeek();

		void completePlay();

		void completeFlush();

		std::vector<std::unique_ptr<Media>> mMedias{};
		RecordDemuxer *mDemuxer = nullptr;
		bool mTrackEnabled = false;
		unsigned int mTrackId = 0;
		std::string mTrackName{};
		Media::Type mMediaType = Media::Type::UNKNOWN;
		bool mIsReference = false;
		bool mFirstSample = true;
		unsigned int mSampleIndex = 0;
		std::string mMetadataMimeType{};
		std::vector<uint8_t> mMetadataBuffer{};
		uint32_t mTimescale = 0;
		int64_t mAvgOutputInterval = 0;
		uint64_t mLastSampleOutputTime = 0;
		int64_t mLastSampleDuration = 0;
		int64_t mLastOutputError = 0;
		bool mPendingSeek = false;
		bool mPendingPlay = false;
		bool mPendingPause = false;
		int64_t mPendingSeekTs = -1;
		bool mPendingSeekExact = false;
		bool mPendingSeekToPrevSample = false;
		bool mPendingSeekInPlay = false;
		bool mPendingSeekToNextSample = false;
		int mSeekResponse = 0;
		int mPlayResponse = 0;
		int mPauseResponse = 0;
		bool mRunning = false;
		bool mFlushing = false;
		bool mFlushDiscard = false;
		bool mTearingDown = false;
		unsigned int mFlushChannelCount = 0;
		bool mDestroyAfterFlush = false;

	private:
		static void timerCb(struct pomp_timer *timer, void *userdata);

		struct pomp_timer *mTimer;
	};

	class DemuxerCodedVideoMedia : public DemuxerMedia {
	public:
		explicit DemuxerCodedVideoMedia(RecordDemuxer *demuxer);

		~DemuxerCodedVideoMedia() override;

		void flush(bool discard = true) override;

		void stop() override;

	private:
		int setupMedia(const struct mp4_track_info *tkinfo) override;

		void teardownMedia() override;

		int processSample(struct mp4_track_sample *sample,
				  bool *silent,
				  bool *retry,
				  bool *didSeek,
				  bool *waitFlush) override;

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

		struct h264_reader *mH264Reader = nullptr;
		struct h265_reader *mH265Reader = nullptr;
		struct mbuf_coded_video_frame *mCurrentFrame = nullptr;
		struct mbuf_mem *mCurrentMem = nullptr;
		uint64_t mCurrentFrameCaptureTs = 0;
		uint64_t mDecodingTs = 0;
		uint64_t mDecodingTsInc = 0;
		uint64_t mFirstTs = UINT64_MAX;
		static const struct h264_ctx_cbs mH264ReaderCbs;
		static const struct h265_ctx_cbs mH265ReaderCbs;
	};

	class DemuxerRawVideoMedia : public DemuxerMedia {
	public:
		explicit DemuxerRawVideoMedia(RecordDemuxer *demuxer);

		~DemuxerRawVideoMedia() override;

		void flush(bool discard = true) override;

		void stop() override;

	private:
		int setupMedia(const struct mp4_track_info *tkinfo) override;

		void teardownMedia() override;

		int processSample(struct mp4_track_sample *sample,
				  bool *silent,
				  bool *retry,
				  bool *didSeek,
				  bool *waitFlush) override;

		RawVideoMedia *mRawVideoMedia = nullptr;
		struct mbuf_raw_video_frame *mCurrentFrame = nullptr;
		struct mbuf_mem *mCurrentMem = nullptr;
		uint64_t mCurrentFrameCaptureTs = 0;
		uint64_t mDecodingTs = 0;
		uint64_t mDecodingTsInc = 0;
		uint64_t mFirstTs = UINT64_MAX;
	};

	class DemuxerAudioMedia : public DemuxerMedia {
	public:
		explicit DemuxerAudioMedia(RecordDemuxer *demuxer);

		~DemuxerAudioMedia() override;

		void flush(bool discard = true) override;

		void stop() override;

	private:
		int setupMedia(const struct mp4_track_info *tkinfo) override;

		void teardownMedia() override;

		int processSample(struct mp4_track_sample *sample,
				  bool *silent,
				  bool *retry,
				  bool *didSeek,
				  bool *waitFlush) override;

		AudioMedia *mAudioMedia = nullptr;
		struct mbuf_audio_frame *mCurrentFrame = nullptr;
		struct mbuf_mem *mCurrentMem = nullptr;
		uint64_t mCurrentFrameCaptureTs = 0;
		uint64_t mDecodingTs = 0;
		uint64_t mDecodingTsInc = 0;
		uint64_t mFirstTs = UINT64_MAX;
	};

	int completeStart();

	static void idleCompleteStart(void *userdata);

	int selectReferenceTrack();

	int processSelectedMedias();

	int internalPlay(float speed);

	int internalPause();

	int fetchSessionMetadata(unsigned int trackId,
				 struct vmeta_session *meta);

	void setRunning(bool running);

	void onMediaSeekComplete(int seekResponse);

	void onMediaPlayComplete(int playResponse);

	void onMediaRunningStateChanged();

	int flush(bool discard = true) override;

	void completeFlush();

	void completeTeardown();

	void destroyAllMedias();

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	static bool isMediaTrack(const struct mp4_track_info *tkinfo,
				 char **keys,
				 char **values,
				 int count);

	std::string mFileName{};
	bool mRunning = false;
	/* Whether mRunning has been set to true at least once;
	 * needed to handle the start_in_pause (PAUSE_NEXT) feature */
	bool mWasRunningOnce = false;
	bool mPendingSeek = false;
	int mSeekResponse = 0;
	int mPlayResponse = 0;
	int mPauseResponse = 0;
	bool mFrameByFrame = false;
	struct mp4_demux *mDemux = nullptr;
	std::vector<std::unique_ptr<RecordDemuxer::DemuxerMedia>> mMedias{};
	uint64_t mDuration = 0;
	uint64_t mCurrentTime = 0;
	float mSpeed = 1.f;
	int mChannelsFlushing = 0;
	enum pdraw_playback_mode mPlaybackMode = PDRAW_PLAYBACK_MODE_REALTIME;
};

} /* namespace Pdraw */
