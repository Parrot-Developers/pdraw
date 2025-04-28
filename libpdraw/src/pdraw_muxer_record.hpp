/**
 * Parrot Drones Audio and Video Vector library
 * Record muxer
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

#ifndef _PDRAW_MUXER_RECORD_HPP_
#define _PDRAW_MUXER_RECORD_HPP_

#include "pdraw_muxer.hpp"
#include <futils/futils.h>

#include <map>
#include <string>
#include <unordered_map>

#include <libmp4.h>

namespace Pdraw {

#define DEFAULT_MP4_TIMESCALE 90000

class RecordMuxer : public Muxer {
public:
	RecordMuxer(Session *session,
		    Element::Listener *elementListener,
		    IPdraw::IMuxer::Listener *listener,
		    MuxerWrapper *wrapper,
		    const std::string &fileName,
		    const struct pdraw_muxer_params *params);

	~RecordMuxer(void);

	int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	};

	int removeInputMedia(Media *media) override;

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int forceSync(void) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

private:
	class Track {
	public:
		Track(const std::string &name,
		      Pdraw::Media::Type type,
		      uint32_t mediaId,
		      uint32_t trackId,
		      uint32_t metaTrackId,
		      uint32_t chaptersTrackId,
		      uint64_t trackTime,
		      int64_t firstSampleTs,
		      int64_t lastSampleTs,
		      uint32_t timescale,
		      bool isDefault,
		      const struct vmeta_session *sessionMeta) :
				mName(name),
				mMediaType(type), mMediaId(mediaId),
				mTrackId(trackId), mMetaTrackId(metaTrackId),
				mChaptersTrackId(chaptersTrackId),
				mTrackTime(trackTime),
				mFirstSampleTs(firstSampleTs),
				mLastSampleTs(lastSampleTs),
				mFirstSampleIndex(INT32_MAX),
				mFirstCaptureTs(INT64_MAX), mSampleCount(0),
				mHasMetadataTrack(false),
				mHasChaptersTrack(false), mTimescale(timescale),
				mIsDefault(isDefault), mSessionMeta({})
		{
			if (sessionMeta != nullptr)
				mSessionMeta = *sessionMeta;
		}

		Track(void) :
				Track("Unknown",
				      Pdraw::Media::Type::UNKNOWN,
				      0,
				      0,
				      0,
				      0,
				      0,
				      INT64_MAX,
				      INT64_MAX,
				      DEFAULT_MP4_TIMESCALE,
				      false,
				      {})
		{
		}

		std::string mName;
		Pdraw::Media::Type mMediaType;
		uint32_t mMediaId;
		uint32_t mTrackId;
		uint32_t mMetaTrackId;
		uint32_t mChaptersTrackId;
		uint64_t mTrackTime;
		int64_t mFirstSampleTs;
		int64_t mLastSampleTs;
		int32_t mFirstSampleIndex;
		int64_t mFirstCaptureTs;
		uint32_t mSampleCount;
		bool mHasMetadataTrack;
		bool mHasChaptersTrack;
		uint32_t mTimescale;
		bool mIsDefault;
		struct vmeta_session mSessionMeta;
	};

	struct SessionMetaWriteTrackCbUserdata {
		RecordMuxer *muxer;
		uint32_t trackId;
	};

	int internalStart(void) override;

	int internalStop(void) override;

	void mergeSessionMetadata(void);

	int addTrackForMedia(Media *media,
			     const struct pdraw_muxer_media_params *params);

	int addMetadataTrack(Track *ref, enum vmeta_frame_type metaType);

	int addChaptersTrack(void);

	void onChannelFlush(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	int process(void) override;

	int processMedia(int index);

	int checkFreeSpace(size_t spaceNeeded);

	int processFrame(Track *track, struct mbuf_coded_video_frame *frame);

	int processFrame(Track *track, struct mbuf_raw_video_frame *frame);

	int processFrame(Track *track, struct mbuf_audio_frame *frame);

	int setThumbnail(enum pdraw_muxer_thumbnail_type type,
			 const uint8_t *data,
			 size_t size) override;

	int addChapter(uint64_t timestamp, const char *name) override;

	int internalAddTrackForMedia(const struct add_track_params *params);

	int internalAddQueueEvtToLoop(Media::Type type, void *queue);

	int internalRemoveQueueEvtFromLoop(Media::Type type, void *queue);

	int internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size);

	int internalAddChapter(uint64_t timestamp, const char *name);

	int internalSetMetadata(uint32_t mediaId,
				const struct vmeta_session *metadata);

	int internalFlush(void);

	int internalStopThread(void);

	int
	internalSetDynParams(const struct pdraw_muxer_dyn_params *dyn_params);

	int internalForceSync(void);

	int internalSync(bool writeTables);

	static void sessionMetaWriteFileCb(enum vmeta_record_type type,
					   const char *key,
					   const char *value,
					   void *userdata);

	static void sessionMetaWriteTrackCb(enum vmeta_record_type type,
					    const char *key,
					    const char *value,
					    void *userdata);

	static void *writerThread(void *arg);

	static void syncCb(struct pomp_timer *timer, void *userdata);

	static void tablesSyncCb(struct pomp_timer *timer, void *userdata);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

	static void callCompleteStop(void *userdata);

	struct {
		std::string mLinkFile;
		std::string mTablesFile;
		uint32_t mSyncPeriodMs;
		bool enabled;
		bool metadataChanged;
		bool checkStorageUuid;
	} mRecovery;
	std::string mFileName;
	mode_t mFileMode;

	struct mp4_mux *mMux;
	uint64_t mMediaDate;
	int32_t mMediaDateGmtOff;
	std::unordered_map<unsigned int, Track> mTracks;
	std::map<uint64_t, std::string> mPendingChapters;
	bool mHasChaptersTrack;
	uint32_t mChaptersTrackId;
	uint8_t *mMetaBuffer;
	struct pdraw_muxer_stats mStats;

	struct {
		pthread_t thread;
		std::atomic_bool started;
		std::atomic_bool running;
		std::atomic_bool shouldStop;
		struct mbox *mbox;
		pomp_loop *loop;
	} mWriterThread;
	size_t mTablesSizeMb;
	size_t mFreeSpaceLeft;
	std::atomic_bool mPendingStop;
	struct pomp_timer *mTimerSync;
	uint32_t mTablesSyncPeriodMs;
	struct pomp_timer *mTimerTablesSync;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MUXER_RECORD_HPP_ */
