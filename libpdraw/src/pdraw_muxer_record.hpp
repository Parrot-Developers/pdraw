/**
 * Parrot Drones Awesome Video Viewer Library
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
		    IPdraw::IMuxer *muxer,
		    const std::string &fileName,
		    const struct pdraw_muxer_params *params);

	~RecordMuxer(void);

	int addInputMedia(
		Media *media,
		const struct pdraw_muxer_video_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	};

	int removeInputMedia(Media *media) override;

private:
	class Track {
	public:
		Track(std::string name,
		      uint32_t trackId,
		      uint32_t metaTrackId,
		      uint64_t trackTime,
		      int64_t lastSampleTs,
		      uint32_t timescale,
		      bool isDefault) :
				mName(name),
				trackId(trackId), metaTrackId(metaTrackId),
				trackTime(trackTime),
				lastSampleTs(lastSampleTs),
				mHasMetadataTrack(false), mTimescale(timescale),
				mIsDefault(isDefault)
		{
		}

		Track(void) :
				Track("Unknown",
				      0,
				      0,
				      0,
				      INT64_MAX,
				      DEFAULT_MP4_TIMESCALE,
				      false)
		{
		}

		std::string mName;
		uint32_t trackId;
		uint32_t metaTrackId;
		uint64_t trackTime;
		int64_t lastSampleTs;
		bool mHasMetadataTrack;
		uint32_t mTimescale;
		bool mIsDefault;
	};

	struct SessionMetaWriteTrackCbUserdata {
		RecordMuxer *muxer;
		uint32_t trackId;
	};

	int internalStart(void) override;

	int internalStop(void) override;

	void mergeSessionMetadata(void);

	int
	addTrackForMedia(Media *media,
			 uint64_t trackTime,
			 const struct pdraw_muxer_video_media_params *params);

	int addMetadataTrack(Track *ref, enum vmeta_frame_type metaType);

	int process(void) override;

	int processMedia(Media *media);

	int checkFreeSpace(size_t spaceNeeded, size_t &spaceLeft);

	int processFrame(CodedVideoMedia *media,
			 Track *track,
			 struct mbuf_coded_video_frame *frame);

	int processFrame(RawVideoMedia *media,
			 Track *track,
			 struct mbuf_raw_video_frame *frame);

	int setThumbnail(enum pdraw_muxer_thumbnail_type type,
			 const uint8_t *data,
			 size_t size) override;

	static void sessionMetaWriteFileCb(enum vmeta_record_type type,
					   const char *key,
					   const char *value,
					   void *userdata);

	static void sessionMetaWriteTrackCb(enum vmeta_record_type type,
					    const char *key,
					    const char *value,
					    void *userdata);

	static void *writerThread(void *arg);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

	static void callNoSpaceLeft(void *userdata);

	pthread_mutex_t mMp4Mutex;
	std::string mFileName;
	struct mp4_mux *mMux;
	time_t mMediaDate;
	std::unordered_map<Media *, Track> mTracks;
	uint8_t *mMetaBuffer;

	struct {
		pthread_t thread;
		std::atomic_bool started;
		struct mbox *mbox;
		pomp_loop *loop;
	} mWriterThread;

	size_t mFreeSpaceLeft;
	bool mNoSpaceLeft;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MUXER_RECORD_HPP_ */
