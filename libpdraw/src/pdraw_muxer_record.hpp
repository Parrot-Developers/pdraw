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

#include <string>
#include <unordered_map>

#include <libmp4.h>

namespace Pdraw {


class RecordMuxer : public Muxer {
public:
	RecordMuxer(Session *session,
		    Element::Listener *elementListener,
		    const std::string &fileName);

	~RecordMuxer(void);

	int addInputMedia(CodedVideoMedia *media) override;

private:
	class Track {
	public:
		Track(uint32_t trackId,
		      uint32_t metaTrackId,
		      uint64_t trackTime,
		      int64_t lastSampleTs) :
				trackId(trackId),
				metaTrackId(metaTrackId), trackTime(trackTime),
				lastSampleTs(lastSampleTs)
		{
		}

		Track(void) : Track(0, 0, 0, INT64_MAX) {}

		uint32_t trackId;
		uint32_t metaTrackId;
		uint64_t trackTime;
		int64_t lastSampleTs;
	};

	struct SessionMetaWriteTrackCbUserdata {
		RecordMuxer *muxer;
		uint32_t trackId;
	};

	int internalStart(void) override;

	int internalStop(void) override;

	void mergeSessionMetadata(void);

	int addTrackForMedia(CodedVideoMedia *media, uint64_t trackTime);

	int addMetadataTrack(Track *ref, enum vmeta_frame_type metaType);

	int process(void) override;

	int processMedia(CodedVideoMedia *media);

	int processFrame(CodedVideoMedia *media,
			 Track *track,
			 struct mbuf_coded_video_frame *frame);

	static void sessionMetaWriteFileCb(enum vmeta_record_type type,
					   const char *key,
					   const char *value,
					   void *userdata);

	static void sessionMetaWriteTrackCb(enum vmeta_record_type type,
					    const char *key,
					    const char *value,
					    void *userdata);

	std::string mFileName;
	struct mp4_mux *mMux;
	time_t mMediaDate;
	std::unordered_map<CodedVideoMedia *, Track> mTracks;
	bool mHasMetadataTrack;
	uint8_t *mMetaBuffer;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MUXER_RECORD_HPP_ */
