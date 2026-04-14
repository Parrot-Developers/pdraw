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

#pragma once

#include "pdraw_muxer_record_isobmff.hpp"
#include "pdraw_muxer_record_media.hpp"
#include <futils/futils.h>
#include <media-buffers/mbuf_queue.hpp>

#include <array>
#include <string>

#include <libmp4.h>

namespace Pdraw {


class IsobmffRecordMuxer::IsobmffMuxerMedia : public RecordMuxer::MuxerMedia {
public:
	IsobmffMuxerMedia(IsobmffRecordMuxer *muxer,
			  const MuxerMediaConfig &cfg);

	~IsobmffMuxerMedia() override = default;

	int setup(const struct pdraw_media_info *mediaInfo,
		  const struct pdraw_muxer_media_params *params) override = 0;

	int process() override = 0;

	int addMetadata(enum vmeta_frame_type metaType);

	int writeRecordingMetadata(struct vmeta_session *session);

	void setChapters(uint32_t id)
	{
		mChaptersTrackId = id;
		mHasChapters = true;
	}

	uint32_t getTrackId() const
	{
		return mTrackId;
	}

	bool hasChapters() const
	{
		return mHasChapters;
	}

	uint64_t getMediaTime() const
	{
		return mMediaTime;
	}

	bool isDefault() const
	{
		return mIsDefault;
	}

	int32_t getFirstSampleIndex() const
	{
		return mFirstSampleIndex;
	}

	int64_t getFirstCaptureTs() const
	{
		return mFirstCaptureTs;
	}

protected:
	IsobmffRecordMuxer *mIsoMuxer = nullptr;
	uint32_t mTrackId = 0;
	uint32_t mMetaTrackId = 0;
	uint32_t mChaptersTrackId = 0;
	uint64_t mMediaTime = 0;
	int64_t mFirstSampleTs = INT64_MAX;
	int64_t mLastSampleTs = INT64_MAX;
	int32_t mFirstSampleIndex = INT32_MAX;
	int64_t mFirstCaptureTs = INT64_MAX;
	bool mHasMetadata = false;
	bool mHasChapters = false;
	uint32_t mSampleCount = 0;
	uint32_t mTimescale = DEFAULT_MP4_TIMESCALE;
	bool mIsDefault = false;

private:
	static void sessionMetaWriteMediaCb(enum vmeta_record_type type,
					    const char *key,
					    const char *value,
					    void *userdata);
};


class IsobmffRecordMuxer::IsobmffMuxerCodedVideoMedia
		: public IsobmffRecordMuxer::IsobmffMuxerMedia {
public:
	IsobmffMuxerCodedVideoMedia(IsobmffRecordMuxer *muxer,
				    const MuxerMediaConfig &cfg);

	int setup(const struct pdraw_media_info *mediaInfo,
		  const struct pdraw_muxer_media_params *params) override;

	int process() override;

protected:
	int processFrame(struct mbuf_coded_video_frame *frame);

	std::array<const void *, MAX_NALUS_PER_FRAME> mNalusPtr{};
	std::array<size_t, MAX_NALUS_PER_FRAME> mNalusSize{};
};


class IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia
		: public IsobmffRecordMuxer::IsobmffMuxerMedia {
public:
	IsobmffMuxerRawVideoMedia(IsobmffRecordMuxer *muxer,
				  const MuxerMediaConfig &cfg);

	int setup(const struct pdraw_media_info *mediaInfo,
		  const struct pdraw_muxer_media_params *params) override;

	int process() override;

protected:
	int processFrame(struct mbuf_raw_video_frame *frame);
};


class IsobmffRecordMuxer::IsobmffMuxerAudioMedia
		: public IsobmffRecordMuxer::IsobmffMuxerMedia {
public:
	IsobmffMuxerAudioMedia(IsobmffRecordMuxer *muxer,
			       const MuxerMediaConfig &cfg);

	int setup(const struct pdraw_media_info *mediaInfo,
		  const struct pdraw_muxer_media_params *params) override;

	int process() override;

protected:
	int processFrame(struct mbuf_audio_frame *frame);
};

} /* namespace Pdraw */
