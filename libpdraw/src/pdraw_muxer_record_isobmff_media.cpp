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

#define ULOG_TAG pdraw_recmux_isobmff_media
#include <ulog.h>

#include "pdraw_muxer_record_isobmff.hpp"
#include "pdraw_muxer_record_isobmff_media.hpp"
#include "pdraw_session.hpp"

#include <array>
#include <memory>

#include <time.h>

#include <futils/futils.h>
#include <libmp4.h>
#include <media-buffers/mbuf_coded_video_frame.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


#define PDRAW_CHECK_MUXER_WRITER_THREAD(expectWriter)                          \
	mIsoMuxer->logThreadCheckWarning(__func__, expectWriter)


IsobmffRecordMuxer::IsobmffMuxerMedia::IsobmffMuxerMedia(
	IsobmffRecordMuxer *muxer,
	const MuxerMediaConfig &cfg) :
		RecordMuxer::MuxerMedia(muxer, cfg),
		mIsoMuxer(muxer), mTrackId(cfg.trackId),
		mMetaTrackId(cfg.metaTrackId),
		mChaptersTrackId(cfg.chaptersTrackId),
		mMediaTime(cfg.trackTime), mFirstSampleTs(cfg.firstSampleTs),
		mLastSampleTs(cfg.lastSampleTs), mTimescale(cfg.timescale),
		mIsDefault(cfg.isDefault)
{
	std::string name = muxer->getName() + "#IsobmffMuxerMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerMedia::addMetadata(
	enum vmeta_frame_type metaType)
{
	int res;
	const char *mimeType = nullptr;
	const char *contentEncoding = nullptr;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mHasMetadata, EALREADY);

	switch (metaType) {
	case VMETA_FRAME_TYPE_V2:
		mimeType = VMETA_FRAME_V2_MIME_TYPE;
		contentEncoding = VMETA_FRAME_V2_CONTENT_ENCODING;
		break;
	case VMETA_FRAME_TYPE_V3:
		mimeType = VMETA_FRAME_V3_MIME_TYPE;
		contentEncoding = VMETA_FRAME_V3_CONTENT_ENCODING;
		break;
	case VMETA_FRAME_TYPE_PROTO:
		mimeType = VMETA_FRAME_PROTO_MIME_TYPE;
		contentEncoding = VMETA_FRAME_PROTO_CONTENT_ENCODING;
		break;
	default:
		PDRAW_LOGE("%s: unsupported metadata type '%s'",
			   __func__,
			   vmeta_frame_type_str(metaType));
		return -ENOSYS;
	}

	/* Add a new metadata track */
	std::string name = mTrackName + "::TimedMetadata";
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_METADATA,
		.name = name.c_str(),
		.enabled = false,
		.in_movie = false,
		.in_preview = false,
		.timescale = mTimescale,
		.creation_time = mMediaTime,
		.modification_time = mMediaTime,
	};
	res = mp4_mux_add_track(mIsoMuxer->mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	mMetaTrackId = (uint32_t)res;

	/* Set the metadata mime type */
	res = mp4_mux_track_set_metadata_mime_type(
		mIsoMuxer->mMux, mMetaTrackId, contentEncoding, mimeType);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_metadata_mime_type", -res);
		mMetaTrackId = 0;
		return res;
	}

	/* Add track reference */
	res = mp4_mux_add_ref_to_track(
		mIsoMuxer->mMux, mMetaTrackId, getTrackId());
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track", -res);
		mMetaTrackId = 0;
		return res;
	}

	mHasMetadata = true;

	/* Add an initial empty sample if needed to ensure that the newly-
	 * created metadata track is synchronized with its referenced track. */
	if ((mLastSampleTs != INT64_MAX) && (mFirstSampleTs != INT64_MAX) &&
	    (mLastSampleTs > mFirstSampleTs)) {
		const uint64_t emptyCookie = VMETA_FRAME_PROTO_EMPTY_COOKIE;
		const void *emptyCookiePtr = &emptyCookie;
		struct mp4_mux_sample emptyMetaSample = {
			.buffer = static_cast<const uint8_t *>(emptyCookiePtr),
			.len = sizeof(emptyCookie),
			.sync = 1,
			.dts = mFirstSampleTs,
		};
		PDRAW_LOGI("%s(%s): add first metadata sample",
			   __func__,
			   mTrackName.c_str());
		res = mp4_mux_track_add_sample(
			mIsoMuxer->mMux, mMetaTrackId, &emptyMetaSample);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
			return res;
		}
	}

	return 0;
}


int IsobmffRecordMuxer::IsobmffMuxerMedia::writeRecordingMetadata(
	const struct vmeta_session *session)
{
	return vmeta_session_recording_write(
		session, &IsobmffMuxerMedia::sessionMetaWriteMediaCb, this);
}


void IsobmffRecordMuxer::IsobmffMuxerMedia::sessionMetaWriteMediaCb(
	[[maybe_unused]] enum vmeta_record_type type,
	const char *key,
	const char *value,
	void *userdata)
{
	auto *self =
		static_cast<IsobmffRecordMuxer::IsobmffMuxerMedia *>(userdata);

	if (self == nullptr || self->mIsoMuxer == nullptr)
		return;

	int res = mp4_mux_add_track_metadata(
		self->mIsoMuxer->mMux, self->mTrackId, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_track_metadata", -res);
}


IsobmffRecordMuxer::IsobmffMuxerCodedVideoMedia::IsobmffMuxerCodedVideoMedia(
	IsobmffRecordMuxer *muxer,
	const MuxerMediaConfig &cfg) :
		IsobmffMuxerMedia(muxer, cfg)
{
	std::string name = muxer->getName() + "#IsobmffMuxerCodedVideoMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerCodedVideoMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	struct mp4_video_decoder_config cfg = {};
	const auto &video = mediaInfo->video.coded;

	cfg.width = video.info.resolution.width;
	cfg.height = video.info.resolution.height;

	if (video.format.encoding == VDEF_ENCODING_H264) {
		cfg.codec = MP4_VIDEO_CODEC_AVC;
		cfg.avc.c_sps = video.h264.sps;
		cfg.avc.sps_size = video.h264.spslen;
		cfg.avc.c_pps = video.h264.pps;
		cfg.avc.pps_size = video.h264.ppslen;
	} else if (video.format.encoding == VDEF_ENCODING_H265) {
		cfg.codec = MP4_VIDEO_CODEC_HEVC;
		cfg.hevc.c_vps = video.h265.vps;
		cfg.hevc.vps_size = video.h265.vpslen;
		cfg.hevc.c_sps = video.h265.sps;
		cfg.hevc.sps_size = video.h265.spslen;
		cfg.hevc.c_pps = video.h265.pps;
		cfg.hevc.pps_size = video.h265.ppslen;
	}

	return mp4_mux_track_set_video_decoder_config(
		mIsoMuxer->mMux, mTrackId, &cfg);
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerCodedVideoMedia::process()
{
	struct mbuf_coded_video_frame *frame = nullptr;
	int res = mQueue->popFrame(&frame);
	if (res < 0)
		return res;

	res = processFrame(frame);
	mbuf_coded_video_frame_unref(frame);
	return res;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerCodedVideoMedia::processFrame(
	struct mbuf_coded_video_frame *frame)
{
	int res;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	struct vmeta_frame *metadata = nullptr;
	struct vdef_coded_frame info = {};
	const CodedVideoMedia::Frame *meta;
	struct mp4_mux_scattered_sample sample = {};
	const void *aData;
	uint64_t spaceNeeded = 0;
	int naluCount;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	if (mIsoMuxer->mUnrecoverableError)
		return -EPROTO;

	if (mIsoMuxer->mMux == nullptr || !mIsoMuxer->isThreadRunning())
		return -EPROTO;

	naluCount = mbuf_coded_video_frame_get_nalu_count(frame);
	if (naluCount < 0) {
		res = naluCount;
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		goto out;
	}

	if ((size_t)naluCount > mNalusPtr.size()) {
		PDRAW_LOGE("too many NALUs: %d (max %zu)",
			   naluCount,
			   mNalusPtr.size());
		res = -ENOBUFS;
		goto out;
	}
	sample.nbuffers = naluCount;

	for (int i = 0; i < sample.nbuffers; i++) {
		struct vdef_nalu nalu;
		const void *naluRawPtr = nullptr;
		res = mbuf_coded_video_frame_get_nalu(
			frame, i, &naluRawPtr, &nalu);
		mNalusPtr[i] = static_cast<const uint8_t *>(naluRawPtr);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_coded_video_frame_get_nalu(%d)", -res, i);
			goto out;
		}
		mNalusSize[i] = nalu.size;
		spaceNeeded += nalu.size;
	}

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto out;
	}

	res = mbuf_coded_video_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = static_cast<const CodedVideoMedia::Frame *>(aData);

	if (mSampleCount == 0) {
		if (!meta->isSync) {
			PDRAW_LOGI("frame %u skipped (not a sync point)",
				   info.info.index);
			goto out;
		} else {
			PDRAW_LOGI("frame %u (%s) recorded -> segment started",
				   info.info.index,
				   vdef_coded_frame_type_to_str(info.type));
		}
	}

	if ((mFirstSampleIndex == INT32_MAX) &&
	    (mFirstCaptureTs == INT64_MAX) &&
	    !(info.info.flags & VDEF_FRAME_FLAG_FAKE)) {
		/* First valid frame */
		mFirstSampleIndex = mSampleCount;
		mFirstCaptureTs = info.info.capture_timestamp;
	}

	/* Add a video sample to the MP4 muxer */
	sample.buffers = mNalusPtr.data();
	sample.len = mNalusSize.data();
	sample.sync = meta->isSync;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((mLastSampleTs != INT64_MAX) && (sample.dts == mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   mLastSampleTs);
		sample.dts = mLastSampleTs + 1;
	} else if ((mLastSampleTs != INT64_MAX) &&
		   (sample.dts < mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   mLastSampleTs,
			   sample.dts);
		sample.dts = mLastSampleTs + 1;
	}
	if (mFirstSampleTs == INT64_MAX)
		mFirstSampleTs = sample.dts;
	mLastSampleTs = sample.dts;

	res = mIsoMuxer->ensureFreeSpace(spaceNeeded);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("ensureFreeSpace", -res);
		mIsoMuxer->onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_scattered_sample(
		mIsoMuxer->mMux, mTrackId, &sample);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_add_scattered_sample", -res);
		if (res == -ENOENT) {
			/* File has been removed */
			mIsoMuxer->onUnrecoverableError(res);
		}
		goto out;
	} else {
		mSampleCount++;
		mIsoMuxer->mStats.record.coded_video_frames++;
	}

	res = mbuf_coded_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, skip to the end */
		res = 0;
		goto out;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);
		goto out;
	}

	if (!mHasMetadata) {
		int err = addMetadata(metadata->type);
		if (err < 0)
			PDRAW_LOG_ERRNO("addMetadata", -err);
	}
	if (mMetaTrackId) {
		/* Add a metadata sample to the MP4 muxer */
		const uint8_t *metaContent = nullptr;
		size_t metaLen = 0;
		if (metadata->type != VMETA_FRAME_TYPE_PROTO) {
			struct vmeta_buffer metaBuf;
			vmeta_buffer_set_data(&metaBuf,
					      mIsoMuxer->mMetaBuffer.data(),
					      mIsoMuxer->mMetaBuffer.size(),
					      0);
			res = vmeta_frame_write(&metaBuf, metadata);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_write", -res);
				goto out;
			}
			metaContent = metaBuf.data;
			metaLen = metaBuf.pos;
		} else {
			res = vmeta_frame_proto_get_buffer(
				metadata, &metaContent, &metaLen);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_proto_get_buffer",
						-res);
			}
		}
		if ((metaContent != nullptr) && (metaLen > 0)) {
			struct mp4_mux_sample metaSample = {
				.buffer = metaContent,
				.len = metaLen,
				.sync = 1,
				.dts = sample.dts,
			};
			res = mp4_mux_track_add_sample(
				mIsoMuxer->mMux, mMetaTrackId, &metaSample);
			if (res < 0)
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
		}
		if (metadata->type == VMETA_FRAME_TYPE_PROTO) {
			vmeta_frame_proto_release_buffer(metadata, metaContent);
		}
	}

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	for (int i = 0; i < naluCount; i++) {
		if (mNalusPtr[i] == nullptr)
			continue;
		if (frame) {
			int err = mbuf_coded_video_frame_release_nalu(
				frame, i, mNalusPtr[i]);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_release_nalu",
					-err);
		}
		mNalusPtr[i] = nullptr;
		mNalusSize[i] = 0;
	}

	return res;
}


IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia::IsobmffMuxerRawVideoMedia(
	IsobmffRecordMuxer *muxer,
	const MuxerMediaConfig &cfg) :
		IsobmffMuxerMedia(muxer, cfg)
{
	std::string name = muxer->getName() + "#IsobmffMuxerRawVideoMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	unique_c_ptr<char> format_str;
	unique_c_ptr<char> info_str;
	unique_c_ptr<char> mime;
	const auto &raw = mediaInfo->video.raw;
	int res;

	{
		char *p = nullptr;
		res = vdef_raw_format_to_csv(&raw.format, &p);
		if (res < 0) {
			PDRAW_LOG_ERRNO("vdef_raw_format_to_csv", -res);
			return res;
		}
		format_str.reset(p);
	}
	{
		char *p = nullptr;
		res = vdef_format_info_to_csv(&raw.info, &p);
		if (res < 0) {
			PDRAW_LOG_ERRNO("vdef_format_info_to_csv", -res);
			return res;
		}
		info_str.reset(p);
	}
	{
		char *p = nullptr;
		res = asprintf(&p,
			       VDEF_RAW_MIME_TYPE ";%s;%s",
			       format_str.get(),
			       info_str.get());
		if (res < 0)
			return -ENOMEM;
		mime.reset(p);
	}

	return mp4_mux_track_set_metadata_mime_type(
		mIsoMuxer->mMux, mTrackId, "", mime.get());
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia::process()
{
	struct mbuf_raw_video_frame *frame = nullptr;
	int res = mQueue->popFrame(&frame);
	if (res < 0)
		return res;

	res = processFrame(frame);
	mbuf_raw_video_frame_unref(frame);
	return res;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerRawVideoMedia::processFrame(
	struct mbuf_raw_video_frame *frame)
{
	int res = 0;
	const uint8_t *buf = nullptr;
	size_t len;
	struct vdef_raw_frame info;
	struct mp4_mux_sample sample;
	struct vmeta_frame *metadata = nullptr;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	[[maybe_unused]] const RawVideoMedia::Frame *meta;
	const void *aData;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	if (mIsoMuxer->mUnrecoverableError)
		return -EPROTO;

	if (mIsoMuxer->mMux == nullptr || !mIsoMuxer->isThreadRunning())
		return -EPROTO;

	res = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	if ((mFirstSampleIndex == INT32_MAX) &&
	    (mFirstCaptureTs == INT64_MAX) &&
	    !(info.info.flags & VDEF_FRAME_FLAG_FAKE)) {
		/* First valid frame */
		mFirstSampleIndex = mSampleCount;
		mFirstCaptureTs = info.info.capture_timestamp;
	}

	{
		const void *rawBuf = nullptr;
		res = mbuf_raw_video_frame_get_packed_buffer(
			frame, &rawBuf, &len);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_get_packed_buffer", -res);
			goto out;
		}
		buf = static_cast<const uint8_t *>(rawBuf);
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME, &ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = static_cast<const RawVideoMedia::Frame *>(aData);

	/* Add a video sample to the MP4 muxer */
	sample.buffer = buf;
	sample.len = len;
	sample.sync = 1;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((mLastSampleTs != INT64_MAX) && (sample.dts == mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   mLastSampleTs);
		sample.dts = mLastSampleTs + 1;
	} else if ((mLastSampleTs != INT64_MAX) &&
		   (sample.dts < mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   mLastSampleTs,
			   sample.dts);
		sample.dts = mLastSampleTs + 1;
	}
	if (mFirstSampleTs == INT64_MAX)
		mFirstSampleTs = sample.dts;
	mLastSampleTs = sample.dts;

	res = mIsoMuxer->ensureFreeSpace(len);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("ensureFreeSpace", -res);
		mIsoMuxer->onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_sample(mIsoMuxer->mMux, mTrackId, &sample);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
	else {
		mSampleCount++;
		mIsoMuxer->mStats.record.raw_video_frames++;
	}

	res = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, skip to the end */
		res = 0;
		goto out;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		goto out;
	}

	if (!mHasMetadata) {
		int err = addMetadata(metadata->type);
		if (err < 0)
			PDRAW_LOG_ERRNO("addMetadata", -err);
	}
	if (mMetaTrackId) {
		/* Add a metadata sample to the MP4 muxer */
		const uint8_t *metaContent = nullptr;
		size_t metaLen = 0;
		if (metadata->type != VMETA_FRAME_TYPE_PROTO) {
			struct vmeta_buffer metaBuf;
			vmeta_buffer_set_data(&metaBuf,
					      mIsoMuxer->mMetaBuffer.data(),
					      mIsoMuxer->mMetaBuffer.size(),
					      0);
			res = vmeta_frame_write(&metaBuf, metadata);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_write", -res);
				goto out;
			}
			metaContent = metaBuf.data;
			metaLen = metaBuf.pos;
		} else {
			res = vmeta_frame_proto_get_buffer(
				metadata, &metaContent, &metaLen);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_proto_get_buffer",
						-res);
			}
		}
		if ((metaContent != nullptr) && (metaLen > 0)) {
			struct mp4_mux_sample metaSample = {
				.buffer = metaContent,
				.len = metaLen,
				.sync = 1,
				.dts = sample.dts,
			};
			res = mp4_mux_track_add_sample(
				mIsoMuxer->mMux, mMetaTrackId, &metaSample);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
			}
		}
		if (metadata->type == VMETA_FRAME_TYPE_PROTO)
			vmeta_frame_proto_release_buffer(metadata, metaContent);
	}

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (buf)
		mbuf_raw_video_frame_release_packed_buffer(frame, buf);

	return res;
}


IsobmffRecordMuxer::IsobmffMuxerAudioMedia::IsobmffMuxerAudioMedia(
	IsobmffRecordMuxer *muxer,
	const MuxerMediaConfig &cfg) :
		IsobmffMuxerMedia(muxer, cfg)
{
	std::string name = muxer->getName() + "#IsobmffMuxerAudioMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerAudioMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	const auto &audio = mediaInfo->audio;

	if (audio.format.encoding != ADEF_ENCODING_AAC_LC) {
		PDRAW_LOGE("unsupported encoding (only AAC-LC is supported)");
		return -EINVAL;
	}

	return mp4_mux_track_set_audio_specific_config(
		mIsoMuxer->mMux,
		mTrackId,
		audio.aac_lc.asc,
		audio.aac_lc.asclen,
		audio.format.channel_count,
		1024,
		static_cast<float>(audio.format.sample_rate));
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerAudioMedia::process()
{
	struct mbuf_audio_frame *frame = nullptr;
	int res = mQueue->popFrame(&frame);
	if (res < 0)
		return res;

	res = processFrame(frame);
	mbuf_audio_frame_unref(frame);
	return res;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::IsobmffMuxerAudioMedia::processFrame(
	struct mbuf_audio_frame *frame)
{
	int res = 0;
	const uint8_t *buf = nullptr;
	size_t len;
	struct adef_frame info;
	struct mp4_mux_sample sample;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	[[maybe_unused]] const AudioMedia::Frame *meta;
	const void *aData;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	if (mIsoMuxer->mUnrecoverableError)
		return -EPROTO;

	if (mIsoMuxer->mMux == nullptr || !mIsoMuxer->isThreadRunning())
		return -EPROTO;

	res = mbuf_audio_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -res);
		return res;
	}

	{
		const void *rawBuf = nullptr;
		res = mbuf_audio_frame_get_buffer(frame, &rawBuf, &len);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_audio_frame_get_buffer", -res);
			goto out;
		}
		buf = static_cast<const uint8_t *>(rawBuf);
	}

	res = mbuf_audio_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_ancillary_data", -res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = static_cast<const AudioMedia::Frame *>(aData);

	/* Add an audio sample to the MP4 muxer */
	sample.buffer = buf;
	sample.len = len;
	sample.sync = 1;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((mLastSampleTs != INT64_MAX) && (sample.dts == mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   mLastSampleTs);
		sample.dts = mLastSampleTs + 1;
	} else if ((mLastSampleTs != INT64_MAX) &&
		   (sample.dts < mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   mLastSampleTs,
			   sample.dts);
		sample.dts = mLastSampleTs + 1;
	}
	if (mFirstSampleTs == INT64_MAX)
		mFirstSampleTs = sample.dts;
	mLastSampleTs = sample.dts;

	res = mIsoMuxer->ensureFreeSpace(len);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("ensureFreeSpace", -res);
		mIsoMuxer->onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_sample(mIsoMuxer->mMux, mTrackId, &sample);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
	else {
		mSampleCount++;
		mIsoMuxer->mStats.record.audio_frames++;
	}

out:
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (buf)
		mbuf_audio_frame_release_buffer(frame, buf);

	return res;
}

} /* namespace Pdraw */
