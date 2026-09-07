/**
 * Parrot Drones Audio and Video Vector library
 * Recording demuxer raw video media
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

#define ULOG_TAG pdraw_dmxrec
#include <ulog.h>

#include "pdraw_demuxer_record.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <string>

#include <media-buffers/mbuf_ancillary_data.h>
#include <video-streaming/vstrm.h>

namespace Pdraw {


RecordDemuxer::DemuxerRawVideoMedia::DemuxerRawVideoMedia(
	RecordDemuxer *demuxer) :
		DemuxerMedia(demuxer)

{
	mMediaType = Media::Type::RAW_VIDEO;
	std::string name = demuxer->getName() + "#DemuxerRawVideoMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerRawVideoMedia::~DemuxerRawVideoMedia()
{
	teardownMedia();
}


void RecordDemuxer::DemuxerRawVideoMedia::flush(bool discard)
{
	if (mCurrentFrame != nullptr) {
		int err = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		int err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}

	DemuxerMedia::flush(discard);
}


void RecordDemuxer::DemuxerRawVideoMedia::stop()
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}
	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}

	DemuxerMedia::stop();
}


void RecordDemuxer::DemuxerRawVideoMedia::teardownMedia()
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}

	DemuxerMedia::teardownMedia();
	mRawVideoMedia = nullptr;
}


int RecordDemuxer::DemuxerRawVideoMedia::setupMedia(
	const struct mp4_track_info *tkinfo)
{
	int ret;
	unsigned int count = 0;
	char **keys = nullptr;
	char **values = nullptr;
	const char *formatStr = nullptr;
	const char *resolutionStr = nullptr;
	[[maybe_unused]] const char *dataInterpretationStr = nullptr;
	struct vdef_raw_format format = {};
	struct vdef_format_info info = {};
	bool unknownFormat = true;
	ssize_t ret2;
	size_t capacity;

	/* Get the track-level session metadata */
	ret = mp4_demux_get_track_metadata_strings(
		mDemuxer->mDemux, mTrackId, &count, &keys, &values);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_metadata_strings", -ret);
		return ret;
	}

	/* Get regis-specific session metadata strings */
	for (unsigned int i = 0; i < count; i++) {
		if (strcmp(keys[i], "com.parrot.regis.format") == 0) {
			formatStr = values[i];
		} else if (strcmp(keys[i], "com.parrot.regis.resolution") ==
			   0) {
			resolutionStr = values[i];
		} else if (strcmp(keys[i],
				  "com.parrot.regis.first_timestamp") == 0) {
			char *endptr = nullptr;
			long long int parsedint =
				strtol(values[i], &endptr, 10);
			if (values[i][0] == '\0' || endptr[0] != '\0' ||
			    parsedint < 0 || errno != 0) {
				ret = -errno;
				PDRAW_LOG_ERRNO("strtol: %s", -ret, values[i]);
			} else {
				mFirstTs = parsedint;
			}
		} else if (strcmp(keys[i],
				  "com.parrot.regis.data_interpretation") ==
			   0) {
			dataInterpretationStr = values[i];
			/* TODO: this value should be added as a metadata of
			 * the media */
		}
	}

	/* Old regis-specific raw video track */
	if ((formatStr != nullptr) && (strcmp(formatStr, "raw32") == 0)) {
		format = vdef_raw32;
		unknownFormat = false;
	} else if ((formatStr != nullptr) && (strcmp(formatStr, "grey") == 0)) {
		format = vdef_gray;
		unknownFormat = false;
	}
	if (resolutionStr != nullptr) {
		ret = sscanf(resolutionStr,
			     "%ux%u",
			     &info.resolution.width,
			     &info.resolution.height);
		if (ret != 2) {
			PDRAW_LOGE("invalid raw video media resolution string");
			info.resolution.width = 0;
			info.resolution.height = 0;
		}
		info.sar.width = 1;
		info.sar.height = 1;
	}

	/* "video/raw" track with full format as MIME type parameters */
	if ((tkinfo->mime_format != nullptr) &&
	    (strncmp(tkinfo->mime_format,
		     VDEF_RAW_MIME_TYPE ";",
		     strlen(VDEF_RAW_MIME_TYPE ";")) == 0)) {
		/* Get the raw format and format info from the MIME
		 * format parameters */
		ret = vdef_raw_format_from_csv(tkinfo->mime_format, &format);
		if (ret < 0)
			ULOG_ERRNO("vdef_raw_format_from_csv", -ret);
		else
			unknownFormat = false;
		ret = vdef_format_info_from_csv(tkinfo->mime_format, &info);
		if (ret < 0)
			ULOG_ERRNO("vdef_format_info_from_csv", -ret);
		if (info.sar.width == 0 || info.sar.height == 0) {
			info.sar.width = 1;
			info.sar.height = 1;
		}
	}

	if (unknownFormat) {
		PDRAW_LOGE("invalid raw video media format");
		return -ENOSYS;
	}
	if ((info.resolution.width == 0) || (info.resolution.height == 0)) {
		PDRAW_LOGE("invalid raw video media resolution");
		return -ENOSYS;
	}
	if ((info.sar.width == 0) || (info.sar.height == 0)) {
		PDRAW_LOGE("invalid raw video media SAR");
		return -ENOSYS;
	}
	ret2 = vdef_calc_raw_contiguous_frame_size(&format,
						   &info.resolution,
						   nullptr,
						   nullptr,
						   nullptr,
						   nullptr,
						   nullptr);
	if (ret2 < 0) {
		ret = static_cast<int>(ret2);
		PDRAW_LOG_ERRNO("vdef_calc_raw_contiguous_frame_size", -ret);
		return ret;
	}
	capacity = ret2;

	mDemuxer->Source::lock();

	std::unique_ptr<RawVideoMedia> rawVideoMedia = nullptr;

	try {
		rawVideoMedia =
			std::make_unique<RawVideoMedia>(mDemuxer->mSession);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media allocation failed");
		return ret;
	}
	mMedias.push_back(std::move(rawVideoMedia));
	const auto &mediaPtr = mMedias.back();
	mRawVideoMedia = dynamic_cast<RawVideoMedia *>(mediaPtr.get());
	if (!mRawVideoMedia) {
		mDemuxer->Source::unlock();
		ULOGE("media is not an RawVideoMedia");
		return -EPROTO;
	}

	ret = mDemuxer->addOutputPort(mRawVideoMedia, mDemuxer->getDemuxer());
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}
	mRawVideoMedia->format = format;
	mRawVideoMedia->info = info;

	PDRAW_LOGI("%dx%d @ %d/%d fps, format=" VDEF_RAW_FORMAT_TO_STR_FMT,
		   info.resolution.width,
		   info.resolution.height,
		   info.framerate.num,
		   info.framerate.den,
		   VDEF_RAW_FORMAT_TO_STR_ARG(&format));

	std::string path =
		mDemuxer->Element::getName() + "$" + mRawVideoMedia->getName();
	mRawVideoMedia->setPath(path);
	(void)mDemuxer->fetchSessionMetadata(mTrackId,
					     &mRawVideoMedia->sessionMeta);
	if (mRawVideoMedia->sessionMeta.first_frame_capture_ts != 0)
		mFirstTs = mRawVideoMedia->sessionMeta.first_frame_capture_ts;
	mRawVideoMedia->setPlaybackType(PDRAW_PLAYBACK_TYPE_REPLAY);
	mRawVideoMedia->setDuration(mDemuxer->mDuration);
	if (tkinfo->has_metadata && tkinfo->metadata_mime_format != nullptr)
		mMetadataMimeType = std::string(tkinfo->metadata_mime_format);

	ret = mDemuxer->createOutputPortMemoryPool(
		mRawVideoMedia,
		DEMUXER_RECORD_RAW_VIDEO_MEDIA_OUTPUT_BUFFER_COUNT,
		capacity);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		return ret;
	}
	mDemuxer->Source::unlock();

	if (mDemuxer->Source::mListener) {
		mDemuxer->Source::mListener->onOutputMediaAdded(
			mDemuxer, mRawVideoMedia, mDemuxer->getDemuxer());
	}

	return 0;
}


int RecordDemuxer::DemuxerRawVideoMedia::processSample(
	struct mp4_track_sample *sample,
	bool *silent,
	bool *retry,
	bool *didSeek,
	bool *waitFlush)
{
	int ret = 0;
	int err;
	uint8_t *buf = nullptr;
	size_t bufSize = 0;
	size_t frameSize;
	size_t offset = 0;
	ssize_t ret2;
	unsigned int planeCount;
	std::array<size_t, VDEF_RAW_MAX_PLANE_COUNT> planeSize;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	struct vdef_raw_frame frameInfo = {};
	RawVideoMedia::Frame data = {};
	unsigned int outputChannelCount;

	/* Get an output buffer */
	if (mCurrentFrame != nullptr) {
		ret = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}
	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}
	ret = mDemuxer->getOutputMemory(mRawVideoMedia, &mCurrentMem);
	if ((ret < 0) || (mCurrentMem == nullptr)) {
		if (mDemuxer->mPlaybackMode != PDRAW_PLAYBACK_MODE_OFFLINE)
			PDRAW_LOGW("failed to get an input buffer (%d)", ret);
		*waitFlush = true;
		goto exit;
	}
	{
		void *rawBuf = nullptr;
		ret = mbuf_mem_get_data(mCurrentMem, &rawBuf, &bufSize);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
			goto exit;
		}
		buf = static_cast<uint8_t *>(rawBuf);
	}

	/* Get a sample size */
	ret = mp4_demux_get_track_sample(
		mDemuxer->mDemux, mTrackId, 0, nullptr, 0, nullptr, 0, sample);
	if (ret != 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_sample", -ret);
		goto exit;
	}
	/* Reallocate if needed */
	if (mMetadataBuffer.size() < sample->metadata_size) {
		try {
			mMetadataBuffer.resize(sample->metadata_size);
		} catch (const std::bad_alloc &) {
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("std::vector resize failed", -ret);
			goto exit;
		}
	}
	/* Get a sample */
	ret = mp4_demux_get_track_sample(
		mDemuxer->mDemux,
		mTrackId,
		1,
		buf,
		static_cast<unsigned int>(bufSize),
		mMetadataBuffer.data(),
		static_cast<unsigned int>(mMetadataBuffer.size()),
		sample);
	if (ret != 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_sample", -ret);
		/* Go to the next sample */
		ret = mp4_demux_get_track_sample(mDemuxer->mDemux,
						 mTrackId,
						 1,
						 nullptr,
						 0,
						 nullptr,
						 0,
						 sample);
		if (ret != 0)
			PDRAW_LOG_ERRNO("mp4_demux_get_track_sample", -ret);
		*retry = true;
		goto exit;
	}
	if (sample->size == 0) {
		ret = -ENOENT;
		goto exit;
	}
	*silent = ((sample->silent) && (mPendingSeekExact)) ? true : false;

	mPendingSeekTs = -1;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	/* Previous frame seek end on the first non-silent frame */
	if (mPendingSeekExact && !*silent)
		*didSeek = true;
	mPendingSeekExact = (*silent) ? mPendingSeekExact : false;

	frameInfo.format = mRawVideoMedia->format;
	vdef_format_to_frame_info(&mRawVideoMedia->info, &frameInfo.info);
	frameInfo.info.timestamp = mDecodingTs;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.index = mSampleIndex++;
	if (*silent || (mFirstSample && !sample->sync))
		frameInfo.info.flags |= VDEF_FRAME_FLAG_SILENT;

	ret = mbuf_raw_video_frame_new(&frameInfo, &mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto exit;
	}

	/* Sample data */
	ret2 = vdef_calc_raw_frame_size(&mRawVideoMedia->format,
					&mRawVideoMedia->info.resolution,
					frameInfo.plane_stride,
					nullptr,
					nullptr,
					nullptr,
					planeSize.data(),
					nullptr);
	if (ret2 < 0) {
		ret = static_cast<int>(ret2);
		PDRAW_LOG_ERRNO("vdef_calc_raw_frame_size", -ret);
		goto exit;
	}
	frameSize = ret2;
	if (frameSize > bufSize) {
		ret = -ENOBUFS;
		PDRAW_LOG_ERRNO("size mismatch", -ret);
		goto exit;
	}
	planeCount = vdef_get_raw_frame_plane_count(&mRawVideoMedia->format);
	for (unsigned int i = 0; i < planeCount; i++) {
		ret = mbuf_raw_video_frame_set_plane(
			mCurrentFrame, i, mCurrentMem, offset, planeSize[i]);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -ret);
			goto exit;
		}
		offset += planeSize[i];
	}

	data.ntpTimestamp = mDecodingTs;
	data.ntpUnskewedTimestamp = mDecodingTs;
	data.ntpRawTimestamp = mDecodingTs;
	data.ntpRawUnskewedTimestamp = mDecodingTs;
	if (sample->next_dts > 0) {
		mDecodingTsInc = mp4_sample_time_to_usec(
			sample->next_dts - sample->dts, mTimescale);
		if (mDemuxer->mSpeed != 0.) {
			mDecodingTsInc = static_cast<uint64_t>(
				static_cast<double>(mDecodingTsInc) /
				std::fabs(
					static_cast<double>(mDemuxer->mSpeed)));
		}
	}
	mCurrentFrameCaptureTs =
		(mFirstTs != UINT64_MAX) ? mFirstTs + mDecodingTs : mDecodingTs;
	mDecodingTs += mDecodingTsInc;

	/* Frame metadata */
	if (sample->metadata_size > 0) {
		/* Set the metadata */
		struct vmeta_frame *meta = nullptr;
		struct vmeta_buffer meta_buf;
		vmeta_buffer_set_cdata(&meta_buf,
				       mMetadataBuffer.data(),
				       sample->metadata_size,
				       0);
		ret = vmeta_frame_read(
			&meta_buf, mMetadataMimeType.c_str(), &meta);
		if (ret < 0) {
			if (ret != -ENODATA) {
				PDRAW_LOG_ERRNO("vmeta_frame_read", -ret);
				goto exit;
			}
		} else {
			ret = mbuf_raw_video_frame_set_metadata(mCurrentFrame,
								meta);
			vmeta_frame_unref(meta);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_set_metadata",
					-ret);
				goto exit;
			}
		}
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	data.demuxOutputTimestamp = curTime;
	data.playTimestamp = mp4_sample_time_to_usec(sample->dts, mTimescale);
	data.captureTimestamp = mCurrentFrameCaptureTs;
	data.localTimestamp = curTime;
	data.localTimestampPrecision =
		1; /* no estimation here, the precision is 1 microsecond */
	data.recvStartTimestamp = curTime;
	data.recvEndTimestamp = curTime;
	if (isReference())
		mDemuxer->mCurrentTime = data.playTimestamp;

	frameInfo.info.capture_timestamp = mCurrentFrameCaptureTs;

	/* Update the frame info */
	ret = mbuf_raw_video_frame_set_frame_info(mCurrentFrame, &frameInfo);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_set_frame_info", -ret);
		goto exit;
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&data,
		sizeof(data));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		goto exit;
	}

	ret = mbuf_raw_video_frame_finalize(mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto exit;
	}

	/* Queue the buffer in the output channels */
	outputChannelCount = mDemuxer->getOutputChannelCount(mRawVideoMedia);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		const struct vdef_raw_format *caps;
		int capsCount;

		Channel *c = mDemuxer->getOutputChannel(mRawVideoMedia, i);
		auto *channel = dynamic_cast<RawVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGW("invalid channel");
			continue;
		}

		capsCount = channel->getRawVideoMediaFormatCaps(&caps);
		if (capsCount < 0) {
			PDRAW_LOGW("invalid channel (no caps)");
			continue;
		}

		if (!vdef_raw_format_intersect(
			    &frameInfo.format, caps, capsCount)) {
			PDRAW_LOGW("incompatible raw video format on channel");
			continue;
		}

		ret = channel->queue(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->queue", -ret);
		else
			mDemuxer->setFlushingState(FlushingState::UNFLUSHED);
	}
	if (mFirstSample &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
		mFirstSample = false;
	}

exit:
	if (mCurrentMem != nullptr) {
		err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}
	if (mCurrentFrame != nullptr) {
		err = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	return ret;
}

} /* namespace Pdraw */
