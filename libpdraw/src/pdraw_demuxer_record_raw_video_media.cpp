/**
 * Parrot Drones Awesome Video Viewer Library
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

#include <string>

#include <media-buffers/mbuf_ancillary_data.h>
#include <video-streaming/vstrm.h>

namespace Pdraw {


RecordDemuxer::DemuxerRawVideoMedia::DemuxerRawVideoMedia(
	RecordDemuxer *demuxer) :
		DemuxerMedia(demuxer),
		mRawVideoMedia(nullptr), mCurrentFrame(nullptr),
		mCurrentMem(nullptr), mCurrentFrameCaptureTs(0), mDecodingTs(0),
		mDecodingTsInc(0), mFirstTs(UINT64_MAX)

{
	mMediaType = Media::Type::RAW_VIDEO;
	std::string name = demuxer->getName() + "#DemuxerRawVideoMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerRawVideoMedia::~DemuxerRawVideoMedia(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_raw_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
	}

	/* Remove the output ports */
	if (mDemuxer->Source::mListener) {
		mDemuxer->Source::mListener->onOutputMediaRemoved(
			mDemuxer, mRawVideoMedia, mDemuxer->getDemuxer());
	}
	ret = mDemuxer->removeOutputPort(mRawVideoMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mRawVideoMedia;
	}
}


void RecordDemuxer::DemuxerRawVideoMedia::stop(void)
{
	int ret;

	RecordDemuxer::DemuxerMedia::stop();

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
}


void RecordDemuxer::DemuxerRawVideoMedia::sendDownstreamEvent(
	Channel::DownstreamEvent event)
{
	int ret;

	ret = mDemuxer->Source::sendDownstreamEvent(mRawVideoMedia, event);
	if (ret < 0)
		PDRAW_LOG_ERRNO("Source::sendDownstreamEvent", -ret);
}


int RecordDemuxer::DemuxerRawVideoMedia::setupMedia(
	struct mp4_track_info *tkinfo)
{
	int ret;
	unsigned int count = 0, i;
	char **keys = nullptr, **values = nullptr;
	const char *formatStr = nullptr, *resolutionStr = nullptr;
	const char *dataInterpretationStr = nullptr;
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
	for (i = 0; i < count; i++) {
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
		ret = ret2;
		PDRAW_LOG_ERRNO("vdef_calc_raw_contiguous_frame_size", -ret);
		return ret;
	}
	capacity = ret2;

	mDemuxer->Source::lock();

	mRawVideoMedia = new RawVideoMedia(mDemuxer->mSession);
	if (mRawVideoMedia == nullptr) {
		ret = -ENOMEM;
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media allocation failed");
		return -ENOMEM;
	}
	ret = mDemuxer->addOutputPort(mRawVideoMedia, mDemuxer->getDemuxer());
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}
	mRawVideoMedia->format = format;
	mRawVideoMedia->info = info;

	char *fmt = vdef_raw_format_to_str(&format);
	PDRAW_LOGI("%dx%d @ %d/%d fps, format=%s",
		   info.resolution.width,
		   info.resolution.height,
		   info.framerate.num,
		   info.framerate.den,
		   fmt);
	free(fmt);

	std::string path =
		mDemuxer->Element::getName() + "$" + mRawVideoMedia->getName();
	mRawVideoMedia->setPath(path);
	(void)mDemuxer->fetchSessionMetadata(mTrackId,
					     &mRawVideoMedia->sessionMeta);
	mRawVideoMedia->playbackType = PDRAW_PLAYBACK_TYPE_REPLAY;
	mRawVideoMedia->duration = mDemuxer->mDuration;
	if (tkinfo->has_metadata)
		mMetadataMimeType = strdup(tkinfo->metadata_mime_format);

	ret = mDemuxer->createOutputPortMemoryPool(
		mRawVideoMedia, DEMUXER_OUTPUT_BUFFER_COUNT, capacity);
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
	uint8_t *buf = nullptr;
	size_t bufSize = 0, frameSize, offset = 0;
	ssize_t ret2;
	unsigned int planeCount;
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
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
	ret = mDemuxer->getRawVideoOutputMemory(mRawVideoMedia, &mCurrentMem);
	if ((ret < 0) || (mCurrentMem == nullptr)) {
		PDRAW_LOGW("failed to get an input buffer (%d)", ret);
		*waitFlush = true;
		goto exit;
	}
	ret = mbuf_mem_get_data(mCurrentMem, (void **)&buf, &bufSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto exit;
	}

	/* Get a sample */
	ret = mp4_demux_get_track_sample(mDemuxer->mDemux,
					 mTrackId,
					 1,
					 buf,
					 bufSize,
					 mMetadataBuffer,
					 mMetadataBufferSize,
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
		if (mDemuxer->mFrameByFrame)
			mDemuxer->mRunning = false;
		ret = 0;
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
					planeSize,
					nullptr);
	if (ret2 < 0) {
		ret = ret2;
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
		if (mDemuxer->mSpeed != 0.)
			mDecodingTsInc /= fabs(mDemuxer->mSpeed);
	}
	mCurrentFrameCaptureTs =
		(mFirstTs != UINT64_MAX) ? mFirstTs + mDecodingTs : mDecodingTs;
	mDecodingTs += mDecodingTsInc;

	/* Frame metadata */
	if (sample->metadata_size > 0) {
		/* Set the metadata */
		struct vmeta_frame *meta = nullptr;
		struct vmeta_buffer meta_buf;
		vmeta_buffer_set_cdata(
			&meta_buf, mMetadataBuffer, sample->metadata_size, 0);
		ret = vmeta_frame_read(&meta_buf, mMetadataMimeType, &meta);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("vmeta_frame_read", -ret);
			goto exit;
		}

		ret = mbuf_raw_video_frame_set_metadata(mCurrentFrame, meta);
		vmeta_frame_unref(meta);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_set_metadata",
					-ret);
			goto exit;
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

	if (*didSeek) {
		/* TODO: signal once, not for all medias */
		mDemuxer->seekResponse(mSeekResponse,
				       mDemuxer->mCurrentTime,
				       mDemuxer->mSpeed);
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
		RawVideoChannel *channel = dynamic_cast<RawVideoChannel *>(c);
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
		if (ret < 0) {
			PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	}
	if ((mFirstSample) &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
		mFirstSample = false;
	}

exit:
	mbuf_mem_unref(mCurrentMem);
	mCurrentMem = nullptr;
	mbuf_raw_video_frame_unref(mCurrentFrame);
	mCurrentFrame = nullptr;

	return ret;
}

} /* namespace Pdraw */
