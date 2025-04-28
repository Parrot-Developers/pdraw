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

#include <string>

#include <media-buffers/mbuf_ancillary_data.h>

#define DEFAULT_IN_BUF_CAPACITY (4 * 1024)

namespace Pdraw {


RecordDemuxer::DemuxerAudioMedia::DemuxerAudioMedia(RecordDemuxer *demuxer) :
		DemuxerMedia(demuxer), mAudioMedia(nullptr),
		mCurrentFrame(nullptr), mCurrentMem(nullptr),
		mCurrentFrameCaptureTs(0), mDecodingTs(0), mDecodingTsInc(0),
		mFirstTs(UINT64_MAX)

{
	mMediaType = Media::Type::RAW_VIDEO;
	std::string name = demuxer->getName() + "#DemuxerAudioMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerAudioMedia::~DemuxerAudioMedia(void)
{
	teardownMedia();
}


void RecordDemuxer::DemuxerAudioMedia::flush(bool destroy)
{
	if (mCurrentFrame != nullptr) {
		int err = mbuf_audio_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		int err = mbuf_mem_unref(mCurrentMem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
		mCurrentMem = nullptr;
	}

	DemuxerMedia::flush(destroy);
}


void RecordDemuxer::DemuxerAudioMedia::stop(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_audio_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -ret);
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


void RecordDemuxer::DemuxerAudioMedia::teardownMedia(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_audio_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}

	DemuxerMedia::teardownMedia();
	mAudioMedia = nullptr;
}


int RecordDemuxer::DemuxerAudioMedia::setupMedia(
	const struct mp4_track_info *tkinfo)
{
	int ret;
	uint8_t *asc;
	unsigned int asc_size;

	ret = mp4_demux_get_track_audio_specific_config(
		mDemuxer->mDemux, mTrackId, &asc, &asc_size);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_audio_specific_config",
				-ret);
		return ret;
	}

	switch (tkinfo->audio_codec) {
	case MP4_AUDIO_CODEC_AAC_LC:
		if ((asc == nullptr) || (asc_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid ASC");
			return ret;
		}
		break;
	default:
		mDemuxer->Source::unlock();
		ret = -EPROTO;
		PDRAW_LOGE("invalid audio codec");
		return ret;
	}

	mAudioMedia = new AudioMedia(mDemuxer->mSession);
	if (mAudioMedia == nullptr) {
		ret = -ENOMEM;
		mDemuxer->Source::unlock();
		PDRAW_LOGE("media allocation failed");
		return -ENOMEM;
	}
	mMedias.push_back(mAudioMedia);

	/* Create the output port */
	mDemuxer->Source::lock();
	ret = mDemuxer->addOutputPort(mAudioMedia, mDemuxer->getDemuxer());
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	/* Set the output media info */
	switch (tkinfo->audio_codec) {
	case MP4_AUDIO_CODEC_AAC_LC:
		struct aac_asc audioSpecificConfig;
		ret = aac_parse_asc(asc, asc_size, &audioSpecificConfig);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("aac_parse_asc", -ret);
			return ret;
		}
		ret = aac_asc_to_adef_format(&audioSpecificConfig,
					     &mAudioMedia->format);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("aac_asc_to_adef_format", -ret);
			return ret;
		}
		ret = mAudioMedia->setAacAsc(asc, asc_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("media->setAacAsc", -ret);
			return ret;
		}
		break;
	default:
		mDemuxer->Source::unlock();
		ret = -EPROTO;
		PDRAW_LOGE("invalid audio codec");
		return ret;
	}

	std::string path =
		mDemuxer->Element::getName() + "$" + mAudioMedia->getName();
	mAudioMedia->setPath(path);
	mAudioMedia->playbackType = PDRAW_PLAYBACK_TYPE_REPLAY;
	mAudioMedia->duration = mDemuxer->mDuration;

	ret = mDemuxer->createOutputPortMemoryPool(
		mAudioMedia,
		DEMUXER_RECORD_AUDIO_MEDIA_OUTPUT_BUFFER_COUNT,
		DEFAULT_IN_BUF_CAPACITY); /* TODO */
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		return ret;
	}
	mDemuxer->Source::unlock();

	if (mDemuxer->Source::mListener)
		mDemuxer->Source::mListener->onOutputMediaAdded(
			mDemuxer, mAudioMedia, mDemuxer->getDemuxer());

	return 0;
}


int RecordDemuxer::DemuxerAudioMedia::processSample(
	struct mp4_track_sample *sample,
	bool *silent,
	bool *retry,
	bool *didSeek,
	bool *waitFlush)
{
	int ret = 0, err;
	uint8_t *buf = nullptr;
	size_t bufSize = 0;
	size_t length = 0;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	struct adef_frame frameInfo = {};
	AudioMedia::Frame data = {};
	unsigned int outputChannelCount;

	/* Get an output buffer */
	if (mCurrentFrame != nullptr) {
		ret = mbuf_audio_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}
	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}
	ret = mDemuxer->getOutputMemory(mAudioMedia, &mCurrentMem);
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
	length = sample->size;
	if (length == 0) {
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

	frameInfo.format = mAudioMedia->format;
	frameInfo.info.timestamp = mDecodingTs;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.index = mSampleIndex++;

	ret = mbuf_audio_frame_new(&frameInfo, &mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_new", -ret);
		goto exit;
	}

	ret = mbuf_audio_frame_set_buffer(
		mCurrentFrame, mCurrentMem, 0, length);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_set_buffer", -ret);
		goto exit;
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
	frameInfo.format = mAudioMedia->format;
	frameInfo.info.capture_timestamp = mCurrentFrameCaptureTs;

	ret = mbuf_audio_frame_add_ancillary_buffer(
		mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&data,
		sizeof(data));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		goto exit;
	}

	ret = mbuf_audio_frame_finalize(mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_finalize", -ret);
		goto exit;
	}

	/* Queue the buffer in the output channels */
	outputChannelCount = mDemuxer->getOutputChannelCount(mAudioMedia);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		const struct adef_format *caps;
		int capsCount;

		Channel *c = mDemuxer->getOutputChannel(mAudioMedia, i);
		AudioChannel *channel = dynamic_cast<AudioChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGW("invalid channel");
			continue;
		}

		capsCount = channel->getAudioMediaFormatCaps(&caps);
		if (capsCount < 0) {
			PDRAW_LOGW("invalid channel (no caps)");
			continue;
		}

		if (!adef_format_intersect(
			    &frameInfo.format, caps, capsCount)) {
			PDRAW_LOGW("incompatible audio format on channel");
			continue;
		}

		ret = channel->queue(mCurrentFrame);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	}
	if ((mFirstSample) && (true)) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
		mFirstSample = false;
	}

exit:
	err = mbuf_mem_unref(mCurrentMem);
	if (err < 0)
		PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
	mCurrentMem = nullptr;
	err = mbuf_audio_frame_unref(mCurrentFrame);
	if (err < 0)
		PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
	mCurrentFrame = nullptr;

	return ret;
}

} /* namespace Pdraw */
