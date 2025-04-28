/**
 * Parrot Drones Audio and Video Vector library
 * Recording demuxer coded video media
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


const struct h264_ctx_cbs
	RecordDemuxer::DemuxerCodedVideoMedia::mH264ReaderCbs = {
		.au_end = nullptr,
		.nalu_begin = nullptr,
		.nalu_end = nullptr,
		.slice = nullptr,
		.slice_data_begin = nullptr,
		.slice_data_end = nullptr,
		.slice_data_mb = nullptr,
		.sps = nullptr,
		.pps = nullptr,
		.aud = nullptr,
		.sei = nullptr,
		.sei_buffering_period = nullptr,
		.sei_pic_timing = &RecordDemuxer::DemuxerCodedVideoMedia::
					  h264PicTimingSeiCb,
		.sei_pan_scan_rect = nullptr,
		.sei_filler_payload = nullptr,
		.sei_user_data_registered = nullptr,
		.sei_user_data_unregistered =
			&RecordDemuxer::DemuxerCodedVideoMedia::
				h264UserDataSeiCb,
		.sei_recovery_point = nullptr,
};


const struct h265_ctx_cbs
	RecordDemuxer::DemuxerCodedVideoMedia::mH265ReaderCbs = {
		.nalu_begin = nullptr,
		.nalu_end = nullptr,
		.au_end = nullptr,
		.vps = nullptr,
		.sps = nullptr,
		.pps = nullptr,
		.aud = nullptr,
		.sei = nullptr,
		.sei_user_data_unregistered =
			&RecordDemuxer::DemuxerCodedVideoMedia::
				h265UserDataSeiCb,
		.sei_recovery_point = nullptr,
		.sei_time_code = &RecordDemuxer::DemuxerCodedVideoMedia::
					 h265TimeCodeSeiCb,
		.sei_mastering_display_colour_volume =
			&RecordDemuxer::DemuxerCodedVideoMedia::h265MdcvSeiCb,
		.sei_content_light_level =
			&RecordDemuxer::DemuxerCodedVideoMedia::h265CllSeiCb,
};


RecordDemuxer::DemuxerCodedVideoMedia::DemuxerCodedVideoMedia(
	RecordDemuxer *demuxer) :
		DemuxerMedia(demuxer),
		mH264Reader(nullptr), mH265Reader(nullptr),
		mCurrentFrame(nullptr), mCurrentMem(nullptr),
		mCurrentFrameCaptureTs(0), mDecodingTs(0), mDecodingTsInc(0),
		mFirstTs(UINT64_MAX)
{
	mMediaType = Media::Type::CODED_VIDEO;
	std::string name = demuxer->getName() + "#DemuxerCodedVideoMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerCodedVideoMedia::~DemuxerCodedVideoMedia(void)
{
	teardownMedia();
}


void RecordDemuxer::DemuxerCodedVideoMedia::flush(bool destroy)
{
	if (mCurrentFrame != nullptr) {
		int err = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
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


void RecordDemuxer::DemuxerCodedVideoMedia::stop(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
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


void RecordDemuxer::DemuxerCodedVideoMedia::teardownMedia(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}

	if (mH264Reader != nullptr) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = nullptr;
	}
	if (mH265Reader != nullptr) {
		ret = h265_reader_destroy(mH265Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h265_reader_destroy", -ret);
		mH265Reader = nullptr;
	}

	DemuxerMedia::teardownMedia();
}


int RecordDemuxer::DemuxerCodedVideoMedia::setupMedia(
	const struct mp4_track_info *tkinfo)
{
	int ret, err;
	unsigned int count = 0, i;
	char **keys = nullptr, **values = nullptr;
	Source::OutputPort *basePort, *mediaPort;
	struct mp4_video_decoder_config vdc = {};
	const size_t NB_MEDIAS = 2;
	CodedVideoMedia *codedMedias[NB_MEDIAS] = {};

	/* Get the track-level session metadata */
	ret = mp4_demux_get_track_metadata_strings(
		mDemuxer->mDemux, mTrackId, &count, &keys, &values);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_metadata_strings", -ret);
		return ret;
	}

	for (i = 0; i < count; i++) {
		if (strcmp(keys[i], "com.parrot.regis.first_timestamp") == 0) {
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
		}
	}

	ret = mp4_demux_get_track_video_decoder_config(
		mDemuxer->mDemux, mTrackId, &vdc);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_video_decoder_config",
				-ret);
		goto error;
	}

	switch (vdc.codec) {
	case MP4_VIDEO_CODEC_AVC:
		if ((vdc.avc.sps == nullptr) || (vdc.avc.sps_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid SPS");
			goto error;
		}
		if ((vdc.avc.pps == nullptr) || (vdc.avc.pps_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid PPS");
			goto error;
		}
		break;
	case MP4_VIDEO_CODEC_HEVC:
		if ((vdc.hevc.vps == nullptr) || (vdc.hevc.vps_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid VPS");
			goto error;
		}
		if ((vdc.hevc.sps == nullptr) || (vdc.hevc.sps_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid SPS");
			goto error;
		}
		if ((vdc.hevc.pps == nullptr) || (vdc.hevc.pps_size == 0)) {
			ret = -EPROTO;
			PDRAW_LOGE("invalid PPS");
			goto error;
		}
		break;
	default:
		ret = -EPROTO;
		PDRAW_LOGE("invalid video codec");
		goto error;
	}

	/* Create the H.264 and H.265 readers */
	ret = h264_reader_new(&mH264ReaderCbs, this, &mH264Reader);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_reader_new", -ret);
		goto error;
	}
	ret = h265_reader_new(&mH265ReaderCbs, this, &mH265Reader);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h265_reader_new", -ret);
		goto error;
	}

	mDemuxer->Source::lock();

	if (!mMedias.empty()) {
		ret = -EEXIST;
		mDemuxer->Source::unlock();
		PDRAW_LOGE("video track already defined");
		goto error;
	}

	for (unsigned int i = 0; i < NB_MEDIAS; i++) {
		codedMedias[i] = new CodedVideoMedia(mDemuxer->mSession);
		if (codedMedias[i] == nullptr) {
			ret = -ENOMEM;
			mDemuxer->Source::unlock();
			PDRAW_LOGE("media allocation failed");
			goto error;
		}
		mMedias.push_back(codedMedias[i]);
		ret = mDemuxer->addOutputPort(codedMedias[i],
					      mDemuxer->getDemuxer());
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("addOutputPort", -ret);
			goto error;
		}
		std::string path = mDemuxer->Element::getName() + "$" +
				   codedMedias[i]->getName();
		codedMedias[i]->setPath(path);
		(void)mDemuxer->fetchSessionMetadata(
			mTrackId, &codedMedias[i]->sessionMeta);
		codedMedias[i]->playbackType = PDRAW_PLAYBACK_TYPE_REPLAY;
		codedMedias[i]->duration = mDemuxer->mDuration;
	}
	if (tkinfo->has_metadata)
		mMetadataMimeType = strdup(tkinfo->metadata_mime_format);

	/* Set the output media info */
	switch (tkinfo->video_codec) {
	case MP4_VIDEO_CODEC_AVC:
		codedMedias[0]->format = vdef_h264_avcc;
		codedMedias[1]->format = vdef_h264_byte_stream;
		for (auto m = mMedias.begin(); m != mMedias.end(); m++) {
			CodedVideoMedia *codedMedia =
				dynamic_cast<CodedVideoMedia *>(*m);
			if (codedMedia == nullptr) {
				ret = -EPROTO;
				PDRAW_LOG_ERRNO("dynamic_cast", -ret);
				goto error;
			}
			ret = codedMedia->setPs(nullptr,
						0,
						vdc.avc.sps,
						vdc.avc.sps_size,
						vdc.avc.pps,
						vdc.avc.pps_size);
			if (ret < 0) {
				mDemuxer->Source::unlock();
				PDRAW_LOG_ERRNO("media->setPs", -ret);
				goto error;
			}
		}
		/* Initialize the H.264 parsing */
		ret = h264_reader_parse_nalu(
			mH264Reader, 0, vdc.avc.sps, vdc.avc.sps_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h264_reader_parse_nalu(
			mH264Reader, 0, vdc.avc.pps, vdc.avc.pps_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			goto error;
		}
		break;
	case MP4_VIDEO_CODEC_HEVC:
		codedMedias[0]->format = vdef_h265_hvcc;
		codedMedias[1]->format = vdef_h265_byte_stream;
		for (auto m = mMedias.begin(); m != mMedias.end(); m++) {
			CodedVideoMedia *codedMedia =
				dynamic_cast<CodedVideoMedia *>(*m);
			if (codedMedia == nullptr) {
				ret = -EPROTO;
				PDRAW_LOG_ERRNO("dynamic_cast", -ret);
				goto error;
			}
			ret = codedMedia->setPs(vdc.hevc.vps,
						vdc.hevc.vps_size,
						vdc.hevc.sps,
						vdc.hevc.sps_size,
						vdc.hevc.pps,
						vdc.hevc.pps_size);
			if (ret < 0) {
				mDemuxer->Source::unlock();
				PDRAW_LOG_ERRNO("media->setPs", -ret);
				goto error;
			}
		}
		/* Initialize the H.265 parsing */
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.vps, vdc.hevc.vps_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.sps, vdc.hevc.sps_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.pps, vdc.hevc.pps_size);
		if (ret < 0) {
			mDemuxer->Source::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		break;
	default:
		mDemuxer->Source::unlock();
		ret = -EPROTO;
		PDRAW_LOGE("invalid video codec");
		goto error;
	}

	/* Create the output buffers pool on the last media. As the medias will
	 * be destroyed in creation order, this ensures that the media which
	 * owns the buffers pool will be the last destroyed */
	ret = mDemuxer->createOutputPortMemoryPool(
		codedMedias[1],
		DEMUXER_RECORD_CODED_VIDEO_MEDIA_OUTPUT_BUFFER_COUNT,
		codedMedias[1]->info.resolution.width *
			codedMedias[1]->info.resolution.height * 3 / 4);
	if (ret < 0) {
		mDemuxer->Source::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		goto error;
	}
	/* Make the pool shared between all medias */
	basePort = mDemuxer->getOutputPort(codedMedias[1]);
	mediaPort = mDemuxer->getOutputPort(codedMedias[0]);
	if (basePort == nullptr || mediaPort == nullptr) {
		PDRAW_LOGW("unable to share memory pool between medias");
	} else {
		mediaPort->pool = basePort->pool;
		mediaPort->sharedPool = true;
	}
	mDemuxer->Source::unlock();

	if (mDemuxer->Source::mListener) {
		for (auto m = mMedias.begin(); m != mMedias.end(); m++) {
			mDemuxer->Source::mListener->onOutputMediaAdded(
				mDemuxer, *m, mDemuxer->getDemuxer());
		}
	}

	return 0;

error:
	if (mH264Reader != nullptr) {
		err = h264_reader_destroy(mH264Reader);
		if (err < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -err);
		mH264Reader = nullptr;
	}
	if (mH265Reader != nullptr) {
		err = h265_reader_destroy(mH265Reader);
		if (err < 0)
			PDRAW_LOG_ERRNO("h265_reader_destroy", -err);
		mH265Reader = nullptr;
	}
	return ret;
}


int RecordDemuxer::DemuxerCodedVideoMedia::processSample(
	struct mp4_track_sample *sample,
	bool *silent,
	bool *retry,
	bool *didSeek,
	bool *waitFlush)
{
	int ret = 0;
	unsigned int requiredMediaIndex;
	CodedVideoMedia *requiredMedia;
	struct vdef_coded_frame frameInfo = {};
	struct mbuf_coded_video_frame *outputFrame = nullptr;
	unsigned int outputChannelCount = 0;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	CodedVideoMedia::Frame data = {};
	uint8_t *buf = nullptr, *tmp, *sei = nullptr;
	size_t bufSize = 0, offset, naluSize, seiSize = 0;

	/* Get an output buffer */
	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
		mCurrentFrame = nullptr;
	}
	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		mCurrentMem = nullptr;
	}
	std::vector<CodedVideoMedia *> codedMedias;
	for (auto m = mMedias.begin(); m != mMedias.end(); m++) {
		CodedVideoMedia *codedMedia =
			dynamic_cast<CodedVideoMedia *>(*m);
		if (codedMedia == nullptr) {
			ret = -EPROTO;
			PDRAW_LOG_ERRNO("dynamic_cast", -ret);
			goto exit;
		}
		codedMedias.push_back(codedMedia);
	}
	ret = mDemuxer->getCodedVideoOutputMemory(
		codedMedias, &mCurrentMem, &requiredMediaIndex);
	if ((ret < 0) || (mCurrentMem == nullptr)) {
		PDRAW_LOGW("failed to get an input buffer (%d)", ret);
		*waitFlush = true;
		goto exit;
	}
	requiredMedia = codedMedias[requiredMediaIndex];
	ret = mbuf_mem_get_data(mCurrentMem, (void **)&buf, &bufSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto exit;
	}
	mCurrentFrameCaptureTs = 0;

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

	frameInfo.format = requiredMedia->format;
	vdef_format_to_frame_info(&requiredMedia->info, &frameInfo.info);
	frameInfo.info.timestamp = mDecodingTs;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.index = mSampleIndex++;
	if (*silent || (mFirstSample && !sample->sync))
		frameInfo.info.flags |= VDEF_FRAME_FLAG_SILENT;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_I;

	ret = mbuf_coded_video_frame_new(&frameInfo, &mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto exit;
	}

	switch (requiredMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		/* Parse the H.264 bitstream to convert to byte stream, fill
		 * the mbuf frame and find an optional SEI NALU */
		tmp = buf;
		offset = 0;
		while (offset < sample->size) {
			enum h264_nalu_type naluType;
			enum h264_slice_type sliceType =
				H264_SLICE_TYPE_UNKNOWN;
			memcpy(&naluSize, tmp, sizeof(uint32_t));
			naluSize = ntohl(naluSize);
			if (naluSize == 0) {
				PDRAW_LOGE(
					"invalid NALU size (%zu), "
					"skipping frame",
					naluSize);
				goto exit;
			}
			naluType = (enum h264_nalu_type)(*(tmp + 4) & 0x1F);
			if (naluType == H264_NALU_TYPE_SEI) {
				sei = tmp + 4;
				seiSize = naluSize;
			} else if (naluType == H264_NALU_TYPE_SLICE_IDR) {
				frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;
				sliceType = H264_SLICE_TYPE_I;
			}
			/* Add the NAL unit to the frame */
			struct vdef_nalu nalu = {};
			nalu.size = naluSize + 4;
			nalu.h264.type = naluType;
			nalu.h264.slice_type = sliceType;
			/* Note: h264.slice_mb_count is not known here */
			ret = mbuf_coded_video_frame_add_nalu(
				mCurrentFrame, mCurrentMem, offset, &nalu);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_add_nalu",
					-ret);
				goto exit;
			}
			tmp += 4 + naluSize;
			offset += 4 + naluSize;
		}
		if ((sei != nullptr) && (seiSize != 0)) {
			ret = h264_reader_parse_nalu(
				mH264Reader, 0, sei, seiSize);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			}
		}
		break;
	case VDEF_ENCODING_H265:
		/* Parse the H.265 bitstream to convert to byte stream, fill
		 * the mbuf frame and find an optional SEI NALU */
		tmp = buf;
		offset = 0;
		while (offset < sample->size) {
			enum h265_nalu_type naluType;
			memcpy(&naluSize, tmp, sizeof(uint32_t));
			naluSize = ntohl(naluSize);
			if (naluSize == 0) {
				PDRAW_LOGE(
					"invalid NALU size (%zu), "
					"skipping frame",
					naluSize);
				goto exit;
			}
			naluType =
				(enum h265_nalu_type)((*(tmp + 4) >> 1) & 0x3F);
			if ((naluType == H265_NALU_TYPE_PREFIX_SEI_NUT) ||
			    (naluType == H265_NALU_TYPE_SUFFIX_SEI_NUT)) {
				sei = tmp + 4;
				seiSize = naluSize;
			} else if ((naluType == H265_NALU_TYPE_IDR_W_RADL) ||
				   (naluType == H265_NALU_TYPE_IDR_N_LP)) {
				frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;
			}
			/* Add the NAL unit to the frame */
			struct vdef_nalu nalu = {};
			nalu.size = naluSize + 4;
			nalu.h265.type = naluType;
			ret = mbuf_coded_video_frame_add_nalu(
				mCurrentFrame, mCurrentMem, offset, &nalu);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_add_nalu",
					-ret);
				goto exit;
			}
			tmp += 4 + naluSize;
			offset += 4 + naluSize;
		}
		if ((sei != nullptr) && (seiSize != 0)) {
			ret = h265_reader_parse_nalu(
				mH265Reader, 0, sei, seiSize);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			}
		}
		break;
	default:
		break;
	}

	data.isSync = sample->sync;
	data.isRef = true; /* Note: could be set more precisely by getting the
			      info from the NRI bits */
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
	if (mCurrentFrameCaptureTs == 0 && mFirstTs != UINT64_MAX)
		mCurrentFrameCaptureTs = mFirstTs + mDecodingTs;
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
			if (ret != -ENODATA) {
				PDRAW_LOG_ERRNO("vmeta_frame_read", -ret);
				goto exit;
			}
		} else {
			ret = mbuf_coded_video_frame_set_metadata(mCurrentFrame,
								  meta);
			vmeta_frame_unref(meta);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_set_metadata",
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
	mDemuxer->mCurrentTime = data.playTimestamp;

	frameInfo.info.capture_timestamp = mCurrentFrameCaptureTs;

	/* Update the frame info */
	ret = mbuf_coded_video_frame_set_frame_info(mCurrentFrame, &frameInfo);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_set_frame_info", -ret);
		goto exit;
	}

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&data,
		sizeof(data));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto exit;
	}

	if (*didSeek) {
		/* TODO: signal once, not for all medias */
		mDemuxer->seekResponse(mSeekResponse,
				       mDemuxer->mCurrentTime,
				       mDemuxer->mSpeed);
	}

	/* Convert to byte stream if required */
	if (requiredMedia->format.data_format ==
	    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		switch (requiredMedia->format.encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_avcc_to_byte_stream(buf, sample->size);
			if (ret < 0)
				PDRAW_LOG_ERRNO("h264_avcc_to_byte_stream",
						-ret);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_hvcc_to_byte_stream(buf, sample->size);
			if (ret < 0)
				PDRAW_LOG_ERRNO("h265_hvcc_to_byte_stream",
						-ret);
			break;
		default:
			break;
		}
	}

	ret = mbuf_coded_video_frame_finalize(mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto exit;
	}

	/* Queue the buffer in the output channels */
	for (auto m = mMedias.begin(); m != mMedias.end(); m++) {
		CodedVideoMedia *codedMedia =
			dynamic_cast<CodedVideoMedia *>(*m);
		if (codedMedia == nullptr) {
			ret = -EPROTO;
			PDRAW_LOG_ERRNO("dynamic_cast", -ret);
			goto exit;
		}
		outputChannelCount =
			mDemuxer->getOutputChannelCount(codedMedia);
		if (outputChannelCount == 0)
			continue;
		if (outputFrame != nullptr)
			mbuf_coded_video_frame_unref(outputFrame);
		outputFrame = mCurrentFrame;
		if (!vdef_coded_format_cmp(&requiredMedia->format,
					   &codedMedia->format)) {
			/* The format is different, we need to pick another
			 * frame */
			ret = mDemuxer->copyCodedVideoOutputFrame(requiredMedia,
								  mCurrentFrame,
								  codedMedia,
								  &outputFrame);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("copyCodedVideoOutputFrame",
						-ret);
				outputFrame = nullptr;
				continue;
			}
		} else {
			mbuf_coded_video_frame_ref(outputFrame);
		}
		ret = mbuf_coded_video_frame_get_frame_info(outputFrame,
							    &frameInfo);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
					-ret);
			goto exit;
		}
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			const struct vdef_coded_format *caps;
			int capsCount;

			Channel *c = mDemuxer->getOutputChannel(codedMedia, j);
			CodedVideoChannel *channel =
				dynamic_cast<CodedVideoChannel *>(c);
			if (channel == nullptr) {
				PDRAW_LOGW("invalid channel");
				continue;
			}

			capsCount =
				channel->getCodedVideoMediaFormatCaps(&caps);
			if (capsCount < 0) {
				PDRAW_LOGW("invalid channel (no caps)");
				continue;
			}

			if (!vdef_coded_format_intersect(
				    &frameInfo.format, caps, capsCount)) {
				PDRAW_LOGW(
					"incompatible coded video format "
					"on channel");
				continue;
			}

			ret = channel->queue(outputFrame);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("channel->queue", -ret);
			}
		}
	}
	if (outputFrame != nullptr)
		mbuf_coded_video_frame_unref(outputFrame);
	if ((mFirstSample) &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		sendDownstreamEvent(Channel::DownstreamEvent::SOS);
		mFirstSample = false;
	}

exit:
	if (mCurrentMem != nullptr) {
		mbuf_mem_unref(mCurrentMem);
		mCurrentMem = nullptr;
	}
	if (mCurrentFrame != nullptr) {
		mbuf_coded_video_frame_unref(mCurrentFrame);
		mCurrentFrame = nullptr;
	}
	return ret;
}


void RecordDemuxer::DemuxerCodedVideoMedia::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	int ret = 0;
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;

	if (self == nullptr)
		return;
	if ((buf == nullptr) || (len == 0))
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	/* Ignore "Parrot Streaming" user data SEI */
	if (vstrm_h264_is_sei_streaming(sei->uuid))
		return;

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		self->mCurrentFrame, MBUF_ANCILLARY_KEY_USERDATA_SEI, buf, len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}
}


void RecordDemuxer::DemuxerCodedVideoMedia::h264PicTimingSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_pic_timing *sei,
	void *userdata)
{
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	self->mCurrentFrameCaptureTs = h264_ctx_sei_pic_timing_to_us(ctx, sei);
}


void RecordDemuxer::DemuxerCodedVideoMedia::h265UserDataSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_user_data_unregistered *sei,
	void *userdata)
{
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;
	int ret = 0;

	if (self == nullptr)
		return;
	if ((buf == nullptr) || (len == 0))
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		self->mCurrentFrame, MBUF_ANCILLARY_KEY_USERDATA_SEI, buf, len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}
}


void RecordDemuxer::DemuxerCodedVideoMedia::h265TimeCodeSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_time_code *sei,
	void *userdata)
{
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;
	if (self->mCurrentFrame == nullptr)
		return;

	self->mCurrentFrameCaptureTs = h265_ctx_sei_time_code_to_us(ctx, sei);
}


void RecordDemuxer::DemuxerCodedVideoMedia::h265MdcvSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_mastering_display_colour_volume *sei,
	void *userdata)
{
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;

	for (auto m = self->mMedias.begin(); m != self->mMedias.end(); m++) {
		CodedVideoMedia *codedMedia =
			dynamic_cast<CodedVideoMedia *>(*m);
		ULOG_ERRNO_RETURN_IF(codedMedia == nullptr, EPROTO);
		for (unsigned int k = 0; k < 3; k++) {
			codedMedia->info.mdcv.display_primaries_val
				.color_primaries[k]
				.x =
				(float)sei->display_primaries_x[k] / 50000.;
			codedMedia->info.mdcv.display_primaries_val
				.color_primaries[k]
				.y =
				(float)sei->display_primaries_y[k] / 50000.;
		}
		codedMedia->info.mdcv.display_primaries_val.white_point.x =
			(float)sei->white_point_x / 50000.;
		codedMedia->info.mdcv.display_primaries_val.white_point.y =
			(float)sei->white_point_y / 50000.;
		codedMedia->info.mdcv.display_primaries =
			vdef_color_primaries_from_values(
				&codedMedia->info.mdcv.display_primaries_val);
		codedMedia->info.mdcv.max_display_mastering_luminance =
			(float)sei->max_display_mastering_luminance / 10000.;
		codedMedia->info.mdcv.min_display_mastering_luminance =
			(float)sei->min_display_mastering_luminance / 10000.;
	}
}


void RecordDemuxer::DemuxerCodedVideoMedia::h265CllSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_content_light_level *sei,
	void *userdata)
{
	DemuxerCodedVideoMedia *self = (DemuxerCodedVideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;

	for (auto m = self->mMedias.begin(); m != self->mMedias.end(); m++) {
		CodedVideoMedia *codedMedia =
			dynamic_cast<CodedVideoMedia *>(*m);
		ULOG_ERRNO_RETURN_IF(codedMedia == nullptr, EPROTO);
		codedMedia->info.cll.max_cll = sei->max_content_light_level;
		codedMedia->info.cll.max_fall =
			sei->max_pic_average_light_level;
	}
}

} /* namespace Pdraw */
