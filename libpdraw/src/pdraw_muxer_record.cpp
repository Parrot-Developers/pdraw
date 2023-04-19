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

#define ULOG_TAG pdraw_recmuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <media-buffers/mbuf_coded_video_frame.h>

#define MP4_TIMESCALE 90000


namespace Pdraw {


#define NB_SUPPORTED_FORMATS 2
static struct vdef_coded_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_h264_avcc;
	supportedFormats[1] = vdef_h265_hvcc;
}


RecordMuxer::RecordMuxer(Session *session,
			 Element::Listener *elementListener,
			 const std::string &fileName) :
		Muxer(session, elementListener),
		mFileName(fileName), mMux(nullptr), mMediaDate(0),
		mHasMetadataTrack(false), mMetaBuffer(nullptr)
{

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);
	setCodedVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);
}


RecordMuxer::~RecordMuxer(void)
{
	int err;

	err = internalStop();

	return;
}


/* Must be called on the loop thread */
int RecordMuxer::addInputMedia(Media *media)
{
	int res;

	/* Only accept coded video media */
	CodedVideoMedia *m = dynamic_cast<CodedVideoMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -ENOSYS;
	}

	res = Muxer::addInputMedia(m);
	if (res < 0)
		return res;

	if (mMux != nullptr) {
		/* Add a track for the media if the muxer is already opened */
		res = addTrackForMedia(m, 0);
		if (res < 0)
			return res;
	}

	return 0;
}


int RecordMuxer::internalStart(void)
{
	int res, inputMediaCount, i;
	uint64_t now;

	mMediaDate = time(nullptr);
	now = mMediaDate;

	/* Create the metadata buffer */
	mMetaBuffer = (uint8_t *)malloc(VMETA_FRAME_MAX_SIZE);
	if (mMetaBuffer == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("malloc", -res);
		return res;
	}

	/* Create the MP4 muxer */
	res = mp4_mux_open(mFileName.c_str(), MP4_TIMESCALE, now, now, &mMux);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_open", -res);
		return res;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Add a track for all existing medias */
	for (i = 0; i < inputMediaCount; i++) {
		CodedVideoMedia *media =
			dynamic_cast<CodedVideoMedia *>(getInputMedia(i));
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = addTrackForMedia(media, now);
		if (res < 0)
			continue;
	}

	Sink::unlock();

	return 0;
}


int RecordMuxer::internalStop(void)
{
	int err;

	if (mMux != nullptr) {
		mergeSessionMetadata();

		/* Finalize the MP4 file */
		err = mp4_mux_close(mMux);
		if (err < 0)
			PDRAW_LOG_ERRNO("mp4_mux_close", -err);
		mMux = nullptr;
	}

	/* Free the metadata buffer */
	free(mMetaBuffer);
	mMetaBuffer = nullptr;

	return 0;
}


void RecordMuxer::mergeSessionMetadata(void)
{
	int err;
	struct vmeta_session fileSessionMeta = {};

	/* First pass: fill fileSessionMeta with identical values */
	bool first = true;
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		CodedVideoMedia *media = it->first;

		if (first) {
			fileSessionMeta = media->sessionMeta;
			first = false;
			continue;
		}

		if (strcmp(fileSessionMeta.friendly_name,
			   media->sessionMeta.friendly_name) != 0)
			fileSessionMeta.friendly_name[0] = '\0';

		if (strcmp(fileSessionMeta.maker, media->sessionMeta.maker) !=
		    0)
			fileSessionMeta.maker[0] = '\0';

		if (strcmp(fileSessionMeta.model, media->sessionMeta.model) !=
		    0)
			fileSessionMeta.model[0] = '\0';

		if (strcmp(fileSessionMeta.model_id,
			   media->sessionMeta.model_id) != 0)
			fileSessionMeta.model_id[0] = '\0';

		if (strcmp(fileSessionMeta.serial_number,
			   media->sessionMeta.serial_number) != 0)
			fileSessionMeta.serial_number[0] = '\0';

		if (strcmp(fileSessionMeta.software_version,
			   media->sessionMeta.software_version) != 0)
			fileSessionMeta.software_version[0] = '\0';

		if (strcmp(fileSessionMeta.build_id,
			   media->sessionMeta.build_id) != 0)
			fileSessionMeta.build_id[0] = '\0';

		if (strcmp(fileSessionMeta.title, media->sessionMeta.title) !=
		    0)
			fileSessionMeta.title[0] = '\0';

		if (strcmp(fileSessionMeta.comment,
			   media->sessionMeta.comment) != 0)
			fileSessionMeta.comment[0] = '\0';

		if (strcmp(fileSessionMeta.copyright,
			   media->sessionMeta.copyright) != 0)
			fileSessionMeta.copyright[0] = '\0';

		if (fileSessionMeta.media_date !=
		    media->sessionMeta.media_date) {
			fileSessionMeta.media_date = 0;
			fileSessionMeta.media_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_date != media->sessionMeta.run_date) {
			fileSessionMeta.run_date = 0;
			fileSessionMeta.run_date_gmtoff = 0;
		}

		if (strcmp(fileSessionMeta.run_id, media->sessionMeta.run_id) !=
		    0)
			fileSessionMeta.run_id[0] = '\0';

		if (strcmp(fileSessionMeta.boot_id,
			   media->sessionMeta.boot_id) != 0)
			fileSessionMeta.boot_id[0] = '\0';

		if (strcmp(fileSessionMeta.flight_id,
			   media->sessionMeta.flight_id) != 0)
			fileSessionMeta.flight_id[0] = '\0';

		if (strcmp(fileSessionMeta.custom_id,
			   media->sessionMeta.custom_id) != 0)
			fileSessionMeta.custom_id[0] = '\0';

		if (memcmp(&fileSessionMeta.takeoff_loc,
			   &media->sessionMeta.takeoff_loc,
			   sizeof(struct vmeta_location)) != 0)
			fileSessionMeta.takeoff_loc.valid = 0;

		if (memcmp(&fileSessionMeta.picture_fov,
			   &media->sessionMeta.picture_fov,
			   sizeof(struct vmeta_fov)) != 0) {
			fileSessionMeta.picture_fov.has_horz = 0;
			fileSessionMeta.picture_fov.has_vert = 0;
		}

		if ((fileSessionMeta.has_thermal !=
		     media->sessionMeta.has_thermal) ||
		    (memcmp(&fileSessionMeta.thermal,
			    &media->sessionMeta.thermal,
			    sizeof(struct vmeta_thermal)) != 0))
			fileSessionMeta.has_thermal = 0;

		if (fileSessionMeta.camera_type !=
		    media->sessionMeta.camera_type)
			fileSessionMeta.camera_type = VMETA_CAMERA_TYPE_UNKNOWN;

		if (fileSessionMeta.video_mode != media->sessionMeta.video_mode)
			fileSessionMeta.video_mode = VMETA_VIDEO_MODE_UNKNOWN;
	}

	/* Second pass: cancel identical values from trackSessionMeta and
	 * write track-level metadata */
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		CodedVideoMedia *media = it->first;
		Track &track = it->second;
		struct vmeta_session trackSessionMeta = media->sessionMeta;

		if (fileSessionMeta.friendly_name[0] != '\0')
			trackSessionMeta.friendly_name[0] = '\0';

		if (fileSessionMeta.maker[0] != '\0')
			trackSessionMeta.maker[0] = '\0';

		if (fileSessionMeta.model[0] != '\0')
			trackSessionMeta.model[0] = '\0';

		if (fileSessionMeta.model_id[0] != '\0')
			trackSessionMeta.model_id[0] = '\0';

		if (fileSessionMeta.serial_number[0] != '\0')
			trackSessionMeta.serial_number[0] = '\0';

		if (fileSessionMeta.software_version[0] != '\0')
			trackSessionMeta.software_version[0] = '\0';

		if (fileSessionMeta.build_id[0] != '\0')
			trackSessionMeta.build_id[0] = '\0';

		if (fileSessionMeta.title[0] != '\0')
			trackSessionMeta.title[0] = '\0';

		if (fileSessionMeta.comment[0] != '\0')
			trackSessionMeta.comment[0] = '\0';

		if (fileSessionMeta.copyright[0] != '\0')
			trackSessionMeta.copyright[0] = '\0';

		if (fileSessionMeta.media_date != 0) {
			trackSessionMeta.media_date = 0;
			trackSessionMeta.media_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_date != 0) {
			trackSessionMeta.run_date = 0;
			trackSessionMeta.run_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_id[0] != '\0')
			trackSessionMeta.run_id[0] = '\0';

		if (fileSessionMeta.boot_id[0] != '\0')
			trackSessionMeta.boot_id[0] = '\0';

		if (fileSessionMeta.flight_id[0] != '\0')
			trackSessionMeta.flight_id[0] = '\0';

		if (fileSessionMeta.custom_id[0] != '\0')
			trackSessionMeta.custom_id[0] = '\0';

		if (fileSessionMeta.takeoff_loc.valid)
			trackSessionMeta.takeoff_loc.valid = 0;

		if (fileSessionMeta.picture_fov.has_horz ||
		    fileSessionMeta.picture_fov.has_vert) {
			trackSessionMeta.picture_fov.has_horz = 0;
			trackSessionMeta.picture_fov.has_vert = 0;
		}

		if (fileSessionMeta.has_thermal)
			trackSessionMeta.has_thermal = 0;

		if (fileSessionMeta.camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
			trackSessionMeta.camera_type =
				VMETA_CAMERA_TYPE_UNKNOWN;
		}

		if (fileSessionMeta.video_mode != VMETA_VIDEO_MODE_UNKNOWN)
			trackSessionMeta.video_mode = VMETA_VIDEO_MODE_UNKNOWN;

		struct SessionMetaWriteTrackCbUserdata ud = {
			.muxer = this,
			.trackId = track.trackId,
		};
		err = vmeta_session_recording_write(
			&trackSessionMeta, &sessionMetaWriteTrackCb, &ud);
		if (err < 0)
			PDRAW_LOG_ERRNO("vmeta_session_recording_write", -err);
	}

	if (fileSessionMeta.media_date == 0) {
		uint64_t mediaDate = 0;
		int32_t mediaDateGmtoff = 0;
		err = time_local_get(&mediaDate, &mediaDateGmtoff);
		if (err < 0)
			PDRAW_LOG_ERRNO("time_local_get", -err);
		fileSessionMeta.media_date = mediaDate;
		fileSessionMeta.media_date_gmtoff = mediaDateGmtoff;
	}
	if (fileSessionMeta.title[0] == '\0') {
		err = time_local_format(fileSessionMeta.media_date,
					fileSessionMeta.media_date_gmtoff,
					TIME_FMT_RFC1123,
					fileSessionMeta.title,
					sizeof(fileSessionMeta.title));
		if (err < 0)
			PDRAW_LOG_ERRNO("time_local_format", -err);
	}

	err = vmeta_session_recording_write(
		&fileSessionMeta, &sessionMetaWriteFileCb, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("vmeta_session_recording_write", -err);
}


int RecordMuxer::addTrackForMedia(CodedVideoMedia *media, uint64_t trackTime)
{
	int res, trackId;
	const uint8_t *vps = nullptr, *sps = nullptr, *pps = nullptr;
	size_t vpsSize = 0, spsSize = 0, ppsSize = 0;
	struct mp4_video_decoder_config cfg = {};

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	if (trackTime == 0)
		trackTime = time(nullptr);

	/* Add a new video track */
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_VIDEO,
		.name = "DefaultVideo", /* TODO */
		.enabled = 1, /* TODO */
		.in_movie = 1, /* TODO */
		.in_preview = 1, /* TODO */
		.timescale = MP4_TIMESCALE, /* TODO */
		.creation_time = trackTime,
		.modification_time = trackTime,
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	trackId = res;
	mTracks.insert(
		{media, Track((uint32_t)trackId, 0, trackTime, INT64_MAX)});

	switch (media->format.encoding) {
	case VDEF_ENCODING_H264:
		/* Set the H.264 parameter sets */
		res = media->getPs(
			nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
		if (res < 0) {
			PDRAW_LOG_ERRNO("CodedVideoMedia::getPs", -res);
			return res;
		}
		cfg.codec = MP4_VIDEO_CODEC_AVC;
		cfg.width = media->info.resolution.width;
		cfg.height = media->info.resolution.height;
		cfg.avc.c_sps = sps;
		cfg.avc.sps_size = spsSize;
		cfg.avc.c_pps = pps;
		cfg.avc.pps_size = ppsSize;
		break;
	case VDEF_ENCODING_H265:
		/* Set the H.265 parameter sets */
		res = media->getPs(
			&vps, &vpsSize, &sps, &spsSize, &pps, &ppsSize);
		if (res < 0) {
			PDRAW_LOG_ERRNO("CodedVideoMedia::getPs", -res);
			return res;
		}
		cfg.codec = MP4_VIDEO_CODEC_HEVC;
		cfg.width = media->info.resolution.width;
		cfg.height = media->info.resolution.height;
		cfg.hevc.c_vps = vps;
		cfg.hevc.vps_size = vpsSize;
		cfg.hevc.c_sps = sps;
		cfg.hevc.sps_size = spsSize;
		cfg.hevc.c_pps = pps;
		cfg.hevc.pps_size = ppsSize;
		/* TODO: fill cfg.hevc.hvcc_info */
		break;
	default:
		break;
	}

	res = mp4_mux_track_set_video_decoder_config(mMux, trackId, &cfg);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_video_decoder_config", -res);
		return res;
	}

	return 0;
}


int RecordMuxer::addMetadataTrack(Track *ref, enum vmeta_frame_type metaType)
{
	int res;
	const char *mimeType = nullptr;
	const char *contentEncoding = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mHasMetadataTrack, EALREADY);

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
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_METADATA,
		.name = "ParrotVideoMetadata", /* TODO */
		.enabled = 1, /* TODO */
		.in_movie = 1, /* TODO */
		.in_preview = 1, /* TODO */
		.timescale = MP4_TIMESCALE, /* TODO */
		.creation_time = ref->trackTime,
		.modification_time = ref->trackTime,
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	ref->metaTrackId = (uint32_t)res;

	/* Set the metadata mime type */
	res = mp4_mux_track_set_metadata_mime_type(
		mMux, ref->metaTrackId, contentEncoding, mimeType);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_metadata_mime_type", -res);
		ref->metaTrackId = 0;
		return res;
	}

	/* Add track reference */
	res = mp4_mux_add_ref_to_track(mMux, ref->metaTrackId, ref->trackId);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track", -res);
		ref->metaTrackId = 0;
		return res;
	}

	mHasMetadataTrack = true;

	return 0;
}


int RecordMuxer::process(void)
{
	int res, inputMediaCount, i;

	if (mState != STARTED)
		return 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	for (i = 0; i < inputMediaCount; i++) {
		CodedVideoMedia *media =
			dynamic_cast<CodedVideoMedia *>(getInputMedia(i));
		if (media == nullptr) {
			res = -ENOENT;
			PDRAW_LOG_ERRNO("getInputMedia", -res);
			continue;
		}

		/* Process each media */
		processMedia(media);
	}

	Sink::unlock();

	return 0;
}


int RecordMuxer::processMedia(CodedVideoMedia *media)
{
	int res, err;
	struct mbuf_coded_video_frame *frame = nullptr;
	Track &track = mTracks[media];

	if (track.trackId == 0) {
		res = -ENOENT;
		PDRAW_LOG_ERRNO("track->trackId", -res);
		return res;
	}
	CodedVideoChannel *channel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Channel::getQueue", -res);
		return res;
	}

	/* TODO: This loops drops frames if the processFrame function fails.
	 * This is a problem for most coded streams, so the current behavior
	 * will result in a buggy output file.
	 * We should instead check what the error was, and decide to either:
	 * - Retry later, which can be achieved by using queue_peek() instead of
	 *   queue_pop(), or
	 * - Discard the frame, flush the queue, and ask the upstream elements
	 *   for a complete resync (or mabye even terminate the record and warn
	 *   the app ?)
	 * The second choice is important to have, because if an error persists,
	 * then this whole element will be stuck on the buggy frame. */
	do {
		res = mbuf_coded_video_frame_queue_pop(queue, &frame);
		if (res < 0) {
			if (res != -EAGAIN)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-res);
			continue;
		}

		/* Process each buffer */
		res = processFrame(media, &track, frame);

		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	} while (res == 0);

	return 0;
}


int RecordMuxer::processFrame(CodedVideoMedia *media,
			      Track *track,
			      struct mbuf_coded_video_frame *frame)
{
	int res;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	struct vmeta_frame *metadata = nullptr;
	CodedVideoMedia::Frame *meta;
	struct mp4_mux_scattered_sample sample = {};
	const void *aData;
	const void **frameNalus = nullptr;
	size_t *frameNalusSize = nullptr;

	sample.nbuffers = mbuf_coded_video_frame_get_nalu_count(frame);
	if (sample.nbuffers < 0) {
		res = sample.nbuffers;
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		goto out;
	}
	frameNalus =
		(const void **)calloc(sample.nbuffers, sizeof(*sample.buffers));
	frameNalusSize = (size_t *)calloc(sample.nbuffers, sizeof(*sample.len));
	if (!frameNalus || !frameNalusSize) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -res);
		goto out;
	}

	for (int i = 0; i < sample.nbuffers; i++) {
		struct vdef_nalu nalu;
		res = mbuf_coded_video_frame_get_nalu(
			frame, i, &frameNalus[i], &nalu);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_coded_video_frame_get_nalu(%d)", -res, i);
			goto out;
		}
		frameNalusSize[i] = nalu.size;
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
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, NULL);
	meta = (CodedVideoMedia::Frame *)aData;

	/* Add a video sample to the MP4 muxer */
	sample.buffers = (const uint8_t *const *)frameNalus;
	sample.len = (const size_t *)frameNalusSize;
	sample.sync = meta->isSync;
	sample.dts =
		mp4_usec_to_sample_time(meta->ntpRawTimestamp, MP4_TIMESCALE);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->lastSampleTs != INT64_MAX) &&
	    (sample.dts == track->lastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->lastSampleTs);
		sample.dts = track->lastSampleTs + 1;
	} else if ((track->lastSampleTs != INT64_MAX) &&
		   (sample.dts < track->lastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->lastSampleTs,
			   sample.dts);
		sample.dts = track->lastSampleTs + 1;
	}
	track->lastSampleTs = sample.dts;

	res = mp4_mux_track_add_scattered_sample(mMux, track->trackId, &sample);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_add_scattered_sample", -res);
		goto out;
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
	if (!mHasMetadataTrack)
		addMetadataTrack(track, metadata->type);
	if (track->metaTrackId) {
		/* Add a metadata sample to the MP4 muxer */
		const uint8_t *metaContent = nullptr;
		size_t metaLen = 0;
		if (metadata->type != VMETA_FRAME_TYPE_PROTO) {
			struct vmeta_buffer metaBuf;
			vmeta_buffer_set_data(
				&metaBuf, mMetaBuffer, VMETA_FRAME_MAX_SIZE, 0);
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
				mMux, track->metaTrackId, &metaSample);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
				goto out;
			}
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
	if (frame && frameNalus) {
		for (int i = 0; i < sample.nbuffers; i++) {
			if (!frameNalus[i])
				continue;
			int err = mbuf_coded_video_frame_release_nalu(
				frame, i, frameNalus[i]);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_release_nalu",
					-err);
		}
	}
	free(frameNalus);
	free(frameNalusSize);

	return res;
}


void RecordMuxer::sessionMetaWriteFileCb(enum vmeta_record_type type,
					 const char *key,
					 const char *value,
					 void *userdata)
{
	int res;
	RecordMuxer *self = (RecordMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	res = mp4_mux_add_file_metadata(self->mMux, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_file_metadata", -res);
}


void RecordMuxer::sessionMetaWriteTrackCb(enum vmeta_record_type type,
					  const char *key,
					  const char *value,
					  void *userdata)
{
	int res;
	struct SessionMetaWriteTrackCbUserdata *ud =
		(struct SessionMetaWriteTrackCbUserdata *)userdata;

	ULOG_ERRNO_RETURN_IF(ud == nullptr, EINVAL);

	RecordMuxer *self = ud->muxer;
	uint32_t trackId = ud->trackId;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	res = mp4_mux_add_track_metadata(self->mMux, trackId, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_track_metadata", -res);
}

} /* namespace Pdraw */
