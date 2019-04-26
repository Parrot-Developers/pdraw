/**
 * Parrot Drones Awesome Video Viewer Library
 * Recording demuxer
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

#include "pdraw_demuxer_record.hpp"
#include "pdraw_session.hpp"

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <string>

#include <video-streaming/vstrm.h>
#define ULOG_TAG pdraw_dmxrec
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_dmxrec);

namespace Pdraw {


const struct h264_ctx_cbs RecordDemuxer::mH264Cbs = {
	.nalu_begin = NULL,
	.nalu_end = NULL,
	.slice = NULL,
	.slice_data_begin = NULL,
	.slice_data_end = NULL,
	.slice_data_mb = NULL,
	.sps = NULL,
	.pps = NULL,
	.aud = NULL,
	.sei = NULL,
	.sei_buffering_period = NULL,
	.sei_pic_timing = &RecordDemuxer::h264PicTimingSeiCb,
	.sei_pan_scan_rect = NULL,
	.sei_filler_payload = NULL,
	.sei_user_data_registered = NULL,
	.sei_user_data_unregistered = &RecordDemuxer::h264UserDataSeiCb,
	.sei_recovery_point = NULL,
};


RecordDemuxer::RecordDemuxer(Session *session,
			     Element::Listener *elementListener,
			     Source::Listener *sourceListener,
			     Demuxer::Listener *demuxerListener) :
		Demuxer(session,
			elementListener,
			sourceListener,
			demuxerListener)
{
	int ret;

	Element::mName = "RecordDemuxer";
	Source::mName = "RecordDemuxer";
	mFirstFrame = true;
	mVideoMedia = NULL;
	mDemux = NULL;
	mTimer = NULL;
	mH264Reader = NULL;
	mRunning = false;
	mFrameByFrame = false;
	mVideoTrackId = 0;
	mMetadataMimeType = NULL;
	mAvgOutputInterval = 0;
	mLastFrameOutputTime = 0;
	mLastFrameDuration = 0;
	mLastOutputError = 0;
	mDuration = 0;
	mCurrentTime = 0;
	mPendingSeekTs = -1;
	mPendingSeekExact = false;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	mCurrentBuffer = NULL;
	mCurrentBufferCaptureTs = 0;
	mDecodingTs = 0;
	mDecodingTsInc = 0;
	mSpeed = 1.0;
	mHfov = 0.;
	mVfov = 0.;
	mFrameByFrame = true;
	mSeekResponse = 0;

	mMetadataBufferSize = 1024;
	mMetadataBuffer = (uint8_t *)malloc(mMetadataBufferSize);
	if (mMetadataBuffer == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		goto err;
	}

	/* Create the demux timer */
	mTimer = pomp_timer_new(mSession->getLoop(), timerCb, this);
	if (mTimer == NULL) {
		ULOGE("pomp_timer_new failed");
		goto err;
	}

	/* Create the H.264 reader */
	ret = h264_reader_new(&mH264Cbs, this, &mH264Reader);
	if (ret < 0) {
		ULOG_ERRNO("h264_reader_new", -ret);
		goto err;
	}

	setState(CREATED);
	return;

err:
	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}
	if (mH264Reader != NULL) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = NULL;
	}
	free(mMetadataBuffer);
	mMetadataBuffer = NULL;
}


RecordDemuxer::~RecordDemuxer(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		ULOGW("demuxer is still running");

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	if (mVideoMedia != NULL)
		ULOGW("output media was not properly removed");

	if (mDemux != NULL) {
		ret = mp4_demux_close(mDemux);
		if (ret < 0)
			ULOG_ERRNO("mp4_demux_close", -ret);
		mDemux = NULL;
	}

	if (mTimer != NULL) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_destroy", -ret);
		mTimer = NULL;
	}

	if (mH264Reader != NULL) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = NULL;
	}

	free(mMetadataBuffer);
	mMetadataBuffer = NULL;
	free(mMetadataMimeType);
	mMetadataMimeType = NULL;
}


int RecordDemuxer::fetchSessionMetadata(void)
{
	SessionPeerMetadata *peerMeta = mSession->getPeerMetadata();
	int ret;
	unsigned int count = 0, i;
	char **keys = NULL, *key;
	char **values = NULL, *value;
	struct vmeta_session meta;
	memset(&meta, 0, sizeof(meta));

	/* File-level session metadata */
	ret = mp4_demux_get_metadata_strings(mDemux, &count, &keys, &values);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_metadata_strings", -ret);
		return ret;
	}
	for (i = 0; i < count; i++) {
		key = keys[i];
		value = values[i];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, &meta);
			if (ret < 0) {
				ULOG_ERRNO("vmeta_session_recording_read",
					   -ret);
				continue;
			}
		}
	}

	/* Track-level session metadata */
	ret = mp4_demux_get_track_metadata_strings(
		mDemux, mVideoTrackId, &count, &keys, &values);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_track_metadata_strings", -ret);
		return ret;
	}
	for (i = 0; i < count; i++) {
		key = keys[i];
		value = values[i];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, &meta);
			if (ret < 0) {
				ULOG_ERRNO("vmeta_session_recording_read",
					   -ret);
				continue;
			}
		}
	}

	peerMeta->set(&meta);
	if (meta.picture_fov.has_horz)
		mHfov = meta.picture_fov.horz;
	if (meta.picture_fov.has_vert)
		mVfov = meta.picture_fov.vert;

	return 0;
}


int RecordDemuxer::addVideoTrack(struct mp4_track_info *tkinfo)
{
	int ret;
	uint8_t *sps = NULL, *pps = NULL;
	unsigned int spsSize = 0, ppsSize = 0;

	Source::lock();
	if (mVideoMedia != NULL) {
		Source::unlock();
		ULOGE("video track already defined");
		return -EEXIST;
	}

	mVideoTrackId = tkinfo->id;
	mTimescale = tkinfo->timescale;
	ret = mp4_demux_get_track_avc_decoder_config(
		mDemux, mVideoTrackId, &sps, &spsSize, &pps, &ppsSize);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("mp4_demux_get_track_avc_decoder_config", -ret);
		return ret;
	}
	if ((sps == NULL) || (spsSize == 0)) {
		Source::unlock();
		ULOGE("invalid SPS");
		return -EPROTO;
	}
	if ((pps == NULL) || (ppsSize == 0)) {
		Source::unlock();
		ULOGE("invalid PPS");
		return -EPROTO;
	}

	/* Create the output port */
	mVideoMedia = new VideoMedia(mSession);
	if (mVideoMedia == NULL) {
		Source::unlock();
		ULOGE("media allocation failed");
		return -ENOMEM;
	}
	ret = addOutputPort(mVideoMedia);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	/* Set the output media info */
	switch (tkinfo->video_codec) {
	case MP4_VIDEO_CODEC_AVC:
		mVideoMedia->format = VideoMedia::Format::H264;
		break;
	default:
		mVideoMedia->format = VideoMedia::Format::FORMAT_UNKNOWN;
		break;
	}

	if (tkinfo->has_metadata)
		mMetadataMimeType = strdup(tkinfo->metadata_mime_format);

	ret = mVideoMedia->setSpsPps(sps, spsSize, pps, ppsSize);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("media->setSpsPps", -ret);
		return ret;
	}

	mVideoMedia->hfov = mHfov;
	mVideoMedia->vfov = mVfov;

	/* Initialize the H.264 parsing */
	ret = h264_reader_parse_nalu(mH264Reader, 0, sps, spsSize);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("h264_reader_parse_nalu", -ret);
		return ret;
	}
	ret = h264_reader_parse_nalu(mH264Reader, 0, pps, ppsSize);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("h264_reader_parse_nalu", -ret);
		return ret;
	}

	/* Create the output buffers pool */
	ret = createOutputPortBuffersPool(mVideoMedia,
					  DEMUXER_OUTPUT_BUFFER_COUNT,
					  mVideoMedia->width *
						  mVideoMedia->height * 3 / 4);
	if (ret < 0) {
		Source::unlock();
		ULOG_ERRNO("createOutputBuffersPool", -ret);
		return ret;
	}

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(this, mVideoMedia);

	return 0;
}


int RecordDemuxer::setup(const std::string &fileName)
{
	if (mState != CREATED) {
		ULOGE("invalid state");
		return -EPROTO;
	}

	mFileName = fileName;

	return 0;
}


int RecordDemuxer::start(void)
{
	int ret;

	int i, tkCount = 0;
	struct mp4_media_info info;
	struct mp4_track_info tk;
	struct pdraw_demuxer_media *medias = NULL;
	struct pdraw_demuxer_media *selectedMedia = NULL;
	size_t mediasCount = 0;
	int defaultMediaIndex = -1;

	unsigned int hrs = 0, min = 0, sec = 0;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		ULOGE("demuxer is not created");
		return -EPROTO;
	}
	setState(STARTING);

	/* Create the MP4 demuxer */
	ret = mp4_demux_open(mFileName.c_str(), &mDemux);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_open", -ret);
		goto exit;
	}

	ret = mp4_demux_get_media_info(mDemux, &info);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_media_info", -ret);
		goto exit;
	}

	mDuration = info.duration;
	tkCount = info.track_count;
	ULOGD("track count: %d", tkCount);
	pdraw_friendlyTimeFromUs(info.duration, &hrs, &min, &sec, NULL);
	ULOGD("duration: %02d:%02d:%02d", hrs, min, sec);

	/* List all tracks */
	for (i = 0; i < tkCount; i++) {
		ret = mp4_demux_get_track_info(mDemux, i, &tk);
		if (ret != 0 || tk.type != MP4_TRACK_TYPE_VIDEO)
			continue;
		struct pdraw_demuxer_media *tmp =
			(struct pdraw_demuxer_media *)realloc(
				medias, (mediasCount + 1) * sizeof(*medias));
		if (!tmp) {
			ret = -ENOMEM;
			goto exit;
		}
		mediasCount++;
		medias = tmp;
		struct pdraw_demuxer_media *current = &medias[mediasCount - 1];
		memset(current, 0, sizeof(*current));
		current->media_id = tk.id;
		current->idx = i;
		current->name = strdup(tk.name);
		current->is_default = tk.enabled;
		if (current->is_default && defaultMediaIndex == -1)
			defaultMediaIndex = mediasCount - 1;
	}
	if (mediasCount == 0) {
		ULOGE("no video track");
		ret = -ENOENT;
		goto exit;
	}

	/* Ask which media to use from the application */
	ret = mDemuxerListener->selectDemuxerMedia(this, medias, mediasCount);
	if (ret < 0 && ret != -ENOSYS) {
		ULOG_ERRNO("application failed to select a video media", -ret);
		ret = -EPROTO;
		goto exit;
	} else if (ret == 0 || ret == -ENOSYS) {
		if (defaultMediaIndex == -1) {
			ULOGE("application requested default media, "
			      "but no default media found");
			ret = -ENOENT;
			goto exit;
		}
		selectedMedia = &medias[defaultMediaIndex];
		ULOGI("auto-selecting media %d (%s)",
		      selectedMedia->media_id,
		      selectedMedia->name);
	} else {
		selectedMedia = NULL;
		for (size_t j = 0; j < mediasCount; j++) {
			if (medias[j].media_id == ret) {
				selectedMedia = &medias[j];
				break;
			}
		}
		if (!selectedMedia) {
			ULOGE("application requested media %d, "
			      "but it is not a valid video media",
			      ret);
			ret = -ENOENT;
			goto exit;
		}
		ULOGI("application selected media %d (%s)",
		      selectedMedia->media_id,
		      selectedMedia->name);
	}

	/* Create the output ports from selected media */
	ret = mp4_demux_get_track_info(mDemux, selectedMedia->idx, &tk);
	if (ret != 0) {
		ULOG_ERRNO("mp4_demux_get_track_info", -ret);
		goto exit;
	}
	ret = addVideoTrack(&tk);

	/* Get the session metadata */
	ret = fetchSessionMetadata();
	if (ret < 0) {
		ULOG_ERRNO("fetchSessionMetadata", -ret);
		goto exit;
	}

exit:
	/* Cleanup track list */
	for (size_t j = 0; j < mediasCount; j++) {
		free((void *)medias[j].name);
	}
	free(medias);

	if (ret == 0) {
		setState(STARTED);
		mDemuxerListener->onReadyToPlay(this, true);
		/* TODO: Put back onReadyToPlay to false at end of file */
	} else {
		setState(CREATED);
	}
	return ret;
}


int RecordDemuxer::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED && mState != STARTING) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}
	setState(STOPPING);

	mDemuxerListener->onReadyToPlay(this, false);

	mRunning = false;
	pomp_timer_clear(mTimer);

	Source::lock();

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	ret = flush();
	if (ret < 0)
		ULOG_ERRNO("flush", -ret);

	Source::unlock();

	return 0;
}


int RecordDemuxer::flush(void)
{
	int ret;
	unsigned int outputChannelCount = 0, i;
	Channel *channel;

	if ((mState != STARTED) && (mState != STOPPING)) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	Source::lock();

	if (mCurrentBuffer != NULL) {
		ret = vbuf_unref(mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		mCurrentBuffer = NULL;
	}

	if (mVideoMedia != NULL)
		outputChannelCount = getOutputChannelCount(mVideoMedia);

	/* Flush the output channels */
	for (i = 0; i < outputChannelCount; i++) {
		channel = getOutputChannel(mVideoMedia, i);
		if (channel == NULL) {
			ULOGW("failed to get channel at index %d", i);
			continue;
		}
		ret = channel->flush();
		if (ret < 0)
			ULOG_ERRNO("channel->flush", -ret);
	}

	Source::unlock();

	return 0;
}


void RecordDemuxer::onChannelFlushed(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}
	ULOGD("'%s': channel flushed media id=%d type=%s (channel key=%p)",
	      Source::mName.c_str(),
	      media->id,
	      Media::getMediaTypeStr(media->type),
	      channel->getKey());

	if (mState == STOPPING)
		channel->teardown();
}


void RecordDemuxer::onChannelUnlink(Channel *channel)
{
	if (channel == NULL) {
		ULOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == NULL) {
		ULOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel->getKey());
	if (ret < 0)
		ULOG_ERRNO("removeOutputChannel", -ret);

	completeTeardown();
}


void RecordDemuxer::completeTeardown(void)
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();

	if (mVideoMedia == NULL) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mVideoMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener)
		Source::mListener->onOutputMediaRemoved(this, mVideoMedia);
	ret = removeOutputPort(mVideoMedia);
	if (ret < 0) {
		ULOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mVideoMedia;
		mVideoMedia = NULL;
	}

	Source::unlock();

exit:
	if (mState == STOPPING)
		setState(STOPPED);
}


int RecordDemuxer::play(float speed)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (speed == 0.) {
		/* speed is null => pause */
		mRunning = false;
		mFrameByFrame = true;
		mDemuxerListener->pauseResp(this, 0, getCurrentTime());
	} else {
		mRunning = true;
		mFrameByFrame = false;
		mPendingSeekToPrevSample = false;
		mPendingSeekToNextSample = false;
		mSpeed = speed;
		pomp_timer_set(mTimer, 1);
		mDemuxerListener->playResp(this, 0, getCurrentTime(), mSpeed);
	}

	return 0;
}


bool RecordDemuxer::isPaused(void)
{
	if (mState != STARTED) {
		ULOG_ERRNO("demuxer is not started", EPROTO);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int RecordDemuxer::previous(void)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		ULOGE("demuxer is not paused");
		return -EPROTO;
	}

	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeekToPrevSample = true;
		mPendingSeekToNextSample = false;
		mPendingSeekExact = true;
		mRunning = true;
		pomp_timer_set(mTimer, 1);
	}

	return 0;
}


int RecordDemuxer::next(void)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not configured");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		ULOGE("demuxer is not paused");
		return -EPROTO;
	}

	mPendingSeekToNextSample = true;
	mPendingSeekToPrevSample = false;
	mRunning = true;
	pomp_timer_set(mTimer, 1);

	return 0;
}


int RecordDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	int64_t ts = (int64_t)mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDuration)
		ts = mDuration;
	return seekTo(ts, exact);
}


int RecordDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != STARTED) {
		ULOGE("demuxer is not started");
		return -EPROTO;
	}

	if (timestamp > mDuration)
		timestamp = mDuration;
	mPendingSeekTs = (int64_t)timestamp;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	mRunning = true;
	pomp_timer_set(mTimer, 1);

	return 0;
}


void RecordDemuxer::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	RecordDemuxer *demuxer = (RecordDemuxer *)userdata;
	int ret = 0;

	if (demuxer == NULL)
		return;
	if ((buf == NULL) || (len == 0))
		return;
	if (sei == NULL)
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	/* Ignore "Parrot Streaming" v1 and v2 user data SEI */
	if ((vstrm_h264_sei_streaming_is_v1(sei->uuid)) ||
	    (vstrm_h264_sei_streaming_is_v2(sei->uuid)))
		return;

	ret = vbuf_set_userdata_capacity(demuxer->mCurrentBuffer, len);
	if (ret < (signed)len) {
		ULOG_ERRNO("vbuf_set_userdata_capacity", -ret);
		return;
	}

	uint8_t *dstBuf = vbuf_get_userdata(demuxer->mCurrentBuffer);
	memcpy(dstBuf, buf, len);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, len);
}


void RecordDemuxer::h264PicTimingSeiCb(struct h264_ctx *ctx,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_sei_pic_timing *sei,
				       void *userdata)
{
	RecordDemuxer *demuxer = (RecordDemuxer *)userdata;
	const struct h264_sps *sps;
	uint64_t clock_timestamp;

	if (demuxer == NULL)
		return;
	if (ctx == NULL)
		return;
	if ((buf == NULL) || (len == 0))
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	sps = h264_ctx_get_sps(ctx);

	clock_timestamp =
		(((uint64_t)sei->clk_ts[0].hours_value * 60 +
		  sei->clk_ts[0].minutes_value) *
			 60 +
		 sei->clk_ts[0].seconds_value) *
			sps->vui.time_scale +
		((uint64_t)sei->clk_ts[0].n_frames *
		 ((uint64_t)sps->vui.num_units_in_tick *
		  (1 + (uint64_t)sei->clk_ts[0].nuit_field_based_flag)));

	if (sei->clk_ts[0].time_offset < 0 &&
	    ((uint64_t)-sei->clk_ts[0].time_offset > clock_timestamp))
		clock_timestamp = 0;
	else
		clock_timestamp += sei->clk_ts[0].time_offset;

	demuxer->mCurrentBufferCaptureTs =
		(clock_timestamp * 1000000 + sps->vui.time_scale / 2) /
		sps->vui.time_scale;
}


void RecordDemuxer::timerCb(struct pomp_timer *timer, void *userdata)
{
	RecordDemuxer *demuxer = (RecordDemuxer *)userdata;
	bool silent = false, byteStreamRequired = false;
	float speed = 1.0;
	struct mp4_track_sample sample;
	VideoMedia::Frame *data = NULL;
	uint8_t *buf = NULL, *tmp, *sei = NULL;
	size_t bufSize = 0, offset, naluSize, seiSize = 0;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	int64_t error, duration, wait = 0;
	uint32_t waitMs = 0;
	int ret, retry = 0;
	uint32_t start = htonl(0x00000001);
	unsigned int outputChannelCount = 0, i;
	Channel *channel;
	struct vbuf_pool *pool;
	VideoMedia::H264BitstreamFormat format;
	int didSeek = 0;

	if (demuxer == NULL) {
		return;
	}
	if (demuxer->mState != STARTED) {
		ULOGE("demuxer is not started");
		return;
	}

	speed = demuxer->mSpeed;

	if (!demuxer->mRunning) {
		demuxer->mLastFrameDuration = 0;
		demuxer->mLastOutputError = 0;
		return;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	memset(&sample, 0, sizeof(sample));

	/* Seeking */
	if (demuxer->mPendingSeekTs >= 0) {
		ret = mp4_demux_seek(demuxer->mDemux,
				     (uint64_t)demuxer->mPendingSeekTs,
				     MP4_SEEK_METHOD_PREVIOUS_SYNC);
		if (ret < 0) {
			ULOG_ERRNO("mp4_demux_seek", -ret);
		} else {
			demuxer->mLastFrameDuration = 0;
			demuxer->mLastOutputError = 0;
		}
		demuxer->mSeekResponse = ret;
		didSeek = 1;
	} else if (demuxer->mPendingSeekToPrevSample) {
		ret = mp4_demux_seek_to_track_prev_sample(
			demuxer->mDemux, demuxer->mVideoTrackId);
		if (ret != 0) {
			ULOG_ERRNO("mp4_demux_seek_to_track_prev_sample", -ret);
		} else {
			demuxer->mLastFrameDuration = 0;
			demuxer->mLastOutputError = 0;
		}
		demuxer->mSeekResponse = ret;
		/* If not error, seek to prev sample only finishes when
		 * mPendingSeekExact goes back to false */
		if (ret != 0)
			didSeek = 1;
	} else if (demuxer->mPendingSeekToNextSample) {
		/* Cannot fail, as there is nothing to do */
		demuxer->mSeekResponse = 0;
		didSeek = 1;
	}

	demuxer->Source::lock();

	/* Get an output buffer */
	if (demuxer->mCurrentBuffer != NULL) {
		ret = vbuf_unref(demuxer->mCurrentBuffer);
		if (ret < 0)
			ULOG_ERRNO("vbuf_unref", -ret);
		demuxer->mCurrentBuffer = NULL;
	}
	ret = demuxer->getH264OutputBuffer(demuxer->mVideoMedia,
					   &demuxer->mCurrentBuffer,
					   &byteStreamRequired);
	if ((ret < 0) || (demuxer->mCurrentBuffer == NULL)) {
		/* TODO: flush the output and resync */
		ULOGW("failed to get an input buffer (%d)", ret);
		retry = 1;
		goto out;
	}
	buf = vbuf_get_data(demuxer->mCurrentBuffer);
	bufSize = vbuf_get_capacity(demuxer->mCurrentBuffer);
	demuxer->mCurrentBufferCaptureTs = 0;

	/* Get a sample */
	ret = mp4_demux_get_track_sample(demuxer->mDemux,
					 demuxer->mVideoTrackId,
					 1,
					 buf,
					 bufSize,
					 demuxer->mMetadataBuffer,
					 demuxer->mMetadataBufferSize,
					 &sample);
	if (ret != 0) {
		ULOG_ERRNO("mp4_demux_get_track_sample", -ret);
		if (ret == -ENOBUFS) {
			/* Go to the next sample */
			ret = mp4_demux_get_track_sample(demuxer->mDemux,
							 demuxer->mVideoTrackId,
							 1,
							 NULL,
							 0,
							 NULL,
							 0,
							 &sample);
			if (ret != 0)
				ULOG_ERRNO("mp4_demux_get_track_sample", -ret);
		}
		retry = 1;
		goto out;
	}
	if (sample.size == 0) {
		if (demuxer->mFrameByFrame)
			demuxer->mRunning = false;
		goto out;
	}
	vbuf_set_size(demuxer->mCurrentBuffer, sample.size);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, 0);
	silent = ((sample.silent) && (demuxer->mPendingSeekExact)) ? true
								   : false;
	demuxer->mPendingSeekTs = -1;
	demuxer->mPendingSeekToPrevSample = false;
	demuxer->mPendingSeekToNextSample = false;
	/* Previous frame seek end on the first non-silent frame */
	if (demuxer->mPendingSeekExact && !silent)
		didSeek = 1;
	demuxer->mPendingSeekExact =
		(silent) ? demuxer->mPendingSeekExact : false;

	/* Parse the H.264 SEI to find user data SEI */
	tmp = buf;
	offset = 0;
	while (offset < sample.size) {
		memcpy(&naluSize, tmp, sizeof(uint32_t));
		naluSize = ntohl(naluSize);
		if ((*(tmp + 4) & 0x1F) == H264_NALU_TYPE_SEI) {
			sei = tmp + 4;
			seiSize = naluSize;
			break;
		}
		tmp += 4 + naluSize;
		offset += 4 + naluSize;
	}
	if ((sei != NULL) && (seiSize != 0)) {
		ret = h264_reader_parse_nalu(
			demuxer->mH264Reader, 0, sei, seiSize);
		if (ret < 0) {
			ULOG_ERRNO("h264_reader_parse_nalu", -ret);
		}
	}

	/* Set the metadata */
	ret = vbuf_metadata_add(demuxer->mCurrentBuffer,
				demuxer->mVideoMedia,
				1,
				sizeof(*data),
				(uint8_t **)&data);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_add", -ret);
		goto out;
	}
	data->format = VideoMedia::Format::H264;
	data->h264Frame.isComplete = true; /* TODO? */
	data->hasErrors = false; /* TODO? */
	data->h264Frame.isSync = sample.sync;
	data->h264Frame.isRef = true; /* TODO? */
	data->isSilent = silent;
	data->ntpTimestamp = demuxer->mDecodingTs;
	data->ntpUnskewedTimestamp = demuxer->mDecodingTs;
	data->ntpRawTimestamp = demuxer->mDecodingTs;
	data->ntpRawUnskewedTimestamp = demuxer->mDecodingTs;
	if (sample.next_dts > 0) {
		demuxer->mDecodingTsInc = mp4_sample_time_to_usec(
			sample.next_dts - sample.dts, demuxer->mTimescale);
	}
	demuxer->mDecodingTs += demuxer->mDecodingTsInc;
	/* TODO: auSyncType */

	/* Frame metadata */
	data->hasMetadata = false;
	if (sample.metadata_size > 0) {
		struct vmeta_buffer meta_buf;
		vmeta_buffer_set_cdata(&meta_buf,
				       demuxer->mMetadataBuffer,
				       sample.metadata_size,
				       0);

		ret = vmeta_frame_read(
			&meta_buf, &data->metadata, demuxer->mMetadataMimeType);
		if (ret < 0)
			ULOG_ERRNO("vmeta_frame_read", -ret);
		else
			data->hasMetadata = true;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	data->demuxOutputTimestamp = curTime;
	data->playTimestamp =
		mp4_sample_time_to_usec(sample.dts, demuxer->mTimescale);
	data->captureTimestamp = demuxer->mCurrentBufferCaptureTs;
	data->localTimestamp = data->demuxOutputTimestamp;
	demuxer->mCurrentTime = data->playTimestamp;

	if (didSeek) {
		demuxer->mDemuxerListener->seekResp(demuxer,
						    demuxer->mSeekResponse,
						    demuxer->mCurrentTime,
						    demuxer->mSpeed);
	}

	/* Convert to byte stream if necessary */
	if (byteStreamRequired) {
		tmp = buf;
		offset = 0;
		while (offset < sample.size) {
			naluSize = ntohl(*((uint32_t *)tmp));
			memcpy(tmp, &start, sizeof(uint32_t));
			tmp += 4 + naluSize;
			offset += 4 + naluSize;
		}
		format = VideoMedia::H264BitstreamFormat::BYTE_STREAM;
	} else {
		format = VideoMedia::H264BitstreamFormat::AVCC;
	}
	data->h264Frame.format = format;
	ret = vbuf_write_lock(demuxer->mCurrentBuffer);
	if (ret < 0)
		ULOG_ERRNO("vbuf_write_lock", -ret);

	/* Queue the buffer in the output channels */
	outputChannelCount =
		demuxer->getOutputChannelCount(demuxer->mVideoMedia);
	for (i = 0; i < outputChannelCount; i++) {
		channel = demuxer->getOutputChannel(demuxer->mVideoMedia, i);
		if (channel == NULL) {
			ULOGW("invalid channel");
			continue;
		}
		pool = channel->getPool();

		if ((!(channel->getVideoMediaSubFormatCaps() & format)) &&
		    (pool == NULL)) {
			ULOGW("incompatible sub-format on channel");
			continue;
		}

		ret = channel->queue(demuxer->mCurrentBuffer);
		if (ret < 0) {
			ULOG_ERRNO("channel->queue", -ret);
		}
	}
	if ((demuxer->mFirstFrame) && (!data->isSilent)) {
		ret = demuxer->Source::sendDownstreamEvent(
			demuxer->mVideoMedia, Channel::DownstreamEvent::SOS);
		if (ret < 0)
			ULOG_ERRNO("sendDownstreamEvent", -ret);
		demuxer->mFirstFrame = false;
	}

	vbuf_unref(demuxer->mCurrentBuffer);
	demuxer->mCurrentBuffer = NULL;

	if ((demuxer->mFrameByFrame) && (!silent))
		demuxer->mRunning = false;

out:
#define PREV_SAMPLE_TIME_BEFORE mp4_demux_get_track_prev_sample_time_before
#define NEXT_SAMPLE_TIME_AFTER mp4_demux_get_track_next_sample_time_after
	if (retry) {
		waitMs = 5;
	} else if (demuxer->mRunning) {
		/* Schedule the next sample */
		uint64_t nextSampleDts = mp4_sample_time_to_usec(
			sample.next_dts, demuxer->mTimescale);

		/* If error > 0 we are late, if error < 0 we are early */
		error = ((demuxer->mLastFrameOutputTime == 0) ||
			 (demuxer->mLastFrameDuration == 0) || (speed == 0.) ||
			 (speed >= PDRAW_PLAY_SPEED_MAX) || (silent))
				? 0
				: curTime - demuxer->mLastFrameOutputTime -
					  demuxer->mLastFrameDuration +
					  demuxer->mLastOutputError;
		if (demuxer->mLastFrameOutputTime) {
			/* Average frame output rate
			 * (sliding average, alpha = 1/2) */
			demuxer->mAvgOutputInterval +=
				((int64_t)(curTime -
					   demuxer->mLastFrameOutputTime) -
				 demuxer->mAvgOutputInterval) >>
				1;
		}

		/* Sample duration */
		if ((speed >= PDRAW_PLAY_SPEED_MAX) || (nextSampleDts == 0) ||
		    (silent)) {
			duration = 0;
		} else if (speed < 0.) {
			/* Negative speed => play backward */
			nextSampleDts = mp4_sample_time_to_usec(
				sample.prev_sync_dts, demuxer->mTimescale);
			uint64_t pendingSeekTs = nextSampleDts;
			uint64_t nextSyncSampleDts = nextSampleDts;
			uint64_t wantedSampleDts;
			duration = nextSampleDts -
				   mp4_sample_time_to_usec(sample.dts,
							   demuxer->mTimescale);
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				wantedSampleDts = nextSyncSampleDts;
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				ret = PREV_SAMPLE_TIME_BEFORE(
					demuxer->mDemux,
					demuxer->mVideoTrackId,
					wantedSampleDts,
					1,
					&nextSyncSampleDts);
				if (ret < 0) {
					ULOG_ERRNO(
						"mp4_demux_get_track_"
						"prev_sample_time_before",
						-ret);
				}
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration =
						nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts,
							demuxer->mTimescale);
					if (speed != 0.) {
						newDuration = (int64_t)(
							(float)newDuration /
							speed);
					}
				} else {
					break;
				}
			}
			if (pendingSeekTs > 0) {
				duration = newDuration;
				nextSampleDts = nextSyncSampleDts;
				ret = mp4_demux_seek(
					demuxer->mDemux,
					pendingSeekTs,
					MP4_SEEK_METHOD_PREVIOUS_SYNC);
				if (ret < 0) {
					ULOG_ERRNO("mp4_demux_seek", -ret);
				}
			}
		} else {
			/* Positive speed => play forward */
			uint64_t pendingSeekTs = 0;
			uint64_t nextSyncSampleDts = nextSampleDts;
			uint64_t wantedSampleDts;
			duration = nextSampleDts -
				   mp4_sample_time_to_usec(sample.dts,
							   demuxer->mTimescale);
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				wantedSampleDts = nextSyncSampleDts;
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				ret = NEXT_SAMPLE_TIME_AFTER(
					demuxer->mDemux,
					demuxer->mVideoTrackId,
					wantedSampleDts,
					1,
					&nextSyncSampleDts);
				if (ret < 0) {
					ULOG_ERRNO(
						"mp4_demux_get_track_"
						"next_sample_time_after",
						-ret);
				}
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration =
						nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts,
							demuxer->mTimescale);
					if (speed != 0.) {
						newDuration = (int64_t)(
							(float)newDuration /
							speed);
					}
				} else {
					break;
				}
			}
			if ((pendingSeekTs > 0) &&
			    (newDuration - error <
			     2 * demuxer->mAvgOutputInterval)) {
				/* Only seek if the resulting wait time is less
				 * than twice the average frame output rate */
				ULOGD("unable to keep up with playback "
				      "timings, seek forward %.2f ms",
				      (float)(nextSyncSampleDts -
					      mp4_sample_time_to_usec(
						      sample.dts,
						      demuxer->mTimescale)) /
					      1000.);
				duration = newDuration;
				nextSampleDts = nextSyncSampleDts;
				ret = mp4_demux_seek(
					demuxer->mDemux,
					pendingSeekTs,
					MP4_SEEK_METHOD_PREVIOUS_SYNC);
				if (ret < 0) {
					ULOG_ERRNO("mp4_demux_seek", -ret);
				}
			}
		}

		if (nextSampleDts != 0) {
			wait = duration - error;
			/* TODO: loop in the timer cb when silent
			 * or speed>=PDRAW_PLAY_SPEED_MAX */
			if (wait < 0) {
				if (duration > 0) {
					ULOGD("unable to keep "
					      "up with playback timings "
					      "(%.1f ms late, speed=%.2f)",
					      -(float)wait / 1000.,
					      speed);
				}
				wait = 0;
			}
			waitMs = (wait + 500) / 1000;
			if (waitMs == 0)
				waitMs = 1;
		} else if (demuxer->mRunning) {
			ret = demuxer->Source::sendDownstreamEvent(
				demuxer->mVideoMedia,
				Channel::DownstreamEvent::EOS);
			if (ret < 0)
				ULOG_ERRNO("sendDownstreamEvent", -ret);

			/* Notify of the end of range */
			demuxer->mDemuxerListener->onEndOfRange(
				demuxer, demuxer->mCurrentTime);
		}
		demuxer->mLastFrameOutputTime = curTime;
		demuxer->mLastFrameDuration = duration;
		demuxer->mLastOutputError = error;

#if 0
		/* TODO: remove debug */
		ULOGD("timerCb: error=%d duration=%d wait=%d%s",
			(int)error, (int)duration, (int)wait,
			(silent) ? " (silent)" : "");
#endif
	} else {
		demuxer->mLastFrameOutputTime = curTime;
		demuxer->mLastFrameDuration = 0;
		demuxer->mLastOutputError = 0;
	}

	demuxer->Source::unlock();

	if (waitMs > 0) {
		ret = pomp_timer_set(timer, waitMs);
		if (ret < 0)
			ULOG_ERRNO("pomp_timer_set", -ret);
	}
}

} /* namespace Pdraw */
