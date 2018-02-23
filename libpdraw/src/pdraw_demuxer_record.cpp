/**
 * Parrot Drones Awesome Video Viewer Library
 * Recording demuxer
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_demuxer_record.hpp"
#include "pdraw_session.hpp"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <json-c/json.h>
#include <video-streaming/vstrm.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <string>

namespace Pdraw {


RecordDemuxer::RecordDemuxer(
	Session *session)
{
	int ret;

	if (session == NULL) {
		ULOGE("RecordDemuxer: invalid session");
		return;
	}

	mSession = session;
	mConfigured = false;
	mDemux = NULL;
	mTimer = NULL;
	mH264Reader = NULL;
	mRunning = false;
	mFrameByFrame = false;
	mVideoTrackCount = 0;
	mVideoTrackId = 0;
	mMetadataMimeType = NULL;
	mFirstFrame = true;
	mDecoder = NULL;
	mDecoderBitstreamFormat = AVCDECODER_BITSTREAM_FORMAT_UNKNOWN;
	mAvgOutputInterval = 0;
	mLastFrameOutputTime = 0;
	mLastFrameDuration = 0;
	mLastOutputError = 0;
	mDuration = 0;
	mCurrentTime = 0;
	mPendingSeekTs = -1;
	mPendingSeekExact = false;
	mPendingSeekToPrevSample = false;
	mCurrentBuffer = NULL;
	mWidth = mHeight = 0;
	mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
	mSarWidth = mSarHeight = 0;
	mHfov = mVfov = 0.;
	mSpeed = 1.0;

	mMetadataBufferSize = 1024;
	mMetadataBuffer = (uint8_t*)malloc(mMetadataBufferSize);
	if (mMetadataBuffer == NULL) {
		ULOGE("RecordDemuxer: allocation failed size %zu",
			mMetadataBufferSize);
		goto err;
	}

	ret = pthread_mutex_init(&mDemuxerMutex, NULL);
	if (ret != 0) {
		ULOGE("RecordDemuxer: mutex creation failed (%d)", ret);
		goto err;
	}

	mTimer = pomp_timer_new(mSession->getLoop(), timerCb, this);
	if (mTimer == NULL) {
		ULOGE("RecordDemuxer: pomp timer creation failed");
		goto err;
	}
	/* TODO: remove wakeup once everything is called within the loop */
	ret = pomp_loop_wakeup(mSession->getLoop());
	if (ret < 0) {
		ULOG_ERRNO("RecordDemuxer: pomp_loop_wakeup", -ret);
		goto err;
	}

	struct h264_ctx_cbs h264_cbs;
	memset(&h264_cbs, 0, sizeof(h264_cbs));
	h264_cbs.userdata = this;
	h264_cbs.sei_user_data_unregistered = &h264UserDataSeiCb;
	ret = h264_reader_new(&h264_cbs, &mH264Reader);
	if (ret < 0) {
		ULOGE("RecordDemuxer: h264_reader_new() failed (%d)", ret);
		goto err;
	}

	return;

err:
	pthread_mutex_destroy(&mDemuxerMutex);
	if (mTimer != NULL) {
		pomp_timer_clear(mTimer);
		pomp_timer_destroy(mTimer);
		mTimer = NULL;
		/* TODO: remove wakeup once everything is
		 * called within the loop */
		ret = pomp_loop_wakeup(mSession->getLoop());
		if (ret < 0)
			ULOG_ERRNO("RecordDemuxer: pomp_loop_wakeup", -ret);
	}
	if (mH264Reader != NULL) {
		h264_reader_destroy(mH264Reader);
		mH264Reader = NULL;
	}
	free(mMetadataBuffer);
	mMetadataBuffer = NULL;
}


RecordDemuxer::~RecordDemuxer(
	void)
{
	int ret;

	ret = close();
	if (ret != 0)
		ULOGE("RecordDemuxer: close() failed (%d)", ret);

	pthread_mutex_destroy(&mDemuxerMutex);

	if (mCurrentBuffer != NULL) {
		vbuf_unref(&mCurrentBuffer);
		mCurrentBuffer = NULL;
	}

	if (mDemux != NULL) {
		mp4_demux_close(mDemux);
		mDemux = NULL;
	}

	if (mTimer != NULL) {
		pomp_timer_clear(mTimer);
		pomp_timer_destroy(mTimer);
		mTimer = NULL;
		/* TODO: remove wakeup once everything is
		 * called within the loop */
		ret = pomp_loop_wakeup(mSession->getLoop());
		if (ret < 0)
			ULOG_ERRNO("RecordDemuxer: pomp_loop_wakeup", -ret);
	}

	if (mH264Reader != NULL) {
		h264_reader_destroy(mH264Reader);
		mH264Reader = NULL;
	}

	free(mMetadataBuffer);
	mMetadataBuffer = NULL;
	free(mMetadataMimeType);
	mMetadataMimeType = NULL;
}


int RecordDemuxer::fetchVideoDimensions(
	void)
{
	uint8_t *sps = NULL, *pps = NULL;
	unsigned int spsSize = 0, ppsSize = 0;
	int ret = mp4_demux_get_track_avc_decoder_config(mDemux,
		mVideoTrackId, &sps, &spsSize, &pps, &ppsSize);
	if (ret != 0) {
		ULOGE("RecordDemuxer: failed to get "
			"decoder configuration (%d)", ret);
	} else {
		int _ret = pdraw_videoDimensionsFromH264Sps(sps, spsSize,
			&mWidth, &mHeight, &mCropLeft, &mCropRight,
			&mCropTop, &mCropBottom, &mSarWidth, &mSarHeight);
		if (_ret != 0)
			ULOGW("RecordDemuxer: "
				"pdraw_videoDimensionsFromH264Sps() "
				"failed (%d)", _ret);
	}

	return ret;
}


int RecordDemuxer::fetchSessionMetadata(
	void)
{
	if (mSession == NULL) {
		ULOGE("RecordDemuxer: invalid session");
		return -1;
	}

	SessionPeerMetadata *peerMeta = mSession->getPeerMetadata();
	unsigned int count = 0, i;
	char **keys = NULL, *key;
	char **values = NULL, *value;
	struct vmeta_session meta;
	memset(&meta, 0, sizeof(meta));

	int ret = mp4_demux_get_metadata_strings(mDemux,
		&count, &keys, &values);
	if (ret != 0) {
		ULOGE("RecordDemuxer: mp4_demux_get_metadata_strings() "
			"failed (%d)", ret);
		return -1;
	}

	for (i = 0; i < count; i++) {
		key = keys[i];
		value = values[i];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, &meta);
			if (ret != 0) {
				ULOGE("RecordDemuxer: "
					"vmeta_session_recording_read() "
					"failed: %d(%s)\n",
					ret, strerror(-ret));
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


int RecordDemuxer::open(
	const std::string &fileName)
{
	int ret;

	if (mConfigured) {
		ULOGE("RecordDemuxer: demuxer is already configured");
		return -1;
	}

	mFileName = fileName;

	mDemux = mp4_demux_open(mFileName.c_str());
	if (mDemux == NULL) {
		ULOGE("RecordDemuxer: mp4_demux_open() failed");
		return -1;
	}

	int i, tkCount = 0, found = 0;
	struct mp4_media_info info;
	struct mp4_track_info tk;

	ret = mp4_demux_get_media_info(mDemux, &info);
	if (ret != 0) {
		ULOGE("RecordDemuxer: mp4_demux_open() failed");
		return -1;
	}

	mDuration = info.duration;
	tkCount = info.track_count;
	ULOGI("RecordDemuxer: track count: %d", tkCount);
	unsigned int hrs = 0, min = 0, sec = 0;
	pdraw_friendlyTimeFromUs(info.duration, &hrs, &min, &sec, NULL);
	ULOGI("RecordDemuxer: duration: %02d:%02d:%02d", hrs, min, sec);

	for (i = 0; i < tkCount; i++) {
		ret = mp4_demux_get_track_info(mDemux, i, &tk);
		if ((ret == 0) && (tk.type == MP4_TRACK_TYPE_VIDEO)) {
			mVideoTrackId = tk.id;
			mVideoTrackCount++;
			if (tk.has_metadata)
				mMetadataMimeType =
					strdup(tk.metadata_mime_format);
			found = 1;
			break;
		}
	}

	if (!found) {
		ULOGE("RecordDemuxer: failed to find a video track");
		return -1;
	}

	ULOGI("RecordDemuxer: video track ID: %d", mVideoTrackId);

	ret = fetchVideoDimensions();
	if (ret != 0) {
		ULOGE("RecordDemuxer: failed to fetch video dimensions");
		return -1;
	}

	if (mSession) {
		ret = fetchSessionMetadata();
		if (ret != 0) {
			ULOGE("RecordDemuxer: failed to fetch "
				"session metadata");
			return -1;
		}
	}

	mConfigured = true;
	ULOGI("RecordDemuxer: demuxer is configured");

	return 0;
}


int RecordDemuxer::close(
	void)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	mRunning = false;
	pomp_timer_clear(mTimer);

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


int RecordDemuxer::getElementaryStreamCount(
	void)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	/* TODO: handle multiple streams */
	return mVideoTrackCount;
}


enum elementary_stream_type RecordDemuxer::getElementaryStreamType(
	int esIndex)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return (enum elementary_stream_type)-1;
	}
	if ((esIndex < 0) || (esIndex >= mVideoTrackCount)) {
		ULOGE("RecordDemuxer: invalid ES index");
		return (enum elementary_stream_type)-1;
	}

	/* TODO: handle multiple streams */
	return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;
}


int RecordDemuxer::getElementaryStreamVideoDimensions(
	int esIndex,
	unsigned int *width,
	unsigned int *height,
	unsigned int *cropLeft,
	unsigned int *cropRight,
	unsigned int *cropTop,
	unsigned int *cropBottom,
	unsigned int *sarWidth,
	unsigned int *sarHeight)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= mVideoTrackCount)) {
		ULOGE("RecordDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	if (width)
		*width = mWidth;
	if (height)
		*height = mHeight;
	if (cropLeft)
		*cropLeft = mCropLeft;
	if (cropRight)
		*cropRight = mCropRight;
	if (cropTop)
		*cropTop = mCropTop;
	if (cropBottom)
		*cropBottom = mCropBottom;
	if (sarWidth)
		*sarWidth = mSarWidth;
	if (sarHeight)
		*sarHeight = mSarHeight;

	return 0;
}


int RecordDemuxer::getElementaryStreamVideoFov(
	int esIndex,
	float *hfov,
	float *vfov)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= mVideoTrackCount)) {
		ULOGE("RecordDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	if (hfov)
		*hfov = mHfov;
	if (vfov)
		*vfov = mVfov;

	return 0;
}


int RecordDemuxer::setElementaryStreamDecoder(
	int esIndex,
	Decoder *decoder)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}
	if (decoder == NULL) {
		ULOGE("RecordDemuxer: invalid decoder");
		return -1;
	}
	if ((esIndex < 0) || (esIndex >= mVideoTrackCount)) {
		ULOGE("RecordDemuxer: invalid ES index");
		return -1;
	}

	/* TODO: handle multiple streams */
	mDecoder = (AvcDecoder*)decoder;
	uint32_t formatCaps = mDecoder->getInputBitstreamFormatCaps();
	if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) {
		mDecoderBitstreamFormat =
			AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM;
	} else if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_AVCC) {
		mDecoderBitstreamFormat =
			AVCDECODER_BITSTREAM_FORMAT_AVCC;
	} else {
		ULOGE("RecordDemuxer: unsupported "
			"decoder input bitstream format");
		return -1;
	}

	return 0;
}


int RecordDemuxer::play(
	float speed)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	if (speed == 0.) {
		/* speed is null => pause */
		mRunning = false;
		mFrameByFrame = true;
	} else {
		mRunning = true;
		mFrameByFrame = false;
		mPendingSeekToPrevSample = false;
		mSpeed = speed;
		pomp_timer_set(mTimer, 1);
	}

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


bool RecordDemuxer::isPaused(
	void)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return false;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	bool running = mRunning && !mFrameByFrame;

	pthread_mutex_unlock(&mDemuxerMutex);

	return !running;
}


int RecordDemuxer::previous(
	void)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeekToPrevSample = true;
		mPendingSeekExact = true;
		mRunning = true;
		pomp_timer_set(mTimer, 1);
	}

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


int RecordDemuxer::next(
	void)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	mRunning = true;
	pomp_timer_set(mTimer, 1);

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


int RecordDemuxer::seek(
	int64_t delta,
	bool exact)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	/* TODO: merge with seekTo() */
	int64_t ts = (int64_t)mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDuration)
		ts = mDuration;
	mPendingSeekTs = ts;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mRunning = true;
	pomp_timer_set(mTimer, 1);

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


int RecordDemuxer::seekTo(
	uint64_t timestamp,
	bool exact)
{
	if (!mConfigured) {
		ULOGE("RecordDemuxer: demuxer is not configured");
		return -1;
	}

	pthread_mutex_lock(&mDemuxerMutex);

	if (timestamp > mDuration)
		timestamp = mDuration;
	mPendingSeekTs = (int64_t)timestamp;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mRunning = true;
	pomp_timer_set(mTimer, 1);

	pthread_mutex_unlock(&mDemuxerMutex);

	return 0;
}


void RecordDemuxer::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	RecordDemuxer *demuxer = (RecordDemuxer*)userdata;
	int ret = 0;

	if (demuxer == NULL)
		return;
	if ((buf == NULL) || (len == 0))
		return;
	if (demuxer->mCurrentBuffer == NULL)
		return;

	/* ignore "Parrot Streaming" v1 and v2 user data SEI */
	if ((vstrm_h264_sei_streaming_is_v1(sei->uuid)) ||
		(vstrm_h264_sei_streaming_is_v2(sei->uuid)))
		return;

	ret = vbuf_set_userdata_capacity(demuxer->mCurrentBuffer, len);
	if (ret < (signed)len) {
		ULOGE("RecordDemuxer: failed to realloc user data buffer");
		return;
	}

	uint8_t *dstBuf = vbuf_get_userdata_ptr(demuxer->mCurrentBuffer);
	memcpy(dstBuf, buf, len);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, len);
}


int RecordDemuxer::getAvcDecoderConfig(
	RecordDemuxer *demuxer,
	uint8_t **pSps,
	size_t *spsSize,
	uint8_t **pPps,
	size_t *ppsSize)
{
	uint8_t *sps = NULL, *pps = NULL;
	uint8_t *spsBuffer = NULL, *ppsBuffer = NULL;
	unsigned int spsSz = 0, ppsSz = 0;
	uint32_t start;
	int ret;

	if (demuxer == NULL) {
		ULOGE("RecordDemuxer: invalid demuxer");
		return -1;
	}
	if ((pSps == NULL) || (spsSize == NULL) ||
		(pPps == NULL) | (ppsSize == NULL)) {
		ULOGE("RecordDemuxer: invalid parameters");
		return -1;
	}

	ret = mp4_demux_get_track_avc_decoder_config(
		demuxer->mDemux, demuxer->mVideoTrackId,
		&sps, &spsSz, &pps, &ppsSz);
	if (ret < 0) {
		ULOGE("RecordDemuxer: mp4_demux_get_track_avc_decoder_config() "
			"failed (%d)", ret);
		return -1;
	}
	if ((sps == NULL) || (spsSz == 0)) {
		ULOGE("RecordDemuxer: invalid SPS");
		return -1;
	}
	if ((pps == NULL) || (ppsSz == 0)) {
		ULOGE("RecordDemuxer: invalid PPS");
		return -1;
	}

	ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0, sps, spsSz);
	if (ret < 0) {
		ULOGW("RecordDemuxer: h264_reader_parse_nalu() "
			"failed (%d)", ret);
		return -1;
	}

	ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0, pps, ppsSz);
	if (ret < 0) {
		ULOGW("RecordDemuxer: h264_reader_parse_nalu() "
			"failed (%d)", ret);
		return -1;
	}

	spsBuffer = (uint8_t *)malloc(spsSz + 4);
	if (spsBuffer == NULL) {
		ULOGE("RecordDemuxer: SPS buffer allocation failed");
		return -1;
	}

	start = (demuxer->mDecoderBitstreamFormat ==
		AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
		htonl(0x00000001) : htonl(spsSz);
	*((uint32_t*)spsBuffer) = start;
	memcpy(spsBuffer + 4, sps, spsSz);

	ppsBuffer = (uint8_t *)malloc(ppsSz + 4);
	if (ppsBuffer == NULL) {
		ULOGE("RecordDemuxer: PPS buffer allocation failed");
		free(spsBuffer);
		return -1;
	}

	start = (demuxer->mDecoderBitstreamFormat ==
		AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
		htonl(0x00000001) : htonl(ppsSz);
	*((uint32_t*)ppsBuffer) = start;
	memcpy(ppsBuffer + 4, pps, ppsSz);

	ret = demuxer->mDecoder->open(demuxer->mDecoderBitstreamFormat,
		spsBuffer, (unsigned int)spsSz + 4,
		ppsBuffer, (unsigned int)ppsSz + 4);
	if (ret != 0) {
		ULOGE("RecordDemuxer: decoder configuration failed (%d)", ret);
		free(spsBuffer);
		free(ppsBuffer);
		return -1;
	}

	*pSps = spsBuffer;
	*spsSize = spsSz;
	*pPps = ppsBuffer;
	*ppsSize = ppsSz;

	return 0;
}


void RecordDemuxer::timerCb(
	struct pomp_timer *timer,
	void *userdata)
{
	RecordDemuxer *demuxer = (RecordDemuxer *)userdata;
	bool silent = false;
	float speed = 1.0;
	int64_t seekTs = -1;
	struct mp4_track_sample sample;
	struct avcdecoder_input_buffer *data = NULL;
	uint8_t *buf = NULL, *_buf, *sps = NULL, *pps = NULL, *sei = NULL;
	size_t bufSize = 0, outSize = 0, offset = 0, naluSize;
	size_t spsSize = 0, ppsSize = 0, seiSize = 0;
	struct timespec t1;
	uint64_t curTime;
	int64_t error, duration, wait = 0;
	uint32_t waitMs = 0;
	int ret, retry = 0;

	if (demuxer == NULL) {
		ULOGE("RecordDemuxer: invalid context pointer");
		return;
	}

	pthread_mutex_lock(&demuxer->mDemuxerMutex);

	speed = demuxer->mSpeed;
	seekTs = demuxer->mPendingSeekTs;

	if ((demuxer->mDecoder == NULL) || (!demuxer->mRunning)) {
		demuxer->mLastFrameDuration = 0;
		demuxer->mLastOutputError = 0;
		pthread_mutex_unlock(&demuxer->mDemuxerMutex);
		return;
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
	memset(&sample, 0, sizeof(sample));

	if (demuxer->mFirstFrame) {
		/* Get the H.264 config and configure the decoder */
		ret = getAvcDecoderConfig(demuxer,
			&sps, &spsSize, &pps, &ppsSize);
		if (ret != 0) {
			ULOGE("RecordDemuxer: failed to get "
				"AVC decoder config (%d)", ret);
		}
	}

	if (!demuxer->mDecoder->isConfigured()) {
		ULOGE("RecordDemuxer: decoder is not configured");
		retry = 1;
		goto out;
	}

	if (demuxer->mCurrentBuffer == NULL) {
		ret = demuxer->mDecoder->getInputBuffer(
			&demuxer->mCurrentBuffer, false);
		if ((ret != 0) || (demuxer->mCurrentBuffer == NULL)) {
			ULOGW("RecordDemuxer: failed to get "
				"an output buffer (%d)", ret);
			retry = 1;
			goto out;
		}
	}

	buf = vbuf_get_ptr(demuxer->mCurrentBuffer);
	bufSize = vbuf_get_capacity(demuxer->mCurrentBuffer);

	if ((demuxer->mFirstFrame) && (sps != NULL) && (pps != NULL)) {
		/* Insert the SPS and PPS */
		if (spsSize + 4 <= bufSize) {
			memcpy(buf, sps, spsSize + 4);
			buf += (spsSize + 4);
			bufSize -= (spsSize + 4);
			outSize += (spsSize + 4);
		}
		if (ppsSize + 4 <= bufSize) {
			memcpy(buf, pps, ppsSize + 4);
			buf += (ppsSize + 4);
			bufSize -= (ppsSize + 4);
			outSize += (ppsSize + 4);
		}
		demuxer->mFirstFrame = false;
	}

	/* Seeking */
	if (demuxer->mPendingSeekTs >= 0) {
		ret = mp4_demux_seek(demuxer->mDemux,
			(uint64_t)demuxer->mPendingSeekTs, 1);
		if (ret != 0) {
			ULOGW("RecordDemuxer: mp4_demux_seek() "
				"failed (%d)", ret);
		} else {
			demuxer->mLastFrameDuration = 0;
			demuxer->mLastOutputError = 0;
		}
	} else if (demuxer->mPendingSeekToPrevSample) {
		ret = mp4_demux_seek_to_track_prev_sample(
			demuxer->mDemux, demuxer->mVideoTrackId);
		if (ret != 0) {
			ULOGW("RecordDemuxer: "
				"mp4_demux_seek_to_track_prev_sample() "
				"failed (%d)", ret);
		} else {
			demuxer->mLastFrameDuration = 0;
			demuxer->mLastOutputError = 0;
		}
	}

	/* Get a sample */
	ret = mp4_demux_get_track_next_sample(demuxer->mDemux,
		demuxer->mVideoTrackId, buf, bufSize,
		demuxer->mMetadataBuffer, demuxer->mMetadataBufferSize,
		&sample);
	if (ret != 0) {
		ULOGW("RecordDemuxer: failed to get sample (%d)", ret);
		if (ret == -ENOBUFS) {
			/* Go to the next sample */
			ret = mp4_demux_get_track_next_sample(
				demuxer->mDemux, demuxer->mVideoTrackId,
				NULL, 0, NULL, 0, &sample);
		}
		retry = 1;
		goto out;
	}
	if (sample.sample_size == 0) {
		goto out;
	}

	vbuf_set_size(demuxer->mCurrentBuffer, outSize + sample.sample_size);
	vbuf_set_userdata_size(demuxer->mCurrentBuffer, 0);

	silent = ((sample.silent) && (demuxer->mPendingSeekExact)) ?
		true : false;
	demuxer->mPendingSeekTs = -1;
	demuxer->mPendingSeekToPrevSample = false;
	demuxer->mPendingSeekExact = (silent) ?
		demuxer->mPendingSeekExact : false;

	/* Parse the H.264 bitstream and convert
	 * to byte stream if necessary */
	_buf = buf;
	while (offset < sample.sample_size) {
		naluSize = ntohl(*((uint32_t*)_buf));
		if (demuxer->mDecoderBitstreamFormat ==
			AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM)
			*((uint32_t*)_buf) = htonl(0x00000001);
		if (*(_buf + 4) == 0x06) {
			sei = _buf + 4;
			seiSize = naluSize;
		}
		_buf += 4 + naluSize;
		offset += 4 + naluSize;
	}

	/* Parse the H.264 SEI to find user data SEI */
	if ((sei != NULL) && (seiSize != 0)) {
		ret = h264_reader_parse_nalu(demuxer->mH264Reader,
			0, sei, seiSize);
		if (ret < 0)
			ULOGW("RecordDemuxer: h264_reader_parse_nalu() "
				"failed (%d)", ret);
	}

	data = (struct avcdecoder_input_buffer *)
		vbuf_get_metadata_ptr(demuxer->mCurrentBuffer);
	memset(data, 0, sizeof(*data));
	data->isComplete = true; /* TODO? */
	data->hasErrors = false; /* TODO? */
	data->isRef = true; /* TODO? */
	data->isSilent = silent;
	data->auNtpTimestamp = sample.sample_dts;
	data->auNtpTimestampRaw = sample.sample_dts;
	/* TODO: auSyncType */

	/* Metadata */
	data->hasMetadata = VideoFrameMetadata::decodeMetadata(
		demuxer->mMetadataBuffer, sample.metadata_size,
		FRAME_METADATA_SOURCE_RECORDING,
		demuxer->mMetadataMimeType, &data->metadata);

	clock_gettime(CLOCK_MONOTONIC, &t1);
	curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
	data->demuxOutputTimestamp = curTime;
	data->auNtpTimestampLocal = data->demuxOutputTimestamp;
	demuxer->mCurrentTime = sample.sample_dts;

	/* Queue the buffer for decoding */
	ret = demuxer->mDecoder->queueInputBuffer(demuxer->mCurrentBuffer);
	if (ret != 0) {
		ULOGW("RecordDemuxer: failed to release "
			"the output buffer (%d)", ret);
	} else {
		vbuf_unref(&demuxer->mCurrentBuffer);
		demuxer->mCurrentBuffer = NULL;
	}

	if ((demuxer->mFrameByFrame) && (!silent))
		demuxer->mRunning = false;

out:
	if (retry) {
		waitMs = 5;
	} else if (demuxer->mRunning) {
		/* Schedule the next sample */
		uint64_t nextSampleDts = sample.next_sample_dts;

		/* If error > 0 we are late, if error < 0 we are early */
		error = ((demuxer->mLastFrameOutputTime == 0) ||
			(demuxer->mLastFrameDuration == 0) ||
			(speed == 0.) || (speed >= PDRAW_PLAY_SPEED_MAX) ||
			(silent)) ? 0 :
			curTime - demuxer->mLastFrameOutputTime -
			demuxer->mLastFrameDuration + demuxer->mLastOutputError;
		if (demuxer->mLastFrameOutputTime) {
			/* Average frame output rate
			 * (sliding average, alpha = 1/2) */
			demuxer->mAvgOutputInterval += ((int64_t)(curTime -
				demuxer->mLastFrameOutputTime) -
				demuxer->mAvgOutputInterval) >> 1;
		}

		/* Sample duration */
		if ((speed >= PDRAW_PLAY_SPEED_MAX) ||
			(nextSampleDts == 0) || (silent)) {
			duration = 0;
		} else if (speed < 0.) {
			/* Negative speed => play backward */
			nextSampleDts = sample.prev_sync_sample_dts;
			uint64_t pendingSeekTs = nextSampleDts;
			uint64_t nextSyncSampleDts = nextSampleDts;
			duration = nextSampleDts - sample.sample_dts;
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				nextSyncSampleDts =
					mp4_demux_get_track_prev_sample_time_before(
					demuxer->mDemux, demuxer->mVideoTrackId,
					nextSyncSampleDts, 1);
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration = nextSyncSampleDts -
						sample.sample_dts;
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
				ret = mp4_demux_seek(demuxer->mDemux,
					pendingSeekTs, 1);
				if (ret != 0) {
					ULOGW("RecordDemuxer: mp4_demux_seek() "
						"failed (%d)", ret);
				}
			}
		} else {
			/* Positive speed => play forward */
			uint64_t pendingSeekTs = 0;
			uint64_t nextSyncSampleDts = nextSampleDts;
			duration = nextSampleDts - sample.sample_dts;
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				nextSyncSampleDts =
					mp4_demux_get_track_next_sample_time_after(
					demuxer->mDemux, demuxer->mVideoTrackId,
					nextSyncSampleDts, 1);
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration = nextSyncSampleDts -
						sample.sample_dts;
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
				ULOGD("RecordDemuxer: unable to keep "
					"up with playback timings, "
					"seek forward %.2f ms",
					(float)(nextSyncSampleDts -
					sample.sample_dts) / 1000.);
				duration = newDuration;
				nextSampleDts = nextSyncSampleDts;
				ret = mp4_demux_seek(demuxer->mDemux,
					pendingSeekTs, 1);
				if (ret != 0) {
					ULOGW("RecordDemuxer: mp4_demux_seek() "
						"failed (%d)", ret);
				}
			}
		}

		if (nextSampleDts != 0) {
			wait = duration - error;
			/* TODO: loop in the timer cb when silent
			 * or speed>=PDRAW_PLAY_SPEED_MAX */
			if (wait < 0) {
				if (duration > 0) {
					ULOGD("RecordDemuxer: unable to keep "
						"up with playback timings "
						"(%.1f ms late, speed=%.2f)",
						-(float)wait / 1000., speed);
				}
				wait = 0;
			}
			waitMs = (wait + 500) / 1000;
			if (waitMs == 0)
				waitMs = 1;
		}
		demuxer->mLastFrameOutputTime = curTime;
		demuxer->mLastFrameDuration = duration;
		demuxer->mLastOutputError = error;

		ULOGD("RecordDemuxer: timerCb: error=%d duration=%d wait=%d%s",
			(int)error, (int)duration, (int)wait, (silent) ? " (silent)" : "");
	} else {
		demuxer->mLastFrameOutputTime = curTime;
		demuxer->mLastFrameDuration = 0;
		demuxer->mLastOutputError = 0;
	}

	if (waitMs > 0) {
		ret = pomp_timer_set(timer, waitMs);
		if (ret < 0) {
			ULOGE("RecordDemuxer: pomp_timer_set() failed (%d)",
				ret);
		}
	}

	pthread_mutex_unlock(&demuxer->mDemuxerMutex);

	free(sps);
	free(pps);
}

} /* namespace Pdraw */
