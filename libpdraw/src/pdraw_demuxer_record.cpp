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

#define ULOG_TAG pdraw_dmxrec
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

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


const struct h264_ctx_cbs RecordDemuxer::VideoMedia::mH264ReaderCbs = {
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
	.sei_pic_timing = &RecordDemuxer::VideoMedia::h264PicTimingSeiCb,
	.sei_pan_scan_rect = nullptr,
	.sei_filler_payload = nullptr,
	.sei_user_data_registered = nullptr,
	.sei_user_data_unregistered =
		&RecordDemuxer::VideoMedia::h264UserDataSeiCb,
	.sei_recovery_point = nullptr,
};


const struct h265_ctx_cbs RecordDemuxer::VideoMedia::mH265ReaderCbs = {
	.nalu_begin = nullptr,
	.nalu_end = nullptr,
	.au_end = nullptr,
	.vps = nullptr,
	.sps = nullptr,
	.pps = nullptr,
	.aud = nullptr,
	.sei = nullptr,
	.sei_user_data_unregistered =
		&RecordDemuxer::VideoMedia::h265UserDataSeiCb,
	.sei_recovery_point = nullptr,
	.sei_time_code = &RecordDemuxer::VideoMedia::h265TimeCodeSeiCb,
	.sei_mastering_display_colour_volume =
		&RecordDemuxer::VideoMedia::h265MdcvSeiCb,
	.sei_content_light_level = &RecordDemuxer::VideoMedia::h265CllSeiCb,
};


RecordDemuxer::RecordDemuxer(Session *session,
			     Element::Listener *elementListener,
			     CodedSource::Listener *sourceListener,
			     IPdraw::IDemuxer *demuxer,
			     IPdraw::IDemuxer::Listener *demuxerListener,
			     const std::string &fileName) :
		Demuxer(session,
			elementListener,
			sourceListener,
			demuxer,
			demuxerListener),
		mFileName(fileName), mRunning(false), mFrameByFrame(false),
		mDemux(nullptr), mDuration(0), mCurrentTime(0), mSpeed(1.0),
		mChannelsFlushing(0)
{
	Element::setClassName(__func__);

	setState(CREATED);
}


RecordDemuxer::~RecordDemuxer(void)
{
	int ret;

	if (mState != STOPPED && mState != CREATED)
		PDRAW_LOGW("demuxer is still running");

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		delete *p;
		p++;
	}

	if (mDemux != nullptr) {
		ret = mp4_demux_close(mDemux);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mp4_demux_close", -ret);
		mDemux = nullptr;
	}
}


int RecordDemuxer::fetchSessionMetadata(unsigned int trackId,
					struct vmeta_session *meta)
{
	int ret;
	unsigned int count = 0, i;
	char **keys = nullptr, *key;
	char **values = nullptr, *value;

	memset(meta, 0, sizeof(*meta));

	/* File-level session metadata */
	ret = mp4_demux_get_metadata_strings(mDemux, &count, &keys, &values);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_metadata_strings", -ret);
		return ret;
	}
	for (i = 0; i < count; i++) {
		key = keys[i];
		value = values[i];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, meta);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("vmeta_session_recording_read",
						-ret);
				continue;
			}
		}
	}

	/* Track-level session metadata */
	ret = mp4_demux_get_track_metadata_strings(
		mDemux, trackId, &count, &keys, &values);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_metadata_strings", -ret);
		return ret;
	}
	for (i = 0; i < count; i++) {
		key = keys[i];
		value = values[i];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, meta);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("vmeta_session_recording_read",
						-ret);
				continue;
			}
		}
	}

	return 0;
}


int RecordDemuxer::start(void)
{
	int ret;
	unsigned int i, tkCount = 0;
	struct mp4_media_info info;
	struct mp4_track_info tk;
	struct pdraw_demuxer_media *medias = nullptr;
	std::vector<struct pdraw_demuxer_media *> selectedMedias;
	std::vector<struct pdraw_demuxer_media *>::iterator p;
	size_t mediasCount = 0;
	int defaultMediaIndex = -1;
	unsigned int hrs = 0, min = 0, sec = 0;
	bool ready = true;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("demuxer is not created");
		return -EPROTO;
	}
	setState(STARTING);

	/* Create the MP4 demuxer */
	ret = mp4_demux_open(mFileName.c_str(), &mDemux);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_open", -ret);
		goto exit;
	}

	ret = mp4_demux_get_media_info(mDemux, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_media_info", -ret);
		goto exit;
	}

	mDuration = info.duration;
	tkCount = info.track_count;
	PDRAW_LOGD("track count: %d", tkCount);
	pdraw_friendlyTimeFromUs(info.duration, &hrs, &min, &sec, nullptr);
	PDRAW_LOGD("duration: %02d:%02d:%02d", hrs, min, sec);

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
		(void)fetchSessionMetadata(tk.id, &current->session_meta);
	}
	if (mediasCount == 0) {
		PDRAW_LOGE("no video track");
		ret = -ENOENT;
		goto exit;
	}

	/* Ask which media(s) to use from the application */
	ret = selectMedia(medias, mediasCount);
	if (ret == 0 || ret == -ENOSYS) {
		if (defaultMediaIndex == -1) {
			PDRAW_LOGE(
				"application requested default media, "
				"but no default media found");
			ret = -ENOENT;
			goto exit;
		}
		selectedMedias.push_back(&medias[defaultMediaIndex]);
		PDRAW_LOGI("auto-selecting media %d (%s)",
			   selectedMedias.back()->media_id,
			   selectedMedias.back()->name);
	} else if (ret == -ECANCELED) {
		PDRAW_LOGI("application cancelled the media selection");
		ready = false;
		goto exit;
	} else if (ret < 0) {
		PDRAW_LOG_ERRNO("application failed to select a video media",
				-ret);
		ret = -EPROTO;
		goto exit;
	} else {
		for (size_t j = 0; j < mediasCount; j++) {
			if (!(ret & (1 << medias[j].media_id)))
				continue;
			selectedMedias.push_back(&medias[j]);
			PDRAW_LOGI("application selected media %d (%s)",
				   selectedMedias.back()->media_id,
				   selectedMedias.back()->name);
		}
		if (selectedMedias.empty()) {
			PDRAW_LOGE("the application requested no valid media");
			ret = -ENOENT;
			goto exit;
		}
	}

	/* Create the output ports for the selected medias */
	p = selectedMedias.begin();
	while (p != selectedMedias.end()) {
		ret = mp4_demux_get_track_info(mDemux, (*p)->idx, &tk);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_info", -ret);
			goto exit;
		}
		RecordDemuxer::VideoMedia *videoMedia =
			new RecordDemuxer::VideoMedia(this);
		ret = videoMedia->setup(&tk);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("VideoMedia::setup", -ret);
			delete videoMedia;
			goto exit;
		}
		mVideoMedias.push_back(videoMedia);
		p++;
	}

exit:
	/* Cleanup track list */
	for (size_t j = 0; j < mediasCount; j++) {
		free((void *)medias[j].name);
	}
	free(medias);

	if ((ret == 0) || (ret == -ECANCELED)) {
		ret = 0;
		setState(STARTED);
		openResponse(ret);
		readyToPlay(ready);
		/* TODO: notify readyToPlay = false at end of file */
		if (!ready)
			onUnrecoverableError();
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
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}
	setState(STOPPING);

	readyToPlay(false);

	mRunning = false;

	CodedSource::lock();

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->stop();
		p++;
	}

	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	CodedSource::unlock();

	return 0;
}


int RecordDemuxer::flush(void)
{
	if ((mState != STARTED) && (mState != STOPPING)) {
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}

	CodedSource::lock();

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->stop();
		p++;
	}

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		CodedVideoMedia *media = getOutputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGW("failed to get media at index %d", i);
			continue;
		}

		unsigned int outputChannelCount = getOutputChannelCount(media);

		/* Flush the output channels */
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			CodedChannel *channel = getOutputChannel(media, j);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   j);
				continue;
			}
			int ret = channel->flush();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->flush", -ret);
			mChannelsFlushing++;
		}
	}

	CodedSource::unlock();

	if (mChannelsFlushing == 0)
		completeFlush();

	return 0;
}


void RecordDemuxer::onChannelFlushed(CodedChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("channel flushed media name=%s (channel key=%p)",
		   media->getName().c_str(),
		   channel->getKey());

	if (mState == STOPPING) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mChannelsFlushing <= 0) {
		mChannelsFlushing = 0;
		completeFlush();
	}
}


void RecordDemuxer::completeFlush(void)
{
	if (mRunning) {
		/* restart playing */
		auto p = mVideoMedias.begin();
		while (p != mVideoMedias.end()) {
			(*p)->play();
			p++;
		}
	}
	if (mState == STOPPING)
		completeTeardown();
}


void RecordDemuxer::onChannelUnlink(CodedChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	CodedVideoMedia *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel->getKey());
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeOutputChannel", -ret);

	completeTeardown();
}


void RecordDemuxer::completeTeardown(void)
{
	CodedSource::lock();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		CodedVideoMedia *media = getOutputMedia(i);
		if (media && getOutputChannelCount(media) > 0) {
			CodedSource::unlock();
			return;
		}
	}

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		delete *p;
		p++;
	}
	mVideoMedias.clear();

	CodedSource::unlock();

	if (mState == STOPPING) {
		closeResponse(0);
		setStateAsyncNotify(STOPPED);
	}
}


int RecordDemuxer::play(float speed)
{
	if (mState != STARTED) {
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}

	if (speed == 0.) {
		/* speed is null => pause */
		mRunning = false;
		mFrameByFrame = true;
		pauseResponse(0, getCurrentTime());
	} else {
		mRunning = true;
		mFrameByFrame = false;
		mSpeed = speed;
		auto p = mVideoMedias.begin();
		while (p != mVideoMedias.end()) {
			(*p)->play();
			p++;
		}
		playResponse(0, getCurrentTime(), mSpeed);
	}

	return 0;
}


bool RecordDemuxer::isReadyToPlay(void)
{
	if (mState != STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return false;
	}

	return mReadyToPlay;
}


bool RecordDemuxer::isPaused(void)
{
	if (mState != STARTED) {
		PDRAW_LOG_ERRNO("demuxer is not started", EPROTO);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int RecordDemuxer::previous(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		PDRAW_LOGE("demuxer is not paused");
		return -EPROTO;
	}

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->previous();
		p++;
	}
	mRunning = true;

	return 0;
}


int RecordDemuxer::next(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("demuxer is not configured");
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		PDRAW_LOGE("demuxer is not paused");
		return -EPROTO;
	}

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->next();
		p++;
	}
	mRunning = true;

	return 0;
}


int RecordDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}

	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->seek(delta, exact);
		p++;
	}

	return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("demuxer is not started");
		return -EPROTO;
	}

	if (timestamp > mDuration)
		timestamp = mDuration;
	auto p = mVideoMedias.begin();
	while (p != mVideoMedias.end()) {
		(*p)->seekTo(timestamp, exact);
		p++;
	}
	mRunning = true;

	return 0;
}


RecordDemuxer::VideoMedia::VideoMedia(RecordDemuxer *demuxer) :
		mDemuxer(demuxer), mFirstFrame(true), mTimer(nullptr),
		mH264Reader(nullptr), mH265Reader(nullptr),
		mVideoMedias(nullptr), mNbVideoMedias(0), mVideoTrackId(0),
		mMetadataMimeType(nullptr), mMetadataBufferSize(0),
		mMetadataBuffer(nullptr), mTimescale(0), mAvgOutputInterval(0),
		mLastFrameOutputTime(0), mLastFrameDuration(0),
		mLastOutputError(0), mPendingSeekTs(-1),
		mPendingSeekExact(false), mPendingSeekToPrevSample(false),
		mPendingSeekToNextSample(false), mSeekResponse(0),
		mCurrentFrame(nullptr), mCurrentMem(nullptr),
		mCurrentFrameCaptureTs(0), mDecodingTs(0), mDecodingTsInc(0),
		mFrameIndex(0)
{
	std::string name = demuxer->getName() + "#VideoMedia";
	Loggable::setName(name);
}


RecordDemuxer::VideoMedia::~VideoMedia(void)
{
	int ret;

	if (mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
	}

	if (mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
	}

	if (mTimer != nullptr) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
	}

	if (mH264Reader != nullptr) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -ret);
	}
	if (mH265Reader != nullptr) {
		ret = h265_reader_destroy(mH265Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h265_reader_destroy", -ret);
	}

	/* Remove the output ports */
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		if (mDemuxer->CodedSource::mListener) {
			mDemuxer->CodedSource::mListener->onOutputMediaRemoved(
				mDemuxer, mVideoMedias[i]);
		}
		ret = mDemuxer->removeOutputPort(mVideoMedias[i]);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("removeOutputPort", -ret);
		} else {
			delete mVideoMedias[i];
		}
	}

	free(mVideoMedias);
	free(mMetadataBuffer);
	free(mMetadataMimeType);
}


int RecordDemuxer::VideoMedia::setup(struct mp4_track_info *tkinfo)
{
	int ret, err;
	struct mp4_video_decoder_config vdc = {};
	CodedSource::OutputPort *basePort, *mediaPort;

	std::string name =
		mDemuxer->getName() + "#track#" + std::to_string(tkinfo->id);
	Loggable::setName(name);

	mMetadataBufferSize = 1024;
	mMetadataBuffer = (uint8_t *)malloc(mMetadataBufferSize);
	if (mMetadataBuffer == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("malloc", -ret);
		goto error;
	}

	/* Create the demux timer */
	mTimer = pomp_timer_new(mDemuxer->mSession->getLoop(), timerCb, this);
	if (mTimer == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -ret);
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

	mDemuxer->CodedSource::lock();

	if (mNbVideoMedias != 0) {
		ret = -EEXIST;
		mDemuxer->CodedSource::unlock();
		PDRAW_LOGE("video track already defined");
		goto error;
	}

	mVideoTrackId = tkinfo->id;
	mTimescale = tkinfo->timescale;
	ret = mp4_demux_get_track_video_decoder_config(
		mDemuxer->mDemux, mVideoTrackId, &vdc);
	if (ret < 0) {
		mDemuxer->CodedSource::unlock();
		PDRAW_LOG_ERRNO("mp4_demux_get_track_video_decoder_config",
				-ret);
		goto error;
	}

	switch (vdc.codec) {
	case MP4_VIDEO_CODEC_AVC:
		if ((vdc.avc.sps == nullptr) || (vdc.avc.sps_size == 0)) {
			ret = -EPROTO;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("invalid SPS");
			goto error;
		}
		if ((vdc.avc.pps == nullptr) || (vdc.avc.pps_size == 0)) {
			ret = -EPROTO;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("invalid PPS");
			goto error;
		}
		break;
	case MP4_VIDEO_CODEC_HEVC:
		if ((vdc.hevc.vps == nullptr) || (vdc.hevc.vps_size == 0)) {
			ret = -EPROTO;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("invalid VPS");
			goto error;
		}
		if ((vdc.hevc.sps == nullptr) || (vdc.hevc.sps_size == 0)) {
			ret = -EPROTO;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("invalid SPS");
			goto error;
		}
		if ((vdc.hevc.pps == nullptr) || (vdc.hevc.pps_size == 0)) {
			ret = -EPROTO;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("invalid PPS");
			goto error;
		}
		break;
	default:
		ret = -EPROTO;
		mDemuxer->CodedSource::unlock();
		PDRAW_LOGE("invalid video codec");
		goto error;
	}

	/* Create the output port */
	mNbVideoMedias = 2;
	mVideoMedias = (CodedVideoMedia **)calloc(mNbVideoMedias,
						  sizeof(*mVideoMedias));
	if (mVideoMedias == nullptr) {
		ret = -ENOMEM;
		mDemuxer->CodedSource::unlock();
		PDRAW_LOGE("media allocation failed");
		goto error;
	}
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		mVideoMedias[i] = new CodedVideoMedia(mDemuxer->mSession);
		if (mVideoMedias[i] == nullptr) {
			ret = -ENOMEM;
			mDemuxer->CodedSource::unlock();
			PDRAW_LOGE("media allocation failed");
			goto error;
		}
		ret = mDemuxer->addOutputPort(mVideoMedias[i]);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("addOutputPort", -ret);
			goto error;
		}
		std::string path = mDemuxer->Element::getName() + "$" +
				   mVideoMedias[i]->getName();
		mVideoMedias[i]->setPath(path);
	}

	/* Set the output media info */
	switch (tkinfo->video_codec) {
	case MP4_VIDEO_CODEC_AVC:
		mVideoMedias[0]->format = vdef_h264_avcc;
		mVideoMedias[1]->format = vdef_h264_byte_stream;
		for (unsigned int i = 0; i < mNbVideoMedias; i++) {
			ret = mVideoMedias[i]->setPs(nullptr,
						     0,
						     vdc.avc.sps,
						     vdc.avc.sps_size,
						     vdc.avc.pps,
						     vdc.avc.pps_size);
			if (ret < 0) {
				mDemuxer->CodedSource::unlock();
				PDRAW_LOG_ERRNO("media->setPs", -ret);
				goto error;
			}
		}
		/* Initialize the H.264 parsing */
		ret = h264_reader_parse_nalu(
			mH264Reader, 0, vdc.avc.sps, vdc.avc.sps_size);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h264_reader_parse_nalu(
			mH264Reader, 0, vdc.avc.pps, vdc.avc.pps_size);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			goto error;
		}
		break;
	case MP4_VIDEO_CODEC_HEVC:
		mVideoMedias[0]->format = vdef_h265_hvcc;
		mVideoMedias[1]->format = vdef_h265_byte_stream;
		for (unsigned int i = 0; i < mNbVideoMedias; i++) {
			ret = mVideoMedias[i]->setPs(vdc.hevc.vps,
						     vdc.hevc.vps_size,
						     vdc.hevc.sps,
						     vdc.hevc.sps_size,
						     vdc.hevc.pps,
						     vdc.hevc.pps_size);
			if (ret < 0) {
				mDemuxer->CodedSource::unlock();
				PDRAW_LOG_ERRNO("media->setPs", -ret);
				goto error;
			}
		}
		/* Initialize the H.265 parsing */
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.vps, vdc.hevc.vps_size);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.sps, vdc.hevc.sps_size);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		ret = h265_reader_parse_nalu(
			mH265Reader, 0, vdc.hevc.pps, vdc.hevc.pps_size);
		if (ret < 0) {
			mDemuxer->CodedSource::unlock();
			PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto error;
		}
		break;
	default:
		mVideoMedias[0]->format = (struct vdef_coded_format){
			.encoding = VDEF_ENCODING_UNKNOWN,
			.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
		};
		mVideoMedias[1]->format = (struct vdef_coded_format){
			.encoding = VDEF_ENCODING_UNKNOWN,
			.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
		};
		break;
	}

	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		(void)mDemuxer->fetchSessionMetadata(
			mVideoTrackId, &mVideoMedias[i]->sessionMeta);
		mVideoMedias[i]->playbackType = PDRAW_PLAYBACK_TYPE_REPLAY;
		mVideoMedias[i]->duration = mDemuxer->mDuration;
	}
	if (tkinfo->has_metadata)
		mMetadataMimeType = strdup(tkinfo->metadata_mime_format);

	/* Create the output buffers pool on the last media. As the medias will
	 * be destroyed in creation order, this ensures that the media which
	 * owns the buffers pool will be the last destroyed */
	ret = mDemuxer->createOutputPortMemoryPool(
		mVideoMedias[1],
		DEMUXER_OUTPUT_BUFFER_COUNT,
		mVideoMedias[1]->info.resolution.width *
			mVideoMedias[1]->info.resolution.height * 3 / 4);
	if (ret < 0) {
		mDemuxer->CodedSource::unlock();
		PDRAW_LOG_ERRNO("createOutputPortMemoryPool", -ret);
		goto error;
	}
	/* Make the pool shared between all medias */
	basePort = mDemuxer->getOutputPort(mVideoMedias[1]);
	mediaPort = mDemuxer->getOutputPort(mVideoMedias[0]);
	if (basePort == nullptr || mediaPort == nullptr) {
		PDRAW_LOGW("unable to share memory pool between medias");
	} else {
		mediaPort->pool = basePort->pool;
		mediaPort->sharedPool = true;
	}
	mDemuxer->CodedSource::unlock();

	if (mDemuxer->CodedSource::mListener) {
		for (unsigned int i = 0; i < mNbVideoMedias; i++)
			mDemuxer->CodedSource::mListener->onOutputMediaAdded(
				mDemuxer, mVideoMedias[i]);
	}

	return 0;

error:
	if (mTimer != nullptr) {
		err = pomp_timer_clear(mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mTimer = nullptr;
	}
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
	free(mMetadataBuffer);
	mMetadataBuffer = nullptr;
	return ret;
}


void RecordDemuxer::VideoMedia::play(void)
{
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::VideoMedia::previous(void)
{
	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeekToPrevSample = true;
		mPendingSeekToNextSample = false;
		mPendingSeekExact = true;
		pomp_timer_set(mTimer, 1);
	}
}


void RecordDemuxer::VideoMedia::next(void)
{
	mPendingSeekToNextSample = true;
	mPendingSeekToPrevSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::VideoMedia::seek(int64_t delta, bool exact)
{
	int64_t ts = (int64_t)mDemuxer->mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDemuxer->mDuration)
		ts = mDemuxer->mDuration;
	seekTo(ts, exact);
}


void RecordDemuxer::VideoMedia::seekTo(uint64_t timestamp, bool exact)
{
	if (timestamp > mDemuxer->mDuration)
		timestamp = mDemuxer->mDuration;
	mPendingSeekTs = (int64_t)timestamp;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::VideoMedia::stop(void)
{
	int ret;

	pomp_timer_clear(mTimer);

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
}


void RecordDemuxer::VideoMedia::sendDownstreamEvent(
	CodedChannel::DownstreamEvent event)
{
	for (unsigned int i = 0; i < mNbVideoMedias; i++) {
		int res = mDemuxer->CodedSource::sendDownstreamEvent(
			mVideoMedias[i], event);
		if (res < 0)
			PDRAW_LOG_ERRNO("CodedSource::sendDownstreamEvent",
					-res);
	}
}


void RecordDemuxer::VideoMedia::h264UserDataSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	int ret = 0;
	VideoMedia *self = (VideoMedia *)userdata;

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


void RecordDemuxer::VideoMedia::h264PicTimingSeiCb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_pic_timing *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

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


void RecordDemuxer::VideoMedia::h265UserDataSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_user_data_unregistered *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
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


void RecordDemuxer::VideoMedia::h265TimeCodeSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_time_code *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

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


void RecordDemuxer::VideoMedia::h265MdcvSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_mastering_display_colour_volume *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;

	for (unsigned int i = 0; i < self->mNbVideoMedias; i++) {
		for (unsigned int k = 0; k < 3; k++) {
			self->mVideoMedias[i]
				->info.mdcv.display_primaries_val
				.color_primaries[k]
				.x =
				(float)sei->display_primaries_x[k] / 50000.;
			self->mVideoMedias[i]
				->info.mdcv.display_primaries_val
				.color_primaries[k]
				.y =
				(float)sei->display_primaries_y[k] / 50000.;
		}
		self->mVideoMedias[i]
			->info.mdcv.display_primaries_val.white_point.x =
			(float)sei->white_point_x / 50000.;
		self->mVideoMedias[i]
			->info.mdcv.display_primaries_val.white_point.y =
			(float)sei->white_point_y / 50000.;
		self->mVideoMedias[i]->info.mdcv.display_primaries =
			vdef_color_primaries_from_values(
				&self->mVideoMedias[i]
					 ->info.mdcv.display_primaries_val);
		self->mVideoMedias[i]
			->info.mdcv.max_display_mastering_luminance =
			(float)sei->max_display_mastering_luminance / 10000.;
		self->mVideoMedias[i]
			->info.mdcv.min_display_mastering_luminance =
			(float)sei->min_display_mastering_luminance / 10000.;
	}
}


void RecordDemuxer::VideoMedia::h265CllSeiCb(
	struct h265_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h265_sei_content_light_level *sei,
	void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;

	if (self == nullptr)
		return;
	if (ctx == nullptr)
		return;
	if (sei == nullptr)
		return;

	for (unsigned int i = 0; i < self->mNbVideoMedias; i++) {
		self->mVideoMedias[i]->info.cll.max_cll =
			sei->max_content_light_level;
		self->mVideoMedias[i]->info.cll.max_fall =
			sei->max_pic_average_light_level;
	}
}


void RecordDemuxer::VideoMedia::timerCb(struct pomp_timer *timer,
					void *userdata)
{
	VideoMedia *self = (VideoMedia *)userdata;
	bool silent = false;
	float speed = 1.0;
	struct mp4_track_sample sample;
	CodedVideoMedia::Frame data = {};
	uint8_t *buf = nullptr, *tmp, *sei = nullptr;
	size_t bufSize = 0, offset, naluSize, seiSize = 0;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	int64_t error, duration, wait = 0;
	uint32_t waitMs = 0;
	int ret, retry = 0, waitFlush = 0;
	unsigned int outputChannelCount = 0;
	CodedChannel *channel;
	int didSeek = 0;
	unsigned int requiredMediaIndex;
	CodedVideoMedia *requiredMedia;
	struct vdef_coded_frame frameInfo;
	struct mbuf_coded_video_frame *outputFrame = nullptr;

	if (self == nullptr)
		return;

	RecordDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED) {
		PDRAW_LOGE("demuxer is not started");
		return;
	}

	speed = demuxer->mSpeed;

	if (!demuxer->mRunning) {
		self->mLastFrameDuration = 0;
		self->mLastOutputError = 0;
		return;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	memset(&sample, 0, sizeof(sample));

	/* Seeking */
	if (self->mPendingSeekTs >= 0) {
		ret = mp4_demux_seek(demuxer->mDemux,
				     (uint64_t)self->mPendingSeekTs,
				     MP4_SEEK_METHOD_PREVIOUS_SYNC);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
		} else {
			self->mLastFrameDuration = 0;
			self->mLastOutputError = 0;
		}
		self->mSeekResponse = ret;
		didSeek = 1;
	} else if (self->mPendingSeekToPrevSample) {
		ret = mp4_demux_seek_to_track_prev_sample(demuxer->mDemux,
							  self->mVideoTrackId);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_seek_to_track_prev_sample",
					-ret);
		} else {
			self->mLastFrameDuration = 0;
			self->mLastOutputError = 0;
		}
		self->mSeekResponse = ret;
		/* If not error, seek to prev sample only finishes when
		 * mPendingSeekExact goes back to false */
		if (ret != 0)
			didSeek = 1;
	} else if (self->mPendingSeekToNextSample) {
		/* Cannot fail, as there is nothing to do */
		self->mSeekResponse = 0;
		didSeek = 1;
	}

	demuxer->CodedSource::lock();

	/* Get an output buffer */
	if (self->mCurrentFrame != nullptr) {
		ret = mbuf_coded_video_frame_unref(self->mCurrentFrame);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
		self->mCurrentFrame = nullptr;
	}
	if (self->mCurrentMem != nullptr) {
		ret = mbuf_mem_unref(self->mCurrentMem);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -ret);
		self->mCurrentMem = nullptr;
	}
	ret = demuxer->getOutputMemory(self->mVideoMedias,
				       self->mNbVideoMedias,
				       &self->mCurrentMem,
				       &requiredMediaIndex);
	if ((ret < 0) || (self->mCurrentMem == nullptr)) {
		PDRAW_LOGW("failed to get an input buffer (%d)", ret);
		waitFlush = 1;
		goto out;
	}
	requiredMedia = self->mVideoMedias[requiredMediaIndex];
	ret = mbuf_mem_get_data(self->mCurrentMem, (void **)&buf, &bufSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	self->mCurrentFrameCaptureTs = 0;

	/* Get a sample */
	ret = mp4_demux_get_track_sample(demuxer->mDemux,
					 self->mVideoTrackId,
					 1,
					 buf,
					 bufSize,
					 self->mMetadataBuffer,
					 self->mMetadataBufferSize,
					 &sample);
	if (ret != 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_track_sample", -ret);
		/* Go to the next sample */
		ret = mp4_demux_get_track_sample(demuxer->mDemux,
						 self->mVideoTrackId,
						 1,
						 nullptr,
						 0,
						 nullptr,
						 0,
						 &sample);
		if (ret != 0)
			PDRAW_LOG_ERRNO("mp4_demux_get_track_sample", -ret);
		retry = 1;
		goto out;
	}
	if (sample.size == 0) {
		if (demuxer->mFrameByFrame)
			demuxer->mRunning = false;
		goto out;
	}
	silent = ((sample.silent) && (self->mPendingSeekExact)) ? true : false;

	self->mPendingSeekTs = -1;
	self->mPendingSeekToPrevSample = false;
	self->mPendingSeekToNextSample = false;
	/* Previous frame seek end on the first non-silent frame */
	if (self->mPendingSeekExact && !silent)
		didSeek = 1;
	self->mPendingSeekExact = (silent) ? self->mPendingSeekExact : false;

	frameInfo.format = requiredMedia->format;
	vdef_format_to_frame_info(&requiredMedia->info, &frameInfo.info);
	frameInfo.info.timestamp = self->mDecodingTs;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.index = self->mFrameIndex++;
	ret = mbuf_coded_video_frame_new(&frameInfo, &self->mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	if (silent || (self->mFirstFrame && !sample.sync))
		frameInfo.info.flags |= VDEF_FRAME_FLAG_SILENT;

	frameInfo.type = VDEF_CODED_FRAME_TYPE_I;
	switch (requiredMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		/* Parse the H.264 SEI to find user data SEI */
		tmp = buf;
		offset = 0;
		while (offset < sample.size) {
			enum h264_nalu_type naluType;
			enum h264_slice_type sliceType =
				H264_SLICE_TYPE_UNKNOWN;
			memcpy(&naluSize, tmp, sizeof(uint32_t));
			naluSize = ntohl(naluSize);
			naluType = (enum h264_nalu_type)(*(tmp + 4) & 0x1F);
			if (naluType == H264_NALU_TYPE_SEI) {
				sei = tmp + 4;
				seiSize = naluSize;
			} else if (naluType == H264_NALU_TYPE_SLICE_IDR) {
				frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

			} else if (naluType == H264_NALU_TYPE_SLICE) {
				sliceType = /* TODO */ H264_SLICE_TYPE_UNKNOWN;
				/* TODO (coverity complains that the code can
				 * never be reached)
				 * if (sliceType == H264_SLICE_TYPE_P)
				 *   frameInfo.type = VDEF_CODED_FRAME_TYPE_P;*/
			}
			/* add nalu to frame */
			struct vdef_nalu nalu = {};
			nalu.size = naluSize + 4;
			nalu.h264.type = naluType;
			nalu.h264.slice_type = sliceType;
			/* TODO: h264.slice_mb_count */
			ret = mbuf_coded_video_frame_add_nalu(
				self->mCurrentFrame,
				self->mCurrentMem,
				offset,
				&nalu);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_add_nalu",
					-ret);
				goto out;
			}
			tmp += 4 + naluSize;
			offset += 4 + naluSize;
		}
		if ((sei != nullptr) && (seiSize != 0)) {
			ret = h264_reader_parse_nalu(
				self->mH264Reader, 0, sei, seiSize);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			}
		}
		break;
	case VDEF_ENCODING_H265:
		/* Parse the H.265 SEI to find user data SEI */
		tmp = buf;
		offset = 0;
		while (offset < sample.size) {
			enum h265_nalu_type naluType;
			memcpy(&naluSize, tmp, sizeof(uint32_t));
			naluSize = ntohl(naluSize);
			naluType =
				(enum h265_nalu_type)((*(tmp + 4) >> 1) & 0x3F);
			if ((naluType == H265_NALU_TYPE_PREFIX_SEI_NUT) ||
			    (naluType == H265_NALU_TYPE_SUFFIX_SEI_NUT)) {
				sei = tmp + 4;
				seiSize = naluSize;
			}
			/* TODO find I/P/IDR frames from nalu type ??? */
			/* add nalu to frame */
			struct vdef_nalu nalu = {};
			nalu.size = naluSize + 4;
			nalu.h265.type = naluType;
			ret = mbuf_coded_video_frame_add_nalu(
				self->mCurrentFrame,
				self->mCurrentMem,
				offset,
				&nalu);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_add_nalu",
					-ret);
				goto out;
			}
			tmp += 4 + naluSize;
			offset += 4 + naluSize;
		}
		if ((sei != nullptr) && (seiSize != 0)) {
			ret = h265_reader_parse_nalu(
				self->mH265Reader, 0, sei, seiSize);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			}
		}
		break;
	default:
		break;
	}

	data.isSync = sample.sync;
	data.isRef = true; /* TODO? */
	data.ntpTimestamp = self->mDecodingTs;
	data.ntpUnskewedTimestamp = self->mDecodingTs;
	data.ntpRawTimestamp = self->mDecodingTs;
	data.ntpRawUnskewedTimestamp = self->mDecodingTs;
	if (sample.next_dts > 0) {
		self->mDecodingTsInc = mp4_sample_time_to_usec(
			sample.next_dts - sample.dts, self->mTimescale);
	}
	self->mDecodingTs += self->mDecodingTsInc;
	/* TODO: auSyncType */

	/* Frame metadata */
	if (sample.metadata_size > 0) {
		/* Set the metadata */
		struct vmeta_frame *meta = nullptr;
		struct vmeta_buffer meta_buf;
		vmeta_buffer_set_cdata(&meta_buf,
				       self->mMetadataBuffer,
				       sample.metadata_size,
				       0);
		ret = vmeta_frame_read(
			&meta_buf, self->mMetadataMimeType, &meta);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("vmeta_frame_read", -ret);
			goto out;
		}

		ret = mbuf_coded_video_frame_set_metadata(self->mCurrentFrame,
							  meta);
		vmeta_frame_unref(meta);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_set_metadata",
					-ret);
			goto out;
		}
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	data.demuxOutputTimestamp = curTime;
	data.playTimestamp =
		mp4_sample_time_to_usec(sample.dts, self->mTimescale);
	data.captureTimestamp = self->mCurrentFrameCaptureTs;
	data.localTimestamp = data.demuxOutputTimestamp;
	demuxer->mCurrentTime = data.playTimestamp;

	frameInfo.info.capture_timestamp = self->mCurrentFrameCaptureTs;

	/* update the frame info */
	ret = mbuf_coded_video_frame_set_frame_info(self->mCurrentFrame,
						    &frameInfo);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_set_frame_info", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		self->mCurrentFrame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&data,
		sizeof(data));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	if (didSeek) {
		/* TODO: signal once, not for all medias */
		demuxer->seekResponse(self->mSeekResponse,
				      demuxer->mCurrentTime,
				      demuxer->mSpeed);
	}

	/* Convert to byte stream if required */
	if (requiredMedia->format.data_format ==
	    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		switch (requiredMedia->format.encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_avcc_to_byte_stream(buf, sample.size);
			if (ret < 0)
				PDRAW_LOG_ERRNO("h264_avcc_to_byte_stream",
						-ret);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_hvcc_to_byte_stream(buf, sample.size);
			if (ret < 0)
				PDRAW_LOG_ERRNO("h265_hvcc_to_byte_stream",
						-ret);
			break;
		default:
			break;
		}
	}

	ret = mbuf_coded_video_frame_finalize(self->mCurrentFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

	/* Queue the buffer in the output channels */
	for (unsigned int i = 0; i < self->mNbVideoMedias; i++) {
		outputChannelCount =
			demuxer->getOutputChannelCount(self->mVideoMedias[i]);
		if (outputChannelCount == 0)
			continue;
		if (outputFrame != nullptr)
			mbuf_coded_video_frame_unref(outputFrame);
		outputFrame = self->mCurrentFrame;
		if (!vdef_coded_format_cmp(&requiredMedia->format,
					   &self->mVideoMedias[i]->format)) {
			/* The format is different, we need to pick another
			 * frame */
			ret = demuxer->copyOutputFrame(requiredMedia,
						       self->mCurrentFrame,
						       self->mVideoMedias[i],
						       &outputFrame);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("copyOutputFrame", -ret);
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
			goto out;
		}
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			const struct vdef_coded_format *caps;
			int capsCount;

			channel = demuxer->getOutputChannel(
				self->mVideoMedias[i], j);
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
	if ((self->mFirstFrame) &&
	    (!(frameInfo.info.flags & VDEF_FRAME_FLAG_SILENT))) {
		self->sendDownstreamEvent(CodedChannel::DownstreamEvent::SOS);
		self->mFirstFrame = false;
	}

	mbuf_mem_unref(self->mCurrentMem);
	self->mCurrentMem = nullptr;
	mbuf_coded_video_frame_unref(self->mCurrentFrame);
	self->mCurrentFrame = nullptr;

	if ((demuxer->mFrameByFrame) && (!silent))
		demuxer->mRunning = false;

out:
#define PREV_SAMPLE_TIME_BEFORE mp4_demux_get_track_prev_sample_time_before
#define NEXT_SAMPLE_TIME_AFTER mp4_demux_get_track_next_sample_time_after
	if (waitFlush) {
		uint64_t nextSampleTime;
		/* Flush */
		demuxer->flush();
		/* Seek to next sync sample */
		ret = mp4_demux_get_track_next_sample_time(
			demuxer->mDemux, self->mVideoTrackId, &nextSampleTime);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_next_sample_time",
					-ret);
			demuxer->CodedSource::unlock();
			return;
		}
		ret = mp4_demux_seek(demuxer->mDemux,
				     nextSampleTime,
				     MP4_SEEK_METHOD_NEXT_SYNC);
		if (ret != 0)
			PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
		/* Stop the timer */
		waitMs = 0;
	} else if (retry) {
		waitMs = 5;
	} else if (demuxer->mRunning) {
		/* Schedule the next sample */
		uint64_t nextSampleDts = mp4_sample_time_to_usec(
			sample.next_dts, self->mTimescale);

		/* If error > 0 we are late, if error < 0 we are early */
		error = ((self->mLastFrameOutputTime == 0) ||
			 (self->mLastFrameDuration == 0) || (speed == 0.) ||
			 (speed >= PDRAW_PLAY_SPEED_MAX) || (silent))
				? 0
				: curTime - self->mLastFrameOutputTime -
					  self->mLastFrameDuration +
					  self->mLastOutputError;
		if (self->mLastFrameOutputTime) {
			/* Average frame output rate
			 * (sliding average, alpha = 1/2) */
			self->mAvgOutputInterval +=
				((int64_t)(curTime -
					   self->mLastFrameOutputTime) -
				 self->mAvgOutputInterval) >>
				1;
		}

		/* Sample duration */
		if ((speed >= PDRAW_PLAY_SPEED_MAX) || (nextSampleDts == 0) ||
		    (silent)) {
			duration = 0;
		} else if (speed < 0.) {
			/* Negative speed => play backward */
			nextSampleDts = mp4_sample_time_to_usec(
				sample.prev_sync_dts, self->mTimescale);
			uint64_t pendingSeekTs = nextSampleDts;
			uint64_t nextSyncSampleDts = nextSampleDts;
			uint64_t wantedSampleDts;
			duration = nextSampleDts -
				   mp4_sample_time_to_usec(sample.dts,
							   self->mTimescale);
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				wantedSampleDts = nextSyncSampleDts;
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				ret = PREV_SAMPLE_TIME_BEFORE(
					demuxer->mDemux,
					self->mVideoTrackId,
					wantedSampleDts,
					1,
					&nextSyncSampleDts);
				if (ret < 0) {
					PDRAW_LOG_ERRNO(
						"mp4_demux_get_track_"
						"prev_sample_time_before",
						-ret);
				}
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration = nextSyncSampleDts -
						      mp4_sample_time_to_usec(
							      sample.dts,
							      self->mTimescale);
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
					PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
				}
			}
		} else {
			/* Positive speed => play forward */
			uint64_t pendingSeekTs = 0;
			uint64_t nextSyncSampleDts = nextSampleDts;
			uint64_t wantedSampleDts;
			duration = nextSampleDts -
				   mp4_sample_time_to_usec(sample.dts,
							   self->mTimescale);
			if (speed != 0.)
				duration = (int64_t)((float)duration / speed);
			int64_t newDuration = duration;
			while (newDuration - error < 0) {
				wantedSampleDts = nextSyncSampleDts;
				/* We can't keep up => seek to the next sync
				 * sample that gives a positive wait time */
				ret = NEXT_SAMPLE_TIME_AFTER(
					demuxer->mDemux,
					self->mVideoTrackId,
					wantedSampleDts,
					1,
					&nextSyncSampleDts);
				if (ret < 0) {
					PDRAW_LOG_ERRNO(
						"mp4_demux_get_track_"
						"next_sample_time_after",
						-ret);
				}
				if (nextSyncSampleDts > 0) {
					pendingSeekTs = nextSyncSampleDts;
					newDuration = nextSyncSampleDts -
						      mp4_sample_time_to_usec(
							      sample.dts,
							      self->mTimescale);
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
			     2 * self->mAvgOutputInterval)) {
				/* Only seek if the resulting wait time is less
				 * than twice the average frame output rate */
				PDRAW_LOGD(
					"unable to keep up with playback "
					"timings, seek forward %.2f ms",
					(float)(nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts,
							self->mTimescale)) /
						1000.);
				duration = newDuration;
				nextSampleDts = nextSyncSampleDts;
				ret = mp4_demux_seek(
					demuxer->mDemux,
					pendingSeekTs,
					MP4_SEEK_METHOD_PREVIOUS_SYNC);
				if (ret < 0) {
					PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
				}
			}
		}

		if (nextSampleDts != 0) {
			wait = duration - error;
			/* TODO: loop in the timer cb when silent
			 * or speed>=PDRAW_PLAY_SPEED_MAX */
			if (wait < 0) {
				if (duration > 0) {
					PDRAW_LOGD(
						"unable to keep "
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
			self->sendDownstreamEvent(
				CodedChannel::DownstreamEvent::EOS);

			/* Notify of the end of range */
			/* TODO: signal once, not for all medias */
			demuxer->onEndOfRange(demuxer->mCurrentTime);
		}
		self->mLastFrameOutputTime = curTime;
		self->mLastFrameDuration = duration;
		self->mLastOutputError = error;

#if 0
		/* TODO: remove debug */
		PDRAW_LOGD("timerCb: error=%d duration=%d wait=%d%s",
			   (int)error,
			   (int)duration,
			   (int)wait,
			   (silent) ? " (silent)" : "");
#endif
	} else {
		self->mLastFrameOutputTime = curTime;
		self->mLastFrameDuration = 0;
		self->mLastOutputError = 0;
	}

	demuxer->CodedSource::unlock();

	if (waitMs > 0) {
		ret = pomp_timer_set(timer, waitMs);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -ret);
	}
}

} /* namespace Pdraw */
