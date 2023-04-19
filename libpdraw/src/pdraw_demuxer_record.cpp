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


RecordDemuxer::RecordDemuxer(Session *session,
			     Element::Listener *elementListener,
			     Source::Listener *sourceListener,
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

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
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


bool RecordDemuxer::isMediaTrack(struct mp4_track_info *tkinfo,
				 char **keys,
				 char **values,
				 int count)
{
	int i, c = 0;

	if (tkinfo->type == MP4_TRACK_TYPE_VIDEO)
		return true;

	/* "video/raw" track with full format as MIME type parameters */
	if ((tkinfo->mime_format != nullptr) &&
	    (strncmp(tkinfo->mime_format,
		     VDEF_RAW_MIME_TYPE ";",
		     strlen(VDEF_RAW_MIME_TYPE ";")) == 0))
		return true;

	/* Old regis-specific raw video track */
	for (i = 0; i < count; i++) {
		if (strcmp(keys[i], "com.parrot.regis.format") == 0)
			c++;
		else if (strcmp(keys[i], "com.parrot.regis.resolution") == 0)
			c++;
	}
	if (c == 2)
		return true;

	return false;
}


int RecordDemuxer::start(void)
{
	int ret;
	unsigned int i, tkCount = 0;
	struct mp4_media_info info;
	struct mp4_track_info tk;
	struct pdraw_demuxer_media *medias = nullptr;
	std::vector<struct pdraw_demuxer_media *> selectedMedias;
	std::vector<struct pdraw_demuxer_media *>::iterator m;
	size_t mediasCount = 0, mediaIndex = 0;
	unsigned int hrs = 0, min = 0, sec = 0;
	bool ready = true;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: demuxer is not created", __func__);
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

	/* Count the number of media tracks */
	for (i = 0; i < tkCount; i++) {
		unsigned int count = 0;
		char **keys = nullptr, **values = nullptr;
		ret = mp4_demux_get_track_info(mDemux, i, &tk);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_info", -ret);
			continue;
		}
		ret = mp4_demux_get_track_metadata_strings(
			mDemux, tk.id, &count, &keys, &values);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_metadata_strings",
					-ret);
			continue;
		}
		if (!isMediaTrack(&tk, keys, values, count))
			continue;
		mediasCount++;
	}
	if (mediasCount == 0) {
		PDRAW_LOGE("no media track");
		ret = -ENOENT;
		goto exit;
	}
	medias = (struct pdraw_demuxer_media *)calloc(mediasCount,
						      sizeof(*medias));
	if (medias == nullptr) {
		ret = -ENOMEM;
		goto exit;
	}

	/* List all tracks */
	for (i = 0; i < tkCount; i++) {
		ret = mp4_demux_get_track_info(mDemux, i, &tk);
		if (ret != 0 || tk.type != MP4_TRACK_TYPE_VIDEO)
			continue;
		struct pdraw_demuxer_media *current = &medias[mediaIndex];
		memset(current, 0, sizeof(*current));
		current->media_id = tk.id;
		current->idx = i;
		current->name = strdup(tk.name);
		current->is_default = tk.enabled;
		mediaIndex++;
		if (current->is_default)
			selectedMedias.push_back(current);
		(void)fetchSessionMetadata(tk.id, &current->session_meta);
	}

	/* Ask which media(s) to use from the application */
	ret = selectMedia(medias, mediasCount);
	if (ret == 0 || ret == -ENOSYS) {
		if (selectedMedias.empty()) {
			PDRAW_LOGE(
				"application requested default media, "
				"but no default media found");
			ret = -ENOENT;
			goto exit;
		}
		if (selectedMedias.size() == 1) {
			PDRAW_LOGI("auto-selecting media %d (%s)",
				   selectedMedias.back()->media_id,
				   selectedMedias.back()->name);
		} else {
			PDRAW_LOGI("audo-selecting medias {");
			m = selectedMedias.begin();
			while (m != selectedMedias.end()) {
				PDRAW_LOGI(" - %d (%s)",
					   (*m)->media_id,
					   (*m)->name);
				m++;
			}
			PDRAW_LOGI("}");
		}
	} else if (ret == -ECANCELED) {
		PDRAW_LOGI("application cancelled the media selection");
		ready = false;
		goto exit;
	} else if (ret < 0) {
		PDRAW_LOG_ERRNO("application failed to select a media", -ret);
		ret = -EPROTO;
		goto exit;
	} else {
		selectedMedias.clear();
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
	m = selectedMedias.begin();
	while (m != selectedMedias.end()) {
		ret = mp4_demux_get_track_info(mDemux, (*m)->idx, &tk);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_info", -ret);
			goto exit;
		}
		RecordDemuxer::DemuxerMedia *media;
		switch (tk.type) {
		case MP4_TRACK_TYPE_VIDEO:
			media = new RecordDemuxer::DemuxerCodedVideoMedia(this);
			break;
		case MP4_TRACK_TYPE_METADATA:
			media = new RecordDemuxer::DemuxerRawVideoMedia(this);
			break;
		default:
			ret = -EPROTO;
			PDRAW_LOGE("unsupported track type");
			goto exit;
		}
		ret = media->setup(&tk);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("VideoMedia::setup", -ret);
			delete media;
			goto exit;
		}
		mMedias.push_back(media);
		m++;
	}

exit:
	/* Cleanup track list */
	if (medias != nullptr) {
		for (size_t j = 0; j < mediasCount; j++) {
			free((void *)medias[j].name);
		}
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
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);

	readyToPlay(false);

	mRunning = false;

	Source::lock();

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->stop();
		p++;
	}

	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	Source::unlock();

	return 0;
}


int RecordDemuxer::flush(void)
{
	if ((mState != STARTED) && (mState != STOPPING)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	Source::lock();

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->stop();
		p++;
	}

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		Media *media = getOutputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGW("failed to get media at index %d", i);
			continue;
		}

		unsigned int outputChannelCount = getOutputChannelCount(media);

		/* Flush the output channels */
		for (unsigned int j = 0; j < outputChannelCount; j++) {
			Channel *channel = getOutputChannel(media, j);
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

	Source::unlock();

	if (mChannelsFlushing == 0)
		completeFlush();

	return 0;
}


void RecordDemuxer::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("channel flushed media name=%s (channel owner=%p)",
		   media->getName().c_str(),
		   channel->getOwner());

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
		auto p = mMedias.begin();
		while (p != mMedias.end()) {
			(*p)->play();
			p++;
		}
	}
	if (mState == STOPPING)
		completeTeardown();
}


void RecordDemuxer::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}

	int ret = removeOutputChannel(media, channel);
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeOutputChannel", -ret);

	completeTeardown();
}


void RecordDemuxer::completeTeardown(void)
{
	Source::lock();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		Media *media = getOutputMedia(i);
		if (media && getOutputChannelCount(media) > 0) {
			Source::unlock();
			return;
		}
	}

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		delete *p;
		p++;
	}
	mMedias.clear();

	Source::unlock();

	if (mState == STOPPING) {
		closeResponse(0);
		setStateAsyncNotify(STOPPED);
	}
}


int RecordDemuxer::play(float speed)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
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
		auto p = mMedias.begin();
		while (p != mMedias.end()) {
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
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	return mReadyToPlay;
}


bool RecordDemuxer::isPaused(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int RecordDemuxer::previous(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->previous();
		p++;
	}
	mRunning = true;

	return 0;
}


int RecordDemuxer::next(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->next();
		p++;
	}
	mRunning = true;

	return 0;
}


int RecordDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->seek(delta, exact);
		p++;
	}

	return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (timestamp > mDuration)
		timestamp = mDuration;
	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->seekTo(timestamp, exact);
		p++;
	}
	mRunning = true;

	return 0;
}


RecordDemuxer::DemuxerMedia::DemuxerMedia(RecordDemuxer *demuxer) :
		mDemuxer(demuxer), mTrackId(0),
		mMediaType(Media::Type::UNKNOWN), mFirstSample(true),
		mSampleIndex(0), mMetadataMimeType(nullptr),
		mMetadataBufferSize(0), mMetadataBuffer(nullptr), mTimescale(0),
		mAvgOutputInterval(0), mLastSampleOutputTime(0),
		mLastSampleDuration(0), mLastOutputError(0), mPendingSeekTs(-1),
		mPendingSeekExact(false), mPendingSeekToPrevSample(false),
		mPendingSeekToNextSample(false), mSeekResponse(0),
		mTimer(nullptr)
{
	std::string name = demuxer->getName() + "#DemuxerMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerMedia::~DemuxerMedia(void)
{
	int ret;

	if (mTimer != nullptr) {
		ret = pomp_timer_clear(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -ret);
		ret = pomp_timer_destroy(mTimer);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -ret);
	}

	free(mMetadataBuffer);
	free(mMetadataMimeType);
}


int RecordDemuxer::DemuxerMedia::setup(struct mp4_track_info *tkinfo)
{
	int ret, err;

	std::string name =
		mDemuxer->getName() + "#track#" + std::to_string(tkinfo->id);
	Loggable::setName(name);

	mTrackId = tkinfo->id;
	mTimescale = tkinfo->timescale;
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

	ret = setupMedia(tkinfo);
	if (ret < 0)
		goto error;

	mMediaType = Media::Type::RAW_VIDEO;

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
	free(mMetadataBuffer);
	mMetadataBuffer = nullptr;
	return ret;
}


void RecordDemuxer::DemuxerMedia::play(void)
{
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::DemuxerMedia::previous(void)
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


void RecordDemuxer::DemuxerMedia::next(void)
{
	mPendingSeekToNextSample = true;
	mPendingSeekToPrevSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::DemuxerMedia::seek(int64_t delta, bool exact)
{
	int64_t ts = (int64_t)mDemuxer->mCurrentTime + delta;
	if (ts < 0)
		ts = 0;
	if (ts > (int64_t)mDemuxer->mDuration)
		ts = mDemuxer->mDuration;
	seekTo(ts, exact);
}


void RecordDemuxer::DemuxerMedia::seekTo(uint64_t timestamp, bool exact)
{
	if (timestamp > mDemuxer->mDuration)
		timestamp = mDemuxer->mDuration;
	mPendingSeekTs = (int64_t)timestamp;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	pomp_timer_set(mTimer, 1);
}


void RecordDemuxer::DemuxerMedia::stop(void)
{
	(void)pomp_timer_clear(mTimer);
}


void RecordDemuxer::DemuxerMedia::timerCb(struct pomp_timer *timer,
					  void *userdata)
{
	DemuxerMedia *self = (DemuxerMedia *)userdata;
	int ret;
	bool retry = false, silent = false;
	float speed = 1.0;
	int64_t error, duration, wait = 0;
	uint32_t waitMs = 0;
	bool didSeek = false, waitFlush = false;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	struct mp4_track_sample sample = {};

	if (self == nullptr)
		return;

	RecordDemuxer *demuxer = self->mDemuxer;

	if (demuxer->mState != STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return;
	}

	speed = demuxer->mSpeed;

	if (!demuxer->mRunning) {
		self->mLastSampleDuration = 0;
		self->mLastOutputError = 0;
		return;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);

	/* Seeking */
	if (self->mPendingSeekTs >= 0) {
		ret = mp4_demux_seek(demuxer->mDemux,
				     (uint64_t)self->mPendingSeekTs,
				     MP4_SEEK_METHOD_PREVIOUS_SYNC);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
		} else {
			self->mLastSampleDuration = 0;
			self->mLastOutputError = 0;
		}
		self->mSeekResponse = ret;
		didSeek = true;
	} else if (self->mPendingSeekToPrevSample) {
		ret = mp4_demux_seek_to_track_prev_sample(demuxer->mDemux,
							  self->mTrackId);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_seek_to_track_prev_sample",
					-ret);
		} else {
			self->mLastSampleDuration = 0;
			self->mLastOutputError = 0;
		}
		self->mSeekResponse = ret;
		/* If not error, seek to prev sample only finishes when
		 * mPendingSeekExact goes back to false */
		if (ret != 0)
			didSeek = true;
	} else if (self->mPendingSeekToNextSample) {
		/* Cannot fail, as there is nothing to do */
		self->mSeekResponse = 0;
		didSeek = true;
	}

	demuxer->Source::lock();

	ret = self->processSample(
		&sample, &silent, &retry, &didSeek, &waitFlush);
	if (ret < 0)
		goto out;

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
			demuxer->mDemux, self->mTrackId, &nextSampleTime);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_next_sample_time",
					-ret);
			demuxer->Source::unlock();
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
		error = ((self->mLastSampleOutputTime == 0) ||
			 (self->mLastSampleDuration == 0) || (speed == 0.) ||
			 (speed >= PDRAW_PLAY_SPEED_MAX) || (silent))
				? 0
				: curTime - self->mLastSampleOutputTime -
					  self->mLastSampleDuration +
					  self->mLastOutputError;
		if (self->mLastSampleOutputTime) {
			/* Average frame output rate
			 * (sliding average, alpha = 1/2) */
			self->mAvgOutputInterval +=
				((int64_t)(curTime -
					   self->mLastSampleOutputTime) -
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
				/* We can't keep up => seek to the next
				 * sync sample that gives a positive
				 * wait time */
				ret = PREV_SAMPLE_TIME_BEFORE(
					demuxer->mDemux,
					self->mTrackId,
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
						newDuration =
							(float)newDuration /
							speed;
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
				/* We can't keep up => seek to the next
				 * sync sample that gives a positive
				 * wait time */
				ret = NEXT_SAMPLE_TIME_AFTER(
					demuxer->mDemux,
					self->mTrackId,
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
						newDuration =
							(float)newDuration /
							speed;
					}
				} else {
					break;
				}
			}
			if ((pendingSeekTs > 0) &&
			    (newDuration - error <
			     2 * self->mAvgOutputInterval)) {
				/* Only seek if the resulting wait time
				 * is less than twice the average frame
				 * output rate */
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
				Channel::DownstreamEvent::EOS);

			/* Notify of the end of range */
			/* TODO: signal once, not for all medias */
			demuxer->onEndOfRange(demuxer->mCurrentTime);
		}
		self->mLastSampleOutputTime = curTime;
		self->mLastSampleDuration = duration;
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
		self->mLastSampleOutputTime = curTime;
		self->mLastSampleDuration = 0;
		self->mLastOutputError = 0;
	}

	demuxer->Source::unlock();

	if (waitMs > 0) {
		ret = pomp_timer_set(timer, waitMs);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -ret);
	}
}

} /* namespace Pdraw */
