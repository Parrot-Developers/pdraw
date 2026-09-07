/**
 * Parrot Drones Audio and Video Vector library
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

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


RecordDemuxer::RecordDemuxer(Session *session,
			     Element::Listener *elementListener,
			     Source::Listener *sourceListener,
			     DemuxerWrapper *wrapper,
			     IPdraw::IDemuxer::Listener *demuxerListener,
			     const std::string &fileName,
			     const struct pdraw_demuxer_params *params) :
		Demuxer(session,
			elementListener,
			sourceListener,
			wrapper,
			demuxerListener,
			params),
		mFileName(fileName)
{
	Element::setClassName(__func__);

	mCompleteStartHandler.set([this] { idleCompleteStart(); });

	setState(State::CREATED);

	if (params != nullptr)
		mPlaybackMode = params->playback_mode;
}


RecordDemuxer::~RecordDemuxer()
{
	int err;

	if (mState != State::STOPPED && mState != State::CREATED)
		PDRAW_LOGW("demuxer is still running");

	/* Remove any leftover idle callbacks */
	err = mSession->getPompLoop()->idleRemove(this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	destroyAllMedias();

	if (mDemux != nullptr) {
		err = mp4_demux_close(mDemux);
		if (err < 0)
			PDRAW_LOG_ERRNO("mp4_demux_close", -err);
		mDemux = nullptr;
	}
}


int RecordDemuxer::fetchSessionMetadata(unsigned int trackId,
					struct vmeta_session *meta)
{
	int ret;
	unsigned int count = 0;
	unsigned int i;
	char **keys = nullptr;
	const char *key;
	char **values = nullptr;
	const char *value;

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
		if (key && value) {
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
		if (key && value) {
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


bool RecordDemuxer::isMediaTrack(const struct mp4_track_info *tkinfo,
				 char **keys,
				 [[maybe_unused]] char **values,
				 int count)
{

	int c = 0;

	if (tkinfo->type == MP4_TRACK_TYPE_VIDEO)
		return true;

	if (tkinfo->type == MP4_TRACK_TYPE_AUDIO)
		return true;

	/* "video/raw" track with full format as MIME type parameters */
	if ((tkinfo->mime_format != nullptr) &&
	    (strncmp(tkinfo->mime_format,
		     VDEF_RAW_MIME_TYPE ";",
		     strlen(VDEF_RAW_MIME_TYPE ";")) == 0))
		return true;

	/* Old regis-specific raw video track */
	for (int i = 0; i < count; i++) {
		if (strcmp(keys[i], "com.parrot.regis.format") == 0)
			c++;
		else if (strcmp(keys[i], "com.parrot.regis.resolution") == 0)
			c++;
	}
	if (c == 2)
		return true;

	return false;
}


int RecordDemuxer::start()
{
	int ret;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: demuxer is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Create the MP4 demuxer */
	ret = mp4_demux_open(mFileName.c_str(), &mDemux);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_open", -ret);
		return ret;
	}

	ret = mSession->getPompLoop()->idleAdd(&mCompleteStartHandler, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -ret);
		return ret;
	}

	return 0;
}


int RecordDemuxer::completeStart()
{
	int ret;
	unsigned int i;
	unsigned int tkCount = 0;
	struct mp4_media_info info;
	struct mp4_track_info tk;
	size_t mediasCount = 0;
	size_t mediaIndex = 0;
	unsigned int hrs = 0;
	unsigned int min = 0;
	unsigned int sec = 0;
	bool ready = true;
	uint32_t selectedMedias = 0;
	struct pdraw_demuxer_media *newMediaList = nullptr;
	size_t newMediaListSize = 0;
	std::vector<struct pdraw_demuxer_media *> newDefaultMedias;

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
		char **keys = nullptr;
		char **values = nullptr;
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

	newMediaList = static_cast<struct pdraw_demuxer_media *>(
		calloc(mediasCount, sizeof(*newMediaList)));
	if (newMediaList == nullptr) {
		PDRAW_LOGE("calloc");
		goto exit;
	}
	newMediaListSize = mediasCount;

	/* List all tracks */
	for (i = 0; i < tkCount; i++) {
		char **keys = nullptr;
		char **values = nullptr;
		unsigned int count = 0;
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
		struct pdraw_demuxer_media *current = &newMediaList[mediaIndex];
		current->media_id = tk.id;
		current->idx = i;
		switch (tk.type) {
		case MP4_TRACK_TYPE_VIDEO:
		case MP4_TRACK_TYPE_METADATA:
			current->type = PDRAW_MEDIA_TYPE_VIDEO;
			break;
		case MP4_TRACK_TYPE_AUDIO:
			current->type = PDRAW_MEDIA_TYPE_AUDIO;
			break;
		default:
			current->type = PDRAW_MEDIA_TYPE_UNKNOWN;
			break;
		}
		current->name = strdup(tk.name);
		current->is_default = tk.enabled;
		mediaIndex++;
		if (current->is_default)
			newDefaultMedias.push_back(current);
		if (current->type == PDRAW_MEDIA_TYPE_VIDEO) {
			(void)fetchSessionMetadata(
				tk.id, &current->video.session_meta);
		}
	}

	ret = updateMediaList(newMediaList,
			      newMediaListSize,
			      newDefaultMedias,
			      &selectedMedias);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("updateMediaList", -ret);
		goto exit;
	}

	newMediaList = nullptr;
	newMediaListSize = 0;
	newDefaultMedias.clear();

	/* Ask which media(s) to use from the application */
	ret = callSelectMedia(selectedMedias);
	if (ret >= 0) {
		selectedMedias = ret;
	} else if (ret == -ENOSYS) {
		selectedMedias = 0;
	} else if (ret == -ECANCELED) {
		PDRAW_LOGI("application cancelled the media selection");
		ready = false;
		goto exit;
	} else if (ret < 0) {
		PDRAW_LOGE("application failed to select a media");
		/* Selecting a wrong media is an error, stop the demuxer
		 * to either report an open response, or an unrecoverable error
		 * to the application */
		goto exit;
	}

	ret = Demuxer::selectMedia(selectedMedias);
	if (ret < 0) {
		ready = false;
		goto exit;
	}

	processSelectedMedias();

	ret = 0;

exit:
	if ((ret == 0) || (ret == -ECANCELED)) {
		setState(State::STARTED);
		openResponse(ret);
		readyToPlay(ready);
		/* TODO: notify readyToPlay = false at end of file */
		if (!ready && (ret != -ECANCELED))
			onUnrecoverableError();
		ret = 0;
	} else {
		setState(State::CREATED);
	}

	pdraw_demuxerMediaListFree(newMediaList, newMediaListSize);

	return ret;
}


void RecordDemuxer::idleCompleteStart()
{
	(void)completeStart();
}


int RecordDemuxer::stop()
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState != State::STARTED && mState != State::STARTING) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);

	/* Note: the demuxer listener is not cleared here to allow calling
	 * the IDemuxer::Listener::demuxerCloseResponse listener function when
	 * the IDemuxer::close function was called; clearing the listener when
	 * deleting the API object is done by calling
	 * Demuxer::clearDemuxerListener in the API object destructor prior
	 * to calling Demuxer::stop */

	readyToPlay(false);

	setRunning(false);

	Source::lock();

	auto p = mMedias.begin();
	while (p != mMedias.end()) {
		(*p)->stop();
		p++;
	}

	ret = flush();
	if ((ret < 0) && (ret != -EALREADY))
		PDRAW_LOG_ERRNO("flush", -ret);

	Source::unlock();

	return 0;
}


void RecordDemuxer::setRunning(bool running)
{
	for (const auto &m : mMedias)
		m->setRunning(running);
	mRunning = running;
	if (!mRunning)
		mFrameByFrame = true;
	if (mRunning && !mWasRunningOnce)
		mWasRunningOnce = true;
}


void RecordDemuxer::onMediaSeekComplete(int seekResponse)
{
	bool anySeeking = false;
	if ((seekResponse != 0) && (mSeekResponse == 0))
		mSeekResponse = seekResponse;
	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		anySeeking |= m->isSeeking();
	}
	/* Demuxer stops running when no medias are running */
	if (anySeeking)
		return;
	if (mFrameByFrame) {
		this->drain();
		/* Call seekResponse once drained */
	} else {
		this->seekResponse(mSeekResponse, mCurrentTime, mSpeed);
		mPendingSeek = false;
		mSeekResponse = 0;
	}
}


void RecordDemuxer::onMediaPlayComplete(int playResponse)
{
	bool any = false;
	if ((playResponse != 0) && (mPlayResponse == 0))
		mPlayResponse = playResponse;
	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		any |= m->isPendingPlay();
	}
	if (!any && (getPendingCommand() == Command::PLAY)) {
		mFrameByFrame = false;
		this->playResponse(mPlayResponse, mCurrentTime, mSpeed);
		mPlayResponse = 0;
	}
}


void RecordDemuxer::onMediaRunningStateChanged()
{
	bool any = false;
	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		any |= m->isRunning();
	}
	/* Demuxer stops running when no medias are running */
	if (!any)
		setRunning(false);
}


int RecordDemuxer::flush(bool discard)
{
	if ((mState != State::STARTED) && (mState != State::STOPPING)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("demuxer is already flushed, nothing to do");
		/* No need to call complete flush */
		break;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	Source::lock();

	mChannelsFlushing = 0;

	for (const auto &m : mMedias) {
		m->setDestroyAfterFlush(false);
		if (mFlushDiscard)
			m->flush();
		else
			m->drain();
	}

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		const Media *media = getOutputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGW("failed to get media at index %d", i);
			continue;
		}

		mChannelsFlushing += getOutputChannelCount(media);
	}

	Source::unlock();

	if (mChannelsFlushing == 0)
		completeFlush();

	return 0;
}


void RecordDemuxer::destroyAllMedias()
{
	mMedias.clear();
}


void RecordDemuxer::onChannelFlushed(Channel *channel)
{
	bool destroyMedia = false;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("channel flushed media name=%s (channel owner=%p)",
		   media->getName().c_str(),
		   channel->getOwner());

	for (const auto &m : mMedias) {
		if (m->hasMedia(media)) {
			m->channelFlushed(channel);
			destroyMedia = m->getDestroyAfterFlush();
			break;
		}
	}

	if (mState == State::STOPPING || destroyMedia) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mChannelsFlushing <= 0) {
		mChannelsFlushing = 0;
		completeFlush();
	}
}


void RecordDemuxer::onChannelDrained(Channel *channel)
{
	bool destroyMedia = false;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("channel drained media name=%s (channel owner=%p)",
		   media->getName().c_str(),
		   channel->getOwner());

	for (const auto &m : mMedias) {
		if (m->hasMedia(media)) {
			m->channelDrained(channel);
			destroyMedia = m->getDestroyAfterFlush();
			break;
		}
	}

	if (mState == State::STOPPING || destroyMedia) {
		int ret = channel->teardown();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->teardown", -ret);
	}

	if (--mChannelsFlushing <= 0) {
		mChannelsFlushing = 0;
		completeFlush();
	}
}


void RecordDemuxer::completeFlush()
{
	setFlushingState(FlushingState::FLUSHED);

	if ((getPendingCommand() == Command::SEEK) && mPendingSeek &&
	    mFrameByFrame) {
		this->seekResponse(mSeekResponse, mCurrentTime, mSpeed);
		mPendingSeek = false;
		mSeekResponse = 0;
	}

	if (getPendingCommand() == Command::PAUSE_NEXT && !mWasRunningOnce) {
		next();
	} else if ((getPendingCommand() == Command::PAUSE_NEXT) ||
		   (getPendingCommand() == Command::PAUSE)) {
		pauseResponse(0, getCurrentTime());
	}

	if (mRunning && !mFrameByFrame) {
		/* restart playing */
		for (const auto &m : mMedias) {
			if (m->isTearingDown())
				continue;
			m->play();
		}
	}

	if (mState == State::STOPPING)
		completeTeardown();
}


void RecordDemuxer::onChannelUnlink(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}

	int ret = removeOutputChannel(media, channel);
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeOutputChannel", -ret);

	for (auto p = mMedias.begin(); p != mMedias.end(); p++) {
		if ((*p)->hasMedia(media)) {
			(*p)->channelUnlink(channel);
			if ((*p)->getMediaCount() == 0 &&
			    (*p)->isTearingDown()) {
				/* Delete media */
				PDRAW_LOGI("removing media %s",
					   (*p)->getCName());
				p = mMedias.erase(p);
				break;
			}
			break;
		}
	}
	selectReferenceTrack();
	completeTeardown();
}


void RecordDemuxer::completeTeardown()
{
	Source::lock();

	unsigned int outputMediaCount = getOutputMediaCount();
	for (unsigned int i = 0; i < outputMediaCount; i++) {
		const Media *media = getOutputMedia(i);
		if (media && getOutputChannelCount(media) > 0) {
			Source::unlock();
			return;
		}
	}

	for (const auto &m : mMedias) {
		if (!m->isTearingDown()) {
			Source::unlock();
			return;
		}
	}

	destroyAllMedias();

	Source::unlock();

	if (mState == State::STOPPING) {
		closeResponse(0);
		setStateAsyncNotify(State::STOPPED);
	}
}


int RecordDemuxer::internalPlay(float speed)
{
	mSpeed = speed;
	setRunning(true);
	/* mFrameByFrame unset in onMediaPlayComplete */

	/* Async play resp */
	return 0;
}


int RecordDemuxer::internalPause()
{
	setRunning(false);
	mFrameByFrame = true;
	drain();

	/* Async pause resp */
	return 0;
}


int RecordDemuxer::play(float speed)
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	switch (getPendingCommand()) {
	case Command::NONE:
		/* OK */
		break;
	case Command::PLAY:
	case Command::PAUSE:
	case Command::PAUSE_NEXT:
		if (((getPendingCommand() == Command::PLAY) && (speed != 0.)) ||
		    ((getPendingCommand() == Command::PAUSE ||
		      getPendingCommand() == Command::PAUSE_NEXT) &&
		     (speed == 0.)))
			return -EALREADY;
		[[fallthrough]];
	default:
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	if (speed == 0.) {
		if (!mWasRunningOnce)
			setPendingCommand(Command::PAUSE_NEXT);
		else
			setPendingCommand(Command::PAUSE);
		internalPause();
	} else {
		setPendingCommand(Command::PLAY);
		for (const auto &m : mMedias) {
			if (m->isTearingDown())
				continue;
			m->play();
		}
		internalPlay(speed);
	}

	return 0;
}


bool RecordDemuxer::isReadyToPlay() const
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	return mReadyToPlay;
}


bool RecordDemuxer::isPaused() const
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return false;
	}

	bool running = mRunning && !mFrameByFrame;

	return !running;
}


int RecordDemuxer::previous()
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	if (getPendingCommand() != Command::NONE) {
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		m->previous();
	}
	mPendingSeek = true;
	mSeekResponse = 0;
	setRunning(true);

	setPendingCommand(Command::SEEK);

	return 0;
}


int RecordDemuxer::next()
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}
	if (!mFrameByFrame) {
		PDRAW_LOGE("%s: demuxer is not paused", __func__);
		return -EPROTO;
	}

	switch (getPendingCommand()) {
	case Command::NONE:
		/* OK */
		break;
	case Command::PAUSE_NEXT:
		if (!mWasRunningOnce)
			break;
		[[fallthrough]];
	case Command::SEEK:
	default:
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		m->next();
	}
	mPendingSeek = true;
	mSeekResponse = 0;
	setRunning(true);

	if (getPendingCommand() != Command::PAUSE_NEXT)
		setPendingCommand(Command::SEEK);

	return 0;
}


int RecordDemuxer::seek(int64_t delta, bool exact)
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	if (getPendingCommand() != Command::NONE) {
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		m->seek(delta, exact);
	}
	mPendingSeek = true;
	mSeekResponse = 0;
	setRunning(true);

	setPendingCommand(Command::SEEK);

	return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}
	if (!isReadyToPlay()) {
		PDRAW_LOGE("%s: demuxer is not ready to play", __func__);
		return -EPROTO;
	}

	if (getPendingCommand() != Command::NONE) {
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(getPendingCommand()));
		return -EBUSY;
	}

	if (timestamp > mDuration)
		timestamp = mDuration;

	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		m->seekTo(timestamp, exact);
	}
	mPendingSeek = true;
	mSeekResponse = 0;
	if (!mRunning) {
		/* When seeking in pause, enable frame by frame */
		mFrameByFrame = true;
	}
	setRunning(true);

	setPendingCommand(Command::SEEK);

	return 0;
}


int RecordDemuxer::getChapterList(struct pdraw_chapter **chapterList,
				  size_t *chapterCount)
{
	int ret;
	unsigned int count = 0;
	uint64_t *times = nullptr;
	char **names = nullptr;
	struct pdraw_chapter *_chapterList = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(chapterList == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(chapterCount == nullptr, EINVAL);

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	ret = mp4_demux_get_chapters(mDemux, &count, &times, &names);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_demux_get_chapters", -ret);
		return ret;
	}

	if (count == 0) {
		*chapterList = nullptr;
		*chapterCount = 0;
		return -ENOENT;
	}

	_chapterList = static_cast<struct pdraw_chapter *>(
		calloc(count, sizeof(*_chapterList)));
	if (_chapterList == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		return ret;
	}

	/* Chapters deep copy */
	for (size_t i = 0; i < count; i++) {
		_chapterList[i].ts_us = times[i];
		_chapterList[i].name = xstrdup(names[i]);
	}

	*chapterList = _chapterList;
	*chapterCount = count;

	return 0;
}


int RecordDemuxer::selectReferenceTrack() const
{
	const DemuxerMedia *prevRefMedia = nullptr;
	const DemuxerMedia *refMedia = nullptr;
	size_t count = 0;

	for (const auto &m : mMedias) {
		if (m->isReference())
			prevRefMedia = m.get();
		if (m->isTearingDown())
			continue;
		count++;
	}

	if (count == 0)
		return -ENOENT;

	/* Three passes:
	 * #1: video (coded or raw) & enabled
	 * #2: video (coded or raw)
	 * #3: 1st track
	 */
	for (size_t i = 0; i < 3; i++) {
		for (const auto &m : mMedias) {
			if (m->isTearingDown())
				continue;
			if ((i <= 1) &&
			    (m->getMediaType() != Media::Type::CODED_VIDEO) &&
			    (m->getMediaType() != Media::Type::RAW_VIDEO))
				continue;
			if ((i == 0) && !m->isTrackEnabled())
				continue;
			refMedia = m.get();
			break;
		}
		if (refMedia != nullptr)
			break;
	}

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(refMedia == nullptr, ENOENT);

	if (refMedia != prevRefMedia)
		PDRAW_LOGI("select %s as ref media", refMedia->getCName());

	for (const auto &m : mMedias)
		m->setReference(m.get() == refMedia);

	return 0;
}


int RecordDemuxer::processSelectedMedias()
{
	int ret;
	bool mediaListChanged = false;

	for (auto s : mSelectedMedias) {
		struct mp4_track_info tk = {};
		ret = mp4_demux_get_track_info(mDemux, s->idx, &tk);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_info", -ret);
			goto exit;
		}
		bool found = false;
		for (const auto &m : mMedias) {
			if (m->isTearingDown())
				continue;
			if (m->getTrackId() == tk.id) {
				PDRAW_LOGI("media '%s' is already set up",
					   s->name);
				found = true;
				break;
			}
		}
		if (!found) {
			std::unique_ptr<RecordDemuxer::DemuxerMedia> media;
			try {
				switch (tk.type) {
				case MP4_TRACK_TYPE_VIDEO:
					media = std::make_unique<
						RecordDemuxer::
							DemuxerCodedVideoMedia>(
						this);
					break;
				case MP4_TRACK_TYPE_METADATA:
					media = std::make_unique<
						RecordDemuxer::
							DemuxerRawVideoMedia>(
						this);
					break;
				case MP4_TRACK_TYPE_AUDIO:
					media = std::make_unique<
						RecordDemuxer::
							DemuxerAudioMedia>(
						this);
					break;
				default:
					ret = -EPROTO;
					PDRAW_LOGE("unsupported track type");
					goto exit;
				}
			} catch (const std::bad_alloc &) {
				ret = -ENOMEM;
				PDRAW_LOGE("DemuxerMedia allocation failed");
				goto exit;
			}
			ret = media->setup(&tk);
			if (ret != 0) {
				PDRAW_LOG_ERRNO("VideoMedia::setup", -ret);
				goto exit;
			}
			PDRAW_LOGI("media '%s' not set up, setting up",
				   s->name);
			mMedias.push_back(std::move(media));
			const auto &mediaPtr = mMedias.back();
			mediaListChanged = true;
			if (mRunning) {
				/* Play newly setup medias */
				mediaPtr->play();
				internalPlay(mSpeed);
			}
		}
	}
	for (const auto &m : mMedias) {
		if (m->isTearingDown())
			continue;
		bool found = false;
		for (auto s : mSelectedMedias) {
			if (s->media_id == (int)m->getTrackId()) {
				found = true;
				break;
			}
		}
		if (!found) {
			PDRAW_LOGI(
				"media '%s' not selected anymore, "
				"tear it down",
				m->getTrackName().c_str());
			m->sendDownstreamEvent(Channel::DownstreamEvent::EOS);
			m->setDestroyAfterFlush(true);
			m->flush();
			m->setTearingDown();
			mediaListChanged = true;
		}
	}

	ret = 0;

exit:
	if (mediaListChanged)
		selectReferenceTrack();
	return ret;
}


int RecordDemuxer::selectMedia(uint32_t selectedMedias)
{
	int ret = 0;

	if (!mCalledOpenResp) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	ret = Demuxer::selectMedia(selectedMedias);
	if (ret < 0)
		goto stop;

	processSelectedMedias();

	return 0;

stop:
	readyToPlay(false);
	flush();

	return ret;
}


RecordDemuxer::DemuxerMedia::DemuxerMedia(RecordDemuxer *demuxer) :
		mDemuxer(demuxer)
{
	mTimerHandler.set([this] { onTimer(); });

	std::string name = demuxer->getName() + "#DemuxerMedia";
	Loggable::setName(name);
}


RecordDemuxer::DemuxerMedia::~DemuxerMedia()
{
	mTimer.reset();
}


bool RecordDemuxer::DemuxerMedia::hasMedia(const Media *media) const
{
	return std::any_of(mMedias.begin(),
			   mMedias.end(),
			   [media](const std::unique_ptr<Media> &m) {
				   return m.get() == media;
			   });
}


int RecordDemuxer::DemuxerMedia::setup(const struct mp4_track_info *tkinfo)
{
	int ret;

	std::string name =
		mDemuxer->getName() + "#track#" + std::to_string(tkinfo->id);
	Loggable::setName(name);

	mTrackEnabled = tkinfo->enabled;
	mTrackId = tkinfo->id;
	if (tkinfo->name != nullptr)
		mTrackName = std::string(tkinfo->name);
	mTimescale = tkinfo->timescale;
	try {
		mMetadataBuffer.resize(1024);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("std::vector allocation failed", -ret);
		goto error;
	}

	/* Create the demux timer */
	try {
		mTimer = std::make_unique<pomp::Timer>(
			mDemuxer->mSession->getPompLoop(), &mTimerHandler);
	} catch (const std::bad_alloc &) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp::Timer", -ret);
		goto error;
	}

	ret = setupMedia(tkinfo);
	if (ret < 0)
		goto error;

	mMediaType = Media::Type::RAW_VIDEO;

	return 0;

error:
	mTimer.reset();
	return ret;
}


void RecordDemuxer::DemuxerMedia::play()
{
	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeekToPrevSample = false;
		mPendingSeekInPlay = true;
		mPendingSeekToNextSample = true;
		mPendingSeekExact = true;
		mPendingPlay = true;
		mTimer->set(1);
	} else {
		PDRAW_LOGW("%s: mPendingSeekExact not reset", __func__);
	}
}


void RecordDemuxer::DemuxerMedia::previous()
{
	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeek = true;
		mPendingSeekToPrevSample = true;
		mPendingSeekInPlay = false;
		mPendingSeekToNextSample = false;
		mPendingSeekExact = true;
		mTimer->set(1);
	} else {
		PDRAW_LOGW("%s: mPendingSeekExact not reset", __func__);
	}
}


void RecordDemuxer::DemuxerMedia::next()
{
	if (!mPendingSeekExact) {
		/* Avoid seeking back too much if a seek to a
		 * previous frame is already in progress */
		mPendingSeek = true;
		mPendingSeekToPrevSample = false;
		mPendingSeekInPlay = false;
		mPendingSeekToNextSample = true;
		mPendingSeekExact = true;
		mTimer->set(1);
	} else {
		PDRAW_LOGW("%s: mPendingSeekExact not reset", __func__);
	}
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
	mPendingSeek = true;
	mPendingSeekTs = (int64_t)timestamp;
	mPendingSeekExact = exact;
	mPendingSeekToPrevSample = false;
	mPendingSeekToNextSample = false;
	mTimer->set(1);
}


void RecordDemuxer::DemuxerMedia::completeSeek()
{
	mPendingSeek = false;
	mDemuxer->onMediaSeekComplete(mSeekResponse);
}


void RecordDemuxer::DemuxerMedia::completePlay()
{
	mPendingPlay = false;
	mPendingSeekInPlay = false;
	mDemuxer->onMediaPlayComplete(mPlayResponse);
}


void RecordDemuxer::DemuxerMedia::completeFlush()
{
	mFlushChannelCount--;
	if (mFlushChannelCount <= 0)
		mFlushing = false;
}


void RecordDemuxer::DemuxerMedia::flush(bool discard)
{
	int err;

	mDemuxer->Source::lock();

	if (mFlushing)
		return;

	mFlushing = true;
	mFlushDiscard = discard;
	mFlushChannelCount = 0;

	for (const auto &m : mMedias) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(m.get());
		mFlushChannelCount += outputChannelCount;

		/* Flush the output channels */
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *channel =
				mDemuxer->getOutputChannel(m.get(), i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			if (mFlushDiscard)
				err = channel->flush();
			else
				err = channel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO("channel->%s",
						-err,
						mFlushDiscard ? "flush"
							      : "drain");
			}
		}
	}

	if (mFlushChannelCount <= 0)
		mFlushing = false;

	mDemuxer->Source::unlock();
}


void RecordDemuxer::DemuxerMedia::sendDownstreamEvent(
	Channel::DownstreamEvent event)
{
	int err;

	for (const auto &m : mMedias) {
		err = mDemuxer->Source::sendDownstreamEvent(m.get(), event);
		if (err < 0)
			PDRAW_LOG_ERRNO("Source::sendDownstreamEvent", -err);
	}
}


void RecordDemuxer::DemuxerMedia::channelFlushed(
	[[maybe_unused]] const Channel *channel)
{

	completeFlush();
}


void RecordDemuxer::DemuxerMedia::channelDrained(
	[[maybe_unused]] const Channel *channel)
{

	completeFlush();
}


void RecordDemuxer::DemuxerMedia::channelUnlink(
	[[maybe_unused]] const Channel *channel)
{

	mDemuxer->Source::lock();

	for (const auto &m : mMedias) {
		unsigned int outputChannelCount =
			mDemuxer->getOutputChannelCount(m.get());
		if (outputChannelCount > 0) {
			mDemuxer->Source::unlock();
			return;
		}
	}

	mDemuxer->Source::unlock();

	if (mTearingDown)
		teardownMedia();
}


void RecordDemuxer::DemuxerMedia::stop()
{
	(void)mTimer->clear();

	setRunning(false);
	mDemuxer->onMediaRunningStateChanged();
	setTearingDown();

	mDemuxer->Source::lock();

	for (const auto &m : mMedias)
		m->setTearingDown();

	mDemuxer->Source::unlock();
}


void RecordDemuxer::DemuxerMedia::teardownMedia()
{
	int err;
	/* Remove the output ports */
	auto m = mMedias.begin();
	while (m != mMedias.end()) {
		if ((*m) == nullptr) {
			m++;
			continue;
		}
		if (mDemuxer->Source::mListener) {
			mDemuxer->Source::mListener->onOutputMediaRemoved(
				mDemuxer, m->get(), mDemuxer->getDemuxer());
		}
		/* Initiate teardown of any channels still attached (e.g.
		 * AudioDecoder or VideoDecoder when the session is destroyed
		 * without a prior close()). teardown() sends TEARDOWN to the
		 * sink asynchronously — it does NOT synchronously remove the
		 * channel, so removeOutputPort() may still return -EBUSY. */
		mDemuxer->teardownOutputChannels(m->get());
		err = mDemuxer->removeOutputPort(m->get());
		if (err < 0) {
			PDRAW_LOG_ERRNO("removeOutputPort", -err);
			if (err == -EBUSY) {
				/* Channels are in async teardown: the port
				 * cannot be removed now, but mMedias.clear()
				 * below will free the media object.
				 * 1. Null each attached Sink's InputPort::media
				 *    before the media is freed, so that
				 *    Sink::~Sink() → removeInputMediasImpl()
				 *    does not dereference freed memory.
				 * 2. Null the port's own media pointer so that
				 *    Source::~Source() → removeOutputPorts()
				 *    does not dereference freed memory.
				 * Order matters: clearAttachedSinksInputMedia()
				 * finds the port by media pointer, so it must
				 * run before clearOutputPortMedia() nulls it.
				 */
				mDemuxer->clearAttachedSinksInputMedia(
					m->get());
				mDemuxer->clearOutputPortMedia(m->get());
			}
			m++;
		} else {
			m = mMedias.erase(m);
		}
	}
	mMedias.clear();
}


void RecordDemuxer::DemuxerMedia::onTimer()
{
	int ret = 0;
	bool retry = false;
	bool silent = false;
	float speed = 1.0;
	int64_t error;
	int64_t duration;
	int64_t wait = 0;
	uint32_t waitMs = 0;
	bool didSeek = false;
	bool waitFlush = false;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	struct mp4_track_sample sample = {};

	RecordDemuxer *demuxer = mDemuxer;

	if (demuxer->mState != State::STARTED) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return;
	}

	speed = demuxer->mSpeed;

	if (!demuxer->mRunning) {
		mLastSampleDuration = 0;
		mLastOutputError = 0;
		return;
	}

	if (mTearingDown) {
		/* Media is tearing down, ignore frames */
		return;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);

	/* Seeking */
	if ((mPendingSeekTs >= 0) || mPendingSeekToPrevSample ||
	    mPendingSeekToNextSample) {
		if (isReference()) {
			if (mPendingSeekTs >= 0) {
				ret = mp4_demux_seek(
					demuxer->mDemux,
					(uint64_t)mPendingSeekTs,
					MP4_SEEK_METHOD_PREVIOUS_SYNC);
				if (ret < 0)
					PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
			} else if (mPendingSeekToPrevSample) {
				ret = mp4_demux_seek_to_track_prev_sample(
					demuxer->mDemux, mTrackId);
				if (ret < 0) {
					PDRAW_LOG_ERRNO(
						"mp4_demux_seek_to"
						"_track_prev_sample",
						-ret);
				}
			} else if (mPendingSeekToNextSample) {
				ret = mp4_demux_seek_to_track_next_sample(
					demuxer->mDemux, mTrackId, true);
				if (ret < 0) {
					PDRAW_LOG_ERRNO(
						"mp4_demux_seek_to"
						"_track_next_sample",
						-ret);
				}
			}
		}
		if (ret == 0) {
			mLastSampleDuration = 0;
			mLastOutputError = 0;
		}
		mSeekResponse = ret;
		/* If not error, seek to next sample only finishes when
		 * mPendingSeekExact goes back to false */
		if (ret != 0 || !mPendingSeekExact) {
			didSeek = true;
			mPendingSeekExact = false;
		}
	}

	demuxer->Source::lock();

	if (mSeekResponse == 0) {
		ret = processSample(
			&sample, &silent, &retry, &didSeek, &waitFlush);
		if (ret == -ENOENT) {
			didSeek = true;
			mPendingSeekExact = false;
		} else if ((demuxer->mPlaybackMode ==
			    PDRAW_PLAYBACK_MODE_OFFLINE) &&
			   (ret < 0) && waitFlush) {
			waitFlush = false;
			retry = true;
			goto out;
		} else if (ret < 0)
			goto out;
	}

	if (didSeek) {
		if (!mPendingSeekInPlay) {
			completeSeek();
		} else if (mPendingPlay) {
			mPlayResponse = mSeekResponse;
			completePlay();
		}
	}

	if (demuxer->mFrameByFrame &&
	    (!silent || (didSeek && (demuxer->mSeekResponse < 0)))) {
		setRunning(false);
		demuxer->onMediaRunningStateChanged();
	}

out:
#define PREV_SAMPLE_TIME_BEFORE mp4_demux_get_track_prev_sample_time_before
#define NEXT_SAMPLE_TIME_AFTER mp4_demux_get_track_next_sample_time_after
	if (waitFlush) {
		uint64_t nextSampleTime;
		/* Flush */
		demuxer->flush();
		/* Reset so this seek isn't replayed on the next pass (it
		 * would undo the NEXT_SYNC reposition below); mPendingSeekExact
		 * stays set so processSample() can still complete this seek. */
		mPendingSeekTs = -1;
		mPendingSeekToPrevSample = false;
		mPendingSeekToNextSample = false;
		/* Seek to next sync sample */
		ret = mp4_demux_get_track_next_sample_time(
			demuxer->mDemux, mTrackId, &nextSampleTime);
		if (ret != 0) {
			PDRAW_LOG_ERRNO("mp4_demux_get_track_next_sample_time",
					-ret);
			/* No later pass will run: resolve a pending seek now */
			if (mPendingSeek) {
				mSeekResponse = ret;
				mPendingSeekExact = false;
				completeSeek();
			}
			demuxer->Source::unlock();
			return;
		}
		ret = mp4_demux_seek(demuxer->mDemux,
				     nextSampleTime,
				     MP4_SEEK_METHOD_NEXT_SYNC);
		if (ret != 0)
			PDRAW_LOG_ERRNO("mp4_demux_seek", -ret);
		/* Re-arm (like "retry" below) when an exact seek is pending:
		 * completeFlush() -> play() won't restart it otherwise. */
		waitMs = mPendingSeekExact ? 5 : 0;
	} else if (retry) {
		waitMs = 5;
	} else if (demuxer->mRunning) {
		/* Schedule the next sample */
		uint64_t nextSampleDts =
			mp4_sample_time_to_usec(sample.next_dts, mTimescale);

		/* If error > 0 we are late, if error < 0 we are early */
		error = ((mLastSampleOutputTime == 0) ||
			 (mLastSampleDuration == 0) || (speed == 0.) ||
			 (speed >= PDRAW_PLAY_SPEED_MAX) || silent)
				? 0
				: curTime - mLastSampleOutputTime -
					  mLastSampleDuration +
					  mLastOutputError;
		if (mLastSampleOutputTime) {
			/* Average frame output rate
			 * (sliding average, alpha = 1/2) */
			mAvgOutputInterval +=
				((int64_t)(curTime - mLastSampleOutputTime) -
				 mAvgOutputInterval) >>
				1;
		}

		/* Sample duration */
		if ((speed >= PDRAW_PLAY_SPEED_MAX) || (nextSampleDts == 0) ||
		    silent) {
			duration = 0;
		} else if (speed < 0.) {
			/* Negative speed => play backward */
			nextSampleDts = mp4_sample_time_to_usec(
				sample.prev_sync_dts, mTimescale);
			uint64_t pendingSeekTs = nextSampleDts;
			uint64_t nextSyncSampleDts = nextSampleDts;
			uint64_t wantedSampleDts;
			duration =
				nextSampleDts -
				mp4_sample_time_to_usec(sample.dts, mTimescale);
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
					mTrackId,
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
					newDuration =
						nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts, mTimescale);
					if (speed != 0.) {
						newDuration = static_cast<
							int64_t>(
							static_cast<double>(
								newDuration) /
							static_cast<double>(
								speed));
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
			duration =
				nextSampleDts -
				mp4_sample_time_to_usec(sample.dts, mTimescale);
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
					mTrackId,
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
					newDuration =
						nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts, mTimescale);
					if (speed != 0.) {
						newDuration = static_cast<
							int64_t>(
							static_cast<double>(
								newDuration) /
							static_cast<double>(
								speed));
					}
				} else {
					break;
				}
			}
			if ((pendingSeekTs > 0) &&
			    (newDuration - error < 2 * mAvgOutputInterval)) {
				/* Only seek if the resulting wait time
				 * is less than twice the average frame
				 * output rate */
				PDRAW_LOGD(
					"unable to keep up with playback "
					"timings, seek forward %.2f ms",
					(float)(nextSyncSampleDts -
						mp4_sample_time_to_usec(
							sample.dts,
							mTimescale)) /
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
			wait = MIN(wait, UINT32_MAX - 500);
			waitMs = static_cast<uint32_t>(wait + 500) / 1000;
			if (waitMs == 0)
				waitMs = 1;
		} else if (demuxer->mRunning && !demuxer->mFrameByFrame) {
			sendDownstreamEvent(Channel::DownstreamEvent::EOS);
			mFirstSample = true;

			drain();
			setRunning(false);
			demuxer->onMediaRunningStateChanged();
			/* Notify of the end of range */
			/* TODO: signal once, not for all medias */
			demuxer->onEndOfRange(demuxer->mCurrentTime);
		}
		mLastSampleOutputTime = curTime;
		mLastSampleDuration = duration;
		mLastOutputError = error;

#if 0
		/* TODO: remove debug */
		PDRAW_LOGD("timerCb: error=%d duration=%d wait=%d%s",
			   (int)error,
			   (int)duration,
			   (int)wait,
			   (silent) ? " (silent)" : "");
#endif
	} else {
		mLastSampleOutputTime = curTime;
		mLastSampleDuration = 0;
		mLastOutputError = 0;
	}

	demuxer->Source::unlock();

	if (waitMs > 0) {
		if ((demuxer->mPlaybackMode == PDRAW_PLAYBACK_MODE_OFFLINE) &&
		    !retry)
			waitMs = 1;
		ret = mTimer->set(waitMs);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp::Timer::set", -ret);
	}
}

} /* namespace Pdraw */
