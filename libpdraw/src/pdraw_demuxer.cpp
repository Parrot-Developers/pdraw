/**
 * Parrot Drones Audio and Video Vector library
 * Generic demuxer
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

#define ULOG_TAG pdraw_demuxer
#include <ulog.h>

#include "pdraw_demuxer.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


Demuxer::Demuxer(Session *session,
		 Element::Listener *elementListener,
		 Source::Listener *sourceListener,
		 DemuxerWrapper *wrapper,
		 IPdraw::IDemuxer::Listener *demuxerListener,
		 const struct pdraw_demuxer_params *params) :
		SourceElement(session,
			      elementListener,
			      wrapper,
			      UINT_MAX,
			      sourceListener),
		mDemuxer(wrapper), mDemuxerListener(demuxerListener),
		mParams(*params)
{
	mWatchdogTimerHandler.set([this] { onWatchdogTimer(); });
	mCallOpenResponseHandler.set([this] { callOpenResponse(); });
	mCallCloseResponseHandler.set([this] { callCloseResponse(); });
	mCallOnUnrecoverableErrorHandler.set(
		[this] { callOnUnrecoverableError(); });
	mCallReadyToPlayHandler.set([this] { callReadyToPlay(); });
	mCallEndOfRangeHandler.set([this] { callEndOfRange(); });
	mCallPlayResponseHandler.set([this] { callPlayResponse(); });
	mCallPauseResponseHandler.set([this] { callPauseResponse(); });
	mCallSeekResponseHandler.set([this] { callSeekResponse(); });
}


Demuxer::~Demuxer()
{
	int err;

	/* Make sure listener functions will no longer be called */
	mDemuxerListener = nullptr;

	clearMediaList();

	/* Remove any leftover idle callbacks */
	err = mSession->getPompLoop()->idleRemove(this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	mWatchdogTimer.reset();
}


int Demuxer::getMediaList(struct pdraw_demuxer_media **mediaList,
			  size_t *mediaCount,
			  uint32_t *selectedMedias)
{
	struct pdraw_demuxer_media *_mediaList = nullptr;

	if ((mediaList == nullptr) || (mediaCount == nullptr) ||
	    (selectedMedias == nullptr)) {
		return -EINVAL;
	}

	if ((mState != State::STARTING) && (mState != State::STARTED)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (mCallingSelectMedia) {
		PDRAW_LOGE("%s: already selecting a media", __func__);
		return -EBUSY;
	}

	if (mMediaListSize == 0)
		return -ENOENT;

	_mediaList = static_cast<struct pdraw_demuxer_media *>(
		calloc(mMediaListSize, sizeof(*_mediaList)));
	if (_mediaList == nullptr)
		return -ENOMEM;

	/* Media list deep copy */
	for (size_t i = 0; i < mMediaListSize; i++) {
		_mediaList[i] = mMediaList[i];
		_mediaList[i].name = xstrdup(mMediaList[i].name);
		_mediaList[i].uri = xstrdup(mMediaList[i].uri);
	}

	*mediaList = _mediaList;
	*mediaCount = mMediaListSize;
	*selectedMedias = selectedMediasToBitfield();
	return 0;
}


int Demuxer::selectMedia(uint32_t selectedMedias)
{
	int ret;

	if ((mState != State::STARTING) && (mState != State::STARTED)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (mCallingSelectMedia) {
		PDRAW_LOGE("%s: already selecting a media", __func__);
		return -EBUSY;
	}

	if (mMediaListSize == 0) {
		ret = -EPROTO;
		goto error;
	}

	mSelectedMedias.clear();

	if (selectedMedias == 0) {
		if (mDefaultMedias.empty()) {
			PDRAW_LOGE(
				"application requested default media, "
				"but no default media found");
			ret = -ENOENT;
			goto error;
		}
		if (mDefaultMedias.size() == 1) {
			mSelectedMedias.push_back(mDefaultMedias.back());
			PDRAW_LOGI("auto-selecting media %d (%s)",
				   mSelectedMedias.back()->media_id,
				   mSelectedMedias.back()->name);
		} else {
			PDRAW_LOGI("auto-selecting medias {");
			for (auto m : mDefaultMedias) {
				mSelectedMedias.push_back(m);
				PDRAW_LOGI(" - %d (%s)", m->media_id, m->name);
			}
			PDRAW_LOGI("}");
		}
	} else {
		for (size_t i = 0; i < mMediaListSize; i++) {
			if (!(selectedMedias & (1 << mMediaList[i].media_id)))
				continue;
			mSelectedMedias.push_back(&mMediaList[i]);
			PDRAW_LOGI("application selected media %d (%s)",
				   mSelectedMedias.back()->media_id,
				   mSelectedMedias.back()->name);
		}
		if (mSelectedMedias.empty()) {
			PDRAW_LOGE("the application requested no valid media");
			ret = -ENOENT;
			goto error;
		}
	}

	return 0;

error:
	return ret;
}


int Demuxer::callSelectMedia(uint32_t selectedMedias)
{
	int ret;

	if (mDemuxerListener == nullptr)
		return -ENOSYS;

	mCallingSelectMedia = true;
	ret = mDemuxerListener->demuxerSelectMedia(
		mSession, mDemuxer, mMediaList, mMediaListSize, selectedMedias);
	mCallingSelectMedia = false;

	return ret;
}


int Demuxer::getChapterList(struct pdraw_chapter **chapterList,
			    size_t *chapterCount)
{
	return -ENOSYS;
}


void Demuxer::openResponse(int status)
{
	if (mCalledOpenResp) {
		PDRAW_LOGW("multiple openResponse call blocked");
		return;
	}
	mOpenRespStatusArgs.push(status);
	int err = mSession->getPompLoop()->idleAdd(&mCallOpenResponseHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
	mCalledOpenResp = true;
}


void Demuxer::closeResponse(int status)
{
	mCloseRespStatusArgs.push(status);
	int err = mSession->getPompLoop()->idleAdd(&mCallCloseResponseHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::onUnrecoverableError(int error)
{
	/* If openResponse was not yet called, call it instead */
	if (!mCalledOpenResp) {
		openResponse(error);
		return;
	}
	/* Report only the first error */
	if (mUnrecoverableError)
		return;

	mUnrecoverableError = true;

	int err = mSession->getPompLoop()->idleAdd(
		&mCallOnUnrecoverableErrorHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::readyToPlay(bool ready)
{
	/* Report only changes in value */
	if (mReadyToPlay == ready)
		return;

	mReadyToPlay = ready;

	mReadyToPlayReadyArgs.push(ready);
	int err = mSession->getPompLoop()->idleAdd(&mCallReadyToPlayHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::onEndOfRange(uint64_t timestamp)
{
	mEndOfRangeTimestampArgs.push(timestamp);
	int err =
		mSession->getPompLoop()->idleAdd(&mCallEndOfRangeHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::playResponse(int status, uint64_t timestamp, float speed)
{
	mPlayRespStatusArgs.push(status);
	mPlayRespTimestampArgs.push(timestamp);
	mPlayRespSpeedArgs.push(speed);
	int err = mSession->getPompLoop()->idleAdd(&mCallPlayResponseHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::pauseResponse(int status, uint64_t timestamp)
{
	mPauseRespStatusArgs.push(status);
	mPauseRespTimestampArgs.push(timestamp);
	int err = mSession->getPompLoop()->idleAdd(&mCallPauseResponseHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Demuxer::seekResponse(int status, uint64_t timestamp, float speed)
{
	mSeekRespStatusArgs.push(status);
	mSeekRespTimestampArgs.push(timestamp);
	mSeekRespSpeedArgs.push(speed);
	int err = mSession->getPompLoop()->idleAdd(&mCallSeekResponseHandler,
						   this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


int Demuxer::updateMediaList(
	struct pdraw_demuxer_media *newMediaList,
	size_t newMediaListSize,
	const std::vector<struct pdraw_demuxer_media *> &newDefaultMedias,
	uint32_t *selectedMedias)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(newMediaList == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(newMediaListSize == 0, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(selectedMedias == nullptr, EINVAL);

	uint32_t bitfield = 0;
	for (auto m : mSelectedMedias) {
		for (size_t i = 0; i < newMediaListSize; i++) {
			if (strcmp(m->name, newMediaList[i].name) != 0)
				continue;
			/* Selected media is in the new list */
			bitfield |= (1 << newMediaList[i].media_id);
		}
	}

	clearMediaList();

	mMediaList = newMediaList;
	mMediaListSize = newMediaListSize;
	mDefaultMedias = newDefaultMedias;
	mSelectedMedias.clear();
	*selectedMedias = bitfield;

	return 0;
}


void Demuxer::clearMediaList()
{
	mSelectedMedias.clear();
	mDefaultMedias.clear();

	pdraw_demuxerMediaListFree(mMediaList, mMediaListSize);
	mMediaList = nullptr;
	mMediaListSize = 0;
}


uint32_t Demuxer::selectedMediasToBitfield() const
{
	uint32_t bitfield = 0;
	for (auto m : mSelectedMedias)
		bitfield |= (1 << m->media_id);
	return bitfield;
}


const char *Demuxer::getCommandStr(Demuxer::Command cmd)
{
	switch (cmd) {
	case Command::NONE:
		return "NONE";
	case Command::PLAY:
		return "PLAY";
	case Command::PAUSE:
		return "PAUSE";
	case Command::PAUSE_NEXT:
		return "PAUSE_NEXT";
	case Command::SEEK:
		return "SEEK";
	default:
		return "UNKNOWN";
	}
}


void Demuxer::onWatchdogTimer()
{
	PDRAW_LOGE("pending operation (%s) timed out",
		   getCommandStr(mPendingCmd));

	switch (mPendingCmd) {
	case Command::PLAY:
		playResponse(-ETIMEDOUT, 0, 0);
		break;
	case Command::PAUSE:
	case Command::PAUSE_NEXT:
		pauseResponse(-ETIMEDOUT, 0);
		break;
	case Command::SEEK:
		seekResponse(-ETIMEDOUT, 0, 0);
		break;
	default:
		PDRAW_LOGW("unsupported operation (%s)",
			   getCommandStr(mPendingCmd));
		clearPendingCommand();
		break;
	}
}


int Demuxer::setPendingCommand(Demuxer::Command cmd)
{
	int ret;

	if (mPendingCmd != Command::NONE) {
		PDRAW_LOGE("%s: another operation (%s) is pending",
			   __func__,
			   getCommandStr(mPendingCmd));
		return -EBUSY;
	}

	if (!mWatchdogTimer) {
		try {
			mWatchdogTimer = std::make_unique<pomp::Timer>(
				mSession->getPompLoop(),
				&mWatchdogTimerHandler);
		} catch (const std::bad_alloc &) {
			ret = -ENOMEM;
			PDRAW_LOGE("pomp::Timer allocation failed");
			goto error;
		}
	}

	ret = mWatchdogTimer->set(DEMUXER_PENDING_COMMAND_TIMEOUT_MS);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp::Timer::set", -ret);
		goto error;
	}

	mPendingCmd = cmd;

	return 0;

error:
	return ret;
}


void Demuxer::clearPendingCommand()
{
	int err = mWatchdogTimer->clear();
	if (err < 0) {
		PDRAW_LOG_ERRNO("pomp::Timer::set", -err);
	}
	mPendingCmd = Command::NONE;
}


/* Listener call from an idle function */
void Demuxer::callOpenResponse()
{
	int status = mOpenRespStatusArgs.front();
	mOpenRespStatusArgs.pop();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerOpenResponse(mSession, mDemuxer, status);
}


/* Listener call from an idle function */
void Demuxer::callCloseResponse()
{
	int status = mCloseRespStatusArgs.front();
	mCloseRespStatusArgs.pop();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerCloseResponse(mSession, mDemuxer, status);
}


/* Listener call from an idle function */
void Demuxer::callOnUnrecoverableError()
{
	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->onDemuxerUnrecoverableError(mSession, mDemuxer);
}


/* Listener call from an idle function */
void Demuxer::callReadyToPlay()
{
	bool ready = mReadyToPlayReadyArgs.front();
	mReadyToPlayReadyArgs.pop();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerReadyToPlay(mSession, mDemuxer, ready);
}


/* Listener call from an idle function */
void Demuxer::callEndOfRange()
{
	uint64_t timestamp = mEndOfRangeTimestampArgs.front();
	mEndOfRangeTimestampArgs.pop();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->onDemuxerEndOfRange(mSession, mDemuxer, timestamp);
}


/* Listener call from an idle function */
void Demuxer::callPlayResponse()
{
	int status = mPlayRespStatusArgs.front();
	uint64_t timestamp = mPlayRespTimestampArgs.front();
	float speed = mPlayRespSpeedArgs.front();
	mPlayRespStatusArgs.pop();
	mPlayRespTimestampArgs.pop();
	mPlayRespSpeedArgs.pop();

	if (mPendingCmd == Command::NONE) {
		PDRAW_LOGE("%s: no pending command", __func__);
	} else if (mPendingCmd != Command::PLAY) {
		PDRAW_LOGE("%s: unexpected pending command (%s)",
			   __func__,
			   getCommandStr(mPendingCmd));
	}
	clearPendingCommand();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerPlayResponse(
		mSession, mDemuxer, status, timestamp, speed);
}


/* Listener call from an idle function */
void Demuxer::callPauseResponse()
{
	int status = mPauseRespStatusArgs.front();
	uint64_t timestamp = mPauseRespTimestampArgs.front();
	mPauseRespStatusArgs.pop();
	mPauseRespTimestampArgs.pop();

	if (mPendingCmd == Command::NONE) {
		PDRAW_LOGE("%s: no pending command", __func__);
	} else if ((mPendingCmd != Command::PAUSE) &&
		   (mPendingCmd != Command::PAUSE_NEXT)) {
		PDRAW_LOGE("%s: unexpected pending command (%s)",
			   __func__,
			   getCommandStr(mPendingCmd));
	}
	clearPendingCommand();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerPauseResponse(
		mSession, mDemuxer, status, timestamp);
}


/* Listener call from an idle function */
void Demuxer::callSeekResponse()
{
	int status = mSeekRespStatusArgs.front();
	uint64_t timestamp = mSeekRespTimestampArgs.front();
	float speed = mSeekRespSpeedArgs.front();
	mSeekRespStatusArgs.pop();
	mSeekRespTimestampArgs.pop();
	mSeekRespSpeedArgs.pop();

	if (mPendingCmd == Command::NONE) {
		PDRAW_LOGE("%s: no pending command", __func__);
	} else if (mPendingCmd != Command::SEEK) {
		PDRAW_LOGE("%s: unexpected pending command (%s)",
			   __func__,
			   getCommandStr(mPendingCmd));
	}
	clearPendingCommand();

	if (mDemuxerListener == nullptr)
		return;

	mDemuxerListener->demuxerSeekResponse(
		mSession, mDemuxer, status, timestamp, speed);
}


DemuxerWrapper::DemuxerWrapper(Session *session,
			       const std::string &url,
			       struct mux_ctx *mux,
			       const struct pdraw_demuxer_params *params,
			       IPdraw::IDemuxer::Listener *listener)
{
	std::string ext;
	std::unique_ptr<Demuxer> impl;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	try {
		if ((mux != nullptr) && (url.substr(0, 7) == "rtsp://")) {
#ifdef BUILD_LIBMUX
			impl = std::make_unique<StreamDemuxerMux>(session,
								  session,
								  session,
								  this,
								  listener,
								  url,
								  mux,
								  params);
#else /* BUILD_LIBMUX */
			ULOGE("%s: libmux is not supported", __func__);
#endif /* BUILD_LIBMUX */
		} else if ((url.substr(0, 7) == "rtsp://") ||
			   (url.substr(0, 8) == "rtsps://")) {
			impl = std::make_unique<StreamDemuxerNet>(session,
								  session,
								  session,
								  this,
								  listener,
								  url,
								  params);
		} else if (ext == ".mp4" || ext == ".m4a") {
			impl = std::make_unique<RecordDemuxer>(session,
							       session,
							       session,
							       this,
							       listener,
							       url,
							       params);
		} else {
			ULOGE("%s: unsupported URL ('%s')",
			      __func__,
			      url.c_str());
		}
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to allocate demuxer", __func__);
	}

	/* Ownership is transferred to Session::mElements immediately after
	 * construction via unique_ptr<Element>(wrapper->getElement()) */
	if (impl) {
		mDemuxer = impl.get();
		mElement = impl.release();
	}
}


DemuxerWrapper::DemuxerWrapper(Session *session,
			       const std::string &localAddr,
			       uint16_t localStreamPort,
			       uint16_t localControlPort,
			       const std::string &remoteAddr,
			       uint16_t remoteStreamPort,
			       uint16_t remoteControlPort,
			       const struct pdraw_demuxer_params *params,
			       IPdraw::IDemuxer::Listener *listener)
{
	try {
		auto impl =
			std::make_unique<StreamDemuxerNet>(session,
							   session,
							   session,
							   this,
							   listener,
							   localAddr,
							   localStreamPort,
							   localControlPort,
							   remoteAddr,
							   remoteStreamPort,
							   remoteControlPort,
							   params);
		/* Ownership is transferred to Session::mElements immediately
		 * after construction via unique_ptr<Element>(wrapper->
		 * getElement()) */
		mDemuxer = impl.get();
		mElement = impl.release();
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to allocate demuxer", __func__);
	}
}


DemuxerWrapper::~DemuxerWrapper()
{
	if (mDemuxer == nullptr)
		return;

	/* Clear the listener as it is not done by the Demuxer::stop function
	 * (to allow calling the closeResponse listener function) */
	mDemuxer->clearDemuxerListener();

	if (mElementStopped)
		return;

	int res = mDemuxer->stop();
	if (res < 0)
		ULOG_ERRNO("Demuxer::stop", -res);
	mElementStopped = true;
}


int DemuxerWrapper::close()
{
	int res;

	if (isElementStopped())
		return -EPROTO;

	res = mDemuxer->stop();
	if (res < 0) {
		ULOG_ERRNO("Demuxer::stop", -res);
		return res;
	}

	/* Waiting for the asynchronous stop; closeResponse()
	 * will be called when it's done */
	mElementStopped = true;
	return 0;
}


int DemuxerWrapper::getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias)
{
	if (isElementStopped())
		return -EPROTO;

	return mDemuxer->getMediaList(mediaList, mediaCount, selectedMedias);
}


int DemuxerWrapper::selectMedia(uint32_t selectedMedias)
{
	if (isElementStopped())
		return -EPROTO;

	return mDemuxer->selectMedia(selectedMedias);
}


uint16_t DemuxerWrapper::getSingleStreamLocalStreamPort()
{
	if (isElementStopped())
		return 0;

	auto *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalStreamPort();
}


uint16_t DemuxerWrapper::getSingleStreamLocalControlPort()
{
	if (isElementStopped())
		return 0;

	auto *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalControlPort();
}


bool DemuxerWrapper::isReadyToPlay()
{
	if (isElementStopped())
		return false;

	return mDemuxer->isReadyToPlay();
}


bool DemuxerWrapper::isPaused()
{
	if (isElementStopped())
		return false;

	return mDemuxer->isPaused();
}


int DemuxerWrapper::play(float speed)
{
	if (isElementStopped())
		return -EPROTO;

	return mDemuxer->play(speed);
}


int DemuxerWrapper::pause()
{
	return play(0.);
}


int DemuxerWrapper::previousFrame()
{
	if (isElementStopped())
		return -EPROTO;
	return mDemuxer->previous();
}


int DemuxerWrapper::nextFrame()
{
	if (isElementStopped())
		return -EPROTO;
	return mDemuxer->next();
}


int DemuxerWrapper::seek(int64_t delta, bool exact)
{
	if (isElementStopped())
		return -EPROTO;
	return mDemuxer->seek(delta, exact);
}


int DemuxerWrapper::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta);
}


int DemuxerWrapper::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta));
}


int DemuxerWrapper::seekTo(uint64_t timestamp, bool exact)
{
	if (isElementStopped())
		return -EPROTO;
	return mDemuxer->seekTo(timestamp, exact);
}


int DemuxerWrapper::getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount)
{
	if (isElementStopped())
		return -EPROTO;

	return mDemuxer->getChapterList(chapterList, chapterCount);
}


uint64_t DemuxerWrapper::getDuration()
{
	if (isElementStopped())
		return 0;

	return mDemuxer->getDuration();
}


uint64_t DemuxerWrapper::getCurrentTime()
{
	if (isElementStopped())
		return 0;

	return mDemuxer->getCurrentTime();
}

} /* namespace Pdraw */
