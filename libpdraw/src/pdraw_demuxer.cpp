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
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_demuxer.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_session.hpp"

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
		mReadyToPlay(false), mUnrecoverableError(false),
		mCalledOpenResp(false), mCallingSelectMedia(false),
		mMediaList(nullptr), mMediaListSize(0)
{
	mParams = *params;
}


Demuxer::~Demuxer(void)
{
	/* Make sure listener functions will no longer be called */
	mDemuxerListener = nullptr;

	clearMediaList();

	/* Remove any leftover idle callbacks */
	int err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
}


int Demuxer::getMediaList(struct pdraw_demuxer_media **mediaList,
			  size_t *mediaCount,
			  uint32_t *selectedMedias)
{
	struct pdraw_demuxer_media *_mediaList = NULL;

	if ((mediaList == nullptr) || (mediaCount == nullptr) ||
	    (selectedMedias == nullptr)) {
		return -EINVAL;
	}

	if ((mState != STARTING) && (mState != STARTED)) {
		PDRAW_LOGE("%s: demuxer is not started", __func__);
		return -EPROTO;
	}

	if (mCallingSelectMedia) {
		PDRAW_LOGE("%s: already selecting a media", __func__);
		return -EBUSY;
	}

	if (mMediaListSize == 0)
		return -ENOENT;

	_mediaList = (struct pdraw_demuxer_media *)calloc(mMediaListSize,
							  sizeof(*_mediaList));
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

	if ((mState != STARTING) && (mState != STARTED)) {
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
			for (auto m = mDefaultMedias.begin();
			     m != mDefaultMedias.end();
			     m++) {
				mSelectedMedias.push_back((*m));
				PDRAW_LOGI(" - %d (%s)",
					   (*m)->media_id,
					   (*m)->name);
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
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callOpenResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	mCalledOpenResp = true;
}


void Demuxer::closeResponse(int status)
{
	mCloseRespStatusArgs.push(status);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callCloseResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
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

	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callOnUnrecoverableError, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Demuxer::readyToPlay(bool ready)
{
	/* Report only changes in value */
	if (mReadyToPlay == ready)
		return;

	mReadyToPlay = ready;

	mReadyToPlayReadyArgs.push(ready);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callReadyToPlay, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Demuxer::onEndOfRange(uint64_t timestamp)
{
	mEndOfRangeTimestampArgs.push(timestamp);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callEndOfRange, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Demuxer::playResponse(int status, uint64_t timestamp, float speed)
{
	mPlayRespStatusArgs.push(status);
	mPlayRespTimestampArgs.push(timestamp);
	mPlayRespSpeedArgs.push(speed);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPlayResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Demuxer::pauseResponse(int status, uint64_t timestamp)
{
	mPauseRespStatusArgs.push(status);
	mPauseRespTimestampArgs.push(timestamp);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPauseResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Demuxer::seekResponse(int status, uint64_t timestamp, float speed)
{
	mSeekRespStatusArgs.push(status);
	mSeekRespTimestampArgs.push(timestamp);
	mSeekRespSpeedArgs.push(speed);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callSeekResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


int Demuxer::updateMediaList(
	struct pdraw_demuxer_media *newMediaList,
	size_t newMediaListSize,
	std::vector<struct pdraw_demuxer_media *> &newDefaultMedias,
	uint32_t *selectedMedias)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(newMediaList == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(newMediaListSize == 0, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(selectedMedias == nullptr, EINVAL);

	uint32_t bitfield = 0;
	for (auto it = mSelectedMedias.begin(); it != mSelectedMedias.end();
	     it++) {
		for (size_t i = 0; i < newMediaListSize; i++) {
			if (strcmp((*it)->name, newMediaList[i].name) != 0)
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


void Demuxer::clearMediaList(void)
{
	mSelectedMedias.clear();
	mDefaultMedias.clear();

	for (size_t i = 0; i < mMediaListSize; i++) {
		free((void *)mMediaList[i].name);
		free((void *)mMediaList[i].uri);
	}
	free(mMediaList);
	mMediaList = nullptr;
	mMediaListSize = 0;
}


uint32_t Demuxer::selectedMediasToBitfield(void)
{
	uint32_t bitfield = 0;
	for (auto it = mSelectedMedias.begin(); it != mSelectedMedias.end();
	     it++) {
		bitfield |= (1 << (*it)->media_id);
	}
	return bitfield;
}


/* Listener call from an idle function */
void Demuxer::callOpenResponse(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mOpenRespStatusArgs.front();
	self->mOpenRespStatusArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerOpenResponse(
		self->mSession, self->mDemuxer, status);
}


/* Listener call from an idle function */
void Demuxer::callCloseResponse(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mCloseRespStatusArgs.front();
	self->mCloseRespStatusArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerCloseResponse(
		self->mSession, self->mDemuxer, status);
}


/* Listener call from an idle function */
void Demuxer::callOnUnrecoverableError(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->onDemuxerUnrecoverableError(self->mSession,
							    self->mDemuxer);
}


/* Listener call from an idle function */
void Demuxer::callReadyToPlay(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	bool ready = self->mReadyToPlayReadyArgs.front();
	self->mReadyToPlayReadyArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerReadyToPlay(
		self->mSession, self->mDemuxer, ready);
}


/* Listener call from an idle function */
void Demuxer::callEndOfRange(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	uint64_t timestamp = self->mEndOfRangeTimestampArgs.front();
	self->mEndOfRangeTimestampArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->onDemuxerEndOfRange(
		self->mSession, self->mDemuxer, timestamp);
}


/* Listener call from an idle function */
void Demuxer::callPlayResponse(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mPlayRespStatusArgs.front();
	uint64_t timestamp = self->mPlayRespTimestampArgs.front();
	float speed = self->mPlayRespSpeedArgs.front();
	self->mPlayRespStatusArgs.pop();
	self->mPlayRespTimestampArgs.pop();
	self->mPlayRespSpeedArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerPlayResponse(
		self->mSession, self->mDemuxer, status, timestamp, speed);
}


/* Listener call from an idle function */
void Demuxer::callPauseResponse(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mPauseRespStatusArgs.front();
	uint64_t timestamp = self->mPauseRespTimestampArgs.front();
	self->mPauseRespStatusArgs.pop();
	self->mPauseRespTimestampArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerPauseResponse(
		self->mSession, self->mDemuxer, status, timestamp);
}


/* Listener call from an idle function */
void Demuxer::callSeekResponse(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mSeekRespStatusArgs.front();
	uint64_t timestamp = self->mSeekRespTimestampArgs.front();
	float speed = self->mSeekRespSpeedArgs.front();
	self->mSeekRespStatusArgs.pop();
	self->mSeekRespTimestampArgs.pop();
	self->mSeekRespSpeedArgs.pop();

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->demuxerSeekResponse(
		self->mSession, self->mDemuxer, status, timestamp, speed);
}


DemuxerWrapper::DemuxerWrapper(Session *session,
			       const std::string &url,
			       struct mux_ctx *mux,
			       const struct pdraw_demuxer_params *params,
			       IPdraw::IDemuxer::Listener *listener) :
		mDemuxer(nullptr)
{
	std::string ext;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if ((mux != nullptr) && (url.substr(0, 7) == "rtsp://")) {
#ifdef BUILD_LIBMUX
		mElement = mDemuxer = new StreamDemuxerMux(session,
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
	} else if (url.substr(0, 7) == "rtsp://") {
		mElement = mDemuxer = new StreamDemuxerNet(
			session, session, session, this, listener, url, params);
	} else if (ext == ".mp4") {
		mElement = mDemuxer = new RecordDemuxer(
			session, session, session, this, listener, url, params);
	} else {
		ULOGE("%s: unsupported URL ('%s')", __func__, url.c_str());
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
			       IPdraw::IDemuxer::Listener *listener) :
		mDemuxer(nullptr)
{
	mElement = mDemuxer = new StreamDemuxerNet(session,
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
}


DemuxerWrapper::~DemuxerWrapper(void)
{
	if (mDemuxer == nullptr)
		return;

	/* Clear the listener as it is not done by the Demuxer::stop function
	 * (to allow calling the closeResponse listener function) */
	mDemuxer->clearDemuxerListener();

	int res = mDemuxer->stop();
	if (res < 0)
		ULOG_ERRNO("Demuxer::stop", -res);
}


int DemuxerWrapper::close(void)
{
	int res;

	if (mDemuxer == nullptr)
		return -EPROTO;

	res = mDemuxer->stop();
	if (res < 0) {
		ULOG_ERRNO("Demuxer::stop", -res);
		return res;
	}

	/* Waiting for the asynchronous stop; closeResponse()
	 * will be called when it's done */
	mDemuxer = nullptr;
	return 0;
}


int DemuxerWrapper::getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getMediaList(mediaList, mediaCount, selectedMedias);
}


int DemuxerWrapper::selectMedia(uint32_t selectedMedias)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->selectMedia(selectedMedias);
}


uint16_t DemuxerWrapper::getSingleStreamLocalStreamPort(void)
{
	if (mDemuxer == nullptr)
		return 0;

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalStreamPort();
}


uint16_t DemuxerWrapper::getSingleStreamLocalControlPort(void)
{
	if (mDemuxer == nullptr)
		return 0;

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalControlPort();
}


bool DemuxerWrapper::isReadyToPlay(void)
{
	if (mDemuxer == nullptr)
		return false;

	return mDemuxer->isReadyToPlay();
}


bool DemuxerWrapper::isPaused(void)
{
	if (mDemuxer == nullptr)
		return false;

	return mDemuxer->isPaused();
}


int DemuxerWrapper::play(float speed)
{
	if (mDemuxer == nullptr)
		return -EPROTO;

	return mDemuxer->play(speed);
}


int DemuxerWrapper::pause(void)
{
	return play(0.);
}


int DemuxerWrapper::previousFrame(void)
{
	if (mDemuxer == nullptr)
		return -EPROTO;
	return mDemuxer->previous();
}


int DemuxerWrapper::nextFrame(void)
{
	if (mDemuxer == nullptr)
		return -EPROTO;
	return mDemuxer->next();
}


int DemuxerWrapper::seek(int64_t delta, bool exact)
{
	if (mDemuxer == nullptr)
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
	if (mDemuxer == nullptr)
		return -EPROTO;

	return mDemuxer->seekTo(timestamp, exact);
}


int DemuxerWrapper::getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getChapterList(chapterList, chapterCount);
}


uint64_t DemuxerWrapper::getDuration(void)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getDuration();
}


uint64_t DemuxerWrapper::getCurrentTime(void)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getCurrentTime();
}

} /* namespace Pdraw */
