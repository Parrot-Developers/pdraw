/**
 * Parrot Drones Audio and Video Vector library
 * Generic muxer
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

#define ULOG_TAG pdraw_muxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer.hpp"
#include "pdraw_muxer_record_dng.hpp"
#include "pdraw_muxer_record_isobmff.hpp"
#include "pdraw_muxer_record_jfif.hpp"
#include "pdraw_muxer_record_png.hpp"
#include "pdraw_muxer_stream_rtmp.hpp"
#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <h264/h264.h>
#include <h265/h265.h>

namespace Pdraw {


Muxer::Muxer(Session *session,
	     Element::Listener *elementListener,
	     IPdraw::IMuxer::Listener *listener,
	     MuxerWrapper *wrapper,
	     const struct pdraw_muxer_params *params) :
		SinkElement(session,
			    elementListener,
			    wrapper,
			    UINT_MAX,
			    nullptr,
			    0,
			    nullptr,
			    0,
			    nullptr,
			    0),
		mMuxer(wrapper), mMuxerListener(listener), mParams(*params)
{
	Element::setClassName(__func__);

	setState(State::CREATED);
}


Muxer::~Muxer()
{
	int err;

	/* Make sure listener functions will no longer be called */
	mMuxerListener = nullptr;

	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if ((mState == State::STARTED) || (mState == State::STARTING))
		PDRAW_LOGW("%s: still running (%s)",
			   __func__,
			   Element::getElementStateStr(mState));

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("%s: not all input media have been removed",
			   __func__);
		(void)removeInputMedias();
	}
}


int Muxer::start()
{
	int res;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(State::STARTING);

	mReadyToStart = true;

	res = internalStart();
	if (res < 0)
		goto error;

	if (mReadyToStart)
		setState(State::STARTED);

	return 0;

error:
	/* Do not call stop() here to avoid replying with muxerCloseResponse on
	 * internally called stop */
	setState(State::STOPPING);

	(void)completeStop();
	return res;
}


int Muxer::stop()
{
	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if ((mState != State::STARTED) && (mState != State::STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(State::STOPPING);

	mClosing = true;

	/* Note: the muxer listener is not cleared here to allow calling
	 * the IMuxer::Listener::muxerCloseResponse listener function when the
	 * IMuxer::close function was called; clearing the listener when
	 * deleting the API object is done by calling
	 * Muxer::clearMuxerListener in the API object destructor prior
	 * to calling Muxer::stop */

	if (mFlushing)
		return 0;

	return completeStop();
}


int Muxer::completeStop()
{
	int res;

	if (mState == State::STOPPED)
		return 0;
	if (mState != State::STOPPING) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	mReadyToStop = true;

	res = internalStop();
	if (res < 0)
		return res;

	if (!mReadyToStop)
		return 0;

	res = removeInputMedias();
	if (res < 0)
		return res;

	if (!mClosing) {
		setState(State::STOPPED);
	} else {
		closeResponse(0);
		setStateAsyncNotify(State::STOPPED);
	}

	return 0;
}


void Muxer::completeFlush(const Channel *channel, bool discard)
{
	int err;
	int inputMediaCount;
	int i;
	bool flushPending = false;

	PDRAW_LOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (media == nullptr)
			continue;
		Channel *_channel = getInputChannel(media);
		if (_channel != channel)
			continue;
		if (discard)
			err = _channel->flushDone();
		else
			err = _channel->drainDone();
		if (err < 0) {
			PDRAW_LOG_ERRNO("channel->%s",
					-err,
					discard ? "flushDone" : "drainDone");
		}
		break;
	}

	inputMediaCount = getInputMediaCount();
	for (i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (media == nullptr)
			continue;
		const Channel *_channel = getInputChannel(media);
		if (_channel == nullptr)
			continue;
		flushPending |= (_channel->isFlushPending() ||
				 _channel->isDrainPending());
	}

	mFlushing = flushPending;

	Sink::unlock();

	if ((!mFlushing) && (mState == State::STOPPING))
		(void)completeStop();
}


struct asyncCompleteFlushParams {
	Muxer *muxer;
	Channel *channel;
	bool discard;
};


int Muxer::asyncCompleteFlush(Channel *channel, bool discard)
{
	int ret;
	struct asyncCompleteFlushParams *params = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(channel == nullptr, EINVAL);

	params = static_cast<struct asyncCompleteFlushParams *>(
		calloc(1, sizeof(*params)));
	if (params == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		return ret;
	}
	params->muxer = this;
	params->channel = channel;
	params->discard = discard;

	ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleCompleteFlush, params, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		return ret;
	}
	return 0;
}


int Muxer::asyncCompleteStop()
{
	int ret;

	ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleCompleteStop, this, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		return ret;
	}
	return 0;
}


void Muxer::idleCompleteFlush(void *userdata)
{
	Muxer *self = nullptr;
	struct asyncCompleteFlushParams *params = nullptr;

	params = static_cast<struct asyncCompleteFlushParams *>(userdata);
	ULOG_ERRNO_RETURN_IF(params == nullptr, EPROTO);

	self = params->muxer;
	PDRAW_LOG_ERRNO_RETURN_IF(params->muxer == nullptr, EPROTO);
	PDRAW_LOG_ERRNO_RETURN_IF(params->channel == nullptr, EPROTO);

	params->muxer->completeFlush(params->channel, params->discard);

	free(params);
}


void Muxer::idleCompleteStop(void *userdata)
{
	auto *self = static_cast<Muxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EPROTO);

	self->completeStop();
}


int Muxer::createInputQueue(Media::Type type, mbuf::Queue **queue)
{
	mbuf::Queue::Type qtype;
	switch (type) {
	case Media::Type::CODED_VIDEO:
		qtype = mbuf::Queue::Type::CODED_VIDEO;
		break;
	case Media::Type::RAW_VIDEO:
		qtype = mbuf::Queue::Type::RAW_VIDEO;
		break;
	case Media::Type::AUDIO:
		qtype = mbuf::Queue::Type::AUDIO;
		break;
	default:
		return -EINVAL;
	}

	try {
		auto q = mbuf::Queue::create(qtype);
		*queue = q.release();
	} catch (const std::bad_alloc &) {
		ULOGE("queue allocation failed");
		return -ENOMEM;
	}

	return 0;
}


int Muxer::addInputMedia(Media *media,
			 const struct pdraw_muxer_media_params *params)
{
	int res;
	mbuf::Queue *queue = nullptr;

	Sink::lock();

	res = Sink::addInputMedia(media);
	if (res < 0) {
		if (res != -EEXIST)
			PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		goto out_unlock;
	}

	{
		Channel *channel = getInputChannel(media);
		if (!channel) {
			PDRAW_LOGE("No channel found for media %p", media);
			res = -ENODEV;
			goto error_remove;
		}

		res = createInputQueue(media->type, &queue);
		if (res < 0) {
			PDRAW_LOG_ERRNO("createInputQueue", -res);
			goto error_remove;
		}

		channel->setQueue(this, queue);
	}

	res = queue->attachToLoop(mSession->getLoop(), &queueEventCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("queue::attachToLoop", -res);
		goto error_remove;
	}

	Sink::unlock();
	return 0;

error_remove:
	removeInputMedia(media);

out_unlock:
	Sink::unlock();
	return res;
}


int Muxer::removeInputMedia(Media *media)
{
	int res;

	Sink::lock();

	Channel *channel = getInputChannel(media);
	if (!channel) {
		Sink::unlock();
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}

	mbuf::Queue *queue = channel->getQueue(this);

	res = Sink::removeInputMedia(media);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);
		return res;
	}
	media = nullptr;

	if (queue != nullptr) {
		queue->detachFromLoop(mSession->getLoop());
		queue->flush();
		delete queue;
	}

	Sink::unlock();
	return 0;
}


int Muxer::removeInputMedias()
{
	int inputMediaCount;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (int i = inputMediaCount - 1; i >= 0; i--) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		int err = removeInputMedia(media);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -err);
	}

	Sink::unlock();

	return 0;
}


/* Called on the loop thread */
void Muxer::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	PDRAW_UNUSED(evt);

	auto *self = static_cast<Muxer *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	(void)self->process();
}


/* Can be called from any thread */
void Muxer::onChannelFlush(Channel *channel)
{
	int ret;
	int inputMediaCount;
	const Media *foundMedia = nullptr;

	PDRAW_LOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (int i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (media != nullptr && getInputChannel(media) == channel) {
			foundMedia = media;
			break;
		}
	}

	if (!foundMedia) {
		Sink::unlock();
		PDRAW_LOGE("channel not found (%p)", channel);
		return;
	}

	mbuf::Queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}

	queue->flush();

	Sink::unlock();

	mFlushing = true;

	if (mAsyncFlush)
		return;

	ret = asyncCompleteFlush(channel, true);
	if (ret < 0)
		PDRAW_LOG_ERRNO("asyncCompleteFlush", -ret);
}


/* Can be called from any thread */
void Muxer::onChannelDrain(Channel *channel)
{
	int ret;
	const Media *foundMedia = nullptr;

	PDRAW_LOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Sink::lock();

	unsigned int inputMediaCount = getInputMediaCount();
	for (unsigned int i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (media != nullptr && getInputChannel(media) == channel) {
			foundMedia = media;
			break;
		}
	}

	if (!foundMedia) {
		Sink::unlock();
		PDRAW_LOGE("channel not found (%p)", channel);
		return;
	}

	mbuf::Queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}

	int count = queue->getCount();
	if (count < 0) {
		PDRAW_LOG_ERRNO("getQueueCount", -count);
	} else if (count > 0) {
		PDRAW_LOGW("%s: %d frames still in queue", __func__, count);
	}

	queue->flush();

	Sink::unlock();

	mFlushing = true;

	if (mAsyncFlush)
		return;

	ret = asyncCompleteFlush(channel, false);
	if (ret < 0)
		PDRAW_LOG_ERRNO("asyncCompleteFlush", -ret);
}


void Muxer::onChannelTeardown(Channel *channel)
{
	Sink::onChannelTeardown(channel);

	if (!mInputPorts.empty())
		return;

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


int Muxer::addMedia(unsigned int mediaId,
		    const struct pdraw_muxer_media_params *params)
{
	return mSession->addMediaToMuxer(mediaId, this, params);
}


int Muxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			const uint8_t *data,
			size_t size)
{
	return -ENOSYS;
}


int Muxer::addChapter(uint64_t timestamp, const char *name)
{
	return -ENOSYS;
}


int Muxer::setFileMetadata(enum pdraw_muxer_metadata_type type,
			   const uint8_t *data,
			   size_t size,
			   const void *params,
			   size_t paramsSize)
{
	return -ENOSYS;
}


int Muxer::getStats(struct pdraw_muxer_stats *stats)
{
	return -ENOSYS;
}


int Muxer::setDynParams(const struct pdraw_muxer_dyn_params *dyn_params)
{
	return -ENOSYS;
}


int Muxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	return -ENOSYS;
}


int Muxer::forceSync()
{
	return -ENOSYS;
}


void Muxer::closeResponse(int status)
{
	mCloseRespStatusArgs.push(status);
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callCloseResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Muxer::onConnectionStateChanged(
	enum pdraw_muxer_connection_state state,
	enum pdraw_muxer_disconnection_reason reason)
{
	mConnectionStateChangedStateArgs.push(state);
	mConnectionStateChangedReasonArgs.push(reason);

	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callOnConnectionStateChanged, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


/* Can be called from any thread */
void Muxer::onUnrecoverableError(int error)
{
	/* Report only the first error */
	bool expected = false;
	if (!std::atomic_compare_exchange_strong(
		    &mUnrecoverableError, &expected, true))
		return;

	mUnrecoverableErrorStatusArgs.push(error);

	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callOnUnrecoverableError, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


/* Listener call from an idle function */
void Muxer::callCloseResponse(void *userdata)
{
	auto *self = static_cast<Muxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mCloseRespStatusArgs.front();
	self->mCloseRespStatusArgs.pop();

	if (self->mMuxerListener == nullptr)
		return;

	self->mMuxerListener->muxerCloseResponse(
		self->mSession, self->mMuxer, status);
}


/* Listener call from an idle function */
void Muxer::callOnConnectionStateChanged(void *userdata)
{
	auto *self = static_cast<Muxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	enum pdraw_muxer_connection_state state =
		self->mConnectionStateChangedStateArgs.front();
	self->mConnectionStateChangedStateArgs.pop();
	enum pdraw_muxer_disconnection_reason reason =
		self->mConnectionStateChangedReasonArgs.front();
	self->mConnectionStateChangedReasonArgs.pop();

	if (self->mMuxerListener == nullptr)
		return;

	self->mMuxerListener->onMuxerConnectionStateChanged(
		self->mSession, self->mMuxer, state, reason);
}


/* Listener call from an idle function */
void Muxer::callOnUnrecoverableError(void *userdata)
{
	auto *self = static_cast<Muxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	int status = self->mUnrecoverableErrorStatusArgs.front();
	self->mUnrecoverableErrorStatusArgs.pop();

	if (self->mMuxerListener == nullptr)
		return;

	self->mMuxerListener->onMuxerUnrecoverableError(
		self->mSession, self->mMuxer, status);
}


MuxerWrapper::MuxerWrapper(Session *session,
			   const std::string &url,
			   const struct pdraw_muxer_params *params,
			   IPdraw::IMuxer::Listener *listener)
{
	std::string ext;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if ((url.substr(0, 7) == "rtmp://") ||
	    (url.substr(0, 8) == "rtmps://")) {
#ifdef BUILD_LIBRTMP
		mElement = mMuxer = new Pdraw::RtmpStreamMuxer(
			session, session, listener, this, url, params);
#else
		ULOGE("%s: librtmp is not supported", __func__);
#endif
	} else if ((url.substr(0, 7) == "rtsp://") ||
		   (url.substr(0, 8) == "rtsps://")) {
		mElement = mMuxer = new Pdraw::RtspStreamMuxer(
			session, session, listener, this, url, params);
	} else if (ext == ".mp4" || ext == ".tmp") {
		mElement = mMuxer = new Pdraw::IsobmffRecordMuxer(
			session, session, listener, this, url, params);
	} else if (ext == ".jpg" || ext == ".tpg") {
#ifdef BUILD_LIBJFIF
		mElement = mMuxer = new Pdraw::JfifRecordMuxer(
			session, session, listener, this, url, params);
#else
		ULOGE("%s: libjfif is not supported", __func__);
#endif
	} else if (ext == ".dng" || ext == ".tdn") {
#ifdef BUILD_LIBDNG_PARROT
		mElement = mMuxer = new Pdraw::DngRecordMuxer(
			session, session, listener, this, url, params);
#else
		ULOGE("%s: libdng-parrot is not supported", __func__);
#endif
	} else if (ext == ".png" || ext == ".tpn") {
		mElement = mMuxer = new Pdraw::PngRecordMuxer(
			session, session, listener, this, url, params);
	} else {
		ULOGE("%s: unsupported URL ('%s')", __func__, url.c_str());
	}
}


MuxerWrapper::~MuxerWrapper()
{
	if (mMuxer == nullptr)
		return;

	/* Clear the listener as it is not done by the Muxer::stop function
	 * (to allow calling the closeResponse listener function) */
	mMuxer->clearMuxerListener();

	if (mElementStopped)
		return;

	int res = mMuxer->stop();
	if (res < 0)
		ULOG_ERRNO("Muxer::stop", -res);
	mElementStopped = true;
}


int MuxerWrapper::close()
{
	int res;

	if (isElementStopped())
		return -EPROTO;

	res = mMuxer->stop();
	if (res < 0) {
		ULOG_ERRNO("mMuxer::stop", -res);
		return res;
	}

	/* Waiting for the asynchronous stop; closeResponse()
	 * will be called when it's done */
	mElementStopped = true;
	return 0;
}


int MuxerWrapper::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			       const uint8_t *data,
			       size_t size)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->setThumbnail(type, data, size);
}


int MuxerWrapper::addChapter(uint64_t timestamp, const char *name)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->addChapter(timestamp, name);
}


int MuxerWrapper::setFileMetadata(enum pdraw_muxer_metadata_type type,
				  const uint8_t *data,
				  size_t size,
				  const void *params,
				  size_t paramsSize)
{
	if (isElementStopped())
		return -EPROTO;
	return mMuxer->setFileMetadata(type, data, size, params, paramsSize);
}


int MuxerWrapper::getStats(struct pdraw_muxer_stats *stats)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->getStats(stats);
}


int MuxerWrapper::setDynParams(const struct pdraw_muxer_dyn_params *dyn_params)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->setDynParams(dyn_params);
}


int MuxerWrapper::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->getDynParams(dyn_params);
}


int MuxerWrapper::forceSync()
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->forceSync();
}


int MuxerWrapper::addMedia(unsigned int mediaId,
			   const struct pdraw_muxer_media_params *params)
{
	if (isElementStopped())
		return -EPROTO;

	return mMuxer->addMedia(mediaId, params);
}

} /* namespace Pdraw */
