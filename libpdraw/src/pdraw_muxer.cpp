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

#include "pdraw_muxer.hpp"
#include "pdraw_muxer_record_dng.hpp"
#include "pdraw_muxer_record_isobmff.hpp"
#include "pdraw_muxer_record_jfif.hpp"
#include "pdraw_muxer_record_png.hpp"
#include "pdraw_muxer_stream_rtmp.hpp"
#include "pdraw_muxer_stream_rtsp.hpp"
#include "pdraw_muxer_stream_rtsp_mux.hpp"
#include "pdraw_muxer_stream_rtsp_net.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <h264/h264.h>
#include <h265/h265.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


template <typename F> static void doRemoveInputMedias(Muxer *self, F fn)
{
	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	self->Sink::lock();
	for (int i = (int)self->getInputMediaCount() - 1; i >= 0; i--) {
		Media *media = self->getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		int err = fn(media);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -err);
	}
	self->Sink::unlock();
}


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

	mCompleteFlushHandler.set([this] { callCompleteFlush(); });
	mCompleteStopHandler.set([this] { idleCompleteStop(); });
	mCloseResponseHandler.set([this] { callCloseResponse(); });
	mOnConnectionStateChangedHandler.set(
		[this] { callOnConnectionStateChanged(); });
	mOnUnrecoverableErrorHandler.set(
		[this] { callOnUnrecoverableError(); });

	setState(State::CREATED);
}


Muxer::~Muxer()
{
	int err;

	/* Make sure listener functions will no longer be called */
	mMuxerListener = nullptr;

	err = mSession->getPompLoop()->idleRemove(this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleRemove", -err);

	if ((mState == State::STARTED) || (mState == State::STARTING))
		PDRAW_LOGW("%s: still running (%s)",
			   __func__,
			   Element::getElementStateStr(mState));

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("%s: not all input media have been removed",
			   __func__);
		/* Explicit non-virtual call: derived class is already destroyed
		 */
		doRemoveInputMedias(this, [this](Media *m) {
			return Muxer::removeInputMedia(m);
		});
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


int Muxer::asyncCompleteFlush(Channel *channel, bool discard)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(channel == nullptr, EINVAL);

	{
		std::scoped_lock lock(mCompleteFlushMutex);
		mCompleteFlushArgs.push({channel, discard});
	}

	int ret =
		mSession->getPompLoop()->idleAdd(&mCompleteFlushHandler, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -ret);
		return ret;
	}
	return 0;
}


void Muxer::callCompleteFlush()
{
	completeFlushArgs args;
	{
		std::scoped_lock lock(mCompleteFlushMutex);
		args = mCompleteFlushArgs.front();
		mCompleteFlushArgs.pop();
	}
	completeFlush(args.channel, args.discard);
}


int Muxer::asyncCompleteStop()
{
	int ret;

	ret = mSession->getPompLoop()->idleAdd(&mCompleteStopHandler, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -ret);
		return ret;
	}
	return 0;
}


void Muxer::idleCompleteStop()
{
	completeStop();
}


int Muxer::createInputQueue(Media::Type type,
			    std::unique_ptr<mbuf::Queue> &queue)
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
		queue = mbuf::Queue::create(qtype);
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
	std::unique_ptr<mbuf::Queue> queue;

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

		res = createInputQueue(media->getType(), queue);
		if (res < 0) {
			PDRAW_LOG_ERRNO("createInputQueue", -res);
			goto error_remove;
		}

		channel->setQueue(this, queue.get());
		mInputQueues[media] = std::move(queue);
	}

	res = mInputQueues[media]->attachToLoop(
		mSession->getLoop(), &queueEventCb, this);
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

	const Channel *channel = getInputChannel(media);
	if (!channel) {
		Sink::unlock();
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}

	res = Sink::removeInputMedia(media);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);
		return res;
	}

	auto it = mInputQueues.find(media);
	if (it != mInputQueues.end()) {
		it->second->detachFromLoop(mSession->getLoop());
		it->second->flush();
		mInputQueues.erase(it);
	}

	Sink::unlock();
	return 0;
}


int Muxer::removeInputMedias()
{
	doRemoveInputMedias(this,
			    [this](Media *m) { return removeInputMedia(m); });
	return 0;
}


/* Called on the loop thread */
void Muxer::queueEventCb([[maybe_unused]] struct pomp_evt *evt, void *userdata)
{

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


int Muxer::setFileMetadata(const struct pdraw_muxer_metadata_params *params,
			   const uint8_t *data,
			   size_t size)
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
	try {
		mCloseRespStatusArgs.push(status);
	} catch (const std::bad_alloc &e) {
		PDRAW_LOGE("%s: failed to push status: %s", __func__, e.what());
		return;
	}

	int err =
		mSession->getPompLoop()->idleAdd(&mCloseResponseHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


void Muxer::onConnectionStateChanged(
	enum pdraw_muxer_connection_state state,
	enum pdraw_muxer_disconnection_reason reason)
{
	mConnectionStateChangedStateArgs.push(state);
	mConnectionStateChangedReasonArgs.push(reason);

	int err = mSession->getPompLoop()->idleAdd(
		&mOnConnectionStateChangedHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
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
	mUnrecoverableErrorStatus = error;

	int err = mSession->getPompLoop()->idleAdd(
		&mOnUnrecoverableErrorHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp::Loop::idleAdd", -err);
}


/* Listener call from an idle function */
void Muxer::callCloseResponse()
{
	int status = mCloseRespStatusArgs.front();
	mCloseRespStatusArgs.pop();

	if (mMuxerListener == nullptr)
		return;

	mMuxerListener->muxerCloseResponse(mSession, mMuxer, status);
}


/* Listener call from an idle function */
void Muxer::callOnConnectionStateChanged()
{
	enum pdraw_muxer_connection_state state =
		mConnectionStateChangedStateArgs.front();
	mConnectionStateChangedStateArgs.pop();
	enum pdraw_muxer_disconnection_reason reason =
		mConnectionStateChangedReasonArgs.front();
	mConnectionStateChangedReasonArgs.pop();

	if (mMuxerListener == nullptr)
		return;

	mMuxerListener->onMuxerConnectionStateChanged(
		mSession, mMuxer, state, reason);
}


/* Listener call from an idle function */
void Muxer::callOnUnrecoverableError()
{
	int status = mUnrecoverableErrorStatusArgs.front();
	mUnrecoverableErrorStatusArgs.pop();

	if (mMuxerListener == nullptr)
		return;

	mMuxerListener->onMuxerUnrecoverableError(mSession, mMuxer, status);
}


MuxerWrapper::MuxerWrapper(Session *session,
			   const std::string &url,
			   struct mux_ctx *mux,
			   [[maybe_unused]] const std::string &remoteHost,
			   const struct pdraw_muxer_params *params,
			   IPdraw::IMuxer::Listener *listener)
{
	std::string ext;
	std::unique_ptr<Muxer> impl;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	try {
		if ((mux != nullptr) && ((url.substr(0, 7) == "rtsp://") ||
					 (url.substr(0, 8) == "rtsps://"))) {
#ifdef BUILD_LIBMUX
			impl = std::make_unique<RtspStreamMuxerMux>(session,
								    session,
								    listener,
								    this,
								    url,
								    mux,
								    remoteHost,
								    params);
#else /* BUILD_LIBMUX */
			ULOGE("%s: libmux is not supported", __func__);
#endif /* BUILD_LIBMUX */
		} else if ((url.substr(0, 7) == "rtmp://") ||
			   (url.substr(0, 8) == "rtmps://")) {
#ifdef BUILD_LIBRTMP
			impl = std::make_unique<RtmpStreamMuxer>(
				session, session, listener, this, url, params);
#else
			ULOGE("%s: librtmp is not supported", __func__);
#endif
		} else if ((url.substr(0, 7) == "rtsp://") ||
			   (url.substr(0, 8) == "rtsps://")) {
			impl = std::make_unique<RtspStreamMuxerNet>(
				session, session, listener, this, url, params);
		} else if (ext == ".mp4" || ext == ".m4a" || ext == ".tmp") {
			impl = std::make_unique<IsobmffRecordMuxer>(
				session, session, listener, this, url, params);
		} else if (ext == ".jpg" || ext == ".tpg") {
#ifdef BUILD_LIBJFIF
			impl = std::make_unique<JfifRecordMuxer>(
				session, session, listener, this, url, params);
#else
			ULOGE("%s: libjfif is not supported", __func__);
#endif
		} else if (ext == ".dng" || ext == ".tdn") {
#ifdef BUILD_LIBDNG_PARROT
			impl = std::make_unique<DngRecordMuxer>(
				session, session, listener, this, url, params);
#else
			ULOGE("%s: libdng-parrot is not supported", __func__);
#endif
		} else if (ext == ".png" || ext == ".tpn") {
			impl = std::make_unique<PngRecordMuxer>(
				session, session, listener, this, url, params);
		} else {
			ULOGE("%s: unsupported URL ('%s')",
			      __func__,
			      url.c_str());
		}
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to allocate muxer", __func__);
	}

	/* Ownership is transferred to Session::mElements immediately after
	 * construction via unique_ptr<Element>(wrapper->getElement()) */
	if (impl) {
		mMuxer = impl.get();
		mElement = impl.release();
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

	try {
		int res = mMuxer->stop();
		if (res < 0)
			ULOG_ERRNO("Muxer::stop", -res);
	} catch (const std::exception &e) {
		ULOGW("~MuxerWrapper: exception caught in stop: %s", e.what());
	}
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


int MuxerWrapper::setFileMetadata(
	const struct pdraw_muxer_metadata_params *params,
	const uint8_t *data,
	size_t size)
{
	if (isElementStopped())
		return -EPROTO;
	return mMuxer->setFileMetadata(params, data, size);
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
