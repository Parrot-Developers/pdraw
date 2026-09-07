/**
 * Parrot Drones Audio and Video Vector
 * PDrAW back-end library
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

#include <errno.h>
#include <limits.h>
#include <pthread.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

#define ULOG_TAG pdraw_backend
#include <ulog.h>

#include "pdraw_backend_impl.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);


namespace PdrawBackend {


constexpr size_t MAX_STR_LEN = 200;
#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


constexpr const char *RAW_VIDEO_SOURCE = "raw video source";
constexpr const char *RAW_VIDEO_SINK = "raw video sink";
constexpr const char *CODED_VIDEO_SOURCE = "coded video source";
constexpr const char *CODED_VIDEO_SINK = "coded video sink";
constexpr const char *DEMUXER = "demuxer";
constexpr const char *MUXER = "muxer";
constexpr const char *VIDEO_RENDERER = "video renderer";
constexpr const char *AUDIO_RENDERER = "audio renderer";
constexpr const char *VIPC_SOURCE = "VIPC source";
constexpr const char *ALSA_SOURCE = "ALSA source";
constexpr const char *AUDIO_SOURCE = "audio source";
constexpr const char *AUDIO_SINK = "audio sink";
constexpr const char *VIDEO_ENCODER = "video encoder";
constexpr const char *VIDEO_SCALER = "video scaler";
constexpr const char *AUDIO_ENCODER = "audio encoder";


template <typename T>
void PdrawBackend::deleteElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { self->resetElement(); };
	backend->runOnLoop(func);
}


template <typename T>
bool PdrawBackend::isElementReadyToPlay(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(backend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!backend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!self->hasElement(), EPROTO, false);

	auto func = [&self]() { return self->getElement()->isReadyToPlay(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::playElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->play(); };
	return backend->runOnLoop(func);
}


template <typename T>
bool PdrawBackend::isElementPaused(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(backend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!backend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!self->hasElement(), EPROTO, false);

	auto func = [&self]() { return self->getElement()->isPaused(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::pauseElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->pause(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::closeElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->close(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::flushElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->flush(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::elementQueueFlushed(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->queueFlushed(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::drainElement(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->drain(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::elementQueueDrained(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self]() { return self->getElement()->queueDrained(); };
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::setElementSessionMetadata(T *self,
					    PdrawBackend *backend,
					    const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self, &meta]() {
		return self->getElement()->setSessionMetadata(meta);
	};
	return backend->runOnLoop(func);
}


template <typename T>
int PdrawBackend::getElementSessionMetadata(T *self,
					    PdrawBackend *backend,
					    struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!backend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!self->hasElement(), EPROTO);

	auto func = [&self, meta]() {
		return self->getElement()->getSessionMetadata(meta);
	};
	return backend->runOnLoop(func);
}


template <typename T>
unsigned int PdrawBackend::getElementMediaId(T *self, PdrawBackend *backend)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(backend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!backend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!self->hasElement(), EPROTO, 0);

	auto func = [&self]() { return self->getElement()->getMediaId(); };
	return backend->runOnLoop(func);
}


/**
 * Public functions
 */

int createPdrawBackend(IPdraw::Listener *listener, IPdrawBackend **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);

	try {
		auto backend = std::make_unique<PdrawBackend>(listener);
		/* Ownership transferred to caller via public API raw pointer */
		*retObj = backend.release();
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
	return 0;
}


PdrawBackend::PdrawBackend(IPdraw::Listener *listener) : mListener(listener) {}


PdrawBackend::~PdrawBackend()
{
	int err;

	if (mStarted)
		ULOGW("destroying pdraw backend while still running");

	mThreadShouldStop = true;

	{
		std::scoped_lock lock(mMutex);
		if (mLoop) {
			err = mLoop->wakeup();
			if (err < 0)
				ULOG_ERRNO("pomp::Loop::wakeup", -err);
		}
	}

	if (mLoopThreadLaunched && mLoopThread.joinable()) {
		mLoopThread.join();
		mLoopThreadLaunched = false;
	}
	mPdraw.reset();
	mStarted = false;
}


int PdrawBackend::start()
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mListener == nullptr, EPROTO);

	std::unique_lock lock(mMutex);
	mRetValReady = false;

	try {
		mLoopThread = std::thread(&PdrawBackend::loopThread, this);
	} catch (const std::system_error &e) {
		ULOG_ERRNO("std::thread", e.code().value());
		return -e.code().value();
	}

	mLoopThreadLaunched = true;

	mCond.wait(lock, [this] { return mRetValReady; });

	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;
	mStarted = true;

	return res;
}


int PdrawBackend::stop()
{
	auto func = [this]() { return mPdraw->stop(); };

	if (!mStarted)
		return 0;

	int res = runOnLoop(func);

	{
		std::scoped_lock lock(mApiMutex);
		mStarted = false;
	}

	return res;
}


struct pomp_loop *PdrawBackend::getLoop()
{
	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, nullptr);

	return mLoop->get();
}


int PdrawBackend::createDemuxer(const std::string &url,
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, &url, &params, &listener, &retObj]() {
		return doCreateDemuxer(url, params, listener, retObj);
	};
	return runOnLoop(func);
}


int PdrawBackend::createDemuxer(const std::string &localAddr,
				uint16_t localStreamPort,
				uint16_t localControlPort,
				const std::string &remoteAddr,
				uint16_t remoteStreamPort,
				uint16_t remoteControlPort,
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(localAddr.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(remoteAddr.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this,
		     &localAddr,
		     localStreamPort,
		     localControlPort,
		     &remoteAddr,
		     remoteStreamPort,
		     remoteControlPort,
		     &params,
		     &listener,
		     &retObj]() {
		return doCreateDemuxer(localAddr,
				       localStreamPort,
				       localControlPort,
				       remoteAddr,
				       remoteStreamPort,
				       remoteControlPort,
				       params,
				       listener,
				       retObj);
	};
	return runOnLoop(func);
}


int PdrawBackend::createDemuxer(const std::string &url,
				struct mux_ctx *mux,
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, &url, &mux, &params, &listener, &retObj]() {
		return doCreateDemuxer(url, mux, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::Demuxer::~Demuxer()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::Demuxer::close()
{
	return closeElement(this, mBackend);
}


int PdrawBackend::Demuxer::getMediaList(struct pdraw_demuxer_media **mediaList,
					size_t *mediaCount,
					uint32_t *selectedMedias)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, &mediaList, &mediaCount, &selectedMedias]() {
		return getElement()->getMediaList(
			mediaList, mediaCount, selectedMedias);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::selectMedia(uint32_t selectedMedias)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, selectedMedias]() {
		return getElement()->selectMedia(selectedMedias);
	};
	return mBackend->runOnLoop(func);
}


uint16_t PdrawBackend::Demuxer::getSingleStreamLocalStreamPort()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, 0);

	auto func = [this]() {
		return getElement()->getSingleStreamLocalStreamPort();
	};
	return mBackend->runOnLoop(func);
}


uint16_t PdrawBackend::Demuxer::getSingleStreamLocalControlPort()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, 0);

	auto func = [this]() {
		return getElement()->getSingleStreamLocalControlPort();
	};
	return mBackend->runOnLoop(func);
}


bool PdrawBackend::Demuxer::isReadyToPlay()
{
	return isElementReadyToPlay(this, mBackend);
}


bool PdrawBackend::Demuxer::isPaused()
{
	return isElementPaused(this, mBackend);
}


int PdrawBackend::Demuxer::play(float speed)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, speed]() { return getElement()->play(speed); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::pause()
{
	return play(0.);
}


int PdrawBackend::Demuxer::previousFrame()
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this]() { return getElement()->previousFrame(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::nextFrame()
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this]() { return getElement()->nextFrame(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::seek(int64_t delta, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);


	auto func = [this, delta, exact]() {
		return getElement()->seek(delta, exact);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta, exact);
}


int PdrawBackend::Demuxer::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta), exact);
}


int PdrawBackend::Demuxer::seekTo(uint64_t timestamp, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, timestamp, exact]() {
		return getElement()->seekTo(timestamp, exact);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Demuxer::getChapterList(struct pdraw_chapter **chapterList,
					  size_t *chapterCount)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, chapterList, chapterCount]() {
		return getElement()->getChapterList(chapterList, chapterCount);
	};
	return mBackend->runOnLoop(func);
}


uint64_t PdrawBackend::Demuxer::getDuration()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, 0);

	auto func = [this]() { return getElement()->getDuration(); };
	return mBackend->runOnLoop(func);
}


uint64_t PdrawBackend::Demuxer::getCurrentTime()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, 0);

	auto func = [this]() { return getElement()->getCurrentTime(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::createMuxer(const std::string &url,
			      const struct pdraw_muxer_params *params,
			      IPdraw::IMuxer::Listener *listener,
			      IPdraw::IMuxer **retObj)
{
	return createMuxer(url, nullptr, {}, params, listener, retObj);
}


int PdrawBackend::createMuxer(const std::string &url,
			      struct mux_ctx *mux,
			      const std::string &remoteHost,
			      const struct pdraw_muxer_params *params,
			      IPdraw::IMuxer::Listener *listener,
			      IPdraw::IMuxer **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(mux != nullptr && remoteHost.empty(), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func =
		[this, &url, &mux, &remoteHost, params, listener, retObj]() {
			return doCreateMuxer(
				url, mux, remoteHost, params, listener, retObj);
		};
	return runOnLoop(func);
}


PdrawBackend::Muxer::~Muxer()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::Muxer::close()
{
	return closeElement(this, mBackend);
}


int PdrawBackend::Muxer::addMedia(unsigned int mediaId,
				  const struct pdraw_muxer_media_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, mediaId, params]() {
		return getElement()->addMedia(mediaId, params);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Muxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
				      const uint8_t *data,
				      size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	auto func = [this, type, data, size]() {
		return getElement()->setThumbnail(type, data, size);
	};
	return mBackend->runOnLoop(func);
};


int PdrawBackend::Muxer::setFileMetadata(
	const struct pdraw_muxer_metadata_params *params,
	const uint8_t *data,
	size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	auto func = [this, params, data, size]() {
		return getElement()->setFileMetadata(params, data, size);
	};
	return mBackend->runOnLoop(func);
};


int PdrawBackend::Muxer::addChapter(uint64_t timestamp, const char *name)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(timestamp == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(name == nullptr, EINVAL);

	auto func = [this, timestamp, name]() {
		return getElement()->addChapter(timestamp, name);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Muxer::getStats(struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	auto func = [this, stats]() { return getElement()->getStats(stats); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Muxer::setDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	auto func = [this, dyn_params]() {
		return getElement()->setDynParams(dyn_params);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Muxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	auto func = [this, dyn_params]() {
		return getElement()->getDynParams(dyn_params);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::Muxer::forceSync()
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this]() { return getElement()->forceSync(); };
	return mBackend->runOnLoop(func);
}


/* Called on the rendering thread */
int PdrawBackend::createVideoRenderer(
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	IPdraw::IVideoRenderer **retObj)
{
	int res = 0;
	IPdraw::IVideoRenderer *rnd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	std::scoped_lock apiLock(mApiMutex);

	{
		std::unique_ptr<PdrawBackend::VideoRenderer> renderer;
		try {
			renderer =
				std::make_unique<PdrawBackend::VideoRenderer>(
					this, renderPos, params, listener);
		} catch (const std::bad_alloc &) {
			res = -ENOMEM;
			goto out;
		}

		mPendingVideoRendererAndListener = {
			.e = renderer.get(),
			.l = listener,
		};

		res = mPdraw->createVideoRenderer(
			mediaId, renderPos, params, this, &rnd);
		if (res < 0) {
			ULOG_ERRNO("pdraw->createVideoRenderer", -res);
			/* renderer destroyed automatically */
			goto out;
		}
		renderer->setElement(rnd);

		{
			std::scoped_lock lock(mMapsMutex);

			auto [it, inserted] =
				mVideoRendererListenersMap.try_emplace(
					renderer->getElement(),
					mPendingVideoRendererAndListener);
			if (!inserted) {
				ULOGW("failed to insert the video renderer "
				      "listener in the map");
			}
		}

		/* Ownership transferred to caller via public API raw pointer */
		*retObj = renderer.release();
	}

out:
	mPendingVideoRendererAndListener = {};
	if (res < 0)
		*retObj = nullptr;

	return res;
}


/* Called on the rendering thread */
PdrawBackend::VideoRenderer::~VideoRenderer()
{
	size_t erased;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);

	if (!hasElement())
		return;

	std::scoped_lock apiLock(mBackend->mApiMutex);

	{
		std::scoped_lock lock(mBackend->mMapsMutex);
		erased = mBackend->mVideoRendererListenersMap.erase(
			getElement());
		if (erased != 1) {
			ULOGW("failed to erase the video renderer listener "
			      "from the map");
		}
	}

	resetElement();
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::resize(const struct pdraw_rect *renderPos)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->resize(renderPos);
	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::setMediaId(unsigned int mediaId)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->setMediaId(mediaId);
	return res;
}


/* Called on the rendering thread */
unsigned int PdrawBackend::VideoRenderer::getMediaId()
{
	unsigned int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->getMediaId();
	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::setParams(
	const struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->setParams(params);
	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::getParams(
	struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->getParams(params);
	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::render(struct pdraw_rect *contentPos,
					const float *viewMat,
					const float *projMat)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	std::scoped_lock lock(mBackend->mApiMutex);
	res = getElement()->render(contentPos, viewMat, projMat);
	return res;
}


int PdrawBackend::createVipcSource(
	const struct pdraw_vipc_source_params *params,
	IPdraw::IVipcSource::Listener *listener,
	IPdraw::IVipcSource **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(strlen(params->address) > MAX_STR_LEN,
				 ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		(params->friendly_name != nullptr) &&
			(strlen(params->friendly_name) > MAX_STR_LEN),
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		(params->backend_name != nullptr) &&
			(strlen(params->backend_name) > MAX_STR_LEN),
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);


	auto func = [this, params, listener, retObj]() {
		return doCreateVipcSource(params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::VipcSource::~VipcSource()
{
	deleteElement(this, mBackend);
}


bool PdrawBackend::VipcSource::isReadyToPlay()
{
	return isElementReadyToPlay(this, mBackend);
}


bool PdrawBackend::VipcSource::isPaused()
{
	return isElementPaused(this, mBackend);
}


int PdrawBackend::VipcSource::play()
{
	return playElement(this, mBackend);
}


int PdrawBackend::VipcSource::pause()
{
	return pauseElement(this, mBackend);
}


int PdrawBackend::VipcSource::insertGreyFrame(uint64_t tsUs)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, tsUs]() {
		return getElement()->insertGreyFrame(tsUs);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::VipcSource::configure(const struct vdef_dim *resolution,
					const struct vdef_rectf *crop)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, resolution, crop]() {
		return getElement()->configure(resolution, crop);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::VipcSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	return setElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::VipcSource::getSessionMetadata(struct vmeta_session *meta)
{
	return getElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::createCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, params, listener, retObj]() {
		return doCreateCodedVideoSource(params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::CodedVideoSource::~CodedVideoSource()
{
	deleteElement(this, mBackend);
}


struct mbuf_coded_video_frame_queue *PdrawBackend::CodedVideoSource::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::CodedVideoSource::flush()
{
	return flushElement(this, mBackend);
}


int PdrawBackend::CodedVideoSource::drain()
{
	return drainElement(this, mBackend);
}


int PdrawBackend::CodedVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	return setElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::CodedVideoSource::getSessionMetadata(
	struct vmeta_session *meta)
{
	return getElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::createRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, params, listener, retObj]() {
		return doCreateRawVideoSource(params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::RawVideoSource::~RawVideoSource()
{
	deleteElement(this, mBackend);
}


struct mbuf_raw_video_frame_queue *PdrawBackend::RawVideoSource::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::RawVideoSource::flush()
{
	return flushElement(this, mBackend);
}


int PdrawBackend::RawVideoSource::drain()
{
	return drainElement(this, mBackend);
}


int PdrawBackend::RawVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	return setElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::RawVideoSource::getSessionMetadata(struct vmeta_session *meta)
{
	return getElementSessionMetadata(this, mBackend, meta);
}


int PdrawBackend::createCodedVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateCodedVideoSink(
			mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::CodedVideoSink::~CodedVideoSink()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::CodedVideoSink::setMediaId(unsigned int mediaId)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, &mediaId]() {
		return getElement()->setMediaId(mediaId);
	};
	return mBackend->runOnLoop(func);
}


unsigned int PdrawBackend::CodedVideoSink::getMediaId()
{
	return getElementMediaId(this, mBackend);
}


int PdrawBackend::CodedVideoSink::resync()
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this]() { return getElement()->resync(); };
	return mBackend->runOnLoop(func);
}


struct mbuf_coded_video_frame_queue *PdrawBackend::CodedVideoSink::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::CodedVideoSink::queueFlushed()
{
	return elementQueueFlushed(this, mBackend);
}


int PdrawBackend::CodedVideoSink::queueDrained()
{
	return elementQueueDrained(this, mBackend);
}


int PdrawBackend::createRawVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateRawVideoSink(mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::RawVideoSink::~RawVideoSink()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::RawVideoSink::setMediaId(unsigned int mediaId)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, mediaId]() {
		return getElement()->setMediaId(mediaId);
	};
	return mBackend->runOnLoop(func);
}


unsigned int PdrawBackend::RawVideoSink::getMediaId()
{
	return getElementMediaId(this, mBackend);
}


struct mbuf_raw_video_frame_queue *PdrawBackend::RawVideoSink::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::RawVideoSink::queueFlushed()
{
	return elementQueueFlushed(this, mBackend);
}


int PdrawBackend::RawVideoSink::queueDrained()
{
	return elementQueueDrained(this, mBackend);
}


int PdrawBackend::createAlsaSource(
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener,
	IPdraw::IAlsaSource **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(strlen(params->address) > MAX_STR_LEN,
				 ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, params, listener, retObj]() {
		return doCreateAlsaSource(params, listener, retObj);
	};
	return runOnLoop(func);
}


int PdrawBackend::createAudioSource(
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener,
	IPdraw::IAudioSource **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, params, listener, retObj]() {
		return doCreateAudioSource(params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::AlsaSource::~AlsaSource()
{
	deleteElement(this, mBackend);
}


bool PdrawBackend::AlsaSource::isReadyToPlay()
{
	return isElementReadyToPlay(this, mBackend);
}


bool PdrawBackend::AlsaSource::isPaused()
{
	return isElementPaused(this, mBackend);
}


int PdrawBackend::AlsaSource::play()
{
	return playElement(this, mBackend);
}


int PdrawBackend::AlsaSource::pause()
{
	return pauseElement(this, mBackend);
}


PdrawBackend::AudioSource::~AudioSource()
{
	deleteElement(this, mBackend);
}


struct mbuf_audio_frame_queue *PdrawBackend::AudioSource::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::AudioSource::flush()
{
	return flushElement(this, mBackend);
}


int PdrawBackend::AudioSource::drain()
{
	return drainElement(this, mBackend);
}


int PdrawBackend::createAudioSink(unsigned int mediaId,
				  IPdraw::IAudioSink::Listener *listener,
				  IPdraw::IAudioSink **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, listener, retObj]() {
		return doCreateAudioSink(mediaId, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::AudioSink::~AudioSink()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::AudioSink::setMediaId(unsigned int mediaId)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, mediaId]() {
		return getElement()->setMediaId(mediaId);
	};
	return mBackend->runOnLoop(func);
}


unsigned int PdrawBackend::AudioSink::getMediaId()
{
	return getElementMediaId(this, mBackend);
}


struct mbuf_audio_frame_queue *PdrawBackend::AudioSink::getQueue()
{
	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!hasElement(), EPROTO, nullptr);

	auto func = [this]() { return getElement()->getQueue(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::AudioSink::queueFlushed()
{
	return elementQueueFlushed(this, mBackend);
}


int PdrawBackend::AudioSink::queueDrained()
{
	return elementQueueDrained(this, mBackend);
}


int PdrawBackend::createAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateAudioRenderer(mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::AudioRenderer::~AudioRenderer()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::AudioRenderer::setMediaId(unsigned int mediaId)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, mediaId]() {
		return getElement()->setMediaId(mediaId);
	};
	return mBackend->runOnLoop(func);
}


unsigned int PdrawBackend::AudioRenderer::getMediaId()
{
	return getElementMediaId(this, mBackend);
}


int PdrawBackend::AudioRenderer::setParams(
	const struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, &params]() {
		return getElement()->setParams(params);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::AudioRenderer::getParams(
	struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this, &params]() {
		return getElement()->getParams(params);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::createVideoEncoder(unsigned int mediaId,
				     const struct venc_config *params,
				     IPdraw::IVideoEncoder::Listener *listener,
				     IPdraw::IVideoEncoder **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateVideoEncoder(mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::VideoEncoder::~VideoEncoder()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::VideoEncoder::configure(const struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(config == nullptr, EINVAL);

	auto func = [this, &config]() {
		return getElement()->configure(config);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::VideoEncoder::getConfig(struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(config == nullptr, EINVAL);

	auto func = [this, &config]() {
		return getElement()->getConfig(config);
	};
	return mBackend->runOnLoop(func);
}


int PdrawBackend::VideoEncoder::requestKeyFrame()
{
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!hasElement(), EPROTO);

	auto func = [this]() { return getElement()->requestKeyFrame(); };
	return mBackend->runOnLoop(func);
}


int PdrawBackend::createVideoScaler(unsigned int mediaId,
				    const struct vscale_config *params,
				    IPdraw::IVideoScaler::Listener *listener,
				    IPdraw::IVideoScaler **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);


	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateVideoScaler(mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::VideoScaler::~VideoScaler()
{
	deleteElement(this, mBackend);
}


int PdrawBackend::createAudioEncoder(unsigned int mediaId,
				     const struct aenc_config *params,
				     IPdraw::IAudioEncoder::Listener *listener,
				     IPdraw::IAudioEncoder **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, mediaId, params, listener, retObj]() {
		return doCreateAudioEncoder(mediaId, params, listener, retObj);
	};
	return runOnLoop(func);
}


PdrawBackend::AudioEncoder::~AudioEncoder()
{
	deleteElement(this, mBackend);
}


void PdrawBackend::getFriendlyNameSetting(std::string *friendlyName)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	auto func = [this, &friendlyName]() {
		mPdraw->getFriendlyNameSetting(friendlyName);
	};
	runOnLoop(func);
}


void PdrawBackend::setFriendlyNameSetting(const std::string &friendlyName)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(friendlyName.length() > MAX_STR_LEN, ENOBUFS);

	auto func = [this, &friendlyName]() {
		mPdraw->setFriendlyNameSetting(friendlyName);
	};
	runOnLoop(func);
}


void PdrawBackend::getSerialNumberSetting(std::string *serialNumber)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	auto func = [this, &serialNumber]() {
		mPdraw->getSerialNumberSetting(serialNumber);
	};
	runOnLoop(func);
}


void PdrawBackend::setSerialNumberSetting(const std::string &serialNumber)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(serialNumber.length() > MAX_STR_LEN, ENOBUFS);

	auto func = [this, &serialNumber]() {
		mPdraw->setSerialNumberSetting(serialNumber);
	};
	runOnLoop(func);
}


void PdrawBackend::getSoftwareVersionSetting(std::string *softwareVersion)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	auto func = [this, &softwareVersion]() {
		mPdraw->getSoftwareVersionSetting(softwareVersion);
	};
	runOnLoop(func);
}


void PdrawBackend::setSoftwareVersionSetting(const std::string &softwareVersion)
{
	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(softwareVersion.length() > MAX_STR_LEN, ENOBUFS);

	auto func = [this, &softwareVersion]() {
		mPdraw->setSoftwareVersionSetting(softwareVersion);
	};
	runOnLoop(func);
}


int PdrawBackend::dumpPipeline(const std::string &fileName)
{
	ULOG_ERRNO_RETURN_ERR_IF(fileName.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	auto func = [this, &fileName]() {
		return mPdraw->dumpPipeline(fileName);
	};
	return runOnLoop(func);
}


/**
 * Private functions
 */

void PdrawBackend::stopResponse(IPdraw *pdraw, int status)
{
	PDRAW_CHECK_LOOP_THREAD();
	mListener->stopResponse(this, status);
	mThreadShouldStop = true;
	if (mLoop) {
		int err = mLoop->wakeup();
		if (err < 0)
			ULOG_ERRNO("pomp::Loop::wakeup", -err);
	}
}


void PdrawBackend::onMediaAdded(IPdraw *pdraw,
				const struct pdraw_media_info *info,
				void *elementUserData)
{
	bool found = false;

	PDRAW_CHECK_LOOP_THREAD();

	if (elementUserData == nullptr)
		goto cb;

	{
		std::scoped_lock lock(mMapsMutex);
		found = tryResolveElementUserData(elementUserData,
						  mDemuxerListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mMuxerListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mVipcSourceListenersMap) ||
			tryResolveElementUserData(
				elementUserData,
				mCodedVideoSourceListenersMap) ||
			tryResolveElementUserData(
				elementUserData, mRawVideoSourceListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mVideoEncoderListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mVideoScalerListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mAlsaSourceListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mAudioSourceListenersMap) ||
			tryResolveElementUserData(elementUserData,
						  mAudioEncoderListenersMap);
		if (!found) {
			ULOGW("%s: element userdata not found", __func__);
			elementUserData = nullptr;
		}
	}

cb:
	mListener->onMediaAdded(this, info, elementUserData);
}


void PdrawBackend::onMediaRemoved(IPdraw *pdraw,
				  const struct pdraw_media_info *info,
				  void *elementUserData)
{
	bool found = false;

	PDRAW_CHECK_LOOP_THREAD();

	if (elementUserData == nullptr)
		goto cb;

	{
		std::scoped_lock lock(mMapsMutex);
		if (tryResolveElementUserData(elementUserData,
					      mDemuxerListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mMuxerListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mVipcSourceListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mCodedVideoSourceListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mRawVideoSourceListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mVideoEncoderListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mVideoScalerListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mAlsaSourceListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mAudioSourceListenersMap) ||
		    tryResolveElementUserData(elementUserData,
					      mAudioEncoderListenersMap)) {
			found = true;
		} else if (elementUserData ==
			   mPendingRemovedElementUserdata.internal) {
			elementUserData =
				mPendingRemovedElementUserdata.external;
			found = true;
		}

		if (!found) {
			ULOGW("%s: element userdata not found", __func__);
			elementUserData = nullptr;
		}
	}

cb:
	mListener->onMediaRemoved(this, info, elementUserData);
}


void PdrawBackend::onSocketCreated(IPdraw *pdraw, int fd)
{
	PDRAW_CHECK_LOOP_THREAD();
	mListener->onSocketCreated(this, fd);
}


void PdrawBackend::demuxerOpenResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerOpenResponse(this, dl.e, status);
}


void PdrawBackend::demuxerCloseResponse(IPdraw *pdraw,
					IPdraw::IDemuxer *demuxer,
					int status)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerCloseResponse(this, dl.e, status);
}


void PdrawBackend::onDemuxerUnrecoverableError(IPdraw *pdraw,
					       IPdraw::IDemuxer *demuxer)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->onDemuxerUnrecoverableError(this, dl.e);
}


int PdrawBackend::demuxerSelectMedia(IPdraw *pdraw,
				     IPdraw::IDemuxer *demuxer,
				     const struct pdraw_demuxer_media *medias,
				     size_t count,
				     uint32_t selectedMedias)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return -ENOENT;

	return dl.l->demuxerSelectMedia(
		this, dl.e, medias, count, selectedMedias);
}


void PdrawBackend::demuxerReadyToPlay(IPdraw *pdraw,
				      IPdraw::IDemuxer *demuxer,
				      bool ready)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerReadyToPlay(this, dl.e, ready);
}


void PdrawBackend::onDemuxerEndOfRange(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       uint64_t timestamp)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->onDemuxerEndOfRange(this, dl.e, timestamp);
}


void PdrawBackend::demuxerPlayResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status,
				       uint64_t timestamp,
				       float speed)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerPlayResponse(this, dl.e, status, timestamp, speed);
}


void PdrawBackend::demuxerPauseResponse(IPdraw *pdraw,
					IPdraw::IDemuxer *demuxer,
					int status,
					uint64_t timestamp)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerPauseResponse(this, dl.e, status, timestamp);
}


void PdrawBackend::demuxerSeekResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status,
				       uint64_t timestamp,
				       float speed)
{
	demuxerAndListener dl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(demuxer,
				    dl,
				    __func__,
				    mDemuxerListenersMap,
				    mPendingDemuxerAndListener,
				    mMapsMutex,
				    DEMUXER))
		return;

	dl.l->demuxerSeekResponse(this, dl.e, status, timestamp, speed);
}


void PdrawBackend::onMuxerConnectionStateChanged(
	IPdraw *pdraw,
	IPdraw::IMuxer *muxer,
	enum pdraw_muxer_connection_state connectionState,
	enum pdraw_muxer_disconnection_reason disconnectionReason)
{
	muxerAndListener ml;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(muxer,
				    ml,
				    __func__,
				    mMuxerListenersMap,
				    mPendingMuxerAndListener,
				    mMapsMutex,
				    MUXER))
		return;

	ml.l->onMuxerConnectionStateChanged(
		this, ml.e, connectionState, disconnectionReason);
}


void PdrawBackend::onMuxerMediaReady(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IMuxer *muxer,
				     const char *mediaPath,
				     const struct iovec *iov,
				     int iovcnt)
{
	muxerAndListener ml;

	if (!findElementAndListener(muxer,
				    ml,
				    __func__,
				    mMuxerListenersMap,
				    mPendingMuxerAndListener,
				    mMapsMutex,
				    MUXER))
		return;

	ml.l->onMuxerMediaReady(this, ml.e, mediaPath, iov, iovcnt);
}


void PdrawBackend::onMuxerMediaSaved(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IMuxer *muxer,
				     const char *mediaPath)
{
	muxerAndListener ml;

	if (!findElementAndListener(muxer,
				    ml,
				    __func__,
				    mMuxerListenersMap,
				    mPendingMuxerAndListener,
				    mMapsMutex,
				    MUXER))
		return;

	ml.l->onMuxerMediaSaved(this, ml.e, mediaPath);
}


void PdrawBackend::onMuxerUnrecoverableError(IPdraw *pdraw,
					     IPdraw::IMuxer *muxer,
					     int status)
{
	muxerAndListener ml;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(muxer,
				    ml,
				    __func__,
				    mMuxerListenersMap,
				    mPendingMuxerAndListener,
				    mMapsMutex,
				    MUXER))
		return;

	ml.l->onMuxerUnrecoverableError(this, ml.e, status);
}


void PdrawBackend::muxerCloseResponse(IPdraw *pdraw,
				      IPdraw::IMuxer *muxer,
				      int status)
{
	muxerAndListener ml;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(muxer,
				    ml,
				    __func__,
				    mMuxerListenersMap,
				    mPendingMuxerAndListener,
				    mMapsMutex,
				    MUXER))
		return;

	ml.l->muxerCloseResponse(this, ml.e, status);
}


void PdrawBackend::onVideoRendererMediaAdded(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info)
{
	videoRendererAndListener rl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mVideoRendererListenersMap,
				    mPendingVideoRendererAndListener,
				    mMapsMutex,
				    VIDEO_RENDERER))
		return;

	rl.l->onVideoRendererMediaAdded(this, rl.e, info);
}


void PdrawBackend::onVideoRendererMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info,
	bool restart)
{
	videoRendererAndListener rl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mVideoRendererListenersMap,
				    mPendingVideoRendererAndListener,
				    mMapsMutex,
				    VIDEO_RENDERER))
		return;

	rl.l->onVideoRendererMediaRemoved(this, rl.e, info, restart);
}


void PdrawBackend::onVideoRenderReady(IPdraw *pdraw,
				      IPdraw::IVideoRenderer *renderer)
{
	videoRendererAndListener rl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mVideoRendererListenersMap,
				    mPendingVideoRendererAndListener,
				    mMapsMutex,
				    VIDEO_RENDERER))
		return;

	rl.l->onVideoRenderReady(this, rl.e);
}


int PdrawBackend::loadVideoTexture(IPdraw *pdraw,
				   IPdraw::IVideoRenderer *renderer,
				   unsigned int textureWidth,
				   unsigned int textureHeight,
				   const struct pdraw_media_info *mediaInfo,
				   struct mbuf_raw_video_frame *frame,
				   const void *frameUserdata,
				   size_t frameUserdataLen)
{
	videoRendererAndListener rl;

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mVideoRendererListenersMap,
				    mPendingVideoRendererAndListener,
				    mMapsMutex,
				    VIDEO_RENDERER))
		return -ENOENT;

	return rl.l->loadVideoTexture(this,
				      rl.e,
				      textureWidth,
				      textureHeight,
				      mediaInfo,
				      frame,
				      frameUserdata,
				      frameUserdataLen);
}


int PdrawBackend::renderVideoOverlay(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_rect *renderPos,
	const struct pdraw_rect *contentPos,
	const float *viewMat,
	const float *projMat,
	const struct pdraw_media_info *mediaInfo,
	struct vmeta_frame *frameMeta,
	const struct pdraw_video_frame_extra *frameExtra)
{
	videoRendererAndListener rl;

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mVideoRendererListenersMap,
				    mPendingVideoRendererAndListener,
				    mMapsMutex,
				    VIDEO_RENDERER))
		return -ENOENT;

	return rl.l->renderVideoOverlay(this,
					rl.e,
					renderPos,
					contentPos,
					viewMat,
					projMat,
					mediaInfo,
					frameMeta,
					frameExtra);
}


void PdrawBackend::onAudioRendererMediaAdded(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IAudioRenderer *renderer,
	const struct pdraw_media_info *info)
{
	audioRendererAndListener rl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mAudioRendererListenersMap,
				    mPendingAudioRendererAndListener,
				    mMapsMutex,
				    AUDIO_RENDERER))
		return;

	rl.l->onAudioRendererMediaAdded(this, rl.e, info);
}


void PdrawBackend::onAudioRendererMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IAudioRenderer *renderer,
	const struct pdraw_media_info *info)
{
	audioRendererAndListener rl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(renderer,
				    rl,
				    __func__,
				    mAudioRendererListenersMap,
				    mPendingAudioRendererAndListener,
				    mMapsMutex,
				    AUDIO_RENDERER))
		return;

	rl.l->onAudioRendererMediaRemoved(this, rl.e, info);
}


void PdrawBackend::vipcSourceReadyToPlay(
	IPdraw *pdraw,
	IPdraw::IVipcSource *source,
	bool ready,
	enum pdraw_vipc_source_eos_reason eosReason)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return;

	sl.l->vipcSourceReadyToPlay(this, sl.e, ready, eosReason);
}


void PdrawBackend::vipcSourcePlayResponse(IPdraw *pdraw,
					  IPdraw::IVipcSource *source)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return;

	sl.l->vipcSourcePlayResponse(this, sl.e);
}


void PdrawBackend::vipcSourcePauseResponse(IPdraw *pdraw,
					   IPdraw::IVipcSource *source)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return;

	sl.l->vipcSourcePauseResponse(this, sl.e);
}


bool PdrawBackend::vipcSourceFramerateChanged(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVipcSource *source,
	const struct vdef_frac *prevFramerate,
	const struct vdef_frac *newFramerate)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return false;

	return sl.l->vipcSourceFramerateChanged(
		this, sl.e, prevFramerate, newFramerate);
}


void PdrawBackend::vipcSourceConfigured(IPdraw *pdraw,
					IPdraw::IVipcSource *source,
					int status,
					const struct vdef_format_info *info,
					const struct vdef_rectf *crop)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return;

	sl.l->vipcSourceConfigured(this, sl.e, status, info, crop);
}


void PdrawBackend::vipcSourceFrameReady(IPdraw *pdraw,
					IPdraw::IVipcSource *source,
					struct mbuf_raw_video_frame *frame)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return;

	sl.l->vipcSourceFrameReady(this, sl.e, frame);
}


bool PdrawBackend::vipcSourceEndOfStream(
	IPdraw *pdraw,
	IPdraw::IVipcSource *source,
	enum pdraw_vipc_source_eos_reason eosReason)
{
	vipcSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mVipcSourceListenersMap,
				    mPendingVipcSourceAndListener,
				    mMapsMutex,
				    VIPC_SOURCE))
		return false;

	return sl.l->vipcSourceEndOfStream(this, sl.e, eosReason);
}


void PdrawBackend::onCodedVideoSourceFlushed(IPdraw *pdraw,
					     IPdraw::ICodedVideoSource *source)
{
	codedVideoSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mCodedVideoSourceListenersMap,
				    mPendingCodedVideoSourceAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SOURCE))
		return;

	sl.l->onCodedVideoSourceFlushed(this, sl.e);
}


void PdrawBackend::onCodedVideoSourceDrained(IPdraw *pdraw,
					     IPdraw::ICodedVideoSource *source)
{
	codedVideoSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mCodedVideoSourceListenersMap,
				    mPendingCodedVideoSourceAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SOURCE))
		return;

	sl.l->onCodedVideoSourceDrained(this, sl.e);
}


void PdrawBackend::onRawVideoSourceFlushed(IPdraw *pdraw,
					   IPdraw::IRawVideoSource *source)
{
	rawVideoSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mRawVideoSourceListenersMap,
				    mPendingRawVideoSourceAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SOURCE))
		return;

	sl.l->onRawVideoSourceFlushed(this, sl.e);
}


void PdrawBackend::onCodedVideoSinkMediaAdded(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::ICodedVideoSink *sink,
	const struct pdraw_media_info *info)
{
	std::map<IPdraw::ICodedVideoSink *, codedVideoSinkAndListener>::iterator
		it;
	codedVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mCodedVideoSinkListenersMap,
				    mPendingCodedVideoSinkAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SINK))
		return;

	sl.l->onCodedVideoSinkMediaAdded(this, sl.e, info);
}


void PdrawBackend::onCodedVideoSinkMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::ICodedVideoSink *sink,
	const struct pdraw_media_info *info,
	bool restart)
{
	std::map<IPdraw::ICodedVideoSink *, codedVideoSinkAndListener>::iterator
		it;
	codedVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mCodedVideoSinkListenersMap,
				    mPendingCodedVideoSinkAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SINK))
		return;

	sl.l->onCodedVideoSinkMediaRemoved(this, sl.e, info, restart);
}


void PdrawBackend::onRawVideoSourceDrained(IPdraw *pdraw,
					   IPdraw::IRawVideoSource *source)
{
	rawVideoSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mRawVideoSourceListenersMap,
				    mPendingRawVideoSourceAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SOURCE))
		return;

	sl.l->onRawVideoSourceDrained(this, sl.e);
}


void PdrawBackend::onCodedVideoSinkFlush(IPdraw *pdraw,
					 IPdraw::ICodedVideoSink *sink)
{
	codedVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mCodedVideoSinkListenersMap,
				    mPendingCodedVideoSinkAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SINK))
		return;

	sl.l->onCodedVideoSinkFlush(this, sl.e);
}


void PdrawBackend::onCodedVideoSinkDrain(IPdraw *pdraw,
					 IPdraw::ICodedVideoSink *sink)
{
	codedVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mCodedVideoSinkListenersMap,
				    mPendingCodedVideoSinkAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SINK))
		return;

	sl.l->onCodedVideoSinkDrain(this, sl.e);
}


void PdrawBackend::onCodedVideoSinkSessionMetaUpdate(
	IPdraw *pdraw,
	IPdraw::ICodedVideoSink *sink,
	const struct vmeta_session *meta)
{
	codedVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mCodedVideoSinkListenersMap,
				    mPendingCodedVideoSinkAndListener,
				    mMapsMutex,
				    CODED_VIDEO_SINK))
		return;

	sl.l->onCodedVideoSinkSessionMetaUpdate(this, sl.e, meta);
}


void PdrawBackend::onRawVideoSinkFlush(IPdraw *pdraw,
				       IPdraw::IRawVideoSink *sink)
{
	rawVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mRawVideoSinkListenersMap,
				    mPendingRawVideoSinkAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SINK))
		return;

	sl.l->onRawVideoSinkFlush(this, sl.e);
}


void PdrawBackend::onRawVideoSinkDrain(IPdraw *pdraw,
				       IPdraw::IRawVideoSink *sink)
{
	rawVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mRawVideoSinkListenersMap,
				    mPendingRawVideoSinkAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SINK))
		return;

	sl.l->onRawVideoSinkDrain(this, sl.e);
}


void PdrawBackend::onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
					    Pdraw::IPdraw::IRawVideoSink *sink,
					    const struct pdraw_media_info *info)
{
	rawVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mRawVideoSinkListenersMap,
				    mPendingRawVideoSinkAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SINK))
		return;

	sl.l->onRawVideoSinkMediaAdded(this, sl.e, info);
}


void PdrawBackend::onRawVideoSinkMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IRawVideoSink *sink,
	const struct pdraw_media_info *info,
	bool restart)
{
	rawVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mRawVideoSinkListenersMap,
				    mPendingRawVideoSinkAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SINK))
		return;

	sl.l->onRawVideoSinkMediaRemoved(this, sl.e, info, restart);
}


void PdrawBackend::onRawVideoSinkSessionMetaUpdate(
	IPdraw *pdraw,
	IPdraw::IRawVideoSink *sink,
	const struct vmeta_session *meta)
{
	rawVideoSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mRawVideoSinkListenersMap,
				    mPendingRawVideoSinkAndListener,
				    mMapsMutex,
				    RAW_VIDEO_SINK))
		return;

	sl.l->onRawVideoSinkSessionMetaUpdate(this, sl.e, meta);
}


void PdrawBackend::alsaSourceReadyToPlay(
	IPdraw *pdraw,
	IPdraw::IAlsaSource *source,
	bool ready,
	enum pdraw_alsa_source_eos_reason eosReason)
{
	alsaSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAlsaSourceListenersMap,
				    mPendingAlsaSourceAndListener,
				    mMapsMutex,
				    ALSA_SOURCE))
		return;

	sl.l->alsaSourceReadyToPlay(this, sl.e, ready, eosReason);
}


void PdrawBackend::alsaSourcePlayResponse(IPdraw *pdraw,
					  IPdraw::IAlsaSource *source)
{
	alsaSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAlsaSourceListenersMap,
				    mPendingAlsaSourceAndListener,
				    mMapsMutex,
				    ALSA_SOURCE))
		return;

	sl.l->alsaSourcePlayResponse(this, sl.e);
}


void PdrawBackend::alsaSourcePauseResponse(IPdraw *pdraw,
					   IPdraw::IAlsaSource *source)
{
	alsaSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAlsaSourceListenersMap,
				    mPendingAlsaSourceAndListener,
				    mMapsMutex,
				    ALSA_SOURCE))
		return;

	sl.l->alsaSourcePauseResponse(this, sl.e);
}


void PdrawBackend::alsaSourceFrameReady(IPdraw *pdraw,
					IPdraw::IAlsaSource *source,
					struct mbuf_audio_frame *frame)
{
	alsaSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAlsaSourceListenersMap,
				    mPendingAlsaSourceAndListener,
				    mMapsMutex,
				    ALSA_SOURCE))
		return;

	sl.l->alsaSourceFrameReady(this, sl.e, frame);
}


void PdrawBackend::onAudioSourceFlushed(IPdraw *pdraw,
					IPdraw::IAudioSource *source)
{
	audioSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAudioSourceListenersMap,
				    mPendingAudioSourceAndListener,
				    mMapsMutex,
				    AUDIO_SOURCE))
		return;

	sl.l->onAudioSourceFlushed(this, sl.e);
}


void PdrawBackend::onAudioSourceDrained(IPdraw *pdraw,
					IPdraw::IAudioSource *source)
{
	audioSourceAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(source,
				    sl,
				    __func__,
				    mAudioSourceListenersMap,
				    mPendingAudioSourceAndListener,
				    mMapsMutex,
				    AUDIO_SOURCE))
		return;

	sl.l->onAudioSourceDrained(this, sl.e);
}


void PdrawBackend::onAudioSinkMediaAdded(Pdraw::IPdraw *pdraw,
					 Pdraw::IPdraw::IAudioSink *sink,
					 const struct pdraw_media_info *info)
{
	std::map<IPdraw::IAudioSink *, audioSinkAndListener>::iterator it;
	audioSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mAudioSinkListenersMap,
				    mPendingAudioSinkAndListener,
				    mMapsMutex,
				    AUDIO_SINK))
		return;

	sl.l->onAudioSinkMediaAdded(this, sl.e, info);
}


void PdrawBackend::onAudioSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					   Pdraw::IPdraw::IAudioSink *sink,
					   const struct pdraw_media_info *info,
					   bool restart)
{
	std::map<IPdraw::IAudioSink *, audioSinkAndListener>::iterator it;
	audioSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mAudioSinkListenersMap,
				    mPendingAudioSinkAndListener,
				    mMapsMutex,
				    AUDIO_SINK))
		return;

	sl.l->onAudioSinkMediaRemoved(this, sl.e, info, restart);
}


void PdrawBackend::onAudioSinkFlush(IPdraw *pdraw, IPdraw::IAudioSink *sink)
{
	audioSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mAudioSinkListenersMap,
				    mPendingAudioSinkAndListener,
				    mMapsMutex,
				    AUDIO_SINK))
		return;

	sl.l->onAudioSinkFlush(this, sl.e);
}


void PdrawBackend::onAudioSinkDrain(IPdraw *pdraw, IPdraw::IAudioSink *sink)
{
	audioSinkAndListener sl;

	PDRAW_CHECK_LOOP_THREAD();

	if (!findElementAndListener(sink,
				    sl,
				    __func__,
				    mAudioSinkListenersMap,
				    mPendingAudioSinkAndListener,
				    mMapsMutex,
				    AUDIO_SINK))
		return;

	sl.l->onAudioSinkDrain(this, sl.e);
}


void PdrawBackend::videoEncoderFrameOutput(IPdraw *pdraw,
					   IPdraw::IVideoEncoder *encoder,
					   struct mbuf_coded_video_frame *frame)
{
	videoEncoderAndListener el;

	if (!findElementAndListener(encoder,
				    el,
				    __func__,
				    mVideoEncoderListenersMap,
				    mPendingVideoEncoderAndListener,
				    mMapsMutex,
				    VIDEO_ENCODER))
		return;

	el.l->videoEncoderFrameOutput(this, el.e, frame);
}


void PdrawBackend::videoEncoderFramePreRelease(
	IPdraw *pdraw,
	IPdraw::IVideoEncoder *encoder,
	struct mbuf_coded_video_frame *frame)
{
	videoEncoderAndListener el;

	if (!findElementAndListener(encoder,
				    el,
				    __func__,
				    mVideoEncoderListenersMap,
				    mPendingVideoEncoderAndListener,
				    mMapsMutex,
				    VIDEO_ENCODER))
		return;

	el.l->videoEncoderFramePreRelease(this, el.e, frame);
}


void PdrawBackend::videoScalerFrameOutput(IPdraw *pdraw,
					  IPdraw::IVideoScaler *scaler,
					  struct mbuf_raw_video_frame *frame)
{
	videoScalerAndListener sl;

	if (!findElementAndListener(scaler,
				    sl,
				    __func__,
				    mVideoScalerListenersMap,
				    mPendingVideoScalerAndListener,
				    mMapsMutex,
				    VIDEO_SCALER))
		return;

	sl.l->videoScalerFrameOutput(this, sl.e, frame);
}


void PdrawBackend::audioEncoderFrameOutput(IPdraw *pdraw,
					   IPdraw::IAudioEncoder *encoder,
					   struct mbuf_audio_frame *frame)
{
	audioEncoderAndListener el;

	if (!findElementAndListener(encoder,
				    el,
				    __func__,
				    mAudioEncoderListenersMap,
				    mPendingAudioEncoderAndListener,
				    mMapsMutex,
				    AUDIO_ENCODER))
		return;

	el.l->audioEncoderFrameOutput(this, el.e, frame);
}


void PdrawBackend::audioEncoderFramePreRelease(IPdraw *pdraw,
					       IPdraw::IAudioEncoder *encoder,
					       struct mbuf_audio_frame *frame)
{
	audioEncoderAndListener el;

	if (!findElementAndListener(encoder,
				    el,
				    __func__,
				    mAudioEncoderListenersMap,
				    mPendingAudioEncoderAndListener,
				    mMapsMutex,
				    AUDIO_ENCODER))
		return;

	el.l->audioEncoderFramePreRelease(this, el.e, frame);
}


void PdrawBackend::checkLoopThread(const char *func) const
{
	if (std::this_thread::get_id() != mLoopThread.get_id())
		ULOGW("%s not called from the loop thread", func);
}


void PdrawBackend::loopThread(PdrawBackend *self)
{
	int res = 0;
	int err;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	err = pthread_setname_np("pdraw_backend");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#	endif
#else
	err = pthread_setname_np(pthread_self(), "pdraw_backend");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#endif

	{
		std::scoped_lock lock(self->mMutex);

		IPdraw *pdraw = nullptr;

		try {
			self->mLoop = std::make_unique<pomp::Loop>();
		} catch (const std::bad_alloc &) {
			res = -ENOMEM;
			ULOG_ERRNO("pomp::Loop::new", -res);
			goto error;
		}

		res = createPdraw(self->mLoop->get(), self, &pdraw);
		self->mPdraw.reset(pdraw);
		if (res < 0) {
			ULOG_ERRNO("createPdraw", -res);
			goto error;
		}

		res = 0;

		/* clang-format off */
error:
		/* clang-format on */
		self->mRetStatus = res;
		self->mRetValReady = true;
		self->mCond.notify_one();
	}

	if (res == 0) {
		while (!self->mThreadShouldStop) {
			err = self->mLoop->waitAndProcess(-1);
			if (err < 0)
				ULOG_ERRNO("pomp::Loop::waitAndProcess", -err);
		}
	}

	{
		std::scoped_lock lock(self->mMutex);
		self->mPdraw.reset();
		self->mLoop.reset();
	}
}


int PdrawBackend::doCreateDemuxer(const std::string &url,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::Demuxer>(this, listener),
			listener,
			retObj,
			mPendingDemuxerAndListener,
			mDemuxerListenersMap,
			[this, &url, &params](IPdraw::IDemuxer **internal) {
				return mPdraw->createDemuxer(
					url, params, this, internal);
			},
			DEMUXER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateDemuxer(const std::string &localAddr,
				  uint16_t localStreamPort,
				  uint16_t localControlPort,
				  const std::string &remoteAddr,
				  uint16_t remoteStreamPort,
				  uint16_t remoteControlPort,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::Demuxer>(this, listener),
			listener,
			retObj,
			mPendingDemuxerAndListener,
			mDemuxerListenersMap,
			[this,
			 &localAddr,
			 localStreamPort,
			 localControlPort,
			 &remoteAddr,
			 remoteStreamPort,
			 remoteControlPort,
			 &params](IPdraw::IDemuxer **internal) {
				return mPdraw->createDemuxer(localAddr,
							     localStreamPort,
							     localControlPort,
							     remoteAddr,
							     remoteStreamPort,
							     remoteControlPort,
							     params,
							     this,
							     internal);
			},
			DEMUXER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateDemuxer(const std::string &url,
				  struct mux_ctx *mux,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::Demuxer>(this, listener),
			listener,
			retObj,
			mPendingDemuxerAndListener,
			mDemuxerListenersMap,
			[this, &url, &mux, &params](
				IPdraw::IDemuxer **internal) {
				return mPdraw->createDemuxer(
					url, mux, params, this, internal);
			},
			DEMUXER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateMuxer(const std::string &url,
				const struct pdraw_muxer_params *params,
				IPdraw::IMuxer::Listener *listener,
				IPdraw::IMuxer **retObj)
{
	return doCreateMuxer(url, nullptr, {}, params, listener, retObj);
}


int PdrawBackend::doCreateMuxer(const std::string &url,
				struct mux_ctx *mux,
				const std::string &remoteHost,
				const struct pdraw_muxer_params *params,
				IPdraw::IMuxer::Listener *listener,
				IPdraw::IMuxer **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::Muxer>(
				this, url, listener),
			listener,
			retObj,
			mPendingMuxerAndListener,
			mMuxerListenersMap,
			[this, &url, &mux, &remoteHost, &params](
				IPdraw::IMuxer **internal) {
				return mPdraw->createMuxer(url,
							   mux,
							   remoteHost,
							   params,
							   this,
							   internal);
			},
			MUXER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateVipcSource(
	const struct pdraw_vipc_source_params *params,
	IPdraw::IVipcSource::Listener *listener,
	IPdraw::IVipcSource **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::VipcSource>(this,
								   listener),
			listener,
			retObj,
			mPendingVipcSourceAndListener,
			mVipcSourceListenersMap,
			[this, &params](IPdraw::IVipcSource **internal) {
				return mPdraw->createVipcSource(
					params, this, internal);
			},
			VIPC_SOURCE);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::CodedVideoSource>(
				this, listener),
			listener,
			retObj,
			mPendingCodedVideoSourceAndListener,
			mCodedVideoSourceListenersMap,
			[this, &params](IPdraw::ICodedVideoSource **internal) {
				return mPdraw->createCodedVideoSource(
					params, this, internal);
			},
			CODED_VIDEO_SOURCE);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::RawVideoSource>(
				this, listener),
			listener,
			retObj,
			mPendingRawVideoSourceAndListener,
			mRawVideoSourceListenersMap,
			[this, &params](IPdraw::IRawVideoSource **internal) {
				return mPdraw->createRawVideoSource(
					params, this, internal);
			},
			RAW_VIDEO_SOURCE);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateCodedVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::CodedVideoSink>(
				this, mediaId, params, listener),
			listener,
			retObj,
			mPendingCodedVideoSinkAndListener,
			mCodedVideoSinkListenersMap,
			[this, mediaId, &params](
				IPdraw::ICodedVideoSink **internal) {
				return mPdraw->createCodedVideoSink(
					mediaId, params, this, internal);
			},
			CODED_VIDEO_SINK);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateRawVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::RawVideoSink>(
				this, mediaId, params, listener),
			listener,
			retObj,
			mPendingRawVideoSinkAndListener,
			mRawVideoSinkListenersMap,
			[this, mediaId, &params](
				IPdraw::IRawVideoSink **internal) {
				return mPdraw->createRawVideoSink(
					mediaId, params, this, internal);
			},
			RAW_VIDEO_SINK);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateAlsaSource(
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener,
	IPdraw::IAlsaSource **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::AlsaSource>(this,
								   listener),
			listener,
			retObj,
			mPendingAlsaSourceAndListener,
			mAlsaSourceListenersMap,
			[this, &params](IPdraw::IAlsaSource **internal) {
				return mPdraw->createAlsaSource(
					params, this, internal);
			},
			ALSA_SOURCE);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateAudioSource(
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener,
	IPdraw::IAudioSource **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::AudioSource>(this,
								    listener),
			listener,
			retObj,
			mPendingAudioSourceAndListener,
			mAudioSourceListenersMap,
			[this, &params](IPdraw::IAudioSource **internal) {
				return mPdraw->createAudioSource(
					params, this, internal);
			},
			AUDIO_SOURCE);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateAudioSink(unsigned int mediaId,
				    IPdraw::IAudioSink::Listener *listener,
				    IPdraw::IAudioSink **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::AudioSink>(
				this, mediaId, listener),
			listener,
			retObj,
			mPendingAudioSinkAndListener,
			mAudioSinkListenersMap,
			[this, mediaId](IPdraw::IAudioSink **internal) {
				return mPdraw->createAudioSink(
					mediaId, this, internal);
			},
			AUDIO_SINK);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::AudioRenderer>(this,
								      listener),
			listener,
			retObj,
			mPendingAudioRendererAndListener,
			mAudioRendererListenersMap,
			[this, mediaId, &params](
				IPdraw::IAudioRenderer **internal) {
				return mPdraw->createAudioRenderer(
					mediaId, params, this, internal);
			},
			AUDIO_RENDERER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateVideoEncoder(
	unsigned int mediaId,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener,
	IPdraw::IVideoEncoder **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::VideoEncoder>(
				this, mediaId, params, listener),
			listener,
			retObj,
			mPendingVideoEncoderAndListener,
			mVideoEncoderListenersMap,
			[this, mediaId, &params](IPdraw::IVideoEncoder **out) {
				return mPdraw->createVideoEncoder(
					mediaId, params, this, out);
			},
			VIDEO_ENCODER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateVideoScaler(unsigned int mediaId,
				      const struct vscale_config *params,
				      IPdraw::IVideoScaler::Listener *listener,
				      IPdraw::IVideoScaler **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::VideoScaler>(
				this, mediaId, params, listener),
			listener,
			retObj,
			mPendingVideoScalerAndListener,
			mVideoScalerListenersMap,
			[this, mediaId, &params](
				IPdraw::IVideoScaler **internal) {
				return mPdraw->createVideoScaler(
					mediaId, params, this, internal);
			},
			VIDEO_SCALER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


int PdrawBackend::doCreateAudioEncoder(
	unsigned int mediaId,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener,
	IPdraw::IAudioEncoder **retObj)
{
	try {
		return internalElementCreate(
			std::make_unique<PdrawBackend::AudioEncoder>(
				this, mediaId, params, listener),
			listener,
			retObj,
			mPendingAudioEncoderAndListener,
			mAudioEncoderListenersMap,
			[this, mediaId, &params, &listener](
				IPdraw::IAudioEncoder **internal) {
				return mPdraw->createAudioEncoder(
					mediaId, params, listener, internal);
			},
			AUDIO_ENCODER);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}


} /* namespace PdrawBackend */
