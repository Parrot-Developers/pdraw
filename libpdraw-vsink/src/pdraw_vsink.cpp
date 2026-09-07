/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

#include "pdraw_vsink_priv.hpp"

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif
#include <errno.h>

#include <system_error>

ULOG_DECLARE_TAG(pdraw_vsink);

namespace PdrawVsink {

Vsink::Vsink(const struct pdraw_vsink_params *params,
	     IPdrawVsink::Listener *listener) :
		mType(params->video_media_type),
		mUrl(params->url), mPlaybackMode(params->playback_mode),
		mCameraType(params->camera_type), mListener(listener)
{
}


Vsink::~Vsink()
{
	/* Unblock (with -ECANCELED) any thread currently parked in
	 * getRawFrame()/getCodedFrame()'s cond_wait and wait for it to have
	 * returned before going any further: mLoop/mPdraw/mRaw/mCoded are
	 * torn down below, so a concurrent getFrame() caller still
	 * referencing them past this point would be a use-after-free. */
	{
		std::unique_lock lock(mMutex);
		mStopping = true;
		mCond.notify_all();
		mCond.wait(lock, [this] { return mWaiters == 0; });
	}

	if (mPdraw != nullptr) {
		mStopPdrawIdleHandler.set([this] { stopPdrawIdle(); });
		int res = mLoop->idleAdd(&mStopPdrawIdleHandler);
		if (res < 0)
			ULOG_ERRNO("pomp::Loop::idleAdd", -res);
		std::unique_lock lock(mMutex);
		mCond.wait(lock, [this] { return mCondReady; });
		mCondReady = false;
	}

	if (mThreadLaunched) {
		mThreadShouldStop = true;
		int res = mLoop->wakeup();
		if (res < 0)
			ULOG_ERRNO("pomp::Loop::wakeup", -res);
		if (mThread.joinable())
			mThread.join();
		mLoop.reset();
	}

	std::scoped_lock lock(mMutex);
	pdraw_media_info_free(mMediaInfo);
}


void Vsink::loopThread(Vsink *self)
{
#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	int err = pthread_setname_np("pdraw_vsink");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#	endif
#else
	int err = pthread_setname_np(pthread_self(), "pdraw_vsink");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#endif

	while (!self->mThreadShouldStop)
		self->mLoop->waitAndProcess(-1);
}


void Vsink::deletePdrawIdle()
{
	if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
		detachCodedEvent();
		destroyCodedSink();
	} else {
		detachRawEvent();
		destroyRawSink();
	}

	mDemuxer.reset();
	mPdraw.reset();

	std::scoped_lock lock(mMutex);
	mCondReady = true;
	mCond.notify_one();
}


void Vsink::demuxerOpenResponse(
	[[maybe_unused]] Pdraw::IPdraw *pdraw,
	[[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
	int status)
{
	ULOGI("%s: status=%d", __func__, status);

	if (status != 0) {
		std::scoped_lock lock(mMutex);
		mResult = status;
		mCondReady = true;
		mCond.notify_one();
	}
}


int Vsink::demuxerSelectMedia([[maybe_unused]] Pdraw::IPdraw *pdraw,
			      [[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
			      const struct pdraw_demuxer_media *medias,
			      size_t count,
			      [[maybe_unused]] uint32_t selectedMedias)
{
	for (size_t i = 0; i < count; i++) {
		if (medias[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		if (mCameraType == VMETA_CAMERA_TYPE_UNKNOWN &&
		    medias[i].is_default) {
			ULOGI("%s: selecting media '%s'",
			      __func__,
			      medias[i].name);
			return 1 << medias[i].media_id;
		}
		if (vmeta_camera_type_subtype_pair_cmp(
			    medias[i].video.session_meta.camera_type,
			    medias[i].video.session_meta.camera_subtype,
			    mCameraType,
			    VMETA_CAMERA_SUBTYPE_UNKNOWN) == 1) {
			ULOGI("%s: selecting media '%s'",
			      __func__,
			      medias[i].name);
			return 1 << medias[i].media_id;
		}
	}

	ULOGI("%s: no media selected", __func__);
	return -ECANCELED;
}


void Vsink::demuxerReadyToPlay(
	[[maybe_unused]] Pdraw::IPdraw *pdraw,
	[[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
	bool ready)
{
	ULOGI("%s: ready=%d", __func__, (int)ready);

	if (ready) {
		int res = mDemuxer->play();
		if (res < 0) {
			ULOG_ERRNO("IDemuxer::play", -res);
			std::scoped_lock lock(mMutex);
			mResult = res;
			mCondReady = true;
			mCond.notify_one();
		}
	}
}


void Vsink::demuxerPlayResponse(
	[[maybe_unused]] Pdraw::IPdraw *pdraw,
	[[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
	int status,
	[[maybe_unused]] uint64_t timestamp,
	float speed)
{
	ULOGI("%s: status=%d speed=%f", __func__, status, speed);

	if (status != 0)
		ULOG_ERRNO("demuxerPlayResponse", -status);
}


void Vsink::stopResponse([[maybe_unused]] Pdraw::IPdraw *pdraw, int status)
{
	ULOGI("%s: status=%d", __func__, status);

	mDeletePdrawIdleHandler.set([this] { deletePdrawIdle(); });
	int res = mLoop->idleAdd(&mDeletePdrawIdleHandler);
	if (res < 0) {
		ULOG_ERRNO("pomp::Loop::idleAdd", -res);
		std::scoped_lock lock(mMutex);
		mCondReady = true;
		mCond.notify_one();
	}
}


void Vsink::onMediaAdded([[maybe_unused]] Pdraw::IPdraw *pdraw,
			 const struct pdraw_media_info *info,
			 [[maybe_unused]] void *elementUserData)
{
	int res = 0;

	if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
		return;

	if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED &&
	    info->video.format != VDEF_FRAME_TYPE_CODED)
		return;

	if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW &&
	    info->video.format != VDEF_FRAME_TYPE_RAW)
		return;

	ULOGI("%s: id=%d (%s)",
	      __func__,
	      info->id,
	      mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED ? "coded" : "raw");

	std::scoped_lock lock(mMutex);

	if (mMediaInfo != nullptr)
		pdraw_media_info_free(mMediaInfo);
	mMediaInfo = pdraw_media_info_dup(info);
	if (mMediaInfo == nullptr) {
		ULOG_ERRNO("pdraw_media_info_dup", ENOMEM);
		goto out;
	}

	if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED)
		processCodedMediaAdded(info);
	else
		processRawMediaAdded(info);

out:
	if (mStarting) {
		mResult = res;
		mCondReady = true;
		mCond.notify_one();
	}
}


void Vsink::onMediaRemoved([[maybe_unused]] Pdraw::IPdraw *pdraw,
			   const struct pdraw_media_info *info,
			   [[maybe_unused]] void *elementUserData)
{
	ULOGI("%s: id=%d", __func__, info->id);
}


void Vsink::startPdrawIdle()
{
	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode =
		mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED
			? PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE
			: PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL;
	params.playback_mode = mPlaybackMode;

	Pdraw::IPdraw *pdraw = nullptr;
	int res = Pdraw::createPdraw(mLoop->get(), this, &pdraw);
	if (res < 0) {
		ULOG_ERRNO("createPdraw", -res);
		std::scoped_lock lock(mMutex);
		mResult = res;
		mCondReady = true;
		mCond.notify_one();
		return;
	}
	mPdraw.reset(pdraw);

	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	res = mPdraw->createDemuxer(mUrl, &params, this, &demuxer);
	if (res < 0) {
		ULOG_ERRNO("IPdraw::createDemuxer", -res);
		std::scoped_lock lock(mMutex);
		mResult = res;
		mCondReady = true;
		mCond.notify_one();
		return;
	}
	mDemuxer.reset(demuxer);
}


void Vsink::stopPdrawIdle()
{
	int res;

	{
		std::scoped_lock lock(mMutex);

		if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED)
			detachCodedEvent();
		else
			detachRawEvent();

		if (mDemuxer != nullptr) {
			res = mDemuxer->close();
			if (res < 0) {
				ULOG_ERRNO("IDemuxer::close", -res);
				goto error;
			}
		}
	}

	res = mPdraw->stop();
	if (res < 0) {
		ULOG_ERRNO("IPdraw::stop", -res);
		goto error;
	}

	return;

error:
	mDeletePdrawIdleHandler.set([this] { deletePdrawIdle(); });
	res = mLoop->idleAdd(&mDeletePdrawIdleHandler);
	if (res < 0) {
		ULOG_ERRNO("pomp::Loop::idleAdd", -res);
		std::scoped_lock lock(mMutex);
		mCondReady = true;
		mCond.notify_one();
	}
}


int Vsink::start(struct pdraw_media_info **mediaInfo)
{
	int res;

	try {
		mLoop = std::make_unique<pomp::Loop>();
	} catch (const std::bad_alloc &) {
		res = -ENOMEM;
		ULOGE("failed to create pomp loop");
		return res;
	}

	try {
		mThread = std::thread(&Vsink::loopThread, this);
	} catch (const std::system_error &e) {
		res = -e.code().value();
		ULOG_ERRNO("std::thread", -res);
		return res;
	}
	mStarting = true;
	mThreadLaunched = true;

	mStartPdrawIdleHandler.set([this] { startPdrawIdle(); });
	res = mLoop->idleAdd(&mStartPdrawIdleHandler);
	if (res < 0) {
		ULOG_ERRNO("pomp::Loop::idleAdd", -res);
		return res;
	}

	{
		std::unique_lock lock(mMutex);
		mCond.wait(lock, [this] { return mCondReady; });
		mCondReady = false;
		mStarting = false;
		res = mResult;
	}

	if (res < 0) {
		ULOG_ERRNO("failed to start pdraw vsink", -res);
		return res;
	}

	if (mediaInfo != nullptr) {
		std::scoped_lock lock(mMutex);
		*mediaInfo = pdraw_media_info_dup(mMediaInfo);
	}

	return 0;
}


int Vsink::getFrame(int timeoutMs,
		    struct mbuf_mem *frameMemory,
		    struct pdraw_video_frame *frameInfo,
		    struct pdraw_vsink_frame *retFrame)
{
	ULOG_ERRNO_RETURN_ERR_IF(frameInfo == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retFrame == nullptr, EINVAL);

	if (mType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
		retFrame->type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
		return getCodedFrame(
			timeoutMs, frameMemory, frameInfo, &retFrame->coded);
	} else {
		retFrame->type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;
		return getRawFrame(
			timeoutMs, frameMemory, frameInfo, &retFrame->raw);
	}
}


int createPdrawVsink(const struct pdraw_vsink_params *params,
		     IPdrawVsink::Listener *listener,
		     struct pdraw_media_info **mediaInfo,
		     IPdrawVsink **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->url == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	std::unique_ptr<Vsink> self;
	try {
		self = std::make_unique<Vsink>(params, listener);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}

	int res = self->start(mediaInfo);
	if (res < 0)
		return res;

	*retObj = self.release();
	return 0;
}

} /* namespace PdrawVsink */
