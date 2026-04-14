/**
 * Parrot Drones Audio and Video Vector
 * Qt PDrAW demuxer object
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

#include "qpdraw_demuxer_priv.hpp"
#include "qpdraw_priv.hpp"


#define ULOG_TAG qpdraw_demuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


namespace QPdraw {
namespace Internal {


QPdrawDemuxerPriv::QPdrawDemuxerPriv(QPdrawDemuxer *parent) : mParent(parent)
{
	qRegisterMetaType<pdraw_chapter>("pdraw_chapter");
}


IPdraw *QPdrawDemuxerPriv::getPdrawInternal()
{
	ULOG_ERRNO_RETURN_VAL_IF(mParent == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mParent->parent() == nullptr, EPROTO, nullptr);
	auto *qpdraw = reinterpret_cast<QPdraw *>(mParent->parent());
	ULOG_ERRNO_RETURN_VAL_IF(qpdraw == nullptr, EPROTO, nullptr);
	return reinterpret_cast<IPdraw *>(qpdraw->getInternal());
}


int QPdrawDemuxerPriv::open(const std::string &url,
			    const struct pdraw_demuxer_params *params)
{
	int ret;
	IPdraw::IDemuxer *demuxer = nullptr;
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	ret = pdrawInternal->createDemuxer(url, params, this, &demuxer);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	mDemuxer.reset(demuxer);
	return 0;
}


int QPdrawDemuxerPriv::open(const std::string &localAddr,
			    uint16_t localStreamPort,
			    uint16_t localControlPort,
			    const std::string &remoteAddr,
			    uint16_t remoteStreamPort,
			    uint16_t remoteControlPort,
			    const struct pdraw_demuxer_params *params)
{
	int ret;
	IPdraw::IDemuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	ret = pdrawInternal->createDemuxer(localAddr,
					   localStreamPort,
					   localControlPort,
					   remoteAddr,
					   remoteStreamPort,
					   remoteControlPort,
					   params,
					   this,
					   &demuxer);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	mDemuxer.reset(demuxer);
	return 0;
}


int QPdrawDemuxerPriv::open(const std::string &url,
			    struct mux_ctx *mux,
			    const struct pdraw_demuxer_params *params)
{
	int ret;
	IPdraw::IDemuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	ret = pdrawInternal->createDemuxer(url, mux, params, this, &demuxer);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	mDemuxer.reset(demuxer);
	return 0;
}


int QPdrawDemuxerPriv::close()
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);

	int res = mDemuxer->close();
	if (res == 0)
		mClosing = true;
	return res;
}


int QPdrawDemuxerPriv::getMediaList(struct pdraw_demuxer_media **mediaList,
				    size_t *mediaCount,
				    uint32_t *selectedMedias)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getMediaList(mediaList, mediaCount, selectedMedias);
}


int QPdrawDemuxerPriv::selectMedia(uint32_t selectedMedias)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->selectMedia(selectedMedias);
}


uint16_t QPdrawDemuxerPriv::getSingleStreamLocalStreamPort()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getSingleStreamLocalStreamPort();
}


uint16_t QPdrawDemuxerPriv::getSingleStreamLocalControlPort()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getSingleStreamLocalControlPort();
}


bool QPdrawDemuxerPriv::isReadyToPlay()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, false);

	return mDemuxer->isReadyToPlay();
}


bool QPdrawDemuxerPriv::isPaused()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, false);

	return mDemuxer->isPaused();
}


int QPdrawDemuxerPriv::play(float speed)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->play(speed);
}


int QPdrawDemuxerPriv::pause()
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->pause();
}


int QPdrawDemuxerPriv::previousFrame()
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->previousFrame();
}


int QPdrawDemuxerPriv::nextFrame()
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->nextFrame();
}


int QPdrawDemuxerPriv::seek(int64_t delta, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->seek(delta, exact);
}


int QPdrawDemuxerPriv::seekForward(uint64_t delta, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->seekForward(delta, exact);
}


int QPdrawDemuxerPriv::seekBack(uint64_t delta, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->seekBack(delta, exact);
}


int QPdrawDemuxerPriv::seekTo(uint64_t timestamp, bool exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->seekTo(timestamp, exact);
}


int QPdrawDemuxerPriv::getChapterList(struct pdraw_chapter **chapterList,
				      size_t *chapterCount)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->getChapterList(chapterList, chapterCount);
}


uint64_t QPdrawDemuxerPriv::getDuration()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getDuration();
}


uint64_t QPdrawDemuxerPriv::getCurrentTime()
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getCurrentTime();
}


void QPdrawDemuxerPriv::demuxerOpenResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->openResponse(status);
}


void QPdrawDemuxerPriv::demuxerCloseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->closeResponse(status);
}


void QPdrawDemuxerPriv::onDemuxerUnrecoverableError(IPdraw *pdraw,
						    IPdraw::IDemuxer *demuxer)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->onUnrecoverableError();
}


int QPdrawDemuxerPriv::demuxerSelectMedia(
	IPdraw *pdraw,
	IPdraw::IDemuxer *demuxer,
	const struct pdraw_demuxer_media *medias,
	size_t count,
	uint32_t selectedMedias)
{
	int ret = -ENOSYS;

	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->demuxerSelectMedia(
		medias, (unsigned int)count, selectedMedias, &ret);

	return ret;
}


void QPdrawDemuxerPriv::demuxerReadyToPlay(IPdraw *pdraw,
					   IPdraw::IDemuxer *demuxer,
					   bool ready)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->readyToPlay(ready);
}


void QPdrawDemuxerPriv::onDemuxerEndOfRange(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    uint64_t timestamp)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->onEndOfRange(timestamp);
}


void QPdrawDemuxerPriv::demuxerPlayResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->playResponse(status, timestamp, speed);
}


void QPdrawDemuxerPriv::demuxerPauseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status,
					     uint64_t timestamp)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->pauseResponse(status, timestamp);
}


void QPdrawDemuxerPriv::demuxerSeekResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed)
{
	PDRAW_UNUSED(pdraw);
	PDRAW_UNUSED(demuxer);

	emit mParent->seekResponse(status, timestamp, speed);
}

} /* namespace Internal */


QPdrawDemuxer::QPdrawDemuxer(QPdraw *parent) :
		QObject(parent),
		mPriv(make_unique<Internal::QPdrawDemuxerPriv>(this))
{
	qRegisterMetaType<pdraw_demuxer_media>("pdraw_demuxer_media");
}


/* The destructor must be defined here because QPdrawDemuxerPriv is an
 * incomplete type in the header (PIMPL idiom) */
QPdrawDemuxer::~QPdrawDemuxer() = default;


int QPdrawDemuxer::open(const std::string &url,
			const struct pdraw_demuxer_params *params)
{
	return mPriv->open(url, params);
}


int QPdrawDemuxer::open(const std::string &localAddr,
			uint16_t localStreamPort,
			uint16_t localControlPort,
			const std::string &remoteAddr,
			uint16_t remoteStreamPort,
			uint16_t remoteControlPort,
			const struct pdraw_demuxer_params *params)
{
	return mPriv->open(localAddr,
			   localStreamPort,
			   localControlPort,
			   remoteAddr,
			   remoteStreamPort,
			   remoteControlPort,
			   params);
}


int QPdrawDemuxer::open(const std::string &url,
			struct mux_ctx *mux,
			const struct pdraw_demuxer_params *params)
{
	return mPriv->open(url, mux, params);
}


int QPdrawDemuxer::close()
{
	return mPriv->close();
}


int QPdrawDemuxer::getMediaList(struct pdraw_demuxer_media **mediaList,
				size_t *mediaCount,
				uint32_t *selectedMedias)
{
	return mPriv->getMediaList(mediaList, mediaCount, selectedMedias);
}


int QPdrawDemuxer::selectMedia(uint32_t selectedMedias)
{
	return mPriv->selectMedia(selectedMedias);
}


uint16_t QPdrawDemuxer::getSingleStreamLocalStreamPort()
{
	return mPriv->getSingleStreamLocalStreamPort();
}


uint16_t QPdrawDemuxer::getSingleStreamLocalControlPort()
{
	return mPriv->getSingleStreamLocalControlPort();
}


bool QPdrawDemuxer::isReadyToPlay()
{
	return mPriv->isReadyToPlay();
}


bool QPdrawDemuxer::isPaused()
{
	return mPriv->isPaused();
}


int QPdrawDemuxer::play(float speed)
{
	return mPriv->play(speed);
}


int QPdrawDemuxer::pause()
{
	return mPriv->pause();
}


int QPdrawDemuxer::previousFrame()
{
	return mPriv->previousFrame();
}


int QPdrawDemuxer::nextFrame()
{
	return mPriv->nextFrame();
}


int QPdrawDemuxer::seek(int64_t delta, bool exact)
{
	return mPriv->seek(delta, exact);
}


int QPdrawDemuxer::seekForward(uint64_t delta, bool exact)
{
	return mPriv->seekForward(delta, exact);
}


int QPdrawDemuxer::seekBack(uint64_t delta, bool exact)
{
	return mPriv->seekBack(delta, exact);
}


int QPdrawDemuxer::seekTo(uint64_t timestamp, bool exact)
{
	return mPriv->seekTo(timestamp, exact);
}


int QPdrawDemuxer::getChapterList(struct pdraw_chapter **chapterList,
				  size_t *chapterCount)
{
	return mPriv->getChapterList(chapterList, chapterCount);
}


uint64_t QPdrawDemuxer::getDuration()
{
	return mPriv->getDuration();
}


uint64_t QPdrawDemuxer::getCurrentTime()
{
	return mPriv->getCurrentTime();
}

} /* namespace QPdraw */
