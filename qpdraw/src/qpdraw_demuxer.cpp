/**
 * Parrot Drones Awesome Video Viewer
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

#define ULOG_TAG qpdraw_demuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


Q_DECLARE_METATYPE(struct pdraw_demuxer_media);


namespace QPdraw {
namespace Internal {


QPdrawDemuxerPriv::QPdrawDemuxerPriv(QPdrawDemuxer *parent) :
		mParent(parent), mDemuxer(nullptr), mClosing(false)
{
}


QPdrawDemuxerPriv::~QPdrawDemuxerPriv()
{
	if (mDemuxer != nullptr)
		delete mDemuxer;
}


IPdraw *QPdrawDemuxerPriv::getPdrawInternal()
{
	ULOG_ERRNO_RETURN_VAL_IF(mParent == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mParent->parent() == nullptr, EPROTO, nullptr);
	QPdraw *qpdraw = reinterpret_cast<QPdraw *>(mParent->parent());
	ULOG_ERRNO_RETURN_VAL_IF(qpdraw == nullptr, EPROTO, nullptr);
	return reinterpret_cast<IPdraw *>(qpdraw->getInternal());
}


int QPdrawDemuxerPriv::open(const std::string &url)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	return pdrawInternal->createDemuxer(url, this, &mDemuxer);
}


int QPdrawDemuxerPriv::open(const std::string &localAddr,
			    uint16_t localStreamPort,
			    uint16_t localControlPort,
			    const std::string &remoteAddr,
			    uint16_t remoteStreamPort,
			    uint16_t remoteControlPort)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	return pdrawInternal->createDemuxer(localAddr,
					    localStreamPort,
					    localControlPort,
					    remoteAddr,
					    remoteStreamPort,
					    remoteControlPort,
					    this,
					    &mDemuxer);
}


int QPdrawDemuxerPriv::open(const std::string &url, struct mux_ctx *mux)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer != nullptr, EBUSY);

	IPdraw *pdrawInternal = getPdrawInternal();
	ULOG_ERRNO_RETURN_ERR_IF(pdrawInternal == nullptr, EPROTO);

	return pdrawInternal->createDemuxer(url, mux, this, &mDemuxer);
}


int QPdrawDemuxerPriv::close(void)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);

	int res = mDemuxer->close();
	if (res == 0)
		mClosing = true;
	return res;
}


uint16_t QPdrawDemuxerPriv::getSingleStreamLocalStreamPort(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getSingleStreamLocalStreamPort();
}


uint16_t QPdrawDemuxerPriv::getSingleStreamLocalControlPort(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getSingleStreamLocalControlPort();
}


bool QPdrawDemuxerPriv::isReadyToPlay(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, false);

	return mDemuxer->isReadyToPlay();
}


bool QPdrawDemuxerPriv::isPaused(void)
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


int QPdrawDemuxerPriv::pause(void)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->pause();
}


int QPdrawDemuxerPriv::previousFrame(void)
{
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mClosing, EPERM);

	return mDemuxer->previousFrame();
}


int QPdrawDemuxerPriv::nextFrame(void)
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


uint64_t QPdrawDemuxerPriv::getDuration(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getDuration();
}


uint64_t QPdrawDemuxerPriv::getCurrentTime(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mClosing, EPERM, 0);

	return mDemuxer->getCurrentTime();
}


void QPdrawDemuxerPriv::demuxerOpenResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->openResponse(status);
}


void QPdrawDemuxerPriv::demuxerCloseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->closeResponse(status);
}


void QPdrawDemuxerPriv::onDemuxerUnrecoverableError(IPdraw *pdraw,
						    IPdraw::IDemuxer *demuxer)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->onUnrecoverableError();
}


int QPdrawDemuxerPriv::demuxerSelectMedia(
	IPdraw *pdraw,
	IPdraw::IDemuxer *demuxer,
	const struct pdraw_demuxer_media *medias,
	size_t count)
{
	int ret = -ENOSYS;

	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->selectMedia(medias, (unsigned int)count, &ret);

	return ret;
}


void QPdrawDemuxerPriv::demuxerReadyToPlay(IPdraw *pdraw,
					   IPdraw::IDemuxer *demuxer,
					   bool ready)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->readyToPlay(ready);
}


void QPdrawDemuxerPriv::onDemuxerEndOfRange(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    uint64_t timestamp)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->onEndOfRange(timestamp);
}


void QPdrawDemuxerPriv::demuxerPlayResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->playResponse(status, timestamp, speed);
}


void QPdrawDemuxerPriv::demuxerPauseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status,
					     uint64_t timestamp)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->pauseResponse(status, timestamp);
}


void QPdrawDemuxerPriv::demuxerSeekResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed)
{
	Q_UNUSED(pdraw);
	Q_UNUSED(demuxer);

	emit mParent->seekResponse(status, timestamp, speed);
}

} /* namespace Internal */


QPdrawDemuxer::QPdrawDemuxer(QPdraw *parent) : QObject(parent)
{
	qRegisterMetaType<pdraw_demuxer_media>("pdraw_demuxer_media");

	mPriv = new Internal::QPdrawDemuxerPriv(this);
}


QPdrawDemuxer::~QPdrawDemuxer()
{
	delete mPriv;
}


int QPdrawDemuxer::open(const std::string &url)
{
	return mPriv->open(url);
}


int QPdrawDemuxer::open(const std::string &localAddr,
			uint16_t localStreamPort,
			uint16_t localControlPort,
			const std::string &remoteAddr,
			uint16_t remoteStreamPort,
			uint16_t remoteControlPort)
{
	return mPriv->open(localAddr,
			   localStreamPort,
			   localControlPort,
			   remoteAddr,
			   remoteStreamPort,
			   remoteControlPort);
}


int QPdrawDemuxer::open(const std::string &url, struct mux_ctx *mux)
{
	return mPriv->open(url, mux);
}


int QPdrawDemuxer::close(void)
{
	return mPriv->close();
}


uint16_t QPdrawDemuxer::getSingleStreamLocalStreamPort(void)
{
	return mPriv->getSingleStreamLocalStreamPort();
}


uint16_t QPdrawDemuxer::getSingleStreamLocalControlPort(void)
{
	return mPriv->getSingleStreamLocalControlPort();
}


bool QPdrawDemuxer::isReadyToPlay(void)
{
	return mPriv->isReadyToPlay();
}


bool QPdrawDemuxer::isPaused(void)
{
	return mPriv->isPaused();
}


int QPdrawDemuxer::play(float speed)
{
	return mPriv->play(speed);
}


int QPdrawDemuxer::pause(void)
{
	return mPriv->pause();
}


int QPdrawDemuxer::previousFrame(void)
{
	return mPriv->previousFrame();
}


int QPdrawDemuxer::nextFrame(void)
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


uint64_t QPdrawDemuxer::getDuration(void)
{
	return mPriv->getDuration();
}


uint64_t QPdrawDemuxer::getCurrentTime(void)
{
	return mPriv->getCurrentTime();
}

} /* namespace QPdraw */
