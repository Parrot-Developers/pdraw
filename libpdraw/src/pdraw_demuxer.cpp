/**
 * Parrot Drones Awesome Video Viewer Library
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
#include "pdraw_session.hpp"

namespace Pdraw {


Demuxer::~Demuxer(void)
{
	/* Remove any leftover idle callbacks */
	pomp_loop_idle_remove(mSession->getLoop(), callOpenResponse, this);
	pomp_loop_idle_remove(mSession->getLoop(), callCloseResponse, this);
	pomp_loop_idle_remove(
		mSession->getLoop(), callOnUnrecoverableError, this);
	pomp_loop_idle_remove(mSession->getLoop(), callReadyToPlay, this);
	pomp_loop_idle_remove(mSession->getLoop(), callEndOfRange, this);
	pomp_loop_idle_remove(mSession->getLoop(), callPlayResponse, this);
	pomp_loop_idle_remove(mSession->getLoop(), callPauseResponse, this);
	pomp_loop_idle_remove(mSession->getLoop(), callSeekResponse, this);
}


void Demuxer::openResponse(int status)
{
	if (mCalledOpenResp) {
		PDRAW_LOGW("multiple openResponse call blocked");
		return;
	}
	mOpenRespStatusArgs.push(status);
	pomp_loop_idle_add(mSession->getLoop(), callOpenResponse, this);
	mCalledOpenResp = true;
}


void Demuxer::closeResponse(int status)
{
	mCloseRespStatusArgs.push(status);
	pomp_loop_idle_add(mSession->getLoop(), callCloseResponse, this);
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

	pomp_loop_idle_add(mSession->getLoop(), callOnUnrecoverableError, this);
}


int Demuxer::selectMedia(const struct pdraw_demuxer_media *medias, size_t count)
{
	if (mDemuxerListener == nullptr)
		return -ENOSYS;

	return mDemuxerListener->demuxerSelectMedia(
		mSession, mDemuxer, medias, count);
}


void Demuxer::readyToPlay(bool ready)
{
	/* Report only changes in value */
	if (mReadyToPlay == ready)
		return;

	mReadyToPlay = ready;

	mReadyToPlayReadyArgs.push(ready);
	pomp_loop_idle_add(mSession->getLoop(), callReadyToPlay, this);
}


void Demuxer::onEndOfRange(uint64_t timestamp)
{
	mEndOfRangeTimestampArgs.push(timestamp);
	pomp_loop_idle_add(mSession->getLoop(), callEndOfRange, this);
}


void Demuxer::playResponse(int status, uint64_t timestamp, float speed)
{
	mPlayRespStatusArgs.push(status);
	mPlayRespTimestampArgs.push(timestamp);
	mPlayRespSpeedArgs.push(speed);
	pomp_loop_idle_add(mSession->getLoop(), callPlayResponse, this);
}


void Demuxer::pauseResponse(int status, uint64_t timestamp)
{
	mPauseRespStatusArgs.push(status);
	mPauseRespTimestampArgs.push(timestamp);
	pomp_loop_idle_add(mSession->getLoop(), callPauseResponse, this);
}


void Demuxer::seekResponse(int status, uint64_t timestamp, float speed)
{
	mSeekRespStatusArgs.push(status);
	mSeekRespTimestampArgs.push(timestamp);
	mSeekRespSpeedArgs.push(speed);
	pomp_loop_idle_add(mSession->getLoop(), callSeekResponse, this);
}


/**
 * Demuxer listener calls from idle functions
 */

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


void Demuxer::callOnUnrecoverableError(void *userdata)
{
	Demuxer *self = reinterpret_cast<Demuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mDemuxerListener == nullptr)
		return;

	self->mDemuxerListener->onDemuxerUnrecoverableError(self->mSession,
							    self->mDemuxer);
}


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

} /* namespace Pdraw */
