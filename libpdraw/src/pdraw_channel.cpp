/**
 * Parrot Drones Audio and Video Vector library
 * Pipeline source to sink channel
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

#define ULOG_TAG pdraw_channel
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_channel.hpp"
#include "pdraw_media.hpp"
#include "pdraw_sink.hpp"

#include <errno.h>

namespace Pdraw {


Channel::Channel(Sink *owner,
		 SinkListener *sinkListener,
		 struct pomp_loop *loop) :
		mOwner(owner),
		mSinkListener(sinkListener), mSourceListener(nullptr),
		mPool(nullptr), mLoop(loop), mFlushPending(false)
{
}


Channel::~Channel(void)
{
	/* Remove any leftover idle callbacks */
	if (mLoop != nullptr) {
		int err = pomp_loop_idle_remove_by_cookie(mLoop, this);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
	}
}


struct mbuf_pool *Channel::getPool(const Sink *owner) const
{
	if (owner != mOwner) {
		ULOGE("Channel::getPool: wrong owner");
		return nullptr;
	}
	return mPool;
}


void Channel::setPool(const Sink *owner, struct mbuf_pool *pool)
{
	if (owner != mOwner) {
		ULOGE("Channel::setPool: wrong owner");
		return;
	}
	mPool = pool;
}


int Channel::flush(void)
{
	int res;

	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, DownstreamEvent::FLUSH, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mFlushPending = true;
	mSinkListener->onChannelDownstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::flushDone(void)
{
	int res;

	if (!mFlushPending)
		return 0;

	mFlushPending = false;
	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, UpstreamEvent::FLUSHED, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mSourceListener->onChannelUpstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::asyncFlushDone(void)
{
	if (mLoop == nullptr) {
		ULOGE("invalid loop");
		return -EPROTO;
	}

	int ret = pomp_loop_idle_add_with_cookie(
		mLoop, &idleFlushDone, this, this);
	if (ret < 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return ret;
}


void Channel::idleFlushDone(void *userdata)
{
	Channel *self = (Channel *)userdata;
	(void)self->flushDone();
}


int Channel::resync(void)
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, UpstreamEvent::RESYNC, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mSourceListener->onChannelUpstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::teardown(void)
{
	int res;

	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, DownstreamEvent::TEARDOWN, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mSinkListener->onChannelDownstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::unlink(void)
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, UpstreamEvent::UNLINK, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mSourceListener->onChannelUpstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::sendVideoPresStats(VideoPresStats *stats)
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = stats->writeMsg(event, UpstreamEvent::VIDEO_PRES_STATS);
	if (res < 0) {
		ULOG_ERRNO("stats->writeMsg", -res);
		goto out;
	}

	mSourceListener->onChannelUpstreamEvent(this, event);

out:
	int err = pomp_msg_destroy(event);
	if (err < 0)
		ULOG_ERRNO("pomp_msg_destroy", -err);

	return res;
}


int Channel::sendDownstreamEvent(DownstreamEvent downstreamEvent)
{
	int res;

	if ((downstreamEvent == DownstreamEvent::FLUSH) ||
	    (downstreamEvent == DownstreamEvent::TEARDOWN)) {
		ULOGE("invalid event");
		return -EPROTO;
	}
	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, downstreamEvent, nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mSinkListener->onChannelDownstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


const char *Channel::getDownstreamEventStr(DownstreamEvent val)
{
	switch (val) {
	case FLUSH:
		return "FLUSH";
	case TEARDOWN:
		return "TEARDOWN";
	case SOS:
		return "SOS";
	case EOS:
		return "EOS";
	case RECONFIGURE:
		return "RECONFIGURE";
	case RESOLUTION_CHANGE:
		return "RESOLUTION_CHANGE";
	case FRAMERATE_CHANGE:
		return "FRAMERATE_CHANGE";
	case TIMEOUT:
		return "TIMEOUT";
	case PHOTO_TRIGGER:
		return "PHOTO_TRIGGER";
	case SESSION_META_UPDATE:
		return "SESSION_META_UPDATE";
	default:
		return nullptr;
	}
}


const char *Channel::getUpstreamEventStr(UpstreamEvent val)
{
	switch (val) {
	case UNLINK:
		return "UNLINK";
	case FLUSHED:
		return "FLUSHED";
	case RESYNC:
		return "RESYNC";
	case VIDEO_PRES_STATS:
		return "VIDEO_PRES_STATS";
	default:
		return nullptr;
	}
}

} /* namespace Pdraw */
