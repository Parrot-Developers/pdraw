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
		mSinkListener(sinkListener), mLoop(loop)
{
}


Channel::~Channel()
{
	/* Remove any leftover idle callbacks */
	if (mLoop != nullptr) {
		int err = pomp_loop_idle_remove_by_cookie(mLoop, this);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
	}
}


mbuf::Queue *Channel::getQueue(const Sink *owner) const
{
	if (owner != mOwner) {
		ULOGE("Channel::getQueue: wrong owner");
		return nullptr;
	}
	return mQueue;
}


void Channel::setQueue(const Sink *owner, mbuf::Queue *queue)
{
	if (owner != mOwner) {
		ULOGE("Channel::setQueue: wrong owner");
		return;
	}
	mQueue = queue;
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


int Channel::flush()
{
	int res;

	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	/* Flush and drain are mutually exclusive */
	if (mFlushPending || mDrainPending)
		return -EALREADY;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, toMsgId(DownstreamEvent::FLUSH), nullptr);
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


int Channel::flushDone()
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

	res = pomp_msg_write(event, toMsgId(UpstreamEvent::FLUSHED), nullptr);
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


int Channel::asyncFlushDone()
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
	auto *self = static_cast<Channel *>(userdata);
	(void)self->flushDone();
}


int Channel::drain()
{
	int res;

	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	/* Drain and flush are mutually exclusive */
	if (mDrainPending || mFlushPending)
		return -EALREADY;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, toMsgId(DownstreamEvent::DRAIN), nullptr);
	if (res < 0) {
		ULOG_ERRNO("pomp_msg_write", -res);
		return res;
	}

	mDrainPending = true;
	mSinkListener->onChannelDownstreamEvent(this, event);

	res = pomp_msg_destroy(event);
	if (res < 0)
		ULOG_ERRNO("pomp_msg_destroy", -res);

	return 0;
}


int Channel::drainDone()
{
	int res;

	if (!mDrainPending)
		return 0;

	mDrainPending = false;
	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, toMsgId(UpstreamEvent::DRAINED), nullptr);
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


int Channel::asyncDrainDone()
{
	if (mLoop == nullptr) {
		ULOGE("invalid loop");
		return -EPROTO;
	}

	int ret = pomp_loop_idle_add_with_cookie(
		mLoop, &idleDrainDone, this, this);
	if (ret < 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);

	return ret;
}


void Channel::idleDrainDone(void *userdata)
{
	auto *self = static_cast<Channel *>(userdata);
	(void)self->drainDone();
}


int Channel::resync()
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, toMsgId(UpstreamEvent::RESYNC), nullptr);
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


int Channel::teardown()
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

	res = pomp_msg_write(
		event, toMsgId(DownstreamEvent::TEARDOWN), nullptr);
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


int Channel::unlink()
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = pomp_msg_write(event, toMsgId(UpstreamEvent::UNLINK), nullptr);
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


int Channel::sendVideoPresStats(const VideoPresStats *stats)
{
	int res;

	if (mSourceListener == nullptr)
		return 0;

	struct pomp_msg *event = pomp_msg_new();
	if (event == nullptr) {
		ULOG_ERRNO("pomp_msg_new", ENOMEM);
		return -ENOMEM;
	}

	res = stats->writeMsg(event, toMsgId(UpstreamEvent::VIDEO_PRES_STATS));
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
	    (downstreamEvent == DownstreamEvent::DRAIN) ||
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

	res = pomp_msg_write(event, toMsgId(downstreamEvent), nullptr);
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
	case DownstreamEvent::FLUSH:
		return "FLUSH";
	case DownstreamEvent::DRAIN:
		return "DRAIN";
	case DownstreamEvent::TEARDOWN:
		return "TEARDOWN";
	case DownstreamEvent::SOS:
		return "SOS";
	case DownstreamEvent::EOS:
		return "EOS";
	case DownstreamEvent::RECONFIGURE:
		return "RECONFIGURE";
	case DownstreamEvent::RESOLUTION_CHANGE:
		return "RESOLUTION_CHANGE";
	case DownstreamEvent::FRAMERATE_CHANGE:
		return "FRAMERATE_CHANGE";
	case DownstreamEvent::TIMEOUT:
		return "TIMEOUT";
	case DownstreamEvent::PHOTO_TRIGGER:
		return "PHOTO_TRIGGER";
	case DownstreamEvent::SESSION_META_UPDATE:
		return "SESSION_META_UPDATE";
	default:
		return nullptr;
	}
}


const char *Channel::getUpstreamEventStr(UpstreamEvent val)
{
	switch (val) {
	case UpstreamEvent::UNLINK:
		return "UNLINK";
	case UpstreamEvent::FLUSHED:
		return "FLUSHED";
	case UpstreamEvent::DRAINED:
		return "DRAINED";
	case UpstreamEvent::RESYNC:
		return "RESYNC";
	case UpstreamEvent::VIDEO_PRES_STATS:
		return "VIDEO_PRES_STATS";
	default:
		return nullptr;
	}
}

} /* namespace Pdraw */
