/**
 * Parrot Drones Awesome Video Viewer Library
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

#define ULOG_TAG pdraw_channel_raw_video
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_channel_raw_video.hpp"
#include "pdraw_media.hpp"

#include <errno.h>

namespace Pdraw {


RawChannel::RawChannel(SinkListener *sinkListener) :
		mSinkListener(sinkListener), mSourceListener(nullptr),
		mKey(nullptr), mRawVideoMediaFormatCaps(nullptr),
		mRawVideoMediaFormatCapsCount(0), mQueue(nullptr),
		mPool(nullptr), mFlushPending(false)
{
}


int RawChannel::queue(mbuf_raw_video_frame *frame)
{
	if (frame == nullptr)
		return -EINVAL;
	if (mSinkListener == nullptr) {
		ULOGE("invalid sink listener");
		return -EPROTO;
	}

	mSinkListener->onChannelQueue(this, frame);
	return 0;
}


int RawChannel::flush(void)
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


int RawChannel::flushDone(void)
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


int RawChannel::resync(void)
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


int RawChannel::teardown(void)
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


int RawChannel::unlink(void)
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


int RawChannel::sendVideoPresStats(VideoPresStats *stats)
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


int RawChannel::sendDownstreamEvent(DownstreamEvent downstreamEvent)
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


const char *RawChannel::getDownstreamEventStr(DownstreamEvent val)
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
	case TIMEOUT:
		return "TIMEOUT";
	case PHOTO_TRIGGER:
		return "PHOTO_TRIGGER";
	default:
		return nullptr;
	}
}


const char *RawChannel::getUpstreamEventStr(UpstreamEvent val)
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
