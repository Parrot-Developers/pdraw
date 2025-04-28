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

#ifndef _PDRAW_CHANNEL_HPP_
#define _PDRAW_CHANNEL_HPP_

#include <inttypes.h>

#include <libpomp.h>
#include <media-buffers/mbuf_mem.h>

#include "pdraw_video_pres_stats.hpp"

namespace Pdraw {

class Sink;

class Channel {
public:
	enum DownstreamEvent {
		/* flush required */
		FLUSH,

		/* teardown required */
		TEARDOWN,

		/* start of stream */
		SOS,

		/* end of stream */
		EOS,

		/* reconfiguration in progress */
		RECONFIGURE,

		/* resolution change in progress */
		RESOLUTION_CHANGE,

		/* framerate change in progress */
		FRAMERATE_CHANGE,

		/* reception timeout */
		TIMEOUT,

		/* photo trigger */
		PHOTO_TRIGGER,

		/* session metadata update */
		SESSION_META_UPDATE,
	};

	enum UpstreamEvent {
		/* unlink required */
		UNLINK,

		/* flush completed */
		FLUSHED,

		/* resynchronization required */
		RESYNC,

		/* video presentation statistics */
		VIDEO_PRES_STATS,
	};

	class SinkListener {
	public:
		virtual ~SinkListener(void) {}

		virtual void
		onChannelDownstreamEvent(Channel *channel,
					 const struct pomp_msg *event) = 0;
	};

	class SourceListener {
	public:
		virtual ~SourceListener(void) {}

		virtual void
		onChannelUpstreamEvent(Channel *channel,
				       const struct pomp_msg *event) = 0;
	};

	static const char *getDownstreamEventStr(DownstreamEvent val);

	static const char *getUpstreamEventStr(UpstreamEvent val);

	Channel(Sink *owner,
		SinkListener *sinkListener,
		struct pomp_loop *loop);

	virtual ~Channel(void) = 0;

	int flush(void);

	bool isFlushPending(void)
	{
		return mFlushPending;
	}

	int flushDone(void);

	int asyncFlushDone(void);

	int resync(void);

	int unlink(void);

	int teardown(void);

	int sendVideoPresStats(VideoPresStats *stats);

	int sendDownstreamEvent(DownstreamEvent downstreamEvent);

	SourceListener *getSourceListener(void) const
	{
		return mSourceListener;
	}

	void setSourceListener(SourceListener *sourceListener)
	{
		mSourceListener = sourceListener;
	}

	Sink *getOwner(void) const
	{
		return mOwner;
	}

	struct mbuf_pool *getPool(const Sink *owner) const;

	void setPool(const Sink *owner, struct mbuf_pool *pool);

protected:
	Sink *mOwner;

private:
	static void idleFlushDone(void *userdata);

	SinkListener *mSinkListener;
	SourceListener *mSourceListener;
	struct mbuf_pool *mPool;
	struct pomp_loop *mLoop;
	bool mFlushPending;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_CHANNEL_HPP_ */
