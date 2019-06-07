/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline source to sink channel for raw video
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

#ifndef _PDRAW_CHANNEL_RAW_VIDEO_HPP_
#define _PDRAW_CHANNEL_RAW_VIDEO_HPP_

#include <inttypes.h>

#include <libpomp.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>

namespace Pdraw {

class RawChannel {
public:
	friend class RawSource;
	friend class RawSink;
	friend class VideoDecoder;
	friend class VideoEncoder;
	friend class VideoScaler;
	friend class Gles2Renderer;
	friend class ExternalRawVideoSink;

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

		/* reception timeout */
		TIMEOUT,

		/* photo trigger */
		PHOTO_TRIGGER,
	};

	enum UpstreamEvent {
		/* unlink required */
		UNLINK,

		/* flush completed */
		FLUSHED,

		/* resynchronization required */
		RESYNC,
	};

	class SinkListener {
	public:
		virtual ~SinkListener(void) {}

		virtual void
		onChannelQueue(RawChannel *channel,
			       struct mbuf_raw_video_frame *frame) = 0;

		virtual void
		onChannelDownstreamEvent(RawChannel *channel,
					 struct pomp_msg *event) = 0;
	};

	class SourceListener {
	public:
		virtual ~SourceListener(void) {}

		virtual void onChannelUpstreamEvent(RawChannel *channel,
						    struct pomp_msg *event) = 0;
	};

	static const char *getDownstreamEventStr(DownstreamEvent val);

	static const char *getUpstreamEventStr(UpstreamEvent val);

	RawChannel(SinkListener *sinkListener);

	~RawChannel(void) {}

	int queue(mbuf_raw_video_frame *frame);

	int flush(void);

	bool isFlushPending(void)
	{
		return mFlushPending;
	}

	int flushDone(void);

	int resync(void);

	int unlink(void);

	int teardown(void);

	int sendDownstreamEvent(DownstreamEvent downstreamEvent);

	SourceListener *getSourceListener(void)
	{
		return mSourceListener;
	}

	void setSourceListener(SourceListener *sourceListener)
	{
		mSourceListener = sourceListener;
	}

	void *getKey(void)
	{
		return mKey;
	}

	int getRawVideoMediaFormatCaps(const struct vdef_raw_format **caps)
	{
		if (!caps)
			return -EINVAL;
		*caps = mRawVideoMediaFormatCaps;
		return mRawVideoMediaFormatCapsCount;
	}

protected:
	void setKey(void *key)
	{
		mKey = key;
	}

	void setRawVideoMediaFormatCaps(const struct vdef_raw_format *caps,
					int count)
	{
		mRawVideoMediaFormatCaps = caps;
		mRawVideoMediaFormatCapsCount = count;
	}

	struct mbuf_raw_video_frame_queue *getQueue(void)
	{
		return mQueue;
	}

	void setQueue(struct mbuf_raw_video_frame_queue *queue)
	{
		mQueue = queue;
	}

	struct mbuf_pool *getPool(void)
	{
		return mPool;
	}

	void setPool(struct mbuf_pool *pool)
	{
		mPool = pool;
	}

private:
	SinkListener *mSinkListener;
	SourceListener *mSourceListener;
	void *mKey;
	const struct vdef_raw_format *mRawVideoMediaFormatCaps;
	int mRawVideoMediaFormatCapsCount;
	struct mbuf_raw_video_frame_queue *mQueue;
	struct mbuf_pool *mPool;
	bool mFlushPending;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_CHANNEL_RAW_VIDEO_HPP_ */
