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

#ifndef _PDRAW_CHANNEL_HPP_
#define _PDRAW_CHANNEL_HPP_

#include <inttypes.h>

#include <libpomp.h>
#include <video-buffers/vbuf.h>

namespace Pdraw {

class Channel {
public:
	friend class Source;
	friend class Sink;
	friend class AvcDecoder;
	friend class RecordDemuxer;
	friend class StreamDemuxer;
	friend class Gles2Renderer;
	friend class VideoSink;

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

		virtual void onChannelQueue(Channel *channel,
					    vbuf_buffer *buf) = 0;

		virtual void
		onChannelDownstreamEvent(Channel *channel,
					 struct pomp_msg *event) = 0;
	};

	class SourceListener {
	public:
		virtual ~SourceListener(void) {}

		virtual void onChannelUpstreamEvent(Channel *channel,
						    struct pomp_msg *event) = 0;
	};

	static const char *getDownstreamEventStr(DownstreamEvent val);

	static const char *getUpstreamEventStr(UpstreamEvent val);

	Channel(SinkListener *sinkListener);

	~Channel(void) {}

	int queue(vbuf_buffer *buf);

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

	uint32_t getMediaTypeCaps(void)
	{
		return mMediaTypeCaps;
	}

	uint32_t getVideoMediaFormatCaps(void)
	{
		return mVideoMediaFormatCaps;
	}

	uint32_t getVideoMediaSubFormatCaps(void)
	{
		return mVideoMediaSubFormatCaps;
	}

protected:
	void setKey(void *key)
	{
		mKey = key;
	}

	void setMediaTypeCaps(uint32_t caps)
	{
		mMediaTypeCaps = caps;
	}

	void setVideoMediaFormatCaps(uint32_t caps)
	{
		mVideoMediaFormatCaps = caps;
	}

	void setVideoMediaSubFormatCaps(uint32_t caps)
	{
		mVideoMediaSubFormatCaps = caps;
	}

	struct vbuf_queue *getQueue(void)
	{
		return mQueue;
	}

	void setQueue(struct vbuf_queue *queue)
	{
		mQueue = queue;
	}

	struct vbuf_pool *getPool(void)
	{
		return mPool;
	}

	void setPool(struct vbuf_pool *pool)
	{
		mPool = pool;
	}

private:
	SinkListener *mSinkListener;
	SourceListener *mSourceListener;
	void *mKey;
	uint32_t mMediaTypeCaps;
	uint32_t mVideoMediaFormatCaps;
	uint32_t mVideoMediaSubFormatCaps;
	struct vbuf_queue *mQueue;
	struct vbuf_pool *mPool;
	bool mFlushPending;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_CHANNEL_HPP_ */
