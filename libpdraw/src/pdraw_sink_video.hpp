/**
 * Parrot Drones Awesome Video Viewer Library
 * Application video sink
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

#ifndef _PDRAW_SINK_VIDEO_HPP_
#define _PDRAW_SINK_VIDEO_HPP_

#include "pdraw_element.hpp"
#include "pdraw_sink.hpp"

#include <inttypes.h>

#include <pdraw/pdraw.hpp>
#include <video-buffers/vbuf.h>

namespace Pdraw {


class VideoSink : public Element, public Sink {
public:
	VideoSink(Session *session,
		  unsigned int requiredFormat,
		  Element::Listener *elementListener);

	~VideoSink(void);

	int setup(IPdraw::VideoSinkListener *listener,
		  const struct pdraw_video_sink_params *params);

	int start(void);

	int stop(void);

	int resync(void);


	int flushDone(void);

	struct vbuf_queue *getQueue(void)
	{
		return mInputBufferQueue;
	}

	IPdraw::VideoSinkListener *getVideoSinkListener(void)
	{
		return mVideoSinkListener;
	}

private:
	int flush(void);

	int channelTeardown(Channel *channel);

	void onChannelQueue(Channel *channel, vbuf_buffer *buf);

	void onChannelFlush(Channel *channel);

	void onChannelTeardown(Channel *channel);

	IPdraw::VideoSinkListener *mVideoSinkListener;
	struct pdraw_video_sink_params mParams;
	VideoMedia *mInputMedia;
	struct vbuf_queue *mInputBufferQueue;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mTearingDown;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SINK_VIDEO_HPP_ */
