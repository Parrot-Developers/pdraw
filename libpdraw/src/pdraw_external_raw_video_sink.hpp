/**
 * Parrot Drones Awesome Video Viewer Library
 * Application external raw video sink
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

#ifndef _PDRAW_EXTERNAL_RAW_VIDEO_SINK_HPP_
#define _PDRAW_EXTERNAL_RAW_VIDEO_SINK_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {


class ExternalRawVideoSink : public RawSinkElement {
public:
	ExternalRawVideoSink(Session *session,
			     Element::Listener *elementListener,
			     IPdraw::IRawVideoSink::Listener *listener,
			     IPdraw::IRawVideoSink *sink,
			     const struct pdraw_video_sink_params *params);

	~ExternalRawVideoSink(void);

	int start(void);

	int stop(void);

	int flushDone(void);

	struct mbuf_raw_video_frame_queue *getQueue(void)
	{
		return mInputFrameQueue;
	}

	IPdraw::IRawVideoSink *getVideoSink(void)
	{
		return mVideoSink;
	}

	IPdraw::IRawVideoSink::Listener *getVideoSinkListener(void)
	{
		return mVideoSinkListener;
	}

private:
	int flush(void);

	int channelTeardown(RawChannel *channel);

	void onChannelQueue(RawChannel *channel,
			    struct mbuf_raw_video_frame *frame);

	void onChannelFlush(RawChannel *channel);

	void onChannelTeardown(RawChannel *channel);

	int prepareRawVideoFrame(RawChannel *channel,
				 struct mbuf_raw_video_frame *frame);

	/* Video sink listener calls from idle functions */
	static void callVideoSinkFlush(void *userdata);

	IPdraw::IRawVideoSink *mVideoSink;
	IPdraw::IRawVideoSink::Listener *mVideoSinkListener;
	struct pdraw_video_sink_params mParams;
	RawVideoMedia *mInputMedia;
	struct mbuf_raw_video_frame_queue *mInputFrameQueue;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mTearingDown;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_EXTERNAL_RAW_VIDEO_SINK_HPP_ */
