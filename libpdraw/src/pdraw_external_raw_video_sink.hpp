/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_queue.hpp>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class RawVideoSinkWrapper;


class ExternalRawVideoSink : public SinkElement {
public:
	ExternalRawVideoSink(Session *session,
			     Element::Listener *elementListener,
			     IPdraw::IRawVideoSink::Listener *listener,
			     RawVideoSinkWrapper *wrapper,
			     unsigned int mediaId,
			     const struct pdraw_video_sink_params *params);

	~ExternalRawVideoSink() override;

	int start() override;

	int stop() override;

	int setMediaId(unsigned int mediaId);

	unsigned int getMediaId() const;

	int flushDone(bool discard = true);

	inline int drainDone()
	{
		return flushDone(false);
	}

	mbuf::Queue *getQueue() const
	{
		return mInputFrameQueue.get();
	}

	IPdraw::IRawVideoSink *getVideoSink() const
	{
		return mVideoSink;
	}

	int addInputMedia(Media *media) override;

	int removeInputMedia(Media *media) override;

private:
	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	int channelTeardown(RawVideoChannel *channel);

	void
	onRawVideoChannelQueue(RawVideoChannel *channel,
			       struct mbuf_raw_video_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	void onChannelReconfigure(Channel *channel) override;

	void onChannelResolutionChange(Channel *channel) override;

	void onChannelFramerateChange(Channel *channel) override;

	int prepareRawVideoFrame(const RawVideoChannel *channel,
				 struct mbuf_raw_video_frame *frame);

	static void idleFlushDone(void *userdata);

	/* Video sink listener calls from idle functions */
	static void callVideoSinkFlush(void *userdata);

	static void idleRenewMedia(void *userdata);

	IPdraw::IRawVideoSink *mVideoSink = nullptr;
	IPdraw::IRawVideoSink::Listener *mVideoSinkListener = nullptr;
	struct pdraw_video_sink_params mParams {
	};
	RawVideoMedia *mInputMedia = nullptr;
	struct pdraw_media_info mMediaInfo {
	};
	struct vmeta_session mMediaInfoSessionMeta {
	};
	unsigned int mMediaId = 0;
	unsigned int mTargetMediaId = 0;
	std::unique_ptr<mbuf::Queue> mInputFrameQueue;
	bool mInputChannelFlushPending = false;
	bool mTearingDown = false;
	bool mPendingRestart = false;
};


class RawVideoSinkWrapper : public IPdraw::IRawVideoSink,
			    public ElementWrapper {
public:
	RawVideoSinkWrapper(Session *session,
			    unsigned int mediaId,
			    const struct pdraw_video_sink_params *params,
			    IPdraw::IRawVideoSink::Listener *listener);

	~RawVideoSinkWrapper() override;

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId() override;

	struct mbuf_raw_video_frame_queue *getQueue() override;

	int queueFlushed() override;

	int queueDrained() override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mSink = nullptr;
	}

	Sink *getSink() const
	{
		return mSink;
	}

	ExternalRawVideoSink *getRawVideoSink() const
	{
		return mSink;
	}

private:
	bool isElementStopped() const override
	{
		return (ElementWrapper::isElementStopped() || mSink == nullptr);
	}

	ExternalRawVideoSink *mSink = nullptr;
};

} /* namespace Pdraw */
