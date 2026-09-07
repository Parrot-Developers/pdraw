/**
 * Parrot Drones Audio and Video Vector library
 * Video scaler element
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
#include <memory>

#include <media-buffers/mbuf_queue.hpp>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>
#include <video-scale/vscale.h>

namespace Pdraw {

class VideoScalerWrapper;


class VideoScaler : public FilterElement {
public:
	VideoScaler(Session *session,
		    Element::Listener *elementListener,
		    Source::Listener *sourceListener,
		    IPdraw::IVideoScaler::Listener *listener,
		    VideoScalerWrapper *wrapper,
		    const struct vscale_config *params);

	~VideoScaler() override;

	int start() override;

	int stop() override;

	void completeFlush();

	void completeStop();

	IPdraw::IVideoScaler *getVideoScaler() const
	{
		return mScaler;
	}

private:
	int createOutputMedia(const struct vdef_raw_frame *frameInfo,
			      const RawVideoMedia::Frame &frame);

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	int tryStop();

	void
	onRawVideoChannelQueue(RawVideoChannel *channel,
			       struct mbuf_raw_video_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	static void frameOutputCb(struct vscale_scaler *scaler,
				  int status,
				  struct mbuf_raw_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct vscale_scaler *scaler, void *userdata);

	static void stopCb(struct vscale_scaler *scaler, void *userdata);

	void idleCompleteFlush();

	IPdraw::IVideoScaler *mScaler{};
	IPdraw::IVideoScaler::Listener *mScalerListener{};
	RawVideoMedia *mInputMedia{};
	std::unique_ptr<RawVideoMedia> mOutputMedia{};
	struct mbuf_pool *mInputBufferPool{};
	std::unique_ptr<mbuf::Queue> mInputBufferQueue;
	unique_c_ptr<vscale_config> mScalerConfig;
	std::string mScalerName{};
	struct vscale_scaler *mVscale{};
	bool mInputChannelFlushPending{};
	bool mOutputChannelDrainRequired{};
	bool mVscaleFlushPending{};
	bool mVscaleStopPending{};
	bool mVscaleStopIssued{};
	pomp::Loop::IdleHandlerFunc mCompleteFlushHandler;
	static const struct vscale_cbs mScalerCbs;
};


class VideoScalerWrapper : public IPdraw::IVideoScaler, public ElementWrapper {
public:
	VideoScalerWrapper(Session *session,
			   const struct vscale_config *params,
			   IPdraw::IVideoScaler::Listener *listener);

	~VideoScalerWrapper() override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mScaler = nullptr;
	}

	Sink *getScaler() const
	{
		return mScaler;
	}

	VideoScaler *getVideoScaler() const
	{
		return mScaler;
	}

private:
	bool isElementStopped() const final
	{
		return (ElementWrapper::isElementStopped() ||
			mScaler == nullptr);
	}

	VideoScaler *mScaler{};
};

} /* namespace Pdraw */
