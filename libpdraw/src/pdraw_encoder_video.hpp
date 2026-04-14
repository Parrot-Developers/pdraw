/**
 * Parrot Drones Audio and Video Vector library
 * Video encoder element
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

#include <mutex>
#include <string>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_queue.hpp>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>
#include <video-encode/venc.h>

namespace Pdraw {

class VideoEncoderWrapper;


class VideoEncoder : public FilterElement {
public:
	VideoEncoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener,
		     IPdraw::IVideoEncoder::Listener *listener,
		     VideoEncoderWrapper *wrapper,
		     const struct venc_config *params);

	~VideoEncoder() override;

	int start() override;

	int stop() override;

	void completeFlush();

	void completeStop();

	int configure(const struct venc_dyn_config *config);

	int getConfig(struct venc_dyn_config *config);

	int requestKeyFrame();

	IPdraw::IVideoEncoder *getVideoEncoder() const
	{
		return mEncoder;
	}

private:
	int createOutputMedia(const struct vdef_coded_frame *frame_info,
			      const CodedVideoMedia::Frame &frame);

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	int tryStop();

	void removeEncoderListener();

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

	static void frameOutputCb(struct venc_encoder *enc,
				  int status,
				  struct mbuf_coded_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct venc_encoder *enc, void *userdata);

	static void stopCb(struct venc_encoder *enc, void *userdata);

	/* Can be called from any thread */
	static void framePreReleaseCb(struct mbuf_coded_video_frame *frame,
				      void *userdata);

	static void idleCompleteFlush(void *userdata);

	IPdraw::IVideoEncoder *mEncoder = nullptr;
	IPdraw::IVideoEncoder::Listener *mEncoderListener = nullptr;
	std::mutex mListenerMutex{};
	RawVideoMedia *mInputMedia = nullptr;
	std::unique_ptr<CodedVideoMedia> mOutputMedia{};
	struct mbuf_pool *mInputBufferPool = nullptr;
	std::unique_ptr<mbuf::Queue> mInputBufferQueue;
	struct venc_config *mEncoderConfig = nullptr;
	std::string mEncoderName{};
	std::string mEncoderDevice{};
	struct venc_encoder *mVenc = nullptr;
	bool mInputChannelFlushPending = false;
	bool mOutputChannelDrainRequired = false;
	bool mVencFlushPending = false;
	bool mVencStopPending = false;
	static const struct venc_cbs mEncoderCbs;
};


class VideoEncoderWrapper : public IPdraw::IVideoEncoder,
			    public ElementWrapper {
public:
	VideoEncoderWrapper(Session *session,
			    const struct venc_config *params,
			    IPdraw::IVideoEncoder::Listener *listener);

	~VideoEncoderWrapper() override;

	int configure(const struct venc_dyn_config *config) override;

	int getConfig(struct venc_dyn_config *config) override;

	int requestKeyFrame() override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mEncoder = nullptr;
	}

	Sink *getEncoder() const
	{
		return mEncoder;
	}

	VideoEncoder *getVideoEncoder() const
	{
		return mEncoder;
	}

private:
	bool isElementStopped() const override
	{
		return (ElementWrapper::isElementStopped() ||
			mEncoder == nullptr);
	}

	VideoEncoder *mEncoder = nullptr;
};

} /* namespace Pdraw */
