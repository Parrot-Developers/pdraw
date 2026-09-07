/**
 * Parrot Drones Audio and Video Vector library
 * Video decoder element
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

#include <vector>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_queue.hpp>
#include <video-decode/vdec.h>

namespace Pdraw {

class VideoDecoder : public FilterElement {
public:
	VideoDecoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener);

	~VideoDecoder() override;

	int start() override;

	int stop() override;

	void completeFlush();

	void completeStop();

	void resync();

private:
	int createOutputMedia(const struct vdef_raw_frame *frameInfo,
			      const RawVideoMedia::Frame &frame);

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	void completeResync();

	int tryStop();

	void
	onCodedVideoChannelQueue(CodedVideoChannel *channel,
				 struct mbuf_coded_video_frame *buf) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	static void frameOutputCb(struct vdec_decoder *dec,
				  int status,
				  struct mbuf_raw_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct vdec_decoder *dec, void *userdata);

	static void stopCb(struct vdec_decoder *dec, void *userdata);

	void idleCompleteFlush();

	static std::vector<uint8_t>
	preparePsVector(const uint8_t *ps,
			size_t psSize,
			enum vdef_coded_data_format fmt);

	CodedVideoMedia *mInputMedia = nullptr;
	std::unique_ptr<RawVideoMedia> mOutputMedia{};
	struct mbuf_pool *mInputBufferPool = nullptr;
	std::unique_ptr<mbuf::Queue> mInputBufferQueue;
	struct vdec_decoder *mVdec = nullptr;
	bool mInputChannelFlushPending = false;
	bool mOutputChannelDrainRequired = false;
	bool mResyncPending = false;
	bool mVdecFlushPending = false;
	bool mVdecStopPending = false;
	bool mVdecStopIssued = false;
	pomp::Loop::IdleHandlerFunc mCompleteFlushHandler;
	static const struct vdec_cbs mDecoderCbs;
};

} /* namespace Pdraw */
