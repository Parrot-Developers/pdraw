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

#ifndef _PDRAW_DECODER_VIDEO_HPP_
#define _PDRAW_DECODER_VIDEO_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <video-decode/vdec.h>

namespace Pdraw {

class VideoDecoder : public FilterElement {
public:
	VideoDecoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener);

	~VideoDecoder(void);

	int start(void) override;

	int stop(void) override;

	void completeFlush(void);

	void completeStop(void);

	void resync(void);

private:
	int createOutputMedia(const struct vdef_raw_frame *frameInfo,
			      const RawVideoMedia::Frame &frame);

	int flush(void);

	void completeResync(void);

	int tryStop(void);

	void
	onCodedVideoChannelQueue(CodedVideoChannel *channel,
				 struct mbuf_coded_video_frame *buf) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelFlushed(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	static void frameOutputCb(struct vdec_decoder *dec,
				  int status,
				  struct mbuf_raw_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct vdec_decoder *dec, void *userdata);

	static void stopCb(struct vdec_decoder *dec, void *userdata);

	static void idleCompleteFlush(void *userdata);

	static ssize_t preparePsBuffer(const uint8_t *ps,
				       size_t psSize,
				       enum vdef_coded_data_format fmt,
				       uint8_t **ret);

	CodedVideoMedia *mInputMedia;
	RawVideoMedia *mOutputMedia;
	struct mbuf_pool *mInputBufferPool;
	struct mbuf_coded_video_frame_queue *mInputBufferQueue;
	struct vdec_decoder *mVdec;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mResyncPending;
	bool mVdecFlushPending;
	bool mVdecStopPending;
	static const struct vdec_cbs mDecoderCbs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DECODER_VIDEO_HPP_ */
