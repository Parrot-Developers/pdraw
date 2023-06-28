/**
 * Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_ENCODER_VIDEO_HPP_
#define _PDRAW_ENCODER_VIDEO_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <string>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>
#include <video-encode/venc.h>

namespace Pdraw {

class VideoEncoder : public FilterElement {
public:
	VideoEncoder(Session *session,
		     Element::Listener *elementListener,
		     Source::Listener *sourceListener,
		     IPdraw::IVideoEncoder::Listener *listener,
		     IPdraw::IVideoEncoder *encoder,
		     const struct venc_config *params);

	~VideoEncoder(void);

	int start(void);

	int stop(void);

	void completeFlush(void);

	void completeStop(void);

	int configure(const struct venc_dyn_config *config);

	IPdraw::IVideoEncoder *getVideoEncoder(void)
	{
		return mEncoder;
	}

	IPdraw::IVideoEncoder::Listener *getVideoEncoderListener(void)
	{
		return mEncoderListener;
	}

private:
	int createOutputMedia(struct vdef_coded_frame *frame_info,
			      CodedVideoMedia::Frame &frame);

	int flush(void);

	int tryStop(void);

	void onRawVideoChannelQueue(RawVideoChannel *channel,
				    struct mbuf_raw_video_frame *frame);

	void onChannelFlush(Channel *channel);

	void onChannelFlushed(Channel *channel);

	void onChannelTeardown(Channel *channel);

	void onChannelUnlink(Channel *channel);

	void onChannelSessionMetaUpdate(Channel *channel);

	static void frameOutputCb(struct venc_encoder *enc,
				  int status,
				  struct mbuf_coded_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct venc_encoder *enc, void *userdata);

	static void stopCb(struct venc_encoder *enc, void *userdata);

	static void framePreReleaseCb(struct mbuf_coded_video_frame *frame,
				      void *userdata);

	static void idleCompleteFlush(void *userdata);

	IPdraw::IVideoEncoder *mEncoder;
	IPdraw::IVideoEncoder::Listener *mEncoderListener;
	RawVideoMedia *mInputMedia;
	CodedVideoMedia *mOutputMedia;
	struct mbuf_pool *mInputBufferPool;
	struct mbuf_raw_video_frame_queue *mInputBufferQueue;
	struct venc_config *mEncoderConfig;
	std::string mEncoderName;
	std::string mEncoderDevice;
	struct venc_encoder *mVenc;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mVencFlushPending;
	bool mVencStopPending;
	static const struct venc_cbs mEncoderCbs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ENCODER_VIDEO_HPP_ */
