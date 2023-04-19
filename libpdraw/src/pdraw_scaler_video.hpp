/**
 * Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_SCALER_VIDEO_HPP_
#define _PDRAW_SCALER_VIDEO_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-scale/vscale.h>

namespace Pdraw {

class VideoScaler : public FilterElement {
public:
	VideoScaler(Session *session,
		    Element::Listener *elementListener,
		    Source::Listener *sourceListener);

	~VideoScaler(void);

	int start(void);

	int stop(void);

	void completeFlush(void);

	void completeStop(void);

private:
	int createOutputMedia(struct vdef_raw_frame *frameInfo,
			      RawVideoMedia::Frame &frame);

	int flush(void);

	int tryStop(void);

	void onRawVideoChannelQueue(RawVideoChannel *channel,
				    struct mbuf_raw_video_frame *frame);

	void onChannelFlush(Channel *channel);

	void onChannelFlushed(Channel *channel);

	void onChannelTeardown(Channel *channel);

	void onChannelUnlink(Channel *channel);

	static void frameOutputCb(struct vscale_scaler *scaler,
				  int status,
				  struct mbuf_raw_video_frame *out_frame,
				  void *userdata);

	static void flushCb(struct vscale_scaler *scaler, void *userdata);

	static void stopCb(struct vscale_scaler *scaler, void *userdata);

	RawVideoMedia *mInputMedia;
	RawVideoMedia *mOutputMedia;
	struct mbuf_pool *mInputBufferPool;
	struct mbuf_raw_video_frame_queue *mInputBufferQueue;
	struct vscale_scaler *mVscale;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mVscaleFlushPending;
	bool mVscaleStopPending;
	static const struct vscale_cbs mScalerCbs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SCALER_VIDEO_HPP_ */
