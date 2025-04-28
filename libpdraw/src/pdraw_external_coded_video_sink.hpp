/**
 * Parrot Drones Audio and Video Vector library
 * Application external coded video sink
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

#ifndef _PDRAW_EXTERNAL_CODED_VIDEO_SINK_HPP_
#define _PDRAW_EXTERNAL_CODED_VIDEO_SINK_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class CodedVideoSinkWrapper;


class ExternalCodedVideoSink : public SinkElement {
public:
	ExternalCodedVideoSink(
		Session *session,
		const struct vdef_coded_format *requiredCodedFormat,
		Element::Listener *elementListener,
		IPdraw::ICodedVideoSink::Listener *listener,
		CodedVideoSinkWrapper *wrapper,
		const struct pdraw_video_sink_params *params);

	~ExternalCodedVideoSink(void);

	int start(void) override;

	int stop(void) override;

	int resync(void);


	int flushDone(void);

	struct mbuf_coded_video_frame_queue *getQueue(void) const
	{
		return mInputFrameQueue;
	}

	IPdraw::ICodedVideoSink *getVideoSink(void) const
	{
		return mVideoSink;
	}

private:
	int flush(void);

	int channelTeardown(CodedVideoChannel *channel);

	void
	onCodedVideoChannelQueue(CodedVideoChannel *channel,
				 struct mbuf_coded_video_frame *frame) override;

	void onChannelFlush(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	int prepareCodedVideoFrame(CodedVideoChannel *channel,
				   struct mbuf_coded_video_frame *frame);

	int writeGreyIdr(CodedVideoChannel *channel,
			 struct CodedVideoMedia::Frame *inFrame,
			 struct vdef_coded_frame *inInfo,
			 uint64_t *ntpDelta,
			 uint64_t *ntpUnskewedDelta,
			 uint64_t *ntpRawDelta,
			 uint64_t *ntpRawUnskewedDelta,
			 uint64_t *playDelta);

	static void naluEndCb(struct h264_ctx *ctx,
			      enum h264_nalu_type type,
			      const uint8_t *buf,
			      size_t len,
			      const struct h264_nalu_header *nh,
			      void *userdata);

	static void sliceCb(struct h264_ctx *ctx,
			    const uint8_t *buf,
			    size_t len,
			    const struct h264_slice_header *sh,
			    void *userdata);

	static void spsCb(struct h264_ctx *ctx,
			  const uint8_t *buf,
			  size_t len,
			  const struct h264_sps *sps,
			  void *userdata);

	static void
	seiRecoveryPointCb(struct h264_ctx *ctx,
			   const uint8_t *buf,
			   size_t len,
			   const struct h264_sei_recovery_point *sei,
			   void *userdata);

	static void idleFlushDone(void *userdata);

	/* Video sink listener calls from idle functions */
	static void callVideoSinkFlush(void *userdata);

	IPdraw::ICodedVideoSink *mVideoSink;
	IPdraw::ICodedVideoSink::Listener *mVideoSinkListener;
	struct pdraw_video_sink_params mParams;
	CodedVideoMedia *mInputMedia;
	struct mbuf_coded_video_frame_queue *mInputFrameQueue;
	bool mIsFlushed;
	bool mInputChannelFlushPending;
	bool mTearingDown;
	bool mNeedSync;
	struct h264_reader *mH264Reader;
	static const struct h264_ctx_cbs mH264ReaderCbs;
};


class CodedVideoSinkWrapper : public IPdraw::ICodedVideoSink,
			      public ElementWrapper {
public:
	CodedVideoSinkWrapper(Session *session,
			      const struct pdraw_video_sink_params *params,
			      IPdraw::ICodedVideoSink::Listener *listener);

	~CodedVideoSinkWrapper(void);

	int resync(void) override;

	struct mbuf_coded_video_frame_queue *getQueue(void) override;

	int queueFlushed(void) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mSink = nullptr;
	}

	Sink *getSink() const
	{
		return mSink;
	}

	ExternalCodedVideoSink *getCodedVideoSink() const
	{
		return mSink;
	}

private:
	ExternalCodedVideoSink *mSink;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_EXTERNAL_CODED_VIDEO_SINK_HPP_ */
