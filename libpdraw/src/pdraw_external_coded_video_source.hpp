/**
 * Parrot Drones Awesome Video Viewer Library
 * Application external coded video source
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

#ifndef _PDRAW_EXTERNAL_CODED_VIDEO_SOURCE_HPP_
#define _PDRAW_EXTERNAL_CODED_VIDEO_SOURCE_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {


class ExternalCodedVideoSource : public SourceElement {
public:
	ExternalCodedVideoSource(
		Session *session,
		Element::Listener *elementListener,
		Source::Listener *sourceListener,
		IPdraw::ICodedVideoSource::Listener *listener,
		IPdraw::ICodedVideoSource *source,
		const struct pdraw_video_source_params *params);

	~ExternalCodedVideoSource(void);

	int start(void) override;

	int stop(void) override;

	int flush(void);

	int setSessionMetadata(const struct vmeta_session *meta);

	int getSessionMetadata(struct vmeta_session *meta);

	struct mbuf_coded_video_frame_queue *getQueue(void)
	{
		return mFrameQueue;
	}

	IPdraw::ICodedVideoSource *getVideoSource(void)
	{
		return mVideoSource;
	}

	IPdraw::ICodedVideoSource::Listener *getVideoSourceListener(void)
	{
		return mVideoSourceListener;
	}

private:
	int processFrame(struct mbuf_coded_video_frame *frame);

	void completeFlush(void);

	int tryStop(void);

	void completeStop(void);

	void onChannelFlushed(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	static bool inputFilter(struct mbuf_coded_video_frame *frame,
				void *userdata);

	/* Video source listener calls from idle functions */
	static void callOnMediaAdded(void *userdata);

	static void callVideoSourceFlushed(void *userdata);

	IPdraw::ICodedVideoSource *mVideoSource;
	IPdraw::ICodedVideoSource::Listener *mVideoSourceListener;
	struct pdraw_video_source_params mParams;
	struct mbuf_coded_video_frame_queue *mFrameQueue;
	CodedVideoMedia *mOutputMedia;
	uint64_t mLastTimestamp;
	bool mFlushPending;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_EXTERNAL_CODED_VIDEO_SOURCE_HPP_ */
