/**
 * Parrot Drones Awesome Video Viewer Library
 * Generic muxer
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

#ifndef _PDRAW_MUXER_HPP_
#define _PDRAW_MUXER_HPP_

#include "pdraw_element.hpp"
#include "pdraw_sink_raw_video.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <pdraw/pdraw.hpp>

namespace Pdraw {


class Muxer : public CodedSinkElement {
public:
	Muxer(Session *session, Element::Listener *elementListener);

	virtual ~Muxer(void) = 0;

	int start(void);

	int stop(void);

	/* Must be called on the loop thread */
	virtual int addInputMedia(CodedVideoMedia *media);

	/* Must be called on the loop thread */
	virtual int removeInputMedia(CodedVideoMedia *media);

	/* Must be called on the loop thread */
	virtual int removeInputMedias(void);

protected:
	virtual int internalStart(void) = 0;

	virtual int internalStop(void) = 0;

	virtual int process(void) = 0;

	/* Must be called on the loop thread */
	int addQueueEvtToLoop(struct mbuf_coded_video_frame_queue *queue,
			      struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int removeQueueEvtFromLoop(struct mbuf_coded_video_frame_queue *queue,
				   struct pomp_loop *loop);

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	virtual void onChannelTeardown(CodedChannel *channel);
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MUXER_HPP_ */
