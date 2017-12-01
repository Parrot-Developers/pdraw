/**
 * Parrot Drones Awesome Video Viewer Library
 * Null renderer
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_RENDERER_NULL_HPP_
#define _PDRAW_RENDERER_NULL_HPP_

#include <pthread.h>
#include "pdraw_renderer.hpp"

namespace Pdraw {


class NullRenderer : public Renderer {
public:
	NullRenderer(
		Session *session);

	~NullRenderer(
		void);

	int addAvcDecoder(
		AvcDecoder *decoder);

	int removeAvcDecoder(
		AvcDecoder *decoder);

	int setRendererParams(
		int windowWidth,
		int windowHeight,
		int renderX,
		int renderY,
		int renderWidth,
		int renderHeight,
		bool hmdDistorsionCorrection,
		bool headtracking,
		void *uiHandler);

	int render(
		uint64_t lastRenderTime);

	Session *getSession(
		void) {
		return mSession;
	}

	Media *getMedia(
		void) {
		return mMedia;
	}

	VideoMedia *getVideoMedia(
		void) {
		return (VideoMedia *)mMedia;
	}

private:
	static void* runRendererThread(
		void *ptr);

	AvcDecoder *mDecoder;
	struct vbuf_queue *mDecoderOutputBufferQueue;
	pthread_t mRendererThread;
	bool mRendererThreadLaunched;
	bool mThreadShouldStop;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_RENDERER_NULL_HPP_ */
