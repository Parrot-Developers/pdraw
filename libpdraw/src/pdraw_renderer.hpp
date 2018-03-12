/**
 * Parrot Drones Awesome Video Viewer Library
 * Renderer interface
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

#ifndef _PDRAW_RENDERER_HPP_
#define _PDRAW_RENDERER_HPP_

#include "pdraw_avcdecoder.hpp"

namespace Pdraw {


class Session;
class Media;
class VideoMedia;


class Renderer {
public:
	virtual ~Renderer(
		void) {}

	virtual int open(
		unsigned int windowWidth,
		unsigned int windowHeight,
		int renderX,
		int renderY,
		unsigned int renderWidth,
		unsigned int renderHeight,
		bool hud,
		bool hmdDistorsionCorrection,
		bool headtracking,
		struct egl_display *eglDisplay) = 0;

	virtual int close(
		void) = 0;

	virtual int render(
		int renderX,
		int renderY,
		unsigned int renderWidth,
		unsigned int renderHeight,
		uint64_t timestamp) = 0;

	virtual int addInputSource(
		Media *media) = 0;

	virtual int removeInputSource(
		Media *media) = 0;

	virtual int getInputSourceQueue(
		Media *media,
		struct vbuf_queue **queue) = 0;

	virtual Session *getSession(
		void) = 0;

	virtual Media *getMedia(
		void) = 0;

	virtual VideoMedia *getVideoMedia(
		void) = 0;

	static Renderer *create(
		Session *session);

protected:
	Session *mSession;
	Media *mMedia;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_RENDERER_HPP_ */
