/**
 * Parrot Drones Awesome Video Viewer Library
 * Demuxer interface
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

#ifndef _PDRAW_DEMUXER_HPP_
#define _PDRAW_DEMUXER_HPP_

#include "pdraw_media.hpp"
#include "pdraw_decoder.hpp"
#include <string>

namespace Pdraw {


enum demuxer_type {
	DEMUXER_TYPE_RECORD = 0,
	DEMUXER_TYPE_STREAM,
};


class Session;


class Demuxer {
public:
	virtual ~Demuxer(
		void) {}

	virtual enum demuxer_type getType(
		void) = 0;

	virtual bool isConfigured(
		void) = 0;

	virtual int close(
		void) = 0;

	virtual int getElementaryStreamCount(
		void) = 0;

	virtual enum elementary_stream_type getElementaryStreamType(
		int esIndex) = 0;

	virtual int getElementaryStreamVideoDimensions(
		int esIndex,
		unsigned int *width,
		unsigned int *height,
		unsigned int *cropLeft,
		unsigned int *cropRight,
		unsigned int *cropTop,
		unsigned int *cropBottom,
		unsigned int *sarWidth,
		unsigned int *sarHeight) = 0;

	virtual int getElementaryStreamVideoFov(
		int esIndex,
		float *hfov,
		float *vfov) = 0;

	virtual int setElementaryStreamDecoder(
		int esIndex,
		Decoder *decoder) = 0;

	virtual int play(
		float speed = 1.0f) = 0;

	virtual bool isPaused(
		void) = 0;

	virtual int previous(
		void) = 0;

	virtual int next(
		void) = 0;

	virtual int seek(
		int64_t delta,
		bool exact = false) = 0;

	virtual int seekTo(
		uint64_t timestamp,
		bool exact = false) = 0;

	virtual uint64_t getDuration(
		void) = 0;

	virtual uint64_t getCurrentTime(
		void) = 0;

	virtual Session *getSession(
		void) = 0;

protected:
	bool mConfigured;
	Session *mSession;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_HPP_ */
