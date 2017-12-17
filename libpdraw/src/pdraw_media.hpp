/**
 * Parrot Drones Awesome Video Viewer Library
 * Media interface
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

#ifndef _PDRAW_MEDIA_HPP_
#define _PDRAW_MEDIA_HPP_

#include "pdraw_decoder.hpp"
#include <pdraw/pdraw_defs.h>
#include <inttypes.h>
#include <string>

namespace Pdraw {


enum elementary_stream_type {
	ELEMENTARY_STREAM_TYPE_UNKNOWN = 0,
	ELEMENTARY_STREAM_TYPE_VIDEO_AVC,
};


class Session;


class Media {
public:
	virtual ~Media(
		void) {}

	virtual enum pdraw_media_type getType(
		void) = 0;

	virtual unsigned int getId(
		void) = 0;

	virtual int enableDecoder(
		void) = 0;

	virtual int disableDecoder(
		void) = 0;

	virtual Session *getSession(
		void) = 0;

	virtual Decoder *getDecoder(
		void) = 0;

protected:
	unsigned int mId;
	Session *mSession;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MEDIA_HPP_ */
