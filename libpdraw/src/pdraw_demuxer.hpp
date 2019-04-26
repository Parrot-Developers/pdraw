/**
 * Parrot Drones Awesome Video Viewer Library
 * Demuxer interface
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

#ifndef _PDRAW_DEMUXER_HPP_
#define _PDRAW_DEMUXER_HPP_

#include "pdraw_element.hpp"
#include "pdraw_media.hpp"
#include "pdraw_source.hpp"

namespace Pdraw {


#define DEMUXER_OUTPUT_BUFFER_COUNT (30)


class Demuxer : public Element, public Source {
public:
	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onReadyToPlay(Demuxer *demuxer, bool ready) = 0;

		virtual void onEndOfRange(Demuxer *demuxer,
					  uint64_t timestamp) = 0;

		virtual int
		selectDemuxerMedia(Demuxer *demuxer,
				   const struct pdraw_demuxer_media *medias,
				   size_t count) = 0;

		virtual void playResp(Demuxer *demuxer,
				      int status,
				      uint64_t timestamp,
				      float speed) = 0;

		virtual void
		pauseResp(Demuxer *demuxer, int status, uint64_t timestamp) = 0;

		virtual void seekResp(Demuxer *demuxer,
				      int status,
				      uint64_t timestamp,
				      float speed) = 0;
	};

	virtual ~Demuxer(void) {}

	virtual int flush(void) = 0;

	virtual int play(float speed = 1.0f) = 0;

	virtual bool isPaused(void) = 0;

	virtual int previous(void) = 0;

	virtual int next(void) = 0;

	virtual int seek(int64_t delta, bool exact = false) = 0;

	virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

	virtual uint64_t getDuration(void) = 0;

	virtual uint64_t getCurrentTime(void) = 0;

protected:
	Demuxer(Session *session,
		Element::Listener *elementListener,
		Source::Listener *sourceListener,
		Demuxer::Listener *demuxerListener) :
			Element(session, elementListener),
			Source(sourceListener),
			mDemuxerListener(demuxerListener)
	{
	}

	Demuxer::Listener *mDemuxerListener;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_HPP_ */
