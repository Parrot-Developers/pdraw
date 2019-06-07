/**
 * Parrot Drones Awesome Video Viewer
 * Qt PDrAW demuxer object
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

#ifndef _QPDRAW_DEMUXER_PRIV_HPP_
#define _QPDRAW_DEMUXER_PRIV_HPP_

#include <pdraw/pdraw_backend.hpp>
#include <pdraw/qpdraw_demuxer.hpp>

using namespace Pdraw;

namespace QPdraw {
namespace Internal {


class QPdrawDemuxerPriv : public IPdraw::IDemuxer::Listener {

public:
	explicit QPdrawDemuxerPriv(QPdrawDemuxer *parent);
	~QPdrawDemuxerPriv();

	int open(const std::string &url);

	int open(const std::string &localAddr,
		 uint16_t localStreamPort,
		 uint16_t localControlPort,
		 const std::string &remoteAddr,
		 uint16_t remoteStreamPort,
		 uint16_t remoteControlPort);

	int open(const std::string &url, struct mux_ctx *mux);

	int close(void);

	uint16_t getSingleStreamLocalStreamPort(void);

	uint16_t getSingleStreamLocalControlPort(void);

	bool isReadyToPlay(void);

	bool isPaused(void);

	int play(float speed = 1.0f);

	int pause(void);

	int previousFrame(void);

	int nextFrame(void);

	int seek(int64_t delta, bool exact = false);

	int seekForward(uint64_t delta, bool exact = false);

	int seekBack(uint64_t delta, bool exact = false);

	int seekTo(uint64_t timestamp, bool exact = false);

	uint64_t getDuration(void);

	uint64_t getCurrentTime(void);

private:
	IPdraw *getPdrawInternal();

	void demuxerOpenResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status);

	void demuxerCloseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status);

	void onDemuxerUnrecoverableError(IPdraw *pdraw,
					 IPdraw::IDemuxer *demuxer);

	int demuxerSelectMedia(IPdraw *pdraw,
			       IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count);

	void demuxerReadyToPlay(IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer,
				bool ready);

	void onDemuxerEndOfRange(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp);

	void demuxerPlayResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed);

	void demuxerPauseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp);

	void demuxerSeekResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed);

	QPdrawDemuxer *mParent;
	IPdraw::IDemuxer *mDemuxer;
	bool mClosing;
};

} /* namespace Internal */
} /* namespace QPdraw */

#endif /* !_QPDRAW_DEMUXER_PRIV_HPP_ */
