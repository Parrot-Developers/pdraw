/**
 * Parrot Drones Awesome Video Viewer Library
 * Generic demuxer
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

#include <pdraw/pdraw.hpp>

#include <queue>

namespace Pdraw {


#define DEMUXER_OUTPUT_BUFFER_COUNT (30)


class Demuxer : public CodedSourceElement {
public:
	virtual ~Demuxer(void);

	virtual int flush(void) = 0;

	virtual int play(float speed = 1.0f) = 0;

	virtual bool isReadyToPlay(void) = 0;

	virtual bool isPaused(void) = 0;

	virtual int previous(void) = 0;

	virtual int next(void) = 0;

	virtual int seek(int64_t delta, bool exact = false) = 0;

	virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

	virtual uint64_t getDuration(void) = 0;

	virtual uint64_t getCurrentTime(void) = 0;

	IPdraw::IDemuxer *getDemuxer(void)
	{
		return mDemuxer;
	}

	IPdraw::IDemuxer::Listener *getDemuxerListener(void)
	{
		return mDemuxerListener;
	}

protected:
	Demuxer(Session *session,
		Element::Listener *elementListener,
		CodedSource::Listener *sourceListener,
		IPdraw::IDemuxer *demuxer,
		IPdraw::IDemuxer::Listener *demuxerListener) :
			CodedSourceElement(session,
					   elementListener,
					   UINT_MAX,
					   sourceListener),
			mDemuxer(demuxer), mDemuxerListener(demuxerListener),
			mReadyToPlay(false), mUnrecoverableError(false),
			mCalledOpenResp(false)
	{
	}

	void openResponse(int status);

	void closeResponse(int status);

	void onUnrecoverableError(int error = -EPROTO);

	int selectMedia(const struct pdraw_demuxer_media *medias, size_t count);

	void readyToPlay(bool ready);

	void onEndOfRange(uint64_t timestamp);

	void playResponse(int status, uint64_t timestamp, float speed);

	void pauseResponse(int status, uint64_t timestamp);

	void seekResponse(int status, uint64_t timestamp, float speed);

	IPdraw::IDemuxer *mDemuxer;
	IPdraw::IDemuxer::Listener *mDemuxerListener;
	bool mReadyToPlay;
	bool mUnrecoverableError;
	bool mCalledOpenResp;

	/* Demuxer listener calls from idle functions */
	static void callOpenResponse(void *userdata);
	std::queue<int> mOpenRespStatusArgs;
	static void callCloseResponse(void *userdata);
	std::queue<int> mCloseRespStatusArgs;
	static void callOnUnrecoverableError(void *userdata);
	/* Note: callSelectMedia omitted: function has to be synchronous */
	static void callReadyToPlay(void *userdata);
	std::queue<bool> mReadyToPlayReadyArgs;
	static void callEndOfRange(void *userdata);
	std::queue<uint64_t> mEndOfRangeTimestampArgs;
	static void callPlayResponse(void *userdata);
	std::queue<int> mPlayRespStatusArgs;
	std::queue<uint64_t> mPlayRespTimestampArgs;
	std::queue<float> mPlayRespSpeedArgs;
	static void callPauseResponse(void *userdata);
	std::queue<int> mPauseRespStatusArgs;
	std::queue<uint64_t> mPauseRespTimestampArgs;
	static void callSeekResponse(void *userdata);
	std::queue<int> mSeekRespStatusArgs;
	std::queue<uint64_t> mSeekRespTimestampArgs;
	std::queue<float> mSeekRespSpeedArgs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_HPP_ */
