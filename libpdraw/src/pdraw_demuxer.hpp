/**
 * Parrot Drones Audio and Video Vector library
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

class DemuxerWrapper;


class Demuxer : public SourceElement {
public:
	virtual ~Demuxer(void);

	const struct pdraw_demuxer_params *getParams(void)
	{
		return &mParams;
	}

	virtual int getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias);

	virtual int selectMedia(uint32_t selectedMedias);

	virtual int flush(void) = 0;

	virtual int play(float speed = 1.0f) = 0;

	virtual bool isReadyToPlay(void) = 0;

	virtual bool isPaused(void) = 0;

	virtual int previous(void) = 0;

	virtual int next(void) = 0;

	virtual int seek(int64_t delta, bool exact = false) = 0;

	virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

	virtual int getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount);

	virtual uint64_t getDuration(void) = 0;

	virtual uint64_t getCurrentTime(void) = 0;

	IPdraw::IDemuxer *getDemuxer(void) const
	{
		return mDemuxer;
	}

	void clearDemuxerListener(void)
	{
		mDemuxerListener = nullptr;
	}

protected:
	Demuxer(Session *session,
		Element::Listener *elementListener,
		Source::Listener *sourceListener,
		DemuxerWrapper *wrapper,
		IPdraw::IDemuxer::Listener *demuxerListener,
		const struct pdraw_demuxer_params *params);

	void openResponse(int status);

	void closeResponse(int status);

	void onUnrecoverableError(int error = -EPROTO);

	int callSelectMedia(uint32_t selectedMedia);

	void readyToPlay(bool ready);

	void onEndOfRange(uint64_t timestamp);

	void playResponse(int status, uint64_t timestamp, float speed);

	void pauseResponse(int status, uint64_t timestamp);

	void seekResponse(int status, uint64_t timestamp, float speed);

	int updateMediaList(
		struct pdraw_demuxer_media *newMediaList,
		size_t newMediaListSize,
		std::vector<struct pdraw_demuxer_media *> &newDefaultMedias,
		uint32_t *selectedMedias);

	void clearMediaList(void);

	uint32_t selectedMediasToBitfield(void);

	IPdraw::IDemuxer *mDemuxer;
	IPdraw::IDemuxer::Listener *mDemuxerListener;
	struct pdraw_demuxer_params mParams;
	bool mReadyToPlay;
	bool mUnrecoverableError;
	bool mCalledOpenResp;
	bool mCallingSelectMedia;
	struct pdraw_demuxer_media *mMediaList;
	size_t mMediaListSize;
	std::vector<struct pdraw_demuxer_media *> mDefaultMedias;
	std::vector<struct pdraw_demuxer_media *> mSelectedMedias;

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


class DemuxerWrapper : public IPdraw::IDemuxer, public ElementWrapper {
public:
	DemuxerWrapper(Session *session,
		       const std::string &url,
		       struct mux_ctx *mux,
		       const struct pdraw_demuxer_params *params,
		       IPdraw::IDemuxer::Listener *listener);

	DemuxerWrapper(Session *session,
		       const std::string &localAddr,
		       uint16_t localStreamPort,
		       uint16_t localControlPort,
		       const std::string &remoteAddr,
		       uint16_t remoteStreamPort,
		       uint16_t remoteControlPort,
		       const struct pdraw_demuxer_params *params,
		       IPdraw::IDemuxer::Listener *listener);

	~DemuxerWrapper(void);

	int close(void) override;

	int getMediaList(struct pdraw_demuxer_media **mediaList,
			 size_t *mediaCount,
			 uint32_t *selectedMedias) override;

	int selectMedia(uint32_t selectedMedias) override;

	uint16_t getSingleStreamLocalStreamPort(void) override;

	uint16_t getSingleStreamLocalControlPort(void) override;

	bool isReadyToPlay(void) override;

	bool isPaused(void) override;

	int play(float speed = 1.0f) override;

	int pause(void) override;

	int previousFrame(void) override;

	int nextFrame(void) override;

	int seek(int64_t delta, bool exact = false) override;

	int seekForward(uint64_t delta, bool exact = false) override;

	int seekBack(uint64_t delta, bool exact = false) override;

	int seekTo(uint64_t timestamp, bool exact = false) override;

	uint64_t getDuration(void) override;

	uint64_t getCurrentTime(void) override;

	int getChapterList(struct pdraw_chapter **chapterList,
			   size_t *chapterCount) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mDemuxer = nullptr;
	}

	Demuxer *getDemuxer() const
	{
		return mDemuxer;
	}

private:
	Demuxer *mDemuxer;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_HPP_ */
