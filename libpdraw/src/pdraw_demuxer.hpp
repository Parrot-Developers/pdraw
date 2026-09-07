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

#pragma once

#include "pdraw_element.hpp"
#include "pdraw_media.hpp"

#include <pdraw/pdraw.hpp>

#include <queue>

constexpr size_t DEMUXER_PENDING_COMMAND_TIMEOUT_MS = 3000;

namespace Pdraw {

class DemuxerWrapper;


class Demuxer : public SourceElement {
public:
	~Demuxer() override;

	const struct pdraw_demuxer_params *getParams() const
	{
		return &mParams;
	}

	virtual int getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias);

	virtual int selectMedia(uint32_t selectedMedias);

	virtual int flush(bool discard = true) = 0;

	inline virtual int drain()
	{
		return flush(false);
	}

	virtual int play(float speed = 1.0f) = 0;

	virtual bool isReadyToPlay() const = 0;

	virtual bool isPaused() const = 0;

	virtual int previous() = 0;

	virtual int next() = 0;

	virtual int seek(int64_t delta, bool exact = false) = 0;

	virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

	virtual int getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount);

	virtual uint64_t getDuration() const = 0;

	virtual uint64_t getCurrentTime() const = 0;

	IPdraw::IDemuxer *getDemuxer() const
	{
		return mDemuxer;
	}

	void clearDemuxerListener()
	{
		mDemuxerListener = nullptr;
	}

protected:
	enum class Command {
		NONE,
		PLAY,
		PAUSE,
		PAUSE_NEXT,
		SEEK,
	};

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

	int updateMediaList(struct pdraw_demuxer_media *newMediaList,
			    size_t newMediaListSize,
			    const std::vector<struct pdraw_demuxer_media *>
				    &newDefaultMedias,
			    uint32_t *selectedMedias);

	void clearMediaList();

	uint32_t selectedMediasToBitfield() const;

	static const char *getCommandStr(Demuxer::Command cmd);

	int setPendingCommand(Demuxer::Command cmd);

	Demuxer::Command getPendingCommand() const
	{
		return mPendingCmd;
	}

	void clearPendingCommand();

	IPdraw::IDemuxer *mDemuxer = nullptr;
	IPdraw::IDemuxer::Listener *mDemuxerListener = nullptr;
	struct pdraw_demuxer_params mParams {
	};
	bool mReadyToPlay = false;
	bool mUnrecoverableError = false;
	bool mCalledOpenResp = false;
	bool mCallingSelectMedia = false;
	struct pdraw_demuxer_media *mMediaList = nullptr;
	size_t mMediaListSize = 0;
	std::vector<struct pdraw_demuxer_media *> mDefaultMedias{};
	std::vector<struct pdraw_demuxer_media *> mSelectedMedias{};

	/* Demuxer listener calls from idle functions */
	void callOpenResponse();
	void callCloseResponse();
	void callOnUnrecoverableError();
	/* Note: callSelectMedia omitted: function has to be synchronous */
	void callReadyToPlay();
	void callEndOfRange();
	void callPlayResponse();
	void callPauseResponse();
	void callSeekResponse();

	std::queue<int> mOpenRespStatusArgs{};
	std::queue<int> mCloseRespStatusArgs{};
	std::queue<bool> mReadyToPlayReadyArgs{};
	std::queue<uint64_t> mEndOfRangeTimestampArgs{};
	std::queue<int> mPlayRespStatusArgs{};
	std::queue<uint64_t> mPlayRespTimestampArgs{};
	std::queue<float> mPlayRespSpeedArgs{};
	std::queue<int> mPauseRespStatusArgs{};
	std::queue<uint64_t> mPauseRespTimestampArgs{};
	std::queue<int> mSeekRespStatusArgs{};
	std::queue<uint64_t> mSeekRespTimestampArgs{};
	std::queue<float> mSeekRespSpeedArgs{};
	pomp::Loop::IdleHandlerFunc mCallOpenResponseHandler;
	pomp::Loop::IdleHandlerFunc mCallCloseResponseHandler;
	pomp::Loop::IdleHandlerFunc mCallOnUnrecoverableErrorHandler;
	pomp::Loop::IdleHandlerFunc mCallReadyToPlayHandler;
	pomp::Loop::IdleHandlerFunc mCallEndOfRangeHandler;
	pomp::Loop::IdleHandlerFunc mCallPlayResponseHandler;
	pomp::Loop::IdleHandlerFunc mCallPauseResponseHandler;
	pomp::Loop::IdleHandlerFunc mCallSeekResponseHandler;

private:
	void onWatchdogTimer();

	Demuxer::Command mPendingCmd = Command::NONE;
	pomp::Timer::HandlerFunc mWatchdogTimerHandler;
	std::unique_ptr<pomp::Timer> mWatchdogTimer;
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

	~DemuxerWrapper() override;

	int close() override;

	int getMediaList(struct pdraw_demuxer_media **mediaList,
			 size_t *mediaCount,
			 uint32_t *selectedMedias) override;

	int selectMedia(uint32_t selectedMedias) override;

	uint16_t getSingleStreamLocalStreamPort() override;

	uint16_t getSingleStreamLocalControlPort() override;

	bool isReadyToPlay() override;

	bool isPaused() override;

	int play(float speed = 1.0f) override;

	int pause() override;

	int previousFrame() override;

	int nextFrame() override;

	int seek(int64_t delta, bool exact = false) override;

	int seekForward(uint64_t delta, bool exact = false) override;

	int seekBack(uint64_t delta, bool exact = false) override;

	int seekTo(uint64_t timestamp, bool exact = false) override;

	uint64_t getDuration() override;

	uint64_t getCurrentTime() override;

	int getChapterList(struct pdraw_chapter **chapterList,
			   size_t *chapterCount) override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mDemuxer = nullptr;
	}

	Demuxer *getDemuxer() const
	{
		return mDemuxer;
	}

private:
	bool isElementStopped() const final
	{
		return (ElementWrapper::isElementStopped() ||
			mDemuxer == nullptr);
	}

	Demuxer *mDemuxer = nullptr;
};

} /* namespace Pdraw */
