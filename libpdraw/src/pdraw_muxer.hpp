/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <pdraw/pdraw.hpp>

#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>

struct mux_ctx;

namespace Pdraw {

class MuxerWrapper;


class Muxer : public SinkElement {
public:
	Muxer(Session *session,
	      Element::Listener *elementListener,
	      IPdraw::IMuxer::Listener *listener,
	      MuxerWrapper *wrapper,
	      const struct pdraw_muxer_params *params);

	~Muxer() override;

	int start() override;

	int stop() override;

	/* Must be called on the loop thread */
	virtual int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params);

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	}

	/* Must be called on the loop thread */
	int removeInputMedia(Media *media) override;

	/* Must be called on the loop thread */
	int removeInputMedias() final;

	IPdraw::IMuxer *getMuxer() const
	{
		return mMuxer;
	}

	void clearMuxerListener()
	{
		mMuxerListener = nullptr;
	}

	/* Must be called on the loop thread */
	int addMedia(unsigned int mediaId,
		     const struct pdraw_muxer_media_params *params);

	/* Must be called on the loop thread */
	virtual int setThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size);

	/* Must be called on the loop thread */
	virtual int addChapter(uint64_t timestamp, const char *name);

	/* Must be called on the loop thread */
	virtual int
	setFileMetadata(const struct pdraw_muxer_metadata_params *params,
			const uint8_t *data,
			size_t size);

	/* Must be called on the loop thread */
	virtual int getStats(struct pdraw_muxer_stats *stats);

	virtual int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params);

	virtual int getDynParams(struct pdraw_muxer_dyn_params *dyn_params);

	virtual int forceSync();

protected:
	virtual int internalStart() = 0;

	virtual int internalStop() = 0;

	virtual int process() = 0;

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	static int createInputQueue(Media::Type type,
				    std::unique_ptr<mbuf::Queue> &queue);

	/* Can be called from any thread */
	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelTeardown(Channel *channel) override;

	int asyncCompleteFlush(Channel *channel, bool discard);

	int asyncCompleteStop();

	void idleCompleteStop();

	int completeStop();

	void
	onConnectionStateChanged(enum pdraw_muxer_connection_state state,
				 enum pdraw_muxer_disconnection_reason reason);

	void closeResponse(int status);

	void onUnrecoverableError(int error);

	std::unordered_map<Media *, std::unique_ptr<mbuf::Queue>> mInputQueues;
	IPdraw::IMuxer *mMuxer = nullptr;
	IPdraw::IMuxer::Listener *mMuxerListener = nullptr;
	struct pdraw_muxer_params mParams {
	};
	std::atomic_bool mReadyToStart{false};
	std::atomic_bool mReadyToStop{false};
	std::atomic_bool mAsyncFlush{false};
	std::atomic_bool mUnrecoverableError{false};
	std::atomic_int mUnrecoverableErrorStatus{};
	pomp::Loop::IdleHandlerFunc mCompleteStopHandler;

private:
	void completeFlush(const Channel *channel, bool discard);

	std::atomic_bool mFlushing{false};
	bool mClosing = false;

	void callCompleteFlush();

	/* Muxer listener calls from idle functions */
	void callOnConnectionStateChanged();
	void callCloseResponse();
	void callOnUnrecoverableError();

	struct completeFlushArgs {
		Channel *channel;
		bool discard;
	};
	std::mutex mCompleteFlushMutex;
	std::queue<completeFlushArgs> mCompleteFlushArgs;
	pomp::Loop::IdleHandlerFunc mCompleteFlushHandler;
	std::queue<enum pdraw_muxer_connection_state>
		mConnectionStateChangedStateArgs;
	std::queue<enum pdraw_muxer_disconnection_reason>
		mConnectionStateChangedReasonArgs;
	std::queue<int> mCloseRespStatusArgs;
	std::queue<int> mUnrecoverableErrorStatusArgs;
	pomp::Loop::IdleHandlerFunc mOnConnectionStateChangedHandler;
	pomp::Loop::IdleHandlerFunc mCloseResponseHandler;
	pomp::Loop::IdleHandlerFunc mOnUnrecoverableErrorHandler;
};


class MuxerWrapper : public IPdraw::IMuxer, public ElementWrapper {
public:
	MuxerWrapper(Session *session,
		     const std::string &url,
		     struct mux_ctx *mux,
		     const std::string &remoteHost,
		     const struct pdraw_muxer_params *params,
		     IPdraw::IMuxer::Listener *listener);

	/* Convenience overload for callers that have no mux context */
	MuxerWrapper(Session *session,
		     const std::string &url,
		     const struct pdraw_muxer_params *params,
		     IPdraw::IMuxer::Listener *listener) :
			MuxerWrapper(session,
				     url,
				     nullptr,
				     {},
				     params,
				     listener)
	{
	}

	~MuxerWrapper() override;

	int addMedia(unsigned int mediaId,
		     const struct pdraw_muxer_media_params *params) override;

	int setThumbnail(enum pdraw_muxer_thumbnail_type type,
			 const uint8_t *data,
			 size_t size) override;

	int addChapter(uint64_t timestamp, const char *name) override;

	int setFileMetadata(const struct pdraw_muxer_metadata_params *params,
			    const uint8_t *data,
			    size_t size) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int forceSync() override;

	int close() override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mMuxer = nullptr;
	}

	Muxer *getMuxer() const
	{
		return mMuxer;
	}

private:
	bool isElementStopped() const final
	{
		return (ElementWrapper::isElementStopped() ||
			mMuxer == nullptr);
	}

	Muxer *mMuxer = nullptr;
};

} /* namespace Pdraw */
