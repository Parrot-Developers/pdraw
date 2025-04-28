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

#ifndef _PDRAW_MUXER_HPP_
#define _PDRAW_MUXER_HPP_

#include "pdraw_element.hpp"

#include <inttypes.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <pdraw/pdraw.hpp>

#include <queue>

namespace Pdraw {

class MuxerWrapper;


class Muxer : public SinkElement {
public:
	Muxer(Session *session,
	      Element::Listener *elementListener,
	      IPdraw::IMuxer::Listener *listener,
	      MuxerWrapper *wrapper,
	      const struct pdraw_muxer_params *params);

	virtual ~Muxer(void) = 0;

	int start(void) override;

	int stop(void) override;

	/* Must be called on the loop thread */
	virtual int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params);

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	}

	/* Must be called on the loop thread */
	virtual int removeInputMedia(Media *media) override;

	/* Must be called on the loop thread */
	virtual int removeInputMedias(void) override;

	IPdraw::IMuxer *getMuxer(void) const
	{
		return mMuxer;
	}

	void clearMuxerListener(void)
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
	virtual int getStats(struct pdraw_muxer_stats *stats);

	virtual int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params);

	virtual int getDynParams(struct pdraw_muxer_dyn_params *dyn_params);

	virtual int forceSync(void);

protected:
	virtual int internalStart(void) = 0;

	virtual int internalStop(void) = 0;

	virtual int process(void) = 0;

	/* Must be called on the loop thread */
	int addQueueEvtToLoop(struct mbuf_coded_video_frame_queue *queue,
			      struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int addQueueEvtToLoop(struct mbuf_raw_video_frame_queue *queue,
			      struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int addQueueEvtToLoop(struct mbuf_audio_frame_queue *queue,
			      struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int removeQueueEvtFromLoop(struct mbuf_coded_video_frame_queue *queue,
				   struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int removeQueueEvtFromLoop(struct mbuf_raw_video_frame_queue *queue,
				   struct pomp_loop *loop);

	/* Must be called on the loop thread */
	int removeQueueEvtFromLoop(struct mbuf_audio_frame_queue *queue,
				   struct pomp_loop *loop);

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	virtual void onChannelFlush(Channel *channel) override;

	virtual void onChannelTeardown(Channel *channel) override;

	static void idleCompleteStop(void *userdata);

	static void idleCompleteFlush(void *userdata);

	void
	onConnectionStateChanged(enum pdraw_muxer_connection_state state,
				 enum pdraw_muxer_disconnection_reason reason);

	void closeResponse(int status);

	void onUnrecoverableError(int error);

	IPdraw::IMuxer *mMuxer;
	IPdraw::IMuxer::Listener *mMuxerListener;
	struct pdraw_muxer_params mParams;
	std::atomic_bool mReadyToStart;
	std::atomic_bool mReadyToStop;
	std::atomic_bool mAsyncFlush;
	std::atomic_bool mUnrecoverableError;

private:
	int completeStop(void);

	void completeFlush(void);

	bool mFlushing;
	bool mClosing;

	/* Muxer listener calls from idle functions */
	static void callOnConnectionStateChanged(void *userdata);
	std::queue<enum pdraw_muxer_connection_state>
		mConnectionStateChangedStateArgs;
	std::queue<enum pdraw_muxer_disconnection_reason>
		mConnectionStateChangedReasonArgs;
	static void callCloseResponse(void *userdata);
	std::queue<int> mCloseRespStatusArgs;
	static void callOnUnrecoverableError(void *userdata);
	std::queue<int> mUnrecoverableErrorStatusArgs;
};


class MuxerWrapper : public IPdraw::IMuxer, public ElementWrapper {
public:
	MuxerWrapper(Session *session,
		     const std::string &url,
		     const struct pdraw_muxer_params *params,
		     IPdraw::IMuxer::Listener *listener);

	~MuxerWrapper(void);

	int addMedia(unsigned int mediaId,
		     const struct pdraw_muxer_media_params *params) override;

	int setThumbnail(enum pdraw_muxer_thumbnail_type type,
			 const uint8_t *data,
			 size_t size) override;

	int addChapter(uint64_t timestamp, const char *name) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int forceSync(void) override;

	int close(void) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mMuxer = nullptr;
	}

	Muxer *getMuxer() const
	{
		return mMuxer;
	}

private:
	Muxer *mMuxer;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_MUXER_HPP_ */
