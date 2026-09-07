/**
 * Parrot Drones Audio and Video Vector library
 * Application external coded video source
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
#include <media-buffers/mbuf_queue.hpp>
#include <pdraw/pdraw.hpp>

namespace Pdraw {

class CodedVideoSourceWrapper;


class ExternalCodedVideoSource : public SourceElement {
public:
	ExternalCodedVideoSource(
		Session *session,
		Element::Listener *elementListener,
		Source::Listener *sourceListener,
		IPdraw::ICodedVideoSource::Listener *listener,
		CodedVideoSourceWrapper *wrapper,
		const struct pdraw_video_source_params *params);

	~ExternalCodedVideoSource() override;

	int start() override;

	int stop() override;

	int flush(bool discard = true);

	inline int drain()
	{
		return flush(false);
	}

	int setSessionMetadata(const struct vmeta_session *meta);

	int getSessionMetadata(struct vmeta_session *meta) const;

	mbuf::Queue *getQueue() const
	{
		return mFrameQueue.get();
	}

	IPdraw::ICodedVideoSource *getVideoSource() const
	{
		return mVideoSource;
	}

private:
	int process();

	int processFrame(struct mbuf_coded_video_frame *frame);

	void completeFlush();

	int tryStop();

	void completeStop();

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	static bool inputFilter(struct mbuf_coded_video_frame *frame,
				void *userdata);

	void idleCompleteFlush();

	/* Video source listener calls from idle functions */
	void callOnMediaAdded();
	void callVideoSourceFlushed();

	IPdraw::ICodedVideoSource *mVideoSource = nullptr;
	IPdraw::ICodedVideoSource::Listener *mVideoSourceListener = nullptr;
	struct pdraw_video_source_params mParams {
	};
	std::unique_ptr<mbuf::Queue> mFrameQueue;
	std::unique_ptr<CodedVideoMedia> mOutputMedia{};
	uint64_t mLastTimestamp = UINT64_MAX;
	pomp::Loop::IdleHandlerFunc mCompleteFlushHandler;
	pomp::Loop::IdleHandlerFunc mCallOnMediaAddedHandler;
	pomp::Loop::IdleHandlerFunc mCallVideoSourceFlushedHandler;
};


class CodedVideoSourceWrapper : public IPdraw::ICodedVideoSource,
				public ElementWrapper {
public:
	CodedVideoSourceWrapper(Session *session,
				const struct pdraw_video_source_params *params,
				IPdraw::ICodedVideoSource::Listener *listener);

	~CodedVideoSourceWrapper() override;

	struct mbuf_coded_video_frame_queue *getQueue() override;

	int flush() override;

	int drain() override;

	int setSessionMetadata(const struct vmeta_session *meta) override;

	int getSessionMetadata(struct vmeta_session *meta) override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
		mSource = nullptr;
	}

	Source *getSource() const
	{
		return mSource;
	}

	ExternalCodedVideoSource *getCodedVideoSource() const
	{
		return mSource;
	}

private:
	bool isElementStopped() const final
	{
		return (ElementWrapper::isElementStopped() ||
			mSource == nullptr);
	}

	ExternalCodedVideoSource *mSource = nullptr;
};

} /* namespace Pdraw */
