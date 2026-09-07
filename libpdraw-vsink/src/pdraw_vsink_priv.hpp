/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <libpomp.hpp>

#include <futils/futils.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <pdraw-vsink/pdraw_vsink.hpp>
#include <pdraw/pdraw.h>
#include <pdraw/pdraw.hpp>

#define ULOG_TAG pdraw_vsink
#include <ulog.h>

namespace PdrawVsink {

/*
 * Owns a background thread running its own pomp::Loop, on which a real
 * IPdraw / demuxer / raw-or-coded video sink is created and driven --
 * modeled directly on libpdraw-backend's PdrawBackend class
 * (packages/pdraw/libpdraw-backend/src/pdraw_backend_impl.{hpp,cpp}), the
 * closest structural analog in this codebase for a class that owns its own
 * pomp_loop-processing thread.
 *
 * Threading model (unchanged from the pre-C++17 implementation): all public
 * methods below may be called from any thread; start()/getFrame() block
 * their caller on mCond until the corresponding operation completes on the
 * loop thread. All the IPdraw/IDemuxer/IRawVideoSink/ICodedVideoSink Listener
 * callbacks are invoked BY libpdraw itself on the loop thread, except
 * IPdrawVsink::Listener::onFrameReady(), which is invoked from this
 * instance's own thread too (see rawQueueEventCb/codedQueueEventCb's doc
 * comment history) -- callers must expect it to block this thread until it
 * returns.
 */
class Vsink : public IPdrawVsink,
	      public Pdraw::IPdraw::Listener,
	      public Pdraw::IPdraw::IDemuxer::Listener,
	      public Pdraw::IPdraw::IRawVideoSink::Listener,
	      public Pdraw::IPdraw::ICodedVideoSink::Listener {
public:
	explicit Vsink(const struct pdraw_vsink_params *params,
		       IPdrawVsink::Listener *listener);
	~Vsink() override;

	/* Blocking: launches the background thread, creates the real pdraw
	 * instance/demuxer on it, and waits until the demuxer/media_added
	 * handshake completes (success) or fails. */
	int start(struct pdraw_media_info **mediaInfo);

	int getFrame(int timeoutMs,
		     struct mbuf_mem *frameMemory,
		     struct pdraw_video_frame *frameInfo,
		     struct pdraw_vsink_frame *retFrame) override;

	/* Internal API below, kept public only so the whitebox unit tests
	 * (tests/test_*.cpp) can exercise these guard clauses directly
	 * without a live pdraw pipeline -- mirrors the intent of the
	 * pre-C++17 implementation's pdraw_vsink_priv.h, which exposed the
	 * equivalent free functions for the exact same reason. Not part of
	 * the public IPdrawVsink interface. */
	int getRawFrame(int timeoutMs,
			struct mbuf_mem *frameMemory,
			struct pdraw_video_frame *frameInfo,
			struct mbuf_raw_video_frame **retFrame);
	int getCodedFrame(int timeoutMs,
			  struct mbuf_mem *frameMemory,
			  struct pdraw_video_frame *frameInfo,
			  struct mbuf_coded_video_frame **retFrame);
	int processRawMediaAdded(const struct pdraw_media_info *info);
	int processCodedMediaAdded(const struct pdraw_media_info *info);
	int detachRawEvent();
	int detachCodedEvent();
	int destroyRawSink();
	int destroyCodedSink();

private:
	static void loopThread(Vsink *self);

	/* IPdraw::Listener overrides (run on the loop thread) */
	void stopResponse(Pdraw::IPdraw *pdraw, int status) override;
	void onMediaAdded(Pdraw::IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override;
	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override;
	void onSocketCreated([[maybe_unused]] Pdraw::IPdraw *pdraw,
			     [[maybe_unused]] int fd) override
	{
		/* Intentional no-op. */
	}

	/* IPdraw::IDemuxer::Listener overrides */
	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status) override;
	void
	demuxerCloseResponse([[maybe_unused]] Pdraw::IPdraw *pdraw,
			     [[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
			     [[maybe_unused]] int status) override
	{
		/* Intentional no-op. */
	}
	void onDemuxerUnrecoverableError(
		[[maybe_unused]] Pdraw::IPdraw *pdraw,
		[[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer) override
	{
		/* Intentional no-op. */
	}
	int demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override;
	void demuxerReadyToPlay(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IDemuxer *demuxer,
				bool ready) override;
	void
	onDemuxerEndOfRange([[maybe_unused]] Pdraw::IPdraw *pdraw,
			    [[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
			    [[maybe_unused]] uint64_t timestamp) override
	{
		/* Intentional no-op. */
	}
	void demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override;
	void
	demuxerPauseResponse([[maybe_unused]] Pdraw::IPdraw *pdraw,
			     [[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
			     [[maybe_unused]] int status,
			     [[maybe_unused]] uint64_t timestamp) override
	{
		/* Intentional no-op. */
	}
	void
	demuxerSeekResponse([[maybe_unused]] Pdraw::IPdraw *pdraw,
			    [[maybe_unused]] Pdraw::IPdraw::IDemuxer *demuxer,
			    [[maybe_unused]] int status,
			    [[maybe_unused]] uint64_t timestamp,
			    [[maybe_unused]] float speed) override
	{
		/* Intentional no-op. */
	}

	/* IPdraw::IRawVideoSink::Listener overrides */
	static void rawQueueEventCb(struct pomp_evt *evt, void *userdata);
	void
	onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink,
				 const struct pdraw_media_info *info) override;
	void onRawVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IRawVideoSink *sink,
					const struct pdraw_media_info *info,
					bool restart) override;
	void onRawVideoSinkFlush(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override;
	void onRawVideoSinkDrain(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override;
	void onRawVideoSinkSessionMetaUpdate(
		[[maybe_unused]] Pdraw::IPdraw *pdraw,
		[[maybe_unused]] Pdraw::IPdraw::IRawVideoSink *sink,
		[[maybe_unused]] const struct vmeta_session *meta) override
	{
		/* Intentional no-op. */
	}

	/* IPdraw::ICodedVideoSink::Listener overrides */
	static void codedQueueEventCb(struct pomp_evt *evt, void *userdata);
	void onCodedVideoSinkMediaAdded(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSink *sink,
		const struct pdraw_media_info *info) override;
	void onCodedVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
					  const struct pdraw_media_info *info,
					  bool restart) override;
	void
	onCodedVideoSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override;
	void
	onCodedVideoSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override;
	void onCodedVideoSinkSessionMetaUpdate(
		[[maybe_unused]] Pdraw::IPdraw *pdraw,
		[[maybe_unused]] Pdraw::IPdraw::ICodedVideoSink *sink,
		[[maybe_unused]] const struct vmeta_session *meta) override
	{
		/* Intentional no-op. */
	}

	/* idle callbacks, run on the loop thread */
	void startPdrawIdle();
	void stopPdrawIdle();
	void deletePdrawIdle();

	std::mutex mMutex;
	std::condition_variable mCond;
	bool mStarting = false;
	bool mCondReady = false;
	bool mFrameReady = false;
	/* Set by ~Vsink() to unblock any thread currently parked in
	 * getRawFrame()/getCodedFrame()'s cond_wait, and checked by those
	 * functions before touching this instance again. mWaiters counts
	 * such in-flight callers so ~Vsink() can wait for all of them to
	 * have observed mStopping and returned before tearing down mLoop/
	 * mPdraw and the rest of this instance. */
	bool mStopping = false;
	int mWaiters = 0;

	std::thread mThread;
	bool mThreadLaunched = false;
	std::atomic_bool mThreadShouldStop{false};
	std::unique_ptr<pomp::Loop> mLoop;

	std::unique_ptr<Pdraw::IPdraw> mPdraw;
	std::unique_ptr<Pdraw::IPdraw::IDemuxer> mDemuxer;
	enum pdraw_vsink_video_media_type mType;

	struct {
		std::unique_ptr<Pdraw::IPdraw::IRawVideoSink> sink;
		struct mbuf_raw_video_frame_queue *queue = nullptr;
	} mRaw;
	struct {
		std::unique_ptr<Pdraw::IPdraw::ICodedVideoSink> sink;
		struct mbuf_coded_video_frame_queue *queue = nullptr;
	} mCoded;

	struct pdraw_media_info *mMediaInfo = nullptr;
	std::string mUrl;
	enum pdraw_playback_mode mPlaybackMode;
	enum vmeta_camera_type mCameraType;
	int mResult = 0;
	IPdrawVsink::Listener *mListener = nullptr;

	pomp::Loop::IdleHandlerFunc mStartPdrawIdleHandler;
	pomp::Loop::IdleHandlerFunc mStopPdrawIdleHandler;
	pomp::Loop::IdleHandlerFunc mDeletePdrawIdleHandler;
};

} /* namespace PdrawVsink */
