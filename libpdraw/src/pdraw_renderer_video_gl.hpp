/**
 * Parrot Drones Audio and Video Vector library
 * OpenGL video renderer
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

#ifdef PDRAW_USE_GL

#	include "pdraw_gl_video.hpp"
#	include "pdraw_renderer_video.hpp"
#	include <atomic>

namespace Pdraw {

class GlVideoRenderer : public VideoRenderer {
public:
	GlVideoRenderer(Session *session,
			Element::Listener *listener,
			VideoRendererWrapper *wrapper,
			IPdraw::IVideoRenderer::Listener *rndListener,
			unsigned int mediaId,
			const struct pdraw_rect *renderPos,
			const struct pdraw_video_renderer_params *params);

	~GlVideoRenderer() override;

	int start() override;

	int stop() override;

	int render(struct pdraw_rect *contentPos,
		   const float *viewMat = nullptr,
		   const float *projMat = nullptr) override;

	int resize(const struct pdraw_rect *renderPos) override;

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId() override;

	int setParams(const struct pdraw_video_renderer_params *params,
		      bool force) override;


	int getParams(struct pdraw_video_renderer_params *params) override;

	int addInputMedia(Media *media) override;

	int removeInputMedia(Media *media) override;

	int removeInputMedias() override;

	void completeStop() override;

protected:
	enum class Transition {
		NONE = 0,
		FADE_FROM_BLACK,
		FADE_TO_BLACK,
		FADE_TO_BLACK_AND_WHITE,
		FADE_TO_BLUR,
		FLASH_THEN_BLACK_AND_WHITE,
	};

	virtual int setup(const struct pdraw_rect *renderPos,
			  const struct pdraw_video_renderer_params *params);

	int startExtLoad();

	int stopExtLoad();

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelSos(Channel *channel) override;

	void onChannelEos(Channel *channel) override;

	void onChannelReconfigure(Channel *channel) override;

	void onChannelResolutionChange(Channel *channel) override;

	void onChannelFramerateChange(Channel *channel) override;

	void onChannelTimeout(Channel *channel) override;

	void onChannelPhotoTrigger(Channel *channel) override;

	int doTransition(uint64_t timestamp, bool frameReady, bool *loadFrame);

	void abortTransition();

	virtual int loadVideoFrame(struct mbuf_raw_video_frame *frame);

	void createProjMatrix(Eigen::Matrix4f &projMat,
			      float aspectRatio,
			      float near,
			      float far) const;

	void updateViewProjMatrix(Eigen::Matrix4f &viewProjMat,
				  vmeta_frame *meta);

	virtual int
	loadExternalVideoFrame(struct mbuf_raw_video_frame *frame,
			       const struct pdraw_media_info *mediaInfo);

	virtual int renderVideoFrame(const struct pdraw_rect *renderPos,
				     struct pdraw_rect *contentPos,
				     Eigen::Matrix4f &viewProjMat);

	virtual int renderExternalVideoFrame(const struct pdraw_rect *renderPos,
					     struct pdraw_rect *contentPos,
					     Eigen::Matrix4f &viewProjMat);

	void onTimer();

	void onWatchdogTimer();

	void onVideoPresStatsTimer();

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	unsigned int mTargetPrimaryMediaId = 0;
	unsigned int mPrimaryMediaId = 0;
	std::atomic_bool mRunning{false};
	/* Currently loaded frame */
	struct {
		struct mbuf_raw_video_frame *frame = nullptr;
		struct vdef_raw_frame info {
		};
		RawVideoMedia::Frame data{};
		struct vmeta_frame *metadata = nullptr;
	} mLoadedFrame;
	/* Next frame to be processed */
	struct {
		struct mbuf_raw_video_frame *frame = nullptr;
		struct vdef_raw_frame info {
		};
		RawVideoMedia::Frame data{};
		struct vmeta_frame *metadata = nullptr;
		/* Non-owning pointer to the media that produced this frame,
		 * captured at dequeue time in scheduleFrame(). Used by
		 * onNextFrameLoaded() so that mMediaInfo always reflects the
		 * media of the displayed frame, not the current primary media.
		 * All accesses are under Sink::lock(). */
		RawVideoMedia *media = nullptr;
	} mNextFrame;
	RawVideoMedia *mPrimaryMedia = nullptr;
	std::unique_ptr<mbuf::Queue> mInputQueue;
	struct pdraw_media_info mMediaInfo {
	};
	struct vmeta_session mMediaInfoSessionMeta {
	};
	pomp::Timer::HandlerFunc mTimerHandler;
	std::unique_ptr<pomp::Timer> mTimer;
	std::unique_ptr<GlVideo> mGlVideo{};
	unsigned int mGlVideoFirstTexUnit = 0;
	GLint mDefaultFbo = 0;
	GLuint mExtLoadFbo = 0;
	GLuint mExtLoadFboTexture = 0;
	int mX = 0;
	int mY = 0;
	unsigned int mWidth = 0;
	unsigned int mHeight = 0;
	Transition mPendingTransition = Transition::NONE;
	Transition mCurrentTransition = Transition::NONE;
	uint64_t mTransitionStartTime = 0;
	uint64_t mTransitionHoldTime = 0;
	struct pdraw_video_renderer_params mParams {
	};
	bool mExtLoadVideoTexture = false;
	unsigned int mExtVideoTextureWidth = 0;
	unsigned int mExtVideoTextureHeight = 0;
	bool mRenderVideoOverlay = false;
	bool mFirstFrame = false;
	std::atomic_bool mFrameLoaded{false};
	uint64_t mLastLoadTimestamp = UINT64_MAX;
	uint64_t mLastRenderTimestamp = UINT64_MAX;
	float mAvgRenderRate = 0.;
	uint64_t mLastFrameTimestamp = UINT64_MAX;
	VideoPresStats mVideoPresStats{};
	pomp::Timer::HandlerFunc mVideoPresStatsTimerHandler;
	std::unique_ptr<pomp::Timer> mVideoPresStatsTimer;
	uint64_t mSchedLastInputTimestamp = UINT64_MAX;
	uint64_t mSchedLastOutputTimestamp = UINT64_MAX;
	bool mSchedInitialBuffering = false;
	bool mRenderReadyScheduled = false;
	bool mPendingRestart = false;
	std::string mAncillaryKey{};

	/* Logging-related variables */
	/* previous mFrameLoaded value logged */
	bool mFrameLoadedLogged = false;
	/* Watchdog timer: triggered if no new frame is received for a given
	 * amount of time */
	pomp::Timer::HandlerFunc mWatchdogTimerHandler;
	std::unique_ptr<pomp::Timer> mWatchdogTimer;
	std::atomic_bool mWatchdogTriggered{false};
	std::atomic_bool mEos{false};
	bool mPendingResize = false;
	bool mMbStatusOverlay = false;

private:
	/* Actual implementation of removeInputMedia(), factored out into a
	 * non-virtual method so the destructor never invokes anything
	 * virtual (derived class is already destroyed by then). */
	int removeInputMediaImpl(Media *media);

	int setupExtTexture(const struct vdef_raw_frame *frameInfo);

	void onNextFrameLoaded();

	void setNormalization();

	void renewMedia();

	static bool queueFilter(struct mbuf_raw_video_frame *frame,
				void *userdata);

	static uint64_t getFrameU64(struct mbuf_raw_video_frame *frame,
				    const char *key);

	unsigned int getPrimaryMediaFrameIntervalMs() const;

	mbuf::Queue *getPrimaryMediaQueue();

	int getNextFrameDelay(mbuf::Queue *queue,
			      uint64_t curTime,
			      bool allowDrop,
			      bool *shouldBreak,
			      bool *processAnyway,
			      uint64_t *delayUs,
			      int64_t *compensationUs,
			      int64_t *timingErrorUs,
			      uint64_t *frameTsUs,
			      int logLevel);

	int
	scheduleFrame(uint64_t curTime, bool *load, int64_t *compensationUs);

	void idleRenewMedia();
	void idleStart();
	void completeDrain();
	void idleCompleteDrain();

	void onChannelSessionMetaUpdate(Channel *channel) override;

	pomp::Loop::IdleHandlerFunc mIdleRenewMediaHandler;
	pomp::Loop::IdleHandlerFunc mIdleStartHandler;
	pomp::Loop::IdleHandlerFunc mIdleCompleteDrainHandler;
};

} /* namespace Pdraw */

#endif /* PDRAW_USE_GL */
