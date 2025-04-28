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

#ifndef _PDRAW_RENDERER_VIDEO_GL_HPP_
#define _PDRAW_RENDERER_VIDEO_GL_HPP_

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

	~GlVideoRenderer(void);

	int start(void) override;

	int stop(void) override;

	int render(struct pdraw_rect *contentPos,
		   const float *viewMat = nullptr,
		   const float *projMat = nullptr) override;

	int resize(const struct pdraw_rect *renderPos) override;

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId(void) override;

	int setParams(const struct pdraw_video_renderer_params *params,
		      bool force) override;


	int getParams(struct pdraw_video_renderer_params *params) override;

	int addInputMedia(Media *media) override;

	int removeInputMedia(Media *media) override;

	int removeInputMedias(void) override;

	void completeStop(void) override;

protected:
	enum Transition {
		NONE = 0,
		FADE_FROM_BLACK,
		FADE_TO_BLACK,
		FADE_TO_BLACK_AND_WHITE,
		FADE_TO_BLUR,
		FLASH_THEN_BLACK_AND_WHITE,
	};

	virtual int setup(const struct pdraw_rect *renderPos,
			  const struct pdraw_video_renderer_params *params);

	int startExtLoad(void);

	int stopExtLoad(void);

	void onChannelFlush(Channel *channel) override;

	void onChannelSos(Channel *channel) override;

	void onChannelEos(Channel *channel) override;

	void onChannelReconfigure(Channel *channel) override;

	void onChannelResolutionChange(Channel *channel) override;

	void onChannelFramerateChange(Channel *channel) override;

	void onChannelTimeout(Channel *channel) override;

	void onChannelPhotoTrigger(Channel *channel) override;

	int doTransition(uint64_t timestamp, bool frameReady, bool *loadFrame);

	void abortTransition(void);

	virtual int loadVideoFrame(struct mbuf_raw_video_frame *frame);

	void createProjMatrix(Eigen::Matrix4f &projMat,
			      float aspectRatio,
			      float near,
			      float far);

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

	static void timerCb(struct pomp_timer *timer, void *userdata);

	static void watchdogTimerCb(struct pomp_timer *timer, void *userdata);

	static void videoPresStatsTimerCb(struct pomp_timer *timer,
					  void *userdata);

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	int removeQueueFdFromPomp(struct mbuf_raw_video_frame_queue *queue);

	unsigned int mTargetPrimaryMediaId;
	unsigned int mPrimaryMediaId;
	std::atomic_bool mRunning;
	struct mbuf_raw_video_frame *mCurrentFrame;
	RawVideoMedia::Frame mCurrentFrameData;
	struct vdef_raw_frame mCurrentFrameInfo;
	struct vmeta_frame *mCurrentFrameMetadata;
	RawVideoMedia *mPrimaryMedia;
	struct pdraw_media_info mMediaInfo;
	struct vmeta_session mMediaInfoSessionMeta;
	struct pomp_timer *mTimer;
	GlVideo *mGlVideo;
	unsigned int mGlVideoFirstTexUnit;
	GLint mDefaultFbo;
	GLuint mExtLoadFbo;
	GLuint mExtLoadFboTexture;
	int mX;
	int mY;
	unsigned int mWidth;
	unsigned int mHeight;
	enum Transition mPendingTransition;
	enum Transition mCurrentTransition;
	uint64_t mTransitionStartTime;
	uint64_t mTransitionHoldTime;
	struct pdraw_video_renderer_params mParams;
	bool mExtLoadVideoTexture;
	unsigned int mExtVideoTextureWidth;
	unsigned int mExtVideoTextureHeight;
	bool mRenderVideoOverlay;
	bool mFirstFrame;
	bool mFrameLoaded;
	uint64_t mLastLoadTimestamp;
	uint64_t mLastRenderTimestamp;
	float mAvgRenderRate;
	uint64_t mLastFrameTimestamp;
	VideoPresStats mVideoPresStats;
	struct pomp_timer *mVideoPresStatsTimer;
	uint64_t mSchedLastInputTimestamp;
	uint64_t mSchedLastOutputTimestamp;
	bool mSchedInitialBuffering;
	bool mRenderReadyScheduled;
	bool mPendingRestart;
	std::string mAncillaryKey;

	/* Logging-related variables */
	/* previous mFrameLoaded value logged */
	bool mFrameLoadedLogged;
	/* Watchdog timer: triggered if no new frame is received for a given
	 * amount of time */
	struct pomp_timer *mWatchdogTimer;
	std::atomic_bool mWatchdogTriggered;
	std::atomic_bool mEos;
	bool mPendingResize;
	bool mMbStatusOverlay;

private:
	int setupExtTexture(const struct vdef_raw_frame *frameInfo);

	void setNormalization(void);

	static bool queueFilter(struct mbuf_raw_video_frame *frame,
				void *userdata);

	static uint64_t getFrameU64(struct mbuf_raw_video_frame *frame,
				    const char *key);

	unsigned int getPrimaryMediaFrameIntervalMs(void);

	struct mbuf_raw_video_frame_queue *getPrimaryMediaQueue(void);

	int getNextFrameDelay(mbuf_raw_video_frame_queue *queue,
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

	static void idleRenewMedia(void *userdata);

	static void idleStart(void *renderer);

	void onChannelSessionMetaUpdate(Channel *channel) override;
};

} /* namespace Pdraw */

#endif /* PDRAW_USE_GL */

#endif /* !_PDRAW_RENDERER_VIDEO_GL_HPP_ */
