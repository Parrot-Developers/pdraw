/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 renderer
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

#ifndef _PDRAW_RENDERER_GLES2_HPP_
#define _PDRAW_RENDERER_GLES2_HPP_

#ifdef USE_GLES2

#	include "pdraw_gles2_hmd.hpp"
#	include "pdraw_gles2_video.hpp"
#	include "pdraw_renderer.hpp"
#	include <atomic>

namespace Pdraw {

class Gles2Renderer : public Renderer {
public:
	Gles2Renderer(Session *session,
		      Element::Listener *listener,
		      IPdraw::IVideoRenderer *renderer,
		      IPdraw::IVideoRenderer::Listener *rndListener,
		      unsigned int mediaId,
		      const struct pdraw_rect *renderPos,
		      const struct pdraw_video_renderer_params *params,
		      struct egl_display *eglDisplay);

	~Gles2Renderer(void);

	int start(void);

	int stop(void);

	int render(struct pdraw_rect *contentPos,
		   const float *viewMat = nullptr,
		   const float *projMat = nullptr);

	int resize(const struct pdraw_rect *renderPos);

	int setMediaId(unsigned int mediaId);

	unsigned int getMediaId(void);

	int setParams(const struct pdraw_video_renderer_params *params);


	int getParams(struct pdraw_video_renderer_params *params);

	int addInputMedia(RawVideoMedia *media);

	int removeInputMedia(RawVideoMedia *media);

	int removeInputMedias(void);

	void completeStop(void);

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
			  const struct pdraw_video_renderer_params *params,
			  struct egl_display *eglDisplay);

	int startHmd(void);

	int stopHmd(void);

	int startExtLoad(void);

	int stopExtLoad(void);

	void onChannelFlush(RawChannel *channel);

	void onChannelSos(RawChannel *channel);

	void onChannelEos(RawChannel *channel);

	void onChannelReconfigure(RawChannel *channel);

	void onChannelTimeout(RawChannel *channel);

	void onChannelPhotoTrigger(RawChannel *channel);

	int doTransition(uint64_t timestamp, bool frameReady, bool *loadFrame);

	void abortTransition(void);

	virtual int loadVideoFrame(struct mbuf_raw_video_frame *mbufFrame,
				   RawVideoMedia::Frame *frame);

	void createProjMatrix(Eigen::Matrix4f &projMat,
			      float aspectRatio,
			      float near,
			      float far);

	void updateViewProjMatrix(Eigen::Matrix4f &viewProjMat,
				  vmeta_frame *meta);

	virtual int
	loadExternalVideoFrame(struct mbuf_raw_video_frame *mbufFrame,
			       RawVideoMedia::Frame *frame,
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

	unsigned int mMediaId;
	unsigned int mCurrentMediaId;
	bool mRunning;
	struct mbuf_raw_video_frame *mCurrentFrame;
	RawVideoMedia::Frame mCurrentFrameData;
	struct vdef_raw_frame mCurrentFrameInfo;
	struct vmeta_frame *mCurrentFrameMetadata;
	RawVideoMedia *mLastAddedMedia;
	struct pdraw_media_info mMediaInfo;
	struct vmeta_session mMediaInfoSessionMeta;
	struct pomp_timer *mTimer;
	Gles2Hmd *mGles2Hmd;
	unsigned int mGles2HmdFirstTexUnit;
	Gles2Video *mGles2Video;
	unsigned int mGles2VideoFirstTexUnit;
	GLint mDefaultFbo;
	unsigned int mHmdFboSize;
	GLuint mHmdFbo;
	GLuint mHmdFboTexture;
	GLuint mHmdFboRenderBuffer;
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
	uint64_t mLastRenderTimestamp;
	uint64_t mLastFrameTimestamp;
	VideoPresStats mVideoPresStats;
	struct pomp_timer *mVideoPresStatsTimer;
	uint64_t mSchedLastInputTimestamp;
	uint64_t mSchedLastOutputTimestamp;
	bool mRenderReadyScheduled;

	/* Logging-related variables */
	/* previous mFrameLoaded value logged */
	bool mFrameLoadedLogged;
	/* Watchdog timer: triggered if no new frame is received for a given
	 * amount of time */
	struct pomp_timer *mWatchdogTimer;
	std::atomic_bool mWatchdogTriggered;

private:
	int setupExtTexture(const struct vdef_raw_frame *frameInfo,
			    const RawVideoMedia::Frame *frame);

	static bool queueFilter(struct mbuf_raw_video_frame *frame,
				void *userdata);

	static uint64_t getFrameU64(struct mbuf_raw_video_frame *frame,
				    const char *key);

	struct mbuf_raw_video_frame_queue *getLastAddedMediaQueue(void);

	int getNextFrameDelay(mbuf_raw_video_frame_queue *queue,
			      uint64_t curTime,
			      bool allowDrop,
			      bool *shouldBreak,
			      uint64_t *delayUs,
			      int64_t *compensationUs,
			      int64_t *timingErrorUs,
			      uint64_t *frameTsUs,
			      int logLevel);

	int
	scheduleFrame(uint64_t curTime, bool *load, int64_t *compensationUs);

	static void idleRenewMedia(void *userdata);

	static void idleStart(void *renderer);
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_RENDERER_GLES2_HPP_ */
