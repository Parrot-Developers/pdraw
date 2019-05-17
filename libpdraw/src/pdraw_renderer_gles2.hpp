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

namespace Pdraw {

class Gles2Renderer : public Renderer {
public:
	Gles2Renderer(Session *session,
		      Element::Listener *listener,
		      IPdraw::VideoRendererListener *rndListener);

	~Gles2Renderer(void);

	virtual int setup(const struct pdraw_rect *renderPos,
			  const struct pdraw_video_renderer_params *params,
			  struct egl_display *eglDisplay);

	int start(void);

	int stop(void);

	int render(struct pdraw_rect *contentPos,
		   const float *viewMat = NULL,
		   const float *projMat = NULL);

	int resize(const struct pdraw_rect *renderPos);

	int setParams(const struct pdraw_video_renderer_params *params);


	int getParams(struct pdraw_video_renderer_params *params);

	int addInputMedia(Media *media);

	int removeInputMedia(Media *media);

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

	int startHmd(void);

	int stopHmd(void);

	int startExtLoad(void);

	int stopExtLoad(void);

	void onChannelFlush(Channel *channel);

	void onChannelSos(Channel *channel);

	void onChannelEos(Channel *channel);

	void onChannelReconfigure(Channel *channel);

	void onChannelTimeout(Channel *channel);

	void onChannelPhotoTrigger(Channel *channel);

	int doTransition(uint64_t timestamp, bool frameReady, bool *loadFrame);

	void abortTransition(void);

	virtual int loadVideoFrame(const uint8_t *data,
				   VideoMedia::Frame *frame);

	void createProjMatrix(Eigen::Matrix4f &projMat,
			      float aspectRatio,
			      float near,
			      float far);

	void updateViewProjMatrix(Eigen::Matrix4f &viewProjMat,
				  vmeta_frame *meta);

	virtual int
	loadExternalVideoFrame(const uint8_t *data,
			       VideoMedia::Frame *frame,
			       const struct pdraw_session_info *session_info,
			       const struct vmeta_session *session_meta);

	virtual int renderVideoFrame(VideoMedia::Frame *frame,
				     const struct pdraw_rect *renderPos,
				     struct pdraw_rect *contentPos,
				     Eigen::Matrix4f &viewProjMat);

	virtual int renderExternalVideoFrame(VideoMedia::Frame *frame,
					     const struct pdraw_rect *renderPos,
					     struct pdraw_rect *contentPos,
					     Eigen::Matrix4f &viewProjMat);

	static void timerCb(struct pomp_timer *timer, void *userdata);

	static void queueEventCb(struct pomp_evt *evt, void *userdata);

	int removeQueueFdFromPomp(struct vbuf_queue *queue);

	bool mRunning;
	bool mRendering;
	pthread_mutex_t mRenderMutex;
	bool mRenderMutexCreated;
	pthread_cond_t mRenderCond;
	bool mRenderCondCreated;
	struct vbuf_buffer *mCurrentBuffer;
	Media *mLastAddedMedia;
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
	enum gles2_video_color_conversion mColorConversion;
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

private:
	int setupExtTexture(const VideoMedia::Frame *frame);
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_RENDERER_GLES2_HPP_ */
