/**
 * Parrot Drones Awesome Video Viewer
 * PDrAW back-end library
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

#ifndef _PDRAW_BACKEND_IMPL_HPP_
#define _PDRAW_BACKEND_IMPL_HPP_

#include <pthread.h>

#include <map>
#include <vector>

#include <futils/futils.h>
#include <libpomp.h>
#include <pdraw/pdraw_backend.hpp>

namespace PdrawBackend {


class PdrawBackend : public IPdrawBackend,
		     public IPdraw::Listener,
		     public IPdraw::VideoRendererListener,
		     public IPdraw::VideoSinkListener {
public:
	PdrawBackend(IPdrawBackend::Listener *listener);

	~PdrawBackend(void);

	int start(void);

	int stop(void);

	struct pomp_loop *getLoop(void);

	int open(const std::string &url);

	int open(const std::string &localAddr,
		 uint16_t localStreamPort,
		 uint16_t localControlPort,
		 const std::string &remoteAddr,
		 uint16_t remoteStreamPort,
		 uint16_t remoteControlPort);

	int open(const std::string &url, struct mux_ctx *mux);

	int close(void);

	uint16_t getSingleStreamLocalStreamPort(void);

	uint16_t getSingleStreamLocalControlPort(void);

	bool isReadyToPlay(void);

	bool isPaused(void);

	int play(float speed = 1.0f);

	int pause(void);

	int previousFrame(void);

	int nextFrame(void);

	int seek(int64_t delta, bool exact = false);

	int seekForward(uint64_t delta, bool exact = false);

	int seekBack(uint64_t delta, bool exact = false);

	int seekTo(uint64_t timestamp, bool exact = false);

	uint64_t getDuration(void);

	uint64_t getCurrentTime(void);

	/* Called on the rendering thread */
	int startVideoRenderer(const struct pdraw_rect *renderPos,
			       const struct pdraw_video_renderer_params *params,
			       IPdrawBackend::VideoRendererListener *listener,
			       struct pdraw_video_renderer **retObj,
			       struct egl_display *eglDisplay = NULL);

	/* Called on the rendering thread */
	int stopVideoRenderer(struct pdraw_video_renderer *renderer);

	/* Called on the rendering thread */
	int resizeVideoRenderer(struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *renderPos);


	/* Called on the rendering thread */
	int setVideoRendererParams(
		struct pdraw_video_renderer *renderer,
		const struct pdraw_video_renderer_params *params);


	/* Called on the rendering thread */
	int getVideoRendererParams(struct pdraw_video_renderer *renderer,
				   struct pdraw_video_renderer_params *params);

	/* Called on the rendering thread */
	int renderVideo(struct pdraw_video_renderer *renderer,
			struct pdraw_rect *contentPos,
			const float *viewMat = NULL,
			const float *projMat = NULL);

	int startVideoSink(unsigned int mediaId,
			   const struct pdraw_video_sink_params *params,
			   IPdrawBackend::VideoSinkListener *listener,
			   struct pdraw_video_sink **retObj);

	int stopVideoSink(struct pdraw_video_sink *sink);

	int resyncVideoSink(struct pdraw_video_sink *sink);

	struct vbuf_queue *getVideoSinkQueue(struct pdraw_video_sink *sink);

	int videoSinkQueueFlushed(struct pdraw_video_sink *sink);

	enum pdraw_session_type getSessionType(void);

	void getSelfFriendlyName(std::string *friendlyName);

	void setSelfFriendlyName(const std::string &friendlyName);

	void getSelfSerialNumber(std::string *serialNumber);

	void setSelfSerialNumber(const std::string &serialNumber);

	void getSelfSoftwareVersion(std::string *softwareVersion);

	void setSelfSoftwareVersion(const std::string &softwareVersion);

	bool isSelfPilot(void);

	void setSelfPilot(bool isPilot);

	void getPeerSessionMetadata(struct vmeta_session *session);

	enum pdraw_drone_model getPeerDroneModel(void);

	enum pdraw_pipeline_mode getPipelineModeSetting(void);

	void setPipelineModeSetting(enum pdraw_pipeline_mode mode);

	void getDisplayScreenSettings(float *xdpi,
				      float *ydpi,
				      float *deviceMarginTop,
				      float *deviceMarginBottom,
				      float *deviceMarginLeft,
				      float *deviceMarginRight);

	void setDisplayScreenSettings(float xdpi,
				      float ydpi,
				      float deviceMarginTop,
				      float deviceMarginBottom,
				      float deviceMarginLeft,
				      float deviceMarginRight);

	enum pdraw_hmd_model getHmdModelSetting(void);

	void setHmdModelSetting(enum pdraw_hmd_model hmdModel);

	void setAndroidJvm(void *jvm);

private:
	void openResponse(IPdraw *pdraw, int status);

	void closeResponse(IPdraw *pdraw, int status);

	void onUnrecoverableError(IPdraw *pdraw);

	int selectDemuxerMedia(IPdraw *pdraw,
			       const struct pdraw_demuxer_media *medias,
			       size_t count);

	void onMediaAdded(IPdraw *pdraw, const struct pdraw_media_info *info);

	void onMediaRemoved(IPdraw *pdraw, const struct pdraw_media_info *info);

	void readyToPlay(IPdraw *pdraw, bool ready);

	void onEndOfRange(IPdraw *pdraw, uint64_t timestamp);

	void playResponse(IPdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed);

	void pauseResponse(IPdraw *pdraw, int status, uint64_t timestamp);

	void seekResponse(IPdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed);

	void onSocketCreated(IPdraw *pdraw, int fd);

	void onVideoRenderReady(IPdraw *pdraw,
				struct pdraw_video_renderer *renderer);

	int loadVideoTexture(IPdraw *pdraw,
			     struct pdraw_video_renderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_session_info *sessionInfo,
			     const struct vmeta_session *sessionMeta,
			     const struct pdraw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen);

	int
	renderVideoOverlay(IPdraw *pdraw,
			   struct pdraw_video_renderer *renderer,
			   const struct pdraw_rect *renderPos,
			   const struct pdraw_rect *contentPos,
			   const float *viewMat,
			   const float *projMat,
			   const struct pdraw_session_info *sessionInfo,
			   const struct vmeta_session *sessionMeta,
			   const struct vmeta_frame *frameMeta,
			   const struct pdraw_video_frame_extra *frameExtra);

	void onVideoSinkFlush(IPdraw *pdraw, struct pdraw_video_sink *sink);

	static void *loopThread(void *ptr);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

	int doStartVideoSink(unsigned int mediaId,
			     const struct pdraw_video_sink_params *params,
			     IPdrawBackend::VideoSinkListener *listener,
			     struct pdraw_video_sink **retObj);

	void internalOpen(const std::string &url);

	void internalOpen(const std::string &localAddr,
			  uint16_t localStreamPort,
			  uint16_t localControlPort,
			  const std::string &remoteAddr,
			  uint16_t remoteStreamPort,
			  uint16_t remoteControlPort);

	void internalOpen(const std::string &url, struct mux_ctx *mux);

	void internalClose(void);

	void internalGetSingleStreamLocalStreamPort(void);

	void internalGetSingleStreamLocalControlPort(void);

	void internalIsReadyToPlay(void);

	void internalIsPaused(void);

	void internalPlay(float speed = 1.0f);

	void internalPreviousFrame(void);

	void internalNextFrame(void);

	void internalSeek(int64_t delta, bool exact = false);

	void internalSeekTo(uint64_t timestamp, bool exact = false);

	void internalGetDuration(void);

	void internalGetCurrentTime(void);

	void
	internalStartVideoSink(unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdrawBackend::VideoSinkListener *listener);

	void internalStopVideoSink(struct pdraw_video_sink *sink);

	void internalResyncVideoSink(struct pdraw_video_sink *sink);

	void internalGetVideoSinkQueue(struct pdraw_video_sink *sink);

	void internalVideoSinkQueueFlushed(struct pdraw_video_sink *sink);

	void internalGetSessionType(void);

	void internalGetSelfFriendlyName(void);

	void internalSetSelfFriendlyName(const std::string &friendlyName);

	void internalGetSelfSerialNumber(void);

	void internalSetSelfSerialNumber(const std::string &serialNumber);

	void internalGetSelfSoftwareVersion(void);

	void internalSetSelfSoftwareVersion(const std::string &softwareVersion);

	void internalIsSelfPilot(void);

	void internalSetSelfPilot(bool isPilot);

	void internalGetPeerSessionMetadata(void);

	void internalGetPeerDroneModel(void);

	void internalGetPipelineModeSetting(void);

	void internalSetPipelineModeSetting(enum pdraw_pipeline_mode mode);

	void internalGetDisplayScreenSettings(void);

	void internalSetDisplayScreenSettings(float xdpi,
					      float ydpi,
					      float deviceMarginTop,
					      float deviceMarginBottom,
					      float deviceMarginLeft,
					      float deviceMarginRight);

	void internalGetHmdModelSetting(void);

	void internalSetHmdModelSetting(enum pdraw_hmd_model hmdModel);

	void internalSetAndroidJvm(void *jvm);

	pthread_mutex_t mApiMutex;
	bool mApiMutexCreated;
	bool mApiReady;
	pthread_mutex_t mMutex;
	bool mMutexCreated;
	pthread_cond_t mCond;
	bool mCondCreated;
	pthread_t mLoopThread;
	bool mLoopThreadLaunched;
	bool mThreadShouldStop;
	struct pomp_loop *mLoop;
	struct mbox *mMbox;
	bool mStarted;
	bool mRetValReady;
	int mRetStatus;
	bool mRetBool;
	uint16_t mRetUint16;
	uint64_t mRetUint64;
	float mRetFloat[6];
	std::string mRetString;
	enum pdraw_session_type mRetSessionType;
	enum pdraw_drone_model mRetDroneModel;
	enum pdraw_pipeline_mode mRetPipelineMode;
	enum pdraw_hmd_model mRetHmdModel;
	struct vbuf_queue *mRetQueue;
	struct pdraw_media_info mRetMediaInfo;
	struct pdraw_video_sink *mRetVideoSink;
	struct vmeta_session mRetSessionMeta;
	IPdraw *mPdraw;
	IPdrawBackend::Listener *mListener;
	pthread_mutex_t mMapsMutex;
	bool mMapsMutexCreated;
	std::map<struct pdraw_video_renderer *,
		 IPdrawBackend::VideoRendererListener *>
		mVideoRendererListenersMap;
	IPdrawBackend::VideoRendererListener *mPendingVideoRendererListener;
	std::map<struct pdraw_video_sink *, IPdrawBackend::VideoSinkListener *>
		mVideoSinkListenersMap;
	IPdrawBackend::VideoSinkListener *mPendingVideoSinkListener;
};

} /* namespace PdrawBackend */

#endif /* !_PDRAW_BACKEND_IMPL_HPP_ */
