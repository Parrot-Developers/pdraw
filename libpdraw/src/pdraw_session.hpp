/**
 * Parrot Drones Awesome Video Viewer Library
 * Session
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

#ifndef _PDRAW_SESSION_HPP_
#define _PDRAW_SESSION_HPP_

#include "pdraw_avcdecoder.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_media.hpp"
#include "pdraw_metadata_session.hpp"
#include "pdraw_renderer.hpp"
#include "pdraw_settings.hpp"
#include "pdraw_sink_video.hpp"

#include <inttypes.h>

#ifdef _WIN32
#	include <winsock2.h>
#	undef OPAQUE
#	undef near
#	undef far
#	define IPTOS_PREC_INTERNETCONTROL 0xc0
#	define IPTOS_PREC_FLASHOVERRIDE 0x80
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <queue>
#include <string>
#include <vector>

#include <futils/futils.h>
#include <libpomp.h>
#include <pdraw/pdraw.hpp>

#ifdef _WIN32
#	define PIPE_BUF 4096
#endif /* _WIN32 */

namespace Pdraw {


class Settings;


class Session : public IPdraw,
		public Element::Listener,
		public Source::Listener,
		public Demuxer::Listener {
public:
	enum State {
		INVALID = 0,
		CREATED,
		OPENING,
		OPENED,
		CLOSING,
		CLOSED,
	};


	Session(struct pomp_loop *loop, IPdraw::Listener *listener);

	~Session(void);


	/*
	 * API methods
	 */

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
			       VideoRendererListener *listener,
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
			   VideoSinkListener *listener,
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

	void *getAndroidJvm(void)
	{
		return mAndroidJvm;
	}

	void setAndroidJvm(void *jvm)
	{
		mAndroidJvm = jvm;
	}


	/*
	 * Internal methods
	 */

	Demuxer *getDemuxer(void)
	{
		return mDemuxer;
	}

	Settings *getSettings(void)
	{
		return &mSettings;
	}

	SessionSelfMetadata *getSelfMetadata(void)
	{
		return &mSelfMetadata;
	}

	SessionPeerMetadata *getPeerMetadata(void)
	{
		return &mPeerMetadata;
	}

	int getSessionInfo(struct pdraw_session_info *sessionInfo);

	struct pomp_loop *getLoop()
	{
		return mLoop;
	}

	int asyncElementDelete(Element *element);

	int asyncAvcDecoderCompleteFlush(AvcDecoder *decoder);

	int asyncAvcDecoderCompleteStop(AvcDecoder *decoder);

	int asyncAvcDecoderResync(AvcDecoder *decoder);

	int asyncRendererCompleteStop(Renderer *renderer);

	/* Called on the loop thread */
	void socketCreated(int fd);

	int flushVideoSink(VideoSink *sink);

private:
	/* Called on the loop thread */
	void inloopElementDelete(Element *element);

	/* Called on the loop thread */
	void inloopAvcDecoderCompleteFlush(AvcDecoder *decoder);

	/* Called on the loop thread */
	void inloopAvcDecoderCompleteStop(AvcDecoder *decoder);

	/* Called on the loop thread */
	void inloopAvcDecoderResync(AvcDecoder *decoder);

	/* Called on the loop thread */
	void inloopRendererCompleteStop(Renderer *renderer);

	int addDecoderForMedia(Source *source, Media *media);

	void fillMediaInfo(VideoMedia *media, struct pdraw_media_info *info);

	int
	addMediaToRenderer(Source *source, Media *media, Renderer *renderer);

	int addMediaToAllRenderers(Source *source, Media *media);

	int addAllMediaToRenderer(Renderer *renderer);

	void setState(enum State state);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

	static void *runLoopThread(void *ptr);

	void onElementStateChanged(Element *element, Element::State state);

	void asyncElementStateChange(Element *element, Element::State state);

	void onOutputMediaAdded(Source *source, Media *media);

	void onOutputMediaRemoved(Source *source, Media *media);

	void onReadyToPlay(Demuxer *demuxer, bool ready);

	void onEndOfRange(Demuxer *demuxer, uint64_t timestamp);

	int selectDemuxerMedia(Demuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count);

	void
	playResp(Demuxer *demuxer, int status, uint64_t timestamp, float speed);

	void pauseResp(Demuxer *demuxer, int status, uint64_t timestamp);

	void
	seekResp(Demuxer *demuxer, int status, uint64_t timestamp, float speed);

	void updateReadyToPlay();

	void onUnrecoverableError(Demuxer *demuxer);

	static const char *stateStr(enum State val);

	IPdraw::Listener *mListener;
	enum State mState;
	struct pomp_loop *mLoop;
	struct mbox *mMbox;
	pthread_mutex_t mMutex;
	enum pdraw_session_type mSessionType;
	Settings mSettings;
	SessionSelfMetadata mSelfMetadata;
	SessionPeerMetadata mPeerMetadata;
	std::vector<Element *> mElements;
	Demuxer *mDemuxer;
	unsigned int mMediaIdCounter;
	void *mAndroidJvm;
	bool mDemuxerReady;
	bool mReadyToPlay;
	bool mUrecoverableErrorOccured;

	/* Listener calls from idle functions */
	static void callOpenResponse(void *userdata);
	static void callCloseResponse(void *userdata);
	static void callOnUnrecoverableError(void *userdata);
	/* callSelectMediaTrack omitted. Function has to be synchronous */
	static void callOnMediaAdded(void *userdata);
	static void callOnMediaRemoved(void *userdata);
	static void callReadyToPlay(void *userdata);
	static void callEndOfRange(void *userdata);
	static void callPlayResponse(void *userdata);
	static void callPauseResponse(void *userdata);
	static void callSeekResponse(void *userdata);
	static void callVideoSinkFlush(void *userdata);
	/* callOnSocketCreated omitted. Function has to be synchronous */
	std::queue<int> mOpenRespStatusArgs;
	std::queue<int> mCloseRespStatusArgs;
	std::queue<struct pdraw_media_info> mMediaAddedInfoArgs;
	std::queue<struct pdraw_media_info> mMediaRemovedInfoArgs;
	std::queue<bool> mReadyToPlayReadyArgs;
	std::queue<uint64_t> mEndOfRangeTimestampArgs;
	std::queue<int> mPlayRespStatusArgs;
	std::queue<uint64_t> mPlayRespTimestampArgs;
	std::queue<float> mPlayRespSpeedArgs;
	std::queue<int> mPauseRespStatusArgs;
	std::queue<uint64_t> mPauseRespTimestampArgs;
	std::queue<int> mSeekRespStatusArgs;
	std::queue<uint64_t> mSeekRespTimestampArgs;
	std::queue<float> mSeekRespSpeedArgs;
	std::queue<VideoSink *> mFlushVideoSinkArgs;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SESSION_HPP_ */
