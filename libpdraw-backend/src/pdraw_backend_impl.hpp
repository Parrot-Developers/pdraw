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
		     public IPdraw::IDemuxer::Listener,
		     public IPdraw::IVideoRenderer::Listener,
		     public IPdraw::ICodedVideoSink::Listener,
		     public IPdraw::IRawVideoSink::Listener {
public:
	class Demuxer : public IPdraw::IDemuxer {
	public:
		Demuxer(PdrawBackend *backend,
			IPdraw::IDemuxer::Listener *listener) :
				mBackend(backend),
				mListener(listener), mDemuxer(nullptr)
		{
		}

		~Demuxer(void);

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

		IPdraw::IDemuxer *getDemuxer()
		{
			return mDemuxer;
		}

		void setDemuxer(IPdraw::IDemuxer *demuxer)
		{
			mDemuxer = demuxer;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IDemuxer::Listener *mListener;
		IPdraw::IDemuxer *mDemuxer;
	};

	class Muxer : public IPdraw::IMuxer {
	public:
		Muxer(PdrawBackend *backend, const std::string &url);

		~Muxer(void);

		int
		addMedia(unsigned int mediaId,
			 const struct pdraw_muxer_video_media_params *params);

		IPdraw::IMuxer *getMuxer()
		{
			return mMuxer;
		}

		void setMuxer(IPdraw::IMuxer *muxer)
		{
			mMuxer = muxer;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IMuxer *mMuxer;
	};

	class VideoRenderer : public IPdraw::IVideoRenderer {
	public:
		/* Called on the rendering thread */
		VideoRenderer(PdrawBackend *backend,
			      const struct pdraw_rect *renderPos,
			      const struct pdraw_video_renderer_params *params,
			      IPdraw::IVideoRenderer::Listener *listener,
			      struct egl_display *eglDisplay = nullptr);

		/* Called on the rendering thread */
		~VideoRenderer(void);

		/* Called on the rendering thread */
		int resize(const struct pdraw_rect *renderPos);

		/* Called on the rendering thread */
		int setMediaId(unsigned int mediaId);

		/* Called on the rendering thread */
		unsigned int getMediaId(void);

		/* Called on the rendering thread */
		int setParams(const struct pdraw_video_renderer_params *params);

		/* Called on the rendering thread */
		int getParams(struct pdraw_video_renderer_params *params);

		/* Called on the rendering thread */
		int render(struct pdraw_rect *contentPos,
			   const float *viewMat = nullptr,
			   const float *projMat = nullptr);

		IPdraw::IVideoRenderer *getRenderer()
		{
			return mRenderer;
		}

		void setRenderer(IPdraw::IVideoRenderer *renderer)
		{
			mRenderer = renderer;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IVideoRenderer::Listener *mListener;
		IPdraw::IVideoRenderer *mRenderer;
	};

	class CodedVideoSink : public IPdraw::ICodedVideoSink {
	public:
		CodedVideoSink(PdrawBackend *backend,
			       unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::ICodedVideoSink::Listener *listener);

		~CodedVideoSink(void);

		int resync(void);

		struct mbuf_coded_video_frame_queue *getQueue(void);

		int queueFlushed(void);

		IPdraw::ICodedVideoSink *getCodedVideoSink()
		{
			return mSink;
		}

		void setCodedVideoSink(IPdraw::ICodedVideoSink *sink)
		{
			mSink = sink;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::ICodedVideoSink::Listener *mListener;
		IPdraw::ICodedVideoSink *mSink;
	};

	class RawVideoSink : public IPdraw::IRawVideoSink {
	public:
		RawVideoSink(PdrawBackend *backend,
			     unsigned int mediaId,
			     const struct pdraw_video_sink_params *params,
			     IPdraw::IRawVideoSink::Listener *listener);

		~RawVideoSink(void);

		int resync(void);

		struct mbuf_raw_video_frame_queue *getQueue(void);

		int queueFlushed(void);

		IPdraw::IRawVideoSink *getRawVideoSink()
		{
			return mSink;
		}

		void setRawVideoSink(IPdraw::IRawVideoSink *sink)
		{
			mSink = sink;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IRawVideoSink::Listener *mListener;
		IPdraw::IRawVideoSink *mSink;
	};

	PdrawBackend(IPdrawBackend::Listener *listener);

	~PdrawBackend(void);

	int start(void);

	int stop(void);

	struct pomp_loop *getLoop(void);

	int createDemuxer(const std::string &url,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj);

	int createDemuxer(const std::string &localAddr,
			  uint16_t localStreamPort,
			  uint16_t localControlPort,
			  const std::string &remoteAddr,
			  uint16_t remoteStreamPort,
			  uint16_t remoteControlPort,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj);

	int createDemuxer(const std::string &url,
			  struct mux_ctx *mux,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj);

	int createMuxer(const std::string &url, IPdraw::IMuxer **retObj);

	/* Called on the rendering thread */
	int
	createVideoRenderer(unsigned int mediaId,
			    const struct pdraw_rect *renderPos,
			    const struct pdraw_video_renderer_params *params,
			    IPdraw::IVideoRenderer::Listener *listener,
			    IPdraw::IVideoRenderer **retObj,
			    struct egl_display *eglDisplay = nullptr);

	int createCodedVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::ICodedVideoSink::Listener *listener,
				 IPdraw::ICodedVideoSink **retObj);

	int createRawVideoSink(unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::IRawVideoSink::Listener *listener,
			       IPdraw::IRawVideoSink **retObj);

	void getFriendlyNameSetting(std::string *friendlyName);

	void setFriendlyNameSetting(const std::string &friendlyName);

	void getSerialNumberSetting(std::string *serialNumber);

	void setSerialNumberSetting(const std::string &serialNumber);

	void getSoftwareVersionSetting(std::string *softwareVersion);

	void setSoftwareVersionSetting(const std::string &softwareVersion);

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

	int dumpPipeline(const std::string &fileName);

private:
	void stopResponse(IPdraw *pdraw, int status);

	void onMediaAdded(IPdraw *pdraw, const struct pdraw_media_info *info);

	void onMediaRemoved(IPdraw *pdraw, const struct pdraw_media_info *info);

	void onSocketCreated(IPdraw *pdraw, int fd);

	void demuxerOpenResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status);

	void demuxerCloseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status);

	void onDemuxerUnrecoverableError(IPdraw *pdraw,
					 IPdraw::IDemuxer *demuxer);

	int demuxerSelectMedia(IPdraw *pdraw,
			       IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count);

	void demuxerReadyToPlay(IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer,
				bool ready);

	void onDemuxerEndOfRange(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp);

	void demuxerPlayResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed);

	void demuxerPauseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp);

	void demuxerSeekResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed);

	void onVideoRendererMediaAdded(IPdraw *pdraw,
				       IPdraw::IVideoRenderer *renderer,
				       const struct pdraw_media_info *info);

	void onVideoRendererMediaRemoved(IPdraw *pdraw,
					 IPdraw::IVideoRenderer *renderer,
					 const struct pdraw_media_info *info);

	void onVideoRenderReady(IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer);

	int loadVideoTexture(IPdraw *pdraw,
			     IPdraw::IVideoRenderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_media_info *mediaInfo,
			     struct mbuf_raw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen);

	int
	renderVideoOverlay(IPdraw *pdraw,
			   IPdraw::IVideoRenderer *renderer,
			   const struct pdraw_rect *renderPos,
			   const struct pdraw_rect *contentPos,
			   const float *viewMat,
			   const float *projMat,
			   const struct pdraw_media_info *mediaInfo,
			   struct vmeta_frame *frameMeta,
			   const struct pdraw_video_frame_extra *frameExtra);

	void onCodedVideoSinkFlush(IPdraw *pdraw,
				   IPdraw::ICodedVideoSink *sink);

	void onRawVideoSinkFlush(IPdraw *pdraw, IPdraw::IRawVideoSink *sink);

	static void *loopThread(void *ptr);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

	int doCreateDemuxer(const std::string &url,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateDemuxer(const std::string &localAddr,
			    uint16_t localStreamPort,
			    uint16_t localControlPort,
			    const std::string &remoteAddr,
			    uint16_t remoteStreamPort,
			    uint16_t remoteControlPort,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateDemuxer(const std::string &url,
			    struct mux_ctx *mux,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateMuxer(const std::string &url, IPdraw::IMuxer **retObj);

	int doCreateCodedVideoSink(unsigned int mediaId,
				   const struct pdraw_video_sink_params *params,
				   IPdraw::ICodedVideoSink::Listener *listener,
				   IPdraw::ICodedVideoSink **retObj);

	int doCreateRawVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::IRawVideoSink::Listener *listener,
				 IPdraw::IRawVideoSink **retObj);

	void internalStop(void);

	void internalDemuxerCreate(const std::string &url,
				   IPdraw::IDemuxer::Listener *listener);

	void internalDemuxerCreate(const std::string &localAddr,
				   uint16_t localStreamPort,
				   uint16_t localControlPort,
				   const std::string &remoteAddr,
				   uint16_t remoteStreamPort,
				   uint16_t remoteControlPort,
				   IPdraw::IDemuxer::Listener *listener);

	void internalDemuxerCreate(const std::string &url,
				   struct mux_ctx *mux,
				   IPdraw::IDemuxer::Listener *listener);

	void internalDemuxerDestroy(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerClose(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerGetSingleStreamLocalStreamPort(
		PdrawBackend::Demuxer *demuxer);

	void internalDemuxerGetSingleStreamLocalControlPort(
		PdrawBackend::Demuxer *demuxer);

	void internalDemuxerIsReadyToPlay(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerIsPaused(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerPlay(PdrawBackend::Demuxer *demuxer,
				 float speed = 1.0f);

	void internalDemuxerPreviousFrame(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerNextFrame(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerSeek(PdrawBackend::Demuxer *demuxer,
				 int64_t delta,
				 bool exact = false);

	void internalDemuxerSeekTo(PdrawBackend::Demuxer *demuxer,
				   uint64_t timestamp,
				   bool exact = false);

	void internalDemuxerGetDuration(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerGetCurrentTime(PdrawBackend::Demuxer *demuxer);

	void internalMuxerCreate(const std::string &url);

	void internalMuxerDestroy(PdrawBackend::Muxer *muxer);

	void internalMuxerAddMedia(
		PdrawBackend::Muxer *muxer,
		unsigned int mediaId,
		const struct pdraw_muxer_video_media_params *params);

	void internalCodedVideoSinkCreate(
		unsigned int mediaId,
		const struct pdraw_video_sink_params *params,
		IPdraw::ICodedVideoSink::Listener *listener);

	void internalCodedVideoSinkDestroy(PdrawBackend::CodedVideoSink *sink);

	void internalCodedVideoSinkResync(PdrawBackend::CodedVideoSink *sink);

	void internalCodedVideoSinkGetQueue(PdrawBackend::CodedVideoSink *sink);

	void
	internalCodedVideoSinkQueueFlushed(PdrawBackend::CodedVideoSink *sink);

	void
	internalRawVideoSinkCreate(unsigned int mediaId,
				   const struct pdraw_video_sink_params *params,
				   IPdraw::IRawVideoSink::Listener *listener);

	void internalRawVideoSinkDestroy(PdrawBackend::RawVideoSink *sink);

	void internalRawVideoSinkGetQueue(PdrawBackend::RawVideoSink *sink);

	void internalRawVideoSinkQueueFlushed(PdrawBackend::RawVideoSink *sink);

	void internalGetFriendlyNameSetting(void);

	void internalSetFriendlyNameSetting(const std::string &friendlyName);

	void internalGetSerialNumberSetting(void);

	void internalSetSerialNumberSetting(const std::string &serialNumber);

	void internalGetSoftwareVersionSetting(void);

	void
	internalSetSoftwareVersionSetting(const std::string &softwareVersion);

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

	void internalDumpPipeline(const std::string &fileName);

	struct demuxerAndListener {
		IPdraw::IDemuxer *d;
		IPdraw::IDemuxer::Listener *l;
	};

	struct videoRendererAndListener {
		IPdraw::IVideoRenderer *r;
		IPdraw::IVideoRenderer::Listener *l;
	};

	struct codedVideoSinkAndListener {
		IPdraw::ICodedVideoSink *s;
		IPdraw::ICodedVideoSink::Listener *l;
	};

	struct rawVideoSinkAndListener {
		IPdraw::IRawVideoSink *s;
		IPdraw::IRawVideoSink::Listener *l;
	};

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
	enum pdraw_pipeline_mode mRetPipelineMode;
	enum pdraw_hmd_model mRetHmdModel;
	struct mbuf_coded_video_frame_queue *mRetCodedQueue;
	struct mbuf_raw_video_frame_queue *mRetRawQueue;
	struct pdraw_media_info mRetMediaInfo;
	IPdraw::ICodedVideoSink *mRetCodedVideoSink;
	IPdraw::IRawVideoSink *mRetRawVideoSink;
	IPdraw::IDemuxer *mRetDemuxer;
	IPdraw::IMuxer *mRetMuxer;
	IPdraw *mPdraw;
	IPdraw::Listener *mListener;
	pthread_mutex_t mMapsMutex;
	bool mMapsMutexCreated;
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>
		mDemuxerListenersMap;
	struct demuxerAndListener mPendingDemuxerAndListener;
	std::map<IPdraw::IVideoRenderer *, struct videoRendererAndListener>
		mVideoRendererListenersMap;
	struct videoRendererAndListener mPendingVideoRendererAndListener;
	std::map<IPdraw::ICodedVideoSink *, struct codedVideoSinkAndListener>
		mCodedVideoSinkListenersMap;
	struct codedVideoSinkAndListener mPendingCodedVideoSinkAndListener;
	std::map<IPdraw::IRawVideoSink *, struct rawVideoSinkAndListener>
		mRawVideoSinkListenersMap;
	struct rawVideoSinkAndListener mPendingRawVideoSinkAndListener;
};

} /* namespace PdrawBackend */

#endif /* !_PDRAW_BACKEND_IMPL_HPP_ */
