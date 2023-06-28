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

#include "pdraw_decoder_video.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_encoder_video.hpp"
#include "pdraw_external_coded_video_sink.hpp"
#include "pdraw_external_coded_video_source.hpp"
#include "pdraw_external_raw_video_sink.hpp"
#include "pdraw_external_raw_video_source.hpp"
#include "pdraw_media.hpp"
#include "pdraw_muxer.hpp"
#include "pdraw_renderer_video.hpp"
#include "pdraw_settings.hpp"
#include "pdraw_source.hpp"
#include "pdraw_vipc_source.hpp"

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
#	include <netinet/ip.h>
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
		public Source::Listener {
public:
	enum State {
		STOPPED = 0,
		READY,
		STOPPING,
	};


	class PipelineFactory {
	public:
		PipelineFactory(Session *session);

		~PipelineFactory(void);

		void onElementStateChanged(Element *element,
					   Element::State state);

		void onOutputMediaAdded(Source *source, Media *media);

		void onOutputMediaRemoved(Source *source, Media *media);

		int dumpPipeline(const std::string &fileName);

		int addMediaToRenderer(unsigned int mediaId,
				       Pdraw::VideoRenderer *renderer);

		int
		addEncoderForMedia(Source *source,
				   RawVideoMedia *media,
				   const struct venc_config *params,
				   IPdraw::IVideoEncoder::Listener *listener,
				   Pdraw::VideoEncoder *encoder = nullptr);

	private:
		int addDecoderForMedia(Source *source, CodedVideoMedia *media);

		int addScalerForMedia(Source *source, RawVideoMedia *media);

		int addMediaToRenderer(Source *source,
				       RawVideoMedia *media,
				       Pdraw::VideoRenderer *renderer);

		int addMediaToAllRenderers(Source *source,
					   RawVideoMedia *media);

		int addAllMediaToRenderer(Pdraw::VideoRenderer *renderer);

		Session *mSession;
	};


	class Demuxer : public IPdraw::IDemuxer {
	public:
		Demuxer(Session *session,
			const std::string &url,
			struct mux_ctx *mux,
			IPdraw::IDemuxer::Listener *listener);

		Demuxer(Session *session,
			const std::string &localAddr,
			uint16_t localStreamPort,
			uint16_t localControlPort,
			const std::string &remoteAddr,
			uint16_t remoteStreamPort,
			uint16_t remoteControlPort,
			IPdraw::IDemuxer::Listener *listener);

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

		Element *getElement()
		{
			return mDemuxer;
		}

		Pdraw::Demuxer *getDemuxer()
		{
			return mDemuxer;
		}

	private:
		Session *mSession;
		Pdraw::Demuxer *mDemuxer;
	};


	class Muxer : public IPdraw::IMuxer {
	public:
		Muxer(Session *session,
		      const std::string &url,
		      const struct pdraw_muxer_params *params,
		      IPdraw::IMuxer::Listener *listener);

		~Muxer(void);

		int
		addMedia(unsigned int mediaId,
			 const struct pdraw_muxer_video_media_params *params);

		int setThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size);

		Element *getElement()
		{
			return mMuxer;
		}

		Pdraw::Muxer *getMuxer()
		{
			return mMuxer;
		}

	private:
		Session *mSession;
		Pdraw::Muxer *mMuxer;
	};


	class VideoRenderer : public IPdraw::IVideoRenderer {
	public:
		/* Called on the rendering thread */
		VideoRenderer(Session *session,
			      unsigned int mediaId,
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

		Element *getElement()
		{
			return mRenderer;
		}

	private:
		Pdraw::VideoRenderer *mRenderer;
	};


	class VipcSource : public IPdraw::IVipcSource {
	public:
		VipcSource(Session *session,
			   const struct pdraw_vipc_source_params *params,
			   IPdraw::IVipcSource::Listener *listener);

		~VipcSource(void);

		bool isReadyToPlay(void);

		bool isPaused(void);

		int play(void);

		int pause(void);

		int configure(const struct vdef_dim *resolution,
			      const struct vdef_rectf *crop);

		int setSessionMetadata(const struct vmeta_session *meta);

		int getSessionMetadata(struct vmeta_session *meta);

#ifdef BUILD_LIBVIDEO_IPC
		Element *getElement()
		{
			return mSource;
		}

		Source *getSource()
		{
			return mSource;
		}

		Pdraw::VipcSource *getVipcSource()
		{
			return mSource;
		}

	private:
		Pdraw::VipcSource *mSource;
#endif
	};


	class CodedVideoSource : public IPdraw::ICodedVideoSource {
	public:
		CodedVideoSource(Session *session,
				 const struct pdraw_video_source_params *params,
				 IPdraw::ICodedVideoSource::Listener *listener);

		~CodedVideoSource(void);

		struct mbuf_coded_video_frame_queue *getQueue(void);

		int flush(void);

		int setSessionMetadata(const struct vmeta_session *meta);

		int getSessionMetadata(struct vmeta_session *meta);

		Element *getElement()
		{
			return mSource;
		}

		Source *getSource()
		{
			return mSource;
		}

		Pdraw::ExternalCodedVideoSource *getCodedVideoSource()
		{
			return mSource;
		}

	private:
		Pdraw::ExternalCodedVideoSource *mSource;
	};


	class RawVideoSource : public IPdraw::IRawVideoSource {
	public:
		RawVideoSource(Session *session,
			       const struct pdraw_video_source_params *params,
			       IPdraw::IRawVideoSource::Listener *listener);

		~RawVideoSource(void);

		struct mbuf_raw_video_frame_queue *getQueue(void);

		int flush(void);

		int setSessionMetadata(const struct vmeta_session *meta);

		int getSessionMetadata(struct vmeta_session *meta);

		Element *getElement()
		{
			return mSource;
		}

		Source *getSource()
		{
			return mSource;
		}

		Pdraw::ExternalRawVideoSource *getRawVideoSource()
		{
			return mSource;
		}

	private:
		Pdraw::ExternalRawVideoSource *mSource;
	};


	class CodedVideoSink : public IPdraw::ICodedVideoSink {
	public:
		CodedVideoSink(Session *session,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::ICodedVideoSink::Listener *listener);

		~CodedVideoSink(void);

		int resync(void);

		struct mbuf_coded_video_frame_queue *getQueue(void);

		int queueFlushed(void);

		Element *getElement()
		{
			return mSink;
		}

		Sink *getSink()
		{
			return mSink;
		}

		Pdraw::ExternalCodedVideoSink *getCodedVideoSink()
		{
			return mSink;
		}

	private:
		Pdraw::ExternalCodedVideoSink *mSink;
	};


	class RawVideoSink : public IPdraw::IRawVideoSink {
	public:
		RawVideoSink(Session *session,
			     const struct pdraw_video_sink_params *params,
			     IPdraw::IRawVideoSink::Listener *listener);

		~RawVideoSink(void);

		int resync(void);

		struct mbuf_raw_video_frame_queue *getQueue(void);

		int queueFlushed(void);

		Element *getElement()
		{
			return mSink;
		}

		Sink *getSink()
		{
			return mSink;
		}

		Pdraw::ExternalRawVideoSink *getRawVideoSink()
		{
			return mSink;
		}

	private:
		Pdraw::ExternalRawVideoSink *mSink;
	};


	class VideoEncoderWrapper : public IPdraw::IVideoEncoder {
	public:
		VideoEncoderWrapper(Session *session,
				    const struct venc_config *params,
				    IPdraw::IVideoEncoder::Listener *listener);

		~VideoEncoderWrapper(void);

		int configure(const struct venc_dyn_config *config);

		Element *getElement()
		{
			return mEncoder;
		}

		Sink *getEncoder()
		{
			return mEncoder;
		}

		Pdraw::VideoEncoder *getVideoEncoder()
		{
			return mEncoder;
		}

	private:
		Pdraw::VideoEncoder *mEncoder;
	};


	Session(struct pomp_loop *loop, IPdraw::Listener *listener);

	~Session(void);


	/*
	 * API methods
	 */

	int stop(void);

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

	int createMuxer(const std::string &url,
			const struct pdraw_muxer_params *params,
			IPdraw::IMuxer::Listener *listener,
			IPdraw::IMuxer **retObj);

	/* Called on the rendering thread */
	int
	createVideoRenderer(unsigned int mediaId,
			    const struct pdraw_rect *renderPos,
			    const struct pdraw_video_renderer_params *params,
			    IPdraw::IVideoRenderer::Listener *listener,
			    IPdraw::IVideoRenderer **retObj,
			    struct egl_display *eglDisplay = nullptr);

	int createVipcSource(const struct pdraw_vipc_source_params *params,
			     IPdraw::IVipcSource::Listener *listener,
			     IPdraw::IVipcSource **retObj);

	int
	createCodedVideoSource(const struct pdraw_video_source_params *params,
			       IPdraw::ICodedVideoSource::Listener *listener,
			       IPdraw::ICodedVideoSource **retObj);

	int createRawVideoSource(const struct pdraw_video_source_params *params,
				 IPdraw::IRawVideoSource::Listener *listener,
				 IPdraw::IRawVideoSource **retObj);

	int createCodedVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::ICodedVideoSink::Listener *listener,
				 IPdraw::ICodedVideoSink **retObj);

	int createRawVideoSink(unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::IRawVideoSink::Listener *listener,
			       IPdraw::IRawVideoSink **retObj);

	int createVideoEncoder(unsigned int mediaId,
			       const struct venc_config *params,
			       IPdraw::IVideoEncoder::Listener *listener,
			       IPdraw::IVideoEncoder **retObj);

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

	void *getAndroidJvm(void)
	{
		return mAndroidJvm;
	}

	void setAndroidJvm(void *jvm)
	{
		mAndroidJvm = jvm;
	}

	int dumpPipeline(const std::string &fileName);


	/*
	 * Internal methods
	 */

	Settings *getSettings(void)
	{
		return &mSettings;
	}

	struct pomp_loop *getLoop()
	{
		return mLoop;
	}

	int addMediaToRenderer(unsigned int mediaId,
			       Pdraw::VideoRenderer *renderer);

	void asyncElementDelete(Element *element);

	/* Called on the loop thread */
	void socketCreated(int fd);

private:
	int internalCreateCodedVideoSink(
		Source *source,
		CodedVideoMedia *media,
		const struct pdraw_video_sink_params *params,
		ICodedVideoSink::Listener *listener,
		ICodedVideoSink **retObj);

	int
	internalCreateRawVideoSink(Source *source,
				   RawVideoMedia *media,
				   const struct pdraw_video_sink_params *params,
				   IRawVideoSink::Listener *listener,
				   IRawVideoSink **retObj);

	int
	internalCreateVideoEncoder(Source *source,
				   RawVideoMedia *media,
				   const struct venc_config *params,
				   IPdraw::IVideoEncoder::Listener *listener,
				   IPdraw::IVideoEncoder **retObj);

	void setState(enum State state);

	static void *runLoopThread(void *ptr);

	void onElementStateChanged(Element *element, Element::State state);

	void asyncElementStateChange(Element *element, Element::State state);

	void
	onOutputMediaAdded(Source *source, Media *media, void *elementUserData);

	void onOutputMediaRemoved(Source *source,
				  Media *media,
				  void *elementUserData);

	void stopResp(int status);

	static const char *stateStr(enum State val);

	PipelineFactory mFactory;
	IPdraw::Listener *mListener;
	enum State mState;
	struct pomp_loop *mLoop;
	pthread_t mLoopThread;
	pthread_mutex_t mMutex;
	Settings mSettings;
	std::vector<Element *> mElements;
	void *mAndroidJvm;

	/* Calls from idle functions */
	pthread_mutex_t mAsyncMutex;
	static void idleElementStateChange(void *userdata);
	std::queue<Element *> mElementStateChangeElementArgs;
	std::queue<Element::State> mElementStateChangeStateArgs;
	static void idleElementDelete(void *userdata);
	std::queue<Element *> mElementDeleteElementArgs;
	static void callStopResponse(void *userdata);
	std::queue<int> mStopRespStatusArgs;
	static void callOnMediaAdded(void *userdata);
	std::queue<struct pdraw_media_info> mMediaAddedInfoArgs;
	std::queue<void *> mMediaAddedElementUserDataArgs;
	static void callOnMediaRemoved(void *userdata);
	std::queue<struct pdraw_media_info> mMediaRemovedInfoArgs;
	std::queue<void *> mMediaRemovedElementUserDataArgs;
	/* callOnSocketCreated omitted. Function has to be synchronous */
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SESSION_HPP_ */
