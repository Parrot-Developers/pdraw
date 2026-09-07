/**
 * Parrot Drones Audio and Video Vector
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

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <futils/futils.h>
#include <libpomp.hpp>
#include <pdraw/pdraw_backend.hpp>


#define PDRAW_CHECK_LOOP_THREAD() checkLoopThread(__func__)


/* Disable copy constructor and assignment operator */
#define PDRAW_DISABLE_COPY(_cls)                                               \
private:                                                                       \
	_cls(const _cls &);                                                    \
	_cls &operator=(const _cls &);


namespace PdrawBackend {

template <typename T> class IPostCreatable {
public:
	virtual void postCreate(T *impl) noexcept = 0;

protected:
	~IPostCreatable() = default;
};


template <typename T> struct ElementAndListener {
	T *e;
	typename T::Listener *l;

	void setElement(T *n)
	{
		e = n;
	}

	T *getElement() const
	{
		return e;
	}
};


template <typename T> class ElementWithListener {
	PDRAW_DISABLE_COPY(ElementWithListener)

public:
	explicit ElementWithListener(typename T::Listener *l) :
			mElementAndListener{
				.e = nullptr,
				.l = l,
			}
	{
	}

	void setElement(T *n)
	{
		mOwnedElement.reset(n);
		mElementAndListener.setElement(n);
	}

	T *getElement() const
	{
		return mElementAndListener.getElement();
	}

	void resetElement()
	{
		mOwnedElement.reset();
		mElementAndListener.setElement(nullptr);
	}

	bool hasElement() const
	{
		return (mElementAndListener.getElement() != nullptr);
	}

private:
	/* Owns the inner element; mElementAndListener holds a non-owning
	 * view so it remains copyable for use in maps and pending structs */
	std::unique_ptr<T> mOwnedElement;
	ElementAndListener<T> mElementAndListener;
};


using demuxerAndListener = ElementAndListener<IPdraw::IDemuxer>;
using muxerAndListener = ElementAndListener<IPdraw::IMuxer>;
using videoRendererAndListener = ElementAndListener<IPdraw::IVideoRenderer>;
using audioRendererAndListener = ElementAndListener<IPdraw::IAudioRenderer>;
using vipcSourceAndListener = ElementAndListener<IPdraw::IVipcSource>;
using codedVideoSourceAndListener =
	ElementAndListener<IPdraw::ICodedVideoSource>;
using rawVideoSourceAndListener = ElementAndListener<IPdraw::IRawVideoSource>;
using codedVideoSinkAndListener = ElementAndListener<IPdraw::ICodedVideoSink>;
using rawVideoSinkAndListener = ElementAndListener<IPdraw::IRawVideoSink>;
using alsaSourceAndListener = ElementAndListener<IPdraw::IAlsaSource>;
using audioSourceAndListener = ElementAndListener<IPdraw::IAudioSource>;
using audioSinkAndListener = ElementAndListener<IPdraw::IAudioSink>;
using videoEncoderAndListener = ElementAndListener<IPdraw::IVideoEncoder>;
using videoScalerAndListener = ElementAndListener<IPdraw::IVideoScaler>;
using audioEncoderAndListener = ElementAndListener<IPdraw::IAudioEncoder>;


class PdrawBackend : public IPdrawBackend,
		     public IPdraw::Listener,
		     public IPdraw::IDemuxer::Listener,
		     public IPdraw::IMuxer::Listener,
		     public IPdraw::IVideoRenderer::Listener,
		     public IPdraw::IAudioRenderer::Listener,
		     public IPdraw::IVipcSource::Listener,
		     public IPdraw::ICodedVideoSource::Listener,
		     public IPdraw::IRawVideoSource::Listener,
		     public IPdraw::ICodedVideoSink::Listener,
		     public IPdraw::IRawVideoSink::Listener,
		     public IPdraw::IVideoEncoder::Listener,
		     public IPdraw::IVideoScaler::Listener,
		     public IPdraw::IAudioEncoder::Listener,
		     public IPdraw::IAlsaSource::Listener,
		     public IPdraw::IAudioSource::Listener,
		     public IPdraw::IAudioSink::Listener {
	PDRAW_DISABLE_COPY(PdrawBackend)

public:
	class Demuxer : public IPdraw::IDemuxer,
			public ElementWithListener<IPdraw::IDemuxer> {
	public:
		Demuxer(PdrawBackend *backend,
			IPdraw::IDemuxer::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~Demuxer() override;

		int close() override;

		int getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias) override;

		int selectMedia(uint32_t selectedMedias) override;

		uint16_t getSingleStreamLocalStreamPort() override;

		uint16_t getSingleStreamLocalControlPort() override;

		bool isReadyToPlay() override;

		bool isPaused() override;

		int play(float speed = 1.0f) override;

		int pause() override;

		int previousFrame() override;

		int nextFrame() override;

		int seek(int64_t delta, bool exact = false) override;

		int seekForward(uint64_t delta, bool exact = false) override;

		int seekBack(uint64_t delta, bool exact = false) override;

		int seekTo(uint64_t timestamp, bool exact = false) override;

		int getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount) override;

		uint64_t getDuration() override;

		uint64_t getCurrentTime() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class Muxer : public IPdraw::IMuxer,
		      public ElementWithListener<IPdraw::IMuxer> {
	public:
		Muxer(PdrawBackend *backend,
		      [[maybe_unused]] std::string_view url,
		      IPdraw::IMuxer::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~Muxer() override;

		int addMedia(
			unsigned int mediaId,
			const struct pdraw_muxer_media_params *params) override;

		int setThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size) override;

		int setFileMetadata(
			const struct pdraw_muxer_metadata_params *params,
			const uint8_t *data,
			size_t size) override;

		int addChapter(uint64_t timestamp, const char *name) override;

		int getStats(struct pdraw_muxer_stats *stats) override;

		int setDynParams(const struct pdraw_muxer_dyn_params
					 *dyn_params) override;

		int getDynParams(
			struct pdraw_muxer_dyn_params *dyn_params) override;

		int forceSync() override;

		int close() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class VideoRenderer
			: public IPdraw::IVideoRenderer,
			  public ElementWithListener<IPdraw::IVideoRenderer> {
	public:
		/* Called on the rendering thread */
		VideoRenderer(
			PdrawBackend *backend,
			[[maybe_unused]] const struct pdraw_rect *renderPos,
			[[maybe_unused]] const struct
			pdraw_video_renderer_params *params,
			IPdraw::IVideoRenderer::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		/* Called on the rendering thread */
		~VideoRenderer() override;

		/* Called on the rendering thread */
		int resize(const struct pdraw_rect *renderPos) override;

		/* Called on the rendering thread */
		int setMediaId(unsigned int mediaId) override;

		/* Called on the rendering thread */
		unsigned int getMediaId() override;

		/* Called on the rendering thread */
		int setParams(const struct pdraw_video_renderer_params *params)
			override;

		/* Called on the rendering thread */
		int
		getParams(struct pdraw_video_renderer_params *params) override;

		/* Called on the rendering thread */
		int render(struct pdraw_rect *contentPos,
			   const float *viewMat = nullptr,
			   const float *projMat = nullptr) override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class VipcSource : public IPdraw::IVipcSource,
			   public ElementWithListener<IPdraw::IVipcSource> {
	public:
		VipcSource(PdrawBackend *backend,
			   IPdraw::IVipcSource::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~VipcSource() override;

		bool isReadyToPlay() override;

		bool isPaused() override;

		int play() override;

		int pause() override;

		int configure(const struct vdef_dim *resolution,
			      const struct vdef_rectf *crop) override;

		int insertGreyFrame(uint64_t tsUs) override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class CodedVideoSource : public IPdraw::ICodedVideoSource,
				 public ElementWithListener<
					 IPdraw::ICodedVideoSource> {
	public:
		CodedVideoSource(
			PdrawBackend *backend,
			IPdraw::ICodedVideoSource::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~CodedVideoSource() override;

		struct mbuf_coded_video_frame_queue *getQueue() override;

		int flush() override;

		int drain() override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class RawVideoSource
			: public IPdraw::IRawVideoSource,
			  public ElementWithListener<IPdraw::IRawVideoSource> {
	public:
		RawVideoSource(PdrawBackend *backend,
			       IPdraw::IRawVideoSource::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~RawVideoSource() override;

		struct mbuf_raw_video_frame_queue *getQueue() override;

		int flush() override;

		int drain() override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class CodedVideoSink
			: public IPdraw::ICodedVideoSink,
			  public ElementWithListener<IPdraw::ICodedVideoSink> {
	public:
		CodedVideoSink(
			PdrawBackend *backend,
			[[maybe_unused]] unsigned int mediaId,
			[[maybe_unused]] const struct pdraw_video_sink_params
				*params,
			IPdraw::ICodedVideoSink::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~CodedVideoSink() override;

		int setMediaId(unsigned int mediaId) override;

		unsigned int getMediaId() override;

		int resync() override;

		struct mbuf_coded_video_frame_queue *getQueue() override;

		int queueFlushed() override;

		int queueDrained() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class RawVideoSink : public IPdraw::IRawVideoSink,
			     public ElementWithListener<IPdraw::IRawVideoSink> {
	public:
		RawVideoSink(
			PdrawBackend *backend,
			[[maybe_unused]] unsigned int mediaId,
			[[maybe_unused]] const struct pdraw_video_sink_params
				*params,
			IPdraw::IRawVideoSink::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~RawVideoSink() override;

		int setMediaId(unsigned int mediaId) override;

		unsigned int getMediaId() override;

		struct mbuf_raw_video_frame_queue *getQueue() override;

		int queueFlushed() override;

		int queueDrained() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class AlsaSource : public IPdraw::IAlsaSource,
			   public ElementWithListener<IPdraw::IAlsaSource> {
	public:
		AlsaSource(PdrawBackend *backend,
			   IPdraw::IAlsaSource::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~AlsaSource() override;

		bool isReadyToPlay() override;

		bool isPaused() override;

		int play() override;

		int pause() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class AudioSource : public IPdraw::IAudioSource,
			    public ElementWithListener<IPdraw::IAudioSource> {
	public:
		AudioSource(PdrawBackend *backend,
			    IPdraw::IAudioSource::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~AudioSource() override;

		struct mbuf_audio_frame_queue *getQueue() override;

		int flush() override;

		int drain() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class AudioSink : public IPdraw::IAudioSink,
			  public ElementWithListener<IPdraw::IAudioSink> {
	public:
		AudioSink(PdrawBackend *backend,
			  [[maybe_unused]] unsigned int mediaId,
			  IPdraw::IAudioSink::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~AudioSink() override;

		int setMediaId(unsigned int mediaId) override;

		unsigned int getMediaId() override;

		struct mbuf_audio_frame_queue *getQueue() override;

		int queueFlushed() override;

		int queueDrained() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class AudioRenderer
			: public IPdraw::IAudioRenderer,
			  public ElementWithListener<IPdraw::IAudioRenderer> {
	public:
		AudioRenderer(PdrawBackend *backend,
			      IPdraw::IAudioRenderer::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~AudioRenderer() override;

		int setMediaId(unsigned int mediaId) override;

		unsigned int getMediaId() override;

		int setParams(const struct pdraw_audio_renderer_params *params)
			override;

		int
		getParams(struct pdraw_audio_renderer_params *params) override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class VideoEncoder : public IPdraw::IVideoEncoder,
			     public ElementWithListener<IPdraw::IVideoEncoder> {
	public:
		VideoEncoder(PdrawBackend *backend,
			     [[maybe_unused]] unsigned int mediaId,
			     [[maybe_unused]] const struct venc_config *params,
			     IPdraw::IVideoEncoder::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~VideoEncoder() override;

		int configure(const struct venc_dyn_config *config) override;

		int getConfig(struct venc_dyn_config *config) override;

		int requestKeyFrame() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class VideoScaler : public IPdraw::IVideoScaler,
			    public ElementWithListener<IPdraw::IVideoScaler> {
	public:
		VideoScaler(PdrawBackend *backend,
			    [[maybe_unused]] unsigned int mediaId,
			    [[maybe_unused]] const struct vscale_config *params,
			    IPdraw::IVideoScaler::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~VideoScaler() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	class AudioEncoder : public IPdraw::IAudioEncoder,
			     public ElementWithListener<IPdraw::IAudioEncoder> {
	public:
		AudioEncoder(PdrawBackend *backend,
			     [[maybe_unused]] unsigned int mediaId,
			     [[maybe_unused]] const struct aenc_config *params,
			     IPdraw::IAudioEncoder::Listener *listener) :
				ElementWithListener(listener),
				mBackend(backend)
		{
		}

		~AudioEncoder() override;

	private:
		PdrawBackend *mBackend = nullptr;
	};

	explicit PdrawBackend(IPdrawBackend::Listener *listener);

	~PdrawBackend() override;

	int start() override;

	int stop() override;

	struct pomp_loop *getLoop() override;

	int createDemuxer(const std::string &url,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createDemuxer(const std::string &localAddr,
			  uint16_t localStreamPort,
			  uint16_t localControlPort,
			  const std::string &remoteAddr,
			  uint16_t remoteStreamPort,
			  uint16_t remoteControlPort,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createDemuxer(const std::string &url,
			  struct mux_ctx *mux,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createMuxer(const std::string &url,
			const struct pdraw_muxer_params *params,
			IPdraw::IMuxer::Listener *listener,
			IPdraw::IMuxer **retObj) override;

	int createMuxer(const std::string &url,
			struct mux_ctx *mux,
			const std::string &remoteHost,
			const struct pdraw_muxer_params *params,
			IPdraw::IMuxer::Listener *listener,
			IPdraw::IMuxer **retObj) override;

	/* Called on the rendering thread */
	int
	createVideoRenderer(unsigned int mediaId,
			    const struct pdraw_rect *renderPos,
			    const struct pdraw_video_renderer_params *params,
			    IPdraw::IVideoRenderer::Listener *listener,
			    IPdraw::IVideoRenderer **retObj) override;

	int
	createAudioRenderer(unsigned int mediaId,
			    const struct pdraw_audio_renderer_params *params,
			    IPdraw::IAudioRenderer::Listener *listener,
			    IPdraw::IAudioRenderer **retObj) override;

	int createVipcSource(const struct pdraw_vipc_source_params *params,
			     IPdraw::IVipcSource::Listener *listener,
			     IPdraw::IVipcSource **retObj) override;

	int
	createCodedVideoSource(const struct pdraw_video_source_params *params,
			       IPdraw::ICodedVideoSource::Listener *listener,
			       IPdraw::ICodedVideoSource **retObj) override;

	int createRawVideoSource(const struct pdraw_video_source_params *params,
				 IPdraw::IRawVideoSource::Listener *listener,
				 IPdraw::IRawVideoSource **retObj) override;

	int createCodedVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::ICodedVideoSink::Listener *listener,
				 IPdraw::ICodedVideoSink **retObj) override;

	int createRawVideoSink(unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::IRawVideoSink::Listener *listener,
			       IPdraw::IRawVideoSink **retObj) override;

	int createAlsaSource(const struct pdraw_alsa_source_params *params,
			     IPdraw::IAlsaSource::Listener *listener,
			     IPdraw::IAlsaSource **retObj) override;

	int createAudioSource(const struct pdraw_audio_source_params *params,
			      IPdraw::IAudioSource::Listener *listener,
			      IPdraw::IAudioSource **retObj) override;

	int createAudioSink(unsigned int mediaId,
			    IPdraw::IAudioSink::Listener *listener,
			    IPdraw::IAudioSink **retObj) override;

	int createVideoEncoder(unsigned int mediaId,
			       const struct venc_config *params,
			       IPdraw::IVideoEncoder::Listener *listener,
			       IPdraw::IVideoEncoder **retObj) override;

	int createVideoScaler(unsigned int mediaId,
			      const struct vscale_config *params,
			      IPdraw::IVideoScaler::Listener *listener,
			      IPdraw::IVideoScaler **retObj) override;

	int createAudioEncoder(unsigned int mediaId,
			       const struct aenc_config *params,
			       IPdraw::IAudioEncoder::Listener *listener,
			       IPdraw::IAudioEncoder **retObj) override;

	void getFriendlyNameSetting(std::string *friendlyName) override;

	void setFriendlyNameSetting(const std::string &friendlyName) override;

	void getSerialNumberSetting(std::string *serialNumber) override;

	void setSerialNumberSetting(const std::string &serialNumber) override;

	void getSoftwareVersionSetting(std::string *softwareVersion) override;

	void
	setSoftwareVersionSetting(const std::string &softwareVersion) override;

	int dumpPipeline(const std::string &fileName) override;

private:
	template <typename T>
	static void deleteElement(T *self, PdrawBackend *backend);

	template <typename T>
	static bool isElementReadyToPlay(T *self, PdrawBackend *backend);

	template <typename T>
	static int playElement(T *self, PdrawBackend *backend);

	template <typename T>
	static bool isElementPaused(T *self, PdrawBackend *backend);

	template <typename T>
	static int pauseElement(T *self, PdrawBackend *backend);

	template <typename T>
	static int closeElement(T *self, PdrawBackend *backend);

	template <typename T>
	static int flushElement(T *self, PdrawBackend *backend);

	template <typename T>
	static int elementQueueFlushed(T *self, PdrawBackend *backend);

	template <typename T>
	static int drainElement(T *self, PdrawBackend *backend);

	template <typename T>
	static int elementQueueDrained(T *self, PdrawBackend *backend);

	template <typename T>
	static int setElementSessionMetadata(T *self,
					     PdrawBackend *backend,
					     const struct vmeta_session *meta);

	template <typename T>
	static int getElementSessionMetadata(T *self,
					     PdrawBackend *backend,
					     struct vmeta_session *meta);

	template <typename T>
	static unsigned int getElementMediaId(T *self, PdrawBackend *backend);

	void stopResponse(IPdraw *pdraw, int status) override;

	void onMediaAdded(IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override;

	void onMediaRemoved(IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override;

	void onSocketCreated(IPdraw *pdraw, int fd) override;

	void demuxerOpenResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status) override;

	void demuxerCloseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status) override;

	void onDemuxerUnrecoverableError(IPdraw *pdraw,
					 IPdraw::IDemuxer *demuxer) override;

	int demuxerSelectMedia(IPdraw *pdraw,
			       IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override;

	void demuxerReadyToPlay(IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer,
				bool ready) override;

	void onDemuxerEndOfRange(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override;

	void demuxerPlayResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override;

	void demuxerPauseResponse(IPdraw *pdraw,
				  IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp) override;

	void demuxerSeekResponse(IPdraw *pdraw,
				 IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override;

	void onMuxerConnectionStateChanged(
		IPdraw *pdraw,
		IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state connectionState,
		enum pdraw_muxer_disconnection_reason disconnectionReason)
		override;

	void onMuxerMediaReady(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath,
			       const struct iovec *iov,
			       int iovcnt) override;

	void onMuxerMediaSaved(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath) override;

	void onMuxerUnrecoverableError(IPdraw *pdraw,
				       IPdraw::IMuxer *muxer,
				       int status) override;

	void muxerCloseResponse(IPdraw *pdraw,
				IPdraw::IMuxer *muxer,
				int status) override;

	void
	onVideoRendererMediaAdded(IPdraw *pdraw,
				  IPdraw::IVideoRenderer *renderer,
				  const struct pdraw_media_info *info) override;

	void onVideoRendererMediaRemoved(IPdraw *pdraw,
					 IPdraw::IVideoRenderer *renderer,
					 const struct pdraw_media_info *info,
					 bool restart) override;

	void onVideoRenderReady(IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer) override;

	int loadVideoTexture(IPdraw *pdraw,
			     IPdraw::IVideoRenderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_media_info *mediaInfo,
			     struct mbuf_raw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen) override;

	int renderVideoOverlay(
		IPdraw *pdraw,
		IPdraw::IVideoRenderer *renderer,
		const struct pdraw_rect *renderPos,
		const struct pdraw_rect *contentPos,
		const float *viewMat,
		const float *projMat,
		const struct pdraw_media_info *mediaInfo,
		struct vmeta_frame *frameMeta,
		const struct pdraw_video_frame_extra *frameExtra) override;

	void
	onAudioRendererMediaAdded(IPdraw *pdraw,
				  IPdraw::IAudioRenderer *renderer,
				  const struct pdraw_media_info *info) override;

	void onAudioRendererMediaRemoved(
		IPdraw *pdraw,
		IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override;

	void vipcSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		bool ready,
		enum pdraw_vipc_source_eos_reason eosReason) override;

	void vipcSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IVipcSource *source) override;

	void vipcSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IVipcSource *source) override;

	bool vipcSourceFramerateChanged(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		const struct vdef_frac *prevFramerate,
		const struct vdef_frac *newFramerate) override;

	void vipcSourceConfigured(IPdraw *pdraw,
				  IPdraw::IVipcSource *source,
				  int status,
				  const struct vdef_format_info *info,
				  const struct vdef_rectf *crop) override;

	void vipcSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IVipcSource *source,
				  struct mbuf_raw_video_frame *frame) override;

	bool vipcSourceEndOfStream(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		enum pdraw_vipc_source_eos_reason eosReason) override;

	void onCodedVideoSinkMediaAdded(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSink *sink,
		const struct pdraw_media_info *info) override;

	void onCodedVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
					  const struct pdraw_media_info *info,
					  bool restart) override;

	void
	onCodedVideoSourceFlushed(IPdraw *pdraw,
				  IPdraw::ICodedVideoSource *source) override;

	void
	onCodedVideoSourceDrained(IPdraw *pdraw,
				  IPdraw::ICodedVideoSource *source) override;

	void onRawVideoSourceFlushed(IPdraw *pdraw,
				     IPdraw::IRawVideoSource *source) override;

	void onRawVideoSourceDrained(IPdraw *pdraw,
				     IPdraw::IRawVideoSource *source) override;

	void onCodedVideoSinkFlush(IPdraw *pdraw,
				   IPdraw::ICodedVideoSink *sink) override;

	void onCodedVideoSinkDrain(IPdraw *pdraw,
				   IPdraw::ICodedVideoSink *sink) override;

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::ICodedVideoSink *sink,
		const struct vmeta_session *meta) override;

	void
	onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink,
				 const struct pdraw_media_info *info) override;

	void onRawVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IRawVideoSink *sink,
					const struct pdraw_media_info *info,
					bool restart) override;

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::IRawVideoSink *sink,
		const struct vmeta_session *meta) override;

	void onRawVideoSinkFlush(IPdraw *pdraw,
				 IPdraw::IRawVideoSink *sink) override;

	void onRawVideoSinkDrain(IPdraw *pdraw,
				 IPdraw::IRawVideoSink *sink) override;

	void alsaSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override;

	void alsaSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IAlsaSource *source) override;

	void alsaSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IAlsaSource *source) override;

	void alsaSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override;

	void onAudioSourceFlushed(IPdraw *pdraw,
				  IPdraw::IAudioSource *source) override;

	void onAudioSourceDrained(IPdraw *pdraw,
				  IPdraw::IAudioSource *source) override;

	void
	onAudioSinkMediaAdded(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink,
			      const struct pdraw_media_info *info) override;

	void onAudioSinkMediaRemoved(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioSink *sink,
				     const struct pdraw_media_info *info,
				     bool restart) override;

	void onAudioSinkFlush(IPdraw *pdraw, IPdraw::IAudioSink *sink) override;

	void onAudioSinkDrain(IPdraw *pdraw, IPdraw::IAudioSink *sink) override;

	void
	videoEncoderFrameOutput(IPdraw *pdraw,
				IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) override;

	void videoEncoderFramePreRelease(
		IPdraw *pdraw,
		IPdraw::IVideoEncoder *encoder,
		struct mbuf_coded_video_frame *frame) override;

	void
	videoScalerFrameOutput(IPdraw *pdraw,
			       IPdraw::IVideoScaler *scaler,
			       struct mbuf_raw_video_frame *frame) override;

	void audioEncoderFrameOutput(IPdraw *pdraw,
				     IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame) override;

	void
	audioEncoderFramePreRelease(IPdraw *pdraw,
				    IPdraw::IAudioEncoder *encoder,
				    struct mbuf_audio_frame *frame) override;

	void checkLoopThread(const char *func) const;

	static void loopThread(PdrawBackend *self);

	int doCreateDemuxer(const std::string &url,
			    const struct pdraw_demuxer_params *params,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateDemuxer(const std::string &localAddr,
			    uint16_t localStreamPort,
			    uint16_t localControlPort,
			    const std::string &remoteAddr,
			    uint16_t remoteStreamPort,
			    uint16_t remoteControlPort,
			    const struct pdraw_demuxer_params *params,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateDemuxer(const std::string &url,
			    struct mux_ctx *mux,
			    const struct pdraw_demuxer_params *params,
			    IPdraw::IDemuxer::Listener *listener,
			    IPdraw::IDemuxer **retObj);

	int doCreateMuxer(const std::string &url,
			  const struct pdraw_muxer_params *params,
			  IPdraw::IMuxer::Listener *listener,
			  IPdraw::IMuxer **retObj);

	int doCreateMuxer(const std::string &url,
			  struct mux_ctx *mux,
			  const std::string &remoteHost,
			  const struct pdraw_muxer_params *params,
			  IPdraw::IMuxer::Listener *listener,
			  IPdraw::IMuxer **retObj);

	int doCreateVipcSource(const struct pdraw_vipc_source_params *params,
			       IPdraw::IVipcSource::Listener *listener,
			       IPdraw::IVipcSource **retObj);

	int
	doCreateCodedVideoSource(const struct pdraw_video_source_params *params,
				 IPdraw::ICodedVideoSource::Listener *listener,
				 IPdraw::ICodedVideoSource **retObj);

	int
	doCreateRawVideoSource(const struct pdraw_video_source_params *params,
			       IPdraw::IRawVideoSource::Listener *listener,
			       IPdraw::IRawVideoSource **retObj);

	int doCreateCodedVideoSink(unsigned int mediaId,
				   const struct pdraw_video_sink_params *params,
				   IPdraw::ICodedVideoSink::Listener *listener,
				   IPdraw::ICodedVideoSink **retObj);

	int doCreateRawVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::IRawVideoSink::Listener *listener,
				 IPdraw::IRawVideoSink **retObj);

	int doCreateAlsaSource(const struct pdraw_alsa_source_params *params,
			       IPdraw::IAlsaSource::Listener *listener,
			       IPdraw::IAlsaSource **retObj);

	int doCreateAudioSource(const struct pdraw_audio_source_params *params,
				IPdraw::IAudioSource::Listener *listener,
				IPdraw::IAudioSource **retObj);

	int doCreateAudioSink(unsigned int mediaId,
			      IPdraw::IAudioSink::Listener *listener,
			      IPdraw::IAudioSink **retObj);

	int
	doCreateAudioRenderer(unsigned int mediaId,
			      const struct pdraw_audio_renderer_params *params,
			      IPdraw::IAudioRenderer::Listener *listener,
			      IPdraw::IAudioRenderer **retObj);

	int doCreateVideoEncoder(unsigned int mediaId,
				 const struct venc_config *params,
				 IPdraw::IVideoEncoder::Listener *listener,
				 IPdraw::IVideoEncoder **retObj);

	int doCreateVideoScaler(unsigned int mediaId,
				const struct vscale_config *params,
				IPdraw::IVideoScaler::Listener *listener,
				IPdraw::IVideoScaler **retObj);

	int doCreateAudioEncoder(unsigned int mediaId,
				 const struct aenc_config *params,
				 IPdraw::IAudioEncoder::Listener *listener,
				 IPdraw::IAudioEncoder **retObj);

	template <typename Func>
	inline auto runOnLoop(Func &&func) ->
		typename std::invoke_result_t<Func>;

	inline void setRetValue();

	inline void setRetValue(int value);

	inline void setRetValue(unsigned int value);

	inline void setRetValue(bool value);

	inline void setRetValue(std::string &&value);

	template <typename T>
	inline void setRetValue(int res, const T &value, T &field);

	template <typename T> inline void setRetValue(const T &value, T &field);

	template <typename WrapperT,
		  typename ListenerT,
		  typename InterfaceT,
		  typename PendingStructT,
		  typename MapT,
		  typename CreateFunc>
	int internalElementCreate(std::unique_ptr<WrapperT> wrapper,
				  ListenerT *listener,
				  InterfaceT **retObj,
				  PendingStructT &pending,
				  MapT &map,
				  CreateFunc createFunc,
				  const char *elementName);

	template <typename KeyT, typename StructT>
	bool tryResolveElementUserData(void *&elementUserData,
				       std::map<KeyT, StructT> &map) const;

	template <typename KeyT, typename StructT>
	bool findElementAndListener(KeyT key,
				    StructT &out,
				    const char *caller,
				    std::map<KeyT, StructT> &map,
				    StructT &pending,
				    std::mutex &mutex,
				    const char *elementName) const;

	std::recursive_mutex mApiMutex{};
	std::mutex mMutex{};
	std::condition_variable mCond;
	std::thread mLoopThread;
	bool mLoopThreadLaunched = false;
	std::atomic_bool mThreadShouldStop{false};
	std::unique_ptr<pomp::Loop> mLoop;
	bool mStarted = false;
	bool mRetValReady = false;
	int mRetStatus = 0;
	std::unique_ptr<IPdraw> mPdraw;
	IPdraw::Listener *mListener = nullptr;
	std::mutex mMapsMutex{};
	bool mMapsMutexCreated = false;
	std::map<IPdraw::IDemuxer *, demuxerAndListener> mDemuxerListenersMap{};
	demuxerAndListener mPendingDemuxerAndListener{};
	std::map<IPdraw::IMuxer *, muxerAndListener> mMuxerListenersMap{};
	muxerAndListener mPendingMuxerAndListener{};
	std::map<IPdraw::IVideoRenderer *, videoRendererAndListener>
		mVideoRendererListenersMap{};
	videoRendererAndListener mPendingVideoRendererAndListener{};
	std::map<IPdraw::IAudioRenderer *, audioRendererAndListener>
		mAudioRendererListenersMap{};
	audioRendererAndListener mPendingAudioRendererAndListener{};
	std::map<IPdraw::IVipcSource *, vipcSourceAndListener>
		mVipcSourceListenersMap{};
	vipcSourceAndListener mPendingVipcSourceAndListener{};
	std::map<IPdraw::ICodedVideoSource *, codedVideoSourceAndListener>
		mCodedVideoSourceListenersMap{};
	codedVideoSourceAndListener mPendingCodedVideoSourceAndListener{};
	std::map<IPdraw::IRawVideoSource *, rawVideoSourceAndListener>
		mRawVideoSourceListenersMap{};
	rawVideoSourceAndListener mPendingRawVideoSourceAndListener{};
	std::map<IPdraw::ICodedVideoSink *, codedVideoSinkAndListener>
		mCodedVideoSinkListenersMap{};
	codedVideoSinkAndListener mPendingCodedVideoSinkAndListener{};
	std::map<IPdraw::IRawVideoSink *, rawVideoSinkAndListener>
		mRawVideoSinkListenersMap{};
	rawVideoSinkAndListener mPendingRawVideoSinkAndListener{};
	std::map<IPdraw::IAlsaSource *, alsaSourceAndListener>
		mAlsaSourceListenersMap{};
	alsaSourceAndListener mPendingAlsaSourceAndListener{};
	std::map<IPdraw::IAudioSource *, audioSourceAndListener>
		mAudioSourceListenersMap{};
	audioSourceAndListener mPendingAudioSourceAndListener{};
	std::map<IPdraw::IAudioSink *, audioSinkAndListener>
		mAudioSinkListenersMap{};
	audioSinkAndListener mPendingAudioSinkAndListener{};
	std::map<IPdraw::IVideoEncoder *, videoEncoderAndListener>
		mVideoEncoderListenersMap{};
	videoEncoderAndListener mPendingVideoEncoderAndListener{};
	std::map<IPdraw::IVideoScaler *, videoScalerAndListener>
		mVideoScalerListenersMap{};
	videoScalerAndListener mPendingVideoScalerAndListener{};
	std::map<IPdraw::IAudioEncoder *, audioEncoderAndListener>
		mAudioEncoderListenersMap{};
	audioEncoderAndListener mPendingAudioEncoderAndListener{};
	struct {
		void *internal;
		void *external;
	} mPendingRemovedElementUserdata{};
};


template <typename WrapperT,
	  typename ListenerT,
	  typename InterfaceT,
	  typename PendingStructT,
	  typename MapT,
	  typename CreateFunc>
int PdrawBackend::internalElementCreate(std::unique_ptr<WrapperT> wrapper,
					ListenerT *listener,
					InterfaceT **retObj,
					PendingStructT &pending,
					MapT &map,
					CreateFunc createFunc,
					const char *elementName)
{
	int res;
	InterfaceT *internal = nullptr;
	std::pair<typename MapT::iterator, bool> inserted;

	/* Fill pending */
	pending = {
		.e = wrapper.get(),
		.l = listener,
	};

	/* Call user-provided create function (lambda) */
	res = createFunc(&internal);
	if (res < 0) {
		ULOG_ERRNO("failed to create %s", -res, elementName);
		/* wrapper destroyed automatically */
		return res;
	}

	/* Bind internal to wrapper */
	wrapper->setElement(internal);

	/* Insert into listeners map */
	{
		std::scoped_lock lock(mMapsMutex);
		inserted = map.insert(
			std::make_pair(wrapper->getElement(), pending));
		if (!inserted.second)
			ULOGW("failed to insert %s listener in the map",
			      elementName);
	}

	/* Ownership transferred to caller via public API raw pointer */
	pending = {};
	WrapperT *rawWrapper = wrapper.release();
	*retObj = rawWrapper;
	if (auto *pc = dynamic_cast<IPostCreatable<InterfaceT> *>(listener))
		pc->postCreate(rawWrapper);
	return 0;
}


template <typename KeyT, typename StructT>
bool PdrawBackend::tryResolveElementUserData(void *&elementUserData,
					     std::map<KeyT, StructT> &map) const
{
	static_assert(std::is_pointer_v<KeyT>, "KeyT must be a pointer type");

	if (auto it = map.find(static_cast<KeyT>(elementUserData));
	    it != map.end()) {
		elementUserData = it->second.e;
		return true;
	}
	return false;
}


template <typename T> T dispatchReturnOnError(int err)
{
	if constexpr (std::is_void_v<T>) {
		return;
	} else if constexpr (std::is_pointer_v<T>) {
		return nullptr;
	} else if constexpr (std::is_signed_v<T>) {
		return static_cast<T>(err);
	} else {
		return static_cast<T>(0);
	}
}


/**
 * Execute a function on the event loop thread (mLoop) and return its result
 * synchronously.
 * Supported return types:
 *  - void
 *  - pointer types
 *  - integral types
 */
template <typename Func>
inline auto PdrawBackend::runOnLoop(Func &&func) ->
	typename std::invoke_result_t<Func>
{
	std::scoped_lock apiLock(mApiMutex);

	/* Raw return type of the provided callable */
	using RawReturnType = typename std::invoke_result_t<Func>;
	/* Decayed return type (remove references and cv-qualifiers) */
	using ReturnType = typename std::decay_t<RawReturnType>;

	/* Enforce that only supported return types are allowed */
	static_assert(std::is_void_v<ReturnType> ||
			      std::is_pointer_v<ReturnType> ||
			      std::is_integral_v<ReturnType>,
		      "runOnLoop: unsupported return type");

	/* If we are already on the loop thread, execute directly */
	if (std::this_thread::get_id() == mLoopThread.get_id())
		return func();

	std::unique_lock lock(mMutex);

	struct SharedState {
		/* Function and return value */
		std::packaged_task<RawReturnType()> task;
		/* Indicates when execution has finished */
		std::atomic<bool> done{false};
		explicit SharedState(Func &&f) : task(std::move<Func>(f)) {}
	};

	std::shared_ptr<SharedState> state;
	std::unique_ptr<pomp::Loop::IdleHandlerFunc> handler;

	try {
		state = std::make_shared<SharedState>(std::forward<Func>(func));
		handler = std::make_unique<pomp::Loop::IdleHandlerFunc>();
	} catch (const std::bad_alloc &) {
		int err = -ENOMEM;
		ULOG_ERRNO("failed to allocate memory", -err);
		return dispatchReturnOnError<ReturnType>(err);
	}

	std::future<RawReturnType> res = state->task.get_future();

	/* Wrapper executed on the loop thread.
	 * Calls the callback and wakes up waiting threads. */
	auto wrapper = [this, state]() {
		state->task();
		state->done = true;
		this->mCond.notify_all();
	};

	handler->set([wrapper]() { wrapper(); });

	int err = mLoop->idleAdd(handler.get());
	if (err < 0) {
		ULOG_ERRNO("pomp::Loop::idleAdd", -err);
		return dispatchReturnOnError<ReturnType>(err);
	}

	/* Wait until the loop thread has executed the function */
	mCond.wait(lock, [state]() { return state->done.load(); });

	/* Retrieve and return the result */
	try {
		return res.get();
	} catch (const std::future_error &e) {
		ULOGE("future_error in runOnLoop: %s (code: %d)",
		      e.what(),
		      e.code().value());
		return dispatchReturnOnError<ReturnType>(-EPROTO);
	}
}


template <typename KeyT, typename StructT>
bool PdrawBackend::findElementAndListener(KeyT key,
					  StructT &out,
					  const char *caller,
					  std::map<KeyT, StructT> &map,
					  StructT &pending,
					  std::mutex &mutex,
					  const char *elementName) const
{
	{
		std::scoped_lock lock(mutex);
		auto it = map.find(key);
		out = (it == map.end()) ? pending : it->second;
	}

	if (out.l == nullptr) {
		ULOGE("%s: failed to find the %s listener in the map",
		      caller,
		      elementName);
		return false;
	}
	if (out.e == nullptr) {
		ULOGE("%s: failed to find the %s in the map",
		      caller,
		      elementName);
		return false;
	}
	return true;
}


} /* namespace PdrawBackend */
