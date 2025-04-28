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

#ifndef _PDRAW_BACKEND_IMPL_HPP_
#define _PDRAW_BACKEND_IMPL_HPP_

#include <pthread.h>

#include <atomic>
#include <map>
#include <vector>

#include <futils/futils.h>
#include <libpomp.h>
#include <pdraw/pdraw_backend.hpp>

namespace PdrawBackend {


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

		int close(void) override;

		int getMediaList(struct pdraw_demuxer_media **mediaList,
				 size_t *mediaCount,
				 uint32_t *selectedMedias) override;

		int selectMedia(uint32_t selectedMedias) override;

		uint16_t getSingleStreamLocalStreamPort(void) override;

		uint16_t getSingleStreamLocalControlPort(void) override;

		bool isReadyToPlay(void) override;

		bool isPaused(void) override;

		int play(float speed = 1.0f) override;

		int pause(void) override;

		int previousFrame(void) override;

		int nextFrame(void) override;

		int seek(int64_t delta, bool exact = false) override;

		int seekForward(uint64_t delta, bool exact = false) override;

		int seekBack(uint64_t delta, bool exact = false) override;

		int seekTo(uint64_t timestamp, bool exact = false) override;

		int getChapterList(struct pdraw_chapter **chapterList,
				   size_t *chapterCount) override;

		uint64_t getDuration(void) override;

		uint64_t getCurrentTime(void) override;

		IPdraw::IDemuxer *getDemuxer() const
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
		Muxer(PdrawBackend *backend,
		      const std::string &url,
		      IPdraw::IMuxer::Listener *listener);

		~Muxer(void);

		int addMedia(
			unsigned int mediaId,
			const struct pdraw_muxer_media_params *params) override;

		int setThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size) override;

		int addChapter(uint64_t timestamp, const char *name) override;

		int getStats(struct pdraw_muxer_stats *stats) override;

		int setDynParams(const struct pdraw_muxer_dyn_params
					 *dyn_params) override;

		int getDynParams(
			struct pdraw_muxer_dyn_params *dyn_params) override;

		int forceSync(void) override;

		int close(void) override;

		IPdraw::IMuxer *getMuxer() const
		{
			return mMuxer;
		}

		void setMuxer(IPdraw::IMuxer *muxer)
		{
			mMuxer = muxer;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IMuxer::Listener *mListener;
		IPdraw::IMuxer *mMuxer;
	};

	class VideoRenderer : public IPdraw::IVideoRenderer {
	public:
		/* Called on the rendering thread */
		VideoRenderer(PdrawBackend *backend,
			      const struct pdraw_rect *renderPos,
			      const struct pdraw_video_renderer_params *params,
			      IPdraw::IVideoRenderer::Listener *listener);

		/* Called on the rendering thread */
		~VideoRenderer(void);

		/* Called on the rendering thread */
		int resize(const struct pdraw_rect *renderPos) override;

		/* Called on the rendering thread */
		int setMediaId(unsigned int mediaId) override;

		/* Called on the rendering thread */
		unsigned int getMediaId(void) override;

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

		IPdraw::IVideoRenderer *getRenderer() const
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

	class VipcSource : public IPdraw::IVipcSource {
	public:
		VipcSource(PdrawBackend *backend,
			   IPdraw::IVipcSource::Listener *listener);

		~VipcSource(void);

		bool isReadyToPlay(void) override;

		bool isPaused(void) override;

		int play(void) override;

		int pause(void) override;

		int configure(const struct vdef_dim *resolution,
			      const struct vdef_rectf *crop) override;

		int insertGreyFrame(uint64_t tsUs) override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

		IPdraw::IVipcSource *getVipcSource() const
		{
			return mSource;
		}

		void setVipcSource(IPdraw::IVipcSource *source)
		{
			mSource = source;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IVipcSource::Listener *mListener;
		IPdraw::IVipcSource *mSource;
	};

	class CodedVideoSource : public IPdraw::ICodedVideoSource {
	public:
		CodedVideoSource(PdrawBackend *backend,
				 IPdraw::ICodedVideoSource::Listener *listener);

		~CodedVideoSource(void);

		struct mbuf_coded_video_frame_queue *getQueue(void) override;

		int flush(void) override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

		IPdraw::ICodedVideoSource *getCodedVideoSource() const
		{
			return mSource;
		}

		void setCodedVideoSource(IPdraw::ICodedVideoSource *source)
		{
			mSource = source;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::ICodedVideoSource::Listener *mListener;
		IPdraw::ICodedVideoSource *mSource;
	};

	class RawVideoSource : public IPdraw::IRawVideoSource {
	public:
		RawVideoSource(PdrawBackend *backend,
			       IPdraw::IRawVideoSource::Listener *listener);

		~RawVideoSource(void);

		struct mbuf_raw_video_frame_queue *getQueue(void) override;

		int flush(void) override;

		int
		setSessionMetadata(const struct vmeta_session *meta) override;

		int getSessionMetadata(struct vmeta_session *meta) override;

		IPdraw::IRawVideoSource *getRawVideoSource() const
		{
			return mSource;
		}

		void setRawVideoSource(IPdraw::IRawVideoSource *source)
		{
			mSource = source;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IRawVideoSource::Listener *mListener;
		IPdraw::IRawVideoSource *mSource;
	};

	class CodedVideoSink : public IPdraw::ICodedVideoSink {
	public:
		CodedVideoSink(PdrawBackend *backend,
			       unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::ICodedVideoSink::Listener *listener);

		~CodedVideoSink(void);

		int resync(void) override;

		struct mbuf_coded_video_frame_queue *getQueue(void) override;

		int queueFlushed(void) override;

		IPdraw::ICodedVideoSink *getCodedVideoSink() const
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

		struct mbuf_raw_video_frame_queue *getQueue(void) override;

		int queueFlushed(void) override;

		IPdraw::IRawVideoSink *getRawVideoSink() const
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

	class AlsaSource : public IPdraw::IAlsaSource {
	public:
		AlsaSource(PdrawBackend *backend,
			   IPdraw::IAlsaSource::Listener *listener);

		~AlsaSource(void);

		bool isReadyToPlay(void) override;

		bool isPaused(void) override;

		int play(void) override;

		int pause(void) override;

		IPdraw::IAlsaSource *getAlsaSource() const
		{
			return mSource;
		}

		void setAlsaSource(IPdraw::IAlsaSource *source)
		{
			mSource = source;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IAlsaSource::Listener *mListener;
		IPdraw::IAlsaSource *mSource;
	};

	class AudioSource : public IPdraw::IAudioSource {
	public:
		AudioSource(PdrawBackend *backend,
			    IPdraw::IAudioSource::Listener *listener);

		~AudioSource(void);

		struct mbuf_audio_frame_queue *getQueue(void) override;

		int flush(void) override;

		IPdraw::IAudioSource *getAudioSource() const
		{
			return mSource;
		}

		void setAudioSource(IPdraw::IAudioSource *source)
		{
			mSource = source;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IAudioSource::Listener *mListener;
		IPdraw::IAudioSource *mSource;
	};

	class AudioSink : public IPdraw::IAudioSink {
	public:
		AudioSink(PdrawBackend *backend,
			  unsigned int mediaId,
			  IPdraw::IAudioSink::Listener *listener);

		~AudioSink(void);

		struct mbuf_audio_frame_queue *getQueue(void) override;

		int queueFlushed(void) override;

		IPdraw::IAudioSink *getAudioSink() const
		{
			return mSink;
		}

		void setAudioSink(IPdraw::IAudioSink *sink)
		{
			mSink = sink;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IAudioSink::Listener *mListener;
		IPdraw::IAudioSink *mSink;
	};

	class AudioRenderer : public IPdraw::IAudioRenderer {
	public:
		AudioRenderer(PdrawBackend *backend,
			      IPdraw::IAudioRenderer::Listener *listener);

		~AudioRenderer(void);

		int setMediaId(unsigned int mediaId) override;

		unsigned int getMediaId(void) override;

		int setParams(const struct pdraw_audio_renderer_params *params)
			override;

		int
		getParams(struct pdraw_audio_renderer_params *params) override;

		IPdraw::IAudioRenderer *getAudioRenderer()
		{
			return mRenderer;
		}

		void setAudioRenderer(IPdraw::IAudioRenderer *renderer)
		{
			mRenderer = renderer;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IAudioRenderer::Listener *mListener;
		IPdraw::IAudioRenderer *mRenderer;
	};

	class VideoEncoder : public IPdraw::IVideoEncoder {
	public:
		VideoEncoder(PdrawBackend *backend,
			     unsigned int mediaId,
			     const struct venc_config *params,
			     IPdraw::IVideoEncoder::Listener *listener);

		~VideoEncoder(void);

		int configure(const struct venc_dyn_config *config) override;

		int getConfig(struct venc_dyn_config *config) override;

		IPdraw::IVideoEncoder *getVideoEncoder() const
		{
			return mEncoder;
		}

		void setVideoEncoder(IPdraw::IVideoEncoder *encoder)
		{
			mEncoder = encoder;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IVideoEncoder::Listener *mListener;
		IPdraw::IVideoEncoder *mEncoder;
	};

	class VideoScaler : public IPdraw::IVideoScaler {
	public:
		VideoScaler(PdrawBackend *backend,
			    unsigned int mediaId,
			    const struct vscale_config *params,
			    IPdraw::IVideoScaler::Listener *listener);

		~VideoScaler(void);

		IPdraw::IVideoScaler *getVideoScaler() const
		{
			return mScaler;
		}

		void setVideoScaler(IPdraw::IVideoScaler *scaler)
		{
			mScaler = scaler;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IVideoScaler::Listener *mListener;
		IPdraw::IVideoScaler *mScaler;
	};

	class AudioEncoder : public IPdraw::IAudioEncoder {
	public:
		AudioEncoder(PdrawBackend *backend,
			     unsigned int mediaId,
			     const struct aenc_config *params,
			     IPdraw::IAudioEncoder::Listener *listener);

		~AudioEncoder(void);

		IPdraw::IAudioEncoder *getAudioEncoder() const
		{
			return mEncoder;
		}

		void setAudioEncoder(IPdraw::IAudioEncoder *encoder)
		{
			mEncoder = encoder;
		}

	private:
		PdrawBackend *mBackend;
		IPdraw::IAudioEncoder::Listener *mListener;
		IPdraw::IAudioEncoder *mEncoder;
	};

	PdrawBackend(IPdrawBackend::Listener *listener);

	~PdrawBackend(void);

	int start(void) override;

	int stop(void) override;

	struct pomp_loop *getLoop(void) override;

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
	int pushCmdAndWait(const struct cmd_msg *cmd);

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

	void
	onCodedVideoSourceFlushed(IPdraw *pdraw,
				  IPdraw::ICodedVideoSource *source) override;

	void onRawVideoSourceFlushed(IPdraw *pdraw,
				     IPdraw::IRawVideoSource *source) override;

	void onCodedVideoSinkFlush(IPdraw *pdraw,
				   IPdraw::ICodedVideoSink *sink) override;

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::ICodedVideoSink *sink,
		const struct vmeta_session *meta) override;

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::IRawVideoSink *sink,
		const struct vmeta_session *meta) override;

	void onRawVideoSinkFlush(IPdraw *pdraw,
				 IPdraw::IRawVideoSink *sink) override;

	void alsaSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override;

	void alsaSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override;

	void onAudioSourceFlushed(IPdraw *pdraw,
				  IPdraw::IAudioSource *source) override;

	void onAudioSinkFlush(IPdraw *pdraw, IPdraw::IAudioSink *sink) override;

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

	static void *loopThread(void *ptr);

	static void mboxCb(int fd, uint32_t revents, void *userdata);

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

	void internalStop(void);

	void internalDemuxerCreate(const std::string &url,
				   const struct pdraw_demuxer_params *params,
				   IPdraw::IDemuxer::Listener *listener);

	void internalDemuxerCreate(const std::string &localAddr,
				   uint16_t localStreamPort,
				   uint16_t localControlPort,
				   const std::string &remoteAddr,
				   uint16_t remoteStreamPort,
				   uint16_t remoteControlPort,
				   const struct pdraw_demuxer_params *params,
				   IPdraw::IDemuxer::Listener *listener);

	void internalDemuxerCreate(const std::string &url,
				   struct mux_ctx *mux,
				   const struct pdraw_demuxer_params *params,
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

	void internalDemuxerGetChapterList(PdrawBackend::Demuxer *demuxer,
					   struct pdraw_chapter **chapterList,
					   size_t *chapterCount);

	void internalDemuxerGetDuration(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerGetCurrentTime(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerGetMediaList(PdrawBackend::Demuxer *demuxer);

	void internalDemuxerSelectMedia(PdrawBackend::Demuxer *demuxer,
					uint32_t selectedMedias);

	void internalMuxerCreate(const std::string &url,
				 const struct pdraw_muxer_params *params,
				 IPdraw::IMuxer::Listener *listener);

	void internalMuxerDestroy(PdrawBackend::Muxer *muxer);

	void internalMuxerClose(PdrawBackend::Muxer *muxer);

	void
	internalMuxerAddMedia(PdrawBackend::Muxer *muxer,
			      unsigned int mediaId,
			      const struct pdraw_muxer_media_params *params);

	void internalMuxerSetThumbnail(PdrawBackend::Muxer *muxer,
				       enum pdraw_muxer_thumbnail_type type,
				       const uint8_t *data,
				       size_t size);

	void internalMuxerAddChapter(PdrawBackend::Muxer *muxer,
				     uint64_t timestamp,
				     const char *name);

	void internalMuxerGetStats(PdrawBackend::Muxer *muxer);

	void
	internalMuxerSetDynParams(PdrawBackend::Muxer *muxer,
				  struct pdraw_muxer_dyn_params dyn_params);

	void internalMuxerGetDynParams(PdrawBackend::Muxer *muxer);

	void internalMuxerForceSync(PdrawBackend::Muxer *muxer);

	void internalVipcSourceCreate(IPdraw::IVipcSource::Listener *listener);

	void internalVipcSourceDestroy(PdrawBackend::VipcSource *source);

	void internalVipcSourceIsReadyToPlay(PdrawBackend::VipcSource *source);

	void internalVipcSourceIsPaused(PdrawBackend::VipcSource *source);

	void internalVipcSourcePlay(PdrawBackend::VipcSource *source);

	void internalVipcSourcePause(PdrawBackend::VipcSource *source);

	void internalVipcSourceConfigure(PdrawBackend::VipcSource *source,
					 const struct vdef_dim *resolution,
					 const struct vdef_rectf *crop);

	void internalVipcSourceInsertGreyFrame(PdrawBackend::VipcSource *source,
					       uint64_t tsUs);

	void
	internalVipcSourceSetSessionMetadata(PdrawBackend::VipcSource *source);

	void
	internalVipcSourceGetSessionMetadata(PdrawBackend::VipcSource *source);

	void internalCodedVideoSourceCreate(
		IPdraw::ICodedVideoSource::Listener *listener);

	void
	internalCodedVideoSourceDestroy(PdrawBackend::CodedVideoSource *source);

	void internalCodedVideoSourceGetQueue(
		PdrawBackend::CodedVideoSource *source);

	void
	internalCodedVideoSourceFlush(PdrawBackend::CodedVideoSource *source);

	void internalCodedVideoSourceSetSessionMetadata(
		PdrawBackend::CodedVideoSource *source);

	void internalCodedVideoSourceGetSessionMetadata(
		PdrawBackend::CodedVideoSource *source);

	void internalRawVideoSourceCreate(
		IPdraw::IRawVideoSource::Listener *listener);

	void
	internalRawVideoSourceDestroy(PdrawBackend::RawVideoSource *source);

	void
	internalRawVideoSourceGetQueue(PdrawBackend::RawVideoSource *source);

	void internalRawVideoSourceFlush(PdrawBackend::RawVideoSource *source);

	void internalRawVideoSourceSetSessionMetadata(
		PdrawBackend::RawVideoSource *source);

	void internalRawVideoSourceGetSessionMetadata(
		PdrawBackend::RawVideoSource *source);

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

	void
	internalAlsaSourceCreate(const struct pdraw_alsa_source_params *params,
				 IPdraw::IAlsaSource::Listener *listener);

	void internalAlsaSourceDestroy(PdrawBackend::AlsaSource *source);

	void internalAlsaSourceIsReadyToPlay(PdrawBackend::AlsaSource *source);

	void internalAlsaSourceIsPaused(PdrawBackend::AlsaSource *source);

	void internalAlsaSourcePlay(PdrawBackend::AlsaSource *source);

	void internalAlsaSourcePause(PdrawBackend::AlsaSource *source);

	void internalAudioSourceCreate(
		const struct pdraw_audio_source_params *params,
		IPdraw::IAudioSource::Listener *listener);

	void internalAudioSourceDestroy(PdrawBackend::AudioSource *source);

	void internalAudioSourceGetQueue(PdrawBackend::AudioSource *source);

	void internalAudioSourceFlush(PdrawBackend::AudioSource *source);

	void internalAudioSinkCreate(unsigned int mediaId,
				     IPdraw::IAudioSink::Listener *listener);

	void internalAudioSinkDestroy(PdrawBackend::AudioSink *sink);

	void internalAudioSinkGetQueue(PdrawBackend::AudioSink *sink);

	void internalAudioSinkQueueFlushed(PdrawBackend::AudioSink *sink);

	void internalAudioRendererCreate(
		unsigned int mediaId,
		const struct pdraw_audio_renderer_params *params,
		IPdraw::IAudioRenderer::Listener *listener);

	void
	internalAudioRendererDestroy(PdrawBackend::AudioRenderer *renderer);

	void
	internalAudioRendererSetMediaId(PdrawBackend::AudioRenderer *renderer,
					unsigned int mediaId);

	void
	internalAudioRendererGetMediaId(PdrawBackend::AudioRenderer *renderer);

	void
	internalAudioRendererSetParams(PdrawBackend::AudioRenderer *renderer);

	void
	internalAudioRendererGetParams(PdrawBackend::AudioRenderer *renderer);

	void
	internalVideoEncoderCreate(unsigned int mediaId,
				   const struct venc_config *params,
				   IPdraw::IVideoEncoder::Listener *listener);

	void internalVideoEncoderDestroy(PdrawBackend::VideoEncoder *encoder);

	void
	internalVideoEncoderConfigure(PdrawBackend::VideoEncoder *encoder,
				      const struct venc_dyn_config *config);

	void internalVideoEncoderGetConfig(PdrawBackend::VideoEncoder *encoder,
					   struct venc_dyn_config *config);

	void
	internalVideoScalerCreate(unsigned int mediaId,
				  const struct vscale_config *params,
				  IPdraw::IVideoScaler::Listener *listener);

	void internalVideoScalerDestroy(PdrawBackend::VideoScaler *encoder);

	void
	internalAudioEncoderCreate(unsigned int mediaId,
				   const struct aenc_config *params,
				   IPdraw::IAudioEncoder::Listener *listener);

	void internalAudioEncoderDestroy(PdrawBackend::AudioEncoder *encoder);

	void internalGetFriendlyNameSetting(void);

	void internalSetFriendlyNameSetting(const std::string &friendlyName);

	void internalGetSerialNumberSetting(void);

	void internalSetSerialNumberSetting(const std::string &serialNumber);

	void internalGetSoftwareVersionSetting(void);

	void
	internalSetSoftwareVersionSetting(const std::string &softwareVersion);

	void internalDumpPipeline(const std::string &fileName);

	struct demuxerAndListener {
		IPdraw::IDemuxer *d;
		IPdraw::IDemuxer::Listener *l;
	};

	struct muxerAndListener {
		IPdraw::IMuxer *m;
		IPdraw::IMuxer::Listener *l;
	};

	struct videoRendererAndListener {
		IPdraw::IVideoRenderer *r;
		IPdraw::IVideoRenderer::Listener *l;
	};

	struct audioRendererAndListener {
		IPdraw::IAudioRenderer *r;
		IPdraw::IAudioRenderer::Listener *l;
	};

	struct vipcSourceAndListener {
		IPdraw::IVipcSource *s;
		IPdraw::IVipcSource::Listener *l;
	};

	struct codedVideoSourceAndListener {
		IPdraw::ICodedVideoSource *s;
		IPdraw::ICodedVideoSource::Listener *l;
	};

	struct rawVideoSourceAndListener {
		IPdraw::IRawVideoSource *s;
		IPdraw::IRawVideoSource::Listener *l;
	};

	struct codedVideoSinkAndListener {
		IPdraw::ICodedVideoSink *s;
		IPdraw::ICodedVideoSink::Listener *l;
	};

	struct rawVideoSinkAndListener {
		IPdraw::IRawVideoSink *s;
		IPdraw::IRawVideoSink::Listener *l;
	};

	struct alsaSourceAndListener {
		IPdraw::IAlsaSource *s;
		IPdraw::IAlsaSource::Listener *l;
	};

	struct audioSourceAndListener {
		IPdraw::IAudioSource *s;
		IPdraw::IAudioSource::Listener *l;
	};

	struct audioSinkAndListener {
		IPdraw::IAudioSink *s;
		IPdraw::IAudioSink::Listener *l;
	};

	struct videoEncoderAndListener {
		IPdraw::IVideoEncoder *e;
		IPdraw::IVideoEncoder::Listener *l;
	};

	struct videoScalerAndListener {
		IPdraw::IVideoScaler *s;
		IPdraw::IVideoScaler::Listener *l;
	};

	struct audioEncoderAndListener {
		IPdraw::IAudioEncoder *e;
		IPdraw::IAudioEncoder::Listener *l;
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
	std::atomic_bool mThreadShouldStop;
	struct pomp_loop *mLoop;
	struct mbox *mMbox;
	bool mStarted;
	struct pdraw_video_source_params mParamVideoSource;
	struct pdraw_vipc_source_params mParamVipcSource;
	struct pdraw_audio_renderer_params mParamAudioRenderer;
	struct vmeta_session mParamVmetaSession;
	bool mRetValReady;
	int mRetStatus;
	bool mRetBool;
	uint16_t mRetUint16;
	uint32_t mRetUint32;
	uint64_t mRetUint64;
	size_t mRetSizeT;
	unsigned int mRetUint;
	std::string mRetString;
	struct venc_dyn_config mRetConfig;
	struct pdraw_muxer_stats mRetMuxerStats;
	struct pdraw_muxer_dyn_params mRetMuxerDynParams;
	struct mbuf_coded_video_frame_queue *mRetCodedQueue;
	struct mbuf_raw_video_frame_queue *mRetRawQueue;
	struct mbuf_audio_frame_queue *mRetAudioQueue;
	struct pdraw_demuxer_media *mRetMediaList;
	IPdraw::IVipcSource *mRetVipcSource;
	IPdraw::ICodedVideoSource *mRetCodedVideoSource;
	IPdraw::IRawVideoSource *mRetRawVideoSource;
	IPdraw::ICodedVideoSink *mRetCodedVideoSink;
	IPdraw::IRawVideoSink *mRetRawVideoSink;
	IPdraw::IAlsaSource *mRetAlsaSource;
	IPdraw::IAudioSource *mRetAudioSource;
	IPdraw::IAudioSink *mRetAudioSink;
	IPdraw::IAudioRenderer *mRetAudioRenderer;
	IPdraw::IVideoEncoder *mRetVideoEncoder;
	IPdraw::IVideoScaler *mRetVideoScaler;
	IPdraw::IAudioEncoder *mRetAudioEncoder;
	IPdraw::IDemuxer *mRetDemuxer;
	IPdraw::IMuxer *mRetMuxer;
	IPdraw *mPdraw;
	IPdraw::Listener *mListener;
	pthread_mutex_t mMapsMutex;
	bool mMapsMutexCreated;
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>
		mDemuxerListenersMap;
	struct demuxerAndListener mPendingDemuxerAndListener;
	std::map<IPdraw::IMuxer *, struct muxerAndListener> mMuxerListenersMap;
	struct muxerAndListener mPendingMuxerAndListener;
	std::map<IPdraw::IVideoRenderer *, struct videoRendererAndListener>
		mVideoRendererListenersMap;
	struct videoRendererAndListener mPendingVideoRendererAndListener;
	std::map<IPdraw::IAudioRenderer *, struct audioRendererAndListener>
		mAudioRendererListenersMap;
	struct audioRendererAndListener mPendingAudioRendererAndListener;
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>
		mVipcSourceListenersMap;
	struct vipcSourceAndListener mPendingVipcSourceAndListener;
	std::map<IPdraw::ICodedVideoSource *,
		 struct codedVideoSourceAndListener>
		mCodedVideoSourceListenersMap;
	struct codedVideoSourceAndListener mPendingCodedVideoSourceAndListener;
	std::map<IPdraw::IRawVideoSource *, struct rawVideoSourceAndListener>
		mRawVideoSourceListenersMap;
	struct rawVideoSourceAndListener mPendingRawVideoSourceAndListener;
	std::map<IPdraw::ICodedVideoSink *, struct codedVideoSinkAndListener>
		mCodedVideoSinkListenersMap;
	struct codedVideoSinkAndListener mPendingCodedVideoSinkAndListener;
	std::map<IPdraw::IRawVideoSink *, struct rawVideoSinkAndListener>
		mRawVideoSinkListenersMap;
	struct rawVideoSinkAndListener mPendingRawVideoSinkAndListener;
	std::map<IPdraw::IAlsaSource *, struct alsaSourceAndListener>
		mAlsaSourceListenersMap;
	struct alsaSourceAndListener mPendingAlsaSourceAndListener;
	std::map<IPdraw::IAudioSource *, struct audioSourceAndListener>
		mAudioSourceListenersMap;
	struct audioSourceAndListener mPendingAudioSourceAndListener;
	std::map<IPdraw::IAudioSink *, struct audioSinkAndListener>
		mAudioSinkListenersMap;
	struct audioSinkAndListener mPendingAudioSinkAndListener;
	std::map<IPdraw::IVideoEncoder *, struct videoEncoderAndListener>
		mVideoEncoderListenersMap;
	struct videoEncoderAndListener mPendingVideoEncoderAndListener;
	std::map<IPdraw::IVideoScaler *, struct videoScalerAndListener>
		mVideoScalerListenersMap;
	struct videoScalerAndListener mPendingVideoScalerAndListener;
	std::map<IPdraw::IAudioEncoder *, struct audioEncoderAndListener>
		mAudioEncoderListenersMap;
	struct audioEncoderAndListener mPendingAudioEncoderAndListener;
	struct {
		void *internal;
		void *external;
	} mPendingRemovedElementUserdata;
};

} /* namespace PdrawBackend */

#endif /* !_PDRAW_BACKEND_IMPL_HPP_ */
