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

#include <errno.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#define ULOG_TAG pdraw_backend
#include <ulog.h>

#include <pdraw/pdraw_backend.h>

#include "pdraw_backend_impl.hpp"


struct pdraw_demuxer {
	Pdraw::IPdraw::IDemuxer *impl;
};

struct pdraw_muxer {
	Pdraw::IPdraw::IMuxer *impl;
};

struct pdraw_video_renderer {
	Pdraw::IPdraw::IVideoRenderer *impl;
};

struct pdraw_audio_renderer {
	Pdraw::IPdraw::IAudioRenderer *impl;
};

struct pdraw_vipc_source {
	Pdraw::IPdraw::IVipcSource *impl;
};

struct pdraw_coded_video_source {
	Pdraw::IPdraw::ICodedVideoSource *impl;
};

struct pdraw_raw_video_source {
	Pdraw::IPdraw::IRawVideoSource *impl;
};

struct pdraw_coded_video_sink {
	Pdraw::IPdraw::ICodedVideoSink *impl;
};

struct pdraw_raw_video_sink {
	Pdraw::IPdraw::IRawVideoSink *impl;
};

struct pdraw_alsa_source {
	Pdraw::IPdraw::IAlsaSource *impl;
};

struct pdraw_audio_source {
	Pdraw::IPdraw::IAudioSource *impl;
};

struct pdraw_audio_sink {
	Pdraw::IPdraw::IAudioSink *impl;
};

struct pdraw_video_encoder {
	Pdraw::IPdraw::IVideoEncoder *impl;
};

struct pdraw_video_scaler {
	Pdraw::IPdraw::IVideoScaler *impl;
};

struct pdraw_audio_encoder {
	Pdraw::IPdraw::IAudioEncoder *impl;
};


class ListenerBase {
public:
	virtual void *getImplVoid() const = 0;
	virtual void *getWrapperVoid() = 0;

protected:
	~ListenerBase() = default;
};

struct pdraw_backend;
static void registerListener(struct pdraw_backend *p, ListenerBase *l);
static void unregisterListener(struct pdraw_backend *p, ListenerBase *l);
static void *findWrapperForImpl(struct pdraw_backend *p, void *impl);


template <typename HostT, typename CbsT, typename ImplT, typename WrapperT>
class ElementHolder : public ListenerBase,
		      public PdrawBackend::IPostCreatable<ImplT> {
public:
	WrapperT *getWrapper()
	{
		return &mWrapper;
	}
	ImplT *getImpl() const
	{
		return mWrapper.impl;
	}
	void setImpl(ImplT *impl)
	{
		mWrapper.impl = impl;
	}
	void postCreate(ImplT *impl) noexcept override
	{
		setImpl(impl);
	}
	void ensureImpl(ImplT *impl)
	{
		if (mWrapper.impl == nullptr)
			mWrapper.impl = impl;
	}
	void *getImplVoid() const override
	{
		return static_cast<void *>(mWrapper.impl);
	}
	void *getWrapperVoid() override
	{
		return static_cast<void *>(&mWrapper);
	}
	ElementHolder(HostT *pdraw, const CbsT *cbs, void *userdata) :
			mPdraw(pdraw), mCbs(*cbs), mUserdata(userdata)
	{
		registerListener(mPdraw, this);
	}

protected:
	~ElementHolder()
	{
		unregisterListener(mPdraw, this);
	}

	HostT *mPdraw = nullptr;
	CbsT mCbs;
	void *mUserdata = nullptr;
	WrapperT mWrapper{};
};


class PdrawBackendListener : public Pdraw::IPdraw::Listener {
public:
	PdrawBackendListener(struct pdraw_backend *pdraw,
			     const struct pdraw_backend_cbs *cbs,
			     void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendListener() override = default;

	void stopResponse(Pdraw::IPdraw *pdraw, int status) override
	{
		if (mCbs.stop_resp) {
			(*mCbs.stop_resp)(mPdraw, status, mUserdata);
		}
	}

	void onMediaAdded(Pdraw::IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override
	{
		if (mCbs.media_added) {
			(*mCbs.media_added)(
				mPdraw,
				info,
				findWrapperForImpl(mPdraw, elementUserData),
				mUserdata);
		}
	}

	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override;

	void onSocketCreated(Pdraw::IPdraw *pdraw, int fd) override
	{
		if (mCbs.socket_created)
			(*mCbs.socket_created)(mPdraw, fd, mUserdata);
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_cbs mCbs;
	void *mUserdata = nullptr;
};


class PdrawBackendDemuxerListener
		: public Pdraw::IPdraw::IDemuxer::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_demuxer_cbs,
				       Pdraw::IPdraw::IDemuxer,
				       struct pdraw_demuxer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendDemuxerListener() override = default;

	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status) override
	{
		ensureImpl(demuxer);
		if (mCbs.open_resp)
			(*mCbs.open_resp)(mPdraw, &mWrapper, status, mUserdata);
	}

	void demuxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status) override
	{
		ensureImpl(demuxer);
		if (mCbs.close_resp)
			(*mCbs.close_resp)(
				mPdraw, &mWrapper, status, mUserdata);
	}

	void
	onDemuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IDemuxer *demuxer) override
	{
		ensureImpl(demuxer);
		if (mCbs.unrecoverable_error)
			(*mCbs.unrecoverable_error)(
				mPdraw, &mWrapper, mUserdata);
	}

	int demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override
	{
		ensureImpl(demuxer);
		if (mCbs.select_media) {
			return (*mCbs.select_media)(mPdraw,
						    &mWrapper,
						    medias,
						    count,
						    selectedMedias,
						    mUserdata);
		}
		return -ENOSYS;
	}

	void demuxerReadyToPlay(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IDemuxer *demuxer,
				bool ready) override
	{
		ensureImpl(demuxer);
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(
				mPdraw, &mWrapper, ready ? 1 : 0, mUserdata);
	}

	void onDemuxerEndOfRange(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override
	{
		ensureImpl(demuxer);
		if (mCbs.end_of_range)
			(*mCbs.end_of_range)(
				mPdraw, &mWrapper, timestamp, mUserdata);
	}

	void demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		ensureImpl(demuxer);
		if (mCbs.play_resp)
			(*mCbs.play_resp)(mPdraw,
					  &mWrapper,
					  status,
					  timestamp,
					  speed,
					  mUserdata);
	}

	void demuxerPauseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp) override
	{
		ensureImpl(demuxer);
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(mPdraw,
					   &mWrapper,
					   status,
					   timestamp,
					   mUserdata);
	}

	void demuxerSeekResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		ensureImpl(demuxer);
		if (mCbs.seek_resp)
			(*mCbs.seek_resp)(mPdraw,
					  &mWrapper,
					  status,
					  timestamp,
					  speed,
					  mUserdata);
	}
};


class PdrawBackendMuxerListener
		: public Pdraw::IPdraw::IMuxer::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_muxer_cbs,
				       Pdraw::IPdraw::IMuxer,
				       struct pdraw_muxer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendMuxerListener() override = default;

	void onMuxerConnectionStateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state connectionState,
		enum pdraw_muxer_disconnection_reason disconnectionReason)
		override
	{
		ensureImpl(muxer);
		if (mCbs.connection_state_changed)
			(*mCbs.connection_state_changed)(mPdraw,
							 &mWrapper,
							 connectionState,
							 disconnectionReason,
							 mUserdata);
	}

	void onMuxerMediaReady(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath,
			       const struct iovec *iov,
			       int iovcnt) override
	{
		ensureImpl(muxer);
		if (mCbs.media_ready)
			(*mCbs.media_ready)(mPdraw,
					    &mWrapper,
					    mediaPath,
					    iov,
					    iovcnt,
					    mUserdata);
	}

	void onMuxerMediaSaved(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath) override
	{
		ensureImpl(muxer);
		if (mCbs.media_saved)
			(*mCbs.media_saved)(
				mPdraw, &mWrapper, mediaPath, mUserdata);
	}

	void onMuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IMuxer *muxer,
				       int status) override
	{
		ensureImpl(muxer);
		if (mCbs.unrecoverable_error)
			(*mCbs.unrecoverable_error)(
				mPdraw, &mWrapper, status, mUserdata);
	}

	void muxerCloseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IMuxer *muxer,
				int status) override
	{
		ensureImpl(muxer);
		if (mCbs.close_resp)
			(*mCbs.close_resp)(
				mPdraw, &mWrapper, status, mUserdata);
	}
};


class PdrawBackendVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_video_renderer_cbs,
				       Pdraw::IPdraw::IVideoRenderer,
				       struct pdraw_video_renderer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendVideoRendererListener() override = default;

	void
	onVideoRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVideoRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		ensureImpl(renderer);
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void
	onVideoRendererMediaRemoved(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoRenderer *renderer,
				    const struct pdraw_media_info *info,
				    bool restart) override
	{
		ensureImpl(renderer);
		if (mCbs.media_removed)
			(*mCbs.media_removed)(mPdraw,
					      &mWrapper,
					      info,
					      restart ? 1 : 0,
					      mUserdata);
	}

	void
	onVideoRenderReady(Pdraw::IPdraw *pdraw,
			   Pdraw::IPdraw::IVideoRenderer *renderer) override
	{
		ensureImpl(renderer);
		if (mCbs.render_ready)
			(*mCbs.render_ready)(mPdraw, &mWrapper, mUserdata);
	}

	int loadVideoTexture(Pdraw::IPdraw *pdraw,
			     Pdraw::IPdraw::IVideoRenderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_media_info *mediaInfo,
			     struct mbuf_raw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen) override
	{
		ensureImpl(renderer);
		if (mCbs.load_texture == nullptr)
			return -ENOSYS;
		return (*mCbs.load_texture)(mPdraw,
					    &mWrapper,
					    textureWidth,
					    textureHeight,
					    mediaInfo,
					    frame,
					    frameUserdata,
					    frameUserdataLen,
					    mUserdata);
	}

	int renderVideoOverlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVideoRenderer *renderer,
		const struct pdraw_rect *renderPos,
		const struct pdraw_rect *contentPos,
		const float *viewMat,
		const float *projMat,
		const struct pdraw_media_info *mediaInfo,
		struct vmeta_frame *frameMeta,
		const struct pdraw_video_frame_extra *frameExtra) override
	{
		ensureImpl(renderer);
		if (mCbs.render_overlay == nullptr)
			return -ENOSYS;
		if ((renderer == nullptr) || (renderPos == nullptr) ||
		    (contentPos == nullptr) || (viewMat == nullptr) ||
		    (projMat == nullptr))
			return -EINVAL;
		(*mCbs.render_overlay)(mPdraw,
				       &mWrapper,
				       renderPos,
				       contentPos,
				       viewMat,
				       projMat,
				       mediaInfo,
				       frameMeta,
				       frameExtra,
				       mUserdata);
		return 0;
	}
};


class PdrawBackendAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_audio_renderer_cbs,
				       Pdraw::IPdraw::IAudioRenderer,
				       struct pdraw_audio_renderer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendAudioRendererListener() override = default;

	void
	onAudioRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		ensureImpl(renderer);
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onAudioRendererMediaRemoved(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override
	{
		ensureImpl(renderer);
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, mUserdata);
	}
};


class PdrawBackendVipcSourceListener
		: public Pdraw::IPdraw::IVipcSource::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_vipc_source_cbs,
				       Pdraw::IPdraw::IVipcSource,
				       struct pdraw_vipc_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendVipcSourceListener() override = default;

	void vipcSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		bool ready,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		ensureImpl(source);
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(mPdraw,
					      &mWrapper,
					      (int)ready,
					      eosReason,
					      mUserdata);
	}

	void vipcSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IVipcSource *source) override
	{
		ensureImpl(source);
		if (mCbs.play_resp)
			(*mCbs.play_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void vipcSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IVipcSource *source) override
	{
		ensureImpl(source);
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(mPdraw, &mWrapper, mUserdata);
	}

	bool vipcSourceFramerateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
		const struct vdef_frac *prevFramerate,
		const struct vdef_frac *newFramerate) override
	{
		ensureImpl(source);
		if (mCbs.framerate_changed)
			return (*mCbs.framerate_changed)(mPdraw,
							 &mWrapper,
							 prevFramerate,
							 newFramerate,
							 mUserdata);
		return false;
	}

	void vipcSourceConfigured(IPdraw *pdraw,
				  IPdraw::IVipcSource *source,
				  int status,
				  const struct vdef_format_info *info,
				  const struct vdef_rectf *crop) override
	{
		ensureImpl(source);
		if (mCbs.configured)
			(*mCbs.configured)(mPdraw,
					   &mWrapper,
					   status,
					   info,
					   crop,
					   mUserdata);
	}

	void vipcSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IVipcSource *source,
				  struct mbuf_raw_video_frame *frame) override
	{
		ensureImpl(source);
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	bool vipcSourceEndOfStream(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		ensureImpl(source);
		if (mCbs.end_of_stream)
			return (*mCbs.end_of_stream)(
				mPdraw, &mWrapper, eosReason, mUserdata);
		return false;
	}
};


class PdrawBackendCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener,
		  public ElementHolder<
			  struct pdraw_backend,
			  struct pdraw_backend_coded_video_source_cbs,
			  Pdraw::IPdraw::ICodedVideoSource,
			  struct pdraw_coded_video_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendCodedVideoSourceListener() override = default;

	void onCodedVideoSourceFlushed(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		ensureImpl(source);
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void onCodedVideoSourceDrained(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		ensureImpl(source);
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawBackendRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener,
		  public ElementHolder<
			  struct pdraw_backend,
			  struct pdraw_backend_raw_video_source_cbs,
			  Pdraw::IPdraw::IRawVideoSource,
			  struct pdraw_raw_video_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendRawVideoSourceListener() override = default;

	void
	onRawVideoSourceFlushed(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		ensureImpl(source);
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void
	onRawVideoSourceDrained(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		ensureImpl(source);
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawBackendCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener,
		  public ElementHolder<
			  struct pdraw_backend,
			  struct pdraw_backend_coded_video_sink_cbs,
			  Pdraw::IPdraw::ICodedVideoSink,
			  struct pdraw_coded_video_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendCodedVideoSinkListener() override = default;

	void
	onCodedVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::ICodedVideoSink *sink,
				   const struct pdraw_media_info *info) override
	{
		ensureImpl(sink);
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onCodedVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
					  const struct pdraw_media_info *info,
					  bool restart) override
	{
		ensureImpl(sink);
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void
	onCodedVideoSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void
	onCodedVideoSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::ICodedVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		ensureImpl(sink);
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw, &mWrapper, meta, mUserdata);
	}
};


class PdrawBackendRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_raw_video_sink_cbs,
				       Pdraw::IPdraw::IRawVideoSink,
				       struct pdraw_raw_video_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendRawVideoSinkListener() override = default;

	void
	onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink,
				 const struct pdraw_media_info *info) override
	{
		ensureImpl(sink);
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onRawVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IRawVideoSink *sink,
					const struct pdraw_media_info *info,
					bool restart) override
	{
		ensureImpl(sink);
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void onRawVideoSinkFlush(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void onRawVideoSinkDrain(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::IRawVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		ensureImpl(sink);
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw, &mWrapper, meta, mUserdata);
	}
};


class PdrawBackendAlsaSourceListener
		: public Pdraw::IPdraw::IAlsaSource::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_alsa_source_cbs,
				       Pdraw::IPdraw::IAlsaSource,
				       struct pdraw_alsa_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendAlsaSourceListener() override = default;

	void alsaSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override
	{
		ensureImpl(source);
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(mPdraw,
					      &mWrapper,
					      (int)ready,
					      eosReason,
					      mUserdata);
	}

	void alsaSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IAlsaSource *source) override
	{
		ensureImpl(source);
		if (mCbs.play_resp)
			(*mCbs.play_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void alsaSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IAlsaSource *source) override
	{
		ensureImpl(source);
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void alsaSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override
	{
		ensureImpl(source);
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawBackendAudioSourceListener
		: public Pdraw::IPdraw::IAudioSource::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_audio_source_cbs,
				       Pdraw::IPdraw::IAudioSource,
				       struct pdraw_audio_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendAudioSourceListener() override = default;

	void onAudioSourceFlushed(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		ensureImpl(source);
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void onAudioSourceDrained(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		ensureImpl(source);
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawBackendAudioSinkListener
		: public Pdraw::IPdraw::IAudioSink::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_audio_sink_cbs,
				       Pdraw::IPdraw::IAudioSink,
				       struct pdraw_audio_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendAudioSinkListener() override = default;

	void onAudioSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IAudioSink *sink,
				   const struct pdraw_media_info *info) override
	{
		ensureImpl(sink);
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onAudioSinkMediaRemoved(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioSink *sink,
				     const struct pdraw_media_info *info,
				     bool restart) override
	{
		ensureImpl(sink);
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void onAudioSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void onAudioSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		ensureImpl(sink);
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawBackendVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_video_encoder_cbs,
				       Pdraw::IPdraw::IVideoEncoder,
				       struct pdraw_video_encoder> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendVideoEncoderListener() override = default;

	void
	videoEncoderFrameOutput(IPdraw *pdraw,
				IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) override
	{
		ensureImpl(encoder);
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	void videoEncoderFramePreRelease(
		IPdraw *pdraw,
		IPdraw::IVideoEncoder *encoder,
		struct mbuf_coded_video_frame *frame) override
	{
		ensureImpl(encoder);
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawBackendVideoScalerListener
		: public Pdraw::IPdraw::IVideoScaler::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_video_scaler_cbs,
				       Pdraw::IPdraw::IVideoScaler,
				       struct pdraw_video_scaler> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendVideoScalerListener() override = default;

	void videoScalerFrameOutput(IPdraw *pdraw,
				    IPdraw::IVideoScaler *scaler,
				    struct mbuf_raw_video_frame *frame) override
	{
		ensureImpl(scaler);
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawBackendAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener,
		  public ElementHolder<struct pdraw_backend,
				       struct pdraw_backend_audio_encoder_cbs,
				       Pdraw::IPdraw::IAudioEncoder,
				       struct pdraw_audio_encoder> {
public:
	using ElementHolder::ElementHolder;

	~PdrawBackendAudioEncoderListener() override = default;

	void audioEncoderFrameOutput(IPdraw *pdraw,
				     IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame) override
	{
		ensureImpl(encoder);
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	void
	audioEncoderFramePreRelease(IPdraw *pdraw,
				    IPdraw::IAudioEncoder *encoder,
				    struct mbuf_audio_frame *frame) override
	{
		ensureImpl(encoder);
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


struct pdraw_backend {
	std::unique_ptr<PdrawBackend::IPdrawBackend> pdraw{};
	std::unique_ptr<PdrawBackendListener> listener{};
	/* Declared first so it outlives all listener vectors below and remains
	 * valid during their destruction (listeners call unregisterListener in
	 * ~ElementHolder). */
	std::vector<ListenerBase *> allListeners{};
	std::mutex allListenersMutex{};
	std::vector<std::unique_ptr<PdrawBackendDemuxerListener>>
		demuxerListeners{};
	std::vector<std::unique_ptr<PdrawBackendMuxerListener>>
		muxerListeners{};
	std::vector<std::unique_ptr<PdrawBackendVideoRendererListener>>
		videoRendererListeners{};
	std::vector<std::unique_ptr<PdrawBackendAudioRendererListener>>
		audioRendererListeners{};
	std::vector<std::unique_ptr<PdrawBackendVipcSourceListener>>
		vipcSourceListeners{};
	std::vector<std::unique_ptr<PdrawBackendCodedVideoSourceListener>>
		codedVideoSourceListeners{};
	std::vector<std::unique_ptr<PdrawBackendRawVideoSourceListener>>
		rawVideoSourceListeners{};
	std::vector<std::unique_ptr<PdrawBackendCodedVideoSinkListener>>
		codedVideoSinkListeners{};
	std::vector<std::unique_ptr<PdrawBackendRawVideoSinkListener>>
		rawVideoSinkListeners{};
	std::vector<std::unique_ptr<PdrawBackendAlsaSourceListener>>
		alsaSourceListeners{};
	std::vector<std::unique_ptr<PdrawBackendAudioSourceListener>>
		audioSourceListeners{};
	std::vector<std::unique_ptr<PdrawBackendAudioSinkListener>>
		audioSinkListeners{};
	std::vector<std::unique_ptr<PdrawBackendVideoEncoderListener>>
		videoEncoderListeners{};
	std::vector<std::unique_ptr<PdrawBackendVideoScalerListener>>
		videoScalerListeners{};
	std::vector<std::unique_ptr<PdrawBackendAudioEncoderListener>>
		audioEncoderListeners{};
	/* Maps impl pointer -> C wrapper, populated before
	 * deleteImplAndEraseListener so that onMediaRemoved can resolve the
	 * wrapper even after the listener has been removed from allListeners.
	 * Entries are never erased individually (a multi-track element fires
	 * onMediaRemoved once per track with the same impl); they are
	 * overwritten if an impl address is reused. */
	std::unordered_map<void *, void *> pendingRemovedUserdataMap{};
	std::mutex pendingRemovedMutex{};
};


static void registerListener(struct pdraw_backend *p, ListenerBase *l)
{
	if (p == nullptr || l == nullptr)
		return;
	std::scoped_lock lock(p->allListenersMutex);
	p->allListeners.push_back(l);
}


static void unregisterListener(struct pdraw_backend *p, ListenerBase *l)
{
	if (p == nullptr || l == nullptr)
		return;
	std::scoped_lock lock(p->allListenersMutex);
	auto it = std::find(p->allListeners.begin(), p->allListeners.end(), l);
	if (it != p->allListeners.end())
		p->allListeners.erase(it);
}


static void *findWrapperForImpl(struct pdraw_backend *p, void *impl)
{
	if (p == nullptr || impl == nullptr)
		return nullptr;
	std::scoped_lock lock(p->allListenersMutex);
	for (auto *l : p->allListeners) {
		if (l->getImplVoid() == impl)
			return l->getWrapperVoid();
	}
	return nullptr;
}


void PdrawBackendListener::onMediaRemoved(Pdraw::IPdraw *pdraw,
					  const struct pdraw_media_info *info,
					  void *elementUserData)
{
	if (mCbs.media_removed) {
		void *wrapper = findWrapperForImpl(mPdraw, elementUserData);
		if (wrapper == nullptr && elementUserData != nullptr) {
			std::scoped_lock lock(mPdraw->pendingRemovedMutex);
			auto it = mPdraw->pendingRemovedUserdataMap.find(
				elementUserData);
			if (it != mPdraw->pendingRemovedUserdataMap.end())
				wrapper = it->second;
		}
		(*mCbs.media_removed)(mPdraw, info, wrapper, mUserdata);
	}
}


int pdraw_be_new(const struct pdraw_backend_cbs *cbs,
		 void *userdata,
		 struct pdraw_backend **ret_obj)
{
	int res = 0;
	PdrawBackend::IPdrawBackend *pdraw = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	std::unique_ptr<struct pdraw_backend> self;

	try {
		self = std::make_unique<struct pdraw_backend>();
		self->listener = std::make_unique<PdrawBackendListener>(
			self.get(), cbs, userdata);
	} catch (const std::bad_alloc &) {
		res = -ENOMEM;
		goto error;
	}

	res = createPdrawBackend(self->listener.get(), &pdraw);
	if (res < 0)
		goto error;

	self->pdraw.reset(pdraw);

	res = self->pdraw->start();
	if (res < 0)
		goto error;

	/* Ownership transferred to caller via public C API raw pointer */
	*ret_obj = self.release();
	return 0;

error:
	(void)pdraw_be_destroy(self.release());
	*ret_obj = nullptr;
	return res;
}


int pdraw_be_destroy(struct pdraw_backend *self)
{
	if (self == nullptr)
		return 0;

	auto owner = std::unique_ptr<struct pdraw_backend>(self);

	if (owner->pdraw != nullptr) {
		owner->pdraw->stop();
		owner->pdraw.reset();
	}

	owner->listener.reset();
	return 0;
}


int pdraw_be_stop(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	return self->pdraw->stop();
}


struct pomp_loop *pdraw_be_get_loop(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);

	return self->pdraw->getLoop();
}


int pdraw_be_demuxer_new_from_url(struct pdraw_backend *self,
				  const char *url,
				  const struct pdraw_demuxer_params *params,
				  const struct pdraw_backend_demuxer_cbs *cbs,
				  void *userdata,
				  struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	std::unique_ptr<PdrawBackendDemuxerListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	*ret_obj = l->getWrapper();
	res = self->pdraw->createDemuxer(u, params, l.get(), &demuxer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_demuxer_new_single_stream(
	struct pdraw_backend *self,
	const char *local_addr,
	uint16_t local_stream_port,
	uint16_t local_control_port,
	const char *remote_addr,
	uint16_t remote_stream_port,
	uint16_t remote_control_port,
	const struct pdraw_demuxer_params *params,
	const struct pdraw_backend_demuxer_cbs *cbs,
	void *userdata,
	struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	std::unique_ptr<PdrawBackendDemuxerListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string local(local_addr ? local_addr : "");
	std::string remote(remote_addr ? remote_addr : "");
	*ret_obj = l->getWrapper();
	res = self->pdraw->createDemuxer(local,
					 local_stream_port,
					 local_control_port,
					 remote,
					 remote_stream_port,
					 remote_control_port,
					 params,
					 l.get(),
					 &demuxer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_demuxer_new_from_url_on_mux(
	struct pdraw_backend *self,
	const char *url,
	struct mux_ctx *mux,
	const struct pdraw_demuxer_params *params,
	const struct pdraw_backend_demuxer_cbs *cbs,
	void *userdata,
	struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	std::unique_ptr<PdrawBackendDemuxerListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	*ret_obj = l->getWrapper();
	res = self->pdraw->createDemuxer(u, mux, params, l.get(), &demuxer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	return 0;
}


template <typename Container, typename Impl>
static void deleteImplAndEraseListener(struct pdraw_backend *self,
				       Container &container,
				       const Impl *impl)
{
	using ValueType = typename Container::value_type;

	auto it = std::find_if(container.begin(),
			       container.end(),
			       [impl](const ValueType &listener) {
				       return listener->getImpl() == impl;
			       });

	if (it != container.end()) {
		std::scoped_lock lock(self->pendingRemovedMutex);
		self->pendingRemovedUserdataMap[(void *)impl] =
			(*it)->getWrapperVoid();
	}

	/* The object must be destroyed before the listener */
	{
		std::unique_ptr<const Impl> owner(impl);
	}

	if (it != container.end())
		container.erase(it);
}


int pdraw_be_demuxer_destroy(struct pdraw_backend *self,
			     const struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	const auto *d = demuxer->impl;

	deleteImplAndEraseListener(self, self->demuxerListeners, d);

	return 0;
}


int pdraw_be_demuxer_close(const struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->close();
}


int pdraw_be_demuxer_get_media_list(const struct pdraw_backend *self,
				    struct pdraw_demuxer *demuxer,
				    struct pdraw_demuxer_media **media_list,
				    size_t *media_count,
				    uint32_t *selected_medias)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_be_demuxer_select_media(const struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint32_t selected_medias)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->selectMedia(selected_medias);
}


uint16_t pdraw_be_demuxer_get_single_stream_local_stream_port(
	const struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	auto *d = demuxer->impl;

	return d->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_be_demuxer_get_single_stream_local_control_port(
	const struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	auto *d = demuxer->impl;

	return d->getSingleStreamLocalControlPort();
}


int pdraw_be_demuxer_play(const struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->play();
}


int pdraw_be_demuxer_play_with_speed(const struct pdraw_backend *self,
				     struct pdraw_demuxer *demuxer,
				     float speed)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->play(speed);
}


int pdraw_be_demuxer_is_ready_to_play(const struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_be_demuxer_pause(const struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->pause();
}


int pdraw_be_demuxer_is_paused(const struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_be_demuxer_previous_frame(const struct pdraw_backend *self,
				    struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->previousFrame();
}


int pdraw_be_demuxer_next_frame(const struct pdraw_backend *self,
				struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->nextFrame();
}


int pdraw_be_demuxer_seek(const struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer,
			  int64_t delta,
			  int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seek(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_forward(const struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint64_t delta,
				  int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_back(const struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_to(const struct pdraw_backend *self,
			     struct pdraw_demuxer *demuxer,
			     uint64_t timestamp,
			     int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_be_demuxer_get_chapter_list(const struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer,
				      struct pdraw_chapter **chapter_list,
				      size_t *chapter_count)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_be_demuxer_get_duration(const struct pdraw_backend *self,
				       struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	auto *d = demuxer->impl;

	return d->getDuration();
}


uint64_t pdraw_be_demuxer_get_current_time(const struct pdraw_backend *self,
					   struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	auto *d = demuxer->impl;

	return d->getCurrentTime();
}


int pdraw_be_muxer_new(struct pdraw_backend *self,
		       const char *url,
		       const struct pdraw_muxer_params *params,
		       const struct pdraw_backend_muxer_cbs *cbs,
		       void *userdata,
		       struct pdraw_muxer **ret_obj)
{
	return pdraw_be_muxer_new_on_mux(
		self, url, nullptr, nullptr, params, cbs, userdata, ret_obj);
}


int pdraw_be_muxer_new_on_mux(struct pdraw_backend *self,
			      const char *url,
			      struct mux_ctx *mux,
			      const char *remote_host,
			      const struct pdraw_muxer_params *params,
			      const struct pdraw_backend_muxer_cbs *cbs,
			      void *userdata,
			      struct pdraw_muxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IMuxer *muxer = nullptr;
	std::unique_ptr<PdrawBackendMuxerListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendMuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	std::string rh(remote_host != nullptr ? remote_host : "");

	*ret_obj = l->getWrapper();
	res = self->pdraw->createMuxer(u, mux, rh, params, l.get(), &muxer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(muxer);
	self->muxerListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_muxer_destroy(struct pdraw_backend *self,
			   const struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	const auto *m = muxer->impl;

	deleteImplAndEraseListener(self, self->muxerListeners, m);

	return 0;
}


int pdraw_be_muxer_close(const struct pdraw_backend *self,
			 struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->close();
}


int pdraw_be_muxer_add_media(const struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     unsigned int media_id,
			     const struct pdraw_muxer_media_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->addMedia(media_id, params);
}


int pdraw_be_muxer_set_thumbnail(const struct pdraw_backend *self,
				 struct pdraw_muxer *muxer,
				 enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->setThumbnail(type, data, size);
}


int pdraw_muxer_set_file_metadata(
	const struct pdraw_backend *pdraw,
	struct pdraw_muxer *muxer,
	const struct pdraw_muxer_metadata_params *params,
	const uint8_t *data,
	size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->setFileMetadata(params, data, size);
}


int pdraw_be_muxer_add_chapter(const struct pdraw_backend *self,
			       struct pdraw_muxer *muxer,
			       uint64_t timestamp,
			       const char *name)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->addChapter(timestamp, name);
}


int pdraw_be_muxer_get_stats(const struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->getStats(stats);
}


int pdraw_be_video_renderer_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_backend_video_renderer_cbs *cbs,
	void *userdata,
	struct pdraw_video_renderer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVideoRenderer *renderer = nullptr;
	std::unique_ptr<PdrawBackendVideoRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendVideoRendererListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video renderer listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createVideoRenderer(
		media_id, render_pos, params, l.get(), &renderer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(renderer);
	self->videoRendererListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_video_renderer_destroy(struct pdraw_backend *self,
				    const struct pdraw_video_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	const auto *rnd = renderer->impl;

	deleteImplAndEraseListener(self, self->videoRendererListeners, rnd);

	return 0;
}


int pdraw_be_video_renderer_resize(const struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   const struct pdraw_rect *render_pos)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->resize(render_pos);
}


int pdraw_be_video_renderer_set_media_id(const struct pdraw_backend *self,
					 struct pdraw_video_renderer *renderer,
					 unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_video_renderer_get_media_id(const struct pdraw_backend *self,
				     struct pdraw_video_renderer *renderer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	auto *rnd = renderer->impl;

	return rnd->getMediaId();
}


int pdraw_be_video_renderer_set_params(
	const struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setParams(params);
}


int pdraw_be_video_renderer_get_params(
	const struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->getParams(params);
}


int pdraw_be_video_renderer_render(const struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   struct pdraw_rect *content_pos)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->render(content_pos, nullptr, nullptr);
}


int pdraw_be_video_renderer_render_mat(const struct pdraw_backend *self,
				       struct pdraw_video_renderer *renderer,
				       struct pdraw_rect *content_pos,
				       const float *view_mat,
				       const float *proj_mat)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->render(content_pos, view_mat, proj_mat);
}


int pdraw_be_audio_renderer_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_audio_renderer_params *params,
	const struct pdraw_backend_audio_renderer_cbs *cbs,
	void *userdata,
	struct pdraw_audio_renderer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioRenderer *renderer = nullptr;
	std::unique_ptr<PdrawBackendAudioRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendAudioRendererListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio renderer listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createAudioRenderer(
		media_id, params, l.get(), &renderer);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(renderer);
	self->audioRendererListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_audio_renderer_destroy(struct pdraw_backend *self,
				    const struct pdraw_audio_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	const auto *rnd = renderer->impl;

	deleteImplAndEraseListener(self, self->audioRendererListeners, rnd);

	return 0;
}


int pdraw_be_audio_renderer_set_media_id(const struct pdraw_backend *self,
					 struct pdraw_audio_renderer *renderer,
					 unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_audio_renderer_get_media_id(const struct pdraw_backend *self,
				     struct pdraw_audio_renderer *renderer)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	auto *rnd = renderer->impl;

	return rnd->getMediaId();
}


int pdraw_be_audio_renderer_set_params(
	const struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	const struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setParams(params);
}


int pdraw_be_audio_renderer_get_params(
	const struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->getParams(params);
}


int pdraw_be_vipc_source_new(struct pdraw_backend *self,
			     const struct pdraw_vipc_source_params *params,
			     const struct pdraw_backend_vipc_source_cbs *cbs,
			     void *userdata,
			     struct pdraw_vipc_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVipcSource *source = nullptr;
	std::unique_ptr<PdrawBackendVipcSourceListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendVipcSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create VIPC source listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createVipcSource(params, l.get(), &source);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(source);
	self->vipcSourceListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_vipc_source_destroy(struct pdraw_backend *self,
				 const struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListener(self, self->vipcSourceListeners, s);

	return 0;
}


int pdraw_be_vipc_source_is_ready_to_play(const struct pdraw_backend *self,
					  struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	auto *s = source->impl;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_vipc_source_is_paused(const struct pdraw_backend *self,
				   struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	auto *s = source->impl;

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_vipc_source_play(const struct pdraw_backend *self,
			      struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->play();
}


int pdraw_be_vipc_source_pause(const struct pdraw_backend *self,
			       struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->pause();
}


int pdraw_be_vipc_source_configure(const struct pdraw_backend *self,
				   struct pdraw_vipc_source *source,
				   const struct vdef_dim *resolution,
				   const struct vdef_rectf *crop)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->configure(resolution, crop);
}


int pdraw_be_vipc_source_insert_grey_frame(const struct pdraw_backend *self,
					   struct pdraw_vipc_source *source,
					   uint64_t ts_us)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->insertGreyFrame(ts_us);
}


int pdraw_be_vipc_source_set_session_metadata(const struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_be_vipc_source_get_session_metadata(const struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_be_coded_video_source_new(
	struct pdraw_backend *self,
	const struct pdraw_video_source_params *params,
	const struct pdraw_backend_coded_video_source_cbs *cbs,
	void *userdata,
	struct pdraw_coded_video_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::ICodedVideoSource *source = nullptr;
	std::unique_ptr<PdrawBackendCodedVideoSourceListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drained == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendCodedVideoSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video source listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createCodedVideoSource(params, l.get(), &source);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(source);
	self->codedVideoSourceListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_coded_video_source_destroy(
	struct pdraw_backend *self,
	const struct pdraw_coded_video_source *source)
{

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListener(self, self->codedVideoSourceListeners, s);


	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_source_get_queue(const struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_be_coded_video_source_flush(const struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_be_coded_video_source_drain(const struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_be_coded_video_source_set_session_metadata(
	const struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_be_coded_video_source_get_session_metadata(
	const struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_be_raw_video_source_new(
	struct pdraw_backend *self,
	const struct pdraw_video_source_params *params,
	const struct pdraw_backend_raw_video_source_cbs *cbs,
	void *userdata,
	struct pdraw_raw_video_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IRawVideoSource *source = nullptr;
	std::unique_ptr<PdrawBackendRawVideoSourceListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drained == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendRawVideoSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video source listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createRawVideoSource(params, l.get(), &source);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(source);
	self->rawVideoSourceListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_raw_video_source_destroy(
	struct pdraw_backend *self,
	const struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListener(self, self->rawVideoSourceListeners, s);

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_source_get_queue(const struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_be_raw_video_source_flush(const struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_be_raw_video_source_drain(const struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_be_raw_video_source_set_session_metadata(
	const struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_be_raw_video_source_get_session_metadata(
	const struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_be_coded_video_sink_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_video_sink_params *params,
	const struct pdraw_backend_coded_video_sink_cbs *cbs,
	void *userdata,
	struct pdraw_coded_video_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::ICodedVideoSink *sink = nullptr;
	std::unique_ptr<PdrawBackendCodedVideoSinkListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drain == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendCodedVideoSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video sink listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createCodedVideoSink(
		media_id, params, l.get(), &sink);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(sink);
	self->codedVideoSinkListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_coded_video_sink_destroy(struct pdraw_backend *self,
				      const struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListener(self, self->codedVideoSinkListeners, s);

	return 0;
}


int pdraw_be_coded_video_sink_resync(const struct pdraw_backend *self,
				     struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->resync();
}


int pdraw_be_coded_video_sink_set_media_id(const struct pdraw_backend *self,
					   struct pdraw_coded_video_sink *sink,
					   unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int
pdraw_be_coded_video_sink_get_media_id(const struct pdraw_backend *self,
				       struct pdraw_coded_video_sink *sink)
{
	if (self == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_sink_get_queue(const struct pdraw_backend *self,
				    struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_be_coded_video_sink_queue_flushed(const struct pdraw_backend *self,
					    struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_be_coded_video_sink_queue_drained(const struct pdraw_backend *self,
					    struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_be_raw_video_sink_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_video_sink_params *params,
	const struct pdraw_backend_raw_video_sink_cbs *cbs,
	void *userdata,
	struct pdraw_raw_video_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IRawVideoSink *sink = nullptr;
	std::unique_ptr<PdrawBackendRawVideoSinkListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drain == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendRawVideoSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createRawVideoSink(media_id, params, l.get(), &sink);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(sink);
	self->rawVideoSinkListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_raw_video_sink_destroy(struct pdraw_backend *self,
				    const struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListener(self, self->rawVideoSinkListeners, s);

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_sink_get_queue(const struct pdraw_backend *self,
				  struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_be_raw_video_sink_set_media_id(const struct pdraw_backend *self,
					 struct pdraw_raw_video_sink *sink,
					 unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int
pdraw_be_raw_video_sink_get_media_id(const struct pdraw_backend *self,
				     struct pdraw_raw_video_sink *sink)
{
	if (self == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


int pdraw_be_raw_video_sink_queue_flushed(const struct pdraw_backend *self,
					  struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_be_raw_video_sink_queue_drained(const struct pdraw_backend *self,
					  struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_be_alsa_source_new(struct pdraw_backend *self,
			     const struct pdraw_alsa_source_params *params,
			     const struct pdraw_backend_alsa_source_cbs *cbs,
			     void *userdata,
			     struct pdraw_alsa_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAlsaSource *source = nullptr;
	std::unique_ptr<PdrawBackendAlsaSourceListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendAlsaSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create ALSA source listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createAlsaSource(params, l.get(), &source);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(source);
	self->alsaSourceListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_alsa_source_destroy(struct pdraw_backend *self,
				 const struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListener(self, self->alsaSourceListeners, s);

	return 0;
}


int pdraw_be_alsa_source_is_ready_to_play(const struct pdraw_backend *self,
					  struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	auto *s = source->impl;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_alsa_source_is_paused(const struct pdraw_backend *self,
				   struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	auto *s = source->impl;

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_alsa_source_play(const struct pdraw_backend *self,
			      struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->play();
}


int pdraw_be_alsa_source_pause(const struct pdraw_backend *self,
			       struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->pause();
}


int pdraw_be_audio_source_new(struct pdraw_backend *self,
			      const struct pdraw_audio_source_params *params,
			      const struct pdraw_backend_audio_source_cbs *cbs,
			      void *userdata,
			      struct pdraw_audio_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSource *source = nullptr;
	std::unique_ptr<PdrawBackendAudioSourceListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drained == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendAudioSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createAudioSource(params, l.get(), &source);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(source);
	self->audioSourceListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_audio_source_destroy(struct pdraw_backend *self,
				  const struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListener(self, self->audioSourceListeners, s);

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_source_get_queue(const struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_be_audio_source_flush(const struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_be_audio_source_drain(const struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_be_audio_sink_new(struct pdraw_backend *self,
			    unsigned int media_id,
			    const struct pdraw_backend_audio_sink_cbs *cbs,
			    void *userdata,
			    struct pdraw_audio_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSink *sink = nullptr;
	std::unique_ptr<PdrawBackendAudioSinkListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->drain == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendAudioSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createAudioSink(media_id, l.get(), &sink);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(sink);
	self->audioSinkListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_audio_sink_destroy(struct pdraw_backend *self,
				const struct pdraw_audio_sink *sink)
{

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListener(self, self->audioSinkListeners, s);

	return 0;
}


int pdraw_be_audio_sink_set_media_id(const struct pdraw_backend *self,
				     struct pdraw_audio_sink *sink,
				     unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int pdraw_be_audio_sink_get_media_id(const struct pdraw_backend *self,
					      struct pdraw_audio_sink *sink)
{
	if (self == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_sink_get_queue(const struct pdraw_backend *self,
			      struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_be_audio_sink_queue_flushed(const struct pdraw_backend *self,
				      struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_be_audio_sink_queue_drained(const struct pdraw_backend *self,
				      struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_be_video_encoder_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct venc_config *params,
	const struct pdraw_backend_video_encoder_cbs *cbs,
	void *userdata,
	struct pdraw_video_encoder **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVideoEncoder *encoder = nullptr;
	std::unique_ptr<PdrawBackendVideoEncoderListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendVideoEncoderListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createVideoEncoder(
		media_id, params, l.get(), &encoder);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(encoder);
	self->videoEncoderListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_video_encoder_destroy(struct pdraw_backend *self,
				   const struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	const auto *e = encoder->impl;

	deleteImplAndEraseListener(self, self->videoEncoderListeners, e);

	return 0;
}


int pdraw_be_video_encoder_configure(const struct pdraw_backend *self,
				     struct pdraw_video_encoder *encoder,
				     const struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->configure(config);
}


int pdraw_be_video_encoder_get_config(const struct pdraw_backend *self,
				      struct pdraw_video_encoder *encoder,
				      struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->getConfig(config);
}


int pdraw_be_video_encoder_request_key_frame(
	const struct pdraw_backend *self,
	struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->requestKeyFrame();
}


int pdraw_be_video_scaler_new(struct pdraw_backend *self,
			      unsigned int media_id,
			      const struct vscale_config *params,
			      const struct pdraw_backend_video_scaler_cbs *cbs,
			      void *userdata,
			      struct pdraw_video_scaler **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVideoScaler *scaler = nullptr;
	std::unique_ptr<PdrawBackendVideoScalerListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendVideoScalerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video scaler listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createVideoScaler(
		media_id, params, l.get(), &scaler);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(scaler);
	self->videoScalerListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_video_scaler_destroy(struct pdraw_backend *self,
				  const struct pdraw_video_scaler *scaler)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(scaler == nullptr, EINVAL);

	const auto *s = scaler->impl;

	deleteImplAndEraseListener(self, self->videoScalerListeners, s);

	return 0;
}


int pdraw_be_audio_encoder_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct aenc_config *params,
	const struct pdraw_backend_audio_encoder_cbs *cbs,
	void *userdata,
	struct pdraw_audio_encoder **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioEncoder *encoder = nullptr;
	std::unique_ptr<PdrawBackendAudioEncoderListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		l = std::make_unique<PdrawBackendAudioEncoderListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio encoder listener");
		return -ENOMEM;
	}

	*ret_obj = l->getWrapper();
	res = self->pdraw->createAudioEncoder(
		media_id, params, l.get(), &encoder);
	if (res < 0) {
		*ret_obj = nullptr;
		return res;
	}

	l->setImpl(encoder);
	self->audioEncoderListeners.push_back(std::move(l));

	return 0;
}


int pdraw_be_audio_encoder_destroy(struct pdraw_backend *self,
				   const struct pdraw_audio_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	const auto *e = encoder->impl;

	deleteImplAndEraseListener(self, self->audioEncoderListeners, e);

	return 0;
}


int pdraw_be_get_friendly_name_setting(struct pdraw_backend *self,
				       char *str,
				       size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	std::string fn;
	self->pdraw->getFriendlyNameSetting(&fn);
	if (str && (fn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, fn.c_str());
	return 0;
}


int pdraw_be_set_friendly_name_setting(struct pdraw_backend *self,
				       const char *friendly_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(friendly_name == nullptr, EINVAL);

	std::string fn(friendly_name);
	self->pdraw->setFriendlyNameSetting(fn);
	return 0;
}


int pdraw_be_get_serial_number_setting(struct pdraw_backend *self,
				       char *str,
				       size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	std::string sn;
	self->pdraw->getSerialNumberSetting(&sn);
	if (str && (sn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sn.c_str());
	return 0;
}


int pdraw_be_set_serial_number_setting(struct pdraw_backend *self,
				       const char *serial_number)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(serial_number == nullptr, EINVAL);

	std::string sn(serial_number);
	self->pdraw->setSerialNumberSetting(sn);
	return 0;
}


int pdraw_be_get_software_version_setting(struct pdraw_backend *self,
					  char *str,
					  size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	std::string sv;
	self->pdraw->getSoftwareVersionSetting(&sv);
	if (str && (sv.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sv.c_str());
	return 0;
}


int pdraw_be_set_software_version_setting(struct pdraw_backend *self,
					  const char *software_version)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(software_version == nullptr, EINVAL);

	std::string sv(software_version);
	self->pdraw->setSoftwareVersionSetting(sv);
	return 0;
}


int pdraw_be_dump_pipeline(struct pdraw_backend *self, const char *file_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	std::string f(file_name ? file_name : "");
	return self->pdraw->dumpPipeline(f);
}
