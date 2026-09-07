/**
 * Parrot Drones Audio and Video Vector library
 * C wrapper functions
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

#define ULOG_TAG pdraw_wrapper
#include <ulog.h>

#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>
#include <pthread.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <pdraw/pdraw.h>

ULOG_DECLARE_TAG(ULOG_TAG);

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

struct pdraw;
static void registerListener(struct pdraw *p, ListenerBase *l);
static void unregisterListener(struct pdraw *p, ListenerBase *l);
static void *findWrapperForImpl(struct pdraw *p, void *impl);


template <typename HostT, typename CbsT, typename ImplT, typename WrapperT>
class ElementHolder : public ListenerBase {
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


/* codecheck_ignore[COMPLEX_MACRO] */
#define ENUM_CASE(_prefix, _name)                                              \
	case _prefix##_name:                                                   \
		return #_name


class PdrawListener : public Pdraw::IPdraw::Listener {
public:
	PdrawListener(struct pdraw *pdraw,
		      const struct pdraw_cbs *cbs,
		      void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_cbs mCbs;
	void *mUserdata = nullptr;
};


class PdrawDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener,
			     public ElementHolder<struct pdraw,
						  struct pdraw_demuxer_cbs,
						  Pdraw::IPdraw::IDemuxer,
						  struct pdraw_demuxer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawDemuxerListener() override = default;

	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status) override
	{
		if (mCbs.open_resp) {
			(*mCbs.open_resp)(mPdraw, &mWrapper, status, mUserdata);
		}
	}


	void demuxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status) override
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(
				mPdraw, &mWrapper, status, mUserdata);
		}
	}

	void
	onDemuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IDemuxer *demuxer) override
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(
				mPdraw, &mWrapper, mUserdata);
		}
	}

	int demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override
	{
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
		if (mCbs.ready_to_play) {
			(*mCbs.ready_to_play)(
				mPdraw, &mWrapper, ready ? 1 : 0, mUserdata);
		}
	}

	void onDemuxerEndOfRange(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override
	{
		if (mCbs.end_of_range) {
			(*mCbs.end_of_range)(
				mPdraw, &mWrapper, timestamp, mUserdata);
		}
	}

	void demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		if (mCbs.play_resp) {
			(*mCbs.play_resp)(mPdraw,
					  &mWrapper,
					  status,
					  timestamp,
					  speed,
					  mUserdata);
		}
	}

	void demuxerPauseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp) override
	{
		if (mCbs.pause_resp) {
			(*mCbs.pause_resp)(mPdraw,
					   &mWrapper,
					   status,
					   timestamp,
					   mUserdata);
		}
	}

	void demuxerSeekResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		if (mCbs.seek_resp) {
			(*mCbs.seek_resp)(mPdraw,
					  &mWrapper,
					  status,
					  timestamp,
					  speed,
					  mUserdata);
		}
	}
};


class PdrawMuxerListener : public Pdraw::IPdraw::IMuxer::Listener,
			   public ElementHolder<struct pdraw,
						struct pdraw_muxer_cbs,
						Pdraw::IPdraw::IMuxer,
						struct pdraw_muxer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawMuxerListener() override = default;

	void onMuxerConnectionStateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state connectionState,
		enum pdraw_muxer_disconnection_reason disconnectionReason)
		override
	{
		if (mCbs.connection_state_changed) {
			(*mCbs.connection_state_changed)(mPdraw,
							 &mWrapper,
							 connectionState,
							 disconnectionReason,
							 mUserdata);
		}
	}

	void onMuxerMediaReady(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath,
			       const struct iovec *iov,
			       int iovcnt) override
	{
		if (mCbs.media_ready) {
			(*mCbs.media_ready)(mPdraw,
					    &mWrapper,
					    mediaPath,
					    iov,
					    iovcnt,
					    mUserdata);
		}
	}

	void onMuxerMediaSaved(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath) override
	{
		if (mCbs.media_saved) {
			(*mCbs.media_saved)(
				mPdraw, &mWrapper, mediaPath, mUserdata);
		}
	}

	void onMuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IMuxer *muxer,
				       int status) override
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(
				mPdraw, &mWrapper, status, mUserdata);
		}
	}

	void muxerCloseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IMuxer *muxer,
				int status) override
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(
				mPdraw, &mWrapper, status, mUserdata);
		}
	}
};


class PdrawVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_video_renderer_cbs,
				       Pdraw::IPdraw::IVideoRenderer,
				       struct pdraw_video_renderer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawVideoRendererListener() override = default;

	void
	onVideoRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVideoRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void
	onVideoRendererMediaRemoved(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoRenderer *renderer,
				    const struct pdraw_media_info *info,
				    bool restart) override
	{
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
		if (mCbs.load_texture == nullptr)
			return -ENOSYS;
		if (pdraw == nullptr || renderer == nullptr)
			return -EINVAL;
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
		if (mCbs.render_overlay == nullptr)
			return -ENOSYS;
		if (pdraw == nullptr || renderer == nullptr)
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


class PdrawAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_audio_renderer_cbs,
				       Pdraw::IPdraw::IAudioRenderer,
				       struct pdraw_audio_renderer> {
public:
	using ElementHolder::ElementHolder;

	~PdrawAudioRendererListener() override = default;

	void
	onAudioRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onAudioRendererMediaRemoved(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, mUserdata);
	}
};


class PdrawVipcSourceListener
		: public Pdraw::IPdraw::IVipcSource::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_vipc_source_cbs,
				       Pdraw::IPdraw::IVipcSource,
				       struct pdraw_vipc_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawVipcSourceListener() override = default;

	void vipcSourceReadyToPlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
		bool ready,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(mPdraw,
					      &mWrapper,
					      (int)ready,
					      eosReason,
					      mUserdata);
	}

	void vipcSourcePlayResponse(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVipcSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void
	vipcSourcePauseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IVipcSource *source) override
	{
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(mPdraw, &mWrapper, mUserdata);
	}

	bool vipcSourceFramerateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
		const struct vdef_frac *prevFramerate,
		const struct vdef_frac *newFramerate) override
	{
		if (mCbs.framerate_changed)
			return (*mCbs.framerate_changed)(mPdraw,
							 &mWrapper,
							 prevFramerate,
							 newFramerate,
							 mUserdata);
		return false;
	}

	void vipcSourceConfigured(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
				  int status,
				  const struct vdef_format_info *info,
				  const struct vdef_rectf *crop) override
	{
		if (mCbs.configured)
			(*mCbs.configured)(mPdraw,
					   &mWrapper,
					   status,
					   info,
					   crop,
					   mUserdata);
	}

	void vipcSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
				  struct mbuf_raw_video_frame *frame) override
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	bool vipcSourceEndOfStream(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		if (mCbs.end_of_stream)
			return (*mCbs.end_of_stream)(
				mPdraw, &mWrapper, eosReason, mUserdata);
		return false;
	}
};


class PdrawCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_coded_video_source_cbs,
				       Pdraw::IPdraw::ICodedVideoSource,
				       struct pdraw_coded_video_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawCodedVideoSourceListener() override = default;

	void onCodedVideoSourceFlushed(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void onCodedVideoSourceDrained(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_raw_video_source_cbs,
				       Pdraw::IPdraw::IRawVideoSource,
				       struct pdraw_raw_video_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawRawVideoSourceListener() override = default;

	void
	onRawVideoSourceFlushed(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void
	onRawVideoSourceDrained(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_coded_video_sink_cbs,
				       Pdraw::IPdraw::ICodedVideoSink,
				       struct pdraw_coded_video_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawCodedVideoSinkListener() override = default;

	void
	onCodedVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::ICodedVideoSink *sink,
				   const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onCodedVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
					  const struct pdraw_media_info *info,
					  bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void
	onCodedVideoSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void
	onCodedVideoSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}

	void onCodedVideoSinkSessionMetaUpdate(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw, &mWrapper, meta, mUserdata);
	}
};


class PdrawRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_raw_video_sink_cbs,
				       Pdraw::IPdraw::IRawVideoSink,
				       struct pdraw_raw_video_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawRawVideoSinkListener() override = default;

	void
	onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink,
				 const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onRawVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IRawVideoSink *sink,
					const struct pdraw_media_info *info,
					bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void onRawVideoSinkFlush(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void onRawVideoSinkDrain(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}

	void onRawVideoSinkSessionMetaUpdate(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IRawVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw, &mWrapper, meta, mUserdata);
	}
};


class PdrawAlsaSourceListener
		: public Pdraw::IPdraw::IAlsaSource::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_alsa_source_cbs,
				       Pdraw::IPdraw::IAlsaSource,
				       struct pdraw_alsa_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawAlsaSourceListener() override = default;

	void alsaSourceReadyToPlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override
	{
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(mPdraw,
					      &mWrapper,
					      (int)ready,
					      eosReason,
					      mUserdata);
	}

	void alsaSourcePlayResponse(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IAlsaSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void
	alsaSourcePauseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IAlsaSource *source) override
	{
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(mPdraw, &mWrapper, mUserdata);
	}

	void alsaSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawAudioSourceListener
		: public Pdraw::IPdraw::IAudioSource::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_audio_source_cbs,
				       Pdraw::IPdraw::IAudioSource,
				       struct pdraw_audio_source> {
public:
	using ElementHolder::ElementHolder;

	~PdrawAudioSourceListener() override = default;

	void onAudioSourceFlushed(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(mPdraw, &mWrapper, mUserdata);
	}

	void onAudioSourceDrained(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawAudioSinkListener : public Pdraw::IPdraw::IAudioSink::Listener,
			       public ElementHolder<struct pdraw,
						    struct pdraw_audio_sink_cbs,
						    Pdraw::IPdraw::IAudioSink,
						    struct pdraw_audio_sink> {
public:
	using ElementHolder::ElementHolder;

	~PdrawAudioSinkListener() override = default;

	void onAudioSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IAudioSink *sink,
				   const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(mPdraw, &mWrapper, info, mUserdata);
	}

	void onAudioSinkMediaRemoved(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioSink *sink,
				     const struct pdraw_media_info *info,
				     bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw, &mWrapper, info, restart, mUserdata);
	}

	void onAudioSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, &mWrapper, mUserdata);
	}

	void onAudioSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(mPdraw, &mWrapper, mUserdata);
	}
};


class PdrawVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_video_encoder_cbs,
				       Pdraw::IPdraw::IVideoEncoder,
				       struct pdraw_video_encoder> {
public:
	using ElementHolder::ElementHolder;

	~PdrawVideoEncoderListener() override = default;

	void
	videoEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	void videoEncoderFramePreRelease(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVideoEncoder *encoder,
		struct mbuf_coded_video_frame *frame) override
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawVideoScalerListener
		: public Pdraw::IPdraw::IVideoScaler::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_video_scaler_cbs,
				       Pdraw::IPdraw::IVideoScaler,
				       struct pdraw_video_scaler> {
public:
	using ElementHolder::ElementHolder;

	~PdrawVideoScalerListener() override = default;

	void videoScalerFrameOutput(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoScaler *scaler,
				    struct mbuf_raw_video_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


class PdrawAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener,
		  public ElementHolder<struct pdraw,
				       struct pdraw_audio_encoder_cbs,
				       Pdraw::IPdraw::IAudioEncoder,
				       struct pdraw_audio_encoder> {
public:
	using ElementHolder::ElementHolder;

	~PdrawAudioEncoderListener() override = default;

	void audioEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw, &mWrapper, frame, mUserdata);
	}

	void
	audioEncoderFramePreRelease(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IAudioEncoder *encoder,
				    struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw, &mWrapper, frame, mUserdata);
	}
};


struct pdraw {
	std::unique_ptr<Pdraw::IPdraw> pdraw{};
	std::unique_ptr<PdrawListener> listener{};
	std::mutex mutex{};
	/* Declared first so it outlives all listener vectors below and remains
	 * valid during their destruction (listeners call unregisterListener in
	 * ~ElementHolder). */
	std::vector<ListenerBase *> allListeners{};
	std::mutex allListenersMutex{};
	std::vector<std::unique_ptr<PdrawDemuxerListener>> demuxerListeners{};
	std::vector<std::unique_ptr<PdrawMuxerListener>> muxerListeners{};
	std::vector<std::unique_ptr<PdrawVideoRendererListener>>
		videoRendererListeners{};
	std::vector<std::unique_ptr<PdrawAudioRendererListener>>
		audioRendererListeners{};
	std::vector<std::unique_ptr<PdrawVipcSourceListener>>
		vipcSourceListeners{};
	std::vector<std::unique_ptr<PdrawCodedVideoSourceListener>>
		codedVideoSourceListeners{};
	std::vector<std::unique_ptr<PdrawRawVideoSourceListener>>
		rawVideoSourceListeners{};
	std::vector<std::unique_ptr<PdrawCodedVideoSinkListener>>
		codedVideoSinkListeners{};
	std::vector<std::unique_ptr<PdrawRawVideoSinkListener>>
		rawVideoSinkListeners{};
	std::vector<std::unique_ptr<PdrawAlsaSourceListener>>
		alsaSourceListeners{};
	std::vector<std::unique_ptr<PdrawAudioSourceListener>>
		audioSourceListeners{};
	std::vector<std::unique_ptr<PdrawAudioSinkListener>>
		audioSinkListeners{};
	std::vector<std::unique_ptr<PdrawVideoEncoderListener>>
		videoEncoderListeners{};
	std::vector<std::unique_ptr<PdrawVideoScalerListener>>
		videoScalerListeners{};
	std::vector<std::unique_ptr<PdrawAudioEncoderListener>>
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


static void registerListener(struct pdraw *p, ListenerBase *l)
{
	if (p == nullptr || l == nullptr)
		return;
	std::scoped_lock lock(p->allListenersMutex);
	p->allListeners.push_back(l);
}


static void unregisterListener(struct pdraw *p, ListenerBase *l)
{
	if (p == nullptr || l == nullptr)
		return;
	std::scoped_lock lock(p->allListenersMutex);
	auto it = std::find(p->allListeners.begin(), p->allListeners.end(), l);
	if (it != p->allListeners.end())
		p->allListeners.erase(it);
}


static void *findWrapperForImpl(struct pdraw *p, void *impl)
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


void PdrawListener::onMediaRemoved(Pdraw::IPdraw *pdraw,
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


int pdraw_new(struct pomp_loop *loop,
	      const struct pdraw_cbs *cbs,
	      void *userdata,
	      struct pdraw **ret_obj)
{
	ULOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	std::unique_ptr<struct pdraw> pdraw;

	try {
		pdraw = std::make_unique<struct pdraw>();
		pdraw->listener = std::make_unique<PdrawListener>(
			pdraw.get(), cbs, userdata);
		pdraw->pdraw = std::make_unique<Pdraw::Session>(
			loop, pdraw->listener.get());
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}

	/* Ownership transferred to caller via public C API raw pointer */
	*ret_obj = pdraw.release();
	return 0;
}


int pdraw_destroy(struct pdraw *pdraw)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	auto owner = std::unique_ptr<struct pdraw>(pdraw);

	owner->pdraw.reset();
	owner->listener.reset();

	owner->demuxerListeners.clear();
	owner->muxerListeners.clear();
	owner->codedVideoSourceListeners.clear();
	owner->vipcSourceListeners.clear();
	owner->rawVideoSourceListeners.clear();
	owner->codedVideoSinkListeners.clear();
	owner->rawVideoSinkListeners.clear();
	owner->alsaSourceListeners.clear();
	owner->audioSourceListeners.clear();
	owner->audioSinkListeners.clear();
	owner->videoScalerListeners.clear();
	owner->audioEncoderListeners.clear();
	owner->audioEncoderListeners.clear();

	{
		std::scoped_lock lock(owner->mutex);
		owner->videoRendererListeners.clear();
		owner->audioRendererListeners.clear();
	}

	return 0;
}


int pdraw_stop(struct pdraw *pdraw)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	return pdraw->pdraw->stop();
}


int pdraw_demuxer_new_from_url(struct pdraw *pdraw,
			       const char *url,
			       const struct pdraw_demuxer_params *params,
			       const struct pdraw_demuxer_cbs *cbs,
			       void *userdata,
			       struct pdraw_demuxer **ret_obj)
{
	return pdraw_demuxer_new_from_url_on_mux(
		pdraw, url, nullptr, params, cbs, userdata, ret_obj);
}


int pdraw_demuxer_new_single_stream(struct pdraw *pdraw,
				    const char *local_addr,
				    uint16_t local_stream_port,
				    uint16_t local_control_port,
				    const char *remote_addr,
				    uint16_t remote_stream_port,
				    uint16_t remote_control_port,
				    const struct pdraw_demuxer_params *params,
				    const struct pdraw_demuxer_cbs *cbs,
				    void *userdata,
				    struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	std::unique_ptr<PdrawDemuxerListener> demuxerListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_addr == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		demuxerListener = std::make_unique<PdrawDemuxerListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string local(local_addr);
	std::string remote(remote_addr);
	res = pdraw->pdraw->createDemuxer(local,
					  local_stream_port,
					  local_control_port,
					  remote,
					  remote_stream_port,
					  remote_control_port,
					  params,
					  demuxerListener.get(),
					  &demuxer);
	if (res < 0)
		return res;

	demuxerListener->setImpl(demuxer);
	*ret_obj = demuxerListener->getWrapper();
	pdraw->demuxerListeners.push_back(std::move(demuxerListener));

	return 0;
}


int pdraw_demuxer_new_from_url_on_mux(struct pdraw *pdraw,
				      const char *url,
				      struct mux_ctx *mux,
				      const struct pdraw_demuxer_params *params,
				      const struct pdraw_demuxer_cbs *cbs,
				      void *userdata,
				      struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;
	std::unique_ptr<PdrawDemuxerListener> demuxerListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == nullptr, EINVAL);
	/* Note: deliberately not testing the mux pointer, as
	 * pdraw_demuxer_new_from_url() calls this function with
	 * a null mux pointer */
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		demuxerListener = std::make_unique<PdrawDemuxerListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createDemuxer(
		u, mux, params, demuxerListener.get(), &demuxer);
	if (res < 0)
		return res;

	demuxerListener->setImpl(demuxer);
	*ret_obj = demuxerListener->getWrapper();
	pdraw->demuxerListeners.push_back(std::move(demuxerListener));

	return 0;
}


template <typename Container, typename Impl>
static void deleteImplAndEraseListenerUnlocked(struct pdraw *pdraw,
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
		std::scoped_lock lock(pdraw->pendingRemovedMutex);
		pdraw->pendingRemovedUserdataMap[(void *)impl] =
			(*it)->getWrapperVoid();
	}

	/* The object must be destroyed before the listener */
	{
		std::unique_ptr<const Impl> owner(impl);
	}

	if (it != container.end())
		container.erase(it);
}


template <typename Container, typename Impl>
static void deleteImplAndEraseListenerLocked(struct pdraw *pdraw,
					     Container &container,
					     const Impl *impl,
					     std::mutex &mutex)
{
	std::scoped_lock lock(mutex);
	deleteImplAndEraseListenerUnlocked(pdraw, container, impl);
}


int pdraw_demuxer_destroy(struct pdraw *pdraw,
			  const struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	const auto *d = demuxer->impl;

	deleteImplAndEraseListenerUnlocked(pdraw, pdraw->demuxerListeners, d);

	return 0;
}


int pdraw_demuxer_close(const struct pdraw *pdraw,
			struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->close();
}


int pdraw_demuxer_get_media_list(const struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer,
				 struct pdraw_demuxer_media **media_list,
				 size_t *media_count,
				 uint32_t *selected_medias)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_demuxer_select_media(const struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint32_t selected_medias)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->selectMedia(selected_medias);
}


uint16_t
pdraw_demuxer_get_single_stream_local_stream_port(const struct pdraw *pdraw,
						  struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return d->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_demuxer_get_single_stream_local_control_port(
	const struct pdraw *pdraw,
	struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return d->getSingleStreamLocalControlPort();
}


int pdraw_demuxer_is_ready_to_play(const struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_demuxer_is_paused(const struct pdraw *pdraw,
			    struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_demuxer_play(const struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->play();
}


int pdraw_demuxer_play_with_speed(const struct pdraw *pdraw,
				  struct pdraw_demuxer *demuxer,
				  float speed)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->play(speed);
}


int pdraw_demuxer_pause(const struct pdraw *pdraw,
			struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->pause();
}


int pdraw_demuxer_previous_frame(const struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->previousFrame();
}


int pdraw_demuxer_next_frame(const struct pdraw *pdraw,
			     struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->nextFrame();
}


int pdraw_demuxer_seek(const struct pdraw *pdraw,
		       struct pdraw_demuxer *demuxer,
		       int64_t delta,
		       int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seek(delta, exact ? true : false);
}


int pdraw_demuxer_seek_forward(const struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_demuxer_seek_back(const struct pdraw *pdraw,
			    struct pdraw_demuxer *demuxer,
			    uint64_t delta,
			    int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_demuxer_seek_to(const struct pdraw *pdraw,
			  struct pdraw_demuxer *demuxer,
			  uint64_t timestamp,
			  int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_demuxer_get_chapter_list(const struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer,
				   struct pdraw_chapter **chapter_list,
				   size_t *chapter_count)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = demuxer->impl;

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_demuxer_get_duration(const struct pdraw *pdraw,
				    struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return d->getDuration();
}


uint64_t pdraw_demuxer_get_current_time(const struct pdraw *pdraw,
					struct pdraw_demuxer *demuxer)
{
	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	auto *d = demuxer->impl;

	return d->getCurrentTime();
}


int pdraw_muxer_new(struct pdraw *pdraw,
		    const char *url,
		    const struct pdraw_muxer_params *params,
		    const struct pdraw_muxer_cbs *cbs,
		    void *userdata,
		    struct pdraw_muxer **ret_obj)
{
	return pdraw_muxer_new_on_mux(
		pdraw, url, nullptr, nullptr, params, cbs, userdata, ret_obj);
}


int pdraw_muxer_new_on_mux(struct pdraw *pdraw,
			   const char *url,
			   struct mux_ctx *mux,
			   const char *remote_host,
			   const struct pdraw_muxer_params *params,
			   const struct pdraw_muxer_cbs *cbs,
			   void *userdata,
			   struct pdraw_muxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IMuxer *muxer = nullptr;
	std::unique_ptr<PdrawMuxerListener> muxerListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == nullptr, EINVAL);
	/* Note: deliberately not testing the mux pointer, as
	 * pdraw_muxer_new() calls this function with a null mux pointer */
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		muxerListener = std::make_unique<PdrawMuxerListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	std::string rh(remote_host != nullptr ? remote_host : "");
	res = pdraw->pdraw->createMuxer(
		u, mux, rh, params, muxerListener.get(), &muxer);
	if (res < 0)
		return res;

	muxerListener->setImpl(muxer);
	*ret_obj = muxerListener->getWrapper();
	pdraw->muxerListeners.push_back(std::move(muxerListener));
	return 0;
}


int pdraw_muxer_destroy(struct pdraw *pdraw, const struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	const auto *m = muxer->impl;

	deleteImplAndEraseListenerUnlocked(pdraw, pdraw->muxerListeners, m);

	return 0;
}


int pdraw_muxer_close(const struct pdraw *self, struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->close();
}


int pdraw_muxer_add_media(const struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  unsigned int media_id,
			  const struct pdraw_muxer_media_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->addMedia(media_id, params);
}


int pdraw_muxer_set_thumbnail(const struct pdraw *pdraw,
			      struct pdraw_muxer *muxer,
			      enum pdraw_muxer_thumbnail_type type,
			      const uint8_t *data,
			      size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->setThumbnail(type, data, size);
}


int pdraw_muxer_set_file_metadata(
	const struct pdraw *pdraw,
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


int pdraw_muxer_add_chapter(const struct pdraw *pdraw,
			    struct pdraw_muxer *muxer,
			    uint64_t timestamp,
			    const char *name)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->addChapter(timestamp, name);
}


int pdraw_muxer_get_stats(const struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->getStats(stats);
}


int pdraw_muxer_set_dyn_params(const struct pdraw *pdraw,
			       struct pdraw_muxer *muxer,
			       const struct pdraw_muxer_dyn_params *dyn_params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->setDynParams(dyn_params);
}


int pdraw_muxer_get_dyn_params(const struct pdraw *pdraw,
			       struct pdraw_muxer *muxer,
			       struct pdraw_muxer_dyn_params *dyn_params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->getDynParams(dyn_params);
}


int pdraw_muxer_force_sync(const struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = muxer->impl;

	return m->forceSync();
}


int pdraw_video_renderer_new(struct pdraw *pdraw,
			     unsigned int media_id,
			     const struct pdraw_rect *render_pos,
			     const struct pdraw_video_renderer_params *params,
			     const struct pdraw_video_renderer_cbs *cbs,
			     void *userdata,
			     struct pdraw_video_renderer **ret_obj)
{
	int ret = 0;
	Pdraw::IPdraw::IVideoRenderer *renderer = nullptr;
	std::unique_ptr<PdrawVideoRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	{
		std::scoped_lock lock(pdraw->mutex);
		try {
			l = std::make_unique<PdrawVideoRendererListener>(
				pdraw, cbs, userdata);
		} catch (const std::bad_alloc &) {
			ULOGE("failed to create video renderer listener");
			ret = -ENOMEM;
			return ret;
		}

		ret = pdraw->pdraw->createVideoRenderer(
			media_id, render_pos, params, l.get(), &renderer);
		if (ret < 0)
			return ret;

		l->setImpl(renderer);
		*ret_obj = l->getWrapper();
		pdraw->videoRendererListeners.push_back(std::move(l));
	}

	return 0;
}


int pdraw_video_renderer_destroy(struct pdraw *pdraw,
				 const struct pdraw_video_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	const auto *rnd = renderer->impl;

	deleteImplAndEraseListenerLocked(
		pdraw, pdraw->videoRendererListeners, rnd, pdraw->mutex);

	return 0;
}


int pdraw_video_renderer_resize(const struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *render_pos)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->resize(render_pos);
}


int pdraw_video_renderer_set_media_id(const struct pdraw *pdraw,
				      struct pdraw_video_renderer *renderer,
				      unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_video_renderer_get_media_id(const struct pdraw *pdraw,
				  struct pdraw_video_renderer *renderer)
{
	if (pdraw == nullptr)
		return 0;
	if (renderer == nullptr)
		return 0;

	auto *rnd = renderer->impl;

	return rnd->getMediaId();
}


int pdraw_video_renderer_set_params(
	const struct pdraw *pdraw,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setParams(params);
}


int pdraw_video_renderer_get_params(const struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->getParams(params);
}


int pdraw_video_renderer_render(const struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				struct pdraw_rect *content_pos)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->render(content_pos, nullptr, nullptr);
}


int pdraw_video_renderer_render_mat(const struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_rect *content_pos,
				    const float *view_mat,
				    const float *proj_mat)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->render(content_pos, view_mat, proj_mat);
}


int pdraw_audio_renderer_new(struct pdraw *pdraw,
			     unsigned int media_id,
			     const struct pdraw_audio_renderer_params *params,
			     const struct pdraw_audio_renderer_cbs *cbs,
			     void *userdata,
			     struct pdraw_audio_renderer **ret_obj)
{
	int ret = 0;
	Pdraw::IPdraw::IAudioRenderer *renderer = nullptr;
	std::unique_ptr<PdrawAudioRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	{
		std::scoped_lock lock(pdraw->mutex);
		try {
			l = std::make_unique<PdrawAudioRendererListener>(
				pdraw, cbs, userdata);
		} catch (const std::bad_alloc &) {
			ULOGE("failed to create audio renderer listener");
			ret = -ENOMEM;
			return ret;
		}

		ret = pdraw->pdraw->createAudioRenderer(
			media_id, params, l.get(), &renderer);
		if (ret < 0)
			return ret;

		l->setImpl(renderer);
		*ret_obj = l->getWrapper();
		pdraw->audioRendererListeners.push_back(std::move(l));
	}

	return 0;
}


int pdraw_audio_renderer_destroy(struct pdraw *pdraw,
				 const struct pdraw_audio_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	const auto *rnd = renderer->impl;

	deleteImplAndEraseListenerLocked(
		pdraw, pdraw->audioRendererListeners, rnd, pdraw->mutex);

	return 0;
}


int pdraw_audio_renderer_set_media_id(const struct pdraw *pdraw,
				      struct pdraw_audio_renderer *renderer,
				      unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_audio_renderer_get_media_id(const struct pdraw *pdraw,
				  struct pdraw_audio_renderer *renderer)
{
	if (pdraw == nullptr)
		return 0;
	if (renderer == nullptr)
		return 0;

	auto *rnd = renderer->impl;

	return rnd->getMediaId();
}


int pdraw_audio_renderer_set_params(
	const struct pdraw *pdraw,
	struct pdraw_audio_renderer *renderer,
	const struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->setParams(params);
}


int pdraw_audio_renderer_get_params(const struct pdraw *pdraw,
				    struct pdraw_audio_renderer *renderer,
				    struct pdraw_audio_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = renderer->impl;

	return rnd->getParams(params);
}


int pdraw_vipc_source_new(struct pdraw *pdraw,
			  const struct pdraw_vipc_source_params *params,
			  const struct pdraw_vipc_source_cbs *cbs,
			  void *userdata,
			  struct pdraw_vipc_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVipcSource *source = nullptr;
	std::unique_ptr<PdrawVipcSourceListener> vipcSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		vipcSourceListener = std::make_unique<PdrawVipcSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create VIPC source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVipcSource(
		params, vipcSourceListener.get(), &source);
	if (res < 0)
		return res;

	vipcSourceListener->setImpl(source);
	*ret_obj = vipcSourceListener->getWrapper();
	pdraw->vipcSourceListeners.push_back(std::move(vipcSourceListener));

	return 0;
}


int pdraw_vipc_source_destroy(struct pdraw *pdraw,
			      const struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->vipcSourceListeners, s);

	return 0;
}


int pdraw_vipc_source_is_ready_to_play(const struct pdraw *pdraw,
				       struct pdraw_vipc_source *source)
{
	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	auto *s = source->impl;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_vipc_source_is_paused(const struct pdraw *pdraw,
				struct pdraw_vipc_source *source)
{
	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	auto *s = source->impl;

	return s->isPaused() ? 1 : 0;
}


int pdraw_vipc_source_play(const struct pdraw *pdraw,
			   struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->play();
}


int pdraw_vipc_source_pause(const struct pdraw *pdraw,
			    struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->pause();
}


int pdraw_vipc_source_configure(const struct pdraw *pdraw,
				struct pdraw_vipc_source *source,
				const struct vdef_dim *resolution,
				const struct vdef_rectf *crop)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->configure(resolution, crop);
}


int pdraw_vipc_source_insert_grey_frame(const struct pdraw *pdraw,
					struct pdraw_vipc_source *source,
					uint64_t ts_us)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->insertGreyFrame(ts_us);
}


int pdraw_vipc_source_set_session_metadata(const struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_vipc_source_get_session_metadata(const struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_coded_video_source_new(struct pdraw *pdraw,
				 const struct pdraw_video_source_params *params,
				 const struct pdraw_coded_video_source_cbs *cbs,
				 void *userdata,
				 struct pdraw_coded_video_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::ICodedVideoSource *source = nullptr;
	std::unique_ptr<PdrawCodedVideoSourceListener> videoSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSourceListener =
			std::make_unique<PdrawCodedVideoSourceListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSource(
		params, videoSourceListener.get(), &source);
	if (res < 0)
		return res;

	videoSourceListener->setImpl(source);
	*ret_obj = videoSourceListener->getWrapper();
	pdraw->codedVideoSourceListeners.push_back(
		std::move(videoSourceListener));

	return 0;
}


int pdraw_coded_video_source_destroy(
	struct pdraw *pdraw,
	const struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->codedVideoSourceListeners, s);

	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_source_get_queue(const struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_coded_video_source_flush(const struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_coded_video_source_drain(const struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_coded_video_source_set_session_metadata(
	const struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_coded_video_source_get_session_metadata(
	const struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_raw_video_source_new(struct pdraw *pdraw,
			       const struct pdraw_video_source_params *params,
			       const struct pdraw_raw_video_source_cbs *cbs,
			       void *userdata,
			       struct pdraw_raw_video_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IRawVideoSource *source = nullptr;
	std::unique_ptr<PdrawRawVideoSourceListener> videoSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSourceListener =
			std::make_unique<PdrawRawVideoSourceListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSource(
		params, videoSourceListener.get(), &source);
	if (res < 0)
		return res;

	videoSourceListener->setImpl(source);
	*ret_obj = videoSourceListener->getWrapper();
	pdraw->rawVideoSourceListeners.push_back(
		std::move(videoSourceListener));

	return 0;
}


int pdraw_raw_video_source_destroy(struct pdraw *pdraw,
				   const struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->rawVideoSourceListeners, s);

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_source_get_queue(const struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_raw_video_source_flush(const struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_raw_video_source_drain(const struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_raw_video_source_set_session_metadata(
	const struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->setSessionMetadata(meta);
}


int pdraw_raw_video_source_get_session_metadata(
	const struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->getSessionMetadata(meta);
}


int pdraw_coded_video_sink_new(struct pdraw *pdraw,
			       unsigned int media_id,
			       const struct pdraw_video_sink_params *params,
			       const struct pdraw_coded_video_sink_cbs *cbs,
			       void *userdata,
			       struct pdraw_coded_video_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::ICodedVideoSink *sink = nullptr;
	std::unique_ptr<PdrawCodedVideoSinkListener> videoSinkListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSinkListener =
			std::make_unique<PdrawCodedVideoSinkListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSink(
		media_id, params, videoSinkListener.get(), &sink);
	if (res < 0)
		return res;

	videoSinkListener->setImpl(sink);
	*ret_obj = videoSinkListener->getWrapper();
	pdraw->codedVideoSinkListeners.push_back(std::move(videoSinkListener));

	return 0;
}


int pdraw_coded_video_sink_destroy(struct pdraw *pdraw,
				   const struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->codedVideoSinkListeners, s);

	return 0;
}


int pdraw_coded_video_sink_resync(const struct pdraw *pdraw,
				  struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->resync();
}


int pdraw_coded_video_sink_set_media_id(const struct pdraw *pdraw,
					struct pdraw_coded_video_sink *sink,
					unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int
pdraw_coded_video_sink_get_media_id(const struct pdraw *pdraw,
				    struct pdraw_coded_video_sink *sink)
{
	if (pdraw == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;

	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_sink_get_queue(const struct pdraw *pdraw,
				 struct pdraw_coded_video_sink *sink)
{
	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_coded_video_sink_queue_flushed(const struct pdraw *pdraw,
					 struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_coded_video_sink_queue_drained(const struct pdraw *pdraw,
					 struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_raw_video_sink_new(struct pdraw *pdraw,
			     unsigned int media_id,
			     const struct pdraw_video_sink_params *params,
			     const struct pdraw_raw_video_sink_cbs *cbs,
			     void *userdata,
			     struct pdraw_raw_video_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IRawVideoSink *sink = nullptr;
	std::unique_ptr<PdrawRawVideoSinkListener> videoSinkListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSinkListener = std::make_unique<PdrawRawVideoSinkListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSink(
		media_id, params, videoSinkListener.get(), &sink);
	if (res < 0)
		return res;

	videoSinkListener->setImpl(sink);
	*ret_obj = videoSinkListener->getWrapper();
	pdraw->rawVideoSinkListeners.push_back(std::move(videoSinkListener));

	return 0;
}


int pdraw_raw_video_sink_destroy(struct pdraw *pdraw,
				 const struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->rawVideoSinkListeners, s);

	return 0;
}


int pdraw_raw_video_sink_set_media_id(const struct pdraw *pdraw,
				      struct pdraw_raw_video_sink *sink,
				      unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int
pdraw_raw_video_sink_get_media_id(const struct pdraw *pdraw,
				  struct pdraw_raw_video_sink *sink)
{
	if (pdraw == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;

	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_sink_get_queue(const struct pdraw *pdraw,
			       struct pdraw_raw_video_sink *sink)
{
	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_raw_video_sink_queue_flushed(const struct pdraw *pdraw,
				       struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_raw_video_sink_queue_drained(const struct pdraw *pdraw,
				       struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_alsa_source_new(struct pdraw *pdraw,
			  const struct pdraw_alsa_source_params *params,
			  const struct pdraw_alsa_source_cbs *cbs,
			  void *userdata,
			  struct pdraw_alsa_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAlsaSource *source = nullptr;
	std::unique_ptr<PdrawAlsaSourceListener> alsaSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		alsaSourceListener = std::make_unique<PdrawAlsaSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create ALSA source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAlsaSource(
		params, alsaSourceListener.get(), &source);
	if (res < 0)
		return res;

	alsaSourceListener->setImpl(source);
	*ret_obj = alsaSourceListener->getWrapper();
	pdraw->alsaSourceListeners.push_back(std::move(alsaSourceListener));

	return 0;
}


int pdraw_alsa_source_destroy(struct pdraw *pdraw,
			      const struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->alsaSourceListeners, s);

	return 0;
}


int pdraw_alsa_source_is_ready_to_play(const struct pdraw *pdraw,
				       struct pdraw_alsa_source *source)
{
	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	auto *s = source->impl;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_alsa_source_is_paused(const struct pdraw *pdraw,
				struct pdraw_alsa_source *source)
{
	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	auto *s = source->impl;

	return s->isPaused() ? 1 : 0;
}


int pdraw_alsa_source_play(const struct pdraw *pdraw,
			   struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->play();
}


int pdraw_alsa_source_pause(const struct pdraw *pdraw,
			    struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->pause();
}


int pdraw_audio_source_new(struct pdraw *pdraw,
			   const struct pdraw_audio_source_params *params,
			   const struct pdraw_audio_source_cbs *cbs,
			   void *userdata,
			   struct pdraw_audio_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSource *source = nullptr;
	std::unique_ptr<PdrawAudioSourceListener> audioSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		audioSourceListener =
			std::make_unique<PdrawAudioSourceListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSource(
		params, audioSourceListener.get(), &source);
	if (res < 0)
		return res;

	audioSourceListener->setImpl(source);
	*ret_obj = audioSourceListener->getWrapper();
	pdraw->audioSourceListeners.push_back(std::move(audioSourceListener));

	return 0;
}


int pdraw_audio_source_destroy(struct pdraw *pdraw,
			       const struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	const auto *s = source->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->audioSourceListeners, s);

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_audio_source_get_queue(const struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	auto *s = source->impl;

	return s->getQueue();
}


int pdraw_audio_source_flush(const struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->flush();
}


int pdraw_audio_source_drain(const struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = source->impl;

	return s->drain();
}


int pdraw_audio_sink_new(struct pdraw *pdraw,
			 unsigned int media_id,
			 const struct pdraw_audio_sink_cbs *cbs,
			 void *userdata,
			 struct pdraw_audio_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSink *sink = nullptr;
	std::unique_ptr<PdrawAudioSinkListener> audioSinkListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		audioSinkListener = std::make_unique<PdrawAudioSinkListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSink(
		media_id, audioSinkListener.get(), &sink);
	if (res < 0)
		return res;

	audioSinkListener->setImpl(sink);
	*ret_obj = audioSinkListener->getWrapper();
	pdraw->audioSinkListeners.push_back(std::move(audioSinkListener));

	return 0;
}


int pdraw_audio_sink_destroy(struct pdraw *pdraw,
			     const struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	const auto *s = sink->impl;

	deleteImplAndEraseListenerUnlocked(pdraw, pdraw->audioSinkListeners, s);

	return 0;
}


int pdraw_audio_sink_set_media_id(const struct pdraw *pdraw,
				  struct pdraw_audio_sink *sink,
				  unsigned int media_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->setMediaId(media_id);
}


unsigned int pdraw_audio_sink_get_media_id(const struct pdraw *pdraw,
					   struct pdraw_audio_sink *sink)
{
	if (pdraw == nullptr)
		return 0;
	if (sink == nullptr)
		return 0;

	auto *s = sink->impl;

	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_audio_frame_queue *
pdraw_audio_sink_get_queue(const struct pdraw *pdraw,
			   struct pdraw_audio_sink *sink)
{
	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	auto *s = sink->impl;

	return s->getQueue();
}


int pdraw_audio_sink_queue_flushed(const struct pdraw *pdraw,
				   struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueFlushed();
}


int pdraw_audio_sink_queue_drained(const struct pdraw *pdraw,
				   struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = sink->impl;

	return s->queueDrained();
}


int pdraw_video_encoder_new(struct pdraw *pdraw,
			    unsigned int media_id,
			    const struct venc_config *params,
			    const struct pdraw_video_encoder_cbs *cbs,
			    void *userdata,
			    struct pdraw_video_encoder **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVideoEncoder *encoder = nullptr;
	std::unique_ptr<PdrawVideoEncoderListener> videoEncoderListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoEncoderListener =
			std::make_unique<PdrawVideoEncoderListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoEncoder(
		media_id, params, videoEncoderListener.get(), &encoder);
	if (res < 0)
		return res;

	videoEncoderListener->setImpl(encoder);
	*ret_obj = videoEncoderListener->getWrapper();
	pdraw->videoEncoderListeners.push_back(std::move(videoEncoderListener));

	return 0;
}


int pdraw_video_encoder_destroy(struct pdraw *pdraw,
				const struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	const auto *e = encoder->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->videoEncoderListeners, e);

	return 0;
}


int pdraw_video_encoder_configure(const struct pdraw *pdraw,
				  struct pdraw_video_encoder *encoder,
				  const struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->configure(config);
}


int pdraw_video_encoder_get_config(const struct pdraw *pdraw,
				   struct pdraw_video_encoder *encoder,
				   struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->getConfig(config);
}


int pdraw_video_encoder_request_key_frame(const struct pdraw *pdraw,
					  struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = encoder->impl;

	return e->requestKeyFrame();
}


int pdraw_video_scaler_new(struct pdraw *pdraw,
			   unsigned int media_id,
			   const struct vscale_config *params,
			   const struct pdraw_video_scaler_cbs *cbs,
			   void *userdata,
			   struct pdraw_video_scaler **ret_obj)
{
	int res;
	Pdraw::IPdraw::IVideoScaler *encoder = nullptr;
	std::unique_ptr<PdrawVideoScalerListener> videoScalerListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoScalerListener =
			std::make_unique<PdrawVideoScalerListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video scaler listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoScaler(
		media_id, params, videoScalerListener.get(), &encoder);
	if (res < 0)
		return res;

	videoScalerListener->setImpl(encoder);
	*ret_obj = videoScalerListener->getWrapper();
	pdraw->videoScalerListeners.push_back(std::move(videoScalerListener));

	return 0;
}


int pdraw_video_scaler_destroy(struct pdraw *pdraw,
			       const struct pdraw_video_scaler *scaler)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(scaler == nullptr, EINVAL);

	const auto *s = scaler->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->videoScalerListeners, s);

	return 0;
}


int pdraw_audio_encoder_new(struct pdraw *pdraw,
			    unsigned int media_id,
			    const struct aenc_config *params,
			    const struct pdraw_audio_encoder_cbs *cbs,
			    void *userdata,
			    struct pdraw_audio_encoder **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioEncoder *encoder = nullptr;
	std::unique_ptr<PdrawAudioEncoderListener> audioEncoderListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		audioEncoderListener =
			std::make_unique<PdrawAudioEncoderListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioEncoder(
		media_id, params, audioEncoderListener.get(), &encoder);
	if (res < 0)
		return res;

	audioEncoderListener->setImpl(encoder);
	*ret_obj = audioEncoderListener->getWrapper();
	pdraw->audioEncoderListeners.push_back(std::move(audioEncoderListener));

	return 0;
}


int pdraw_audio_encoder_destroy(struct pdraw *pdraw,
				const struct pdraw_audio_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	const auto *e = encoder->impl;

	deleteImplAndEraseListenerUnlocked(
		pdraw, pdraw->audioEncoderListeners, e);

	return 0;
}


int pdraw_get_friendly_name_setting(struct pdraw *pdraw, char *str, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string fn;
	pdraw->pdraw->getFriendlyNameSetting(&fn);
	if (str && (fn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, fn.c_str());
	return 0;
}


int pdraw_set_friendly_name_setting(struct pdraw *pdraw,
				    const char *friendly_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string fn(friendly_name);
	pdraw->pdraw->setFriendlyNameSetting(fn);
	return 0;
}


int pdraw_get_serial_number_setting(struct pdraw *pdraw, char *str, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string sn;
	pdraw->pdraw->getSerialNumberSetting(&sn);
	if (str && (sn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sn.c_str());
	return 0;
}


int pdraw_set_serial_number_setting(struct pdraw *pdraw,
				    const char *serial_number)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string sn(serial_number);
	pdraw->pdraw->setSerialNumberSetting(sn);
	return 0;
}


int pdraw_get_software_version_setting(struct pdraw *pdraw,
				       char *str,
				       size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string sv;
	pdraw->pdraw->getSoftwareVersionSetting(&sv);
	if (str && (sv.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sv.c_str());
	return 0;
}


int pdraw_set_software_version_setting(struct pdraw *pdraw,
				       const char *software_version)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string sv(software_version);
	pdraw->pdraw->setSoftwareVersionSetting(sv);
	return 0;
}


int pdraw_dump_pipeline(struct pdraw *pdraw, const char *file_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(file_name == nullptr, EINVAL);

	std::string f(file_name);
	return pdraw->pdraw->dumpPipeline(f);
}


const char *
pdraw_demuxer_autodecoding_mode_str(enum pdraw_demuxer_autodecoding_mode val)
{
	return pdraw_demuxerAutodecodingModeStr(val);
}


enum pdraw_demuxer_autodecoding_mode
pdraw_demuxer_autodecoding_mode_from_str(const char *val)
{
	return pdraw_demuxerAutodecodingModeFromStr(val);
}


const char *pdraw_playback_type_str(enum pdraw_playback_type val)
{
	return pdraw_playbackTypeStr(val);
}


enum pdraw_playback_type pdraw_playback_type_from_str(const char *val)
{
	return pdraw_playbackTypeFromStr(val);
}


const char *pdraw_media_type_str(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


const char *
pdraw_muxer_connection_state_str(enum pdraw_muxer_connection_state val)
{
	return pdraw_muxerConnectionStateStr(val);
}


const char *
pdraw_muxer_disconnection_reason_str(enum pdraw_muxer_disconnection_reason val)
{
	return pdraw_muxerDisconnectionReasonStr(val);
}


const char *pdraw_muxer_rtsp_transport_str(enum pdraw_muxer_rtsp_transport val)
{
	return pdraw_muxerRtpTransportStr(val);
}


enum pdraw_muxer_rtsp_transport
pdraw_muxer_rtsp_transport_from_str(const char *val)
{
	return pdraw_muxerRtpTransportFromStr(val);
}


enum pdraw_media_type pdraw_media_type_from_str(const char *val)
{
	return pdraw_mediaTypeFromStr(val);
}


const char *pdraw_histogram_channel_str(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
}


enum pdraw_histogram_channel pdraw_histogram_channel_from_str(const char *val)
{
	return pdraw_histogramChannelFromStr(val);
}


const char *pdraw_video_renderer_scheduling_mode_str(
	enum pdraw_video_renderer_scheduling_mode val)
{
	return pdraw_videoRendererSchedulingModeStr(val);
}


enum pdraw_video_renderer_scheduling_mode
pdraw_video_renderer_scheduling_mode_from_str(const char *val)
{
	return pdraw_videoRendererSchedulingModeFromStr(val);
}


const char *
pdraw_video_renderer_fill_mode_str(enum pdraw_video_renderer_fill_mode val)
{
	return pdraw_videoRendererFillModeStr(val);
}


enum pdraw_video_renderer_fill_mode
pdraw_video_renderer_fill_mode_from_str(const char *val)
{
	return pdraw_videoRendererFillModeFromStr(val);
}


const char *pdraw_video_renderer_transition_flag_str(
	enum pdraw_video_renderer_transition_flag val)
{
	return pdraw_videoRendererTransitionFlagStr(val);
}


enum pdraw_video_renderer_transition_flag
pdraw_video_renderer_transition_flag_from_str(const char *val)
{
	return pdraw_videoRendererTransitionFlagFromStr(val);
}


const char *
pdraw_vipc_source_eos_reason_str(enum pdraw_vipc_source_eos_reason val)
{
	return pdraw_vipcSourceEosReasonStr(val);
}


enum pdraw_vipc_source_eos_reason
pdraw_vipc_source_eos_reason_from_str(const char *val)
{
	return pdraw_vipcSourceEosReasonFromStr(val);
}


int pdraw_video_frame_to_json_str(const struct pdraw_video_frame *frame,
				  struct vmeta_frame *metadata,
				  char *str,
				  unsigned int len)
{
	return pdraw_frameMetadataToJsonStr(frame, metadata, str, len);
}


int pdraw_video_frame_to_json(const struct pdraw_video_frame *frame,
			      struct vmeta_frame *metadata,
			      struct json_object *jobj)
{
	return pdraw_frameMetadataToJson(frame, metadata, jobj);
}


struct pdraw_media_info *
pdraw_media_info_dup(const struct pdraw_media_info *src)
{
	return pdraw_mediaInfoDup(src);
}


void pdraw_media_info_free(struct pdraw_media_info *media_info)
{
	return pdraw_mediaInfoFree(media_info);
}


struct pdraw_vipc_source_params *
pdraw_vipc_source_params_dup(const struct pdraw_vipc_source_params *src)
{
	return pdraw_vipcSourceParamsDup(src);
}


void pdraw_vipc_source_params_free(struct pdraw_vipc_source_params *params)
{
	pdraw_vipcSourceParamsFree(params);
}


struct pdraw_muxer_params *
pdraw_muxer_params_dup(const struct pdraw_muxer_params *src)
{
	return pdraw_muxerParamsDup(src);
}


void pdraw_muxer_params_free(struct pdraw_muxer_params *params)
{
	pdraw_muxerParamsFree(params);
}


struct pdraw_muxer_media_params *
pdraw_muxer_media_params_dup(const struct pdraw_muxer_media_params *src)
{
	return pdraw_muxerMediaParamsDup(src);
}


void pdraw_muxer_media_params_free(struct pdraw_muxer_media_params *params)
{
	pdraw_muxerMediaParamsFree(params);
}


void pdraw_demuxer_media_list_free(struct pdraw_demuxer_media *media_list,
				   size_t media_count)
{
	pdraw_demuxerMediaListFree(media_list, media_count);
}


int pdraw_alsa_source_get_capabilities(const char *address,
				       struct pdraw_alsa_source_caps *caps)
{
	ULOG_ERRNO_RETURN_ERR_IF(address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(caps == nullptr, EINVAL);

#ifdef PDRAW_USE_ALSA
	return Pdraw::AlsaSource::getCapabilities(std::string(address), caps);
#else
	return -ENOSYS;
#endif
}
