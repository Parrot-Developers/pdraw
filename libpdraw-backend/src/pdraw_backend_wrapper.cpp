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

#include <memory>
#include <string>
#include <vector>

#define ULOG_TAG pdraw_backend
#include <ulog.h>

#include <pdraw/pdraw_backend.h>

#include "pdraw_backend_impl.hpp"


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
				mPdraw, info, elementUserData, mUserdata);
		}
	}

	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override
	{
		if (mCbs.media_removed) {
			(*mCbs.media_removed)(
				mPdraw, info, elementUserData, mUserdata);
		}
	}

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


class PdrawBackendDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener {
public:
	PdrawBackendDemuxerListener(struct pdraw_backend *pdraw,
				    const struct pdraw_backend_demuxer_cbs *cbs,
				    void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendDemuxerListener() override = default;

	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status) override
	{
		if (mCbs.open_resp) {
			(*mCbs.open_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				status,
				mUserdata);
		}
	}

	void demuxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status) override
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				status,
				mUserdata);
		}
	}

	void
	onDemuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IDemuxer *demuxer) override
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				mUserdata);
		}
	}

	int demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override
	{
		if (mCbs.select_media) {
			return (*mCbs.select_media)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
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
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				ready ? 1 : 0,
				mUserdata);
		}
	}

	void onDemuxerEndOfRange(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override
	{
		if (mCbs.end_of_range) {
			(*mCbs.end_of_range)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				timestamp,
				mUserdata);
		}
	}

	void demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
		if (mCbs.play_resp) {
			(*mCbs.play_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
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
			(*mCbs.pause_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
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
			(*mCbs.seek_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				status,
				timestamp,
				speed,
				mUserdata);
		}
	}

	Pdraw::IPdraw::IDemuxer *getDemuxer() const
	{
		return mDemuxer;
	}

	void setDemuxer(Pdraw::IPdraw::IDemuxer *demuxer)
	{
		mDemuxer = demuxer;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_demuxer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IDemuxer *mDemuxer = nullptr;
};


class PdrawBackendMuxerListener : public Pdraw::IPdraw::IMuxer::Listener {
public:
	PdrawBackendMuxerListener(struct pdraw_backend *pdraw,
				  const struct pdraw_backend_muxer_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendMuxerListener() override = default;

	void onMuxerConnectionStateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state connectionState,
		enum pdraw_muxer_disconnection_reason disconnectionReason)
		override
	{
		if (mCbs.connection_state_changed) {
			(*mCbs.connection_state_changed)(
				mPdraw,
				reinterpret_cast<struct pdraw_muxer *>(muxer),
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
			(*mCbs.media_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_muxer *>(muxer),
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
				mPdraw,
				reinterpret_cast<struct pdraw_muxer *>(muxer),
				mediaPath,
				mUserdata);
		}
	}

	void onMuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IMuxer *muxer,
				       int status) override
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(
				mPdraw,
				reinterpret_cast<struct pdraw_muxer *>(muxer),
				status,
				mUserdata);
		}
	}

	void muxerCloseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IMuxer *muxer,
				int status) override
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_muxer *>(muxer),
				status,
				mUserdata);
		}
	}

	Pdraw::IPdraw::IMuxer *getMuxer() const
	{
		return mMuxer;
	}

	void setMuxer(Pdraw::IPdraw::IMuxer *muxer)
	{
		mMuxer = muxer;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_muxer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IMuxer *mMuxer = nullptr;
};


class PdrawBackendVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener {
public:
	PdrawBackendVideoRendererListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_renderer_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendVideoRendererListener() override = default;

	void
	onVideoRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVideoRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_renderer *>(
					renderer),
				info,
				mUserdata);
	}

	void
	onVideoRendererMediaRemoved(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoRenderer *renderer,
				    const struct pdraw_media_info *info,
				    bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_renderer *>(
					renderer),
				info,
				restart ? 1 : 0,
				mUserdata);
	}

	void
	onVideoRenderReady(Pdraw::IPdraw *pdraw,
			   Pdraw::IPdraw::IVideoRenderer *renderer) override
	{
		if (mCbs.render_ready)
			(*mCbs.render_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_renderer *>(
					renderer),
				mUserdata);
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
		return (*mCbs.load_texture)(
			mPdraw,
			reinterpret_cast<struct pdraw_video_renderer *>(
				renderer),
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
		if ((renderer == nullptr) || (renderPos == nullptr) ||
		    (contentPos == nullptr) || (viewMat == nullptr) ||
		    (projMat == nullptr))
			return -EINVAL;
		(*mCbs.render_overlay)(
			mPdraw,
			reinterpret_cast<struct pdraw_video_renderer *>(
				renderer),
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

	Pdraw::IPdraw::IVideoRenderer *getVideoRenderer() const
	{
		return mRenderer;
	}

	void setVideoRenderer(Pdraw::IPdraw::IVideoRenderer *renderer)
	{
		mRenderer = renderer;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_video_renderer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoRenderer *mRenderer = nullptr;
};


class PdrawBackendAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener {
public:
	PdrawBackendAudioRendererListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_renderer_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendAudioRendererListener() override = default;

	void
	onAudioRendererMediaAdded(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioRenderer *renderer,
				  const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_renderer *>(
					renderer),
				info,
				mUserdata);
	}

	void onAudioRendererMediaRemoved(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_renderer *>(
					renderer),
				info,
				mUserdata);
	}

	Pdraw::IPdraw::IAudioRenderer *getAudioRenderer() const
	{
		return mRenderer;
	}

	void setAudioRenderer(Pdraw::IPdraw::IAudioRenderer *renderer)
	{
		mRenderer = renderer;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_audio_renderer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioRenderer *mRenderer = nullptr;
};


class PdrawBackendVipcSourceListener
		: public Pdraw::IPdraw::IVipcSource::Listener {
public:
	PdrawBackendVipcSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_vipc_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendVipcSourceListener() override = default;

	void vipcSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		bool ready,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				(int)ready,
				eosReason,
				mUserdata);
	}

	void vipcSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IVipcSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				mUserdata);
	}

	void vipcSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IVipcSource *source) override
	{
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				mUserdata);
	}

	bool vipcSourceFramerateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
		const struct vdef_frac *prevFramerate,
		const struct vdef_frac *newFramerate) override
	{
		if (mCbs.framerate_changed)
			return (*mCbs.framerate_changed)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
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
		if (mCbs.configured)
			(*mCbs.configured)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				status,
				info,
				crop,
				mUserdata);
	}

	void vipcSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IVipcSource *source,
				  struct mbuf_raw_video_frame *frame) override
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				frame,
				mUserdata);
	}

	bool vipcSourceEndOfStream(
		IPdraw *pdraw,
		IPdraw::IVipcSource *source,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		if (mCbs.end_of_stream)
			return (*mCbs.end_of_stream)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				eosReason,
				mUserdata);
		return false;
	}

	Pdraw::IPdraw::IVipcSource *getVipcSource() const
	{
		return mSource;
	}

	void setVipcSource(Pdraw::IPdraw::IVipcSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_vipc_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVipcSource *mSource = nullptr;
};


class PdrawBackendCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener {
public:
	PdrawBackendCodedVideoSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_coded_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendCodedVideoSourceListener() override = default;

	void onCodedVideoSourceFlushed(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_coded_video_source
							 *>(source),
				mUserdata);
	}

	void onCodedVideoSourceDrained(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(
				mPdraw,
				reinterpret_cast<struct pdraw_coded_video_source
							 *>(source),
				mUserdata);
	}

	Pdraw::IPdraw::ICodedVideoSource *getCodedVideoSource() const
	{
		return mSource;
	}

	void setCodedVideoSource(Pdraw::IPdraw::ICodedVideoSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_coded_video_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::ICodedVideoSource *mSource = nullptr;
};


class PdrawBackendRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener {
public:
	PdrawBackendRawVideoSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_raw_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendRawVideoSourceListener() override = default;

	void
	onRawVideoSourceFlushed(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_source
							 *>(source),
				mUserdata);
	}

	void
	onRawVideoSourceDrained(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_source
							 *>(source),
				mUserdata);
	}

	Pdraw::IPdraw::IRawVideoSource *getRawVideoSource() const
	{
		return mSource;
	}

	void setRawVideoSource(Pdraw::IPdraw::IRawVideoSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_raw_video_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IRawVideoSource *mSource = nullptr;
};


class PdrawBackendCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener {
public:
	PdrawBackendCodedVideoSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_coded_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendCodedVideoSinkListener() override = default;

	void
	onCodedVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::ICodedVideoSink *sink,
				   const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				info,
				mUserdata);
	}

	void onCodedVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
					  const struct pdraw_media_info *info,
					  bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				info,
				restart,
				mUserdata);
	}

	void
	onCodedVideoSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				mUserdata);
	}

	void
	onCodedVideoSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::ICodedVideoSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				mUserdata);
	}

	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::ICodedVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				meta,
				mUserdata);
	}

	Pdraw::IPdraw::ICodedVideoSink *getCodedVideoSink() const
	{
		return mSink;
	}

	void setCodedVideoSink(Pdraw::IPdraw::ICodedVideoSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_coded_video_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::ICodedVideoSink *mSink = nullptr;
};


class PdrawBackendRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener {
public:
	PdrawBackendRawVideoSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_raw_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendRawVideoSinkListener() override = default;

	void
	onRawVideoSinkMediaAdded(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink,
				 const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				info,
				mUserdata);
	}

	void onRawVideoSinkMediaRemoved(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IRawVideoSink *sink,
					const struct pdraw_media_info *info,
					bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				info,
				restart,
				mUserdata);
	}

	void onRawVideoSinkFlush(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				mUserdata);
	}

	void onRawVideoSinkDrain(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				mUserdata);
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw *pdraw,
		IPdraw::IRawVideoSink *sink,
		const struct vmeta_session *meta) override
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				meta,
				mUserdata);
	}

	Pdraw::IPdraw::IRawVideoSink *getRawVideoSink() const
	{
		return mSink;
	}

	void setRawVideoSink(Pdraw::IPdraw::IRawVideoSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_raw_video_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IRawVideoSink *mSink = nullptr;
};


class PdrawBackendAlsaSourceListener
		: public Pdraw::IPdraw::IAlsaSource::Listener {
public:
	PdrawBackendAlsaSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_alsa_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendAlsaSourceListener() override = default;

	void alsaSourceReadyToPlay(
		IPdraw *pdraw,
		IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override
	{
		if (mCbs.ready_to_play)
			(*mCbs.ready_to_play)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				(int)ready,
				eosReason,
				mUserdata);
	}

	void alsaSourcePlayResponse(IPdraw *pdraw,
				    IPdraw::IAlsaSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				mUserdata);
	}

	void alsaSourcePauseResponse(IPdraw *pdraw,
				     IPdraw::IAlsaSource *source) override
	{
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				mUserdata);
	}

	void alsaSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IAlsaSource *getAlsaSource() const
	{
		return mSource;
	}

	void setAlsaSource(Pdraw::IPdraw::IAlsaSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_alsa_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAlsaSource *mSource = nullptr;
};


class PdrawBackendAudioSourceListener
		: public Pdraw::IPdraw::IAudioSource::Listener {
public:
	PdrawBackendAudioSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendAudioSourceListener() override = default;

	void onAudioSourceFlushed(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_source *>(
					source),
				mUserdata);
	}

	void onAudioSourceDrained(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source) override
	{
		if (mCbs.drained)
			(*mCbs.drained)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_source *>(
					source),
				mUserdata);
	}

	Pdraw::IPdraw::IAudioSource *getAudioSource() const
	{
		return mSource;
	}

	void setAudioSource(Pdraw::IPdraw::IAudioSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_audio_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioSource *mSource = nullptr;
};


class PdrawBackendAudioSinkListener
		: public Pdraw::IPdraw::IAudioSink::Listener {
public:
	PdrawBackendAudioSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendAudioSinkListener() override = default;

	void onAudioSinkMediaAdded(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IAudioSink *sink,
				   const struct pdraw_media_info *info) override
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_sink *>(
					sink),
				info,
				mUserdata);
	}

	void onAudioSinkMediaRemoved(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioSink *sink,
				     const struct pdraw_media_info *info,
				     bool restart) override
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_sink *>(
					sink),
				info,
				restart,
				mUserdata);
	}

	void onAudioSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_sink *>(
					sink),
				mUserdata);
	}

	void onAudioSinkDrain(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink) override
	{
		if (mCbs.drain)
			(*mCbs.drain)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_sink *>(
					sink),
				mUserdata);
	}

	Pdraw::IPdraw::IAudioSink *getAudioSink() const
	{
		return mSink;
	}

	void setAudioSink(Pdraw::IPdraw::IAudioSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_audio_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioSink *mSink = nullptr;
};


class PdrawBackendVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener {
public:
	PdrawBackendVideoEncoderListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_encoder_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendVideoEncoderListener() override = default;

	void
	videoEncoderFrameOutput(IPdraw *pdraw,
				IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	void videoEncoderFramePreRelease(
		IPdraw *pdraw,
		IPdraw::IVideoEncoder *encoder,
		struct mbuf_coded_video_frame *frame) override
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IVideoEncoder *getVideoEncoder() const
	{
		return mEncoder;
	}

	void setVideoEncoder(Pdraw::IPdraw::IVideoEncoder *encoder)
	{
		mEncoder = encoder;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_video_encoder_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoEncoder *mEncoder = nullptr;
};


class PdrawBackendVideoScalerListener
		: public Pdraw::IPdraw::IVideoScaler::Listener {
public:
	PdrawBackendVideoScalerListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_scaler_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendVideoScalerListener() override = default;

	void videoScalerFrameOutput(IPdraw *pdraw,
				    IPdraw::IVideoScaler *scaler,
				    struct mbuf_raw_video_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_scaler *>(
					scaler),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IVideoScaler *getVideoScaler() const
	{
		return mScaler;
	}

	void setVideoScaler(Pdraw::IPdraw::IVideoScaler *scaler)
	{
		mScaler = scaler;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_video_scaler_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoScaler *mScaler = nullptr;
};


class PdrawBackendAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener {
public:
	PdrawBackendAudioEncoderListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_encoder_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendAudioEncoderListener() override = default;

	void audioEncoderFrameOutput(IPdraw *pdraw,
				     IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	void
	audioEncoderFramePreRelease(IPdraw *pdraw,
				    IPdraw::IAudioEncoder *encoder,
				    struct mbuf_audio_frame *frame) override
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IAudioEncoder *getAudioEncoder() const
	{
		return mEncoder;
	}

	void setAudioEncoder(Pdraw::IPdraw::IAudioEncoder *encoder)
	{
		mEncoder = encoder;
	}

private:
	struct pdraw_backend *mPdraw = nullptr;
	struct pdraw_backend_audio_encoder_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioEncoder *mEncoder = nullptr;
};


struct pdraw_backend {
	std::unique_ptr<PdrawBackend::IPdrawBackend> pdraw{};
	std::unique_ptr<PdrawBackendListener> listener{};
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
};


int pdraw_be_new(const struct pdraw_backend_cbs *cbs,
		 void *userdata,
		 struct pdraw_backend **ret_obj)
{
	int res = 0;
	struct pdraw_backend *self;
	PdrawBackend::IPdrawBackend *pdraw = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	self = new struct pdraw_backend();
	if (self == nullptr)
		return -ENOMEM;

	try {
		self->listener =
			make_unique<PdrawBackendListener>(self, cbs, userdata);
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

	*ret_obj = self;
	return 0;

error:
	(void)pdraw_be_destroy(self);
	*ret_obj = nullptr;
	return res;
}


int pdraw_be_destroy(struct pdraw_backend *self)
{
	if (self == nullptr)
		return 0;

	if (self->pdraw != nullptr) {
		self->pdraw->stop();
		self->pdraw.reset();
	}

	self->listener.reset();

	delete self;
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
		l = make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, params, l.get(), &demuxer);
	if (res < 0)
		return res;

	l->setDemuxer(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
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
		l = make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string local(local_addr ? local_addr : "");
	std::string remote(remote_addr ? remote_addr : "");
	res = self->pdraw->createDemuxer(local,
					 local_stream_port,
					 local_control_port,
					 remote,
					 remote_stream_port,
					 remote_control_port,
					 params,
					 l.get(),
					 &demuxer);
	if (res < 0)
		return res;

	l->setDemuxer(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
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
		l = make_unique<PdrawBackendDemuxerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, mux, params, l.get(), &demuxer);
	if (res < 0)
		return res;

	l->setDemuxer(demuxer);
	self->demuxerListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
	return 0;
}


int pdraw_be_demuxer_destroy(struct pdraw_backend *self,
			     struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	/* The object must be destroyed before the listener */
	delete d;

	auto &listeners = self->demuxerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getDemuxer() == d) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_demuxer_close(struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->close();
}


int pdraw_be_demuxer_get_media_list(struct pdraw_backend *self,
				    struct pdraw_demuxer *demuxer,
				    struct pdraw_demuxer_media **media_list,
				    size_t *media_count,
				    uint32_t *selected_medias)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_be_demuxer_select_media(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint32_t selected_medias)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->selectMedia(selected_medias);
}


uint16_t pdraw_be_demuxer_get_single_stream_local_stream_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_be_demuxer_get_single_stream_local_control_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getSingleStreamLocalControlPort();
}


int pdraw_be_demuxer_play(struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play();
}


int pdraw_be_demuxer_play_with_speed(struct pdraw_backend *self,
				     struct pdraw_demuxer *demuxer,
				     float speed)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play(speed);
}


int pdraw_be_demuxer_is_ready_to_play(struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_be_demuxer_pause(struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->pause();
}


int pdraw_be_demuxer_is_paused(struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_be_demuxer_previous_frame(struct pdraw_backend *self,
				    struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->previousFrame();
}


int pdraw_be_demuxer_next_frame(struct pdraw_backend *self,
				struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->nextFrame();
}


int pdraw_be_demuxer_seek(struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer,
			  int64_t delta,
			  int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seek(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_forward(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint64_t delta,
				  int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_back(struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_to(struct pdraw_backend *self,
			     struct pdraw_demuxer *demuxer,
			     uint64_t timestamp,
			     int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_be_demuxer_get_chapter_list(struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer,
				      struct pdraw_chapter **chapter_list,
				      size_t *chapter_count)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_be_demuxer_get_duration(struct pdraw_backend *self,
				       struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getDuration();
}


uint64_t pdraw_be_demuxer_get_current_time(struct pdraw_backend *self,
					   struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getCurrentTime();
}


int pdraw_be_muxer_new(struct pdraw_backend *self,
		       const char *url,
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
		l = make_unique<PdrawBackendMuxerListener>(self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");

	res = self->pdraw->createMuxer(u, params, l.get(), &muxer);
	if (res < 0)
		return res;

	l->setMuxer(muxer);
	self->muxerListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
}


int pdraw_be_muxer_destroy(struct pdraw_backend *self,
			   struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	/* The object must be destroyed before the listener */
	delete m;

	auto &listeners = self->muxerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getMuxer() == m) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_muxer_close(struct pdraw_backend *self, struct pdraw_muxer *muxer)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->close();
}


int pdraw_be_muxer_add_media(struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     unsigned int media_id,
			     const struct pdraw_muxer_media_params *params)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addMedia(media_id, params);
}


int pdraw_be_muxer_set_thumbnail(struct pdraw_backend *self,
				 struct pdraw_muxer *muxer,
				 enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->setThumbnail(type, data, size);
}


int pdraw_muxer_set_file_metadata(struct pdraw_backend *pdraw,
				  struct pdraw_muxer *muxer,
				  enum pdraw_muxer_metadata_type type,
				  const uint8_t *data,
				  size_t size,
				  const void *params,
				  size_t params_size)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->setFileMetadata(type, data, size, params, params_size);
}


int pdraw_be_muxer_add_chapter(struct pdraw_backend *self,
			       struct pdraw_muxer *muxer,
			       uint64_t timestamp,
			       const char *name)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addChapter(timestamp, name);
}


int pdraw_be_muxer_get_stats(struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     struct pdraw_muxer_stats *stats)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendVideoRendererListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video renderer listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoRenderer(
		media_id, render_pos, params, l.get(), &renderer);
	if (res < 0)
		return res;

	l->setVideoRenderer(renderer);
	self->videoRendererListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_video_renderer *>(renderer);
	return 0;
}


int pdraw_be_video_renderer_destroy(struct pdraw_backend *self,
				    struct pdraw_video_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	/* The object must be destroyed before the listener */
	delete rnd;

	auto &listeners = self->videoRendererListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVideoRenderer() == rnd) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_video_renderer_resize(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   const struct pdraw_rect *render_pos)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->resize(render_pos);
}


int pdraw_be_video_renderer_set_media_id(struct pdraw_backend *self,
					 struct pdraw_video_renderer *renderer,
					 unsigned int media_id)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_video_renderer_get_media_id(struct pdraw_backend *self,
				     struct pdraw_video_renderer *renderer)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	return rnd->getMediaId();
}


int pdraw_be_video_renderer_set_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_be_video_renderer_get_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	struct pdraw_video_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->getParams(params);
}


int pdraw_be_video_renderer_render(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   struct pdraw_rect *content_pos)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->render(content_pos, nullptr, nullptr);
}


int pdraw_be_video_renderer_render_mat(struct pdraw_backend *self,
				       struct pdraw_video_renderer *renderer,
				       struct pdraw_rect *content_pos,
				       const float *view_mat,
				       const float *proj_mat)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendAudioRendererListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio renderer listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioRenderer(
		media_id, params, l.get(), &renderer);
	if (res < 0)
		return res;

	l->setAudioRenderer(renderer);
	self->audioRendererListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_audio_renderer *>(renderer);
	return 0;
}


int pdraw_be_audio_renderer_destroy(struct pdraw_backend *self,
				    struct pdraw_audio_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	/* The object must be destroyed before the listener */
	delete rnd;

	auto &listeners = self->audioRendererListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioRenderer() == rnd) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_audio_renderer_set_media_id(struct pdraw_backend *self,
					 struct pdraw_audio_renderer *renderer,
					 unsigned int media_id)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_audio_renderer_get_media_id(struct pdraw_backend *self,
				     struct pdraw_audio_renderer *renderer)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	return rnd->getMediaId();
}


int pdraw_be_audio_renderer_set_params(
	struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	const struct pdraw_audio_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_be_audio_renderer_get_params(
	struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	struct pdraw_audio_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendVipcSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create VIPC source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVipcSource(params, l.get(), &source);
	if (res < 0)
		return res;

	l->setVipcSource(source);
	self->vipcSourceListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_vipc_source *>(source);
	return 0;
}


int pdraw_be_vipc_source_destroy(struct pdraw_backend *self,
				 struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->vipcSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVipcSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_vipc_source_is_ready_to_play(struct pdraw_backend *self,
					  struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_vipc_source_is_paused(struct pdraw_backend *self,
				   struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_vipc_source_play(struct pdraw_backend *self,
			      struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_be_vipc_source_pause(struct pdraw_backend *self,
			       struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->pause();
}


int pdraw_be_vipc_source_configure(struct pdraw_backend *self,
				   struct pdraw_vipc_source *source,
				   const struct vdef_dim *resolution,
				   const struct vdef_rectf *crop)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->configure(resolution, crop);
}


int pdraw_be_vipc_source_insert_grey_frame(struct pdraw_backend *self,
					   struct pdraw_vipc_source *source,
					   uint64_t ts_us)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->insertGreyFrame(ts_us);
}


int pdraw_be_vipc_source_set_session_metadata(struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_vipc_source_get_session_metadata(struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendCodedVideoSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createCodedVideoSource(params, l.get(), &source);
	if (res < 0)
		return res;

	l->setCodedVideoSource(source);
	self->codedVideoSourceListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_source *>(source);
	return 0;
}


int pdraw_be_coded_video_source_destroy(struct pdraw_backend *self,
					struct pdraw_coded_video_source *source)
{

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->codedVideoSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getCodedVideoSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_source_get_queue(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_coded_video_source_flush(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_coded_video_source_drain(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->drain();
}


int pdraw_be_coded_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_coded_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendRawVideoSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createRawVideoSource(params, l.get(), &source);
	if (res < 0)
		return res;

	l->setRawVideoSource(source);
	self->rawVideoSourceListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_source *>(source);
	return 0;
}


int pdraw_be_raw_video_source_destroy(struct pdraw_backend *self,
				      struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->rawVideoSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getRawVideoSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_source_get_queue(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_raw_video_source_flush(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_raw_video_source_drain(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->drain();
}


int pdraw_be_raw_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_raw_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendCodedVideoSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createCodedVideoSink(
		media_id, params, l.get(), &sink);
	if (res < 0)
		return res;

	l->setCodedVideoSink(sink);
	self->codedVideoSinkListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_sink *>(sink);
	return 0;
}


int pdraw_be_coded_video_sink_destroy(struct pdraw_backend *self,
				      struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->codedVideoSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getCodedVideoSink() == s) {
			listeners.erase(it);
			break;
		}
	}
	return 0;
}


int pdraw_be_coded_video_sink_resync(struct pdraw_backend *self,
				     struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->resync();
}


int pdraw_be_coded_video_sink_set_media_id(struct pdraw_backend *self,
					   struct pdraw_coded_video_sink *sink,
					   unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int
pdraw_be_coded_video_sink_get_media_id(struct pdraw_backend *self,
				       struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	if (self == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_sink_get_queue(struct pdraw_backend *self,
				    struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_coded_video_sink_queue_flushed(struct pdraw_backend *self,
					    struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_be_coded_video_sink_queue_drained(struct pdraw_backend *self,
					    struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendRawVideoSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createRawVideoSink(media_id, params, l.get(), &sink);
	if (res < 0)
		return res;

	l->setRawVideoSink(sink);
	self->rawVideoSinkListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_sink *>(sink);
	return 0;
}


int pdraw_be_raw_video_sink_destroy(struct pdraw_backend *self,
				    struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->rawVideoSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getRawVideoSink() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_sink_get_queue(struct pdraw_backend *self,
				  struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_raw_video_sink_set_media_id(struct pdraw_backend *self,
					 struct pdraw_raw_video_sink *sink,
					 unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int
pdraw_be_raw_video_sink_get_media_id(struct pdraw_backend *self,
				     struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	if (self == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


int pdraw_be_raw_video_sink_queue_flushed(struct pdraw_backend *self,
					  struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_be_raw_video_sink_queue_drained(struct pdraw_backend *self,
					  struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendAlsaSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create ALSA source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAlsaSource(params, l.get(), &source);
	if (res < 0)
		return res;

	l->setAlsaSource(source);
	self->alsaSourceListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_alsa_source *>(source);
	return 0;
}


int pdraw_be_alsa_source_destroy(struct pdraw_backend *self,
				 struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->alsaSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAlsaSource() == s) {
			listeners.erase(it);
			break;
		}
	}


	return 0;
}


int pdraw_be_alsa_source_is_ready_to_play(struct pdraw_backend *self,
					  struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_alsa_source_is_paused(struct pdraw_backend *self,
				   struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_alsa_source_play(struct pdraw_backend *self,
			      struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_be_alsa_source_pause(struct pdraw_backend *self,
			       struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendAudioSourceListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioSource(params, l.get(), &source);
	if (res < 0)
		return res;

	l->setAudioSource(source);
	self->audioSourceListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_audio_source *>(source);
	return 0;
}


int pdraw_be_audio_source_destroy(struct pdraw_backend *self,
				  struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->audioSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_source_get_queue(struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_audio_source_flush(struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_audio_source_drain(struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendAudioSinkListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioSink(media_id, l.get(), &sink);
	if (res < 0)
		return res;

	l->setAudioSink(sink);
	self->audioSinkListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_audio_sink *>(sink);
	return 0;
}


int pdraw_be_audio_sink_destroy(struct pdraw_backend *self,
				struct pdraw_audio_sink *sink)
{

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->audioSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioSink() == s) {
			listeners.erase(it);
			break;
		}
	}
	return 0;
}


int pdraw_be_audio_sink_set_media_id(struct pdraw_backend *self,
				     struct pdraw_audio_sink *sink,
				     unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int pdraw_be_audio_sink_get_media_id(struct pdraw_backend *self,
					      struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	if (self == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_sink_get_queue(struct pdraw_backend *self,
			      struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_audio_sink_queue_flushed(struct pdraw_backend *self,
				      struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_be_audio_sink_queue_drained(struct pdraw_backend *self,
				      struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendVideoEncoderListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoEncoder(
		media_id, params, l.get(), &encoder);
	if (res < 0)
		return res;

	l->setVideoEncoder(encoder);
	self->videoEncoderListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_video_encoder *>(encoder);
	return 0;
}


int pdraw_be_video_encoder_destroy(struct pdraw_backend *self,
				   struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	/* The object must be destroyed before the listener */
	delete e;

	auto &listeners = self->videoEncoderListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVideoEncoder() == e) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_be_video_encoder_configure(struct pdraw_backend *self,
				     struct pdraw_video_encoder *encoder,
				     const struct venc_dyn_config *config)
{
	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->configure(config);
}


int pdraw_be_video_encoder_get_config(struct pdraw_backend *self,
				      struct pdraw_video_encoder *encoder,
				      struct venc_dyn_config *config)
{
	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->getConfig(config);
}


int pdraw_be_video_encoder_request_key_frame(
	struct pdraw_backend *self,
	struct pdraw_video_encoder *encoder)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

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
		l = make_unique<PdrawBackendVideoScalerListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video scaler listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoScaler(
		media_id, params, l.get(), &scaler);
	if (res < 0)
		return res;

	l->setVideoScaler(scaler);
	self->videoScalerListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_video_scaler *>(scaler);
	return 0;
}


int pdraw_be_video_scaler_destroy(struct pdraw_backend *self,
				  struct pdraw_video_scaler *scaler)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(scaler == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(scaler);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = self->videoScalerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVideoScaler() == s) {
			listeners.erase(it);
			break;
		}
	}

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
		l = make_unique<PdrawBackendAudioEncoderListener>(
			self, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio encoder listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioEncoder(
		media_id, params, l.get(), &encoder);
	if (res < 0)
		return res;

	l->setAudioEncoder(encoder);
	self->audioEncoderListeners.push_back(std::move(l));

	*ret_obj = reinterpret_cast<struct pdraw_audio_encoder *>(encoder);
	return 0;
}


int pdraw_be_audio_encoder_destroy(struct pdraw_backend *self,
				   struct pdraw_audio_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(encoder);

	/* The object must be destroyed before the listener */
	delete e;

	auto &listeners = self->audioEncoderListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioEncoder() == e) {
			listeners.erase(it);
			break;
		}
	}

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
