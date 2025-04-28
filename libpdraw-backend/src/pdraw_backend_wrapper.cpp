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

	~PdrawBackendListener() {}

	void stopResponse(Pdraw::IPdraw *pdraw, int status)
	{
		if (mCbs.stop_resp) {
			(*mCbs.stop_resp)(mPdraw, status, mUserdata);
		}
	}

	void onMediaAdded(Pdraw::IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData)
	{
		if (mCbs.media_added) {
			(*mCbs.media_added)(
				mPdraw, info, elementUserData, mUserdata);
		}
	}

	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData)
	{
		if (mCbs.media_removed) {
			(*mCbs.media_removed)(
				mPdraw, info, elementUserData, mUserdata);
		}
	}

	void onSocketCreated(Pdraw::IPdraw *pdraw, int fd)
	{
		if (mCbs.socket_created)
			(*mCbs.socket_created)(mPdraw, fd, mUserdata);
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_cbs mCbs;
	void *mUserdata;
};


class PdrawBackendDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener {
public:
	PdrawBackendDemuxerListener(struct pdraw_backend *pdraw,
				    const struct pdraw_backend_demuxer_cbs *cbs,
				    void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mDemuxer(nullptr)
	{
	}

	~PdrawBackendDemuxerListener() {}

	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status)
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
				  int status)
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

	void onDemuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
					 Pdraw::IPdraw::IDemuxer *demuxer)
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
			       uint32_t selectedMedias)
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
				bool ready)
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
				 uint64_t timestamp)
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
				 float speed)
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
				  uint64_t timestamp)
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
				 float speed)
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

	Pdraw::IPdraw::IDemuxer *getDemuxer()
	{
		return mDemuxer;
	}

	void setDemuxer(Pdraw::IPdraw::IDemuxer *demuxer)
	{
		mDemuxer = demuxer;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_demuxer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IDemuxer *mDemuxer;
};


class PdrawBackendMuxerListener : public Pdraw::IPdraw::IMuxer::Listener {
public:
	PdrawBackendMuxerListener(struct pdraw_backend *pdraw,
				  const struct pdraw_backend_muxer_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mMuxer(nullptr)
	{
	}

	~PdrawBackendMuxerListener() {}

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

	Pdraw::IPdraw::IMuxer *getMuxer()
	{
		return mMuxer;
	}

	void setMuxer(Pdraw::IPdraw::IMuxer *muxer)
	{
		mMuxer = muxer;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_muxer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IMuxer *mMuxer;
};


class PdrawBackendVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener {
public:
	PdrawBackendVideoRendererListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_renderer_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(nullptr)
	{
	}

	~PdrawBackendVideoRendererListener() {}

	void onVideoRendererMediaAdded(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IVideoRenderer *renderer,
				       const struct pdraw_media_info *info)
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
				    bool restart)
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

	void onVideoRenderReady(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IVideoRenderer *renderer)
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
			     size_t frameUserdataLen)
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

	int renderVideoOverlay(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IVideoRenderer *renderer,
			       const struct pdraw_rect *renderPos,
			       const struct pdraw_rect *contentPos,
			       const float *viewMat,
			       const float *projMat,
			       const struct pdraw_media_info *mediaInfo,
			       struct vmeta_frame *frameMeta,
			       const struct pdraw_video_frame_extra *frameExtra)
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

	Pdraw::IPdraw::IVideoRenderer *getVideoRenderer()
	{
		return mRenderer;
	}

	void setVideoRenderer(Pdraw::IPdraw::IVideoRenderer *renderer)
	{
		mRenderer = renderer;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_renderer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoRenderer *mRenderer;
};


class PdrawBackendAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener {
public:
	PdrawBackendAudioRendererListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_renderer_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(nullptr)
	{
	}

	~PdrawBackendAudioRendererListener() {}

	void onAudioRendererMediaAdded(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IAudioRenderer *renderer,
				       const struct pdraw_media_info *info)
	{
		if (mCbs.media_added)
			(*mCbs.media_added)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_renderer *>(
					renderer),
				info,
				mUserdata);
	}

	void
	onAudioRendererMediaRemoved(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IAudioRenderer *renderer,
				    const struct pdraw_media_info *info)
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_renderer *>(
					renderer),
				info,
				mUserdata);
	}

	Pdraw::IPdraw::IAudioRenderer *getAudioRenderer()
	{
		return mRenderer;
	}

	void setAudioRenderer(Pdraw::IPdraw::IAudioRenderer *renderer)
	{
		mRenderer = renderer;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_audio_renderer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioRenderer *mRenderer;
};


class PdrawBackendVipcSourceListener
		: public Pdraw::IPdraw::IVipcSource::Listener {
public:
	PdrawBackendVipcSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_vipc_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawBackendVipcSourceListener() {}

	void vipcSourceReadyToPlay(IPdraw *pdraw,
				   IPdraw::IVipcSource *source,
				   bool ready,
				   enum pdraw_vipc_source_eos_reason eosReason)
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

	bool vipcSourceFramerateChanged(Pdraw::IPdraw *pdraw,
					Pdraw::IPdraw::IVipcSource *source,
					const struct vdef_frac *prevFramerate,
					const struct vdef_frac *newFramerate)
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
				  const struct vdef_rectf *crop)
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
				  struct mbuf_raw_video_frame *frame)
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				frame,
				mUserdata);
	}

	bool vipcSourceEndOfStream(IPdraw *pdraw,
				   IPdraw::IVipcSource *source,
				   enum pdraw_vipc_source_eos_reason eosReason)
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

	Pdraw::IPdraw::IVipcSource *getVipcSource()
	{
		return mSource;
	}

	void setVipcSource(Pdraw::IPdraw::IVipcSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_vipc_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVipcSource *mSource;
};


class PdrawBackendCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener {
public:
	PdrawBackendCodedVideoSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_coded_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawBackendCodedVideoSourceListener() {}

	void onCodedVideoSourceFlushed(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::ICodedVideoSource *source)
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_coded_video_source
							 *>(source),
				mUserdata);
	}

	Pdraw::IPdraw::ICodedVideoSource *getCodedVideoSource()
	{
		return mSource;
	}

	void setCodedVideoSource(Pdraw::IPdraw::ICodedVideoSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_coded_video_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::ICodedVideoSource *mSource;
};


class PdrawBackendRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener {
public:
	PdrawBackendRawVideoSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_raw_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawBackendRawVideoSourceListener() {}

	void onRawVideoSourceFlushed(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IRawVideoSource *source)
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_source
							 *>(source),
				mUserdata);
	}

	Pdraw::IPdraw::IRawVideoSource *getRawVideoSource()
	{
		return mSource;
	}

	void setRawVideoSource(Pdraw::IPdraw::IRawVideoSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_raw_video_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IRawVideoSource *mSource;
};


class PdrawBackendCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener {
public:
	PdrawBackendCodedVideoSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_coded_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawBackendCodedVideoSinkListener() {}

	void onCodedVideoSinkFlush(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::ICodedVideoSink *sink)
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				mUserdata);
	}

	void onCodedVideoSinkSessionMetaUpdate(IPdraw *pdraw,
					       IPdraw::ICodedVideoSink *sink,
					       const struct vmeta_session *meta)
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw,
				reinterpret_cast<
					struct pdraw_coded_video_sink *>(sink),
				meta,
				mUserdata);
	}

	Pdraw::IPdraw::ICodedVideoSink *getCodedVideoSink()
	{
		return mSink;
	}

	void setCodedVideoSink(Pdraw::IPdraw::ICodedVideoSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_coded_video_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::ICodedVideoSink *mSink;
};


class PdrawBackendRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener {
public:
	PdrawBackendRawVideoSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_raw_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawBackendRawVideoSinkListener() {}

	void onRawVideoSinkFlush(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IRawVideoSink *sink)
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				mUserdata);
	}

	void onRawVideoSinkSessionMetaUpdate(IPdraw *pdraw,
					     IPdraw::IRawVideoSink *sink,
					     const struct vmeta_session *meta)
	{
		if (mCbs.session_metadata_update)
			(*mCbs.session_metadata_update)(
				mPdraw,
				reinterpret_cast<struct pdraw_raw_video_sink *>(
					sink),
				meta,
				mUserdata);
	}

	Pdraw::IPdraw::IRawVideoSink *getRawVideoSink()
	{
		return mSink;
	}

	void setRawVideoSink(Pdraw::IPdraw::IRawVideoSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_raw_video_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IRawVideoSink *mSink;
};


class PdrawBackendAlsaSourceListener
		: public Pdraw::IPdraw::IAlsaSource::Listener {
public:
	PdrawBackendAlsaSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_alsa_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawBackendAlsaSourceListener() {}

	void alsaSourceReadyToPlay(IPdraw *pdraw,
				   IPdraw::IAlsaSource *source,
				   bool ready,
				   enum pdraw_alsa_source_eos_reason eosReason)
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

	void alsaSourceFrameReady(IPdraw *pdraw,
				  IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame)
	{
		if (mCbs.frame_ready)
			(*mCbs.frame_ready)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IAlsaSource *getAlsaSource()
	{
		return mSource;
	}

	void setAlsaSource(Pdraw::IPdraw::IAlsaSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_alsa_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAlsaSource *mSource;
};


class PdrawBackendAudioSourceListener
		: public Pdraw::IPdraw::IAudioSource::Listener {
public:
	PdrawBackendAudioSourceListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawBackendAudioSourceListener() {}

	void onAudioSourceFlushed(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAudioSource *source)
	{
		if (mCbs.flushed)
			(*mCbs.flushed)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_source *>(
					source),
				mUserdata);
	}

	Pdraw::IPdraw::IAudioSource *getAudioSource()
	{
		return mSource;
	}

	void setAudioSource(Pdraw::IPdraw::IAudioSource *source)
	{
		mSource = source;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_audio_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioSource *mSource;
};


class PdrawBackendAudioSinkListener
		: public Pdraw::IPdraw::IAudioSink::Listener {
public:
	PdrawBackendAudioSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawBackendAudioSinkListener() {}

	void onAudioSinkFlush(Pdraw::IPdraw *pdraw,
			      Pdraw::IPdraw::IAudioSink *sink)
	{
		if (mCbs.flush)
			(*mCbs.flush)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_sink *>(
					sink),
				mUserdata);
	}

	Pdraw::IPdraw::IAudioSink *getAudioSink()
	{
		return mSink;
	}

	void setAudioSink(Pdraw::IPdraw::IAudioSink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_audio_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioSink *mSink;
};


class PdrawBackendVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener {
public:
	PdrawBackendVideoEncoderListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_encoder_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mEncoder(nullptr)
	{
	}

	~PdrawBackendVideoEncoderListener() {}

	void videoEncoderFrameOutput(IPdraw *pdraw,
				     IPdraw::IVideoEncoder *encoder,
				     struct mbuf_coded_video_frame *frame)
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	void videoEncoderFramePreRelease(IPdraw *pdraw,
					 IPdraw::IVideoEncoder *encoder,
					 struct mbuf_coded_video_frame *frame)
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IVideoEncoder *getVideoEncoder()
	{
		return mEncoder;
	}

	void setVideoEncoder(Pdraw::IPdraw::IVideoEncoder *encoder)
	{
		mEncoder = encoder;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_encoder_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoEncoder *mEncoder;
};


class PdrawBackendVideoScalerListener
		: public Pdraw::IPdraw::IVideoScaler::Listener {
public:
	PdrawBackendVideoScalerListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_scaler_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mScaler(nullptr)
	{
	}

	~PdrawBackendVideoScalerListener() {}

	void videoScalerFrameOutput(IPdraw *pdraw,
				    IPdraw::IVideoScaler *scaler,
				    struct mbuf_raw_video_frame *frame)
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_scaler *>(
					scaler),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IVideoScaler *getVideoScaler()
	{
		return mScaler;
	}

	void setVideoScaler(Pdraw::IPdraw::IVideoScaler *scaler)
	{
		mScaler = scaler;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_scaler_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoScaler *mScaler;
};


class PdrawBackendAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener {
public:
	PdrawBackendAudioEncoderListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_audio_encoder_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mEncoder(nullptr)
	{
	}

	~PdrawBackendAudioEncoderListener() {}

	void audioEncoderFrameOutput(IPdraw *pdraw,
				     IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame)
	{
		if (mCbs.frame_output)
			(*mCbs.frame_output)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	void audioEncoderFramePreRelease(IPdraw *pdraw,
					 IPdraw::IAudioEncoder *encoder,
					 struct mbuf_audio_frame *frame)
	{
		if (mCbs.frame_pre_release)
			(*mCbs.frame_pre_release)(
				mPdraw,
				reinterpret_cast<struct pdraw_audio_encoder *>(
					encoder),
				frame,
				mUserdata);
	}

	Pdraw::IPdraw::IAudioEncoder *getAudioEncoder()
	{
		return mEncoder;
	}

	void setAudioEncoder(Pdraw::IPdraw::IAudioEncoder *encoder)
	{
		mEncoder = encoder;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_audio_encoder_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioEncoder *mEncoder;
};


struct pdraw_backend {
	PdrawBackend::IPdrawBackend *pdraw;
	PdrawBackendListener *listener;
	std::vector<PdrawBackendDemuxerListener *> *demuxerListeners;
	std::vector<PdrawBackendMuxerListener *> *muxerListeners;
	std::vector<PdrawBackendVideoRendererListener *>
		*videoRendererListeners;
	std::vector<PdrawBackendAudioRendererListener *>
		*audioRendererListeners;
	std::vector<PdrawBackendVipcSourceListener *> *vipcSourceListeners;
	std::vector<PdrawBackendCodedVideoSourceListener *>
		*codedVideoSourceListeners;
	std::vector<PdrawBackendRawVideoSourceListener *>
		*rawVideoSourceListeners;
	std::vector<PdrawBackendCodedVideoSinkListener *>
		*codedVideoSinkListeners;
	std::vector<PdrawBackendRawVideoSinkListener *> *rawVideoSinkListeners;
	std::vector<PdrawBackendAlsaSourceListener *> *alsaSourceListeners;
	std::vector<PdrawBackendAudioSourceListener *> *audioSourceListeners;
	std::vector<PdrawBackendAudioSinkListener *> *audioSinkListeners;
	std::vector<PdrawBackendVideoEncoderListener *> *videoEncoderListeners;
	std::vector<PdrawBackendVideoScalerListener *> *videoScalerListeners;
	std::vector<PdrawBackendAudioEncoderListener *> *audioEncoderListeners;
};


int pdraw_be_new(const struct pdraw_backend_cbs *cbs,
		 void *userdata,
		 struct pdraw_backend **ret_obj)
{
	int res = 0;
	struct pdraw_backend *self;

	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	self = (struct pdraw_backend *)calloc(1, sizeof(*self));
	if (self == nullptr)
		return -ENOMEM;

	self->demuxerListeners =
		new std::vector<PdrawBackendDemuxerListener *>();
	if (self->demuxerListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->muxerListeners = new std::vector<PdrawBackendMuxerListener *>();
	if (self->muxerListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->videoRendererListeners =
		new std::vector<PdrawBackendVideoRendererListener *>();
	if (self->videoRendererListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->audioRendererListeners =
		new std::vector<PdrawBackendAudioRendererListener *>();
	if (self->audioRendererListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->vipcSourceListeners =
		new std::vector<PdrawBackendVipcSourceListener *>();
	if (self->vipcSourceListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->codedVideoSourceListeners =
		new std::vector<PdrawBackendCodedVideoSourceListener *>();
	if (self->codedVideoSourceListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->rawVideoSourceListeners =
		new std::vector<PdrawBackendRawVideoSourceListener *>();
	if (self->rawVideoSourceListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->codedVideoSinkListeners =
		new std::vector<PdrawBackendCodedVideoSinkListener *>();
	if (self->codedVideoSinkListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->rawVideoSinkListeners =
		new std::vector<PdrawBackendRawVideoSinkListener *>();
	if (self->rawVideoSinkListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->alsaSourceListeners =
		new std::vector<PdrawBackendAlsaSourceListener *>();
	if (self->alsaSourceListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->audioSourceListeners =
		new std::vector<PdrawBackendAudioSourceListener *>();
	if (self->audioSourceListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->audioSinkListeners =
		new std::vector<PdrawBackendAudioSinkListener *>();
	if (self->audioSinkListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->videoEncoderListeners =
		new std::vector<PdrawBackendVideoEncoderListener *>();
	if (self->videoEncoderListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->videoScalerListeners =
		new std::vector<PdrawBackendVideoScalerListener *>();
	if (self->videoScalerListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->audioEncoderListeners =
		new std::vector<PdrawBackendAudioEncoderListener *>();
	if (self->audioEncoderListeners == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	self->listener = new PdrawBackendListener(self, cbs, userdata);
	if (self->listener == nullptr) {
		res = -ENOMEM;
		goto error;
	}

	res = createPdrawBackend(self->listener, &self->pdraw);
	if (res < 0)
		goto error;

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
		delete self->pdraw;
		self->pdraw = nullptr;
	}

	if (self->listener != nullptr)
		delete self->listener;

	if (self->vipcSourceListeners != nullptr) {
		std::vector<PdrawBackendVipcSourceListener *>::iterator l =
			self->vipcSourceListeners->begin();
		while (l != self->vipcSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->vipcSourceListeners->clear();
		delete self->vipcSourceListeners;
	}

	if (self->codedVideoSourceListeners != nullptr) {
		std::vector<PdrawBackendCodedVideoSourceListener *>::iterator
			l = self->codedVideoSourceListeners->begin();
		while (l != self->codedVideoSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->codedVideoSourceListeners->clear();
		delete self->codedVideoSourceListeners;
	}

	if (self->rawVideoSourceListeners != nullptr) {
		std::vector<PdrawBackendRawVideoSourceListener *>::iterator l =
			self->rawVideoSourceListeners->begin();
		while (l != self->rawVideoSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->rawVideoSourceListeners->clear();
		delete self->rawVideoSourceListeners;
	}

	if (self->codedVideoSinkListeners != nullptr) {
		std::vector<PdrawBackendCodedVideoSinkListener *>::iterator l =
			self->codedVideoSinkListeners->begin();
		while (l != self->codedVideoSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->codedVideoSinkListeners->clear();
		delete self->codedVideoSinkListeners;
	}

	if (self->rawVideoSinkListeners != nullptr) {
		std::vector<PdrawBackendRawVideoSinkListener *>::iterator l =
			self->rawVideoSinkListeners->begin();
		while (l != self->rawVideoSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->rawVideoSinkListeners->clear();
		delete self->rawVideoSinkListeners;
	}

	if (self->alsaSourceListeners != nullptr) {
		std::vector<PdrawBackendAlsaSourceListener *>::iterator l =
			self->alsaSourceListeners->begin();
		while (l != self->alsaSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->alsaSourceListeners->clear();
		delete self->alsaSourceListeners;
	}

	if (self->audioSourceListeners != nullptr) {
		std::vector<PdrawBackendAudioSourceListener *>::iterator l =
			self->audioSourceListeners->begin();
		while (l != self->audioSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->audioSourceListeners->clear();
		delete self->audioSourceListeners;
	}

	if (self->audioSinkListeners != nullptr) {
		std::vector<PdrawBackendAudioSinkListener *>::iterator l =
			self->audioSinkListeners->begin();
		while (l != self->audioSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->audioSinkListeners->clear();
		delete self->audioSinkListeners;
	}

	if (self->videoEncoderListeners != nullptr) {
		std::vector<PdrawBackendVideoEncoderListener *>::iterator l =
			self->videoEncoderListeners->begin();
		while (l != self->videoEncoderListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->videoEncoderListeners->clear();
		delete self->videoEncoderListeners;
	}

	if (self->videoScalerListeners != nullptr) {
		std::vector<PdrawBackendVideoScalerListener *>::iterator l =
			self->videoScalerListeners->begin();
		while (l != self->videoScalerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->videoScalerListeners->clear();
		delete self->videoScalerListeners;
	}

	if (self->audioEncoderListeners != nullptr) {
		std::vector<PdrawBackendAudioEncoderListener *>::iterator l =
			self->audioEncoderListeners->begin();
		while (l != self->audioEncoderListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->audioEncoderListeners->clear();
		delete self->audioEncoderListeners;
	}

	if (self->audioRendererListeners != nullptr) {
		std::vector<PdrawBackendAudioRendererListener *>::iterator r =
			self->audioRendererListeners->begin();
		while (r != self->audioRendererListeners->end()) {
			if (*r != nullptr)
				delete (*r);
			r++;
		}
		self->audioRendererListeners->clear();
		delete self->audioRendererListeners;
	}

	if (self->videoRendererListeners != nullptr) {
		std::vector<PdrawBackendVideoRendererListener *>::iterator r =
			self->videoRendererListeners->begin();
		while (r != self->videoRendererListeners->end()) {
			if (*r != nullptr)
				delete (*r);
			r++;
		}
		self->videoRendererListeners->clear();
		delete self->videoRendererListeners;
	}

	if (self->muxerListeners != nullptr) {
		std::vector<PdrawBackendMuxerListener *>::iterator l =
			self->muxerListeners->begin();
		while (l != self->muxerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->muxerListeners->clear();
		delete self->muxerListeners;
	}

	if (self->demuxerListeners != nullptr) {
		std::vector<PdrawBackendDemuxerListener *>::iterator l =
			self->demuxerListeners->begin();
		while (l != self->demuxerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		self->demuxerListeners->clear();
		delete self->demuxerListeners;
	}

	free(self);
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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendDemuxerListener *l =
		new PdrawBackendDemuxerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, params, l, &demuxer);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setDemuxer(demuxer);
	self->demuxerListeners->push_back(l);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendDemuxerListener *l =
		new PdrawBackendDemuxerListener(self, cbs, userdata);
	if (l == nullptr) {
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
					 l,
					 &demuxer);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setDemuxer(demuxer);
	self->demuxerListeners->push_back(l);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendDemuxerListener *l =
		new PdrawBackendDemuxerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, mux, params, l, &demuxer);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setDemuxer(demuxer);
	self->demuxerListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
	return 0;
}


int pdraw_be_demuxer_destroy(struct pdraw_backend *self,
			     struct pdraw_demuxer *demuxer)
{
	std::vector<PdrawBackendDemuxerListener *>::iterator l;
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete d;

	l = self->demuxerListeners->begin();
	while (l != self->demuxerListeners->end()) {
		if ((*l)->getDemuxer() != d) {
			l++;
			continue;
		}
		delete *l;
		self->demuxerListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_demuxer_close(struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

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
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_be_demuxer_select_media(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint32_t selected_medias)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->selectMedia(selected_medias);
}


uint16_t pdraw_be_demuxer_get_single_stream_local_stream_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_be_demuxer_get_single_stream_local_control_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getSingleStreamLocalControlPort();
}


int pdraw_be_demuxer_play(struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play();
}


int pdraw_be_demuxer_play_with_speed(struct pdraw_backend *self,
				     struct pdraw_demuxer *demuxer,
				     float speed)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play(speed);
}


int pdraw_be_demuxer_is_ready_to_play(struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_be_demuxer_pause(struct pdraw_backend *self,
			   struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->pause();
}


int pdraw_be_demuxer_is_paused(struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_be_demuxer_previous_frame(struct pdraw_backend *self,
				    struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->previousFrame();
}


int pdraw_be_demuxer_next_frame(struct pdraw_backend *self,
				struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->nextFrame();
}


int pdraw_be_demuxer_seek(struct pdraw_backend *self,
			  struct pdraw_demuxer *demuxer,
			  int64_t delta,
			  int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seek(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_forward(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer,
				  uint64_t delta,
				  int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_back(struct pdraw_backend *self,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_be_demuxer_seek_to(struct pdraw_backend *self,
			     struct pdraw_demuxer *demuxer,
			     uint64_t timestamp,
			     int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_be_demuxer_get_chapter_list(struct pdraw_backend *self,
				      struct pdraw_demuxer *demuxer,
				      struct pdraw_chapter **chapter_list,
				      size_t *chapter_count)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_be_demuxer_get_duration(struct pdraw_backend *self,
				       struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(demuxer == nullptr, EINVAL, 0);

	return d->getDuration();
}


uint64_t pdraw_be_demuxer_get_current_time(struct pdraw_backend *self,
					   struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendMuxerListener *l =
		new PdrawBackendMuxerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");

	res = self->pdraw->createMuxer(u, params, l, &muxer);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setMuxer(muxer);
	self->muxerListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
}


int pdraw_be_muxer_destroy(struct pdraw_backend *self,
			   struct pdraw_muxer *muxer)
{
	std::vector<PdrawBackendMuxerListener *>::iterator l;
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete m;

	l = self->muxerListeners->begin();
	while (l != self->muxerListeners->end()) {
		if ((*l)->getMuxer() != m) {
			l++;
			continue;
		}
		delete *l;
		self->muxerListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_muxer_close(struct pdraw_backend *self, struct pdraw_muxer *muxer)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->close();
}


int pdraw_be_muxer_add_media(struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     unsigned int media_id,
			     const struct pdraw_muxer_media_params *params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

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
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->setThumbnail(type, data, size);
}


int pdraw_be_muxer_add_chapter(struct pdraw_backend *self,
			       struct pdraw_muxer *muxer,
			       uint64_t timestamp,
			       const char *name)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addChapter(timestamp, name);
}


int pdraw_be_muxer_get_stats(struct pdraw_backend *self,
			     struct pdraw_muxer *muxer,
			     struct pdraw_muxer_stats *stats)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendVideoRendererListener *l =
		new PdrawBackendVideoRendererListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video renderer listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoRenderer(
		media_id, render_pos, params, l, &renderer);
	if (res < 0) {
		delete l;
		return res;
	}

	self->videoRendererListeners->push_back(l);
	l->setVideoRenderer(renderer);

	*ret_obj = reinterpret_cast<struct pdraw_video_renderer *>(renderer);
	return 0;
}


int pdraw_be_video_renderer_destroy(struct pdraw_backend *self,
				    struct pdraw_video_renderer *renderer)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete rnd;

	std::vector<PdrawBackendVideoRendererListener *>::iterator l =
		self->videoRendererListeners->begin();
	while (l != self->videoRendererListeners->end()) {
		if ((*l)->getVideoRenderer() != rnd) {
			l++;
			continue;
		}
		delete *l;
		self->videoRendererListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_video_renderer_resize(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   const struct pdraw_rect *render_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->resize(render_pos);
}


int pdraw_be_video_renderer_set_media_id(struct pdraw_backend *self,
					 struct pdraw_video_renderer *renderer,
					 unsigned int media_id)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_video_renderer_get_media_id(struct pdraw_backend *self,
				     struct pdraw_video_renderer *renderer)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	return rnd->getMediaId();
}


int pdraw_be_video_renderer_set_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_be_video_renderer_get_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	struct pdraw_video_renderer_params *params)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->getParams(params);
}


int pdraw_be_video_renderer_render(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   struct pdraw_rect *content_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

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
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendAudioRendererListener *l =
		new PdrawBackendAudioRendererListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create audio renderer listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioRenderer(media_id, params, l, &renderer);
	if (res < 0) {
		delete l;
		return res;
	}

	self->audioRendererListeners->push_back(l);
	l->setAudioRenderer(renderer);

	*ret_obj = reinterpret_cast<struct pdraw_audio_renderer *>(renderer);
	return 0;
}


int pdraw_be_audio_renderer_destroy(struct pdraw_backend *self,
				    struct pdraw_audio_renderer *renderer)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete rnd;

	std::vector<PdrawBackendAudioRendererListener *>::iterator l =
		self->audioRendererListeners->begin();
	while (l != self->audioRendererListeners->end()) {
		if ((*l)->getAudioRenderer() != rnd) {
			l++;
			continue;
		}
		delete *l;
		self->audioRendererListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_audio_renderer_set_media_id(struct pdraw_backend *self,
					 struct pdraw_audio_renderer *renderer,
					 unsigned int media_id)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_be_audio_renderer_get_media_id(struct pdraw_backend *self,
				     struct pdraw_audio_renderer *renderer)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(renderer == nullptr, EINVAL, 0);

	return rnd->getMediaId();
}


int pdraw_be_audio_renderer_set_params(
	struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	const struct pdraw_audio_renderer_params *params)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_be_audio_renderer_get_params(
	struct pdraw_backend *self,
	struct pdraw_audio_renderer *renderer,
	struct pdraw_audio_renderer_params *params)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendVipcSourceListener *l =
		new PdrawBackendVipcSourceListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create VIPC source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVipcSource(params, l, &source);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setVipcSource(source);
	self->vipcSourceListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_vipc_source *>(source);
	return 0;
}


int pdraw_be_vipc_source_destroy(struct pdraw_backend *self,
				 struct pdraw_vipc_source *source)
{
	std::vector<PdrawBackendVipcSourceListener *>::iterator l;
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->vipcSourceListeners->begin();
	while (l != self->vipcSourceListeners->end()) {
		if ((*l)->getVipcSource() != s) {
			l++;
			continue;
		}
		delete *l;
		self->vipcSourceListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_vipc_source_is_ready_to_play(struct pdraw_backend *self,
					  struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_vipc_source_is_paused(struct pdraw_backend *self,
				   struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_vipc_source_play(struct pdraw_backend *self,
			      struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_be_vipc_source_pause(struct pdraw_backend *self,
			       struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->pause();
}


int pdraw_be_vipc_source_configure(struct pdraw_backend *self,
				   struct pdraw_vipc_source *source,
				   const struct vdef_dim *resolution,
				   const struct vdef_rectf *crop)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->configure(resolution, crop);
}


int pdraw_be_vipc_source_insert_grey_frame(struct pdraw_backend *self,
					   struct pdraw_vipc_source *source,
					   uint64_t ts_us)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->insertGreyFrame(ts_us);
}


int pdraw_be_vipc_source_set_session_metadata(struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      const struct vmeta_session *meta)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_vipc_source_get_session_metadata(struct pdraw_backend *self,
					      struct pdraw_vipc_source *source,
					      struct vmeta_session *meta)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendCodedVideoSourceListener *l =
		new PdrawBackendCodedVideoSourceListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createCodedVideoSource(params, l, &source);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setCodedVideoSource(source);
	self->codedVideoSourceListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_source *>(source);
	return 0;
}


int pdraw_be_coded_video_source_destroy(struct pdraw_backend *self,
					struct pdraw_coded_video_source *source)
{
	std::vector<PdrawBackendCodedVideoSourceListener *>::iterator l;
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->codedVideoSourceListeners->begin();
	while (l != self->codedVideoSourceListeners->end()) {
		if ((*l)->getCodedVideoSource() != s) {
			l++;
			continue;
		}
		delete *l;
		self->codedVideoSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_source_get_queue(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_coded_video_source_flush(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_coded_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_coded_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendRawVideoSourceListener *l =
		new PdrawBackendRawVideoSourceListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createRawVideoSource(params, l, &source);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setRawVideoSource(source);
	self->rawVideoSourceListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_source *>(source);
	return 0;
}


int pdraw_be_raw_video_source_destroy(struct pdraw_backend *self,
				      struct pdraw_raw_video_source *source)
{
	std::vector<PdrawBackendRawVideoSourceListener *>::iterator l;
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->rawVideoSourceListeners->begin();
	while (l != self->rawVideoSourceListeners->end()) {
		if ((*l)->getRawVideoSource() != s) {
			l++;
			continue;
		}
		delete *l;
		self->rawVideoSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_source_get_queue(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_raw_video_source_flush(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_raw_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_be_raw_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendCodedVideoSinkListener *l =
		new PdrawBackendCodedVideoSinkListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createCodedVideoSink(media_id, params, l, &sink);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setCodedVideoSink(sink);
	self->codedVideoSinkListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_sink *>(sink);
	return 0;
}


int pdraw_be_coded_video_sink_destroy(struct pdraw_backend *self,
				      struct pdraw_coded_video_sink *sink)
{
	std::vector<PdrawBackendCodedVideoSinkListener *>::iterator l;
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->codedVideoSinkListeners->begin();
	while (l != self->codedVideoSinkListeners->end()) {
		if ((*l)->getCodedVideoSink() != s) {
			l++;
			continue;
		}
		delete *l;
		self->codedVideoSinkListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_coded_video_sink_resync(struct pdraw_backend *self,
				     struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->resync();
}


struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_sink_get_queue(struct pdraw_backend *self,
				    struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_coded_video_sink_queue_flushed(struct pdraw_backend *self,
					    struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendRawVideoSinkListener *l =
		new PdrawBackendRawVideoSinkListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createRawVideoSink(media_id, params, l, &sink);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setRawVideoSink(sink);
	self->rawVideoSinkListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_sink *>(sink);
	return 0;
}


int pdraw_be_raw_video_sink_destroy(struct pdraw_backend *self,
				    struct pdraw_raw_video_sink *sink)
{
	std::vector<PdrawBackendRawVideoSinkListener *>::iterator l;
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->rawVideoSinkListeners->begin();
	while (l != self->rawVideoSinkListeners->end()) {
		if ((*l)->getRawVideoSink() != s) {
			l++;
			continue;
		}
		delete *l;
		self->rawVideoSinkListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_sink_get_queue(struct pdraw_backend *self,
				  struct pdraw_raw_video_sink *sink)
{
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_raw_video_sink_queue_flushed(struct pdraw_backend *self,
					  struct pdraw_raw_video_sink *sink)
{
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_be_alsa_source_new(struct pdraw_backend *self,
			     const struct pdraw_alsa_source_params *params,
			     const struct pdraw_backend_alsa_source_cbs *cbs,
			     void *userdata,
			     struct pdraw_alsa_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAlsaSource *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendAlsaSourceListener *l =
		new PdrawBackendAlsaSourceListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create ALSA source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAlsaSource(params, l, &source);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setAlsaSource(source);
	self->alsaSourceListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_alsa_source *>(source);
	return 0;
}


int pdraw_be_alsa_source_destroy(struct pdraw_backend *self,
				 struct pdraw_alsa_source *source)
{
	std::vector<PdrawBackendAlsaSourceListener *>::iterator l;
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->alsaSourceListeners->begin();
	while (l != self->alsaSourceListeners->end()) {
		if ((*l)->getAlsaSource() != s) {
			l++;
			continue;
		}
		delete *l;
		self->alsaSourceListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_alsa_source_is_ready_to_play(struct pdraw_backend *self,
					  struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_be_alsa_source_is_paused(struct pdraw_backend *self,
				   struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, 0);

	return s->isPaused() ? 1 : 0;
}


int pdraw_be_alsa_source_play(struct pdraw_backend *self,
			      struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_be_alsa_source_pause(struct pdraw_backend *self,
			       struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flushed == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendAudioSourceListener *l =
		new PdrawBackendAudioSourceListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioSource(params, l, &source);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setAudioSource(source);
	self->audioSourceListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_audio_source *>(source);
	return 0;
}


int pdraw_be_audio_source_destroy(struct pdraw_backend *self,
				  struct pdraw_audio_source *source)
{
	std::vector<PdrawBackendAudioSourceListener *>::iterator l;
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->audioSourceListeners->begin();
	while (l != self->audioSourceListeners->end()) {
		if ((*l)->getAudioSource() != s) {
			l++;
			continue;
		}
		delete *l;
		self->audioSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_source_get_queue(struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(source == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_audio_source_flush(struct pdraw_backend *self,
				struct pdraw_audio_source *source)
{
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_be_audio_sink_new(struct pdraw_backend *self,
			    unsigned int media_id,
			    const struct pdraw_backend_audio_sink_cbs *cbs,
			    void *userdata,
			    struct pdraw_audio_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSink *sink = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendAudioSinkListener *l =
		new PdrawBackendAudioSinkListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioSink(media_id, l, &sink);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setAudioSink(sink);
	self->audioSinkListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_audio_sink *>(sink);
	return 0;
}


int pdraw_be_audio_sink_destroy(struct pdraw_backend *self,
				struct pdraw_audio_sink *sink)
{
	std::vector<PdrawBackendAudioSinkListener *>::iterator l;
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = self->audioSinkListeners->begin();
	while (l != self->audioSinkListeners->end()) {
		if ((*l)->getAudioSink() != s) {
			l++;
			continue;
		}
		delete *l;
		self->audioSinkListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_be_audio_sink_get_queue(struct pdraw_backend *self,
			      struct pdraw_audio_sink *sink)
{
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(sink == nullptr, EINVAL, nullptr);

	return s->getQueue();
}


int pdraw_be_audio_sink_queue_flushed(struct pdraw_backend *self,
				      struct pdraw_audio_sink *sink)
{
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendVideoEncoderListener *l =
		new PdrawBackendVideoEncoderListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoEncoder(media_id, params, l, &encoder);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setVideoEncoder(encoder);
	self->videoEncoderListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_video_encoder *>(encoder);
	return 0;
}


int pdraw_be_video_encoder_destroy(struct pdraw_backend *self,
				   struct pdraw_video_encoder *encoder)
{
	std::vector<PdrawBackendVideoEncoderListener *>::iterator l;
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = self->videoEncoderListeners->begin();
	while (l != self->videoEncoderListeners->end()) {
		if ((*l)->getVideoEncoder() != e) {
			l++;
			continue;
		}
		delete *l;
		self->videoEncoderListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_video_encoder_configure(struct pdraw_backend *self,
				     struct pdraw_video_encoder *encoder,
				     const struct venc_dyn_config *config)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->configure(config);
}


int pdraw_be_video_encoder_get_config(struct pdraw_backend *self,
				      struct pdraw_video_encoder *encoder,
				      struct venc_dyn_config *config)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->getConfig(config);
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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendVideoScalerListener *l =
		new PdrawBackendVideoScalerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video scaler listener");
		return -ENOMEM;
	}

	res = self->pdraw->createVideoScaler(media_id, params, l, &scaler);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setVideoScaler(scaler);
	self->videoScalerListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_video_scaler *>(scaler);
	return 0;
}


int pdraw_be_video_scaler_destroy(struct pdraw_backend *self,
				  struct pdraw_video_scaler *scaler)
{
	std::vector<PdrawBackendVideoScalerListener *>::iterator l;
	Pdraw::IPdraw::IVideoScaler *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(scaler);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(scaler == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = self->videoScalerListeners->begin();
	while (l != self->videoScalerListeners->end()) {
		if ((*l)->getVideoScaler() != e) {
			l++;
			continue;
		}
		delete *l;
		self->videoScalerListeners->erase(l);
		break;
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

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendAudioEncoderListener *l =
		new PdrawBackendAudioEncoderListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create audio encoder listener");
		return -ENOMEM;
	}

	res = self->pdraw->createAudioEncoder(media_id, params, l, &encoder);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setAudioEncoder(encoder);
	self->audioEncoderListeners->push_back(l);

	*ret_obj = reinterpret_cast<struct pdraw_audio_encoder *>(encoder);
	return 0;
}


int pdraw_be_audio_encoder_destroy(struct pdraw_backend *self,
				   struct pdraw_audio_encoder *encoder)
{
	std::vector<PdrawBackendAudioEncoderListener *>::iterator l;
	Pdraw::IPdraw::IAudioEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = self->audioEncoderListeners->begin();
	while (l != self->audioEncoderListeners->end()) {
		if ((*l)->getAudioEncoder() != e) {
			l++;
			continue;
		}
		delete *l;
		self->audioEncoderListeners->erase(l);
		break;
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
	if ((str) && (fn.length() >= len))
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
	if ((str) && (sn.length() >= len))
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
	if ((str) && (sv.length() >= len))
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
