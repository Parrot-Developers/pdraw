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
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>
#include <pthread.h>

#include <string>

#include <pdraw/pdraw.h>


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

	~PdrawListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_cbs mCbs;
	void *mUserdata;
};


class PdrawDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener {
public:
	PdrawDemuxerListener(struct pdraw *pdraw,
			     const struct pdraw_demuxer_cbs *cbs,
			     void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mDemuxer(nullptr)
	{
	}

	~PdrawDemuxerListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_demuxer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IDemuxer *mDemuxer;
};


class PdrawMuxerListener : public Pdraw::IPdraw::IMuxer::Listener {
public:
	PdrawMuxerListener(struct pdraw *pdraw,
			   const struct pdraw_muxer_cbs *cbs,
			   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mMuxer(nullptr)
	{
	}

	~PdrawMuxerListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_muxer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IMuxer *mMuxer;
};


class PdrawVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener {
public:
	PdrawVideoRendererListener(struct pdraw *pdraw,
				   const struct pdraw_video_renderer_cbs *cbs,
				   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(nullptr)
	{
	}

	~PdrawVideoRendererListener() {}

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
		if (pdraw == nullptr || renderer == nullptr)
			return -EINVAL;
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
		if (pdraw == nullptr || renderer == nullptr)
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
	struct pdraw *mPdraw;
	struct pdraw_video_renderer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoRenderer *mRenderer;
};


class PdrawAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener {
public:
	PdrawAudioRendererListener(struct pdraw *pdraw,
				   const struct pdraw_audio_renderer_cbs *cbs,
				   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(nullptr)
	{
	}

	~PdrawAudioRendererListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_audio_renderer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioRenderer *mRenderer;
};


class PdrawVipcSourceListener : public Pdraw::IPdraw::IVipcSource::Listener {
public:
	PdrawVipcSourceListener(struct pdraw *pdraw,
				const struct pdraw_vipc_source_cbs *cbs,
				void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawVipcSourceListener() {}

	void vipcSourceReadyToPlay(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IVipcSource *source,
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

	void vipcSourceConfigured(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
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

	void vipcSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
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

	bool vipcSourceEndOfStream(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IVipcSource *source,
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
	struct pdraw *mPdraw;
	struct pdraw_vipc_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVipcSource *mSource;
};


class PdrawCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener {
public:
	PdrawCodedVideoSourceListener(
		struct pdraw *pdraw,
		const struct pdraw_coded_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawCodedVideoSourceListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_coded_video_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::ICodedVideoSource *mSource;
};


class PdrawRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener {
public:
	PdrawRawVideoSourceListener(
		struct pdraw *pdraw,
		const struct pdraw_raw_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawRawVideoSourceListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_raw_video_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IRawVideoSource *mSource;
};


class PdrawCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener {
public:
	PdrawCodedVideoSinkListener(
		struct pdraw *pdraw,
		const struct pdraw_coded_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawCodedVideoSinkListener() {}

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

	void
	onCodedVideoSinkSessionMetaUpdate(Pdraw::IPdraw *pdraw,
					  Pdraw::IPdraw::ICodedVideoSink *sink,
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
	struct pdraw *mPdraw;
	struct pdraw_coded_video_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::ICodedVideoSink *mSink;
};


class PdrawRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener {
public:
	PdrawRawVideoSinkListener(struct pdraw *pdraw,
				  const struct pdraw_raw_video_sink_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawRawVideoSinkListener() {}

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

	void onRawVideoSinkSessionMetaUpdate(Pdraw::IPdraw *pdraw,
					     Pdraw::IPdraw::IRawVideoSink *sink,
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
	struct pdraw *mPdraw;
	struct pdraw_raw_video_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IRawVideoSink *mSink;
};


class PdrawAlsaSourceListener : public Pdraw::IPdraw::IAlsaSource::Listener {
public:
	PdrawAlsaSourceListener(struct pdraw *pdraw,
				const struct pdraw_alsa_source_cbs *cbs,
				void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawAlsaSourceListener() {}

	void alsaSourceReadyToPlay(Pdraw::IPdraw *pdraw,
				   Pdraw::IPdraw::IAlsaSource *source,
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

	void alsaSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAlsaSource *source,
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
	struct pdraw *mPdraw;
	struct pdraw_alsa_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAlsaSource *mSource;
};


class PdrawAudioSourceListener : public Pdraw::IPdraw::IAudioSource::Listener {
public:
	PdrawAudioSourceListener(struct pdraw *pdraw,
				 const struct pdraw_audio_source_cbs *cbs,
				 void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSource(nullptr)
	{
	}

	~PdrawAudioSourceListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_audio_source_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioSource *mSource;
};


class PdrawAudioSinkListener : public Pdraw::IPdraw::IAudioSink::Listener {
public:
	PdrawAudioSinkListener(struct pdraw *pdraw,
			       const struct pdraw_audio_sink_cbs *cbs,
			       void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(nullptr)
	{
	}

	~PdrawAudioSinkListener() {}

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
	struct pdraw *mPdraw;
	struct pdraw_audio_sink_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioSink *mSink;
};


class PdrawVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener {
public:
	PdrawVideoEncoderListener(struct pdraw *pdraw,
				  const struct pdraw_video_encoder_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mEncoder(nullptr)
	{
	}

	~PdrawVideoEncoderListener() {}

	void videoEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IVideoEncoder *encoder,
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

	void videoEncoderFramePreRelease(Pdraw::IPdraw *pdraw,
					 Pdraw::IPdraw::IVideoEncoder *encoder,
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
	struct pdraw *mPdraw;
	struct pdraw_video_encoder_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoEncoder *mEncoder;
};


class PdrawVideoScalerListener : public Pdraw::IPdraw::IVideoScaler::Listener {
public:
	PdrawVideoScalerListener(struct pdraw *pdraw,
				 const struct pdraw_video_scaler_cbs *cbs,
				 void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mScaler(nullptr)
	{
	}

	~PdrawVideoScalerListener() {}

	void videoScalerFrameOutput(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoScaler *scaler,
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
	struct pdraw *mPdraw;
	struct pdraw_video_scaler_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoScaler *mScaler;
};


class PdrawAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener {
public:
	PdrawAudioEncoderListener(struct pdraw *pdraw,
				  const struct pdraw_audio_encoder_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mEncoder(nullptr)
	{
	}

	~PdrawAudioEncoderListener() {}

	void audioEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioEncoder *encoder,
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

	void audioEncoderFramePreRelease(Pdraw::IPdraw *pdraw,
					 Pdraw::IPdraw::IAudioEncoder *encoder,
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
	struct pdraw *mPdraw;
	struct pdraw_audio_encoder_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IAudioEncoder *mEncoder;
};


struct pdraw {
	Pdraw::IPdraw *pdraw;
	PdrawListener *listener;
	pthread_mutex_t mutex;
	std::vector<PdrawDemuxerListener *> *demuxerListeners;
	std::vector<PdrawMuxerListener *> *muxerListeners;
	std::vector<PdrawVideoRendererListener *> *videoRendererListeners;
	std::vector<PdrawAudioRendererListener *> *audioRendererListeners;
	std::vector<PdrawVipcSourceListener *> *vipcSourceListeners;
	std::vector<PdrawCodedVideoSourceListener *> *codedVideoSourceListeners;
	std::vector<PdrawRawVideoSourceListener *> *rawVideoSourceListeners;
	std::vector<PdrawCodedVideoSinkListener *> *codedVideoSinkListeners;
	std::vector<PdrawRawVideoSinkListener *> *rawVideoSinkListeners;
	std::vector<PdrawAlsaSourceListener *> *alsaSourceListeners;
	std::vector<PdrawAudioSourceListener *> *audioSourceListeners;
	std::vector<PdrawAudioSinkListener *> *audioSinkListeners;
	std::vector<PdrawVideoEncoderListener *> *videoEncoderListeners;
	std::vector<PdrawVideoScalerListener *> *videoScalerListeners;
	std::vector<PdrawAudioEncoderListener *> *audioEncoderListeners;
};


int pdraw_new(struct pomp_loop *loop,
	      const struct pdraw_cbs *cbs,
	      void *userdata,
	      struct pdraw **ret_obj)
{
	int ret = 0;
	struct pdraw *pdraw;

	ULOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	pdraw = (struct pdraw *)calloc(1, sizeof(*pdraw));
	if (pdraw == nullptr)
		return -ENOMEM;

	ret = pthread_mutex_init(&pdraw->mutex, nullptr);
	if (ret != 0) {
		free(pdraw);
		return -ret;
	}

	pdraw->demuxerListeners = new std::vector<PdrawDemuxerListener *>();
	if (pdraw->demuxerListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->muxerListeners = new std::vector<PdrawMuxerListener *>();
	if (pdraw->muxerListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->videoRendererListeners =
		new std::vector<PdrawVideoRendererListener *>();
	if (pdraw->videoRendererListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->audioRendererListeners =
		new std::vector<PdrawAudioRendererListener *>();
	if (pdraw->audioRendererListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->vipcSourceListeners =
		new std::vector<PdrawVipcSourceListener *>();
	if (pdraw->vipcSourceListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->codedVideoSourceListeners =
		new std::vector<PdrawCodedVideoSourceListener *>();
	if (pdraw->codedVideoSourceListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->rawVideoSourceListeners =
		new std::vector<PdrawRawVideoSourceListener *>();
	if (pdraw->rawVideoSourceListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->codedVideoSinkListeners =
		new std::vector<PdrawCodedVideoSinkListener *>();
	if (pdraw->codedVideoSinkListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->rawVideoSinkListeners =
		new std::vector<PdrawRawVideoSinkListener *>();
	if (pdraw->rawVideoSinkListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->alsaSourceListeners =
		new std::vector<PdrawAlsaSourceListener *>();
	if (pdraw->alsaSourceListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->audioSourceListeners =
		new std::vector<PdrawAudioSourceListener *>();
	if (pdraw->audioSourceListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->audioSinkListeners = new std::vector<PdrawAudioSinkListener *>();
	if (pdraw->audioSinkListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->videoEncoderListeners =
		new std::vector<PdrawVideoEncoderListener *>();
	if (pdraw->videoEncoderListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->videoScalerListeners =
		new std::vector<PdrawVideoScalerListener *>();
	if (pdraw->videoScalerListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->audioEncoderListeners =
		new std::vector<PdrawAudioEncoderListener *>();
	if (pdraw->audioEncoderListeners == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->listener = new PdrawListener(pdraw, cbs, userdata);
	if (pdraw->listener == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->pdraw = new Pdraw::Session(loop, pdraw->listener);
	if (pdraw->pdraw == nullptr) {
		ret = -ENOMEM;
		goto error;
	}

	*ret_obj = pdraw;
	return 0;

error:
	pdraw_destroy(pdraw);
	return ret;
}


int pdraw_destroy(struct pdraw *pdraw)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	if (pdraw->pdraw != nullptr)
		delete pdraw->pdraw;
	if (pdraw->listener != nullptr)
		delete pdraw->listener;

	if (pdraw->demuxerListeners != nullptr) {
		std::vector<PdrawDemuxerListener *>::iterator l =
			pdraw->demuxerListeners->begin();
		while (l != pdraw->demuxerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->demuxerListeners->clear();
		delete pdraw->demuxerListeners;
	}

	if (pdraw->muxerListeners != nullptr) {
		std::vector<PdrawMuxerListener *>::iterator l =
			pdraw->muxerListeners->begin();
		while (l != pdraw->muxerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->muxerListeners->clear();
		delete pdraw->muxerListeners;
	}

	if (pdraw->codedVideoSourceListeners != nullptr) {
		std::vector<PdrawCodedVideoSourceListener *>::iterator l =
			pdraw->codedVideoSourceListeners->begin();
		while (l != pdraw->codedVideoSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->codedVideoSourceListeners->clear();
		delete pdraw->codedVideoSourceListeners;
	}

	if (pdraw->vipcSourceListeners != nullptr) {
		std::vector<PdrawVipcSourceListener *>::iterator l =
			pdraw->vipcSourceListeners->begin();
		while (l != pdraw->vipcSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->vipcSourceListeners->clear();
		delete pdraw->vipcSourceListeners;
	}

	if (pdraw->rawVideoSourceListeners != nullptr) {
		std::vector<PdrawRawVideoSourceListener *>::iterator l =
			pdraw->rawVideoSourceListeners->begin();
		while (l != pdraw->rawVideoSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->rawVideoSourceListeners->clear();
		delete pdraw->rawVideoSourceListeners;
	}

	if (pdraw->codedVideoSinkListeners != nullptr) {
		std::vector<PdrawCodedVideoSinkListener *>::iterator l =
			pdraw->codedVideoSinkListeners->begin();
		while (l != pdraw->codedVideoSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->codedVideoSinkListeners->clear();
		delete pdraw->codedVideoSinkListeners;
	}

	if (pdraw->rawVideoSinkListeners != nullptr) {
		std::vector<PdrawRawVideoSinkListener *>::iterator l =
			pdraw->rawVideoSinkListeners->begin();
		while (l != pdraw->rawVideoSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->rawVideoSinkListeners->clear();
		delete pdraw->rawVideoSinkListeners;
	}

	if (pdraw->alsaSourceListeners != nullptr) {
		std::vector<PdrawAlsaSourceListener *>::iterator l =
			pdraw->alsaSourceListeners->begin();
		while (l != pdraw->alsaSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->alsaSourceListeners->clear();
		delete pdraw->alsaSourceListeners;
	}

	if (pdraw->audioSourceListeners != nullptr) {
		std::vector<PdrawAudioSourceListener *>::iterator l =
			pdraw->audioSourceListeners->begin();
		while (l != pdraw->audioSourceListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->audioSourceListeners->clear();
		delete pdraw->audioSourceListeners;
	}

	if (pdraw->audioSinkListeners != nullptr) {
		std::vector<PdrawAudioSinkListener *>::iterator l =
			pdraw->audioSinkListeners->begin();
		while (l != pdraw->audioSinkListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->audioSinkListeners->clear();
		delete pdraw->audioSinkListeners;
	}

	if (pdraw->videoEncoderListeners != nullptr) {
		std::vector<PdrawVideoEncoderListener *>::iterator l =
			pdraw->videoEncoderListeners->begin();
		while (l != pdraw->videoEncoderListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->videoEncoderListeners->clear();
		delete pdraw->videoEncoderListeners;
	}

	if (pdraw->videoScalerListeners != nullptr) {
		std::vector<PdrawVideoScalerListener *>::iterator l =
			pdraw->videoScalerListeners->begin();
		while (l != pdraw->videoScalerListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->videoScalerListeners->clear();
		delete pdraw->videoScalerListeners;
	}

	if (pdraw->audioEncoderListeners != nullptr) {
		std::vector<PdrawAudioEncoderListener *>::iterator l =
			pdraw->audioEncoderListeners->begin();
		while (l != pdraw->audioEncoderListeners->end()) {
			if (*l != nullptr)
				delete (*l);
			l++;
		}
		pdraw->audioEncoderListeners->clear();
		delete pdraw->audioEncoderListeners;
	}

	pthread_mutex_lock(&pdraw->mutex);
	if (pdraw->videoRendererListeners != nullptr) {
		std::vector<PdrawVideoRendererListener *>::iterator r =
			pdraw->videoRendererListeners->begin();
		while (r != pdraw->videoRendererListeners->end()) {
			if (*r != nullptr)
				delete (*r);
			r++;
		}
		pdraw->videoRendererListeners->clear();
		delete pdraw->videoRendererListeners;
	}

	if (pdraw->audioRendererListeners != nullptr) {
		std::vector<PdrawAudioRendererListener *>::iterator r =
			pdraw->audioRendererListeners->begin();
		while (r != pdraw->audioRendererListeners->end()) {
			if (*r != nullptr)
				delete (*r);
			r++;
		}
		pdraw->audioRendererListeners->clear();
		delete pdraw->audioRendererListeners;
	}
	pthread_mutex_unlock(&pdraw->mutex);
	pthread_mutex_destroy(&pdraw->mutex);

	free(pdraw);
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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_addr == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawDemuxerListener *demuxerListener =
		new PdrawDemuxerListener(pdraw, cbs, userdata);
	if (demuxerListener == nullptr) {
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
					  demuxerListener,
					  &demuxer);
	if (res < 0) {
		delete demuxerListener;
		return res;
	}

	demuxerListener->setDemuxer(demuxer);
	pdraw->demuxerListeners->push_back(demuxerListener);

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == nullptr, EINVAL);
	/* Note: deliberately not testing the mux pointer, as
	 * pdraw_demuxer_new_from_url() calls this function with
	 * a null mux pointer */
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawDemuxerListener *demuxerListener =
		new PdrawDemuxerListener(pdraw, cbs, userdata);
	if (demuxerListener == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createDemuxer(
		u, mux, params, demuxerListener, &demuxer);
	if (res < 0) {
		delete demuxerListener;
		return res;
	}

	demuxerListener->setDemuxer(demuxer);
	pdraw->demuxerListeners->push_back(demuxerListener);

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
	return 0;
}


int pdraw_demuxer_destroy(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	std::vector<PdrawDemuxerListener *>::iterator l;
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete d;

	l = pdraw->demuxerListeners->begin();
	while (l != pdraw->demuxerListeners->end()) {
		if ((*l)->getDemuxer() != d) {
			l++;
			continue;
		}
		delete *l;
		pdraw->demuxerListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_demuxer_close(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->close();
}


int pdraw_demuxer_get_media_list(struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer,
				 struct pdraw_demuxer_media **media_list,
				 size_t *media_count,
				 uint32_t *selected_medias)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_demuxer_select_media(struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint32_t selected_medias)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->selectMedia(selected_medias);
}


uint16_t
pdraw_demuxer_get_single_stream_local_stream_port(struct pdraw *pdraw,
						  struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_demuxer_get_single_stream_local_control_port(
	struct pdraw *pdraw,
	struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getSingleStreamLocalControlPort();
}


int pdraw_demuxer_is_ready_to_play(struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_demuxer_is_paused(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_demuxer_play(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play();
}


int pdraw_demuxer_play_with_speed(struct pdraw *pdraw,
				  struct pdraw_demuxer *demuxer,
				  float speed)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play(speed);
}


int pdraw_demuxer_pause(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->pause();
}


int pdraw_demuxer_previous_frame(struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->previousFrame();
}


int pdraw_demuxer_next_frame(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->nextFrame();
}


int pdraw_demuxer_seek(struct pdraw *pdraw,
		       struct pdraw_demuxer *demuxer,
		       int64_t delta,
		       int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seek(delta, exact ? true : false);
}


int pdraw_demuxer_seek_forward(struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_demuxer_seek_back(struct pdraw *pdraw,
			    struct pdraw_demuxer *demuxer,
			    uint64_t delta,
			    int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_demuxer_seek_to(struct pdraw *pdraw,
			  struct pdraw_demuxer *demuxer,
			  uint64_t timestamp,
			  int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_demuxer_get_chapter_list(struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer,
				   struct pdraw_chapter **chapter_list,
				   size_t *chapter_count)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_demuxer_get_duration(struct pdraw *pdraw,
				    struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getDuration();
}


uint64_t pdraw_demuxer_get_current_time(struct pdraw *pdraw,
					struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getCurrentTime();
}


int pdraw_muxer_new(struct pdraw *pdraw,
		    const char *url,
		    const struct pdraw_muxer_params *params,
		    const struct pdraw_muxer_cbs *cbs,
		    void *userdata,
		    struct pdraw_muxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IMuxer *muxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawMuxerListener *muxerListener =
		new PdrawMuxerListener(pdraw, cbs, userdata);
	if (muxerListener == nullptr) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createMuxer(u, params, muxerListener, &muxer);
	if (res < 0) {
		delete muxerListener;
		return res;
	}

	muxerListener->setMuxer(muxer);
	pdraw->muxerListeners->push_back(muxerListener);

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
}


int pdraw_muxer_destroy(struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	std::vector<PdrawMuxerListener *>::iterator l;
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete m;

	l = pdraw->muxerListeners->begin();
	while (l != pdraw->muxerListeners->end()) {
		if ((*l)->getMuxer() != m) {
			l++;
			continue;
		}
		delete *l;
		pdraw->muxerListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_muxer_close(struct pdraw *self, struct pdraw_muxer *muxer)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->close();
}


int pdraw_muxer_add_media(struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  unsigned int media_id,
			  const struct pdraw_muxer_media_params *params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addMedia(media_id, params);
}


int pdraw_muxer_set_thumbnail(struct pdraw *pdraw,
			      struct pdraw_muxer *muxer,
			      enum pdraw_muxer_thumbnail_type type,
			      const uint8_t *data,
			      size_t size)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->setThumbnail(type, data, size);
}


int pdraw_muxer_add_chapter(struct pdraw *pdraw,
			    struct pdraw_muxer *muxer,
			    uint64_t timestamp,
			    const char *name)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addChapter(timestamp, name);
}


int pdraw_muxer_get_stats(struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  struct pdraw_muxer_stats *stats)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->getStats(stats);
}


PDRAW_API int
pdraw_muxer_set_dyn_params(struct pdraw *pdraw,
			   struct pdraw_muxer *muxer,
			   const struct pdraw_muxer_dyn_params *dyn_params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->setDynParams(dyn_params);
}


PDRAW_API int
pdraw_muxer_get_dyn_params(struct pdraw *pdraw,
			   struct pdraw_muxer *muxer,
			   struct pdraw_muxer_dyn_params *dyn_params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->getDynParams(dyn_params);
}


int pdraw_muxer_force_sync(struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

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
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	pthread_mutex_lock(&pdraw->mutex);
	PdrawVideoRendererListener *l =
		new PdrawVideoRendererListener(pdraw, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video renderer listener");
		ret = -ENOMEM;
		goto error;
	}

	ret = pdraw->pdraw->createVideoRenderer(
		media_id, render_pos, params, l, &renderer);
	if (ret < 0) {
		delete l;
		goto error;
	}

	pdraw->videoRendererListeners->push_back(l);
	l->setVideoRenderer(renderer);
	pthread_mutex_unlock(&pdraw->mutex);

	*ret_obj = reinterpret_cast<struct pdraw_video_renderer *>(renderer);
	return 0;

error:
	pthread_mutex_unlock(&pdraw->mutex);
	return ret;
}


int pdraw_video_renderer_destroy(struct pdraw *pdraw,
				 struct pdraw_video_renderer *renderer)
{
	std::vector<PdrawVideoRendererListener *>::iterator l;
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete rnd;

	pthread_mutex_lock(&pdraw->mutex);
	l = pdraw->videoRendererListeners->begin();
	while (l != pdraw->videoRendererListeners->end()) {
		if ((*l)->getVideoRenderer() != rnd) {
			l++;
			continue;
		}
		delete *l;
		pdraw->videoRendererListeners->erase(l);
		break;
	}
	pthread_mutex_unlock(&pdraw->mutex);

	return 0;
}


int pdraw_video_renderer_resize(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *render_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->resize(render_pos);
}


int pdraw_video_renderer_set_media_id(struct pdraw *pdraw,
				      struct pdraw_video_renderer *renderer,
				      unsigned int media_id)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_video_renderer_get_media_id(struct pdraw *pdraw,
				  struct pdraw_video_renderer *renderer)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	if (pdraw == nullptr)
		return 0;
	if (renderer == nullptr)
		return 0;

	return rnd->getMediaId();
}


int pdraw_video_renderer_set_params(
	struct pdraw *pdraw,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_video_renderer_get_params(struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->getParams(params);
}


int pdraw_video_renderer_render(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				struct pdraw_rect *content_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->render(content_pos, nullptr, nullptr);
}


int pdraw_video_renderer_render_mat(struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_rect *content_pos,
				    const float *view_mat,
				    const float *proj_mat)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

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
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	pthread_mutex_lock(&pdraw->mutex);
	PdrawAudioRendererListener *l =
		new PdrawAudioRendererListener(pdraw, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create audio renderer listener");
		ret = -ENOMEM;
		goto error;
	}

	ret = pdraw->pdraw->createAudioRenderer(media_id, params, l, &renderer);
	if (ret < 0) {
		delete l;
		goto error;
	}

	pdraw->audioRendererListeners->push_back(l);
	l->setAudioRenderer(renderer);
	pthread_mutex_unlock(&pdraw->mutex);

	*ret_obj = reinterpret_cast<struct pdraw_audio_renderer *>(renderer);
	return 0;

error:
	pthread_mutex_unlock(&pdraw->mutex);
	return ret;
}


int pdraw_audio_renderer_destroy(struct pdraw *pdraw,
				 struct pdraw_audio_renderer *renderer)
{
	std::vector<PdrawAudioRendererListener *>::iterator l;
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete rnd;

	pthread_mutex_lock(&pdraw->mutex);
	l = pdraw->audioRendererListeners->begin();
	while (l != pdraw->audioRendererListeners->end()) {
		if ((*l)->getAudioRenderer() != rnd) {
			l++;
			continue;
		}
		delete *l;
		pdraw->audioRendererListeners->erase(l);
		break;
	}
	pthread_mutex_unlock(&pdraw->mutex);

	return 0;
}


int pdraw_audio_renderer_set_media_id(struct pdraw *pdraw,
				      struct pdraw_audio_renderer *renderer,
				      unsigned int media_id)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_audio_renderer_get_media_id(struct pdraw *pdraw,
				  struct pdraw_audio_renderer *renderer)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	if (pdraw == nullptr)
		return 0;
	if (renderer == nullptr)
		return 0;

	return rnd->getMediaId();
}


int pdraw_audio_renderer_set_params(
	struct pdraw *pdraw,
	struct pdraw_audio_renderer *renderer,
	const struct pdraw_audio_renderer_params *params)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_audio_renderer_get_params(struct pdraw *pdraw,
				    struct pdraw_audio_renderer *renderer,
				    struct pdraw_audio_renderer_params *params)
{
	Pdraw::IPdraw::IAudioRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawVipcSourceListener *vipcSourceListener =
		new PdrawVipcSourceListener(pdraw, cbs, userdata);
	if (vipcSourceListener == nullptr) {
		ULOGE("failed to create video IPC source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVipcSource(
		params, vipcSourceListener, &source);
	if (res < 0) {
		delete vipcSourceListener;
		return res;
	}

	vipcSourceListener->setVipcSource(source);
	pdraw->vipcSourceListeners->push_back(vipcSourceListener);

	*ret_obj = reinterpret_cast<struct pdraw_vipc_source *>(source);
	return 0;
}


int pdraw_vipc_source_destroy(struct pdraw *pdraw,
			      struct pdraw_vipc_source *source)
{
	std::vector<PdrawVipcSourceListener *>::iterator l;
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->vipcSourceListeners->begin();
	while (l != pdraw->vipcSourceListeners->end()) {
		if ((*l)->getVipcSource() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->vipcSourceListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_vipc_source_is_ready_to_play(struct pdraw *pdraw,
				       struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_vipc_source_is_paused(struct pdraw *pdraw,
				struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isPaused() ? 1 : 0;
}


int pdraw_vipc_source_play(struct pdraw *pdraw,
			   struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_vipc_source_pause(struct pdraw *pdraw,
			    struct pdraw_vipc_source *source)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->pause();
}


int pdraw_vipc_source_configure(struct pdraw *pdraw,
				struct pdraw_vipc_source *source,
				const struct vdef_dim *resolution,
				const struct vdef_rectf *crop)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->configure(resolution, crop);
}


int pdraw_vipc_source_insert_grey_frame(struct pdraw *pdraw,
					struct pdraw_vipc_source *source,
					uint64_t ts_us)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->insertGreyFrame(ts_us);
}


int pdraw_vipc_source_set_session_metadata(struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   const struct vmeta_session *meta)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_vipc_source_get_session_metadata(struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   struct vmeta_session *meta)
{
	Pdraw::IPdraw::IVipcSource *s =
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawCodedVideoSourceListener *videoSourceListener =
		new PdrawCodedVideoSourceListener(pdraw, cbs, userdata);
	if (videoSourceListener == nullptr) {
		ULOGE("failed to create coded video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSource(
		params, videoSourceListener, &source);
	if (res < 0) {
		delete videoSourceListener;
		return res;
	}

	videoSourceListener->setCodedVideoSource(source);
	pdraw->codedVideoSourceListeners->push_back(videoSourceListener);

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_source *>(source);
	return 0;
}


int pdraw_coded_video_source_destroy(struct pdraw *pdraw,
				     struct pdraw_coded_video_source *source)
{
	std::vector<PdrawCodedVideoSourceListener *>::iterator l;
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->codedVideoSourceListeners->begin();
	while (l != pdraw->codedVideoSourceListeners->end()) {
		if ((*l)->getCodedVideoSource() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->codedVideoSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_source_get_queue(struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_coded_video_source_flush(struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_coded_video_source_set_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_coded_video_source_get_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	Pdraw::IPdraw::ICodedVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawRawVideoSourceListener *videoSourceListener =
		new PdrawRawVideoSourceListener(pdraw, cbs, userdata);
	if (videoSourceListener == nullptr) {
		ULOGE("failed to create raw video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSource(
		params, videoSourceListener, &source);
	if (res < 0) {
		delete videoSourceListener;
		return res;
	}

	videoSourceListener->setRawVideoSource(source);
	pdraw->rawVideoSourceListeners->push_back(videoSourceListener);

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_source *>(source);
	return 0;
}


int pdraw_raw_video_source_destroy(struct pdraw *pdraw,
				   struct pdraw_raw_video_source *source)
{
	std::vector<PdrawRawVideoSourceListener *>::iterator l;
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->rawVideoSourceListeners->begin();
	while (l != pdraw->rawVideoSourceListeners->end()) {
		if ((*l)->getRawVideoSource() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->rawVideoSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_source_get_queue(struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_raw_video_source_flush(struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_raw_video_source_set_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_raw_video_source_get_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	Pdraw::IPdraw::IRawVideoSource *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawCodedVideoSinkListener *videoSinkListener =
		new PdrawCodedVideoSinkListener(pdraw, cbs, userdata);
	if (videoSinkListener == nullptr) {
		ULOGE("failed to create coded video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSink(
		media_id, params, videoSinkListener, &sink);
	if (res < 0) {
		delete videoSinkListener;
		return res;
	}

	videoSinkListener->setCodedVideoSink(sink);
	pdraw->codedVideoSinkListeners->push_back(videoSinkListener);

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_sink *>(sink);
	return 0;
}


int pdraw_coded_video_sink_destroy(struct pdraw *pdraw,
				   struct pdraw_coded_video_sink *sink)
{
	std::vector<PdrawCodedVideoSinkListener *>::iterator l;
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->codedVideoSinkListeners->begin();
	while (l != pdraw->codedVideoSinkListeners->end()) {
		if ((*l)->getCodedVideoSink() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->codedVideoSinkListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_coded_video_sink_resync(struct pdraw *pdraw,
				  struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->resync();
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_sink_get_queue(struct pdraw *pdraw,
				 struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_coded_video_sink_queue_flushed(struct pdraw *pdraw,
					 struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);


	PdrawRawVideoSinkListener *videoSinkListener =
		new PdrawRawVideoSinkListener(pdraw, cbs, userdata);
	if (videoSinkListener == nullptr) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSink(
		media_id, params, videoSinkListener, &sink);
	if (res < 0) {
		delete videoSinkListener;
		return res;
	}

	videoSinkListener->setRawVideoSink(sink);
	pdraw->rawVideoSinkListeners->push_back(videoSinkListener);

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_sink *>(sink);
	return 0;
}


int pdraw_raw_video_sink_destroy(struct pdraw *pdraw,
				 struct pdraw_raw_video_sink *sink)
{
	std::vector<PdrawRawVideoSinkListener *>::iterator l;
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->rawVideoSinkListeners->begin();
	while (l != pdraw->rawVideoSinkListeners->end()) {
		if ((*l)->getRawVideoSink() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->rawVideoSinkListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_sink_get_queue(struct pdraw *pdraw,
			       struct pdraw_raw_video_sink *sink)
{
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_raw_video_sink_queue_flushed(struct pdraw *pdraw,
				       struct pdraw_raw_video_sink *sink)
{
	Pdraw::IPdraw::IRawVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_alsa_source_new(struct pdraw *pdraw,
			  const struct pdraw_alsa_source_params *params,
			  const struct pdraw_alsa_source_cbs *cbs,
			  void *userdata,
			  struct pdraw_alsa_source **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAlsaSource *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawAlsaSourceListener *alsaSourceListener =
		new PdrawAlsaSourceListener(pdraw, cbs, userdata);
	if (alsaSourceListener == nullptr) {
		ULOGE("failed to create video IPC source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAlsaSource(
		params, alsaSourceListener, &source);
	if (res < 0) {
		delete alsaSourceListener;
		return res;
	}

	alsaSourceListener->setAlsaSource(source);
	pdraw->alsaSourceListeners->push_back(alsaSourceListener);

	*ret_obj = reinterpret_cast<struct pdraw_alsa_source *>(source);
	return 0;
}


int pdraw_alsa_source_destroy(struct pdraw *pdraw,
			      struct pdraw_alsa_source *source)
{
	std::vector<PdrawAlsaSourceListener *>::iterator l;
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->alsaSourceListeners->begin();
	while (l != pdraw->alsaSourceListeners->end()) {
		if ((*l)->getAlsaSource() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->alsaSourceListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_alsa_source_is_ready_to_play(struct pdraw *pdraw,
				       struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_alsa_source_is_paused(struct pdraw *pdraw,
				struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isPaused() ? 1 : 0;
}


int pdraw_alsa_source_play(struct pdraw *pdraw,
			   struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_alsa_source_pause(struct pdraw *pdraw,
			    struct pdraw_alsa_source *source)
{
	Pdraw::IPdraw::IAlsaSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawAudioSourceListener *audioSourceListener =
		new PdrawAudioSourceListener(pdraw, cbs, userdata);
	if (audioSourceListener == nullptr) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSource(
		params, audioSourceListener, &source);
	if (res < 0) {
		delete audioSourceListener;
		return res;
	}

	audioSourceListener->setAudioSource(source);
	pdraw->audioSourceListeners->push_back(audioSourceListener);

	*ret_obj = reinterpret_cast<struct pdraw_audio_source *>(source);
	return 0;
}


int pdraw_audio_source_destroy(struct pdraw *pdraw,
			       struct pdraw_audio_source *source)
{
	std::vector<PdrawAudioSourceListener *>::iterator l;
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->audioSourceListeners->begin();
	while (l != pdraw->audioSourceListeners->end()) {
		if ((*l)->getAudioSource() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->audioSourceListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_audio_source_get_queue(struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_audio_source_flush(struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	Pdraw::IPdraw::IAudioSource *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_audio_sink_new(struct pdraw *pdraw,
			 unsigned int media_id,
			 const struct pdraw_audio_sink_cbs *cbs,
			 void *userdata,
			 struct pdraw_audio_sink **ret_obj)
{
	int res;
	Pdraw::IPdraw::IAudioSink *sink = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawAudioSinkListener *audioSinkListener =
		new PdrawAudioSinkListener(pdraw, cbs, userdata);
	if (audioSinkListener == nullptr) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSink(media_id, audioSinkListener, &sink);
	if (res < 0) {
		delete audioSinkListener;
		return res;
	}

	audioSinkListener->setAudioSink(sink);
	pdraw->audioSinkListeners->push_back(audioSinkListener);

	*ret_obj = reinterpret_cast<struct pdraw_audio_sink *>(sink);
	return 0;
}


int pdraw_audio_sink_destroy(struct pdraw *pdraw, struct pdraw_audio_sink *sink)
{
	std::vector<PdrawAudioSinkListener *>::iterator l;
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete s;

	l = pdraw->audioSinkListeners->begin();
	while (l != pdraw->audioSinkListeners->end()) {
		if ((*l)->getAudioSink() != s) {
			l++;
			continue;
		}
		delete *l;
		pdraw->audioSinkListeners->erase(l);
		break;
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_audio_sink_get_queue(struct pdraw *pdraw, struct pdraw_audio_sink *sink)
{
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_audio_sink_queue_flushed(struct pdraw *pdraw,
				   struct pdraw_audio_sink *sink)
{
	Pdraw::IPdraw::IAudioSink *s =
		reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawVideoEncoderListener *videoEncoderListener =
		new PdrawVideoEncoderListener(pdraw, cbs, userdata);
	if (videoEncoderListener == nullptr) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoEncoder(
		media_id, params, videoEncoderListener, &encoder);
	if (res < 0) {
		delete videoEncoderListener;
		return res;
	}

	videoEncoderListener->setVideoEncoder(encoder);
	pdraw->videoEncoderListeners->push_back(videoEncoderListener);

	*ret_obj = reinterpret_cast<struct pdraw_video_encoder *>(encoder);
	return 0;
}


int pdraw_video_encoder_destroy(struct pdraw *pdraw,
				struct pdraw_video_encoder *encoder)
{
	std::vector<PdrawVideoEncoderListener *>::iterator l;
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = pdraw->videoEncoderListeners->begin();
	while (l != pdraw->videoEncoderListeners->end()) {
		if ((*l)->getVideoEncoder() != e) {
			l++;
			continue;
		}
		delete *l;
		pdraw->videoEncoderListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_video_encoder_configure(struct pdraw *pdraw,
				  struct pdraw_video_encoder *encoder,
				  const struct venc_dyn_config *config)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->configure(config);
}


int pdraw_video_encoder_get_config(struct pdraw *pdraw,
				   struct pdraw_video_encoder *encoder,
				   struct venc_dyn_config *config)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->getConfig(config);
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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawVideoScalerListener *videoScalerListener =
		new PdrawVideoScalerListener(pdraw, cbs, userdata);
	if (videoScalerListener == nullptr) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoScaler(
		media_id, params, videoScalerListener, &encoder);
	if (res < 0) {
		delete videoScalerListener;
		return res;
	}

	videoScalerListener->setVideoScaler(encoder);
	pdraw->videoScalerListeners->push_back(videoScalerListener);

	*ret_obj = reinterpret_cast<struct pdraw_video_scaler *>(encoder);
	return 0;
}


int pdraw_video_scaler_destroy(struct pdraw *pdraw,
			       struct pdraw_video_scaler *encoder)
{
	std::vector<PdrawVideoScalerListener *>::iterator l;
	Pdraw::IPdraw::IVideoScaler *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = pdraw->videoScalerListeners->begin();
	while (l != pdraw->videoScalerListeners->end()) {
		if ((*l)->getVideoScaler() != e) {
			l++;
			continue;
		}
		delete *l;
		pdraw->videoScalerListeners->erase(l);
		break;
	}

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

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawAudioEncoderListener *audioEncoderListener =
		new PdrawAudioEncoderListener(pdraw, cbs, userdata);
	if (audioEncoderListener == nullptr) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioEncoder(
		media_id, params, audioEncoderListener, &encoder);
	if (res < 0) {
		delete audioEncoderListener;
		return res;
	}

	audioEncoderListener->setAudioEncoder(encoder);
	pdraw->audioEncoderListeners->push_back(audioEncoderListener);

	*ret_obj = reinterpret_cast<struct pdraw_audio_encoder *>(encoder);
	return 0;
}


int pdraw_audio_encoder_destroy(struct pdraw *pdraw,
				struct pdraw_audio_encoder *encoder)
{
	std::vector<PdrawAudioEncoderListener *>::iterator l;
	Pdraw::IPdraw::IAudioEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	/* The object must be destroyed before the listener */
	delete e;

	l = pdraw->audioEncoderListeners->begin();
	while (l != pdraw->audioEncoderListeners->end()) {
		if ((*l)->getAudioEncoder() != e) {
			l++;
			continue;
		}
		delete *l;
		pdraw->audioEncoderListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_get_friendly_name_setting(struct pdraw *pdraw, char *str, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	std::string fn;
	pdraw->pdraw->getFriendlyNameSetting(&fn);
	if ((str) && (fn.length() >= len))
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
	if ((str) && (sn.length() >= len))
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
	if ((str) && (sv.length() >= len))
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


enum pdraw_media_type pdraw_media_type_from_str(const char *val)
{
	return pdraw_mediaTypeFromStr(val);
}


const char *pdraw_video_type_str(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


enum pdraw_video_type pdraw_video_type_from_str(const char *val)
{
	return pdraw_videoTypeFromStr(val);
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
