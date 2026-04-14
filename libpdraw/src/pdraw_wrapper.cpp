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

#include <memory>
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_cbs mCbs;
	void *mUserdata = nullptr;
};


class PdrawDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener {
public:
	PdrawDemuxerListener(struct pdraw *pdraw,
			     const struct pdraw_demuxer_cbs *cbs,
			     void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawDemuxerListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_demuxer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IDemuxer *mDemuxer = nullptr;
};


class PdrawMuxerListener : public Pdraw::IPdraw::IMuxer::Listener {
public:
	PdrawMuxerListener(struct pdraw *pdraw,
			   const struct pdraw_muxer_cbs *cbs,
			   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawMuxerListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_muxer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IMuxer *mMuxer = nullptr;
};


class PdrawVideoRendererListener
		: public Pdraw::IPdraw::IVideoRenderer::Listener {
public:
	PdrawVideoRendererListener(struct pdraw *pdraw,
				   const struct pdraw_video_renderer_cbs *cbs,
				   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawVideoRendererListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_video_renderer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoRenderer *mRenderer = nullptr;
};


class PdrawAudioRendererListener
		: public Pdraw::IPdraw::IAudioRenderer::Listener {
public:
	PdrawAudioRendererListener(struct pdraw *pdraw,
				   const struct pdraw_audio_renderer_cbs *cbs,
				   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawAudioRendererListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_audio_renderer_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioRenderer *mRenderer = nullptr;
};


class PdrawVipcSourceListener : public Pdraw::IPdraw::IVipcSource::Listener {
public:
	PdrawVipcSourceListener(struct pdraw *pdraw,
				const struct pdraw_vipc_source_cbs *cbs,
				void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawVipcSourceListener() override = default;

	void vipcSourceReadyToPlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
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

	void vipcSourcePlayResponse(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVipcSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_vipc_source *>(
					source),
				mUserdata);
	}

	void
	vipcSourcePauseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IVipcSource *source) override
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

	void vipcSourceConfigured(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
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

	void vipcSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IVipcSource *source,
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
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVipcSource *source,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_vipc_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVipcSource *mSource = nullptr;
};


class PdrawCodedVideoSourceListener
		: public Pdraw::IPdraw::ICodedVideoSource::Listener {
public:
	PdrawCodedVideoSourceListener(
		struct pdraw *pdraw,
		const struct pdraw_coded_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawCodedVideoSourceListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_coded_video_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::ICodedVideoSource *mSource = nullptr;
};


class PdrawRawVideoSourceListener
		: public Pdraw::IPdraw::IRawVideoSource::Listener {
public:
	PdrawRawVideoSourceListener(
		struct pdraw *pdraw,
		const struct pdraw_raw_video_source_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawRawVideoSourceListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_raw_video_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IRawVideoSource *mSource = nullptr;
};


class PdrawCodedVideoSinkListener
		: public Pdraw::IPdraw::ICodedVideoSink::Listener {
public:
	PdrawCodedVideoSinkListener(
		struct pdraw *pdraw,
		const struct pdraw_coded_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawCodedVideoSinkListener() override = default;

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
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::ICodedVideoSink *sink,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_coded_video_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::ICodedVideoSink *mSink = nullptr;
};


class PdrawRawVideoSinkListener
		: public Pdraw::IPdraw::IRawVideoSink::Listener {
public:
	PdrawRawVideoSinkListener(struct pdraw *pdraw,
				  const struct pdraw_raw_video_sink_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawRawVideoSinkListener() override = default;

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
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IRawVideoSink *sink,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_raw_video_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IRawVideoSink *mSink = nullptr;
};


class PdrawAlsaSourceListener : public Pdraw::IPdraw::IAlsaSource::Listener {
public:
	PdrawAlsaSourceListener(struct pdraw *pdraw,
				const struct pdraw_alsa_source_cbs *cbs,
				void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawAlsaSourceListener() override = default;

	void alsaSourceReadyToPlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAlsaSource *source,
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

	void alsaSourcePlayResponse(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IAlsaSource *source) override
	{
		if (mCbs.play_resp)
			(*mCbs.play_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				mUserdata);
	}

	void
	alsaSourcePauseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IAlsaSource *source) override
	{
		if (mCbs.pause_resp)
			(*mCbs.pause_resp)(
				mPdraw,
				reinterpret_cast<struct pdraw_alsa_source *>(
					source),
				mUserdata);
	}

	void alsaSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAlsaSource *source,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_alsa_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAlsaSource *mSource = nullptr;
};


class PdrawAudioSourceListener : public Pdraw::IPdraw::IAudioSource::Listener {
public:
	PdrawAudioSourceListener(struct pdraw *pdraw,
				 const struct pdraw_audio_source_cbs *cbs,
				 void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawAudioSourceListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_audio_source_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioSource *mSource = nullptr;
};


class PdrawAudioSinkListener : public Pdraw::IPdraw::IAudioSink::Listener {
public:
	PdrawAudioSinkListener(struct pdraw *pdraw,
			       const struct pdraw_audio_sink_cbs *cbs,
			       void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawAudioSinkListener() override = default;

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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_audio_sink_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioSink *mSink = nullptr;
};


class PdrawVideoEncoderListener
		: public Pdraw::IPdraw::IVideoEncoder::Listener {
public:
	PdrawVideoEncoderListener(struct pdraw *pdraw,
				  const struct pdraw_video_encoder_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawVideoEncoderListener() override = default;

	void
	videoEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IVideoEncoder *encoder,
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
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IVideoEncoder *encoder,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_video_encoder_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoEncoder *mEncoder = nullptr;
};


class PdrawVideoScalerListener : public Pdraw::IPdraw::IVideoScaler::Listener {
public:
	PdrawVideoScalerListener(struct pdraw *pdraw,
				 const struct pdraw_video_scaler_cbs *cbs,
				 void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawVideoScalerListener() override = default;

	void videoScalerFrameOutput(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IVideoScaler *scaler,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_video_scaler_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IVideoScaler *mScaler = nullptr;
};


class PdrawAudioEncoderListener
		: public Pdraw::IPdraw::IAudioEncoder::Listener {
public:
	PdrawAudioEncoderListener(struct pdraw *pdraw,
				  const struct pdraw_audio_encoder_cbs *cbs,
				  void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawAudioEncoderListener() override = default;

	void audioEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioEncoder *encoder,
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
	audioEncoderFramePreRelease(Pdraw::IPdraw *pdraw,
				    Pdraw::IPdraw::IAudioEncoder *encoder,
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
	struct pdraw *mPdraw = nullptr;
	struct pdraw_audio_encoder_cbs mCbs;
	void *mUserdata = nullptr;
	Pdraw::IPdraw::IAudioEncoder *mEncoder = nullptr;
};


struct pdraw {
	std::unique_ptr<Pdraw::IPdraw> pdraw{};
	std::unique_ptr<PdrawListener> listener{};
	std::mutex mutex{};
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
};


int pdraw_new(struct pomp_loop *loop,
	      const struct pdraw_cbs *cbs,
	      void *userdata,
	      struct pdraw **ret_obj)
{
	struct pdraw *pdraw;

	ULOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	pdraw = new struct pdraw();
	if (pdraw == nullptr)
		return -ENOMEM;

	try {
		pdraw->listener =
			make_unique<PdrawListener>(pdraw, cbs, userdata);
		pdraw->pdraw = make_unique<Pdraw::Session>(
			loop, pdraw->listener.get());
	} catch (const std::bad_alloc &) {
		delete pdraw;
		return -ENOMEM;
	}

	*ret_obj = pdraw;
	return 0;
}


int pdraw_destroy(struct pdraw *pdraw)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);

	pdraw->pdraw.reset();
	pdraw->listener.reset();

	pdraw->demuxerListeners.clear();
	pdraw->muxerListeners.clear();
	pdraw->codedVideoSourceListeners.clear();
	pdraw->vipcSourceListeners.clear();
	pdraw->rawVideoSourceListeners.clear();
	pdraw->codedVideoSinkListeners.clear();
	pdraw->rawVideoSinkListeners.clear();
	pdraw->alsaSourceListeners.clear();
	pdraw->audioSourceListeners.clear();
	pdraw->audioSinkListeners.clear();
	pdraw->videoScalerListeners.clear();
	pdraw->audioEncoderListeners.clear();
	pdraw->audioEncoderListeners.clear();

	{
		std::unique_lock<std::mutex> lock(pdraw->mutex);
		pdraw->videoRendererListeners.clear();
		pdraw->audioRendererListeners.clear();
	}

	delete pdraw;
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
		demuxerListener =
			make_unique<PdrawDemuxerListener>(pdraw, cbs, userdata);
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

	demuxerListener->setDemuxer(demuxer);
	pdraw->demuxerListeners.push_back(std::move(demuxerListener));

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
		demuxerListener =
			make_unique<PdrawDemuxerListener>(pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createDemuxer(
		u, mux, params, demuxerListener.get(), &demuxer);
	if (res < 0)
		return res;

	demuxerListener->setDemuxer(demuxer);
	pdraw->demuxerListeners.push_back(std::move(demuxerListener));

	*ret_obj = reinterpret_cast<struct pdraw_demuxer *>(demuxer);
	return 0;
}


int pdraw_demuxer_destroy(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	/* The object must be destroyed before the listener */
	delete d;

	auto &listeners = pdraw->demuxerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getDemuxer() == d) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_demuxer_close(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

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
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getMediaList(media_list, media_count, selected_medias);
}


int pdraw_demuxer_select_media(struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint32_t selected_medias)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->selectMedia(selected_medias);
}


uint16_t
pdraw_demuxer_get_single_stream_local_stream_port(struct pdraw *pdraw,
						  struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

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
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getSingleStreamLocalControlPort();
}


int pdraw_demuxer_is_ready_to_play(struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_demuxer_is_paused(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_demuxer_play(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play();
}


int pdraw_demuxer_play_with_speed(struct pdraw *pdraw,
				  struct pdraw_demuxer *demuxer,
				  float speed)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->play(speed);
}


int pdraw_demuxer_pause(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->pause();
}


int pdraw_demuxer_previous_frame(struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->previousFrame();
}


int pdraw_demuxer_next_frame(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->nextFrame();
}


int pdraw_demuxer_seek(struct pdraw *pdraw,
		       struct pdraw_demuxer *demuxer,
		       int64_t delta,
		       int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seek(delta, exact ? true : false);
}


int pdraw_demuxer_seek_forward(struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_demuxer_seek_back(struct pdraw *pdraw,
			    struct pdraw_demuxer *demuxer,
			    uint64_t delta,
			    int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_demuxer_seek_to(struct pdraw *pdraw,
			  struct pdraw_demuxer *demuxer,
			  uint64_t timestamp,
			  int exact)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->seekTo(timestamp, exact ? true : false);
}


int pdraw_demuxer_get_chapter_list(struct pdraw *pdraw,
				   struct pdraw_demuxer *demuxer,
				   struct pdraw_chapter **chapter_list,
				   size_t *chapter_count)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(demuxer == nullptr, EINVAL);

	return d->getChapterList(chapter_list, chapter_count);
}


uint64_t pdraw_demuxer_get_duration(struct pdraw *pdraw,
				    struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return 0;
	if (demuxer == nullptr)
		return 0;

	return d->getDuration();
}


uint64_t pdraw_demuxer_get_current_time(struct pdraw *pdraw,
					struct pdraw_demuxer *demuxer)
{
	auto *d = reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

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
	std::unique_ptr<PdrawMuxerListener> muxerListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		muxerListener =
			make_unique<PdrawMuxerListener>(pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create muxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createMuxer(u, params, muxerListener.get(), &muxer);
	if (res < 0)
		return res;

	muxerListener->setMuxer(muxer);
	pdraw->muxerListeners.push_back(std::move(muxerListener));

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
}


int pdraw_muxer_destroy(struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	/* The object must be destroyed before the listener */
	delete m;

	auto &listeners = pdraw->muxerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getMuxer() == m) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_muxer_close(struct pdraw *self, struct pdraw_muxer *muxer)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->close();
}


int pdraw_muxer_add_media(struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  unsigned int media_id,
			  const struct pdraw_muxer_media_params *params)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

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
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->setThumbnail(type, data, size);
}


int pdraw_muxer_set_file_metadata(struct pdraw *pdraw,
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


int pdraw_muxer_add_chapter(struct pdraw *pdraw,
			    struct pdraw_muxer *muxer,
			    uint64_t timestamp,
			    const char *name)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addChapter(timestamp, name);
}


int pdraw_muxer_get_stats(struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  struct pdraw_muxer_stats *stats)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->getStats(stats);
}


int pdraw_muxer_set_dyn_params(struct pdraw *pdraw,
			       struct pdraw_muxer *muxer,
			       const struct pdraw_muxer_dyn_params *dyn_params)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->setDynParams(dyn_params);
}


int pdraw_muxer_get_dyn_params(struct pdraw *pdraw,
			       struct pdraw_muxer *muxer,
			       struct pdraw_muxer_dyn_params *dyn_params)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	return m->getDynParams(dyn_params);
}


int pdraw_muxer_force_sync(struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	auto *m = reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

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
	std::unique_ptr<PdrawVideoRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	{
		std::unique_lock<std::mutex> lock(pdraw->mutex);
		try {
			l = make_unique<PdrawVideoRendererListener>(
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

		l->setVideoRenderer(renderer);
		pdraw->videoRendererListeners.push_back(std::move(l));
	}


	*ret_obj = reinterpret_cast<struct pdraw_video_renderer *>(renderer);
	return 0;
}


int pdraw_video_renderer_destroy(struct pdraw *pdraw,
				 struct pdraw_video_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	/* The object must be destroyed before the listener */
	delete rnd;

	{
		std::unique_lock<std::mutex> lock(pdraw->mutex);
		auto &listeners = pdraw->videoRendererListeners;
		for (auto it = listeners.begin(); it != listeners.end(); ++it) {
			if ((*it)->getVideoRenderer() == rnd) {
				listeners.erase(it);
				break;
			}
		}
	}

	return 0;
}


int pdraw_video_renderer_resize(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *render_pos)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->resize(render_pos);
}


int pdraw_video_renderer_set_media_id(struct pdraw *pdraw,
				      struct pdraw_video_renderer *renderer,
				      unsigned int media_id)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_video_renderer_get_media_id(struct pdraw *pdraw,
				  struct pdraw_video_renderer *renderer)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

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
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_video_renderer_get_params(struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->getParams(params);
}


int pdraw_video_renderer_render(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				struct pdraw_rect *content_pos)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

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
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

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
	std::unique_ptr<PdrawAudioRendererListener> l;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	{
		std::unique_lock<std::mutex> lock(pdraw->mutex);
		try {
			l = make_unique<PdrawAudioRendererListener>(
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

		l->setAudioRenderer(renderer);
		pdraw->audioRendererListeners.push_back(std::move(l));
	}

	*ret_obj = reinterpret_cast<struct pdraw_audio_renderer *>(renderer);
	return 0;
}


int pdraw_audio_renderer_destroy(struct pdraw *pdraw,
				 struct pdraw_audio_renderer *renderer)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	/* The object must be destroyed before the listener */
	delete rnd;

	{
		std::unique_lock<std::mutex> lock(pdraw->mutex);
		auto &listeners = pdraw->audioRendererListeners;
		for (auto it = listeners.begin(); it != listeners.end(); ++it) {
			if ((*it)->getAudioRenderer() == rnd) {
				listeners.erase(it);
				break;
			}
		}
	}

	return 0;
}


int pdraw_audio_renderer_set_media_id(struct pdraw *pdraw,
				      struct pdraw_audio_renderer *renderer,
				      unsigned int media_id)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setMediaId(media_id);
}


unsigned int
pdraw_audio_renderer_get_media_id(struct pdraw *pdraw,
				  struct pdraw_audio_renderer *renderer)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

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
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(renderer == nullptr, EINVAL);

	return rnd->setParams(params);
}


int pdraw_audio_renderer_get_params(struct pdraw *pdraw,
				    struct pdraw_audio_renderer *renderer,
				    struct pdraw_audio_renderer_params *params)
{
	auto *rnd = reinterpret_cast<Pdraw::IPdraw::IAudioRenderer *>(renderer);

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
	std::unique_ptr<PdrawVipcSourceListener> vipcSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		vipcSourceListener = make_unique<PdrawVipcSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create VIPC source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVipcSource(
		params, vipcSourceListener.get(), &source);
	if (res < 0)
		return res;

	vipcSourceListener->setVipcSource(source);
	pdraw->vipcSourceListeners.push_back(std::move(vipcSourceListener));

	*ret_obj = reinterpret_cast<struct pdraw_vipc_source *>(source);
	return 0;
}


int pdraw_vipc_source_destroy(struct pdraw *pdraw,
			      struct pdraw_vipc_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->vipcSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVipcSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_vipc_source_is_ready_to_play(struct pdraw *pdraw,
				       struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_vipc_source_is_paused(struct pdraw *pdraw,
				struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isPaused() ? 1 : 0;
}


int pdraw_vipc_source_play(struct pdraw *pdraw,
			   struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_vipc_source_pause(struct pdraw *pdraw,
			    struct pdraw_vipc_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->pause();
}


int pdraw_vipc_source_configure(struct pdraw *pdraw,
				struct pdraw_vipc_source *source,
				const struct vdef_dim *resolution,
				const struct vdef_rectf *crop)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->configure(resolution, crop);
}


int pdraw_vipc_source_insert_grey_frame(struct pdraw *pdraw,
					struct pdraw_vipc_source *source,
					uint64_t ts_us)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->insertGreyFrame(ts_us);
}


int pdraw_vipc_source_set_session_metadata(struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_vipc_source_get_session_metadata(struct pdraw *pdraw,
					   struct pdraw_vipc_source *source,
					   struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(source);

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
	std::unique_ptr<PdrawCodedVideoSourceListener> videoSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSourceListener =
			make_unique<PdrawCodedVideoSourceListener>(
				pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create coded video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSource(
		params, videoSourceListener.get(), &source);
	if (res < 0)
		return res;

	videoSourceListener->setCodedVideoSource(source);
	pdraw->codedVideoSourceListeners.push_back(
		std::move(videoSourceListener));

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_source *>(source);
	return 0;
}


int pdraw_coded_video_source_destroy(struct pdraw *pdraw,
				     struct pdraw_coded_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->codedVideoSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getCodedVideoSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_source_get_queue(struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_coded_video_source_flush(struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_coded_video_source_drain(struct pdraw *pdraw,
				   struct pdraw_coded_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->drain();
}


int pdraw_coded_video_source_set_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_coded_video_source_get_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(source);

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
	std::unique_ptr<PdrawRawVideoSourceListener> videoSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSourceListener = make_unique<PdrawRawVideoSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSource(
		params, videoSourceListener.get(), &source);
	if (res < 0)
		return res;

	videoSourceListener->setRawVideoSource(source);
	pdraw->rawVideoSourceListeners.push_back(
		std::move(videoSourceListener));

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_source *>(source);
	return 0;
}


int pdraw_raw_video_source_destroy(struct pdraw *pdraw,
				   struct pdraw_raw_video_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->rawVideoSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getRawVideoSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_source_get_queue(struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_raw_video_source_flush(struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_raw_video_source_drain(struct pdraw *pdraw,
				 struct pdraw_raw_video_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->drain();
}


int pdraw_raw_video_source_set_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->setSessionMetadata(meta);
}


int pdraw_raw_video_source_get_session_metadata(
	struct pdraw *pdraw,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(source);

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
	std::unique_ptr<PdrawCodedVideoSinkListener> videoSinkListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flush == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		videoSinkListener = make_unique<PdrawCodedVideoSinkListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createCodedVideoSink(
		media_id, params, videoSinkListener.get(), &sink);
	if (res < 0)
		return res;

	videoSinkListener->setCodedVideoSink(sink);
	pdraw->codedVideoSinkListeners.push_back(std::move(videoSinkListener));

	*ret_obj = reinterpret_cast<struct pdraw_coded_video_sink *>(sink);
	return 0;
}


int pdraw_coded_video_sink_destroy(struct pdraw *pdraw,
				   struct pdraw_coded_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->codedVideoSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getCodedVideoSink() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_coded_video_sink_resync(struct pdraw *pdraw,
				  struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->resync();
}


int pdraw_coded_video_sink_set_media_id(struct pdraw *pdraw,
					struct pdraw_coded_video_sink *sink,
					unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int
pdraw_coded_video_sink_get_media_id(struct pdraw *pdraw,
				    struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	if (pdraw == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_coded_video_frame_queue *
pdraw_coded_video_sink_get_queue(struct pdraw *pdraw,
				 struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_coded_video_sink_queue_flushed(struct pdraw *pdraw,
					 struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_coded_video_sink_queue_drained(struct pdraw *pdraw,
					 struct pdraw_coded_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		videoSinkListener = make_unique<PdrawRawVideoSinkListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create raw video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createRawVideoSink(
		media_id, params, videoSinkListener.get(), &sink);
	if (res < 0)
		return res;

	videoSinkListener->setRawVideoSink(sink);
	pdraw->rawVideoSinkListeners.push_back(std::move(videoSinkListener));

	*ret_obj = reinterpret_cast<struct pdraw_raw_video_sink *>(sink);
	return 0;
}


int pdraw_raw_video_sink_destroy(struct pdraw *pdraw,
				 struct pdraw_raw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->rawVideoSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getRawVideoSink() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_raw_video_sink_set_media_id(struct pdraw *pdraw,
				      struct pdraw_raw_video_sink *sink,
				      unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int
pdraw_raw_video_sink_get_media_id(struct pdraw *pdraw,
				  struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	if (pdraw == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_raw_video_frame_queue *
pdraw_raw_video_sink_get_queue(struct pdraw *pdraw,
			       struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_raw_video_sink_queue_flushed(struct pdraw *pdraw,
				       struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_raw_video_sink_queue_drained(struct pdraw *pdraw,
				       struct pdraw_raw_video_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IRawVideoSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		alsaSourceListener = make_unique<PdrawAlsaSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create ALSA source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAlsaSource(
		params, alsaSourceListener.get(), &source);
	if (res < 0)
		return res;

	alsaSourceListener->setAlsaSource(source);
	pdraw->alsaSourceListeners.push_back(std::move(alsaSourceListener));

	*ret_obj = reinterpret_cast<struct pdraw_alsa_source *>(source);
	return 0;
}


int pdraw_alsa_source_destroy(struct pdraw *pdraw,
			      struct pdraw_alsa_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->alsaSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAlsaSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_alsa_source_is_ready_to_play(struct pdraw *pdraw,
				       struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isReadyToPlay() ? 1 : 0;
}


int pdraw_alsa_source_is_paused(struct pdraw *pdraw,
				struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	if (pdraw == nullptr)
		return 0;
	if (source == nullptr)
		return 0;

	return s->isPaused() ? 1 : 0;
}


int pdraw_alsa_source_play(struct pdraw *pdraw,
			   struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->play();
}


int pdraw_alsa_source_pause(struct pdraw *pdraw,
			    struct pdraw_alsa_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(source);

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
	std::unique_ptr<PdrawAudioSourceListener> audioSourceListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr || cbs->flushed == nullptr,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		audioSourceListener = make_unique<PdrawAudioSourceListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio source listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSource(
		params, audioSourceListener.get(), &source);
	if (res < 0)
		return res;

	audioSourceListener->setAudioSource(source);
	pdraw->audioSourceListeners.push_back(std::move(audioSourceListener));

	*ret_obj = reinterpret_cast<struct pdraw_audio_source *>(source);
	return 0;
}


int pdraw_audio_source_destroy(struct pdraw *pdraw,
			       struct pdraw_audio_source *source)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->audioSourceListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioSource() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


struct mbuf_audio_frame_queue *
pdraw_audio_source_get_queue(struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	if (pdraw == nullptr)
		return nullptr;
	if (source == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_audio_source_flush(struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

	return s->flush();
}


int pdraw_audio_source_drain(struct pdraw *pdraw,
			     struct pdraw_audio_source *source)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(source);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(source == nullptr, EINVAL);

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
		audioSinkListener = make_unique<PdrawAudioSinkListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioSink(
		media_id, audioSinkListener.get(), &sink);
	if (res < 0)
		return res;

	audioSinkListener->setAudioSink(sink);
	pdraw->audioSinkListeners.push_back(std::move(audioSinkListener));

	*ret_obj = reinterpret_cast<struct pdraw_audio_sink *>(sink);
	return 0;
}


int pdraw_audio_sink_destroy(struct pdraw *pdraw, struct pdraw_audio_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->audioSinkListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioSink() == s) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_audio_sink_set_media_id(struct pdraw *pdraw,
				  struct pdraw_audio_sink *sink,
				  unsigned int media_id)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->setMediaId(media_id);
}


unsigned int pdraw_audio_sink_get_media_id(struct pdraw *pdraw,
					   struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	if (pdraw == nullptr)
		return 0;
	if (s == nullptr)
		return 0;

	return s->getMediaId();
}


struct mbuf_audio_frame_queue *
pdraw_audio_sink_get_queue(struct pdraw *pdraw, struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	if (pdraw == nullptr)
		return nullptr;
	if (sink == nullptr)
		return nullptr;

	return s->getQueue();
}


int pdraw_audio_sink_queue_flushed(struct pdraw *pdraw,
				   struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

	return s->queueFlushed();
}


int pdraw_audio_sink_queue_drained(struct pdraw *pdraw,
				   struct pdraw_audio_sink *sink)
{
	auto *s = reinterpret_cast<Pdraw::IPdraw::IAudioSink *>(sink);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sink == nullptr, EINVAL);

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
		videoEncoderListener = make_unique<PdrawVideoEncoderListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoEncoder(
		media_id, params, videoEncoderListener.get(), &encoder);
	if (res < 0)
		return res;

	videoEncoderListener->setVideoEncoder(encoder);
	pdraw->videoEncoderListeners.push_back(std::move(videoEncoderListener));

	*ret_obj = reinterpret_cast<struct pdraw_video_encoder *>(encoder);
	return 0;
}


int pdraw_video_encoder_destroy(struct pdraw *pdraw,
				struct pdraw_video_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	/* The object must be destroyed before the listener */
	delete e;

	auto &listeners = pdraw->videoEncoderListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVideoEncoder() == e) {
			listeners.erase(it);
			break;
		}
	}

	return 0;
}


int pdraw_video_encoder_configure(struct pdraw *pdraw,
				  struct pdraw_video_encoder *encoder,
				  const struct venc_dyn_config *config)
{
	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->configure(config);
}


int pdraw_video_encoder_get_config(struct pdraw *pdraw,
				   struct pdraw_video_encoder *encoder,
				   struct venc_dyn_config *config)
{
	auto *e = reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	return e->getConfig(config);
}


int pdraw_video_encoder_request_key_frame(struct pdraw *pdraw,
					  struct pdraw_video_encoder *encoder)
{
	Pdraw::IPdraw::IVideoEncoder *e =
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(encoder);

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

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
		videoScalerListener = make_unique<PdrawVideoScalerListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create video scaler listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createVideoScaler(
		media_id, params, videoScalerListener.get(), &encoder);
	if (res < 0)
		return res;

	videoScalerListener->setVideoScaler(encoder);
	pdraw->videoScalerListeners.push_back(std::move(videoScalerListener));

	*ret_obj = reinterpret_cast<struct pdraw_video_scaler *>(encoder);
	return 0;
}


int pdraw_video_scaler_destroy(struct pdraw *pdraw,
			       struct pdraw_video_scaler *scaler)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(scaler == nullptr, EINVAL);

	auto *s = reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(scaler);

	/* The object must be destroyed before the listener */
	delete s;

	auto &listeners = pdraw->videoScalerListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getVideoScaler() == s) {
			listeners.erase(it);
			break;
		}
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
	std::unique_ptr<PdrawAudioEncoderListener> audioEncoderListener;

	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	try {
		audioEncoderListener = make_unique<PdrawAudioEncoderListener>(
			pdraw, cbs, userdata);
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create audio encoder listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->createAudioEncoder(
		media_id, params, audioEncoderListener.get(), &encoder);
	if (res < 0)
		return res;

	audioEncoderListener->setAudioEncoder(encoder);
	pdraw->audioEncoderListeners.push_back(std::move(audioEncoderListener));

	*ret_obj = reinterpret_cast<struct pdraw_audio_encoder *>(encoder);
	return 0;
}


int pdraw_audio_encoder_destroy(struct pdraw *pdraw,
				struct pdraw_audio_encoder *encoder)
{
	ULOG_ERRNO_RETURN_ERR_IF(pdraw == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(encoder == nullptr, EINVAL);

	auto *e = reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(encoder);

	/* The object must be destroyed before the listener */
	delete e;

	auto &listeners = pdraw->audioEncoderListeners;
	for (auto it = listeners.begin(); it != listeners.end(); ++it) {
		if ((*it)->getAudioEncoder() == e) {
			listeners.erase(it);
			break;
		}
	}

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
