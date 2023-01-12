/**
 * Parrot Drones Awesome Video Viewer Library
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
			  const struct pdraw_media_info *info)
	{
		if (mCbs.media_added) {
			(*mCbs.media_added)(mPdraw, info, mUserdata);
		}
	}

	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info)
	{
		if (mCbs.media_removed) {
			(*mCbs.media_removed)(mPdraw, info, mUserdata);
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
			       size_t count)
	{
		if (mCbs.select_media) {
			return (*mCbs.select_media)(
				mPdraw,
				reinterpret_cast<struct pdraw_demuxer *>(
					demuxer),
				medias,
				count,
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
				    const struct pdraw_media_info *info)
	{
		if (mCbs.media_removed)
			(*mCbs.media_removed)(
				mPdraw,
				reinterpret_cast<struct pdraw_video_renderer *>(
					renderer),
				info,
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
		if ((renderer == nullptr) || (mediaInfo == nullptr) ||
		    (frame == nullptr))
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
		if ((renderer == nullptr) || (renderPos == nullptr) ||
		    (contentPos == nullptr) || (viewMat == nullptr) ||
		    (projMat == nullptr) || (mediaInfo == nullptr))
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


struct pdraw {
	Pdraw::IPdraw *pdraw;
	PdrawListener *listener;
	pthread_mutex_t mutex;
	std::vector<PdrawDemuxerListener *> *demuxerListeners;
	std::vector<PdrawVideoRendererListener *> *videoRendererListeners;
	std::vector<PdrawCodedVideoSinkListener *> *codedVideoSinkListeners;
	std::vector<PdrawRawVideoSinkListener *> *rawVideoSinkListeners;
};


int pdraw_new(struct pomp_loop *loop,
	      const struct pdraw_cbs *cbs,
	      void *userdata,
	      struct pdraw **ret_obj)
{
	int ret = 0;
	struct pdraw *pdraw;

	if (loop == nullptr)
		return -EINVAL;
	if (cbs == nullptr)
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

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

	pdraw->videoRendererListeners =
		new std::vector<PdrawVideoRendererListener *>();
	if (pdraw->videoRendererListeners == nullptr) {
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
	if (pdraw == nullptr)
		return -EINVAL;
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
	pthread_mutex_unlock(&pdraw->mutex);
	pthread_mutex_destroy(&pdraw->mutex);

	free(pdraw);
	return 0;
}


int pdraw_stop(struct pdraw *pdraw)
{
	if (pdraw == nullptr)
		return -EINVAL;

	return pdraw->pdraw->stop();
}


int pdraw_demuxer_new_from_url(struct pdraw *pdraw,
			       const char *url,
			       const struct pdraw_demuxer_cbs *cbs,
			       void *userdata,
			       struct pdraw_demuxer **ret_obj)
{
	return pdraw_demuxer_new_from_url_on_mux(
		pdraw, url, nullptr, cbs, userdata, ret_obj);
}


int pdraw_demuxer_new_single_stream(struct pdraw *pdraw,
				    const char *local_addr,
				    uint16_t local_stream_port,
				    uint16_t local_control_port,
				    const char *remote_addr,
				    uint16_t remote_stream_port,
				    uint16_t remote_control_port,
				    const struct pdraw_demuxer_cbs *cbs,
				    void *userdata,
				    struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;

	if (pdraw == nullptr)
		return -EINVAL;
	if (local_addr == nullptr)
		return -EINVAL;
	if (remote_addr == nullptr)
		return -EINVAL;
	if (cbs == nullptr)
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

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
				      const struct pdraw_demuxer_cbs *cbs,
				      void *userdata,
				      struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;

	if (pdraw == nullptr)
		return -EINVAL;
	if (url == nullptr)
		return -EINVAL;
	/* Note: deliberately not testing the mux pointer, as
	 * pdraw_demuxer_new_from_url() calls this function with
	 * a NULL mux pointer */
	if (cbs == nullptr)
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

	PdrawDemuxerListener *demuxerListener =
		new PdrawDemuxerListener(pdraw, cbs, userdata);
	if (demuxerListener == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url);
	res = pdraw->pdraw->createDemuxer(u, mux, demuxerListener, &demuxer);
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

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

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

	delete d;

	return 0;
}


int pdraw_demuxer_close(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->close();
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
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return (d->isReadyToPlay()) ? 1 : 0;
}


int pdraw_demuxer_is_paused(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return (d->isPaused()) ? 1 : 0;
}


int pdraw_demuxer_play(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->play();
}


int pdraw_demuxer_play_with_speed(struct pdraw *pdraw,
				  struct pdraw_demuxer *demuxer,
				  float speed)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->play(speed);
}


int pdraw_demuxer_pause(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->pause();
}


int pdraw_demuxer_previous_frame(struct pdraw *pdraw,
				 struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->previousFrame();
}


int pdraw_demuxer_next_frame(struct pdraw *pdraw, struct pdraw_demuxer *demuxer)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->nextFrame();
}


int pdraw_demuxer_seek(struct pdraw *pdraw,
		       struct pdraw_demuxer *demuxer,
		       int64_t delta,
		       int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->seek(delta, exact ? true : false);
}


int pdraw_demuxer_seek_forward(struct pdraw *pdraw,
			       struct pdraw_demuxer *demuxer,
			       uint64_t delta,
			       int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->seekForward(delta, exact ? true : false);
}


int pdraw_demuxer_seek_back(struct pdraw *pdraw,
			    struct pdraw_demuxer *demuxer,
			    uint64_t delta,
			    int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->seekBack(delta, exact ? true : false);
}


int pdraw_demuxer_seek_to(struct pdraw *pdraw,
			  struct pdraw_demuxer *demuxer,
			  uint64_t timestamp,
			  int exact)
{
	Pdraw::IPdraw::IDemuxer *d =
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(demuxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (demuxer == nullptr)
		return -EINVAL;

	return d->seekTo(timestamp, exact ? true : false);
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
		    struct pdraw_muxer **ret_obj)
{
	if (pdraw == nullptr)
		return -EINVAL;
	if (url == nullptr)
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

#if MUXER_TEST
	int res;
	Pdraw::IPdraw::IMuxer *muxer = nullptr;

	std::string u(url);
	res = pdraw->pdraw->createMuxer(u, &muxer);
	if (res < 0)
		return res;

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
#else
	/* Not supported yet */
	return -ENOSYS;
#endif
}


int pdraw_muxer_destroy(struct pdraw *pdraw, struct pdraw_muxer *muxer)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (muxer == nullptr)
		return -EINVAL;

	delete m;

	return 0;
}


int pdraw_muxer_add_media(struct pdraw *pdraw,
			  struct pdraw_muxer *muxer,
			  unsigned int media_id,
			  const struct pdraw_muxer_video_media_params *params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (muxer == nullptr)
		return -EINVAL;

	return m->addMedia(media_id, params);
}


int pdraw_video_renderer_new(struct pdraw *pdraw,
			     unsigned int media_id,
			     const struct pdraw_rect *render_pos,
			     const struct pdraw_video_renderer_params *params,
			     const struct pdraw_video_renderer_cbs *cbs,
			     void *userdata,
			     struct pdraw_video_renderer **ret_obj)
{
	return pdraw_video_renderer_new_egl(pdraw,
					    media_id,
					    render_pos,
					    params,
					    cbs,
					    userdata,
					    nullptr,
					    ret_obj);
}


int pdraw_video_renderer_new_egl(
	struct pdraw *pdraw,
	unsigned int media_id,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_video_renderer_cbs *cbs,
	void *userdata,
	struct egl_display *egl_display,
	struct pdraw_video_renderer **ret_obj)
{
	int ret = 0;
	Pdraw::IPdraw::IVideoRenderer *renderer = nullptr;
	if (pdraw == nullptr)
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&pdraw->mutex);
	PdrawVideoRendererListener *l =
		new PdrawVideoRendererListener(pdraw, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create video renderer listener");
		ret = -ENOMEM;
		goto error;
	}

	ret = pdraw->pdraw->createVideoRenderer(
		media_id, render_pos, params, l, &renderer, egl_display);
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

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

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

	delete rnd;

	return 0;
}


int pdraw_video_renderer_resize(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *render_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

	return rnd->resize(render_pos);
}


int pdraw_video_renderer_set_media_id(struct pdraw *pdraw,
				      struct pdraw_video_renderer *renderer,
				      unsigned int media_id)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

	return rnd->setParams(params);
}


int pdraw_video_renderer_get_params(struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

	return rnd->getParams(params);
}


int pdraw_video_renderer_render(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				struct pdraw_rect *content_pos)
{
	Pdraw::IPdraw::IVideoRenderer *rnd =
		reinterpret_cast<Pdraw::IPdraw::IVideoRenderer *>(renderer);

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (renderer == nullptr)
		return -EINVAL;

	return rnd->render(content_pos, view_mat, proj_mat);
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

	if (pdraw == nullptr)
		return -EINVAL;
	if (params == nullptr)
		return -EINVAL;
	if ((cbs == nullptr) || (cbs->flush == nullptr))
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (sink == nullptr)
		return -EINVAL;

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

	delete s;

	return 0;
}


int pdraw_coded_video_sink_resync(struct pdraw *pdraw,
				  struct pdraw_coded_video_sink *sink)
{
	Pdraw::IPdraw::ICodedVideoSink *s =
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSink *>(sink);

	if (pdraw == nullptr)
		return -EINVAL;
	if (sink == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (sink == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (params == nullptr)
		return -EINVAL;
	if ((cbs == nullptr) || (cbs->flush == nullptr))
		return -EINVAL;
	if (ret_obj == nullptr)
		return -EINVAL;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (sink == nullptr)
		return -EINVAL;

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

	delete s;

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

	if (pdraw == nullptr)
		return -EINVAL;
	if (sink == nullptr)
		return -EINVAL;

	return s->queueFlushed();
}


int pdraw_get_friendly_name_setting(struct pdraw *pdraw, char *str, size_t len)
{
	if (pdraw == nullptr)
		return -EINVAL;

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
	if (pdraw == nullptr)
		return -EINVAL;

	std::string fn(friendly_name);
	pdraw->pdraw->setFriendlyNameSetting(fn);
	return 0;
}


int pdraw_get_serial_number_setting(struct pdraw *pdraw, char *str, size_t len)
{
	if (pdraw == nullptr)
		return -EINVAL;

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
	if (pdraw == nullptr)
		return -EINVAL;

	std::string sn(serial_number);
	pdraw->pdraw->setSerialNumberSetting(sn);
	return 0;
}


int pdraw_get_software_version_setting(struct pdraw *pdraw,
				       char *str,
				       size_t len)
{
	if (pdraw == nullptr)
		return -EINVAL;

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
	if (pdraw == nullptr)
		return -EINVAL;

	std::string sv(software_version);
	pdraw->pdraw->setSoftwareVersionSetting(sv);
	return 0;
}


enum pdraw_pipeline_mode pdraw_get_pipeline_mode_setting(struct pdraw *pdraw)
{
	if (pdraw == nullptr)
		return (enum pdraw_pipeline_mode)(-EINVAL);

	return pdraw->pdraw->getPipelineModeSetting();
}


int pdraw_set_pipeline_mode_setting(struct pdraw *pdraw,
				    enum pdraw_pipeline_mode mode)
{
	if (pdraw == nullptr)
		return -EINVAL;

	pdraw->pdraw->setPipelineModeSetting(mode);
	return 0;
}


int pdraw_get_display_screen_settings(struct pdraw *pdraw,
				      float *xdpi,
				      float *ydpi,
				      float *device_margin_top,
				      float *device_margin_bottom,
				      float *device_margin_left,
				      float *device_margin_right)
{
	if (pdraw == nullptr)
		return -EINVAL;

	pdraw->pdraw->getDisplayScreenSettings(xdpi,
					       ydpi,
					       device_margin_top,
					       device_margin_bottom,
					       device_margin_left,
					       device_margin_right);
	return 0;
}


int pdraw_set_display_screen_settings(struct pdraw *pdraw,
				      float xdpi,
				      float ydpi,
				      float device_margin_top,
				      float device_margin_bottom,
				      float device_margin_left,
				      float device_margin_right)
{
	if (pdraw == nullptr)
		return -EINVAL;

	pdraw->pdraw->setDisplayScreenSettings(xdpi,
					       ydpi,
					       device_margin_top,
					       device_margin_bottom,
					       device_margin_left,
					       device_margin_right);
	return 0;
}


enum pdraw_hmd_model pdraw_get_hmd_model_setting(struct pdraw *pdraw)
{
	if (pdraw == nullptr)
		return (enum pdraw_hmd_model)(-EINVAL);

	return pdraw->pdraw->getHmdModelSetting();
}


int pdraw_set_hmd_model_setting(struct pdraw *pdraw,
				enum pdraw_hmd_model hmd_model)
{
	if (pdraw == nullptr)
		return -EINVAL;

	pdraw->pdraw->setHmdModelSetting(hmd_model);
	return 0;
}


int pdraw_set_android_jvm(struct pdraw *pdraw, void *jvm)
{
	if (pdraw == nullptr)
		return -EINVAL;

	pdraw->pdraw->setAndroidJvm(jvm);
	return 0;
}


int pdraw_dump_pipeline(struct pdraw *pdraw, const char *file_name)
{
	if ((pdraw == nullptr) || (file_name == nullptr))
		return -EINVAL;

	std::string f(file_name);
	return pdraw->pdraw->dumpPipeline(f);
}


const char *pdraw_hmd_model_str(enum pdraw_hmd_model val)
{
	return pdraw_hmdModelStr(val);
}


const char *pdraw_pipeline_mode_str(enum pdraw_pipeline_mode val)
{
	return pdraw_pipelineModeStr(val);
}


const char *pdraw_playback_type_str(enum pdraw_playback_type val)
{
	return pdraw_playbackTypeStr(val);
}


const char *pdraw_media_type_str(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


const char *pdraw_video_type_str(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


const char *pdraw_histogram_channel_str(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
}


const char *pdraw_video_renderer_scheduling_mode_str(
	enum pdraw_video_renderer_scheduling_mode val)
{
	return pdraw_videoRendererSchedulingModeStr(val);
}


const char *
pdraw_video_renderer_fill_mode_str(enum pdraw_video_renderer_fill_mode val)
{
	return pdraw_videoRendererFillModeStr(val);
}


const char *pdraw_video_renderer_transition_flag_str(
	enum pdraw_video_renderer_transition_flag val)
{
	return pdraw_videoRendererTransitionFlagStr(val);
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
