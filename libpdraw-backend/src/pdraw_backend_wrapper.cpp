/**
 * Parrot Drones Awesome Video Viewer
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


class PdrawBackendListener : public PdrawBackend::IPdrawBackend::Listener {
public:
	PdrawBackendListener(struct pdraw_backend *pdraw,
			     const struct pdraw_backend_cbs *cbs,
			     void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata)
	{
	}

	~PdrawBackendListener() {}

	void openResponse(PdrawBackend::IPdrawBackend *pdraw, int status)
	{
		if (mCbs.open_resp) {
			(*mCbs.open_resp)(mPdraw, status, mUserdata);
		}
	}

	void closeResponse(PdrawBackend::IPdrawBackend *pdraw, int status)
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(mPdraw, status, mUserdata);
		}
	}

	void onUnrecoverableError(PdrawBackend::IPdrawBackend *pdraw)
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(mPdraw, mUserdata);
		}
	}

	int selectDemuxerMedia(PdrawBackend::IPdrawBackend *pdraw,
			       const struct pdraw_demuxer_media *medias,
			       size_t count)
	{
		if (mCbs.select_demuxer_media) {
			return (*mCbs.select_demuxer_media)(
				mPdraw, medias, count, mUserdata);
		}
		return -ENOSYS;
	}

	void onMediaAdded(PdrawBackend::IPdrawBackend *pdraw,
			  const struct pdraw_media_info *info)
	{
		if (mCbs.media_added) {
			(*mCbs.media_added)(mPdraw, info, mUserdata);
		}
	}

	void onMediaRemoved(PdrawBackend::IPdrawBackend *pdraw,
			    const struct pdraw_media_info *info)
	{
		if (mCbs.media_removed) {
			(*mCbs.media_removed)(mPdraw, info, mUserdata);
		}
	}

	void readyToPlay(PdrawBackend::IPdrawBackend *pdraw, bool ready)
	{
		if (mCbs.ready_to_play) {
			(*mCbs.ready_to_play)(mPdraw, ready ? 1 : 0, mUserdata);
		}
	}

	void onEndOfRange(PdrawBackend::IPdrawBackend *pdraw,
			  uint64_t timestamp)
	{
		if (mCbs.end_of_range) {
			(*mCbs.end_of_range)(mPdraw, timestamp, mUserdata);
		}
	}

	void playResponse(PdrawBackend::IPdrawBackend *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed)
	{
		if (mCbs.play_resp) {
			(*mCbs.play_resp)(
				mPdraw, status, timestamp, speed, mUserdata);
		}
	}

	void pauseResponse(PdrawBackend::IPdrawBackend *pdraw,
			   int status,
			   uint64_t timestamp)
	{
		if (mCbs.pause_resp) {
			(*mCbs.pause_resp)(
				mPdraw, status, timestamp, mUserdata);
		}
	}

	void seekResponse(PdrawBackend::IPdrawBackend *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed)
	{
		if (mCbs.seek_resp) {
			(*mCbs.seek_resp)(
				mPdraw, status, timestamp, speed, mUserdata);
		}
	}

	void onSocketCreated(PdrawBackend::IPdrawBackend *pdraw, int fd)
	{
		if (mCbs.socket_created)
			(*mCbs.socket_created)(mPdraw, fd, mUserdata);
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_cbs mCbs;
	void *mUserdata;
};


class PdrawBackendVideoRendererListener
		: public PdrawBackend::IPdrawBackend::VideoRendererListener {
public:
	PdrawBackendVideoRendererListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_renderer_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(NULL)
	{
	}

	~PdrawBackendVideoRendererListener() {}

	void onVideoRenderReady(PdrawBackend::IPdrawBackend *pdraw,
				struct pdraw_video_renderer *renderer)
	{
		if (mCbs.render_ready)
			(*mCbs.render_ready)(mPdraw, renderer, mUserdata);
	}

	int loadVideoTexture(PdrawBackend::IPdrawBackend *pdraw,
			     struct pdraw_video_renderer *renderer,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_session_info *sessionInfo,
			     const struct vmeta_session *sessionMeta,
			     const struct pdraw_video_frame *frame,
			     const void *frameUserdata,
			     size_t frameUserdataLen)
	{
		if (mCbs.load_texture == NULL)
			return -ENOSYS;
		return (*mCbs.load_texture)(mPdraw,
					    renderer,
					    textureWidth,
					    textureHeight,
					    sessionInfo,
					    sessionMeta,
					    frame,
					    frameUserdata,
					    frameUserdataLen,
					    mUserdata);
	}

	int renderVideoOverlay(PdrawBackend::IPdrawBackend *pdraw,
			       struct pdraw_video_renderer *renderer,
			       const struct pdraw_rect *renderPos,
			       const struct pdraw_rect *contentPos,
			       const float *viewMat,
			       const float *projMat,
			       const struct pdraw_session_info *sessionInfo,
			       const struct vmeta_session *sessionMeta,
			       const struct vmeta_frame *frameMeta,
			       const struct pdraw_video_frame_extra *frameExtra)
	{
		if (mCbs.render_overlay == NULL)
			return -ENOSYS;
		if ((renderer == NULL) || (renderPos == NULL) ||
		    (contentPos == NULL) || (viewMat == NULL) ||
		    (projMat == NULL) || (sessionInfo == NULL) ||
		    (sessionMeta == NULL) || (frameMeta == NULL))
			return -EINVAL;
		(*mCbs.render_overlay)(mPdraw,
				       renderer,
				       renderPos,
				       contentPos,
				       viewMat,
				       projMat,
				       sessionInfo,
				       sessionMeta,
				       frameMeta,
				       frameExtra,
				       mUserdata);
		return 0;
	}

	struct pdraw_video_renderer *getVideoRenderer()
	{
		return mRenderer;
	}

	void setVideoRenderer(struct pdraw_video_renderer *renderer)
	{
		mRenderer = renderer;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_renderer_cbs mCbs;
	void *mUserdata;
	struct pdraw_video_renderer *mRenderer;
};


class PdrawBackendVideoSinkListener
		: public PdrawBackend::IPdrawBackend::VideoSinkListener {
public:
	PdrawBackendVideoSinkListener(
		struct pdraw_backend *pdraw,
		const struct pdraw_backend_video_sink_cbs *cbs,
		void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(NULL)
	{
	}

	~PdrawBackendVideoSinkListener() {}

	void onVideoSinkFlush(PdrawBackend::IPdrawBackend *pdraw,
			      struct pdraw_video_sink *sink)
	{
		if (mCbs.flush)
			(*mCbs.flush)(mPdraw, mSink, mUserdata);
	}

	struct pdraw_video_sink *getVideoSink()
	{
		return mSink;
	}

	void setVideoSink(struct pdraw_video_sink *sink)
	{
		mSink = sink;
	}

private:
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_sink_cbs mCbs;
	void *mUserdata;
	struct pdraw_video_sink *mSink;
};


struct pdraw_backend {
	PdrawBackend::IPdrawBackend *pdraw;
	PdrawBackendListener *listener;
	std::vector<PdrawBackendVideoRendererListener *>
		*videoRendererListeners;
	std::vector<PdrawBackendVideoSinkListener *> *videoSinkListeners;
};


int pdraw_be_new(const struct pdraw_backend_cbs *cbs,
		 void *userdata,
		 struct pdraw_backend **ret_obj)
{
	int res = 0;
	struct pdraw_backend *self;

	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = (struct pdraw_backend *)calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->videoRendererListeners =
		new std::vector<PdrawBackendVideoRendererListener *>();
	if (self->videoRendererListeners == NULL) {
		res = -ENOMEM;
		goto error;
	}

	self->videoSinkListeners =
		new std::vector<PdrawBackendVideoSinkListener *>();
	if (self->videoSinkListeners == NULL) {
		res = -ENOMEM;
		goto error;
	}

	self->listener = new PdrawBackendListener(self, cbs, userdata);
	if (self->listener == NULL) {
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
	pdraw_be_destroy(self);
	*ret_obj = NULL;
	return res;
}


int pdraw_be_destroy(struct pdraw_backend *self)
{
	if (self == NULL)
		return 0;

	if (self->pdraw != NULL) {
		self->pdraw->stop();
		delete self->pdraw;
		self->pdraw = NULL;
	}

	if (self->listener != NULL)
		delete self->listener;

	if (self->videoSinkListeners != NULL) {
		std::vector<PdrawBackendVideoSinkListener *>::iterator l =
			self->videoSinkListeners->begin();
		while (l != self->videoSinkListeners->end()) {
			if (*l != NULL)
				delete (*l);
			l++;
		}
		self->videoSinkListeners->clear();
		delete self->videoSinkListeners;
	}

	if (self->videoRendererListeners != NULL) {
		std::vector<PdrawBackendVideoRendererListener *>::iterator r =
			self->videoRendererListeners->begin();
		while (r != self->videoRendererListeners->end()) {
			if (*r != NULL)
				delete (*r);
			r++;
		}
		self->videoRendererListeners->clear();
		delete self->videoRendererListeners;
	}

	free(self);
	return 0;
}


struct pomp_loop *pdraw_be_get_loop(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->pdraw->getLoop();
}


int pdraw_be_open_url(struct pdraw_backend *self, const char *url)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string u(url ? url : "");
	return self->pdraw->open(u);
}


int pdraw_be_open_single_stream(struct pdraw_backend *self,
				const char *local_addr,
				uint16_t local_stream_port,
				uint16_t local_control_port,
				const char *remote_addr,
				uint16_t remote_stream_port,
				uint16_t remote_control_port)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string local(local_addr ? local_addr : "");
	std::string remote(remote_addr ? remote_addr : "");
	return self->pdraw->open(local,
				 local_stream_port,
				 local_control_port,
				 remote,
				 remote_stream_port,
				 remote_control_port);
}


int pdraw_be_open_url_mux(struct pdraw_backend *self,
			  const char *url,
			  struct mux_ctx *mux)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string u(url ? url : "");
	return self->pdraw->open(u, mux);
}


int pdraw_be_close(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->close();
}


uint16_t
pdraw_be_get_single_stream_local_stream_port(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);

	return self->pdraw->getSingleStreamLocalStreamPort();
}


uint16_t
pdraw_be_get_single_stream_local_control_port(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);

	return self->pdraw->getSingleStreamLocalControlPort();
}


int pdraw_be_play(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->play();
}


int pdraw_be_play_with_speed(struct pdraw_backend *self, float speed)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->play(speed);
}


int pdraw_be_is_ready_to_play(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return (self->pdraw->isReadyToPlay()) ? 1 : 0;
}


int pdraw_be_pause(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->pause();
}


int pdraw_be_is_paused(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return (self->pdraw->isPaused()) ? 1 : 0;
}


int pdraw_be_previous_frame(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->previousFrame();
}


int pdraw_be_next_frame(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->nextFrame();
}


int pdraw_be_seek(struct pdraw_backend *self, int64_t delta, int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->seek(delta, exact ? true : false);
}


int pdraw_be_seek_forward(struct pdraw_backend *self, uint64_t delta, int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->seekForward(delta, exact ? true : false);
}


int pdraw_be_seek_back(struct pdraw_backend *self, uint64_t delta, int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->seekBack(delta, exact ? true : false);
}


int pdraw_be_seek_to(struct pdraw_backend *self, uint64_t timestamp, int exact)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->seekTo(timestamp, exact ? true : false);
}


uint64_t pdraw_be_get_duration(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);

	return self->pdraw->getDuration();
}


uint64_t pdraw_be_get_current_time(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);

	return self->pdraw->getCurrentTime();
}


int pdraw_be_start_video_renderer(
	struct pdraw_backend *self,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_backend_video_renderer_cbs *cbs,
	void *userdata,
	struct pdraw_video_renderer **ret_obj)
{
	return pdraw_be_start_video_renderer_egl(
		self, render_pos, params, cbs, userdata, NULL, ret_obj);
}


int pdraw_be_start_video_renderer_egl(
	struct pdraw_backend *self,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_backend_video_renderer_cbs *cbs,
	void *userdata,
	struct egl_display *egl_display,
	struct pdraw_video_renderer **ret_obj)
{
	int res;
	struct pdraw_video_renderer *renderer = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	PdrawBackendVideoRendererListener *l =
		new PdrawBackendVideoRendererListener(self, cbs, userdata);
	if (l == NULL) {
		ULOGE("failed to create video renderer listener");
		return -ENOMEM;
	}

	res = self->pdraw->startVideoRenderer(
		render_pos, params, l, &renderer, egl_display);
	if (res < 0) {
		delete l;
		return res;
	}

	self->videoRendererListeners->push_back(l);
	l->setVideoRenderer(renderer);

	*ret_obj = renderer;
	return 0;
}


int pdraw_be_stop_video_renderer(struct pdraw_backend *self,
				 struct pdraw_video_renderer *renderer)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	res = self->pdraw->stopVideoRenderer(renderer);
	if (res < 0)
		return res;

	std::vector<PdrawBackendVideoRendererListener *>::iterator l =
		self->videoRendererListeners->begin();
	while (l != self->videoRendererListeners->end()) {
		if ((*l)->getVideoRenderer() != renderer) {
			l++;
			continue;
		}
		delete *l;
		self->videoRendererListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_resize_video_renderer(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   const struct pdraw_rect *render_pos)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->resizeVideoRenderer(renderer, render_pos);
}


int pdraw_be_set_video_renderer_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->setVideoRendererParams(renderer, params);
}


int pdraw_be_get_video_renderer_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	struct pdraw_video_renderer_params *params)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->getVideoRendererParams(renderer, params);
}


int pdraw_be_render_video(struct pdraw_backend *self,
			  struct pdraw_video_renderer *renderer,
			  struct pdraw_rect *content_pos)
{
	return pdraw_be_render_video_mat(
		self, renderer, content_pos, NULL, NULL);
}


int pdraw_be_render_video_mat(struct pdraw_backend *self,
			      struct pdraw_video_renderer *renderer,
			      struct pdraw_rect *content_pos,
			      const float *view_mat,
			      const float *proj_mat)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->renderVideo(
		renderer, content_pos, view_mat, proj_mat);
}


int pdraw_be_start_video_sink(struct pdraw_backend *self,
			      unsigned int media_id,
			      const struct pdraw_video_sink_params *params,
			      const struct pdraw_backend_video_sink_cbs *cbs,
			      void *userdata,
			      struct pdraw_video_sink **ret_obj)
{
	int res;
	struct pdraw_video_sink *sink = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->flush == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	PdrawBackendVideoSinkListener *l =
		new PdrawBackendVideoSinkListener(self, cbs, userdata);
	if (l == NULL) {
		ULOGE("failed to create video sink listener");
		return -ENOMEM;
	}

	res = self->pdraw->startVideoSink(media_id, params, l, &sink);
	if (res < 0) {
		delete l;
		return res;
	}

	l->setVideoSink(sink);
	self->videoSinkListeners->push_back(l);

	*ret_obj = sink;
	return 0;
}


int pdraw_be_stop_video_sink(struct pdraw_backend *self,
			     struct pdraw_video_sink *sink)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	res = self->pdraw->stopVideoSink(sink);
	if (res < 0) {
		ULOG_ERRNO("stopVideoSink", -res);
		return res;
	}

	std::vector<PdrawBackendVideoSinkListener *>::iterator l =
		self->videoSinkListeners->begin();
	while (l != self->videoSinkListeners->end()) {
		if ((*l)->getVideoSink() != sink) {
			l++;
			continue;
		}
		delete *l;
		self->videoSinkListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_be_resync_video_sink(struct pdraw_backend *self,
			       struct pdraw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->resyncVideoSink(sink);
}


struct vbuf_queue *pdraw_be_get_video_sink_queue(struct pdraw_backend *self,
						 struct pdraw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->pdraw->getVideoSinkQueue(sink);
}


int pdraw_be_video_sink_queue_flushed(struct pdraw_backend *self,
				      struct pdraw_video_sink *sink)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->pdraw->videoSinkQueueFlushed(sink);
}


enum pdraw_session_type pdraw_be_get_session_type(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, PDRAW_SESSION_TYPE_UNKNOWN);

	return self->pdraw->getSessionType();
}


int pdraw_be_get_self_friendly_name(struct pdraw_backend *self,
				    char *str,
				    size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string fn;
	self->pdraw->getSelfFriendlyName(&fn);
	if ((str) && (fn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, fn.c_str());
	return 0;
}


int pdraw_be_set_self_friendly_name(struct pdraw_backend *self,
				    const char *friendly_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(friendly_name == NULL, EINVAL);

	std::string fn(friendly_name);
	self->pdraw->setSelfFriendlyName(fn);
	return 0;
}


int pdraw_be_get_self_serial_number(struct pdraw_backend *self,
				    char *str,
				    size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string sn;
	self->pdraw->getSelfSerialNumber(&sn);
	if ((str) && (sn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sn.c_str());
	return 0;
}


int pdraw_be_set_self_serial_number(struct pdraw_backend *self,
				    const char *serial_number)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(serial_number == NULL, EINVAL);

	std::string sn(serial_number);
	self->pdraw->setSelfSerialNumber(sn);
	return 0;
}


int pdraw_be_get_self_software_version(struct pdraw_backend *self,
				       char *str,
				       size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	std::string sv;
	self->pdraw->getSelfSoftwareVersion(&sv);
	if ((str) && (sv.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sv.c_str());
	return 0;
}


int pdraw_be_set_self_software_version(struct pdraw_backend *self,
				       const char *software_version)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(software_version == NULL, EINVAL);

	std::string sv(software_version);
	self->pdraw->setSelfSoftwareVersion(sv);
	return 0;
}


int pdraw_be_is_self_pilot(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return (self->pdraw->isSelfPilot()) ? 1 : 0;
}


int pdraw_be_set_self_pilot(struct pdraw_backend *self, int is_pilot)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->setSelfPilot((is_pilot) ? true : false);
	return 0;
}


int pdraw_be_get_peer_session_metadata(struct pdraw_backend *self,
				       struct vmeta_session *session)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->getPeerSessionMetadata(session);
	return 0;
}


enum pdraw_drone_model pdraw_be_get_peer_drone_model(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, PDRAW_DRONE_MODEL_UNKNOWN);

	return self->pdraw->getPeerDroneModel();
}


enum pdraw_pipeline_mode
pdraw_be_get_pipeline_mode_setting(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, PDRAW_PIPELINE_MODE_DECODE_ALL);

	return self->pdraw->getPipelineModeSetting();
}


int pdraw_be_set_pipeline_mode_setting(struct pdraw_backend *self,
				       enum pdraw_pipeline_mode mode)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->setPipelineModeSetting(mode);
	return 0;
}


int pdraw_be_get_display_screen_settings(struct pdraw_backend *self,
					 float *xdpi,
					 float *ydpi,
					 float *device_margin_top,
					 float *device_margin_bottom,
					 float *device_margin_left,
					 float *device_margin_right)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->getDisplayScreenSettings(xdpi,
					      ydpi,
					      device_margin_top,
					      device_margin_bottom,
					      device_margin_left,
					      device_margin_right);
	return 0;
}


int pdraw_be_set_display_screen_settings(struct pdraw_backend *self,
					 float xdpi,
					 float ydpi,
					 float device_margin_top,
					 float device_margin_bottom,
					 float device_margin_left,
					 float device_margin_right)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->setDisplayScreenSettings(xdpi,
					      ydpi,
					      device_margin_top,
					      device_margin_bottom,
					      device_margin_left,
					      device_margin_right);
	return 0;
}


enum pdraw_hmd_model pdraw_be_get_hmd_model_setting(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, PDRAW_HMD_MODEL_UNKNOWN);

	return self->pdraw->getHmdModelSetting();
}


int pdraw_be_set_hmd_model_setting(struct pdraw_backend *self,
				   enum pdraw_hmd_model hmd_model)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->setHmdModelSetting(hmd_model);
	return 0;
}


int pdraw_be_set_android_jvm(struct pdraw_backend *self, void *jvm)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->pdraw->setAndroidJvm(jvm);
	return 0;
}
