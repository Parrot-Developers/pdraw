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
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_demuxer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IDemuxer *mDemuxer;
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
	struct pdraw_backend *mPdraw;
	struct pdraw_backend_video_renderer_cbs mCbs;
	void *mUserdata;
	Pdraw::IPdraw::IVideoRenderer *mRenderer;
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


struct pdraw_backend {
	PdrawBackend::IPdrawBackend *pdraw;
	PdrawBackendListener *listener;
	std::vector<PdrawBackendDemuxerListener *> *demuxerListeners;
	std::vector<PdrawBackendVideoRendererListener *>
		*videoRendererListeners;
	std::vector<PdrawBackendCodedVideoSinkListener *>
		*codedVideoSinkListeners;
	std::vector<PdrawBackendRawVideoSinkListener *> *rawVideoSinkListeners;
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

	self->videoRendererListeners =
		new std::vector<PdrawBackendVideoRendererListener *>();
	if (self->videoRendererListeners == nullptr) {
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
	pdraw_be_destroy(self);
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
				  const struct pdraw_backend_demuxer_cbs *cbs,
				  void *userdata,
				  struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendDemuxerListener *l =
		new PdrawBackendDemuxerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, l, &demuxer);
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
	const struct pdraw_backend_demuxer_cbs *cbs,
	void *userdata,
	struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
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
	const struct pdraw_backend_demuxer_cbs *cbs,
	void *userdata,
	struct pdraw_demuxer **ret_obj)
{
	int res;
	Pdraw::IPdraw::IDemuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

	PdrawBackendDemuxerListener *l =
		new PdrawBackendDemuxerListener(self, cbs, userdata);
	if (l == nullptr) {
		ULOGE("failed to create demuxer listener");
		return -ENOMEM;
	}

	std::string u(url ? url : "");
	res = self->pdraw->createDemuxer(u, mux, l, &demuxer);
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
		       struct pdraw_muxer **ret_obj)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == nullptr, EINVAL);

#if MUXER_TEST
	int res;
	Pdraw::IPdraw::IMuxer *muxer = nullptr;

	std::string u(url ? url : "");

	res = self->pdraw->createMuxer(u, &muxer);
	if (res < 0)
		return res;

	*ret_obj = reinterpret_cast<struct pdraw_muxer *>(muxer);
	return 0;
#else
	/* Not supported yet */
	return -ENOSYS;
#endif
}


int pdraw_be_muxer_destroy(struct pdraw_backend *self,
			   struct pdraw_muxer *muxer)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	delete m;

	return 0;
}


int pdraw_be_muxer_add_media(
	struct pdraw_backend *self,
	struct pdraw_muxer *muxer,
	unsigned int media_id,
	const struct pdraw_muxer_video_media_params *params)
{
	Pdraw::IPdraw::IMuxer *m =
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(muxer);

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(muxer == nullptr, EINVAL);

	return m->addMedia(media_id, params);
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
	return pdraw_be_video_renderer_new_egl(self,
					       media_id,
					       render_pos,
					       params,
					       cbs,
					       userdata,
					       nullptr,
					       ret_obj);
}


int pdraw_be_video_renderer_new_egl(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_backend_video_renderer_cbs *cbs,
	void *userdata,
	struct egl_display *egl_display,
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
		media_id, render_pos, params, l, &renderer, egl_display);
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


enum pdraw_pipeline_mode
pdraw_be_get_pipeline_mode_setting(struct pdraw_backend *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == nullptr, EINVAL, PDRAW_PIPELINE_MODE_DECODE_ALL);

	return self->pdraw->getPipelineModeSetting();
}


int pdraw_be_set_pipeline_mode_setting(struct pdraw_backend *self,
				       enum pdraw_pipeline_mode mode)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

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
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

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
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

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
	ULOG_ERRNO_RETURN_VAL_IF(
		self == nullptr, EINVAL, PDRAW_HMD_MODEL_UNKNOWN);

	return self->pdraw->getHmdModelSetting();
}


int pdraw_be_set_hmd_model_setting(struct pdraw_backend *self,
				   enum pdraw_hmd_model hmd_model)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	self->pdraw->setHmdModelSetting(hmd_model);
	return 0;
}


int pdraw_be_set_android_jvm(struct pdraw_backend *self, void *jvm)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	self->pdraw->setAndroidJvm(jvm);
	return 0;
}


int pdraw_be_dump_pipeline(struct pdraw_backend *self, const char *file_name)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	std::string f(file_name ? file_name : "");
	return self->pdraw->dumpPipeline(f);
}
