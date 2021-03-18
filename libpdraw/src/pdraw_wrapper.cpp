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

#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <errno.h>
#include <pthread.h>

#include <string>

#include <pdraw/pdraw.h>
#define ULOG_TAG pdraw_wrapper
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_wrapper);


/* codecheck_ignore[COMPLEX_MACRO] */
#define ENUM_CASE(_prefix, _name)                                              \
	case _prefix##_name:                                                   \
		return #_name


/* NOTE: Pdraw::VideoMedia::Format and enum pdraw_video_media_format values
 * must be synchronized */
PDRAW_STATIC_ASSERT((int)Pdraw::VideoMedia::Format::FORMAT_UNKNOWN ==
		    PDRAW_VIDEO_MEDIA_FORMAT_UNKNOWN);
PDRAW_STATIC_ASSERT((int)Pdraw::VideoMedia::Format::YUV ==
		    PDRAW_VIDEO_MEDIA_FORMAT_YUV);
PDRAW_STATIC_ASSERT((int)Pdraw::VideoMedia::Format::H264 ==
		    PDRAW_VIDEO_MEDIA_FORMAT_H264);
PDRAW_STATIC_ASSERT((int)Pdraw::VideoMedia::Format::OPAQUE ==
		    PDRAW_VIDEO_MEDIA_FORMAT_OPAQUE);
#include <SViewData.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
boost::shared_ptr<SViewData> _frameViewData;
boost::mutex sviewMutex;
bool _viewDataInitialized = false;
//vmeta_frame* dgdframeMeta;
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

	void openResponse(Pdraw::IPdraw *pdraw, int status)
	{
		if (mCbs.open_resp) {
			(*mCbs.open_resp)(mPdraw, status, mUserdata);
		}
	}


	void closeResponse(Pdraw::IPdraw *pdraw, int status)
	{
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(mPdraw, status, mUserdata);
		}
	}

	void onUnrecoverableError(Pdraw::IPdraw *pdraw)
	{
		if (mCbs.unrecoverable_error) {
			(*mCbs.unrecoverable_error)(mPdraw, mUserdata);
		}
	}

	int selectDemuxerMedia(Pdraw::IPdraw *pdraw,
			       const struct pdraw_demuxer_media *medias,
			       size_t count)
	{
		if (mCbs.select_demuxer_media) {
			return (*mCbs.select_demuxer_media)(
				mPdraw, medias, count, mUserdata);
		}
		return -ENOSYS;
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

	void readyToPlay(Pdraw::IPdraw *pdraw, bool ready)
	{
		if (mCbs.ready_to_play) {
			(*mCbs.ready_to_play)(mPdraw, ready ? 1 : 0, mUserdata);
		}
	}

	void onEndOfRange(Pdraw::IPdraw *pdraw, uint64_t timestamp)
	{
		if (mCbs.end_of_range) {
			(*mCbs.end_of_range)(mPdraw, timestamp, mUserdata);
		}
	}

	void playResponse(Pdraw::IPdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed)
	{
		if (mCbs.play_resp) {
			(*mCbs.play_resp)(
				mPdraw, status, timestamp, speed, mUserdata);
		}
	}

	void pauseResponse(Pdraw::IPdraw *pdraw, int status, uint64_t timestamp)
	{
		if (mCbs.pause_resp) {
			(*mCbs.pause_resp)(
				mPdraw, status, timestamp, mUserdata);
		}
	}

	void seekResponse(Pdraw::IPdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed)
	{
		if (mCbs.seek_resp) {
			(*mCbs.seek_resp)(
				mPdraw, status, timestamp, speed, mUserdata);
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


class PdrawVideoRendererListener : public Pdraw::IPdraw::VideoRendererListener {
public:
	PdrawVideoRendererListener(struct pdraw *pdraw,
				   const struct pdraw_video_renderer_cbs *cbs,
				   void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mRenderer(NULL)
	{
	}

	~PdrawVideoRendererListener() {}

	void onVideoRenderReady(Pdraw::IPdraw *pdraw,
				struct pdraw_video_renderer *renderer)
	{
		if (mCbs.render_ready)
			(*mCbs.render_ready)(mPdraw, renderer, mUserdata);
	}

	int loadVideoTexture(Pdraw::IPdraw *pdraw,
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

    static double rad2deg(float x) {
        return (57.2958 * x);
    }
	int renderVideoOverlay(Pdraw::IPdraw *pdraw,
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
        struct vmeta_euler vme,cme;
        //dgdframeMeta = (vmeta_frame*)frameMeta;
        float cameraPan, cameraTilt,dronePitch, droneHeading, droneRoll;
        struct vmeta_location location;
        vmeta_frame_get_location(frameMeta, &location);
        vmeta_frame_get_camera_pan(frameMeta,&cameraPan);
        vmeta_frame_get_camera_tilt(frameMeta,&cameraTilt);
        cameraPan = rad2deg(cameraPan);
        cameraTilt = rad2deg(cameraTilt);
        vmeta_frame_get_drone_euler(frameMeta,&vme);
        vmeta_frame_get_frame_euler(frameMeta,&cme);
        struct vmeta_fov fov;
        float hfov,vfov;
        vmeta_frame_get_picture_h_fov(frameMeta,&hfov);
        vmeta_frame_get_picture_v_fov(frameMeta,&vfov);
        double sec,lt,ln,alt,p,h,r,hf,vf,agl,az,el;
        //hr = times / 3600.0;
        //min = (times - hr * 3600.0) / 60.0;
        //sec = (times - hr * 3600.0 - min * 60.0);
        lt = location.latitude;
        ln = location.longitude;
        alt = location.altitude;
        p = rad2deg(cme.pitch);
        h = rad2deg(cme.yaw);
        r = rad2deg(cme.roll);
        
        {
            boost::mutex::scoped_lock scoped_lock(sviewMutex);
            if(frameMeta != NULL)
                _viewDataInitialized = true;
            _frameViewData.reset(new SViewData);
            _frameViewData->dfov = vfov;
            _frameViewData->dVehicleRoll = 0.0;
            
            
            _frameViewData->dfov = vfov;
            _frameViewData->dFovVerticalAngle = vfov;
            _frameViewData->dFovHorizontalAngle = hfov;
            _frameViewData->dVehicleAltitude = alt;
            //46.764594, -92.151235 Lincoln Park
            //esko 46.685940, -92.363685
            _frameViewData->dVehicleLat =lt;
            _frameViewData->dVehicleLon =ln;
            _frameViewData->dCameraPitch = p;
            _frameViewData->dCameraHeading = h;
            _frameViewData->dCameraRoll = r;
            _frameViewData->dVehiclePitch = 0.0;
            _frameViewData->dVehicleHeading = 0.0;
            _frameViewData->dVehicleRoll = 0.0;
            _frameViewData->dVehicldAltitudeAGL = alt;
        }
        
        
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
	struct pdraw *mPdraw;
	struct pdraw_video_renderer_cbs mCbs;
	void *mUserdata;
	struct pdraw_video_renderer *mRenderer;
};


class PdrawVideoSinkListener : public Pdraw::IPdraw::VideoSinkListener {
public:
	PdrawVideoSinkListener(struct pdraw *pdraw,
			       const struct pdraw_video_sink_cbs *cbs,
			       void *userdata) :
			mPdraw(pdraw),
			mCbs(*cbs), mUserdata(userdata), mSink(NULL)
	{
	}

	~PdrawVideoSinkListener() {}

	void onVideoSinkFlush(Pdraw::IPdraw *pdraw,
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
	struct pdraw *mPdraw;
	struct pdraw_video_sink_cbs mCbs;
	void *mUserdata;
	struct pdraw_video_sink *mSink;
};


struct pdraw {
	Pdraw::IPdraw *pdraw;
	PdrawListener *listener;
	pthread_mutex_t mutex;
	std::vector<PdrawVideoRendererListener *> *videoRendererListeners;
	std::vector<PdrawVideoSinkListener *> *videoSinkListeners;
};


int pdraw_new(struct pomp_loop *loop,
	      const struct pdraw_cbs *cbs,
	      void *userdata,
	      struct pdraw **ret_obj)
{
	int ret = 0;
	struct pdraw *pdraw;

	if (loop == NULL)
		return -EINVAL;
	if (cbs == NULL)
		return -EINVAL;
	if (ret_obj == NULL)
		return -EINVAL;

	pdraw = (struct pdraw *)calloc(1, sizeof(*pdraw));
	if (pdraw == NULL)
		return -ENOMEM;

	ret = pthread_mutex_init(&pdraw->mutex, NULL);
	if (ret != 0) {
		free(pdraw);
		return -ret;
	}

	pdraw->videoRendererListeners =
		new std::vector<PdrawVideoRendererListener *>();
	if (pdraw->videoRendererListeners == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->videoSinkListeners = new std::vector<PdrawVideoSinkListener *>();
	if (pdraw->videoSinkListeners == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->listener = new PdrawListener(pdraw, cbs, userdata);
	if (pdraw->listener == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	pdraw->pdraw = new Pdraw::Session(loop, pdraw->listener);
	if (pdraw->pdraw == NULL) {
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
	if (pdraw == NULL)
		return -EINVAL;
	if (pdraw->pdraw != NULL)
		delete pdraw->pdraw;
	if (pdraw->listener != NULL)
		delete pdraw->listener;

	if (pdraw->videoSinkListeners != NULL) {
		std::vector<PdrawVideoSinkListener *>::iterator l =
			pdraw->videoSinkListeners->begin();
		while (l != pdraw->videoSinkListeners->end()) {
			if (*l != NULL)
				delete (*l);
			l++;
		}
		pdraw->videoSinkListeners->clear();
		delete pdraw->videoSinkListeners;
	}

	pthread_mutex_lock(&pdraw->mutex);
	if (pdraw->videoRendererListeners != NULL) {
		std::vector<PdrawVideoRendererListener *>::iterator r =
			pdraw->videoRendererListeners->begin();
		while (r != pdraw->videoRendererListeners->end()) {
			if (*r != NULL)
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


int pdraw_open_url(struct pdraw *pdraw, const char *url)
{
	if ((pdraw == NULL) || (url == NULL))
		return -EINVAL;

	std::string u(url);
	return pdraw->pdraw->open(u);
}


int pdraw_open_single_stream(struct pdraw *pdraw,
			     const char *local_addr,
			     uint16_t local_stream_port,
			     uint16_t local_control_port,
			     const char *remote_addr,
			     uint16_t remote_stream_port,
			     uint16_t remote_control_port)
{
	if ((pdraw == NULL) || (local_addr == NULL) || (remote_addr == NULL))
		return -EINVAL;

	std::string local(local_addr);
	std::string remote(remote_addr);
	return pdraw->pdraw->open(local,
				  local_stream_port,
				  local_control_port,
				  remote,
				  remote_stream_port,
				  remote_control_port);
}


int pdraw_open_url_mux(struct pdraw *pdraw,
		       const char *url,
		       struct mux_ctx *mux)
{
	if ((pdraw == NULL) || (url == NULL) || (mux == NULL))
		return -EINVAL;

	std::string u(url);
	return pdraw->pdraw->open(u, mux);
}


int pdraw_close(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->close();
}


uint16_t pdraw_get_single_stream_local_stream_port(struct pdraw *pdraw)
{
	if (pdraw == NULL) {
		ULOG_ERRNO("pdraw", EINVAL);
		return 0;
	}

	return pdraw->pdraw->getSingleStreamLocalStreamPort();
}


uint16_t pdraw_get_single_stream_local_control_port(struct pdraw *pdraw)
{
	if (pdraw == NULL) {
		ULOG_ERRNO("pdraw", EINVAL);
		return 0;
	}

	return pdraw->pdraw->getSingleStreamLocalControlPort();
}


int pdraw_is_ready_to_play(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return (pdraw->pdraw->isReadyToPlay()) ? 1 : 0;
}


int pdraw_is_paused(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return (pdraw->pdraw->isPaused()) ? 1 : 0;
}


int pdraw_play(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->play();
}


int pdraw_play_with_speed(struct pdraw *pdraw, float speed)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->play(speed);
}


int pdraw_pause(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->pause();
}


int pdraw_previous_frame(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->previousFrame();
}


int pdraw_next_frame(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->nextFrame();
}


int pdraw_seek(struct pdraw *pdraw, int64_t delta, int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seek(delta, exact ? true : false);
}


int pdraw_seek_forward(struct pdraw *pdraw, uint64_t delta, int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekForward(delta, exact ? true : false);
}


int pdraw_seek_back(struct pdraw *pdraw, uint64_t delta, int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekBack(delta, exact ? true : false);
}


int pdraw_seek_to(struct pdraw *pdraw, uint64_t timestamp, int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekTo(timestamp, exact ? true : false);
}


uint64_t pdraw_get_duration(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return 0;

	return pdraw->pdraw->getDuration();
}


uint64_t pdraw_get_current_time(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return 0;

	return pdraw->pdraw->getCurrentTime();
}


int pdraw_start_video_renderer(struct pdraw *pdraw,
			       const struct pdraw_rect *render_pos,
			       const struct pdraw_video_renderer_params *params,
			       const struct pdraw_video_renderer_cbs *cbs,
			       void *userdata,
			       struct pdraw_video_renderer **ret_obj)
{
	return pdraw_start_video_renderer_egl(
		pdraw, render_pos, params, cbs, userdata, NULL, ret_obj);
}

#include <iostream>
int pdraw_start_video_renderer_egl(
	struct pdraw *pdraw,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_video_renderer_cbs *cbs,
	void *userdata,
	struct egl_display *egl_display,
	struct pdraw_video_renderer **ret_obj)
{
	int ret = 0;
	struct pdraw_video_renderer *renderer = NULL;
	if (pdraw == NULL)
		return -EINVAL;
	if (ret_obj == NULL)
		return -EINVAL;

	pthread_mutex_lock(&pdraw->mutex);
	PdrawVideoRendererListener *l =
		new PdrawVideoRendererListener(pdraw, cbs, userdata);
	if (l == NULL) {
		ULOGE("failed to create video renderer listener");
		ret = -ENOMEM;
		goto error;
	}

	ret = pdraw->pdraw->startVideoRenderer(
		render_pos, params, l, &renderer, egl_display);
	if (ret < 0) {
        //std::cout<<"dgd fail"<<std::endl;
		delete l;
		goto error;
	}

	pdraw->videoRendererListeners->push_back(l);
	l->setVideoRenderer(renderer);
	pthread_mutex_unlock(&pdraw->mutex);
	*ret_obj = renderer;

	return 0;
error:
	pthread_mutex_unlock(&pdraw->mutex);
	return ret;
}


int pdraw_stop_video_renderer(struct pdraw *pdraw,
			      struct pdraw_video_renderer *renderer)
{
	int ret = 0;
	std::vector<PdrawVideoRendererListener *>::iterator l;

	if (pdraw == NULL)
		return -EINVAL;

	pthread_mutex_lock(&pdraw->mutex);
	ret = pdraw->pdraw->stopVideoRenderer(renderer);
	if (ret < 0)
		goto out;

	l = pdraw->videoRendererListeners->begin();
	while (l != pdraw->videoRendererListeners->end()) {
		if ((*l)->getVideoRenderer() != renderer) {
			l++;
			continue;
		}

		delete *l;
		pdraw->videoRendererListeners->erase(l);
		break;
	}

out:
	pthread_mutex_unlock(&pdraw->mutex);
	return ret;
}


int pdraw_resize_video_renderer(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_rect *render_pos)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->resizeVideoRenderer(renderer, render_pos);
}


int pdraw_set_video_renderer_params(
	struct pdraw *pdraw,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->setVideoRendererParams(renderer, params);
}


int pdraw_get_video_renderer_params(struct pdraw *pdraw,
				    struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->getVideoRendererParams(renderer, params);
}


int pdraw_render_video(struct pdraw *pdraw,
		       struct pdraw_video_renderer *renderer,
		       struct pdraw_rect *content_pos)
{
	return pdraw_render_video_mat(pdraw, renderer, content_pos, NULL, NULL);
}


int pdraw_render_video_mat(struct pdraw *pdraw,
			   struct pdraw_video_renderer *renderer,
			   struct pdraw_rect *content_pos,
			   const float *view_mat,
			   const float *proj_mat)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->renderVideo(
		renderer, content_pos, view_mat, proj_mat);
}


int pdraw_start_video_sink(struct pdraw *pdraw,
			   unsigned int media_id,
			   const struct pdraw_video_sink_params *params,
			   const struct pdraw_video_sink_cbs *cbs,
			   void *userdata,
			   struct pdraw_video_sink **ret_obj)
{
	int res;
	struct pdraw_video_sink *sink = NULL;

	if (pdraw == NULL)
		return -EINVAL;
	if (params == NULL)
		return -EINVAL;
	if ((cbs == NULL) || (cbs->flush == NULL))
		return -EINVAL;
	if (ret_obj == NULL)
		return -EINVAL;

	PdrawVideoSinkListener *videoSinkListener =
		new PdrawVideoSinkListener(pdraw, cbs, userdata);
	if (videoSinkListener == NULL) {
		ULOGE("failed to create video sink listener");
		return -ENOMEM;
	}

	res = pdraw->pdraw->startVideoSink(
		media_id, params, videoSinkListener, &sink);
	if (res < 0) {
		delete videoSinkListener;
		return res;
	}

	videoSinkListener->setVideoSink(sink);
	pdraw->videoSinkListeners->push_back(videoSinkListener);
	*ret_obj = sink;
	return 0;
}


int pdraw_stop_video_sink(struct pdraw *pdraw, struct pdraw_video_sink *sink)
{
	int res;

	if (pdraw == NULL)
		return -EINVAL;
	if (sink == NULL)
		return -EINVAL;

	res = pdraw->pdraw->stopVideoSink(sink);
	if (res < 0) {
		ULOG_ERRNO("stopVideoSink", -res);
		return res;
	}

	std::vector<PdrawVideoSinkListener *>::iterator l =
		pdraw->videoSinkListeners->begin();
	while (l != pdraw->videoSinkListeners->end()) {
		if ((*l)->getVideoSink() != sink) {
			l++;
			continue;
		}
		delete *l;
		pdraw->videoSinkListeners->erase(l);
		break;
	}

	return 0;
}


int pdraw_resync_video_sink(struct pdraw *pdraw, struct pdraw_video_sink *sink)
{
	int res;

	if (pdraw == NULL)
		return -EINVAL;
	if (sink == NULL)
		return -EINVAL;

	res = pdraw->pdraw->resyncVideoSink(sink);
	if (res < 0) {
		ULOG_ERRNO("resyncVideoSink", -res);
		return res;
	}

	return 0;
}


struct vbuf_queue *pdraw_get_video_sink_queue(struct pdraw *pdraw,
					      struct pdraw_video_sink *sink)
{
	if (pdraw == NULL) {
		ULOG_ERRNO("pdraw", EINVAL);
		return NULL;
	}
	if (sink == NULL) {
		ULOG_ERRNO("sink", EINVAL);
		return NULL;
	}

	return pdraw->pdraw->getVideoSinkQueue(sink);
}


int pdraw_video_sink_queue_flushed(struct pdraw *pdraw,
				   struct pdraw_video_sink *sink)
{
	int res;

	if (pdraw == NULL)
		return -EINVAL;
	if (sink == NULL)
		return -EINVAL;

	res = pdraw->pdraw->videoSinkQueueFlushed(sink);
	if (res < 0) {
		ULOG_ERRNO("videoSinkQueueFlushed", -res);
		return res;
	}

	return 0;
}


enum pdraw_session_type pdraw_get_session_type(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return PDRAW_SESSION_TYPE_UNKNOWN;

	return pdraw->pdraw->getSessionType();
}


int pdraw_get_self_friendly_name(struct pdraw *pdraw, char *str, size_t len)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string fn;
	pdraw->pdraw->getSelfFriendlyName(&fn);
	if ((str) && (fn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, fn.c_str());
	return 0;
}


int pdraw_set_self_friendly_name(struct pdraw *pdraw, const char *friendly_name)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string fn(friendly_name);
	pdraw->pdraw->setSelfFriendlyName(fn);
	return 0;
}


int pdraw_get_self_serial_number(struct pdraw *pdraw, char *str, size_t len)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sn;
	pdraw->pdraw->getSelfSerialNumber(&sn);
	if ((str) && (sn.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sn.c_str());
	return 0;
}


int pdraw_set_self_serial_number(struct pdraw *pdraw, const char *serial_number)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sn(serial_number);
	pdraw->pdraw->setSelfSerialNumber(sn);
	return 0;
}


int pdraw_get_self_software_version(struct pdraw *pdraw, char *str, size_t len)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sv;
	pdraw->pdraw->getSelfSoftwareVersion(&sv);
	if ((str) && (sv.length() >= len))
		return -ENOBUFS;

	if (str)
		strcpy(str, sv.c_str());
	return 0;
}


int pdraw_set_self_software_version(struct pdraw *pdraw,
				    const char *software_version)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sv(software_version);
	pdraw->pdraw->setSelfSoftwareVersion(sv);
	return 0;
}


int pdraw_is_self_pilot(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return (pdraw->pdraw->isSelfPilot()) ? 1 : 0;
}


int pdraw_set_self_pilot(struct pdraw *pdraw, int is_pilot)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfPilot((is_pilot) ? true : false);
	return 0;
}


int pdraw_get_peer_session_metadata(struct pdraw *pdraw,
				    struct vmeta_session *session)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getPeerSessionMetadata(session);
	return 0;
}


enum pdraw_drone_model pdraw_get_peer_drone_model(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return PDRAW_DRONE_MODEL_UNKNOWN;

	return pdraw->pdraw->getPeerDroneModel();
}


enum pdraw_pipeline_mode pdraw_get_pipeline_mode_setting(struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return (enum pdraw_pipeline_mode)(-EINVAL);

	return pdraw->pdraw->getPipelineModeSetting();
}


int pdraw_set_pipeline_mode_setting(struct pdraw *pdraw,
				    enum pdraw_pipeline_mode mode)
{
	if (pdraw == NULL)
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
	if (pdraw == NULL)
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
	if (pdraw == NULL)
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
	if (pdraw == NULL)
		return (enum pdraw_hmd_model)(-EINVAL);

	return pdraw->pdraw->getHmdModelSetting();
}


int pdraw_set_hmd_model_setting(struct pdraw *pdraw,
				enum pdraw_hmd_model hmd_model)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setHmdModelSetting(hmd_model);
	return 0;
}


int pdraw_set_android_jvm(struct pdraw *pdraw, void *jvm)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setAndroidJvm(jvm);
	return 0;
}


const char *pdraw_drone_model_str(enum pdraw_drone_model val)
{
	return pdraw_droneModelStr(val);
}


const char *pdraw_hmd_model_str(enum pdraw_hmd_model val)
{
	return pdraw_hmdModelStr(val);
}


const char *pdraw_pipeline_mode_str(enum pdraw_pipeline_mode val)
{
	return pdraw_pipelineModeStr(val);
}


const char *pdraw_session_type_str(enum pdraw_session_type val)
{
	return pdraw_sessionTypeStr(val);
}


const char *pdraw_media_type_str(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


const char *pdraw_video_media_format_str(enum pdraw_video_media_format val)
{
	return pdraw_videoMediaFormatStr(val);
}


const char *pdraw_yuv_format_str(enum pdraw_yuv_format val)
{
	return pdraw_yuvFormatStr(val);
}


const char *pdraw_h264_format_str(enum pdraw_h264_format val)
{
	return pdraw_h264FormatStr(val);
}


const char *pdraw_video_type_str(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


const char *pdraw_histogram_channel_str(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
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
				  char *str,
				  unsigned int len)
{
	return pdraw_frameMetadataToJsonStr(frame, str, len);
}


int pdraw_video_frame_to_json(const struct pdraw_video_frame *frame,
			      struct json_object *jobj)
{
	return pdraw_frameMetadataToJson(frame, jobj);
}


int pdraw_pack_yuv_frame(const struct pdraw_video_frame *in_frame,
			 struct pdraw_video_frame *out_frame,
			 struct vbuf_buffer *out_buf)
{
	return pdraw_packYUVFrame(in_frame, out_frame, out_buf);
}
