/**
 * Parrot Drones Awesome Video Viewer Library
 * Session
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

#define ULOG_TAG pdraw_session
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_decoder_video.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_encoder_video.hpp"
#include "pdraw_muxer_record.hpp"
#include "pdraw_muxer_stream_rtmp.hpp"
#include "pdraw_scaler_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

namespace Pdraw {


int createPdraw(struct pomp_loop *loop,
		IPdraw::Listener *listener,
		IPdraw **retObj)
{
	IPdraw *pdraw = nullptr;
	if (loop == nullptr) {
		ULOGE("invalid loop");
		return -EINVAL;
	}
	if (retObj == nullptr) {
		ULOGE("invalid pointer");
		return -EINVAL;
	}
	pdraw = new Session(loop, listener);
	if (pdraw == nullptr) {
		ULOGE("failed to create pdraw instance");
		return -EPROTO;
	}
	*retObj = pdraw;
	return 0;
}


const char *pdrawHmdModelStr(enum pdraw_hmd_model val)
{
	return pdraw_hmdModelStr(val);
}


const char *pdrawPipelineModeStr(enum pdraw_pipeline_mode val)
{
	return pdraw_pipelineModeStr(val);
}


const char *pdrawPlaybackTypeStr(enum pdraw_playback_type val)
{
	return pdraw_playbackTypeStr(val);
}


const char *pdrawMediaTypeStr(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


const char *pdrawVideoTypeStr(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


const char *pdrawHistogramChannelStr(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
}


const char *pdrawVideoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val)
{
	return pdraw_videoRendererSchedulingModeStr(val);
}


const char *
pdrawVideoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val)
{
	return pdraw_videoRendererFillModeStr(val);
}


const char *pdrawVideoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val)
{
	return pdraw_videoRendererTransitionFlagStr(val);
}


const char *pdrawVipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val)
{
	return pdraw_vipcSourceEosReasonStr(val);
}


int pdrawVideoFrameToJsonStr(const struct pdraw_video_frame *frame,
			     struct vmeta_frame *metadata,
			     char *str,
			     unsigned int len)
{
	return pdraw_frameMetadataToJsonStr(frame, metadata, str, len);
}


int pdrawVideoFrameToJson(const struct pdraw_video_frame *frame,
			  struct vmeta_frame *metadata,
			  struct json_object *jobj)
{
	return pdraw_frameMetadataToJson(frame, metadata, jobj);
}


struct pdraw_media_info *pdrawMediaInfoDup(const struct pdraw_media_info *src)
{
	return pdraw_mediaInfoDup(src);
}


void pdrawMediaInfoFree(struct pdraw_media_info *media_info)
{
	return pdraw_mediaInfoFree(media_info);
}


Session::Session(struct pomp_loop *loop, IPdraw::Listener *listener) :
		mFactory(this), mListener(listener), mState(STOPPED),
		mLoop(loop), mAndroidJvm(nullptr)

{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;
	bool mutex_created = false;

	mLoopThread = pthread_self();

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init(mMutex)", res);
		goto error;
	}
	pthread_mutexattr_destroy(&attr);
	mutex_created = true;

	res = pthread_mutex_init(&mAsyncMutex, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init(mAsyncMutex)", res);
		goto error;
	}

	setState(READY);
	return;

error:
	if (mutex_created)
		pthread_mutex_destroy(&mMutex);
	else if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


Session::~Session(void)
{
	if (mState != STOPPED)
		ULOGW("destroying while instance is still running");

	/* Remove any leftover idle callbacks */
	if (mLoop != nullptr) {
		int err = pomp_loop_idle_remove_by_cookie(mLoop, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
	}

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		delete *e;
		e++;
	}
	mElements.clear();
	pthread_mutex_unlock(&mMutex);

	pthread_mutex_lock(&mAsyncMutex);
	while (!mMediaAddedInfoArgs.empty()) {
		struct pdraw_media_info info = mMediaAddedInfoArgs.front();
		mMediaAddedInfoArgs.pop();
		Media::cleanupMediaInfo(&info);
	}
	while (!mMediaRemovedInfoArgs.empty()) {
		struct pdraw_media_info info = mMediaRemovedInfoArgs.front();
		mMediaRemovedInfoArgs.pop();
		Media::cleanupMediaInfo(&info);
	}
	pthread_mutex_unlock(&mAsyncMutex);

	pthread_mutex_destroy(&mMutex);
	pthread_mutex_destroy(&mAsyncMutex);
}


/*
 * API methods
 */

int Session::stop(void)
{
	int ret;
	bool stopped = true;
	std::vector<Element *>::iterator e;

	if (mState == STOPPING) {
		/* Return without calling the stopResponse() function */
		ULOGI("%s: already in %s state, nothing to do",
		      __func__,
		      stateStr(mState));
		return 0;
	}

	if (mState == STOPPED) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: state is %s, nothing to do",
		      __func__,
		      stateStr(mState));
		ret = 0;
		goto already_stopped;
	}

	if (mState != READY) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	setState(STOPPING);

	pthread_mutex_lock(&mMutex);
	e = mElements.begin();
	while (e != mElements.end()) {
		if ((*e)->getState() != Element::State::STOPPED) {
			stopped = false;
		} else {
			int err = (*e)->stop();
			if (err < 0)
				ULOG_ERRNO("element->stop", -err);
		}
		e++;
	}
	pthread_mutex_unlock(&mMutex);

	if (stopped) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: all elements are stopped, closing", __func__);
		setState(STOPPED);
		ret = 0;
		goto already_stopped;
	}

	/* Waiting for the asynchronous stop; stopResponse()
	 * will be called when it's done */
	return 0;

already_stopped:
	if (mListener != nullptr && ret == 0)
		stopResp(ret);
	return ret;
}


/* Called on the rendering thread */
int Session::createVideoRenderer(
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	IPdraw::IVideoRenderer **retObj,
	struct egl_display *eglDisplay)
{
	int res;
	Session::VideoRenderer *renderer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(renderPos == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	renderer = new Session::VideoRenderer(
		this, mediaId, renderPos, params, listener, eglDisplay);
	if (renderer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the video renderer", __func__);
		return -ENOMEM;
	}

	mElements.push_back(renderer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = renderer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("renderer->start", -res);
		return res;
	}

	*retObj = renderer;

	return 0;
}


/* Called on the rendering thread */
Session::VideoRenderer::VideoRenderer(
	Session *session,
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	struct egl_display *eglDisplay)
{
	mRenderer = Pdraw::VideoRenderer::create(session,
						 session,
						 this,
						 listener,
						 mediaId,
						 renderPos,
						 params,
						 eglDisplay);
	if (mRenderer == nullptr) {
		ULOGE("%s: failed to create the video renderer", __func__);
		return;
	}
}


/* Called on the rendering thread */
Session::VideoRenderer::~VideoRenderer(void)
{
	if (mRenderer == nullptr)
		return;

	int ret = mRenderer->stop();
	if (ret < 0)
		ULOG_ERRNO("renderer->stop", -ret);
}


/* Called on the rendering thread */
int Session::VideoRenderer::resize(const struct pdraw_rect *renderPos)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->resize(renderPos);
}


/* Called on the rendering thread */
int Session::VideoRenderer::setMediaId(unsigned int mediaId)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->setMediaId(mediaId);
}


/* Called on the rendering thread */
unsigned int Session::VideoRenderer::getMediaId(void)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->getMediaId();
}


/* Called on the rendering thread */
int Session::VideoRenderer::setParams(
	const struct pdraw_video_renderer_params *params)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->setParams(params);
}


/* Called on the rendering thread */
int Session::VideoRenderer::getParams(
	struct pdraw_video_renderer_params *params)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->getParams(params);
}


/* Called on the rendering thread */
int Session::VideoRenderer::render(struct pdraw_rect *contentPos,
				   const float *viewMat,
				   const float *projMat)
{
	if (mRenderer == nullptr)
		return -EPROTO;

	return mRenderer->render(contentPos, viewMat, projMat);
}


int Session::createDemuxer(const std::string &url,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	return createDemuxer(url, nullptr, listener, retObj);
}


int Session::createDemuxer(const std::string &localAddr,
			   uint16_t localStreamPort,
			   uint16_t localControlPort,
			   const std::string &remoteAddr,
			   uint16_t remoteStreamPort,
			   uint16_t remoteControlPort,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	int res;
	Session::Demuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	demuxer = new Session::Demuxer(this,
				       localAddr,
				       localStreamPort,
				       localControlPort,
				       remoteAddr,
				       remoteStreamPort,
				       remoteControlPort,
				       listener);
	if (demuxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		return -ENOMEM;
	}
	if (demuxer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		delete demuxer;
		return -EINVAL;
	}

	mElements.push_back(demuxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		goto error;
	}

	*retObj = demuxer;

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error:
	if (demuxer != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != demuxer->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete demuxer;
	}
	return res;
}


int Session::createDemuxer(const std::string &url,
			   struct mux_ctx *mux,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	int res;
	Session::Demuxer *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	demuxer = new Session::Demuxer(this, url, mux, listener);
	if (demuxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		return -ENOMEM;
	}
	if (demuxer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		delete demuxer;
		return -EINVAL;
	}

	mElements.push_back(demuxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		goto error;
	}

	*retObj = demuxer;

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error:
	if (demuxer != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != demuxer->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete demuxer;
	}
	return res;
}


Session::Demuxer::Demuxer(Session *session,
			  const std::string &url,
			  struct mux_ctx *mux,
			  IPdraw::IDemuxer::Listener *listener) :
		mSession(session),
		mDemuxer(nullptr)
{
	std::string ext;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if ((mux != nullptr) && (url.substr(0, 7) == "rtsp://")) {
#ifdef BUILD_LIBMUX
		StreamDemuxerMux *demuxer;
		demuxer = new StreamDemuxerMux(
			mSession, mSession, mSession, this, listener, url, mux);
		if (demuxer == nullptr) {
			ULOGE("%s: failed to create the stream demuxer",
			      __func__);
			return;
		}
		mDemuxer = demuxer;
#else /* BUILD_LIBMUX */
		ULOGE("%s: libmux is not supported", __func__);
#endif /* BUILD_LIBMUX */
	} else if (url.substr(0, 7) == "rtsp://") {
		StreamDemuxerNet *demuxer;
		demuxer = new StreamDemuxerNet(
			mSession, mSession, mSession, this, listener, url);
		if (demuxer == nullptr) {
			ULOGE("%s: failed to create the stream demuxer",
			      __func__);
			return;
		}
		mDemuxer = demuxer;
	} else if (ext == ".mp4") {
		RecordDemuxer *demuxer;
		demuxer = new RecordDemuxer(
			mSession, mSession, mSession, this, listener, url);
		if (demuxer == nullptr) {
			ULOGE("%s: failed to create the record demuxer",
			      __func__);
			return;
		}
		mDemuxer = demuxer;
	} else {
		ULOGE("%s: unsupported URL", __func__);
	}
}


Session::Demuxer::Demuxer(Session *session,
			  const std::string &localAddr,
			  uint16_t localStreamPort,
			  uint16_t localControlPort,
			  const std::string &remoteAddr,
			  uint16_t remoteStreamPort,
			  uint16_t remoteControlPort,
			  IPdraw::IDemuxer::Listener *listener) :
		mSession(session),
		mDemuxer(nullptr)
{
	StreamDemuxerNet *demuxer;
	demuxer = new StreamDemuxerNet(mSession,
				       mSession,
				       mSession,
				       this,
				       listener,
				       localAddr,
				       localStreamPort,
				       localControlPort,
				       remoteAddr,
				       remoteStreamPort,
				       remoteControlPort);
	if (demuxer == nullptr) {
		ULOGE("%s: failed to create the stream demuxer", __func__);
		return;
	}
	mDemuxer = demuxer;
}


Session::Demuxer::~Demuxer(void)
{
	if (mDemuxer == nullptr)
		return;

	int res = mDemuxer->stop();
	if (res < 0)
		ULOG_ERRNO("Demuxer::stop", -res);
}


int Session::Demuxer::close(void)
{
	int res;

	if (mDemuxer == nullptr)
		return -EPROTO;

	res = mDemuxer->stop();
	if (res < 0) {
		ULOG_ERRNO("Demuxer::stop", -res);
		return res;
	}

	/* Waiting for the asynchronous stop; closeResponse()
	 * will be called when it's done */
	mDemuxer = nullptr;
	return 0;
}


uint16_t Session::Demuxer::getSingleStreamLocalStreamPort(void)
{
	if (mDemuxer == nullptr)
		return 0;

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalStreamPort();
}


uint16_t Session::Demuxer::getSingleStreamLocalControlPort(void)
{
	if (mDemuxer == nullptr)
		return 0;

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == nullptr) {
		ULOGE("%s: invalid demuxer", __func__);
		return 0;
	}

	return demuxer->getSingleStreamLocalControlPort();
}


bool Session::Demuxer::isReadyToPlay(void)
{
	if (mDemuxer == nullptr)
		return false;

	return mDemuxer->isReadyToPlay();
}


bool Session::Demuxer::isPaused(void)
{
	if (mDemuxer == nullptr)
		return false;

	return mDemuxer->isPaused();
}


int Session::Demuxer::play(float speed)
{
	if (mDemuxer == nullptr)
		return -EPROTO;

	return mDemuxer->play(speed);
}


int Session::Demuxer::pause(void)
{
	return play(0.);
}


int Session::Demuxer::previousFrame(void)
{
	if (mDemuxer == nullptr)
		return -EPROTO;
	return mDemuxer->previous();
}


int Session::Demuxer::nextFrame(void)
{
	if (mDemuxer == nullptr)
		return -EPROTO;
	return mDemuxer->next();
}


int Session::Demuxer::seek(int64_t delta, bool exact)
{
	if (mDemuxer == nullptr)
		return -EPROTO;

	return mDemuxer->seek(delta, exact);
}


int Session::Demuxer::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta);
}


int Session::Demuxer::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta));
}


int Session::Demuxer::seekTo(uint64_t timestamp, bool exact)
{
	if (mDemuxer == nullptr)
		return -EPROTO;

	return mDemuxer->seekTo(timestamp, exact);
}


uint64_t Session::Demuxer::getDuration(void)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getDuration();
}


uint64_t Session::Demuxer::getCurrentTime(void)
{
	if (mDemuxer == nullptr)
		return 0;

	return mDemuxer->getCurrentTime();
}


int Session::createMuxer(const std::string &url,
			 const struct pdraw_muxer_params *params,
			 IPdraw::IMuxer::Listener *listener,
			 IPdraw::IMuxer **retObj)
{
	int res;
	Session::Muxer *muxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("muxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	muxer = new Session::Muxer(this, url, params, listener);
	if (muxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the muxer", __func__);
		return -ENOMEM;
	}

	mElements.push_back(muxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = muxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("muxer->start", -res);
		goto error;
	}

	*retObj = muxer;

	return 0;

error:
	if (muxer != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != muxer->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete muxer;
	}
	return res;
}


Session::Muxer::Muxer(Session *session,
		      const std::string &url,
		      const struct pdraw_muxer_params *params,
		      IPdraw::IMuxer::Listener *listener)
{
	std::string ext;

	mSession = session;
	mMuxer = nullptr;

	if (url.length() < 4) {
		ULOGE("%s: invalid URL length", __func__);
		return;
	}
	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if (url.substr(0, 7) == "rtmp://") {
#ifdef BUILD_LIBRTMP
		RtmpStreamMuxer *muxer;
		muxer = new Pdraw::RtmpStreamMuxer(
			session, session, listener, this, url, params);
		if (muxer == nullptr) {
			ULOGE("%s: failed to create the RTMP stream muxer",
			      __func__);
			return;
		}
		mMuxer = muxer;
#else
		ULOGE("%s: librtmp is not supported", __func__);
#endif
	} else if (ext == ".mp4" || ext == ".tmp") {
		RecordMuxer *muxer;
		muxer = new Pdraw::RecordMuxer(
			session, session, listener, this, url, params);
		if (muxer == nullptr) {
			ULOGE("%s: failed to create the record muxer",
			      __func__);
			return;
		}
		mMuxer = muxer;
	} else {
		ULOGE("%s: unsupported URL", __func__);
	}
}


Session::Muxer::~Muxer(void)
{
	if (mMuxer == nullptr)
		return;

	int res = mMuxer->stop();
	if (res < 0)
		ULOG_ERRNO("Muxer::stop", -res);
}


int Session::Muxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size)
{
	if (mMuxer == nullptr)
		return -EPROTO;

	return mMuxer->setThumbnail(type, data, size);
}


int Session::Muxer::addMedia(
	unsigned int mediaId,
	const struct pdraw_muxer_video_media_params *params)
{
	int res = 0;
	Source *source = nullptr;
	Media *m = nullptr;
	CodedVideoMedia *codedMedia = nullptr;
	RawVideoMedia *rawMedia = nullptr;
	Channel *channel = nullptr;
	bool found = false;

	if (mMuxer == nullptr)
		return -EPROTO;

	pthread_mutex_lock(&mSession->mMutex);

	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			m = source->getOutputMedia(i);
			codedMedia = dynamic_cast<CodedVideoMedia *>(m);
			rawMedia = dynamic_cast<RawVideoMedia *>(m);
			if ((codedMedia != nullptr) &&
			    (codedMedia->id == mediaId)) {
				found = true;
				break;
			} else if ((rawMedia != nullptr) &&
				   (rawMedia->id == mediaId)) {
				found = true;
				break;
			}
		}
		if (found)
			break;
		e++;
	}
	if ((!found) || (source == nullptr) ||
	    (codedMedia == nullptr && rawMedia == nullptr)) {
		pthread_mutex_unlock(&mSession->mMutex);
		return -ENOENT;
	}

	res = mMuxer->addInputMedia(m, params);
	if (res < 0) {
		ULOG_ERRNO("RecordMuxer::addInputMedia", -res);
		goto out;
	}
	channel = mMuxer->getInputChannel(m);
	if (channel == nullptr) {
		ULOGE("failed to get muxer input channel");
		res = -EPROTO;
		goto out;
	}
	res = source->addOutputChannel(m, channel);
	if (res < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -res);
		goto out;
	}

out:
	pthread_mutex_unlock(&mSession->mMutex);
	return res;
}


Session::VipcSource::VipcSource(Session *session,
				const struct pdraw_vipc_source_params *params,
				IPdraw::IVipcSource::Listener *listener)
{
#ifdef BUILD_LIBVIDEO_IPC
	mSource = new Pdraw::VipcSource(
		session, session, session, listener, this, params);
	if (mSource == nullptr) {
		ULOGE("%s: failed to create the video IPC source", __func__);
		return;
	}
#endif
}


Session::VipcSource::~VipcSource(void)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
#endif
}


bool Session::VipcSource::isReadyToPlay(void)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return false;
	return mSource->isReadyToPlay();
#else
	return false;
#endif
}


bool Session::VipcSource::isPaused(void)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return false;
	return mSource->isPaused();
#else
	return false;
#endif
}


int Session::VipcSource::play(void)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->play();
#else
	return -ENOSYS;
#endif
}


int Session::VipcSource::pause(void)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->pause();
#else
	return -ENOSYS;
#endif
}


int Session::VipcSource::configure(const struct vdef_dim *resolution,
				   const struct vdef_rectf *crop)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->configure(resolution, crop);
#else
	return -ENOSYS;
#endif
}


int Session::VipcSource::setSessionMetadata(const struct vmeta_session *meta)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->setSessionMetadata(meta);
#else
	return -ENOSYS;
#endif
}


int Session::VipcSource::getSessionMetadata(struct vmeta_session *meta)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->getSessionMetadata(meta);
#else
	return -ENOSYS;
#endif
}


Session::CodedVideoSource::CodedVideoSource(
	Session *session,
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener)
{
	mSource = new Pdraw::ExternalCodedVideoSource(
		session, session, session, listener, this, params);
	if (mSource == nullptr) {
		ULOGE("%s: failed to create the video source", __func__);
		return;
	}
}


Session::CodedVideoSource::~CodedVideoSource(void)
{
	if (mSource == nullptr)
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
}


struct mbuf_coded_video_frame_queue *Session::CodedVideoSource::getQueue(void)
{
	if (mSource == nullptr)
		return nullptr;
	return mSource->getQueue();
}


int Session::CodedVideoSource::flush(void)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->flush();
}


int Session::CodedVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->setSessionMetadata(meta);
}


int Session::CodedVideoSource::getSessionMetadata(struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->getSessionMetadata(meta);
}


Session::RawVideoSource::RawVideoSource(
	Session *session,
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener)
{
	mSource = new Pdraw::ExternalRawVideoSource(
		session, session, session, listener, this, params);
	if (mSource == nullptr) {
		ULOGE("%s: failed to create the video source", __func__);
		return;
	}
}


Session::RawVideoSource::~RawVideoSource(void)
{
	if (mSource == nullptr)
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
}


struct mbuf_raw_video_frame_queue *Session::RawVideoSource::getQueue(void)
{
	if (mSource == nullptr)
		return nullptr;
	return mSource->getQueue();
}


int Session::RawVideoSource::flush(void)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->flush();
}


int Session::RawVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->setSessionMetadata(meta);
}


int Session::RawVideoSource::getSessionMetadata(struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->getSessionMetadata(meta);
}


Session::CodedVideoSink::CodedVideoSink(
	Session *session,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener)
{
	mSink = new Pdraw::ExternalCodedVideoSink(
		session,
		&params->required_coded_format,
		session,
		listener,
		this,
		params);
	if (mSink == nullptr) {
		ULOGE("%s: failed to create the video sink", __func__);
		return;
	}
}


Session::CodedVideoSink::~CodedVideoSink(void)
{
	if (mSink == nullptr)
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("sink->stop", -ret);
}


int Session::CodedVideoSink::resync(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->resync();
}


struct mbuf_coded_video_frame_queue *Session::CodedVideoSink::getQueue(void)
{
	if (mSink == nullptr)
		return nullptr;
	return mSink->getQueue();
}


int Session::CodedVideoSink::queueFlushed(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->flushDone();
}


Session::RawVideoSink::RawVideoSink(
	Session *session,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener)
{
	mSink = new Pdraw::ExternalRawVideoSink(
		session, session, listener, this, params);
	if (mSink == nullptr) {
		ULOGE("%s: failed to create the video sink", __func__);
		return;
	}
}


Session::RawVideoSink::~RawVideoSink(void)
{
	if (mSink == nullptr)
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("sink->stop", -ret);
}


struct mbuf_raw_video_frame_queue *Session::RawVideoSink::getQueue(void)
{
	if (mSink == nullptr)
		return nullptr;
	return mSink->getQueue();
}


int Session::RawVideoSink::queueFlushed(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->flushDone();
}


int Session::internalCreateCodedVideoSink(
	Source *source,
	CodedVideoMedia *media,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	/* Note: mMutex is held while this function is called */
	int res;
	Session::CodedVideoSink *sink = nullptr;
	Channel *channel = nullptr;

	sink = new CodedVideoSink(this, params, listener);
	if (sink == nullptr) {
		ULOGE("coded video sink creation failed");
		return -ENOMEM;
	}

	mElements.push_back(sink->getElement());

	res = sink->getSink()->addInputMedia(media);
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->addInputMedia", -res);
		goto error;
	}

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->start", -res);
		goto error;
	}

	channel = sink->getSink()->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get coded video sink input channel");
		res = -EPROTO;
		goto error;
	}

	res = source->addOutputChannel(media, channel);
	if (res < 0) {
		ULOG_ERRNO("source->addOutputChannel", -res);
		goto error;
	}

	/* Force a resync after linking the elements; this allows a coded
	 * video sink to start on an IDR frame for example */
	res = sink->getCodedVideoSink()->resync();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->resync", -res);
		goto error;
	}

	*retObj = sink;

	return 0;

error:
	if (sink != nullptr) {
		if (channel != nullptr) {
			/* removeOutputChannel must be called without mMutex
			 * being held, so release it here */
			pthread_mutex_unlock(&mMutex);
			source->removeOutputChannel(media, channel);
			pthread_mutex_lock(&mMutex);
		}
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != sink->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		delete sink;
	}
	return res;
}


int Session::internalCreateRawVideoSink(
	Source *source,
	RawVideoMedia *media,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	/* Note: mMutex is held while this function is called */
	int res;
	Session::RawVideoSink *sink = nullptr;
	Channel *channel = nullptr;

	sink = new RawVideoSink(this, params, listener);
	if (sink == nullptr) {
		ULOGE("raw video sink creation failed");
		return -ENOMEM;
	}

	mElements.push_back(sink->getElement());

	res = sink->getSink()->addInputMedia(media);
	if (res < 0) {
		ULOG_ERRNO("rawVideoSink->addInputMedia", -res);
		goto error;
	}

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSink->start", -res);
		goto error;
	}

	channel = sink->getSink()->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get raw video sink input channel");
		res = -EPROTO;
		goto error;
	}

	res = source->addOutputChannel(media, channel);
	if (res < 0) {
		ULOG_ERRNO("source->addOutputChannel", -res);
		goto error;
	}

	*retObj = sink;

	return 0;

error:
	if (sink != nullptr) {
		if (channel != nullptr) {
			/* removeOutputChannel must be called without mMutex
			 * being held, so release it here */
			pthread_mutex_unlock(&mMutex);
			source->removeOutputChannel(media, channel);
			pthread_mutex_lock(&mMutex);
		}
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != sink->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		delete sink;
	}
	return res;
}


int Session::createVipcSource(const struct pdraw_vipc_source_params *params,
			      IPdraw::IVipcSource::Listener *listener,
			      IPdraw::IVipcSource **retObj)
{
#ifdef BUILD_LIBVIDEO_IPC
	int res;
	Session::VipcSource *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new VipcSource(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("video IPC source creation failed");
		return -ENOMEM;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("vipcSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	if (source != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != source->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete source;
	}
	return res;
#else
	return -ENOSYS;
#endif
}


int Session::createCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	int res;
	Session::CodedVideoSource *source = nullptr;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;
	if (params->video.format != VDEF_FRAME_TYPE_CODED)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	source = new CodedVideoSource(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("coded video source creation failed");
		return -ENOMEM;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	if (source != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != source->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete source;
	}
	return res;
}


int Session::createRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	int res;
	Session::RawVideoSource *source = nullptr;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;
	if (params->video.format != VDEF_FRAME_TYPE_RAW)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	source = new RawVideoSource(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("raw video source creation failed");
		return -ENOMEM;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	if (source != nullptr) {
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if (*e != source->getElement()) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete source;
	}
	return res;
}


int Session::createCodedVideoSink(unsigned int mediaId,
				  const struct pdraw_video_sink_params *params,
				  IPdraw::ICodedVideoSink::Listener *listener,
				  IPdraw::ICodedVideoSink **retObj)
{
	int ret = 0;
	bool found = false;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		CodedVideoMedia *codedMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				codedMedia =
					dynamic_cast<CodedVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && codedMedia != nullptr) {
			ret = internalCreateCodedVideoSink(
				source, codedMedia, params, listener, retObj);
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::createRawVideoSink(unsigned int mediaId,
				const struct pdraw_video_sink_params *params,
				IPdraw::IRawVideoSink::Listener *listener,
				IPdraw::IRawVideoSink **retObj)
{
	int ret = 0;
	bool found = false;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				rawMedia = dynamic_cast<RawVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && rawMedia != nullptr) {
			ret = internalCreateRawVideoSink(
				source, rawMedia, params, listener, retObj);
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Session::VideoEncoderWrapper::VideoEncoderWrapper(
	Session *session,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener)
{
	mEncoder = new Pdraw::VideoEncoder(
		session, session, session, listener, this, params);
	if (mEncoder == nullptr) {
		ULOGE("%s: failed to create the video encoder", __func__);
		return;
	}
}


Session::VideoEncoderWrapper::~VideoEncoderWrapper(void)
{
	if (mEncoder == nullptr)
		return;
	int ret = mEncoder->stop();
	if (ret < 0)
		ULOG_ERRNO("VideoEncoder::stop", -ret);
}


int Session::VideoEncoderWrapper::configure(
	const struct venc_dyn_config *config)
{
	if (mEncoder == nullptr)
		return -EPROTO;
	return mEncoder->configure(config);
}


int Session::createVideoEncoder(unsigned int mediaId,
				const struct venc_config *params,
				IPdraw::IVideoEncoder::Listener *listener,
				IPdraw::IVideoEncoder **retObj)
{
	int ret = 0;
	bool found = false;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				rawMedia = dynamic_cast<RawVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && rawMedia != nullptr) {
			VideoEncoderWrapper *wrapper =
				new VideoEncoderWrapper(this, params, listener);
			if (wrapper == nullptr) {
				ULOGE("failed to create the "
				      "VideoEncoderWrapper object");
				ret = -ENOMEM;
				goto exit;
			}
			ret = mFactory.addEncoderForMedia(
				source,
				rawMedia,
				params,
				listener,
				wrapper->getVideoEncoder());
			if (ret < 0) {
				ULOG_ERRNO(
					"PipelineFactory::addEncoderForMedia",
					-ret);
				goto exit;
			}
			*retObj = wrapper;
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Session::getFriendlyNameSetting(std::string *friendlyName)
{
	mSettings.getFriendlyName(friendlyName);
}


void Session::setFriendlyNameSetting(const std::string &friendlyName)
{
	mSettings.setFriendlyName(friendlyName);
}


void Session::getSerialNumberSetting(std::string *serialNumber)
{
	mSettings.getSerialNumber(serialNumber);
}


void Session::setSerialNumberSetting(const std::string &serialNumber)
{
	mSettings.setSerialNumber(serialNumber);
}


void Session::getSoftwareVersionSetting(std::string *softwareVersion)
{
	mSettings.getSoftwareVersion(softwareVersion);
}


void Session::setSoftwareVersionSetting(const std::string &softwareVersion)
{
	mSettings.setSoftwareVersion(softwareVersion);
}


enum pdraw_pipeline_mode Session::getPipelineModeSetting(void)
{
	return mSettings.getPipelineMode();
}


void Session::setPipelineModeSetting(enum pdraw_pipeline_mode mode)
{
	if (mState != READY) {
		ULOGE("%s: invalid state", __func__);
		return;
	}

	mSettings.setPipelineMode(mode);
}


void Session::getDisplayScreenSettings(float *xdpi,
				       float *ydpi,
				       float *deviceMarginTop,
				       float *deviceMarginBottom,
				       float *deviceMarginLeft,
				       float *deviceMarginRight)
{
	mSettings.getDisplayScreenSettings(xdpi,
					   ydpi,
					   deviceMarginTop,
					   deviceMarginBottom,
					   deviceMarginLeft,
					   deviceMarginRight);
}


void Session::setDisplayScreenSettings(float xdpi,
				       float ydpi,
				       float deviceMarginTop,
				       float deviceMarginBottom,
				       float deviceMarginLeft,
				       float deviceMarginRight)
{
	mSettings.setDisplayScreenSettings(xdpi,
					   ydpi,
					   deviceMarginTop,
					   deviceMarginBottom,
					   deviceMarginLeft,
					   deviceMarginRight);
}


enum pdraw_hmd_model Session::getHmdModelSetting(void)
{
	return mSettings.getHmdModelSetting();
}


void Session::setHmdModelSetting(enum pdraw_hmd_model hmdModel)
{
	mSettings.setHmdModelSetting(hmdModel);
}


int Session::dumpPipeline(const std::string &fileName)
{
	return mFactory.dumpPipeline(fileName);
}


/*
 * Internal methods
 */

void Session::asyncElementStateChange(Element *element, Element::State state)
{
	pthread_mutex_lock(&mAsyncMutex);
	mElementStateChangeElementArgs.push(element);
	mElementStateChangeStateArgs.push(state);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementStateChange, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


int Session::addMediaToRenderer(unsigned int mediaId,
				Pdraw::VideoRenderer *renderer)
{
	return mFactory.addMediaToRenderer(mediaId, renderer);
}


void Session::asyncElementDelete(Element *element)
{
	pthread_mutex_lock(&mAsyncMutex);
	mElementDeleteElementArgs.push(element);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementDelete, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


void Session::setState(enum State state)
{
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	mState = state;
	pthread_mutex_unlock(&mMutex);
	ULOGI("state change to %s", stateStr(state));
}


void Session::socketCreated(int fd)
{
	if (mListener != nullptr)
		mListener->onSocketCreated(this, fd);
}


/**
 * Calls from idle functions
 */

void Session::idleElementStateChange(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	Element *element = self->mElementStateChangeElementArgs.front();
	Element::State state = self->mElementStateChangeStateArgs.front();
	self->mElementStateChangeElementArgs.pop();
	self->mElementStateChangeStateArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	ULOG_ERRNO_RETURN_IF(element == nullptr, EINVAL);
	self->onElementStateChanged(element, state);
}


void Session::idleElementDelete(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	Element *element = self->mElementDeleteElementArgs.front();
	self->mElementDeleteElementArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	ULOG_ERRNO_RETURN_IF(element == nullptr, EINVAL);

	pthread_mutex_lock(&self->mMutex);
	std::vector<Element *>::iterator e = self->mElements.begin();
	while (e != self->mElements.end()) {
		if (*e != element) {
			e++;
			continue;
		}
		self->mElements.erase(e);
		delete element;
		break;
	}
	pthread_mutex_unlock(&self->mMutex);
}


void Session::callStopResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	int status = self->mStopRespStatusArgs.front();
	self->mStopRespStatusArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	if (self->mListener == nullptr)
		return;
	self->mListener->stopResponse(self, status);
}


void Session::callOnMediaAdded(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	struct pdraw_media_info info = self->mMediaAddedInfoArgs.front();
	self->mMediaAddedInfoArgs.pop();
	void *elementUserData = self->mMediaAddedElementUserDataArgs.front();
	self->mMediaAddedElementUserDataArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	self->mListener->onMediaAdded(self, &info, elementUserData);
	Media::cleanupMediaInfo(&info);
}


void Session::callOnMediaRemoved(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	struct pdraw_media_info info = self->mMediaRemovedInfoArgs.front();
	self->mMediaRemovedInfoArgs.pop();
	void *elementUserData = self->mMediaRemovedElementUserDataArgs.front();
	self->mMediaRemovedElementUserDataArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	self->mListener->onMediaRemoved(self, &info, elementUserData);
	Media::cleanupMediaInfo(&info);
}


/* Must be called on the loop thread */
void Session::onElementStateChanged(Element *element, Element::State state)
{
	mFactory.onElementStateChanged(element, state);

	if (state == Element::State::STOPPED) {
		bool stopped = true;
		State curState;

		pthread_mutex_lock(&mMutex);

		curState = mState;

		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if ((*e)->getState() != Element::State::STOPPED) {
				stopped = false;
				break;
			}
			e++;
		}

		pthread_mutex_unlock(&mMutex);

		asyncElementDelete(element);

		if (stopped && curState == STOPPING) {
			setState(STOPPED);

			if (mListener != nullptr)
				stopResp(0);
		}
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaAdded(Source *source,
				 Media *media,
				 void *elementUserData)
{
	ULOGD("onOutputMediaAdded(raw) name=%s", media->getName().c_str());

	mFactory.onOutputMediaAdded(source, media);

	if (mListener != nullptr) {
		struct pdraw_media_info info;
		media->fillMediaInfo(&info);
		if (pthread_self() == mLoopThread) {
			mListener->onMediaAdded(this, &info, elementUserData);
			Media::cleanupMediaInfo(&info);
		} else {
			pthread_mutex_lock(&mAsyncMutex);
			mMediaAddedInfoArgs.push(info);
			mMediaAddedElementUserDataArgs.push(elementUserData);
			int err = pomp_loop_idle_add_with_cookie(
				mLoop, callOnMediaAdded, this, this);
			if (err > 0) {
				ULOG_ERRNO("pomp_loop_idle_add_with_cookie",
					   -err);
			}
			pthread_mutex_unlock(&mAsyncMutex);
		}
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaRemoved(Source *source,
				   Media *media,
				   void *elementUserData)
{
	ULOGD("onOutputMediaRemoved name=%s", media->getName().c_str());

	mFactory.onOutputMediaRemoved(source, media);

	if (mListener != nullptr) {
		struct pdraw_media_info info;
		media->fillMediaInfo(&info);
		pthread_mutex_lock(&mAsyncMutex);
		mMediaRemovedInfoArgs.push(info);
		mMediaRemovedElementUserDataArgs.push(elementUserData);
		int err = pomp_loop_idle_add_with_cookie(
			mLoop, callOnMediaRemoved, this, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
		pthread_mutex_unlock(&mAsyncMutex);
	}
}


void Session::stopResp(int status)
{
	pthread_mutex_lock(&mAsyncMutex);
	mStopRespStatusArgs.push(status);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, callStopResponse, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


const char *Session::stateStr(enum State val)
{
	switch (val) {
	case State::STOPPED:
		return "STOPPED";
	case State::READY:
		return "READY";
	case State::STOPPING:
		return "STOPPING";
	default:
		return nullptr;
	}
}


Session::PipelineFactory::PipelineFactory(Session *session) : mSession(session)
{
	return;
}


Session::PipelineFactory::~PipelineFactory(void)
{
	return;
}


void Session::PipelineFactory::onElementStateChanged(Element *element,
						     Element::State state)
{
	if (state == Element::State::STARTED) {
		Pdraw::VideoRenderer *r =
			dynamic_cast<Pdraw::VideoRenderer *>(element);
		if (r != nullptr) {
			int ret = addAllMediaToRenderer(r);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToRenderer", -ret);
		}
	}
}


void Session::PipelineFactory::onOutputMediaAdded(Source *source, Media *media)
{
	Pdraw::Demuxer *demuxer = dynamic_cast<Pdraw::Demuxer *>(source);
	VideoDecoder *decoder = dynamic_cast<VideoDecoder *>(source);
	CodedVideoMedia *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);
	if ((demuxer != nullptr) && (codedMedia != nullptr)) {
		if (mSession->mSettings.getPipelineMode() ==
		    PDRAW_PIPELINE_MODE_DECODE_ALL) {
			int ret = addDecoderForMedia(source, codedMedia);
			if (ret < 0)
				ULOG_ERRNO("addDecoderForMedia", -ret);
		}
	} else if ((decoder != nullptr) && (rawMedia != nullptr)) {
		int ret = addMediaToAllRenderers(source, rawMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllRenderers", -ret);
	}
}


void Session::PipelineFactory::onOutputMediaRemoved(Source *source,
						    Media *media)
{
	return;
}


int Session::PipelineFactory::dumpPipeline(const std::string &fileName)
{
	int ret;
	FILE *f;

	f = fopen(fileName.c_str(), "w");
	if (f == nullptr) {
		ret = -errno;
		ULOG_ERRNO("fopen", -ret);
		return ret;
	}

	fprintf(f, "digraph {\n");
	fprintf(f, "\tnode [margin=0.2,fontsize=12];\n");

	pthread_mutex_lock(&mSession->mMutex);

	/* First pass: list the elements with their sink and source medias */
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		unsigned int elmId = (*e)->getId();
		const char *elmName = (*e)->getName().c_str();
		fprintf(f, "\te%u [shape=record,label=\"", elmId);

		/* Element input medias */
		Sink *sink = dynamic_cast<Sink *>(*e);
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			if (count > 0)
				fprintf(f, "{ ");
			for (unsigned int i = 0; i < count; i++) {
				Media *media = sink->getInputMedia(i);
				if (media == nullptr)
					continue;
				fprintf(f,
					"%s<e%um%u> %s",
					(i > 0) ? " | " : "",
					elmId,
					media->id,
					media->getName().c_str());
			}
			if (count > 0)
				fprintf(f, " } | ");
		}

		/* Element name */
		fprintf(f, "<e%u> %s", elmId, elmName);

		/* Element output medias */
		Source *source = dynamic_cast<Source *>(*e);
		if (source != nullptr) {
			unsigned int count = source->getOutputMediaCount();
			if (count > 0)
				fprintf(f, " | { ");
			for (unsigned int i = 0; i < count; i++) {
				Media *media = source->getOutputMedia(i);
				if (media == nullptr)
					continue;
				fprintf(f,
					"%s<e%um%u> %s",
					(i > 0) ? " | " : "",
					elmId,
					media->id,
					media->getName().c_str());
			}
			if (count > 0)
				fprintf(f, " }");
		}

		fprintf(f, "\"];\n");
		e++;
	}

	/* Second pass: list the links between sources and sinks */
	e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		unsigned int dstElmId = (*e)->getId();

		/* Element input medias */
		Sink *sink = dynamic_cast<Sink *>(*e);
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			for (unsigned int i = 0; i < count; i++) {
				Media *media = sink->getInputMedia(i);
				std::vector<Element *>::iterator e2 =
					mSession->mElements.begin();
				while (e2 != mSession->mElements.end()) {
					Source *source =
						dynamic_cast<Source *>(*e2);
					if (source != nullptr) {
						if (source->findOutputMedia(
							    media) == nullptr) {
							e2++;
							continue;
						}
						unsigned int srcElmId =
							(*e2)->getId();
						fprintf(f,
							"\te%u:e%um%u -> "
							"e%u:e%um%u;\n",
							srcElmId,
							srcElmId,
							media->id,
							dstElmId,
							dstElmId,
							media->id);
						break;
					}
					e2++;
				}
			}
		}

		e++;
	}

	pthread_mutex_unlock(&mSession->mMutex);

	fprintf(f, "}");
	fclose(f);

	ULOGI("pipeline dumped to file '%s'", fileName.c_str());

	return 0;
}


int Session::PipelineFactory::addDecoderForMedia(Source *source,
						 CodedVideoMedia *media)
{
	int ret;

	VideoDecoder *decoder = new VideoDecoder(mSession, mSession, mSession);
	if (decoder == nullptr) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}
	ret = decoder->addInputMedia(media);
	if (ret < 0) {
		if (ret == -ENOSYS)
			ret = 0;
		else
			ULOG_ERRNO("decoder->addInputMedia", -ret);
		delete decoder;
		return ret;
	}
	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(decoder);
	pthread_mutex_unlock(&mSession->mMutex);
	ret = decoder->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoder->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get decoder input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	/* Force a resync after linking the elements; this allows a H.264
	 * decoder to start on an IDR frame for example */
	decoder->resync();

	return 0;
}


int Session::PipelineFactory::addEncoderForMedia(
	Source *source,
	RawVideoMedia *media,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener,
	VideoEncoder *encoder)
{
	int ret;
	bool allocated = false;
	Channel *channel = nullptr;

	if (encoder == nullptr) {
		encoder = new VideoEncoder(mSession,
					   mSession,
					   mSession,
					   listener,
					   nullptr,
					   params);
		if (encoder == nullptr) {
			ULOGE("encoder creation failed");
			return -ENOMEM;
		}
		allocated = true;
	}

	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(encoder);
	pthread_mutex_unlock(&mSession->mMutex);

	ret = encoder->addInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("VideoEncoder::addInputMedia", -ret);
		goto error;
	}
	ret = encoder->start();
	if (ret < 0) {
		ULOG_ERRNO("VideoEncoder::start", -ret);
		goto error;
	}
	channel = encoder->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get encoder input channel");
		ret = -EPROTO;
		goto error;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -ret);
		goto error;
	}

	return 0;

error:
	if (encoder != nullptr) {
		if (channel != nullptr)
			source->removeOutputChannel(media, channel);
		std::vector<Element *>::iterator e =
			mSession->mElements.begin();
		while (e != mSession->mElements.end()) {
			if (*e != encoder) {
				e++;
				continue;
			}
			mSession->mElements.erase(e);
			break;
		}
		if (allocated)
			delete encoder;
	}
	return ret;
}

int Session::PipelineFactory::addScalerForMedia(Source *source,
						RawVideoMedia *media)
{
	int ret;
	VideoScaler *scaler = new VideoScaler(mSession, mSession, mSession);
	if (scaler == nullptr) {
		ULOGE("scaler creation failed");
		return -ENOMEM;
	}

	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(scaler);
	pthread_mutex_unlock(&mSession->mMutex);
	ret = scaler->addInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("scaler->addInputMedia", -ret);
		return ret;
	}
	ret = scaler->start();
	if (ret < 0) {
		ULOG_ERRNO("scaler->start", -ret);
		return ret;
	}
	Channel *channel = scaler->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get scaler input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}

	return 0;
}


int Session::PipelineFactory::addMediaToRenderer(Source *source,
						 RawVideoMedia *media,
						 Pdraw::VideoRenderer *renderer)
{
	int ret;

	ret = renderer->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("renderer->addInputMedia", -ret);
		return ret;
	}
	Channel *channel = renderer->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get renderer input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToRenderer(unsigned int mediaId,
						 Pdraw::VideoRenderer *renderer)
{
	int ret;
	bool found = false;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			RawVideoMedia *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToRenderer", -ret);
			found = true;
			break;
		}
		if (found)
			break;
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}


int Session::PipelineFactory::addMediaToAllRenderers(Source *source,
						     RawVideoMedia *media)
{
	int ret = 0;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		Pdraw::VideoRenderer *r =
			dynamic_cast<Pdraw::VideoRenderer *>(*e);
		e++;
		if (r == nullptr)
			continue;
		ret = addMediaToRenderer(source, media, r);
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return ret;
}


int Session::PipelineFactory::addAllMediaToRenderer(
	Pdraw::VideoRenderer *renderer)
{
	int ret;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			RawVideoMedia *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToRenderer", -ret);
		}
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}

} /* namespace Pdraw */
