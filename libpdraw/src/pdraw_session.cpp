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

#include "pdraw_session.hpp"
#include "pdraw_avcdecoder.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_utils.hpp"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#define ULOG_TAG pdraw_session
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_session);

namespace Pdraw {


enum cmd_type {
	CMD_TYPE_ELEMENT_SET_STATE,
	CMD_TYPE_ELEMENT_DELETE,
	CMD_TYPE_AVCDECODER_COMPLETE_FLUSH,
	CMD_TYPE_AVCDECODER_COMPLETE_STOP,
	CMD_TYPE_AVCDECODER_RESYNC,
	CMD_TYPE_RENDERER_COMPLETE_STOP,
};


struct cmd_base {
	enum cmd_type type;
};


struct cmd_element_set_state {
	struct cmd_base base;
	Element *element;
	Element::State state;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_element_set_state) <= PIPE_BUF - 1);


struct cmd_element_delete {
	struct cmd_base base;
	Element *element;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_element_delete) <= PIPE_BUF - 1);


struct cmd_avcdecoder_complete_flush {
	struct cmd_base base;
	AvcDecoder *decoder;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_avcdecoder_complete_flush) <=
		    PIPE_BUF - 1);


struct cmd_avcdecoder_complete_stop {
	struct cmd_base base;
	AvcDecoder *decoder;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_avcdecoder_complete_stop) <=
		    PIPE_BUF - 1);


struct cmd_avcdecoder_resync {
	struct cmd_base base;
	AvcDecoder *decoder;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_avcdecoder_resync) <= PIPE_BUF - 1);


struct cmd_renderer_complete_stop {
	struct cmd_base base;
	Renderer *renderer;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_renderer_complete_stop) <= PIPE_BUF - 1);


int createPdraw(struct pomp_loop *loop,
		IPdraw::Listener *listener,
		IPdraw **retObj)
{
	IPdraw *pdraw = NULL;
	if (loop == NULL) {
		ULOGE("invalid loop");
		return -EINVAL;
	}
	if (retObj == NULL) {
		ULOGE("invalid pointer");
		return -EINVAL;
	}
	pdraw = new Session(loop, listener);
	if (pdraw == NULL) {
		ULOGE("failed to create pdraw instance");
		return -EPROTO;
	}
	*retObj = pdraw;
	return 0;
}


const char *pdrawDroneModelStr(enum pdraw_drone_model val)
{
	return pdraw_droneModelStr(val);
}


const char *pdrawHmdModelStr(enum pdraw_hmd_model val)
{
	return pdraw_hmdModelStr(val);
}


const char *pdrawPipelineModeStr(enum pdraw_pipeline_mode val)
{
	return pdraw_pipelineModeStr(val);
}


const char *pdrawSessionTypeStr(enum pdraw_session_type val)
{
	return pdraw_sessionTypeStr(val);
}


const char *pdrawMediaTypeStr(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


const char *pdrawVideoMediaFormatStr(enum pdraw_video_media_format val)
{
	return pdraw_videoMediaFormatStr(val);
}


const char *pdrawYuvFormatStr(enum pdraw_yuv_format val)
{
	return pdraw_yuvFormatStr(val);
}


const char *pdrawH264FormatStr(enum pdraw_h264_format val)
{
	return pdraw_h264FormatStr(val);
}


const char *pdrawVideoTypeStr(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


const char *pdrawHistogramChannelStr(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
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


int pdrawVideoFrameToJsonStr(const struct pdraw_video_frame *frame,
			     char *str,
			     unsigned int len)
{
	return pdraw_frameMetadataToJsonStr(frame, str, len);
}


int pdrawVideoFrameToJson(const struct pdraw_video_frame *frame,
			  struct json_object *jobj)
{
	return pdraw_frameMetadataToJson(frame, jobj);
}


Session::Session(struct pomp_loop *loop, IPdraw::Listener *listener)
{
	int res;
	pthread_mutexattr_t attr;
	bool mutex_created = false, attr_created = false;

	mListener = listener;
	mState = INVALID;
	mSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
	mDemuxer = NULL;
	mDemuxerReady = false;
	mMediaIdCounter = 0;
	mLoop = loop;
	mAndroidJvm = NULL;
	mUrecoverableErrorOccured = false;
	mReadyToPlay = false;
	mMbox = NULL;

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
		ULOG_ERRNO("pthread_mutex_init", res);
		goto error;
	}
	mutex_created = true;

	mMbox = mbox_new(PIPE_BUF - 1);
	if (mMbox == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("mbox_new", -res);
		goto error;
	}

	res = pomp_loop_add(mLoop,
			    mbox_get_read_fd(mMbox),
			    POMP_FD_EVENT_IN,
			    &mboxCb,
			    this);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_add", -res);
		goto error;
	}

	pthread_mutexattr_destroy(&attr);
	setState(CREATED);
	return;

error:
	if (mMbox != NULL) {
		if (mLoop != NULL) {
			res = pomp_loop_remove(mLoop, mbox_get_read_fd(mMbox));
			if (res < 0)
				ULOG_ERRNO("pomp_loop_remove", -res);
		}
		mbox_destroy(mMbox);
		mMbox = NULL;
	}
	if (mutex_created)
		pthread_mutex_destroy(&mMutex);
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


Session::~Session(void)
{
	int res;

	if ((mState != INVALID) && (mState != CREATED) && (mState != CLOSED))
		ULOGW("destroying while instance is still running");

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		delete *e;
		e++;
	}
	mElements.clear();
	pthread_mutex_unlock(&mMutex);

	/* Remove any leftover idle callbacks */
	if (mLoop != NULL) {
		pomp_loop_idle_remove(mLoop, callOpenResponse, this);
		pomp_loop_idle_remove(mLoop, callCloseResponse, this);
		pomp_loop_idle_remove(mLoop, callOnUnrecoverableError, this);
		pomp_loop_idle_remove(mLoop, callOnMediaAdded, this);
		pomp_loop_idle_remove(mLoop, callOnMediaRemoved, this);
		pomp_loop_idle_remove(mLoop, callReadyToPlay, this);
		pomp_loop_idle_remove(mLoop, callEndOfRange, this);
		pomp_loop_idle_remove(mLoop, callPlayResponse, this);
		pomp_loop_idle_remove(mLoop, callPauseResponse, this);
		pomp_loop_idle_remove(mLoop, callSeekResponse, this);
	}

	if (mMbox != NULL) {
		if (mLoop != NULL) {
			res = pomp_loop_remove(mLoop, mbox_get_read_fd(mMbox));
			if (res < 0)
				ULOG_ERRNO("pomp_loop_remove", -res);
		}
		mbox_destroy(mMbox);
		mMbox = NULL;
	}

	pthread_mutex_destroy(&mMutex);
}


/*
 * API methods
 */

int Session::open(const std::string &url)
{
	int ret;
	std::string ext;

	if (mState != CREATED) {
		ULOGE("%s: invalid state", __func__);
		return -EPROTO;
	}

	if (mDemuxer != NULL) {
		ULOGE("%s: a demuxer is already running", __func__);
		return -EBUSY;
	}

	setState(OPENING);

	ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if ((url.front() == '/') && (ext == ".mp4")) {
		pthread_mutex_lock(&mMutex);
		mSessionType = PDRAW_SESSION_TYPE_REPLAY;
		pthread_mutex_unlock(&mMutex);
		mDemuxer = new RecordDemuxer(this, this, this, this);
		if (mDemuxer == NULL) {
			ULOGE("failed to alloc demuxer");
			ret = -ENOMEM;
			goto error_restore;
		}
		pthread_mutex_lock(&mMutex);
		mElements.push_back(mDemuxer);
		pthread_mutex_unlock(&mMutex);
		ret = ((RecordDemuxer *)mDemuxer)->setup(url);
		if (ret < 0) {
			ULOG_ERRNO("demuxer->setup", -ret);
			pthread_mutex_lock(&mMutex);
			mElements.erase(std::remove(mElements.begin(),
						    mElements.end(),
						    mDemuxer),
					mElements.end());
			pthread_mutex_unlock(&mMutex);
			delete mDemuxer;
			mDemuxer = NULL;
			mDemuxerReady = false;
			goto error_restore;
		}
		ret = mDemuxer->start();
		if (ret < 0) {
			ULOG_ERRNO("demuxer->start", -ret);
			pthread_mutex_lock(&mMutex);
			mElements.erase(std::remove(mElements.begin(),
						    mElements.end(),
						    mDemuxer),
					mElements.end());
			pthread_mutex_unlock(&mMutex);
			delete mDemuxer;
			mDemuxer = NULL;
			mDemuxerReady = false;
			goto error_restore;
		}
	} else if (url.substr(0, 7) == "rtsp://") {
		pthread_mutex_lock(&mMutex);
		mSessionType = PDRAW_SESSION_TYPE_LIVE; /* TODO: live/replay */
		pthread_mutex_unlock(&mMutex);
		mDemuxer = new StreamDemuxerNet(this, this, this, this);
		if (mDemuxer == NULL) {
			ULOGE("failed to alloc demuxer");
			ret = -ENOMEM;
			goto error_restore;
		}
		pthread_mutex_lock(&mMutex);
		mElements.push_back(mDemuxer);
		pthread_mutex_unlock(&mMutex);
		ret = ((StreamDemuxerNet *)mDemuxer)->setup(url);
		if (ret < 0) {
			ULOG_ERRNO("demuxer->setup", -ret);
			pthread_mutex_lock(&mMutex);
			mElements.erase(std::remove(mElements.begin(),
						    mElements.end(),
						    mDemuxer),
					mElements.end());
			pthread_mutex_unlock(&mMutex);
			delete mDemuxer;
			mDemuxer = NULL;
			mDemuxerReady = false;
			goto error_restore;
		}
		ret = mDemuxer->start();
		if (ret < 0) {
			ULOG_ERRNO("demuxer->start", -ret);
			pthread_mutex_lock(&mMutex);
			mElements.erase(std::remove(mElements.begin(),
						    mElements.end(),
						    mDemuxer),
					mElements.end());
			pthread_mutex_unlock(&mMutex);
			delete mDemuxer;
			mDemuxer = NULL;
			mDemuxerReady = false;
			goto error_restore;
		}
	} else {
		ULOGE("unsupported URL");
		ret = -ENOSYS;
		goto error_restore;
	}

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error_restore:
	/* State was OPENING, restore it to CREATED */
	setState(CREATED);
	return ret;
}


int Session::open(const std::string &localAddr,
		  uint16_t localStreamPort,
		  uint16_t localControlPort,
		  const std::string &remoteAddr,
		  uint16_t remoteStreamPort,
		  uint16_t remoteControlPort)
{
	int ret;

	if (mState != CREATED) {
		ULOGE("%s: invalid state", __func__);
		return -EPROTO;
	}

	if (mDemuxer != NULL) {
		ULOGE("%s: a demuxer is already running", __func__);
		return -EBUSY;
	}

	setState(OPENING);

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_LIVE;
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerNet(this, this, this, this);
	if (mDemuxer == NULL) {
		ULOGE("failed to alloc demuxer");
		ret = -ENOMEM;
		goto error_restore;
	}
	pthread_mutex_lock(&mMutex);
	mElements.push_back(mDemuxer);
	pthread_mutex_unlock(&mMutex);

	ret = ((StreamDemuxerNet *)mDemuxer)
		      ->setup(localAddr,
			      localStreamPort,
			      localControlPort,
			      remoteAddr,
			      remoteStreamPort,
			      remoteControlPort);
	if (ret < 0) {
		ULOG_ERRNO("demuxer->setup", -ret);
		pthread_mutex_lock(&mMutex);
		mElements.erase(std::remove(mElements.begin(),
					    mElements.end(),
					    mDemuxer),
				mElements.end());
		pthread_mutex_unlock(&mMutex);
		delete mDemuxer;
		mDemuxer = NULL;
		mDemuxerReady = false;
		goto error_restore;
	}
	ret = mDemuxer->start();
	if (ret < 0) {
		ULOG_ERRNO("demuxer->start", -ret);
		pthread_mutex_lock(&mMutex);
		mElements.erase(std::remove(mElements.begin(),
					    mElements.end(),
					    mDemuxer),
				mElements.end());
		pthread_mutex_unlock(&mMutex);
		delete mDemuxer;
		mDemuxer = NULL;
		mDemuxerReady = false;
		goto error_restore;
	}

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error_restore:
	/* State was OPENING, restore it to CREATED */
	setState(CREATED);
	return ret;
}


int Session::open(const std::string &url, struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
	int ret;

	if (mState != CREATED) {
		ULOGE("%s: invalid state", __func__);
		return -EPROTO;
	}

	if (mDemuxer != NULL) {
		ULOGE("%s: a demuxer is already running", __func__);
		return -EBUSY;
	}

	setState(OPENING);

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_LIVE; /* TODO: live/replay */
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerMux(this, this, this, this);
	if (mDemuxer == NULL) {
		ULOGE("failed to alloc demuxer");
		ret = -ENOMEM;
		goto error_restore;
	}
	pthread_mutex_lock(&mMutex);
	mElements.push_back(mDemuxer);
	pthread_mutex_unlock(&mMutex);

	ret = ((StreamDemuxerMux *)mDemuxer)->setup(url, mux);
	if (ret < 0) {
		ULOG_ERRNO("demuxer->setup", -ret);
		pthread_mutex_lock(&mMutex);
		mElements.erase(std::remove(mElements.begin(),
					    mElements.end(),
					    mDemuxer),
				mElements.end());
		pthread_mutex_unlock(&mMutex);
		delete mDemuxer;
		mDemuxer = NULL;
		mDemuxerReady = false;
		goto error_restore;
	}
	ret = mDemuxer->start();
	if (ret < 0) {
		ULOG_ERRNO("demuxer->start", -ret);
		pthread_mutex_lock(&mMutex);
		mElements.erase(std::remove(mElements.begin(),
					    mElements.end(),
					    mDemuxer),
				mElements.end());
		pthread_mutex_unlock(&mMutex);
		delete mDemuxer;
		mDemuxer = NULL;
		mDemuxerReady = false;
		goto error_restore;
	}

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error_restore:
	/* State was OPENING, restore it to CREATED */
	setState(CREATED);
	return ret;
#else /* BUILD_LIBMUX */
	return -ENOSYS;
#endif /* BUILD_LIBMUX */
}


int Session::close(void)
{
	int ret;

	if (mState == CLOSING) {
		/* Return without calling the closeResponse() function */
		ULOGI("%s: already in %s state, nothing to do",
		      __func__,
		      stateStr(mState));
		return 0;
	}

	if (mState == CREATED || mState == CLOSED) {
		/* Call the closeResponse() function with OK status */
		ULOGI("%s: state is %s, nothing to do",
		      __func__,
		      stateStr(mState));
		ret = 0;
		goto already_closed;
	}

	if (mState != OPENED && mState != OPENING) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	setState(CLOSING);

	if (mDemuxer == NULL) {
		ULOGI("%s: no demuxer is running", __func__);

		bool stopped = true;
		pthread_mutex_lock(&mMutex);
		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if ((*e)->getState() != Element::State::STOPPED) {
				stopped = false;
				break;
			}
			e++;
		}
		pthread_mutex_unlock(&mMutex);

		if (stopped) {
			/* Call the closeResponse() function with OK status */
			ULOGI("%s: all elements are stopped, closing",
			      __func__);
			setState(CLOSED);
			ret = 0;
			goto already_closed;
		}

		/* Waiting for the asynchronous stop of all elements;
		 * closeResponse() will be called when it's done */
		return 0;
	}

	ret = mDemuxer->stop();
	if (ret < 0) {
		ULOG_ERRNO("demuxer->stop", -ret);
		return ret;
	}
	/* Waiting for the asynchronous stop; closeResponse()
	 * will be called when it's done */
	return 0;

already_closed:
	if (mListener != NULL && ret == 0) {
		mCloseRespStatusArgs.push(ret);
		pomp_loop_idle_add(mLoop, callCloseResponse, this);
	}
	return ret;
}


uint16_t Session::getSingleStreamLocalStreamPort(void)
{
	if (mDemuxer == NULL) {
		ULOG_ERRNO("invalid demuxer", EPROTO);
		return 0;
	}

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == NULL) {
		ULOG_ERRNO("invalid demuxer", ENOSYS);
		return 0;
	}

	return demuxer->getSingleStreamLocalStreamPort();
}


uint16_t Session::getSingleStreamLocalControlPort(void)
{
	if (mDemuxer == NULL) {
		ULOG_ERRNO("invalid demuxer", EPROTO);
		return 0;
	}

	StreamDemuxerNet *demuxer = dynamic_cast<StreamDemuxerNet *>(mDemuxer);
	if (demuxer == NULL) {
		ULOG_ERRNO("invalid demuxer", ENOSYS);
		return 0;
	}

	return demuxer->getSingleStreamLocalControlPort();
}


bool Session::isReadyToPlay(void)
{
	return mReadyToPlay;
}


bool Session::isPaused(void)
{
	/* TODO */
	if (mDemuxer != NULL)
		return mDemuxer->isPaused();
	else
		return false;
}


int Session::play(float speed)
{
	int ret = 0;

	if (mState != OPENED) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	if (mDemuxer == NULL) {
		ULOGE("%s: no demuxer is running", __func__);
		return -EPROTO;
	}

	ret = mDemuxer->play(speed);
	if (ret < 0)
		ULOG_ERRNO("demuxer->play", -ret);
	return ret;
}


int Session::pause(void)
{
	return play(0.);
}


int Session::previousFrame(void)
{
	int ret = 0;

	if (mState != OPENED) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	if (mDemuxer == NULL) {
		ULOGE("%s: no demuxer is running", __func__);
		return -EPROTO;
	}

	ret = mDemuxer->previous();
	if (ret < 0)
		ULOG_ERRNO("demuxer->previous", -ret);
	return ret;
}


int Session::nextFrame(void)
{
	int ret = 0;

	if (mState != OPENED) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	if (mDemuxer == NULL) {
		ULOGE("%s: no demuxer is running", __func__);
		return -EPROTO;
	}

	ret = mDemuxer->next();
	if (ret < 0)
		ULOG_ERRNO("demuxer->next", -ret);
	return ret;
}


int Session::seek(int64_t delta, bool exact)
{
	int ret = 0;

	if (mState != OPENED) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	if (mDemuxer == NULL) {
		ULOGE("%s: no demuxer is running", __func__);
		return -EPROTO;
	}

	ret = mDemuxer->seek(delta, exact);
	if (ret < 0)
		ULOG_ERRNO("demuxer->seek", -ret);
	return ret;
}


int Session::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta);
}


int Session::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta));
}


int Session::seekTo(uint64_t timestamp, bool exact)
{
	int ret = 0;

	if (mState != OPENED) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	if (mDemuxer == NULL) {
		ULOGE("%s: no demuxer is running", __func__);
		return -EPROTO;
	}

	ret = mDemuxer->seekTo(timestamp, exact);
	if (ret < 0)
		ULOG_ERRNO("demuxer->seekTo", -ret);
	return ret;
}


uint64_t Session::getDuration(void)
{
	/* TODO */
	return (mDemuxer) ? mDemuxer->getDuration() : 0;
}


uint64_t Session::getCurrentTime(void)
{
	/* TODO */
	return (mDemuxer) ? mDemuxer->getCurrentTime() : 0;
}


/* Called on the rendering thread */
int Session::startVideoRenderer(
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	VideoRendererListener *listener,
	struct pdraw_video_renderer **retObj,
	struct egl_display *eglDisplay)
{
	int res;
	Renderer *renderer = NULL;

	if (renderPos == NULL)
		return -EINVAL;
	if (params == NULL)
		return -EINVAL;
	if (listener == NULL)
		return -EINVAL;
	if (retObj == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);
	if (mState == CLOSING || mState == CLOSED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	renderer = Renderer::create(this, this, listener);
	if (renderer == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("failed to create renderer");
		return -EPROTO;
	}

	mElements.push_back(renderer);
	pthread_mutex_unlock(&mMutex);

	res = renderer->setup(renderPos, params, eglDisplay);
	if (res < 0) {
		ULOG_ERRNO("renderer->setup", -res);
		return res;
	}
	res = renderer->start();
	if (res < 0) {
		ULOG_ERRNO("renderer->start", -res);
		return res;
	}

	*retObj = (struct pdraw_video_renderer *)renderer;

	return 0;
}


/* Called on the rendering thread */
int Session::stopVideoRenderer(struct pdraw_video_renderer *renderer)
{
	if (renderer == NULL)
		return -EINVAL;

	Renderer *r = reinterpret_cast<Renderer *>(renderer);
	int ret = r->stop();
	if (ret < 0)
		ULOG_ERRNO("renderer->stop", -ret);

	return 0;
}


/* Called on the rendering thread */
int Session::resizeVideoRenderer(struct pdraw_video_renderer *renderer,
				 const struct pdraw_rect *renderPos)
{
	if (renderer == NULL)
		return -EINVAL;

	Renderer *r = reinterpret_cast<Renderer *>(renderer);
	return r->resize(renderPos);
}


/* Called on the rendering thread */
int Session::setVideoRendererParams(
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	if (renderer == NULL)
		return -EINVAL;

	Renderer *r = reinterpret_cast<Renderer *>(renderer);
	return r->setParams(params);
}


/* Called on the rendering thread */
int Session::getVideoRendererParams(struct pdraw_video_renderer *renderer,
				    struct pdraw_video_renderer_params *params)
{
	if (renderer == NULL)
		return -EINVAL;

	Renderer *r = reinterpret_cast<Renderer *>(renderer);
	return r->getParams(params);
}


/* Called on the rendering thread */
int Session::renderVideo(struct pdraw_video_renderer *renderer,
			 struct pdraw_rect *contentPos,
			 const float *viewMat,
			 const float *projMat)
{
	if (renderer == NULL)
		return -EINVAL;

	Renderer *r = reinterpret_cast<Renderer *>(renderer);
	return r->render(contentPos, viewMat, projMat);
}


int Session::startVideoSink(unsigned int mediaId,
			    const struct pdraw_video_sink_params *params,
			    VideoSinkListener *listener,
			    struct pdraw_video_sink **retObj)
{
	int res;
	Source *source = NULL;
	VideoSink *sink = NULL;
	VideoMedia *videoMedia = NULL;
	Channel *channel = NULL;
	bool found = false;

	if (params == NULL)
		return -EINVAL;
	if (listener == NULL)
		return -EINVAL;
	if (retObj == NULL)
		return -EINVAL;

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		source = dynamic_cast<Source *>(*e);
		if (source == NULL) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *media = source->getOutputMedia(i);
			videoMedia = dynamic_cast<VideoMedia *>(media);
			if (videoMedia == NULL)
				continue;
			if (videoMedia->id == mediaId) {
				found = true;
				break;
			}
		}
		if (found)
			break;
		e++;
	}
	if ((!found) || (source == NULL) || (videoMedia == NULL)) {
		pthread_mutex_unlock(&mMutex);
		return -ENOENT;
	}

	sink = new VideoSink(this, params->required_format, this);
	if (sink == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("video sink creation failed");
		return -ENOMEM;
	}
	mElements.push_back(sink);
	res = sink->addInputMedia(videoMedia);
	if (res < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("videoSink->addInputMedia", -res);
		goto error;
	}
	res = sink->setup(listener, params);
	if (res < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("videoSink->setup", -res);
		goto error;
	}
	res = sink->start();
	if (res < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("videoSink->start", -res);
		goto error;
	}
	channel = sink->getInputChannel(videoMedia);
	if (channel == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("failed to get video sink input channel");
		res = -EPROTO;
		goto error;
	}
	res = source->addOutputChannel(videoMedia, channel);
	if (res < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("source->addOutputChannel", -res);
		goto error;
	}
	/* Force a resync after linking the elements; this allows a H.264
	 * video sink to start on an IDR frame for example */
	res = sink->resync();
	if (res < 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("videoSink->resync", -res);
		goto error;
	}

	pthread_mutex_unlock(&mMutex);
	*retObj = (struct pdraw_video_sink *)sink;

	return 0;

error:
	if (sink != NULL) {
		if (channel != NULL) {
			source->removeOutputChannel(videoMedia,
						    channel->getKey());
		}
		pthread_mutex_lock(&mMutex);
		while (e != mElements.end()) {
			if (*e != sink) {
				e++;
				continue;
			}
			mElements.erase(e);
			break;
		}
		pthread_mutex_unlock(&mMutex);
		delete sink;
	}
	return res;
}


int Session::stopVideoSink(struct pdraw_video_sink *sink)
{
	if (sink == NULL)
		return -EINVAL;

	VideoSink *s = reinterpret_cast<VideoSink *>(sink);
	int ret = s->stop();
	if (ret < 0)
		ULOG_ERRNO("videoSink->stop", -ret);

	return ret;
}


int Session::resyncVideoSink(struct pdraw_video_sink *sink)
{
	if (sink == NULL)
		return -EINVAL;

	VideoSink *s = reinterpret_cast<VideoSink *>(sink);
	int ret = s->resync();
	if (ret < 0)
		ULOG_ERRNO("videoSink->resync", -ret);

	return ret;
}


struct vbuf_queue *Session::getVideoSinkQueue(struct pdraw_video_sink *sink)
{
	if (sink == NULL) {
		ULOG_ERRNO("sink", EINVAL);
		return NULL;
	}

	VideoSink *s = reinterpret_cast<VideoSink *>(sink);
	return s->getQueue();
}


int Session::videoSinkQueueFlushed(struct pdraw_video_sink *sink)
{
	if (sink == NULL)
		return -EINVAL;

	VideoSink *s = reinterpret_cast<VideoSink *>(sink);
	int ret = s->flushDone();
	if (ret < 0)
		ULOG_ERRNO("videoSink->flushDone", -ret);

	return ret;
}


enum pdraw_session_type Session::getSessionType(void)
{
	pthread_mutex_lock(&mMutex);
	enum pdraw_session_type ret = mSessionType;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Session::getSelfFriendlyName(std::string *friendlyName)
{
	mSelfMetadata.getFriendlyName(friendlyName);
}


void Session::setSelfFriendlyName(const std::string &friendlyName)
{
	mSelfMetadata.setFriendlyName(friendlyName);
}


void Session::getSelfSerialNumber(std::string *serialNumber)
{
	mSelfMetadata.getSerialNumber(serialNumber);
}


void Session::setSelfSerialNumber(const std::string &serialNumber)
{
	mSelfMetadata.setSerialNumber(serialNumber);
}


void Session::getSelfSoftwareVersion(std::string *softwareVersion)
{
	mSelfMetadata.getSoftwareVersion(softwareVersion);
}


void Session::setSelfSoftwareVersion(const std::string &softwareVersion)
{
	mSelfMetadata.setSoftwareVersion(softwareVersion);
}


bool Session::isSelfPilot(void)
{
	return mSelfMetadata.isPilot();
}


void Session::setSelfPilot(bool isPilot)
{
	mSelfMetadata.setPilot(isPilot);
}


void Session::getPeerSessionMetadata(struct vmeta_session *session)
{
	mPeerMetadata.get(session);
}


enum pdraw_drone_model Session::getPeerDroneModel(void)
{
	return mPeerMetadata.getDroneModel();
}


enum pdraw_pipeline_mode Session::getPipelineModeSetting(void)
{
	return mSettings.getPipelineMode();
}


void Session::setPipelineModeSetting(enum pdraw_pipeline_mode mode)
{
	if (mState != CREATED) {
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


/*
 * Internal methods
 */

int Session::getSessionInfo(struct pdraw_session_info *sessionInfo)
{
	if (sessionInfo == NULL)
		return -EINVAL;

	memset(sessionInfo, 0, sizeof(*sessionInfo));
	std::string fn;
	mSelfMetadata.getFriendlyName(&fn);
	strncpy(sessionInfo->friendly_name,
		fn.c_str(),
		sizeof(sessionInfo->friendly_name));
	sessionInfo->friendly_name[sizeof(sessionInfo->friendly_name) - 1] =
		'\0';
	std::string sn;
	mSelfMetadata.getSerialNumber(&sn);
	strncpy(sessionInfo->serial_number,
		sn.c_str(),
		sizeof(sessionInfo->serial_number));
	sessionInfo->serial_number[sizeof(sessionInfo->serial_number) - 1] =
		'\0';
	std::string sv;
	mSelfMetadata.getSoftwareVersion(&sv);
	strncpy(sessionInfo->software_version,
		sv.c_str(),
		sizeof(sessionInfo->software_version));
	sessionInfo
		->software_version[sizeof(sessionInfo->software_version) - 1] =
		'\0';
	sessionInfo->drone_model = mPeerMetadata.getDroneModel();
	sessionInfo->session_type = getSessionType();
	sessionInfo->is_pilot = mSelfMetadata.isPilot() ? 1 : 0;
	sessionInfo->duration = getDuration();

	return 0;
}


void Session::asyncElementStateChange(Element *element, Element::State state)
{
	if (element == NULL) {
		ULOG_ERRNO("element", EINVAL);
		return;
	}

	/* Send a message to the loop */
	int res;
	struct cmd_element_set_state *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return;
	}
	cmd = (struct cmd_element_set_state *)msg;
	cmd->base.type = CMD_TYPE_ELEMENT_SET_STATE;
	cmd->element = element;
	cmd->state = state;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
}


int Session::asyncElementDelete(Element *element)
{
	if (element == NULL)
		return -EINVAL;

	/* Send a message to the loop */
	int res;
	struct cmd_element_delete *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL)
		return -ENOMEM;
	cmd = (struct cmd_element_delete *)msg;
	cmd->base.type = CMD_TYPE_ELEMENT_DELETE;
	cmd->element = element;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
	return res;
}


void Session::inloopElementDelete(Element *element)
{
	if (element == NULL) {
		ULOG_ERRNO("element", EINVAL);
		return;
	}

	pthread_mutex_lock(&mMutex);

	bool found = false;
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		if (*e != element) {
			e++;
			continue;
		}
		found = true;
		mElements.erase(e);
		break;
	}

	if (found) {
		delete element;
		if (element == mDemuxer) {
			mDemuxer = NULL;
			mDemuxerReady = false;
		}
	}

	pthread_mutex_unlock(&mMutex);
}


int Session::asyncAvcDecoderCompleteFlush(AvcDecoder *decoder)
{
	if (decoder == NULL)
		return -EINVAL;

	/* Send a message to the loop */
	int res;
	struct cmd_avcdecoder_complete_flush *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL)
		return -ENOMEM;
	cmd = (struct cmd_avcdecoder_complete_flush *)msg;
	cmd->base.type = CMD_TYPE_AVCDECODER_COMPLETE_FLUSH;
	cmd->decoder = decoder;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
	return res;
}


void Session::inloopAvcDecoderCompleteFlush(AvcDecoder *decoder)
{
	if (decoder == NULL) {
		ULOG_ERRNO("decoder", EINVAL);
		return;
	}

	decoder->completeFlush();
}


int Session::asyncAvcDecoderCompleteStop(AvcDecoder *decoder)
{
	if (decoder == NULL)
		return -EINVAL;

	/* Send a message to the loop */
	int res;
	struct cmd_avcdecoder_complete_stop *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL)
		return -ENOMEM;
	cmd = (struct cmd_avcdecoder_complete_stop *)msg;
	cmd->base.type = CMD_TYPE_AVCDECODER_COMPLETE_STOP;
	cmd->decoder = decoder;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
	return res;
}


void Session::inloopAvcDecoderCompleteStop(AvcDecoder *decoder)
{
	if (decoder == NULL) {
		ULOG_ERRNO("decoder", EINVAL);
		return;
	}

	decoder->completeStop();
}


int Session::asyncAvcDecoderResync(AvcDecoder *decoder)
{
	if (decoder == NULL)
		return -EINVAL;

	/* Send a message to the loop */
	int res;
	struct cmd_avcdecoder_resync *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL)
		return -ENOMEM;
	cmd = (struct cmd_avcdecoder_resync *)msg;
	cmd->base.type = CMD_TYPE_AVCDECODER_RESYNC;
	cmd->decoder = decoder;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
	return res;
}


void Session::inloopAvcDecoderResync(AvcDecoder *decoder)
{
	if (decoder == NULL) {
		ULOG_ERRNO("decoder", EINVAL);
		return;
	}

	decoder->resync();
}


int Session::asyncRendererCompleteStop(Renderer *renderer)
{
	if (renderer == NULL)
		return -EINVAL;

	/* Send a message to the loop */
	int res;
	struct cmd_renderer_complete_stop *cmd = NULL;
	void *msg = calloc(PIPE_BUF - 1, 1);
	if (msg == NULL)
		return -ENOMEM;
	cmd = (struct cmd_renderer_complete_stop *)msg;
	cmd->base.type = CMD_TYPE_RENDERER_COMPLETE_STOP;
	cmd->renderer = renderer;
	res = mbox_push(mMbox, msg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
	free(msg);
	return res;
}


void Session::inloopRendererCompleteStop(Renderer *renderer)
{
	if (renderer == NULL) {
		ULOG_ERRNO("renderer", EINVAL);
		return;
	}

	renderer->completeStop();
}


void Session::fillMediaInfo(VideoMedia *media, struct pdraw_media_info *info)
{
	if (!media || !info)
		return;

	memset(info, 0, sizeof(*info));

	info->type = PDRAW_MEDIA_TYPE_VIDEO;
	info->id = media->id;
	info->video.format = (pdraw_video_media_format)media->format;
	info->video.type = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA;
	switch (media->format) {
	case VideoMedia::Format::YUV:
		info->video.yuv.width = media->width;
		info->video.yuv.height = media->height;
		info->video.yuv.crop_left = media->cropLeft;
		info->video.yuv.crop_width = media->cropWidth;
		info->video.yuv.crop_top = media->cropTop;
		info->video.yuv.crop_height = media->cropHeight;
		info->video.yuv.sar_width = media->sarWidth;
		info->video.yuv.sar_height = media->sarHeight;
		info->video.yuv.horizontal_fov = media->hfov;
		info->video.yuv.vertical_fov = media->vfov;
		info->video.yuv.full_range = media->fullRange;
		break;
	case VideoMedia::Format::H264:
		uint8_t *sps, *pps;
		size_t spslen, ppslen, cplen;
		info->video.h264.width = media->width;
		info->video.h264.height = media->height;
		media->getSpsPps(&sps, &spslen, &pps, &ppslen);
		cplen = MIN(spslen, sizeof(info->video.h264.sps));
		memcpy(info->video.h264.sps, sps, cplen);
		info->video.h264.spslen = cplen;
		cplen = MIN(ppslen, sizeof(info->video.h264.pps));
		memcpy(info->video.h264.pps, pps, cplen);
		info->video.h264.ppslen = cplen;
		break;
	default:
		break;
	}
}


int Session::addDecoderForMedia(Source *source, Media *media)
{
	int ret;

	VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
	if (videoMedia == NULL)
		return 0;
	AvcDecoder *decoder = new AvcDecoder(this, this, this);
	if (decoder == NULL) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}
	pthread_mutex_lock(&mMutex);
	mElements.push_back(decoder);
	pthread_mutex_unlock(&mMutex);
	ret = decoder->addInputMedia(videoMedia);
	if (ret < 0) {
		ULOG_ERRNO("decoder->addInputMedia", -ret);
		return ret;
	}
	ret = decoder->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoder->getInputChannel(videoMedia);
	if (channel == NULL) {
		ULOGE("failed to get decoder input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(videoMedia, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	/* Force a resync after linking the elements; this allows a H.264
	 * decoder to start on an IDR frame for example */
	decoder->resync();

	return 0;
}


int Session::addMediaToRenderer(Source *source,
				Media *media,
				Renderer *renderer)
{
	int ret;
	VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
	if (videoMedia == NULL)
		return -EPROTO;

	ret = renderer->addInputMedia(videoMedia);
	if (ret == -EEXIST) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("renderer->addInputMedia", -ret);
		return ret;
	}
	Channel *channel = renderer->getInputChannel(videoMedia);
	if (channel == NULL) {
		ULOGE("failed to get renderer input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(videoMedia, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::addMediaToAllRenderers(Source *source, Media *media)
{
	int ret = 0;

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end() && ret == 0) {
		Renderer *r = dynamic_cast<Renderer *>(*e);
		e++;
		if (r == NULL)
			continue;
		ret = addMediaToRenderer(source, media, r);
	}
	pthread_mutex_unlock(&mMutex);

	return ret;
}


int Session::addAllMediaToRenderer(Renderer *renderer)
{
	int ret;

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		AvcDecoder *decoder = dynamic_cast<AvcDecoder *>(*e);
		if (decoder == NULL) {
			e++;
			continue;
		}
		Media *media = decoder->getOutputMedia(0);
		if (media == NULL) {
			ULOGE("invalid media");
			e++;
			continue;
		}
		ret = addMediaToRenderer(decoder, media, renderer);
		if (ret < 0)
			ULOG_ERRNO("addMediaToRenderer", -ret);
		e++;
	}
	pthread_mutex_unlock(&mMutex);

	return 0;
}


void Session::setState(enum State state)
{
	enum State prev;
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	prev = mState;
	mState = state;
	pthread_mutex_unlock(&mMutex);
	ULOGI("state change to %s", stateStr(state));

	if (mListener == NULL)
		return;

	updateReadyToPlay();
}


void Session::socketCreated(int fd)
{
	if (mListener != NULL)
		mListener->onSocketCreated(this, fd);
}


int Session::flushVideoSink(VideoSink *sink)
{
	if (sink == NULL)
		return -EINVAL;

	struct pdraw_video_sink *s = (struct pdraw_video_sink *)sink;
	VideoSinkListener *listener = sink->getVideoSinkListener();
	if (listener == NULL)
		return -ENOENT;

	listener->onVideoSinkFlush(this, s);

	return 0;
}

/* Listener calls from idle functions */
void Session::callOpenResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	int status = self->mOpenRespStatusArgs.front();
	self->mOpenRespStatusArgs.pop();
	self->mListener->openResponse(self, status);
}


void Session::callCloseResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	int status = self->mCloseRespStatusArgs.front();
	self->mCloseRespStatusArgs.pop();
	self->mListener->closeResponse(self, status);
}


void Session::callOnUnrecoverableError(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	self->mListener->onUnrecoverableError(self);
}


void Session::callOnMediaAdded(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	struct pdraw_media_info info = self->mMediaAddedInfoArgs.front();
	self->mMediaAddedInfoArgs.pop();
	self->mListener->onMediaAdded(self, &info);
}


void Session::callOnMediaRemoved(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	struct pdraw_media_info info = self->mMediaRemovedInfoArgs.front();
	self->mMediaRemovedInfoArgs.pop();
	self->mListener->onMediaRemoved(self, &info);
}


void Session::callReadyToPlay(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	bool ready = self->mReadyToPlayReadyArgs.front();
	self->mReadyToPlayReadyArgs.pop();
	self->mListener->readyToPlay(self, ready);
}


void Session::callEndOfRange(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	uint64_t timestamp = self->mEndOfRangeTimestampArgs.front();
	self->mEndOfRangeTimestampArgs.pop();
	self->mListener->onEndOfRange(self, timestamp);
}


void Session::callPlayResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	int status = self->mPlayRespStatusArgs.front();
	uint64_t timestamp = self->mPlayRespTimestampArgs.front();
	float speed = self->mPlayRespSpeedArgs.front();
	self->mPlayRespStatusArgs.pop();
	self->mPlayRespTimestampArgs.pop();
	self->mPlayRespSpeedArgs.pop();
	self->mListener->playResponse(self, status, timestamp, speed);
}


void Session::callPauseResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	int status = self->mPauseRespStatusArgs.front();
	uint64_t timestamp = self->mPauseRespTimestampArgs.front();
	self->mPauseRespStatusArgs.pop();
	self->mPauseRespTimestampArgs.pop();
	self->mListener->pauseResponse(self, status, timestamp);
}


void Session::callSeekResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	int status = self->mSeekRespStatusArgs.front();
	uint64_t timestamp = self->mSeekRespTimestampArgs.front();
	float speed = self->mSeekRespSpeedArgs.front();
	self->mSeekRespStatusArgs.pop();
	self->mSeekRespTimestampArgs.pop();
	self->mSeekRespSpeedArgs.pop();
	self->mListener->seekResponse(self, status, timestamp, speed);
}


void Session::mboxCb(int fd, uint32_t revents, void *userdata)
{
	Session *self = (Session *)userdata;
	int res;
	void *msg;
	struct cmd_base *base;

	if (self == NULL)
		return;

	msg = malloc(PIPE_BUF - 1);
	if (msg == NULL) {
		ULOG_ERRNO("msg", ENOMEM);
		return;
	}

	do {
		/* Read from the mailbox */
		res = mbox_peek(self->mMbox, msg);
		if (res < 0) {
			if (res != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -res);
			break;
		}

		base = (struct cmd_base *)msg;
		switch (base->type) {
		case CMD_TYPE_ELEMENT_SET_STATE: {
			struct cmd_element_set_state *cmd =
				(struct cmd_element_set_state *)msg;
			self->onElementStateChanged(cmd->element, cmd->state);
			break;
		}
		case CMD_TYPE_ELEMENT_DELETE: {
			struct cmd_element_delete *cmd =
				(struct cmd_element_delete *)msg;
			self->inloopElementDelete(cmd->element);
			break;
		}
		case CMD_TYPE_AVCDECODER_COMPLETE_FLUSH: {
			struct cmd_avcdecoder_complete_flush *cmd =
				(struct cmd_avcdecoder_complete_flush *)msg;
			self->inloopAvcDecoderCompleteFlush(cmd->decoder);
			break;
		}
		case CMD_TYPE_AVCDECODER_COMPLETE_STOP: {
			struct cmd_avcdecoder_complete_stop *cmd =
				(struct cmd_avcdecoder_complete_stop *)msg;
			self->inloopAvcDecoderCompleteStop(cmd->decoder);
			break;
		}
		case CMD_TYPE_AVCDECODER_RESYNC: {
			struct cmd_avcdecoder_resync *cmd =
				(struct cmd_avcdecoder_resync *)msg;
			self->inloopAvcDecoderResync(cmd->decoder);
			break;
		}
		case CMD_TYPE_RENDERER_COMPLETE_STOP: {
			struct cmd_renderer_complete_stop *cmd =
				(struct cmd_renderer_complete_stop *)msg;
			self->inloopRendererCompleteStop(cmd->renderer);
			break;
		}
		default:
			ULOGE("unknown command");
			break;
		}
	} while (res == 0);

	free(msg);
}


/* Must be called on the loop thread */
void Session::onElementStateChanged(Element *element, Element::State state)
{
	int ret;

	switch (state) {
	case Element::State::STARTED: {
		if (element == mDemuxer) {
			setState(OPENED);

			if (mListener != NULL) {
				mOpenRespStatusArgs.push(0);
				pomp_loop_idle_add(
					mLoop, callOpenResponse, this);
			}
		} else {
			Renderer *r = dynamic_cast<Renderer *>(element);
			if (r != NULL) {
				ret = addAllMediaToRenderer(r);
				if (ret < 0)
					ULOG_ERRNO("addAllMediaToRenderer",
						   -ret);
			}
		}
		break;
	}
	case Element::State::STOPPED: {
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

		ret = asyncElementDelete(element);
		if (ret < 0)
			ULOG_ERRNO("asyncElementDelete", -ret);

		if (stopped && curState == CLOSING) {
			setState(CLOSED);

			if (mListener != NULL) {
				mCloseRespStatusArgs.push(0);
				pomp_loop_idle_add(
					mLoop, callCloseResponse, this);
			}
		} else if (curState == OPENING && element == mDemuxer) {
			setState(CREATED);
			if (mListener != NULL) {
				mOpenRespStatusArgs.push(-EPROTO);
				pomp_loop_idle_add(
					mLoop, callOpenResponse, this);
			}
		} else if (curState != OPENING && curState != CLOSING &&
			   element == mDemuxer) {
			onUnrecoverableError(mDemuxer);
		}
		break;
	}
	default:
		break;
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaAdded(Source *source, Media *media)
{
	int ret;

	ULOGD("onOutputMediaAdded id=%d type=%s",
	      media->id,
	      Media::getMediaTypeStr(media->type));

	if (source == mDemuxer) {
		if (mSettings.getPipelineMode() ==
		    PDRAW_PIPELINE_MODE_DECODE_ALL) {
			ret = addDecoderForMedia(source, media);
			if (ret < 0)
				ULOG_ERRNO("addDecoderForMedia", -ret);
		}
	} else {
		AvcDecoder *decoder = dynamic_cast<AvcDecoder *>(source);
		if ((decoder != NULL)) {
			ret = addMediaToAllRenderers(source, media);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAllRenderers", -ret);
		}
	}

	VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
	if (videoMedia != NULL) {
		if (mListener != NULL) {
			struct pdraw_media_info info;
			fillMediaInfo(videoMedia, &info);
			mMediaAddedInfoArgs.push(info);
			pomp_loop_idle_add(mLoop, callOnMediaAdded, this);
		}
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaRemoved(Source *source, Media *media)
{
	ULOGD("onOutputMediaRemoved id=%d type=%s",
	      media->id,
	      Media::getMediaTypeStr(media->type));

	VideoMedia *videoMedia = dynamic_cast<VideoMedia *>(media);
	if (videoMedia != NULL) {
		if (mListener != NULL) {
			struct pdraw_media_info info;
			fillMediaInfo(videoMedia, &info);
			mMediaRemovedInfoArgs.push(info);
			pomp_loop_idle_add(mLoop, callOnMediaRemoved, this);
		}
	}
}


/* Must be called on the loop thread */
void Session::onReadyToPlay(Demuxer *demuxer, bool ready)
{
	if (demuxer != mDemuxer)
		return;

	mDemuxerReady = ready;

	updateReadyToPlay();
}


void Session::updateReadyToPlay()
{
	bool next = (mState == OPENED && mDemuxerReady);

	if (mReadyToPlay == next)
		return;

	mReadyToPlay = next;
	mReadyToPlayReadyArgs.push(mReadyToPlay);
	pomp_loop_idle_add(mLoop, callReadyToPlay, this);
}


int Session::selectDemuxerMedia(Demuxer *demuxer,
				const struct pdraw_demuxer_media *medias,
				size_t count)
{
	return mListener->selectDemuxerMedia(this, medias, count);
}


void Session::onEndOfRange(Demuxer *demuxer, uint64_t timestamp)
{
	if (demuxer != mDemuxer)
		return;

	mEndOfRangeTimestampArgs.push(timestamp);
	pomp_loop_idle_add(mLoop, callEndOfRange, this);
}

void Session::playResp(Demuxer *demuxer,
		       int status,
		       uint64_t timestamp,
		       float speed)
{
	if (demuxer != mDemuxer)
		return;

	mPlayRespStatusArgs.push(status);
	mPlayRespTimestampArgs.push(timestamp);
	mPlayRespSpeedArgs.push(speed);
	pomp_loop_idle_add(mLoop, callPlayResponse, this);
}

void Session::pauseResp(Demuxer *demuxer, int status, uint64_t timestamp)
{
	if (demuxer != mDemuxer)
		return;

	mPauseRespStatusArgs.push(status);
	mPauseRespTimestampArgs.push(timestamp);
	pomp_loop_idle_add(mLoop, callPauseResponse, this);
}

void Session::seekResp(Demuxer *demuxer,
		       int status,
		       uint64_t timestamp,
		       float speed)
{
	if (demuxer != mDemuxer)
		return;

	mSeekRespStatusArgs.push(status);
	mSeekRespTimestampArgs.push(timestamp);
	mSeekRespSpeedArgs.push(speed);
	pomp_loop_idle_add(mLoop, callSeekResponse, this);
}


void Session::onUnrecoverableError(Demuxer *demuxer)
{
	if (demuxer != mDemuxer)
		return;

	/* Report to the app only the 1st error.
	 * It is supposed to act accordingly and destroy the session. */
	if (mUrecoverableErrorOccured)
		return;

	mUrecoverableErrorOccured = true;
	pomp_loop_idle_add(mLoop, callOnUnrecoverableError, this);
}


const char *Session::stateStr(enum State val)
{
	switch (val) {
	case State::INVALID:
		return "INVALID";
	case State::CREATED:
		return "CREATED";
	case State::OPENING:
		return "OPENING";
	case State::OPENED:
		return "OPENED";
	case State::CLOSING:
		return "CLOSING";
	case State::CLOSED:
		return "CLOSED";
	default:
		return NULL;
	}
}

} /* namespace Pdraw */
