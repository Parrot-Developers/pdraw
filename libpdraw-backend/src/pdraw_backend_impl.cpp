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
#include <limits.h>
#include <pthread.h>

#define ULOG_TAG pdraw_backend
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_backend);

#include "pdraw_backend_impl.hpp"

namespace PdrawBackend {


#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


enum cmd_type {
	CMD_TYPE_OPEN_SINGLE,
	CMD_TYPE_OPEN_URL,
	CMD_TYPE_OPEN_URL_MUX,
	CMD_TYPE_CLOSE,
	CMD_TYPE_GET_SINGLE_STREAM_LOCAL_STREAM_PORT,
	CMD_TYPE_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT,
	CMD_TYPE_IS_READY_TO_PLAY,
	CMD_TYPE_IS_PAUSED,
	CMD_TYPE_PLAY,
	CMD_TYPE_PREVIOUS_FRAME,
	CMD_TYPE_NEXT_FRAME,
	CMD_TYPE_SEEK,
	CMD_TYPE_SEEK_TO,
	CMD_TYPE_GET_CURRENT_TIME,
	CMD_TYPE_GET_DURATION,
	CMD_TYPE_START_VIDEO_SINK,
	CMD_TYPE_STOP_VIDEO_SINK,
	CMD_TYPE_RESYNC_VIDEO_SINK,
	CMD_TYPE_GET_VIDEO_SINK_QUEUE,
	CMD_TYPE_VIDEO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_GET_SESSION_TYPE,
	CMD_TYPE_GET_SELF_FRIENDLY_NAME,
	CMD_TYPE_SET_SELF_FRIENDLY_NAME,
	CMD_TYPE_GET_SELF_SERIAL_NUMBER,
	CMD_TYPE_SET_SELF_SERIAL_NUMBER,
	CMD_TYPE_GET_SELF_SOFTWARE_VERSION,
	CMD_TYPE_SET_SELF_SOFTWARE_VERSION,
	CMD_TYPE_IS_SELF_PILOT,
	CMD_TYPE_SET_SELF_PILOT,
	CMD_TYPE_GET_PEER_SESSION_METADATA,
	CMD_TYPE_GET_PEER_DRONE_MODEL,
	CMD_TYPE_GET_PIPELINE_MODE_SETTING,
	CMD_TYPE_SET_PIPELINE_MODE_SETTING,
	CMD_TYPE_GET_DISPLAY_SCREEN_SETTINGS,
	CMD_TYPE_SET_DISPLAY_SCREEN_SETTINGS,
	CMD_TYPE_GET_HMD_MODEL_SETTING,
	CMD_TYPE_SET_HMD_MODEL_SETTING,
	CMD_TYPE_SET_ANDROID_JVM,
};


struct cmd_msg {
	enum cmd_type type;
	union {
		struct {
			union {
				bool boolean;
				unsigned int uint;
				uint64_t uint64;
				int64_t int64;
				float flt;
				void *ptr;
				char string[40];
			};
		} generic;
		struct {
			char local_addr[16];
			uint16_t local_stream_port;
			uint16_t local_control_port;
			char remote_addr[16];
			uint16_t remote_stream_port;
			uint16_t remote_control_port;
		} open_single;
		struct {
			char url[200];
		} open_url;
		struct {
			char url[200];
			struct mux_ctx *mux;
		} open_url_mux;
		struct {
			unsigned int media_id;
			struct pdraw_video_sink_params params;
			IPdrawBackend::VideoSinkListener *listener;
		} start_video_sink;
		struct {
			float xdpi;
			float ydpi;
			float device_margin_top;
			float device_margin_bottom;
			float device_margin_left;
			float device_margin_right;
		} set_display_screen_settings;
	};
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_msg) <= PIPE_BUF - 1);


/**
 * Public functions
 */

int createPdrawBackend(IPdrawBackend::Listener *listener,
		       IPdrawBackend **retObj)
{
	IPdrawBackend *self = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == NULL, EINVAL);

	self = new PdrawBackend(listener);
	if (self == NULL) {
		ULOGE("failed to create pdraw backend instance");
		return -ENOMEM;
	}

	*retObj = self;
	return 0;
}


PdrawBackend::PdrawBackend(IPdrawBackend::Listener *listener)
{
	mApiMutexCreated = false;
	mApiReady = false;
	mMutexCreated = false;
	mCondCreated = false;
	mLoopThreadLaunched = false;
	mThreadShouldStop = false;
	mLoop = NULL;
	mMbox = NULL;
	mStarted = false;
	mRetValReady = false;
	mRetStatus = 0;
	mRetBool = false;
	mRetUint16 = 0;
	mRetUint64 = 0;
	memset(mRetFloat, 0, sizeof(mRetFloat));
	mRetSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
	mRetDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;
	mRetPipelineMode = PDRAW_PIPELINE_MODE_DECODE_ALL;
	mRetHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
	mRetQueue = NULL;
	memset(&mRetMediaInfo, 0, sizeof(mRetMediaInfo));
	mRetVideoSink = NULL;
	memset(&mRetSessionMeta, 0, sizeof(mRetSessionMeta));
	mPdraw = NULL;
	mListener = listener;
	mMapsMutexCreated = false;
	mPendingVideoRendererListener = NULL;
	mPendingVideoSinkListener = NULL;
	memset(&mLoopThread, 0, sizeof(pthread_t));
}


PdrawBackend::~PdrawBackend(void)
{
	int err;

	if (mStarted)
		ULOGW("destroying pdraw backend while still running");

	mThreadShouldStop = true;
	if (mLoop != NULL) {
		err = pomp_loop_wakeup(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -err);
	}
	if (mLoopThreadLaunched) {
		err = pthread_join(mLoopThread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
		mLoopThreadLaunched = false;
	}
	if (mPdraw != NULL) {
		delete mPdraw;
		mPdraw = NULL;
	}
	if (mLoop != NULL) {
		err = pomp_loop_destroy(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_destroy", -err);
		mLoop = NULL;
	}
	mStarted = false;
	if (mMbox != NULL) {
		mbox_destroy(mMbox);
		mMbox = NULL;
	}
	if (mCondCreated) {
		err = pthread_cond_destroy(&mCond);
		if (err != 0)
			ULOG_ERRNO("pthread_cond_destroy", err);
		mCondCreated = false;
	}
	if (mMutexCreated) {
		err = pthread_mutex_destroy(&mMutex);
		if (err != 0)
			ULOG_ERRNO("pthread_mutex_destroy", err);
		mMutexCreated = false;
	}
	if (mApiMutexCreated) {
		err = pthread_mutex_destroy(&mApiMutex);
		if (err != 0)
			ULOG_ERRNO("pthread_mutex_destroy", err);
		mApiMutexCreated = false;
	}
	if (mMapsMutexCreated) {
		err = pthread_mutex_destroy(&mMapsMutex);
		if (err != 0)
			ULOG_ERRNO("pthread_mutex_destroy", err);
		mMapsMutexCreated = false;
	}
}


int PdrawBackend::start(void)
{
	int res;
	pthread_mutexattr_t attr;

	ULOG_ERRNO_RETURN_ERR_IF(mListener == NULL, EPROTO);

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		return -res;
	}

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		pthread_mutexattr_destroy(&attr);
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		return -res;
	}

	res = pthread_mutex_init(&mApiMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		return -res;
	}
	mApiMutexCreated = true;
	pthread_mutexattr_destroy(&attr);

	res = pthread_mutex_init(&mMutex, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		return -res;
	}
	mMutexCreated = true;

	res = pthread_cond_init(&mCond, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_cond_init", res);
		return -res;
	}
	mCondCreated = true;

	res = pthread_mutex_init(&mMapsMutex, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		return -res;
	}
	mMapsMutexCreated = true;

	mMbox = mbox_new(sizeof(cmd_msg));
	if (mMbox == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("mbox_new", -res);
		return res;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	res = pthread_create(&mLoopThread, NULL, &loopThread, (void *)this);
	if (res != 0) {
		pthread_mutex_unlock(&mMutex);
		ULOG_ERRNO("pthread_create", res);
		return -res;
	}
	mLoopThreadLaunched = true;

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;
	mStarted = true;
	mApiReady = true;
	pthread_mutex_unlock(&mMutex);

	return res;
}


int PdrawBackend::stop(void)
{
	int err;

	if (mStarted) {
		pthread_mutex_lock(&mApiMutex);

		mThreadShouldStop = true;
		if (mLoop) {
			err = pomp_loop_wakeup(mLoop);
			if (err < 0)
				ULOG_ERRNO("pomp_loop_wakeup", -err);
		}

		if (mLoopThreadLaunched) {
			err = pthread_join(mLoopThread, NULL);
			if (err != 0)
				ULOG_ERRNO("pthread_join", err);
			mLoopThreadLaunched = false;
		}
		mStarted = false;

		pthread_mutex_unlock(&mApiMutex);
	}

	return 0;
}


struct pomp_loop *PdrawBackend::getLoop(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, NULL);

	return mLoop;
}


int PdrawBackend::open(const std::string &url)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() > sizeof(cmd->open_url.url) - 1,
				 ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_OPEN_URL;
	strncpy(cmd->open_url.url, url.c_str(), sizeof(cmd->open_url.url));
	cmd->open_url.url[sizeof(cmd->open_url.url) - 1] = '\0';
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::open(const std::string &localAddr,
		       uint16_t localStreamPort,
		       uint16_t localControlPort,
		       const std::string &remoteAddr,
		       uint16_t remoteStreamPort,
		       uint16_t remoteControlPort)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(
		localAddr.length() > sizeof(cmd->open_single.local_addr) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		remoteAddr.length() > sizeof(cmd->open_single.remote_addr) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_OPEN_SINGLE;
	strncpy(cmd->open_single.local_addr,
		localAddr.c_str(),
		sizeof(cmd->open_single.local_addr));
	cmd->open_single.local_addr[sizeof(cmd->open_single.local_addr) - 1] =
		'\0';
	cmd->open_single.local_stream_port = localStreamPort;
	cmd->open_single.local_control_port = localControlPort;
	strncpy(cmd->open_single.remote_addr,
		remoteAddr.c_str(),
		sizeof(cmd->open_single.remote_addr));
	cmd->open_single.remote_addr[sizeof(cmd->open_single.remote_addr) - 1] =
		'\0';
	cmd->open_single.remote_stream_port = remoteStreamPort;
	cmd->open_single.remote_control_port = remoteControlPort;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::open(const std::string &url, struct mux_ctx *mux)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(
		url.length() > sizeof(cmd->open_url_mux.url) - 1, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(mux == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_OPEN_URL_MUX;
	strncpy(cmd->open_url_mux.url,
		url.c_str(),
		sizeof(cmd->open_url_mux.url));
	cmd->open_url_mux.url[sizeof(cmd->open_url_mux.url) - 1] = '\0';
	cmd->open_url_mux.mux = mux;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::close(void)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CLOSE;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


uint16_t PdrawBackend::getSingleStreamLocalStreamPort(void)
{
	int res;
	uint16_t ret = 0;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, 0);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SINGLE_STREAM_LOCAL_STREAM_PORT;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetUint16;
	mRetUint16 = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


uint16_t PdrawBackend::getSingleStreamLocalControlPort(void)
{
	int res;
	uint16_t ret = 0;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, 0);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetUint16;
	mRetUint16 = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


bool PdrawBackend::isReadyToPlay(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, false);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, false);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_IS_READY_TO_PLAY;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetBool;
	mRetBool = false;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


bool PdrawBackend::isPaused(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, false);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, false);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_IS_PAUSED;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetBool;
	mRetBool = false;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


int PdrawBackend::play(float speed)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_PLAY;
	cmd->generic.flt = speed;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::pause(void)
{
	return play(0.);
}


int PdrawBackend::previousFrame(void)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_PREVIOUS_FRAME;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::nextFrame(void)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_NEXT_FRAME;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::seek(int64_t delta, bool exact)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SEEK;
	cmd->generic.int64 = delta;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta);
}


int PdrawBackend::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta));
}


int PdrawBackend::seekTo(uint64_t timestamp, bool exact)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SEEK_TO;
	cmd->generic.uint64 = timestamp;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


uint64_t PdrawBackend::getDuration(void)
{
	int res;
	uint64_t ret = 0;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, 0);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_DURATION;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetUint64;
	mRetUint64 = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


uint64_t PdrawBackend::getCurrentTime(void)
{
	int res;
	uint64_t ret = 0;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, 0);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_CURRENT_TIME;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetUint64;
	mRetUint64 = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


/* Called on the rendering thread */
int PdrawBackend::startVideoRenderer(
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdrawBackend::VideoRendererListener *listener,
	struct pdraw_video_renderer **retObj,
	struct egl_display *eglDisplay)
{
	int res;
	struct pdraw_video_renderer *renderer = NULL;
	std::pair<std::map<struct pdraw_video_renderer *,
			   IPdrawBackend::VideoRendererListener *>::iterator,
		  bool>
		inserted;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	mPendingVideoRendererListener = listener;

	res = mPdraw->startVideoRenderer(
		renderPos, params, this, &renderer, eglDisplay);
	if (res < 0)
		goto out;

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVideoRendererListenersMap.insert(
		std::pair<struct pdraw_video_renderer *,
			  IPdrawBackend::VideoRendererListener *>(renderer,
								  listener));
	if (inserted.second == false) {
		ULOGW("failed to insert the video renderer listener "
		      "in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

out:
	mPendingVideoRendererListener = NULL;
	*retObj = renderer;

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::stopVideoRenderer(struct pdraw_video_renderer *renderer)
{
	int res;
	size_t erased;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	pthread_mutex_lock(&mApiMutex);

	res = mPdraw->stopVideoRenderer(renderer);
	if (res < 0)
		goto out;

	pthread_mutex_lock(&mMapsMutex);
	erased = mVideoRendererListenersMap.erase(renderer);
	if (erased != 1) {
		ULOGW("failed to erase the video renderer listener "
		      "from the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

out:
	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::resizeVideoRenderer(struct pdraw_video_renderer *renderer,
				      const struct pdraw_rect *renderPos)
{
	int res;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	pthread_mutex_lock(&mApiMutex);

	res = mPdraw->resizeVideoRenderer(renderer, renderPos);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::setVideoRendererParams(
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	pthread_mutex_lock(&mApiMutex);

	res = mPdraw->setVideoRendererParams(renderer, params);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::getVideoRendererParams(
	struct pdraw_video_renderer *renderer,
	struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	pthread_mutex_lock(&mApiMutex);

	res = mPdraw->getVideoRendererParams(renderer, params);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::renderVideo(struct pdraw_video_renderer *renderer,
			      struct pdraw_rect *contentPos,
			      const float *viewMat,
			      const float *projMat)
{
	int res;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, 0);

	pthread_mutex_lock(&mApiMutex);

	res = mPdraw->renderVideo(renderer, contentPos, viewMat, projMat);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::startVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdrawBackend::VideoSinkListener *listener,
				 struct pdraw_video_sink **retObj)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_START_VIDEO_SINK;
	cmd->start_video_sink.media_id = mediaId;
	cmd->start_video_sink.params = *params;
	cmd->start_video_sink.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	*retObj = mRetVideoSink;
	mRetStatus = 0;
	mRetVideoSink = NULL;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::stopVideoSink(struct pdraw_video_sink *sink)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_STOP_VIDEO_SINK;
	cmd->generic.ptr = sink;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


int PdrawBackend::resyncVideoSink(struct pdraw_video_sink *sink)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RESYNC_VIDEO_SINK;
	cmd->generic.ptr = sink;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


struct vbuf_queue *
PdrawBackend::getVideoSinkQueue(struct pdraw_video_sink *sink)
{
	int res;
	struct vbuf_queue *ret = NULL;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, NULL);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, NULL);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_VIDEO_SINK_QUEUE;
	cmd->generic.ptr = sink;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetQueue;
	mRetQueue = NULL;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


int PdrawBackend::videoSinkQueueFlushed(struct pdraw_video_sink *sink)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_ERR_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = sink;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	res = mRetStatus;
	mRetStatus = 0;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


enum pdraw_session_type PdrawBackend::getSessionType(void)
{
	int res;
	enum pdraw_session_type ret = PDRAW_SESSION_TYPE_UNKNOWN;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, PDRAW_SESSION_TYPE_UNKNOWN);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(
		cmd == NULL, ENOMEM, PDRAW_SESSION_TYPE_UNKNOWN);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SESSION_TYPE;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetSessionType;
	mRetSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


void PdrawBackend::getSelfFriendlyName(std::string *friendlyName)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SELF_FRIENDLY_NAME;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	if (friendlyName)
		*friendlyName = mRetString;
	mRetString = "";
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setSelfFriendlyName(const std::string &friendlyName)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(friendlyName.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SELF_FRIENDLY_NAME;
	strncpy(cmd->generic.string,
		friendlyName.c_str(),
		sizeof(cmd->generic.string));
	cmd->generic.string[sizeof(cmd->generic.string) - 1] = '\0';
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getSelfSerialNumber(std::string *serialNumber)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SELF_SERIAL_NUMBER;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	if (serialNumber)
		*serialNumber = mRetString;
	mRetString = "";
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setSelfSerialNumber(const std::string &serialNumber)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(serialNumber.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SELF_SERIAL_NUMBER;
	strncpy(cmd->generic.string,
		serialNumber.c_str(),
		sizeof(cmd->generic.string));
	cmd->generic.string[sizeof(cmd->generic.string) - 1] = '\0';
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getSelfSoftwareVersion(std::string *softwareVersion)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SELF_SOFTWARE_VERSION;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	if (softwareVersion)
		*softwareVersion = mRetString;
	mRetString = "";
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setSelfSoftwareVersion(const std::string &softwareVersion)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(softwareVersion.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SELF_SOFTWARE_VERSION;
	strncpy(cmd->generic.string,
		softwareVersion.c_str(),
		sizeof(cmd->generic.string));
	cmd->generic.string[sizeof(cmd->generic.string) - 1] = '\0';
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


bool PdrawBackend::isSelfPilot(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, false);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, false);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_IS_SELF_PILOT;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetBool;
	mRetBool = false;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


void PdrawBackend::setSelfPilot(bool isPilot)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SELF_PILOT;
	cmd->generic.boolean = isPilot;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getPeerSessionMetadata(struct vmeta_session *session)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_PEER_DRONE_MODEL;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	if (session)
		*session = mRetSessionMeta;
	memset(&mRetSessionMeta, 0, sizeof(mRetSessionMeta));
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


enum pdraw_drone_model PdrawBackend::getPeerDroneModel(void)
{
	int res;
	enum pdraw_drone_model ret = PDRAW_DRONE_MODEL_UNKNOWN;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, PDRAW_DRONE_MODEL_UNKNOWN);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(
		cmd == NULL, ENOMEM, PDRAW_DRONE_MODEL_UNKNOWN);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_PEER_DRONE_MODEL;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetDroneModel;
	mRetDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


enum pdraw_pipeline_mode PdrawBackend::getPipelineModeSetting(void)
{
	int res;
	enum pdraw_pipeline_mode ret = PDRAW_PIPELINE_MODE_DECODE_ALL;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(
		!mStarted, EPROTO, PDRAW_PIPELINE_MODE_DECODE_ALL);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(
		cmd == NULL, ENOMEM, PDRAW_PIPELINE_MODE_DECODE_ALL);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_PIPELINE_MODE_SETTING;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetPipelineMode;
	mRetPipelineMode = PDRAW_PIPELINE_MODE_DECODE_ALL;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


void PdrawBackend::setPipelineModeSetting(enum pdraw_pipeline_mode mode)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_PIPELINE_MODE_SETTING;
	cmd->generic.uint = mode;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getDisplayScreenSettings(float *xdpi,
					    float *ydpi,
					    float *deviceMarginTop,
					    float *deviceMarginBottom,
					    float *deviceMarginLeft,
					    float *deviceMarginRight)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_DISPLAY_SCREEN_SETTINGS;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	if (xdpi)
		*xdpi = mRetFloat[0];
	if (ydpi)
		*ydpi = mRetFloat[1];
	if (deviceMarginTop)
		*deviceMarginTop = mRetFloat[2];
	if (deviceMarginBottom)
		*deviceMarginBottom = mRetFloat[3];
	if (deviceMarginLeft)
		*deviceMarginLeft = mRetFloat[4];
	if (deviceMarginRight)
		*deviceMarginRight = mRetFloat[5];
	memset(mRetFloat, 0, sizeof(mRetFloat));
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setDisplayScreenSettings(float xdpi,
					    float ydpi,
					    float deviceMarginTop,
					    float deviceMarginBottom,
					    float deviceMarginLeft,
					    float deviceMarginRight)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_DISPLAY_SCREEN_SETTINGS;
	cmd->set_display_screen_settings.xdpi = xdpi;
	cmd->set_display_screen_settings.ydpi = ydpi;
	cmd->set_display_screen_settings.device_margin_top = deviceMarginTop;
	cmd->set_display_screen_settings.device_margin_bottom =
		deviceMarginBottom;
	cmd->set_display_screen_settings.device_margin_left = deviceMarginLeft;
	cmd->set_display_screen_settings.device_margin_right =
		deviceMarginRight;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


enum pdraw_hmd_model PdrawBackend::getHmdModelSetting(void)
{
	int res;
	enum pdraw_hmd_model ret = PDRAW_HMD_MODEL_UNKNOWN;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, ret);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_VAL_IF(cmd == NULL, ENOMEM, PDRAW_HMD_MODEL_UNKNOWN);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_HMD_MODEL_SETTING;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	ret = mRetHmdModel;
	mRetHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);

	return ret;
}


void PdrawBackend::setHmdModelSetting(enum pdraw_hmd_model hmdModel)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_HMD_MODEL_SETTING;
	cmd->generic.uint = hmdModel;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setAndroidJvm(void *jvm)
{
	int res;
	struct cmd_msg *cmd = NULL;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	ULOG_ERRNO_RETURN_IF(cmd == NULL, ENOMEM);

	pthread_mutex_lock(&mApiMutex);

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_ANDROID_JVM;
	cmd->generic.ptr = jvm;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);
	mRetValReady = false;

out:
	pthread_mutex_unlock(&mMutex);
	free(cmd);

	pthread_mutex_unlock(&mApiMutex);
}


/**
 * Private functions
 */

void PdrawBackend::openResponse(IPdraw *pdraw, int status)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->openResponse(this, status);
}


void PdrawBackend::closeResponse(IPdraw *pdraw, int status)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->closeResponse(this, status);
}


void PdrawBackend::onUnrecoverableError(IPdraw *pdraw)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onUnrecoverableError(this);
}


int PdrawBackend::selectDemuxerMedia(IPdraw *pdraw,
				     const struct pdraw_demuxer_media *medias,
				     size_t count)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	return mListener->selectDemuxerMedia(this, medias, count);
}


void PdrawBackend::onMediaAdded(IPdraw *pdraw,
				const struct pdraw_media_info *info)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onMediaAdded(this, info);
}


void PdrawBackend::onMediaRemoved(IPdraw *pdraw,
				  const struct pdraw_media_info *info)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onMediaRemoved(this, info);
}


void PdrawBackend::readyToPlay(IPdraw *pdraw, bool ready)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->readyToPlay(this, ready);
}


void PdrawBackend::onEndOfRange(IPdraw *pdraw, uint64_t timestamp)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onEndOfRange(this, timestamp);
}


void PdrawBackend::playResponse(IPdraw *pdraw,
				int status,
				uint64_t timestamp,
				float speed)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->playResponse(this, status, timestamp, speed);
}


void PdrawBackend::pauseResponse(IPdraw *pdraw, int status, uint64_t timestamp)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->pauseResponse(this, status, timestamp);
}


void PdrawBackend::seekResponse(IPdraw *pdraw,
				int status,
				uint64_t timestamp,
				float speed)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->seekResponse(this, status, timestamp, speed);
}


void PdrawBackend::onSocketCreated(IPdraw *pdraw, int fd)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onSocketCreated(this, fd);
}


void PdrawBackend::onVideoRenderReady(IPdraw *pdraw,
				      struct pdraw_video_renderer *renderer)
{
	IPdrawBackend::VideoRendererListener *listener = NULL;
	std::map<struct pdraw_video_renderer *,
		 IPdrawBackend::VideoRendererListener *>::iterator it;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		if (mPendingVideoRendererListener != NULL)
			listener = mPendingVideoRendererListener;
	} else {
		listener = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (listener == NULL) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return;
	}

	listener->onVideoRenderReady(this, renderer);
}


int PdrawBackend::loadVideoTexture(IPdraw *pdraw,
				   struct pdraw_video_renderer *renderer,
				   unsigned int textureWidth,
				   unsigned int textureHeight,
				   const struct pdraw_session_info *sessionInfo,
				   const struct vmeta_session *sessionMeta,
				   const struct pdraw_video_frame *frame,
				   const void *frameUserdata,
				   size_t frameUserdataLen)
{
	IPdrawBackend::VideoRendererListener *listener = NULL;
	std::map<struct pdraw_video_renderer *,
		 IPdrawBackend::VideoRendererListener *>::iterator it;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		if (mPendingVideoRendererListener != NULL)
			listener = mPendingVideoRendererListener;
	} else {
		listener = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (listener == NULL) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return -ENOENT;
	}

	return listener->loadVideoTexture(this,
					  renderer,
					  textureWidth,
					  textureHeight,
					  sessionInfo,
					  sessionMeta,
					  frame,
					  frameUserdata,
					  frameUserdataLen);
}


int PdrawBackend::renderVideoOverlay(
	IPdraw *pdraw,
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
	IPdrawBackend::VideoRendererListener *listener = NULL;
	std::map<struct pdraw_video_renderer *,
		 IPdrawBackend::VideoRendererListener *>::iterator it;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		if (mPendingVideoRendererListener != NULL)
			listener = mPendingVideoRendererListener;
	} else {
		listener = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (listener == NULL) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return -ENOENT;
	}

	return listener->renderVideoOverlay(this,
					    renderer,
					    renderPos,
					    contentPos,
					    viewMat,
					    projMat,
					    sessionInfo,
					    sessionMeta,
					    frameMeta,
					    frameExtra);
}


void PdrawBackend::onVideoSinkFlush(IPdraw *pdraw,
				    struct pdraw_video_sink *sink)
{
	IPdrawBackend::VideoSinkListener *listener = NULL;
	std::map<struct pdraw_video_sink *,
		 IPdrawBackend::VideoSinkListener *>::iterator it;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoSinkListenersMap.find(sink);
	if (it == mVideoSinkListenersMap.end()) {
		if (mPendingVideoSinkListener != NULL)
			listener = mPendingVideoSinkListener;
	} else {
		listener = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (listener == NULL) {
		ULOGE("%s: failed to find the video sink listener in the map",
		      __func__);
		return;
	}

	listener->onVideoSinkFlush(this, sink);
}


void *PdrawBackend::loopThread(void *ptr)
{
	PdrawBackend *self = (PdrawBackend *)ptr;
	int res = 0, err;

	pthread_mutex_lock(&self->mMutex);

	if (self->mMbox == NULL) {
		res = -EPROTO;
		ULOGE("invalid mailbox");
		goto error;
	}

	self->mLoop = pomp_loop_new();
	if (self->mLoop == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_loop_new", -res);
		goto error;
	}

	res = pomp_loop_add(self->mLoop,
			    mbox_get_read_fd(self->mMbox),
			    POMP_FD_EVENT_IN,
			    &mboxCb,
			    self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_add", -res);
		goto error;
	}

	res = createPdraw(self->mLoop, self, &self->mPdraw);
	if (res < 0) {
		ULOG_ERRNO("createPdraw", -res);
		goto error;
	}

	self->mRetStatus = 0;
	self->mRetValReady = true;
	pthread_mutex_unlock(&self->mMutex);
	pthread_cond_signal(&self->mCond);

	while (!self->mThreadShouldStop) {
		pomp_loop_wait_and_process(self->mLoop, -1);
	}

	goto out;

error:
	self->mRetStatus = res;
	self->mRetValReady = true;
	pthread_mutex_unlock(&self->mMutex);
	pthread_cond_signal(&self->mCond);
out:
	if (self->mPdraw != NULL) {
		delete self->mPdraw;
		self->mPdraw = NULL;
	}
	if (self->mLoop != NULL) {
		if (self->mMbox != NULL) {
			err = pomp_loop_remove(self->mLoop,
					       mbox_get_read_fd(self->mMbox));
			if (err < 0)
				ULOG_ERRNO("pomp_loop_remove", -err);
			if (res == 0)
				res = err;
		}

		err = pomp_loop_destroy(self->mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_destroy", -err);
		if (res == 0)
			res = err;
		self->mLoop = NULL;
	}

	return (void *)(intptr_t)res;
}


void PdrawBackend::mboxCb(int fd, uint32_t revents, void *userdata)
{
	PdrawBackend *self = (PdrawBackend *)userdata;
	int res;
	void *message;

	message = malloc(sizeof(cmd_msg));
	if (message == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		return;
	}

	do {
		struct cmd_msg *msg = (struct cmd_msg *)message;

		/* Read from the mailbox */
		res = mbox_peek(self->mMbox, msg);
		if (res < 0) {
			if (res != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -res);
			break;
		}

		switch (msg->type) {
		case CMD_TYPE_OPEN_SINGLE: {
			std::string l(msg->open_single.local_addr);
			std::string r(msg->open_single.remote_addr);
			self->internalOpen(
				l,
				msg->open_single.local_stream_port,
				msg->open_single.local_control_port,
				r,
				msg->open_single.remote_stream_port,
				msg->open_single.remote_control_port);
			break;
		}
		case CMD_TYPE_OPEN_URL: {
			std::string u(msg->open_url.url);
			self->internalOpen(u);
			break;
		}
		case CMD_TYPE_OPEN_URL_MUX: {
			std::string u(msg->open_url_mux.url);
			self->internalOpen(u, msg->open_url_mux.mux);
			break;
		}
		case CMD_TYPE_CLOSE: {
			self->internalClose();
			break;
		}
		case CMD_TYPE_GET_SINGLE_STREAM_LOCAL_STREAM_PORT: {
			self->internalGetSingleStreamLocalStreamPort();
			break;
		}
		case CMD_TYPE_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT: {
			self->internalGetSingleStreamLocalControlPort();
			break;
		}
		case CMD_TYPE_IS_READY_TO_PLAY: {
			self->internalIsReadyToPlay();
			break;
		}
		case CMD_TYPE_IS_PAUSED: {
			self->internalIsPaused();
			break;
		}
		case CMD_TYPE_PLAY: {
			self->internalPlay(msg->generic.flt);
			break;
		}
		case CMD_TYPE_PREVIOUS_FRAME: {
			self->internalPreviousFrame();
			break;
		}
		case CMD_TYPE_NEXT_FRAME: {
			self->internalNextFrame();
			break;
		}
		case CMD_TYPE_SEEK: {
			self->internalSeek(msg->generic.int64);
			break;
		}
		case CMD_TYPE_SEEK_TO: {
			self->internalSeekTo(msg->generic.uint64);
			break;
		}
		case CMD_TYPE_GET_DURATION: {
			self->internalGetDuration();
			break;
		}
		case CMD_TYPE_GET_CURRENT_TIME: {
			self->internalGetCurrentTime();
			break;
		}
		case CMD_TYPE_START_VIDEO_SINK: {
			self->internalStartVideoSink(
				msg->start_video_sink.media_id,
				&msg->start_video_sink.params,
				msg->start_video_sink.listener);
			break;
		}
		case CMD_TYPE_STOP_VIDEO_SINK: {
			self->internalStopVideoSink(
				(struct pdraw_video_sink *)msg->generic.ptr);
			break;
		}
		case CMD_TYPE_RESYNC_VIDEO_SINK: {
			self->internalResyncVideoSink(
				(struct pdraw_video_sink *)msg->generic.ptr);
			break;
		}
		case CMD_TYPE_GET_VIDEO_SINK_QUEUE: {
			self->internalGetVideoSinkQueue(
				(struct pdraw_video_sink *)msg->generic.ptr);
			break;
		}
		case CMD_TYPE_VIDEO_SINK_QUEUE_FLUSHED: {
			self->internalVideoSinkQueueFlushed(
				(struct pdraw_video_sink *)msg->generic.ptr);
			break;
		}
		case CMD_TYPE_GET_SESSION_TYPE: {
			self->internalGetSessionType();
			break;
		}
		case CMD_TYPE_GET_SELF_FRIENDLY_NAME: {
			self->internalGetSelfFriendlyName();
			break;
		}
		case CMD_TYPE_SET_SELF_FRIENDLY_NAME: {
			std::string str = msg->generic.string;
			self->internalSetSelfFriendlyName(str);
			break;
		}
		case CMD_TYPE_GET_SELF_SERIAL_NUMBER: {
			self->internalGetSelfSerialNumber();
			break;
		}
		case CMD_TYPE_SET_SELF_SERIAL_NUMBER: {
			std::string str = msg->generic.string;
			self->internalSetSelfSerialNumber(str);
			break;
		}
		case CMD_TYPE_GET_SELF_SOFTWARE_VERSION: {
			self->internalGetSelfSoftwareVersion();
			break;
		}
		case CMD_TYPE_SET_SELF_SOFTWARE_VERSION: {
			std::string str = msg->generic.string;
			self->internalSetSelfSoftwareVersion(str);
			break;
		}
		case CMD_TYPE_IS_SELF_PILOT: {
			self->internalIsSelfPilot();
			break;
		}
		case CMD_TYPE_SET_SELF_PILOT: {
			self->internalSetSelfPilot(msg->generic.boolean);
			break;
		}
		case CMD_TYPE_GET_PEER_SESSION_METADATA: {
			self->internalGetPeerSessionMetadata();
			break;
		}
		case CMD_TYPE_GET_PEER_DRONE_MODEL: {
			self->internalGetPeerDroneModel();
			break;
		}
		case CMD_TYPE_GET_PIPELINE_MODE_SETTING: {
			self->internalGetPipelineModeSetting();
			break;
		}
		case CMD_TYPE_SET_PIPELINE_MODE_SETTING: {
			self->internalSetPipelineModeSetting(
				(enum pdraw_pipeline_mode)msg->generic.uint);
			break;
		}
		case CMD_TYPE_GET_DISPLAY_SCREEN_SETTINGS: {
			self->internalGetDisplayScreenSettings();
			break;
		}
		case CMD_TYPE_SET_DISPLAY_SCREEN_SETTINGS: {
			self->internalSetDisplayScreenSettings(
				msg->set_display_screen_settings.xdpi,
				msg->set_display_screen_settings.ydpi,
				msg->set_display_screen_settings
					.device_margin_top,
				msg->set_display_screen_settings
					.device_margin_bottom,
				msg->set_display_screen_settings
					.device_margin_left,
				msg->set_display_screen_settings
					.device_margin_right);
			break;
		}
		case CMD_TYPE_GET_HMD_MODEL_SETTING: {
			self->internalGetHmdModelSetting();
			break;
		}
		case CMD_TYPE_SET_HMD_MODEL_SETTING: {
			self->internalSetHmdModelSetting(
				(enum pdraw_hmd_model)msg->generic.uint);
			break;
		}
		case CMD_TYPE_SET_ANDROID_JVM: {
			self->internalSetAndroidJvm(msg->generic.ptr);
			break;
		}
		default:
			ULOGE("unknown command: %d", msg->type);
			break;
		}
	} while (res == 0);

	free(message);
}


void PdrawBackend::internalOpen(const std::string &url)
{
	int res = mPdraw->open(url);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalOpen(const std::string &localAddr,
				uint16_t localStreamPort,
				uint16_t localControlPort,
				const std::string &remoteAddr,
				uint16_t remoteStreamPort,
				uint16_t remoteControlPort)
{
	int res = mPdraw->open(localAddr,
			       localStreamPort,
			       localControlPort,
			       remoteAddr,
			       remoteStreamPort,
			       remoteControlPort);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalOpen(const std::string &url, struct mux_ctx *mux)
{
	int res = mPdraw->open(url, mux);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalClose(void)
{
	int res = mPdraw->close();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSingleStreamLocalStreamPort(void)
{
	uint16_t res = mPdraw->getSingleStreamLocalStreamPort();

	pthread_mutex_lock(&mMutex);
	mRetUint16 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSingleStreamLocalControlPort(void)
{
	uint16_t res = mPdraw->getSingleStreamLocalControlPort();

	pthread_mutex_lock(&mMutex);
	mRetUint16 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalIsReadyToPlay(void)
{
	bool res = mPdraw->isReadyToPlay();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalIsPaused(void)
{
	bool res = mPdraw->isPaused();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalPlay(float speed)
{
	int res = mPdraw->play(speed);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalPreviousFrame(void)
{
	int res = mPdraw->previousFrame();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalNextFrame(void)
{
	int res = mPdraw->nextFrame();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSeek(int64_t delta, bool exact)
{
	int res = mPdraw->seek(delta, exact);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSeekTo(uint64_t timestamp, bool exact)
{
	int res = mPdraw->seekTo(timestamp, exact);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetDuration(void)
{
	uint64_t res = mPdraw->getDuration();

	pthread_mutex_lock(&mMutex);
	mRetUint64 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetCurrentTime(void)
{
	uint64_t res = mPdraw->getCurrentTime();

	pthread_mutex_lock(&mMutex);
	mRetUint64 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalStartVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdrawBackend::VideoSinkListener *listener)
{
	int res;
	struct pdraw_video_sink *sink = NULL;
	std::pair<std::map<struct pdraw_video_sink *,
			   IPdrawBackend::VideoSinkListener *>::iterator,
		  bool>
		inserted;

	mPendingVideoSinkListener = listener;

	res = mPdraw->startVideoSink(mediaId, params, this, &sink);
	if (res < 0)
		goto out;

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVideoSinkListenersMap.insert(
		std::pair<struct pdraw_video_sink *,
			  IPdrawBackend::VideoSinkListener *>(sink, listener));
	if (inserted.second == false)
		ULOGW("failed to insert the video sink listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

out:
	mPendingVideoSinkListener = NULL;
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetVideoSink = sink;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalStopVideoSink(struct pdraw_video_sink *sink)
{
	int res;
	size_t erased;

	res = mPdraw->stopVideoSink(sink);
	if (res < 0)
		goto out;

	pthread_mutex_lock(&mMapsMutex);
	erased = mVideoSinkListenersMap.erase(sink);
	if (erased != 1)
		ULOGW("failed to erase the video sink listener from the map");
	pthread_mutex_unlock(&mMapsMutex);

out:
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalResyncVideoSink(struct pdraw_video_sink *sink)
{
	int res = mPdraw->resyncVideoSink(sink);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetVideoSinkQueue(struct pdraw_video_sink *sink)
{
	struct vbuf_queue *res = mPdraw->getVideoSinkQueue(sink);

	pthread_mutex_lock(&mMutex);
	mRetQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoSinkQueueFlushed(struct pdraw_video_sink *sink)
{
	int res = mPdraw->videoSinkQueueFlushed(sink);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSessionType(void)
{
	enum pdraw_session_type res = mPdraw->getSessionType();

	pthread_mutex_lock(&mMutex);
	mRetSessionType = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSelfFriendlyName(void)
{
	std::string friendlyName;
	mPdraw->getSelfFriendlyName(&friendlyName);

	pthread_mutex_lock(&mMutex);
	mRetString = friendlyName;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSelfFriendlyName(const std::string &friendlyName)
{
	mPdraw->setSelfFriendlyName(friendlyName);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSelfSerialNumber(void)
{
	std::string serialNumber;
	mPdraw->getSelfSerialNumber(&serialNumber);

	pthread_mutex_lock(&mMutex);
	mRetString = serialNumber;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSelfSerialNumber(const std::string &serialNumber)
{
	mPdraw->setSelfSerialNumber(serialNumber);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSelfSoftwareVersion(void)
{
	std::string softwareVersion;
	mPdraw->getSelfSoftwareVersion(&softwareVersion);

	pthread_mutex_lock(&mMutex);
	mRetString = softwareVersion;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSelfSoftwareVersion(
	const std::string &softwareVersion)
{
	mPdraw->setSelfSoftwareVersion(softwareVersion);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalIsSelfPilot(void)
{
	bool res = mPdraw->isSelfPilot();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSelfPilot(bool isPilot)
{
	mPdraw->setSelfPilot(isPilot);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetPeerSessionMetadata(void)
{
	struct vmeta_session meta;
	memset(&meta, 0, sizeof(meta));
	mPdraw->getPeerSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mRetSessionMeta = meta;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetPeerDroneModel(void)
{
	enum pdraw_drone_model res = mPdraw->getPeerDroneModel();

	pthread_mutex_lock(&mMutex);
	mRetDroneModel = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetPipelineModeSetting(void)
{
	enum pdraw_pipeline_mode res = mPdraw->getPipelineModeSetting();

	pthread_mutex_lock(&mMutex);
	mRetPipelineMode = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetPipelineModeSetting(enum pdraw_pipeline_mode mode)
{
	mPdraw->setPipelineModeSetting(mode);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetDisplayScreenSettings(void)
{
	float xdpi = 0.f, ydpi = 0.f;
	float deviceMarginTop = 0.f, deviceMarginBottom = 0.f;
	float deviceMarginLeft = 0.f, deviceMarginRight = 0.f;
	mPdraw->getDisplayScreenSettings(&xdpi,
					 &ydpi,
					 &deviceMarginTop,
					 &deviceMarginBottom,
					 &deviceMarginLeft,
					 &deviceMarginRight);

	pthread_mutex_lock(&mMutex);
	mRetFloat[0] = xdpi;
	mRetFloat[1] = ydpi;
	mRetFloat[2] = deviceMarginTop;
	mRetFloat[3] = deviceMarginBottom;
	mRetFloat[4] = deviceMarginLeft;
	mRetFloat[5] = deviceMarginRight;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetDisplayScreenSettings(float xdpi,
						    float ydpi,
						    float deviceMarginTop,
						    float deviceMarginBottom,
						    float deviceMarginLeft,
						    float deviceMarginRight)
{
	mPdraw->setDisplayScreenSettings(xdpi,
					 ydpi,
					 deviceMarginTop,
					 deviceMarginBottom,
					 deviceMarginLeft,
					 deviceMarginRight);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetHmdModelSetting(void)
{
	enum pdraw_hmd_model res = mPdraw->getHmdModelSetting();

	pthread_mutex_lock(&mMutex);
	mRetHmdModel = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetHmdModelSetting(enum pdraw_hmd_model hmdModel)
{
	mPdraw->setHmdModelSetting(hmdModel);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetAndroidJvm(void *jvm)
{
	mPdraw->setAndroidJvm(jvm);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}

} /* namespace PdrawBackend */
