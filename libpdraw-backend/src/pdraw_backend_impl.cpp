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

#ifdef _WIN32
#	define PIPE_BUF 4096
#endif /* _WIN32 */

namespace PdrawBackend {


#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


enum cmd_type {
	CMD_TYPE_STOP,
	CMD_TYPE_DEMUXER_CREATE_SINGLE,
	CMD_TYPE_DEMUXER_CREATE_URL,
	CMD_TYPE_DEMUXER_CREATE_URL_MUX,
	CMD_TYPE_DEMUXER_DESTROY,
	CMD_TYPE_DEMUXER_CLOSE,
	CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_STREAM_PORT,
	CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT,
	CMD_TYPE_DEMUXER_IS_READY_TO_PLAY,
	CMD_TYPE_DEMUXER_IS_PAUSED,
	CMD_TYPE_DEMUXER_PLAY,
	CMD_TYPE_DEMUXER_PREVIOUS_FRAME,
	CMD_TYPE_DEMUXER_NEXT_FRAME,
	CMD_TYPE_DEMUXER_SEEK,
	CMD_TYPE_DEMUXER_SEEK_TO,
	CMD_TYPE_DEMUXER_GET_CURRENT_TIME,
	CMD_TYPE_DEMUXER_GET_DURATION,
	CMD_TYPE_MUXER_CREATE,
	CMD_TYPE_MUXER_DESTROY,
	CMD_TYPE_MUXER_ADD_MEDIA,
	CMD_TYPE_CODED_VIDEO_SINK_CREATE,
	CMD_TYPE_CODED_VIDEO_SINK_DESTROY,
	CMD_TYPE_CODED_VIDEO_SINK_RESYNC,
	CMD_TYPE_CODED_VIDEO_SINK_GET_QUEUE,
	CMD_TYPE_CODED_VIDEO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_RAW_VIDEO_SINK_CREATE,
	CMD_TYPE_RAW_VIDEO_SINK_DESTROY,
	CMD_TYPE_RAW_VIDEO_SINK_GET_QUEUE,
	CMD_TYPE_RAW_VIDEO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_GET_FRIENDLY_NAME_SETTING,
	CMD_TYPE_SET_FRIENDLY_NAME_SETTING,
	CMD_TYPE_GET_SERIAL_NUMBER_SETTING,
	CMD_TYPE_SET_SERIAL_NUMBER_SETTING,
	CMD_TYPE_GET_SOFTWARE_VERSION_SETTING,
	CMD_TYPE_SET_SOFTWARE_VERSION_SETTING,
	CMD_TYPE_GET_PIPELINE_MODE_SETTING,
	CMD_TYPE_SET_PIPELINE_MODE_SETTING,
	CMD_TYPE_GET_DISPLAY_SCREEN_SETTINGS,
	CMD_TYPE_SET_DISPLAY_SCREEN_SETTINGS,
	CMD_TYPE_GET_HMD_MODEL_SETTING,
	CMD_TYPE_SET_HMD_MODEL_SETTING,
	CMD_TYPE_SET_ANDROID_JVM,
	CMD_TYPE_DUMP_PIPELINE,
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
			IPdraw::IDemuxer::Listener *listener;
		} demuxer_create_single;
		struct {
			char url[200];
			IPdraw::IDemuxer::Listener *listener;
		} demuxer_create_url;
		struct {
			char url[200];
			struct mux_ctx *mux;
			IPdraw::IDemuxer::Listener *listener;
		} demuxer_create_url_mux;
		struct {
			PdrawBackend::Demuxer *demuxer;
			float speed;
		} demuxer_play;
		struct {
			PdrawBackend::Demuxer *demuxer;
			int64_t delta;
			bool exact;
		} demuxer_seek;
		struct {
			PdrawBackend::Demuxer *demuxer;
			uint64_t timestamp;
			bool exact;
		} demuxer_seek_to;
		struct {
			unsigned int media_id;
			struct pdraw_video_sink_params params;
			IPdraw::ICodedVideoSink::Listener *listener;
		} coded_video_sink_create;
		struct {
			unsigned int media_id;
			struct pdraw_video_sink_params params;
			IPdraw::IRawVideoSink::Listener *listener;
		} raw_video_sink_create;
		struct {
			char url[200];
		} muxer_create;
		struct {
			IPdraw::IMuxer *muxer;
			unsigned int media_id;
			struct pdraw_muxer_video_media_params params;
		} muxer_add_media;
		struct {
			float xdpi;
			float ydpi;
			float device_margin_top;
			float device_margin_bottom;
			float device_margin_left;
			float device_margin_right;
		} set_display_screen_settings;
		struct {
			char file_name[200];
		} dump_pipeline;
	};
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_msg) <= PIPE_BUF - 1);


/**
 * Public functions
 */

int createPdrawBackend(IPdraw::Listener *listener, IPdrawBackend **retObj)
{
	IPdrawBackend *self = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);

	self = new PdrawBackend(listener);
	if (self == nullptr) {
		ULOGE("failed to create pdraw backend instance");
		return -ENOMEM;
	}

	*retObj = self;
	return 0;
}


PdrawBackend::PdrawBackend(IPdraw::Listener *listener)
{
	mApiMutexCreated = false;
	mApiReady = false;
	mMutexCreated = false;
	mCondCreated = false;
	mLoopThreadLaunched = false;
	mThreadShouldStop = false;
	mLoop = nullptr;
	mMbox = nullptr;
	mStarted = false;
	mRetValReady = false;
	mRetStatus = 0;
	mRetBool = false;
	mRetUint16 = 0;
	mRetUint64 = 0;
	memset(mRetFloat, 0, sizeof(mRetFloat));
	mRetPipelineMode = PDRAW_PIPELINE_MODE_DECODE_ALL;
	mRetHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
	mRetCodedQueue = nullptr;
	mRetRawQueue = nullptr;
	memset(&mRetMediaInfo, 0, sizeof(mRetMediaInfo));
	mRetCodedVideoSink = nullptr;
	mRetRawVideoSink = nullptr;
	mRetDemuxer = nullptr;
	mRetMuxer = nullptr;
	mPdraw = nullptr;
	mListener = listener;
	mMapsMutexCreated = false;
	mPendingDemuxerAndListener = {};
	mPendingVideoRendererAndListener = {};
	mPendingCodedVideoSinkAndListener = {};
	mPendingRawVideoSinkAndListener = {};
	memset(&mLoopThread, 0, sizeof(pthread_t));
}


PdrawBackend::~PdrawBackend(void)
{
	int err;

	if (mStarted)
		ULOGW("destroying pdraw backend while still running");

	mThreadShouldStop = true;
	if (mLoop != nullptr) {
		err = pomp_loop_wakeup(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -err);
	}
	if (mLoopThreadLaunched) {
		err = pthread_join(mLoopThread, nullptr);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
		mLoopThreadLaunched = false;
	}
	if (mPdraw != nullptr) {
		delete mPdraw;
		mPdraw = nullptr;
	}
	if (mLoop != nullptr) {
		err = pomp_loop_destroy(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_destroy", -err);
		mLoop = nullptr;
	}
	mStarted = false;
	if (mMbox != nullptr) {
		mbox_destroy(mMbox);
		mMbox = nullptr;
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

	ULOG_ERRNO_RETURN_ERR_IF(mListener == nullptr, EPROTO);

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

	res = pthread_mutex_init(&mMutex, nullptr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		return -res;
	}
	mMutexCreated = true;

	res = pthread_cond_init(&mCond, nullptr);
	if (res != 0) {
		ULOG_ERRNO("pthread_cond_init", res);
		return -res;
	}
	mCondCreated = true;

	res = pthread_mutex_init(&mMapsMutex, nullptr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		return -res;
	}
	mMapsMutexCreated = true;

	mMbox = mbox_new(sizeof(cmd_msg));
	if (mMbox == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("mbox_new", -res);
		return res;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	res = pthread_create(&mLoopThread, nullptr, &loopThread, (void *)this);
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
	int res;
	struct cmd_msg *cmd = nullptr;

	if (!mStarted)
		return 0;

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mPdraw->stop();
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_STOP;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	mRetStatus = false;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	mStarted = false;
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


struct pomp_loop *PdrawBackend::getLoop(void)
{
	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, nullptr);

	return mLoop;
}


int PdrawBackend::createDemuxer(const std::string &url,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		url.length() > sizeof(cmd->demuxer_create_url.url) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateDemuxer(url, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_URL;
	strncpy(cmd->demuxer_create_url.url,
		url.c_str(),
		sizeof(cmd->demuxer_create_url.url));
	cmd->demuxer_create_url.url[sizeof(cmd->demuxer_create_url.url) - 1] =
		'\0';
	cmd->demuxer_create_url.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetDemuxer;
	mRetStatus = 0;
	mRetDemuxer = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


int PdrawBackend::createDemuxer(const std::string &localAddr,
				uint16_t localStreamPort,
				uint16_t localControlPort,
				const std::string &remoteAddr,
				uint16_t remoteStreamPort,
				uint16_t remoteControlPort,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		localAddr.length() >
			sizeof(cmd->demuxer_create_single.local_addr) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		remoteAddr.length() >
			sizeof(cmd->demuxer_create_single.remote_addr) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateDemuxer(localAddr,
				      localStreamPort,
				      localControlPort,
				      remoteAddr,
				      remoteStreamPort,
				      remoteControlPort,
				      listener,
				      retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_SINGLE;
	strncpy(cmd->demuxer_create_single.local_addr,
		localAddr.c_str(),
		sizeof(cmd->demuxer_create_single.local_addr));
	cmd->demuxer_create_single
		.local_addr[sizeof(cmd->demuxer_create_single.local_addr) - 1] =
		'\0';
	cmd->demuxer_create_single.local_stream_port = localStreamPort;
	cmd->demuxer_create_single.local_control_port = localControlPort;
	strncpy(cmd->demuxer_create_single.remote_addr,
		remoteAddr.c_str(),
		sizeof(cmd->demuxer_create_single.remote_addr));
	cmd->demuxer_create_single
		.remote_addr[sizeof(cmd->demuxer_create_single.remote_addr) -
			     1] = '\0';
	cmd->demuxer_create_single.remote_stream_port = remoteStreamPort;
	cmd->demuxer_create_single.remote_control_port = remoteControlPort;
	cmd->demuxer_create_single.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetDemuxer;
	mRetStatus = 0;
	mRetDemuxer = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


int PdrawBackend::createDemuxer(const std::string &url,
				struct mux_ctx *mux,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		url.length() > sizeof(cmd->demuxer_create_url_mux.url) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateDemuxer(url, mux, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_URL_MUX;
	strncpy(cmd->demuxer_create_url_mux.url,
		url.c_str(),
		sizeof(cmd->demuxer_create_url_mux.url));
	cmd->demuxer_create_url_mux
		.url[sizeof(cmd->demuxer_create_url_mux.url) - 1] = '\0';
	cmd->demuxer_create_url_mux.mux = mux;
	cmd->demuxer_create_url_mux.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetDemuxer;
	mRetStatus = 0;
	mRetDemuxer = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::Demuxer::~Demuxer(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mDemuxer;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_DESTROY;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::Demuxer::close(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->close();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CLOSE;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


uint16_t PdrawBackend::Demuxer::getSingleStreamLocalStreamPort(void)
{
	int res;
	uint16_t ret = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, 0);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->getSingleStreamLocalStreamPort();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_STREAM_PORT;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetUint16;
	mBackend->mRetUint16 = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


uint16_t PdrawBackend::Demuxer::getSingleStreamLocalControlPort(void)
{
	int res;
	uint16_t ret = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, 0);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->getSingleStreamLocalControlPort();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetUint16;
	mBackend->mRetUint16 = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


bool PdrawBackend::Demuxer::isReadyToPlay(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->isReadyToPlay();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_IS_READY_TO_PLAY;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


bool PdrawBackend::Demuxer::isPaused(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->isPaused();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_IS_PAUSED;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::Demuxer::play(float speed)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->play(speed);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_PLAY;
	cmd->demuxer_play.demuxer = this;
	cmd->demuxer_play.speed = speed;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::pause(void)
{
	return play(0.);
}


int PdrawBackend::Demuxer::previousFrame(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->previousFrame();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_PREVIOUS_FRAME;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::nextFrame(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->nextFrame();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_NEXT_FRAME;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::seek(int64_t delta, bool exact)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->seek(delta, exact);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_SEEK;
	cmd->demuxer_seek.demuxer = this;
	cmd->demuxer_seek.delta = delta;
	cmd->demuxer_seek.exact = exact;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::seekForward(uint64_t delta, bool exact)
{
	return seek((int64_t)delta);
}


int PdrawBackend::Demuxer::seekBack(uint64_t delta, bool exact)
{
	return seek(-((int64_t)delta));
}


int PdrawBackend::Demuxer::seekTo(uint64_t timestamp, bool exact)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mDemuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mDemuxer->seekTo(timestamp, exact);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_SEEK_TO;
	cmd->demuxer_seek_to.demuxer = this;
	cmd->demuxer_seek_to.timestamp = timestamp;
	cmd->demuxer_seek_to.exact = exact;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


uint64_t PdrawBackend::Demuxer::getDuration(void)
{
	int res;
	uint64_t ret = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, 0);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->getDuration();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_DURATION;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetUint64;
	mBackend->mRetUint64 = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


uint64_t PdrawBackend::Demuxer::getCurrentTime(void)
{
	int res;
	uint64_t ret = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(mDemuxer == nullptr, EPROTO, 0);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mDemuxer->getCurrentTime();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_CURRENT_TIME;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetUint64;
	mBackend->mRetUint64 = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::createMuxer(const std::string &url, IPdraw::IMuxer **retObj)
{
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(
		url.length() > sizeof(cmd->muxer_create.url) - 1, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

#if MUXER_TEST
	int res;

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateMuxer(url, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_CREATE;
	strncpy(cmd->muxer_create.url,
		url.c_str(),
		sizeof(cmd->muxer_create.url));
	cmd->muxer_create.url[sizeof(cmd->muxer_create.url) - 1] = '\0';
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetMuxer;
	mRetStatus = 0;
	mRetMuxer = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;

#else
	/* Not supported yet */
	return -ENOSYS;
#endif
}


PdrawBackend::Muxer::Muxer(PdrawBackend *backend, const std::string &url)
{
	mBackend = backend;
	mMuxer = nullptr;
}


PdrawBackend::Muxer::~Muxer(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mMuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mMuxer;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_DESTROY;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::Muxer::addMedia(
	unsigned int mediaId,
	const struct pdraw_muxer_video_media_params *params)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->addMedia(mediaId, params);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_ADD_MEDIA;
	cmd->muxer_add_media.muxer = this;
	cmd->muxer_add_media.media_id = mediaId;
	if (params != nullptr)
		cmd->muxer_add_media.params = *params;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


/* Called on the rendering thread */
int PdrawBackend::createVideoRenderer(
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	IPdraw::IVideoRenderer **retObj,
	struct egl_display *eglDisplay)
{
	int res = 0;
	PdrawBackend::VideoRenderer *renderer = nullptr;
	IPdraw::IVideoRenderer *rnd = nullptr;
	std::pair<std::map<IPdraw::IVideoRenderer *,
			   struct videoRendererAndListener>::iterator,
		  bool>
		inserted;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	renderer = new PdrawBackend::VideoRenderer(
		this, renderPos, params, listener, eglDisplay);
	if (renderer == nullptr)
		goto out;

	mPendingVideoRendererAndListener = {
		.r = renderer,
		.l = listener,
	};

	res = mPdraw->createVideoRenderer(
		mediaId, renderPos, params, this, &rnd, eglDisplay);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createVideoRenderer", -res);
		delete renderer;
		renderer = nullptr;
		goto out;
	}
	renderer->setRenderer(rnd);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVideoRendererListenersMap.insert(
		std::pair<IPdraw::IVideoRenderer *,
			  struct videoRendererAndListener>(
			renderer->getRenderer(),
			mPendingVideoRendererAndListener));
	if (inserted.second == false) {
		ULOGW("failed to insert the video renderer listener "
		      "in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

out:
	mPendingVideoRendererAndListener = {};
	*retObj = renderer;

	pthread_mutex_unlock(&mApiMutex);

	return res;
}


/* Called on the rendering thread */
PdrawBackend::VideoRenderer::VideoRenderer(
	PdrawBackend *backend,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	struct egl_display *eglDisplay)
{
	mBackend = backend;
	mListener = listener;
	mRenderer = nullptr;
}


/* Called on the rendering thread */
PdrawBackend::VideoRenderer::~VideoRenderer(void)
{
	size_t erased;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);

	if (mRenderer == nullptr)
		return;

	pthread_mutex_lock(&mBackend->mApiMutex);

	pthread_mutex_lock(&mBackend->mMapsMutex);
	erased = mBackend->mVideoRendererListenersMap.erase(mRenderer);
	if (erased != 1) {
		ULOGW("failed to erase the video renderer listener "
		      "from the map");
	}
	pthread_mutex_unlock(&mBackend->mMapsMutex);

	delete mRenderer;
	mRenderer = nullptr;

	pthread_mutex_unlock(&mBackend->mApiMutex);
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::resize(const struct pdraw_rect *renderPos)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->resize(renderPos);

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::setMediaId(unsigned int mediaId)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->setMediaId(mediaId);

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


/* Called on the rendering thread */
unsigned int PdrawBackend::VideoRenderer::getMediaId(void)
{
	unsigned int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->getMediaId();

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::setParams(
	const struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->setParams(params);

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::getParams(
	struct pdraw_video_renderer_params *params)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->getParams(params);

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


/* Called on the rendering thread */
int PdrawBackend::VideoRenderer::render(struct pdraw_rect *contentPos,
					const float *viewMat,
					const float *projMat)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	res = mRenderer->render(contentPos, viewMat, projMat);

	pthread_mutex_unlock(&mBackend->mApiMutex);

	return res;
}


int PdrawBackend::createCodedVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateCodedVideoSink(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_CREATE;
	cmd->coded_video_sink_create.media_id = mediaId;
	cmd->coded_video_sink_create.params = *params;
	cmd->coded_video_sink_create.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetCodedVideoSink;
	mRetStatus = 0;
	mRetCodedVideoSink = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::CodedVideoSink::CodedVideoSink(
	PdrawBackend *backend,
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSink = nullptr;
}


PdrawBackend::CodedVideoSink::~CodedVideoSink(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSink == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSink;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_DESTROY;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::CodedVideoSink::resync(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSink == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSink->resync();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_RESYNC;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


struct mbuf_coded_video_frame_queue *
PdrawBackend::CodedVideoSink::getQueue(void)
{
	int res;
	struct mbuf_coded_video_frame_queue *ret = nullptr;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mSink == nullptr, EPROTO, nullptr);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSink->getQueue();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetCodedQueue;
	mBackend->mRetCodedQueue = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::CodedVideoSink::queueFlushed(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSink == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSink->queueFlushed();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createRawVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateRawVideoSink(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_CREATE;
	cmd->raw_video_sink_create.media_id = mediaId;
	cmd->raw_video_sink_create.params = *params;
	cmd->raw_video_sink_create.listener = listener;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	res = mRetStatus;
	*retObj = mRetRawVideoSink;
	mRetStatus = 0;
	mRetRawVideoSink = nullptr;
	mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::RawVideoSink::RawVideoSink(
	PdrawBackend *backend,
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSink = nullptr;
}


PdrawBackend::RawVideoSink::~RawVideoSink(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSink == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSink;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_DESTROY;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


struct mbuf_raw_video_frame_queue *PdrawBackend::RawVideoSink::getQueue(void)
{
	int res;
	struct mbuf_raw_video_frame_queue *ret = nullptr;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mSink == nullptr, EPROTO, nullptr);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSink->getQueue();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	ret = mBackend->mRetRawQueue;
	mBackend->mRetRawQueue = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::RawVideoSink::queueFlushed(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSink == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSink->queueFlushed();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);
	mBackend->mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = this;
	res = mbox_push(mBackend->mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		goto out;
	}

	while (!mBackend->mRetValReady)
		pthread_cond_wait(&mBackend->mCond, &mBackend->mMutex);

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;
	mBackend->mRetValReady = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


void PdrawBackend::getFriendlyNameSetting(std::string *friendlyName)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->getFriendlyNameSetting(friendlyName);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_FRIENDLY_NAME_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setFriendlyNameSetting(const std::string &friendlyName)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(friendlyName.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setFriendlyNameSetting(friendlyName);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_FRIENDLY_NAME_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getSerialNumberSetting(std::string *serialNumber)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->getSerialNumberSetting(serialNumber);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SERIAL_NUMBER_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setSerialNumberSetting(const std::string &serialNumber)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(serialNumber.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setSerialNumberSetting(serialNumber);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SERIAL_NUMBER_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::getSoftwareVersionSetting(std::string *softwareVersion)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->getSoftwareVersionSetting(softwareVersion);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SOFTWARE_VERSION_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setSoftwareVersionSetting(const std::string &softwareVersion)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(softwareVersion.length() >
				     sizeof(cmd->generic.string) - 1,
			     ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setSoftwareVersionSetting(softwareVersion);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SOFTWARE_VERSION_SETTING;
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


enum pdraw_pipeline_mode PdrawBackend::getPipelineModeSetting(void)
{
	int res;
	enum pdraw_pipeline_mode ret = PDRAW_PIPELINE_MODE_DECODE_ALL;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(
		!mStarted, EPROTO, PDRAW_PIPELINE_MODE_DECODE_ALL);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mPdraw->getPipelineModeSetting();
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return ret;
}


void PdrawBackend::setPipelineModeSetting(enum pdraw_pipeline_mode mode)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setPipelineModeSetting(mode);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
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
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->getDisplayScreenSettings(xdpi,
						 ydpi,
						 deviceMarginTop,
						 deviceMarginBottom,
						 deviceMarginLeft,
						 deviceMarginRight);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
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
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setDisplayScreenSettings(xdpi,
						 ydpi,
						 deviceMarginTop,
						 deviceMarginBottom,
						 deviceMarginLeft,
						 deviceMarginRight);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


enum pdraw_hmd_model PdrawBackend::getHmdModelSetting(void)
{
	int res;
	enum pdraw_hmd_model ret = PDRAW_HMD_MODEL_UNKNOWN;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(!mStarted, EPROTO, ret);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mPdraw->getHmdModelSetting();
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return ret;
}


void PdrawBackend::setHmdModelSetting(enum pdraw_hmd_model hmdModel)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setHmdModelSetting(hmdModel);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


void PdrawBackend::setAndroidJvm(void *jvm)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setAndroidJvm(jvm);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


int PdrawBackend::dumpPipeline(const std::string &fileName)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(
		fileName.length() > sizeof(cmd->dump_pipeline.file_name) - 1,
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mPdraw->dumpPipeline(fileName);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);
	mRetValReady = false;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DUMP_PIPELINE;
	strncpy(cmd->dump_pipeline.file_name,
		fileName.c_str(),
		sizeof(cmd->dump_pipeline.file_name));
	cmd->dump_pipeline.file_name[sizeof(cmd->dump_pipeline.file_name) - 1] =
		'\0';
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
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


/**
 * Private functions
 */

void PdrawBackend::stopResponse(IPdraw *pdraw, int status)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->stopResponse(this, status);
	mThreadShouldStop = true;
	if (mLoop) {
		int err = pomp_loop_wakeup(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -err);
	}
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


void PdrawBackend::onSocketCreated(IPdraw *pdraw, int fd)
{
	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);
	mListener->onSocketCreated(this, fd);
}


void PdrawBackend::demuxerOpenResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerOpenResponse(this, dl.d, status);
}


void PdrawBackend::demuxerCloseResponse(IPdraw *pdraw,
					IPdraw::IDemuxer *demuxer,
					int status)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerCloseResponse(this, dl.d, status);
}


void PdrawBackend::onDemuxerUnrecoverableError(IPdraw *pdraw,
					       IPdraw::IDemuxer *demuxer)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->onDemuxerUnrecoverableError(this, dl.d);
}


int PdrawBackend::demuxerSelectMedia(IPdraw *pdraw,
				     IPdraw::IDemuxer *demuxer,
				     const struct pdraw_demuxer_media *medias,
				     size_t count)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return -ENOENT;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return -ENOENT;
	}

	return dl.l->demuxerSelectMedia(this, dl.d, medias, count);
}


void PdrawBackend::demuxerReadyToPlay(IPdraw *pdraw,
				      IPdraw::IDemuxer *demuxer,
				      bool ready)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerReadyToPlay(this, dl.d, ready);
}


void PdrawBackend::onDemuxerEndOfRange(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       uint64_t timestamp)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->onDemuxerEndOfRange(this, dl.d, timestamp);
}


void PdrawBackend::demuxerPlayResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status,
				       uint64_t timestamp,
				       float speed)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerPlayResponse(this, dl.d, status, timestamp, speed);
}


void PdrawBackend::demuxerPauseResponse(IPdraw *pdraw,
					IPdraw::IDemuxer *demuxer,
					int status,
					uint64_t timestamp)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerPauseResponse(this, dl.d, status, timestamp);
}


void PdrawBackend::demuxerSeekResponse(IPdraw *pdraw,
				       IPdraw::IDemuxer *demuxer,
				       int status,
				       uint64_t timestamp,
				       float speed)
{
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it;
	struct demuxerAndListener dl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mDemuxerListenersMap.find(demuxer);
	if (it == mDemuxerListenersMap.end()) {
		dl = mPendingDemuxerAndListener;
	} else {
		dl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (dl.l == nullptr) {
		ULOGE("%s: failed to find the demuxer listener in the map",
		      __func__);
		return;
	}
	if (dl.d == nullptr) {
		ULOGE("%s: failed to find the demuxer in the map", __func__);
		return;
	}

	dl.l->demuxerSeekResponse(this, dl.d, status, timestamp, speed);
}


void PdrawBackend::onVideoRendererMediaAdded(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info)
{
	std::map<IPdraw::IVideoRenderer *,
		 struct videoRendererAndListener>::iterator it;
	struct videoRendererAndListener rl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		rl = mPendingVideoRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the video renderer in the map",
		      __func__);
		return;
	}

	rl.l->onVideoRendererMediaAdded(this, rl.r, info);
}


void PdrawBackend::onVideoRendererMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVideoRenderer *renderer,
	const struct pdraw_media_info *info)
{
	std::map<IPdraw::IVideoRenderer *,
		 struct videoRendererAndListener>::iterator it;
	struct videoRendererAndListener rl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		rl = mPendingVideoRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the video renderer in the map",
		      __func__);
		return;
	}

	rl.l->onVideoRendererMediaRemoved(this, rl.r, info);
}


void PdrawBackend::onVideoRenderReady(IPdraw *pdraw,
				      IPdraw::IVideoRenderer *renderer)
{
	std::map<IPdraw::IVideoRenderer *,
		 struct videoRendererAndListener>::iterator it;
	struct videoRendererAndListener rl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		rl = mPendingVideoRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the video renderer in the map",
		      __func__);
		return;
	}

	rl.l->onVideoRenderReady(this, rl.r);
}


int PdrawBackend::loadVideoTexture(IPdraw *pdraw,
				   IPdraw::IVideoRenderer *renderer,
				   unsigned int textureWidth,
				   unsigned int textureHeight,
				   const struct pdraw_media_info *mediaInfo,
				   struct mbuf_raw_video_frame *frame,
				   const void *frameUserdata,
				   size_t frameUserdataLen)
{
	std::map<IPdraw::IVideoRenderer *,
		 struct videoRendererAndListener>::iterator it;
	struct videoRendererAndListener rl;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		rl = mPendingVideoRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return -ENOENT;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the video renderer in the map",
		      __func__);
		return -ENOENT;
	}

	return rl.l->loadVideoTexture(this,
				      rl.r,
				      textureWidth,
				      textureHeight,
				      mediaInfo,
				      frame,
				      frameUserdata,
				      frameUserdataLen);
}


int PdrawBackend::renderVideoOverlay(
	IPdraw *pdraw,
	IPdraw::IVideoRenderer *renderer,
	const struct pdraw_rect *renderPos,
	const struct pdraw_rect *contentPos,
	const float *viewMat,
	const float *projMat,
	const struct pdraw_media_info *mediaInfo,
	struct vmeta_frame *frameMeta,
	const struct pdraw_video_frame_extra *frameExtra)
{
	std::map<IPdraw::IVideoRenderer *,
		 struct videoRendererAndListener>::iterator it;
	struct videoRendererAndListener rl;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoRendererListenersMap.find(renderer);
	if (it == mVideoRendererListenersMap.end()) {
		rl = mPendingVideoRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the video renderer "
		      "listener in the map",
		      __func__);
		return -ENOENT;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the video renderer in the map",
		      __func__);
		return -ENOENT;
	}

	return rl.l->renderVideoOverlay(this,
					rl.r,
					renderPos,
					contentPos,
					viewMat,
					projMat,
					mediaInfo,
					frameMeta,
					frameExtra);
}


void PdrawBackend::onCodedVideoSinkFlush(IPdraw *pdraw,
					 IPdraw::ICodedVideoSink *sink)
{
	std::map<IPdraw::ICodedVideoSink *,
		 struct codedVideoSinkAndListener>::iterator it;
	struct codedVideoSinkAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mCodedVideoSinkListenersMap.find(sink);
	if (it == mCodedVideoSinkListenersMap.end()) {
		sl = mPendingCodedVideoSinkAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video sink listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video sink in the map", __func__);
		return;
	}

	sl.l->onCodedVideoSinkFlush(this, sl.s);
}


void PdrawBackend::onRawVideoSinkFlush(IPdraw *pdraw,
				       IPdraw::IRawVideoSink *sink)
{
	std::map<IPdraw::IRawVideoSink *,
		 struct rawVideoSinkAndListener>::iterator it;
	struct rawVideoSinkAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mRawVideoSinkListenersMap.find(sink);
	if (it == mRawVideoSinkListenersMap.end()) {
		sl = mPendingRawVideoSinkAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video sink listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video sink in the map", __func__);
		return;
	}

	sl.l->onRawVideoSinkFlush(this, sl.s);
}


void *PdrawBackend::loopThread(void *ptr)
{
	PdrawBackend *self = (PdrawBackend *)ptr;
	int res = 0, err;

	pthread_mutex_lock(&self->mMutex);

	if (self->mMbox == nullptr) {
		res = -EPROTO;
		ULOGE("invalid mailbox");
		goto error;
	}

	self->mLoop = pomp_loop_new();
	if (self->mLoop == nullptr) {
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
	if (self->mPdraw != nullptr) {
		delete self->mPdraw;
		self->mPdraw = nullptr;
	}
	if (self->mLoop != nullptr) {
		if (self->mMbox != nullptr) {
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
		self->mLoop = nullptr;
	}

	return (void *)(intptr_t)res;
}


void PdrawBackend::mboxCb(int fd, uint32_t revents, void *userdata)
{
	PdrawBackend *self = (PdrawBackend *)userdata;
	int res;
	void *message;

	message = malloc(sizeof(cmd_msg));
	if (message == nullptr) {
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
		case CMD_TYPE_STOP: {
			self->internalStop();
			break;
		}
		case CMD_TYPE_DEMUXER_CREATE_SINGLE: {
			std::string l(msg->demuxer_create_single.local_addr);
			std::string r(msg->demuxer_create_single.remote_addr);
			self->internalDemuxerCreate(
				l,
				msg->demuxer_create_single.local_stream_port,
				msg->demuxer_create_single.local_control_port,
				r,
				msg->demuxer_create_single.remote_stream_port,
				msg->demuxer_create_single.remote_control_port,
				msg->demuxer_create_single.listener);
			break;
		}
		case CMD_TYPE_DEMUXER_CREATE_URL: {
			std::string u(msg->demuxer_create_url.url);
			self->internalDemuxerCreate(
				u, msg->demuxer_create_url.listener);
			break;
		}
		case CMD_TYPE_DEMUXER_CREATE_URL_MUX: {
			std::string u(msg->demuxer_create_url_mux.url);
			self->internalDemuxerCreate(
				u,
				msg->demuxer_create_url_mux.mux,
				msg->demuxer_create_url_mux.listener);
			break;
		}
		case CMD_TYPE_DEMUXER_DESTROY: {
			self->internalDemuxerDestroy(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_CLOSE: {
			self->internalDemuxerClose(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_STREAM_PORT: {
			self->internalDemuxerGetSingleStreamLocalStreamPort(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT: {
			self->internalDemuxerGetSingleStreamLocalControlPort(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_IS_READY_TO_PLAY: {
			self->internalDemuxerIsReadyToPlay(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_IS_PAUSED: {
			self->internalDemuxerIsPaused(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_PLAY: {
			self->internalDemuxerPlay(msg->demuxer_play.demuxer,
						  msg->demuxer_play.speed);
			break;
		}
		case CMD_TYPE_DEMUXER_PREVIOUS_FRAME: {
			self->internalDemuxerPreviousFrame(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_NEXT_FRAME: {
			self->internalDemuxerNextFrame(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_SEEK: {
			self->internalDemuxerSeek(msg->demuxer_seek.demuxer,
						  msg->demuxer_seek.delta,
						  msg->demuxer_seek.exact);
			break;
		}
		case CMD_TYPE_DEMUXER_SEEK_TO: {
			self->internalDemuxerSeekTo(
				msg->demuxer_seek_to.demuxer,
				msg->demuxer_seek_to.timestamp,
				msg->demuxer_seek_to.exact);
			break;
		}
		case CMD_TYPE_DEMUXER_GET_DURATION: {
			self->internalDemuxerGetDuration(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_GET_CURRENT_TIME: {
			self->internalDemuxerGetCurrentTime(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_CREATE: {
			std::string u(msg->muxer_create.url);
			self->internalMuxerCreate(u);
			break;
		}
		case CMD_TYPE_MUXER_DESTROY: {
			self->internalMuxerDestroy(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_ADD_MEDIA: {
			self->internalMuxerAddMedia(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->muxer_add_media.muxer),
				msg->muxer_add_media.media_id,
				&msg->muxer_add_media.params);
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SINK_CREATE: {
			self->internalCodedVideoSinkCreate(
				msg->coded_video_sink_create.media_id,
				&msg->coded_video_sink_create.params,
				msg->coded_video_sink_create.listener);
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SINK_DESTROY: {
			self->internalCodedVideoSinkDestroy(
				reinterpret_cast<PdrawBackend::CodedVideoSink
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SINK_RESYNC: {
			self->internalCodedVideoSinkResync(
				reinterpret_cast<PdrawBackend::CodedVideoSink
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SINK_GET_QUEUE: {
			self->internalCodedVideoSinkGetQueue(
				reinterpret_cast<PdrawBackend::CodedVideoSink
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SINK_QUEUE_FLUSHED: {
			self->internalCodedVideoSinkQueueFlushed(
				reinterpret_cast<PdrawBackend::CodedVideoSink
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SINK_CREATE: {
			self->internalRawVideoSinkCreate(
				msg->raw_video_sink_create.media_id,
				&msg->raw_video_sink_create.params,
				msg->raw_video_sink_create.listener);
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SINK_DESTROY: {
			self->internalRawVideoSinkDestroy(
				reinterpret_cast<PdrawBackend::RawVideoSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SINK_GET_QUEUE: {
			self->internalRawVideoSinkGetQueue(
				reinterpret_cast<PdrawBackend::RawVideoSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SINK_QUEUE_FLUSHED: {
			self->internalRawVideoSinkQueueFlushed(
				reinterpret_cast<PdrawBackend::RawVideoSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_GET_FRIENDLY_NAME_SETTING: {
			self->internalGetFriendlyNameSetting();
			break;
		}
		case CMD_TYPE_SET_FRIENDLY_NAME_SETTING: {
			std::string str = msg->generic.string;
			self->internalSetFriendlyNameSetting(str);
			break;
		}
		case CMD_TYPE_GET_SERIAL_NUMBER_SETTING: {
			self->internalGetSerialNumberSetting();
			break;
		}
		case CMD_TYPE_SET_SERIAL_NUMBER_SETTING: {
			std::string str = msg->generic.string;
			self->internalSetSerialNumberSetting(str);
			break;
		}
		case CMD_TYPE_GET_SOFTWARE_VERSION_SETTING: {
			self->internalGetSoftwareVersionSetting();
			break;
		}
		case CMD_TYPE_SET_SOFTWARE_VERSION_SETTING: {
			std::string str = msg->generic.string;
			self->internalSetSoftwareVersionSetting(str);
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
		case CMD_TYPE_DUMP_PIPELINE: {
			std::string f(msg->dump_pipeline.file_name);
			self->internalDumpPipeline(f);
			break;
		}
		default:
			ULOGE("unknown command: %d", msg->type);
			break;
		}
	} while (res == 0);

	free(message);
}


int PdrawBackend::doCreateDemuxer(const std::string &url,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	int res;
	PdrawBackend::Demuxer *demuxer = nullptr;
	IPdraw::IDemuxer *d = nullptr;
	std::pair<std::map<IPdraw::IDemuxer *,
			   struct demuxerAndListener>::iterator,
		  bool>
		inserted;

	demuxer = new PdrawBackend::Demuxer(this, listener);
	if (demuxer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::Demuxer", -res);
		return res;
	}

	mPendingDemuxerAndListener = {
		.d = demuxer,
		.l = listener,
	};

	res = mPdraw->createDemuxer(url, this, &d);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createDemuxer", -res);
		delete demuxer;
		demuxer = nullptr;
		return res;
	}
	demuxer->setDemuxer(d);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mDemuxerListenersMap.insert(
		std::pair<IPdraw::IDemuxer *, struct demuxerAndListener>(
			demuxer->getDemuxer(), mPendingDemuxerAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the demuxer listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateDemuxer(const std::string &localAddr,
				  uint16_t localStreamPort,
				  uint16_t localControlPort,
				  const std::string &remoteAddr,
				  uint16_t remoteStreamPort,
				  uint16_t remoteControlPort,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	int res;
	PdrawBackend::Demuxer *demuxer = nullptr;
	IPdraw::IDemuxer *d = nullptr;
	std::pair<std::map<IPdraw::IDemuxer *,
			   struct demuxerAndListener>::iterator,
		  bool>
		inserted;

	demuxer = new PdrawBackend::Demuxer(this, listener);
	if (demuxer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::Demuxer", -res);
		return res;
	}

	mPendingDemuxerAndListener = {
		.d = demuxer,
		.l = listener,
	};

	res = mPdraw->createDemuxer(localAddr,
				    localStreamPort,
				    localControlPort,
				    remoteAddr,
				    remoteStreamPort,
				    remoteControlPort,
				    this,
				    &d);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createDemuxer", -res);
		delete demuxer;
		demuxer = nullptr;
		return res;
	}
	demuxer->setDemuxer(d);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mDemuxerListenersMap.insert(
		std::pair<IPdraw::IDemuxer *, struct demuxerAndListener>(
			demuxer->getDemuxer(), mPendingDemuxerAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the demuxer listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateDemuxer(const std::string &url,
				  struct mux_ctx *mux,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj)
{
	int res;
	PdrawBackend::Demuxer *demuxer = nullptr;
	IPdraw::IDemuxer *d = nullptr;
	std::pair<std::map<IPdraw::IDemuxer *,
			   struct demuxerAndListener>::iterator,
		  bool>
		inserted;

	demuxer = new PdrawBackend::Demuxer(this, listener);
	if (demuxer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::Demuxer", -res);
		return res;
	}

	mPendingDemuxerAndListener = {
		.d = demuxer,
		.l = listener,
	};

	res = mPdraw->createDemuxer(url, mux, this, &d);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createDemuxer", -res);
		delete demuxer;
		demuxer = nullptr;
		return res;
	}
	demuxer->setDemuxer(d);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mDemuxerListenersMap.insert(
		std::pair<IPdraw::IDemuxer *, struct demuxerAndListener>(
			demuxer->getDemuxer(), mPendingDemuxerAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the demuxer listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateMuxer(const std::string &url, IPdraw::IMuxer **retObj)
{
	int res;
	PdrawBackend::Muxer *muxer = nullptr;
	IPdraw::IMuxer *m = nullptr;

	muxer = new PdrawBackend::Muxer(this, url);
	if (muxer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::Muxer", -res);
		return res;
	}

	res = mPdraw->createMuxer(url, &m);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createMuxer", -res);
		delete muxer;
		muxer = nullptr;
		return res;
	}
	muxer->setMuxer(m);

	*retObj = muxer;
	return 0;
}


int PdrawBackend::doCreateCodedVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	int res;
	PdrawBackend::CodedVideoSink *sink = nullptr;
	IPdraw::ICodedVideoSink *s = nullptr;
	std::pair<std::map<IPdraw::ICodedVideoSink *,
			   struct codedVideoSinkAndListener>::iterator,
		  bool>
		inserted;

	sink = new PdrawBackend::CodedVideoSink(
		this, mediaId, params, listener);
	if (sink == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::CodedVideoSink", -res);
		return res;
	}

	mPendingCodedVideoSinkAndListener = {
		.s = sink,
		.l = listener,
	};

	res = mPdraw->createCodedVideoSink(mediaId, params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createCodedVideoSink", -res);
		delete sink;
		sink = nullptr;
		return res;
	}
	sink->setCodedVideoSink(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mCodedVideoSinkListenersMap.insert(
		std::pair<IPdraw::ICodedVideoSink *,
			  struct codedVideoSinkAndListener>(
			sink->getCodedVideoSink(),
			mPendingCodedVideoSinkAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video sink listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	*retObj = sink;
	return 0;
}


int PdrawBackend::doCreateRawVideoSink(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	int res;
	PdrawBackend::RawVideoSink *sink = nullptr;
	IPdraw::IRawVideoSink *s = nullptr;
	std::pair<std::map<IPdraw::IRawVideoSink *,
			   struct rawVideoSinkAndListener>::iterator,
		  bool>
		inserted;

	sink = new PdrawBackend::RawVideoSink(this, mediaId, params, listener);
	if (sink == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::RawVideoSink", -res);
		return res;
	}

	mPendingRawVideoSinkAndListener = {
		.s = sink,
		.l = listener,
	};

	res = mPdraw->createRawVideoSink(mediaId, params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createRawVideoSink", -res);
		delete sink;
		sink = nullptr;
		return res;
	}
	sink->setRawVideoSink(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mRawVideoSinkListenersMap.insert(
		std::pair<IPdraw::IRawVideoSink *,
			  struct rawVideoSinkAndListener>(
			sink->getRawVideoSink(),
			mPendingRawVideoSinkAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video sink listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	*retObj = sink;
	return 0;
}


void PdrawBackend::internalStop()
{
	int res = mPdraw->stop();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerCreate(const std::string &url,
					 IPdraw::IDemuxer::Listener *listener)
{
	int res;
	IPdraw::IDemuxer *demuxer = nullptr;

	res = doCreateDemuxer(url, listener, &demuxer);

	mPendingDemuxerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetDemuxer = demuxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerCreate(const std::string &localAddr,
					 uint16_t localStreamPort,
					 uint16_t localControlPort,
					 const std::string &remoteAddr,
					 uint16_t remoteStreamPort,
					 uint16_t remoteControlPort,
					 IPdraw::IDemuxer::Listener *listener)
{
	int res;
	IPdraw::IDemuxer *demuxer = nullptr;

	res = doCreateDemuxer(localAddr,
			      localStreamPort,
			      localControlPort,
			      remoteAddr,
			      remoteStreamPort,
			      remoteControlPort,
			      listener,
			      &demuxer);

	mPendingDemuxerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetDemuxer = demuxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerCreate(const std::string &url,
					 struct mux_ctx *mux,
					 IPdraw::IDemuxer::Listener *listener)
{
	int res;
	IPdraw::IDemuxer *demuxer = nullptr;

	res = doCreateDemuxer(url, mux, listener, &demuxer);

	mPendingDemuxerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetDemuxer = demuxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerDestroy(PdrawBackend::Demuxer *demuxer)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	erased = mDemuxerListenersMap.erase(demuxer->getDemuxer());
	if (erased != 1)
		ULOGW("failed to erase the demuxer listener from the map");
	pthread_mutex_unlock(&mMapsMutex);

	delete demuxer->getDemuxer();

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerClose(PdrawBackend::Demuxer *demuxer)
{
	int res = demuxer->getDemuxer()->close();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerGetSingleStreamLocalStreamPort(
	PdrawBackend::Demuxer *demuxer)
{
	uint16_t res = demuxer->getDemuxer()->getSingleStreamLocalStreamPort();

	pthread_mutex_lock(&mMutex);
	mRetUint16 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerGetSingleStreamLocalControlPort(
	PdrawBackend::Demuxer *demuxer)
{
	uint16_t res = demuxer->getDemuxer()->getSingleStreamLocalControlPort();

	pthread_mutex_lock(&mMutex);
	mRetUint16 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerIsReadyToPlay(PdrawBackend::Demuxer *demuxer)
{
	bool res = demuxer->getDemuxer()->isReadyToPlay();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerIsPaused(PdrawBackend::Demuxer *demuxer)
{
	bool res = demuxer->getDemuxer()->isPaused();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerPlay(PdrawBackend::Demuxer *demuxer,
				       float speed)
{
	int res = demuxer->getDemuxer()->play(speed);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerPreviousFrame(PdrawBackend::Demuxer *demuxer)
{
	int res = demuxer->getDemuxer()->previousFrame();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerNextFrame(PdrawBackend::Demuxer *demuxer)
{
	int res = demuxer->getDemuxer()->nextFrame();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerSeek(PdrawBackend::Demuxer *demuxer,
				       int64_t delta,
				       bool exact)
{
	int res = demuxer->getDemuxer()->seek(delta, exact);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerSeekTo(PdrawBackend::Demuxer *demuxer,
					 uint64_t timestamp,
					 bool exact)
{
	int res = demuxer->getDemuxer()->seekTo(timestamp, exact);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerGetDuration(PdrawBackend::Demuxer *demuxer)
{
	uint64_t res = demuxer->getDemuxer()->getDuration();

	pthread_mutex_lock(&mMutex);
	mRetUint64 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerGetCurrentTime(PdrawBackend::Demuxer *demuxer)
{
	uint64_t res = demuxer->getDemuxer()->getCurrentTime();

	pthread_mutex_lock(&mMutex);
	mRetUint64 = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerCreate(const std::string &url)
{
	int res;
	IPdraw::IMuxer *muxer = nullptr;

	res = doCreateMuxer(url, &muxer);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetMuxer = muxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerDestroy(PdrawBackend::Muxer *muxer)
{
	delete muxer->getMuxer();

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerAddMedia(
	PdrawBackend::Muxer *muxer,
	unsigned int mediaId,
	const struct pdraw_muxer_video_media_params *params)
{
	int res = muxer->getMuxer()->addMedia(mediaId, params);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSinkCreate(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener)
{
	int res;
	IPdraw::ICodedVideoSink *sink = nullptr;

	res = doCreateCodedVideoSink(mediaId, params, listener, &sink);

	mPendingCodedVideoSinkAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetCodedVideoSink = sink;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSinkDestroy(
	PdrawBackend::CodedVideoSink *sink)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	erased = mCodedVideoSinkListenersMap.erase(sink->getCodedVideoSink());
	if (erased != 1)
		ULOGW("failed to erase the video sink listener from the map");
	pthread_mutex_unlock(&mMapsMutex);

	delete sink->getCodedVideoSink();

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSinkResync(
	PdrawBackend::CodedVideoSink *sink)
{
	int res = sink->getCodedVideoSink()->resync();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSinkGetQueue(
	PdrawBackend::CodedVideoSink *sink)
{
	struct mbuf_coded_video_frame_queue *res =
		sink->getCodedVideoSink()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetCodedQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSinkQueueFlushed(
	PdrawBackend::CodedVideoSink *sink)
{
	int res = sink->getCodedVideoSink()->queueFlushed();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSinkCreate(
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener)
{
	int res;
	IPdraw::IRawVideoSink *sink = nullptr;

	res = doCreateRawVideoSink(mediaId, params, listener, &sink);

	mPendingRawVideoSinkAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetRawVideoSink = sink;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSinkDestroy(PdrawBackend::RawVideoSink *sink)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	erased = mRawVideoSinkListenersMap.erase(sink->getRawVideoSink());
	if (erased != 1)
		ULOGW("failed to erase the video sink listener from the map");
	pthread_mutex_unlock(&mMapsMutex);

	delete sink->getRawVideoSink();

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSinkGetQueue(
	PdrawBackend::RawVideoSink *sink)
{
	struct mbuf_raw_video_frame_queue *res =
		sink->getRawVideoSink()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetRawQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSinkQueueFlushed(
	PdrawBackend::RawVideoSink *sink)
{
	int res = sink->getRawVideoSink()->queueFlushed();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetFriendlyNameSetting(void)
{
	std::string friendlyName;
	mPdraw->getFriendlyNameSetting(&friendlyName);

	pthread_mutex_lock(&mMutex);
	mRetString = friendlyName;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetFriendlyNameSetting(
	const std::string &friendlyName)
{
	mPdraw->setFriendlyNameSetting(friendlyName);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSerialNumberSetting(void)
{
	std::string serialNumber;
	mPdraw->getSerialNumberSetting(&serialNumber);

	pthread_mutex_lock(&mMutex);
	mRetString = serialNumber;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSerialNumberSetting(
	const std::string &serialNumber)
{
	mPdraw->setSerialNumberSetting(serialNumber);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetSoftwareVersionSetting(void)
{
	std::string softwareVersion;
	mPdraw->getSoftwareVersionSetting(&softwareVersion);

	pthread_mutex_lock(&mMutex);
	mRetString = softwareVersion;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalSetSoftwareVersionSetting(
	const std::string &softwareVersion)
{
	mPdraw->setSoftwareVersionSetting(softwareVersion);

	pthread_mutex_lock(&mMutex);
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


void PdrawBackend::internalDumpPipeline(const std::string &fileName)
{
	int res = mPdraw->dumpPipeline(fileName);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}

} /* namespace PdrawBackend */
