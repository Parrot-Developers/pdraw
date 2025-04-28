/**
 * Parrot Drones Audio and Video Vector
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

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

#define ULOG_TAG pdraw_backend
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_backend);

#include "pdraw_backend_impl.hpp"

#ifdef _WIN32
#	define PIPE_BUF 4096
#endif /* _WIN32 */

namespace PdrawBackend {


#define MAX_STR_LEN 200
#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


enum cmd_type {
	CMD_TYPE_STOP,
	CMD_TYPE_DEMUXER_CREATE_SINGLE,
	CMD_TYPE_DEMUXER_CREATE_URL,
	CMD_TYPE_DEMUXER_CREATE_URL_MUX,
	CMD_TYPE_DEMUXER_DESTROY,
	CMD_TYPE_DEMUXER_CLOSE,
	CMD_TYPE_DEMUXER_GET_MEDIA_LIST,
	CMD_TYPE_DEMUXER_SELECT_MEDIA,
	CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_STREAM_PORT,
	CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT,
	CMD_TYPE_DEMUXER_IS_READY_TO_PLAY,
	CMD_TYPE_DEMUXER_IS_PAUSED,
	CMD_TYPE_DEMUXER_PLAY,
	CMD_TYPE_DEMUXER_PREVIOUS_FRAME,
	CMD_TYPE_DEMUXER_NEXT_FRAME,
	CMD_TYPE_DEMUXER_SEEK,
	CMD_TYPE_DEMUXER_SEEK_TO,
	CMD_TYPE_DEMUXER_GET_CHAPTER_LIST,
	CMD_TYPE_DEMUXER_GET_CURRENT_TIME,
	CMD_TYPE_DEMUXER_GET_DURATION,
	CMD_TYPE_MUXER_CREATE,
	CMD_TYPE_MUXER_DESTROY,
	CMD_TYPE_MUXER_CLOSE,
	CMD_TYPE_MUXER_ADD_MEDIA,
	CMD_TYPE_MUXER_SET_THUMBNAIL,
	CMD_TYPE_MUXER_ADD_CHAPTER,
	CMD_TYPE_MUXER_GET_STATS,
	CMD_TYPE_MUXER_SET_DYN_PARAMS,
	CMD_TYPE_MUXER_GET_DYN_PARAMS,
	CMD_TYPE_MUXER_FORCE_SYNC,
	CMD_TYPE_VIPC_SOURCE_CREATE,
	CMD_TYPE_VIPC_SOURCE_DESTROY,
	CMD_TYPE_VIPC_SOURCE_IS_READY_TO_PLAY,
	CMD_TYPE_VIPC_SOURCE_IS_PAUSED,
	CMD_TYPE_VIPC_SOURCE_PLAY,
	CMD_TYPE_VIPC_SOURCE_PAUSE,
	CMD_TYPE_VIPC_SOURCE_CONFIGURE,
	CMD_TYPE_VIPC_SOURCE_INSERT_GREY_FRAME,
	CMD_TYPE_VIPC_SOURCE_SET_SESSION_METADATA,
	CMD_TYPE_VIPC_SOURCE_GET_SESSION_METADATA,
	CMD_TYPE_CODED_VIDEO_SOURCE_CREATE,
	CMD_TYPE_CODED_VIDEO_SOURCE_DESTROY,
	CMD_TYPE_CODED_VIDEO_SOURCE_GET_QUEUE,
	CMD_TYPE_CODED_VIDEO_SOURCE_FLUSH,
	CMD_TYPE_CODED_VIDEO_SOURCE_SET_SESSION_METADATA,
	CMD_TYPE_CODED_VIDEO_SOURCE_GET_SESSION_METADATA,
	CMD_TYPE_RAW_VIDEO_SOURCE_CREATE,
	CMD_TYPE_RAW_VIDEO_SOURCE_DESTROY,
	CMD_TYPE_RAW_VIDEO_SOURCE_GET_QUEUE,
	CMD_TYPE_RAW_VIDEO_SOURCE_FLUSH,
	CMD_TYPE_RAW_VIDEO_SOURCE_SET_SESSION_METADATA,
	CMD_TYPE_RAW_VIDEO_SOURCE_GET_SESSION_METADATA,
	CMD_TYPE_CODED_VIDEO_SINK_CREATE,
	CMD_TYPE_CODED_VIDEO_SINK_DESTROY,
	CMD_TYPE_CODED_VIDEO_SINK_RESYNC,
	CMD_TYPE_CODED_VIDEO_SINK_GET_QUEUE,
	CMD_TYPE_CODED_VIDEO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_RAW_VIDEO_SINK_CREATE,
	CMD_TYPE_RAW_VIDEO_SINK_DESTROY,
	CMD_TYPE_RAW_VIDEO_SINK_GET_QUEUE,
	CMD_TYPE_RAW_VIDEO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_ALSA_SOURCE_CREATE,
	CMD_TYPE_ALSA_SOURCE_DESTROY,
	CMD_TYPE_ALSA_SOURCE_IS_READY_TO_PLAY,
	CMD_TYPE_ALSA_SOURCE_IS_PAUSED,
	CMD_TYPE_ALSA_SOURCE_PLAY,
	CMD_TYPE_ALSA_SOURCE_PAUSE,
	CMD_TYPE_AUDIO_SOURCE_CREATE,
	CMD_TYPE_AUDIO_SOURCE_DESTROY,
	CMD_TYPE_AUDIO_SOURCE_GET_QUEUE,
	CMD_TYPE_AUDIO_SOURCE_FLUSH,
	CMD_TYPE_AUDIO_SINK_CREATE,
	CMD_TYPE_AUDIO_SINK_DESTROY,
	CMD_TYPE_AUDIO_SINK_GET_QUEUE,
	CMD_TYPE_AUDIO_SINK_QUEUE_FLUSHED,
	CMD_TYPE_AUDIO_RENDERER_CREATE,
	CMD_TYPE_AUDIO_RENDERER_DESTROY,
	CMD_TYPE_AUDIO_RENDERER_SET_MEDIA_ID,
	CMD_TYPE_AUDIO_RENDERER_GET_MEDIA_ID,
	CMD_TYPE_AUDIO_RENDERER_SET_PARAMS,
	CMD_TYPE_AUDIO_RENDERER_GET_PARAMS,
	CMD_TYPE_VIDEO_ENCODER_CREATE,
	CMD_TYPE_VIDEO_ENCODER_DESTROY,
	CMD_TYPE_VIDEO_ENCODER_CONFIGURE,
	CMD_TYPE_VIDEO_ENCODER_GET_CONFIG,
	CMD_TYPE_VIDEO_SCALER_CREATE,
	CMD_TYPE_VIDEO_SCALER_DESTROY,
	CMD_TYPE_AUDIO_ENCODER_CREATE,
	CMD_TYPE_AUDIO_ENCODER_DESTROY,
	CMD_TYPE_GET_FRIENDLY_NAME_SETTING,
	CMD_TYPE_SET_FRIENDLY_NAME_SETTING,
	CMD_TYPE_GET_SERIAL_NUMBER_SETTING,
	CMD_TYPE_SET_SERIAL_NUMBER_SETTING,
	CMD_TYPE_GET_SOFTWARE_VERSION_SETTING,
	CMD_TYPE_SET_SOFTWARE_VERSION_SETTING,
	CMD_TYPE_DUMP_PIPELINE,
};


struct cmd_msg {
	enum cmd_type type;
	union {
		struct {
			union {
				bool boolean;
				int sint;
				unsigned int uint;
				uint64_t uint64;
				int64_t int64;
				float flt;
				void *ptr;
				char *string;
			};
		} generic;
		struct {
			char *local_addr;
			uint16_t local_stream_port;
			uint16_t local_control_port;
			char *remote_addr;
			uint16_t remote_stream_port;
			uint16_t remote_control_port;
			struct pdraw_demuxer_params params;
			IPdraw::IDemuxer::Listener *listener;
		} demuxer_create_single;
		struct {
			char *url;
			struct pdraw_demuxer_params params;
			IPdraw::IDemuxer::Listener *listener;
		} demuxer_create_url;
		struct {
			char *url;
			struct mux_ctx *mux;
			struct pdraw_demuxer_params params;
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
			PdrawBackend::Demuxer *demuxer;
			struct pdraw_chapter **chapter_list;
			size_t *chapter_count;
		} demuxer_get_chapter_list;
		struct {
			PdrawBackend::Demuxer *demuxer;
			uint32_t selected_medias;
		} demuxer_select_media;
		struct {
			char *address;
			char *friendly_name;
			char *backend_name;
			IPdraw::IVipcSource::Listener *listener;
		} vipc_source_create;
		struct {
			IPdraw::IVipcSource *source;
			struct vdef_dim resolution;
			struct vdef_rectf crop;
		} vipc_source_configure;
		struct {
			IPdraw::ICodedVideoSource::Listener *listener;
		} coded_video_source_create;
		struct {
			IPdraw::IRawVideoSource::Listener *listener;
		} raw_video_source_create;
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
			struct pdraw_alsa_source_params params;
			IPdraw::IAlsaSource::Listener *listener;
		} alsa_source_create;
		struct {
			struct pdraw_audio_source_params params;
			IPdraw::IAudioSource::Listener *listener;
		} audio_source_create;
		struct {
			unsigned int media_id;
			IPdraw::IAudioSink::Listener *listener;
		} audio_sink_create;
		struct {
			unsigned int media_id;
			struct pdraw_audio_renderer_params params;
			IPdraw::IAudioRenderer::Listener *listener;
		} audio_renderer_create;
		struct {
			PdrawBackend::AudioRenderer *renderer;
			unsigned int media_id;
		} audio_renderer_set_media_id;
		struct {
			unsigned int media_id;
			struct venc_config params;
			IPdraw::IVideoEncoder::Listener *listener;
		} video_encoder_create;
		struct {
			IPdraw::IVideoEncoder *encoder;
			struct venc_dyn_config config;
		} video_encoder_configure;
		struct {
			IPdraw::IVideoEncoder *encoder;
			struct venc_dyn_config config;
		} video_encoder_get_config;
		struct {
			unsigned int media_id;
			struct vscale_config params;
			IPdraw::IVideoScaler::Listener *listener;
		} video_scaler_create;
		struct {
			unsigned int media_id;
			struct aenc_config params;
			IPdraw::IAudioEncoder::Listener *listener;
		} audio_encoder_create;
		struct {
			char *url;
			struct pdraw_muxer_params params;
			IPdraw::IMuxer::Listener *listener;
		} muxer_create;
		struct {
			IPdraw::IMuxer *muxer;
			unsigned int media_id;
			struct pdraw_muxer_media_params *params;
		} muxer_add_media;
		struct {
			IPdraw::IMuxer *muxer;
			enum pdraw_muxer_thumbnail_type type;
			const uint8_t *data;
			size_t size;
		} muxer_set_thumbnail;
		struct {
			IPdraw::IMuxer *muxer;
			uint64_t timestamp;
			char *name;
		} muxer_add_chapter;
		struct {
			IPdraw::IMuxer *muxer;
			struct pdraw_muxer_dyn_params dyn_params;
		} muxer_set_dyn_params;
		struct {
			float xdpi;
			float ydpi;
			float device_margin_top;
			float device_margin_bottom;
			float device_margin_left;
			float device_margin_right;
		} set_display_screen_settings;
		struct {
			char *file_name;
		} dump_pipeline;
	};
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_msg) <= PIPE_BUF - 1);


/* Execution is not on main thread. mMutex must be locked before calling this
 * function. */
int PdrawBackend::pushCmdAndWait(const struct cmd_msg *cmd)
{
	int res = 0;

	mRetValReady = false;
	res = mbox_push(mMbox, cmd);
	if (res < 0) {
		ULOG_ERRNO("mbox_push", -res);
		return res;
	}

	while (!mRetValReady)
		pthread_cond_wait(&mCond, &mMutex);

	mRetValReady = false;
	res = mRetStatus;
	mRetStatus = 0;
	return res;
}


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
	memset(&mParamVideoSource, 0, sizeof(mParamVideoSource));
	memset(&mParamVipcSource, 0, sizeof(mParamVipcSource));
	memset(&mParamAudioRenderer, 0, sizeof(mParamAudioRenderer));
	memset(&mParamVmetaSession, 0, sizeof(mParamVmetaSession));
	mRetValReady = false;
	mRetStatus = 0;
	mRetBool = false;
	mRetUint16 = 0;
	mRetUint32 = 0;
	mRetUint64 = 0;
	mRetSizeT = 0;
	mRetUint = 0;
	mRetCodedQueue = nullptr;
	mRetRawQueue = nullptr;
	mRetAudioQueue = nullptr;
	mRetVipcSource = nullptr;
	mRetCodedVideoSource = nullptr;
	mRetRawVideoSource = nullptr;
	mRetCodedVideoSink = nullptr;
	mRetRawVideoSink = nullptr;
	mRetAlsaSource = nullptr;
	mRetAudioSource = nullptr;
	mRetAudioSink = nullptr;
	mRetAudioRenderer = nullptr;
	mRetVideoEncoder = nullptr;
	mRetVideoScaler = nullptr;
	mRetAudioEncoder = nullptr;
	mRetDemuxer = nullptr;
	mRetMuxer = nullptr;
	mPdraw = nullptr;
	mListener = listener;
	mMapsMutexCreated = false;
	mPendingDemuxerAndListener = {};
	mPendingMuxerAndListener = {};
	mPendingVideoRendererAndListener = {};
	mPendingVipcSourceAndListener = {};
	mPendingCodedVideoSourceAndListener = {};
	mPendingRawVideoSourceAndListener = {};
	mPendingCodedVideoSinkAndListener = {};
	mPendingRawVideoSinkAndListener = {};
	mPendingAlsaSourceAndListener = {};
	mPendingAudioSourceAndListener = {};
	mPendingAudioSinkAndListener = {};
	mPendingAudioRendererAndListener = {};
	mPendingVideoEncoderAndListener = {};
	mPendingVideoScalerAndListener = {};
	mPendingAudioEncoderAndListener = {};
	mPendingRemovedElementUserdata = {};
	memset(&mLoopThread, 0, sizeof(pthread_t));
	memset(&mRetConfig, 0, sizeof(mRetConfig));
	memset(&mRetMuxerStats, 0, sizeof(mRetMuxerStats));
	memset(&mRetMuxerDynParams, 0, sizeof(mRetMuxerDynParams));
	mRetMediaList = nullptr;
}


PdrawBackend::~PdrawBackend(void)
{
	int err;

	if (mStarted)
		ULOGW("destroying pdraw backend while still running");

	mThreadShouldStop = true;
	if (mMutexCreated)
		pthread_mutex_lock(&mMutex);
	if (mLoop != nullptr) {
		err = pomp_loop_wakeup(mLoop);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -err);
	}
	if (mMutexCreated)
		pthread_mutex_unlock(&mMutex);
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_STOP;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateDemuxer(url, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_URL;
	cmd->demuxer_create_url.url = strdup(url.c_str());
	cmd->demuxer_create_url.params = *params;
	cmd->demuxer_create_url.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetDemuxer;
	mRetDemuxer = nullptr;

out:
	if (cmd && cmd->demuxer_create_url.url)
		free(cmd->demuxer_create_url.url);
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
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(localAddr.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(remoteAddr.length() > MAX_STR_LEN, ENOBUFS);
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
				      params,
				      listener,
				      retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_SINGLE;
	cmd->demuxer_create_single.local_addr = strdup(localAddr.c_str());
	cmd->demuxer_create_single.local_stream_port = localStreamPort;
	cmd->demuxer_create_single.local_control_port = localControlPort;
	cmd->demuxer_create_single.remote_addr = strdup(remoteAddr.c_str());
	cmd->demuxer_create_single.remote_stream_port = remoteStreamPort;
	cmd->demuxer_create_single.remote_control_port = remoteControlPort;
	cmd->demuxer_create_single.params = *params;
	cmd->demuxer_create_single.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetDemuxer;
	mRetDemuxer = nullptr;

out:
	if (cmd) {
		if (cmd->demuxer_create_single.local_addr)
			free(cmd->demuxer_create_single.local_addr);
		if (cmd->demuxer_create_single.remote_addr)
			free(cmd->demuxer_create_single.remote_addr);
		free(cmd);
	}
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


int PdrawBackend::createDemuxer(const std::string &url,
				struct mux_ctx *mux,
				const struct pdraw_demuxer_params *params,
				IPdraw::IDemuxer::Listener *listener,
				IPdraw::IDemuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateDemuxer(url, mux, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CREATE_URL_MUX;
	cmd->demuxer_create_url_mux.url = strdup(url.c_str());
	cmd->demuxer_create_url_mux.mux = mux;
	cmd->demuxer_create_url_mux.params = *params;
	cmd->demuxer_create_url_mux.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetDemuxer;
	mRetDemuxer = nullptr;

out:
	if (cmd && cmd->demuxer_create_url_mux.url)
		free(cmd->demuxer_create_url_mux.url);
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_CLOSE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::getMediaList(struct pdraw_demuxer_media **mediaList,
					size_t *mediaCount,
					uint32_t *selectedMedias)
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
		res = mDemuxer->getMediaList(
			mediaList, mediaCount, selectedMedias);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_MEDIA_LIST;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*mediaList = mBackend->mRetMediaList;
	*mediaCount = mBackend->mRetSizeT;
	*selectedMedias = mBackend->mRetUint32;
	mBackend->mRetMediaList = nullptr;
	mBackend->mRetSizeT = 0;
	mBackend->mRetUint32 = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::selectMedia(uint32_t selectedMedias)
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
		res = mDemuxer->selectMedia(selectedMedias);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_SELECT_MEDIA;
	cmd->demuxer_select_media.demuxer = this;
	cmd->demuxer_select_media.selected_medias = selectedMedias;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_STREAM_PORT;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetUint16;
	mBackend->mRetUint16 = 0;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_SINGLE_STREAM_LOCAL_CONTROL_PORT;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetUint16;
	mBackend->mRetUint16 = 0;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_IS_READY_TO_PLAY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_IS_PAUSED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

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
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_PREVIOUS_FRAME;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_NEXT_FRAME;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Demuxer::getChapterList(struct pdraw_chapter **chapterList,
					  size_t *chapterCount)
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
		res = mDemuxer->getChapterList(chapterList, chapterCount);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_CHAPTER_LIST;
	cmd->demuxer_get_chapter_list.demuxer = this;
	cmd->demuxer_get_chapter_list.chapter_list = chapterList;
	cmd->demuxer_get_chapter_list.chapter_count = chapterCount;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_DURATION;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetUint64;
	mBackend->mRetUint64 = 0;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DEMUXER_GET_CURRENT_TIME;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetUint64;
	mBackend->mRetUint64 = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::createMuxer(const std::string &url,
			      const struct pdraw_muxer_params *params,
			      IPdraw::IMuxer::Listener *listener,
			      IPdraw::IMuxer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateMuxer(url, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_CREATE;
	cmd->muxer_create.url = strdup(url.c_str());
	cmd->muxer_create.params = *params;
	cmd->muxer_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetMuxer;
	mRetMuxer = nullptr;

out:
	if (cmd && cmd->muxer_create.url)
		free(cmd->muxer_create.url);
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::Muxer::Muxer(PdrawBackend *backend,
			   const std::string &url,
			   IPdraw::IMuxer::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::Muxer::close(void)
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
		res = mMuxer->close();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_CLOSE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::addMedia(unsigned int mediaId,
				  const struct pdraw_muxer_media_params *params)
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
	if (params != nullptr) {
		cmd->muxer_add_media.params =
			(struct pdraw_muxer_media_params *)calloc(
				1, sizeof(struct pdraw_muxer_media_params));
		if (cmd->muxer_add_media.params == nullptr) {
			res = -ENOMEM;
			ULOG_ERRNO("calloc", -res);
			goto out;
		}
		*cmd->muxer_add_media.params = *params;
		if (params->track_name != nullptr) {
			cmd->muxer_add_media.params->track_name =
				strdup(params->track_name);
			if (cmd->muxer_add_media.params->track_name ==
			    nullptr) {
				res = -ENOMEM;
				ULOG_ERRNO("strdup", -res);
				goto out;
			}
		}
	}
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	if (cmd && cmd->muxer_add_media.params) {
		free((void *)cmd->muxer_add_media.params->track_name);
		free(cmd->muxer_add_media.params);
	}
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
				      const uint8_t *data,
				      size_t size)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->setThumbnail(type, data, size);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_SET_THUMBNAIL;
	cmd->muxer_set_thumbnail.muxer = this;
	cmd->muxer_set_thumbnail.type = type;
	cmd->muxer_set_thumbnail.data = data;
	cmd->muxer_set_thumbnail.size = size;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
};


int PdrawBackend::Muxer::addChapter(uint64_t timestamp, const char *name)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(timestamp == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(name == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->addChapter(timestamp, name);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_ADD_CHAPTER;
	cmd->muxer_add_chapter.muxer = this;
	cmd->muxer_add_chapter.timestamp = timestamp;
	cmd->muxer_add_chapter.name = strdup(name);
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	if (cmd)
		free(cmd->muxer_add_chapter.name);
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::getStats(struct pdraw_muxer_stats *stats)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->getStats(stats);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_GET_STATS;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*stats = mBackend->mRetMuxerStats;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::setDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->setDynParams(dyn_params);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_SET_DYN_PARAMS;
	cmd->muxer_set_dyn_params.muxer = this;
	cmd->muxer_set_dyn_params.dyn_params = *dyn_params;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->getDynParams(dyn_params);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_GET_DYN_PARAMS;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*dyn_params = mBackend->mRetMuxerDynParams;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::Muxer::forceSync(void)
{
	int res = 0;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mMuxer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mMuxer->forceSync();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_MUXER_FORCE_SYNC;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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
	IPdraw::IVideoRenderer **retObj)
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
		this, renderPos, params, listener);
	if (renderer == nullptr)
		goto out;

	mPendingVideoRendererAndListener = {
		.r = renderer,
		.l = listener,
	};

	res = mPdraw->createVideoRenderer(
		mediaId, renderPos, params, this, &rnd);
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
	IPdraw::IVideoRenderer::Listener *listener)
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


int PdrawBackend::createVipcSource(
	const struct pdraw_vipc_source_params *params,
	IPdraw::IVipcSource::Listener *listener,
	IPdraw::IVipcSource **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(strlen(params->address) > MAX_STR_LEN,
				 ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		(params->friendly_name != nullptr) &&
			(strlen(params->friendly_name) > MAX_STR_LEN),
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(
		(params->backend_name != nullptr) &&
			(strlen(params->backend_name) > MAX_STR_LEN),
		ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateVipcSource(params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_CREATE;
	mParamVipcSource = *params;
	cmd->vipc_source_create.address = strdup(params->address);
	mParamVipcSource.address = cmd->vipc_source_create.address;
	if (params->friendly_name != nullptr) {
		cmd->vipc_source_create.friendly_name =
			strdup(params->friendly_name);
		mParamVipcSource.friendly_name =
			cmd->vipc_source_create.friendly_name;
	}
	if (params->backend_name != nullptr) {
		cmd->vipc_source_create.backend_name =
			strdup(params->backend_name);
		mParamVipcSource.backend_name =
			cmd->vipc_source_create.backend_name;
	}
	cmd->vipc_source_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetVipcSource;
	mRetVipcSource = nullptr;

out:
	if (cmd != nullptr) {
		if (cmd->vipc_source_create.address != nullptr)
			free(cmd->vipc_source_create.address);
		if (cmd->vipc_source_create.friendly_name != nullptr)
			free(cmd->vipc_source_create.friendly_name);
		if (cmd->vipc_source_create.backend_name != nullptr)
			free(cmd->vipc_source_create.backend_name);
		free(cmd);
	}
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::VipcSource::VipcSource(PdrawBackend *backend,
				     IPdraw::IVipcSource::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSource = nullptr;
}


PdrawBackend::VipcSource::~VipcSource(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSource;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


bool PdrawBackend::VipcSource::isReadyToPlay(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->isReadyToPlay();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_IS_READY_TO_PLAY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


bool PdrawBackend::VipcSource::isPaused(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->isPaused();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_IS_PAUSED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::VipcSource::play(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->play();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_PLAY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VipcSource::pause(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->pause();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_PAUSE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VipcSource::insertGreyFrame(uint64_t tsUs)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->insertGreyFrame(tsUs);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_INSERT_GREY_FRAME;
	cmd->generic.ptr = this;
	cmd->generic.uint64 = tsUs;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VipcSource::configure(const struct vdef_dim *resolution,
					const struct vdef_rectf *crop)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->configure(resolution, crop);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_CONFIGURE;
	cmd->vipc_source_configure.source = this;
	cmd->vipc_source_configure.resolution = *resolution;
	cmd->vipc_source_configure.crop = *crop;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VipcSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->setSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_SET_SESSION_METADATA;
	cmd->generic.ptr = this;
	mBackend->mParamVmetaSession = *meta;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VipcSource::getSessionMetadata(struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->getSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIPC_SOURCE_GET_SESSION_METADATA;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*meta = mBackend->mParamVmetaSession;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateCodedVideoSource(params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_CREATE;
	mParamVideoSource = *params;
	cmd->coded_video_source_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetCodedVideoSource;
	mRetCodedVideoSource = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::CodedVideoSource::CodedVideoSource(
	PdrawBackend *backend,
	IPdraw::ICodedVideoSource::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSource = nullptr;
}


PdrawBackend::CodedVideoSource::~CodedVideoSource(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSource;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


struct mbuf_coded_video_frame_queue *
PdrawBackend::CodedVideoSource::getQueue(void)
{
	int res;
	struct mbuf_coded_video_frame_queue *ret = nullptr;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, nullptr);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->getQueue();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetCodedQueue;
	mBackend->mRetCodedQueue = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::CodedVideoSource::flush(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->flush();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_FLUSH;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::CodedVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->setSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_SET_SESSION_METADATA;
	cmd->generic.ptr = this;
	mBackend->mParamVmetaSession = *meta;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::CodedVideoSource::getSessionMetadata(
	struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->getSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SOURCE_GET_SESSION_METADATA;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*meta = mBackend->mParamVmetaSession;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateRawVideoSource(params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_CREATE;
	mParamVideoSource = *params;
	cmd->raw_video_source_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetRawVideoSource;
	mRetRawVideoSource = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::RawVideoSource::RawVideoSource(
	PdrawBackend *backend,
	IPdraw::IRawVideoSource::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSource = nullptr;
}


PdrawBackend::RawVideoSource::~RawVideoSource(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSource;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


struct mbuf_raw_video_frame_queue *PdrawBackend::RawVideoSource::getQueue(void)
{
	int res;
	struct mbuf_raw_video_frame_queue *ret = nullptr;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, nullptr);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->getQueue();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetRawQueue;
	mBackend->mRetRawQueue = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::RawVideoSource::flush(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->flush();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_FLUSH;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::RawVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->setSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_SET_SESSION_METADATA;
	cmd->generic.ptr = this;
	mBackend->mParamVmetaSession = *meta;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::RawVideoSource::getSessionMetadata(struct vmeta_session *meta)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->getSessionMetadata(meta);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SOURCE_GET_SESSION_METADATA;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*meta = mBackend->mParamVmetaSession;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
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
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetCodedVideoSink;
	mRetCodedVideoSink = nullptr;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_RESYNC;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetCodedQueue;
	mBackend->mRetCodedQueue = 0;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_CODED_VIDEO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetRawVideoSink;
	mRetRawVideoSink = nullptr;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetRawQueue;
	mBackend->mRetRawQueue = 0;

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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_RAW_VIDEO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createAlsaSource(
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener,
	IPdraw::IAlsaSource **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(strlen(params->address) > MAX_STR_LEN,
				 ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateAlsaSource(params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_CREATE;
	cmd->alsa_source_create.params = *params;
	cmd->alsa_source_create.params.address = strdup(params->address);
	cmd->alsa_source_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetAlsaSource;
	mRetAlsaSource = nullptr;

out:
	if (cmd != nullptr)
		free((void *)cmd->alsa_source_create.params.address);
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


int PdrawBackend::createAudioSource(
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener,
	IPdraw::IAudioSource **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateAudioSource(params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SOURCE_CREATE;
	cmd->audio_source_create.params = *params;
	cmd->audio_source_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetAudioSource;
	mRetAudioSource = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::AlsaSource::AlsaSource(PdrawBackend *backend,
				     IPdraw::IAlsaSource::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSource = nullptr;
}


PdrawBackend::AlsaSource::~AlsaSource(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSource;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


bool PdrawBackend::AlsaSource::isReadyToPlay(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->isReadyToPlay();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_IS_READY_TO_PLAY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


bool PdrawBackend::AlsaSource::isPaused(void)
{
	int res;
	bool ret = false;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, false);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, false);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->isPaused();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_IS_PAUSED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetBool;
	mBackend->mRetBool = false;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::AlsaSource::play(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->play();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_PLAY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::AlsaSource::pause(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->pause();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_ALSA_SOURCE_PAUSE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


PdrawBackend::AudioSource::AudioSource(PdrawBackend *backend,
				       IPdraw::IAudioSource::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSource = nullptr;
}


PdrawBackend::AudioSource::~AudioSource(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mSource;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SOURCE_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


struct mbuf_audio_frame_queue *PdrawBackend::AudioSource::getQueue(void)
{
	int res;
	struct mbuf_audio_frame_queue *ret = nullptr;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_VAL_IF(mBackend == nullptr, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(!mBackend->mStarted, EPROTO, nullptr);
	ULOG_ERRNO_RETURN_VAL_IF(mSource == nullptr, EPROTO, nullptr);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mSource->getQueue();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SOURCE_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetAudioQueue;
	mBackend->mRetAudioQueue = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::AudioSource::flush(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mSource == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mSource->flush();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SOURCE_FLUSH;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createAudioSink(unsigned int mediaId,
				  IPdraw::IAudioSink::Listener *listener,
				  IPdraw::IAudioSink **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateAudioSink(mediaId, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SINK_CREATE;
	cmd->audio_sink_create.media_id = mediaId;
	cmd->audio_sink_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetAudioSink;
	mRetAudioSink = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::AudioSink::AudioSink(PdrawBackend *backend,
				   unsigned int mediaId,
				   IPdraw::IAudioSink::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mSink = nullptr;
}


PdrawBackend::AudioSink::~AudioSink(void)
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SINK_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


struct mbuf_audio_frame_queue *PdrawBackend::AudioSink::getQueue(void)
{
	int res;
	struct mbuf_audio_frame_queue *ret = nullptr;
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SINK_GET_QUEUE;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	mBackend->mRetAudioQueue = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::AudioSink::queueFlushed(void)
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_SINK_QUEUE_FLUSHED;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateAudioRenderer(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_CREATE;
	cmd->audio_renderer_create.media_id = mediaId;
	cmd->audio_renderer_create.params = *params;
	cmd->audio_renderer_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetAudioRenderer;
	mRetAudioRenderer = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::AudioRenderer::AudioRenderer(
	PdrawBackend *backend,
	IPdraw::IAudioRenderer::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mRenderer = nullptr;
}


PdrawBackend::AudioRenderer::~AudioRenderer(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mRenderer;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::AudioRenderer::setMediaId(unsigned int mediaId)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mRenderer->setMediaId(mediaId);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_SET_MEDIA_ID;
	cmd->audio_renderer_set_media_id.renderer = this;
	cmd->audio_renderer_set_media_id.media_id = mediaId;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


unsigned int PdrawBackend::AudioRenderer::getMediaId(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;
	unsigned int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mRenderer->getMediaId();
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_GET_MEDIA_ID;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetUint;
	mBackend->mRetUint = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::AudioRenderer::setParams(
	const struct pdraw_audio_renderer_params *params)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mRenderer->setParams(params);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_SET_PARAMS;
	cmd->generic.ptr = this;
	mBackend->mParamAudioRenderer = *params;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	res = mBackend->mRetStatus;
	mBackend->mRetStatus = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::AudioRenderer::getParams(
	struct pdraw_audio_renderer_params *params)
{
	int res;
	struct cmd_msg *cmd = nullptr;
	unsigned int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mRenderer == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		ret = mRenderer->getParams(params);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_RENDERER_GET_PARAMS;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	ret = mBackend->mRetStatus;
	*params = mBackend->mParamAudioRenderer;
	mBackend->mRetStatus = 0;

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return ret;
}


int PdrawBackend::createVideoEncoder(unsigned int mediaId,
				     const struct venc_config *params,
				     IPdraw::IVideoEncoder::Listener *listener,
				     IPdraw::IVideoEncoder **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateVideoEncoder(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_ENCODER_CREATE;
	cmd->video_encoder_create.media_id = mediaId;
	cmd->video_encoder_create.params = *params;
	cmd->video_encoder_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetVideoEncoder;
	mRetVideoEncoder = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::VideoEncoder::VideoEncoder(
	PdrawBackend *backend,
	unsigned int mediaId,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mEncoder = nullptr;
}


PdrawBackend::VideoEncoder::~VideoEncoder(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mEncoder == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mEncoder;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_ENCODER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::VideoEncoder::configure(const struct venc_dyn_config *config)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mEncoder == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(config == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mEncoder->configure(config);
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_ENCODER_CONFIGURE;
	cmd->video_encoder_configure.encoder = this;
	cmd->video_encoder_configure.config = *config;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::VideoEncoder::getConfig(struct venc_dyn_config *config)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(mEncoder == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(config == nullptr, EINVAL);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mEncoder->getConfig(config);
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
	cmd->type = CMD_TYPE_VIDEO_ENCODER_GET_CONFIG;
	cmd->video_encoder_get_config.encoder = this;
	cmd->video_encoder_get_config.config = *config;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*config = mBackend->mRetConfig;
	mBackend->mRetConfig = {};

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
	return res;
}


int PdrawBackend::createVideoScaler(unsigned int mediaId,
				    const struct vscale_config *params,
				    IPdraw::IVideoScaler::Listener *listener,
				    IPdraw::IVideoScaler **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateVideoScaler(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_SCALER_CREATE;
	cmd->video_scaler_create.media_id = mediaId;
	cmd->video_scaler_create.params = *params;
	cmd->video_scaler_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetVideoScaler;
	mRetVideoScaler = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::VideoScaler::VideoScaler(PdrawBackend *backend,
				       unsigned int mediaId,
				       const struct vscale_config *params,
				       IPdraw::IVideoScaler::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mScaler = nullptr;
}


PdrawBackend::VideoScaler::~VideoScaler(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mScaler == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mScaler;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_VIDEO_SCALER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
}


int PdrawBackend::createAudioEncoder(unsigned int mediaId,
				     const struct aenc_config *params,
				     IPdraw::IAudioEncoder::Listener *listener,
				     IPdraw::IAudioEncoder **retObj)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = doCreateAudioEncoder(mediaId, params, listener, retObj);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_ENCODER_CREATE;
	cmd->audio_encoder_create.media_id = mediaId;
	cmd->audio_encoder_create.params = *params;
	cmd->audio_encoder_create.listener = listener;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	*retObj = mRetAudioEncoder;
	mRetAudioEncoder = nullptr;

out:
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
	return res;
}


PdrawBackend::AudioEncoder::AudioEncoder(
	PdrawBackend *backend,
	unsigned int mediaId,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener)
{
	mBackend = backend;
	mListener = listener;
	mEncoder = nullptr;
}


PdrawBackend::AudioEncoder::~AudioEncoder(void)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_IF(mBackend == nullptr, EPROTO);
	ULOG_ERRNO_RETURN_IF(!mBackend->mStarted, EPROTO);
	ULOG_ERRNO_RETURN_IF(mEncoder == nullptr, EPROTO);

	pthread_mutex_lock(&mBackend->mApiMutex);

	if (pthread_self() == mBackend->mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		delete mEncoder;
		goto out2;
	}

	pthread_mutex_lock(&mBackend->mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_AUDIO_ENCODER_DESTROY;
	cmd->generic.ptr = this;
	res = mBackend->pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	free(cmd);
	pthread_mutex_unlock(&mBackend->mMutex);

out2:
	pthread_mutex_unlock(&mBackend->mApiMutex);
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_FRIENDLY_NAME_SETTING;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	if (friendlyName)
		*friendlyName = mRetString;
	mRetString = "";

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
	ULOG_ERRNO_RETURN_IF(friendlyName.length() > MAX_STR_LEN, ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setFriendlyNameSetting(friendlyName);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_FRIENDLY_NAME_SETTING;
	cmd->generic.string = strdup(friendlyName.c_str());
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	if (cmd && cmd->generic.string)
		free(cmd->generic.string);
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SERIAL_NUMBER_SETTING;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	if (serialNumber)
		*serialNumber = mRetString;
	mRetString = "";

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
	ULOG_ERRNO_RETURN_IF(serialNumber.length() > MAX_STR_LEN, ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setSerialNumberSetting(serialNumber);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SERIAL_NUMBER_SETTING;
	cmd->generic.string = strdup(serialNumber.c_str());
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	if (cmd && cmd->generic.string)
		free(cmd->generic.string);
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

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_GET_SOFTWARE_VERSION_SETTING;
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

	if (softwareVersion)
		*softwareVersion = mRetString;
	mRetString = "";

out:
	if (cmd && cmd->generic.string)
		free(cmd->generic.string);
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
	ULOG_ERRNO_RETURN_IF(softwareVersion.length() > MAX_STR_LEN, ENOBUFS);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		mPdraw->setSoftwareVersionSetting(softwareVersion);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_SET_SOFTWARE_VERSION_SETTING;
	cmd->generic.string = strdup(softwareVersion.c_str());
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}


out:
	if (cmd && cmd->generic.string)
		free(cmd->generic.string);
	free(cmd);
	pthread_mutex_unlock(&mMutex);

out2:
	pthread_mutex_unlock(&mApiMutex);
}


int PdrawBackend::dumpPipeline(const std::string &fileName)
{
	int res;
	struct cmd_msg *cmd = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(fileName.length() > MAX_STR_LEN, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPROTO);

	pthread_mutex_lock(&mApiMutex);

	if (pthread_self() == mLoopThread) {
		/* Execution is on the loop thread,
		 * call the function directly */
		res = mPdraw->dumpPipeline(fileName);
		goto out2;
	}

	pthread_mutex_lock(&mMutex);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	/* Send a message to the loop */
	cmd->type = CMD_TYPE_DUMP_PIPELINE;
	cmd->dump_pipeline.file_name = strdup(fileName.c_str());
	res = pushCmdAndWait(cmd);
	if (res < 0) {
		ULOG_ERRNO("pushCmdAndWait", -res);
		goto out;
	}

out:
	if (cmd && cmd->dump_pipeline.file_name)
		free(cmd->dump_pipeline.file_name);
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
				const struct pdraw_media_info *info,
				void *elementUserData)
{
	bool found = false;
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it1;
	struct demuxerAndListener sl1;
	std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator it2;
	struct muxerAndListener sl2;
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it3;
	struct vipcSourceAndListener sl3;
	std::map<IPdraw::ICodedVideoSource *,
		 struct codedVideoSourceAndListener>::iterator it4;
	struct codedVideoSourceAndListener sl4;
	std::map<IPdraw::IRawVideoSource *,
		 struct rawVideoSourceAndListener>::iterator it5;
	struct rawVideoSourceAndListener sl5;
	std::map<IPdraw::IVideoEncoder *,
		 struct videoEncoderAndListener>::iterator it6;
	struct videoEncoderAndListener sl6;
	std::map<IPdraw::IVideoScaler *,
		 struct videoScalerAndListener>::iterator it7;
	struct videoScalerAndListener sl7;
	std::map<IPdraw::IAlsaSource *, struct alsaSourceAndListener>::iterator
		it8;
	struct alsaSourceAndListener sl8;
	std::map<IPdraw::IAudioSource *,
		 struct audioSourceAndListener>::iterator it9;
	struct audioSourceAndListener sl9;
	std::map<IPdraw::IAudioEncoder *,
		 struct audioEncoderAndListener>::iterator it10;
	struct audioEncoderAndListener sl10;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	if (elementUserData == nullptr)
		goto cb;

	pthread_mutex_lock(&mMapsMutex);

	it1 = mDemuxerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(elementUserData));
	if (it1 != mDemuxerListenersMap.end()) {
		sl1 = it1->second;
		elementUserData = sl1.d;
		found = true;
		goto cb_unlock;
	}
	it2 = mMuxerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(elementUserData));
	if (it2 != mMuxerListenersMap.end()) {
		sl2 = it2->second;
		elementUserData = sl2.m;
		found = true;
		goto cb_unlock;
	}
	it3 = mVipcSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(
			elementUserData));
	if (it3 != mVipcSourceListenersMap.end()) {
		sl3 = it3->second;
		elementUserData = sl3.s;
		found = true;
		goto cb_unlock;
	}
	it4 = mCodedVideoSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(
			elementUserData));
	if (it4 != mCodedVideoSourceListenersMap.end()) {
		sl4 = it4->second;
		elementUserData = sl4.s;
		found = true;
		goto cb_unlock;
	}
	it5 = mRawVideoSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(
			elementUserData));
	if (it5 != mRawVideoSourceListenersMap.end()) {
		sl5 = it5->second;
		elementUserData = sl5.s;
		found = true;
		goto cb_unlock;
	}
	it6 = mVideoEncoderListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(
			elementUserData));
	if (it6 != mVideoEncoderListenersMap.end()) {
		sl6 = it6->second;
		elementUserData = sl6.e;
		found = true;
		goto cb_unlock;
	}
	it7 = mVideoScalerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(
			elementUserData));
	if (it7 != mVideoScalerListenersMap.end()) {
		sl7 = it7->second;
		elementUserData = sl7.s;
		found = true;
		goto cb_unlock;
	}
	it8 = mAlsaSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(
			elementUserData));
	if (it8 != mAlsaSourceListenersMap.end()) {
		sl8 = it8->second;
		elementUserData = sl8.s;
		found = true;
		goto cb_unlock;
	}
	it9 = mAudioSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(
			elementUserData));
	if (it9 != mAudioSourceListenersMap.end()) {
		sl9 = it9->second;
		elementUserData = sl9.s;
		found = true;
		goto cb_unlock;
	}
	it10 = mAudioEncoderListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(
			elementUserData));
	if (it10 != mAudioEncoderListenersMap.end()) {
		sl10 = it10->second;
		elementUserData = sl10.e;
		found = true;
		goto cb_unlock;
	}

	if (!found) {
		ULOGW("%s: element userdata not found", __func__);
		elementUserData = nullptr;
	}

cb_unlock:
	pthread_mutex_unlock(&mMapsMutex);
cb:
	mListener->onMediaAdded(this, info, elementUserData);
}


void PdrawBackend::onMediaRemoved(IPdraw *pdraw,
				  const struct pdraw_media_info *info,
				  void *elementUserData)
{
	bool found = false;
	std::map<IPdraw::IDemuxer *, struct demuxerAndListener>::iterator it1;
	struct demuxerAndListener sl1;
	std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator it2;
	struct muxerAndListener sl2;
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it3;
	struct vipcSourceAndListener sl3;
	std::map<IPdraw::ICodedVideoSource *,
		 struct codedVideoSourceAndListener>::iterator it4;
	struct codedVideoSourceAndListener sl4;
	std::map<IPdraw::IRawVideoSource *,
		 struct rawVideoSourceAndListener>::iterator it5;
	struct rawVideoSourceAndListener sl5;
	std::map<IPdraw::IVideoEncoder *,
		 struct videoEncoderAndListener>::iterator it6;
	struct videoEncoderAndListener sl6;
	std::map<IPdraw::IVideoScaler *,
		 struct videoScalerAndListener>::iterator it7;
	struct videoScalerAndListener sl7;
	std::map<IPdraw::IAlsaSource *, struct alsaSourceAndListener>::iterator
		it8;
	struct alsaSourceAndListener sl8;
	std::map<IPdraw::IAudioSource *,
		 struct audioSourceAndListener>::iterator it9;
	struct audioSourceAndListener sl9;
	std::map<IPdraw::IAudioEncoder *,
		 struct audioEncoderAndListener>::iterator it10;
	struct audioEncoderAndListener sl10;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	if (elementUserData == nullptr)
		goto cb;

	pthread_mutex_lock(&mMapsMutex);

	it1 = mDemuxerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IDemuxer *>(elementUserData));
	if (it1 != mDemuxerListenersMap.end()) {
		sl1 = it1->second;
		elementUserData = sl1.d;
		found = true;
		goto cb_unlock;
	}
	it2 = mMuxerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IMuxer *>(elementUserData));
	if (it2 != mMuxerListenersMap.end()) {
		sl2 = it2->second;
		elementUserData = sl2.m;
		found = true;
		goto cb_unlock;
	}
	it3 = mVipcSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVipcSource *>(
			elementUserData));
	if (it3 != mVipcSourceListenersMap.end()) {
		sl3 = it3->second;
		elementUserData = sl3.s;
		found = true;
		goto cb_unlock;
	}
	it4 = mCodedVideoSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::ICodedVideoSource *>(
			elementUserData));
	if (it4 != mCodedVideoSourceListenersMap.end()) {
		sl4 = it4->second;
		elementUserData = sl4.s;
		found = true;
		goto cb_unlock;
	}
	it5 = mRawVideoSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IRawVideoSource *>(
			elementUserData));
	if (it5 != mRawVideoSourceListenersMap.end()) {
		sl5 = it5->second;
		elementUserData = sl5.s;
		found = true;
		goto cb_unlock;
	}
	it6 = mVideoEncoderListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVideoEncoder *>(
			elementUserData));
	if (it6 != mVideoEncoderListenersMap.end()) {
		sl6 = it6->second;
		elementUserData = sl6.e;
		found = true;
		goto cb_unlock;
	}
	it7 = mVideoScalerListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IVideoScaler *>(
			elementUserData));
	if (it7 != mVideoScalerListenersMap.end()) {
		sl7 = it7->second;
		elementUserData = sl7.s;
		found = true;
		goto cb_unlock;
	}
	it8 = mAlsaSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAlsaSource *>(
			elementUserData));
	if (it8 != mAlsaSourceListenersMap.end()) {
		sl8 = it8->second;
		elementUserData = sl8.s;
		found = true;
		goto cb_unlock;
	}
	it9 = mAudioSourceListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAudioSource *>(
			elementUserData));
	if (it9 != mAudioSourceListenersMap.end()) {
		sl9 = it9->second;
		elementUserData = sl9.s;
		found = true;
		goto cb_unlock;
	}
	it10 = mAudioEncoderListenersMap.find(
		reinterpret_cast<Pdraw::IPdraw::IAudioEncoder *>(
			elementUserData));
	if (it10 != mAudioEncoderListenersMap.end()) {
		sl10 = it10->second;
		elementUserData = sl10.e;
		found = true;
		goto cb_unlock;
	}
	if (elementUserData == mPendingRemovedElementUserdata.internal) {
		elementUserData = mPendingRemovedElementUserdata.external;
		found = true;
		goto cb_unlock;
	}

	if (!found) {
		ULOGW("%s: element userdata not found", __func__);
		elementUserData = nullptr;
	}

cb_unlock:
	pthread_mutex_unlock(&mMapsMutex);
cb:
	mListener->onMediaRemoved(this, info, elementUserData);
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
				     size_t count,
				     uint32_t selectedMedias)
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

	return dl.l->demuxerSelectMedia(
		this, dl.d, medias, count, selectedMedias);
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


void PdrawBackend::onMuxerConnectionStateChanged(
	IPdraw *pdraw,
	IPdraw::IMuxer *muxer,
	enum pdraw_muxer_connection_state connectionState,
	enum pdraw_muxer_disconnection_reason disconnectionReason)
{
	std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator it;
	struct muxerAndListener ml;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mMuxerListenersMap.find(muxer);
	if (it == mMuxerListenersMap.end()) {
		ml = mPendingMuxerAndListener;
	} else {
		ml = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (ml.l == nullptr) {
		ULOGE("%s: failed to find the muxer listener in the map",
		      __func__);
		return;
	}
	if (ml.m == nullptr) {
		ULOGE("%s: failed to find the muxer in the map", __func__);
		return;
	}

	ml.l->onMuxerConnectionStateChanged(
		this, ml.m, connectionState, disconnectionReason);
}


void PdrawBackend::onMuxerUnrecoverableError(IPdraw *pdraw,
					     IPdraw::IMuxer *muxer,
					     int status)
{
	std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator it;
	struct muxerAndListener ml;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mMuxerListenersMap.find(muxer);
	if (it == mMuxerListenersMap.end()) {
		ml = mPendingMuxerAndListener;
	} else {
		ml = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (ml.l == nullptr) {
		ULOGE("%s: failed to find the muxer listener in the map",
		      __func__);
		return;
	}
	if (ml.m == nullptr) {
		ULOGE("%s: failed to find the muxer in the map", __func__);
		return;
	}

	ml.l->onMuxerUnrecoverableError(this, ml.m, status);
}


void PdrawBackend::muxerCloseResponse(IPdraw *pdraw,
				      IPdraw::IMuxer *muxer,
				      int status)
{
	std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator it;
	struct muxerAndListener ml;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mMuxerListenersMap.find(muxer);
	if (it == mMuxerListenersMap.end()) {
		ml = mPendingMuxerAndListener;
	} else {
		ml = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (ml.l == nullptr) {
		ULOGE("%s: failed to find the muxer listener in the map",
		      __func__);
		return;
	}
	if (ml.m == nullptr) {
		ULOGE("%s: failed to find the muxer in the map", __func__);
		return;
	}

	ml.l->muxerCloseResponse(this, ml.m, status);
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
	const struct pdraw_media_info *info,
	bool restart)
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

	rl.l->onVideoRendererMediaRemoved(this, rl.r, info, restart);
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


void PdrawBackend::onAudioRendererMediaAdded(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IAudioRenderer *renderer,
	const struct pdraw_media_info *info)
{
	std::map<IPdraw::IAudioRenderer *,
		 struct audioRendererAndListener>::iterator it;
	struct audioRendererAndListener rl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioRendererListenersMap.find(renderer);
	if (it == mAudioRendererListenersMap.end()) {
		rl = mPendingAudioRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the audio renderer "
		      "listener in the map",
		      __func__);
		return;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the audio renderer in the map",
		      __func__);
		return;
	}

	rl.l->onAudioRendererMediaAdded(this, rl.r, info);
}


void PdrawBackend::onAudioRendererMediaRemoved(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IAudioRenderer *renderer,
	const struct pdraw_media_info *info)
{
	std::map<IPdraw::IAudioRenderer *,
		 struct audioRendererAndListener>::iterator it;
	struct audioRendererAndListener rl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioRendererListenersMap.find(renderer);
	if (it == mAudioRendererListenersMap.end()) {
		rl = mPendingAudioRendererAndListener;
	} else {
		rl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (rl.l == nullptr) {
		ULOGE("%s: failed to find the audio renderer "
		      "listener in the map",
		      __func__);
		return;
	}
	if (rl.r == nullptr) {
		ULOGE("%s: failed to find the audio renderer in the map",
		      __func__);
		return;
	}

	rl.l->onAudioRendererMediaRemoved(this, rl.r, info);
}


void PdrawBackend::vipcSourceReadyToPlay(
	IPdraw *pdraw,
	IPdraw::IVipcSource *source,
	bool ready,
	enum pdraw_vipc_source_eos_reason eosReason)
{
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it;
	struct vipcSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVipcSourceListenersMap.find(source);
	if (it == mVipcSourceListenersMap.end())
		sl = mPendingVipcSourceAndListener;
	else
		sl = it->second;

	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return;
	}

	sl.l->vipcSourceReadyToPlay(this, sl.s, ready, eosReason);
}


bool PdrawBackend::vipcSourceFramerateChanged(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IVipcSource *source,
	const struct vdef_frac *prevFramerate,
	const struct vdef_frac *newFramerate)
{
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it;
	struct vipcSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVipcSourceListenersMap.find(source);
	if (it == mVipcSourceListenersMap.end())
		sl = mPendingVipcSourceAndListener;
	else
		sl = it->second;

	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return false;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return false;
	}

	return sl.l->vipcSourceFramerateChanged(
		this, sl.s, prevFramerate, newFramerate);
}


void PdrawBackend::vipcSourceConfigured(IPdraw *pdraw,
					IPdraw::IVipcSource *source,
					int status,
					const struct vdef_format_info *info,
					const struct vdef_rectf *crop)
{
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it;
	struct vipcSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVipcSourceListenersMap.find(source);
	if (it == mVipcSourceListenersMap.end()) {
		sl = mPendingVipcSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return;
	}

	sl.l->vipcSourceConfigured(this, sl.s, status, info, crop);
}


void PdrawBackend::vipcSourceFrameReady(IPdraw *pdraw,
					IPdraw::IVipcSource *source,
					struct mbuf_raw_video_frame *frame)
{
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it;
	struct vipcSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVipcSourceListenersMap.find(source);
	if (it == mVipcSourceListenersMap.end()) {
		sl = mPendingVipcSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return;
	}

	sl.l->vipcSourceFrameReady(this, sl.s, frame);
}


bool PdrawBackend::vipcSourceEndOfStream(
	IPdraw *pdraw,
	IPdraw::IVipcSource *source,
	enum pdraw_vipc_source_eos_reason eosReason)
{
	std::map<IPdraw::IVipcSource *, struct vipcSourceAndListener>::iterator
		it;
	struct vipcSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mVipcSourceListenersMap.find(source);
	if (it == mVipcSourceListenersMap.end())
		sl = mPendingVipcSourceAndListener;
	else
		sl = it->second;

	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return false;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return false;
	}

	return sl.l->vipcSourceEndOfStream(this, sl.s, eosReason);
}


void PdrawBackend::onCodedVideoSourceFlushed(IPdraw *pdraw,
					     IPdraw::ICodedVideoSource *source)
{
	std::map<IPdraw::ICodedVideoSource *,
		 struct codedVideoSourceAndListener>::iterator it;
	struct codedVideoSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mCodedVideoSourceListenersMap.find(source);
	if (it == mCodedVideoSourceListenersMap.end()) {
		sl = mPendingCodedVideoSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return;
	}

	sl.l->onCodedVideoSourceFlushed(this, sl.s);
}


void PdrawBackend::onRawVideoSourceFlushed(IPdraw *pdraw,
					   IPdraw::IRawVideoSource *source)
{
	std::map<IPdraw::IRawVideoSource *,
		 struct rawVideoSourceAndListener>::iterator it;
	struct rawVideoSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mRawVideoSourceListenersMap.find(source);
	if (it == mRawVideoSourceListenersMap.end()) {
		sl = mPendingRawVideoSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video source in the map",
		      __func__);
		return;
	}

	sl.l->onRawVideoSourceFlushed(this, sl.s);
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


void PdrawBackend::onCodedVideoSinkSessionMetaUpdate(
	IPdraw *pdraw,
	IPdraw::ICodedVideoSink *sink,
	const struct vmeta_session *meta)
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

	sl.l->onCodedVideoSinkSessionMetaUpdate(this, sl.s, meta);
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


void PdrawBackend::onRawVideoSinkSessionMetaUpdate(
	IPdraw *pdraw,
	IPdraw::IRawVideoSink *sink,
	const struct vmeta_session *meta)
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

	sl.l->onRawVideoSinkSessionMetaUpdate(this, sl.s, meta);
}


void PdrawBackend::alsaSourceReadyToPlay(
	IPdraw *pdraw,
	IPdraw::IAlsaSource *source,
	bool ready,
	enum pdraw_alsa_source_eos_reason eosReason)
{
	std::map<IPdraw::IAlsaSource *, struct alsaSourceAndListener>::iterator
		it;
	struct alsaSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAlsaSourceListenersMap.find(source);
	if (it == mAlsaSourceListenersMap.end())
		sl = mPendingAlsaSourceAndListener;
	else
		sl = it->second;

	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the ALSA source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the ALSA source in the map",
		      __func__);
		return;
	}

	sl.l->alsaSourceReadyToPlay(this, sl.s, ready, eosReason);
}


void PdrawBackend::alsaSourceFrameReady(IPdraw *pdraw,
					IPdraw::IAlsaSource *source,
					struct mbuf_audio_frame *frame)
{
	std::map<IPdraw::IAlsaSource *, struct alsaSourceAndListener>::iterator
		it;
	struct alsaSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAlsaSourceListenersMap.find(source);
	if (it == mAlsaSourceListenersMap.end()) {
		sl = mPendingAlsaSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the ALSA source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the ALSA source in the map",
		      __func__);
		return;
	}

	sl.l->alsaSourceFrameReady(this, sl.s, frame);
}


void PdrawBackend::onAudioSourceFlushed(IPdraw *pdraw,
					IPdraw::IAudioSource *source)
{
	std::map<IPdraw::IAudioSource *,
		 struct audioSourceAndListener>::iterator it;
	struct audioSourceAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioSourceListenersMap.find(source);
	if (it == mAudioSourceListenersMap.end()) {
		sl = mPendingAudioSourceAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the audio source listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the audio source in the map",
		      __func__);
		return;
	}

	sl.l->onAudioSourceFlushed(this, sl.s);
}


void PdrawBackend::onAudioSinkFlush(IPdraw *pdraw, IPdraw::IAudioSink *sink)
{
	std::map<IPdraw::IAudioSink *, struct audioSinkAndListener>::iterator
		it;
	struct audioSinkAndListener sl;

	if (pthread_self() != mLoopThread)
		ULOGW("%s not called from the loop thread", __func__);

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioSinkListenersMap.find(sink);
	if (it == mAudioSinkListenersMap.end()) {
		sl = mPendingAudioSinkAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the audio sink listener in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the audio sink in the map", __func__);
		return;
	}

	sl.l->onAudioSinkFlush(this, sl.s);
}


void PdrawBackend::videoEncoderFrameOutput(IPdraw *pdraw,
					   IPdraw::IVideoEncoder *encoder,
					   struct mbuf_coded_video_frame *frame)
{
	std::map<IPdraw::IVideoEncoder *,
		 struct videoEncoderAndListener>::iterator it;
	struct videoEncoderAndListener el;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoEncoderListenersMap.find(encoder);
	if (it == mVideoEncoderListenersMap.end()) {
		el = mPendingVideoEncoderAndListener;
	} else {
		el = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (el.l == nullptr) {
		ULOGE("%s: failed to find the video encoder listener "
		      "in the map",
		      __func__);
		return;
	}
	if (el.e == nullptr) {
		ULOGE("%s: failed to find the video encoder in the map",
		      __func__);
		return;
	}

	el.l->videoEncoderFrameOutput(this, el.e, frame);
}


void PdrawBackend::videoEncoderFramePreRelease(
	IPdraw *pdraw,
	IPdraw::IVideoEncoder *encoder,
	struct mbuf_coded_video_frame *frame)
{
	std::map<IPdraw::IVideoEncoder *,
		 struct videoEncoderAndListener>::iterator it;
	struct videoEncoderAndListener el;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoEncoderListenersMap.find(encoder);
	if (it == mVideoEncoderListenersMap.end()) {
		el = mPendingVideoEncoderAndListener;
	} else {
		el = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (el.l == nullptr) {
		ULOGD("%s: failed to find the video encoder listener "
		      "in the map",
		      __func__);
		return;
	}
	if (el.e == nullptr) {
		ULOGE("%s: failed to find the video encoder in the map",
		      __func__);
		return;
	}

	el.l->videoEncoderFramePreRelease(this, el.e, frame);
}


void PdrawBackend::videoScalerFrameOutput(IPdraw *pdraw,
					  IPdraw::IVideoScaler *scaler,
					  struct mbuf_raw_video_frame *frame)
{
	std::map<IPdraw::IVideoScaler *,
		 struct videoScalerAndListener>::iterator it;
	struct videoScalerAndListener sl;

	pthread_mutex_lock(&mMapsMutex);
	it = mVideoScalerListenersMap.find(scaler);
	if (it == mVideoScalerListenersMap.end()) {
		sl = mPendingVideoScalerAndListener;
	} else {
		sl = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (sl.l == nullptr) {
		ULOGE("%s: failed to find the video scaler listener "
		      "in the map",
		      __func__);
		return;
	}
	if (sl.s == nullptr) {
		ULOGE("%s: failed to find the video scaler in the map",
		      __func__);
		return;
	}

	sl.l->videoScalerFrameOutput(this, sl.s, frame);
}


void PdrawBackend::audioEncoderFrameOutput(IPdraw *pdraw,
					   IPdraw::IAudioEncoder *encoder,
					   struct mbuf_audio_frame *frame)
{
	std::map<IPdraw::IAudioEncoder *,
		 struct audioEncoderAndListener>::iterator it;
	struct audioEncoderAndListener el;

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioEncoderListenersMap.find(encoder);
	if (it == mAudioEncoderListenersMap.end()) {
		el = mPendingAudioEncoderAndListener;
	} else {
		el = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (el.l == nullptr) {
		ULOGE("%s: failed to find the audio encoder listener "
		      "in the map",
		      __func__);
		return;
	}
	if (el.e == nullptr) {
		ULOGE("%s: failed to find the audio encoder in the map",
		      __func__);
		return;
	}

	el.l->audioEncoderFrameOutput(this, el.e, frame);
}


void PdrawBackend::audioEncoderFramePreRelease(IPdraw *pdraw,
					       IPdraw::IAudioEncoder *encoder,
					       struct mbuf_audio_frame *frame)
{
	std::map<IPdraw::IAudioEncoder *,
		 struct audioEncoderAndListener>::iterator it;
	struct audioEncoderAndListener el;

	pthread_mutex_lock(&mMapsMutex);
	it = mAudioEncoderListenersMap.find(encoder);
	if (it == mAudioEncoderListenersMap.end()) {
		el = mPendingAudioEncoderAndListener;
	} else {
		el = it->second;
	}
	pthread_mutex_unlock(&mMapsMutex);
	if (el.l == nullptr) {
		ULOGE("%s: failed to find the audio encoder listener "
		      "in the map",
		      __func__);
		return;
	}
	if (el.e == nullptr) {
		ULOGE("%s: failed to find the audio encoder in the map",
		      __func__);
		return;
	}

	el.l->audioEncoderFramePreRelease(this, el.e, frame);
}


void *PdrawBackend::loopThread(void *ptr)
{
	PdrawBackend *self = (PdrawBackend *)ptr;
	int res = 0, err;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	err = pthread_setname_np("pdraw_backend");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#	endif
#else
	err = pthread_setname_np(pthread_self(), "pdraw_backend");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#endif

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
		err = pomp_loop_wait_and_process(self->mLoop, -1);
		if (err < 0)
			ULOG_ERRNO("pomp_loop_wait_and_process", -err);
	}

	goto out;

error:
	self->mRetStatus = res;
	self->mRetValReady = true;
	pthread_mutex_unlock(&self->mMutex);
	pthread_cond_signal(&self->mCond);

out:
	pthread_mutex_lock(&self->mMutex);
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
	pthread_mutex_unlock(&self->mMutex);

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
				&msg->demuxer_create_single.params,
				msg->demuxer_create_single.listener);
			break;
		}
		case CMD_TYPE_DEMUXER_CREATE_URL: {
			std::string u(msg->demuxer_create_url.url);
			self->internalDemuxerCreate(
				u,
				&msg->demuxer_create_url.params,
				msg->demuxer_create_url.listener);
			break;
		}
		case CMD_TYPE_DEMUXER_CREATE_URL_MUX: {
			std::string u(msg->demuxer_create_url_mux.url);
			self->internalDemuxerCreate(
				u,
				msg->demuxer_create_url_mux.mux,
				&msg->demuxer_create_url_mux.params,
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
		case CMD_TYPE_DEMUXER_GET_MEDIA_LIST: {
			self->internalDemuxerGetMediaList(
				reinterpret_cast<PdrawBackend::Demuxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_DEMUXER_SELECT_MEDIA: {
			self->internalDemuxerSelectMedia(
				msg->demuxer_select_media.demuxer,
				msg->demuxer_select_media.selected_medias);
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
		case CMD_TYPE_DEMUXER_GET_CHAPTER_LIST: {
			self->internalDemuxerGetChapterList(
				msg->demuxer_get_chapter_list.demuxer,
				msg->demuxer_get_chapter_list.chapter_list,
				msg->demuxer_get_chapter_list.chapter_count);
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
			self->internalMuxerCreate(u,
						  &msg->muxer_create.params,
						  msg->muxer_create.listener);
			break;
		}
		case CMD_TYPE_MUXER_DESTROY: {
			self->internalMuxerDestroy(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_CLOSE: {
			self->internalMuxerClose(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_ADD_MEDIA: {
			self->internalMuxerAddMedia(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->muxer_add_media.muxer),
				msg->muxer_add_media.media_id,
				msg->muxer_add_media.params);
			break;
		}
		case CMD_TYPE_MUXER_SET_THUMBNAIL: {
			self->internalMuxerSetThumbnail(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->muxer_set_thumbnail.muxer),
				msg->muxer_set_thumbnail.type,
				msg->muxer_set_thumbnail.data,
				msg->muxer_set_thumbnail.size);
			break;
		}
		case CMD_TYPE_MUXER_ADD_CHAPTER: {
			self->internalMuxerAddChapter(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->muxer_add_chapter.muxer),
				msg->muxer_add_chapter.timestamp,
				msg->muxer_add_chapter.name);
			break;
		}
		case CMD_TYPE_MUXER_GET_STATS: {
			self->internalMuxerGetStats(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_SET_DYN_PARAMS: {
			self->internalMuxerSetDynParams(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->muxer_set_dyn_params.muxer),
				msg->muxer_set_dyn_params.dyn_params);
			break;
		}
		case CMD_TYPE_MUXER_GET_DYN_PARAMS: {
			self->internalMuxerGetDynParams(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_MUXER_FORCE_SYNC: {
			self->internalMuxerForceSync(
				reinterpret_cast<PdrawBackend::Muxer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_CREATE: {
			self->internalVipcSourceCreate(
				msg->vipc_source_create.listener);
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_DESTROY: {
			self->internalVipcSourceDestroy(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_IS_READY_TO_PLAY: {
			self->internalVipcSourceIsReadyToPlay(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_IS_PAUSED: {
			self->internalVipcSourceIsPaused(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_PLAY: {
			self->internalVipcSourcePlay(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_PAUSE: {
			self->internalVipcSourcePause(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_CONFIGURE: {
			self->internalVipcSourceConfigure(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->vipc_source_configure.source),
				&msg->vipc_source_configure.resolution,
				&msg->vipc_source_configure.crop);
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_INSERT_GREY_FRAME: {
			self->internalVipcSourceInsertGreyFrame(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr),
				msg->generic.uint64);
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_SET_SESSION_METADATA: {
			self->internalVipcSourceSetSessionMetadata(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIPC_SOURCE_GET_SESSION_METADATA: {
			self->internalVipcSourceGetSessionMetadata(
				reinterpret_cast<PdrawBackend::VipcSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_CREATE: {
			self->internalCodedVideoSourceCreate(
				msg->coded_video_source_create.listener);
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_DESTROY: {
			self->internalCodedVideoSourceDestroy(
				reinterpret_cast<PdrawBackend::CodedVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_GET_QUEUE: {
			self->internalCodedVideoSourceGetQueue(
				reinterpret_cast<PdrawBackend::CodedVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_FLUSH: {
			self->internalCodedVideoSourceFlush(
				reinterpret_cast<PdrawBackend::CodedVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_SET_SESSION_METADATA: {
			self->internalCodedVideoSourceSetSessionMetadata(
				reinterpret_cast<PdrawBackend::CodedVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_CODED_VIDEO_SOURCE_GET_SESSION_METADATA: {
			self->internalCodedVideoSourceGetSessionMetadata(
				reinterpret_cast<PdrawBackend::CodedVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_CREATE: {
			self->internalRawVideoSourceCreate(
				msg->raw_video_source_create.listener);
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_DESTROY: {
			self->internalRawVideoSourceDestroy(
				reinterpret_cast<PdrawBackend::RawVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_GET_QUEUE: {
			self->internalRawVideoSourceGetQueue(
				reinterpret_cast<PdrawBackend::RawVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_FLUSH: {
			self->internalRawVideoSourceFlush(
				reinterpret_cast<PdrawBackend::RawVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_SET_SESSION_METADATA: {
			self->internalRawVideoSourceSetSessionMetadata(
				reinterpret_cast<PdrawBackend::RawVideoSource
							 *>(msg->generic.ptr));
			break;
		}
		case CMD_TYPE_RAW_VIDEO_SOURCE_GET_SESSION_METADATA: {
			self->internalRawVideoSourceGetSessionMetadata(
				reinterpret_cast<PdrawBackend::RawVideoSource
							 *>(msg->generic.ptr));
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
		case CMD_TYPE_ALSA_SOURCE_CREATE: {
			self->internalAlsaSourceCreate(
				&msg->alsa_source_create.params,
				msg->alsa_source_create.listener);
			break;
		}
		case CMD_TYPE_ALSA_SOURCE_DESTROY: {
			self->internalAlsaSourceDestroy(
				reinterpret_cast<PdrawBackend::AlsaSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_ALSA_SOURCE_IS_READY_TO_PLAY: {
			self->internalAlsaSourceIsReadyToPlay(
				reinterpret_cast<PdrawBackend::AlsaSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_ALSA_SOURCE_IS_PAUSED: {
			self->internalAlsaSourceIsPaused(
				reinterpret_cast<PdrawBackend::AlsaSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_ALSA_SOURCE_PLAY: {
			self->internalAlsaSourcePlay(
				reinterpret_cast<PdrawBackend::AlsaSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_ALSA_SOURCE_PAUSE: {
			self->internalAlsaSourcePause(
				reinterpret_cast<PdrawBackend::AlsaSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SOURCE_CREATE: {
			self->internalAudioSourceCreate(
				&msg->audio_source_create.params,
				msg->audio_source_create.listener);
			break;
		}
		case CMD_TYPE_AUDIO_SOURCE_DESTROY: {
			self->internalAudioSourceDestroy(
				reinterpret_cast<PdrawBackend::AudioSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SOURCE_GET_QUEUE: {
			self->internalAudioSourceGetQueue(
				reinterpret_cast<PdrawBackend::AudioSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SOURCE_FLUSH: {
			self->internalAudioSourceFlush(
				reinterpret_cast<PdrawBackend::AudioSource *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SINK_CREATE: {
			self->internalAudioSinkCreate(
				msg->audio_sink_create.media_id,
				msg->audio_sink_create.listener);
			break;
		}
		case CMD_TYPE_AUDIO_SINK_DESTROY: {
			self->internalAudioSinkDestroy(
				reinterpret_cast<PdrawBackend::AudioSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SINK_GET_QUEUE: {
			self->internalAudioSinkGetQueue(
				reinterpret_cast<PdrawBackend::AudioSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_SINK_QUEUE_FLUSHED: {
			self->internalAudioSinkQueueFlushed(
				reinterpret_cast<PdrawBackend::AudioSink *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_CREATE: {
			self->internalAudioRendererCreate(
				msg->audio_renderer_create.media_id,
				&msg->audio_renderer_create.params,
				msg->audio_renderer_create.listener);
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_DESTROY: {
			self->internalAudioRendererDestroy(
				reinterpret_cast<PdrawBackend::AudioRenderer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_SET_MEDIA_ID: {
			self->internalAudioRendererSetMediaId(
				msg->audio_renderer_set_media_id.renderer,
				msg->audio_renderer_set_media_id.media_id);
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_GET_MEDIA_ID: {
			self->internalAudioRendererGetMediaId(
				reinterpret_cast<PdrawBackend::AudioRenderer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_SET_PARAMS: {
			self->internalAudioRendererSetParams(
				reinterpret_cast<PdrawBackend::AudioRenderer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_AUDIO_RENDERER_GET_PARAMS: {
			self->internalAudioRendererGetParams(
				reinterpret_cast<PdrawBackend::AudioRenderer *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIDEO_ENCODER_CREATE: {
			self->internalVideoEncoderCreate(
				msg->video_encoder_create.media_id,
				&msg->video_encoder_create.params,
				msg->video_encoder_create.listener);
			break;
		}
		case CMD_TYPE_VIDEO_ENCODER_DESTROY: {
			self->internalVideoEncoderDestroy(
				reinterpret_cast<PdrawBackend::VideoEncoder *>(
					msg->generic.ptr));
			break;
		}
		case CMD_TYPE_VIDEO_ENCODER_CONFIGURE: {
			self->internalVideoEncoderConfigure(
				reinterpret_cast<PdrawBackend::VideoEncoder *>(
					msg->video_encoder_configure.encoder),
				&msg->video_encoder_configure.config);
			break;
		}
		case CMD_TYPE_VIDEO_ENCODER_GET_CONFIG: {
			self->internalVideoEncoderGetConfig(
				reinterpret_cast<PdrawBackend::VideoEncoder *>(
					msg->video_encoder_get_config.encoder),
				&msg->video_encoder_get_config.config);
			break;
		}
		case CMD_TYPE_VIDEO_SCALER_CREATE: {
			self->internalVideoScalerCreate(
				msg->video_scaler_create.media_id,
				&msg->video_scaler_create.params,
				msg->video_scaler_create.listener);
			break;
		}
		case CMD_TYPE_VIDEO_SCALER_DESTROY: {
			self->internalVideoScalerDestroy(
				reinterpret_cast<PdrawBackend::VideoScaler *>(
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
				  const struct pdraw_demuxer_params *params,
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

	res = mPdraw->createDemuxer(url, params, this, &d);
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

	mPendingDemuxerAndListener = {};
	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateDemuxer(const std::string &localAddr,
				  uint16_t localStreamPort,
				  uint16_t localControlPort,
				  const std::string &remoteAddr,
				  uint16_t remoteStreamPort,
				  uint16_t remoteControlPort,
				  const struct pdraw_demuxer_params *params,
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
				    params,
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

	mPendingDemuxerAndListener = {};
	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateDemuxer(const std::string &url,
				  struct mux_ctx *mux,
				  const struct pdraw_demuxer_params *params,
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

	res = mPdraw->createDemuxer(url, mux, params, this, &d);
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

	mPendingDemuxerAndListener = {};
	*retObj = demuxer;
	return 0;
}


int PdrawBackend::doCreateMuxer(const std::string &url,
				const struct pdraw_muxer_params *params,
				IPdraw::IMuxer::Listener *listener,
				IPdraw::IMuxer **retObj)
{
	int res;
	PdrawBackend::Muxer *muxer = nullptr;
	IPdraw::IMuxer *m = nullptr;
	std::pair<std::map<IPdraw::IMuxer *, struct muxerAndListener>::iterator,
		  bool>
		inserted;

	muxer = new PdrawBackend::Muxer(this, url, listener);
	if (muxer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::Muxer", -res);
		return res;
	}

	mPendingMuxerAndListener = {
		.m = muxer,
		.l = listener,
	};

	res = mPdraw->createMuxer(url, params, this, &m);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createMuxer", -res);
		delete muxer;
		muxer = nullptr;
		return res;
	}
	muxer->setMuxer(m);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mMuxerListenersMap.insert(
		std::pair<IPdraw::IMuxer *, struct muxerAndListener>(
			muxer->getMuxer(), mPendingMuxerAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the muxer listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingMuxerAndListener = {};
	*retObj = muxer;
	return 0;
}


int PdrawBackend::doCreateVipcSource(
	const struct pdraw_vipc_source_params *params,
	IPdraw::IVipcSource::Listener *listener,
	IPdraw::IVipcSource **retObj)
{
	int res;
	PdrawBackend::VipcSource *source = nullptr;
	IPdraw::IVipcSource *s = nullptr;
	std::pair<std::map<IPdraw::IVipcSource *,
			   struct vipcSourceAndListener>::iterator,
		  bool>
		inserted;

	source = new PdrawBackend::VipcSource(this, listener);
	if (source == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::VipcSource", -res);
		return res;
	}

	mPendingVipcSourceAndListener = {
		.s = source,
		.l = listener,
	};

	res = mPdraw->createVipcSource(params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createVipcSource", -res);
		delete source;
		source = nullptr;
		return res;
	}
	source->setVipcSource(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVipcSourceListenersMap.insert(
		std::pair<IPdraw::IVipcSource *, struct vipcSourceAndListener>(
			source->getVipcSource(),
			mPendingVipcSourceAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the VIPC source listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingVipcSourceAndListener = {};
	*retObj = source;
	return 0;
}


int PdrawBackend::doCreateCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	int res;
	PdrawBackend::CodedVideoSource *source = nullptr;
	IPdraw::ICodedVideoSource *s = nullptr;
	std::pair<std::map<IPdraw::ICodedVideoSource *,
			   struct codedVideoSourceAndListener>::iterator,
		  bool>
		inserted;

	source = new PdrawBackend::CodedVideoSource(this, listener);
	if (source == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::CodedVideoSource", -res);
		return res;
	}

	mPendingCodedVideoSourceAndListener = {
		.s = source,
		.l = listener,
	};

	res = mPdraw->createCodedVideoSource(params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createCodedVideoSource", -res);
		delete source;
		source = nullptr;
		return res;
	}
	source->setCodedVideoSource(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mCodedVideoSourceListenersMap.insert(
		std::pair<IPdraw::ICodedVideoSource *,
			  struct codedVideoSourceAndListener>(
			source->getCodedVideoSource(),
			mPendingCodedVideoSourceAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video source listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingCodedVideoSourceAndListener = {};
	*retObj = source;
	return 0;
}


int PdrawBackend::doCreateRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	int res;
	PdrawBackend::RawVideoSource *source = nullptr;
	IPdraw::IRawVideoSource *s = nullptr;
	std::pair<std::map<IPdraw::IRawVideoSource *,
			   struct rawVideoSourceAndListener>::iterator,
		  bool>
		inserted;

	source = new PdrawBackend::RawVideoSource(this, listener);
	if (source == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::RawVideoSource", -res);
		return res;
	}

	mPendingRawVideoSourceAndListener = {
		.s = source,
		.l = listener,
	};

	res = mPdraw->createRawVideoSource(params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createRawVideoSource", -res);
		delete source;
		source = nullptr;
		return res;
	}
	source->setRawVideoSource(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mRawVideoSourceListenersMap.insert(
		std::pair<IPdraw::IRawVideoSource *,
			  struct rawVideoSourceAndListener>(
			source->getRawVideoSource(),
			mPendingRawVideoSourceAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video source listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingRawVideoSourceAndListener = {};
	*retObj = source;
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

	mPendingCodedVideoSinkAndListener = {};
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

	mPendingRawVideoSinkAndListener = {};
	*retObj = sink;
	return 0;
}


int PdrawBackend::doCreateAlsaSource(
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener,
	IPdraw::IAlsaSource **retObj)
{
	int res;
	PdrawBackend::AlsaSource *source = nullptr;
	IPdraw::IAlsaSource *s = nullptr;
	std::pair<std::map<IPdraw::IAlsaSource *,
			   struct alsaSourceAndListener>::iterator,
		  bool>
		inserted;

	source = new PdrawBackend::AlsaSource(this, listener);
	if (source == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::AlsaSource", -res);
		return res;
	}

	mPendingAlsaSourceAndListener = {
		.s = source,
		.l = listener,
	};

	res = mPdraw->createAlsaSource(params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createAlsaSource", -res);
		delete source;
		source = nullptr;
		return res;
	}
	source->setAlsaSource(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mAlsaSourceListenersMap.insert(
		std::pair<IPdraw::IAlsaSource *, struct alsaSourceAndListener>(
			source->getAlsaSource(),
			mPendingAlsaSourceAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the ALSA source listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingAlsaSourceAndListener = {};
	*retObj = source;
	return 0;
}


int PdrawBackend::doCreateAudioSource(
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener,
	IPdraw::IAudioSource **retObj)
{
	int res;
	PdrawBackend::AudioSource *source = nullptr;
	IPdraw::IAudioSource *s = nullptr;
	std::pair<std::map<IPdraw::IAudioSource *,
			   struct audioSourceAndListener>::iterator,
		  bool>
		inserted;

	source = new PdrawBackend::AudioSource(this, listener);
	if (source == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::AudioSource", -res);
		return res;
	}

	mPendingAudioSourceAndListener = {
		.s = source,
		.l = listener,
	};

	res = mPdraw->createAudioSource(params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("IPdraw::createAudioSource", -res);
		delete source;
		source = nullptr;
		return res;
	}
	source->setAudioSource(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mAudioSourceListenersMap.insert(
		std::pair<IPdraw::IAudioSource *,
			  struct audioSourceAndListener>(
			source->getAudioSource(),
			mPendingAudioSourceAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the audio source listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingAudioSourceAndListener = {};
	*retObj = source;
	return 0;
}


int PdrawBackend::doCreateAudioSink(unsigned int mediaId,
				    IPdraw::IAudioSink::Listener *listener,
				    IPdraw::IAudioSink **retObj)
{
	int res;
	PdrawBackend::AudioSink *sink = nullptr;
	IPdraw::IAudioSink *s = nullptr;
	std::pair<std::map<IPdraw::IAudioSink *,
			   struct audioSinkAndListener>::iterator,
		  bool>
		inserted;

	sink = new PdrawBackend::AudioSink(this, mediaId, listener);
	if (sink == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::AudioSink", -res);
		return res;
	}

	mPendingAudioSinkAndListener = {
		.s = sink,
		.l = listener,
	};

	res = mPdraw->createAudioSink(mediaId, this, &s);
	if (res < 0) {
		ULOG_ERRNO("IPdraw::createAudioSink", -res);
		delete sink;
		sink = nullptr;
		return res;
	}
	sink->setAudioSink(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mAudioSinkListenersMap.insert(
		std::pair<IPdraw::IAudioSink *, struct audioSinkAndListener>(
			sink->getAudioSink(), mPendingAudioSinkAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the audio sink listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingAudioSinkAndListener = {};
	*retObj = sink;
	return 0;
}


int PdrawBackend::doCreateAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	int res;
	PdrawBackend::AudioRenderer *renderer = nullptr;
	IPdraw::IAudioRenderer *r = nullptr;
	std::pair<std::map<IPdraw::IAudioRenderer *,
			   struct audioRendererAndListener>::iterator,
		  bool>
		inserted;

	renderer = new PdrawBackend::AudioRenderer(this, listener);
	if (renderer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::AudioRenderer", -res);
		return res;
	}

	mPendingAudioRendererAndListener = {
		.r = renderer,
		.l = listener,
	};

	res = mPdraw->createAudioRenderer(mediaId, params, this, &r);
	if (res < 0) {
		ULOG_ERRNO("IPdraw::createAudioRenderer", -res);
		delete renderer;
		renderer = nullptr;
		return res;
	}
	renderer->setAudioRenderer(r);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mAudioRendererListenersMap.insert(
		std::pair<IPdraw::IAudioRenderer *,
			  struct audioRendererAndListener>(
			renderer->getAudioRenderer(),
			mPendingAudioRendererAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the audio sink listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingAudioRendererAndListener = {};
	*retObj = renderer;
	return 0;
}


int PdrawBackend::doCreateVideoEncoder(
	unsigned int mediaId,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener,
	IPdraw::IVideoEncoder **retObj)
{
	int res;
	PdrawBackend::VideoEncoder *encoder = nullptr;
	IPdraw::IVideoEncoder *e = nullptr;
	std::pair<std::map<IPdraw::IVideoEncoder *,
			   struct videoEncoderAndListener>::iterator,
		  bool>
		inserted;

	encoder =
		new PdrawBackend::VideoEncoder(this, mediaId, params, listener);
	if (encoder == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::VideoEncoder", -res);
		return res;
	}

	mPendingVideoEncoderAndListener = {
		.e = encoder,
		.l = listener,
	};

	res = mPdraw->createVideoEncoder(mediaId, params, this, &e);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createVideoEncoder", -res);
		delete encoder;
		encoder = nullptr;
		return res;
	}
	encoder->setVideoEncoder(e);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVideoEncoderListenersMap.insert(
		std::pair<IPdraw::IVideoEncoder *,
			  struct videoEncoderAndListener>(
			encoder->getVideoEncoder(),
			mPendingVideoEncoderAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video encoder listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingVideoEncoderAndListener = {};
	*retObj = encoder;
	return 0;
}


int PdrawBackend::doCreateVideoScaler(unsigned int mediaId,
				      const struct vscale_config *params,
				      IPdraw::IVideoScaler::Listener *listener,
				      IPdraw::IVideoScaler **retObj)
{
	int res;
	PdrawBackend::VideoScaler *scaler = nullptr;
	IPdraw::IVideoScaler *s = nullptr;
	std::pair<std::map<IPdraw::IVideoScaler *,
			   struct videoScalerAndListener>::iterator,
		  bool>
		inserted;

	scaler = new PdrawBackend::VideoScaler(this, mediaId, params, listener);
	if (scaler == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::VideoScaler", -res);
		return res;
	}

	mPendingVideoScalerAndListener = {
		.s = scaler,
		.l = listener,
	};

	res = mPdraw->createVideoScaler(mediaId, params, this, &s);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createVideoScaler", -res);
		delete scaler;
		scaler = nullptr;
		return res;
	}
	scaler->setVideoScaler(s);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mVideoScalerListenersMap.insert(
		std::pair<IPdraw::IVideoScaler *,
			  struct videoScalerAndListener>(
			scaler->getVideoScaler(),
			mPendingVideoScalerAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the video scaler listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingVideoScalerAndListener = {};
	*retObj = scaler;
	return 0;
}


int PdrawBackend::doCreateAudioEncoder(
	unsigned int mediaId,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener,
	IPdraw::IAudioEncoder **retObj)
{
	int res;
	PdrawBackend::AudioEncoder *encoder = nullptr;
	IPdraw::IAudioEncoder *e = nullptr;
	std::pair<std::map<IPdraw::IAudioEncoder *,
			   struct audioEncoderAndListener>::iterator,
		  bool>
		inserted;

	encoder =
		new PdrawBackend::AudioEncoder(this, mediaId, params, listener);
	if (encoder == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("PdrawBackend::AudioEncoder", -res);
		return res;
	}

	mPendingAudioEncoderAndListener = {
		.e = encoder,
		.l = listener,
	};

	res = mPdraw->createAudioEncoder(mediaId, params, listener, &e);
	if (res < 0) {
		ULOG_ERRNO("pdraw->createAudioEncoder", -res);
		delete encoder;
		encoder = nullptr;
		return res;
	}
	encoder->setAudioEncoder(e);

	pthread_mutex_lock(&mMapsMutex);
	inserted = mAudioEncoderListenersMap.insert(
		std::pair<IPdraw::IAudioEncoder *,
			  struct audioEncoderAndListener>(
			encoder->getAudioEncoder(),
			mPendingAudioEncoderAndListener));
	if (inserted.second == false)
		ULOGW("failed to insert the audio encoder listener in the map");
	pthread_mutex_unlock(&mMapsMutex);

	mPendingAudioEncoderAndListener = {};
	*retObj = encoder;
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


void PdrawBackend::internalDemuxerCreate(
	const std::string &url,
	const struct pdraw_demuxer_params *params,
	IPdraw::IDemuxer::Listener *listener)
{
	int res;
	IPdraw::IDemuxer *demuxer = nullptr;

	res = doCreateDemuxer(url, params, listener, &demuxer);

	mPendingDemuxerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetDemuxer = demuxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerCreate(
	const std::string &localAddr,
	uint16_t localStreamPort,
	uint16_t localControlPort,
	const std::string &remoteAddr,
	uint16_t remoteStreamPort,
	uint16_t remoteControlPort,
	const struct pdraw_demuxer_params *params,
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
			      params,
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


void PdrawBackend::internalDemuxerCreate(
	const std::string &url,
	struct mux_ctx *mux,
	const struct pdraw_demuxer_params *params,
	IPdraw::IDemuxer::Listener *listener)
{
	int res;
	IPdraw::IDemuxer *demuxer = nullptr;

	res = doCreateDemuxer(url, mux, params, listener, &demuxer);

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
	auto it = mDemuxerListenersMap.find(demuxer->getDemuxer());
	if (it != mDemuxerListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = demuxer->getDemuxer(),
			.external = it->second.d,
		};
		erased = mDemuxerListenersMap.erase(demuxer->getDemuxer());
		if (erased != 1)
			ULOGW("failed to erase the demuxer listener "
			      "from the map");
	} else {
		ULOGE("failed to find the demuxer in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete demuxer->getDemuxer();
	demuxer->setDemuxer(nullptr);

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


void PdrawBackend::internalDemuxerGetChapterList(
	PdrawBackend::Demuxer *demuxer,
	struct pdraw_chapter **chapterList,
	size_t *chapterCount)
{
	int res = demuxer->getDemuxer()->getChapterList(chapterList,
							chapterCount);

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


void PdrawBackend::internalDemuxerGetMediaList(PdrawBackend::Demuxer *demuxer)
{
	struct pdraw_demuxer_media *media_list = nullptr;
	size_t media_count = 0;
	uint32_t selected_medias = 0;

	int res = demuxer->getDemuxer()->getMediaList(
		&media_list, &media_count, &selected_medias);

	pthread_mutex_lock(&mMutex);
	mRetMediaList = media_list;
	mRetSizeT = media_count;
	mRetUint32 = selected_medias;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalDemuxerSelectMedia(PdrawBackend::Demuxer *demuxer,
					      uint32_t selectedMedias)
{
	int res = demuxer->getDemuxer()->selectMedia(selectedMedias);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerCreate(const std::string &url,
				       const struct pdraw_muxer_params *params,
				       IPdraw::IMuxer::Listener *listener)
{
	int res;
	IPdraw::IMuxer *muxer = nullptr;

	res = doCreateMuxer(url, params, listener, &muxer);

	mPendingMuxerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetMuxer = muxer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerDestroy(PdrawBackend::Muxer *muxer)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mMuxerListenersMap.find(muxer->getMuxer());
	if (it != mMuxerListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = muxer->getMuxer(),
			.external = it->second.m,
		};
		erased = mMuxerListenersMap.erase(muxer->getMuxer());
		if (erased != 1)
			ULOGW("failed to erase the muxer listener "
			      "from the map");
	} else {
		ULOGE("failed to find the muxer in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete muxer->getMuxer();
	muxer->setMuxer(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerClose(PdrawBackend::Muxer *muxer)
{
	int res = muxer->getMuxer()->close();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerAddMedia(
	PdrawBackend::Muxer *muxer,
	unsigned int mediaId,
	const struct pdraw_muxer_media_params *params)
{
	int res = muxer->getMuxer()->addMedia(mediaId, params);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerSetThumbnail(
	PdrawBackend::Muxer *muxer,
	enum pdraw_muxer_thumbnail_type type,
	const uint8_t *data,
	size_t size)
{
	int res = muxer->getMuxer()->setThumbnail(type, data, size);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerAddChapter(PdrawBackend::Muxer *muxer,
					   uint64_t timestamp,
					   const char *name)
{
	int res = muxer->getMuxer()->addChapter(timestamp, name);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerGetStats(PdrawBackend::Muxer *muxer)
{
	struct pdraw_muxer_stats stats;
	int res = muxer->getMuxer()->getStats(&stats);

	pthread_mutex_lock(&mMutex);
	mRetMuxerStats = stats;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerSetDynParams(
	PdrawBackend::Muxer *muxer,
	struct pdraw_muxer_dyn_params dyn_params)
{
	int res = muxer->getMuxer()->setDynParams(&dyn_params);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerGetDynParams(PdrawBackend::Muxer *muxer)
{

	struct pdraw_muxer_dyn_params dyn_params;
	int res = muxer->getMuxer()->getDynParams(&dyn_params);

	pthread_mutex_lock(&mMutex);
	mRetMuxerDynParams = dyn_params;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalMuxerForceSync(PdrawBackend::Muxer *muxer)
{
	int res = muxer->getMuxer()->forceSync();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceCreate(
	IPdraw::IVipcSource::Listener *listener)
{
	int res;
	IPdraw::IVipcSource *source = nullptr;
	struct pdraw_vipc_source_params params;

	pthread_mutex_lock(&mMutex);
	params = mParamVipcSource;
	pthread_mutex_unlock(&mMutex);

	res = doCreateVipcSource(&params, listener, &source);

	mPendingVipcSourceAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetVipcSource = source;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceDestroy(PdrawBackend::VipcSource *source)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mVipcSourceListenersMap.find(source->getVipcSource());
	if (it != mVipcSourceListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = source->getVipcSource(),
			.external = it->second.s,
		};
		erased = mVipcSourceListenersMap.erase(source->getVipcSource());
		if (erased != 1)
			ULOGW("failed to erase the VIPC source listener "
			      "from the map");
	} else {
		ULOGE("failed to find the VIPC source in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete source->getVipcSource();
	source->setVipcSource(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceIsReadyToPlay(
	PdrawBackend::VipcSource *source)
{
	bool res = source->getVipcSource()->isReadyToPlay();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceIsPaused(PdrawBackend::VipcSource *source)
{
	bool res = source->getVipcSource()->isPaused();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourcePlay(PdrawBackend::VipcSource *source)
{
	int res = source->getVipcSource()->play();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourcePause(PdrawBackend::VipcSource *source)
{
	int res = source->getVipcSource()->pause();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceConfigure(
	PdrawBackend::VipcSource *source,
	const struct vdef_dim *resolution,
	const struct vdef_rectf *crop)
{
	int res = source->getVipcSource()->configure(resolution, crop);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceInsertGreyFrame(
	PdrawBackend::VipcSource *source,
	uint64_t tsUs)
{
	int res = source->getVipcSource()->insertGreyFrame(tsUs);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceSetSessionMetadata(
	PdrawBackend::VipcSource *source)
{
	struct vmeta_session meta;

	pthread_mutex_lock(&mMutex);
	meta = mParamVmetaSession;
	pthread_mutex_unlock(&mMutex);

	int res = source->getVipcSource()->setSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVipcSourceGetSessionMetadata(
	PdrawBackend::VipcSource *source)
{
	struct vmeta_session meta;

	int res = source->getVipcSource()->getSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mParamVmetaSession = meta;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceCreate(
	IPdraw::ICodedVideoSource::Listener *listener)
{
	int res;
	IPdraw::ICodedVideoSource *source = nullptr;
	struct pdraw_video_source_params params;

	pthread_mutex_lock(&mMutex);
	params = mParamVideoSource;
	pthread_mutex_unlock(&mMutex);

	res = doCreateCodedVideoSource(&params, listener, &source);

	mPendingCodedVideoSourceAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetCodedVideoSource = source;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceDestroy(
	PdrawBackend::CodedVideoSource *source)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mCodedVideoSourceListenersMap.find(
		source->getCodedVideoSource());
	if (it != mCodedVideoSourceListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = source->getCodedVideoSource(),
			.external = it->second.s,
		};
		erased = mCodedVideoSourceListenersMap.erase(
			source->getCodedVideoSource());
		if (erased != 1)
			ULOGW("failed to erase the video source listener "
			      "from the map");
	} else {
		ULOGE("failed to find the video source in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete source->getCodedVideoSource();
	source->setCodedVideoSource(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceGetQueue(
	PdrawBackend::CodedVideoSource *source)
{
	struct mbuf_coded_video_frame_queue *res =
		source->getCodedVideoSource()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetCodedQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceFlush(
	PdrawBackend::CodedVideoSource *source)
{
	int res = source->getCodedVideoSource()->flush();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceSetSessionMetadata(
	PdrawBackend::CodedVideoSource *source)
{
	struct vmeta_session meta;

	pthread_mutex_lock(&mMutex);
	meta = mParamVmetaSession;
	pthread_mutex_unlock(&mMutex);

	int res = source->getCodedVideoSource()->setSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalCodedVideoSourceGetSessionMetadata(
	PdrawBackend::CodedVideoSource *source)
{
	struct vmeta_session meta;

	int res = source->getCodedVideoSource()->getSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mParamVmetaSession = meta;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceCreate(
	IPdraw::IRawVideoSource::Listener *listener)
{
	int res;
	IPdraw::IRawVideoSource *source = nullptr;
	struct pdraw_video_source_params params;

	pthread_mutex_lock(&mMutex);
	params = mParamVideoSource;
	pthread_mutex_unlock(&mMutex);

	res = doCreateRawVideoSource(&params, listener, &source);

	mPendingRawVideoSourceAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetRawVideoSource = source;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceDestroy(
	PdrawBackend::RawVideoSource *source)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mRawVideoSourceListenersMap.find(source->getRawVideoSource());
	if (it != mRawVideoSourceListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = source->getRawVideoSource(),
			.external = it->second.s,
		};
		erased = mRawVideoSourceListenersMap.erase(
			source->getRawVideoSource());
		if (erased != 1)
			ULOGW("failed to erase the video source listener "
			      "from the map");
	} else {
		ULOGE("failed to find the video source in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete source->getRawVideoSource();
	source->setRawVideoSource(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceGetQueue(
	PdrawBackend::RawVideoSource *source)
{
	struct mbuf_raw_video_frame_queue *res =
		source->getRawVideoSource()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetRawQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceFlush(
	PdrawBackend::RawVideoSource *source)
{
	int res = source->getRawVideoSource()->flush();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceSetSessionMetadata(
	PdrawBackend::RawVideoSource *source)
{
	struct vmeta_session meta;

	pthread_mutex_lock(&mMutex);
	meta = mParamVmetaSession;
	pthread_mutex_unlock(&mMutex);

	int res = source->getRawVideoSource()->setSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalRawVideoSourceGetSessionMetadata(
	PdrawBackend::RawVideoSource *source)
{
	struct vmeta_session meta;

	int res = source->getRawVideoSource()->getSessionMetadata(&meta);

	pthread_mutex_lock(&mMutex);
	mParamVmetaSession = meta;
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
	sink->setCodedVideoSink(nullptr);

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
	sink->setRawVideoSink(nullptr);

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


void PdrawBackend::internalAlsaSourceCreate(
	const struct pdraw_alsa_source_params *params,
	IPdraw::IAlsaSource::Listener *listener)
{
	int res;
	IPdraw::IAlsaSource *source = nullptr;

	res = doCreateAlsaSource(params, listener, &source);

	mPendingAlsaSourceAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetAlsaSource = source;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAlsaSourceDestroy(PdrawBackend::AlsaSource *source)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mAlsaSourceListenersMap.find(source->getAlsaSource());
	if (it != mAlsaSourceListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = source->getAlsaSource(),
			.external = it->second.s,
		};
		erased = mAlsaSourceListenersMap.erase(source->getAlsaSource());
		if (erased != 1)
			ULOGW("failed to erase the ALSA source listener "
			      "from the map");
	} else {
		ULOGE("failed to find the ALSA source in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete source->getAlsaSource();
	source->setAlsaSource(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAlsaSourceIsReadyToPlay(
	PdrawBackend::AlsaSource *source)
{
	bool res = source->getAlsaSource()->isReadyToPlay();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAlsaSourceIsPaused(PdrawBackend::AlsaSource *source)
{
	bool res = source->getAlsaSource()->isPaused();

	pthread_mutex_lock(&mMutex);
	mRetBool = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAlsaSourcePlay(PdrawBackend::AlsaSource *source)
{
	int res = source->getAlsaSource()->play();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAlsaSourcePause(PdrawBackend::AlsaSource *source)
{
	int res = source->getAlsaSource()->pause();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSourceCreate(
	const struct pdraw_audio_source_params *params,
	IPdraw::IAudioSource::Listener *listener)
{
	int res;
	IPdraw::IAudioSource *source = nullptr;

	res = doCreateAudioSource(params, listener, &source);

	mPendingAudioSourceAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetAudioSource = source;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSourceDestroy(PdrawBackend::AudioSource *source)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mAudioSourceListenersMap.find(source->getAudioSource());
	if (it != mAudioSourceListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = source->getAudioSource(),
			.external = it->second.s,
		};
		erased = mAudioSourceListenersMap.erase(
			source->getAudioSource());
		if (erased != 1)
			ULOGW("failed to erase the audio source listener "
			      "from the map");
	} else {
		ULOGE("failed to find the audio source in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete source->getAudioSource();
	source->setAudioSource(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSourceGetQueue(
	PdrawBackend::AudioSource *source)
{
	struct mbuf_audio_frame_queue *res =
		source->getAudioSource()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetAudioQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSourceFlush(PdrawBackend::AudioSource *source)
{
	int res = source->getAudioSource()->flush();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSinkCreate(
	unsigned int mediaId,
	IPdraw::IAudioSink::Listener *listener)
{
	int res;
	IPdraw::IAudioSink *sink = nullptr;

	res = doCreateAudioSink(mediaId, listener, &sink);

	mPendingAudioSinkAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetAudioSink = sink;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSinkDestroy(PdrawBackend::AudioSink *sink)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	erased = mAudioSinkListenersMap.erase(sink->getAudioSink());
	if (erased != 1)
		ULOGW("failed to erase the audio sink listener from the map");
	pthread_mutex_unlock(&mMapsMutex);

	delete sink->getAudioSink();
	sink->setAudioSink(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSinkGetQueue(PdrawBackend::AudioSink *sink)
{
	struct mbuf_audio_frame_queue *res = sink->getAudioSink()->getQueue();

	pthread_mutex_lock(&mMutex);
	mRetAudioQueue = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioSinkQueueFlushed(PdrawBackend::AudioSink *sink)
{
	int res = sink->getAudioSink()->queueFlushed();

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererCreate(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener)
{
	int res;
	IPdraw::IAudioRenderer *renderer = nullptr;

	res = doCreateAudioRenderer(mediaId, params, listener, &renderer);

	mPendingAudioRendererAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetAudioRenderer = renderer;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererDestroy(
	PdrawBackend::AudioRenderer *renderer)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	erased = mAudioRendererListenersMap.erase(renderer->getAudioRenderer());
	if (erased != 1) {
		ULOGW("failed to erase the audio renderer "
		      "listener from the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete renderer->getAudioRenderer();
	renderer->setAudioRenderer(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererSetMediaId(
	PdrawBackend::AudioRenderer *renderer,
	unsigned int mediaId)
{
	int res = renderer->getAudioRenderer()->setMediaId(mediaId);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererGetMediaId(
	PdrawBackend::AudioRenderer *renderer)
{
	unsigned int res = renderer->getAudioRenderer()->getMediaId();

	pthread_mutex_lock(&mMutex);
	mRetUint = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererSetParams(
	PdrawBackend::AudioRenderer *renderer)
{
	struct pdraw_audio_renderer_params params;

	pthread_mutex_lock(&mMutex);
	params = mParamAudioRenderer;
	pthread_mutex_unlock(&mMutex);

	int res = renderer->getAudioRenderer()->setParams(&params);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioRendererGetParams(
	PdrawBackend::AudioRenderer *renderer)
{
	struct pdraw_audio_renderer_params params;

	int res = renderer->getAudioRenderer()->getParams(&params);

	pthread_mutex_lock(&mMutex);
	mParamAudioRenderer = params;
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoEncoderCreate(
	unsigned int mediaId,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener)
{
	int res;
	IPdraw::IVideoEncoder *encoder = nullptr;

	res = doCreateVideoEncoder(mediaId, params, listener, &encoder);

	mPendingVideoEncoderAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetVideoEncoder = encoder;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoEncoderDestroy(
	PdrawBackend::VideoEncoder *encoder)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mVideoEncoderListenersMap.find(encoder->getVideoEncoder());
	if (it != mVideoEncoderListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = encoder->getVideoEncoder(),
			.external = it->second.e,
		};
		erased = mVideoEncoderListenersMap.erase(
			encoder->getVideoEncoder());
		if (erased != 1)
			ULOGW("failed to erase the video encoder listener "
			      "from the map");
	} else {
		ULOGE("failed to find the video encoder in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete encoder->getVideoEncoder();
	encoder->setVideoEncoder(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoEncoderConfigure(
	PdrawBackend::VideoEncoder *encoder,
	const struct venc_dyn_config *config)
{
	int res = encoder->getVideoEncoder()->configure(config);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoEncoderGetConfig(
	PdrawBackend::VideoEncoder *encoder,
	struct venc_dyn_config *config)
{
	int res = encoder->getVideoEncoder()->getConfig(config);

	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetValReady = true;
	mRetConfig = *config;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoScalerCreate(
	unsigned int mediaId,
	const struct vscale_config *params,
	IPdraw::IVideoScaler::Listener *listener)
{
	int res;
	IPdraw::IVideoScaler *scaler = nullptr;

	res = doCreateVideoScaler(mediaId, params, listener, &scaler);

	mPendingVideoScalerAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetVideoScaler = scaler;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalVideoScalerDestroy(PdrawBackend::VideoScaler *scaler)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mVideoScalerListenersMap.find(scaler->getVideoScaler());
	if (it != mVideoScalerListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = scaler->getVideoScaler(),
			.external = it->second.s,
		};
		erased = mVideoScalerListenersMap.erase(
			scaler->getVideoScaler());
		if (erased != 1)
			ULOGW("failed to erase the video scaler listener "
			      "from the map");
	} else {
		ULOGE("failed to find the video scaler in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete scaler->getVideoScaler();
	scaler->setVideoScaler(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioEncoderCreate(
	unsigned int mediaId,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener)
{
	int res;
	IPdraw::IAudioEncoder *encoder = nullptr;

	res = doCreateAudioEncoder(mediaId, params, listener, &encoder);

	mPendingAudioEncoderAndListener = {};
	pthread_mutex_lock(&mMutex);
	mRetStatus = res;
	mRetAudioEncoder = encoder;
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalAudioEncoderDestroy(
	PdrawBackend::AudioEncoder *encoder)
{
	size_t erased;

	pthread_mutex_lock(&mMapsMutex);
	auto it = mAudioEncoderListenersMap.find(encoder->getAudioEncoder());
	if (it != mAudioEncoderListenersMap.end()) {
		mPendingRemovedElementUserdata = {
			.internal = encoder->getAudioEncoder(),
			.external = it->second.e,
		};
		erased = mAudioEncoderListenersMap.erase(
			encoder->getAudioEncoder());
		if (erased != 1)
			ULOGW("failed to erase the audio encoder listener "
			      "from the map");
	} else {
		ULOGE("failed to find the audio encoder in the map");
	}
	pthread_mutex_unlock(&mMapsMutex);

	delete encoder->getAudioEncoder();
	encoder->setAudioEncoder(nullptr);

	pthread_mutex_lock(&mMutex);
	mRetValReady = true;
	pthread_mutex_unlock(&mMutex);
	pthread_cond_signal(&mCond);
}


void PdrawBackend::internalGetFriendlyNameSetting(void)
{
	std::string friendlyName;
	mPdraw->getFriendlyNameSetting(&friendlyName);

	pthread_mutex_lock(&mMutex);
	mRetString = std::move(friendlyName);
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
	mRetString = std::move(serialNumber);
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
	mRetString = std::move(softwareVersion);
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
