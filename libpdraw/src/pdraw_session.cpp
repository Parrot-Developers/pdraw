/**
 * Parrot Drones Awesome Video Viewer Library
 * Session
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_utils.hpp"
#include "pdraw_log.hpp"
#include <math.h>
#include <string.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
ULOG_DECLARE_TAG(libpdraw);
#include <algorithm>
#include <string>
#include <vector>

namespace Pdraw {


enum cmd_type {
	CMD_TYPE_OPEN_SINGLE,
	CMD_TYPE_OPEN_URL,
	CMD_TYPE_OPEN_URL_MUX,
	CMD_TYPE_OPEN_SDP,
	CMD_TYPE_OPEN_SDP_MUX,
	CMD_TYPE_CLOSE,
	CMD_TYPE_PLAY,
	CMD_TYPE_PREVIOUS_FRAME,
	CMD_TYPE_NEXT_FRAME,
	CMD_TYPE_SEEK,
	CMD_TYPE_SEEK_TO,
};


struct cmd_base {
	enum cmd_type type;
};


struct cmd_open_single {
	struct cmd_base base;
	char local_addr[16];
	int local_stream_port;
	int local_control_port;
	char remote_addr[16];
	int remote_stream_port;
	int remote_control_port;
	char iface_addr[16];
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_open_single) <= PIPE_BUF - 1);


struct cmd_open_url {
	struct cmd_base base;
	char url[200];
	char iface_addr[16];
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_open_url) <= PIPE_BUF - 1);


struct cmd_open_url_mux {
	struct cmd_base base;
	char url[200];
	struct mux_ctx *mux;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_open_url_mux) <= PIPE_BUF - 1);


struct cmd_open_sdp {
	struct cmd_base base;
	char sdp[PIPE_BUF - 1 -
		sizeof(struct cmd_base) -
		16 * sizeof(char) - 8]; /* 8 is a margin for alignment */
	char iface_addr[16];
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_open_sdp) <= PIPE_BUF - 1);


struct cmd_open_sdp_mux {
	struct cmd_base base;
	char sdp[PIPE_BUF - 1 -
		sizeof(struct cmd_base) -
		sizeof(struct mux_ctx *) - 8]; /* 8 is a margin for alignment */
	struct mux_ctx *mux;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_open_sdp_mux) <= PIPE_BUF - 1);


struct cmd_play {
	struct cmd_base base;
	float speed;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_play) <= PIPE_BUF - 1);


struct cmd_seek {
	struct cmd_base base;
	int64_t delta;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_seek) <= PIPE_BUF - 1);


struct cmd_seek_to {
	struct cmd_base base;
	uint64_t timestamp;
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_seek_to) <= PIPE_BUF - 1);


int createPdraw(
	struct pomp_loop *loop,
	IPdraw::Listener *listener,
	IPdraw **ret_obj)
{
	IPdraw *pdraw = NULL;
	if (ret_obj == NULL) {
		ULOGE("invalid pointer");
		return -EINVAL;
	}
	pdraw = new Session(loop, listener);
	if (pdraw == NULL) {
		ULOGE("failed to create pdraw instance");
		return -EPROTO;
	}
	*ret_obj = pdraw;
	return 0;
}


const char *pdrawStateStr(
	enum IPdraw::State val)
{
	switch (val) {
	case IPdraw::State::INVALID:
		return "INVALID";
	case IPdraw::State::CREATED:
		return "CREATED";
	case IPdraw::State::OPENED:
		return "OPENED";
	case IPdraw::State::CLOSED:
		return "CLOSED";
	default: return NULL;
	}
}


Session::Session(
	struct pomp_loop *loop,
	Listener *listener)
{
	int res;
	pthread_mutexattr_t attr;
	bool mutex_created = false, attr_created = false;

	mListener = listener;
	mState = INVALID;
	mSessionType = PDRAW_SESSION_TYPE_UNKNOWN;
	mDemuxer = NULL;
	mRenderer = NULL;
	mMediaIdCounter = 0;
	mInternalLoop = false;
	mLoop = loop;
	mThreadShouldStop = false;
	mLoopThreadLaunched = false;

	res = pthread_mutexattr_init(&attr);
	if (res < 0) {
		ULOG_ERRNO("Session: pthread_mutexattr_init", -res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res < 0) {
		ULOG_ERRNO("Session: pthread_mutexattr_settype", -res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, NULL);
	if (res < 0) {
		ULOG_ERRNO("Session: pthread_mutex_init", -res);
		goto error;
	}
	mutex_created = true;

	if (mLoop == NULL) {
		mInternalLoop = true;
		mLoop = pomp_loop_new();
		if (mLoop == NULL) {
			ULOGE("Session: failed to create pomp loop");
			res = -ENOMEM;
			goto error;
		}

		mMbox = mbox_new(PIPE_BUF - 1);
		if (mMbox == NULL) {
			ULOGE("Session: failed to create mailbox");
			res = -ENOMEM;
			goto error;
		}

		res = pomp_loop_add(mLoop, mbox_get_read_fd(mMbox),
			POMP_FD_EVENT_IN, &mboxCb, this);
		if (res < 0) {
			ULOG_ERRNO("Session: pomp_loop_add", -res);
			goto error;
		}

		res = pthread_create(&mLoopThread, NULL,
			runLoopThread, (void *)this);
		if (res < 0) {
			ULOG_ERRNO("Session: pthread_create", -res);
			goto error;
		}

		mLoopThreadLaunched = true;
	}

	pthread_mutexattr_destroy(&attr);
	setState(CREATED);
	return;

error:
	if (mInternalLoop) {
		if (mMbox != NULL) {
			if (mLoop != NULL) {
				res = pomp_loop_remove(mLoop,
					mbox_get_read_fd(mMbox));
				if (res < 0) {
					ULOG_ERRNO("Session: pomp_loop_remove",
						-res);
				}
			}
			mbox_destroy(mMbox);
			mMbox = NULL;
		}
		if (mLoop != NULL) {
			pomp_loop_destroy(mLoop);
			mLoop = NULL;
		}
		mInternalLoop = false;
	}
	if (mutex_created)
		pthread_mutex_destroy(&mMutex);
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


Session::~Session(
	void)
{
	int res;

	if (mDemuxer != NULL) {
		int ret = mDemuxer->close();
		if (ret != 0)
			ULOGE("Session: failed to close demuxer");
		else
			delete mDemuxer;
	}

	if (mRenderer != NULL)
		delete mRenderer;

	std::vector<Media*>::iterator m = mMedias.begin();
	while (m != mMedias.end()) {
		delete *m;
		m++;
	}

	if (mInternalLoop) {
		if (mMbox != NULL) {
			res = pomp_loop_remove(mLoop, mbox_get_read_fd(mMbox));
			if (res < 0)
				ULOG_ERRNO("Session: pomp_loop_remove", -res);
			mbox_destroy(mMbox);
			mMbox = NULL;
		}

		mThreadShouldStop = true;
		if (mLoop) {
			res = pomp_loop_wakeup(mLoop);
			if (res < 0)
				ULOG_ERRNO("Session: pomp_loop_wakeup", -res);
		}

		if (mLoopThreadLaunched) {
			res = pthread_join(mLoopThread, NULL);
			if (res < 0)
				ULOG_ERRNO("Session: pthread_join", -res);
		}

		if (mLoop) {
			res = pomp_loop_destroy(mLoop);
			if (res != 0)
				ULOG_ERRNO("Session: pomp_loop_destroy", -res);
		}
	}

	pthread_mutex_destroy(&mMutex);
}


/*
 * API methods
 */

IPdraw::State Session::getState(
	void)
{
	pthread_mutex_lock(&mMutex);
	IPdraw::State ret = mState;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::open(
	const std::string &url)
{
	return open(url, "");
}


int Session::open(
	const std::string &url,
	const std::string &ifaceAddr)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_open_url *cmd = NULL;
		PDRAW_RETURN_ERR_IF_FAILED(url.length() <
			sizeof(cmd->url) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(ifaceAddr.length() <
			sizeof(cmd->iface_addr) - 1, -ENOBUFS);
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_open_url *)msg;
		cmd->base.type = CMD_TYPE_OPEN_URL;
		strncpy(cmd->url, url.c_str(), sizeof(cmd->url));
		cmd->url[sizeof(cmd->url) - 1] = '\0';
		strncpy(cmd->iface_addr, ifaceAddr.c_str(),
			sizeof(cmd->iface_addr));
		cmd->iface_addr[sizeof(cmd->iface_addr) - 1] = '\0';
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalOpen(url, ifaceAddr);
	}
}


int Session::open(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_open_single *cmd = NULL;
		PDRAW_RETURN_ERR_IF_FAILED(localAddr.length() <
			sizeof(cmd->local_addr) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(remoteAddr.length() <
			sizeof(cmd->remote_addr) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(ifaceAddr.length() <
			sizeof(cmd->iface_addr) - 1, -ENOBUFS);
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_open_single *)msg;
		cmd->base.type = CMD_TYPE_OPEN_SINGLE;
		strncpy(cmd->local_addr, localAddr.c_str(),
			sizeof(cmd->local_addr));
		cmd->local_addr[sizeof(cmd->local_addr) - 1] = '\0';
		cmd->local_stream_port = localStreamPort;
		cmd->local_control_port = localControlPort;
		strncpy(cmd->remote_addr, remoteAddr.c_str(),
			sizeof(cmd->remote_addr));
		cmd->remote_addr[sizeof(cmd->remote_addr) - 1] = '\0';
		cmd->remote_stream_port = remoteStreamPort;
		cmd->remote_control_port = remoteControlPort;
		strncpy(cmd->iface_addr, ifaceAddr.c_str(),
			sizeof(cmd->iface_addr));
		cmd->iface_addr[sizeof(cmd->iface_addr) - 1] = '\0';
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalOpen(localAddr, localStreamPort,
			localControlPort, remoteAddr, remoteStreamPort,
			remoteControlPort, ifaceAddr);
	}
}


int Session::open(
	const std::string &url,
	struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_open_url_mux *cmd = NULL;
		PDRAW_RETURN_ERR_IF_FAILED(url.length() <
			sizeof(cmd->url) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(mux != NULL, -EINVAL);
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_open_url_mux *)msg;
		cmd->base.type = CMD_TYPE_OPEN_URL_MUX;
		strncpy(cmd->url, url.c_str(), sizeof(cmd->url));
		cmd->url[sizeof(cmd->url) - 1] = '\0';
		cmd->mux = mux;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalOpen(url, mux);
	}
#else /* BUILD_LIBMUX */
	return -ENOSYS;
#endif /* BUILD_LIBMUX */
}


int Session::open(
	struct mux_ctx *mux)
{
	return open("", mux);
}


int Session::openSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_open_sdp *cmd = NULL;
		PDRAW_RETURN_ERR_IF_FAILED(sdp.length() <
			sizeof(cmd->sdp) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(ifaceAddr.length() <
			sizeof(cmd->iface_addr) - 1, -ENOBUFS);
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_open_sdp *)msg;
		cmd->base.type = CMD_TYPE_OPEN_SDP;
		strncpy(cmd->sdp, sdp.c_str(), sizeof(cmd->sdp));
		cmd->sdp[sizeof(cmd->sdp) - 1] = '\0';
		strncpy(cmd->iface_addr, ifaceAddr.c_str(),
			sizeof(cmd->iface_addr));
		cmd->iface_addr[sizeof(cmd->iface_addr) - 1] = '\0';
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalOpenSdp(sdp, ifaceAddr);
	}
}


int Session::openSdp(
	const std::string &sdp,
	struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_open_sdp_mux *cmd = NULL;
		PDRAW_RETURN_ERR_IF_FAILED(sdp.length() <
			sizeof(cmd->sdp) - 1, -ENOBUFS);
		PDRAW_RETURN_ERR_IF_FAILED(mux != NULL, -EINVAL);
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_open_sdp_mux *)msg;
		cmd->base.type = CMD_TYPE_OPEN_SDP_MUX;
		strncpy(cmd->sdp, sdp.c_str(), sizeof(cmd->sdp));
		cmd->sdp[sizeof(cmd->sdp) - 1] = '\0';
		cmd->mux = mux;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalOpenSdp(sdp, mux);
	}
#else /* BUILD_LIBMUX */
	return -ENOSYS;
#endif /* BUILD_LIBMUX */
}


int Session::close(
	void)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_base *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_base *)msg;
		cmd->type = CMD_TYPE_CLOSE;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalClose();
	}
}


int Session::play(
	float speed)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_play *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_play *)msg;
		cmd->base.type = CMD_TYPE_PLAY;
		cmd->speed = speed;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalPlay(speed);
	}
}


int Session::pause(
	void)
{
	return play(0.);
}


bool Session::isPaused(
	void)
{
	/* TODO */
	if (mDemuxer != NULL)
		return mDemuxer->isPaused();
	else
		return false;
}


int Session::previousFrame(
	void)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_base *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_base *)msg;
		cmd->type = CMD_TYPE_PREVIOUS_FRAME;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalPreviousFrame();
	}
}


int Session::nextFrame(
	void)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_base *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_base *)msg;
		cmd->type = CMD_TYPE_NEXT_FRAME;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalNextFrame();
	}
}


int Session::seek(
	int64_t delta,
	bool exact)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_seek *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_seek *)msg;
		cmd->base.type = CMD_TYPE_SEEK;
		cmd->delta = delta;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalSeek(delta);
	}
}


int Session::seekForward(
	uint64_t delta,
	bool exact)
{
	return seek((int64_t)delta);
}


int Session::seekBack(
	uint64_t delta,
	bool exact)
{
	return seek(-((int64_t)delta));
}


int Session::seekTo(
	uint64_t timestamp,
	bool exact)
{
	if (mInternalLoop) {
		/* Send a message to the loop */
		int res;
		struct cmd_seek_to *cmd = NULL;
		void *msg = calloc(PIPE_BUF - 1, 1);
		PDRAW_RETURN_ERR_IF_FAILED(msg != NULL, -ENOMEM);
		cmd = (struct cmd_seek_to *)msg;
		cmd->base.type = CMD_TYPE_SEEK_TO;
		cmd->timestamp = timestamp;
		res = mbox_push(mMbox, msg);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbox_push", res);
		free(msg);
		return res;
	} else {
		return internalSeekTo(timestamp);
	}
}


uint64_t Session::getDuration(
	void)
{
	/* TODO */
	return (mDemuxer) ? mDemuxer->getDuration() : 0;
}


uint64_t Session::getCurrentTime(
	void)
{
	/* TODO */
	return (mDemuxer) ? mDemuxer->getCurrentTime() : 0;
}


/* Called on the rendering thread */
int Session::startRenderer(
	int windowWidth,
	int windowHeight,
	int renderX,
	int renderY,
	int renderWidth,
	int renderHeight,
	bool hud,
	bool hmdDistorsionCorrection,
	bool headtracking,
	void *uiHandler)
{
	if (mRenderer == NULL) {
		int ret = enableRenderer();
		if (ret != 0)
			ULOGE("Failed to enable renderer");
	}

	if (mRenderer != NULL) {
		return mRenderer->setRendererParams(
			windowWidth, windowHeight, renderX, renderY,
			renderWidth, renderHeight, hud,
			hmdDistorsionCorrection, headtracking, uiHandler);
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}
}


/* Called on the rendering thread */
int Session::stopRenderer(
	void)
{
	if (mRenderer != NULL) {
		int ret = disableRenderer();
		if (ret != 0) {
			ULOGE("Failed to disable renderer");
			return -1;
		}
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}

	return 0;
}


/* Called on the rendering thread */
int Session::render(
	uint64_t lastRenderTime)
{
	if (mRenderer != NULL) {
		return mRenderer->render(lastRenderTime);
	} else {
		ULOGE("Invalid renderer");
		return -1;
	}
}


enum pdraw_session_type Session::getSessionType(
	void) {
	pthread_mutex_lock(&mMutex);
	enum pdraw_session_type ret = mSessionType;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


std::string Session::getSelfFriendlyName(
	void)
{
	return mSelfMetadata.getFriendlyName();
}


void Session::setSelfFriendlyName(
	const std::string &friendlyName)
{
	mSelfMetadata.setFriendlyName(friendlyName);
}


std::string Session::getSelfSerialNumber(
	void)
{
	return mSelfMetadata.getSerialNumber();
}


void Session::setSelfSerialNumber(
	const std::string &serialNumber)
{
	mSelfMetadata.setSerialNumber(serialNumber);
}


std::string Session::getSelfSoftwareVersion(
	void)
{
	return mSelfMetadata.getSoftwareVersion();
}


void Session::setSelfSoftwareVersion(
	const std::string &softwareVersion)
{
	mSelfMetadata.setSoftwareVersion(softwareVersion);
}


bool Session::isSelfPilot(
	void)
{
	return mSelfMetadata.isPilot();
}


void Session::setSelfPilot(
	bool isPilot)
{
	mSelfMetadata.setPilot(isPilot);
}


void Session::getSelfLocation(
	struct vmeta_location *loc)
{
	mSelfMetadata.getLocation(loc);
}


void Session::setSelfLocation(
	const struct vmeta_location *loc)
{
	mSelfMetadata.setLocation(loc);
}


int Session::getControllerBatteryLevel(
	void)
{
	return mSelfMetadata.getControllerBatteryLevel();
}


void Session::setControllerBatteryLevel(
	int batteryLevel)
{
	mSelfMetadata.setControllerBatteryLevel(batteryLevel);
}


void Session::getSelfControllerOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getControllerOrientation(quat);
}


void Session::getSelfControllerOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getControllerOrientation(euler);
}


void Session::setSelfControllerOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setControllerOrientation(quat);
}


void Session::setSelfControllerOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setControllerOrientation(euler);
}


void Session::getSelfHeadOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getHeadOrientation(quat);
}


void Session::getSelfHeadOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getHeadOrientation(euler);
}


void Session::setSelfHeadOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setHeadOrientation(quat);
}


void Session::setSelfHeadOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setHeadOrientation(euler);
}


void Session::getSelfHeadRefOrientation(
	struct vmeta_quaternion *quat)
{
	mSelfMetadata.getHeadRefOrientation(quat);
}


void Session::getSelfHeadRefOrientation(
	struct vmeta_euler *euler)
{
	mSelfMetadata.getHeadRefOrientation(euler);
}


void Session::setSelfHeadRefOrientation(
	const struct vmeta_quaternion *quat)
{
	mSelfMetadata.setHeadRefOrientation(quat);
}


void Session::setSelfHeadRefOrientation(
	const struct vmeta_euler *euler)
{
	mSelfMetadata.setHeadRefOrientation(euler);
}


void Session::resetSelfHeadRefOrientation(
	void)
{
	mSelfMetadata.resetHeadRefOrientation();
}


std::string Session::getPeerFriendlyName(
	void)
{
	return mPeerMetadata.getFriendlyName();
}


std::string Session::getPeerMaker(
	void)
{
	return mPeerMetadata.getMaker();
}


std::string Session::getPeerModel(
	void)
{
	return mPeerMetadata.getModel();
}


std::string Session::getPeerModelId(
	void)
{
	return mPeerMetadata.getModelId();
}


enum pdraw_drone_model Session::getPeerDroneModel(
	void)
{
	return mPeerMetadata.getDroneModel();
}


std::string Session::getPeerSerialNumber(
	void)
{
	return mPeerMetadata.getSerialNumber();
}


std::string Session::getPeerSoftwareVersion(
	void)
{
	return mPeerMetadata.getSoftwareVersion();
}


std::string Session::getPeerBuildId(
	void)
{
	return mPeerMetadata.getBuildId();
}


std::string Session::getPeerTitle(
	void)
{
	return mPeerMetadata.getTitle();
}


std::string Session::getPeerComment(
	void)
{
	return mPeerMetadata.getComment();
}


std::string Session::getPeerCopyright(
	void)
{
	return mPeerMetadata.getCopyright();
}


std::string Session::getPeerRunDate(
	void)
{
	return mPeerMetadata.getRunDate();
}


std::string Session::getPeerRunUuid(
	void)
{
	return mPeerMetadata.getRunUuid();
}


std::string Session::getPeerMediaDate(
	void)
{
	return mPeerMetadata.getMediaDate();
}


void Session::getPeerTakeoffLocation(
	struct vmeta_location *loc)
{
	mPeerMetadata.getTakeoffLocation(loc);
}


void Session::setPeerTakeoffLocation(
	const struct vmeta_location *loc)
{
	mPeerMetadata.setTakeoffLocation(loc);
}


void Session::getPeerHomeLocation(
	struct vmeta_location *loc)
{
	mPeerMetadata.getHomeLocation(loc);
}


void Session::setPeerHomeLocation(
	const struct vmeta_location *loc)
{
	mPeerMetadata.setHomeLocation(loc);
}


uint64_t Session::getPeerRecordingDuration(
	void)
{
	return mPeerMetadata.getRecordingDuration();
}


void Session::setPeerRecordingDuration(
	uint64_t duration)
{
	mPeerMetadata.setRecordingDuration(duration);
}


void Session::getCameraOrientationForHeadtracking(
	float *pan,
	float *tilt)
{
	Eigen::Quaternionf headQuat =
		mSelfMetadata.getDebiasedHeadOrientation();
	vmeta_quaternion quat;
	quat.w = headQuat.w();
	quat.x = headQuat.x();
	quat.y = headQuat.y();
	quat.z = headQuat.z();
	vmeta_euler euler;
	pdraw_quat2euler(&quat, &euler);

	if (pan)
		*pan = euler.yaw;
	if (tilt)
		*tilt = euler.pitch;
}


int Session::getMediaCount(
	void)
{
	pthread_mutex_lock(&mMutex);
	int ret = mMedias.size();
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::getMediaInfo(
	unsigned int index,
	struct pdraw_media_info *info)
{
	if (info == NULL) {
		ULOGE("Invalid info struct");
		return -1;
	}

	Media *media = getMedia(index);

	if (media == NULL) {
		ULOGE("Invalid media index");
		return -1;
	}

	media->lock();
	switch (media->getType()) {
	case PDRAW_MEDIA_TYPE_VIDEO:
		info->type = PDRAW_MEDIA_TYPE_VIDEO;
		info->info.video.type = ((VideoMedia*)media)->getVideoType();
		((VideoMedia*)media)->getDimensions(
			&info->info.video.width,
			&info->info.video.height,
			&info->info.video.cropLeft,
			&info->info.video.cropRight,
			&info->info.video.cropTop,
			&info->info.video.cropBottom,
			&info->info.video.croppedWidth,
			&info->info.video.croppedHeight,
			&info->info.video.sarWidth,
			&info->info.video.sarHeight);
		((VideoMedia*)media)->getFov(
			&info->info.video.horizontalFov,
			&info->info.video.verticalFov);
		break;
	default:
		info->type = PDRAW_MEDIA_TYPE_UNKNOWN;
		break;
	}
	info->id = media->getId();
	media->unlock();

	return 0;
}


void *Session::addVideoFrameFilterCallback(
	unsigned int mediaId,
	pdraw_video_frame_filter_callback_t cb,
	void *userPtr)
{
	pthread_mutex_lock(&mMutex);

	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media id");
		return NULL;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media type");
		return NULL;
	}

	VideoFrameFilter *filter =
		((VideoMedia*)media)->addVideoFrameFilter(cb, userPtr);
	if (filter == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Failed to create video frame filter");
		return NULL;
	}

	pthread_mutex_unlock(&mMutex);

	return (void *)filter;
}


int Session::removeVideoFrameFilterCallback(
	unsigned int mediaId,
	void *filterCtx)
{
	pthread_mutex_lock(&mMutex);

	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media id");
		return -1;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media type");
		return -1;
	}

	if (filterCtx == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid context pointer");
		return -1;
	}

	VideoFrameFilter *filter = (VideoFrameFilter*)filterCtx;
	int ret = ((VideoMedia*)media)->removeVideoFrameFilter(filter);

	pthread_mutex_unlock(&mMutex);

	return ret;
}


void *Session::addVideoFrameProducer(
	unsigned int mediaId,
	bool frameByFrame)
{
	pthread_mutex_lock(&mMutex);

	Media *media = getMediaById(mediaId);

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media id");
		return NULL;
	}

	if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media type");
		return NULL;
	}

	VideoFrameFilter *filter =
		((VideoMedia*)media)->addVideoFrameFilter(frameByFrame);
	if (filter == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Failed to create video frame filter");
		return NULL;
	}

	pthread_mutex_unlock(&mMutex);

	return (void*)filter;
}


int Session::removeVideoFrameProducer(
	void *producerCtx)
{
	if (producerCtx == NULL) {
		ULOGE("Invalid context pointer");
		return -1;
	}

	pthread_mutex_lock(&mMutex);

	VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;
	VideoMedia *media = filter->getVideoMedia();

	if (media == NULL) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("Invalid media");
		return -1;
	}

	int ret = media->removeVideoFrameFilter(filter);

	pthread_mutex_unlock(&mMutex);

	return ret;
}


int Session::getProducerLastFrame(
	void *producerCtx,
	struct pdraw_video_frame *frame,
	int timeout)
{
	if (producerCtx == NULL) {
		ULOGE("Invalid context pointer");
		return -1;
	}
	if (frame == NULL) {
		ULOGE("Invalid frame structure pointer");
		return -1;
	}

	VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;

	return filter->getLastFrame(frame, timeout);
}


float Session::getControllerRadarAngleSetting(
	void)
{
	return mSettings.getControllerRadarAngle();
}


void Session::setControllerRadarAngleSetting(
	float angle)
{
	mSettings.setControllerRadarAngle(angle);
}


void Session::getDisplayScreenSettings(
	float *xdpi,
	float *ydpi,
	float *deviceMargin)
{
	mSettings.getDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void Session::setDisplayScreenSettings(
	float xdpi,
	float ydpi,
	float deviceMargin)
{
	mSettings.setDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void Session::getHmdDistorsionCorrectionSettings(
	enum pdraw_hmd_model *hmdModel,
	float *ipd,
	float *scale,
	float *panH,
	float *panV)
{
	mSettings.getHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
}


void Session::setHmdDistorsionCorrectionSettings(
	enum pdraw_hmd_model hmdModel,
	float ipd,
	float scale,
	float panH,
	float panV)
{
	mSettings.setHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
}


/*
 * Internal methods
 */

int Session::internalOpen(
	const std::string &url,
	const std::string &ifaceAddr)
{
	int ret = 0;

	std::string ext = url.substr(url.length() - 4, 4);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
	if ((url.front() == '/') && (ext == ".mp4")) {
		pthread_mutex_lock(&mMutex);
		mSessionType = PDRAW_SESSION_TYPE_RECORD;
		pthread_mutex_unlock(&mMutex);
		mDemuxer = new RecordDemuxer(this);
		if (mDemuxer == NULL) {
			ULOGE("Session: failed to alloc demuxer");
			ret = -ENOMEM;
			goto out;
		}
		ret = ((RecordDemuxer *)mDemuxer)->open(url);
		if (ret < 0) {
			ULOGE("Session: failed to open demuxer");
			delete mDemuxer;
			mDemuxer = NULL;
			goto out;
		}
	} else if (((url.front() == '/') && (ext == ".sdp")) ||
		((url.substr(0, 7) == "http://") && (ext == ".sdp")) ||
		(url.substr(0, 7) == "rtsp://")) {
		pthread_mutex_lock(&mMutex);
		mSessionType = PDRAW_SESSION_TYPE_STREAM;
		pthread_mutex_unlock(&mMutex);
		mDemuxer = new StreamDemuxerNet(this);
		if (mDemuxer == NULL) {
			ULOGE("Session: failed to alloc demuxer");
			ret = -ENOMEM;
			goto out;
		}
		ret = ((StreamDemuxerNet *)mDemuxer)->open(
			url, ifaceAddr);
		if (ret < 0) {
			ULOGE("Session: failed to open demuxer");
			delete mDemuxer;
			mDemuxer = NULL;
			goto out;
		}
	} else {
		ULOGE("Session: unsupported URL");
		ret = -ENOSYS;
		goto out;
	}

	ret = addMediaFromDemuxer();
	if (ret < 0) {
		ULOG_ERRNO("Session: addMediaFromDemuxer", -ret);
		goto out;
	}

	setState(OPENED);

out:
	if (mListener)
		mListener->openResponse(this, ret);
	return ret;
}


int Session::internalOpen(
	const std::string &localAddr,
	int localStreamPort,
	int localControlPort,
	const std::string &remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const std::string &ifaceAddr)
{
	int ret = 0;

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerNet(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		ret = -ENOMEM;
		goto out;
	}

	ret = ((StreamDemuxerNet *)mDemuxer)->open(localAddr,
		localStreamPort, localControlPort, remoteAddr,
		remoteStreamPort, remoteControlPort, ifaceAddr);
	if (ret < 0) {
		ULOGE("Session: failed to open demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		goto out;
	}

	ret = addMediaFromDemuxer();
	if (ret < 0) {
		ULOG_ERRNO("Session: addMediaFromDemuxer", -ret);
		goto out;
	}

	setState(OPENED);

out:
	if (mListener)
		mListener->openResponse(this, ret);
	return ret;
}


int Session::internalOpen(
	const std::string &url,
	struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
	int ret = 0;

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerMux(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		ret = -ENOMEM;
		goto out;
	}

	ret = ((StreamDemuxerMux *)mDemuxer)->open(url, mux);
	if (ret < 0) {
		ULOGE("Session: failed to open demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		goto out;
	}

	ret = addMediaFromDemuxer();
	if (ret < 0) {
		ULOG_ERRNO("Session: addMediaFromDemuxer", -ret);
		goto out;
	}

	setState(OPENED);

out:
	if (mListener)
		mListener->openResponse(this, ret);
	return ret;
#else /* BUILD_LIBMUX */
	if (mListener)
		mListener->openResponse(this, -ENOSYS);
	return -ENOSYS;
#endif /* BUILD_LIBMUX */
}


int Session::internalOpenSdp(
	const std::string &sdp,
	const std::string &ifaceAddr)
{
	int ret = 0;

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerNet(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		ret = -ENOMEM;
		goto out;
	}

	ret = ((StreamDemuxerNet *)mDemuxer)->openSdp(sdp, ifaceAddr);
	if (ret < 0) {
		ULOGE("Session: failed to open demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		goto out;
	}

	ret = addMediaFromDemuxer();
	if (ret < 0) {
		ULOG_ERRNO("Session: addMediaFromDemuxer", -ret);
		goto out;
	}

	setState(OPENED);

out:
	if (mListener)
		mListener->openResponse(this, ret);
	return ret;
}


int Session::internalOpenSdp(
	const std::string &sdp,
	struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
	int ret = 0;

	pthread_mutex_lock(&mMutex);
	mSessionType = PDRAW_SESSION_TYPE_STREAM;
	pthread_mutex_unlock(&mMutex);
	mDemuxer = new StreamDemuxerMux(this);
	if (mDemuxer == NULL) {
		ULOGE("Session: failed to alloc demuxer");
		ret = -ENOMEM;
		goto out;
	}

	ret = ((StreamDemuxerMux *)mDemuxer)->openSdp(sdp, mux);
	if (ret < 0) {
		ULOGE("Session: failed to open demuxer");
		delete mDemuxer;
		mDemuxer = NULL;
		goto out;
	}

	ret = addMediaFromDemuxer();
	if (ret < 0) {
		ULOG_ERRNO("Session: addMediaFromDemuxer", -ret);
		goto out;
	}

	setState(OPENED);

out:
	if (mListener)
		mListener->openResponse(this, ret);
	return ret;
#else /* BUILD_LIBMUX */
	if (mListener)
		mListener->openResponse(this, -ENOSYS);
	return -ENOSYS;
#endif /* BUILD_LIBMUX */
}


int Session::internalClose(
	void)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->close();
	if (ret < 0) {
		ULOGE("Failed to close demuxer");
		goto out;
	}

	setState(CLOSED);

out:
	if (mListener)
		mListener->closeResponse(this, ret);
	return ret;
}


int Session::internalPlay(
	float speed)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->play(speed);
	if (ret < 0) {
		ULOGE("Failed to start demuxer");
		goto out;
	}

out:
	if (mListener) {
		if (speed == 0.) {
			mListener->pauseResponse(this,
				ret, getCurrentTime()); /* TODO*/
		} else {
			mListener->playResponse(this,
				ret, getCurrentTime()); /* TODO*/
		}
	}
	return ret;
}


int Session::internalPreviousFrame(
	void)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->previous();
	if (ret < 0) {
		ULOGE("Failed to go to previous frame in the demuxer");
		goto out;
	}

out:
	if (mListener)
		mListener->seekResponse(this, ret, getCurrentTime()); /* TODO*/
	return ret;
}


int Session::internalNextFrame(
	void)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->next();
	if (ret < 0) {
		ULOGE("Failed to go to next frame in the demuxer");
		goto out;
	}

out:
	if (mListener)
		mListener->seekResponse(this, ret, getCurrentTime()); /* TODO*/
	return ret;
}


int Session::internalSeek(
	int64_t delta,
	bool exact)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->seek(delta, exact);
	if (ret < 0) {
		ULOGE("Failed to seek with demuxer");
		goto out;
	}

out:
	if (mListener)
		mListener->seekResponse(this, ret, getCurrentTime()); /* TODO*/
	return ret;
}


int Session::internalSeekTo(
	uint64_t timestamp,
	bool exact)
{
	int ret = 0;

	if (mDemuxer == NULL) {
		ULOGE("Invalid demuxer");
		ret = -EPROTO;
		goto out;
	}

	ret = mDemuxer->seekTo(timestamp, exact);
	if (ret < 0) {
		ULOGE("Failed to seek with demuxer");
		goto out;
	}

out:
	if (mListener)
		mListener->seekResponse(this, ret, getCurrentTime()); /* TODO*/
	return ret;
}


int Session::addMediaFromDemuxer(
	void)
{
	int esCount = 0;

	esCount = mDemuxer->getElementaryStreamCount();
	if (esCount < 0) {
		ULOGE("Session: getElementaryStreamCount() failed (%d)",
			esCount);
		return -1;
	}

	int i;
	for (i = 0; i < esCount; i++) {
		enum elementary_stream_type esType =
			mDemuxer->getElementaryStreamType(i);
		if (esType < 0) {
			ULOGE("Session: getElementaryStreamType() failed (%d)",
				esType);
			continue;
		}

		Media *m = addMedia(esType, mDemuxer, i);
		if (!m)
			ULOGE("Session: media creation failed");
	}

	return 0;
}


Media *Session::addMedia(
	enum elementary_stream_type esType)
{
	Media *m = NULL;
	switch (esType) {
	case ELEMENTARY_STREAM_TYPE_UNKNOWN:
	default:
		break;
	case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
		m = new VideoMedia(this, esType, mMediaIdCounter++);
		m->enableDecoder();
		if (mRenderer != NULL) {
			int ret = mRenderer->addAvcDecoder(
				(AvcDecoder*)(m->getDecoder()));
			if (ret != 0) {
				ULOGE("Session: failed to add decoder "
					"to renderer");
			}
		}
		break;
	}

	if (m == NULL) {
		ULOGE("Session: media creation failed");
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	mMedias.push_back(m);
	pthread_mutex_unlock(&mMutex);

	return m;
}


Media *Session::addMedia(
	enum elementary_stream_type esType,
	Demuxer *demuxer,
	int demuxEsIndex)
{
	Media *m = NULL;
	switch (esType) {
	case ELEMENTARY_STREAM_TYPE_UNKNOWN:
	default:
		break;
	case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
		m = new VideoMedia(this, esType, mMediaIdCounter++,
			demuxer, demuxEsIndex);
		if (demuxer != NULL) {
			unsigned int width, height, sarWidth, sarHeight;
			unsigned int cropLeft, cropRight, cropTop, cropBottom;
			float hfov, vfov;
			demuxer->getElementaryStreamVideoDimensions(
				demuxEsIndex, &width, &height,
				&cropLeft, &cropRight, &cropTop, &cropBottom,
				&sarWidth, &sarHeight);
			((VideoMedia*)m)->setDimensions(width, height,
				cropLeft, cropRight, cropTop, cropBottom,
				sarWidth, sarHeight);
			demuxer->getElementaryStreamVideoFov(
				demuxEsIndex, &hfov, &vfov);
			((VideoMedia*)m)->setFov(hfov, vfov);
		}
		m->enableDecoder();
		if (mRenderer != NULL) {
			int ret = mRenderer->addAvcDecoder(
				(AvcDecoder*)(m->getDecoder()));
			if (ret != 0) {
				ULOGE("Session: failed to add decoder "
					"to renderer");
			}
		}
		break;
	}

	if (m == NULL) {
		ULOGE("Session: media creation failed");
		return NULL;
	}

	pthread_mutex_lock(&mMutex);
	mMedias.push_back(m);
	pthread_mutex_unlock(&mMutex);

	return m;
}


int Session::removeMedia(
	Media *media)
{
	bool found = false;

	pthread_mutex_lock(&mMutex);

	std::vector<Media*>::iterator m = mMedias.begin();

	while (m != mMedias.end()) {
		if (*m == media) {
			mMedias.erase(m);
			delete *m;
			found = true;
			break;
		}
		m++;
	}

	pthread_mutex_unlock(&mMutex);
	return (found) ? 0 : -1;
}


int Session::removeMedia(
	unsigned int index)
{
	pthread_mutex_lock(&mMutex);

	if (index >= mMedias.size()) {
		pthread_mutex_unlock(&mMutex);
		return -1;
	}

	Media *m = mMedias.at(index);
	mMedias.erase(mMedias.begin() + index);
	delete m;

	pthread_mutex_unlock(&mMutex);
	return 0;
}


Media *Session::getMedia(
	unsigned int index)
{
	pthread_mutex_lock(&mMutex);

	if (index >= mMedias.size()) {
		pthread_mutex_unlock(&mMutex);
		return NULL;
	}

	Media *ret = mMedias.at(index);
	pthread_mutex_unlock(&mMutex);
	return ret;
}


Media *Session::getMediaById(
	unsigned int id)
{
	pthread_mutex_lock(&mMutex);
	std::vector<Media*>::iterator m = mMedias.begin();

	while (m != mMedias.end()) {
		if ((*m)->getId() == id) {
			pthread_mutex_unlock(&mMutex);
			return *m;
		}
		m++;
	}

	pthread_mutex_unlock(&mMutex);
	ULOGE("Session: unable to find media by id");
	return NULL;
}


void Session::setState(
	enum State state)
{
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	mState = state;
	pthread_mutex_unlock(&mMutex);
	ULOGI("Session: state change to %s", pdrawStateStr(state));

	if (mListener)
		mListener->onStateChanged(this, state);
}


void Session::socketCreated(
	int fd)
{
	if (mListener)
		mListener->onSocketCreated(this, fd);
}


/* Called on the rendering thread */
int Session::enableRenderer(
	void)
{
	int ret = 0;

	if (mRenderer != NULL) {
		ULOGE("Session: renderer is already enabled");
		return -1;
	}

	mRenderer = Renderer::create(this);
	if (mRenderer == NULL) {
		ULOGE("Session: failed to alloc renderer");
		return -1;
	}

	std::vector<Media*>::iterator m;
	pthread_mutex_lock(&mMutex);

	for (m = mMedias.begin(); m < mMedias.end(); m++) {
		pthread_mutex_unlock(&mMutex);
		if ((*m)->getType() == PDRAW_MEDIA_TYPE_VIDEO) {
			ret = mRenderer->addAvcDecoder(
				(AvcDecoder*)((*m)->getDecoder()));
			if (ret != 0) {
				ULOGE("Session: failed to add decoder "
					"to renderer");
			}
		}
		pthread_mutex_lock(&mMutex);
	}

	pthread_mutex_unlock(&mMutex);
	return ret;
}


/* Called on the rendering thread */
int Session::disableRenderer(
	void)
{
	int ret = 0;

	if (mRenderer == NULL) {
		ULOGE("Session: renderer is not enabled");
		return -1;
	}

	std::vector<Media*>::iterator m;
	pthread_mutex_lock(&mMutex);

	for (m = mMedias.begin(); m < mMedias.end(); m++) {
		pthread_mutex_unlock(&mMutex);
		if ((*m)->getType() == PDRAW_MEDIA_TYPE_VIDEO) {
			ret = mRenderer->removeAvcDecoder(
				(AvcDecoder*)((*m)->getDecoder()));
			if (ret != 0) {
				ULOGE("Session: failed to remove decoder "
					"from renderer");
			}
		}
		pthread_mutex_lock(&mMutex);
	}

	pthread_mutex_unlock(&mMutex);
	delete mRenderer;
	mRenderer = NULL;

	return ret;
}


void Session::mboxCb(
	int fd,
	uint32_t revents,
	void *userdata)
{
	Session *self = (Session *)userdata;
	int res;
	void *msg;
	struct cmd_base *base;

	PDRAW_RETURN_IF_FAILED(self != NULL, -EINVAL);

	msg = malloc(PIPE_BUF - 1);
	PDRAW_RETURN_IF_FAILED(msg != NULL, -ENOMEM);

	do {
		/* Read from the mailbox */
		res = mbox_peek(self->mMbox, msg);
		if (res < 0) {
			if (res != -EAGAIN)
				PDRAW_LOG_ERRNO("mbox_peek", -res);
			break;
		}

		base = (struct cmd_base *)msg;
		switch (base->type) {
		case CMD_TYPE_OPEN_SINGLE:
		{
			struct cmd_open_single *cmd =
				(struct cmd_open_single *)msg;
			std::string l(cmd->local_addr);
			std::string r(cmd->remote_addr);
			std::string i(cmd->iface_addr);
			res = self->internalOpen(l, cmd->local_stream_port,
				cmd->local_control_port, r,
				cmd->remote_stream_port,
				cmd->remote_control_port, i);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalOpen", -res);
			break;
		}
		case CMD_TYPE_OPEN_URL:
		{
			struct cmd_open_url *cmd =
				(struct cmd_open_url *)msg;
			std::string u(cmd->url);
			std::string i(cmd->iface_addr);
			res = self->internalOpen(u, i);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalOpen", -res);
			break;
		}
		case CMD_TYPE_OPEN_URL_MUX:
		{
			struct cmd_open_url_mux *cmd =
				(struct cmd_open_url_mux *)msg;
			std::string u(cmd->url);
			res = self->internalOpen(u, cmd->mux);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalOpen", -res);
			break;
		}
		case CMD_TYPE_OPEN_SDP:
		{
			struct cmd_open_sdp *cmd =
				(struct cmd_open_sdp *)msg;
			std::string s(cmd->sdp);
			std::string i(cmd->iface_addr);
			res = self->internalOpenSdp(s, i);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalOpenSdp", -res);
			break;
		}
		case CMD_TYPE_OPEN_SDP_MUX:
		{
			struct cmd_open_sdp_mux *cmd =
				(struct cmd_open_sdp_mux *)msg;
			std::string s(cmd->sdp);
			res = self->internalOpenSdp(s, cmd->mux);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalOpenSdp", -res);
			break;
		}
		case CMD_TYPE_CLOSE:
		{
			res = self->internalClose();
			if (res < 0)
				PDRAW_LOG_ERRNO("internalClose", -res);
			break;
		}
		case CMD_TYPE_PLAY:
		{
			struct cmd_play *cmd =
				(struct cmd_play *)msg;
			res = self->internalPlay(cmd->speed);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalPlay", -res);
			break;
		}
		case CMD_TYPE_PREVIOUS_FRAME:
		{
			res = self->internalPreviousFrame();
			if (res < 0)
				PDRAW_LOG_ERRNO("internalPreviousFrame", -res);
			break;
		}
		case CMD_TYPE_NEXT_FRAME:
		{
			res = self->internalNextFrame();
			if (res < 0)
				PDRAW_LOG_ERRNO("internalNextFrame", -res);
			break;
		}
		case CMD_TYPE_SEEK:
		{
			struct cmd_seek *cmd =
				(struct cmd_seek *)msg;
			res = self->internalSeek(cmd->delta);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalSeek", -res);
			break;
		}
		case CMD_TYPE_SEEK_TO:
		{
			struct cmd_seek_to *cmd =
				(struct cmd_seek_to *)msg;
			res = self->internalSeekTo(cmd->timestamp);
			if (res < 0)
				PDRAW_LOG_ERRNO("internalSeekTo", -res);
			break;
		}
		default:
			PDRAW_LOGE("unknown command");
			break;
		}
	} while (res == 0);

	free(msg);
}


void *Session::runLoopThread(
	void *ptr)
{
	Session *self = (Session *)ptr;

	if (!self->mInternalLoop)
		return NULL;

	while (!self->mThreadShouldStop) {
		pomp_loop_wait_and_process(self->mLoop, -1);
	}

	return NULL;
}

} /* namespace Pdraw */
