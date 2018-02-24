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

#ifndef _PDRAW_SESSION_HPP_
#define _PDRAW_SESSION_HPP_

#include <inttypes.h>
#include <libpomp.h>
#include <futils/futils.h>
#include <string>
#include <vector>
#include "pdraw_settings.hpp"
#include "pdraw_metadata_session.hpp"
#include "pdraw_media.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_renderer.hpp"
#include <pdraw/pdraw.hpp>

namespace Pdraw {


class Settings;


class Session : public IPdraw {
public:
	Session(
		struct pomp_loop *loop);

	~Session(
		void);


	/*
	 * API methods
	 */

	int open(
		const std::string &url);

	int open(
		const std::string &url,
		const std::string &ifaceAddr);

	int open(
		const std::string &localAddr,
		int localStreamPort,
		int localControlPort,
		const std::string &remoteAddr,
		int remoteStreamPort,
		int remoteControlPort,
		const std::string &ifaceAddr);

	int open(
		const std::string &url,
		struct mux_ctx *mux);

	int open(
		struct mux_ctx *mux);

	int openSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	int openSdp(
		const std::string &sdp,
		struct mux_ctx *mux);

	int close(
		void);

	int play(
		float speed = 1.0f);

	int pause(
		void);

	bool isPaused(
		void);

	int previousFrame(
		void);

	int nextFrame(
		void);

	int seek(
		int64_t delta,
		bool exact = false);

	int seekForward(
		uint64_t delta,
		bool exact = false);

	int seekBack(
		uint64_t delta,
		bool exact = false);

	int seekTo(
		uint64_t timestamp,
		bool exact = false);

	uint64_t getDuration(
		void);

	uint64_t getCurrentTime(
		void);

	/* Called on the rendering thread */
	int startRenderer(
		int windowWidth,
		int windowHeight,
		int renderX,
		int renderY,
		int renderWidth,
		int renderHeight,
		bool hmdDistorsionCorrection,
		bool headtracking,
		void *uiHandler);

	/* Called on the rendering thread */
	int stopRenderer(
		void);

	/* Called on the rendering thread */
	int render(
		uint64_t lastRenderTime);

	enum pdraw_session_type getSessionType(
		void);

	std::string getSelfFriendlyName(
		void);

	void setSelfFriendlyName(
		const std::string &friendlyName);

	std::string getSelfSerialNumber(
		void);

	void setSelfSerialNumber(
		const std::string &serialNumber);

	std::string getSelfSoftwareVersion(
		void);

	void setSelfSoftwareVersion(
		const std::string &softwareVersion);

	bool isSelfPilot(
		void);

	void setSelfPilot(
		bool isPilot);

	void getSelfLocation(
		struct vmeta_location *loc);

	void setSelfLocation(
		const struct vmeta_location *loc);

	int getControllerBatteryLevel(
		void);

	void setControllerBatteryLevel(
		int batteryLevel);

	void getSelfControllerOrientation(
		struct vmeta_quaternion *quat);

	void getSelfControllerOrientation(
		struct vmeta_euler *euler);

	void setSelfControllerOrientation(
		const struct vmeta_quaternion *quat);

	void setSelfControllerOrientation(
		const struct vmeta_euler *euler);

	void getSelfHeadOrientation(
		struct vmeta_quaternion *quat);

	void getSelfHeadOrientation(
		struct vmeta_euler *euler);

	void setSelfHeadOrientation(
		const struct vmeta_quaternion *quat);

	void setSelfHeadOrientation(
		const struct vmeta_euler *euler);

	void getSelfHeadRefOrientation(
		struct vmeta_quaternion *quat);

	void getSelfHeadRefOrientation(
		struct vmeta_euler *euler);

	void setSelfHeadRefOrientation(
		const struct vmeta_quaternion *quat);

	void setSelfHeadRefOrientation(
		const struct vmeta_euler *euler);

	void resetSelfHeadRefOrientation(
		void);

	std::string getPeerFriendlyName(
		void);

	std::string getPeerMaker(
		void);

	std::string getPeerModel(
		void);

	std::string getPeerModelId(
		void);

	enum pdraw_drone_model getPeerDroneModel(
		void);

	std::string getPeerSerialNumber(
		void);

	std::string getPeerSoftwareVersion(
		void);

	std::string getPeerBuildId(
		void);

	std::string getPeerTitle(
		void);

	std::string getPeerComment(
		void);

	std::string getPeerCopyright(
		void);

	std::string getPeerRunDate(
		void);

	std::string getPeerRunUuid(
		void);

	std::string getPeerMediaDate(
		void);

	void getPeerTakeoffLocation(
		struct vmeta_location *loc);

	void setPeerTakeoffLocation(
		const struct vmeta_location *loc);

	void getPeerHomeLocation(
		struct vmeta_location *loc);

	void setPeerHomeLocation(
		const struct vmeta_location *loc);

	uint64_t getPeerRecordingDuration(
		void);

	void setPeerRecordingDuration(
		uint64_t duration);

	void getCameraOrientationForHeadtracking(
		float *pan,
		float *tilt);

	int getMediaCount(
		void);

	int getMediaInfo(
		unsigned int index,
		struct pdraw_media_info *info);

	void *addVideoFrameFilterCallback(
		unsigned int mediaId,
		pdraw_video_frame_filter_callback_t cb,
		void *userPtr);

	int removeVideoFrameFilterCallback(
		unsigned int mediaId,
		void *filterCtx);

	void *addVideoFrameProducer(
		unsigned int mediaId,
		bool frameByFrame = false);

	int removeVideoFrameProducer(
		void *producerCtx);

	/**
	 * get last frame
	 *
	 * timeout: time in microseconds to wait for a frame
	 *  0: don't wait
	 * -1: wait forever
	 * >0: wait time
	 */
	int getProducerLastFrame(
		void *producerCtx,
		struct pdraw_video_frame *frame,
		int timeout = 0);

	float getControllerRadarAngleSetting(
		void);

	void setControllerRadarAngleSetting(
		float angle);

	void getDisplayScreenSettings(
		float *xdpi,
		float *ydpi,
		float *deviceMargin);

	void setDisplayScreenSettings(
		float xdpi,
		float ydpi,
		float deviceMargin);

	void getHmdDistorsionCorrectionSettings(
		enum pdraw_hmd_model *hmdModel,
		float *ipd,
		float *scale,
		float *panH,
		float *panV);

	void setHmdDistorsionCorrectionSettings(
		enum pdraw_hmd_model hmdModel,
		float ipd,
		float scale,
		float panH,
		float panV);

	void *getJniEnv(
		void) {
		return mJniEnv;
	}

	void setJniEnv(
		void *jniEnv) {
		mJniEnv = jniEnv;
	}


	/*
	 * Internal methods
	 */

	int internalOpen(
		const std::string &url,
		const std::string &ifaceAddr);

	int internalOpen(
		const std::string &localAddr,
		int localStreamPort,
		int localControlPort,
		const std::string &remoteAddr,
		int remoteStreamPort,
		int remoteControlPort,
		const std::string &ifaceAddr);

	int internalOpen(
		const std::string &url,
		struct mux_ctx *mux);

	int internalOpenSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	int internalOpenSdp(
		const std::string &sdp,
		struct mux_ctx *mux);

	int internalClose(
		void);

	int internalPlay(
		float speed = 1.0f);

	int internalPreviousFrame(
		void);

	int internalNextFrame(
		void);

	int internalSeek(
		int64_t delta,
		bool exact = false);

	int internalSeekTo(
		uint64_t timestamp,
		bool exact = false);

	Media *addMedia(
		enum elementary_stream_type esType);

	Media *addMedia(
		enum elementary_stream_type esType,
		Demuxer *demuxer,
		int demuxEsIndex);

	int removeMedia(
		Media *media);

	int removeMedia(
		unsigned int index);

	Media *getMedia(
		unsigned int index);

	Media *getMediaById(
		unsigned int id);

	Demuxer *getDemuxer(
		void) {
		return mDemuxer;
	}

	Renderer *getRenderer(
		void) {
		return mRenderer;
	}

	Settings *getSettings(
		void) {
		return &mSettings;
	}

	SessionSelfMetadata *getSelfMetadata(
		void) {
		return &mSelfMetadata;
	}

	SessionPeerMetadata *getPeerMetadata(
		void) {
		return &mPeerMetadata;
	}

	struct pomp_loop *getLoop() {
		return mLoop;
	}

private:
	int enableRenderer(
		void);

	int disableRenderer(
		void);

	int addMediaFromDemuxer(
		void);

	static void mboxCb(
		int fd,
		uint32_t revents,
		void *userdata);

	static void* runLoopThread(
		void *ptr);

	bool mInternalLoop;
	struct pomp_loop *mLoop;
	pthread_t mLoopThread;
	bool mLoopThreadLaunched;
	bool mThreadShouldStop;
	struct mbox *mMbox;
	pthread_mutex_t mMutex;
	enum pdraw_session_type mSessionType;
	Settings mSettings;
	SessionSelfMetadata mSelfMetadata;
	SessionPeerMetadata mPeerMetadata;
	std::vector<Media *> mMedias;
	Demuxer *mDemuxer;
	Renderer *mRenderer;
	unsigned int mMediaIdCounter;
	void *mJniEnv;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SESSION_HPP_ */
