/**
 * Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_HPP_
#define _PDRAW_HPP_

#include <inttypes.h>
#include <string>
#include "pdraw_defs.h"

namespace Pdraw {


class IPdraw {
public:
	enum State {
		INVALID = 0,
		CREATED,
		OPENED,
		CLOSED,
	};

	class Listener {
	public:
		virtual ~Listener(
			void) {}

		virtual void onStateChanged(
			IPdraw *pdraw,
			State state) = 0;

		virtual void openResponse(
			IPdraw *pdraw,
			int status) = 0;

		virtual void closeResponse(
			IPdraw *pdraw,
			int status) = 0;

		virtual void playResponse(
			IPdraw *pdraw,
			int status,
			uint64_t timestamp) = 0;

		virtual void pauseResponse(
			IPdraw *pdraw,
			int status,
			uint64_t timestamp) = 0;

		virtual void seekResponse(
			IPdraw *pdraw,
			int status,
			uint64_t timestamp) = 0;

		virtual void onSocketCreated(
			IPdraw *pdraw,
			int fd) = 0;
	};

	virtual ~IPdraw(
		void) {}

	virtual State getState(
		void) = 0;

	virtual int open(
		const std::string &url) = 0;

	virtual int open(
		const std::string &url,
		const std::string &ifaceAddr) = 0;

	virtual int open(
		const std::string &localAddr,
		int localStreamPort,
		int localControlPort,
		const std::string &remoteAddr,
		int remoteStreamPort,
		int remoteControlPort,
		const std::string &ifaceAddr) = 0;

	virtual int open(
		const std::string &url,
		struct mux_ctx *mux) = 0;

	virtual int open(
		struct mux_ctx *mux) = 0;

	virtual int openSdp(
		const std::string &sdp,
		const std::string &ifaceAddr) = 0;

	virtual int openSdp(
		const std::string &sdp,
		struct mux_ctx *mux) = 0;

	virtual int close(
		void) = 0;

	virtual int play(
		float speed = 1.0f) = 0;

	virtual int pause(
		void) = 0;

	virtual bool isPaused(
		void) = 0;

	virtual int previousFrame(
		void) = 0;

	virtual int nextFrame(
		void) = 0;

	virtual int seek(
		int64_t delta,
		bool exact = false) = 0;

	virtual int seekForward(
		uint64_t delta,
		bool exact = false) = 0;

	virtual int seekBack(
		uint64_t delta,
		bool exact = false) = 0;

	virtual int seekTo(
		uint64_t timestamp,
		bool exact = false) = 0;

	virtual uint64_t getDuration(
		void) = 0;

	virtual uint64_t getCurrentTime(
		void) = 0;

	virtual int startRenderer(
		int windowWidth,
		int windowHeight,
		int renderX,
		int renderY,
		int renderWidth,
		int renderHeight,
		bool hmdDistorsionCorrection,
		bool headtracking,
		void *uiHandler) = 0;

	virtual int stopRenderer(
		void) = 0;

	virtual int render(
		uint64_t lastRenderTime) = 0;

	virtual enum pdraw_session_type getSessionType(
		void) = 0;

	virtual std::string getSelfFriendlyName(
		void) = 0;
	virtual void setSelfFriendlyName(
		const std::string &friendlyName) = 0;

	virtual std::string getSelfSerialNumber(
		void) = 0;
	virtual void setSelfSerialNumber(
		const std::string &serialNumber) = 0;

	virtual std::string getSelfSoftwareVersion(
		void) = 0;
	virtual void setSelfSoftwareVersion(
		const std::string &softwareVersion) = 0;

	virtual bool isSelfPilot(
		void) = 0;
	virtual void setSelfPilot(
		bool isPilot) = 0;

	virtual void getSelfLocation(
		struct vmeta_location *loc) = 0;
	virtual void setSelfLocation(
		const struct vmeta_location *loc) = 0;

	virtual int getControllerBatteryLevel(
		void) = 0;
	virtual void setControllerBatteryLevel(
		int batteryLevel) = 0;

	virtual void getSelfControllerOrientation(
		struct vmeta_quaternion *quat) = 0;
	virtual void getSelfControllerOrientation(
		struct vmeta_euler *euler) = 0;
	virtual void setSelfControllerOrientation(
		const struct vmeta_quaternion *quat) = 0;
	virtual void setSelfControllerOrientation(
		const struct vmeta_euler *euler) = 0;

	virtual void getSelfHeadOrientation(
		struct vmeta_quaternion *quat) = 0;
	virtual void getSelfHeadOrientation(
		struct vmeta_euler *euler) = 0;
	virtual void setSelfHeadOrientation(
		const struct vmeta_quaternion *quat) = 0;
	virtual void setSelfHeadOrientation(
		const struct vmeta_euler *euler) = 0;

	virtual void getSelfHeadRefOrientation(
		struct vmeta_quaternion *quat) = 0;
	virtual void getSelfHeadRefOrientation(
		struct vmeta_euler *euler) = 0;
	virtual void setSelfHeadRefOrientation(
		const struct vmeta_quaternion *quat) = 0;
	virtual void setSelfHeadRefOrientation(
		const struct vmeta_euler *euler) = 0;
	virtual void resetSelfHeadRefOrientation(
		void) = 0;

	virtual std::string getPeerFriendlyName(
		void) = 0;

	virtual std::string getPeerMaker(
		void) = 0;

	virtual std::string getPeerModel(
		void) = 0;

	virtual std::string getPeerModelId(
		void) = 0;

	virtual enum pdraw_drone_model getPeerDroneModel(
		void) = 0;

	virtual std::string getPeerSerialNumber(
		void) = 0;

	virtual std::string getPeerSoftwareVersion(
		void) = 0;

	virtual std::string getPeerBuildId(
		void) = 0;

	virtual std::string getPeerTitle(
		void) = 0;

	virtual std::string getPeerComment(
		void) = 0;

	virtual std::string getPeerCopyright(
		void) = 0;

	virtual std::string getPeerRunDate(
		void) = 0;

	virtual std::string getPeerRunUuid(
		void) = 0;

	virtual std::string getPeerMediaDate(
		void) = 0;

	virtual void getPeerTakeoffLocation(
		struct vmeta_location *loc) = 0;
	virtual void setPeerTakeoffLocation(
		const struct vmeta_location *loc) = 0;

	virtual void getPeerHomeLocation(
		struct vmeta_location *loc) = 0;
	virtual void setPeerHomeLocation(
		const struct vmeta_location *loc) = 0;

	virtual uint64_t getPeerRecordingDuration(
		void) = 0;
	virtual void setPeerRecordingDuration(
		uint64_t duration) = 0;

	virtual void getCameraOrientationForHeadtracking(
		float *pan,
		float *tilt) = 0;

	virtual int getMediaCount(
		void) = 0;

	virtual int getMediaInfo(
		unsigned int index,
		struct pdraw_media_info *info) = 0;

	virtual void *addVideoFrameFilterCallback(
		unsigned int mediaId,
		pdraw_video_frame_filter_callback_t cb,
		void *userPtr) = 0;

	virtual int removeVideoFrameFilterCallback(
		unsigned int mediaId,
		void *filterCtx) = 0;

	virtual void *addVideoFrameProducer(
		unsigned int mediaId,
		bool frameByFrame = false) = 0;

	virtual int removeVideoFrameProducer(
		void *producerCtx) = 0;

	/**
	 * Video frame producer: get the last frame
	 *
	 * timeout : time in microseconds to wait for a frame
	 *  0: don't wait
	 * -1: wait forever
	 * >0: wait time
	 */
	virtual int getProducerLastFrame(
		void *producerCtx,
		struct pdraw_video_frame *frame,
		int timeout = 0) = 0;

	virtual float getControllerRadarAngleSetting(
		void) = 0;
	virtual void setControllerRadarAngleSetting(
		float angle) = 0;

	virtual void getDisplayScreenSettings(
		float *xdpi,
		float *ydpi,
		float *deviceMargin) = 0;
	virtual void setDisplayScreenSettings(
		float xdpi,
		float ydpi,
		float deviceMargin) = 0;

	virtual void getHmdDistorsionCorrectionSettings(
		enum pdraw_hmd_model *hmdModel,
		float *ipd,
		float *scale,
		float *panH,
		float *panV) = 0;
	virtual void setHmdDistorsionCorrectionSettings(
		enum pdraw_hmd_model hmdModel,
		float ipd,
		float scale,
		float panH,
		float panV) = 0;

	virtual void setJniEnv(
		void *jniEnv) = 0;
};


int createPdraw(
	struct pomp_loop *loop,
	IPdraw::Listener *listener,
	IPdraw **ret_obj);


const char *pdrawStateStr(
	enum IPdraw::State val);


} /* namespace Pdraw */

#endif /* !_PDRAW_HPP_ */
