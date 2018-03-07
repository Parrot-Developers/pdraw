/**
 * Parrot Drones Awesome Video Viewer Library
 * C wrapper functions
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
#include "pdraw_utils.hpp"
#include "pdraw_log.hpp"
#include <pdraw/pdraw.h>
#include <errno.h>
#include <string>


/* codecheck_ignore[COMPLEX_MACRO] */
#define ENUM_CASE(_prefix, _name) \
	case _prefix##_name: return #_name


/* NOTE: Pdraw::IPdraw::State and enum pdraw_state values
 * must be synchronized */
PDRAW_STATIC_ASSERT((int)Pdraw::IPdraw::State::INVALID == PDRAW_STATE_INVALID);
PDRAW_STATIC_ASSERT((int)Pdraw::IPdraw::State::CREATED == PDRAW_STATE_CREATED);
PDRAW_STATIC_ASSERT((int)Pdraw::IPdraw::State::OPENED == PDRAW_STATE_OPENED);
PDRAW_STATIC_ASSERT((int)Pdraw::IPdraw::State::CLOSED == PDRAW_STATE_CLOSED);


class PdrawListener : public Pdraw::IPdraw::Listener {
public:
	PdrawListener(
		struct pdraw *pdraw,
		const struct pdraw_cbs *cbs,
		void *userdata) :
		mPdraw(pdraw),
		mCbs(*cbs),
		mUserdata(userdata) {}

	~PdrawListener() {}

	void onStateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::State state) {
		if (mCbs.state_changed) {
			(*mCbs.state_changed)(mPdraw,
				(enum pdraw_state)state, mUserdata);
		}
	}

	void openResponse(
		Pdraw::IPdraw *pdraw,
		int status) {
		if (mCbs.open_resp) {
			(*mCbs.open_resp)(mPdraw, status, mUserdata);
		}
	}

	void closeResponse(
		Pdraw::IPdraw *pdraw,
		int status) {
		if (mCbs.close_resp) {
			(*mCbs.close_resp)(mPdraw, status, mUserdata);
		}
	}

	void playResponse(
		Pdraw::IPdraw *pdraw,
		int status,
		uint64_t timestamp) {
		if (mCbs.play_resp) {
			(*mCbs.play_resp)(mPdraw, status, timestamp, mUserdata);
		}
	}

	void pauseResponse(
		Pdraw::IPdraw *pdraw,
		int status,
		uint64_t timestamp) {
		if (mCbs.pause_resp) {
			(*mCbs.pause_resp)(mPdraw, status, timestamp, mUserdata);
		}
	}

	void seekResponse(
		Pdraw::IPdraw *pdraw,
		int status,
		uint64_t timestamp) {
		if (mCbs.seek_resp) {
			(*mCbs.seek_resp)(mPdraw, status, timestamp, mUserdata);
		}
	}

	void onSocketCreated(
		Pdraw::IPdraw *pdraw,
		int fd) {
		if (mCbs.socket_created)
			(*mCbs.socket_created)(mPdraw, fd, mUserdata);
	}

private:
	struct pdraw *mPdraw;
	struct pdraw_cbs mCbs;
	void *mUserdata;
};


struct pdraw {
	Pdraw::IPdraw *pdraw;
	PdrawListener *listener;
};


const char *pdraw_state_str(
	enum pdraw_state val)
{
	switch (val) {
	ENUM_CASE(PDRAW_STATE_, INVALID);
	ENUM_CASE(PDRAW_STATE_, CREATED);
	ENUM_CASE(PDRAW_STATE_, OPENED);
	ENUM_CASE(PDRAW_STATE_, CLOSED);
	default: return NULL;
	}
}


int pdraw_new(
	struct pomp_loop *loop,
	const struct pdraw_cbs *cbs,
	void *userdata,
	struct pdraw **ret_obj)
{
	int ret = 0;
	struct pdraw *pdraw;

	PDRAW_RETURN_ERR_IF_FAILED(cbs != NULL, -EINVAL);
	PDRAW_RETURN_ERR_IF_FAILED(ret_obj != NULL, -EINVAL);

	pdraw = (struct pdraw *)calloc(1, sizeof(*pdraw));
	PDRAW_RETURN_ERR_IF_FAILED(pdraw != NULL, -ENOMEM);

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


int pdraw_destroy(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;
	if (pdraw->listener != NULL)
		delete pdraw->listener;
	if (pdraw->pdraw != NULL)
		delete pdraw->pdraw;
	free(pdraw);
	return 0;
}


int pdraw_open_url(
	struct pdraw *pdraw,
	const char *url)
{
	if ((pdraw == NULL) || (url == NULL))
		return -EINVAL;

	std::string u(url);
	return pdraw->pdraw->open(u);
}


int pdraw_open_url_mcast(
	struct pdraw *pdraw,
	const char *url,
	const char *ifaceAddr)
{
	if ((pdraw == NULL) || (url == NULL))
		return -EINVAL;

	std::string u(url);
	std::string i(ifaceAddr);
	return pdraw->pdraw->open(u, i);
}


int pdraw_open_single_stream(
	struct pdraw *pdraw,
	const char *localAddr,
	int localStreamPort,
	int localControlPort,
	const char *remoteAddr,
	int remoteStreamPort,
	int remoteControlPort,
	const char *ifaceAddr)
{
	if ((pdraw == NULL) || (localAddr == NULL) || (remoteAddr == NULL))
		return -EINVAL;

	std::string local(localAddr);
	std::string remote(remoteAddr);
	std::string iface(ifaceAddr);
	return pdraw->pdraw->open(local,
		localStreamPort, localControlPort, remote,
		remoteStreamPort, remoteControlPort, iface);
}


int pdraw_open_url_mux(
	struct pdraw *pdraw,
	const char *url,
	struct mux_ctx *mux)
{
	if ((pdraw == NULL) || (url == NULL) || (mux == NULL))
		return -EINVAL;

	std::string u(url);
	return pdraw->pdraw->open(u, mux);
}


int pdraw_open_single_stream_mux(
	struct pdraw *pdraw,
	struct mux_ctx *mux)
{
	if ((pdraw == NULL) || (mux == NULL))
		return -EINVAL;

	return pdraw->pdraw->open(mux);
}


int pdraw_open_sdp(
	struct pdraw *pdraw,
	const char *sdp,
	const char *ifaceAddr)
{
	if ((pdraw == NULL) || (sdp == NULL))
		return -EINVAL;

	std::string s(sdp);
	std::string i(ifaceAddr);
	return pdraw->pdraw->openSdp(s, i);
}


int pdraw_open_sdp_mux(
	struct pdraw *pdraw,
	const char *sdp,
	struct mux_ctx *mux)
{
	if ((pdraw == NULL) || (sdp == NULL) || (mux == NULL))
		return -EINVAL;

	std::string s(sdp);
	return pdraw->pdraw->openSdp(s, mux);
}


int pdraw_close(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->close();
}


int pdraw_play(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->play();
}

int pdraw_play_with_speed(
	struct pdraw *pdraw,
	float speed)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->play(speed);
}

int pdraw_pause(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->pause();
}


int pdraw_is_paused(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return (pdraw->pdraw->isPaused()) ? 1 : 0;
}


int pdraw_previous_frame(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->previousFrame();
}


int pdraw_next_frame(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->nextFrame();
}


int pdraw_seek(
	struct pdraw *pdraw,
	int64_t delta,
	int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seek(delta, exact ? true : false);
}


int pdraw_seek_forward(
	struct pdraw *pdraw,
	uint64_t delta,
	int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekForward(delta, exact ? true : false);
}


int pdraw_seek_back(
	struct pdraw *pdraw,
	uint64_t delta,
	int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekBack(delta, exact ? true : false);
}


int pdraw_seek_to(
	struct pdraw *pdraw,
	uint64_t timestamp,
	int exact)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->seekTo(timestamp, exact ? true : false);
}


uint64_t pdraw_get_duration(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return 0;

	return pdraw->pdraw->getDuration();
}


uint64_t pdraw_get_current_time(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return 0;

	return pdraw->pdraw->getCurrentTime();
}


int pdraw_start_renderer(
	struct pdraw *pdraw,
	int windowWidth,
	int windowHeight,
	int renderX,
	int renderY,
	int renderWidth,
	int renderHeight,
	int hud,
	int hmdDistorsionCorrection,
	int headtracking,
	void *uiHandler)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->startRenderer(windowWidth, windowHeight,
		renderX, renderY, renderWidth, renderHeight,
		(hud) ? true : false,
		(hmdDistorsionCorrection) ? true : false,
		(headtracking) ? true : false, uiHandler);
}


int pdraw_stop_renderer(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->stopRenderer();
}


int pdraw_render(
	struct pdraw *pdraw,
	uint64_t lastRenderTime)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->render(lastRenderTime);
}


enum pdraw_session_type pdraw_get_session_type(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return (enum pdraw_session_type)-EINVAL;

	return pdraw->pdraw->getSessionType();
}


char *pdraw_get_self_friendly_name(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getSelfFriendlyName().c_str());
}


int pdraw_set_self_friendly_name(
	struct pdraw *pdraw,
	const char *friendlyName)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string fn(friendlyName);
	pdraw->pdraw->setSelfFriendlyName(fn);
	return 0;
}


char *pdraw_get_self_serial_number(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getSelfSerialNumber().c_str());
}


int pdraw_set_self_serial_number(
	struct pdraw *pdraw,
	const char *serialNumber)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sn(serialNumber);
	pdraw->pdraw->setSelfSerialNumber(sn);
	return 0;
}


char *pdraw_get_self_software_version(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getSelfSoftwareVersion().c_str());
}


int pdraw_set_self_software_version(
	struct pdraw *pdraw,
	const char *softwareVersion)
{
	if (pdraw == NULL)
		return -EINVAL;

	std::string sv(softwareVersion);
	pdraw->pdraw->setSelfSoftwareVersion(sv);
	return 0;
}


int pdraw_is_self_pilot(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return (pdraw->pdraw->isSelfPilot()) ? 1 : 0;
}


int pdraw_set_self_pilot(
	struct pdraw *pdraw,
	int isPilot)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfPilot((isPilot) ? true : false);
	return 0;
}


int pdraw_get_self_location(
	struct pdraw *pdraw,
	struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfLocation(loc);
	return 0;
}


int pdraw_set_self_location(
	struct pdraw *pdraw,
	const struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfLocation(loc);
	return 0;
}


int pdraw_get_self_controller_battery_level(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->getControllerBatteryLevel();
}


int pdraw_set_self_controller_battery_level(
	struct pdraw *pdraw,
	int batteryLevel)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setControllerBatteryLevel(batteryLevel);
	return 0;
}


int pdraw_get_self_controller_orientation_quat(
	struct pdraw *pdraw,
	struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfControllerOrientation(quat);
	return 0;
}


int pdraw_get_self_controller_orientation_euler(
	struct pdraw *pdraw,
	struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfControllerOrientation(euler);
	return 0;
}


int pdraw_set_self_controller_orientation_quat(
	struct pdraw *pdraw,
	const struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfControllerOrientation(quat);
	return 0;
}


int pdraw_set_self_controller_orientation_euler(
	struct pdraw *pdraw,
	const struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfControllerOrientation(euler);
	return 0;
}


int pdraw_get_self_head_orientation_quat(
	struct pdraw *pdraw,
	struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfHeadOrientation(quat);
	return 0;
}


int pdraw_get_self_head_orientation_euler(
	struct pdraw *pdraw,
	struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfHeadOrientation(euler);
	return 0;
}


int pdraw_set_self_head_orientation_quat(
	struct pdraw *pdraw,
	const struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfHeadOrientation(quat);
	return 0;
}


int pdraw_set_self_head_orientation_euler(
	struct pdraw *pdraw,
	const struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfHeadOrientation(euler);
	return 0;
}


int pdraw_get_self_head_ref_orientation_quat(
	struct pdraw *pdraw,
	struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfHeadRefOrientation(quat);
	return 0;
}


int pdraw_get_self_head_ref_orientation_euler(
	struct pdraw *pdraw,
	struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getSelfHeadRefOrientation(euler);
	return 0;
}


int pdraw_set_self_head_ref_orientation_quat(
	struct pdraw *pdraw,
	const struct vmeta_quaternion *quat)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfHeadRefOrientation(quat);
	return 0;
}


int pdraw_set_self_head_ref_orientation_euler(
	struct pdraw *pdraw,
	const struct vmeta_euler *euler)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setSelfHeadRefOrientation(euler);
	return 0;
}


int pdraw_reset_self_head_ref_orientation(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->resetSelfHeadRefOrientation();
	return 0;
}


char *pdraw_get_peer_friendly_name(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerFriendlyName().c_str());
}


char *pdraw_get_peer_maker(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerMaker().c_str());
}


char *pdraw_get_peer_model(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerModel().c_str());
}


char *pdraw_get_peer_model_id(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerModelId().c_str());
}


enum pdraw_drone_model pdraw_get_peer_drone_model(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return (enum pdraw_drone_model)-EINVAL;

	return pdraw->pdraw->getPeerDroneModel();
}


char *pdraw_get_peer_serial_number(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerSerialNumber().c_str());
}


char *pdraw_get_peer_software_version(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerSoftwareVersion().c_str());
}


char *pdraw_get_peer_build_id(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerBuildId().c_str());
}


char *pdraw_get_peer_title(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerTitle().c_str());
}


char *pdraw_get_peer_comment(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerComment().c_str());
}


char *pdraw_get_peer_copyright(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerCopyright().c_str());
}


char *pdraw_get_peer_run_date(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerRunDate().c_str());
}


char *pdraw_get_peer_run_uuid(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerRunUuid().c_str());
}


char *pdraw_get_peer_media_date(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return NULL;

	return strdup(pdraw->pdraw->getPeerMediaDate().c_str());
}


int pdraw_get_peer_takeoff_location(
	struct pdraw *pdraw,
	struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getPeerTakeoffLocation(loc);
	return 0;
}


int pdraw_set_peer_takeoff_location(
	struct pdraw *pdraw,
	const struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setPeerTakeoffLocation(loc);
	return 0;
}


int pdraw_get_peer_home_location(
	struct pdraw *pdraw,
	struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getPeerHomeLocation(loc);
	return 0;
}


int pdraw_set_peer_home_location(
	struct pdraw *pdraw,
	const struct vmeta_location *loc)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setPeerHomeLocation(loc);
	return 0;
}


uint64_t pdraw_get_peer_recording_duration(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return 0;

	return pdraw->pdraw->getPeerRecordingDuration();
}


int pdraw_set_peer_recording_duration(
	struct pdraw *pdraw,
	uint64_t duration)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setPeerRecordingDuration(duration);
	return 0;
}


int pdraw_get_camera_orientation_for_headtracking(
	struct pdraw *pdraw,
	float *pan,
	float *tilt)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getCameraOrientationForHeadtracking(pan, tilt);
	return 0;
}


int pdraw_get_media_count(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->getMediaCount();
}


int pdraw_get_media_info(
	struct pdraw *pdraw,
	unsigned int index,
	struct pdraw_media_info *info)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->getMediaInfo(index, info);
}


void *pdraw_add_video_frame_filter_callback(
	struct pdraw *pdraw,
	unsigned int mediaId,
	pdraw_video_frame_filter_callback_t cb,
	void *userPtr)
{
	if (pdraw == NULL)
		return NULL;

	return pdraw->pdraw->addVideoFrameFilterCallback(
		mediaId, cb, userPtr);
}


int pdraw_remove_video_frame_filter_callback(
	struct pdraw *pdraw,
	unsigned int mediaId,
	void *filterCtx)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->removeVideoFrameFilterCallback(
		mediaId, filterCtx);
}


void *pdraw_add_video_frame_producer(
	struct pdraw *pdraw,
	unsigned int mediaId,
	int frameByFrame)
{
	if (pdraw == NULL)
		return NULL;

	return pdraw->pdraw->addVideoFrameProducer(
		mediaId, (frameByFrame) ? true : false);
}


int pdraw_remove_video_frame_producer(
	struct pdraw *pdraw,
	void *producerCtx)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->removeVideoFrameProducer(producerCtx);
}


int pdraw_get_producer_last_frame(
	struct pdraw *pdraw,
	void *producerCtx,
	struct pdraw_video_frame *frame,
	int timeout)
{
	if (pdraw == NULL)
		return -EINVAL;

	return pdraw->pdraw->getProducerLastFrame(
		producerCtx, frame, timeout);
}


float pdraw_get_controller_radar_angle_setting(
	struct pdraw *pdraw)
{
	if (pdraw == NULL)
		return (float)-EINVAL;

	return pdraw->pdraw->getControllerRadarAngleSetting();
}


int pdraw_set_controller_radar_angle_setting(
	struct pdraw *pdraw,
	float angle)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setControllerRadarAngleSetting(angle);
	return 0;
}


int pdraw_get_display_screen_settings(
	struct pdraw *pdraw,
	float *xdpi,
	float *ydpi,
	float *deviceMargin)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getDisplayScreenSettings(xdpi, ydpi, deviceMargin);
	return 0;
}


int pdraw_set_display_screen_settings(
	struct pdraw *pdraw,
	float xdpi,
	float ydpi,
	float deviceMargin)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setDisplayScreenSettings(xdpi, ydpi, deviceMargin);
	return 0;
}


int pdraw_get_hmd_distorsion_correction_settings(
	struct pdraw *pdraw,
	enum pdraw_hmd_model *hmdModel,
	float *ipd,
	float *scale,
	float *panH,
	float *panV)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->getHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
	return 0;
}


int pdraw_set_hmd_distorsion_correction_settings(
	struct pdraw *pdraw,
	enum pdraw_hmd_model hmdModel,
	float ipd,
	float scale,
	float panH,
	float panV)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setHmdDistorsionCorrectionSettings(
		hmdModel, ipd, scale, panH, panV);
	return 0;
}


int pdraw_set_jni_env(
	struct pdraw *pdraw,
	void *jniEnv)
{
	if (pdraw == NULL)
		return -EINVAL;

	pdraw->pdraw->setJniEnv(jniEnv);
	return 0;
}
