/**
 * @file pdraw_wrapper.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - C wrapper functions
 * @date 05/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pdraw/pdraw.h>
#include "pdraw_impl.hpp"

#include <errno.h>

using namespace Pdraw;


static IPdraw *toPdraw(struct pdraw *ptr)
{
    return (IPdraw*)ptr;
}


static struct pdraw *fromPdraw(IPdraw *ptr)
{
    return (struct pdraw*)ptr;
}


struct pdraw *pdraw_new()
{
    return fromPdraw(PdrawImpl::create());
}


int pdraw_destroy(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    PdrawImpl::release(toPdraw(pdraw));
    return 0;
}


int pdraw_open_url(struct pdraw *pdraw, const char *url)
{
    if ((pdraw == NULL) || (url == NULL))
    {
        return -EINVAL;
    }
    std::string u(url);
    return toPdraw(pdraw)->open(u);
}


int pdraw_open_url_mcast(struct pdraw *pdraw, const char *url, const char *ifaceAddr)
{
    if ((pdraw == NULL) || (url == NULL))
    {
        return -EINVAL;
    }
    std::string u(url);
    std::string i(ifaceAddr);
    return toPdraw(pdraw)->open(u, i);
}


int pdraw_open_single_stream(struct pdraw *pdraw, const char *srcAddr, const char *ifaceAddr,
                             int srcStreamPort, int srcControlPort,
                             int dstStreamPort, int dstControlPort, int qosMode)
{
    if ((pdraw == NULL) || (srcAddr == NULL))
    {
        return -EINVAL;
    }
    std::string ip(srcAddr);
    std::string iface(ifaceAddr);
    return toPdraw(pdraw)->open(ip, iface, srcStreamPort, srcControlPort,
                                dstStreamPort, dstControlPort, qosMode);
}


int pdraw_open_mux(struct pdraw *pdraw, void *muxContext)
{
    if ((pdraw == NULL) || (muxContext == NULL))
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->open(muxContext);
}


int pdraw_open_sdp(struct pdraw *pdraw, const char *sdp, const char *ifaceAddr)
{
    if ((pdraw == NULL) || (sdp == NULL))
    {
        return -EINVAL;
    }
    std::string s(sdp);
    std::string i(ifaceAddr);
    return toPdraw(pdraw)->openSdp(s, i);
}


int pdraw_start(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->start();
}


int pdraw_pause(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->pause();
}


int pdraw_is_paused(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return (toPdraw(pdraw)->isPaused()) ? 1 : 0;
}


int pdraw_stop(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stop();
}


int pdraw_seek_to(struct pdraw *pdraw, uint64_t timestamp)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekTo(timestamp);
}


int pdraw_seek_forward(struct pdraw *pdraw, uint64_t delta)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekForward(delta);
}


int pdraw_seek_back(struct pdraw *pdraw, uint64_t delta)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekBack(delta);
}


uint64_t pdraw_get_duration(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return 0;
    }
    return toPdraw(pdraw)->getDuration();
}


uint64_t pdraw_get_current_time(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return 0;
    }
    return toPdraw(pdraw)->getCurrentTime();
}


int pdraw_start_recorder(struct pdraw *pdraw, const char *fileName)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string fn(fileName);
    return toPdraw(pdraw)->startRecorder(fn);
}


int pdraw_stop_recorder(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopRecorder();
}


int pdraw_start_resender(struct pdraw *pdraw, const char *dstAddr, const char *ifaceAddr,
                         int srcStreamPort, int srcControlPort,
                         int dstStreamPort, int dstControlPort)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string ip(dstAddr);
    std::string iface(ifaceAddr);
    return toPdraw(pdraw)->startResender(ip, iface,
                                         srcStreamPort, srcControlPort,
                                         dstStreamPort, dstControlPort);
}


int pdraw_stop_resender(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopResender();
}


int pdraw_start_renderer(struct pdraw *pdraw,
                         int windowWidth, int windowHeight,
                         int renderX, int renderY,
                         int renderWidth, int renderHeight,
                         int hmdDistorsionCorrection,
                         int headtracking,
                         void *uiHandler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->startRenderer(windowWidth, windowHeight,
                                         renderX, renderY,
                                         renderWidth, renderHeight,
                                         (hmdDistorsionCorrection) ? true : false,
                                         (headtracking) ? true : false,
                                         uiHandler);
}


int pdraw_stop_renderer(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopRenderer();
}


int pdraw_render(struct pdraw *pdraw, uint64_t lastRenderTime)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->render(lastRenderTime);
}


pdraw_session_type_t pdraw_get_session_type(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return (pdraw_session_type_t)-EINVAL;
    }
    return toPdraw(pdraw)->getSessionType();
}


const char *pdraw_get_self_friendly_name(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getSelfFriendlyName().c_str();
}


int pdraw_set_self_friendly_name(struct pdraw *pdraw, const char *friendlyName)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string fn(friendlyName);
    toPdraw(pdraw)->setSelfFriendlyName(fn);
    return 0;
}


const char *pdraw_get_self_serial_number(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getSelfSerialNumber().c_str();
}


int pdraw_set_self_serial_number(struct pdraw *pdraw, const char *serialNumber)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string sn(serialNumber);
    toPdraw(pdraw)->setSelfSerialNumber(sn);
    return 0;
}


const char *pdraw_get_self_software_version(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getSelfSoftwareVersion().c_str();
}


int pdraw_set_self_software_version(struct pdraw *pdraw, const char *softwareVersion)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string sv(softwareVersion);
    toPdraw(pdraw)->setSelfSoftwareVersion(sv);
    return 0;
}


int pdraw_is_self_pilot(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return (toPdraw(pdraw)->isSelfPilot()) ? 1 : 0;
}


int pdraw_set_self_pilot(struct pdraw *pdraw, int isPilot)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfPilot((isPilot) ? true : false);
    return 0;
}


int pdraw_get_self_location(struct pdraw *pdraw, pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfLocation(loc);
    return 0;
}


int pdraw_set_self_location(struct pdraw *pdraw, const pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfLocation(loc);
    return 0;
}


int pdraw_get_self_controller_battery_level(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getControllerBatteryLevel();
}


int pdraw_set_self_controller_battery_level(struct pdraw *pdraw, int batteryLevel)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setControllerBatteryLevel(batteryLevel);
    return 0;
}


int pdraw_get_self_controller_orientation_quat(struct pdraw *pdraw, pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfControllerOrientation(quat);
    return 0;
}


int pdraw_get_self_controller_orientation_euler(struct pdraw *pdraw, pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfControllerOrientation(euler);
    return 0;
}


int pdraw_set_self_controller_orientation_quat(struct pdraw *pdraw, const pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfControllerOrientation(quat);
    return 0;
}


int pdraw_set_self_controller_orientation_euler(struct pdraw *pdraw, const pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfControllerOrientation(euler);
    return 0;
}


int pdraw_get_self_head_orientation_quat(struct pdraw *pdraw, pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfHeadOrientation(quat);
    return 0;
}


int pdraw_get_self_head_orientation_euler(struct pdraw *pdraw, pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfHeadOrientation(euler);
    return 0;
}


int pdraw_set_self_head_orientation_quat(struct pdraw *pdraw, const pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfHeadOrientation(quat);
    return 0;
}


int pdraw_set_self_head_orientation_euler(struct pdraw *pdraw, const pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfHeadOrientation(euler);
    return 0;
}


int pdraw_get_self_head_ref_orientation_quat(struct pdraw *pdraw, pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfHeadRefOrientation(quat);
    return 0;
}


int pdraw_get_self_head_ref_orientation_euler(struct pdraw *pdraw, pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getSelfHeadRefOrientation(euler);
    return 0;
}


int pdraw_set_self_head_ref_orientation_quat(struct pdraw *pdraw, const pdraw_quaternion_t *quat)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfHeadRefOrientation(quat);
    return 0;
}


int pdraw_set_self_head_ref_orientation_euler(struct pdraw *pdraw, const pdraw_euler_t *euler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setSelfHeadRefOrientation(euler);
    return 0;
}


int pdraw_reset_self_head_ref_orientation(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->resetSelfHeadRefOrientation();
    return 0;
}


const char *pdraw_get_peer_friendly_name(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerFriendlyName().c_str();
}


const char *pdraw_get_peer_maker(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerMaker().c_str();
}


const char *pdraw_get_peer_model(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerModel().c_str();
}


const char *pdraw_get_peer_model_id(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerModelId().c_str();
}


pdraw_drone_model_t pdraw_get_peer_drone_model(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return (pdraw_drone_model_t)-EINVAL;
    }
    return toPdraw(pdraw)->getPeerDroneModel();
}


const char *pdraw_get_peer_serial_number(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerSerialNumber().c_str();
}


const char *pdraw_get_peer_software_version(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerSoftwareVersion().c_str();
}


const char *pdraw_get_peer_build_id(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerBuildId().c_str();
}


const char *pdraw_get_peer_title(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerTitle().c_str();
}


const char *pdraw_get_peer_comment(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerComment().c_str();
}


const char *pdraw_get_peer_copyright(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerCopyright().c_str();
}


const char *pdraw_get_peer_run_date(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerRunDate().c_str();
}


const char *pdraw_get_peer_run_uuid(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerRunUuid().c_str();
}


const char *pdraw_get_peer_media_date(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->getPeerMediaDate().c_str();
}


int pdraw_get_peer_takeoff_location(struct pdraw *pdraw, pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getPeerTakeoffLocation(loc);
    return 0;
}


int pdraw_set_peer_takeoff_location(struct pdraw *pdraw, const pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setPeerTakeoffLocation(loc);
    return 0;
}


int pdraw_get_peer_home_location(struct pdraw *pdraw, pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getPeerHomeLocation(loc);
    return 0;
}


int pdraw_set_peer_home_location(struct pdraw *pdraw, const pdraw_location_t *loc)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setPeerHomeLocation(loc);
    return 0;
}


uint64_t pdraw_get_peer_recording_duration(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return 0;
    }
    return toPdraw(pdraw)->getPeerRecordingDuration();
}


int pdraw_set_peer_recording_duration(struct pdraw *pdraw, uint64_t duration)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setPeerRecordingDuration(duration);
    return 0;
}


int pdraw_get_camera_orientation_for_headtracking
        (struct pdraw *pdraw,
         float *pan,
         float *tilt)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getCameraOrientationForHeadtracking(pan, tilt);
    return 0;
}


int pdraw_get_media_count(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getMediaCount();
}


int pdraw_get_media_info(struct pdraw *pdraw, unsigned int index, pdraw_media_info_t *info)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getMediaInfo(index, info);
}


void *pdraw_add_video_frame_filter_callback(struct pdraw *pdraw, unsigned int mediaId,
                                            pdraw_video_frame_filter_callback_t cb, void *userPtr)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->addVideoFrameFilterCallback(mediaId, cb, userPtr);
}


int pdraw_remove_video_frame_filter_callback(struct pdraw *pdraw, unsigned int mediaId, void *filterCtx)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->removeVideoFrameFilterCallback(mediaId, filterCtx);
}


void *pdraw_add_video_frame_producer(struct pdraw *pdraw, unsigned int mediaId)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->addVideoFrameProducer(mediaId);
}


int pdraw_remove_video_frame_producer(struct pdraw *pdraw, void *producerCtx)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->removeVideoFrameProducer(producerCtx);
}


int pdraw_get_producer_last_frame(struct pdraw *pdraw, void *producerCtx, pdraw_video_frame_t *frame, int timeout)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getProducerLastFrame(producerCtx, frame, timeout);
}


float pdraw_get_controller_radar_angle_setting(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return (float)-EINVAL;
    }
    return toPdraw(pdraw)->getControllerRadarAngleSetting();
}


int pdraw_set_controller_radar_angle_setting(struct pdraw *pdraw, float angle)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setControllerRadarAngleSetting(angle);
    return 0;
}


int pdraw_get_display_screen_settings
        (struct pdraw *pdraw,
         float *xdpi,
         float *ydpi,
         float *deviceMargin)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getDisplayScreenSettings(xdpi, ydpi, deviceMargin);
    return 0;
}


int pdraw_set_display_screen_settings
        (struct pdraw *pdraw,
         float xdpi,
         float ydpi,
         float deviceMargin)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setDisplayScreenSettings(xdpi, ydpi, deviceMargin);
    return 0;
}


int pdraw_get_hmd_distorsion_correction_settings
        (struct pdraw *pdraw,
         pdraw_hmd_model_t *hmdModel,
         float *ipd,
         float *scale,
         float *panH,
         float *panV)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->getHmdDistorsionCorrectionSettings(hmdModel, ipd, scale, panH, panV);
    return 0;
}


int pdraw_set_hmd_distorsion_correction_settings
        (struct pdraw *pdraw,
         pdraw_hmd_model_t hmdModel,
         float ipd,
         float scale,
         float panH,
         float panV)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    toPdraw(pdraw)->setHmdDistorsionCorrectionSettings(hmdModel, ipd, scale, panH, panV);
    return 0;
}
