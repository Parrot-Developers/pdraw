/**
 * @file pdraw.h
 * @brief Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_H_
#define _PDRAW_H_

#ifdef __cplusplus
extern "C"  {
#endif /* __cplusplus */

#include <inttypes.h>
#include "pdraw_defs.h"


struct pdraw;


struct pdraw *pdraw_new(void);


int pdraw_destroy
        (struct pdraw *pdraw);


int pdraw_open_url
        (struct pdraw *pdraw,
         const char *url);


int pdraw_open_url_mcast
        (struct pdraw *pdraw,
         const char *url,
         const char *ifaceAddr);


int pdraw_open_single_stream
        (struct pdraw *pdraw,
         const char *srcAddr,
         const char *ifaceAddr,
         int srcStreamPort,
         int srcControlPort,
         int dstStreamPort,
         int dstControlPort,
         int qosMode);


int pdraw_open_mux
        (struct pdraw *pdraw,
         void *muxContext);


int pdraw_open_sdp
        (struct pdraw *pdraw,
         const char *sdp,
         const char *ifaceAddr);


int pdraw_start
        (struct pdraw *pdraw);


int pdraw_pause
        (struct pdraw *pdraw);


int pdraw_is_paused
        (struct pdraw *pdraw);


int pdraw_stop
        (struct pdraw *pdraw);


int pdraw_seek_to
        (struct pdraw *pdraw,
         uint64_t timestamp);


int pdraw_seek_forward
        (struct pdraw *pdraw,
         uint64_t delta);


int pdraw_seek_back
        (struct pdraw *pdraw,
         uint64_t delta);


int pdraw_start_recorder
        (struct pdraw *pdraw,
         const char *fileName);


int pdraw_stop_recorder
        (struct pdraw *pdraw);


int pdraw_start_resender
        (struct pdraw *pdraw,
         const char *dstAddr,
         const char *ifaceAddr,
         int srcStreamPort,
         int srcControlPort,
         int dstStreamPort,
         int dstControlPort);


int pdraw_stop_resender
        (struct pdraw *pdraw);


int pdraw_start_renderer
        (struct pdraw *pdraw,
         int windowWidth,
         int windowHeight,
         int renderX,
         int renderY,
         int renderWidth,
         int renderHeight,
         int hmdDistorsionCorrection,
         int headtracking,
         void *uiHandler);


int pdraw_stop_renderer
        (struct pdraw *pdraw);


int pdraw_render
        (struct pdraw *pdraw,
         uint64_t lastRenderTime);


pdraw_session_type_t pdraw_get_session_type
        (struct pdraw *pdraw);


const char *pdraw_get_self_friendly_name
        (struct pdraw *pdraw);


int pdraw_set_self_friendly_name
        (struct pdraw *pdraw,
         const char *friendlyName);


const char *pdraw_get_self_serial_number
        (struct pdraw *pdraw);


int pdraw_set_self_serial_number
        (struct pdraw *pdraw,
         const char *serialNumber);


const char *pdraw_get_self_software_version
        (struct pdraw *pdraw);


int pdraw_set_self_software_version
        (struct pdraw *pdraw,
         const char *softwareVersion);


int pdraw_is_self_pilot
        (struct pdraw *pdraw);


int pdraw_set_self_pilot
        (struct pdraw *pdraw,
         int isPilot);


int pdraw_get_self_location
        (struct pdraw *pdraw,
         pdraw_location_t *loc);


int pdraw_set_self_location
        (struct pdraw *pdraw,
         const pdraw_location_t *loc);


int pdraw_get_self_controller_battery_level
        (struct pdraw *pdraw);


int pdraw_set_self_controller_battery_level
        (struct pdraw *pdraw,
         int batteryLevel);


int pdraw_get_self_controller_orientation_quat
        (struct pdraw *pdraw,
         pdraw_quaternion_t *quat);


int pdraw_get_self_controller_orientation_euler
        (struct pdraw *pdraw,
         pdraw_euler_t *euler);


int pdraw_set_self_controller_orientation_quat
        (struct pdraw *pdraw,
         const pdraw_quaternion_t *quat);


int pdraw_set_self_controller_orientation_euler
        (struct pdraw *pdraw,
         const pdraw_euler_t *euler);


int pdraw_get_self_head_orientation_quat
        (struct pdraw *pdraw,
         pdraw_quaternion_t *quat);


int pdraw_get_self_head_orientation_euler
        (struct pdraw *pdraw,
         pdraw_euler_t *euler);


int pdraw_set_self_head_orientation_quat
        (struct pdraw *pdraw,
         const pdraw_quaternion_t *quat);


int pdraw_set_self_head_orientation_euler
        (struct pdraw *pdraw,
         const pdraw_euler_t *euler);


int pdraw_get_self_head_ref_orientation_quat
        (struct pdraw *pdraw,
         pdraw_quaternion_t *quat);


int pdraw_get_self_head_ref_orientation_euler
        (struct pdraw *pdraw,
         pdraw_euler_t *euler);


int pdraw_set_self_head_ref_orientation_quat
        (struct pdraw *pdraw,
         const pdraw_quaternion_t *quat);


int pdraw_set_self_head_ref_orientation_euler
        (struct pdraw *pdraw,
         const pdraw_euler_t *euler);


int pdraw_reset_self_head_ref_orientation
        (struct pdraw *pdraw);


const char *pdraw_get_peer_friendly_name
        (struct pdraw *pdraw);


const char *pdraw_get_peer_maker
        (struct pdraw *pdraw);


const char *pdraw_get_peer_model
        (struct pdraw *pdraw);


const char *pdraw_get_peer_model_id
        (struct pdraw *pdraw);


pdraw_drone_model_t pdraw_get_peer_drone_model
        (struct pdraw *pdraw);


const char *pdraw_get_peer_serial_number
        (struct pdraw *pdraw);


const char *pdraw_get_peer_software_version
        (struct pdraw *pdraw);


const char *pdraw_get_peer_build_id
        (struct pdraw *pdraw);


const char *pdraw_get_peer_title
        (struct pdraw *pdraw);


const char *pdraw_get_peer_comment
        (struct pdraw *pdraw);


const char *pdraw_get_peer_copyright
        (struct pdraw *pdraw);


const char *pdraw_get_peer_run_date
        (struct pdraw *pdraw);


const char *pdraw_get_peer_run_uuid
        (struct pdraw *pdraw);


const char *pdraw_get_peer_media_date
        (struct pdraw *pdraw);


int pdraw_get_peer_takeoff_location
        (struct pdraw *pdraw,
         pdraw_location_t *loc);


int pdraw_set_peer_takeoff_location
        (struct pdraw *pdraw,
         const pdraw_location_t *loc);


int pdraw_get_peer_home_location
        (struct pdraw *pdraw,
         pdraw_location_t *loc);


int pdraw_set_peer_home_location
        (struct pdraw *pdraw,
         const pdraw_location_t *loc);


uint64_t pdraw_get_peer_recording_duration
        (struct pdraw *pdraw);


int pdraw_set_peer_recording_duration
        (struct pdraw *pdraw,
         uint64_t duration);


int pdraw_get_camera_orientation_for_headtracking
        (struct pdraw *pdraw,
         float *pan,
         float *tilt);


int pdraw_get_media_count
        (struct pdraw *pdraw);


int pdraw_get_media_info
        (struct pdraw *pdraw,
         unsigned int index,
         pdraw_media_info_t *info);


void *pdraw_add_video_frame_filter_callback
        (struct pdraw *pdraw,
         unsigned int mediaId,
         pdraw_video_frame_filter_callback_t cb,
         void *userPtr);


int pdraw_remove_video_frame_filter_callback
        (struct pdraw *pdraw,
         unsigned int mediaId,
         void *filterCtx);


void *pdraw_add_video_frame_producer
        (struct pdraw *pdraw,
         unsigned int mediaId);


int pdraw_remove_video_frame_producer
        (struct pdraw *pdraw,
         void *producerCtx);


int pdraw_get_producer_last_frame
        (struct pdraw *pdraw,
         void *producerCtx,
         pdraw_video_frame_t *frame);

float pdraw_get_controller_radar_angle_setting
        (struct pdraw *pdraw);

int pdraw_set_controller_radar_angle_setting
        (struct pdraw *pdraw,
         float angle);

int pdraw_get_display_screen_settings
        (struct pdraw *pdraw,
         float *xdpi,
         float *ydpi,
         float *deviceMargin);

int pdraw_set_display_screen_settings
        (struct pdraw *pdraw,
         float xdpi,
         float ydpi,
         float deviceMargin);

int pdraw_get_hmd_distorsion_correction_settings
        (struct pdraw *pdraw,
         pdraw_hmd_model_t *hmdModel,
         float *ipd,
         float *scale,
         float *panH,
         float *panV);

int pdraw_set_hmd_distorsion_correction_settings
        (struct pdraw *pdraw,
         pdraw_hmd_model_t hmdModel,
         float ipd,
         float scale,
         float panH,
         float panV);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_PDRAW_H_ */
