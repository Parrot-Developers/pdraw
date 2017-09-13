/**
 * @file pdraw.hpp
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

#ifndef _PDRAW_HPP_
#define _PDRAW_HPP_

#include <inttypes.h>
#include <string>
#include "pdraw_defs.h"

namespace Pdraw
{


class IPdraw
{
public:

    virtual ~IPdraw(void) {};

    virtual int open
            (const std::string &url) = 0;

    virtual int open
            (const std::string &url,
             const std::string &ifaceAddr) = 0;

    virtual int open
            (const std::string &srcAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort,
             int qosMode) = 0;

    virtual int open
            (void *muxContext) = 0;

    virtual int openSdp
            (const std::string &sdp,
             const std::string &ifaceAddr) = 0;

    virtual int start(void) = 0;

    virtual int pause(void) = 0;

    virtual bool isPaused(void) = 0;

    virtual int stop(void) = 0;

    virtual int seekTo
            (uint64_t timestamp) = 0;

    virtual int seekForward
            (uint64_t delta) = 0;

    virtual int seekBack
            (uint64_t delta) = 0;

    virtual uint64_t getDuration() = 0;

    virtual uint64_t getCurrentTime() = 0;

    virtual int startRecorder
            (const std::string &fileName) = 0;

    virtual int stopRecorder(void) = 0;

    virtual int startResender
            (const std::string &dstAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort) = 0;

    virtual int stopResender(void) = 0;

    virtual int startRenderer
            (int windowWidth,
             int windowHeight,
             int renderX,
             int renderY,
             int renderWidth,
             int renderHeight,
             bool hmdDistorsionCorrection,
             bool headtracking,
             void *uiHandler) = 0;

    virtual int stopRenderer(void) = 0;

    virtual int render
            (uint64_t lastRenderTime) = 0;

    virtual pdraw_session_type_t getSessionType(void) = 0;

    virtual std::string& getSelfFriendlyName(void) = 0;
    virtual void setSelfFriendlyName
            (const std::string &friendlyName) = 0;

    virtual std::string& getSelfSerialNumber(void) = 0;
    virtual void setSelfSerialNumber
            (const std::string &serialNumber) = 0;

    virtual std::string& getSelfSoftwareVersion(void) = 0;
    virtual void setSelfSoftwareVersion
            (const std::string &softwareVersion) = 0;

    virtual bool isSelfPilot(void) = 0;
    virtual void setSelfPilot
            (bool isPilot) = 0;

    virtual void getSelfLocation
            (pdraw_location_t *loc) = 0;
    virtual void setSelfLocation
            (const pdraw_location_t *loc) = 0;

    virtual int getControllerBatteryLevel(void) = 0;
    virtual void setControllerBatteryLevel(int batteryLevel) = 0;

    virtual void getSelfControllerOrientation
            (pdraw_quaternion_t *quat) = 0;
    virtual void getSelfControllerOrientation
            (pdraw_euler_t *euler) = 0;
    virtual void setSelfControllerOrientation
            (const pdraw_quaternion_t *quat) = 0;
    virtual void setSelfControllerOrientation
            (const pdraw_euler_t *euler) = 0;

    virtual void getSelfHeadOrientation
            (pdraw_quaternion_t *quat) = 0;
    virtual void getSelfHeadOrientation
            (pdraw_euler_t *euler) = 0;
    virtual void setSelfHeadOrientation
            (const pdraw_quaternion_t *quat) = 0;
    virtual void setSelfHeadOrientation
            (const pdraw_euler_t *euler) = 0;

    virtual void getSelfHeadRefOrientation
            (pdraw_quaternion_t *quat) = 0;
    virtual void getSelfHeadRefOrientation
            (pdraw_euler_t *euler) = 0;
    virtual void setSelfHeadRefOrientation
            (const pdraw_quaternion_t *quat) = 0;
    virtual void setSelfHeadRefOrientation
            (const pdraw_euler_t *euler) = 0;
    virtual void resetSelfHeadRefOrientation(void) = 0;

    virtual std::string& getPeerFriendlyName(void) = 0;

    virtual std::string& getPeerMaker(void) = 0;

    virtual std::string& getPeerModel(void) = 0;

    virtual std::string& getPeerModelId(void) = 0;

    virtual pdraw_drone_model_t getPeerDroneModel(void) = 0;

    virtual std::string& getPeerSerialNumber(void) = 0;

    virtual std::string& getPeerSoftwareVersion(void) = 0;

    virtual std::string& getPeerBuildId(void) = 0;

    virtual std::string& getPeerTitle(void) = 0;

    virtual std::string& getPeerComment(void) = 0;

    virtual std::string& getPeerCopyright(void) = 0;

    virtual std::string& getPeerRunDate(void) = 0;

    virtual std::string& getPeerRunUuid(void) = 0;

    virtual std::string& getPeerMediaDate(void) = 0;

    virtual void getPeerTakeoffLocation
            (pdraw_location_t *loc) = 0;
    virtual void setPeerTakeoffLocation
            (const pdraw_location_t *loc) = 0;

    virtual void getPeerHomeLocation
            (pdraw_location_t *loc) = 0;
    virtual void setPeerHomeLocation
            (const pdraw_location_t *loc) = 0;

    virtual uint64_t getPeerRecordingDuration(void) = 0;
    virtual void setPeerRecordingDuration(uint64_t duration) = 0;

    virtual void getCameraOrientationForHeadtracking(float *pan, float *tilt) = 0;

    virtual int getMediaCount(void) = 0;

    virtual int getMediaInfo(unsigned int index, pdraw_media_info_t *info) = 0;

    virtual void *addVideoFrameFilterCallback(unsigned int mediaId, pdraw_video_frame_filter_callback_t cb, void *userPtr) = 0;

    virtual int removeVideoFrameFilterCallback(unsigned int mediaId, void *filterCtx) = 0;

    virtual void *addVideoFrameProducer(unsigned int mediaId) = 0;

    virtual int removeVideoFrameProducer(void *producerCtx) = 0;

    /*
     * get last frame
     *
     * waitUs : time in microseconds to wait a frame
     *  0: don't wait
     * -1: wait forever
     * >0: wait time
     */
    virtual int getProducerLastFrame(void *producerCtx, pdraw_video_frame_t *frame, long waitUs = 0) = 0;

    virtual float getControllerRadarAngleSetting(void) = 0;
    virtual void setControllerRadarAngleSetting(float angle) = 0;

    virtual void getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin) = 0;
    virtual void setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin) = 0;

    virtual void getHmdDistorsionCorrectionSettings(pdraw_hmd_model_t *hmdModel, float *ipd, float *scale, float *panH, float *panV) = 0;
    virtual void setHmdDistorsionCorrectionSettings(pdraw_hmd_model_t hmdModel, float ipd, float scale, float panH, float panV) = 0;
};

IPdraw *createPdraw();

}

#endif /* !_PDRAW_HPP_ */
