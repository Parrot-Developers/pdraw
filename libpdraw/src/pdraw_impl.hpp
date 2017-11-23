/**
 * Parrot Drones Awesome Video Viewer Library
 * Interface implementation
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

#ifndef _PDRAW_IMPL_HPP_
#define _PDRAW_IMPL_HPP_

#include <vector>

#include <pdraw/pdraw.hpp>

#include "pdraw_settings.hpp"
#include "pdraw_session.hpp"


namespace Pdraw
{


class PdrawImpl : public IPdraw
{
public:

    PdrawImpl(void);

    ~PdrawImpl(void);

    int open
            (const std::string &url);

    int open
            (const std::string &url,
             const std::string &ifaceAddr);

    int open
            (const std::string &srcAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort,
             int qosMode);

    int open
            (void *muxContext);

    int openSdp
            (const std::string &sdp,
             const std::string &ifaceAddr);

    int play(float speed = 1.0f);

    int pause(void);

    bool isPaused(void);

    int previousFrame(void);

    int nextFrame(void);

    int stop(void);

    int seekTo
            (uint64_t timestamp, bool exact = false);

    int seekForward
            (uint64_t delta, bool exact = false);

    int seekBack
            (uint64_t delta, bool exact = false);

    uint64_t getDuration();

    uint64_t getCurrentTime();

    int startRecorder
            (const std::string &fileName);

    int stopRecorder(void);

    int startResender
            (const std::string &dstAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort);

    int stopResender(void);

    int startRenderer
            (int windowWidth,
             int windowHeight,
             int renderX,
             int renderY,
             int renderWidth,
             int renderHeight,
             bool hmdDistorsionCorrection,
             bool headtracking,
             void *uiHandler);

    int stopRenderer(void);

    int render
            (uint64_t lastRenderTime);

    enum pdraw_session_type getSessionType(void);

    std::string& getSelfFriendlyName(void);
    void setSelfFriendlyName
            (const std::string &friendlyName);

    std::string& getSelfSerialNumber(void);
    void setSelfSerialNumber
            (const std::string &serialNumber);

    std::string& getSelfSoftwareVersion(void);
    void setSelfSoftwareVersion
            (const std::string &softwareVersion);

    bool isSelfPilot(void);
    void setSelfPilot
            (bool isPilot);

    void getSelfLocation
            (struct vmeta_location *loc);
    void setSelfLocation
            (const struct vmeta_location *loc);

    int getControllerBatteryLevel();
    void setControllerBatteryLevel(int batteryLevel);

    void getSelfControllerOrientation
            (struct vmeta_quaternion *quat);
    void getSelfControllerOrientation
            (struct vmeta_euler *euler);
    void setSelfControllerOrientation
            (const struct vmeta_quaternion *quat);
    void setSelfControllerOrientation
            (const struct vmeta_euler *euler);

    void getSelfHeadOrientation
            (struct vmeta_quaternion *quat);
    void getSelfHeadOrientation
            (struct vmeta_euler *euler);
    void setSelfHeadOrientation
            (const struct vmeta_quaternion *quat);
    void setSelfHeadOrientation
            (const struct vmeta_euler *euler);

    void getSelfHeadRefOrientation
            (struct vmeta_quaternion *quat);
    void getSelfHeadRefOrientation
            (struct vmeta_euler *euler);
    void setSelfHeadRefOrientation
            (const struct vmeta_quaternion *quat);
    void setSelfHeadRefOrientation
            (const struct vmeta_euler *euler);
    void resetSelfHeadRefOrientation(void);

    std::string& getPeerFriendlyName(void);

    std::string& getPeerMaker(void);

    std::string& getPeerModel(void);

    std::string& getPeerModelId(void);

    enum pdraw_drone_model getPeerDroneModel(void);

    std::string& getPeerSerialNumber(void);

    std::string& getPeerSoftwareVersion(void);

    std::string& getPeerBuildId(void);

    std::string& getPeerTitle(void);

    std::string& getPeerComment(void);

    std::string& getPeerCopyright(void);

    std::string& getPeerRunDate(void);

    std::string& getPeerRunUuid(void);

    std::string& getPeerMediaDate(void);

    void getPeerTakeoffLocation
            (struct vmeta_location *loc);
    void setPeerTakeoffLocation
            (const struct vmeta_location *loc);

    void getPeerHomeLocation
            (struct vmeta_location *loc);
    void setPeerHomeLocation
            (const struct vmeta_location *loc);

    uint64_t getPeerRecordingDuration(void);
    void setPeerRecordingDuration(uint64_t duration);

    void getCameraOrientationForHeadtracking(float *pan, float *tilt);

    int getMediaCount();

    int getMediaInfo(unsigned int index, struct pdraw_media_info *info);

    void *addVideoFrameFilterCallback(unsigned int mediaId, pdraw_video_frame_filter_callback_t cb, void *userPtr);

    int removeVideoFrameFilterCallback(unsigned int mediaId, void *filterCtx);

    void *addVideoFrameProducer(unsigned int mediaId, bool frameByFrame = false);

    int removeVideoFrameProducer(void *producerCtx);

    /*
     * get last frame
     *
     * timeout: time in microseconds to wait for a frame
     *  0: don't wait
     * -1: wait forever
     * >0: wait time
     */
    int getProducerLastFrame(void *producerCtx, struct pdraw_video_frame *frame, int timeout = 0);

    float getControllerRadarAngleSetting(void);
    void setControllerRadarAngleSetting(float angle);

    void getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin);
    void setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin);

    void getHmdDistorsionCorrectionSettings(enum pdraw_hmd_model *hmdModel, float *ipd, float *scale, float *panH, float *panV);
    void setHmdDistorsionCorrectionSettings(enum pdraw_hmd_model hmdModel, float ipd, float scale, float panH, float panV);

    void setJniEnv(void *jniEnv);

    inline static IPdraw *create(void)
    {
        return new PdrawImpl();
    }

    inline static void release
            (IPdraw *pdraw)
    {
        delete pdraw;
    }

private:

    int openWithDemux();

    Settings mSettings;
    Session mSession;
};

}

#endif /* !_PDRAW_IMPL_HPP_ */
