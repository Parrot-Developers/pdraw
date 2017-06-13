/**
 * @file pdraw_impl.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - interface implementation
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
            (const std::string &srcAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort,
             int qosMode);

    int open
            (void *muxContext);

    int start(void);

    int pause(void);

    bool isPaused(void);

    int stop(void);

    int seekTo
            (uint64_t timestamp);

    int seekForward
            (uint64_t delta);

    int seekBack
            (uint64_t delta);

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
            (int timeout);

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
            (location_t *loc);
    void setSelfLocation
            (const location_t *loc);

    void getSelfControllerOrientation
            (quaternion_t *quat);
    void getSelfControllerOrientation
            (euler_t *euler);
    void setSelfControllerOrientation
            (const quaternion_t *quat);
    void setSelfControllerOrientation
            (const euler_t *euler);

    void getSelfHeadOrientation
            (quaternion_t *quat);
    void getSelfHeadOrientation
            (euler_t *euler);
    void setSelfHeadOrientation
            (const quaternion_t *quat);
    void setSelfHeadOrientation
            (const euler_t *euler);

    void getSelfHeadRefOrientation
            (quaternion_t *quat);
    void getSelfHeadRefOrientation
            (euler_t *euler);
    void setSelfHeadRefOrientation
            (const quaternion_t *quat);
    void setSelfHeadRefOrientation
            (const euler_t *euler);

    std::string& getPeerFriendlyName(void);

    std::string& getPeerMaker(void);

    std::string& getPeerModel(void);

    std::string& getPeerModelId(void);

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
            (pdraw_location_t *loc);
    void setPeerTakeoffLocation
            (const pdraw_location_t *loc);

    void getPeerHomeLocation
            (pdraw_location_t *loc);
    void setPeerHomeLocation
            (const pdraw_location_t *loc);

    void getCameraOrientationForHeadtracking(float *pan, float *tilt);

    int getMediaCount();

    int getMediaInfo(unsigned int index, pdraw_media_info_t *info);

    void *addVideoFrameFilterCallback(unsigned int mediaId, pdraw_video_frame_filter_callback_t cb, void *userPtr);

    int removeVideoFrameFilterCallback(unsigned int mediaId, void *filterCtx);

    void *addVideoFrameProducer(unsigned int mediaId);

    int removeVideoFrameProducer(void *producerCtx);

    /*
     * get last frame
     *
     * waitUs : time in microseconds to wait a frame
     *  0: don't wait
     * -1: wait forever
     * >0: wait time
     */
    int getProducerLastFrame(void *producerCtx, pdraw_video_frame_t *frame, long waitUs = 0);

    void getHmdDistorsionCorrectionSettings(float *xdpi, float *ydpi,
        float *deviceMargin, float *ipd, float *scale, float *panH, float *panV);
    void setHmdDistorsionCorrectionSettings(float xdpi, float ydpi,
        float deviceMargin, float ipd, float scale, float panH, float panV);

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
    bool mPaused;
    bool mGotRendererParams;
    int mWindowWidth;
    int mWindowHeight;
    int mRenderX;
    int mRenderY;
    int mRenderWidth;
    int mRenderHeight;
    bool mHmdDistorsionCorrection;
    bool mHeadtracking;
    void *mUiHandler;
};

}

#endif /* !_PDRAW_IMPL_HPP_ */
