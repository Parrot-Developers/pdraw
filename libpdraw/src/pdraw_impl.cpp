/**
 * @file pdraw_impl.cpp
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

#include "pdraw_impl.hpp"
#include "pdraw_demuxer_stream.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_decoder.hpp"
#include "pdraw_avcdecoder.hpp"
#include "pdraw_renderer.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_filter_videoframe.hpp"

#include <unistd.h>
#include <sched.h>

#define ULOG_TAG libpdraw
#include <ulog.h>
ULOG_DECLARE_TAG(libpdraw);


namespace Pdraw
{


IPdraw *createPdraw()
{
    return new PdrawImpl;
}


PdrawImpl::PdrawImpl() : mSession(&mSettings)
{
    mPaused = false;
    mGotRendererParams = false;
    mUiHandler = NULL;
}


PdrawImpl::~PdrawImpl()
{

}


int PdrawImpl::open(const std::string &url)
{
    return mSession.open(url);
}


int PdrawImpl::open(const std::string &srcAddr, const std::string &ifaceAddr,
                    int srcStreamPort, int srcControlPort,
                    int dstStreamPort, int dstControlPort, int qosMode)
{
    return mSession.open(srcAddr, ifaceAddr,
                        srcStreamPort, srcControlPort,
                        dstStreamPort, dstControlPort, qosMode);
}


int PdrawImpl::open(void *muxContext)
{
    return mSession.open(muxContext);
}


int PdrawImpl::start()
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->start();
        if (ret != 0)
        {
            ULOGE("Failed to start demuxer");
            return -1;
        }
        else
        {
            mPaused = false;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::pause()
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->pause();
        if (ret != 0)
        {
            ULOGE("Failed to pause demuxer");
            return -1;
        }
        else
        {
            mPaused = true;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


bool PdrawImpl::isPaused()
{
    return mPaused;
}


int PdrawImpl::stop()
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->stop();
        if (ret != 0)
        {
            ULOGE("Failed to stop demuxer");
            return -1;
        }
        else
        {
            mPaused = false;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::seekTo(uint64_t timestamp)
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->seekTo(timestamp);
        if (ret != 0)
        {
            ULOGE("Failed to seek with demuxer");
            return -1;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::seekForward(uint64_t delta)
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->seekForward(delta);
        if (ret != 0)
        {
            ULOGE("Failed to seek with demuxer");
            return -1;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::seekBack(uint64_t delta)
{
    if (mSession.getDemuxer())
    {
        int ret = mSession.getDemuxer()->seekBack(delta);
        if (ret != 0)
        {
            ULOGE("Failed to seek with demuxer");
            return -1;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::startRecorder(const std::string &fileName)
{
    if ((mSession.getDemuxer()) && (mSession.getDemuxer()->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mSession.getDemuxer())->startRecorder(fileName);
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::stopRecorder()
{
    if ((mSession.getDemuxer()) && (mSession.getDemuxer()->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mSession.getDemuxer())->stopRecorder();
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::startResender(const std::string &dstAddr, const std::string &ifaceAddr,
                             int srcStreamPort, int srcControlPort,
                             int dstStreamPort, int dstControlPort)
{
    if ((mSession.getDemuxer()) && (mSession.getDemuxer()->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mSession.getDemuxer())->startResender(dstAddr, ifaceAddr,
                                                       srcStreamPort, srcControlPort,
                                                       dstStreamPort, dstControlPort);
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::stopResender()
{
    if ((mSession.getDemuxer()) && (mSession.getDemuxer()->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mSession.getDemuxer())->stopResender();
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::startRenderer(int windowWidth, int windowHeight,
                             int renderX, int renderY,
                             int renderWidth, int renderHeight,
                             bool hmdDistorsionCorrection,
                             bool headtracking,
                             void *uiHandler)
{
    mGotRendererParams = true;
    mUiHandler = uiHandler;
    mWindowWidth = windowWidth;
    mWindowHeight = windowHeight;
    mRenderX = renderX;
    mRenderY = renderY;
    mRenderWidth = renderWidth;
    mRenderHeight = renderHeight;
    mHmdDistorsionCorrection = hmdDistorsionCorrection;
    mHeadtracking = headtracking;

    if (!mSession.getRenderer())
    {
        int ret = mSession.enableRenderer();
        if (ret != 0)
        {
            ULOGE("Failed to enable renderer");
        }
    }

    if (mSession.getRenderer())
    {
        return mSession.getRenderer()->setRendererParams(mWindowWidth, mWindowHeight,
                                            mRenderX, mRenderY,
                                            mRenderWidth, mRenderHeight,
                                            mHmdDistorsionCorrection, mHeadtracking, mUiHandler);
    }
    else
    {
        ULOGE("Invalid renderer");
        return -1;
    }
}


int PdrawImpl::stopRenderer()
{
    if (mSession.getRenderer())
    {
        int ret = mSession.disableRenderer();
        if (ret != 0)
        {
            ULOGE("Failed to disable renderer");
            return -1;
        }
    }
    else
    {
        ULOGE("Invalid renderer");
        return -1;
    }

    return 0;
}


int PdrawImpl::render(uint64_t lastRenderTime)
{
    if (mSession.getRenderer())
    {
        return mSession.getRenderer()->render(lastRenderTime);
    }
    else
    {
        ULOGE("Invalid renderer");
        return -1;
    }
}


pdraw_session_type_t PdrawImpl::getSessionType(void)
{
    return mSession.getSessionType();
}


std::string& PdrawImpl::getSelfFriendlyName(void)
{
    return mSession.getSelfMetadata()->getFriendlyName();
}


void PdrawImpl::setSelfFriendlyName
        (const std::string &friendlyName)
{
    mSession.getSelfMetadata()->setFriendlyName(friendlyName);
}


std::string& PdrawImpl::getSelfSerialNumber(void)
{
    return mSession.getSelfMetadata()->getSerialNumber();
}


void PdrawImpl::setSelfSerialNumber
        (const std::string &serialNumber)
{
    mSession.getSelfMetadata()->setSerialNumber(serialNumber);
}


std::string& PdrawImpl::getSelfSoftwareVersion(void)
{
    return mSession.getSelfMetadata()->getSoftwareVersion();
}


void PdrawImpl::setSelfSoftwareVersion
        (const std::string &softwareVersion)
{
    mSession.getSelfMetadata()->setSoftwareVersion(softwareVersion);
}


bool PdrawImpl::isSelfPilot(void)
{
    return mSession.getSelfMetadata()->isPilot();
}


void PdrawImpl::setSelfPilot
        (bool isPilot)
{
    mSession.getSelfMetadata()->setPilot(isPilot);
}


void PdrawImpl::getSelfLocation
        (location_t *loc)
{
    mSession.getSelfMetadata()->getLocation(loc);
}


void PdrawImpl::setSelfLocation
        (const location_t *loc)
{
    mSession.getSelfMetadata()->setLocation(loc);
}


int PdrawImpl::getControllerBatteryLevel()
{
    return mSession.getSelfMetadata()->getControllerBatteryLevel();
}


void PdrawImpl::setControllerBatteryLevel(int batteryLevel)
{
    mSession.getSelfMetadata()->setControllerBatteryLevel(batteryLevel);
}


void PdrawImpl::getSelfControllerOrientation
        (quaternion_t *quat)
{
    mSession.getSelfMetadata()->getControllerOrientation(quat);
}


void PdrawImpl::getSelfControllerOrientation
        (euler_t *euler)
{
    mSession.getSelfMetadata()->getControllerOrientation(euler);
}


void PdrawImpl::setSelfControllerOrientation
        (const quaternion_t *quat)
{
    mSession.getSelfMetadata()->setControllerOrientation(quat);
}


void PdrawImpl::setSelfControllerOrientation
        (const euler_t *euler)
{
    mSession.getSelfMetadata()->setControllerOrientation(euler);
}


void PdrawImpl::getSelfHeadOrientation
        (quaternion_t *quat)
{
    mSession.getSelfMetadata()->getHeadOrientation(quat);
}


void PdrawImpl::getSelfHeadOrientation
        (euler_t *euler)
{
    mSession.getSelfMetadata()->getHeadOrientation(euler);
}


void PdrawImpl::setSelfHeadOrientation
        (const quaternion_t *quat)
{
    mSession.getSelfMetadata()->setHeadOrientation(quat);
}


void PdrawImpl::setSelfHeadOrientation
        (const euler_t *euler)
{
    mSession.getSelfMetadata()->setHeadOrientation(euler);
}


void PdrawImpl::getSelfHeadRefOrientation
        (quaternion_t *quat)
{
    mSession.getSelfMetadata()->getHeadRefOrientation(quat);
}


void PdrawImpl::getSelfHeadRefOrientation
        (euler_t *euler)
{
    mSession.getSelfMetadata()->getHeadRefOrientation(euler);
}


void PdrawImpl::setSelfHeadRefOrientation
        (const quaternion_t *quat)
{
    mSession.getSelfMetadata()->setHeadRefOrientation(quat);
}


void PdrawImpl::setSelfHeadRefOrientation
        (const euler_t *euler)
{
    mSession.getSelfMetadata()->setHeadRefOrientation(euler);
}


void PdrawImpl::resetSelfHeadRefOrientation(void)
{
    mSession.getSelfMetadata()->resetHeadRefOrientation();
}


std::string& PdrawImpl::getPeerFriendlyName(void)
{
    return mSession.getPeerMetadata()->getFriendlyName();
}


std::string& PdrawImpl::getPeerMaker(void)
{
    return mSession.getPeerMetadata()->getMaker();
}


std::string& PdrawImpl::getPeerModel(void)
{
    return mSession.getPeerMetadata()->getModel();
}


std::string& PdrawImpl::getPeerModelId(void)
{
    return mSession.getPeerMetadata()->getModelId();
}


pdraw_drone_model_t PdrawImpl::getPeerDroneModel(void)
{
    return mSession.getPeerMetadata()->getDroneModel();
}


std::string& PdrawImpl::getPeerSerialNumber(void)
{
    return mSession.getPeerMetadata()->getSerialNumber();
}


std::string& PdrawImpl::getPeerSoftwareVersion(void)
{
    return mSession.getPeerMetadata()->getSoftwareVersion();
}


std::string& PdrawImpl::getPeerBuildId(void)
{
    return mSession.getPeerMetadata()->getBuildId();
}


std::string& PdrawImpl::getPeerTitle(void)
{
    return mSession.getPeerMetadata()->getTitle();
}


std::string& PdrawImpl::getPeerComment(void)
{
    return mSession.getPeerMetadata()->getComment();
}


std::string& PdrawImpl::getPeerCopyright(void)
{
    return mSession.getPeerMetadata()->getCopyright();
}


std::string& PdrawImpl::getPeerRunDate(void)
{
    return mSession.getPeerMetadata()->getRunDate();
}


std::string& PdrawImpl::getPeerRunUuid(void)
{
    return mSession.getPeerMetadata()->getRunUuid();
}


std::string& PdrawImpl::getPeerMediaDate(void)
{
    return mSession.getPeerMetadata()->getMediaDate();
}


void PdrawImpl::getPeerTakeoffLocation(pdraw_location_t *loc)
{
    mSession.getPeerMetadata()->getTakeoffLocation(loc);
}


void PdrawImpl::setPeerTakeoffLocation(const pdraw_location_t *loc)
{
    mSession.getPeerMetadata()->setTakeoffLocation(loc);
}


void PdrawImpl::getPeerHomeLocation(pdraw_location_t *loc)
{
    mSession.getPeerMetadata()->getHomeLocation(loc);
}


void PdrawImpl::setPeerHomeLocation(const pdraw_location_t *loc)
{
    mSession.getPeerMetadata()->setHomeLocation(loc);
}


uint64_t PdrawImpl::getPeerRecordingDuration(void)
{
    return mSession.getPeerMetadata()->getRecordingDuration();
}


void PdrawImpl::setPeerRecordingDuration(uint64_t duration)
{
    mSession.getPeerMetadata()->setRecordingDuration(duration);
}


void PdrawImpl::getCameraOrientationForHeadtracking(float *pan, float *tilt)
{
    mSession.getCameraOrientationForHeadtracking(pan, tilt);
}


int PdrawImpl::getMediaCount()
{
    return mSession.getMediaCount();
}


int PdrawImpl::getMediaInfo(unsigned int index, pdraw_media_info_t *info)
{
    if (!info)
    {
        ULOGE("Invalid info struct");
        return -1;
    }

    Media *media = mSession.getMedia(index);

    if (!media)
    {
        ULOGE("Invalid media index");
        return -1;
    }

    switch (media->getType())
    {
        case PDRAW_MEDIA_TYPE_VIDEO:
            info->type = PDRAW_MEDIA_TYPE_VIDEO;
            info->info.video.type = ((VideoMedia*)media)->getVideoType();
            ((VideoMedia*)media)->getDimensions(&info->info.video.width, &info->info.video.height,
                &info->info.video.cropLeft, &info->info.video.cropRight,
                &info->info.video.cropTop, &info->info.video.cropBottom,
                &info->info.video.croppedWidth, &info->info.video.croppedHeight,
                &info->info.video.sarWidth, &info->info.video.sarHeight);
            ((VideoMedia*)media)->getFov(&info->info.video.horizontalFov, &info->info.video.verticalFov);
            break;
        default:
            info->type = PDRAW_MEDIA_TYPE_UNKNOWN;
            break;
    }
    info->id = media->getId();

    return 0;
}


void *PdrawImpl::addVideoFrameFilterCallback(unsigned int mediaId, pdraw_video_frame_filter_callback_t cb, void *userPtr)
{
    Media *media = mSession.getMediaById(mediaId);

    if (!media)
    {
        ULOGE("Invalid media id");
        return NULL;
    }

    if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO)
    {
        ULOGE("Invalid media type");
        return NULL;
    }

    VideoFrameFilter *filter = ((VideoMedia*)media)->addVideoFrameFilter(cb, userPtr);
    if (!filter)
    {
        ULOGE("Failed to create video frame filter");
        return NULL;
    }

    return (void*)filter;
}


int PdrawImpl::removeVideoFrameFilterCallback(unsigned int mediaId, void *filterCtx)
{
    Media *media = mSession.getMediaById(mediaId);

    if (!media)
    {
        ULOGE("Invalid media id");
        return -1;
    }

    if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO)
    {
        ULOGE("Invalid media type");
        return -1;
    }

    if (!filterCtx)
    {
        ULOGE("Invalid context pointer");
        return -1;
    }

    VideoFrameFilter *filter = (VideoFrameFilter*)filterCtx;
    return ((VideoMedia*)media)->removeVideoFrameFilter(filter);
}


void *PdrawImpl::addVideoFrameProducer(unsigned int mediaId)
{
    Media *media = mSession.getMediaById(mediaId);

    if (!media)
    {
        ULOGE("Invalid media id");
        return NULL;
    }

    if (media->getType() != PDRAW_MEDIA_TYPE_VIDEO)
    {
        ULOGE("Invalid media type");
        return NULL;
    }

    VideoFrameFilter *filter = ((VideoMedia*)media)->addVideoFrameFilter();
    if (!filter)
    {
        ULOGE("Failed to create video frame filter");
        return NULL;
    }

    return (void*)filter;
}


int PdrawImpl::removeVideoFrameProducer(void *producerCtx)
{
    if (!producerCtx)
    {
        ULOGE("Invalid context pointer");
        return -1;
    }

    VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;
    VideoMedia *media = filter->getVideoMedia();

    if (!media)
    {
        ULOGE("Invalid media");
        return -1;
    }

    return media->removeVideoFrameFilter(filter);
}


int PdrawImpl::getProducerLastFrame(void *producerCtx, pdraw_video_frame_t *frame, long waitUs)
{
    if (!producerCtx)
    {
        ULOGE("Invalid context pointer");
        return -1;
    }
    if (!frame)
    {
        ULOGE("Invalid frame structure pointer");
        return -1;
    }

    VideoFrameFilter *filter = (VideoFrameFilter*)producerCtx;

    return filter->getLastFrame(frame, waitUs);
}


float PdrawImpl::getControllerRadarAngleSetting(void)
{
    return mSettings.getControllerRadarAngle();
}


void PdrawImpl::setControllerRadarAngleSetting(float angle)
{
    mSettings.setControllerRadarAngle(angle);
}


void PdrawImpl::getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin)
{
    mSettings.getDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void PdrawImpl::setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin)
{
    mSettings.setDisplayScreenSettings(xdpi, ydpi, deviceMargin);
}


void PdrawImpl::getHmdDistorsionCorrectionSettings(pdraw_hmd_model_t *hmdModel, float *ipd, float *scale, float *panH, float *panV)
{
    mSettings.getHmdDistorsionCorrectionSettings(hmdModel, ipd, scale, panH, panV);
}


void PdrawImpl::setHmdDistorsionCorrectionSettings(pdraw_hmd_model_t hmdModel, float ipd, float scale, float panH, float panV)
{
    mSettings.setHmdDistorsionCorrectionSettings(hmdModel, ipd, scale, panH, panV);
}

}
