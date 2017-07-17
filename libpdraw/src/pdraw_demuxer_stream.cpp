/**
 * @file pdraw_demuxer_stream.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - streaming demuxer
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

#include "pdraw_demuxer_stream.hpp"
#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#define STREAM_DEMUXER_SESSION_METADATA_FETCH_INTERVAL 1000000

#define STREAM_DEMUXER_DEFAULT_DST_STREAM_PORT 55004
#define STREAM_DEMUXER_DEFAULT_DST_CONTROL_PORT 55005


namespace Pdraw
{


StreamDemuxer::StreamDemuxer(Session *session)
{
    mSession = session;
    mConfigured = false;
    mLoop = NULL;
    mLoopThreadLaunched = false;
    mThreadShouldStop = false;
    mRtspRunning = false;
    mRtspClient = NULL;
    mStreamReceiver = NULL;
    mStreamNetworkThreadLaunched = false;
    mStreamOutputThreadLaunched = false;
    mResender = NULL;
    mCurrentAuSize = 0;
    mMaxPacketSize = 0; //TODO
    mQosMode = 0;
    mCurrentBuffer = NULL;
    mDecoder = NULL;
    mStartTime = mCurrentTime = 0;
    mLastSessionMetadataFetchTime = 0;
    mWidth = mHeight = 0;
    mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
    mSarWidth = mSarHeight = 0;
    mHfov = mVfov = 0.;
}


StreamDemuxer::~StreamDemuxer()
{
    int ret = stop();
    if (ret != 0)
        ULOGE("StreamDemuxer: stop() failed (%d)", ret);

    if (mStreamNetworkThreadLaunched)
    {
        ret = pthread_join(mStreamNetworkThread, NULL);
        if (ret != 0)
            ULOGE("StreamDemuxer: pthread_join() failed (%d)", ret);
    }
    if (mStreamOutputThreadLaunched)
    {
        ret = pthread_join(mStreamOutputThread, NULL);
        if (ret != 0)
            ULOGE("StreamDemuxer: pthread_join() failed (%d)", ret);
    }
    if (mLoopThreadLaunched)
    {
        ret = pthread_join(mLoopThread, NULL);
        if (ret != 0)
            ULOGE("StreamDemuxer: pthread_join() failed (%d)", ret);
    }

    if (mCurrentBuffer)
        mCurrentBuffer->unref();

    if (mRtspClient) {
        ret = rtsp_client_destroy(mRtspClient);
        if (ret != 0)
            ULOGE("StreamDemuxer: rtsp_client_destroy() failed");
    }

    if (mStreamReceiver)
        ARSTREAM2_StreamReceiver_Free(&mStreamReceiver);

    if (mLoop)
    {
        ret = pomp_loop_destroy(mLoop);
        if (ret != 0)
            ULOGE("StreamDemuxer: pomp_loop_destroy() failed");
    }
}


void StreamDemuxer::fetchSessionMetadata(StreamDemuxer *demuxer)
{
    if (!demuxer)
    {
        ULOGE("StreamDemuxer: invalid pointer");
        return;
    }
    if (!demuxer->mSession)
    {
        ULOGE("StreamDemuxer: invalid session");
        return;
    }

    SessionPeerMetadata *peerMeta = demuxer->mSession->getPeerMetadata();
    ARSTREAM2_Stream_UntimedMetadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_GetPeerUntimedMetadata(demuxer->mStreamReceiver, &metadata);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_GetPeerUntimedMetadata() failed: %s", ARSTREAM2_Error_ToString(ret));
        return;
    }

    if (metadata.friendlyName)
        peerMeta->setFriendlyName(metadata.friendlyName);
    if (metadata.title)
        peerMeta->setTitle(metadata.title);
    if (metadata.maker)
        peerMeta->setMaker(metadata.maker);
    if (metadata.model)
        peerMeta->setModel(metadata.model);
    if (metadata.softwareVersion)
        peerMeta->setSoftwareVersion(metadata.softwareVersion);
    if (metadata.serialNumber)
        peerMeta->setSerialNumber(metadata.serialNumber);
    if (metadata.modelId)
        peerMeta->setModelId(metadata.modelId);
    if (metadata.buildId)
        peerMeta->setBuildId(metadata.buildId);
    if (metadata.runUuid)
        peerMeta->setRunUuid(metadata.runUuid);
    if (metadata.runDate)
        peerMeta->setRunDate(metadata.runDate);
    if (metadata.comment)
        peerMeta->setComment(metadata.comment);
    if (metadata.copyright)
        peerMeta->setCopyright(metadata.copyright);
    if ((metadata.pictureHFov != 0.) && (metadata.pictureVFov != 0.) &&
        ((demuxer->mHfov != metadata.pictureHFov) || (demuxer->mVfov != metadata.pictureVFov)))
    {
        demuxer->mHfov = metadata.pictureHFov;
        demuxer->mVfov = metadata.pictureVFov;
        if (demuxer->mDecoder)
        {
            VideoMedia *vm = demuxer->mDecoder->getVideoMedia();
            if (vm)
                vm->setFov(metadata.pictureHFov, metadata.pictureVFov);
        }
    }
    if ((metadata.takeoffLatitude != 500.) && (metadata.takeoffLongitude != 500.))
    {
        location_t takeoffLoc;
        memset(&takeoffLoc, 0, sizeof(takeoffLoc));
        takeoffLoc.latitude = metadata.takeoffLatitude;
        takeoffLoc.longitude = metadata.takeoffLongitude;
        takeoffLoc.altitude = metadata.takeoffAltitude;
        takeoffLoc.isValid = 1;
    }
}


int StreamDemuxer::configure(const std::string &url)
{
    if (mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is already configured");
        return -1;
    }
    if (!mSession)
    {
        ULOGE("StreamDemuxer: invalid session");
        return -1;
    }

    if (url.substr(0, 7) != "rtsp://")
    {
        ULOGE("StreamDemuxer: unsupported URL");
        return -1;
    }

    int ret = 0;
    char *mediaUrl = NULL, *serverAddr = NULL;
    int serverStreamPort = 0, serverControlPort = 0;
    char *sdpStr = NULL;
    struct sdp_session *sdp = NULL;
    SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();

    if (ret == 0)
    {
        mLoop = pomp_loop_new();
        if (!mLoop)
        {
            ULOGE("StreamDemuxer: pomp_loop_new() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mLoopThread, NULL, runLoopThread, (void*)this);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: loop thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mLoopThreadLaunched = true;
        }
    }

    if (ret == 0)
    {
        const char *userAgent = selfMeta->getSoftwareVersion().c_str();
        mRtspClient = rtsp_client_new(url.c_str(), userAgent, mLoop);
        if (!mRtspClient)
        {
            ULOGE("StreamDemuxer: rtsp_client_new() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int err = rtsp_client_options(mRtspClient);
        if (err)
        {
            ULOGE("StreamDemuxer: rtsp_client_options() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int err = rtsp_client_describe(mRtspClient, &sdpStr);
        if (err)
        {
            ULOGE("StreamDemuxer: rtsp_client_describe() failed");
            ret = -1;
        }
        else if (!sdpStr)
        {
            ULOGE("StreamDemuxer: failed to get session description");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        sdp = sdp_parse_session_description(sdpStr);
        if (!sdp)
        {
            ULOGE("StreamDemuxer: sdp_parse_session_description() failed");
            ret = -1;
        }
        else
        {
            struct sdp_media *media;
            for (media = sdp->media; media; media = media->next)
            {
                if ((media->type == SDP_MEDIA_TYPE_VIDEO) &&
                    (media->controlUrl))
                {
                    mediaUrl = strdup(media->controlUrl);
                    break;
                }
            }
            if (!mediaUrl)
            {
                ULOGE("StreamDemuxer: failed to get media control URL");
                ret = -1;
            }
            serverAddr = (strncmp(sdp->connectionAddr, "0.0.0.0", 7)) ? strdup(sdp->connectionAddr) : strdup(sdp->serverAddr);
            if (!serverAddr)
            {
                ULOGE("StreamDemuxer: failed to get server address");
                ret = -1;
            }
            sdp_session_destroy(sdp);
        }
    }

    if (ret == 0)
    {
        int err = rtsp_client_setup(mRtspClient, mediaUrl, STREAM_DEMUXER_DEFAULT_DST_STREAM_PORT,
            STREAM_DEMUXER_DEFAULT_DST_CONTROL_PORT, &serverStreamPort, &serverControlPort);
        if (err)
        {
            ULOGE("StreamDemuxer: rtsp_client_setup() failed");
            ret = -1;
        }
    }

    free(mediaUrl);

    if (ret == 0)
    {
        eARSTREAM2_ERROR err;
        ARSTREAM2_StreamReceiver_Config_t streamReceiverConfig;
        ARSTREAM2_StreamReceiver_NetConfig_t streamReceiverNetConfig;
        memset(&streamReceiverConfig, 0, sizeof(streamReceiverConfig));
        memset(&streamReceiverNetConfig, 0, sizeof(streamReceiverNetConfig));
        streamReceiverNetConfig.serverAddr = serverAddr;
        streamReceiverNetConfig.mcastAddr = NULL; //TODO
        streamReceiverNetConfig.mcastIfaceAddr = NULL; //TODO
        streamReceiverNetConfig.serverStreamPort = serverStreamPort;
        streamReceiverNetConfig.serverControlPort = serverControlPort;
        streamReceiverNetConfig.clientStreamPort = STREAM_DEMUXER_DEFAULT_DST_STREAM_PORT;
        streamReceiverNetConfig.clientControlPort = STREAM_DEMUXER_DEFAULT_DST_CONTROL_PORT;
        streamReceiverNetConfig.classSelector = ARSAL_SOCKET_CLASS_SELECTOR_CS4; //TODO
        streamReceiverConfig.canonicalName = selfMeta->getSerialNumber().c_str();
        streamReceiverConfig.friendlyName = selfMeta->getFriendlyName().c_str();
        streamReceiverConfig.applicationName = selfMeta->getSoftwareVersion().c_str();
        streamReceiverConfig.maxPacketSize = mMaxPacketSize;
        streamReceiverConfig.generateReceiverReports = 1;
        streamReceiverConfig.waitForSync = 1;
        streamReceiverConfig.outputIncompleteAu = 0;
        streamReceiverConfig.filterOutSpsPps = 0;
        streamReceiverConfig.filterOutSei = 1;
        streamReceiverConfig.replaceStartCodesWithNaluSize = 0;
        streamReceiverConfig.generateSkippedPSlices = 1;
        streamReceiverConfig.generateFirstGrayIFrame = 1;
        streamReceiverConfig.debugPath = "./streamdebug";

        err = ARSTREAM2_StreamReceiver_Init(&mStreamReceiver, &streamReceiverConfig, &streamReceiverNetConfig, NULL);
        if (err != ARSTREAM2_OK)
        {
            ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_Init() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    free(serverAddr);

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamNetworkThread, NULL, ARSTREAM2_StreamReceiver_RunNetworkThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream network thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamNetworkThreadLaunched = true;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamOutputThread, NULL, ARSTREAM2_StreamReceiver_RunAppOutputThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream output thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamOutputThreadLaunched = true;
        }
    }

    if (ret == 0)
    {
        int err = rtsp_client_play(mRtspClient);
        if (err)
        {
            ULOGE("StreamDemuxer: rtsp_client_play() failed");
            ret = -1;
        }
        else
        {
            mRtspRunning = true;
        }
    }

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("StreamDemuxer: demuxer is configured");
    }

    return ret;
}


int StreamDemuxer::configure(const std::string &srcAddr,
              const std::string &ifaceAddr,
              int srcStreamPort, int srcControlPort,
              int dstStreamPort, int dstControlPort, int qosMode)
{
    if (mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is already configured");
        return -1;
    }
    if (!mSession)
    {
        ULOGE("StreamDemuxer: invalid session");
        return -1;
    }

    mQosMode = qosMode;
    SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();
    int ret = 0;

    if (ret == 0)
    {
        eARSTREAM2_ERROR err;
        ARSTREAM2_StreamReceiver_Config_t streamReceiverConfig;
        ARSTREAM2_StreamReceiver_NetConfig_t streamReceiverNetConfig;
        memset(&streamReceiverConfig, 0, sizeof(streamReceiverConfig));
        memset(&streamReceiverNetConfig, 0, sizeof(streamReceiverNetConfig));
        streamReceiverNetConfig.serverAddr = srcAddr.c_str();
        int addrFirst = atoi(srcAddr.c_str());
        streamReceiverNetConfig.mcastAddr = ((addrFirst >= 224) && (addrFirst <= 239)) ? srcAddr.c_str() : NULL;
        streamReceiverNetConfig.mcastIfaceAddr = ifaceAddr.c_str();
        streamReceiverNetConfig.serverStreamPort = srcStreamPort;
        streamReceiverNetConfig.serverControlPort = srcControlPort;
        streamReceiverNetConfig.clientStreamPort = dstStreamPort;
        streamReceiverNetConfig.clientControlPort = dstControlPort;
        streamReceiverNetConfig.classSelector = (qosMode == 1) ? ARSAL_SOCKET_CLASS_SELECTOR_CS4 : ARSAL_SOCKET_CLASS_SELECTOR_UNSPECIFIED;
        streamReceiverConfig.canonicalName = selfMeta->getSerialNumber().c_str();
        streamReceiverConfig.friendlyName = selfMeta->getFriendlyName().c_str();
        streamReceiverConfig.applicationName = selfMeta->getSoftwareVersion().c_str();
        streamReceiverConfig.maxPacketSize = mMaxPacketSize;
        streamReceiverConfig.generateReceiverReports = 1;
        streamReceiverConfig.waitForSync = 1;
        streamReceiverConfig.outputIncompleteAu = 0;
        streamReceiverConfig.filterOutSpsPps = 0;
        streamReceiverConfig.filterOutSei = 1;
        streamReceiverConfig.replaceStartCodesWithNaluSize = 0;
        streamReceiverConfig.generateSkippedPSlices = 1;
        streamReceiverConfig.generateFirstGrayIFrame = 1;
        streamReceiverConfig.debugPath = "./streamdebug";

        err = ARSTREAM2_StreamReceiver_Init(&mStreamReceiver, &streamReceiverConfig, &streamReceiverNetConfig, NULL);
        if (err != ARSTREAM2_OK)
        {
            ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_Init() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamNetworkThread, NULL, ARSTREAM2_StreamReceiver_RunNetworkThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream network thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamNetworkThreadLaunched = true;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamOutputThread, NULL, ARSTREAM2_StreamReceiver_RunAppOutputThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream output thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamOutputThreadLaunched = true;
        }
    }

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("StreamDemuxer: demuxer is configured");
    }

    return ret;
}


int StreamDemuxer::configure(void *muxContext)
{
    if (mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is already configured");
        return -1;
    }
    if (!mSession)
    {
        ULOGE("StreamDemuxer: invalid session");
        return -1;
    }

    mQosMode = 1; //TODO
    SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();
    int ret = 0;

    if (ret == 0)
    {
        eARSTREAM2_ERROR err;
        ARSTREAM2_StreamReceiver_Config_t streamReceiverConfig;
        ARSTREAM2_StreamReceiver_MuxConfig_t streamReceiverMuxConfig;
        memset(&streamReceiverConfig, 0, sizeof(streamReceiverConfig));
        memset(&streamReceiverMuxConfig, 0, sizeof(streamReceiverMuxConfig));
        streamReceiverMuxConfig.mux = (struct mux_ctx*)muxContext;
        streamReceiverConfig.canonicalName = selfMeta->getSerialNumber().c_str();
        streamReceiverConfig.friendlyName = selfMeta->getFriendlyName().c_str();
        streamReceiverConfig.applicationName = selfMeta->getSoftwareVersion().c_str();
        streamReceiverConfig.maxPacketSize = mMaxPacketSize;
        streamReceiverConfig.generateReceiverReports = 1;
        streamReceiverConfig.waitForSync = 1;
        streamReceiverConfig.outputIncompleteAu = 0;
        streamReceiverConfig.filterOutSpsPps = 0;
        streamReceiverConfig.filterOutSei = 1;
        streamReceiverConfig.replaceStartCodesWithNaluSize = 0;
        streamReceiverConfig.generateSkippedPSlices = 1;
        streamReceiverConfig.generateFirstGrayIFrame = 1;
        streamReceiverConfig.debugPath = "./streamdebug";

        err = ARSTREAM2_StreamReceiver_Init(&mStreamReceiver, &streamReceiverConfig, NULL, &streamReceiverMuxConfig);
        if (err != ARSTREAM2_OK)
        {
            ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_Init() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamNetworkThread, NULL, ARSTREAM2_StreamReceiver_RunNetworkThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream network thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamNetworkThreadLaunched = true;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamOutputThread, NULL, ARSTREAM2_StreamReceiver_RunAppOutputThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream output thread creation failed (%d)", thErr);
            ret = -1;
        }
        else
        {
            mStreamOutputThreadLaunched = true;
        }
    }

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("StreamDemuxer: demuxer is configured");
    }

    return ret;
}


int StreamDemuxer::getElementaryStreamCount()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    //TODO: handle multiple streams
    return 1;
}


elementary_stream_type_t StreamDemuxer::getElementaryStreamType(int esIndex)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return (elementary_stream_type_t)-1;
    }
    if ((esIndex < 0) || (esIndex >= 1))
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return (elementary_stream_type_t)-1;
    }

    //TODO: handle multiple streams
    return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;

}


int StreamDemuxer::getElementaryStreamVideoDimensions(int esIndex,
    unsigned int *width, unsigned int *height,
    unsigned int *cropLeft, unsigned int *cropRight,
    unsigned int *cropTop, unsigned int *cropBottom,
    unsigned int *sarWidth, unsigned int *sarHeight)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= 1))
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    if (width)
        *width = mWidth;
    if (height)
        *height = mHeight;
    if (cropLeft)
        *cropLeft = mCropLeft;
    if (cropRight)
        *cropRight = mCropRight;
    if (cropTop)
        *cropTop = mCropTop;
    if (cropBottom)
        *cropBottom = mCropBottom;
    if (sarWidth)
        *sarWidth = mSarWidth;
    if (sarHeight)
        *sarHeight = mSarHeight;

    return 0;
}


int StreamDemuxer::getElementaryStreamVideoFov(int esIndex, float *hfov, float *vfov)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= 1))
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    if (hfov)
        *hfov = mHfov;
    if (vfov)
        *vfov = mVfov;

    return 0;
}


int StreamDemuxer::setElementaryStreamDecoder(int esIndex, Decoder *decoder)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }
    if (!decoder)
    {
        ULOGE("StreamDemuxer: invalid decoder");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= 1))
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    mDecoder = (AvcDecoder*)decoder;

    return 0;
}


int StreamDemuxer::start()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StartAppOutput(mStreamReceiver, h264FilterSpsPpsCallback, this,
                                                                   h264FilterGetAuBufferCallback, this,
                                                                   h264FilterAuReadyCallback, this);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StartAppOutput() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


int StreamDemuxer::pause()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StopAppOutput(mStreamReceiver);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StopAppOutput() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


int StreamDemuxer::stop()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    int ret = 0;

    if ((ret == 0) && (mRtspClient) && (mRtspRunning))
    {
        int err = rtsp_client_teardown(mRtspClient);
        if (err)
        {
            ULOGE("StreamDemuxer: rtsp_client_teardown() failed");
            ret = -1;
        }
        else
        {
            mRtspRunning = false;
        }
    }

    if (ret == 0)
    {
        eARSTREAM2_ERROR _ret = ARSTREAM2_StreamReceiver_Stop(mStreamReceiver);
        if (_ret != ARSTREAM2_OK)
        {
            ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_Stop() failed: %s", ARSTREAM2_Error_ToString(_ret));
            ret = -1;
        }
    }

    mThreadShouldStop = true;
    if (mLoop)
        pomp_loop_wakeup(mLoop);

    return ret;
}


int StreamDemuxer::seekTo(uint64_t timestamp)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    return -1;
}


int StreamDemuxer::seekForward(uint64_t delta)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    return -1;
}


int StreamDemuxer::seekBack(uint64_t delta)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    return -1;
}


int StreamDemuxer::startRecorder(const std::string &fileName)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StartRecorder(mStreamReceiver, fileName.c_str());
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StartRecorder() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


int StreamDemuxer::stopRecorder()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StopRecorder(mStreamReceiver);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StopRecorder() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


int StreamDemuxer::startResender(const std::string &dstAddr, const std::string &ifaceAddr,
                                 int srcStreamPort, int srcControlPort,
                                 int dstStreamPort, int dstControlPort)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }
    if (!mSession)
    {
        ULOGE("StreamDemuxer: invalid session");
        return -1;
    }

    SessionSelfMetadata *selfMeta = mSession->getSelfMetadata();

    ARSTREAM2_StreamReceiver_ResenderConfig_t resenderConfig;
    memset(&resenderConfig, 0, sizeof(resenderConfig));
    resenderConfig.canonicalName = selfMeta->getSerialNumber().c_str();
    resenderConfig.friendlyName = selfMeta->getFriendlyName().c_str();
    resenderConfig.applicationName = selfMeta->getSoftwareVersion().c_str();
    resenderConfig.clientAddr = dstAddr.c_str();
    int addrFirst = atoi(dstAddr.c_str());
    resenderConfig.mcastAddr = ((addrFirst >= 224) && (addrFirst <= 239)) ? dstAddr.c_str() : NULL;
    resenderConfig.mcastIfaceAddr = ifaceAddr.c_str();
    resenderConfig.serverStreamPort = srcStreamPort;
    resenderConfig.serverControlPort = srcControlPort;
    resenderConfig.clientStreamPort = dstStreamPort;
    resenderConfig.clientControlPort = dstControlPort;
    resenderConfig.classSelector = (mQosMode == 1) ? ARSAL_SOCKET_CLASS_SELECTOR_CS4 : ARSAL_SOCKET_CLASS_SELECTOR_UNSPECIFIED;
    resenderConfig.streamSocketBufferSize = 0;
    resenderConfig.maxNetworkLatencyMs = 200;
    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StartResender(mStreamReceiver, &mResender, &resenderConfig);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StartResender() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


int StreamDemuxer::stopResender()
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return -1;
    }

    eARSTREAM2_ERROR ret = ARSTREAM2_StreamReceiver_StopResender(mStreamReceiver, &mResender);
    if (ret != ARSTREAM2_OK)
    {
        ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_StopResender() failed: %s", ARSTREAM2_Error_ToString(ret));
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
}


uint64_t StreamDemuxer::getCurrentTime()
{
    return (mStartTime != 0) ? mCurrentTime - mStartTime : 0;
}


eARSTREAM2_ERROR StreamDemuxer::h264FilterSpsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr)
{
    StreamDemuxer *demuxer = (StreamDemuxer*)userPtr;
    int ret;

    if ((!demuxer) || (!spsBuffer) || (!spsSize) || (!ppsBuffer) || (!ppsSize))
    {
        ULOGE("StreamDemuxer: invalid callback params");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!demuxer->mDecoder)
    {
        ULOGE("StreamDemuxer: no decoder configured");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    ULOGD("StreamDemuxer: received SPS/PPS");

    if (spsSize > 4)
    {
        ret = pdraw_videoDimensionsFromH264Sps(spsBuffer + 4, spsSize -4,
            &demuxer->mWidth, &demuxer->mHeight, &demuxer->mCropLeft, &demuxer->mCropRight,
            &demuxer->mCropTop, &demuxer->mCropBottom, &demuxer->mSarWidth, &demuxer->mSarHeight);
        if (ret != 0)
        {
            ULOGW("StreamDemuxer: pdraw_videoDimensionsFromH264Sps() failed (%d)", ret);
        }
        else if (demuxer->mDecoder)
        {
            VideoMedia *vm = demuxer->mDecoder->getVideoMedia();
            if (vm)
                vm->setDimensions(demuxer->mWidth, demuxer->mHeight, demuxer->mCropLeft, demuxer->mCropRight,
                    demuxer->mCropTop, demuxer->mCropBottom, demuxer->mSarWidth, demuxer->mSarHeight);
        }
    }

    ret = demuxer->mDecoder->configure(spsBuffer, (unsigned int)spsSize, ppsBuffer, (unsigned int)ppsSize);
    if (ret != 0)
    {
        ULOGE("StreamDemuxer: decoder configuration failed (%d)", ret);
    }

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR StreamDemuxer::h264FilterGetAuBufferCallback(uint8_t **auBuffer, int *auBufferSize, void **auBufferUserPtr, void *userPtr)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
    StreamDemuxer *demuxer = (StreamDemuxer*)userPtr;
    Buffer *buffer = NULL;
    int err = 0;

    if ((!demuxer) || (!auBuffer) || (!auBufferSize))
    {
        ULOGE("StreamDemuxer: invalid callback params");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!demuxer->mDecoder)
    {
        ULOGE("StreamDemuxer: no decoder configured");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    if (demuxer->mCurrentBuffer)
    {
        buffer = demuxer->mCurrentBuffer;
    }
    else if ((err = demuxer->mDecoder->getInputBuffer(&demuxer->mCurrentBuffer, false)) == 0)
    {
        buffer = demuxer->mCurrentBuffer;
    }
    else
    {
        ULOGW("StreamDemuxer: failed to get an input buffer (%d)", err);
    }

    if (buffer)
    {
        *auBuffer = (uint8_t*)buffer->getPtr();
        *auBufferSize = buffer->getCapacity();
        ret = ARSTREAM2_OK;
    }

    return ret;
}


eARSTREAM2_ERROR StreamDemuxer::h264FilterAuReadyCallback(uint8_t *auBuffer, int auSize,
                                                          ARSTREAM2_StreamReceiver_AuReadyCallbackTimestamps_t *auTimestamps,
                                                          eARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE auSyncType,
                                                          ARSTREAM2_StreamReceiver_AuReadyCallbackMetadata_t *auMetadata,
                                                          void *auBufferUserPtr, void *userPtr)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_ERROR_RESYNC_REQUIRED;
    StreamDemuxer *demuxer = (StreamDemuxer*)userPtr;
    Buffer* buffer;
    int err = 0;

    if ((!demuxer) || (!auBuffer) || (!auSize))
    {
        ULOGE("StreamDemuxer: invalid callback params");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!demuxer->mDecoder)
    {
        ULOGE("StreamDemuxer: no decoder configured");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    switch (auSyncType)
    {
        case ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_IDR:
            //ULOGD("StreamDemuxer: sync: IDR");
            break;
        case ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_IFRAME:
            //ULOGD("StreamDemuxer: sync: IFRAME");
            break;
        case ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_PIR_START:
            //ULOGD("StreamDemuxer: sync: PIR_START");
            break;
        case ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_NONE:
        default:
            break;
    }

    if (demuxer->mCurrentBuffer)
    {
        buffer = demuxer->mCurrentBuffer;
        avc_decoder_input_buffer_t *data = (avc_decoder_input_buffer_t*)buffer->getMetadataPtr();
        struct timespec t1;

        buffer->setSize(auSize);
        data->isComplete = (auMetadata->isComplete) ? true : false;
        data->hasErrors = (auMetadata->hasErrors) ? true : false;
        data->isRef = (auMetadata->isRef) ? true : false;
        data->auNtpTimestamp = auTimestamps->auNtpTimestamp;
        data->auNtpTimestampRaw = auTimestamps->auNtpTimestampRaw;
        data->auNtpTimestampLocal = auTimestamps->auNtpTimestampLocal;
        //TODO: auSyncType

        /* Metadata */
        data->hasMetadata = VideoFrameMetadata::decodeMetadata(auMetadata->auMetadata, auMetadata->auMetadataSize,
            FRAME_METADATA_SOURCE_STREAMING, NULL, &data->metadata);

        clock_gettime(CLOCK_MONOTONIC, &t1);
        uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
        data->demuxOutputTimestamp = curTime;

        //TODO: use auNtpTimestamp
        demuxer->mCurrentTime = auTimestamps->auNtpTimestampRaw;
        if (demuxer->mStartTime == 0)
            demuxer->mStartTime = auTimestamps->auNtpTimestampRaw;

        if (curTime >= demuxer->mLastSessionMetadataFetchTime + STREAM_DEMUXER_SESSION_METADATA_FETCH_INTERVAL)
        {
            fetchSessionMetadata(demuxer);
            demuxer->mLastSessionMetadataFetchTime = curTime;
        }

        /*ULOGI("StreamDemuxer: frame timestamps: NTP=%" PRIu64 " NTPRaw=%" PRIu64 " NTPLocal=%" PRIu64 " Cur=%" PRIu64 " Latency=%.1fms",
              auTimestamps->auNtpTimestamp, auTimestamps->auNtpTimestampRaw, auTimestamps->auNtpTimestampLocal,
              data->demuxOutputTimestamp, (auTimestamps->auNtpTimestampLocal != 0) ? (float)(data->demuxOutputTimestamp - auTimestamps->auNtpTimestampLocal) / 1000. : 0.);*/

        /* release buffer */
        err = demuxer->mDecoder->queueInputBuffer(buffer);
        if (err != 0)
        {
            ULOGW("StreamDemuxer: failed to release the input buffer (%d)", ret);
        }
        else
        {
            buffer->unref();
            demuxer->mCurrentBuffer = NULL;
        }

        ret = ARSTREAM2_OK;
    }
    else
    {
        ULOGE("StreamDemuxer: no buffer held");
    }

    return ret;
}


void* StreamDemuxer::runLoopThread(void *ptr)
{
    StreamDemuxer *demuxer = (StreamDemuxer*)ptr;

    while (!demuxer->mThreadShouldStop)
    {
        pomp_loop_wait_and_process(demuxer->mLoop, -1);
    }

    return NULL;
}

}
