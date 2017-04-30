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

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#define FRAMEBUFFER_MALLOC_CHUNK_SIZE 4096


namespace Pdraw
{


StreamDemuxer::StreamDemuxer()
{
    mConfigured = false;
    mStreamReceiver = NULL;
    mStreamNetworkThreadLaunched = false;
    mStreamOutputThreadLaunched = false;
    mResender = NULL;
    mCurrentAuSize = 0;
    mMaxPacketSize = 0; //TODO
    mQosMode = 0;
    mCurrentBuffer = NULL;
    mDecoder = NULL;
}


StreamDemuxer::~StreamDemuxer()
{
    if (mStreamNetworkThreadLaunched)
    {
        int thErr = pthread_join(mStreamNetworkThread, NULL);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: pthread_join() failed (%d)", thErr);
        }
    }
    if (mStreamOutputThreadLaunched)
    {
        int thErr = pthread_join(mStreamOutputThread, NULL);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: pthread_join() failed (%d)", thErr);
        }
    }

    ARSTREAM2_StreamReceiver_Free(&mStreamReceiver);
}


int StreamDemuxer::configure(const std::string &url)
{
    //TODO
    return -1;
}


int StreamDemuxer::configure(const std::string &canonicalName,
              const std::string &friendlyName,
              const std::string &applicationName,
              const std::string &srcAddr,
              const std::string &ifaceAddr,
              int srcStreamPort, int srcControlPort,
              int dstStreamPort, int dstControlPort, int qosMode)
{
    if (mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is already configured");
        return -1;
    }

    mCanonicalName = canonicalName;
    mFriendlyName = friendlyName;
    mApplicationName = applicationName;
    mQosMode = qosMode;
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
        streamReceiverConfig.canonicalName = mCanonicalName.c_str();
        streamReceiverConfig.friendlyName = mFriendlyName.c_str();
        streamReceiverConfig.applicationName = mApplicationName.c_str();
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


int StreamDemuxer::configure(const std::string &canonicalName,
              const std::string &friendlyName,
              const std::string &applicationName,
              const std::string &sessionDescription, int qosMode)
{
    if (mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is already configured");
        return -1;
    }

    mCanonicalName = canonicalName;
    mFriendlyName = friendlyName;
    mApplicationName = applicationName;
    mQosMode = qosMode;
    int ret = 0;

    if (ret == 0)
    {
        eARSTREAM2_ERROR err;
        ARSTREAM2_StreamReceiver_Config_t streamReceiverConfig;
        ARSTREAM2_StreamReceiver_NetConfig_t streamReceiverNetConfig;
        memset(&streamReceiverConfig, 0, sizeof(streamReceiverConfig));
        memset(&streamReceiverNetConfig, 0, sizeof(streamReceiverNetConfig));
        streamReceiverNetConfig.sessionDescription = sessionDescription.c_str();
        streamReceiverNetConfig.classSelector = (qosMode == 1) ? ARSAL_SOCKET_CLASS_SELECTOR_CS4 : ARSAL_SOCKET_CLASS_SELECTOR_UNSPECIFIED;
        streamReceiverConfig.canonicalName = mCanonicalName.c_str();
        streamReceiverConfig.friendlyName = mFriendlyName.c_str();
        streamReceiverConfig.applicationName = mApplicationName.c_str();
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
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mStreamOutputThread, NULL, ARSTREAM2_StreamReceiver_RunAppOutputThread, (void*)mStreamReceiver);
        if (thErr != 0)
        {
            ULOGE("StreamDemuxer: stream output thread creation failed (%d)", thErr);
            ret = -1;
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

    //TODO
    return 1;
}


elementary_stream_type_t StreamDemuxer::getElementaryStreamType(int esIndex)
{
    if (!mConfigured)
    {
        ULOGE("StreamDemuxer: demuxer is not configured");
        return (elementary_stream_type_t)-1;
    }
    if ((esIndex < 0) || (esIndex >= 1)) //TODO
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return (elementary_stream_type_t)-1;
    }

    //TODO
    return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;

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
    if ((esIndex < 0) || (esIndex >= 1)) //TODO
    {
        ULOGE("StreamDemuxer: invalid ES index");
        return -1;
    }

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

    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (ret == ARSTREAM2_OK)
    {
        ret = ARSTREAM2_StreamReceiver_Stop(mStreamReceiver);
        if (ret != ARSTREAM2_OK)
        {
            ULOGE("StreamDemuxer: ARSTREAM2_StreamReceiver_Stop() failed: %s", ARSTREAM2_Error_ToString(ret));
        }
    }

    return (ret == ARSTREAM2_OK) ? 0 : -1;
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

    ARSTREAM2_StreamReceiver_ResenderConfig_t resenderConfig;
    memset(&resenderConfig, 0, sizeof(resenderConfig));
    resenderConfig.canonicalName = mCanonicalName.c_str();
    resenderConfig.friendlyName = mFriendlyName.c_str();
    resenderConfig.applicationName = mApplicationName.c_str();
    resenderConfig.clientAddr = dstAddr.c_str();
    int addrFirst = atoi(dstAddr.c_str());
    resenderConfig.mcastAddr = ((addrFirst >= 224) && (addrFirst <= 239)) ? dstAddr.c_str() : NULL;
    resenderConfig.mcastIfaceAddr = ifaceAddr.c_str();
    resenderConfig.serverStreamPort = srcStreamPort;
    resenderConfig.serverControlPort = srcControlPort;
    resenderConfig.clientStreamPort = dstStreamPort;
    resenderConfig.clientControlPort = dstControlPort;
    resenderConfig.classSelector = (mQosMode == 1) ? ARSAL_SOCKET_CLASS_SELECTOR_CS4 : ARSAL_SOCKET_CLASS_SELECTOR_UNSPECIFIED;
    resenderConfig.sessionAnnouncement = 1;
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


eARSTREAM2_ERROR StreamDemuxer::h264FilterSpsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr)
{
    StreamDemuxer *demuxer = (StreamDemuxer*)userPtr;

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
    int ret = demuxer->mDecoder->configure(spsBuffer, (unsigned int)spsSize, ppsBuffer, (unsigned int)ppsSize);
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
        data->hasMetadata = Metadata::decodeFrameMetadata(auMetadata->auMetadata, auMetadata->auMetadataSize,
                                                         FRAME_METADATA_SOURCE_STREAMING, NULL, &data->metadata);

        clock_gettime(CLOCK_MONOTONIC, &t1);
        data->demuxOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

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

}
