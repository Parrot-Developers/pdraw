/**
 * @file pdraw_demuxer_stream.hpp
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

#ifndef _PDRAW_DEMUXER_STREAM_HPP_
#define _PDRAW_DEMUXER_STREAM_HPP_

#include <pthread.h>

#include <libARStream2/arstream2_stream_receiver.h>

#include "pdraw_demuxer.hpp"
#include "pdraw_avcdecoder.hpp"


namespace Pdraw
{


class StreamDemuxer : public Demuxer
{
public:

    StreamDemuxer(Session *session);

    ~StreamDemuxer();

    demuxer_type_t getType() { return DEMUXER_TYPE_STREAM; };

    bool isConfigured() { return mConfigured; };

    int configure(const std::string &url);

    int configure(const std::string &srcAddr,
                  const std::string &ifaceAddr,
                  int srcStreamPort, int srcControlPort,
                  int dstStreamPort, int dstControlPort, int qosMode);

    int configure(void *muxContext);

    int getElementaryStreamCount();

    elementary_stream_type_t getElementaryStreamType(int esIndex);

    int getElementaryStreamVideoDimensions(int esIndex,
        unsigned int *width, unsigned int *height,
        unsigned int *cropLeft, unsigned int *cropRight,
        unsigned int *cropTop, unsigned int *cropBottom,
        unsigned int *sarWidth, unsigned int *sarHeight);

    int getElementaryStreamVideoFov(int esIndex, float *hfov, float *vfov);

    int setElementaryStreamDecoder(int esIndex, Decoder *decoder);

    int start();

    int pause();

    int stop();

    int seekTo
            (uint64_t timestamp);

    int seekForward
            (uint64_t delta);

    int seekBack
            (uint64_t delta);

    int startRecorder(const std::string &fileName);

    int stopRecorder();

    int startResender(const std::string &dstAddr, const std::string &ifaceAddr,
                      int srcStreamPort, int srcControlPort,
                      int dstStreamPort, int dstControlPort);

    int stopResender();

    uint64_t getDuration() { return (uint64_t)-1; };

    uint64_t getCurrentTime();

    Session *getSession() { return mSession; };

private:

    static void fetchSessionMetadata(StreamDemuxer *demuxer);

    static eARSTREAM2_ERROR h264FilterSpsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr);

    static eARSTREAM2_ERROR h264FilterGetAuBufferCallback(uint8_t **auBuffer, int *auBufferSize, void **auBufferUserPtr, void *userPtr);

    static eARSTREAM2_ERROR h264FilterAuReadyCallback(uint8_t *auBuffer, int auSize,
                                                      ARSTREAM2_StreamReceiver_AuReadyCallbackTimestamps_t *auTimestamps,
                                                      eARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE auSyncType,
                                                      ARSTREAM2_StreamReceiver_AuReadyCallbackMetadata_t *auMetadata,
                                                      void *auBufferUserPtr, void *userPtr);

    uint32_t mCurrentAuSize;
    int mMaxPacketSize;
    int mQosMode;
    AvcDecoder *mDecoder;
    Buffer *mCurrentBuffer;
    pthread_t mStreamNetworkThread;
    bool mStreamNetworkThreadLaunched;
    pthread_t mStreamOutputThread;
    bool mStreamOutputThreadLaunched;
    ARSTREAM2_StreamReceiver_Handle mStreamReceiver;
    ARSTREAM2_StreamReceiver_ResenderHandle mResender;
    uint64_t mStartTime;
    uint64_t mCurrentTime;
    uint64_t mLastSessionMetadataFetchTime;
    unsigned int mWidth;
    unsigned int mHeight;
    unsigned int mCropLeft;
    unsigned int mCropRight;
    unsigned int mCropTop;
    unsigned int mCropBottom;
    unsigned int mSarWidth;
    unsigned int mSarHeight;
    float mHfov;
    float mVfov;
};

}

#endif /* !_PDRAW_DEMUXER_STREAM_HPP_ */
