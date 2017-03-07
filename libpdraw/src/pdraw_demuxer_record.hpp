/**
 * @file pdraw_demuxer_record.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - recording demuxer
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

#ifndef _PDRAW_DEMUXER_RECORD_HPP_
#define _PDRAW_DEMUXER_RECORD_HPP_

#include <pthread.h>

#include <libmp4.h>

#include "pdraw_demuxer.hpp"
#include "pdraw_avcdecoder.hpp"


namespace Pdraw
{


class RecordDemuxer : public Demuxer
{
public:

    RecordDemuxer();

    ~RecordDemuxer();

    demuxer_type_t getType() { return DEMUXER_TYPE_RECORD; };

    bool isConfigured() { return mConfigured; };

    int configure(const std::string &url);

    int getElementaryStreamCount();

    elementary_stream_type_t getElementaryStreamType(int esIndex);

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

private:

    static void* runDemuxerThread(void *ptr);

    std::string mFileName;
    AvcDecoder *mDecoder;
    pthread_t mDemuxerThread;
    bool mDemuxerThreadLaunched;
    pthread_mutex_t mDemuxerMutex;
    int mRunning;
    int mThreadShouldStop;
    struct mp4_demux *mDemux;
    uint64_t mDuration;
    int mVideoTrackCount;
    unsigned int mVideoTrackId;
    char *mMetadataMimeType;
    int mFirstFrame;
    unsigned int mMetadataBufferSize;
    uint8_t *mMetadataBuffer;
    uint64_t mLastFrameOutputTime;
    uint64_t mLastFrameTimestamp;
    int64_t mPendingSeekTs;
};

}

#endif /* !_PDRAW_DEMUXER_RECORD_HPP_ */
