/**
 * @file pdraw_demuxer_record.cpp
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

#include "pdraw_demuxer_record.hpp"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


RecordDemuxer::RecordDemuxer()
{
    mConfigured = false;
    mDemux = NULL;
    mRunning = false;
    mThreadShouldStop = false;
    mVideoTrackCount = 0;
    mVideoTrackId = 0;
    mMetadataMimeType = NULL;
    mFirstFrame = true;
    mDecoder = NULL;
    mLastFrameOutputTime = 0;
    mLastFrameTimestamp = 0;
    mPendingSeekTs = -1;

    mMetadataBufferSize = 1024;
    mMetadataBuffer = (uint8_t*)malloc(mMetadataBufferSize);
    if (mMetadataBuffer == NULL)
    {
        ULOGE("RecordDemuxer: allocation failed size %d", mMetadataBufferSize);
    }

    int ret = pthread_mutex_init(&mDemuxerMutex, NULL);
    if (ret != 0)
    {
        ULOGE("RecordDemuxer: mutex creation failed (%d)", ret);
    }
}


RecordDemuxer::~RecordDemuxer()
{
    mThreadShouldStop = true;

    int thErr = pthread_join(mDemuxerThread, NULL);
    if (thErr != 0)
    {
        ULOGE("RecordDemuxer: pthread_join() failed (%d)", thErr);
    }

    pthread_mutex_destroy(&mDemuxerMutex);

    if (mDemux) mp4_demux_close(mDemux);
    free(mMetadataBuffer);
    free(mMetadataMimeType);
}


int RecordDemuxer::configure(const std::string &url)
{
    int ret = 0;

    if (mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is already configured");
        return -1;
    }

    mFileName = url;

    if (ret == 0)
    {
        mDemux = mp4_demux_open(mFileName.c_str());
        if (mDemux == NULL)
        {
            ULOGE("RecordDemuxer: mp4_demux_open() failed");
            ret = -1;
        }

        int i, tkCount = 0, found = 0;
        mp4_media_info_t info;
        mp4_track_info_t tk;

        ret = mp4_demux_get_media_info(mDemux, &info);
        if (ret == 0)
        {
            mDuration = info.duration;
            tkCount = info.track_count;
            ULOGI("RecordDemuxer: track count: %d", tkCount);
            unsigned int hrs = (unsigned int)((info.duration + 500000) / 1000000 / 60 / 60);
            unsigned int min = (unsigned int)((info.duration + 500000) / 1000000 / 60 - hrs * 60);
            unsigned int sec = (unsigned int)((info.duration + 500000) / 1000000 - hrs * 60 * 60 - min * 60);
            ULOGI("RecordDemuxer: duration: %02d:%02d:%02d", hrs, min, sec);
        }

        for (i = 0; i < tkCount; i++)
        {
            ret = mp4_demux_get_track_info(mDemux, i, &tk);
            if ((ret == 0) && (tk.type == MP4_TRACK_TYPE_VIDEO))
            {
                mVideoTrackId = tk.id;
                mVideoTrackCount++;
                if (tk.has_metadata)
                {
                    mMetadataMimeType = strdup(tk.metadata_mime_format);
                }
                found = 1;
                break;
            }
        }

        if (found)
        {
            ULOGI("RecordDemuxer: video track ID: %d", mVideoTrackId);
        }
        else
        {
            ULOGE("RecordDemuxer: failed to find a video track");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mDemuxerThread, NULL, runDemuxerThread, (void*)this);
        if (thErr != 0)
        {
            ULOGE("RecordDemuxer: demuxer thread creation failed (%d)", thErr);
        }
    }

    mConfigured = (ret == 0) ? true : false;
    if (mConfigured)
    {
        ULOGI("RecordDemuxer: demuxer is configured");        
    }

    return ret;
}


int RecordDemuxer::getElementaryStreamCount()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    return mVideoTrackCount;
}


elementary_stream_type_t RecordDemuxer::getElementaryStreamType(int esIndex)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return (elementary_stream_type_t)-1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return (elementary_stream_type_t)-1;
    }

    //TODO
    return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;
}


int RecordDemuxer::setElementaryStreamDecoder(int esIndex, Decoder *decoder)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }
    if (!decoder)
    {
        ULOGE("RecordDemuxer: invalid decoder");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return -1;
    }

    mDecoder = (AvcDecoder*)decoder;

    return 0;
}


int RecordDemuxer::start()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mRunning = true;

    return 0;
}


int RecordDemuxer::pause()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mRunning = false;

    return 0;
}


int RecordDemuxer::stop()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mThreadShouldStop = true;

    return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    if (timestamp > mDuration) timestamp = mDuration;
    mPendingSeekTs = (int64_t)timestamp;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekForward(uint64_t delta)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    int64_t ts = (int64_t)mLastFrameTimestamp + (int64_t)delta;
    if (ts < 0) ts = 0;
    if (ts > (int64_t)mDuration) ts = mDuration;
    mPendingSeekTs = ts;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekBack(uint64_t delta)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    int64_t ts = (int64_t)mLastFrameTimestamp - (int64_t)delta;
    if (ts < 0) ts = 0;
    if (ts > (int64_t)mDuration) ts = mDuration;
    mPendingSeekTs = ts;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


void* RecordDemuxer::runDemuxerThread(void *ptr)
{
    RecordDemuxer *demuxer = (RecordDemuxer*)ptr;
    avc_decoder_input_buffer_t *outputBuffer = NULL;
    avc_decoder_input_buffer_t outBuf;
    bool moreFrames = true;
    struct timespec t1;
    uint64_t curTime;
    int32_t outputTimeError = 0;

    while ((!demuxer->mThreadShouldStop) && (moreFrames))
    {
        if ((demuxer->mDecoder) && (demuxer->mRunning))
        {
            uint8_t *sps = NULL, *pps = NULL;
            unsigned int spsSize = 0, ppsSize = 0;
            mp4_track_sample_t sample;
            int ret;

            if (demuxer->mFirstFrame)
            {
                ret = mp4_demux_get_track_avc_decoder_config(demuxer->mDemux, demuxer->mVideoTrackId,
                                                              &sps, &spsSize, &pps, &ppsSize);
                if (ret != 0)
                {
                    ULOGE("RecordDemuxer: failed to get decoder configuration (%d)", ret);
                }
                else
                {
                    ret = demuxer->mDecoder->configure(sps, (unsigned int)spsSize, pps, (unsigned int)ppsSize);
                    if (ret != 0)
                    {
                        ULOGE("RecordDemuxer: decoder configuration failed (%d)", ret);
                    }
                }
            }

            if ((demuxer->mDecoder->isConfigured()) && (outputBuffer == NULL))
            {
                /*clock_gettime(CLOCK_MONOTONIC, &t1);
                uint64_t startTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;*/
                ret = demuxer->mDecoder->getInputBuffer(&outBuf, true);
                /*clock_gettime(CLOCK_MONOTONIC, &t1);
                uint64_t endTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;*/
                if (ret != 0)
                {
                    ULOGW("RecordDemuxer: failed to get an output buffer (%d)", ret);
                }
                else
                {
                    outputBuffer = &outBuf;
                    //ULOGI("RecordDemuxer: getInputBuffer time = %.2fms", (float)(endTime - startTime) / 1000.);
                }
            }

            if (outputBuffer)
            {
                uint8_t *buf = outputBuffer->auBuffer;
                unsigned int bufSize = outputBuffer->auBufferSize;

                if (demuxer->mFirstFrame)
                {
                    if ((sps) && (spsSize + 4 <= bufSize))
                    {
                        *((uint32_t*)buf) = htonl(0x00000001);
                        memcpy(buf + 4, sps, spsSize);
                        buf += (spsSize + 4);
                        bufSize -= (spsSize + 4);
                    }
                    if ((pps) && (ppsSize + 4 <= bufSize))
                    {
                        *((uint32_t*)buf) = htonl(0x00000001);
                        memcpy(buf + 4, pps, ppsSize);
                        buf += (ppsSize + 4);
                        bufSize -= (ppsSize + 4);
                    }
                    demuxer->mFirstFrame = false;
                }

                pthread_mutex_lock(&demuxer->mDemuxerMutex);
                int64_t seekTs = demuxer->mPendingSeekTs;
                demuxer->mPendingSeekTs = -1;
                pthread_mutex_unlock(&demuxer->mDemuxerMutex);

                if (seekTs >= 0)
                {
                    ret = mp4_demux_seek(demuxer->mDemux, (uint64_t)seekTs, 1);
                    if (ret != 0)
                    {
                        ULOGW("RecordDemuxer: mp4_demux_seek() failed (%d)", ret);
                    }
                    else
                    {
                        demuxer->mLastFrameTimestamp = 0;
                        outputTimeError = 0;
                    }
                }

                ret = mp4_demux_get_track_next_sample(demuxer->mDemux, demuxer->mVideoTrackId,
                                                      buf, bufSize, demuxer->mMetadataBuffer, demuxer->mMetadataBufferSize, &sample);
                if ((ret == 0) && (sample.sample_size))
                {
                    outputBuffer->auSize = sample.sample_size;

                    /* Fix the H.264 bitstream: replace NALU size by byte stream start codes */
                    uint32_t offset = 0, naluSize, naluCount = 0;
                    uint8_t *_buf = buf;
                    while (offset < sample.sample_size)
                    {
                        naluSize = ntohl(*((uint32_t*)_buf));
                        *((uint32_t*)_buf) = htonl(0x00000001);
                        _buf += 4 + naluSize;
                        offset += 4 + naluSize;
                        naluCount++;
                    }

                    outputBuffer->isComplete = true; //TODO?
                    outputBuffer->hasErrors = false; //TODO?
                    outputBuffer->isRef = true; //TODO?
                    outputBuffer->auNtpTimestamp = sample.sample_dts;
                    outputBuffer->auNtpTimestampRaw = sample.sample_dts;
                    //TODO: auSyncType

                    /* Metadata */
                    outputBuffer->hasMetadata = Metadata::decodeFrameMetadata(demuxer->mMetadataBuffer, sample.metadata_size,
                                                                              FRAME_METADATA_SOURCE_RECORDING, demuxer->mMetadataMimeType, &outputBuffer->metadata);

                    if ((demuxer->mLastFrameOutputTime) && (demuxer->mLastFrameTimestamp))
                    {
                        clock_gettime(CLOCK_MONOTONIC, &t1);
                        curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                        int32_t sleepTime = (int32_t)((int64_t)(sample.sample_dts - demuxer->mLastFrameTimestamp) - (int64_t)(curTime - demuxer->mLastFrameOutputTime)) + outputTimeError;
                        if (sleepTime >= 1000)
                        {
                            usleep(sleepTime);
                        }
                    }

                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    outputBuffer->demuxOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                    outputBuffer->auNtpTimestampLocal = outputBuffer->demuxOutputTimestamp;
                    outputTimeError = ((demuxer->mLastFrameOutputTime) && (demuxer->mLastFrameTimestamp)) ?
                                        (int32_t)((int64_t)(sample.sample_dts - demuxer->mLastFrameTimestamp) - (int64_t)(outputBuffer->demuxOutputTimestamp - demuxer->mLastFrameOutputTime)) : 0;

                    ret = demuxer->mDecoder->queueInputBuffer(outputBuffer);
                    if (ret != 0)
                    {
                        ULOGW("RecordDemuxer: failed to release the output buffer (%d)", ret);
                    }
                    else
                    {
                        demuxer->mLastFrameOutputTime = outputBuffer->demuxOutputTimestamp;
                        demuxer->mLastFrameTimestamp = sample.sample_dts;
                        outputBuffer = NULL;
                    }
                }
            }
            else
            {
                usleep(5000); //TODO
            }
        }
        else
        {
            usleep(5000); //TODO
        }
    }

    return NULL;
}

}
