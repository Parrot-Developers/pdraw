/**
 * @file pdraw_filter_videoframe.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - video frame filter
 * @date 03/05/2017
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

#ifndef _PDRAW_FILTER_VIDEOFRAME_HPP_
#define _PDRAW_FILTER_VIDEOFRAME_HPP_

#include <pthread.h>

#include <pdraw/pdraw_defs.h>

#include "pdraw_avcdecoder.hpp"


namespace Pdraw
{


class Media;
class VideoMedia;


class VideoFrameFilter
{
public:

    VideoFrameFilter(VideoMedia *media, AvcDecoder *decoder);

    VideoFrameFilter(VideoMedia *media, AvcDecoder *decoder, pdraw_video_frame_filter_callback_t cb, void *userPtr);

    ~VideoFrameFilter();

    /*
     * waitUs : wait a frame, time in microseconds.
     *  0: don't wait
     * -1: wait forever
     * >0: wait time
     */
    int getLastFrame(pdraw_video_frame_t *frame, long waitUs = 0);

    Media *getMedia() { return mMedia; };

    VideoMedia *getVideoMedia() { return (VideoMedia*)mMedia; };

private:

    static void* runThread(void *ptr);

    Media *mMedia;
    AvcDecoder *mDecoder;
    BufferQueue *mDecoderOutputBufferQueue;
    pthread_mutex_t mMutex;
    pthread_t mThread;
    pthread_cond_t mCondition;
    bool mThreadLaunched;
    bool mThreadShouldStop;
    pdraw_video_frame_filter_callback_t mCb;
    void *mUserPtr;
    uint8_t *mBuffer[2];
    pdraw_video_frame_t mBufferData[2];
    unsigned int mBufferIndex;
    pdraw_color_format_t mColorFormat;
    unsigned int mWidth;
    unsigned int mHeight;
    bool mFrameAvailable;
};

}

#endif /* !_PDRAW_FILTER_VIDEOFRAME_HPP_ */
