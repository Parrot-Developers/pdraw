/**
 * @file pdraw_filter_videoframe.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - video frame filter
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

#include "pdraw_filter_videoframe.hpp"

#include <sys/time.h>
#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>


#define VIDEO_FRAME_FILTER_USER_DATA_ALLOC_SIZE 1024


namespace Pdraw
{


VideoFrameFilter::VideoFrameFilter(VideoMedia *media, AvcDecoder *decoder) : VideoFrameFilter(media, decoder, NULL, NULL)
{
}


VideoFrameFilter::VideoFrameFilter(VideoMedia *media, AvcDecoder *decoder, pdraw_video_frame_filter_callback_t cb, void *userPtr)
{
    int ret = 0;
    mMedia = (Media*)media;
    mDecoder = NULL;
    mDecoderOutputBufferQueue = NULL;
    mThreadLaunched = false;
    mThreadShouldStop = false;
    mCb = cb;
    mUserPtr = userPtr;
    mBuffer[0] = NULL;
    mBuffer[1] = NULL;
    mUserData[0] = NULL;
    mUserData[1] = NULL;
    mUserDataBuferSize[0] = 0;
    mUserDataBuferSize[1] = 0;
    mBufferIndex = 0;
    mColorFormat = PDRAW_COLOR_FORMAT_UNKNOWN;
    mWidth = 0;
    mHeight = 0;
    mFrameAvailable = false;
    mCondition = PTHREAD_COND_INITIALIZER;

    if (!media)
    {
        ULOGE("VideoFrameFilter: invalid media");
        ret = -1;
    }

    if (ret == 0)
    {
        if (decoder)
        {
            mDecoderOutputBufferQueue = decoder->addOutputQueue();
            if (mDecoderOutputBufferQueue == NULL)
            {
                ULOGE("VideoFrameFilter: failed to add output queue to decoder");
                ret = -1;
            }

            mDecoder = decoder;
        }
        else
        {
            ULOGE("VideoFrameFilter: invalid decoder");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = pthread_mutex_init(&mMutex, NULL);
        if (ret != 0)
        {
            ULOGE("VideoFrameFilter: mutex creation failed (%d)", ret);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int ret = pthread_create(&mThread, NULL, runThread, (void*)this);
        if (ret != 0)
        {
            ULOGE("VideoFrameFilter: thread creation failed (%d)", ret);
            ret = -1;
        }
        else
        {
            mThreadLaunched = true;
        }
    }
}


VideoFrameFilter::~VideoFrameFilter()
{
    mThreadShouldStop = true;
    if (mDecoderOutputBufferQueue) mDecoderOutputBufferQueue->signal();

    if (mThreadLaunched)
    {
        int thErr = pthread_join(mThread, NULL);
        if (thErr != 0)
        {
            ULOGE("VideoFrameFilter: pthread_join() failed (%d)", thErr);
        }
    }

    /*
     * this won't be sufficiant if another thread get last frame with
     * wait condition ; let's assume the client won't stop while
     * getting a frame
     */
    pthread_mutex_lock(&mMutex);
    mFrameAvailable = false;
    pthread_cond_broadcast(&mCondition);
    pthread_mutex_unlock(&mMutex);

    if (mDecoder)
    {
        if (mDecoderOutputBufferQueue)
        {
            int ret = mDecoder->removeOutputQueue(mDecoderOutputBufferQueue);
            if (ret != 0)
            {
                ULOGE("VideoFrameFilter: failed to remove output queue from decoder");
            }
        }
    }

    free(mBuffer[0]);
    free(mBuffer[1]);
    free(mUserData[0]);
    free(mUserData[1]);

    pthread_mutex_destroy(&mMutex);
}


static void getTimeFromTimeout(struct timespec* ts, int timeout)
{
    struct timeval tp;
    gettimeofday(&tp, NULL);

    ts->tv_sec = tp.tv_sec;
    ts->tv_nsec = tp.tv_usec * 1000 + timeout * 1000;
    ts->tv_sec += ts->tv_nsec / 1000000000L;
    ts->tv_nsec = ts->tv_nsec % 1000000000L;
}


int VideoFrameFilter::getLastFrame(struct pdraw_video_frame *frame, int timeout)
{
    if (!frame)
    {
        ULOGE("VideoFrameFilter: invalid frame structure pointer");
        return -1;
    }
    if (mCb)
    {
        ULOGE("VideoFrameFilter: unsupported in callback mode");
        return -1;
    }

    pthread_mutex_lock(&mMutex);

    if (timeout && !mFrameAvailable) {
        if (timeout == -1) {
            pthread_cond_wait(&mCondition, &mMutex);
        } else {
            struct timespec ts;
            getTimeFromTimeout(&ts, timeout);
            pthread_cond_timedwait(&mCondition, &mMutex, &ts);
        }
    }

    if (!mFrameAvailable)
    {
        pthread_mutex_unlock(&mMutex);
        ULOGI("VideoFrameFilter: no frame available");
        return -2;
    }

    mBufferIndex ^= 1;
    memcpy(frame, &mBufferData[mBufferIndex], sizeof(*frame));

    mFrameAvailable = false;
    pthread_mutex_unlock(&mMutex);

    return 0;
}


void* VideoFrameFilter::runThread(void *ptr)
{
    VideoFrameFilter *filter = (VideoFrameFilter*)ptr;
    int ret;

    while (!filter->mThreadShouldStop)
    {
        if ((filter->mDecoder) && (filter->mDecoder->isConfigured()))
        {
            Buffer *buffer;

            ret = filter->mDecoder->dequeueOutputBuffer(filter->mDecoderOutputBufferQueue, &buffer, true);
            if (ret == 0)
            {
                avc_decoder_output_buffer_t *data = (avc_decoder_output_buffer_t*)buffer->getMetadataPtr();
                struct pdraw_video_frame frame;
                memset(&frame, 0, sizeof(frame));
                switch(data->colorFormat)
                {
                    default:
                    case AVCDECODER_COLORFORMAT_UNKNOWN:
                        frame.colorFormat = PDRAW_COLOR_FORMAT_UNKNOWN;
                        break;
                    case AVCDECODER_COLORFORMAT_YUV420PLANAR:
                        frame.colorFormat = PDRAW_COLOR_FORMAT_YUV420PLANAR;
                        break;
                    case AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR:
                        frame.colorFormat = PDRAW_COLOR_FORMAT_YUV420SEMIPLANAR;
                        break;
                }
                frame.plane[0] = data->plane[0];
                frame.plane[1] = data->plane[1];
                frame.plane[2] = data->plane[2];
                frame.stride[0] = data->stride[0];
                frame.stride[1] = data->stride[1];
                frame.stride[2] = data->stride[2];
                frame.width = data->width;
                frame.height = data->height;
                frame.sarWidth = data->sarWidth;
                frame.sarHeight = data->sarHeight;
                frame.isComplete = (data->isComplete) ? 1 : 0;
                frame.hasErrors = (data->hasErrors) ? 1 : 0;
                frame.isRef = (data->isRef) ? 1 : 0;
                frame.auNtpTimestamp = data->auNtpTimestamp;
                frame.auNtpTimestampRaw = data->auNtpTimestampRaw;
                frame.auNtpTimestampLocal = data->auNtpTimestampLocal;
                frame.hasMetadata = (data->hasMetadata) ? 1 : 0;
                memcpy(&frame.metadata, &data->metadata, sizeof(frame.metadata));
                frame.userData = (uint8_t *)buffer->getUserDataPtr();
                frame.userDataSize = buffer->getUserDataSize();

                if (filter->mCb)
                {
                    filter->mCb(filter, &frame, filter->mUserPtr);
                }
                else
                {
                    if ((filter->mWidth == 0) || (filter->mHeight == 0)
                            || (filter->mColorFormat == PDRAW_COLOR_FORMAT_UNKNOWN))
                    {
                        unsigned int size = frame.width * frame.height * 3 / 2;
                        filter->mBuffer[0] = (uint8_t*)malloc(size);
                        if (filter->mBuffer[0] == NULL)
                        {
                            ULOGE("VideoFrameFilter: frame allocation failed (size %d)", size);
                        }
                        filter->mBuffer[1] = (uint8_t*)malloc(size);
                        if (filter->mBuffer[1] == NULL)
                        {
                            ULOGE("VideoFrameFilter: frame allocation failed (size %d)", size);
                        }
                        if ((filter->mBuffer[0] != NULL) && (filter->mBuffer[1]))
                        {
                            filter->mWidth = frame.width;
                            filter->mHeight = frame.height;
                            filter->mColorFormat = frame.colorFormat;
                        }
                    }

                    if ((frame.width != filter->mWidth) || (frame.height != filter->mHeight)
                            || (frame.colorFormat != filter->mColorFormat))
                    {
                        ULOGW("VideoFrameFilter: unsupported change of frame format");
                    }
                    else
                    {
                        pthread_mutex_lock(&filter->mMutex);

                        switch(frame.colorFormat)
                        {
                            default:
                            case PDRAW_COLOR_FORMAT_UNKNOWN:
                                break;
                            case PDRAW_COLOR_FORMAT_YUV420PLANAR:
                            {
                                uint8_t *pSrcY = data->plane[0];
                                uint8_t *pSrcU = data->plane[1];
                                uint8_t *pSrcV = data->plane[2];
                                uint8_t *pDstY = frame.plane[0] = filter->mBuffer[filter->mBufferIndex ^ 1];
                                uint8_t *pDstU = frame.plane[1] = filter->mBuffer[filter->mBufferIndex ^ 1] + filter->mWidth * filter->mHeight;
                                uint8_t *pDstV = frame.plane[2] = filter->mBuffer[filter->mBufferIndex ^ 1] + filter->mWidth * filter->mHeight * 5 / 4;
                                frame.stride[0] = filter->mWidth;
                                frame.stride[1] = filter->mWidth / 2;
                                frame.stride[2] = filter->mWidth / 2;
                                unsigned int y;
                                for (y = 0; y < filter->mHeight; y++)
                                {
                                    memcpy(pDstY, pSrcY, filter->mWidth);
                                    pSrcY += data->stride[0];
                                    pDstY += filter->mWidth;
                                }
                                for (y = 0; y < filter->mHeight / 2; y++)
                                {
                                    memcpy(pDstU, pSrcU, filter->mWidth / 2);
                                    memcpy(pDstV, pSrcV, filter->mWidth / 2);
                                    pSrcU += data->stride[1];
                                    pSrcV += data->stride[2];
                                    pDstU += filter->mWidth / 2;
                                    pDstV += filter->mWidth / 2;
                                }
                                break;
                            }
                            case PDRAW_COLOR_FORMAT_YUV420SEMIPLANAR:
                            {
                                uint8_t *pSrcY = data->plane[0];
                                uint8_t *pSrcUV = data->plane[1];
                                uint8_t *pDstY = frame.plane[0] = filter->mBuffer[filter->mBufferIndex ^ 1];
                                uint8_t *pDstUV = frame.plane[1] = filter->mBuffer[filter->mBufferIndex ^ 1] + filter->mWidth * filter->mHeight;
                                frame.stride[0] = filter->mWidth;
                                frame.stride[1] = filter->mWidth;
                                unsigned int y;
                                for (y = 0; y < filter->mHeight; y++)
                                {
                                    memcpy(pDstY, pSrcY, filter->mWidth);
                                    pSrcY += data->stride[0];
                                    pDstY += filter->mWidth;
                                }
                                for (y = 0; y < filter->mHeight / 2; y++)
                                {
                                    memcpy(pDstUV, pSrcUV, filter->mWidth);
                                    pSrcUV += data->stride[1];
                                    pDstUV += filter->mWidth;
                                }
                                break;
                            }
                        }

                        memcpy(&filter->mBufferData[filter->mBufferIndex ^ 1], &frame, sizeof(frame));

                        /* user data */
                        if ((frame.userData) && (frame.userDataSize))
                        {
                            if (frame.userDataSize > filter->mUserDataBuferSize[filter->mBufferIndex ^ 1])
                            {
                                unsigned int size = (frame.userDataSize + VIDEO_FRAME_FILTER_USER_DATA_ALLOC_SIZE - 1) & (~(VIDEO_FRAME_FILTER_USER_DATA_ALLOC_SIZE - 1));
                                uint8_t *tmp = (uint8_t *)realloc(filter->mUserData[filter->mBufferIndex ^ 1], size);
                                if (tmp)
                                {
                                    filter->mUserData[filter->mBufferIndex ^ 1] = tmp;
                                    filter->mUserDataBuferSize[filter->mBufferIndex ^ 1] = size;
                                }
                            }
                            if (frame.userDataSize <= filter->mUserDataBuferSize[filter->mBufferIndex ^ 1])
                            {
                                memcpy(filter->mUserData[filter->mBufferIndex ^ 1], frame.userData, frame.userDataSize);
                                filter->mBufferData[filter->mBufferIndex ^ 1].userData = filter->mUserData[filter->mBufferIndex ^ 1];
                                filter->mBufferData[filter->mBufferIndex ^ 1].userDataSize = frame.userDataSize;
                            }
                            else
                            {
                                filter->mBufferData[filter->mBufferIndex ^ 1].userData = NULL;
                                filter->mBufferData[filter->mBufferIndex ^ 1].userDataSize = 0;
                            }
                        }

                        filter->mFrameAvailable = true;
                        pthread_mutex_unlock(&filter->mMutex);
                        pthread_cond_signal(&filter->mCondition);
                    }
                }

                ret = filter->mDecoder->releaseOutputBuffer(buffer);
                if (ret != 0)
                {
                    ULOGE("VideoFrameFilter: failed to release buffer (%d)", ret);
                }
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
