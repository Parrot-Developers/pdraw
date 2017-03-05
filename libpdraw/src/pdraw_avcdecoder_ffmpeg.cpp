/**
 * @file pdraw_avcdecoder_ffmpeg.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - ffmpeg H.264/AVC video decoder
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

#include "pdraw_avcdecoder_ffmpeg.hpp"

#ifdef USE_FFMPEG

#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

//#define FFMPEG_LOG_LEVEL (AV_LOG_WARNING)
#define FFMPEG_LOG_LEVEL (AV_LOG_VERBOSE)
//#define FFMPEG_LOG_LEVEL (AV_LOG_QUIET)


namespace Pdraw
{


FfmpegOutputBufferQueueBuffer::FfmpegOutputBufferQueueBuffer(void *userPtr)
{
    mUserPtr = userPtr;
    mAvFrame = av_frame_alloc();
    if (mAvFrame == NULL)
    {
        ULOGE("ffmpeg: frame allocation failed");
    }
}


FfmpegOutputBufferQueueBuffer::~FfmpegOutputBufferQueueBuffer()
{
    av_frame_free(&mAvFrame);
}


AVFrame *FfmpegOutputBufferQueueBuffer::getFrame()
{
    return mAvFrame;
}


void *FfmpegOutputBufferQueueBuffer::getUserPtr()
{
    return mUserPtr;
}


void FfmpegOutputBufferQueueBuffer::getData(avc_decoder_output_buffer_t *data)
{
    if (data)
        memcpy((void*)data, (void*)&mData, sizeof(avc_decoder_output_buffer_t));
}


void FfmpegOutputBufferQueueBuffer::setData(const avc_decoder_output_buffer_t *data)
{
    if (data)
        memcpy((void*)&mData, (const void*)data, sizeof(avc_decoder_output_buffer_t));
}


FfmpegOutputBufferQueue::FfmpegOutputBufferQueueBufferInternal::FfmpegOutputBufferQueueBufferInternal(void *userPtr) :
    FfmpegOutputBufferQueueBuffer(userPtr)
{
    mStatus = BUFFER_FREE;
}


FfmpegOutputBufferQueue::FfmpegOutputBufferQueueBufferInternal::~FfmpegOutputBufferQueueBufferInternal()
{
}


FfmpegOutputBufferQueue::FfmpegOutputBufferQueue(unsigned int nbElement)
{
    int ret;

    mBuffers = new vector<FfmpegOutputBufferQueueBufferInternal*>(nbElement);

    /* Allocate all buffers */
    unsigned int i;
    uint8_t *usr;
    for (i = 0, usr = NULL; i < mBuffers->size(); i++, usr++)
    {
        (*mBuffers)[i] = new FfmpegOutputBufferQueueBufferInternal((void*)usr);
    }

    mInputQueue = new queue<FfmpegOutputBufferQueueBufferInternal*>;
    mOutputQueue = new queue<FfmpegOutputBufferQueueBufferInternal*>;

    /* Add all buffers in the input queue */
    for (i = 0; i < mBuffers->size(); i++)
    {
        mInputQueue->push(((*mBuffers)[i]));
    }

    ret = pthread_mutex_init(&mInputMutex, NULL);
    if (ret != 0)
    {
        ULOGE("Mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mInputCond, NULL);
    if (ret != 0)
    {
        ULOGE("Cond creation failed (%d)", ret);
    }
    ret = pthread_mutex_init(&mOutputMutex, NULL);
    if (ret != 0)
    {
        ULOGE("Mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mOutputCond, NULL);
    if (ret != 0)
    {
        ULOGE("Cond creation failed (%d)", ret);
    }
}


FfmpegOutputBufferQueue::~FfmpegOutputBufferQueue()
{
    unsigned int i;

    pthread_mutex_destroy(&mInputMutex);
    pthread_cond_destroy(&mInputCond);
    pthread_mutex_destroy(&mOutputMutex);
    pthread_cond_destroy(&mOutputCond);

    for (i = 0; i < mBuffers->size(); i++)
    {
        delete (*mBuffers)[i];
    }

    delete mBuffers;
    delete mInputQueue;
    delete mOutputQueue;
}


bool FfmpegOutputBufferQueue::bufferIsValid(FfmpegOutputBufferQueueBufferInternal* buffer)
{
    /* Find the buffer */
    unsigned int i;
    for (i = 0; i < mBuffers->size(); i++)
    {
        if (buffer == (*mBuffers)[i])
        {
            return true;
        }
    }
    return false;
}


buffer_queue_status_t FfmpegOutputBufferQueue::getInputBuffer(FfmpegOutputBufferQueueBuffer **buffer, bool blocking)
{
    FfmpegOutputBufferQueueBufferInternal *lockedBuffer = NULL;

    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    pthread_mutex_lock(&mInputMutex);

    if (mInputQueue->size() == 0)
    {
        if (blocking)
        {
            /* Queue empty, wait for a new buffer */
            pthread_cond_wait(&mInputCond, &mInputMutex);
        }
        else
        {
            pthread_mutex_unlock(&mInputMutex);
            return BUFFERQUEUE_NO_INPUT_BUFFER_AVAILABLE;
        }
    }
    if (mInputQueue->size() != 0)
    {
        lockedBuffer = mInputQueue->front();
        mInputQueue->pop();
        lockedBuffer->mStatus = FfmpegOutputBufferQueueBufferInternal::BUFFER_INPUT_LOCK;
    }
    else
    {
        pthread_mutex_unlock(&mInputMutex);
        return BUFFERQUEUE_NO_INPUT_BUFFER_AVAILABLE;
    }

    pthread_mutex_unlock(&mInputMutex);

    *buffer = (FfmpegOutputBufferQueueBuffer*)lockedBuffer;
    return BUFFERQUEUE_OK;
}


buffer_queue_status_t FfmpegOutputBufferQueue::releaseInputBuffer(FfmpegOutputBufferQueueBuffer *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct */
    FfmpegOutputBufferQueueBufferInternal *bufferInternal = (FfmpegOutputBufferQueueBufferInternal*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != FfmpegOutputBufferQueueBufferInternal::BUFFER_INPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mOutputMutex);

    /* Put input buffer in the ouput queue */
    bufferInternal->mStatus = FfmpegOutputBufferQueueBufferInternal::BUFFER_FREE;
    mOutputQueue->push(bufferInternal);
    if (mOutputQueue->size() == 1)
    {
        /* The ouput queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mOutputCond);
    }

    pthread_mutex_unlock(&mOutputMutex);

    return BUFFERQUEUE_OK;
}


buffer_queue_status_t FfmpegOutputBufferQueue::cancelInputBuffer(FfmpegOutputBufferQueueBuffer *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct */
    FfmpegOutputBufferQueueBufferInternal *bufferInternal = (FfmpegOutputBufferQueueBufferInternal*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != FfmpegOutputBufferQueueBufferInternal::BUFFER_INPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mInputMutex);

    /* Leave the input buffer in the input queue */
    bufferInternal->mStatus = FfmpegOutputBufferQueueBufferInternal::BUFFER_FREE;
    mInputQueue->push(bufferInternal);
    if (mInputQueue->size() == 1)
    {
        /* The input queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mInputCond);
    }

    pthread_mutex_unlock(&mInputMutex);

    return BUFFERQUEUE_OK;
}


buffer_queue_status_t FfmpegOutputBufferQueue::getOutputBuffer(FfmpegOutputBufferQueueBuffer **buffer, bool blocking)
{
    FfmpegOutputBufferQueueBufferInternal *lockedBuffer=NULL;

    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    pthread_mutex_lock(&mOutputMutex);

    if (mOutputQueue->size() == 0)
    {
        if (blocking)
        {
            /* Queue empty, wait for a new buffer */
            pthread_cond_wait(&mOutputCond, &mOutputMutex);
        }
        else
        {
            pthread_mutex_unlock(&mOutputMutex);
            return BUFFERQUEUE_NO_OUTPUT_BUFFER_AVAILABLE;
        }
    }
    if (mOutputQueue->size() != 0)
    {
        lockedBuffer = mOutputQueue->front();
        mOutputQueue->pop();
        lockedBuffer->mStatus = FfmpegOutputBufferQueueBufferInternal::BUFFER_OUTPUT_LOCK;
    }
    else
    {
        pthread_mutex_unlock(&mOutputMutex);
        return BUFFERQUEUE_NO_OUTPUT_BUFFER_AVAILABLE;
    }

    pthread_mutex_unlock(&mOutputMutex);

    *buffer = (FfmpegOutputBufferQueueBuffer*)lockedBuffer;
    return BUFFERQUEUE_OK;
}


buffer_queue_status_t FfmpegOutputBufferQueue::releaseOutputBuffer(FfmpegOutputBufferQueueBuffer *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct */
    FfmpegOutputBufferQueueBufferInternal *bufferInternal = (FfmpegOutputBufferQueueBufferInternal*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != FfmpegOutputBufferQueueBufferInternal::BUFFER_OUTPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mInputMutex);

    /* Put output buffer in the input queue */
    bufferInternal->mStatus = FfmpegOutputBufferQueueBufferInternal::BUFFER_FREE;
    mInputQueue->push(bufferInternal);
    if (mInputQueue->size() == 1)
    {
        /* The ouput queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mInputCond);
    }

    pthread_mutex_unlock(&mInputMutex);

    return BUFFERQUEUE_OK;
}


buffer_queue_status_t FfmpegOutputBufferQueue::getBufferFromUserPtr(void *userPtr, FfmpegOutputBufferQueueBuffer **buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Find the buffer */
    unsigned int i;
    for (i = 0; i < mBuffers->size(); i++)
    {
        if (userPtr == (*mBuffers)[i]->getUserPtr())
        {
            *buffer = (*mBuffers)[i];
            return BUFFERQUEUE_OK;
        }
    }
    return BUFFERQUEUE_INVALID_ARGUMENTS;
}


FfmpegAvcDecoder::FfmpegAvcDecoder()
{
    mConfigured = false;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
    mInputQueue = NULL;
    mOutputQueue = NULL;
    mThreadShouldStop = false;
    mDecoderThreadLaunched = false;

    avcodec_register_all();
    av_log_set_level(FFMPEG_LOG_LEVEL);
    mCodecH264 = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (NULL == mCodecH264)
    {
        ULOGE("ffmpeg: codec not found");
        return;
    }

    mCodecCtxH264 = avcodec_alloc_context3(mCodecH264);
    if (NULL == mCodecH264)
    {
        ULOGE("ffmpeg: failed to allocate codec context");
        return;
    }

    mCodecCtxH264->pix_fmt = AV_PIX_FMT_YUV420P;
    mCodecCtxH264->skip_frame = AVDISCARD_DEFAULT;
    mCodecCtxH264->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
    mCodecCtxH264->skip_loop_filter = AVDISCARD_DEFAULT;
    mCodecCtxH264->workaround_bugs = FF_BUG_AUTODETECT;
    mCodecCtxH264->codec_type = AVMEDIA_TYPE_VIDEO;
    mCodecCtxH264->codec_id = AV_CODEC_ID_H264;
    mCodecCtxH264->skip_idct = AVDISCARD_DEFAULT;

    if (avcodec_open2(mCodecCtxH264, mCodecH264, NULL) < 0)
    {
        ULOGE("ffmpeg: failed to open codec");
        return;
    }

    av_init_packet(&mPacket);

    mFrameWidth = 0;
    mFrameHeight = 0;

    int thErr = pthread_create(&mDecoderThread, NULL, runDecoderThread, (void*)this);
    if (thErr != 0)
    {
        ULOGE("ffmpeg: decoder thread creation failed (%d)", thErr);
    }
    else
    {
        mDecoderThreadLaunched = true;
    }
}


FfmpegAvcDecoder::~FfmpegAvcDecoder()
{
    mThreadShouldStop = true;
    if (mDecoderThreadLaunched)
    {
        int thErr = pthread_join(mDecoderThread, NULL);
        if (thErr != 0)
        {
            ULOGE("ffmpeg: pthread_join() failed (%d)", thErr);
        }
    }

    if (mInputQueue) delete mInputQueue;
    if (mOutputQueue) delete mOutputQueue;

    avcodec_free_context(&mCodecCtxH264);
}


int FfmpegAvcDecoder::configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;

    if (mConfigured)
    {
        ULOGE("ffmpeg: decoder is already configured");
        return -1;
    }

    /* Nothing to decode here for ffmpeg: SPS/PPS will be decoded with the first picture */

    /* Input buffers queue allocation */
    if (ret == 0)
    {
        mInputQueue = new BufferQueue<avc_decoder_input_buffer_t>(FFMPEG_AVC_DECODER_INPUT_BUFFER_COUNT, FFMPEG_AVC_DECODER_INPUT_BUFFER_SIZE); //TODO: number of buffers and buffers size
        if (mInputQueue == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder input buffers queue");
            ret = -1;
        }
    }

    /* Output buffers queue allocation */
    if (ret == 0)
    {
        mOutputQueue = new FfmpegOutputBufferQueue(FFMPEG_AVC_DECODER_OUTPUT_BUFFER_COUNT); //TODO: number of buffers and buffers size
        if (mOutputQueue == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder output buffers queue");
            ret = -1;
        }
    }

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("ffmpeg: decoder is configured");        
    }

    return ret;
}


int FfmpegAvcDecoder::getInputBuffer(avc_decoder_input_buffer_t *buffer, bool blocking)
{
    if (!buffer)
    {
        ULOGE("ffmpeg: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    if (mInputQueue)
    {
        BufferQueueBuffer<avc_decoder_input_buffer_t> *buf = NULL;
        buffer_queue_status_t status = mInputQueue->getInputBuffer(&buf, blocking);
        if ((status == BUFFERQUEUE_OK) && (buf))
        {
            buf->getData(buffer);
            buffer->userPtr = buf->getUserPtr();
            buffer->auBuffer = buf->getPtr();;
            buffer->auBufferSize = buf->getCapacity();;
            buffer->auSize = 0;
            buffer->hasMetadata = false;
        }
        else
        {
            ULOGE("ffmpeg: failed to get an input buffer");
            return -1;
        }
    }
    else
    {
        ULOGE("ffmpeg: input queue has not been created");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::queueInputBuffer(avc_decoder_input_buffer_t *buffer)
{
    if (!buffer)
    {
        ULOGE("ffmpeg: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    if (mInputQueue)
    {
        BufferQueueBuffer<avc_decoder_input_buffer_t> *buf = NULL;
        buffer_queue_status_t status = mInputQueue->getBufferFromUserPtr(buffer->userPtr, &buf);
        if ((status == BUFFERQUEUE_OK) && (buf))
        {
            buf->setData(buffer);
            status = mInputQueue->releaseInputBuffer(buf);
            if (status != BUFFERQUEUE_OK)
            {
                ULOGE("ffmpeg: failed to release the input buffer");
                return -1;
            }
        }
        else
        {
            ULOGE("ffmpeg: ailed to find the input buffer");
            return -1;
        }
    }
    else
    {
        ULOGE("ffmpeg: input queue has not been created");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::dequeueOutputBuffer(avc_decoder_output_buffer_t *buffer, bool blocking)
{
    if (!buffer)
    {
        ULOGE("ffmpeg: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    if (mOutputQueue)
    {
        FfmpegOutputBufferQueueBuffer *buf = NULL;
        buffer_queue_status_t status = mOutputQueue->getOutputBuffer(&buf, blocking);
        if ((status == BUFFERQUEUE_OK) && (buf))
        {
            buf->getData(buffer);
            buffer->userPtr = buf->getUserPtr();
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            buffer->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
        }
        else
        {
            ULOGE("ffmpeg: failed to get an output buffer");
            return -1;
        }
    }
    else
    {
        ULOGE("ffmpeg: output queue has not been created");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::releaseOutputBuffer(avc_decoder_output_buffer_t *buffer)
{
    if (!buffer)
    {
        ULOGE("ffmpeg: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    if (mOutputQueue)
    {
        FfmpegOutputBufferQueueBuffer *buf = NULL;
        buffer_queue_status_t status = mOutputQueue->getBufferFromUserPtr(buffer->userPtr, &buf);
        if ((status == BUFFERQUEUE_OK) && (buf))
        {
            status = mOutputQueue->releaseOutputBuffer(buf);
            if (status != BUFFERQUEUE_OK)
            {
                ULOGE("ffmpeg: failed to release the output buffer");
                return -1;
            }
        }
        else
        {
            ULOGE("ffmpeg: failed to find the output buffer");
            return -1;
        }
    }
    else
    {
        ULOGE("ffmpeg: output queue has not been created");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::stop()
{
    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    mThreadShouldStop = true;

    return 0;
}


void* FfmpegAvcDecoder::runDecoderThread(void *ptr)
{
    FfmpegAvcDecoder *decoder = (FfmpegAvcDecoder*)ptr;

    while (!decoder->mThreadShouldStop)
    {
        if (decoder->mConfigured)
        {
            BufferQueueBuffer<avc_decoder_input_buffer_t> *inputBuffer;
            FfmpegOutputBufferQueueBuffer *outputBuffer;
            avc_decoder_input_buffer_t inputData;
            avc_decoder_output_buffer_t outputData;
            buffer_queue_status_t status;
            status = decoder->mInputQueue->getOutputBuffer(&inputBuffer, true);
            if (status != BUFFERQUEUE_OK)
            {
                ULOGW("ffmpeg: failed to get an input buffer");
            }
            else
            {
                if (decoder->mOutputQueue->getInputBuffer(&outputBuffer, false) == BUFFERQUEUE_OK)
                {
                    inputBuffer->getData(&inputData);
                    outputBuffer->getData(&outputData);
                    outputData.userPtr = outputBuffer->getUserPtr();

                    outputData.isComplete = inputData.isComplete;
                    outputData.hasErrors = inputData.hasErrors;
                    outputData.isRef = inputData.isRef;
                    outputData.auNtpTimestamp = inputData.auNtpTimestamp;
                    outputData.auNtpTimestampRaw = inputData.auNtpTimestampRaw;
                    outputData.auNtpTimestampLocal = inputData.auNtpTimestampLocal;
                    outputData.demuxOutputTimestamp = inputData.demuxOutputTimestamp;

                    if (inputData.hasMetadata)
                    {
                        memcpy(&outputData.metadata, &inputData.metadata, sizeof(frame_metadata_t));
                        outputData.hasMetadata = true;
                    }
                    else
                    {
                        outputData.hasMetadata = false;
                    }

                    AVFrame *frame = outputBuffer->getFrame(); 
                    int ret = decoder->decode(&inputData, &outputData, frame);
                    if (ret == 0)
                    {
                        outputBuffer->setData(&outputData);
                        if (decoder->mOutputQueue->releaseInputBuffer(outputBuffer) != BUFFERQUEUE_OK)
                        {
                            ULOGW("ffmpeg: failed to release the output buffer");
                        }
                    }
                    else
                    {
                        if (decoder->mOutputQueue->cancelInputBuffer(outputBuffer) != BUFFERQUEUE_OK)
                        {
                            ULOGW("ffmpeg: failed to cancel the output buffer");
                        }
                    }
                }
                else
                {
                    ULOGW("ffmpeg: failed to get an output buffer");
                }
                decoder->mInputQueue->releaseOutputBuffer(inputBuffer);
            }
        }
        else
        {
            usleep(5000); //TODO: do something better (cond?)
        }
    }

    return NULL;
}


int FfmpegAvcDecoder::decode(avc_decoder_input_buffer_t *inputBuffer, avc_decoder_output_buffer_t *outputBuffer, AVFrame *frame)
{
    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    int frameFinished = false;

    mPacket.data = inputBuffer->auBuffer;
    mPacket.size = inputBuffer->auSize;

    avcodec_decode_video2(mCodecCtxH264, frame, &frameFinished, &mPacket);

    if (frameFinished)
    {
        if ((mFrameWidth != (uint32_t)mCodecCtxH264->width)
                || (mFrameHeight != (uint32_t)mCodecCtxH264->height))
        {
            mFrameWidth = mCodecCtxH264->width;
            mFrameHeight = mCodecCtxH264->height;
            mSarWidth = (mCodecCtxH264->sample_aspect_ratio.num > 0) ? mCodecCtxH264->sample_aspect_ratio.num : 1;
            mSarHeight = (mCodecCtxH264->sample_aspect_ratio.den > 0) ? mCodecCtxH264->sample_aspect_ratio.den : 1;
        }
        outputBuffer->plane[0] = frame->data[0];
        outputBuffer->plane[1] = frame->data[1];
        outputBuffer->plane[2] = frame->data[2];
        outputBuffer->stride[0] = frame->linesize[0];
        outputBuffer->stride[1] = frame->linesize[1];
        outputBuffer->stride[2] = frame->linesize[2];
        outputBuffer->width = mFrameWidth;
        outputBuffer->height = mFrameHeight;
        outputBuffer->sarWidth = mSarWidth;
        outputBuffer->sarHeight = mSarHeight;
        outputBuffer->colorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;

        return 0;
    }
    else
    {
        ULOGI("ffmpeg: frame not complete");
        return -1;
    }
}

}

#endif /* USE_FFMPEG */
