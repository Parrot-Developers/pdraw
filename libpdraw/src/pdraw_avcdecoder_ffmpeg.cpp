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


FfmpegAvcDecoder::FfmpegAvcDecoder(VideoMedia *media)
{
    mConfigured = false;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
    mMedia = (Media*)media;
    mInputBufferPool = NULL;
    mInputBufferQueue = NULL;
    mOutputBufferPool = NULL;
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

    if (mInputBufferQueue) delete mInputBufferQueue;
    if (mInputBufferPool) delete mInputBufferPool;
    if (mOutputBufferPool) delete mOutputBufferPool;

    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();
    while (q != mOutputBufferQueues.end())
    {
        delete *q;
        q++;
    }

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

    /* Input buffers pool allocation */
    if (ret == 0)
    {
        mInputBufferPool = new BufferPool(FFMPEG_AVC_DECODER_INPUT_BUFFER_COUNT,
                                          FFMPEG_AVC_DECODER_INPUT_BUFFER_SIZE,
                                          sizeof(avc_decoder_input_buffer_t),
                                          NULL, NULL); //TODO: number of buffers and buffers size
        if (mInputBufferPool == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder input buffers pool");
            ret = -1;
        }
    }

    /* Input buffers queue allocation */
    if (ret == 0)
    {
        mInputBufferQueue = new BufferQueue();
        if (mInputBufferQueue == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder input buffers queue");
            ret = -1;
        }
    }

    /* Output buffers pool allocation */
    if (ret == 0)
    {
        mOutputBufferPool = new BufferPool(FFMPEG_AVC_DECODER_OUTPUT_BUFFER_COUNT, 0,
                                           sizeof(avc_decoder_output_buffer_t),
                                           outputBufferCreationCb, outputBufferDeletionCb); //TODO: number of buffers
        if (mOutputBufferPool == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder output buffers pool");
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


int FfmpegAvcDecoder::getInputBuffer(Buffer **buffer, bool blocking)
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

    if (mInputBufferPool)
    {
        Buffer *buf = mInputBufferPool->getBuffer(blocking);
        if (buf != NULL)
        {
            buf->setSize(0);
            *buffer = buf;
        }
        else
        {
            ULOGD("ffmpeg: failed to get an input buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("ffmpeg: input buffer pool has not been created");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::queueInputBuffer(Buffer *buffer)
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

    if (mInputBufferQueue)
    {
        buffer->ref();
        mInputBufferQueue->pushBuffer(buffer);
    }
    else
    {
        ULOGE("ffmpeg: input queue has not been created");
        return -1;
    }

    return 0;
}


BufferQueue *FfmpegAvcDecoder::addOutputQueue()
{
    BufferQueue *q = new BufferQueue();
    if (q == NULL)
    {
        ULOGE("ffmpeg: queue allocation failed");
        return NULL;
    }

    mOutputBufferQueues.push_back(q);
    return q;
}


int FfmpegAvcDecoder::removeOutputQueue(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("ffmpeg: invalid queue pointer");
        return -1;
    }

    bool found = false;
    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();

    while (q != mOutputBufferQueues.end())
    {
        if (*q == queue)
        {
            mOutputBufferQueues.erase(q);
            delete *q;
            found = true;
            break;
        }
        q++;
    }

    return (found) ? 0 : -1;
}


bool FfmpegAvcDecoder::isOutputQueueValid(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("ffmpeg: invalid queue pointer");
        return false;
    }

    bool found = false;
    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();

    while (q != mOutputBufferQueues.end())
    {
        if (*q == queue)
        {
            found = true;
            break;
        }
        q++;
    }

    return found;
}


int FfmpegAvcDecoder::dequeueOutputBuffer(BufferQueue *queue, Buffer **buffer, bool blocking)
{
    if (!queue)
    {
        ULOGE("ffmpeg: invalid queue pointer");
        return -1;
    }
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

    if (isOutputQueueValid(queue))
    {
        Buffer *buf = queue->popBuffer(blocking);
        if (buf != NULL)
        {
            avc_decoder_output_buffer_t *data = (avc_decoder_output_buffer_t*)buf->getMetadataPtr();
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            data->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            *buffer = buf;
        }
        else
        {
            ULOGD("ffmpeg: failed to dequeue an output buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("ffmpeg: invalid output queue");
        return -1;
    }

    return 0;
}


int FfmpegAvcDecoder::releaseOutputBuffer(Buffer *buffer)
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

    buffer->unref();

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
    mConfigured = false;

    if (mInputBufferPool) mInputBufferPool->signal();
    if (mOutputBufferPool) mOutputBufferPool->signal();
    if (mInputBufferQueue) mInputBufferQueue->signal();

    return 0;
}


int FfmpegAvcDecoder::outputBufferCreationCb(Buffer *buffer)
{
    if (buffer == NULL)
    {
        ULOGE("ffmpeg: invalid buffer");
        return -1;
    }

    AVFrame *avFrame = av_frame_alloc();
    if (avFrame == NULL)
    {
        ULOGE("ffmpeg: avFrame allocation failed");
        return -1;
    }

    buffer->setResPtr((void*)avFrame);
    return 0;
}


int FfmpegAvcDecoder::outputBufferDeletionCb(Buffer *buffer)
{
    if (buffer == NULL)
    {
        ULOGE("ffmpeg: invalid buffer");
        return -1;
    }

    AVFrame *avFrame = (AVFrame*)buffer->getResPtr();
    if (avFrame == NULL)
    {
        ULOGE("ffmpeg: invalid ressource pointer");
        return -1;
    }

    av_frame_free(&avFrame);
    buffer->setResPtr(NULL);
    return 0;
}


void* FfmpegAvcDecoder::runDecoderThread(void *ptr)
{
    FfmpegAvcDecoder *decoder = (FfmpegAvcDecoder*)ptr;

    while (!decoder->mThreadShouldStop)
    {
        if (decoder->mConfigured)
        {
            Buffer *inputBuffer;
            Buffer *outputBuffer;
            inputBuffer = decoder->mInputBufferQueue->popBuffer(true);
            if (inputBuffer == NULL)
            {
                ULOGW("ffmpeg: failed to dequeue an input buffer");
            }
            else
            {
                outputBuffer = decoder->mOutputBufferPool->getBuffer(false);
                if (outputBuffer != NULL)
                {
                    int ret = decoder->decode(inputBuffer, outputBuffer);
                    if (ret == 0)
                    {
                        std::vector<BufferQueue*>::iterator q = decoder->mOutputBufferQueues.begin();
                        while (q != decoder->mOutputBufferQueues.end())
                        {
                            outputBuffer->ref();
                            (*q)->pushBuffer(outputBuffer);
                            q++;
                        }
                    }
                    outputBuffer->unref();
                }
                else
                {
                    ULOGW("ffmpeg: failed to get an output buffer");
                }
                inputBuffer->unref();
            }
        }
        else
        {
            usleep(5000); //TODO: do something better (cond?)
        }
    }

    return NULL;
}


int FfmpegAvcDecoder::decode(Buffer *inputBuffer, Buffer *outputBuffer)
{
    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    int frameFinished = false;
    avc_decoder_input_buffer_t *inputData = (avc_decoder_input_buffer_t*)inputBuffer->getMetadataPtr();
    avc_decoder_output_buffer_t *outputData = (avc_decoder_output_buffer_t*)outputBuffer->getMetadataPtr();
    AVFrame *frame = (AVFrame*)outputBuffer->getResPtr();
    if ((!inputData) || (!outputData) || (!frame))
    {
        ULOGE("ffmpeg: invalid input or output buffer");
        return -1;
    }

    mPacket.data = (uint8_t*)inputBuffer->getPtr();
    mPacket.size = inputBuffer->getSize();

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
        outputData->plane[0] = frame->data[0];
        outputData->plane[1] = frame->data[1];
        outputData->plane[2] = frame->data[2];
        outputData->stride[0] = frame->linesize[0];
        outputData->stride[1] = frame->linesize[1];
        outputData->stride[2] = frame->linesize[2];
        outputData->width = mFrameWidth;
        outputData->height = mFrameHeight;
        outputData->sarWidth = mSarWidth;
        outputData->sarHeight = mSarHeight;
        outputData->colorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;

        outputData->isComplete = inputData->isComplete;
        outputData->hasErrors = inputData->hasErrors;
        outputData->isRef = inputData->isRef;
        outputData->auNtpTimestamp = inputData->auNtpTimestamp;
        outputData->auNtpTimestampRaw = inputData->auNtpTimestampRaw;
        outputData->auNtpTimestampLocal = inputData->auNtpTimestampLocal;
        outputData->demuxOutputTimestamp = inputData->demuxOutputTimestamp;

        if (inputData->hasMetadata)
        {
            memcpy(&outputData->metadata, &inputData->metadata, sizeof(video_frame_metadata_t));
            outputData->hasMetadata = true;
        }
        else
        {
            outputData->hasMetadata = false;
        }

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
