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
#include <video-buffers/vbuf_avframe.h>

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
    mOutputColorFormat = AVCDECODER_COLOR_FORMAT_UNKNOWN;
    mMedia = (Media*)media;
    mInputBufferPool = NULL;
    mInputBufferQueue = NULL;
    mOutputBufferPool = NULL;
    mThreadShouldStop = false;
    mDecoderThreadLaunched = false;
    mCodecH264 = NULL;

    avcodec_register_all();
    av_log_set_level(FFMPEG_LOG_LEVEL);
    if (mCodecH264 == NULL)
    {
        mCodecH264 = avcodec_find_decoder_by_name("h264_cuvid");
        if (mCodecH264 != NULL)
        {
            ULOGI("ffmpeg: using NVDec (cuvid) hardware acceleration");
            mOutputColorFormat = AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR;
        }
    }
    if (mCodecH264 == NULL)
    {
        mCodecH264 = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (mCodecH264 != NULL)
        {
            ULOGI("ffmpeg: using CPU H.264 decoding");
            mOutputColorFormat = AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
        }
    }
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

    if (mInputBufferQueue) vbuf_queue_destroy(mInputBufferQueue);
    if (mInputBufferPool) vbuf_pool_destroy(mInputBufferPool);
    if (mOutputBufferPool) vbuf_pool_destroy(mOutputBufferPool);

    std::vector<struct vbuf_queue*>::iterator q = mOutputBufferQueues.begin();
    while (q != mOutputBufferQueues.end())
    {
        vbuf_queue_destroy(*q);
        q++;
    }

    avcodec_free_context(&mCodecCtxH264);
}


int FfmpegAvcDecoder::configure(uint32_t inputBitstreamFormat,
        const uint8_t *pSps, unsigned int spsSize,
        const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;
    struct vbuf_cbs cbs;

    if (mConfigured)
    {
        ULOGE("ffmpeg: decoder is already configured");
        return -1;
    }
    if (inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM)
    {
        ULOGE("ffmpeg: unsupported input bitstream format");
        return -1;
    }

    /* Nothing to decode here for ffmpeg: SPS/PPS will be decoded with the first picture */

    if (ret == 0)
    {
        ret = vbuf_generic_get_cbs(&cbs);
        if (ret != 0)
        {
            ULOGE("ffmpeg: failed to get allocation callbacks");
        }
    }

    /* Input buffers pool allocation */
    if (ret == 0)
    {
        mInputBufferPool = vbuf_pool_new(FFMPEG_AVC_DECODER_INPUT_BUFFER_COUNT,
                                         FFMPEG_AVC_DECODER_INPUT_BUFFER_SIZE,
                                         sizeof(avc_decoder_input_buffer_t), 0,
                                         &cbs); //TODO: number of buffers and buffers size
        if (mInputBufferPool == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder input buffers pool");
            ret = -1;
        }
    }

    /* Input buffers queue allocation */
    if (ret == 0)
    {
        mInputBufferQueue = vbuf_queue_new();
        if (mInputBufferQueue == NULL)
        {
            ULOGE("ffmpeg: failed to allocate decoder input buffers queue");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = vbuf_avframe_get_cbs(&cbs);
        if (ret != 0)
        {
            ULOGE("ffmpeg: failed to get allocation callbacks");
        }
    }

    /* Output buffers pool allocation */
    if (ret == 0)
    {
        mOutputBufferPool = vbuf_pool_new(FFMPEG_AVC_DECODER_OUTPUT_BUFFER_COUNT, 0,
                                          sizeof(avc_decoder_output_buffer_t), 0,
                                          &cbs); //TODO: number of buffers
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


int FfmpegAvcDecoder::getInputBuffer(struct vbuf_buffer **buffer, bool blocking)
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
        struct vbuf_buffer *buf = NULL;
        int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
        if ((ret == 0) && (buf != NULL))
        {
            vbuf_set_size(buf, 0);
            *buffer = buf;
        }
        else
        {
            ULOGD("ffmpeg: failed to get an input buffer (%d)", ret);
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


int FfmpegAvcDecoder::queueInputBuffer(struct vbuf_buffer *buffer)
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
        int ret = vbuf_queue_push(mInputBufferQueue, buffer);
        if (ret != 0)
            ULOGE("ffmpeg: failed to push the buffer into the input queue");
    }
    else
    {
        ULOGE("ffmpeg: input queue has not been created");
        return -1;
    }

    return 0;
}


struct vbuf_queue *FfmpegAvcDecoder::addOutputQueue()
{
    struct vbuf_queue *q = vbuf_queue_new();
    if (q == NULL)
    {
        ULOGE("ffmpeg: queue allocation failed");
        return NULL;
    }

    mOutputBufferQueues.push_back(q);
    return q;
}


int FfmpegAvcDecoder::removeOutputQueue(struct vbuf_queue *queue)
{
    if (!queue)
    {
        ULOGE("ffmpeg: invalid queue pointer");
        return -1;
    }

    bool found = false;
    std::vector<struct vbuf_queue*>::iterator q = mOutputBufferQueues.begin();

    while (q != mOutputBufferQueues.end())
    {
        if (*q == queue)
        {
            mOutputBufferQueues.erase(q);
            int ret = vbuf_queue_destroy(*q);
            if (ret != 0)
                ULOGE("ffmpeg: vbuf_queue_destroy() failed (%d)", ret);
            found = true;
            break;
        }
        q++;
    }

    return (found) ? 0 : -1;
}


bool FfmpegAvcDecoder::isOutputQueueValid(struct vbuf_queue *queue)
{
    if (!queue)
    {
        ULOGE("ffmpeg: invalid queue pointer");
        return false;
    }

    bool found = false;
    std::vector<struct vbuf_queue*>::iterator q = mOutputBufferQueues.begin();

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


int FfmpegAvcDecoder::dequeueOutputBuffer(struct vbuf_queue *queue, struct vbuf_buffer **buffer, bool blocking)
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
        struct vbuf_buffer *buf = NULL;
        int ret = vbuf_queue_pop(queue, (blocking) ? -1 : 0, &buf);
        if ((ret == 0) && (buf != NULL))
        {
            avc_decoder_output_buffer_t *data = (avc_decoder_output_buffer_t*)vbuf_get_metadata_ptr(buf);
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            data->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            *buffer = buf;
        }
        else
        {
            ULOGD("ffmpeg: failed to dequeue an output buffer (%d)", ret);
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


int FfmpegAvcDecoder::releaseOutputBuffer(struct vbuf_buffer **buffer)
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

    vbuf_unref(buffer);

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

    if (mInputBufferPool) vbuf_pool_abort(mInputBufferPool);
    if (mOutputBufferPool) vbuf_pool_abort(mOutputBufferPool);
    if (mInputBufferQueue) vbuf_queue_abort(mInputBufferQueue);

    return 0;
}


void* FfmpegAvcDecoder::runDecoderThread(void *ptr)
{
    FfmpegAvcDecoder *decoder = (FfmpegAvcDecoder*)ptr;

    while (!decoder->mThreadShouldStop)
    {
        if (decoder->mConfigured)
        {
            struct vbuf_buffer *inputBuffer = NULL;
            struct vbuf_buffer *outputBuffer;
            int ret = vbuf_queue_pop(decoder->mInputBufferQueue, -1, &inputBuffer);
            if ((ret != 0) || (inputBuffer == NULL))
            {
                ULOGW("ffmpeg: failed to dequeue an input buffer (%d)", ret);
            }
            else
            {
                ret = vbuf_pool_get(decoder->mOutputBufferPool, 0, &outputBuffer);
                if ((ret == 0) && (outputBuffer != NULL))
                {
                    ret = decoder->decode(inputBuffer, outputBuffer);
                    if (ret == 1)
                    {
                        std::vector<struct vbuf_queue*>::iterator q = decoder->mOutputBufferQueues.begin();
                        while (q != decoder->mOutputBufferQueues.end())
                        {
                            vbuf_queue_push(*q, outputBuffer);
                            q++;
                        }
                    }
                    else if (ret == 0)
                    {
                        ULOGI("ffmpeg: silent frame (ignored)");
                    }
                    vbuf_unref(&outputBuffer);
                }
                else
                {
                    ULOGW("ffmpeg: failed to get an output buffer (%d)", ret);
                }
                vbuf_unref(&inputBuffer);
            }
        }
        else
        {
            usleep(5000); //TODO: do something better (cond?)
        }
    }

    return NULL;
}


int FfmpegAvcDecoder::decode(struct vbuf_buffer *inputBuffer, struct vbuf_buffer *outputBuffer)
{
    if (!mConfigured)
    {
        ULOGE("ffmpeg: decoder is not configured");
        return -1;
    }

    int frameFinished = false;
    avc_decoder_input_buffer_t *inputData = (avc_decoder_input_buffer_t*)vbuf_get_metadata_ptr(inputBuffer);
    avc_decoder_output_buffer_t *outputData = (avc_decoder_output_buffer_t*)vbuf_get_metadata_ptr(outputBuffer);
    AVFrame *frame = vbuf_avframe_get_frame(outputBuffer);
    if ((!inputData) || (!outputData) || (!frame))
    {
        ULOGE("ffmpeg: invalid input or output buffer %p %p %p", inputData, outputData, frame);
        return -1;
    }

    mPacket.data = vbuf_get_ptr(inputBuffer);
    mPacket.size = vbuf_get_size(inputBuffer);

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
        memset(outputData, 0, sizeof(*outputData));
        outputData->colorFormat = mOutputColorFormat;
        if (mOutputColorFormat == AVCDECODER_COLOR_FORMAT_YUV420SEMIPLANAR)
        {
            outputData->plane[0] = frame->data[0];
            outputData->plane[1] = frame->data[1];
            outputData->stride[0] = frame->linesize[0];
            outputData->stride[1] = frame->linesize[1];
        }
        else if (mOutputColorFormat == AVCDECODER_COLOR_FORMAT_YUV420PLANAR)
        {
            outputData->plane[0] = frame->data[0];
            outputData->plane[1] = frame->data[1];
            outputData->plane[2] = frame->data[2];
            outputData->stride[0] = frame->linesize[0];
            outputData->stride[1] = frame->linesize[1];
            outputData->stride[2] = frame->linesize[2];
        }
        outputData->width = mFrameWidth;
        outputData->height = mFrameHeight;
        outputData->sarWidth = mSarWidth;
        outputData->sarHeight = mSarHeight;

        outputData->isComplete = inputData->isComplete;
        outputData->hasErrors = inputData->hasErrors;
        outputData->isRef = inputData->isRef;
        outputData->isSilent = inputData->isSilent;
        outputData->auNtpTimestamp = inputData->auNtpTimestamp;
        outputData->auNtpTimestampRaw = inputData->auNtpTimestampRaw;
        outputData->auNtpTimestampLocal = inputData->auNtpTimestampLocal;
        outputData->demuxOutputTimestamp = inputData->demuxOutputTimestamp;

        if (inputData->hasMetadata)
        {
            memcpy(&outputData->metadata, &inputData->metadata, sizeof(struct vmeta_frame_v2));
            outputData->hasMetadata = true;
        }
        else
        {
            outputData->hasMetadata = false;
        }

        /* User data */
        unsigned int userDataSize = vbuf_get_userdata_size(inputBuffer);
        uint8_t *userData = vbuf_get_userdata_ptr(inputBuffer);
        if ((userData) && (userDataSize > 0))
        {
            int ret = vbuf_set_userdata_capacity(outputBuffer, userDataSize);
            if (ret < (signed)userDataSize)
            {
                ULOGE("ffmpeg: failed to realloc user data buffer");
            }
            else
            {
                uint8_t *dstBuf = vbuf_get_userdata_ptr(outputBuffer);
                memcpy(dstBuf, userData, userDataSize);
                vbuf_set_userdata_size(outputBuffer, userDataSize);
            }
        }
        else
        {
            vbuf_set_userdata_size(outputBuffer, 0);
        }

        return (outputData->isSilent) ? 0 : 1;
    }
    else
    {
        ULOGI("ffmpeg: failed to decode frame");
        return -1;
    }
}

}

#endif /* USE_FFMPEG */
