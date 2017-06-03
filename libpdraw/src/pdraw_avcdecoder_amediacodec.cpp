/**
 * @file pdraw_avcdecoder_amediacodec.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - Android NDK MediaCodec H.264/AVC video decoder
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

#include "pdraw_avcdecoder_amediacodec.hpp"

#ifdef USE_AMEDIACODEC

#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#define PDRAW_AMEDIACODEC_MIME_TYPE "video/avc"


namespace Pdraw
{


AMediaCodecAvcDecoder::AMediaCodecAvcDecoder(VideoMedia *media)
{
    mConfigured = false;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
    mMedia = (Media*)media;
    mInputBufferPool = NULL;
    mInputBufferQueue = NULL;
    mOutputBufferPool = NULL;

    mCodec = AMediaCodec_createDecoderByType(PDRAW_AMEDIACODEC_MIME_TYPE);
}


AMediaCodecAvcDecoder::~AMediaCodecAvcDecoder()
{
    media_status_t err = AMediaCodec_delete(mCodec);
    if (err != AMEDIA_OK)
    {
        ULOGE("AMediaCodec: AMediaCodec_delete() failed (%d)", err);
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
}


int AMediaCodecAvcDecoder::configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;
    media_status_t err;
    AMediaFormat *format = NULL;
    uint8_t *sps = NULL, *pps = NULL;

    if (mConfigured)
    {
        ULOGE("AMediaCodec: decoder is already configured");
        return -1;
    }
    if ((!pSps) || (spsSize <= 4) || (!pPps) || (ppsSize <= 4))
    {
        ULOGE("AMediaCodec: invalid SPS/PPS");
        return -1;
    }

    if (ret == 0)
    {
        format = AMediaFormat_new();
        if (!format)
        {
            ULOGE("AMediaCodec: AMediaFormat_new() failed");
            ret = -1;
        }
        else
        {
            sps = (uint8_t*)malloc(spsSize + 4);
            if (!sps)
            {
                ULOGE("AMediaCodec: allocation failed (size %d)", spsSize + 4);
                ret = -1;
            }
            pps = (uint8_t*)malloc(ppsSize + 4);
            if (!pps)
            {
                ULOGE("AMediaCodec: allocation failed (size %d)", ppsSize + 4);
                ret = -1;
            }
            if (ret == 0)
            {
                //TODO: demuxer should always output SPS/PPS with start codes
                sps[0] = sps[1] = sps[2] = 0; sps[3] = 1;
                pps[0] = pps[1] = pps[2] = 0; pps[3] = 1;
                memcpy(sps + 4, pSps, spsSize);
                memcpy(pps + 4, pPps, ppsSize);
                AMediaFormat_setString(format, AMEDIAFORMAT_KEY_MIME, PDRAW_AMEDIACODEC_MIME_TYPE);
                AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_WIDTH, 480); //TODO
                AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_HEIGHT, 360); //TODO
                AMediaFormat_setBuffer(format, "csd-0", sps, spsSize + 4);
                AMediaFormat_setBuffer(format, "csd-1", pps, ppsSize + 4);
            }
        }
    }

    if (ret == 0)
    {
        //TODO: get the renderer uiHandler
        err = AMediaCodec_configure(mCodec, format, NULL /*(ANativeWindow*)mUiHandler*/, NULL, 0);
        if (err != AMEDIA_OK)
        {
            ULOGE("AMediaCodec: AMediaCodec_configure() failed (%d)", err);
            ret = -1;
        }
        else
        {
            int32_t colorFormat = 0;
            AMediaFormat *outFormat = AMediaCodec_getOutputFormat(mCodec);
            AMediaFormat_getInt32(outFormat, AMEDIAFORMAT_KEY_COLOR_FORMAT, &colorFormat);
            ULOGI("AMediaCodec: AMediaCodec_configure() OK, output color format = %d", colorFormat);
            switch (colorFormat)
            {
                //TODO: where are these constants defined in NDK?
                case 0x00000013:
                    mOutputColorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
                    break;
                case 0x00000015:
                    mOutputColorFormat = AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR;
                    break;
                default:
                    mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
                    break;
            }
        }
    }

    /* Input buffers pool allocation */
    if (ret == 0)
    {
        mInputBufferPool = new BufferPool(AMEDIACODEC_AVC_DECODER_INPUT_BUFFER_COUNT, 0,
                                          sizeof(avc_decoder_input_buffer_t),
                                          NULL, NULL); //TODO: number of buffers
        if (mInputBufferPool == NULL)
        {
            ULOGE("AMediaCodec: failed to allocate decoder input buffers pool");
            ret = -1;
        }
    }

    /* Input buffers queue allocation */
    if (ret == 0)
    {
        mInputBufferQueue = new BufferQueue();
        if (mInputBufferQueue == NULL)
        {
            ULOGE("AMediaCodec: failed to allocate decoder input buffers queue");
            ret = -1;
        }
    }

    /* Output buffers pool allocation */
    if (ret == 0)
    {
        mOutputBufferPool = new BufferPool(AMEDIACODEC_AVC_DECODER_OUTPUT_BUFFER_COUNT, 0,
                                           sizeof(avc_decoder_output_buffer_t),
                                           NULL, NULL); //TODO: number of buffers
        if (mOutputBufferPool == NULL)
        {
            ULOGE("AMediaCodec: failed to allocate decoder output buffers pool");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        err = AMediaCodec_start(mCodec);
        if (err != AMEDIA_OK)
        {
            ULOGE("AMediaCodec: AMediaCodec_start() failed (%d)", err);
            ret = -1;
        }
        else
        {
            ULOGI("AMediaCodec: AMediaCodec_start() OK");
        }
    }

    if (format)
        AMediaFormat_delete(format);
    free(sps);
    free(pps);

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("AMediaCodec: decoder is configured");
    }

    return ret;
}


int AMediaCodecAvcDecoder::getInputBuffer(Buffer **buffer, bool blocking)
{
    if (!buffer)
    {
        ULOGE("AMediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("AMediaCodec: decoder is not configured");
        return -1;
    }

    if (mInputBufferPool)
    {
        Buffer *buf = mInputBufferPool->getBuffer(blocking);
        if (buf != NULL)
        {
            ssize_t bufIdx = AMediaCodec_dequeueInputBuffer(mCodec, (blocking) ? -1 : 0);
            if (bufIdx < 0)
            {
                ULOGE("AMediaCodec: failed to dequeue an input buffer");
                buf->unref();
                return -2;
            }

            size_t bufSize = 0;
            uint8_t *pBuf = AMediaCodec_getInputBuffer(mCodec, bufIdx, &bufSize);
            if ((pBuf == NULL) || (bufSize <= 0))
            {
                ULOGE("AMediaCodec: failed to get input buffer #%zu", bufIdx);
                buf->unref();
                return -1;
            }

            buf->setSize(0);
            buf->setCapacity(bufSize);
            buf->setPtr(pBuf);
            buf->setResPtr((void*)bufIdx);
            *buffer = buf;
        }
        else
        {
            ULOGD("videoCoreOmx: failed to get an input buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("videoCoreOmx: input buffer pool has not been created");
        return -1;
    }

    return 0;
}


int AMediaCodecAvcDecoder::queueInputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("AMediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("AMediaCodec: decoder is not configured");
        return -1;
    }

    if (mInputBufferQueue)
    {
        uint64_t ts = 0;
        avc_decoder_input_buffer_t *data = (avc_decoder_input_buffer_t*)buffer->getMetadataPtr();
        if (data)
            ts = data->auNtpTimestampRaw;

        buffer->ref();
        mInputBufferQueue->pushBuffer(buffer);

        media_status_t err = AMediaCodec_queueInputBuffer(mCodec, (size_t)buffer->getResPtr(), 0, buffer->getSize(), ts, 0);
        if (err != AMEDIA_OK)
        {
            ULOGE("AMediaCodec: failed to queue input buffer #%zu", (size_t)buffer->getResPtr());
            return -1;
        }
    }
    else
    {
        ULOGE("AMediaCodec: input queue has not been created");
        return -1;
    }

    int pollRet = pollDecoderOutput();
    if (pollRet != 0)
    {
        ULOGE("AMediaCodec: pollDecoderOutput() failed (%d)", pollRet);
    }

    return 0;
}


BufferQueue *AMediaCodecAvcDecoder::addOutputQueue()
{
    BufferQueue *q = new BufferQueue();
    if (q == NULL)
    {
        ULOGE("AMediaCodec: queue allocation failed");
        return NULL;
    }

    mOutputBufferQueues.push_back(q);
    return q;
}


int AMediaCodecAvcDecoder::removeOutputQueue(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("AMediaCodec: invalid queue pointer");
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


bool AMediaCodecAvcDecoder::isOutputQueueValid(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("AMediaCodec: invalid queue pointer");
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


int AMediaCodecAvcDecoder::dequeueOutputBuffer(BufferQueue *queue, Buffer **buffer, bool blocking)
{
    if (!queue)
    {
        ULOGE("AMediaCodec: invalid buffer pointer");
        return -1;
    }
    if (!buffer)
    {
        ULOGE("AMediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("AMediaCodec: decoder is not configured");
        return -1;
    }

    int pollRet = pollDecoderOutput();
    if (pollRet != 0)
    {
        ULOGE("AMediaCodec: pollDecoderOutput() failed (%d)", pollRet);
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
            ULOGD("AMediaCodec: failed to dequeue an output buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("AMediaCodec: invalid output queue");
        return -1;
    }

    return 0;
}


int AMediaCodecAvcDecoder::releaseOutputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("AMediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("AMediaCodec: decoder is not configured");
        return -1;
    }

    buffer->unref();

    if (!buffer->isRef())
    {
        media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, (size_t)buffer->getResPtr(), false);
        if (err != AMEDIA_OK)
        {
            ULOGE("AMediaCodec: failed to release output buffer #%zu", (size_t)buffer->getResPtr());
            return -1;
        }
    }

    return 0;
}


int AMediaCodecAvcDecoder::stop()
{
    if (!mConfigured)
    {
        ULOGE("AMediaCodec: decoder is not configured");
        return -1;
    }

    media_status_t err = AMediaCodec_stop(mCodec);
    if (err != AMEDIA_OK)
    {
        ULOGE("AMediaCodec: AMediaCodec_stop() failed (%d)", err);
    }

    //TODO
    if (mInputBufferPool) mInputBufferPool->signal();
    if (mOutputBufferPool) mOutputBufferPool->signal();
    if (mInputBufferQueue) mInputBufferQueue->signal();

    return 0;
}


int AMediaCodecAvcDecoder::pollDecoderOutput()
{
    int ret = 0;
    AMediaCodecBufferInfo info;
    ssize_t bufIdx = AMediaCodec_dequeueOutputBuffer(mCodec, &info, 0);
    while (bufIdx >= 0)
    {
        bool pushed = false;

        //TODO
        int32_t width = 0, height = 0, colorFormat = 0;
        int32_t cropLeft = 0, cropRight = 0, cropTop = 0, cropBottom = 0;
        AMediaFormat *format = AMediaCodec_getOutputFormat(mCodec);
        AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_WIDTH, &width);
        AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_HEIGHT, &height);
        AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_COLOR_FORMAT, &colorFormat);
        AMediaFormat_getInt32(format, "crop-left", &cropLeft);
        AMediaFormat_getInt32(format, "crop-right", &cropRight);
        AMediaFormat_getInt32(format, "crop-top", &cropTop);
        AMediaFormat_getInt32(format, "crop-bottom", &cropBottom);
#if 0
        int32_t cropWidth = cropRight - cropLeft + 1;
        int32_t cropHeight = cropBottom - cropTop + 1;
        ULOGI("AMediaCodec: cropLeft=%d cropRight=%d cropTop=%d cropBottom=%d", cropLeft, cropRight, cropTop, cropBottom);
        ULOGI("AMediaCodec: cropWidth=%d cropHeight=%d", cropWidth, cropHeight);
        const char *s = AMediaFormat_toString(format);
        ULOGI("AMediaCodec: format: %s", s);
#endif
        if ((!width) || (!height))
        {
            ULOGE("AMediaCodec: invalid width (%d) and height (%d)", width, height);
            media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, bufIdx, false);
            if (err != AMEDIA_OK)
            {
                ULOGE("AMediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            continue;
        }

        size_t bufSize = 0;
        uint8_t *pBuf = AMediaCodec_getOutputBuffer(mCodec, bufIdx, &bufSize);
        if ((pBuf == NULL) || (bufSize <= 0))
        {
            ULOGE("AMediaCodec: failed to get output buffer #%zu", bufIdx);
            media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, bufIdx, false);
            if (err != AMEDIA_OK)
            {
                ULOGE("AMediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            continue;
        }

        Buffer *inputBuffer = NULL, *outputBuffer = NULL;
        avc_decoder_input_buffer_t *inputData = NULL;
        avc_decoder_output_buffer_t *outputData = NULL;
        uint64_t ts = (uint64_t)info.presentationTimeUs;

        if (ts > 0)
        {
            Buffer *b;
            while ((b = mInputBufferQueue->peekBuffer(false)) != NULL)
            {
                avc_decoder_input_buffer_t *d = (avc_decoder_input_buffer_t*)b->getMetadataPtr();

                if (ts > d->auNtpTimestampRaw)
                {
                    b = mInputBufferQueue->popBuffer(false);
                    b->unref();
                }
                else if (ts == d->auNtpTimestampRaw)
                {
                    inputBuffer = mInputBufferQueue->popBuffer(false);
                    inputData = d;
                    break;
                }
                else
                {
                    break;
                }
            }

            if ((inputBuffer == NULL) || (inputData == NULL))
            {
                ULOGW("AMediaCodec: failed to find buffer for TS %" PRIu64, ts);
            }
        }
        else
        {
            ULOGW("AMediaCodec: invalid timestamp in buffer callback");
        }

        if ((inputBuffer == NULL) || (inputData == NULL))
        {
            media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, bufIdx, false);
            if (err != AMEDIA_OK)
            {
                ULOGE("AMediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            continue;
        }

        outputBuffer = mOutputBufferPool->getBuffer(false);
        if (!outputBuffer)
        {
            ULOGE("videoCoreOmx: failed to get an output buffer");
            if (inputBuffer)
            {
                inputBuffer->unref();
            }
            media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, bufIdx, false);
            if (err != AMEDIA_OK)
            {
                ULOGE("AMediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            continue;
        }

        outputBuffer->setResPtr((void*)bufIdx);
        outputData = (avc_decoder_output_buffer_t*)outputBuffer->getMetadataPtr();
        if (outputData)
        {
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            outputData->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            outputData->width = width;
            outputData->height = height;
            outputData->sarWidth = 1; //TODO
            outputData->sarHeight = 1; //TODO
            switch (colorFormat)
            {
                //TODO: where are these constants defined in NDK?
                case 0x00000013:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
                    outputData->plane[0] = pBuf;
                    outputData->plane[1] = pBuf + width * height;
                    outputData->plane[2] = pBuf + width * height * 5 / 4;
                    outputData->stride[0] = width;
                    outputData->stride[1] = width / 2;
                    outputData->stride[2] = width / 2;
                    break;
                case 0x00000015:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR;
                    outputData->plane[0] = pBuf;
                    outputData->plane[1] = pBuf + width * height;
                    outputData->plane[2] = pBuf + width * height;
                    outputData->stride[0] = width;
                    outputData->stride[1] = width;
                    outputData->stride[2] = width;
                    break;
                default:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
                    outputData->plane[0] = pBuf;
                    outputData->plane[1] = pBuf;
                    outputData->plane[2] = pBuf;
                    outputData->stride[0] = width;
                    outputData->stride[1] = width;
                    outputData->stride[2] = width;
                    break;
            }
            if (inputData)
            {
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
            }

            std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();
            while (q != mOutputBufferQueues.end())
            {
                outputBuffer->ref();
                (*q)->pushBuffer(outputBuffer);
                q++;
                pushed = true;
            }
        }
        else
        {
            ULOGW("videoCoreOmx: invalid output buffer data");
        }

        if (!pushed)
        {
            media_status_t err = AMediaCodec_releaseOutputBuffer(mCodec, bufIdx, false);
            if (err != AMEDIA_OK)
            {
                ULOGE("AMediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
        }

        outputBuffer->unref();

        if (inputBuffer)
        {
            inputBuffer->unref();
        }

        bufIdx = AMediaCodec_dequeueOutputBuffer(mCodec, &info, 0);
    }

    return ret;
}

}

#endif /* USE_AMEDIACODEC */
