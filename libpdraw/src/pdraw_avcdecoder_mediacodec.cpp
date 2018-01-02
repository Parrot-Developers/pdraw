/**
 * @file pdraw_avcdecoder_mediacodec.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - Android MediaCodec H.264/AVC video decoder
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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "pdraw_avcdecoder_mediacodec.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_session.hpp"

#ifdef USE_MEDIACODEC

#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#define PDRAW_MEDIACODEC_MIME_TYPE "video/avc"


namespace Pdraw
{


MediaCodecAvcDecoder::MediaCodecAvcDecoder(VideoMedia *media)
{
    mConfigured = false;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
    mMedia = (Media*)media;
    mInputBufferPool = NULL;
    mInputBufferQueue = NULL;
    mOutputBufferPool = NULL;
    mThreadShouldStop = false;
    mOutputPollThreadLaunched = false;
    mWidth = 0;
    mHeight = 0;
    mCropLeft = 0;
    mCropRight = 0;
    mCropTop = 0;
    mCropBottom = 0;
    mCroppedWidth = 0;
    mCroppedHeight = 0;
    mSarWidth = 0;
    mSarHeight = 0;

    void *jniEnv = NULL;
    if (mMedia)
    {
        Session *session = mMedia->getSession();
        if (session)
            jniEnv = session->getJniEnv();
    }

    mMcw = mcw_new((mcw_jnienv *)jniEnv, MCW_IMPLEMENTATION_AUTO);
    if (!mMcw)
    {
        ULOGE("MediaCodec: mcw_new() failed");
    }
    else
    {
        mCodec = mMcw->mediacodec.create_decoder_by_type(PDRAW_MEDIACODEC_MIME_TYPE);
    }

    int thErr = pthread_create(&mOutputPollThread, NULL, runOutputPollThread, (void*)this);
    if (thErr != 0)
    {
        ULOGE("MediaCodec: output poll thread creation failed (%d)", thErr);
    }
    else
    {
        mOutputPollThreadLaunched = true;
    }
}


MediaCodecAvcDecoder::~MediaCodecAvcDecoder()
{
    mThreadShouldStop = true;
    if (mOutputPollThreadLaunched)
    {
        int thErr = pthread_join(mOutputPollThread, NULL);
        if (thErr != 0)
        {
            ULOGE("MediaCodec: pthread_join() failed (%d)", thErr);
        }
    }

    if (mMcw)
    {
        enum mcw_media_status err = mMcw->mediacodec.ddelete(mCodec);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: mediacodec.delete() failed (%d)", err);
        }
        mcw_destroy(mMcw);
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


int MediaCodecAvcDecoder::configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;
    enum mcw_media_status err;
    struct mcw_mediaformat *format = NULL;
    uint8_t *sps = NULL, *pps = NULL;

    if (mConfigured)
    {
        ULOGE("MediaCodec: decoder is already configured");
        return -1;
    }
    if (!mMcw)
    {
        ULOGE("MediaCodec: invalid mediacodec wrapper");
        return -1;
    }
    if ((!pSps) || (spsSize == 0) || (!pPps) || (ppsSize == 0))
    {
        ULOGE("MediaCodec: invalid SPS/PPS");
        return -1;
    }

    if (ret == 0)
    {
        format = mMcw->mediaformat.nnew();
        if (!format)
        {
            ULOGE("MediaCodec: mediaformat.new() failed");
            ret = -1;
        }
        else
        {
            sps = (uint8_t*)malloc(spsSize + 4);
            if (!sps)
            {
                ULOGE("MediaCodec: allocation failed (size %d)", spsSize + 4);
                ret = -1;
            }
            pps = (uint8_t*)malloc(ppsSize + 4);
            if (!pps)
            {
                ULOGE("MediaCodec: allocation failed (size %d)", ppsSize + 4);
                ret = -1;
            }
            if (ret == 0)
            {
                //TODO: demuxer should always output SPS/PPS with start codes
                sps[0] = sps[1] = sps[2] = 0; sps[3] = 1;
                pps[0] = pps[1] = pps[2] = 0; pps[3] = 1;
                memcpy(sps + 4, pSps, spsSize);
                memcpy(pps + 4, pPps, ppsSize);
                mMcw->mediaformat.set_string(format, mMcw->mediaformat.KEY_MIME, PDRAW_MEDIACODEC_MIME_TYPE);
                mMcw->mediaformat.set_int32(format, mMcw->mediaformat.KEY_WIDTH, 480); //TODO
                mMcw->mediaformat.set_int32(format, mMcw->mediaformat.KEY_HEIGHT, 360); //TODO
                mMcw->mediaformat.set_int32(format, mMcw->mediaformat.KEY_MAX_INPUT_SIZE, 1920 * 1088 * 3 / 4); //TODO
                mMcw->mediaformat.set_buffer(format, "csd-0", sps, spsSize + 4);
                mMcw->mediaformat.set_buffer(format, "csd-1", pps, ppsSize + 4);
            }
        }
    }

    if (ret == 0)
    {
        //TODO: get the renderer uiHandler
        err = mMcw->mediacodec.configure(mCodec, format, NULL /*(ANativeWindow*)mUiHandler*/, NULL, 0);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: mediacodec.configure() failed (%d)", err);
            ret = -1;
        }
    }

    /* Input buffers pool allocation */
    if (ret == 0)
    {
        mInputBufferPool = new BufferPool(MEDIACODEC_AVC_DECODER_INPUT_BUFFER_COUNT, 0,
                                          sizeof(avc_decoder_input_buffer_t), 0,
                                          NULL, NULL); //TODO: number of buffers
        if (mInputBufferPool == NULL)
        {
            ULOGE("MediaCodec: failed to allocate decoder input buffers pool");
            ret = -1;
        }
    }

    /* Input buffers queue allocation */
    if (ret == 0)
    {
        mInputBufferQueue = new BufferQueue();
        if (mInputBufferQueue == NULL)
        {
            ULOGE("MediaCodec: failed to allocate decoder input buffers queue");
            ret = -1;
        }
    }

    /* Output buffers pool allocation */
    if (ret == 0)
    {
        mOutputBufferPool = new BufferPool(MEDIACODEC_AVC_DECODER_OUTPUT_BUFFER_COUNT, 0,
                                           sizeof(avc_decoder_output_buffer_t), 0,
                                           NULL, NULL); //TODO: number of buffers
        if (mOutputBufferPool == NULL)
        {
            ULOGE("MediaCodec: failed to allocate decoder output buffers pool");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        err = mMcw->mediacodec.start(mCodec);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: mediacodec.start() failed (%d)", err);
            ret = -1;
        }
        else
        {
            ULOGI("MediaCodec: mediacodec.start() OK");
        }
    }

    if (format)
        mMcw->mediaformat.ddelete(format);
    free(sps);
    free(pps);

    ((VideoMedia*)mMedia)->getDimensions(&mWidth, &mHeight,
        &mCropLeft, &mCropRight, &mCropTop, &mCropBottom,
        &mCroppedWidth, &mCroppedHeight, &mSarWidth, &mSarHeight);

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("MediaCodec: decoder is configured");
    }

    return ret;
}


int MediaCodecAvcDecoder::getInputBuffer(Buffer **buffer, bool blocking)
{
    if (!buffer)
    {
        ULOGE("MediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("MediaCodec: decoder is not configured");
        return -1;
    }
    if (!mMcw)
    {
        ULOGE("MediaCodec: invalid mediacodec wrapper");
        return -1;
    }

    if (mInputBufferPool)
    {
        Buffer *buf = mInputBufferPool->getBuffer(blocking);
        if (buf != NULL)
        {
            ssize_t bufIdx = mMcw->mediacodec.dequeue_input_buffer(mCodec, (blocking) ? -1 : 0);
            if (bufIdx < 0)
            {
                ULOGE("MediaCodec: failed to dequeue an input buffer");
                buf->unref();
                return -2;
            }

            size_t bufSize = 0;
            uint8_t *pBuf = mMcw->mediacodec.get_input_buffer(mCodec, bufIdx, &bufSize);
            if ((pBuf == NULL) || (bufSize <= 0))
            {
                ULOGE("MediaCodec: failed to get input buffer #%zu", bufIdx);
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
            ULOGD("MediaCodec: failed to get an input buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("MediaCodec: input buffer pool has not been created");
        return -1;
    }

    return 0;
}


int MediaCodecAvcDecoder::queueInputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("MediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("MediaCodec: decoder is not configured");
        return -1;
    }
    if (!mMcw)
    {
        ULOGE("MediaCodec: invalid mediacodec wrapper");
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

        enum mcw_media_status err = mMcw->mediacodec.queue_input_buffer(mCodec, (size_t)buffer->getResPtr(), 0, buffer->getSize(), ts, 0);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: failed to queue input buffer #%zu", (size_t)buffer->getResPtr());
            return -1;
        }
    }
    else
    {
        ULOGE("MediaCodec: input queue has not been created");
        return -1;
    }

    return 0;
}


BufferQueue *MediaCodecAvcDecoder::addOutputQueue()
{
    BufferQueue *q = new BufferQueue();
    if (q == NULL)
    {
        ULOGE("MediaCodec: queue allocation failed");
        return NULL;
    }

    mOutputBufferQueues.push_back(q);
    return q;
}


int MediaCodecAvcDecoder::removeOutputQueue(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("MediaCodec: invalid queue pointer");
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


bool MediaCodecAvcDecoder::isOutputQueueValid(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("MediaCodec: invalid queue pointer");
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


int MediaCodecAvcDecoder::dequeueOutputBuffer(BufferQueue *queue, Buffer **buffer, bool blocking)
{
    if (!queue)
    {
        ULOGE("MediaCodec: invalid buffer pointer");
        return -1;
    }
    if (!buffer)
    {
        ULOGE("MediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("MediaCodec: decoder is not configured");
        return -1;
    }

    if (isOutputQueueValid(queue))
    {
        Buffer *buf = queue->popBuffer(blocking);
        if (buf != NULL)
        {
            *buffer = buf;
        }
        else
        {
            ULOGD("MediaCodec: failed to dequeue an output buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("MediaCodec: invalid output queue");
        return -1;
    }

    return 0;
}


int MediaCodecAvcDecoder::releaseOutputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("MediaCodec: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("MediaCodec: decoder is not configured");
        return -1;
    }
    if (!mMcw)
    {
        ULOGE("MediaCodec: invalid mediacodec wrapper");
        return -1;
    }

    buffer->unref();

    if (!buffer->isRef())
    {
        enum mcw_media_status err = mMcw->mediacodec.release_output_buffer(mCodec, (size_t)buffer->getResPtr(), false);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: failed to release output buffer #%zu", (size_t)buffer->getResPtr());
            return -1;
        }
    }

    return 0;
}


int MediaCodecAvcDecoder::stop()
{
    if (!mConfigured)
    {
        ULOGE("MediaCodec: decoder is not configured");
        return -1;
    }

    mThreadShouldStop = true;
    mConfigured = false;

    if (mMcw)
    {
        enum mcw_media_status err = mMcw->mediacodec.stop(mCodec);
        if (err != MCW_MEDIA_STATUS_OK)
        {
            ULOGE("MediaCodec: mediacodec.stop() failed (%d)", err);
        }
    }

    //TODO
    if (mInputBufferPool) mInputBufferPool->signal();
    if (mOutputBufferPool) mOutputBufferPool->signal();
    if (mInputBufferQueue) mInputBufferQueue->signal();

    return 0;
}


int MediaCodecAvcDecoder::pollDecoderOutput()
{
    int ret = 0;
    struct mcw_mediacodec_bufferinfo info;
    ssize_t bufIdx = mMcw->mediacodec.dequeue_output_buffer(mCodec, &info, 5000);
    while (bufIdx >= 0)
    {
        bool pushed = false;
        int32_t colorFormat = 0x00000015;

        /* TODO: do this only on INFO_OUTPUT_FORMAT_CHANGED */
        struct mcw_mediaformat *format = mMcw->mediacodec.get_output_format(mCodec, bufIdx);
        if (format)
        {
            mMcw->mediaformat.get_int32(format, mMcw->mediaformat.KEY_COLOR_FORMAT, &colorFormat);
            mMcw->mediaformat.ddelete(format);
        }

        size_t bufSize = 0;
        uint8_t *pBuf = mMcw->mediacodec.get_output_buffer(mCodec, bufIdx, &bufSize);
        if ((pBuf == NULL) || (bufSize <= 0))
        {
            ULOGE("MediaCodec: failed to get output buffer #%zu", bufIdx);
            enum mcw_media_status err = mMcw->mediacodec.release_output_buffer(mCodec, bufIdx, false);
            if (err != MCW_MEDIA_STATUS_OK)
            {
                ULOGE("MediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            break;
        }

        Buffer *inputBuffer = NULL, *outputBuffer = NULL;
        avc_decoder_input_buffer_t *inputData = NULL;
        avc_decoder_output_buffer_t *outputData = NULL;
        uint64_t ts = (uint64_t)info.presentation_time_us;

        if (info.presentation_time_us >= 0)
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
                ULOGW("MediaCodec: failed to find buffer for TS %" PRIu64, ts);
            }
        }
        else
        {
            ULOGW("MediaCodec: invalid timestamp in buffer callback");
        }
#if 0
        inputBuffer = mInputBufferQueue->popBuffer(false);
        if (inputBuffer)
            inputData = (avc_decoder_input_buffer_t*)inputBuffer->getMetadataPtr();
#endif

        if ((inputBuffer == NULL) || (inputData == NULL))
        {
            enum mcw_media_status err = mMcw->mediacodec.release_output_buffer(mCodec, bufIdx, false);
            if (err != MCW_MEDIA_STATUS_OK)
            {
                ULOGE("MediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            break;
        }

        outputBuffer = mOutputBufferPool->getBuffer(false);
        if (!outputBuffer)
        {
            ULOGE("MediaCodec: failed to get an output buffer");
            if (inputBuffer)
            {
                inputBuffer->unref();
            }
            enum mcw_media_status err = mMcw->mediacodec.release_output_buffer(mCodec, bufIdx, false);
            if (err != MCW_MEDIA_STATUS_OK)
            {
                ULOGE("MediaCodec: failed to release output buffer #%zu", bufIdx);
                return -1;
            }
            ret = -1;
            break;
        }

        outputBuffer->setResPtr((void*)bufIdx);
        outputData = (avc_decoder_output_buffer_t*)outputBuffer->getMetadataPtr();
        if (outputData)
        {
            outputBuffer->setMetadataSize(sizeof(avc_decoder_output_buffer_t));
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            memset(outputData, 0, sizeof(*outputData));
            outputData->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            outputData->width = mCroppedWidth;
            outputData->height = mCroppedHeight;
            outputData->sarWidth = mSarWidth;
            outputData->sarHeight = mSarHeight;
            switch (colorFormat)
            {
                case MCW_COLOR_FORMAT_YUV420_PLANAR:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
                    outputData->plane[0] = pBuf + mCropTop * mWidth + mCropLeft;
                    outputData->plane[1] = pBuf + mWidth * mHeight + mCropTop / 2 * mWidth / 2 + mCropLeft / 2;
                    outputData->plane[2] = pBuf + mWidth * mHeight * 5 / 4 + mCropTop / 2 * mWidth / 2 + mCropLeft / 2;
                    outputData->stride[0] = mWidth;
                    outputData->stride[1] = mWidth / 2;
                    outputData->stride[2] = mWidth / 2;
                    break;
                case MCW_COLOR_FORMAT_YUV420_SEMIPLANAR:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR;
                    outputData->plane[0] = pBuf + mCropTop * mWidth + mCropLeft;
                    outputData->plane[1] = pBuf + mWidth * mHeight + mCropTop / 2 * mWidth + mCropLeft;
                    outputData->plane[2] = pBuf + mWidth * mHeight + mCropTop / 2 * mWidth + mCropLeft;
                    outputData->stride[0] = mWidth;
                    outputData->stride[1] = mWidth;
                    outputData->stride[2] = mWidth;
                    break;
                default:
                    outputData->colorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
                    outputData->plane[0] = pBuf + mCropTop * mWidth + mCropLeft;
                    outputData->plane[1] = pBuf + mCropTop * mWidth + mCropLeft;
                    outputData->plane[2] = pBuf + mCropTop * mWidth + mCropLeft;
                    outputData->stride[0] = mWidth;
                    outputData->stride[1] = mWidth;
                    outputData->stride[2] = mWidth;
                    break;
            }
            if (inputData)
            {
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
            }

            /* User data */
            unsigned int userDataSize = inputBuffer->getUserDataSize();
            void *userData = inputBuffer->getUserDataPtr();
            if ((userData) && (userDataSize > 0))
            {
                int ret = outputBuffer->setUserDataCapacity(userDataSize);
                if (ret < (signed)userDataSize)
                {
                    ULOGE("MediaCodec: failed to realloc user data buffer");
                }
                else
                {
                    void *dstBuf = outputBuffer->getUserDataPtr();
                    memcpy(dstBuf, userData, userDataSize);
                    outputBuffer->setUserDataSize(userDataSize);
                }
            }
            else
            {
                outputBuffer->setUserDataSize(0);
            }

            if (!outputData->isSilent)
            {
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
                ULOGI("MediaCodec: silent frame (ignored)");
            }
        }
        else
        {
            ULOGW("MediaCodec: invalid output buffer data");
        }

        if (!pushed)
        {
            enum mcw_media_status err = mMcw->mediacodec.release_output_buffer(mCodec, bufIdx, false);
            if (err != MCW_MEDIA_STATUS_OK)
            {
                ULOGE("MediaCodec: failed to release output buffer #%zu", bufIdx);
                ret = -1;
                outputBuffer->unref();
                if (inputBuffer)
                {
                    inputBuffer->unref();
                }
                break;
            }
        }

        outputBuffer->unref();

        if (inputBuffer)
        {
            inputBuffer->unref();
        }

        bufIdx = mMcw->mediacodec.dequeue_output_buffer(mCodec, &info, 0);
    }

    return ret;
}


void* MediaCodecAvcDecoder::runOutputPollThread(void *ptr)
{
    MediaCodecAvcDecoder *decoder = (MediaCodecAvcDecoder*)ptr;

    while (!decoder->mThreadShouldStop)
    {
        if ((decoder->mConfigured) && (decoder->mMcw))
        {
            int pollRet = decoder->pollDecoderOutput();
            if (pollRet != 0)
            {
                ULOGE("MediaCodec: pollDecoderOutput() failed (%d)", pollRet);
            }
        }
        else
        {
            usleep(5000); //TODO: do something better (cond?)
        }
    }

    return NULL;
}

}

#endif /* USE_MEDIACODEC */
