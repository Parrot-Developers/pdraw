/**
 * @file pdraw_avcdecoder_ffmpeg.hpp
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

#ifndef _PDRAW_AVCDECODER_FFMPEG_HPP_
#define _PDRAW_AVCDECODER_FFMPEG_HPP_

#ifdef USE_FFMPEG

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

#include <pthread.h>

#include "pdraw_avcdecoder.hpp"
#include "pdraw_bufferqueue.hpp"


#define FFMPEG_AVC_DECODER_INPUT_BUFFER_COUNT 5
#define FFMPEG_AVC_DECODER_INPUT_BUFFER_SIZE 1920 * 1080 / 2
#define FFMPEG_AVC_DECODER_OUTPUT_BUFFER_COUNT 5


namespace Pdraw
{


class FfmpegOutputBufferQueueBuffer
{
public:

    FfmpegOutputBufferQueueBuffer(void *userPtr);
    ~FfmpegOutputBufferQueueBuffer();
    AVFrame *getFrame();
    void *getUserPtr();
    void getData(avc_decoder_output_buffer_t *data);
    void setData(const avc_decoder_output_buffer_t *data);

private:

    void *mUserPtr;
    AVFrame *mAvFrame;
    avc_decoder_output_buffer_t mData;
};


class FfmpegOutputBufferQueue
{
public:

    FfmpegOutputBufferQueue(unsigned int nbElement);
    ~FfmpegOutputBufferQueue();
    buffer_queue_status_t getInputBuffer(FfmpegOutputBufferQueueBuffer **buffer, bool blocking);
    buffer_queue_status_t releaseInputBuffer(FfmpegOutputBufferQueueBuffer *buffer);
    buffer_queue_status_t cancelInputBuffer(FfmpegOutputBufferQueueBuffer *buffer);
    buffer_queue_status_t getOutputBuffer(FfmpegOutputBufferQueueBuffer **buffer, bool blocking);
    buffer_queue_status_t releaseOutputBuffer(FfmpegOutputBufferQueueBuffer *buffer);
    buffer_queue_status_t getBufferFromUserPtr(void *userPtr, FfmpegOutputBufferQueueBuffer **buffer);

private:

    class FfmpegOutputBufferQueueBufferInternal : public FfmpegOutputBufferQueueBuffer
    {
    public:

        typedef enum
        {
            BUFFER_NOT_VALID = -1,
            BUFFER_FREE = 0,
            BUFFER_INPUT_LOCK = 1,
            BUFFER_OUTPUT_LOCK = 2

        } buffer_status_t;

        buffer_status_t mStatus;
        FfmpegOutputBufferQueueBufferInternal(void *userPtr);
        ~FfmpegOutputBufferQueueBufferInternal();
    };

    bool bufferIsValid(FfmpegOutputBufferQueueBufferInternal *buffer);

    vector<FfmpegOutputBufferQueueBufferInternal*> *mBuffers;
    queue<FfmpegOutputBufferQueueBufferInternal*> *mInputQueue;
    pthread_mutex_t mInputMutex;
    pthread_cond_t mInputCond;
    queue<FfmpegOutputBufferQueueBufferInternal*> *mOutputQueue;
    pthread_mutex_t mOutputMutex;
    pthread_cond_t mOutputCond;
};


class FfmpegAvcDecoder : public AvcDecoder
{
public:

    FfmpegAvcDecoder();

    ~FfmpegAvcDecoder();

    bool isConfigured() { return mConfigured; };

    int configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize);

    avc_decoder_color_format_t getOutputColorFormat() { return mOutputColorFormat; };

    int getInputBuffer(avc_decoder_input_buffer_t *buffer, bool blocking);

    int queueInputBuffer(avc_decoder_input_buffer_t *buffer);

    int dequeueOutputBuffer(avc_decoder_output_buffer_t *buffer, bool blocking);

    int releaseOutputBuffer(avc_decoder_output_buffer_t *buffer);

    int stop();

private:

    static void* runDecoderThread(void *ptr);

    int decode(avc_decoder_input_buffer_t *inputBuffer, avc_decoder_output_buffer_t *outputBuffer, AVFrame *frame);

    BufferQueue<avc_decoder_input_buffer_t> *mInputQueue;
    FfmpegOutputBufferQueue *mOutputQueue;
    pthread_t mDecoderThread;
    bool mDecoderThreadLaunched;
    bool mThreadShouldStop;
    AVCodec *mCodecH264;
    AVCodecContext *mCodecCtxH264;
    AVPacket mPacket;
    avc_decoder_color_format_t mOutputColorFormat;
    unsigned int mFrameWidth;
    unsigned int mFrameHeight;
    unsigned int mSarWidth;
    unsigned int mSarHeight;
};

}

#endif /* USE_FFMPEG */

#endif /* !_PDRAW_AVCDECODER_FFMPEG_HPP_ */
