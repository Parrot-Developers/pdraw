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


#define FFMPEG_AVC_DECODER_INPUT_BUFFER_COUNT 5
#define FFMPEG_AVC_DECODER_INPUT_BUFFER_SIZE 1920 * 1080 / 2
#define FFMPEG_AVC_DECODER_OUTPUT_BUFFER_COUNT 5


namespace Pdraw
{


class FfmpegAvcDecoder : public AvcDecoder
{
public:

    FfmpegAvcDecoder(VideoMedia *media);

    ~FfmpegAvcDecoder();

    bool isConfigured() { return mConfigured; };

    int configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize);

    avc_decoder_color_format_t getOutputColorFormat() { return mOutputColorFormat; };

    int getInputBuffer(Buffer **buffer, bool blocking);

    int queueInputBuffer(Buffer *buffer);

    BufferQueue *addOutputQueue();

    int removeOutputQueue(BufferQueue *queue);

    int dequeueOutputBuffer(BufferQueue *queue, Buffer **buffer, bool blocking);

    int releaseOutputBuffer(Buffer *buffer);

    int stop();

    Media *getMedia() { return mMedia; };

    VideoMedia *getVideoMedia() { return (VideoMedia*)mMedia; };

private:

    bool isOutputQueueValid(BufferQueue *queue);

    static int outputBufferCreationCb(Buffer *buffer);

    static int outputBufferDeletionCb(Buffer *buffer);

    static void* runDecoderThread(void *ptr);

    int decode(Buffer *inputBuffer, Buffer *outputBuffer);

    BufferPool *mInputBufferPool;
    BufferQueue *mInputBufferQueue;
    BufferPool *mOutputBufferPool;
    std::vector<BufferQueue*> mOutputBufferQueues;
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
