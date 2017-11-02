/**
 * @file pdraw_avcdecoder.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - H.264/AVC decoder interface
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

#ifndef _PDRAW_AVCDECODER_HPP_
#define _PDRAW_AVCDECODER_HPP_

#include <inttypes.h>
#include "pdraw_decoder.hpp"
#include "pdraw_metadata_videoframe.hpp"

#include <video-buffers/vbuf.h>


namespace Pdraw
{


typedef enum
{
    AVCDECODER_COLORFORMAT_UNKNOWN = 0,
    AVCDECODER_COLORFORMAT_YUV420PLANAR,
    AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR,

} avc_decoder_color_format_t;


typedef struct
{
    bool isComplete;
    bool hasErrors;
    bool isRef;
    bool isSilent;
    uint64_t auNtpTimestamp;
    uint64_t auNtpTimestampRaw;
    uint64_t auNtpTimestampLocal;
    bool hasMetadata;
    struct vmeta_frame_v2 metadata;
    uint64_t demuxOutputTimestamp;

} avc_decoder_input_buffer_t;


typedef struct
{
    uint8_t *plane[3];
    unsigned int stride[3];
    unsigned int width;
    unsigned int height;
    unsigned int sarWidth;
    unsigned int sarHeight;
    avc_decoder_color_format_t colorFormat;
    bool isComplete;
    bool hasErrors;
    bool isRef;
    bool isSilent;
    uint64_t auNtpTimestamp;
    uint64_t auNtpTimestampRaw;
    uint64_t auNtpTimestampLocal;
    bool hasMetadata;
    struct vmeta_frame_v2 metadata;
    uint64_t demuxOutputTimestamp;
    uint64_t decoderOutputTimestamp;

} avc_decoder_output_buffer_t;


class VideoMedia;


class AvcDecoder : public Decoder
{
public:

    virtual int configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize) = 0;

    virtual avc_decoder_color_format_t getOutputColorFormat() = 0;

    virtual int getInputBuffer(struct vbuf_buffer **buffer, bool blocking) = 0;

    virtual int queueInputBuffer(struct vbuf_buffer *buffer) = 0;

    virtual struct vbuf_queue *addOutputQueue() = 0;

    virtual int removeOutputQueue(struct vbuf_queue *queue) = 0;

    virtual int dequeueOutputBuffer(struct vbuf_queue *queue, struct vbuf_buffer **buffer, bool blocking) = 0;

    virtual int releaseOutputBuffer(struct vbuf_buffer **buffer) = 0;

    virtual int stop() = 0;

    virtual VideoMedia *getVideoMedia() = 0;

    static AvcDecoder *create(VideoMedia *media);

protected:

    virtual bool isOutputQueueValid(struct vbuf_queue *queue) = 0;
};

}

#endif /* !_PDRAW_AVCDECODER_HPP_ */
