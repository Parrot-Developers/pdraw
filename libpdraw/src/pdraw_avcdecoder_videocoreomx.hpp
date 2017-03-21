/**
 * @file pdraw_avcdecoder_videocoreomx.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - Broadcom VideoCore 4 OMX H.264/AVC video decoder
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

#ifndef _PDRAW_AVCDECODER_VIDEOCOREOMX_HPP_
#define _PDRAW_AVCDECODER_VIDEOCOREOMX_HPP_

#ifdef USE_VIDEOCOREOMX

#include <vector>
#include <queue>
#include <pthread.h>
extern "C" {
#include <ilclient.h>
}

#include "pdraw_avcdecoder.hpp"
#include "pdraw_renderer.hpp"


namespace Pdraw
{


class VideoCoreOmxAvcDecoder : public AvcDecoder
{
public:

    VideoCoreOmxAvcDecoder();

    ~VideoCoreOmxAvcDecoder();

    bool isConfigured() { return mConfigured; };

    int configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize);

    avc_decoder_color_format_t getOutputColorFormat() { return mOutputColorFormat; };

    int getInputBuffer(avc_decoder_input_buffer_t *buffer, bool blocking);

    int queueInputBuffer(avc_decoder_input_buffer_t *buffer);

    int dequeueOutputBuffer(avc_decoder_output_buffer_t *buffer, bool blocking);

    int releaseOutputBuffer(avc_decoder_output_buffer_t *buffer);

    int stop();

    void setRenderer(Renderer *renderer);

private:

    int portSettingsChanged();

    static void fillBufferDoneCallback(void *data, COMPONENT_T *comp, OMX_BUFFERHEADERTYPE *buf);

    bool mConfigured2;
    bool mFirstFrame;
    ILCLIENT_T *mClient;
    COMPONENT_T *mVideoDecode;
    COMPONENT_T *mEglRender;
    TUNNEL_T mTunnel[3];
    Renderer *mRenderer;
    OMX_BUFFERHEADERTYPE *mEglBuffer[3];
    int mCurrentEglImageIndex;
    void *mEglImage[3];
    avc_decoder_color_format_t mOutputColorFormat;
    int mFrameWidth;
    int mFrameHeight;
    int mSliceHeight;
    int mStride;
    std::vector<avc_decoder_input_buffer_t> *mBufferMeta;
    std::queue<avc_decoder_input_buffer_t*> *mBufferMetaFreeQueue;
    std::queue<avc_decoder_input_buffer_t*> *mBufferMetaPushQueue;
    pthread_mutex_t mMutex;
    pthread_cond_t mCond;
    bool mBufferReady;
    avc_decoder_output_buffer_t mOutputBuffer;
};

}

#endif /* USE_VIDEOCOREOMX */

#endif /* !_PDRAW_AVCDECODER_VIDEOCOREOMX_HPP_ */
