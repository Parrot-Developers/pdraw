/**
 * @file pdraw_renderer_gles2.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - OpenGL ES 2.0 renderer
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

#include "pdraw_renderer_gles2.hpp"

#ifdef USE_GLES2

#include <string.h>
#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


Gles2Renderer::Gles2Renderer()
{
    mWindowWidth = 0;
    mWindowHeight = 0;
    mRenderX = 0;
    mRenderY = 0;
    mRenderWidth = 0;
    mRenderHeight = 0;
    mDecoder = NULL;
    mGles2Video = NULL;
    mGles2Hud = NULL;
    mGles2VideoFirstTexUnit = 1;
    mGles2HudFirstTexUnit = mGles2VideoFirstTexUnit + Gles2Video::getTexUnitCount();
    int ret = 0;

    if (ret == 0)
    {
        mGles2Video = new Gles2Video(mGles2VideoFirstTexUnit);
        if (!mGles2Video)
        {
            ULOGE("Gles2Renderer: failed to create Gles2Video context");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        mGles2Hud = new Gles2Hud(mGles2HudFirstTexUnit);
        if (!mGles2Hud)
        {
            ULOGE("Gles2Renderer: failed to create Gles2Hud context");
            ret = -1;
        }
    }
}


Gles2Renderer::~Gles2Renderer()
{
    glClear(GL_COLOR_BUFFER_BIT);

    if (mGles2Video) delete mGles2Video;
    if (mGles2Hud) delete mGles2Hud;
}


int Gles2Renderer::addAvcDecoder(AvcDecoder *decoder)
{
    if (mDecoder)
    {
        ULOGE("Gles2Renderer: multiple decoders are not supported");
        return -1;
    }

    mDecoder = decoder;

    return 0;
}


int Gles2Renderer::setRendererParams
        (int windowWidth, int windowHeight,
         int renderX, int renderY,
         int renderWidth, int renderHeight,
         void *uiHandler)
{
    int ret = 0;

    mWindowWidth = windowWidth;
    mWindowHeight = windowHeight;
    mRenderX = renderX;
    mRenderY = renderY;
    mRenderWidth = renderWidth;
    mRenderHeight = renderHeight;

    // Set background color and clear buffers
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    //glDisable(GL_BLEND);
    glDisable(GL_DITHER);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    glViewport(mRenderX, mRenderY, mRenderWidth, mRenderHeight);

    return ret;
}


int Gles2Renderer::render(int timeout)
{
    int ret = 0;

    if ((mDecoder) && (mDecoder->isConfigured()))
    {
        avc_decoder_output_buffer_t buf, prevBuf;
        avc_decoder_output_buffer_t *buffer = NULL, *prevBuffer = NULL;
        int dequeueRet;

        while ((dequeueRet = mDecoder->dequeueOutputBuffer(&buf, false)) == 0)
        {
            buffer = &buf;
            if (prevBuffer)
            {
                int releaseRet = mDecoder->releaseOutputBuffer(prevBuffer);
                if (releaseRet != 0)
                {
                    ULOGE("Gles2Renderer: failed to release buffer (%d)", releaseRet);
                }
            }
            memcpy(&prevBuf, &buf, sizeof(buf));
            prevBuffer = &prevBuf;
        }

        if (!buffer)
        {
            if (dequeueRet != -2)
            {
                ULOGE("Gles2Renderer: failed to get buffer from queue (%d)", dequeueRet);
            }
            usleep(5000); //TODO
        }

        if ((buffer) && (ret == 0))
        {
            if ((mRenderWidth) && (mRenderHeight))
            {
                if (ret == 0)
                {
                    if (mGles2Video)
                    {
                        gles2_video_color_conversion_t colorConversion;
                        switch (buffer->colorFormat)
                        {
                            default:
                            case AVCDECODER_COLORFORMAT_YUV420PLANAR:
                                colorConversion = GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB;
                                break;
                            case AVCDECODER_COLORFORMAT_YUV420SEMIPLANAR:
                                colorConversion = GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB;
                                break;
                        }
                        ret = mGles2Video->renderFrame(buffer->plane, buffer->stride,
                                                       buffer->width, buffer->height,
                                                       buffer->sarWidth, buffer->sarHeight,
                                                       mRenderWidth, mRenderHeight,
                                                       colorConversion);
                        if (ret != 0)
                        {
                            ULOGE("Gles2Renderer: failed to render frame");
                        }
                    }
                }

                if (ret == 0)
                {
                    if (mGles2Hud)
                    {
                        ret = mGles2Hud->renderHud((float)buffer->width / (float)buffer->height, &buffer->metadata);
                        if (ret != 0)
                        {
                            ULOGE("Gles2Renderer: failed to render frame");
                        }
                    }
                }

                if (ret == 0)
                {
                    struct timespec t1;
                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    uint64_t renderTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

                    ULOGI("Gles2Renderer: frame (decoding: %.2fms, rendering: %.2fms, est. latency: %.2fms)",
                          (float)(buffer->decoderOutputTimestamp - buffer->demuxOutputTimestamp) / 1000.,
                          (float)(renderTimestamp - buffer->decoderOutputTimestamp) / 1000.,
                          (buffer->auNtpTimestampLocal != 0) ? (float)(renderTimestamp - buffer->auNtpTimestampLocal) / 1000. : 0.);
                }
            }

            ret = mDecoder->releaseOutputBuffer(buffer);
            if (ret != 0)
            {
                ULOGE("Gles2Renderer: failed to release buffer (%d)", ret);
            }
        }
    }
    else
    {
        usleep(5000); //TODO
    }

    return ret;
}

}

#endif /* USE_GLES2 */
