/**
 * @file pdraw_renderer_videocoreegl.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - Broadcom VideoCore 4 EGL renderer
 * @date 17/12/2016
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

#include "pdraw_renderer_videocoreegl.hpp"
#include "pdraw_avcdecoder_videocoreomx.hpp"
#include "pdraw_session.hpp"

#ifdef USE_VIDEOCOREEGL

#include <string.h>
#include <unistd.h>
#include <time.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#include <bcm_host.h>


namespace Pdraw
{


VideoCoreEglRenderer::VideoCoreEglRenderer(Session *session) : Gles2Renderer(session)
{
    mVideoWidth = 0;
    mVideoHeight = 0;
    mDisplay = EGL_NO_DISPLAY;
    mContext = EGL_NO_CONTEXT;
    mSurface = EGL_NO_SURFACE;
    mEglImage[0] = NULL;
    mEglImage[1] = NULL;
    mEglImage[2] = NULL;
    mEglImageIdxReady = 0;
    mEglImageIdxDecoderLocked = 1;
    mEglImageIdxRendererLocked = 2;
    int ret = 0;

    if (ret == 0)
    {
        ret = pthread_mutex_init(&mEglImageMutex, NULL);
        if (ret != 0)
        {
            ULOGE("VideoCoreEglRenderer: mutex creation failed (%d)", ret);
        }
    }
}


VideoCoreEglRenderer::~VideoCoreEglRenderer()
{
    pthread_mutex_destroy(&mEglImageMutex);
}


int VideoCoreEglRenderer::setRendererParams
        (int windowWidth, int windowHeight,
         int renderX, int renderY,
         int renderWidth, int renderHeight,
         bool hmdDistorsionCorrection, bool headtracking,
         void *uiHandler)
{
    int ret = 0;

    /* HMD distorsion correction is not supported with VideoCore:
     * the video decoder outputs frames through the egl_renderer element,
     * this must be changed to render to FBO */
    hmdDistorsionCorrection = false; //TODO

    pthread_mutex_lock(&mMutex);

    ret = Gles2Renderer::setRendererParams_nolock(windowWidth, windowHeight, renderX, renderY,
        renderWidth, renderHeight, hmdDistorsionCorrection, headtracking, uiHandler);

    struct uiParams_s
    {
        EGLDisplay display;
        EGLSurface surface;
        EGLContext context;
    };
    struct uiParams_s *uiParams = (struct uiParams_s*)uiHandler;
    mDisplay = uiParams->display;
    mSurface = uiParams->surface;
    mContext = uiParams->context;

    if (mDecoder) ((VideoCoreOmxAvcDecoder*)mDecoder)->setRenderer(this);

    if (ret > 0)
        mRunning = true;

    pthread_mutex_unlock(&mMutex);

    return ret;
}


int VideoCoreEglRenderer::setVideoDimensions(unsigned int videoWidth, unsigned int videoHeight)
{
    int ret = 0;

    if (!mGles2Video)
    {
        ULOGE("VideoCoreEglRenderer: invalid Gles2Video context");
        return -1;
    }
    if ((mWindowWidth == 0) || (mWindowHeight == 0))
    {
        ULOGE("VideoCoreEglRenderer: invalid screen dimensions");
        return -1;
    }

    if (ret == 0)
    {
        if (eglMakeCurrent(mDisplay, mSurface, mSurface, mContext) == EGL_FALSE)
        {
            ULOGE("VideoCoreEglRenderer: eglMakeCurrent() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = mGles2Video->allocTextures(videoWidth, videoHeight);
        if (ret != 0)
        {
            ULOGE("VideoCoreEglRenderer: Gles2Video->allocTextures() failed (%d)", ret);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        mEglImage[0] = eglCreateImageKHR(mDisplay, mContext, EGL_GL_TEXTURE_2D_KHR,
                                      (EGLClientBuffer)mGles2Video->getTextures()[0], 0);
        if (mEglImage[0] == EGL_NO_IMAGE_KHR)
        {
            ULOGE("VideoCoreEglRenderer: eglCreateImageKHR() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        mEglImage[1] = eglCreateImageKHR(mDisplay, mContext, EGL_GL_TEXTURE_2D_KHR,
                                      (EGLClientBuffer)mGles2Video->getTextures()[1], 0);
        if (mEglImage[1] == EGL_NO_IMAGE_KHR)
        {
            ULOGE("VideoCoreEglRenderer: eglCreateImageKHR() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        mEglImage[2] = eglCreateImageKHR(mDisplay, mContext, EGL_GL_TEXTURE_2D_KHR,
                                      (EGLClientBuffer)mGles2Video->getTextures()[2], 0);
        if (mEglImage[2] == EGL_NO_IMAGE_KHR)
        {
            ULOGE("VideoCoreEglRenderer: eglCreateImageKHR() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        mVideoWidth = videoWidth;
        mVideoHeight = videoHeight;
    }

    eglMakeCurrent(mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    return ret;
}


void* VideoCoreEglRenderer::getVideoEglImage(int index)
{
    if ((index < 0) || (index >= 3))
    {
        ULOGE("VideoCoreEglRenderer: invalid index");
        return NULL;
    }

    return mEglImage[index];
}


int VideoCoreEglRenderer::swapDecoderEglImage()
{
    int ret;

    pthread_mutex_lock(&mEglImageMutex);

    ret = mEglImageIdxReady;
    mEglImageIdxReady = mEglImageIdxDecoderLocked;
    mEglImageIdxDecoderLocked = ret;

    pthread_mutex_unlock(&mEglImageMutex);

    return ret;
}


int VideoCoreEglRenderer::swapRendererEglImage()
{
    int ret;

    pthread_mutex_lock(&mEglImageMutex);

    ret = mEglImageIdxReady;
    mEglImageIdxReady = mEglImageIdxRendererLocked;
    mEglImageIdxRendererLocked = ret;

    pthread_mutex_unlock(&mEglImageMutex);

    return ret;
}


int VideoCoreEglRenderer::render(uint64_t lastRenderTime)
{
    int ret = 0;

    pthread_mutex_lock(&mMutex);

    if (!mRunning)
    {
        pthread_mutex_unlock(&mMutex);
        usleep(5000); //TODO
        return 0;
    }

    if ((!mDecoder) || (!mDecoder->isConfigured()))
    {
        pthread_mutex_unlock(&mMutex);
        usleep(5000); //TODO
        return 0;
    }
    if ((mWindowWidth == 0) || (mWindowHeight == 0))
    {
        pthread_mutex_unlock(&mMutex);
        usleep(5000); //TODO
        return 0;
    }
    if ((mRenderWidth == 0) || (mRenderHeight == 0))
    {
        pthread_mutex_unlock(&mMutex);
        usleep(5000); //TODO
        return 0;
    }
    if ((mVideoWidth == 0) || (mVideoHeight == 0))
    {
        pthread_mutex_unlock(&mMutex);
        usleep(5000); //TODO
        return 0;
    }

    if (eglMakeCurrent(mDisplay, mSurface, mSurface, mContext) == EGL_FALSE)
    {
        pthread_mutex_unlock(&mMutex);
        ULOGE("VideoCoreEglRenderer: eglMakeCurrent() failed");
        return 0;
    }

    Buffer *buffer = NULL, *prevBuffer = NULL;
    avc_decoder_output_buffer_t *data = NULL;
    int dequeueRet;

    while ((dequeueRet = mDecoder->dequeueOutputBuffer(mDecoderOutputBufferQueue, &buffer, false)) == 0)
    {
        if (prevBuffer)
        {
            int releaseRet = mDecoder->releaseOutputBuffer(prevBuffer);
            if (releaseRet != 0)
            {
                ULOGE("VideoCoreEglRenderer: failed to release buffer (%d)", releaseRet);
            }
        }
        prevBuffer = buffer;
    }

    if (!buffer)
    {
        if (dequeueRet != -2)
        {
            ULOGE("VideoCoreEglRenderer: failed to get buffer from queue (%d)", dequeueRet);
        }
        usleep(5000); //TODO
    }
    else
    {
        mCurrentBuffer = buffer;
    }

    if (mCurrentBuffer)
    {
        data = (avc_decoder_output_buffer_t*)mCurrentBuffer->getMetadataPtr();

        if (data)
        {
            if ((ret == 0) && (mHmdDistorsionCorrection))
            {
                glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
                glViewport(0, 0, mRenderWidth, mRenderHeight);
            }

            if ((ret == 0) && (mGles2Video))
            {
                swapRendererEglImage();

                ret = mGles2Video->renderFrame(data->plane, data->stride, data->width, data->height,
                    data->sarWidth, data->sarHeight, (mHmdDistorsionCorrection) ? mRenderWidth / 2 : mRenderWidth,
                    mRenderHeight, GLES2_VIDEO_COLOR_CONVERSION_NONE, &data->metadata, mHeadtracking);
                if (ret != 0)
                {
                    ULOGE("VideoCoreEglRenderer: failed to render frame");
                }
            }

            if ((ret == 0) && (mGles2Hud))
            {
                ret = mGles2Hud->renderHud(data->width * data->sarWidth, data->height * data->sarHeight,
                    (mHmdDistorsionCorrection) ? mRenderWidth / 2 : mRenderWidth, mRenderHeight,
                    &data->metadata, mHmdDistorsionCorrection, mHeadtracking);
                if (ret != 0)
                {
                    ULOGE("VideoCoreEglRenderer: failed to render frame");
                }
            }

            if ((ret == 0) && (mHmdDistorsionCorrection))
            {
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                glViewport(mRenderX, mRenderY, mRenderWidth, mRenderHeight);

                if (mGles2Hmd)
                {
                    ret = mGles2Hmd->renderHmd(mFboTexture, mRenderWidth, mRenderHeight);
                    if (ret != 0)
                    {
                        ULOGE("VideoCoreEglRenderer: failed to render HMD distorsion correction");
                    }
                }
            }

            if (ret == 0)
            {
                struct timespec t1;
                clock_gettime(CLOCK_MONOTONIC, &t1);
                uint64_t renderTimestamp, renderTimestamp1;
                renderTimestamp1 = renderTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

                eglSwapBuffers(mDisplay, mSurface);

                clock_gettime(CLOCK_MONOTONIC, &t1);
                renderTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

                uint64_t currentTime = mSession->getCurrentTime();
                uint64_t duration = mSession->getDuration();
                unsigned int cHrs = 0, cMin = 0, cSec = 0, cMsec = 0;
                unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
                if ((currentTime > 0) && (currentTime != (uint64_t)-1))
                    pdraw_friendlyTimeFromUs(currentTime, &cHrs, &cMin, &cSec, &cMsec);
                if ((duration > 0) && (duration != (uint64_t)-1))
                    pdraw_friendlyTimeFromUs(duration, &dHrs, &dMin, &dSec, &dMsec);

                ULOGI("VideoCoreEglRenderer: %02d:%02d:%02d.%03d / %02d:%02d:%02d.%03d frame (decoding: %.2fms, rendering: %.2fms, est. latency: %.2fms) render@%.1ffps",
                      cHrs, cMin, cSec, cMsec, dHrs, dMin, dSec, dMsec,
                      (float)(data->decoderOutputTimestamp - data->demuxOutputTimestamp) / 1000.,
                      (float)(renderTimestamp1 - data->decoderOutputTimestamp) / 1000.,
                      (data->auNtpTimestampLocal != 0) ? (float)(renderTimestamp - data->auNtpTimestampLocal) / 1000. : 0.,
                      (renderTimestamp - lastRenderTime > 0) ? 1000000. / ((float)(renderTimestamp - lastRenderTime)) : 0.);
            }
        }
    }

    if (buffer)
    {
        ret = mDecoder->releaseOutputBuffer(buffer);
        if (ret != 0)
        {
            ULOGE("VideoCoreEglRenderer: failed to release buffer (%d)", ret);
        }
    }

    pthread_mutex_unlock(&mMutex);

    return 0;
}

}

#endif /* USE_VIDEOCOREEGL */
