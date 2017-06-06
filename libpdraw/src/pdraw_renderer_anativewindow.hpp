/**
 * @file pdraw_renderer_anativewindow.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - Android NDK native window renderer
 * @date 23/11/2016
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

#ifndef _PDRAW_RENDERER_ANATIVEWINDOW_HPP_
#define _PDRAW_RENDERER_ANATIVEWINDOW_HPP_

#ifdef USE_ANATIVEWINDOW

#include <pthread.h>
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <android/native_window.h>

#include "pdraw_renderer_gles2.hpp"
#include "pdraw_gles2_video.hpp"
#include "pdraw_gles2_hud.hpp"


namespace Pdraw
{


class ANativeWindowRenderer : public Gles2Renderer
{
public:

    ANativeWindowRenderer(Session *session);

    ~ANativeWindowRenderer();

    int setRendererParams
            (int windowWidth, int windowHeight,
             int renderX, int renderY,
             int renderWidth, int renderHeight,
             bool hmdDistorsionCorrection, bool headtracking,
             void *uiHandler);

    int render(int timeout);

private:

    static void* runRendererThread(void *ptr);

    pthread_t mRendererThread;
    bool mRendererThreadLaunched;
    bool mThreadShouldStop;
    int mVideoWidth;
    int mVideoHeight;
    ANativeWindow *mWindow;
    EGLDisplay mDisplay;
    EGLSurface mSurface;
    EGLContext mContext;
};

}

#endif /* USE_ANATIVEWINDOW */

#endif /* !_PDRAW_RENDERER_ANATIVEWINDOW_HPP_ */
