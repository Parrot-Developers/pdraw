/**
 * @file pdraw.hpp
 * @brief Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_HPP_
#define _PDRAW_HPP_

#include <vector>

#include <libpdraw.hpp>

#include "pdraw_session.hpp"


namespace Pdraw
{


class PdrawImpl : public IPdraw
{
public:

    PdrawImpl();

    ~PdrawImpl();

    int setup
            (const std::string &canonicalName,
             const std::string &friendlyName,
             const std::string &applicationName);

    int open
            (const std::string &url);

    int open
            (const std::string &sessionDescription,
             int qosMode);

    int open
            (const std::string &srcAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort,
             int qosMode);

    int start();

    int pause();

    bool isPaused();

    int stop();

    int seekTo
            (uint64_t timestamp);

    int seekForward
            (uint64_t delta);

    int seekBack
            (uint64_t delta);

    int startRecorder
            (const std::string &fileName);

    int stopRecorder();

    int startResender
            (const std::string &dstAddr,
             const std::string &ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort);

    int stopResender();

    int startRenderer
            (int windowWidth,
             int windowHeight,
             int renderX,
             int renderY,
             int renderWidth,
             int renderHeight,
             void *uiHandler);

    int stopRenderer();

    int render
            (int timeout);

    int getMediaCount();

    int getMediaInfo(unsigned int index, pdraw_media_info_t *info);

    inline static IPdraw *create()
    {
        return new PdrawImpl();
    }

    inline static void release
            (IPdraw *pdraw)
    {
        delete pdraw;
    }

private:

    int openWithDemux();

    Session mSession;
    bool mSetup;
    bool mPaused;
    bool mGotRendererParams;
    int mWindowWidth;
    int mWindowHeight;
    int mRenderX;
    int mRenderY;
    int mRenderWidth;
    int mRenderHeight;
    void *mUiHandler;
};

}

#endif /* !_PDRAW_HPP_ */
