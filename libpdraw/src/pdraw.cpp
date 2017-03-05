/**
 * @file pdraw.cpp
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

#include "pdraw.hpp"
#include "pdraw_demuxer_stream.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_decoder.hpp"
#include "pdraw_avcdecoder.hpp"
#include "pdraw_renderer.hpp"

#include <unistd.h>
#include <sched.h>

#define ULOG_TAG libpdraw
#include <ulog.h>
ULOG_DECLARE_TAG(libpdraw);


namespace Pdraw
{


IPdraw *createPdraw()
{
    return new PdrawImpl;
}


PdrawImpl::PdrawImpl()
{
    mSetup = false;
    mDemux = NULL;
    mDecoder = new std::vector<Decoder*>();
    mRenderer = NULL;
    mPaused = false;
    mGotRendererParams = false;
    mUiHandler = NULL;
}


PdrawImpl::~PdrawImpl()
{
    if (mDecoder)
    {
        std::vector<Decoder*>::iterator dec = mDecoder->begin();
        while (dec != mDecoder->end())
        {
            delete *dec;
            dec++;
        }
        delete mDecoder;
    }
    if (mDemux) delete mDemux;
    if (mRenderer) delete mRenderer;
}


int PdrawImpl::setup(const std::string &canonicalName,
                     const std::string &friendlyName,
                     const std::string &applicationName)
{
    int ret = 0;

    if (mSetup)
    {
        ULOGE("Pdraw is already set up");
        return -1;
    }

    mCanonicalName = canonicalName;
    mFriendlyName = friendlyName;
    mApplicationName = applicationName;
    mSetup = true;

    return ret;
}


int PdrawImpl::open(const std::string &url)
{
    int ret = 0;

    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    std::string ext = url.substr(url.length() - 4, 4);
    if ((url.front() == '/') && (ext == ".mp4"))
    {
        mDemux = new RecordDemuxer();
        if (mDemux == NULL)
        {
            ULOGE("Failed to alloc demuxer");
            ret = -1;
        }
    }
    else if (((url.front() == '/') && (ext == ".sdp"))
                || (url.substr(0, 7) == "http://"))
    {
        mDemux = new StreamDemuxer();
        if (mDemux == NULL)
        {
            ULOGE("Failed to alloc demuxer");
            ret = -1;
        }
    }
    else
    {
        ULOGE("Unsupported URL");
        ret = -1;
    }

    if (ret == 0)
    {
        ret = mDemux->configure(url);
        if (ret != 0)
        {
            ULOGE("Failed to configure demuxer");
            ret = -1;
        }
    }

    return (ret == 0) ? openWithDemux() : ret;
}


int PdrawImpl::open(const std::string &sessionDescription, int qosMode)
{
    int ret = 0;

    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (ret == 0)
    {
        mDemux = new StreamDemuxer();
        if (mDemux == NULL)
        {
            ULOGE("Failed to alloc demuxer");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = ((StreamDemuxer*)mDemux)->configure(mCanonicalName, mFriendlyName, mApplicationName,
                                                  sessionDescription, qosMode);
        if (ret != 0)
        {
            ULOGE("Failed to configure demuxer");
            ret = -1;
        }
    }

    return (ret == 0) ? openWithDemux() : ret;
}


int PdrawImpl::open(const std::string &srcAddr, const std::string &ifaceAddr,
                    int srcStreamPort, int srcControlPort,
                    int dstStreamPort, int dstControlPort, int qosMode)
{
    int ret = 0;

    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (ret == 0)
    {
        mDemux = new StreamDemuxer();
        if (mDemux == NULL)
        {
            ULOGE("Failed to alloc demuxer");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = ((StreamDemuxer*)mDemux)->configure(mCanonicalName, mFriendlyName, mApplicationName,
                                                  srcAddr, ifaceAddr, srcStreamPort, srcControlPort,
                                                  dstStreamPort, dstControlPort, qosMode);
        if (ret != 0)
        {
            ULOGE("Failed to configure demuxer");
            ret = -1;
        }
    }

    return (ret == 0) ? openWithDemux() : ret;
}


int PdrawImpl::openWithDemux()
{
    int ret = 0;
    Decoder *firstDecoder = NULL; //TODO

    if (ret == 0)
    {
        int esCount = mDemux->getElementaryStreamCount();
        if (esCount < 0)
        {
            ULOGE("getElementaryStreamCount() failed (%d)", esCount);
            ret = -1;
        }
        int i, _ret;
        for (i = 0; i < ((esCount) && (ret == 0)); i++)
        {
            elementary_stream_type_t esType = mDemux->getElementaryStreamType(i);
            if (esType < 0)
            {
                ULOGE("getElementaryStreamType() failed (%d)", esType);
                continue;
            }

            switch (esType)
            {
                case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
                {
                    Decoder *decoder = AvcDecoder::create();
                    if (decoder)
                    {
                        if (firstDecoder == NULL)
                        {
                            firstDecoder = decoder;
                        }
                        mDecoder->push_back(decoder);
                        _ret = mDemux->setElementaryStreamDecoder(i, decoder);
                        if (_ret != 0)
                        {
                            ULOGE("setElementaryStreamDecoder() failed (%d)", _ret);
                            ret = -1;
                        }
                    }
                    else
                    {
                        ULOGE("failed to create AVC decoder");
                    }
                    break;
                }
                default:
                case ELEMENTARY_STREAM_TYPE_UNKNOWN:
                    break;
            }
        }
    }

    if (ret == 0)
    {
        mRenderer = Renderer::create((AvcDecoder*)firstDecoder);
        if (mRenderer == NULL)
        {
            ULOGE("Failed to alloc renderer");
            ret = -1;
        }
        else if (mGotRendererParams)
        {
            mRenderer->setRendererParams(mWindowWidth, mWindowHeight,
                                         mRenderX, mRenderY,
                                         mRenderWidth, mRenderHeight,
                                         mUiHandler);
        }
    }

    return ret;
}


int PdrawImpl::start()
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (mDemux)
    {
        int ret = mDemux->start();
        if (ret != 0)
        {
            ULOGE("Failed to start demuxer");
            return -1;
        }
        else
        {
            mPaused = false;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::pause()
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (mDemux)
    {
        int ret = mDemux->pause();
        if (ret != 0)
        {
            ULOGE("Failed to pause demuxer");
            return -1;
        }
        else
        {
            mPaused = true;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


bool PdrawImpl::isPaused()
{
    return mPaused;
}


int PdrawImpl::stop()
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (mDemux)
    {
        int ret = mDemux->stop();
        if (ret != 0)
        {
            ULOGE("Failed to stop demuxer");
            return -1;
        }
        else
        {
            mPaused = false;
        }
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }

    return 0;
}


int PdrawImpl::startRecorder(const std::string &fileName)
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if ((mDemux) && (mDemux->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mDemux)->startRecorder(fileName);
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::stopRecorder()
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if ((mDemux) && (mDemux->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mDemux)->stopRecorder();
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::startResender(const std::string &dstAddr, const std::string &ifaceAddr,
                             int srcStreamPort, int srcControlPort,
                             int dstStreamPort, int dstControlPort)
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if ((mDemux) && (mDemux->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mDemux)->startResender(dstAddr, ifaceAddr,
                                                       srcStreamPort, srcControlPort,
                                                       dstStreamPort, dstControlPort);
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::stopResender()
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if ((mDemux) && (mDemux->getType() == DEMUXER_TYPE_STREAM))
    {
        return ((StreamDemuxer*)mDemux)->stopResender();
    }
    else
    {
        ULOGE("Invalid demuxer");
        return -1;
    }
}


int PdrawImpl::setRendererParams(int windowWidth, int windowHeight,
                                 int renderX, int renderY,
                                 int renderWidth, int renderHeight,
                                 void *uiHandler)
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    mGotRendererParams = true;
    mUiHandler = uiHandler;
    mWindowWidth = windowWidth;
    mWindowHeight = windowHeight;
    mRenderX = renderX;
    mRenderY = renderY;
    mRenderWidth = renderWidth;
    mRenderHeight = renderHeight;
    if (mRenderer)
    {
        return mRenderer->setRendererParams(mWindowWidth, mWindowHeight,
                                            mRenderX, mRenderY,
                                            mRenderWidth, mRenderHeight,
                                            mUiHandler);
    }
    else
    {
        ULOGE("Invalid renderer");
        return -1;
    }
}


int PdrawImpl::render(int timeout)
{
    if (!mSetup)
    {
        ULOGE("Pdraw is not set up");
        return -1;
    }

    if (mRenderer)
    {
        return mRenderer->render(timeout);
    }
    else
    {
        ULOGE("Invalid renderer");
        return -1;
    }
}

}
