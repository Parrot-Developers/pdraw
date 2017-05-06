/**
 * @file pdraw_wrapper.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - C wrapper functions
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

#include <pdraw/pdraw.h>
#include "pdraw_impl.hpp"

#include <errno.h>

using namespace Pdraw;


static IPdraw *toPdraw(struct pdraw *ptr)
{
    return (IPdraw*)ptr;
}


static struct pdraw *fromPdraw(IPdraw *ptr)
{
    return (struct pdraw*)ptr;
}


struct pdraw *pdraw_new()
{
    return fromPdraw(PdrawImpl::create());
}


int pdraw_destroy(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    PdrawImpl::release(toPdraw(pdraw));
    return 0;
}


int pdraw_setup(struct pdraw *pdraw, const char *canonicalName,
                const char *friendlyName, const char *applicationName)
{
    if ((pdraw == NULL) || (canonicalName == NULL))
    {
        return -EINVAL;
    }
    std::string cn(canonicalName);
    std::string fn(friendlyName);
    std::string an(applicationName);
    return toPdraw(pdraw)->setup(cn, fn, an);
}


int pdraw_open_url(struct pdraw *pdraw, const char *url)
{
    if ((pdraw == NULL) || (url == NULL))
    {
        return -EINVAL;
    }
    std::string u(url);
    return toPdraw(pdraw)->open(u);
}


int pdraw_open_session_description(struct pdraw *pdraw, const char *sessionDescription, int qosMode)
{
    if ((pdraw == NULL) || (sessionDescription == NULL))
    {
        return -EINVAL;
    }
    std::string sdp(sessionDescription);
    return toPdraw(pdraw)->open(sdp, qosMode);
}


int pdraw_open_single_stream(struct pdraw *pdraw, const char *srcAddr, const char *ifaceAddr,
                             int srcStreamPort, int srcControlPort,
                             int dstStreamPort, int dstControlPort, int qosMode)
{
    if ((pdraw == NULL) || (srcAddr == NULL))
    {
        return -EINVAL;
    }
    std::string ip(srcAddr);
    std::string iface(ifaceAddr);
    return toPdraw(pdraw)->open(ip, iface, srcStreamPort, srcControlPort,
                                dstStreamPort, dstControlPort, qosMode);
}


int pdraw_start(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->start();
}


int pdraw_pause(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->pause();
}


int pdraw_is_paused(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return (toPdraw(pdraw)->isPaused()) ? 1 : 0;
}


int pdraw_stop(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stop();
}


int pdraw_seek_to(struct pdraw *pdraw, uint64_t timestamp)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekTo(timestamp);
}


int pdraw_seek_forward(struct pdraw *pdraw, uint64_t delta)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekForward(delta);
}


int pdraw_seek_back(struct pdraw *pdraw, uint64_t delta)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->seekBack(delta);
}


int pdraw_start_recorder(struct pdraw *pdraw, const char *fileName)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string fn(fileName);
    return toPdraw(pdraw)->startRecorder(fn);
}


int pdraw_stop_recorder(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopRecorder();
}


int pdraw_start_resender(struct pdraw *pdraw, const char *dstAddr, const char *ifaceAddr,
                         int srcStreamPort, int srcControlPort,
                         int dstStreamPort, int dstControlPort)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    std::string ip(dstAddr);
    std::string iface(ifaceAddr);
    return toPdraw(pdraw)->startResender(ip, iface,
                                         srcStreamPort, srcControlPort,
                                         dstStreamPort, dstControlPort);
}


int pdraw_stop_resender(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopResender();
}


int pdraw_start_renderer(struct pdraw *pdraw,
                         int windowWidth, int windowHeight,
                         int renderX, int renderY,
                         int renderWidth, int renderHeight,
                         void *uiHandler)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->startRenderer(windowWidth, windowHeight,
                                         renderX, renderY,
                                         renderWidth, renderHeight,
                                         uiHandler);
}


int pdraw_stop_renderer(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->stopRenderer();
}


int pdraw_render(struct pdraw *pdraw, int timeout)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->render(timeout);
}


int pdraw_get_media_count(struct pdraw *pdraw)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getMediaCount();
}


int pdraw_get_media_info(struct pdraw *pdraw, unsigned int index, pdraw_media_info_t *info)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getMediaInfo(index, info);
}


void *pdraw_add_video_frame_filter_callback(struct pdraw *pdraw, unsigned int mediaId,
                                            pdraw_video_frame_filter_callback_t cb, void *userPtr)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->addVideoFrameFilterCallback(mediaId, cb, userPtr);
}


int pdraw_remove_video_frame_filter_callback(struct pdraw *pdraw, unsigned int mediaId, void *filterCtx)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->removeVideoFrameFilterCallback(mediaId, filterCtx);
}


void *pdraw_add_video_frame_producer(struct pdraw *pdraw, unsigned int mediaId)
{
    if (pdraw == NULL)
    {
        return NULL;
    }
    return toPdraw(pdraw)->addVideoFrameProducer(mediaId);
}


int pdraw_remove_video_frame_producer(struct pdraw *pdraw, void *producerCtx)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->removeVideoFrameProducer(producerCtx);
}


int pdraw_get_producer_last_frame(struct pdraw *pdraw, void *producerCtx, pdraw_video_frame_t *frame)
{
    if (pdraw == NULL)
    {
        return -EINVAL;
    }
    return toPdraw(pdraw)->getProducerLastFrame(producerCtx, frame);
}
