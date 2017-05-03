/**
 * @file pdraw_session.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - session
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

#include <math.h>
#include <string.h>

#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"
#include "pdraw_demuxer_stream.hpp"
#include "pdraw_demuxer_record.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


Session::Session()
{
    mDemuxer = NULL;
    mRenderer = NULL;
    mMediaIdCounter = 0;
}


Session::~Session()
{
    if (mDemuxer)
    {
        int ret = mDemuxer->stop();
        if (ret != 0)
        {
            ULOGE("Session: failed to stop demuxer");
        }
        else
        {
            delete mDemuxer;
        }
    }

    if (mRenderer)
    {
        delete mRenderer;
    }

    std::vector<Media*>::iterator m = mMedias.begin();
    while (m != mMedias.end())
    {
        delete *m;
        m++;
    }
}


int Session::setup(const std::string &selfFriendlyName,
                   const std::string &selfSerialNumber,
                   const std::string &selfSoftwareVersion)
{
    selfMetadata.setFriendlyName(selfFriendlyName);
    selfMetadata.setSerialNumber(selfSerialNumber);
    selfMetadata.setSoftwareVersion(selfSoftwareVersion);

    return 0;
}


int Session::open(const std::string &url)
{
    int ret = 0;

    std::string ext = url.substr(url.length() - 4, 4);
    if ((url.front() == '/') && (ext == ".mp4"))
    {
        mDemuxer = new RecordDemuxer();
        if (mDemuxer == NULL)
        {
            ULOGE("Session: failed to alloc demuxer");
            ret = -1;
        }
    }
    else if (((url.front() == '/') && (ext == ".sdp"))
                || (url.substr(0, 7) == "http://"))
    {
        mDemuxer = new StreamDemuxer();
        if (mDemuxer == NULL)
        {
            ULOGE("Session: failed to alloc demuxer");
            ret = -1;
        }
    }
    else
    {
        ULOGE("Session: unsupported URL");
        ret = -1;
    }

    if (ret == 0)
    {
        ret = mDemuxer->configure(url);
        if (ret != 0)
        {
            ULOGE("Session: failed to configure demuxer");
            delete mDemuxer;
            mDemuxer = NULL;
            ret = -1;
        }
    }

    return (ret == 0) ? addMediaFromDemuxer() : ret;
}


int Session::open(const std::string &sessionDescription, int qosMode)
{
    int ret = 0;

    if (ret == 0)
    {
        mDemuxer = new StreamDemuxer();
        if (mDemuxer == NULL)
        {
            ULOGE("Session: failed to alloc demuxer");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = ((StreamDemuxer*)mDemuxer)->configure(selfMetadata.getSerialNumber(),
                                                  selfMetadata.getFriendlyName(),
                                                  selfMetadata.getSoftwareVersion(),
                                                  sessionDescription, qosMode);
        if (ret != 0)
        {
            ULOGE("Session: failed to configure demuxer");
            delete mDemuxer;
            mDemuxer = NULL;
            ret = -1;
        }
    }

    return (ret == 0) ? addMediaFromDemuxer() : ret;
}


int Session::open(const std::string &srcAddr, const std::string &ifaceAddr,
                  int srcStreamPort, int srcControlPort,
                  int dstStreamPort, int dstControlPort, int qosMode)
{
    int ret = 0;

    if (ret == 0)
    {
        mDemuxer = new StreamDemuxer();
        if (mDemuxer == NULL)
        {
            ULOGE("Session: failed to alloc demuxer");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = ((StreamDemuxer*)mDemuxer)->configure(selfMetadata.getSerialNumber(),
                                                  selfMetadata.getFriendlyName(),
                                                  selfMetadata.getSoftwareVersion(),
                                                  srcAddr, ifaceAddr, srcStreamPort, srcControlPort,
                                                  dstStreamPort, dstControlPort, qosMode);
        if (ret != 0)
        {
            ULOGE("Session: failed to configure demuxer");
            delete mDemuxer;
            mDemuxer = NULL;
            ret = -1;
        }
    }

    return (ret == 0) ? addMediaFromDemuxer() : ret;
}


int Session::addMediaFromDemuxer()
{
    int ret = 0, esCount = 0;

    if (ret == 0)
    {
        esCount = mDemuxer->getElementaryStreamCount();
        if (esCount < 0)
        {
            ULOGE("Session: getElementaryStreamCount() failed (%d)", esCount);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int i;
        for (i = 0; i < esCount; i++)
        {
            elementary_stream_type_t esType = mDemuxer->getElementaryStreamType(i);
            if (esType < 0)
            {
                ULOGE("Session: getElementaryStreamType() failed (%d)", esType);
                continue;
            }

            Media *m = addMedia(esType, mDemuxer, i);
            if (!m)
            {
                ULOGE("Session: media creation failed");
            }
        }
    }

    return ret;
}


Media *Session::addMedia(elementary_stream_type_t esType)
{
    Media *m = NULL;
    switch (esType)
    {
        case ELEMENTARY_STREAM_TYPE_UNKNOWN:
        default:
            break;
        case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
            m = new VideoMedia(esType, mMediaIdCounter++);
            m->enableDecoder();
            break;
    }

    if (m == NULL)
    {
        ULOGE("Session: media creation failed");
        return NULL;
    }

    mMedias.push_back(m);

    return m;
}


Media *Session::addMedia(elementary_stream_type_t esType, Demuxer *demuxer, int demuxEsIndex)
{
    Media *m = NULL;
    switch (esType)
    {
        case ELEMENTARY_STREAM_TYPE_UNKNOWN:
        default:
            break;
        case ELEMENTARY_STREAM_TYPE_VIDEO_AVC:
            m = new VideoMedia(esType, mMediaIdCounter++, demuxer, demuxEsIndex);
            m->enableDecoder();
            break;
    }

    if (m == NULL)
    {
        ULOGE("Session: media creation failed");
        return NULL;
    }

    mMedias.push_back(m);

    return m;
}


int Session::removeMedia(Media *media)
{
    bool found = false;
    std::vector<Media*>::iterator m = mMedias.begin();

    while (m != mMedias.end())
    {
        if (*m == media)
        {
            mMedias.erase(m);
            delete *m;
            found = true;
            break;
        }
        m++;
    }

    return (found) ? 0 : -1;
}


int Session::removeMedia(unsigned int index)
{
    if (index >= mMedias.size())
    {
        return -1;
    }

    Media *m = mMedias.at(index);
    mMedias.erase(mMedias.begin() + index);
    delete m;

    return 0;
}


unsigned int Session::getMediaCount()
{
    return mMedias.size();
}


Media *Session::getMedia(unsigned int index)
{
    if (index >= mMedias.size())
    {
        return NULL;
    }

    return mMedias.at(index);
}


int Session::enableRenderer()
{
    int ret = 0;

    if (mRenderer)
    {
        ULOGE("Session: renderer is already enabled");
        return -1;
    }

    mRenderer = Renderer::create();
    if (mRenderer == NULL)
    {
        ULOGE("Session: failed to alloc renderer");
        ret = -1;
    }
    else
    {
        std::vector<Media*>::iterator m;

        for (m = mMedias.begin(); m < mMedias.end(); m++)
        {
            if ((*m)->getType() == MEDIA_TYPE_VIDEO)
            {
                ret = mRenderer->addAvcDecoder((AvcDecoder*)((*m)->getDecoder()));
                if (ret != 0)
                {
                    ULOGE("Session: failed add decoder to renderer");
                }
            }
        }
    }

    return ret;
}


int Session::disableRenderer()
{
    int ret = 0;

    if (!mRenderer)
    {
        ULOGE("Session: renderer is not enabled");
        return -1;
    }

    std::vector<Media*>::iterator m;

    for (m = mMedias.begin(); m < mMedias.end(); m++)
    {
        if ((*m)->getType() == MEDIA_TYPE_VIDEO)
        {
            ret = mRenderer->removeAvcDecoder((AvcDecoder*)((*m)->getDecoder()));
            if (ret != 0)
            {
                ULOGE("Session: failed remove decoder from renderer");
            }
        }
    }

    delete mRenderer;
    mRenderer = NULL;

    return ret;
}


Demuxer *Session::getDemuxer()
{
    return mDemuxer;
}


Renderer *Session::getRenderer()
{
    return mRenderer;
}


SessionSelfMetadata& Session::getSelfMetadata()
{
    return selfMetadata;
}

}
