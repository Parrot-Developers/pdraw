/**
 * @file pdraw_media.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - video media
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

#include "pdraw_media_video.hpp"
#include "pdraw_avcdecoder.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


VideoMedia::VideoMedia(elementary_stream_type_t esType)
{
    mEsType = esType;
    mDemux = NULL;
    mDemuxEsIndex = -1;
    mDecoder = NULL;
}


VideoMedia::VideoMedia(elementary_stream_type_t esType, Demuxer *demux, int demuxEsIndex)
{
    mEsType = esType;
    mDemux = demux;
    mDemuxEsIndex = demuxEsIndex;
    mDecoder = NULL;
}


VideoMedia::~VideoMedia()
{
    if (mDecoder)
    {
        disableDecoder();
    }
}


int VideoMedia::enableDecoder()
{
    int ret = 0;

    if (mDecoder)
    {
        ULOGE("VideoMedia: decoder is already enabled");
        return -1;
    }

    mDecoder = AvcDecoder::create();
    if (mDecoder)
    {
        if ((mDemuxEsIndex != -1) && (mDemux))
        {
            int _ret = mDemux->setElementaryStreamDecoder(mDemuxEsIndex, mDecoder);
            if (_ret != 0)
            {
                ULOGE("VideoMedia: setElementaryStreamDecoder() failed (%d)", _ret);
                ret = -1;
            }
        }
    }
    else
    {
        ULOGE("VideoMedia: failed to create AVC decoder");
    }

    return ret;
}


int VideoMedia::disableDecoder()
{
    int ret = 0;

    if (!mDecoder)
    {
        ULOGE("VideoMedia: decoder is not enabled");
        return -1;
    }

    int _ret = ((AvcDecoder*)mDecoder)->stop();
    if (_ret != 0)
    {
        ULOGE("VideoMedia: failed to stop AVC decoder (%d)", _ret);
        ret = -1;
    }
    else
    {
        delete mDecoder;
        mDecoder = NULL;
    }

    return ret;
}


Decoder *VideoMedia::getDecoder()
{
    return mDecoder;
}

}
