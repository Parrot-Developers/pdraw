/**
 * @file pdraw_session.hpp
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

#ifndef _PDRAW_SESSION_HPP_
#define _PDRAW_SESSION_HPP_

#include <inttypes.h>
#include <string>
#include <vector>

#include "pdraw_metadata_session.hpp"
#include "pdraw_media.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_renderer.hpp"


using namespace std;


namespace Pdraw
{


class Settings;


class Session
{
public:

    Session(Settings *settings);

    ~Session();

    int open(const std::string &url);

    int open(const std::string &srcAddr, const std::string &ifaceAddr,
             int srcStreamPort, int srcControlPort,
             int dstStreamPort, int dstControlPort, int qosMode);

    Media *addMedia(elementary_stream_type_t esType);

    Media *addMedia(elementary_stream_type_t esType, Demuxer *demuxer, int demuxEsIndex);

    int removeMedia(Media *media);

    int removeMedia(unsigned int index);

    unsigned int getMediaCount();

    Media *getMedia(unsigned int index);

    Media *getMediaById(unsigned int id);

    int enableRenderer();
    int disableRenderer();

    uint64_t getDuration();

    uint64_t getCurrentTime();

    Demuxer *getDemuxer() { return mDemuxer; };

    Renderer *getRenderer() { return mRenderer; };

    Settings *getSettings() { return mSettings; };

    SessionSelfMetadata *getSelfMetadata() { return &mSelfMetadata; };

    SessionPeerMetadata *getPeerMetadata() { return &mPeerMetadata; };

    void getCameraOrientationForHeadtracking(float *pan, float *tilt);

private:

    int addMediaFromDemuxer();

    Settings *mSettings;
    SessionSelfMetadata mSelfMetadata;
    SessionPeerMetadata mPeerMetadata;
    std::vector<Media*> mMedias;
    Demuxer *mDemuxer;
    Renderer *mRenderer;
    unsigned int mMediaIdCounter;
};

}

#endif /* !_PDRAW_SESSION_HPP_ */
