/**
 * @file pdraw_demuxer.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - demuxer interface
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

#ifndef _PDRAW_DEMUXER_HPP_
#define _PDRAW_DEMUXER_HPP_

#include "pdraw_decoder.hpp"
#include <string>


namespace Pdraw
{


typedef enum
{
    DEMUXER_TYPE_UNKNOWN = 0,
    DEMUXER_TYPE_RECORD,
    DEMUXER_TYPE_STREAM,

} demuxer_type_t;


typedef enum
{
    ELEMENTARY_STREAM_TYPE_UNKNOWN = 0,
    ELEMENTARY_STREAM_TYPE_VIDEO_AVC,

} elementary_stream_type_t;


class Demuxer
{
public:

    virtual ~Demuxer() {};

    virtual demuxer_type_t getType() = 0;

    virtual bool isConfigured() = 0;

    virtual int configure(const std::string &url) = 0;

    virtual int getElementaryStreamCount() = 0;

    virtual elementary_stream_type_t getElementaryStreamType(int esIndex) = 0;

    virtual int setElementaryStreamDecoder(int esIndex, Decoder *decoder) = 0;

    virtual int start() = 0;

    virtual int pause() = 0;

    virtual int stop() = 0;

    virtual int seekTo
            (uint64_t timestamp) = 0;

    virtual int seekForward
            (uint64_t delta) = 0;

    virtual int seekBack
            (uint64_t delta) = 0;

protected:

    bool mConfigured;
};

}

#endif /* !_PDRAW_DEMUXER_HPP_ */
