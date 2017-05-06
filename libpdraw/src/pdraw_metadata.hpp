/**
 * @file pdraw_metadata.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - metadata
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

#ifndef _PDRAW_METADATA_HPP_
#define _PDRAW_METADATA_HPP_

#include <inttypes.h>
#include <string>

#include <pdraw/pdraw_defs.h>

#include "pdraw_utils.hpp"

using namespace std;


namespace Pdraw
{


typedef enum
{
    FRAME_METADATA_SOURCE_RECORDING = 0,
    FRAME_METADATA_SOURCE_STREAMING,

} frame_metadata_source_t;


#define frame_metadata_t pdraw_frame_metadata_t


class Metadata
{
public:

    Metadata();

    ~Metadata();

    string getTitle(void);
    void setTitle(const string& title);

    string getMaker(void);
    void setMaker(const string& maker);

    string getModel(void);
    void setModel(const string& model);

    string getVersion(void);
    void setVersion(const string& version);

    string getSerial(void);
    void setSerial(const string& serial);

    string getCopyright(void);
    void setCopyright(const string& copyright);

    string getComment(void);
    void setComment(const string& comment);

    string getMediaDate(void);
    void setMediaDate(const string& mediaDate);

    string getRunDate(void);
    void setRunDate(const string& runDate);

    string getRunUuid(void);
    void setRunUuid(const string& runUuid);

    void getTakeoffLocation(location_t *takeoff);
    void setTakeoffLocation(const location_t *takeoff);

    void getHomeLocation(location_t *home);
    void setHomeLocation(const location_t *home);

    void getPilotLocation(location_t *pilot);
    void setPilotLocation(const location_t *pilot);

    void getPictureFov(float *pictureHFov, float *pictureVFov);
    void setPictureFov(float pictureHFov, float pictureVFov);

    static bool decodeFrameMetadata(const void *metadataBuffer, unsigned int metadataSize,
                                    frame_metadata_source_t source, const char *mimeType, frame_metadata_t *metadata);

private:

    string mTitle;
    string mMaker;
    string mModel;
    string mVersion;
    string mSerial;
    string mCopyright;
    string mComment;
    string mMediaDate;
    string mRunDate;
    string mRunUuid;
    location_t mTakeoffLocation;
    location_t mHomeLocation;
    location_t mPilotLocation;
    float mPictureHFov;
    float mPictureVFov;
};

}

#endif /* !_PDRAW_METADATA_HPP_ */
