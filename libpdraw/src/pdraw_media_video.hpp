/**
 * Parrot Drones Awesome Video Viewer Library
 * Video media
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_VIDEO_MEDIA_HPP_
#define _PDRAW_VIDEO_MEDIA_HPP_

#include <inttypes.h>
#include <string>
#include <vector>

#include <pdraw/pdraw_defs.h>

#include "pdraw_media.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_filter_videoframe.hpp"

using namespace std;


namespace Pdraw
{


class VideoMedia : public Media
{
public:

    VideoMedia(Session *session, elementary_stream_type_t esType, unsigned int id);

    VideoMedia(Session *session, elementary_stream_type_t esType, unsigned int id, Demuxer *demux, int demuxEsIndex);

    ~VideoMedia();

    enum pdraw_media_type getType() { return PDRAW_MEDIA_TYPE_VIDEO; };

    unsigned int getId() { return mId; };

    enum pdraw_video_type getVideoType() { return mVideoType; };
    void setVideoType(enum pdraw_video_type type) { mVideoType = type; };

    void getDimensions(unsigned int *width, unsigned int *height,
        unsigned int *cropLeft, unsigned int *cropRight,
        unsigned int *cropTop, unsigned int *cropBottom,
        unsigned int *croppedWidth, unsigned int *croppedHeight,
        unsigned int *sarWidth, unsigned int *sarHeight);
    void setDimensions(unsigned int width, unsigned int height,
        unsigned int cropLeft, unsigned int cropRight,
        unsigned int cropTop, unsigned int cropBottom,
        unsigned int sarWidth, unsigned int sarHeight);

    void getFov(float *hfov, float *vfov);
    void setFov(float hfov, float vfov);

    int enableDecoder();
    int disableDecoder();

    Session *getSession() { return mSession; };

    Decoder *getDecoder() { return mDecoder; };

    VideoFrameFilter *addVideoFrameFilter(bool frameByFrame = false);
    VideoFrameFilter *addVideoFrameFilter(pdraw_video_frame_filter_callback_t cb, void *userPtr, bool frameByFrame = false);
    int removeVideoFrameFilter(VideoFrameFilter *filter);

private:

    bool isVideoFrameFilterValid(VideoFrameFilter *filter);

    elementary_stream_type_t mEsType;
    enum pdraw_video_type mVideoType;
    unsigned int mWidth;
    unsigned int mHeight;
    unsigned int mCropLeft;
    unsigned int mCropRight;
    unsigned int mCropTop;
    unsigned int mCropBottom;
    unsigned int mSarWidth;
    unsigned int mSarHeight;
    float mHfov;
    float mVfov;
    Demuxer *mDemux;
    int mDemuxEsIndex;
    Decoder *mDecoder;
    std::vector<VideoFrameFilter*> mVideoFrameFilters;
};

}

#endif /* !_PDRAW_VIDEO_MEDIA_HPP_ */
