/**
 * Parrot Drones Awesome Video Viewer Library
 * Recording demuxer
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

#include "pdraw_demuxer_record.hpp"
#include "pdraw_session.hpp"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <json-c/json.h>
#include <video-streaming/vstrm.h>

#define ULOG_TAG libpdraw
#include <ulog.h>


#define RECORD_DEMUXER_UDTA_KEY_LOCATION "\251xyz"
#define RECORD_DEMUXER_UDTA_KEY_ARTIST "\251ART"
#define RECORD_DEMUXER_UDTA_KEY_TITLE "\251nam"
#define RECORD_DEMUXER_UDTA_KEY_DATE "\251day"
#define RECORD_DEMUXER_UDTA_KEY_COMMENT "\251cmt"
#define RECORD_DEMUXER_UDTA_KEY_COPYRIGHT "\251cpy"
#define RECORD_DEMUXER_UDTA_KEY_MAKER "\251mak"
#define RECORD_DEMUXER_UDTA_KEY_MODEL "\251mod"
#define RECORD_DEMUXER_UDTA_KEY_VERSION "\251swr"
#define RECORD_DEMUXER_UDTA_KEY_SERIAL "\251too"
#define RECORD_DEMUXER_UDTA_KEY_COVER "covr"

#define RECORD_DEMUXER_META_KEY_LOCATION "com.apple.quicktime.location.ISO6709"
#define RECORD_DEMUXER_META_KEY_ARTIST "com.apple.quicktime.artist"
#define RECORD_DEMUXER_META_KEY_TITLE "com.apple.quicktime.title"
#define RECORD_DEMUXER_META_KEY_DATE "com.apple.quicktime.creationdate"
#define RECORD_DEMUXER_META_KEY_COMMENT "com.apple.quicktime.comment"
#define RECORD_DEMUXER_META_KEY_COPYRIGHT "com.apple.quicktime.copyright"
#define RECORD_DEMUXER_META_KEY_MAKER "com.apple.quicktime.make"
#define RECORD_DEMUXER_META_KEY_MODEL "com.apple.quicktime.model"
#define RECORD_DEMUXER_META_KEY_VERSION "com.apple.quicktime.software"
#define RECORD_DEMUXER_META_KEY_COVER "com.apple.quicktime.artwork"
#define RECORD_DEMUXER_META_KEY_SERIAL "com.parrot.serial"
#define RECORD_DEMUXER_META_KEY_MODEL_ID "com.parrot.model.id"
#define RECORD_DEMUXER_META_KEY_BUILD_ID "com.parrot.build.id"
#define RECORD_DEMUXER_META_KEY_RUN_ID "com.parrot.run.id"
#define RECORD_DEMUXER_META_KEY_RUN_DATE "com.parrot.run.date"
#define RECORD_DEMUXER_META_KEY_PICTURE_HFOV "com.parrot.picture.hfov"
#define RECORD_DEMUXER_META_KEY_PICTURE_VFOV "com.parrot.picture.vfov"

#define RECORD_DEMUXER_JSON_KEY_VERSION "software_version"
#define RECORD_DEMUXER_JSON_KEY_RUN_ID "run_uuid"
#define RECORD_DEMUXER_JSON_KEY_LOCATION "takeoff_position"
#define RECORD_DEMUXER_JSON_KEY_DATE "media_date"
#define RECORD_DEMUXER_JSON_KEY_PICTURE_HFOV "picture_hfov"
#define RECORD_DEMUXER_JSON_KEY_PICTURE_VFOV "picture_vfov"


namespace Pdraw
{


RecordDemuxer::RecordDemuxer(Session *session)
{
    mSession = session;
    mConfigured = false;
    mDemux = NULL;
    mRunning = false;
    mFrameByFrame = false;
    mDemuxerThreadLaunched = false;
    mThreadShouldStop = false;
    mVideoTrackCount = 0;
    mVideoTrackId = 0;
    mMetadataMimeType = NULL;
    mFirstFrame = true;
    mDecoder = NULL;
    mDecoderBitstreamFormat = AVCDECODER_BITSTREAM_FORMAT_UNKNOWN;
    mLastFrameOutputTime = 0;
    mLastFrameTimestamp = 0;
    mDuration = 0;
    mCurrentTime = 0;
    mPendingSeekTs = -1;
    mPendingSeekExact = false;
    mPendingSeekToPrevSample = false;
    mCurrentBuffer = NULL;
    mWidth = mHeight = 0;
    mCropLeft = mCropRight = mCropTop = mCropBottom = 0;
    mSarWidth = mSarHeight = 0;
    mHfov = mVfov = 0.;

    mMetadataBufferSize = 1024;
    mMetadataBuffer = (uint8_t*)malloc(mMetadataBufferSize);
    if (mMetadataBuffer == NULL)
    {
        ULOGE("RecordDemuxer: allocation failed size %d", mMetadataBufferSize);
    }

    int ret = pthread_mutex_init(&mDemuxerMutex, NULL);
    if (ret != 0)
    {
        ULOGE("RecordDemuxer: mutex creation failed (%d)", ret);
    }

    struct h264_ctx_cbs h264_cbs;
    memset(&h264_cbs, 0, sizeof(h264_cbs));
    h264_cbs.userdata = this;
    h264_cbs.sei_user_data_unregistered = &h264UserDataSeiCb;
    ret = h264_reader_new(&h264_cbs, &mH264Reader);
    if (ret < 0)
    {
        ULOGE("RecordDemuxer: h264_reader_new() failed (%d)", ret);
    }
}


RecordDemuxer::~RecordDemuxer()
{
    int ret = stop();
    if (ret != 0)
        ULOGE("RecordDemuxer: stop() failed (%d)", ret);

    if (mDemuxerThreadLaunched)
    {
        int thErr = pthread_join(mDemuxerThread, NULL);
        if (thErr != 0)
            ULOGE("RecordDemuxer: pthread_join() failed (%d)", thErr);
    }

    pthread_mutex_destroy(&mDemuxerMutex);

    if (mCurrentBuffer)
        vbuf_unref(&mCurrentBuffer);

    if (mDemux)
        mp4_demux_close(mDemux);
    if (mH264Reader)
        h264_reader_destroy(mH264Reader);

    free(mMetadataBuffer);
    free(mMetadataMimeType);
}


int RecordDemuxer::fetchVideoDimensions()
{
    uint8_t *sps = NULL, *pps = NULL;
    unsigned int spsSize = 0, ppsSize = 0;
    int ret = mp4_demux_get_track_avc_decoder_config(mDemux, mVideoTrackId,
        &sps, &spsSize, &pps, &ppsSize);
    if (ret != 0)
    {
        ULOGE("RecordDemuxer: failed to get decoder configuration (%d)", ret);
    }
    else
    {
        int _ret = pdraw_videoDimensionsFromH264Sps(sps, spsSize,
            &mWidth, &mHeight, &mCropLeft, &mCropRight,
            &mCropTop, &mCropBottom, &mSarWidth, &mSarHeight);
        if (_ret != 0)
        {
            ULOGW("RecordDemuxer: pdraw_videoDimensionsFromH264Sps() failed (%d)", _ret);
        }
    }

    return ret;
}


int RecordDemuxer::fetchSessionMetadata()
{
    if (!mSession)
    {
        ULOGE("RecordDemuxer: invalid session");
        return -1;
    }

    SessionPeerMetadata *peerMeta = mSession->getPeerMetadata();
    unsigned int count = 0, i;
    char **keys = NULL, *key;
    char **values = NULL, *value;
    struct vmeta_session meta;
    memset(&meta, 0, sizeof(meta));

    int ret = mp4_demux_get_metadata_strings(mDemux, &count, &keys, &values);
    if (ret != 0)
    {
        ULOGE("RecordDemuxer: mp4_demux_get_metadata_strings() failed (%d)", ret);
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        key = keys[i];
        value = values[i];
        if ((key) && (value))
        {
            ret = vmeta_session_recording_read(key, value, &meta);
            if (ret != 0)
            {
                ULOGE("RecordDemuxer: vmeta_session_recording_read() failed: %d(%s)\n", ret, strerror(-ret));
                continue;
            }
        }
    }

    peerMeta->set(&meta);
    if (meta.picture_fov.has_horz)
        mHfov = meta.picture_fov.horz;
    if (meta.picture_fov.has_vert)
        mVfov = meta.picture_fov.vert;

    return 0;
}


int RecordDemuxer::configure(const std::string &url)
{
    int ret = 0;

    if (mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is already configured");
        return -1;
    }

    mFileName = url;

    if (ret == 0)
    {
        mDemux = mp4_demux_open(mFileName.c_str());
        if (mDemux == NULL)
        {
            ULOGE("RecordDemuxer: mp4_demux_open() failed");
            ret = -1;
        }

        int i, tkCount = 0, found = 0;
        struct mp4_media_info info;
        struct mp4_track_info tk;

        ret = mp4_demux_get_media_info(mDemux, &info);
        if (ret == 0)
        {
            mDuration = info.duration;
            tkCount = info.track_count;
            ULOGI("RecordDemuxer: track count: %d", tkCount);
            unsigned int hrs = 0, min = 0, sec = 0;
            pdraw_friendlyTimeFromUs(info.duration, &hrs, &min, &sec, NULL);
            ULOGI("RecordDemuxer: duration: %02d:%02d:%02d", hrs, min, sec);
        }

        for (i = 0; i < tkCount; i++)
        {
            ret = mp4_demux_get_track_info(mDemux, i, &tk);
            if ((ret == 0) && (tk.type == MP4_TRACK_TYPE_VIDEO))
            {
                mVideoTrackId = tk.id;
                mVideoTrackCount++;
                if (tk.has_metadata)
                {
                    mMetadataMimeType = strdup(tk.metadata_mime_format);
                }
                found = 1;
                break;
            }
        }

        if (found)
        {
            ULOGI("RecordDemuxer: video track ID: %d", mVideoTrackId);
        }
        else
        {
            ULOGE("RecordDemuxer: failed to find a video track");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ret = fetchVideoDimensions();
    }

    if ((ret == 0) && (mSession))
    {
        ret = fetchSessionMetadata();
    }

    if (ret == 0)
    {
        int thErr = pthread_create(&mDemuxerThread, NULL, runDemuxerThread, (void*)this);
        if (thErr != 0)
        {
            ULOGE("RecordDemuxer: demuxer thread creation failed (%d)", thErr);
        }
        else
        {
            mDemuxerThreadLaunched = true;
        }
    }

    mConfigured = (ret == 0) ? true : false;
    if (mConfigured)
    {
        ULOGI("RecordDemuxer: demuxer is configured");        
    }

    return ret;
}


int RecordDemuxer::getElementaryStreamCount()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    //TODO: handle multiple streams
    return mVideoTrackCount;
}


elementary_stream_type_t RecordDemuxer::getElementaryStreamType(int esIndex)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return (elementary_stream_type_t)-1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return (elementary_stream_type_t)-1;
    }

    //TODO: handle multiple streams
    return ELEMENTARY_STREAM_TYPE_VIDEO_AVC;
}


int RecordDemuxer::getElementaryStreamVideoDimensions(int esIndex,
    unsigned int *width, unsigned int *height,
    unsigned int *cropLeft, unsigned int *cropRight,
    unsigned int *cropTop, unsigned int *cropBottom,
    unsigned int *sarWidth, unsigned int *sarHeight)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    if (width)
        *width = mWidth;
    if (height)
        *height = mHeight;
    if (cropLeft)
        *cropLeft = mCropLeft;
    if (cropRight)
        *cropRight = mCropRight;
    if (cropTop)
        *cropTop = mCropTop;
    if (cropBottom)
        *cropBottom = mCropBottom;
    if (sarWidth)
        *sarWidth = mSarWidth;
    if (sarHeight)
        *sarHeight = mSarHeight;

    return 0;
}


int RecordDemuxer::getElementaryStreamVideoFov(int esIndex, float *hfov, float *vfov)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    if (hfov)
        *hfov = mHfov;
    if (vfov)
        *vfov = mVfov;

    return 0;
}


int RecordDemuxer::setElementaryStreamDecoder(int esIndex, Decoder *decoder)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }
    if (!decoder)
    {
        ULOGE("RecordDemuxer: invalid decoder");
        return -1;
    }
    if ((esIndex < 0) || (esIndex >= mVideoTrackCount))
    {
        ULOGE("RecordDemuxer: invalid ES index");
        return -1;
    }

    //TODO: handle multiple streams
    mDecoder = (AvcDecoder*)decoder;
    uint32_t formatCaps = mDecoder->getInputBitstreamFormatCaps();
    if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM)
        mDecoderBitstreamFormat = AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM;
    else if (formatCaps & AVCDECODER_BITSTREAM_FORMAT_AVCC)
        mDecoderBitstreamFormat = AVCDECODER_BITSTREAM_FORMAT_AVCC;
    else
    {
        ULOGE("RecordDemuxer: unsupported decoder input bitstream format");
        return -1;
    }

    return 0;
}


int RecordDemuxer::play(float speed)
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    if (speed > 0.)
    {
        mRunning = true;
        mFrameByFrame = false;
        mPendingSeekToPrevSample = false;
        mSpeed = speed;
    }
    else
    {
        /* speed is null or negative => pause */
        mRunning = false;
        mFrameByFrame = true;
    }

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::pause()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    mRunning = false;
    mFrameByFrame = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


bool RecordDemuxer::isPaused()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return false;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    bool running = mRunning && !mFrameByFrame;

    pthread_mutex_unlock(&mDemuxerMutex);

    return !running;
}


int RecordDemuxer::previous()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    if (!mPendingSeekExact)
    {
        /* Avoid seeking back to much if a seek to a
         * previous frame is already in progress */
        mPendingSeekToPrevSample = true;
        mPendingSeekExact = true;
        mRunning = true;
    }

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::next()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    mRunning = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::stop()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    pthread_mutex_lock(&mDemuxerMutex);

    mRunning = false;
    mThreadShouldStop = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp, bool exact)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    if (timestamp > mDuration) timestamp = mDuration;
    mPendingSeekTs = (int64_t)timestamp;
    mPendingSeekExact = exact;
    mPendingSeekToPrevSample = false;
    mRunning = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekForward(uint64_t delta, bool exact)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    int64_t ts = (int64_t)mLastFrameTimestamp + (int64_t)delta;
    if (ts < 0) ts = 0;
    if (ts > (int64_t)mDuration) ts = mDuration;
    mPendingSeekTs = ts;
    mPendingSeekExact = exact;
    mPendingSeekToPrevSample = false;
    mRunning = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekBack(uint64_t delta, bool exact)
{
    pthread_mutex_lock(&mDemuxerMutex);

    if (!mConfigured)
    {
        pthread_mutex_unlock(&mDemuxerMutex);
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    int64_t ts = (int64_t)mLastFrameTimestamp - (int64_t)delta;
    if (ts < 0) ts = 0;
    if (ts > (int64_t)mDuration) ts = mDuration;
    mPendingSeekTs = ts;
    mPendingSeekExact = exact;
    mPendingSeekToPrevSample = false;
    mRunning = true;

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


void RecordDemuxer::h264UserDataSeiCb(struct h264_ctx *ctx, const uint8_t *buf, size_t len,
                                      const struct h264_sei_user_data_unregistered *sei, void *userdata)
{
    RecordDemuxer *demuxer = (RecordDemuxer*)userdata;
    int ret = 0;

    if (!demuxer)
        return;
    if ((!buf) || (len == 0))
        return;
    if (!demuxer->mCurrentBuffer)
        return;

    /* ignore "Parrot Streaming" v1 and v2 user data SEI */
    if ((vstrm_h264_sei_streaming_is_v1(sei->uuid)) || (vstrm_h264_sei_streaming_is_v2(sei->uuid)))
    {
        ULOGI("RecordDemuxer: Parrot Streaming user data SEI => ignored");
        return;
    }

    ret = vbuf_set_userdata_capacity(demuxer->mCurrentBuffer, len);
    if (ret < (signed)len)
    {
        ULOGE("RecordDemuxer: failed to realloc user data buffer");
        return;
    }

    uint8_t *dstBuf = vbuf_get_userdata_ptr(demuxer->mCurrentBuffer);
    memcpy(dstBuf, buf, len);
    vbuf_set_userdata_size(demuxer->mCurrentBuffer, len);
}


void* RecordDemuxer::runDemuxerThread(void *ptr)
{
    RecordDemuxer *demuxer = (RecordDemuxer*)ptr;
    struct timespec t1;
    uint64_t curTime;
    int32_t outputTimeError = 0;

    while (!demuxer->mThreadShouldStop)
    {
        if ((demuxer->mDecoder) && (demuxer->mRunning))
        {
            pthread_mutex_lock(&demuxer->mDemuxerMutex);
            bool running = demuxer->mRunning;
            float speed = demuxer->mSpeed;
            pthread_mutex_unlock(&demuxer->mDemuxerMutex);

            if (!running)
            {
                usleep(5000); //TODO
                continue;
            }

            uint8_t *sps = NULL, *pps = NULL;
            uint8_t *spsBuffer = NULL, *ppsBuffer = NULL;
            unsigned int spsSize = 0, ppsSize = 0;
            struct mp4_track_sample sample;
            int ret;

            if (demuxer->mFirstFrame)
            {
                ret = mp4_demux_get_track_avc_decoder_config(demuxer->mDemux, demuxer->mVideoTrackId,
                                                              &sps, &spsSize, &pps, &ppsSize);
                if (ret != 0)
                {
                    ULOGE("RecordDemuxer: failed to get decoder configuration (%d)", ret);
                }
                else if ((sps) && (spsSize > 0) && (pps) && (ppsSize > 0))
                {
                    ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0, sps, spsSize);
                    if (ret < 0)
                    {
                        ULOGW("RecordDemuxer: h264_reader_parse_nalu() failed (%d)", ret);
                    }
                    ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0, pps, ppsSize);
                    if (ret < 0)
                    {
                        ULOGW("RecordDemuxer: h264_reader_parse_nalu() failed (%d)", ret);
                    }
                    spsBuffer = (uint8_t *)malloc(spsSize + 4);
                    if (spsBuffer)
                    {
                        uint32_t start = (demuxer->mDecoderBitstreamFormat == AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
                            htonl(0x00000001) : htonl(spsSize);
                        *((uint32_t*)spsBuffer) = start;
                        memcpy(spsBuffer + 4, sps, spsSize);
                    }
                    else
                    {
                        ULOGE("RecordDemuxer: SPS buffer allocation failed");
                    }
                    ppsBuffer = (uint8_t *)malloc(ppsSize + 4);
                    if (ppsBuffer)
                    {
                        uint32_t start = (demuxer->mDecoderBitstreamFormat == AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) ?
                            htonl(0x00000001) : htonl(ppsSize);
                        *((uint32_t*)ppsBuffer) = start;
                        memcpy(ppsBuffer + 4, pps, ppsSize);
                    }
                    else
                    {
                        ULOGE("RecordDemuxer: PPS buffer allocation failed");
                    }
                    if (spsBuffer && ppsBuffer)
                    {
                        ret = demuxer->mDecoder->configure(demuxer->mDecoderBitstreamFormat, spsBuffer, (unsigned int)spsSize + 4, ppsBuffer, (unsigned int)ppsSize + 4);
                        if (ret != 0)
                        {
                            ULOGE("RecordDemuxer: decoder configuration failed (%d)", ret);
                        }
                    }
                }
            }

            if ((demuxer->mDecoder->isConfigured()) && (demuxer->mCurrentBuffer == NULL))
            {
                ret = demuxer->mDecoder->getInputBuffer(&demuxer->mCurrentBuffer, true);
                if (ret != 0)
                {
                    ULOGW("RecordDemuxer: failed to get an output buffer (%d)", ret);
                }
            }

            if (demuxer->mCurrentBuffer)
            {
                uint8_t *buf = vbuf_get_ptr(demuxer->mCurrentBuffer);
                unsigned int bufSize = vbuf_get_capacity(demuxer->mCurrentBuffer);
                unsigned int outSize = 0;

                if (demuxer->mFirstFrame)
                {
                    if ((spsBuffer) && (spsSize + 4 <= bufSize))
                    {
                        memcpy(buf, spsBuffer, spsSize + 4);
                        buf += (spsSize + 4);
                        bufSize -= (spsSize + 4);
                        outSize += (spsSize + 4);
                    }
                    if ((ppsBuffer) && (ppsSize + 4 <= bufSize))
                    {
                        memcpy(buf, ppsBuffer, ppsSize + 4);
                        buf += (ppsSize + 4);
                        bufSize -= (ppsSize + 4);
                        outSize += (ppsSize + 4);
                    }
                    demuxer->mFirstFrame = false;
                }

                pthread_mutex_lock(&demuxer->mDemuxerMutex);
                int64_t seekTs = demuxer->mPendingSeekTs;
                bool pendingSeekExact = demuxer->mPendingSeekExact;
                bool pendingSeekToPrevSample = demuxer->mPendingSeekToPrevSample;
                demuxer->mPendingSeekTs = -1;
                demuxer->mPendingSeekToPrevSample = false;
                pthread_mutex_unlock(&demuxer->mDemuxerMutex);

                if (seekTs >= 0)
                {
                    ret = mp4_demux_seek(demuxer->mDemux, (uint64_t)seekTs, 1);
                    if (ret != 0)
                    {
                        ULOGW("RecordDemuxer: mp4_demux_seek() failed (%d)", ret);
                    }
                    else
                    {
                        demuxer->mLastFrameTimestamp = 0;
                        outputTimeError = 0;
                    }
                }
                else if (pendingSeekToPrevSample)
                {
                    ret = mp4_demux_seek_to_track_prev_sample(demuxer->mDemux, demuxer->mVideoTrackId);
                    if (ret != 0)
                    {
                        ULOGW("RecordDemuxer: mp4_demux_seek_to_track_prev_sample() failed (%d)", ret);
                    }
                    else
                    {
                        demuxer->mLastFrameTimestamp = 0;
                        outputTimeError = 0;
                    }
                }

                ret = mp4_demux_get_track_next_sample(demuxer->mDemux, demuxer->mVideoTrackId,
                                                      buf, bufSize, demuxer->mMetadataBuffer, demuxer->mMetadataBufferSize, &sample);
                if ((ret == 0) && (sample.sample_size))
                {
                    vbuf_set_size(demuxer->mCurrentBuffer, outSize + sample.sample_size);
                    vbuf_set_userdata_size(demuxer->mCurrentBuffer, 0);

                    bool silent = ((sample.silent) && (pendingSeekExact)) ? true : false;
                    demuxer->mPendingSeekExact = (silent) ? pendingSeekExact : false;

                    /* Fix the H.264 bitstream: replace NALU size by byte stream start codes */
                    uint32_t offset = 0, naluSize, naluCount = 0;
                    uint8_t *_buf = buf;
                    uint8_t *seiNalu = NULL;
                    int seiNaluSize = 0;
                    while (offset < sample.sample_size)
                    {
                        naluSize = ntohl(*((uint32_t*)_buf));
                        if (demuxer->mDecoderBitstreamFormat == AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM)
                            *((uint32_t*)_buf) = htonl(0x00000001);
                        if (*(_buf + 4) == 0x06)
                        {
                            seiNalu = _buf + 4;
                            seiNaluSize = naluSize;
                        }
                        _buf += 4 + naluSize;
                        offset += 4 + naluSize;
                        naluCount++;
                    }

                    if ((seiNalu) && (seiNaluSize))
                    {
                        ret = h264_reader_parse_nalu(demuxer->mH264Reader, 0, seiNalu, seiNaluSize);
                        if (ret < 0)
                        {
                            ULOGW("RecordDemuxer: h264_reader_parse_nalu() failed (%d)", ret);
                        }
                    }

                    struct avcdecoder_input_buffer *data = (struct avcdecoder_input_buffer*)vbuf_get_metadata_ptr(demuxer->mCurrentBuffer);
                    memset(data, 0, sizeof(*data));
                    data->isComplete = true; //TODO?
                    data->hasErrors = false; //TODO?
                    data->isRef = true; //TODO?
                    data->isSilent = silent;
                    data->auNtpTimestamp = sample.sample_dts;
                    data->auNtpTimestampRaw = sample.sample_dts;
                    //TODO: auSyncType

                    /* Metadata */
                    data->hasMetadata = VideoFrameMetadata::decodeMetadata(demuxer->mMetadataBuffer, sample.metadata_size,
                        FRAME_METADATA_SOURCE_RECORDING, demuxer->mMetadataMimeType, &data->metadata);

                    if ((demuxer->mLastFrameOutputTime) && (demuxer->mLastFrameTimestamp))
                    {
                        clock_gettime(CLOCK_MONOTONIC, &t1);
                        curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                        int32_t sleepTime = 0;

                        if ((speed > 0.) && (speed <= PDRAW_PLAY_SPEED_MAX) && (!silent))
                        {
                            sleepTime = (int32_t)((int64_t)((sample.sample_dts - demuxer->mLastFrameTimestamp) / speed) -
                                (int64_t)(curTime - demuxer->mLastFrameOutputTime)) + outputTimeError;
                        }
                        if (sleepTime >= 1000)
                        {
                            usleep(sleepTime);
                        }
                    }

                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    data->demuxOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                    data->auNtpTimestampLocal = data->demuxOutputTimestamp;
                    outputTimeError = ((demuxer->mLastFrameOutputTime) && (demuxer->mLastFrameTimestamp) && (speed > 0.) && (speed <= PDRAW_PLAY_SPEED_MAX) && (!silent)) ?
                                        (int32_t)((int64_t)((sample.sample_dts - demuxer->mLastFrameTimestamp) / speed) -
                                        (int64_t)(data->demuxOutputTimestamp - demuxer->mLastFrameOutputTime)) : 0;

                    ret = demuxer->mDecoder->queueInputBuffer(demuxer->mCurrentBuffer);
                    if (ret != 0)
                    {
                        ULOGW("RecordDemuxer: failed to release the output buffer (%d)", ret);
                    }
                    else
                    {
                        demuxer->mLastFrameOutputTime = data->demuxOutputTimestamp;
                        demuxer->mLastFrameTimestamp = sample.sample_dts;
                        demuxer->mCurrentTime = sample.sample_dts;
                        vbuf_unref(&demuxer->mCurrentBuffer);
                        demuxer->mCurrentBuffer = NULL;

                        pthread_mutex_lock(&demuxer->mDemuxerMutex);
                        if ((demuxer->mFrameByFrame) && (!silent))
                        {
                            demuxer->mRunning = false;
                        }
                        pthread_mutex_unlock(&demuxer->mDemuxerMutex);
                    }
                }
                else if (ret != 0)
                {
                    ULOGW("RecordDemuxer: failed to get sample (%d)", ret);
                    if (ret == -ENOBUFS)
                    {
                        /* Go to the next sample */
                        ret = mp4_demux_get_track_next_sample(demuxer->mDemux, demuxer->mVideoTrackId,
                                                          NULL, 0, NULL, 0, &sample);
                    }
                }
                free(spsBuffer);
                free(ppsBuffer);
            }
            else
            {
                usleep(5000); //TODO
            }
        }
        else
        {
            usleep(5000); //TODO
        }
    }

    return NULL;
}

}
