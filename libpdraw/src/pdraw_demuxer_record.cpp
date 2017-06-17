/**
 * @file pdraw_demuxer_record.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - recording demuxer
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

#include "pdraw_demuxer_record.hpp"
#include "pdraw_session.hpp"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <json-c/json.h>

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
    mDemuxerThreadLaunched = false;
    mThreadShouldStop = false;
    mVideoTrackCount = 0;
    mVideoTrackId = 0;
    mMetadataMimeType = NULL;
    mFirstFrame = true;
    mDecoder = NULL;
    mLastFrameOutputTime = 0;
    mLastFrameTimestamp = 0;
    mDuration = 0;
    mCurrentTime = 0;
    mPendingSeekTs = -1;
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
}


RecordDemuxer::~RecordDemuxer()
{
    mThreadShouldStop = true;

    if (mDemuxerThreadLaunched)
    {
        int thErr = pthread_join(mDemuxerThread, NULL);
        if (thErr != 0)
            ULOGE("RecordDemuxer: pthread_join() failed (%d)", thErr);
    }

    pthread_mutex_destroy(&mDemuxerMutex);

    if (mCurrentBuffer)
        mCurrentBuffer->unref();

    if (mDemux)
        mp4_demux_close(mDemux);

    free(mMetadataBuffer);
    free(mMetadataMimeType);
}


int RecordDemuxer::fetchVideoDimensions()
{
    uint8_t *spsBuffer = NULL, *ppsBuffer = NULL;
    unsigned int spsSize = 0, ppsSize = 0;
    int ret = mp4_demux_get_track_avc_decoder_config(mDemux, mVideoTrackId,
        &spsBuffer, &spsSize, &ppsBuffer, &ppsSize);
    if (ret != 0)
    {
        ULOGE("RecordDemuxer: failed to get decoder configuration (%d)", ret);
    }
    else
    {
        int _ret = pdraw_videoDimensionsFromH264Sps(spsBuffer, spsSize,
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

    int ret = mp4_demux_get_metadata_strings(mDemux, &count, &keys, &values);
    if ((ret != 0) || (count <= 0))
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
            location_t takeoffLoc;
            peerMeta->getTakeoffLocation(&takeoffLoc);
            if ((peerMeta->getFriendlyName().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_ARTIST, strlen(RECORD_DEMUXER_UDTA_KEY_ARTIST))))
                peerMeta->setFriendlyName(value);
            else if ((peerMeta->getTitle().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_TITLE, strlen(RECORD_DEMUXER_UDTA_KEY_TITLE))))
                peerMeta->setTitle(value);
            else if ((peerMeta->getMediaDate().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_DATE, strlen(RECORD_DEMUXER_UDTA_KEY_DATE))))
                peerMeta->setMediaDate(value);
            else if ((!strncmp(key, RECORD_DEMUXER_UDTA_KEY_COMMENT, strlen(RECORD_DEMUXER_UDTA_KEY_COMMENT))))
            {
                if ((strlen(value) > 0) && (value[0] == '{') && (value[strlen(value) - 1] == '}'))
                {
                    /* parse the JSON string */
                    int error = 0;
                    json_object* jsonObjAll;
                    json_object* jsonObjItem;
                    json_bool jsonRet;
                    if (error == 0)
                    {
                        jsonObjAll = json_tokener_parse(value);
                        if (jsonObjAll == NULL)
                        {
                            error = -1;
                        }
                    }

                    /* software_version */
                    if ((error == 0) && (peerMeta->getSoftwareVersion().empty()))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_VERSION, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            peerMeta->setSoftwareVersion(json_object_get_string(jsonObjItem));
                        }
                    }

                    /* run_uuid */
                    if ((error == 0) && (peerMeta->getRunUuid().empty()))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_RUN_ID, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            peerMeta->setRunUuid(json_object_get_string(jsonObjItem));
                        }
                    }

                    /* takeoff_position */
                    if ((error == 0) && (!takeoffLoc.isValid))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_LOCATION, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            const char *pszLoc = json_object_get_string(jsonObjItem);
                            memset(&takeoffLoc, 0, sizeof(takeoffLoc));
                            sscanf(pszLoc, "%lf,%lf,%lf", &takeoffLoc.latitude, &takeoffLoc.longitude, &takeoffLoc.altitude);
                            if ((takeoffLoc.latitude != 500.) && (takeoffLoc.longitude != 500.))
                            {
                                takeoffLoc.isValid = 1;
                                peerMeta->setTakeoffLocation(&takeoffLoc);
                            }
                        }
                    }

                    /* media_date */
                    if ((error == 0) && (peerMeta->getMediaDate().empty()))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_DATE, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            peerMeta->setMediaDate(json_object_get_string(jsonObjItem));
                        }
                    }

                    /* picture_hfov */
                    if ((error == 0) && (mHfov == 0.))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_PICTURE_HFOV, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            mHfov = json_object_get_double(jsonObjItem);
                        }
                    }

                    /* picture_vfov */
                    if ((error == 0) && (mVfov == 0.))
                    {
                        jsonRet = json_object_object_get_ex(jsonObjAll, RECORD_DEMUXER_JSON_KEY_PICTURE_VFOV, &jsonObjItem);
                        if ((jsonRet) && (jsonObjItem != NULL))
                        {
                            mVfov = json_object_get_double(jsonObjItem);
                        }
                    }
                }
                else if (peerMeta->getComment().empty())
                {
                    peerMeta->setComment(value);
                }
            }
            else if ((peerMeta->getCopyright().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_COPYRIGHT, strlen(RECORD_DEMUXER_UDTA_KEY_COPYRIGHT))))
                peerMeta->setCopyright(value);
            else if ((peerMeta->getMaker().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_MAKER, strlen(RECORD_DEMUXER_UDTA_KEY_MAKER))))
                peerMeta->setMaker(value);
            else if ((peerMeta->getModel().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_MODEL, strlen(RECORD_DEMUXER_UDTA_KEY_MODEL))))
                peerMeta->setModel(value);
            else if ((peerMeta->getSoftwareVersion().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_VERSION, strlen(RECORD_DEMUXER_UDTA_KEY_VERSION))))
                peerMeta->setSoftwareVersion(value);
            else if ((peerMeta->getSerialNumber().empty()) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_SERIAL, strlen(RECORD_DEMUXER_UDTA_KEY_SERIAL))))
                peerMeta->setSerialNumber(value);
            else if ((!takeoffLoc.isValid) && (!strncmp(key, RECORD_DEMUXER_UDTA_KEY_LOCATION, strlen(RECORD_DEMUXER_UDTA_KEY_LOCATION))))
            {
                pdraw_parseLocationString(value, &takeoffLoc);
                if (takeoffLoc.isValid)
                {
                    peerMeta->setTakeoffLocation(&takeoffLoc);
                }
            }
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_ARTIST, strlen(RECORD_DEMUXER_META_KEY_ARTIST)))
                peerMeta->setFriendlyName(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_TITLE, strlen(RECORD_DEMUXER_META_KEY_TITLE)))
                peerMeta->setTitle(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_DATE, strlen(RECORD_DEMUXER_META_KEY_DATE)))
                peerMeta->setMediaDate(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_COMMENT, strlen(RECORD_DEMUXER_META_KEY_COMMENT)))
                peerMeta->setComment(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_COPYRIGHT, strlen(RECORD_DEMUXER_META_KEY_COPYRIGHT)))
                peerMeta->setCopyright(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_MAKER, strlen(RECORD_DEMUXER_META_KEY_MAKER)))
                peerMeta->setMaker(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_MODEL, strlen(RECORD_DEMUXER_META_KEY_MODEL)))
                peerMeta->setModel(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_VERSION, strlen(RECORD_DEMUXER_META_KEY_VERSION)))
                peerMeta->setSoftwareVersion(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_SERIAL, strlen(RECORD_DEMUXER_META_KEY_SERIAL)))
                peerMeta->setSerialNumber(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_MODEL_ID, strlen(RECORD_DEMUXER_META_KEY_MODEL_ID)))
                peerMeta->setModelId(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_BUILD_ID, strlen(RECORD_DEMUXER_META_KEY_BUILD_ID)))
                peerMeta->setBuildId(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_RUN_ID, strlen(RECORD_DEMUXER_META_KEY_RUN_ID)))
                peerMeta->setRunUuid(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_RUN_DATE, strlen(RECORD_DEMUXER_META_KEY_RUN_DATE)))
                peerMeta->setRunDate(value);
            else if ((!strncmp(key, RECORD_DEMUXER_META_KEY_LOCATION, strlen(RECORD_DEMUXER_META_KEY_LOCATION))))
            {
                pdraw_parseLocationString(value, &takeoffLoc);
                if (takeoffLoc.isValid)
                {
                    peerMeta->setTakeoffLocation(&takeoffLoc);
                }
            }
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_PICTURE_HFOV, strlen(RECORD_DEMUXER_META_KEY_PICTURE_HFOV)))
                mHfov = atof(value);
            else if (!strncmp(key, RECORD_DEMUXER_META_KEY_PICTURE_VFOV, strlen(RECORD_DEMUXER_META_KEY_PICTURE_VFOV)))
                mVfov = atof(value);
        }
    }

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
        mp4_media_info_t info;
        mp4_track_info_t tk;

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

    return 0;
}


int RecordDemuxer::start()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mRunning = true;

    return 0;
}


int RecordDemuxer::pause()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mRunning = false;

    return 0;
}


int RecordDemuxer::stop()
{
    if (!mConfigured)
    {
        ULOGE("RecordDemuxer: demuxer is not configured");
        return -1;
    }

    mThreadShouldStop = true;

    return 0;
}


int RecordDemuxer::seekTo(uint64_t timestamp)
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

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekForward(uint64_t delta)
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

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
}


int RecordDemuxer::seekBack(uint64_t delta)
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

    pthread_mutex_unlock(&mDemuxerMutex);

    return 0;
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
            uint8_t *spsBuffer = NULL, *ppsBuffer = NULL;
            unsigned int spsSize = 0, ppsSize = 0;
            mp4_track_sample_t sample;
            int ret;

            if (demuxer->mFirstFrame)
            {
                ret = mp4_demux_get_track_avc_decoder_config(demuxer->mDemux, demuxer->mVideoTrackId,
                                                              &spsBuffer, &spsSize, &ppsBuffer, &ppsSize);
                if (ret != 0)
                {
                    ULOGE("RecordDemuxer: failed to get decoder configuration (%d)", ret);
                }
                else
                {
                    ret = demuxer->mDecoder->configure(spsBuffer, (unsigned int)spsSize, ppsBuffer, (unsigned int)ppsSize);
                    if (ret != 0)
                    {
                        ULOGE("RecordDemuxer: decoder configuration failed (%d)", ret);
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
                uint8_t *buf = (uint8_t*)demuxer->mCurrentBuffer->getPtr();
                unsigned int bufSize = demuxer->mCurrentBuffer->getCapacity();

                if (demuxer->mFirstFrame)
                {
                    if ((spsBuffer) && (spsSize + 4 <= bufSize))
                    {
                        *((uint32_t*)buf) = htonl(0x00000001);
                        memcpy(buf + 4, spsBuffer, spsSize);
                        buf += (spsSize + 4);
                        bufSize -= (spsSize + 4);
                    }
                    if ((ppsBuffer) && (ppsSize + 4 <= bufSize))
                    {
                        *((uint32_t*)buf) = htonl(0x00000001);
                        memcpy(buf + 4, ppsBuffer, ppsSize);
                        buf += (ppsSize + 4);
                        bufSize -= (ppsSize + 4);
                    }
                    demuxer->mFirstFrame = false;
                }

                pthread_mutex_lock(&demuxer->mDemuxerMutex);
                int64_t seekTs = demuxer->mPendingSeekTs;
                demuxer->mPendingSeekTs = -1;
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

                ret = mp4_demux_get_track_next_sample(demuxer->mDemux, demuxer->mVideoTrackId,
                                                      buf, bufSize, demuxer->mMetadataBuffer, demuxer->mMetadataBufferSize, &sample);
                if ((ret == 0) && (sample.sample_size))
                {
                    demuxer->mCurrentBuffer->setSize(sample.sample_size);

                    /* Fix the H.264 bitstream: replace NALU size by byte stream start codes */
                    uint32_t offset = 0, naluSize, naluCount = 0;
                    uint8_t *_buf = buf;
                    while (offset < sample.sample_size)
                    {
                        naluSize = ntohl(*((uint32_t*)_buf));
                        *((uint32_t*)_buf) = htonl(0x00000001);
                        _buf += 4 + naluSize;
                        offset += 4 + naluSize;
                        naluCount++;
                    }

                    avc_decoder_input_buffer_t *data = (avc_decoder_input_buffer_t*)demuxer->mCurrentBuffer->getMetadataPtr();
                    data->isComplete = true; //TODO?
                    data->hasErrors = false; //TODO?
                    data->isRef = true; //TODO?
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
                        int32_t sleepTime = (int32_t)((int64_t)(sample.sample_dts - demuxer->mLastFrameTimestamp) - (int64_t)(curTime - demuxer->mLastFrameOutputTime)) + outputTimeError;
                        if (sleepTime >= 1000)
                        {
                            usleep(sleepTime);
                        }
                    }

                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    data->demuxOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                    data->auNtpTimestampLocal = data->demuxOutputTimestamp;
                    outputTimeError = ((demuxer->mLastFrameOutputTime) && (demuxer->mLastFrameTimestamp)) ?
                                        (int32_t)((int64_t)(sample.sample_dts - demuxer->mLastFrameTimestamp) - (int64_t)(data->demuxOutputTimestamp - demuxer->mLastFrameOutputTime)) : 0;

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
                        demuxer->mCurrentBuffer->unref();
                        demuxer->mCurrentBuffer = NULL;
                    }
                }
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
