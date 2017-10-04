/**
 * @file pdraw_metadata_session.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - session metadata
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

#ifndef _PDRAW_METADATA_SESSION_HPP_
#define _PDRAW_METADATA_SESSION_HPP_

#include <inttypes.h>
#include <string>
#include <vector>

#include <video-metadata/vmeta.h>

#include "pdraw_utils.hpp"


namespace Pdraw
{


class SessionSelfMetadata
{
public:

    SessionSelfMetadata();

    ~SessionSelfMetadata();

    std::string& getFriendlyName(void) { return mFriendlyName; } ;
    void setFriendlyName(const std::string& friendlyName) { mFriendlyName = friendlyName; };

    std::string& getSerialNumber(void) { return mSerialNumber; };
    void setSerialNumber(const std::string& serialNumber) { mSerialNumber = serialNumber; };

    std::string& getSoftwareVersion(void) { return mSoftwareVersion; };
    void setSoftwareVersion(const std::string& softwareVersion) { mSoftwareVersion = softwareVersion; };

    bool isPilot() { return mIsPilot; };
    void setPilot(bool isPilot) { mIsPilot = isPilot; };

    void getLocation(struct pdraw_location *loc);
    void setLocation(const struct pdraw_location *loc);

    int getControllerBatteryLevel() { return mControllerBatteryLevel; };
    void setControllerBatteryLevel(int batteryLevel) { mControllerBatteryLevel = batteryLevel; };

    bool getControllerOrientation(struct pdraw_quaternion *quat);
    bool getControllerOrientation(struct pdraw_euler *euler);

    void setControllerOrientation(const struct pdraw_quaternion *quat);
    void setControllerOrientation(const struct pdraw_euler *euler);

    bool getHeadOrientation(struct pdraw_quaternion *quat);
    bool getHeadOrientation(struct pdraw_euler *euler);
    void setHeadOrientation(const struct pdraw_quaternion *quat);
    void setHeadOrientation(const struct pdraw_euler *euler);

    bool getHeadRefOrientation(struct pdraw_quaternion *quat);
    bool getHeadRefOrientation(struct pdraw_euler *euler);
    void setHeadRefOrientation(const struct pdraw_quaternion *quat);
    void setHeadRefOrientation(const struct pdraw_euler *euler);
    void resetHeadRefOrientation();

private:

    std::string mFriendlyName;
    std::string mSerialNumber;
    std::string mSoftwareVersion;
    bool mIsPilot;
    struct pdraw_location mLocation;
    int mControllerBatteryLevel;
    struct pdraw_quaternion mControllerQuat;
    bool mIsControllerValid;
    struct pdraw_quaternion mHeadQuat;
    bool mIsHeadValid;
    struct pdraw_quaternion mHeadRefQuat;
    bool mIsHeadRefValid;
};


class SessionPeerMetadata
{
public:

    SessionPeerMetadata();

    ~SessionPeerMetadata();

    void set(struct vmeta_session *meta);

    std::string& getFriendlyName(void) { return mFriendlyName; } ;
    void setFriendlyName(const std::string& friendlyName);

    std::string& getMaker(void) { return mMaker; };
    void setMaker(const std::string& maker) { mMaker = maker; };

    std::string& getModel(void) { return mModel; };
    void setModel(const std::string& model);

    std::string& getModelId(void) { return mModelId; };
    void setModelId(const std::string& modelId);

    enum pdraw_drone_model getDroneModel(void) { return mDroneModel; };

    std::string& getSerialNumber(void) { return mSerialNumber; };
    void setSerialNumber(const std::string& serialNumber) { mSerialNumber = serialNumber; };

    std::string& getSoftwareVersion(void) { return mSoftwareVersion; };
    void setSoftwareVersion(const std::string& softwareVersion) { mSoftwareVersion = softwareVersion; };

    std::string& getBuildId(void) { return mBuildId; };
    void setBuildId(const std::string& buildId) { mBuildId = buildId; };

    std::string& getTitle(void) { return mTitle; };
    void setTitle(const std::string& title) { mTitle = title; };

    std::string& getComment(void) { return mComment; };
    void setComment(const std::string& comment) { mComment = comment; };

    std::string& getCopyright(void) { return mCopyright; };
    void setCopyright(const std::string& copyright) { mCopyright = copyright; };

    std::string& getRunDate(void) { return mRunDate; };
    void setRunDate(const std::string& runDate) { mRunDate = runDate; };

    std::string& getRunUuid(void) { return mRunUuid; };
    void setRunUuid(const std::string& runUuid) { mRunUuid = runUuid; };

    std::string& getMediaDate(void) { return mMediaDate; };
    void setMediaDate(const std::string& mediaDate) { mMediaDate = mediaDate; };

    void getTakeoffLocation(struct pdraw_location *loc);
    void setTakeoffLocation(const struct pdraw_location *loc);

    void getHomeLocation(struct pdraw_location *loc);
    void setHomeLocation(const struct pdraw_location *loc);

    uint64_t getRecordingDuration(void);
    void setRecordingDuration(uint64_t duration);

private:

    std::string mFriendlyName;
    std::string mMaker;
    std::string mModel;
    std::string mModelId;
    enum pdraw_drone_model mDroneModel;
    std::string mSerialNumber;
    std::string mSoftwareVersion;
    std::string mBuildId;
    std::string mTitle;
    std::string mComment;
    std::string mCopyright;
    std::string mRunDate;
    std::string mRunUuid;
    std::string mMediaDate;
    struct pdraw_location mTakeoffLocation;
    struct pdraw_location mHomeLocation;
    uint64_t mRecordingStartTime;
};

}

#endif /* !_PDRAW_METADATA_SESSION_HPP_ */
