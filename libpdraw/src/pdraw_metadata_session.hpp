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

#include "pdraw_utils.hpp"


namespace Pdraw
{


#define drone_model_t pdraw_drone_model_t


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

    void getLocation(location_t *loc);
    void setLocation(const location_t *loc);

    int getControllerBatteryLevel() { return mControllerBatteryLevel; };
    void setControllerBatteryLevel(int batteryLevel) { mControllerBatteryLevel = batteryLevel; };

    bool getControllerOrientation(quaternion_t *quat);
    bool getControllerOrientation(euler_t *euler);

    void setControllerOrientation(const quaternion_t *quat);
    void setControllerOrientation(const euler_t *euler);

    bool getHeadOrientation(quaternion_t *quat);
    bool getHeadOrientation(euler_t *euler);
    void setHeadOrientation(const quaternion_t *quat);
    void setHeadOrientation(const euler_t *euler);

    bool getHeadRefOrientation(quaternion_t *quat);
    bool getHeadRefOrientation(euler_t *euler);
    void setHeadRefOrientation(const quaternion_t *quat);
    void setHeadRefOrientation(const euler_t *euler);
    void resetHeadRefOrientation();

private:

    std::string mFriendlyName;
    std::string mSerialNumber;
    std::string mSoftwareVersion;
    bool mIsPilot;
    location_t mLocation;
    int mControllerBatteryLevel;
    quaternion_t mControllerQuat;
    bool mIsControllerValid;
    quaternion_t mHeadQuat;
    bool mIsHeadValid;
    quaternion_t mHeadRefQuat;
    bool mIsHeadRefValid;
};


class SessionPeerMetadata
{
public:

    SessionPeerMetadata();

    ~SessionPeerMetadata();

    std::string& getFriendlyName(void) { return mFriendlyName; } ;
    void setFriendlyName(const std::string& friendlyName);

    std::string& getMaker(void) { return mMaker; };
    void setMaker(const std::string& maker) { mMaker = maker; };

    std::string& getModel(void) { return mModel; };
    void setModel(const std::string& model);

    std::string& getModelId(void) { return mModelId; };
    void setModelId(const std::string& modelId);

    drone_model_t getDroneModel(void) { return mDroneModel; };

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

    void getTakeoffLocation(location_t *loc);
    void setTakeoffLocation(const location_t *loc);

    void getHomeLocation(location_t *loc);
    void setHomeLocation(const location_t *loc);

    uint64_t getRecordingDuration(void);
    void setRecordingDuration(uint64_t duration);

private:

    std::string mFriendlyName;
    std::string mMaker;
    std::string mModel;
    std::string mModelId;
    drone_model_t mDroneModel;
    std::string mSerialNumber;
    std::string mSoftwareVersion;
    std::string mBuildId;
    std::string mTitle;
    std::string mComment;
    std::string mCopyright;
    std::string mRunDate;
    std::string mRunUuid;
    std::string mMediaDate;
    location_t mTakeoffLocation;
    location_t mHomeLocation;
    uint64_t mRecordingStartTime;
};

}

#endif /* !_PDRAW_METADATA_SESSION_HPP_ */
