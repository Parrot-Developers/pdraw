/**
 * @file pdraw_gles2_hud.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - OpenGL ES 2.0 HUD rendering
 * @date 23/11/2016
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

#include "pdraw_gles2_hud.hpp"

#ifdef USE_GLES2

#define ULOG_TAG libpdraw
#include <ulog.h>
#include <stdio.h>

#include "pdraw_session.hpp"
#include "pdraw_settings.hpp"
#include "pdraw_media_video.hpp"

#include "pdraw_gles2_hud_icons.cpp"
#include "pdraw_gles2_hud_text_profontwindows36.cpp"


namespace Pdraw
{


#define RAD_TO_DEG (57.295779513f)

//#define DEBUG_RADAR // used to test the radar on records


static const GLchar *hudVertexShader =
    "uniform mat4 transform_matrix;\n"
    "attribute vec4 vPosition;\n"
    "void main() {\n"
    "  gl_Position = vPosition * transform_matrix;\n"
    "}\n";

static const GLchar *hudFragmentShader =
#if defined(GL_ES_VERSION_2_0) && defined(ANDROID)
    "precision mediump float;\n"
#endif
    "uniform vec4 vColor;\n"
    "void main() {\n"
    "  gl_FragColor = vColor;\n"
    "}\n";

static const GLchar *hudTexVertexShader =
    "uniform mat4 transform_matrix;\n"
    "attribute vec4 position;\n"
    "attribute vec2 texcoord;\n"
    "varying vec2 v_texcoord;\n"
    "\n"
    "void main()\n"
    "{\n"
    "    gl_Position = position * transform_matrix;\n"
    "    v_texcoord = texcoord;\n"
    "}\n";

static const GLchar *hudTexFragmentShader =
#if defined(GL_ES_VERSION_2_0) && defined(ANDROID)
    "precision mediump float;\n"
#endif
    "uniform vec4 vColor;\n"
    "varying vec2 v_texcoord;\n"
    "uniform sampler2D s_texture;\n"
    "\n"
    "void main()\n"
    "{\n"
    "    gl_FragColor = vec4(vColor.r, vColor.g, vColor.b, texture2D(s_texture, v_texcoord).r);\n"
    "}\n";

static const int pdraw_droneModelIconIndex[] =
{
    2,
    0,
    2,
    1,
};

static const char *pdraw_strHeading[] =
{
    ".N.",
    "NW",
    "W",
    "SW",
    "S",
    "SE",
    "E",
    "NE",
};

static const char *pdraw_strFlyingState[] =
{
    "LANDED",
    "TAKING OFF",
    "HOVERING",
    "FLYING",
    "LANDING",
    "EMERGENCY",
};

static const char *pdraw_strPilotingMode[] =
{
    "MANUAL",
    "RETURN HOME",
    "FLIGHT PLAN",
    "FOLLOW ME",
};

static const float colorRed[4] = { 0.9f, 0.0f, 0.0f, 1.0f };
static const float colorGreen[4] = { 0.0f, 0.9f, 0.0f, 1.0f };
static const float colorDarkGreen[4] = { 0.0f, 0.5f, 0.0f, 1.0f };
static const float colorBlue[4] = { 0.0f, 0.0f, 0.9f, 1.0f };
static const float colorGray[4] = { 0.3f, 0.3f, 0.3f, 1.0f };


Gles2Hud::Gles2Hud(Session *session, VideoMedia *media, unsigned int firstTexUnit)
{
    GLint vertexShader, fragmentShader;
    GLint success = 0;
    int ret = 0;

    mSession = session;
    mMedia = media;
    mFirstTexUnit = firstTexUnit;
    mHudCentralZoneSize = GLES2_HUD_DEFAULT_CENTRAL_ZONE_SIZE;
    mHudHeadingZoneVOffset = GLES2_HUD_DEFAULT_HEADING_ZONE_V_OFFSET;
    mHudRollZoneVOffset = GLES2_HUD_DEFAULT_ROLL_ZONE_V_OFFSET;
    mHudVuMeterZoneHOffset = GLES2_HUD_DEFAULT_VU_METER_ZONE_H_OFFSET;
    mHudVuMeterVInterval = GLES2_HUD_DEFAULT_VU_METER_V_INTERVAL;
    mHudRightZoneHOffset = GLES2_HUD_DEFAULT_RIGHT_ZONE_H_OFFSET;
    mHudRadarZoneHOffset = GLES2_HUD_DEFAULT_RADAR_ZONE_H_OFFSET;
    mHudRadarZoneVOffset = GLES2_HUD_DEFAULT_RADAR_ZONE_V_OFFSET;
    mTextSize = GLES2_HUD_DEFAULT_TEXT_SIZE;
    mSmallIconSize = GLES2_HUD_DEFAULT_SMALL_ICON_SIZE;
    mMediumIconSize = GLES2_HUD_DEFAULT_MEDIUM_ICON_SIZE;
    mHudScale = GLES2_HUD_DEFAULT_SCALE;
    mAspectRatio = 1.;
    mVideoAspectRatio = 1.;
    mHfov = 0.;
    mVfov = 0.;
    mScaleW = 1.;
    mScaleH = 1.;
    mRatioW = 1.;
    mRatioH = 1.;

    if (ret == 0)
    {
        vertexShader = glCreateShader(GL_VERTEX_SHADER);
        if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2Hud: failed to create vertex shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(vertexShader, 1, &hudVertexShader, NULL);
        glCompileShader(vertexShader);
        GLint success = 0;
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            ULOGE("Gles2Hud: vertex shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        if ((fragmentShader == 0) || (fragmentShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2Hud: failed to create fragment shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(fragmentShader, 1, &hudFragmentShader, NULL);
        glCompileShader(fragmentShader);
        GLint success = 0;
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            ULOGE("Gles2Hud: fragment shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* Link shaders */
        mProgram[0] = glCreateProgram();
        glAttachShader(mProgram[0], vertexShader);
        glAttachShader(mProgram[0], fragmentShader);
        glLinkProgram(mProgram[0]);
        glGetProgramiv(mProgram[0], GL_LINK_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetProgramInfoLog(mProgram[0], 512, NULL, infoLog);
            ULOGE("Gles2Hud: program link failed '%s'", infoLog);
            ret = -1;
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    if (ret == 0)
    {
        mPositionHandle = glGetAttribLocation(mProgram[0], "vPosition");
        mTransformMatrixHandle = glGetUniformLocation(mProgram[0], "transform_matrix");
        mColorHandle = glGetUniformLocation(mProgram[0], "vColor");
    }

    if (ret == 0)
    {
        vertexShader = glCreateShader(GL_VERTEX_SHADER);
        if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2Hud: failed to create vertex shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(vertexShader, 1, &hudTexVertexShader, NULL);
        glCompileShader(vertexShader);
        GLint success = 0;
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            ULOGE("Gles2Hud: vertex shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        if ((fragmentShader == 0) || (fragmentShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2Hud: failed to create fragment shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(fragmentShader, 1, &hudTexFragmentShader, NULL);
        glCompileShader(fragmentShader);
        GLint success = 0;
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            ULOGE("Gles2Hud: fragment shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* Link shaders */
        mProgram[1] = glCreateProgram();
        glAttachShader(mProgram[1], vertexShader);
        glAttachShader(mProgram[1], fragmentShader);
        glLinkProgram(mProgram[1]);
        glGetProgramiv(mProgram[1], GL_LINK_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetProgramInfoLog(mProgram[1], 512, NULL, infoLog);
            ULOGE("Gles2Hud: program link failed '%s'", infoLog);
            ret = -1;
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    if (ret == 0)
    {
        mTexUniformSampler = glGetUniformLocation(mProgram[1], "s_texture");
        mTexPositionHandle = glGetAttribLocation(mProgram[1], "position");
        mTexTexcoordHandle = glGetAttribLocation(mProgram[1], "texcoord");
        mTexTransformMatrixHandle = glGetUniformLocation(mProgram[1], "transform_matrix");
        mTexColorHandle = glGetUniformLocation(mProgram[1], "vColor");
    }

    if (ret == 0)
    {
        mIconsTexUnit = mFirstTexUnit;
        ret = loadTextureFromBuffer(hudIcons, hudIconsWidth, hudIconsHeight, mTextTexUnit);
        if (ret < 0)
        {
            ULOGE("Gles2Hud: loadTextureFromBuffer() failed (%d)", ret);
        }
        else
        {
            mIconsTexture = (GLuint)ret;
            ret = 0;
        }
    }

    if (ret == 0)
    {
        mTextTexUnit = mFirstTexUnit + 1;
        ret = loadTextureFromBuffer(font_36::image, font_36::imageW, font_36::imageH, mTextTexUnit);
        if (ret < 0)
        {
            ULOGE("Gles2Hud: loadTextureFromBuffer() failed (%d)", ret);
        }
        else
        {
            mTextTexture = (GLuint)ret;
            ret = 0;
        }
    }
}


Gles2Hud::~Gles2Hud()
{
    glDeleteTextures(1, &mIconsTexture);
    glDeleteTextures(1, &mTextTexture);
    glDeleteProgram(mProgram[0]);
    glDeleteProgram(mProgram[1]);
}


int Gles2Hud::renderHud(unsigned int videoWidth, unsigned int videoHeight,
        unsigned int windowWidth, unsigned int windowHeight,
        const video_frame_metadata_t *metadata, bool hmdDistorsionCorrection, bool headtracking)
{
    if ((videoWidth <= 0) || (videoHeight <= 0) || (windowWidth <= 0) || (windowHeight <= 0) || (!metadata))
    {
        return -1;
    }

    if (hmdDistorsionCorrection)
    {
        mHudCentralZoneSize = GLES2_HUD_HMD_CENTRAL_ZONE_SIZE;
        mHudHeadingZoneVOffset = GLES2_HUD_HMD_HEADING_ZONE_V_OFFSET;
        mHudRollZoneVOffset = GLES2_HUD_HMD_ROLL_ZONE_V_OFFSET;
        mHudVuMeterZoneHOffset = GLES2_HUD_HMD_VU_METER_ZONE_H_OFFSET;
        mHudVuMeterVInterval = GLES2_HUD_HMD_VU_METER_V_INTERVAL;
        mHudRightZoneHOffset = GLES2_HUD_HMD_RIGHT_ZONE_H_OFFSET;
        mHudRadarZoneHOffset = GLES2_HUD_HMD_RADAR_ZONE_H_OFFSET;
        mHudRadarZoneVOffset = GLES2_HUD_HMD_RADAR_ZONE_V_OFFSET;
        mTextSize = GLES2_HUD_HMD_TEXT_SIZE;
        mSmallIconSize = GLES2_HUD_HMD_SMALL_ICON_SIZE;
        mMediumIconSize = GLES2_HUD_HMD_MEDIUM_ICON_SIZE;
        mHudScale = GLES2_HUD_HMD_SCALE;
    }
    else
    {
        mHudCentralZoneSize = GLES2_HUD_DEFAULT_CENTRAL_ZONE_SIZE;
        mHudHeadingZoneVOffset = GLES2_HUD_DEFAULT_HEADING_ZONE_V_OFFSET;
        mHudRollZoneVOffset = GLES2_HUD_DEFAULT_ROLL_ZONE_V_OFFSET;
        mHudVuMeterZoneHOffset = GLES2_HUD_DEFAULT_VU_METER_ZONE_H_OFFSET;
        mHudVuMeterVInterval = GLES2_HUD_DEFAULT_VU_METER_V_INTERVAL;
        mHudRightZoneHOffset = GLES2_HUD_DEFAULT_RIGHT_ZONE_H_OFFSET;
        mHudRadarZoneHOffset = GLES2_HUD_DEFAULT_RADAR_ZONE_H_OFFSET;
        mHudRadarZoneVOffset = GLES2_HUD_DEFAULT_RADAR_ZONE_V_OFFSET;
        mTextSize = GLES2_HUD_DEFAULT_TEXT_SIZE;
        mSmallIconSize = GLES2_HUD_DEFAULT_SMALL_ICON_SIZE;
        mMediumIconSize = GLES2_HUD_DEFAULT_MEDIUM_ICON_SIZE;
        mHudScale = GLES2_HUD_DEFAULT_SCALE;
    }

    float windowAR = (float)windowWidth / (float)windowHeight;
    float videoAR = (float)videoWidth / (float)videoHeight;
    float windowW = 1.;
    float windowH = windowAR;
    mRatioW = 1.;
    mRatioH = 1.;
    if (videoAR >= windowAR)
    {
        mRatioW = 1.;
        mRatioH = windowAR / videoAR;
        windowW = 1.;
        windowH = windowAR;
    }
    else
    {
        mRatioW = videoAR / windowAR;
        mRatioH = 1.;
        windowW = 1.;
        windowH = windowAR;
    }
    mRatioW *= mHudScale;
    mRatioH *= mHudScale;
    mScaleW = mRatioW / windowW;
    mScaleH = mRatioH / windowH;
    mAspectRatio = windowAR;
    mVideoAspectRatio = videoAR;

    float controllerRadarAngle = GLES2_HUD_DEFAULT_CONTROLLER_RADAR_ANGLE;
    float hFov = 0.;
    float vFov = 0.;
    if (mMedia)
        mMedia->getFov(&hFov, &vFov);
    if (hFov == 0.)
        hFov = GLES2_HUD_DEFAULT_HFOV;
    if (vFov == 0.)
        vFov = GLES2_HUD_DEFAULT_VFOV;
    mHfov = hFov * M_PI / 180.;
    mVfov = vFov * M_PI / 180.;

    float horizontalSpeed = sqrtf(metadata->groundSpeed.north * metadata->groundSpeed.north
                                  + metadata->groundSpeed.east * metadata->groundSpeed.east);
    float speedRho = sqrtf(metadata->groundSpeed.north * metadata->groundSpeed.north
                           + metadata->groundSpeed.east * metadata->groundSpeed.east
                           + metadata->groundSpeed.down * metadata->groundSpeed.down);
    float speedPsi = atan2f(metadata->groundSpeed.east, metadata->groundSpeed.north);
    float speedTheta = M_PI / 2 - acosf(metadata->groundSpeed.down / speedRho);
    session_type_t sessionType = PDRAW_SESSION_TYPE_UNKNOWN;
    drone_model_t droneModel = PDRAW_DRONE_MODEL_UNKNOWN;
    const char *friendlyName = NULL;
    int controllerBattery = 256;
    euler_t controllerOrientation;
    bool isControllerOrientationValid = false;
    location_t takeoffLocation, selfLocation;
    takeoffLocation.isValid = 0;
    selfLocation.isValid = 0;
    uint64_t recordingDuration = 0;
    if (mSession)
    {
        sessionType = mSession->getSessionType();
        droneModel = mSession->getPeerMetadata()->getDroneModel();
        friendlyName = mSession->getPeerMetadata()->getFriendlyName().c_str();
        mSession->getPeerMetadata()->getTakeoffLocation(&takeoffLocation);
        recordingDuration = mSession->getPeerMetadata()->getRecordingDuration();
        controllerBattery = mSession->getSelfMetadata()->getControllerBatteryLevel();
        mSession->getSelfMetadata()->getLocation(&selfLocation);
        isControllerOrientationValid = mSession->getSelfMetadata()->getControllerOrientation(&controllerOrientation);
#ifdef DEBUG_RADAR // used to test the radar on records
        isControllerOrientationValid = true;
#endif
        if (mSession->getSettings())
        {
            controllerRadarAngle = mSession->getSettings()->getControllerRadarAngle();
        }
    }
    float groundDistance = 0.;
    if (droneModel == PDRAW_DRONE_MODEL_DISCO)
    {
        if ((metadata->location.isValid) && (takeoffLocation.isValid))
        {
            groundDistance = metadata->location.altitude - takeoffLocation.altitude;
        }
    }
    else
    {
        groundDistance = metadata->groundDistance;
    }
    double takeoffDistance = 0.;
    double takeoffBearing = 0.;
    double takeoffElevation = 0.;
    if ((metadata->location.isValid) && (takeoffLocation.isValid))
    {
        pdraw_coordsDistanceAndBearing(metadata->location.latitude, metadata->location.longitude,
                                       takeoffLocation.latitude, takeoffLocation.longitude,
                                       &takeoffDistance, &takeoffBearing);
        takeoffElevation = atan2(takeoffLocation.altitude - metadata->location.altitude, takeoffDistance);
    }
    double selfDistance = 0.;
    double selfBearing = 0.;
    double selfElevation = 0.;
    if ((metadata->location.isValid) && (selfLocation.isValid))
    {
        pdraw_coordsDistanceAndBearing(metadata->location.latitude, metadata->location.longitude,
                                       selfLocation.latitude, selfLocation.longitude,
                                       &selfDistance, &selfBearing);
        selfElevation = atan2(selfLocation.altitude - metadata->location.altitude, selfDistance);
    }
    int headingInt = ((int)(metadata->droneAttitude.psi * RAD_TO_DEG) + 360) % 360;

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(mProgram[0]);

    glEnableVertexAttribArray(mPositionHandle);
    glEnableVertexAttribArray(mColorHandle);

    float deltaX = 0.;
    float deltaY = 0.;
    float angle = 0., cy;
    if ((headtracking) && (mSession))
    {
        quaternion_t headQuat, headRefQuat;
        mSession->getSelfMetadata()->getHeadOrientation(&headQuat);
        mSession->getSelfMetadata()->getHeadRefOrientation(&headRefQuat);

        /* diff * headRefQuat = headQuat  --->  diff = headQuat * inverse(headRefQuat) */
        quaternion_t headDiff, headRefQuatInv;
        pdraw_quat_conj(&headRefQuat, &headRefQuatInv);
        pdraw_quat_mult(&headQuat, &headRefQuatInv, &headDiff);
        euler_t headOrientation;
        pdraw_quat2euler(&headDiff, &headOrientation);
        deltaX = (headOrientation.psi - metadata->cameraPan) / mHfov * mRatioW * 2.;
        deltaY = (headOrientation.theta - metadata->cameraTilt) / mVfov * mRatioH * 2.;
        angle = headOrientation.phi;
    }

    float transformMatrix[16];
    transformMatrix[0] = cosf(angle) * windowW;
    transformMatrix[1] = -sinf(angle) * windowW;
    transformMatrix[2] = 0;
    transformMatrix[3] = -deltaX;
    transformMatrix[4] = sinf(angle) * windowH;
    transformMatrix[5] = cosf(angle) * windowH;
    transformMatrix[6] = 0;
    transformMatrix[7] = -deltaY;
    transformMatrix[8] = 0;
    transformMatrix[9] = 0;
    transformMatrix[10] = 1;
    transformMatrix[11] = 0;
    transformMatrix[12] = 0;
    transformMatrix[13] = 0;
    transformMatrix[14] = 0;
    transformMatrix[15] = 1;

    glUniformMatrix4fv(mTransformMatrixHandle, 1, false, transformMatrix);

    /* World */
    if (takeoffDistance >= 50.)
    {
        //TODO: pilot
        drawPositionPin(&metadata->frameOrientation, takeoffBearing, takeoffElevation, colorBlue);
    }

    /* Cockpit */
    if (headtracking)
    {
        drawCockpitMarks(metadata->cameraPan, metadata->cameraTilt, colorGray, 0.005 * (float)windowWidth);
    }

    transformMatrix[0] = 1; //TODO windowW;
    transformMatrix[1] = 0;
    transformMatrix[2] = 0;
    transformMatrix[3] = 0;
    transformMatrix[4] = 0;
    transformMatrix[5] = 1; //TODO windowH;
    transformMatrix[6] = 0;
    transformMatrix[7] = 0;
    transformMatrix[8] = 0;
    transformMatrix[9] = 0;
    transformMatrix[10] = 1;
    transformMatrix[11] = 0;
    transformMatrix[12] = 0;
    transformMatrix[13] = 0;
    transformMatrix[14] = 0;
    transformMatrix[15] = 1;

    glUniformMatrix4fv(mTransformMatrixHandle, 1, false, transformMatrix);

    /* Helmet */
    if (horizontalSpeed >= 0.2)
    {
        drawFlightPathVector(&metadata->frameOrientation, speedTheta, speedPsi, colorGreen);
    }
    drawArtificialHorizon(&metadata->droneAttitude, &metadata->frameOrientation, colorGreen);
    drawRoll(metadata->droneAttitude.phi, colorGreen);
    drawHeading(metadata->droneAttitude.psi, horizontalSpeed, speedPsi, colorGreen);
    drawAltitude(metadata->location.altitude, groundDistance, metadata->groundSpeed.down, colorGreen);
    drawSpeed(horizontalSpeed, colorGreen);
#ifdef DEBUG_RADAR // used to test the radar on records
    if ((metadata->location.isValid) && (takeoffLocation.isValid) && (isControllerOrientationValid))
    {
        drawControllerRadar(takeoffDistance, takeoffBearing, controllerOrientation.psi, metadata->droneAttitude.psi, controllerRadarAngle, colorGreen);
    }
#else
    if ((metadata->location.isValid) && (selfLocation.isValid) && (isControllerOrientationValid))
    {
        drawControllerRadar(selfDistance, selfBearing, controllerOrientation.psi, metadata->droneAttitude.psi, controllerRadarAngle, colorGreen);
    }
#endif
    if (sessionType == PDRAW_SESSION_TYPE_RECORD)
    {
        uint64_t currentTime = mSession->getCurrentTime();
        uint64_t duration = mSession->getDuration();
        if ((duration > 0) && (duration != (uint64_t)-1))
            drawRecordTimeline(currentTime, duration, colorGreen);
    }
    else if (sessionType == PDRAW_SESSION_TYPE_STREAM)
    {
        drawRecordingStatus(recordingDuration, colorGreen);
    }

    drawVuMeter(mHudVuMeterZoneHOffset, -mHudVuMeterVInterval, 0.05, metadata->batteryPercentage, 0., 100., 0., 20., colorGreen, colorDarkGreen, 2.);
    drawVuMeter(mHudVuMeterZoneHOffset, 0.0, 0.05, metadata->wifiRssi, -90., -20., -90., -70., colorGreen, colorDarkGreen, 2.);
    drawVuMeter(mHudVuMeterZoneHOffset, mHudVuMeterVInterval, 0.05, metadata->location.svCount, 0., 30., 0., 5., colorGreen, colorDarkGreen, 2.);

    glDisableVertexAttribArray(mPositionHandle);
    glDisableVertexAttribArray(mColorHandle);


    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(mProgram[1]);
    glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);

    glActiveTexture(GL_TEXTURE0 + mIconsTexUnit);
    glBindTexture(GL_TEXTURE_2D, mIconsTexture);
    glUniform1i(mTexUniformSampler, mIconsTexUnit);
    glEnableVertexAttribArray(mTexPositionHandle);
    glEnableVertexAttribArray(mTexTexcoordHandle);
    glEnableVertexAttribArray(mTexColorHandle);

    drawIcon(3, mHudVuMeterZoneHOffset * mRatioW, (-mHudVuMeterVInterval - 0.01) * mRatioH, mSmallIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
    drawIcon(4, mHudVuMeterZoneHOffset * mRatioW, (0.0 - 0.01) * mRatioH, mSmallIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
    drawIcon(5, mHudVuMeterZoneHOffset * mRatioW, (mHudVuMeterVInterval - 0.01) * mRatioH, mSmallIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
    if (droneModel != PDRAW_DRONE_MODEL_UNKNOWN)
    {
        drawIcon(pdraw_droneModelIconIndex[droneModel], 0.0, mHudHeadingZoneVOffset * mRatioH, mSmallIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
    }
    float friendlyNameXOffset = (mHudVuMeterZoneHOffset - 0.05) * mRatioW;
    if ((friendlyName) && strlen(friendlyName))
    {
        if (droneModel != PDRAW_DRONE_MODEL_UNKNOWN)
        {
            drawIcon(pdraw_droneModelIconIndex[droneModel], friendlyNameXOffset + 0.025 * mRatioW, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mMediumIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
            friendlyNameXOffset += 0.06 * mRatioW;
        }
    }
#ifdef DEBUG_RADAR // used to test the radar on records
    if ((metadata->location.isValid) && (takeoffLocation.isValid) && (isControllerOrientationValid))
#else
    if ((metadata->location.isValid) && (selfLocation.isValid) && (isControllerOrientationValid))
#endif
    {
        float x = mHudRadarZoneHOffset * mRatioW;
        float y = mHudRadarZoneVOffset * mRatioH;
        drawIcon(8, x, y, mSmallIconSize, mRatioW, mRatioW * mAspectRatio, colorGreen);
        if ((droneModel != PDRAW_DRONE_MODEL_UNKNOWN) && (takeoffDistance > 50.))
        {
#ifdef DEBUG_RADAR // used to test the radar on records
            angle = M_PI / 2. + controllerOrientation.psi - (selfBearing + M_PI);
#else
            angle = M_PI / 2. + controllerOrientation.psi - (takeoffBearing + M_PI);
#endif
            deltaX = x + 0.06 * cosf(angle) * mRatioW;
            deltaY = y + 0.06 * sinf(angle) * mRatioW * mAspectRatio;
            angle = controllerOrientation.psi - metadata->droneAttitude.psi;
            transformMatrix[0] = cosf(angle) * windowW;
            transformMatrix[1] = -sinf(angle) * windowW;
            transformMatrix[2] = 0;
            transformMatrix[3] = deltaX;
            transformMatrix[4] = sinf(angle) * windowH;
            transformMatrix[5] = cosf(angle) * windowH;
            transformMatrix[6] = 0;
            transformMatrix[7] = deltaY;
            transformMatrix[8] = 0;
            transformMatrix[9] = 0;
            transformMatrix[10] = 1;
            transformMatrix[11] = 0;
            transformMatrix[12] = 0;
            transformMatrix[13] = 0;
            transformMatrix[14] = 0;
            transformMatrix[15] = 1;
            glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);
            drawIcon(pdraw_droneModelIconIndex[droneModel], 0., 0., mSmallIconSize, mScaleW, mScaleH * mVideoAspectRatio, colorGreen);
        }
    }

    /* Heading controller icon */
    if (takeoffDistance > 50.)
    {
        //TODO: pilot
        cy = 0.15 * mScaleW;
        deltaX = 0.;
        deltaY = mHudHeadingZoneVOffset * mRatioH;
        angle = metadata->droneAttitude.psi - takeoffBearing;
        int angleDeg = ((int)(angle * 180. / M_PI + 70. + 360.)) % 360;
        if (angleDeg <= 140)
        {
            transformMatrix[0] = cosf(angle) * windowW;
            transformMatrix[1] = -sinf(angle) * windowW;
            transformMatrix[2] = 0;
            transformMatrix[3] = deltaX;
            transformMatrix[4] = sinf(angle) * windowH;
            transformMatrix[5] = cosf(angle) * windowH;
            transformMatrix[6] = 0;
            transformMatrix[7] = deltaY;
            transformMatrix[8] = 0;
            transformMatrix[9] = 0;
            transformMatrix[10] = 1;
            transformMatrix[11] = 0;
            transformMatrix[12] = 0;
            transformMatrix[13] = 0;
            transformMatrix[14] = 0;
            transformMatrix[15] = 1;
            glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);
            drawIcon(8, 0., cy, mSmallIconSize, mScaleW, mScaleH * mVideoAspectRatio, colorGreen);
        }
    }

    glActiveTexture(GL_TEXTURE0 + mTextTexUnit);
    glBindTexture(GL_TEXTURE_2D, mTextTexture);
    glUniform1i(mTexUniformSampler, mTextTexUnit);

    transformMatrix[0] = 1; //TODO windowW;
    transformMatrix[1] = 0;
    transformMatrix[2] = 0;
    transformMatrix[3] = 0;
    transformMatrix[4] = 0;
    transformMatrix[5] = 1; //TODO windowH;
    transformMatrix[6] = 0;
    transformMatrix[7] = 0;
    transformMatrix[8] = 0;
    transformMatrix[9] = 0;
    transformMatrix[10] = 1;
    transformMatrix[11] = 0;
    transformMatrix[12] = 0;
    transformMatrix[13] = 0;
    transformMatrix[14] = 0;
    transformMatrix[15] = 1;

    glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);

    char str[20];
    snprintf(str, sizeof(str), "%d%%", metadata->batteryPercentage);
    drawText(str, mHudVuMeterZoneHOffset * mRatioW, (-mHudVuMeterVInterval - 0.07) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
    snprintf(str, sizeof(str), "%ddBm", metadata->wifiRssi);
    drawText(str, mHudVuMeterZoneHOffset * mRatioW, (0.0 - 0.07) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
    snprintf(str, sizeof(str), "%d", metadata->location.svCount);
    drawText(str, mHudVuMeterZoneHOffset * mRatioW, (mHudVuMeterVInterval - 0.07) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
    snprintf(str, sizeof(str), "ALT");
    drawText(str, mHudCentralZoneSize * mRatioW, (mHudCentralZoneSize - 0.01) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_BOTTOM, colorGreen);
    snprintf(str, sizeof(str), "%.1fm", metadata->location.altitude);
    drawText(str, (mHudCentralZoneSize + 0.04) * mRatioW, 0.0 * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    if (droneModel == PDRAW_DRONE_MODEL_DISCO)
    {
        if ((metadata->location.isValid) && (takeoffLocation.isValid))
        {
            snprintf(str, sizeof(str), "DELTA: %+.1fm", metadata->location.altitude - takeoffLocation.altitude);
            drawText(str, mHudCentralZoneSize * mRatioW, (-mHudCentralZoneSize + 0.01) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
        }
    }
    else
    {
        snprintf(str, sizeof(str), "GND: %.1fm", metadata->groundDistance);
        drawText(str, mHudCentralZoneSize * mRatioW, (-mHudCentralZoneSize + 0.01) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
    }
    snprintf(str, sizeof(str), "SPD");
    drawText(str, -mHudCentralZoneSize * mRatioW, (mHudCentralZoneSize - 0.01) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_BOTTOM, colorGreen);
    snprintf(str, sizeof(str), "%.1fm/s", horizontalSpeed);
    drawText(str, -(mHudCentralZoneSize + 0.04) * mRatioW, 0.0 * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    if (metadata->airSpeed != -1.)
    {
        snprintf(str, sizeof(str), "AIR: %4.1fm/s", metadata->airSpeed);
        drawText(str, -mHudCentralZoneSize * mRatioW, (-mHudCentralZoneSize + 0.01) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
    }
    if (takeoffDistance != 0.)
    {
        //TODO: pilot
        snprintf(str, sizeof(str), "DIST: %.0fm", takeoffDistance);
        drawText(str, 0.0, (-mHudCentralZoneSize / 2. - 0.10) * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    if ((metadata->flyingState == PDRAW_FLYING_STATE_TAKINGOFF) ||
        (metadata->flyingState == PDRAW_FLYING_STATE_LANDING) ||
        (metadata->flyingState == PDRAW_FLYING_STATE_EMERGENCY))
    {
        drawText(pdraw_strFlyingState[metadata->flyingState], 0.0, (mHudCentralZoneSize / 2. + 0.10) * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    else if ((metadata->pilotingMode == PDRAW_PILOTING_MODE_RETURN_HOME) ||
        (metadata->pilotingMode == PDRAW_PILOTING_MODE_FLIGHT_PLAN))
    {
        drawText(pdraw_strPilotingMode[metadata->pilotingMode], 0.0, (mHudCentralZoneSize / 2. + 0.10) * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    else if ((metadata->pilotingMode == PDRAW_PILOTING_MODE_FOLLOW_ME) && (metadata->followMeEnabled))
    {
        drawText((metadata->followMeMode == 1) ? "FOLLOW ME" : "LOOK AT ME", 0.0, (mHudCentralZoneSize / 2. + 0.10) * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    if ((controllerBattery > 0) && (controllerBattery <= 255))
    {
        if (controllerBattery <= 100)
            snprintf(str, sizeof(str), "CTRL BAT: %d%%", controllerBattery);
        else
            snprintf(str, sizeof(str), "CTRL BAT: --%%");
        drawText(str, mHudRightZoneHOffset * mRatioW, 0. * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    if (sessionType == PDRAW_SESSION_TYPE_STREAM)
    {
        snprintf(str, sizeof(str), "CTRL LOC: %s", (selfLocation.isValid) ? "OK" : "NOK");
        drawText(str, mHudRightZoneHOffset * mRatioW, 0.05 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    if ((friendlyName) && strlen(friendlyName))
    {
        drawText(friendlyName, friendlyNameXOffset, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
    }
    if (sessionType == PDRAW_SESSION_TYPE_RECORD)
    {
        uint64_t currentTime = mSession->getCurrentTime();
        uint64_t duration = mSession->getDuration();
        if ((currentTime > 0) && (currentTime != (uint64_t)-1) && (duration > 0) && (duration != (uint64_t)-1))
        {
            uint64_t remainingTime = duration - currentTime;
            unsigned int cHrs = 0, cMin = 0, cSec = 0, cMsec = 0;
            unsigned int rHrs = 0, rMin = 0, rSec = 0, rMsec = 0;
            unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
            pdraw_friendlyTimeFromUs(currentTime, &cHrs, &cMin, &cSec, &cMsec);
            pdraw_friendlyTimeFromUs(remainingTime, &rHrs, &rMin, &rSec, &rMsec);
            pdraw_friendlyTimeFromUs(duration, &dHrs, &dMin, &dSec, &dMsec);
            if (dHrs)
                snprintf(str, sizeof(str), "+%02d:%02d:%02d.%03d", cHrs, cMin, cSec, cMsec);
            else
                snprintf(str, sizeof(str), "+%02d:%02d.%03d", cMin, cSec, cMsec);
            drawText(str, (mHudRightZoneHOffset - 0.4) * mRatioW, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_LEFT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
            if (dHrs)
                snprintf(str, sizeof(str), "-%02d:%02d:%02d.%03d", rHrs, rMin, rSec, rMsec);
            else
                snprintf(str, sizeof(str), "-%02d:%02d.%03d", rMin, rSec, rMsec);
            drawText(str, mHudRightZoneHOffset * mRatioW, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
            if (dHrs)
                snprintf(str, sizeof(str), "DUR: %02d:%02d:%02d", dHrs, dMin, dSec);
            else
                snprintf(str, sizeof(str), "DUR: %02d:%02d", dMin, dSec);
            drawText(str, (mHudRightZoneHOffset - 0.2) * mRatioW, mHudRollZoneVOffset * mRatioH + 0.10 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
        }
    }
    else if (sessionType == PDRAW_SESSION_TYPE_STREAM)
    {
        if (recordingDuration > 0)
        {
            unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
            pdraw_friendlyTimeFromUs(recordingDuration, &dHrs, &dMin, &dSec, &dMsec);
            if (dHrs)
                snprintf(str, sizeof(str), "REC %02d:%02d:%02d", dHrs, dMin, dSec);
            else
                snprintf(str, sizeof(str), "REC %02d:%02d", dMin, dSec);
            drawText(str, mHudRightZoneHOffset * mRatioW, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
        }
        else
        {
            drawText("REC", mHudRightZoneHOffset * mRatioW, mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_RIGHT, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
        }
    }
    snprintf(str, sizeof(str), "%03d", headingInt);
    drawText(str, 0. * mRatioW, (mHudHeadingZoneVOffset + 0.10) * mRatioH, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_BOTTOM, colorGreen);

    float height = mHudCentralZoneSize * mRatioW * mAspectRatio;
    int steps = 6, i;
    for (i = -steps; i <= steps; i++)
    {
        if (i != 0)
        {
            if (!(i & 1))
            {
                snprintf(str, sizeof(str), "%+2d ", i * 10);
                drawText(str, 0. * mRatioW, i * height / 2 / steps, mTextSize * mRatioW, 1., mAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_MIDDLE, colorGreen);
            }
        }
    }

    /* Roll text */
    cy = 0.10 * mScaleW;
    deltaX = 0.;
    deltaY = mHudRollZoneVOffset * mRatioH;
    steps = 2;
    transformMatrix[0] = 1;
    transformMatrix[1] = 0;
    transformMatrix[2] = 0;
    transformMatrix[3] = deltaX;
    transformMatrix[4] = 0;
    transformMatrix[5] = 1;
    transformMatrix[6] = 0;
    transformMatrix[7] = deltaY;
    transformMatrix[8] = 0;
    transformMatrix[9] = 0;
    transformMatrix[10] = 1;
    transformMatrix[11] = 0;
    transformMatrix[12] = 0;
    transformMatrix[13] = 0;
    transformMatrix[14] = 0;
    transformMatrix[15] = 1;
    for (i = -steps, angle = M_PI * (-30. * steps) / 180.; i <= steps; i++, angle += M_PI * 30. / 180.)
    {
        int angleDeg = (i * 30 + 60 + 360) % 360;
        if (angleDeg <= 120)
        {
            transformMatrix[0] = cosf(angle) * windowW;
            transformMatrix[1] = -sinf(angle) * windowW;
            transformMatrix[4] = sinf(angle) * windowH;
            transformMatrix[5] = cosf(angle) * windowH;
            glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);
            if (i == 0)
                snprintf(str, sizeof(str), "0");
            else
                snprintf(str, sizeof(str), "%+2d ", -i * 30);
            drawText(str, 0., cy, mTextSize, mScaleW, mScaleH * mVideoAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
        }
    }

    /* Heading text */
    cy = 0.10 * mScaleW;
    deltaX = 0.;
    deltaY = mHudHeadingZoneVOffset * mRatioH;
    transformMatrix[0] = 1;
    transformMatrix[1] = 0;
    transformMatrix[2] = 0;
    transformMatrix[3] = deltaX;
    transformMatrix[4] = 0;
    transformMatrix[5] = 1;
    transformMatrix[6] = 0;
    transformMatrix[7] = deltaY;
    transformMatrix[8] = 0;
    transformMatrix[9] = 0;
    transformMatrix[10] = 1;
    transformMatrix[11] = 0;
    transformMatrix[12] = 0;
    transformMatrix[13] = 0;
    transformMatrix[14] = 0;
    transformMatrix[15] = 1;
    for (i = 0, angle = metadata->droneAttitude.psi; i < 8; i++, angle += M_PI / 4.)
    {
        int angleDeg = (headingInt + i * 45 + 70 + 360) % 360;
        if (angleDeg <= 140)
        {
            transformMatrix[0] = cosf(angle) * windowW;
            transformMatrix[1] = -sinf(angle) * windowW;
            transformMatrix[4] = sinf(angle) * windowH;
            transformMatrix[5] = cosf(angle) * windowH;
            glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);
            drawText(pdraw_strHeading[i], 0., cy, mTextSize, mScaleW, mScaleH * mVideoAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_TOP, colorGreen);
        }
    }

    /* Radar text */
#ifdef DEBUG_RADAR // used to test the radar on records
    if ((metadata->location.isValid) && (takeoffLocation.isValid) && (isControllerOrientationValid))
#else
    if ((metadata->location.isValid) && (selfLocation.isValid) && (isControllerOrientationValid))
#endif
    {
        cy = 0.09 * mScaleW;
        deltaX = mHudRadarZoneHOffset * mRatioW;
        deltaY = mHudRadarZoneVOffset * mRatioH;
        transformMatrix[0] = 1;
        transformMatrix[1] = 0;
        transformMatrix[2] = 0;
        transformMatrix[3] = deltaX;
        transformMatrix[4] = 0;
        transformMatrix[5] = 1;
        transformMatrix[6] = 0;
        transformMatrix[7] = deltaY;
        transformMatrix[8] = 0;
        transformMatrix[9] = 0;
        transformMatrix[10] = 1;
        transformMatrix[11] = 0;
        transformMatrix[12] = 0;
        transformMatrix[13] = 0;
        transformMatrix[14] = 0;
        transformMatrix[15] = 1;
        for (i = 0, angle = controllerOrientation.psi; i < 8; i += 2, angle += M_PI / 2.)
        {
            transformMatrix[0] = cosf(angle) * windowW;
            transformMatrix[1] = -sinf(angle) * windowW;
            transformMatrix[4] = sinf(angle) * windowH;
            transformMatrix[5] = cosf(angle) * windowH;
            glUniformMatrix4fv(mTexTransformMatrixHandle, 1, false, transformMatrix);
            drawText(pdraw_strHeading[i], 0., cy, mTextSize, mScaleW, mScaleH * mVideoAspectRatio, GLES2_HUD_TEXT_ALIGN_CENTER, GLES2_HUD_TEXT_ALIGN_BOTTOM, colorGreen);
        }
    }

    glDisableVertexAttribArray(mTexPositionHandle);
    glDisableVertexAttribArray(mTexTexcoordHandle);
    glDisableVertexAttribArray(mTexColorHandle);

    return 0;
}


void Gles2Hud::setVideoMedia(VideoMedia *media)
{
    mMedia = media;
}


int Gles2Hud::loadTextureFromBuffer(const uint8_t *buffer, int width, int height, int texUnit)
{
    int ret = 0;

    if ((ret == 0) && (width > 0) && (height > 0) && (buffer != NULL))
    {
        GLuint tex;
        glGenTextures(1, &tex);

        glActiveTexture(GL_TEXTURE0 + texUnit);
        glBindTexture(GL_TEXTURE_2D, tex);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0,
                     GL_LUMINANCE, GL_UNSIGNED_BYTE, buffer);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        ret = (int)tex;
    }

    return ret;
}


void Gles2Hud::drawIcon(int index, float x, float y, float size, float scaleW, float scaleH, const float color[4])
{
    float vertices[8];
    float texCoords[8];

    vertices[0] = x - size * scaleW / 2.;
    vertices[1] = y - size * scaleH / 2.;
    vertices[2] = x + size * scaleW / 2.;
    vertices[3] = y - size * scaleH / 2.;
    vertices[4] = x - size * scaleW / 2.;
    vertices[5] = y + size * scaleH / 2.;
    vertices[6] = x + size * scaleW / 2.;
    vertices[7] = y + size * scaleH / 2.;

    glVertexAttribPointer(mTexPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    int ix = index % 3;
    int iy = index / 3;

    texCoords[0] = ((float)ix + 0.) / 3.;
    texCoords[1] = ((float)iy + 0.99) / 3.;
    texCoords[2] = ((float)ix + 0.99) / 3.;
    texCoords[3] = ((float)iy + 0.99) / 3.;
    texCoords[4] = ((float)ix + 0.) / 3.;
    texCoords[5] = ((float)iy + 0.) / 3.;
    texCoords[6] = ((float)ix + 0.99) / 3.;
    texCoords[7] = ((float)iy + 0.) / 3.;

    glVertexAttribPointer(mTexTexcoordHandle, 2, GL_FLOAT, false, 0, texCoords);

    glUniform4fv(mTexColorHandle, 1, color);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}


void Gles2Hud::getTextDimensions(const char *str, float size, float scaleW, float scaleH, float *width, float *height)
{
    float w, h;

    font_36::FileHeader *glyphInfos = &font_36::font;
    float cx = 0.;
    const char *c = str;
    while (*c != '\0')
    {
        if (*c == '\n')
        {
            break;
        }
        else
        {
            font_36::GlyphInfo &g = glyphInfos->glyphs[(int)(*c)];
            cx += g.norm.advance;
        }
        c++;
    }
    w = cx;
    h = glyphInfos->norm.ascent + glyphInfos->norm.descent; //+ glyphInfos->norm.linegap;
    w *= size * scaleW;
    h *= size * scaleH;

    if (width)
        *width = w;
    if (height)
        *height = h;
}


void Gles2Hud::drawText(const char *str, float x, float y, float size, float scaleW, float scaleH, gles2_hud_text_align_t halign, gles2_hud_text_align_t valign, const float color[4])
{
    float w, h;
    float vertices[8];
    float texCoords[8];
    font_36::FileHeader *glyphInfos = &font_36::font;

    getTextDimensions(str, size, scaleW, scaleH, &w, &h);

    switch (halign)
    {
        default:
        case GLES2_HUD_TEXT_ALIGN_LEFT:
            break;
        case GLES2_HUD_TEXT_ALIGN_CENTER:
            x -= w / 2;
            break;
        case GLES2_HUD_TEXT_ALIGN_RIGHT:
            x -= w;
            break;
    }

    switch (valign)
    {
        default:
        case GLES2_HUD_TEXT_ALIGN_TOP:
            y -= h;
            break;
        case GLES2_HUD_TEXT_ALIGN_MIDDLE:
            y -= h / 2;
            break;
        case GLES2_HUD_TEXT_ALIGN_BOTTOM:
            break;
    }

    glUniform4fv(mTexColorHandle, 1, color);

    const char *c = str;
    float cx = 0.;
    while (*c != '\0')
    {
        if (*c == '\n')
        {
            break;
        }
        else
        {
            font_36::GlyphInfo &g = glyphInfos->glyphs[(int)(*c)];
            vertices[0] = x + cx + g.norm.offX * size * scaleW;
            vertices[1] = y - g.norm.offY * size * scaleH;
            vertices[2] = x + cx + (g.norm.offX + g.norm.width) * size * scaleW;
            vertices[3] = y - g.norm.offY * size * scaleH;
            vertices[4] = x + cx + g.norm.offX * size * scaleW;
            vertices[5] = y - (g.norm.offY + g.norm.height) * size * scaleH;
            vertices[6] = x + cx + (g.norm.offX + g.norm.width) * size * scaleW;
            vertices[7] = y - (g.norm.offY + g.norm.height) * size * scaleH;

            texCoords[0] = g.norm.u;
            texCoords[1] = g.norm.v + g.norm.height;
            texCoords[2] = g.norm.u + g.norm.width;
            texCoords[3] = g.norm.v + g.norm.height;
            texCoords[4] = g.norm.u;
            texCoords[5] = g.norm.v;
            texCoords[6] = g.norm.u + g.norm.width;
            texCoords[7] = g.norm.v;

            glVertexAttribPointer(mTexPositionHandle, 2, GL_FLOAT, false, 0, vertices);
            glVertexAttribPointer(mTexTexcoordHandle, 2, GL_FLOAT, false, 0, texCoords);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            cx += g.norm.advance * size * scaleW;
        }
        c++;
    }
}


void Gles2Hud::drawLine(float x1, float y1, float x2, float y2, const float color[4], float lineWidth)
{
    float vertices[4];

    vertices[0] = x1;
    vertices[1] = y1;
    vertices[2] = x2;
    vertices[3] = y2;

    glLineWidth(lineWidth);

    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    glUniform4fv(mColorHandle, 1, color);

    glDrawArrays(GL_LINES, 0, 2);
}


void Gles2Hud::drawRect(float x1, float y1, float x2, float y2, const float color[4], float lineWidth)
{
    float vertices[8];

    vertices[0] = x1;
    vertices[1] = y1;
    vertices[2] = x1;
    vertices[3] = y2;
    vertices[4] = x2;
    vertices[5] = y2;
    vertices[6] = x2;
    vertices[7] = y1;

    glLineWidth(lineWidth);

    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    glUniform4fv(mColorHandle, 1, color);

    glDrawArrays(GL_LINE_LOOP, 0, 4);
}


void Gles2Hud::drawArc(float cx, float cy, float rx, float ry, float startAngle, float spanAngle, int numSegments, const float color[4], float lineWidth)
{
    int i;
    float theta = spanAngle / (float)numSegments;
    float c = cosf(theta);
    float s = sinf(theta);
    float t;

    float x = cosf(startAngle);
    float y = sinf(startAngle);

    float vertices[2 * (numSegments + 1)];

    for (i = 0; i <= numSegments; i++)
    {
        vertices[2 * i] = x * rx + cx;
        vertices[2 * i + 1] = y * ry + cy;

        // apply the rotation
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }

    glLineWidth(lineWidth);

    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    glUniform4fv(mColorHandle, 1, color);

    glDrawArrays(GL_LINE_STRIP, 0, numSegments + 1);
}


void Gles2Hud::drawEllipse(float cx, float cy, float rx, float ry, int numSegments, const float color[4], float lineWidth)
{
    int i;
    float theta = 2. * M_PI / (float)numSegments;
    float c = cosf(theta);
    float s = sinf(theta);
    float t;

    float x = 1.; // start at angle = 0
    float y = 0.;

    float vertices[2 * numSegments];

    for (i = 0; i < numSegments; i++)
    {
        vertices[2 * i] = x * rx + cx;
        vertices[2 * i + 1] = y * ry + cy;

        // apply the rotation
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }

    glLineWidth(lineWidth);

    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    glUniform4fv(mColorHandle, 1, color);

    glDrawArrays(GL_LINE_LOOP, 0, numSegments);
}


void Gles2Hud::drawEllipseFilled(float cx, float cy, float rx, float ry, int numSegments, const float color[4])
{
    int i;
    numSegments &= ~1;
    float theta = 2. * M_PI / (float)numSegments;
    float c = cosf(theta);
    float s = sinf(theta);
    float t;

    float x = 1.; // start at angle = 0
    float y = 0.;

    float vertices[(3 * (numSegments / 2) + 1) * 2];

    for (i = 0; i < numSegments / 2; i++)
    {
        vertices[6 * i] = x * rx + cx;
        vertices[6 * i + 1] = y * ry + cy;

        // apply the rotation
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;

        vertices[6 * i + 2] = cx;
        vertices[6 * i + 3] = cy;

        vertices[6 * i + 4] = x * rx + cx;
        vertices[6 * i + 5] = y * ry + cy;

        // apply the rotation
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    vertices[6 * i] = x * rx + cx;
    vertices[6 * i + 1] = y * ry + cy;

    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);

    glUniform4fv(mColorHandle, 1, color);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 3 * (numSegments / 2) + 1);
}


void Gles2Hud::drawVuMeter(float x, float y, float r, float value, float minVal, float maxVal, float criticalMin, float criticalMax, const float color[4], const float criticalColor[4], float lineWidth)
{
    x *= mRatioW;
    y *= mRatioH;
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;
    float span = 4. * M_PI / 3.;
    float start = (M_PI - span) / 2.;
    drawArc(x, y, r * mRatioW, r * mRatioW * mAspectRatio, start, span, 20, color, 2.);
    if ((criticalMin >= minVal) && (criticalMin <= maxVal) && (criticalMax >= minVal) && (criticalMax <= maxVal) && (criticalMin < criticalMax))
    {
        float start2 = start + (1. - (criticalMax - minVal) / (maxVal - minVal)) * span;
        float end2 = start + (1. - (criticalMin - minVal) / (maxVal - minVal)) * span;
        drawArc(x, y, r * mRatioW * 0.9, r * mRatioW * mAspectRatio * 0.9, start2, end2 - start2, 10, criticalColor, 2.);
    }
    float angle = start + (1. - (value - minVal) / (maxVal - minVal)) * span;
    float x1 = x + r * mRatioW * 0.4 * cosf(angle);
    float y1 = y + r * mRatioW * mAspectRatio * 0.4 * sinf(angle);
    float x2 = x + r * mRatioW * 0.9 * cosf(angle);
    float y2 = y + r * mRatioW * mAspectRatio * 0.9 * sinf(angle);
    drawLine(x1, y1, x2, y2, color, 2.);
}


void Gles2Hud::drawCockpitMarks(float cameraPan, float cameraTilt, const float color[4], float lineWidth)
{
    int i;
    float cx = -cameraPan / mHfov * 2. * mScaleW;
    float cy = -cameraTilt / mVfov * 2. * mScaleH;
    float rx = 2. / mHfov * mScaleW;
    float ry = 2. / mHfov * mScaleH * mVideoAspectRatio;

    float angle, centerAngle = M_PI / 20., limitAngle = M_PI / 2;
    float x1, y1, x2, y2;
    for (i = 0, angle = 0.; i < 8; i++, angle += M_PI / 4.)
    {
        x1 = centerAngle * rx * cosf(angle);
        y1 = centerAngle * ry * sinf(angle);
        x2 = centerAngle * rx * cosf(angle + M_PI / 4.);
        y2 = centerAngle * ry * sinf(angle + M_PI / 4.);
        drawLine(cx + x1, cy + y1, cx + x2, cy + y2, color, lineWidth);
        x2 = limitAngle * rx * cosf(angle);
        y2 = limitAngle * ry * sinf(angle);
        drawLine(cx + x1, cy + y1, cx + x2, cy + y2, color, lineWidth);
        if (i & 1)
        {
            drawArc(cx, cy, limitAngle / 4. * rx, limitAngle / 4. * ry, angle, M_PI / 4., 20, color, lineWidth);
            drawArc(cx, cy, limitAngle * 3. / 4. * rx, limitAngle * 3. / 4. * ry, angle, M_PI / 4., 40, color, lineWidth);
        }
        else
        {
            drawArc(cx, cy, limitAngle * 2. / 4. * rx, limitAngle * 2. / 4. * ry, angle, M_PI / 4., 30, color, lineWidth);
        }
    }
    drawEllipse(cx, cy, limitAngle * rx, limitAngle * ry, 40 * 8, color, lineWidth);
}


void Gles2Hud::drawArtificialHorizon(const euler_t *drone, const euler_t *frame, const float color[4])
{
    int i;
    float x1, y1, x2, y2;
    float height = mHudCentralZoneSize * mRatioW * mAspectRatio;
    int steps = 6;

    /* Scale */
    for (i = -steps; i <= steps; i++)
    {
        if (i != 0)
        {
            if (i & 1)
            {
                drawLine(-0.01 * mRatioW, i * height / 2 / steps, 0.01 * mRatioW, i * height / 2 / steps, color, 2.);
            }   
        }
    }

    /* Horizon */
    x1 = -0.5 * mHudCentralZoneSize * mRatioW * cosf(frame->phi);
    y1 = -0.5 * mHudCentralZoneSize * mRatioW * mAspectRatio * sinf(frame->phi);
    x2 = 0.5 * mHudCentralZoneSize * mRatioW * cosf(frame->phi);
    y2 = 0.5 * mHudCentralZoneSize * mRatioW * mAspectRatio * sinf(frame->phi);
    drawLine(x1, y1, x2, y2, color, 2.);

    /* Drone */
    float vertices[10];
    float droneY = drone->theta / (M_PI / 18 * steps) * height / 2;
    vertices[0] = -0.06 * mRatioW * cosf(frame->phi - drone->phi);
    vertices[1] = -0.06 * mRatioW * mAspectRatio * sinf(frame->phi - drone->phi) + droneY;
    vertices[2] = -0.015 * mRatioW * cosf(frame->phi - drone->phi);
    vertices[3] = -0.015 * mRatioW * mAspectRatio * sinf(frame->phi - drone->phi) + droneY;
    vertices[4] = 0.015 * mRatioW * sinf(frame->phi - drone->phi);
    vertices[5] = -0.015 * mRatioW * mAspectRatio * cosf(frame->phi - drone->phi) + droneY;
    vertices[6] = 0.015 * mRatioW * cosf(frame->phi - drone->phi);
    vertices[7] = 0.015 * mRatioW * mAspectRatio * sinf(frame->phi - drone->phi) + droneY;
    vertices[8] = 0.06 * mRatioW * cosf(frame->phi - drone->phi);
    vertices[9] = 0.06 * mRatioW * mAspectRatio * sinf(frame->phi - drone->phi) + droneY;
    glLineWidth(6.);
    glVertexAttribPointer(mPositionHandle, 2, GL_FLOAT, false, 0, vertices);
    glUniform4fv(mColorHandle, 1, color);
    glDrawArrays(GL_LINE_STRIP, 0, 5);
}


void Gles2Hud::drawRoll(float droneRoll, const float color[4])
{
    int i;
    float rotation, x1, y1, x2, y2;
    float width = 0.12 * mRatioW;
    float yOffset = mHudRollZoneVOffset * mRatioH;
    int steps = 6;

    drawArc(0., yOffset, width, width * mAspectRatio, M_PI * (90. - 10. * steps) / 180., M_PI * 20. * steps / 180., 100, color, 2.);
    rotation = M_PI / 2. - droneRoll;
    x1 = (width - 0.012 * mRatioW) * cosf(rotation);
    y1 = (width - 0.012 * mRatioW) * mAspectRatio * sinf(rotation) + yOffset;
    x2 = (width + 0.012 * mRatioW) * cosf(rotation);
    y2 = (width + 0.012 * mRatioW) * mAspectRatio * sinf(rotation) + yOffset;
    drawLine(x1, y1, x2, y2, color, 2.);

    for (i = -steps, rotation = M_PI * (90. - 10. * steps) / 180.; i <= steps; i++, rotation += M_PI * 10. / 180.)
    {
        int angle = (i * 10 + 60 + 360) % 360;
        if (angle <= 120)
        {
            x1 = width * cosf(rotation);
            y1 = width * mAspectRatio * sinf(rotation) + yOffset;
            x2 = (width - 0.008 * mRatioW) * cosf(rotation);
            y2 = (width - 0.008 * mRatioW) * mAspectRatio * sinf(rotation) + yOffset;
            drawLine(x1, y1, x2, y2, color, 2.);
        }
    }

    x1 = 0.;
    y1 = yOffset;
    x2 = 0.;
    y2 = yOffset;
    drawLine(x1, y1, x2, y2, color, 2.);
}


void Gles2Hud::drawHeading(float droneYaw, float horizontalSpeed, float speedPsi, const float color[4])
{
    int i;
    int heading = ((int)(droneYaw * RAD_TO_DEG) + 360) % 360;
    float rotation, x1, y1, x2, y2;
    char strHeading[20];
    snprintf(strHeading, sizeof(strHeading), "%d", heading);

    float width = 0.12 * mRatioW;
    float yOffset = mHudHeadingZoneVOffset * mRatioH;

    drawArc(0., yOffset, width, width * mAspectRatio, M_PI * 20. / 180., M_PI * 140. / 180., 100, color, 2.);
    x1 = 0.;
    y1 = yOffset + width * mAspectRatio;
    x2 = 0.;
    y2 = yOffset + (width + 0.01 * mRatioW) * mAspectRatio;
    drawLine(x1, y1, x2, y2, color, 2.);
    //drawText(QRect(-40, -headingWidth / 2 - 25, 80, 20), Qt::AlignCenter, strHeading, NULL);

    for (i = 0, rotation = droneYaw + M_PI / 2.; i < 36; i++, rotation += M_PI * 10. / 180.)
    {
        int angle = (heading + i * 10 + 70 + 360) % 360;
        if (angle <= 140)
        {
            x1 = width * cosf(rotation);
            y1 = width * mAspectRatio * sinf(rotation) + yOffset;
            x2 = (width - 0.01 * mRatioW) * cosf(rotation);
            y2 = (width - 0.01 * mRatioW) * mAspectRatio * sinf(rotation) + yOffset;
            drawLine(x1, y1, x2, y2, color, 2.);
        }
    }
    for (i = 0, rotation = droneYaw + M_PI / 2.; i < 8; i++, rotation += M_PI * 45. / 180.)
    {
        int angle = (heading + i * 45 + 70 + 360) % 360;
        if (angle <= 140)
        {
            //drawText(QRect(-40, -headingWidth / 2 + 10, 80, 20), Qt::AlignCenter, szHeading[i], NULL);
        }
    }

    if (horizontalSpeed >= 0.2)
    {
        rotation = droneYaw - speedPsi + M_PI / 2.;
        x1 = 0.045 * mRatioW * cosf(rotation);
        y1 = 0.045 * mRatioW * mAspectRatio * sinf(rotation) + yOffset;
        x2 = 0.020 * mRatioW * cosf(rotation);
        y2 = 0.020 * mRatioW * mAspectRatio * sinf(rotation) + yOffset;
        drawLine(x1, y1, x2, y2, color, 2.);
        x1 = 0.045 * mRatioW * cosf(rotation);
        y1 = 0.045 * mRatioW * mAspectRatio * sinf(rotation) + yOffset;
        x2 = 0.010 * mRatioW * cosf(rotation - 5. * M_PI / 6.) + x1;
        y2 = 0.010 * mRatioW * mAspectRatio * sinf(rotation - 5. * M_PI / 6.) + y1;
        drawLine(x1, y1, x2, y2, color, 2.);
        x1 = 0.045 * mRatioW * cosf(rotation);
        y1 = 0.045 * mRatioW * mAspectRatio * sinf(rotation) + yOffset;
        x2 = 0.010 * mRatioW * cosf(rotation + 5. * M_PI / 6.) + x1;
        y2 = 0.010 * mRatioW * mAspectRatio * sinf(rotation + 5. * M_PI / 6.) + y1;
        drawLine(x1, y1, x2, y2, color, 2.);
    }
}


void Gles2Hud::drawAltitude(double altitude, float groundDistance, float downSpeed, const float color[4])
{
    char strAltitude[20];
    sprintf(strAltitude, "%.1fm", altitude);

    float xOffset = mHudCentralZoneSize * mRatioW;
    float height = mHudCentralZoneSize * mRatioW * mAspectRatio;
    float altitudeInterval = height / 20.;

    drawLine(xOffset, -height / 2., xOffset, height / 2., color, 2.);
    drawLine(xOffset, -height / 2., xOffset + 0.08 * mRatioW, -height / 2., color, 2.);
    drawLine(xOffset, height / 2., xOffset + 0.08 * mRatioW, height / 2., color, 2.);
    drawRect(xOffset + 0.03 * mRatioW, -0.017 * mRatioW * mAspectRatio, xOffset + 0.03 * mRatioW + 0.1 * mRatioW, 0.017 * mRatioW * mAspectRatio, color, 2.);
    drawLine(xOffset, 0., xOffset + 0.03 * mRatioW, -0.017 * mRatioW * mAspectRatio, color, 2.);
    drawLine(xOffset, 0., xOffset + 0.03 * mRatioW, 0.017 * mRatioW * mAspectRatio, color, 2.);

    float y = (ceil(altitude) - altitude) * altitudeInterval;
    int altInt = ((int)ceil(altitude));
    int altMod5 = altInt % 5;
    while (y < height / 2.)
    {
        drawLine(xOffset, y, xOffset + ((altMod5 == 0) ? 0.03 * mRatioW : 0.01 * mRatioW), y, color, 2.);
        if ((!altMod5) && (y > -height / 2. + 0.03 * mRatioW) && (y < height / 2. - 0.03 * mRatioW)
                && (!((y > -0.03 * mRatioW) && (y < 0.03 * mRatioW))))
        {
            sprintf(strAltitude, "%d", altInt);
            //drawText(QRect(drawWidth / 80, y - drawWidth / 180, drawWidth / 128 * strlen(strAltitude), drawWidth / 80), Qt::AlignCenter, strAltitude, NULL);
        }
        y += altitudeInterval;
        altInt++;
        altMod5 = altInt % 5;
    }
    y = -(altitude - floor(altitude)) * altitudeInterval;
    altInt = ((int)floor(altitude));
    altMod5 = altInt % 5;
    while (y > -height / 2.)
    {
        drawLine(xOffset, y, xOffset + ((altMod5 == 0) ? 0.03 * mRatioW : 0.01 * mRatioW), y, color, 2.);
        if ((!altMod5) && (y > -height / 2. + 0.017 * mRatioW * mAspectRatio) && (y < height / 2. - 0.017 * mRatioW * mAspectRatio)
                && (!((y > -0.017 * mRatioW * mAspectRatio) && (y < 0.017 * mRatioW * mAspectRatio))))
        {
            sprintf(strAltitude, "%d", altInt);
            //drawText(QRect(drawWidth / 80, y - drawWidth / 180, drawWidth / 128 * strlen(strAltitude), drawWidth / 80), Qt::AlignCenter, strAltitude, NULL);
        }
        y -= altitudeInterval;
        altInt--;
        altMod5 = altInt % 5;
    }

    /* Ground distance */
    y = -groundDistance * height / 20.;
    if ((y < height / 2.) && (y > -height / 2. + 0.01 * mRatioW * mAspectRatio))
    {
        if ((y > -0.012 * mRatioW * mAspectRatio) && (y < 0.022 * mRatioW * mAspectRatio))
        {
            drawLine(xOffset, y, xOffset + 0.03 * mRatioW, y, color, 2.);
        }
        else
        {
            drawLine(xOffset, y, xOffset + 0.06 * mRatioW, y, color, 2.);
            drawLine(xOffset + 0.03 * mRatioW, y, xOffset + 0.04 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
            drawLine(xOffset + 0.04 * mRatioW, y, xOffset + 0.05 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
            drawLine(xOffset + 0.05 * mRatioW, y, xOffset + 0.06 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
        }
        drawLine(xOffset, y, xOffset + 0.01 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
        drawLine(xOffset + 0.01 * mRatioW, y, xOffset + 0.02 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
        drawLine(xOffset + 0.02 * mRatioW, y, xOffset + 0.03 * mRatioW, y - 0.01 * mRatioW * mAspectRatio, color, 2.);
    }

    /* Speed indication */
    if (fabs(downSpeed) >= 0.2)
    {
        float x1, y1, x2, y2;
        x1 = xOffset + 0.15 * mRatioW;
        y1 = -0.017 * mRatioW * mAspectRatio;
        x2 = xOffset + 0.15 * mRatioW;
        y2 = 0.017 * mRatioW * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        if (downSpeed < 0.)
        {
            x1 = xOffset + 0.15 * mRatioW;
            y1 = 0.017 * mRatioW * mAspectRatio;
            x2 = xOffset + 0.15 * mRatioW - 0.0056 * mRatioW;
            y2 = 0.017 * mRatioW * mAspectRatio - 0.0098 * mRatioW * mAspectRatio;
            drawLine(x1, y1, x2, y2, color, 2.);
            x1 = xOffset + 0.15 * mRatioW;
            y1 = 0.017 * mRatioW * mAspectRatio;
            x2 = xOffset + 0.15 * mRatioW + 0.0056 * mRatioW;
            y2 = 0.017 * mRatioW * mAspectRatio - 0.0098 * mRatioW * mAspectRatio;
            drawLine(x1, y1, x2, y2, color, 2.);
        }
        else
        {
            x1 = xOffset + 0.15 * mRatioW;
            y1 = -0.017 * mRatioW * mAspectRatio;
            x2 = xOffset + 0.15 * mRatioW - 0.0056 * mRatioW;
            y2 = -0.017 * mRatioW * mAspectRatio + 0.0098 * mRatioW * mAspectRatio;
            drawLine(x1, y1, x2, y2, color, 2.);
            x1 = xOffset + 0.15 * mRatioW;
            y1 = -0.017 * mRatioW * mAspectRatio;
            x2 = xOffset + 0.15 * mRatioW + 0.0056 * mRatioW;
            y2 = -0.017 * mRatioW * mAspectRatio + 0.0098 * mRatioW * mAspectRatio;
            drawLine(x1, y1, x2, y2, color, 2.);
        }
    }
}


void Gles2Hud::drawSpeed(float horizontalSpeed, const float color[4])
{
    char strSpeed[20];
    sprintf(strSpeed, "%.1fm/s", horizontalSpeed);

    float xOffset = -mHudCentralZoneSize * mRatioW;
    float height = mHudCentralZoneSize * mRatioW * mAspectRatio;
    float speedInterval = height / 20.;

    drawLine(xOffset, -height / 2., xOffset, height / 2., color, 2.);
    drawLine(xOffset, -height / 2., xOffset - 0.08 * mRatioW, -height / 2., color, 2.);
    drawLine(xOffset, height / 2., xOffset - 0.08 * mRatioW, height / 2., color, 2.);
    drawRect(xOffset - 0.03 * mRatioW, -0.017 * mRatioW * mAspectRatio, xOffset - 0.03 * mRatioW - 0.1 * mRatioW, 0.017 * mRatioW * mAspectRatio, color, 2.);
    drawLine(xOffset, 0., xOffset - 0.03 * mRatioW, -0.017 * mRatioW * mAspectRatio, color, 2.);
    drawLine(xOffset, 0., xOffset - 0.03 * mRatioW, 0.017 * mRatioW * mAspectRatio, color, 2.);

    float y = (ceil(horizontalSpeed) - horizontalSpeed) * speedInterval;
    int spdInt = ((int)ceil(horizontalSpeed));
    int spdMod5 = spdInt % 5;
    while (y < height / 2.)
    {
        drawLine(xOffset, y, xOffset - ((spdMod5 == 0) ? 0.03 * mRatioW : 0.01 * mRatioW), y, color, 2.);
        if ((!spdMod5) && (y > -height / 2. + 0.017 * mRatioW * mAspectRatio) && (y < height / 2. - 0.017 * mRatioW * mAspectRatio)
                && (!((y > -0.017 * mRatioW * mAspectRatio) && (y < 0.017 * mRatioW * mAspectRatio))))
        {
            sprintf(strSpeed, "%d", spdInt);
            //drawText(QRect(drawWidth / 80, y - drawWidth / 180, drawWidth / 128 * strlen(strAltitude), drawWidth / 80), Qt::AlignCenter, strAltitude, NULL);
        }
        y += speedInterval;
        spdInt++;
        spdMod5 = spdInt % 5;
    }
    y = -(horizontalSpeed - floor(horizontalSpeed)) * speedInterval;
    spdInt = ((int)floor(horizontalSpeed));
    spdMod5 = spdInt % 5;
    while (y > -height / 2.)
    {
        drawLine(xOffset, y, xOffset - ((spdMod5 == 0) ? 0.03 * mRatioW : 0.01 * mRatioW), y, color, 2.);
        if ((!spdMod5) && (y > -height / 2. + 0.017 * mRatioW * mAspectRatio) && (y < height / 2. - 0.017 * mRatioW * mAspectRatio)
                && (!((y > -0.017 * mRatioW * mAspectRatio) && (y < 0.017 * mRatioW * mAspectRatio))))
        {
            sprintf(strSpeed, "%d", spdInt);
            //drawText(QRect(drawWidth / 80, y - drawWidth / 180, drawWidth / 128 * strlen(strAltitude), drawWidth / 80), Qt::AlignCenter, strAltitude, NULL);
        }
        y -= speedInterval;
        spdInt--;
        spdMod5 = spdInt % 5;
    }
}


void Gles2Hud::drawControllerRadar(double distance, double bearing, float controllerYaw, float droneYaw, float controllerRadarAngle, const float color[4])
{
    float width = 0.08 * mRatioW;
    float xOffset = mHudRadarZoneHOffset * mRatioW;
    float yOffset = mHudRadarZoneVOffset * mRatioH;
    float x1, y1, x2, y2;

    drawEllipse(xOffset, yOffset, width, width * mAspectRatio, 100, color, 2.);
    x1 = xOffset;
    y1 = yOffset + width * mAspectRatio;
    x2 = xOffset;
    y2 = yOffset + (width + 0.008 * mRatioW) * mAspectRatio;
    drawLine(x1, y1, x2, y2, color, 2.);

    if (distance > 50.)
    {
        x1 = xOffset - width / 3. * sinf(controllerRadarAngle / 2.);
        y1 = yOffset + width / 3. * cosf(controllerRadarAngle / 2.) * mAspectRatio;
        x2 = xOffset - width * sinf(controllerRadarAngle / 2.);
        y2 = yOffset + width * cosf(controllerRadarAngle / 2.) * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        x1 = xOffset + width / 3. * sinf(controllerRadarAngle / 2.);
        y1 = yOffset + width / 3. * cosf(controllerRadarAngle / 2.) * mAspectRatio;
        x2 = xOffset + width * sinf(controllerRadarAngle / 2.);
        y2 = yOffset + width * cosf(controllerRadarAngle / 2.) * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
    }
}


void Gles2Hud::drawRecordTimeline(uint64_t currentTime, uint64_t duration, const float color[4])
{
    float xOffset = mHudRightZoneHOffset * mRatioW;
    float yOffset = mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio;
    float width = 0.4 * mRatioW;
    float height = 0.015 * mRatioW * mAspectRatio;
    float x1, y1, x2, y2;
    float cw = 0., rw = 0.;

    uint64_t remainingTime = duration - currentTime;
    unsigned int cHrs = 0, cMin = 0, cSec = 0, cMsec = 0;
    unsigned int rHrs = 0, rMin = 0, rSec = 0, rMsec = 0;
    unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
    pdraw_friendlyTimeFromUs(currentTime, &cHrs, &cMin, &cSec, &cMsec);
    pdraw_friendlyTimeFromUs(remainingTime, &rHrs, &rMin, &rSec, &rMsec);
    pdraw_friendlyTimeFromUs(duration, &dHrs, &dMin, &dSec, &dMsec);
    char str[20];
    if (dHrs)
        snprintf(str, sizeof(str), "+%02d:%02d:%02d.%03d", cHrs, cMin, cSec, cMsec);
    else
        snprintf(str, sizeof(str), "+%02d:%02d.%03d", cMin, cSec, cMsec);
    getTextDimensions(str, 0.15 * mRatioW, 1., mAspectRatio, &cw, NULL);
    cw += 0.012;
    if (dHrs)
        snprintf(str, sizeof(str), "-%02d:%02d:%02d.%03d", rHrs, rMin, rSec, rMsec);
    else
        snprintf(str, sizeof(str), "-%02d:%02d.%03d", rMin, rSec, rMsec);
    getTextDimensions(str, 0.15 * mRatioW, 1., mAspectRatio, &rw, NULL);
    rw += 0.01;

    x1 = xOffset - rw;
    y1 = y2 = yOffset;
    x2 = xOffset - width + cw;
    drawLine(x1, y1, x2, y2, color, 2.);
    x1 = x2 = xOffset - rw;
    y1 = yOffset + height / 2.;
    y2 = yOffset - height / 2.;
    drawLine(x1, y1, x2, y2, color, 2.);
    x1 = x2 = xOffset - width + cw;
    y1 = yOffset + height / 2.;
    y2 = yOffset - height / 2.;
    drawLine(x1, y1, x2, y2, color, 2.);
    x1 = x2 = xOffset - rw - (1. - (float)currentTime / (float)duration) * (width - rw - cw);
    y1 = yOffset + height / 2.;
    y2 = yOffset - height / 2.;
    drawLine(x1, y1, x2, y2, color, 2.);
}


void Gles2Hud::drawRecordingStatus(uint64_t recordingDuration, const float color[4])
{
    float xOffset = mHudRightZoneHOffset * mRatioW;
    float yOffset = mHudRollZoneVOffset * mRatioH + 0.12 * mRatioW * mAspectRatio;
    float recSize = 0.008 * mRatioW;
    float w = 0.;

    if (recordingDuration > 0)
    {
        unsigned int dHrs = 0, dMin = 0, dSec = 0, dMsec = 0;
        pdraw_friendlyTimeFromUs(recordingDuration, &dHrs, &dMin, &dSec, &dMsec);
        char str[20];
        if (dHrs)
            snprintf(str, sizeof(str), "REC %02d:%02d:%02d", dHrs, dMin, dSec);
        else
            snprintf(str, sizeof(str), "REC %02d:%02d", dMin, dSec);
        getTextDimensions(str, 0.15 * mRatioW, 1., mAspectRatio, &w, NULL);
        w += 0.02;
        drawEllipseFilled(xOffset - w - recSize / 2., yOffset, recSize, recSize * mAspectRatio, 30, color);
    }
    else
    {
        getTextDimensions("REC", 0.15 * mRatioW, 1., mAspectRatio, &w, NULL);
        w += 0.03;
        drawEllipse(xOffset - w - recSize / 2., yOffset, recSize, recSize * mAspectRatio, 30, color, 2.);
        float x1, y1, x2, y2;
        x1 = xOffset + 0.008 * mRatioW;
        y1 = yOffset + recSize * mAspectRatio + 0.008 * mRatioW * mAspectRatio;
        x2 = x1 - w - recSize - 0.016 * mRatioW;
        y2 = y1 - recSize * 2. * mAspectRatio - 0.016 * mRatioW * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        drawLine(x1, y2, x2, y1, color, 2.);
    }
}


void Gles2Hud::drawFlightPathVector(const euler_t *frame, float speedTheta, float speedPsi, const float color[4])
{
    float x = (speedPsi - frame->psi) / mHfov * 2. * mRatioW;
    float y = (speedTheta - frame->theta) / mVfov * 2. * mRatioH;
    float x1, y1, x2, y2, tx, ty;
    float rotation = frame->phi;

    if ((x > -mRatioW) && (x < mRatioW) && (y > -mRatioH) && (y < mRatioH))
    {
        drawEllipse(x, y, 0.02 * mRatioW, 0.02 * mRatioW * mAspectRatio, 40, color, 2.);
        tx = 0.;
        ty = 0.02;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        tx = 0.;
        ty = 0.03;
        x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        tx = -0.02;
        ty = 0.;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        tx = -0.03;
        ty = 0.;
        x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        tx = 0.02;
        ty = 0.;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        tx = 0.03;
        ty = 0.;
        x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mRatioW;
        y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mRatioW * mAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
    }
}


void Gles2Hud::drawPositionPin(const euler_t *frame, double bearing, double elevation, const float color[4])
{
    float x = (bearing - frame->psi) / mHfov * 2. * mScaleW;
    float y = (elevation - frame->theta) / mVfov * 2. * mScaleH;
    float x1, y1, x2, y2, tx, ty;
    float rotation = frame->phi;

    if ((x > -mScaleW) && (x < mScaleW) && (y > -mScaleH) && (y < mScaleH))
    {
        tx = 0.;
        ty = 0.08;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mScaleW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mScaleH * mVideoAspectRatio;
        drawEllipse(x1, y1,
            0.02 * mScaleW, 0.02 * mScaleH * mVideoAspectRatio, 40, color, 2.);
        tx = 0.;
        ty = 0.04;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mScaleW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mScaleH * mVideoAspectRatio;
        tx = -0.01;
        ty = 0.05;
        x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mScaleW;
        y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mScaleH * mVideoAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
        tx = 0.;
        ty = 0.04;
        x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mScaleW;
        y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mScaleH * mVideoAspectRatio;
        tx = 0.01;
        ty = 0.05;
        x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) * mScaleW;
        y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) * mScaleH * mVideoAspectRatio;
        drawLine(x1, y1, x2, y2, color, 2.);
    }
}

}

#endif /* USE_GLES2 */
