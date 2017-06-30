/**
 * @file pdraw_settings.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - user settings
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

#include "pdraw_settings.hpp"
#include "pdraw_gles2_hmd.hpp"
#include "pdraw_gles2_hud.hpp"


namespace Pdraw
{


Settings::Settings()
{
    mControllerRadarAngle = GLES2_HUD_DEFAULT_CONTROLLER_RADAR_ANGLE;
    mHmdXdpi = GLES2_HMD_DEFAULT_XDPI;
    mHmdYdpi = GLES2_HMD_DEFAULT_YDPI;
    mHmdDeviceMargin = GLES2_HMD_DEFAULT_DEVICE_MARGIN;
    mHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
    mHmdIpd = GLES2_HMD_DEFAULT_IPD;
    mHmdScale = GLES2_HMD_DEFAULT_SCALE;
    mHmdPanH = GLES2_HMD_DEFAULT_PAN_H;
    mHmdPanV = GLES2_HMD_DEFAULT_PAN_V;
}


Settings::~Settings()
{

}


void Settings::getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin)
{
    if (xdpi)
        *xdpi = mHmdXdpi;
    if (ydpi)
        *ydpi = mHmdYdpi;
    if (deviceMargin)
        *deviceMargin = mHmdDeviceMargin;
}


void Settings::setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin)
{
    mHmdXdpi = xdpi;
    mHmdYdpi = ydpi;
    mHmdDeviceMargin = deviceMargin;
}


void Settings::getHmdDistorsionCorrectionSettings(pdraw_hmd_model_t *hmdModel, float *ipd, float *scale, float *panH, float *panV)
{
    if (hmdModel)
        *hmdModel = mHmdModel;
    if (ipd)
        *ipd = mHmdIpd;
    if (scale)
        *scale = mHmdScale;
    if (panH)
        *panH = mHmdPanH;
    if (panV)
        *panV = mHmdPanV;
}


void Settings::setHmdDistorsionCorrectionSettings(pdraw_hmd_model_t hmdModel, float ipd, float scale, float panH, float panV)
{
    mHmdModel = hmdModel;
    mHmdIpd = ipd;
    mHmdScale = scale;
    mHmdPanH = panH;
    mHmdPanV = panV;
}

}
