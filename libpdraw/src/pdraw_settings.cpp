/**
 * Parrot Drones Awesome Video Viewer Library
 * User settings
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

#include "pdraw_settings.hpp"


namespace Pdraw
{


Settings::Settings()
{
    mControllerRadarAngle = SETTINGS_HUD_CONTROLLER_RADAR_ANGLE;
    mDisplayXdpi = SETTINGS_DISPLAY_XDPI;
    mDisplayYdpi = SETTINGS_DISPLAY_YDPI;
    mDisplayDeviceMargin = SETTINGS_DISPLAY_DEVICE_MARGIN;
    mHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
    mHmdIpd = SETTINGS_HMD_IPD;
    mHmdScale = SETTINGS_HMD_SCALE;
    mHmdPanH = SETTINGS_HMD_PAN_H;
    mHmdPanV = SETTINGS_HMD_PAN_V;
}


Settings::~Settings()
{

}


void Settings::getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin)
{
    if (xdpi)
        *xdpi = mDisplayXdpi;
    if (ydpi)
        *ydpi = mDisplayYdpi;
    if (deviceMargin)
        *deviceMargin = mDisplayDeviceMargin;
}


void Settings::setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin)
{
    mDisplayXdpi = xdpi;
    mDisplayYdpi = ydpi;
    mDisplayDeviceMargin = deviceMargin;
}


void Settings::getHmdDistorsionCorrectionSettings(enum pdraw_hmd_model *hmdModel, float *ipd, float *scale, float *panH, float *panV)
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


void Settings::setHmdDistorsionCorrectionSettings(enum pdraw_hmd_model hmdModel, float ipd, float scale, float panH, float panV)
{
    mHmdModel = hmdModel;
    mHmdIpd = ipd;
    mHmdScale = scale;
    mHmdPanH = panH;
    mHmdPanV = panV;
}

}
