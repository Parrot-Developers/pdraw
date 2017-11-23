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

#ifndef _PDRAW_SETTINGS_HPP_
#define _PDRAW_SETTINGS_HPP_

#include <inttypes.h>
#include <math.h>
#include <pdraw/pdraw_defs.h>


#define SETTINGS_HUD_CONTROLLER_RADAR_ANGLE     (M_PI / 3.)
#define SETTINGS_DISPLAY_XDPI                   (200.0f)
#define SETTINGS_DISPLAY_YDPI                   (200.0f)
#define SETTINGS_DISPLAY_DEVICE_MARGIN          (4.0f)
#define SETTINGS_HMD_IPD                        (63.0f)
#define SETTINGS_HMD_SCALE                      (0.75f)
#define SETTINGS_HMD_PAN_H                      (0.0f)
#define SETTINGS_HMD_PAN_V                      (0.0f)


namespace Pdraw
{


class Settings
{
public:

    Settings();

    ~Settings();

    float getControllerRadarAngle() { return mControllerRadarAngle; };
    void setControllerRadarAngle(float angle) { mControllerRadarAngle = angle; };

    void getDisplayScreenSettings(float *xdpi, float *ydpi, float *deviceMargin);
    void setDisplayScreenSettings(float xdpi, float ydpi, float deviceMargin);

    void getHmdDistorsionCorrectionSettings(enum pdraw_hmd_model *hmdModel, float *ipd, float *scale, float *panH, float *panV);
    void setHmdDistorsionCorrectionSettings(enum pdraw_hmd_model hmdModel, float ipd, float scale, float panH, float panV);

private:

    float mControllerRadarAngle;
    float mDisplayXdpi;
    float mDisplayYdpi;
    float mDisplayDeviceMargin;
    enum pdraw_hmd_model mHmdModel;
    float mHmdIpd;
    float mHmdScale;
    float mHmdPanH;
    float mHmdPanV;
};

}

#endif /* !_PDRAW_SETTINGS_HPP_ */
