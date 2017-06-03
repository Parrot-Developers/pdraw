/**
 * @file pdraw_gles2_hmd.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - OpenGL ES 2.0 HMD distorsion correction
 * @date 11/02/2017
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

/* Extracted from FPVToolbox by Frédéric Bertolus
 * https://github.com/niavok/fpvtoolbox
 * and converted from Java to C++ */

#ifndef _PDRAW_GLES2_HMD_HPP_
#define _PDRAW_GLES2_HMD_HPP_

#ifdef USE_GLES2

#if defined(BCM_VIDEOCORE)
    #include <GLES2/gl2.h>
#else
    #define GLFW_INCLUDE_ES2
    #include <GLFW/glfw3.h>
#endif

#define GLES2_HMD_INCH_TO_MILLIMETER     (25.4f)
#define GLES2_HMD_OFFSET                 (34.66f)
#define GLES2_HMD_DEFAULT_XDPI           (200.0f)
#define GLES2_HMD_DEFAULT_YDPI           (200.0f)
#define GLES2_HMD_DEFAULT_DEVICE_MARGIN  (4.0f)
#define GLES2_HMD_DEFAULT_IPD            (63.0f)
#define GLES2_HMD_DEFAULT_SCALE          (0.75f)
#define GLES2_HMD_DEFAULT_PAN_H          (0.0f)
#define GLES2_HMD_DEFAULT_PAN_V          (0.0f)

#define GLES2_HMD_TEX_UNIT_COUNT 1


namespace Pdraw
{


class Gles2HmdEye
{
public:

    Gles2HmdEye(unsigned int firstTexUnit, float scale, float panH, float panV,
               float metricsWidth, float metricsHeight,
               float eyeOffsetX, float eyeOffsetY);

    ~Gles2HmdEye();

    int renderEye(GLuint texture, unsigned int textureWidth, unsigned int textureHeight);

private:

    unsigned int mRotation;
    float mScale;
    float mPanH;
    float mPanV;
    float mMetricsWidth;
    float mMetricsHeight;
    float mEyeOffsetX;
    float mEyeOffsetY;

    unsigned int mFirstTexUnit;
    GLint mProgram;
    GLint mIndicesBufferHandle;
    GLint mPositionBufferHandle;
    GLint mColorBufferHandle;
    GLint mTexCoord0BufferHandle;
    GLint mTexCoord1BufferHandle;
    GLint mTexCoord2BufferHandle;
    GLint mProgramTexture;
    GLint mProgramEyeToSourceUVScale;
    GLint mProgramEyeToSourceUVOffset;
    GLint mProgramEyeToSourceScale;
    GLint mProgramEyeToSourceOffset;
    GLint mProgramChromaticAberrationCorrection;
    GLint mProgramRotation;
    GLint mProgramLensLimits;
    GLint mProgramPosition;
    GLint mProgramColor;
    GLint mProgramTexCoord0;
    GLint mProgramTexCoord1;
    GLint mProgramTexCoord2;
};


class Gles2Hmd
{
public:

    Gles2Hmd(unsigned int firstTexUnit, unsigned int width, unsigned int height,
            float xdpi = GLES2_HMD_DEFAULT_XDPI, float ydpi = GLES2_HMD_DEFAULT_YDPI,
            float deviceMargin = GLES2_HMD_DEFAULT_DEVICE_MARGIN,
            float ipd = GLES2_HMD_DEFAULT_IPD, float scale = GLES2_HMD_DEFAULT_SCALE,
            float panH = GLES2_HMD_DEFAULT_PAN_H, float panV = GLES2_HMD_DEFAULT_PAN_V);

    ~Gles2Hmd();

    static int getTexUnitCount() { return GLES2_HMD_TEX_UNIT_COUNT; }

    int renderHmd(GLuint texture, unsigned int textureWidth, unsigned int textureHeight);

private:

    float mDeviceMargin;
    float mIpd;
    float mScale;
    float mPanH;
    float mPanV;
    float mMetricsWidth;
    float mMetricsHeight;
    Gles2HmdEye *mLeftEye;
    Gles2HmdEye *mRightEye;
};

}

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_HMD_HPP_ */
