/**
 * @file pdraw_gles2_hmd.cpp
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

#include "pdraw_gles2_hmd.hpp"

#ifdef USE_GLES2

#define ULOG_TAG libpdraw
#include <ulog.h>


#define GLES2_HMD_INCH_TO_MILLIMETER     (25.4f)
#define GLES2_HMD_OFFSET                 (34.66f)


extern const float pdraw_gles2HmdCockpitglassesColors[14884];
extern const uint32_t pdraw_gles2HmdCockpitglassesIndices[21600];
extern const float pdraw_gles2HmdCockpitglassesPositions[7442];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsRed[7450];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsGreen[7450];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsBlue[7450];

extern const float pdraw_gles2HmdCockpitglasses2Colors[14884];
extern const uint32_t pdraw_gles2HmdCockpitglasses2Indices[21600];
extern const float pdraw_gles2HmdCockpitglasses2Positions[7442];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsRed[7450];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsGreen[7450];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsBlue[7450];

extern const GLchar *pdraw_gles2HmdVertexShader;
extern const GLchar *pdraw_gles2HmdFragmentShader;


namespace Pdraw
{


Gles2HmdEye::Gles2HmdEye(unsigned int firstTexUnit, enum pdraw_hmd_model hmdModel,
                         float scale, float panH, float panV,
                         float metricsWidth, float metricsHeight,
                         float eyeOffsetX, float eyeOffsetY)
{
    mFirstTexUnit = firstTexUnit;
    mHmdModel = hmdModel;
    mRotation = 0;
    mScale = scale;
    mPanH = panH;
    mPanV = panV;
    mMetricsWidth = metricsWidth;
    mMetricsHeight = metricsHeight;
    mEyeOffsetX = eyeOffsetX;
    mEyeOffsetY = eyeOffsetY;

    GLint vertexShader, fragmentShader;
    GLint success = 0;
    int ret = 0;

    if (ret == 0)
    {
        vertexShader = glCreateShader(GL_VERTEX_SHADER);
        if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2HmdEye: failed to create vertex shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(vertexShader, 1, &pdraw_gles2HmdVertexShader, NULL);
        glCompileShader(vertexShader);
        GLint success = 0;
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            ULOGE("Gles2HmdEye: vertex shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        if ((fragmentShader == 0) || (fragmentShader == GL_INVALID_ENUM))
        {
            ULOGE("Gles2HmdEye: failed to create fragment shader");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        glShaderSource(fragmentShader, 1, &pdraw_gles2HmdFragmentShader, NULL);
        glCompileShader(fragmentShader);
        GLint success = 0;
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            ULOGE("Gles2HmdEye: fragment shader compilation failed '%s'", infoLog);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* Link shaders */
        mProgram = glCreateProgram();
        glAttachShader(mProgram, vertexShader);
        glAttachShader(mProgram, fragmentShader);
        glLinkProgram(mProgram);
        glGetProgramiv(mProgram, GL_LINK_STATUS, &success);
        if (!success)
        {
            GLchar infoLog[512];
            glGetProgramInfoLog(mProgram, 512, NULL, infoLog);
            ULOGE("Gles2HmdEye: program link failed '%s'", infoLog);
            ret = -1;
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    if (ret == 0)
    {
        GLuint buffer[6];
        glGenBuffers(6, buffer);

        //TODO: support different HMD models
        switch (mHmdModel)
        {
        default:
        case PDRAW_HMD_MODEL_COCKPITGLASSES:
            mIndicesBufferHandle = buffer[0];
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBufferHandle);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesIndices), pdraw_gles2HmdCockpitglassesIndices, GL_STATIC_DRAW);

            mPositionBufferHandle = buffer[1];
            glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesPositions), pdraw_gles2HmdCockpitglassesPositions, GL_STATIC_DRAW);

            mColorBufferHandle = buffer[2];
            glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesColors), pdraw_gles2HmdCockpitglassesColors, GL_STATIC_DRAW);

            mTexCoord0BufferHandle = buffer[3];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesTexCoordsRed), pdraw_gles2HmdCockpitglassesTexCoordsRed, GL_STATIC_DRAW);

            mTexCoord1BufferHandle = buffer[4];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesTexCoordsGreen), pdraw_gles2HmdCockpitglassesTexCoordsGreen, GL_STATIC_DRAW);

            mTexCoord2BufferHandle = buffer[5];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglassesTexCoordsBlue), pdraw_gles2HmdCockpitglassesTexCoordsBlue, GL_STATIC_DRAW);
            break;
        case PDRAW_HMD_MODEL_COCKPITGLASSES_2:
            mIndicesBufferHandle = buffer[0];
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBufferHandle);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2Indices), pdraw_gles2HmdCockpitglasses2Indices, GL_STATIC_DRAW);

            mPositionBufferHandle = buffer[1];
            glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2Positions), pdraw_gles2HmdCockpitglasses2Positions, GL_STATIC_DRAW);

            mColorBufferHandle = buffer[2];
            glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2Colors), pdraw_gles2HmdCockpitglasses2Colors, GL_STATIC_DRAW);

            mTexCoord0BufferHandle = buffer[3];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsRed), pdraw_gles2HmdCockpitglasses2TexCoordsRed, GL_STATIC_DRAW);

            mTexCoord1BufferHandle = buffer[4];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsGreen), pdraw_gles2HmdCockpitglasses2TexCoordsGreen, GL_STATIC_DRAW);

            mTexCoord2BufferHandle = buffer[5];
            glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle);
            glBufferData(GL_ARRAY_BUFFER, sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsBlue), pdraw_gles2HmdCockpitglasses2TexCoordsBlue, GL_STATIC_DRAW);
            break;
        }
    }

    if (ret == 0)
    {
        mProgramTexture = glGetUniformLocation(mProgram, "Texture0");
        if (mProgramTexture < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'Texture0' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramEyeToSourceUVScale = glGetUniformLocation(mProgram, "EyeToSourceUVScale");
        if (mProgramEyeToSourceUVScale < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceUVScale' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramEyeToSourceUVOffset = glGetUniformLocation(mProgram, "EyeToSourceUVOffset");
        if (mProgramEyeToSourceUVOffset < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceUVOffset' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramEyeToSourceScale = glGetUniformLocation(mProgram, "EyeToSourceScale");
        if (mProgramEyeToSourceScale < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceScale' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramEyeToSourceOffset = glGetUniformLocation(mProgram, "EyeToSourceOffset");
        if (mProgramEyeToSourceOffset < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceOffset' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramChromaticAberrationCorrection = glGetUniformLocation(mProgram, "ChromaticAberrationCorrection");
        if (mProgramChromaticAberrationCorrection < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'ChromaticAberrationCorrection' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramRotation = glGetUniformLocation(mProgram, "Rotation");
        if (mProgramRotation < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'Rotation' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramLensLimits = glGetUniformLocation(mProgram, "LensLimits");
        if (mProgramLensLimits < 0)
        {
            ULOGE("Gles2HmdEye: failed to get uniform 'LensLimits' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramPosition = glGetAttribLocation(mProgram, "Position");
        if (mProgramPosition < 0)
        {
            ULOGE("Gles2HmdEye: failed to get attribute 'Position' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramColor = glGetAttribLocation(mProgram, "Color");
        if (mProgramColor < 0)
        {
            ULOGE("Gles2HmdEye: failed to get attribute 'Color' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramTexCoord0 = glGetAttribLocation(mProgram, "TexCoord0");
        if (mProgramTexCoord0 < 0)
        {
            ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord0' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramTexCoord1 = glGetAttribLocation(mProgram, "TexCoord1");
        if (mProgramTexCoord1 < 0)
        {
            ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord1' location loading program");
        }
    }

    if (ret == 0)
    {
        mProgramTexCoord2 = glGetAttribLocation(mProgram, "TexCoord2");
        if (mProgramTexCoord2 < 0)
        {
            ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord2' location loading program");
        }
    }
}


Gles2HmdEye::~Gles2HmdEye()
{
    GLuint buffer[6];
    buffer[0] = mIndicesBufferHandle;
    buffer[1] = mPositionBufferHandle;
    buffer[2] = mColorBufferHandle;
    buffer[3] = mTexCoord0BufferHandle;
    buffer[4] = mTexCoord1BufferHandle;
    buffer[5] = mTexCoord2BufferHandle;
    glDeleteBuffers(6, buffer);
    glDeleteProgram(mProgram);
}


int Gles2HmdEye::renderEye(GLuint texture, unsigned int textureWidth, unsigned int textureHeight)
{
    glUseProgram(mProgram);

    glActiveTexture(GL_TEXTURE0 + mFirstTexUnit);
    //glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,  mRenderer.getGLSurfaceTexture());
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(mProgramTexture, mFirstTexUnit);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBufferHandle);

    glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle);
    glEnableVertexAttribArray(mProgramPosition);
    glVertexAttribPointer(mProgramPosition, 2, GL_FLOAT, false, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle);
    glEnableVertexAttribArray(mProgramColor);
    glVertexAttribPointer(mProgramColor, 4, GL_FLOAT, false, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle);
    glEnableVertexAttribArray(mProgramTexCoord0);
    glVertexAttribPointer(mProgramTexCoord0, 2, GL_FLOAT, false, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle);
    glEnableVertexAttribArray(mProgramTexCoord1);
    glVertexAttribPointer(mProgramTexCoord1, 2, GL_FLOAT, false, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle);
    glEnableVertexAttribArray(mProgramTexCoord2);
    glVertexAttribPointer(mProgramTexCoord2, 2, GL_FLOAT, false, 0, 0);

    float ratio;
    if ((mRotation == 90) || (mRotation == 270))
    {
        ratio = (float)textureHeight / (float)textureWidth;
    }
    else
    {
        ratio = (float)textureWidth / (float)textureHeight;
    }

    if (ratio > 1.)
    {
        glUniform2f(mProgramEyeToSourceUVScale, mScale, mScale * ratio);
    }
    else
    {
        glUniform2f(mProgramEyeToSourceUVScale, mScale / ratio, mScale);
    }

    glUniform2f(mProgramEyeToSourceUVOffset, mPanH, mPanV);
    glUniform1i(mProgramChromaticAberrationCorrection, 0);
    glUniform1i(mProgramRotation, mRotation);
    glUniform1i(mProgramLensLimits, 0);
    glUniform2f(mProgramEyeToSourceScale, 2.f / mMetricsWidth, -2.f / mMetricsHeight);
    glUniform2f(mProgramEyeToSourceOffset, 2.f * mEyeOffsetX / mMetricsWidth, 2.f * mEyeOffsetY / mMetricsHeight - 1.f);

    glDrawElements(GL_TRIANGLES, sizeof(pdraw_gles2HmdCockpitglassesIndices) / sizeof(float), GL_UNSIGNED_INT, 0);

    glDisableVertexAttribArray(mProgramPosition);
    glDisableVertexAttribArray(mProgramColor);
    glDisableVertexAttribArray(mProgramTexCoord0);
    glDisableVertexAttribArray(mProgramTexCoord1);
    glDisableVertexAttribArray(mProgramTexCoord2);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return 0;
}


Gles2Hmd::Gles2Hmd(unsigned int firstTexUnit, unsigned int width, unsigned int height,
                   enum pdraw_hmd_model hmdModel, float xdpi, float ydpi, float deviceMargin,
                   float ipd, float scale, float panH, float panV)
{
    mHmdModel = hmdModel;
    mDeviceMargin = deviceMargin;
    mIpd = ipd;
    mScale = scale;
    mPanH = panH;
    mPanV = panV;

    mMetricsWidth = (float)width / xdpi * GLES2_HMD_INCH_TO_MILLIMETER;
    mMetricsHeight = (float)height / ydpi * GLES2_HMD_INCH_TO_MILLIMETER;

    mLeftEye = new Gles2HmdEye(firstTexUnit, mHmdModel, mScale, mPanH, mPanV, mMetricsWidth, mMetricsHeight,
                               -mIpd / 2.f, GLES2_HMD_OFFSET - mDeviceMargin);
    mRightEye = new Gles2HmdEye(firstTexUnit, mHmdModel, mScale, mPanH, mPanV, mMetricsWidth, mMetricsHeight,
                                mIpd / 2.f, GLES2_HMD_OFFSET - mDeviceMargin);
}


Gles2Hmd::~Gles2Hmd()
{
    if (mLeftEye) delete(mLeftEye);
    if (mRightEye) delete(mRightEye);
}


int Gles2Hmd::renderHmd(GLuint texture, unsigned int textureWidth, unsigned int textureHeight)
{
    int ret = 0;

    if ((ret == 0) && (mLeftEye))
    {
        ret = mLeftEye->renderEye(texture, textureWidth, textureHeight);
    }

    if ((ret == 0) && (mRightEye))
    {
        ret = mRightEye->renderEye(texture, textureWidth, textureHeight);
    }

    return ret;
}

}

#endif /* USE_GLES2 */
