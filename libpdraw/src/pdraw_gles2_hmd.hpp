/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 HMD distortion correction
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Extracted from FPVToolbox by Frédéric Bertolus
 * https://github.com/niavok/fpvtoolbox
 * and translated from Java to C++
 */

#ifndef _PDRAW_GLES2_HMD_HPP_
#define _PDRAW_GLES2_HMD_HPP_

#ifdef USE_GLES2

#	include "pdraw_gles2_common.hpp"
#	include "pdraw_session.hpp"
#	include "pdraw_settings.hpp"

#	include <pdraw/pdraw_defs.h>

namespace Pdraw {


#	define GLES2_HMD_TEX_UNIT_COUNT 1


class Gles2HmdEye {
public:
	Gles2HmdEye(unsigned int firstTexUnit,
		    enum pdraw_hmd_model hmdModel,
		    float scale,
		    float panH,
		    float panV,
		    float metricsWidth,
		    float metricsHeight,
		    float eyeOffsetX,
		    float eyeOffsetY);

	~Gles2HmdEye(void);

	int renderEye(GLuint texture,
		      unsigned int textureWidth,
		      unsigned int textureHeight);

private:
	unsigned int mRotation;
	enum pdraw_hmd_model mHmdModel;
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


class Gles2Hmd {
public:
	Gles2Hmd(Session *session,
		 unsigned int firstTexUnit,
		 unsigned int width,
		 unsigned int height,
		 float ipdOffset = 0.f,
		 float xOffset = 0.f,
		 float yOffset = 0.f);

	~Gles2Hmd(void);

	static int getTexUnitCount(void)
	{
		return GLES2_HMD_TEX_UNIT_COUNT;
	}

	int renderHmd(GLuint texture,
		      unsigned int textureWidth,
		      unsigned int textureHeight);

private:
	Session *mSession;
	enum pdraw_hmd_model mHmdModel;
	float mHmdOffset;
	float mHmdIpd;
	float mIpdOffset;
	float mXOffset;
	float mYOffset;
	float mXdpi;
	float mYdpi;
	float mDeviceMarginTop;
	float mDeviceMarginBottom;
	float mDeviceMarginLeft;
	float mDeviceMarginRight;
	float mMetricsWidth;
	float mMetricsHeight;
	Gles2HmdEye *mLeftEye;
	Gles2HmdEye *mRightEye;
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_HMD_HPP_ */
