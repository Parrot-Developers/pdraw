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

#define ULOG_TAG pdraw_gles2hmd
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_gles2_hmd.hpp"

#ifdef USE_GLES2

namespace Pdraw {


#	define GLES2_HMD_INCH_TO_MILLIMETER (25.4f)
#	define GLES2_HMD_OFFSET_COCKPITGLASSES (34.66f)
#	define GLES2_HMD_OFFSET_COCKPITGLASSES_2 (41.0f)
#	define GLES2_HMD_IPD_COCKPITGLASSES (63.0f)
#	define GLES2_HMD_IPD_COCKPITGLASSES_2 (67.0f)


extern const float pdraw_gles2HmdColors[14884];
extern const uint32_t pdraw_gles2HmdIndices[21600];
extern const float pdraw_gles2HmdTexCoords[7442];
extern const float pdraw_gles2HmdTexCoordsCockpitglassesRed[7442];
extern const float pdraw_gles2HmdTexCoordsCockpitglassesBlue[7442];
extern const float pdraw_gles2HmdPositionsCockpitglasses[7442];
extern const float pdraw_gles2HmdPositionsCockpitglasses2[7442];

extern const GLchar *pdraw_gles2HmdVertexShader;
extern const GLchar *pdraw_gles2HmdFragmentShader;


Gles2HmdEye::Gles2HmdEye(unsigned int firstTexUnit,
			 enum pdraw_hmd_model hmdModel,
			 float scale,
			 float panH,
			 float panV,
			 float metricsWidth,
			 float metricsHeight,
			 float eyeOffsetX,
			 float eyeOffsetY)
{
	mRotation = 0;
	mHmdModel = hmdModel;
	mScale = scale;
	mPanH = panH;
	mPanV = panV;
	mMetricsWidth = metricsWidth;
	mMetricsHeight = metricsHeight;
	mEyeOffsetX = eyeOffsetX;
	mEyeOffsetY = eyeOffsetY;
	mFirstTexUnit = firstTexUnit;
	mProgram = 0;
	mIndicesBufferHandle = 0;
	mPositionBufferHandle = 0;
	mColorBufferHandle = 0;
	mTexCoord0BufferHandle = 0;
	mTexCoord1BufferHandle = 0;
	mTexCoord2BufferHandle = 0;
	mProgramTexture = 0;
	mProgramEyeToSourceUVScale = 0;
	mProgramEyeToSourceUVOffset = 0;
	mProgramEyeToSourceScale = 0;
	mProgramEyeToSourceOffset = 0;
	mProgramChromaticAberrationCorrection = 0;
	mProgramRotation = 0;
	mProgramLensLimits = 0;
	mProgramPosition = 0;
	mProgramColor = 0;
	mProgramTexCoord0 = 0;
	mProgramTexCoord1 = 0;
	mProgramTexCoord2 = 0;

	GLint vertexShader = 0, fragmentShader = 0;
	GLint success = 0;

	GLCHK();

	GLuint buffer[6];
	memset(buffer, 0, sizeof(buffer));
	GLCHK(glGenBuffers(6, buffer));

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		goto err;
	}

	glShaderSource(vertexShader, 1, &pdraw_gles2HmdVertexShader, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
		ULOGE("vertex shader compilation failed '%s'", infoLog);
		goto err;
	}

	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShader == 0) || (fragmentShader == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(
		fragmentShader, 1, &pdraw_gles2HmdFragmentShader, nullptr);
	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram = glCreateProgram();
	glAttachShader(mProgram, vertexShader);
	glAttachShader(mProgram, fragmentShader);
	glLinkProgram(mProgram);
	glGetProgramiv(mProgram, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(mProgram, 512, nullptr, infoLog);
		ULOGE("program link failed '%s'", infoLog);
		goto err;
	}

	glDeleteShader(vertexShader);
	vertexShader = 0;
	glDeleteShader(fragmentShader);
	fragmentShader = 0;

	GLCHK();

	switch (mHmdModel) {
	default:
	case PDRAW_HMD_MODEL_COCKPITGLASSES:
		mIndicesBufferHandle = buffer[0];
		GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
				   mIndicesBufferHandle));
		GLCHK(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdIndices),
				   pdraw_gles2HmdIndices,
				   GL_STATIC_DRAW));

		mPositionBufferHandle = buffer[1];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle));
		GLCHK(glBufferData(
			GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdPositionsCockpitglasses),
			pdraw_gles2HmdPositionsCockpitglasses,
			GL_STATIC_DRAW));

		mColorBufferHandle = buffer[2];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdColors),
				   pdraw_gles2HmdColors,
				   GL_STATIC_DRAW));

		mTexCoord0BufferHandle = buffer[3];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
		GLCHK(glBufferData(
			GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdTexCoordsCockpitglassesRed),
			pdraw_gles2HmdTexCoordsCockpitglassesRed,
			GL_STATIC_DRAW));

		mTexCoord1BufferHandle = buffer[4];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdTexCoords),
				   pdraw_gles2HmdTexCoords,
				   GL_STATIC_DRAW));

		mTexCoord2BufferHandle = buffer[5];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
		GLCHK(glBufferData(
			GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdTexCoordsCockpitglassesBlue),
			pdraw_gles2HmdTexCoordsCockpitglassesBlue,
			GL_STATIC_DRAW));
		break;
	case PDRAW_HMD_MODEL_COCKPITGLASSES_2:
		mIndicesBufferHandle = buffer[0];
		GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
				   mIndicesBufferHandle));
		GLCHK(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdIndices),
				   pdraw_gles2HmdIndices,
				   GL_STATIC_DRAW));

		mPositionBufferHandle = buffer[1];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle));
		GLCHK(glBufferData(
			GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdPositionsCockpitglasses2),
			pdraw_gles2HmdPositionsCockpitglasses2,
			GL_STATIC_DRAW));

		mColorBufferHandle = buffer[2];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdColors),
				   pdraw_gles2HmdColors,
				   GL_STATIC_DRAW));

		mTexCoord0BufferHandle = buffer[3];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdTexCoords),
				   pdraw_gles2HmdTexCoords,
				   GL_STATIC_DRAW));

		mTexCoord1BufferHandle = buffer[4];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdTexCoords),
				   pdraw_gles2HmdTexCoords,
				   GL_STATIC_DRAW));

		mTexCoord2BufferHandle = buffer[5];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
				   sizeof(pdraw_gles2HmdTexCoords),
				   pdraw_gles2HmdTexCoords,
				   GL_STATIC_DRAW));
		break;
	}

	mProgramTexture = glGetUniformLocation(mProgram, "Texture0");
	if (mProgramTexture < 0)
		ULOGE("failed to get uniform location 'Texture0'");

	mProgramEyeToSourceUVScale =
		glGetUniformLocation(mProgram, "EyeToSourceUVScale");
	if (mProgramEyeToSourceUVScale < 0)
		ULOGE("failed to get uniform location 'EyeToSourceUVScale'");

	mProgramEyeToSourceUVOffset =
		glGetUniformLocation(mProgram, "EyeToSourceUVOffset");
	if (mProgramEyeToSourceUVOffset < 0)
		ULOGE("failed to get uniform location 'EyeToSourceUVOffset'");

	mProgramEyeToSourceScale =
		glGetUniformLocation(mProgram, "EyeToSourceScale");
	if (mProgramEyeToSourceScale < 0)
		ULOGE("failed to get uniform location 'EyeToSourceScale'");

	mProgramEyeToSourceOffset =
		glGetUniformLocation(mProgram, "EyeToSourceOffset");
	if (mProgramEyeToSourceOffset < 0)
		ULOGE("failed to get uniform location 'EyeToSourceOffset'");

	mProgramChromaticAberrationCorrection =
		glGetUniformLocation(mProgram, "ChromaticAberrationCorrection");
	if (mProgramChromaticAberrationCorrection < 0) {
		ULOGE("failed to get uniform location "
		      "'ChromaticAberrationCorrection'");
	}

	mProgramRotation = glGetUniformLocation(mProgram, "Rotation");
	if (mProgramRotation < 0)
		ULOGE("failed to get uniform location 'Rotation'");

	mProgramLensLimits = glGetUniformLocation(mProgram, "LensLimits");
	if (mProgramLensLimits < 0)
		ULOGE("failed to get uniform location 'LensLimits'");

	mProgramPosition = glGetAttribLocation(mProgram, "Position");
	if (mProgramPosition < 0)
		ULOGE("failed to get attribute location 'Position'");

	mProgramColor = glGetAttribLocation(mProgram, "Color");
	if (mProgramColor < 0)
		ULOGE("failed to get attribute location 'Color'");

	mProgramTexCoord0 = glGetAttribLocation(mProgram, "TexCoord0");
	if (mProgramTexCoord0 < 0)
		ULOGE("failed to get attribute location 'TexCoord0'");

	mProgramTexCoord1 = glGetAttribLocation(mProgram, "TexCoord1");
	if (mProgramTexCoord1 < 0)
		ULOGE("failed to get attribute location 'TexCoord1'");

	mProgramTexCoord2 = glGetAttribLocation(mProgram, "TexCoord2");
	if (mProgramTexCoord2 < 0)
		ULOGE("failed to get attribute location 'TexCoord2'");

	GLCHK();

	return;

err:
	if ((buffer[0]) || (buffer[1]) || (buffer[2]) || (buffer[3]) ||
	    (buffer[4]) || (buffer[5]))
		GLCHK(glDeleteBuffers(6, buffer));
	if (vertexShader > 0)
		GLCHK(glDeleteShader(vertexShader));
	if (fragmentShader > 0)
		GLCHK(glDeleteShader(fragmentShader));
	if (mProgram > 0)
		GLCHK(glDeleteProgram(mProgram));
	mProgram = 0;
	mIndicesBufferHandle = 0;
	mPositionBufferHandle = 0;
	mColorBufferHandle = 0;
	mTexCoord0BufferHandle = 0;
	mTexCoord1BufferHandle = 0;
	mTexCoord2BufferHandle = 0;
}


Gles2HmdEye::~Gles2HmdEye(void)
{
	int count = 0;
	GLuint buffer[6];
	if (mIndicesBufferHandle > 0)
		buffer[count++] = mIndicesBufferHandle;
	if (mPositionBufferHandle > 0)
		buffer[count++] = mPositionBufferHandle;
	if (mColorBufferHandle > 0)
		buffer[count++] = mColorBufferHandle;
	if (mTexCoord0BufferHandle > 0)
		buffer[count++] = mTexCoord0BufferHandle;
	if (mTexCoord1BufferHandle > 0)
		buffer[count++] = mTexCoord1BufferHandle;
	if (mTexCoord2BufferHandle > 0)
		buffer[count++] = mTexCoord2BufferHandle;
	if (count > 0)
		GLCHK(glDeleteBuffers(count, buffer));
	if (mProgram > 0)
		GLCHK(glDeleteProgram(mProgram));
}


int Gles2HmdEye::renderEye(GLuint texture,
			   unsigned int textureWidth,
			   unsigned int textureHeight)
{
	GLCHK(glUseProgram(mProgram));

	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
	/* GLCHK(glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, texture)); */
	GLCHK(glBindTexture(GL_TEXTURE_2D, texture));
	GLCHK(glUniform1i(mProgramTexture, mFirstTexUnit));

	GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBufferHandle));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramPosition));
	GLCHK(glVertexAttribPointer(
		mProgramPosition, 2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramColor));
	GLCHK(glVertexAttribPointer(mProgramColor, 4, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord0));
	GLCHK(glVertexAttribPointer(
		mProgramTexCoord0, 2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord1));
	GLCHK(glVertexAttribPointer(
		mProgramTexCoord1, 2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord2));
	GLCHK(glVertexAttribPointer(
		mProgramTexCoord2, 2, GL_FLOAT, false, 0, 0));

	float ratio;
	if ((mRotation == 90) || (mRotation == 270))
		ratio = (float)textureHeight / (float)textureWidth;
	else
		ratio = (float)textureWidth / (float)textureHeight;

	if (ratio > 1.) {
		GLCHK(glUniform2f(
			mProgramEyeToSourceUVScale, mScale, mScale * ratio));
	} else {
		GLCHK(glUniform2f(
			mProgramEyeToSourceUVScale, mScale / ratio, mScale));
	}

	GLCHK(glUniform2f(mProgramEyeToSourceUVOffset, mPanH, mPanV));
	GLCHK(glUniform1i(mProgramChromaticAberrationCorrection, 0));
	GLCHK(glUniform1i(mProgramRotation, mRotation));
	GLCHK(glUniform1i(mProgramLensLimits, 0));
	GLCHK(glUniform2f(mProgramEyeToSourceScale,
			  2.f / mMetricsWidth,
			  -2.f / mMetricsHeight));
	GLCHK(glUniform2f(mProgramEyeToSourceOffset,
			  2.f * mEyeOffsetX / mMetricsWidth,
			  2.f * mEyeOffsetY / mMetricsHeight - 1.f));

	GLCHK(glDrawElements(GL_TRIANGLES,
			     sizeof(pdraw_gles2HmdIndices) / sizeof(float),
			     GL_UNSIGNED_INT,
			     0));

	GLCHK(glDisableVertexAttribArray(mProgramPosition));
	GLCHK(glDisableVertexAttribArray(mProgramColor));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord0));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord1));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord2));

	GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, 0));

	return 0;
}


Gles2Hmd::Gles2Hmd(Session *session,
		   unsigned int firstTexUnit,
		   unsigned int width,
		   unsigned int height,
		   float ipdOffset,
		   float xOffset,
		   float yOffset)
{
	Settings *settings;

	mSession = session;
	mHmdModel = PDRAW_HMD_MODEL_UNKNOWN;
	mIpdOffset = ipdOffset;
	mXOffset = xOffset;
	mYOffset = yOffset;
	mXdpi = SETTINGS_DISPLAY_XDPI;
	mYdpi = SETTINGS_DISPLAY_YDPI;
	mDeviceMarginTop = 0.f;
	mDeviceMarginBottom = 0.f;
	mDeviceMarginLeft = 0.f;
	mDeviceMarginRight = 0.f;

	settings = mSession->getSettings();
	settings->lock();
	settings->getDisplayScreenSettings(&mXdpi,
					   &mYdpi,
					   &mDeviceMarginTop,
					   &mDeviceMarginBottom,
					   &mDeviceMarginLeft,
					   &mDeviceMarginRight);
	mHmdModel = settings->getHmdModelSetting();
	settings->unlock();

	switch (mHmdModel) {
	default:
	case PDRAW_HMD_MODEL_COCKPITGLASSES:
		mHmdOffset = GLES2_HMD_OFFSET_COCKPITGLASSES;
		mHmdIpd = GLES2_HMD_IPD_COCKPITGLASSES;
		break;
	case PDRAW_HMD_MODEL_COCKPITGLASSES_2:
		mHmdOffset = GLES2_HMD_OFFSET_COCKPITGLASSES_2;
		mHmdIpd = GLES2_HMD_IPD_COCKPITGLASSES_2;
		break;
	}

	mMetricsWidth = (float)width / mXdpi * GLES2_HMD_INCH_TO_MILLIMETER;
	mMetricsHeight = (float)height / mYdpi * GLES2_HMD_INCH_TO_MILLIMETER;

	mLeftEye = new Gles2HmdEye(firstTexUnit,
				   mHmdModel,
				   1.f,
				   0.f,
				   0.f,
				   mMetricsWidth,
				   mMetricsHeight,
				   -(mHmdIpd + mIpdOffset) / 2.f + mXOffset,
				   mHmdOffset - mDeviceMarginBottom - mYOffset);
	mRightEye =
		new Gles2HmdEye(firstTexUnit,
				mHmdModel,
				1.f,
				0.f,
				0.f,
				mMetricsWidth,
				mMetricsHeight,
				(mHmdIpd + mIpdOffset) / 2.f + mXOffset,
				mHmdOffset - mDeviceMarginBottom - mYOffset);
}


Gles2Hmd::~Gles2Hmd(void)
{
	if (mLeftEye)
		delete (mLeftEye);
	if (mRightEye)
		delete (mRightEye);
}


int Gles2Hmd::renderHmd(GLuint texture,
			unsigned int textureWidth,
			unsigned int textureHeight)
{
	int ret = 0;

	if (mLeftEye != nullptr) {
		ret = mLeftEye->renderEye(texture, textureWidth, textureHeight);
		if (ret != 0)
			return ret;
	}

	if (mRightEye != nullptr) {
		ret = mRightEye->renderEye(
			texture, textureWidth, textureHeight);
		if (ret != 0)
			return ret;
	}

	return 0;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
