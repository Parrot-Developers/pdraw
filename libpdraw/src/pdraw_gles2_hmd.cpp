/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 HMD distorsion correction
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

/**
 * Extracted from FPVToolbox by Frédéric Bertolus
 * https://github.com/niavok/fpvtoolbox
 * and translated from Java to C++
 */

#include "pdraw_gles2_hmd.hpp"

#ifdef USE_GLES2

#define ULOG_TAG libpdraw
#include <ulog.h>

namespace Pdraw {


#define GLES2_HMD_INCH_TO_MILLIMETER     (25.4f)
#define GLES2_HMD_OFFSET                 (34.66f)


extern const float pdraw_gles2HmdCockpitglassesColors[14884];
extern const uint32_t pdraw_gles2HmdCockpitglassesIndices[21600];
extern const float pdraw_gles2HmdCockpitglassesPositions[7442];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsRed[7442];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsGreen[7442];
extern const float pdraw_gles2HmdCockpitglassesTexCoordsBlue[7442];

extern const float pdraw_gles2HmdCockpitglasses2Colors[14884];
extern const uint32_t pdraw_gles2HmdCockpitglasses2Indices[21600];
extern const float pdraw_gles2HmdCockpitglasses2Positions[7442];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsRed[7442];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsGreen[7442];
extern const float pdraw_gles2HmdCockpitglasses2TexCoordsBlue[7442];

extern const GLchar *pdraw_gles2HmdVertexShader;
extern const GLchar *pdraw_gles2HmdFragmentShader;


Gles2HmdEye::Gles2HmdEye(
	unsigned int firstTexUnit,
	enum pdraw_hmd_model hmdModel,
	float scale,
	float panH,
	float panV,
	float metricsWidth,
	float metricsHeight,
	float eyeOffsetX,
	float eyeOffsetY)
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
	mProgram = 0;
	mIndicesBufferHandle = 0;
	mPositionBufferHandle = 0;
	mColorBufferHandle = 0;
	mTexCoord0BufferHandle = 0;
	mTexCoord1BufferHandle = 0;
	mTexCoord2BufferHandle = 0;

	GLint vertexShader = 0, fragmentShader = 0;
	GLint success = 0;

	GLCHK();

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM)) {
		ULOGE("Gles2HmdEye: failed to create vertex shader");
		goto err;
	}

	glShaderSource(vertexShader, 1, &pdraw_gles2HmdVertexShader, NULL);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		ULOGE("Gles2HmdEye: vertex shader compilation failed '%s'",
			infoLog);
		goto err;
	}

	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShader == 0) || (fragmentShader == GL_INVALID_ENUM)) {
		ULOGE("Gles2HmdEye: failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShader, 1, &pdraw_gles2HmdFragmentShader, NULL);
	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		ULOGE("Gles2HmdEye: fragment shader compilation failed '%s'",
			infoLog);
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
		glGetProgramInfoLog(mProgram, 512, NULL, infoLog);
		ULOGE("Gles2HmdEye: program link failed '%s'", infoLog);
		goto err;
	}

	glDeleteShader(vertexShader);
	vertexShader = 0;
	glDeleteShader(fragmentShader);
	fragmentShader = 0;

	GLCHK();

	GLuint buffer[6];
	GLCHK(glGenBuffers(6, buffer));

	switch (mHmdModel) {
	default:
	case PDRAW_HMD_MODEL_COCKPITGLASSES:
		mIndicesBufferHandle = buffer[0];
		GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
			mIndicesBufferHandle));
		GLCHK(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesIndices),
			pdraw_gles2HmdCockpitglassesIndices, GL_STATIC_DRAW));

		mPositionBufferHandle = buffer[1];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesPositions),
			pdraw_gles2HmdCockpitglassesPositions, GL_STATIC_DRAW));

		mColorBufferHandle = buffer[2];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesColors),
			pdraw_gles2HmdCockpitglassesColors, GL_STATIC_DRAW));

		mTexCoord0BufferHandle = buffer[3];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesTexCoordsRed),
			pdraw_gles2HmdCockpitglassesTexCoordsRed,
			GL_STATIC_DRAW));

		mTexCoord1BufferHandle = buffer[4];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesTexCoordsGreen),
			pdraw_gles2HmdCockpitglassesTexCoordsGreen,
			GL_STATIC_DRAW));

		mTexCoord2BufferHandle = buffer[5];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglassesTexCoordsBlue),
			pdraw_gles2HmdCockpitglassesTexCoordsBlue,
			GL_STATIC_DRAW));
		break;
	case PDRAW_HMD_MODEL_COCKPITGLASSES_2:
		mIndicesBufferHandle = buffer[0];
		GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
			mIndicesBufferHandle));
		GLCHK(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2Indices),
			pdraw_gles2HmdCockpitglasses2Indices, GL_STATIC_DRAW));

		mPositionBufferHandle = buffer[1];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mPositionBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2Positions),
			pdraw_gles2HmdCockpitglasses2Positions,
			GL_STATIC_DRAW));

		mColorBufferHandle = buffer[2];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2Colors),
			pdraw_gles2HmdCockpitglasses2Colors, GL_STATIC_DRAW));

		mTexCoord0BufferHandle = buffer[3];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsRed),
			pdraw_gles2HmdCockpitglasses2TexCoordsRed,
			GL_STATIC_DRAW));

		mTexCoord1BufferHandle = buffer[4];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsGreen),
			pdraw_gles2HmdCockpitglasses2TexCoordsGreen,
			GL_STATIC_DRAW));

		mTexCoord2BufferHandle = buffer[5];
		GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
		GLCHK(glBufferData(GL_ARRAY_BUFFER,
			sizeof(pdraw_gles2HmdCockpitglasses2TexCoordsBlue),
			pdraw_gles2HmdCockpitglasses2TexCoordsBlue,
			GL_STATIC_DRAW));
		break;
	}

	mProgramTexture = glGetUniformLocation(mProgram, "Texture0");
	if (mProgramTexture < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'Texture0' "
			"location loading program");

	mProgramEyeToSourceUVScale = glGetUniformLocation(mProgram,
		"EyeToSourceUVScale");
	if (mProgramEyeToSourceUVScale < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceUVScale' "
			"location loading program");

	mProgramEyeToSourceUVOffset = glGetUniformLocation(mProgram,
		"EyeToSourceUVOffset");
	if (mProgramEyeToSourceUVOffset < 0)
		ULOGE("Gles2HmdEye: failed to get uniform "
			"'EyeToSourceUVOffset' location loading program");

	mProgramEyeToSourceScale = glGetUniformLocation(mProgram,
		"EyeToSourceScale");
	if (mProgramEyeToSourceScale < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceScale' "
			"location loading program");

	mProgramEyeToSourceOffset = glGetUniformLocation(mProgram,
		"EyeToSourceOffset");
	if (mProgramEyeToSourceOffset < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'EyeToSourceOffset' "
			"location loading program");

	mProgramChromaticAberrationCorrection = glGetUniformLocation(mProgram,
		"ChromaticAberrationCorrection");
	if (mProgramChromaticAberrationCorrection < 0)
		ULOGE("Gles2HmdEye: failed to get uniform "
			"'ChromaticAberrationCorrection' location "
			"loading program");

	mProgramRotation = glGetUniformLocation(mProgram, "Rotation");
	if (mProgramRotation < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'Rotation' "
			"location loading program");

	mProgramLensLimits = glGetUniformLocation(mProgram, "LensLimits");
	if (mProgramLensLimits < 0)
		ULOGE("Gles2HmdEye: failed to get uniform 'LensLimits' "
			"location loading program");

	mProgramPosition = glGetAttribLocation(mProgram, "Position");
	if (mProgramPosition < 0)
		ULOGE("Gles2HmdEye: failed to get attribute 'Position' "
			"location loading program");

	mProgramColor = glGetAttribLocation(mProgram, "Color");
	if (mProgramColor < 0)
		ULOGE("Gles2HmdEye: failed to get attribute 'Color' "
			"location loading program");

	mProgramTexCoord0 = glGetAttribLocation(mProgram, "TexCoord0");
	if (mProgramTexCoord0 < 0)
		ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord0' "
			"location loading program");

	mProgramTexCoord1 = glGetAttribLocation(mProgram, "TexCoord1");
	if (mProgramTexCoord1 < 0)
		ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord1' "
			"location loading program");

	mProgramTexCoord2 = glGetAttribLocation(mProgram, "TexCoord2");
	if (mProgramTexCoord2 < 0)
		ULOGE("Gles2HmdEye: failed to get attribute 'TexCoord2' "
			"location loading program");

	GLCHK();

	return;

err:
	if ((buffer[0]) || (buffer[1]) || (buffer[2]) ||
		(buffer[3]) || (buffer[4]) || (buffer[5]))
		glDeleteBuffers(6, buffer);
	if (vertexShader > 0)
		glDeleteShader(vertexShader);
	if (fragmentShader > 0)
		glDeleteShader(fragmentShader);
	if (mProgram > 0)
		glDeleteProgram(mProgram);
	mProgram = 0;
	mIndicesBufferHandle = 0;
	mPositionBufferHandle = 0;
	mColorBufferHandle = 0;
	mTexCoord0BufferHandle = 0;
	mTexCoord1BufferHandle = 0;
	mTexCoord2BufferHandle = 0;
}


Gles2HmdEye::~Gles2HmdEye(
	void)
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
		glDeleteBuffers(count, buffer);
	if (mProgram > 0)
		glDeleteProgram(mProgram);
}


int Gles2HmdEye::renderEye(
	GLuint texture,
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
	GLCHK(glVertexAttribPointer(mProgramPosition,
		2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mColorBufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramColor));
	GLCHK(glVertexAttribPointer(mProgramColor, 4, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord0BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord0));
	GLCHK(glVertexAttribPointer(mProgramTexCoord0,
		2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord1BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord1));
	GLCHK(glVertexAttribPointer(mProgramTexCoord1,
		2, GL_FLOAT, false, 0, 0));

	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, mTexCoord2BufferHandle));
	GLCHK(glEnableVertexAttribArray(mProgramTexCoord2));
	GLCHK(glVertexAttribPointer(mProgramTexCoord2,
		2, GL_FLOAT, false, 0, 0));

	float ratio;
	if ((mRotation == 90) || (mRotation == 270))
		ratio = (float)textureHeight / (float)textureWidth;
	else
		ratio = (float)textureWidth / (float)textureHeight;

	if (ratio > 1.) {
		GLCHK(glUniform2f(mProgramEyeToSourceUVScale,
			mScale, mScale * ratio));
	} else {
		GLCHK(glUniform2f(mProgramEyeToSourceUVScale,
			mScale / ratio, mScale));
	}

	GLCHK(glUniform2f(mProgramEyeToSourceUVOffset, mPanH, mPanV));
	GLCHK(glUniform1i(mProgramChromaticAberrationCorrection, 0));
	GLCHK(glUniform1i(mProgramRotation, mRotation));
	GLCHK(glUniform1i(mProgramLensLimits, 0));
	GLCHK(glUniform2f(mProgramEyeToSourceScale,
		2.f / mMetricsWidth, -2.f / mMetricsHeight));
	GLCHK(glUniform2f(mProgramEyeToSourceOffset,
		2.f * mEyeOffsetX / mMetricsWidth,
		2.f * mEyeOffsetY / mMetricsHeight - 1.f));

	GLCHK(glDrawElements(GL_TRIANGLES,
		sizeof(pdraw_gles2HmdCockpitglassesIndices) / sizeof(float),
		GL_UNSIGNED_INT, 0));

	GLCHK(glDisableVertexAttribArray(mProgramPosition));
	GLCHK(glDisableVertexAttribArray(mProgramColor));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord0));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord1));
	GLCHK(glDisableVertexAttribArray(mProgramTexCoord2));

	GLCHK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
	GLCHK(glBindBuffer(GL_ARRAY_BUFFER, 0));

	return 0;
}


Gles2Hmd::Gles2Hmd(
	unsigned int firstTexUnit,
	unsigned int width,
	unsigned int height,
	enum pdraw_hmd_model hmdModel,
	float xdpi,
	float ydpi,
	float deviceMargin,
	float ipd,
	float scale,
	float panH,
	float panV)
{
	mHmdModel = hmdModel;
	mDeviceMargin = deviceMargin;
	mIpd = ipd;
	mScale = scale;
	mPanH = panH;
	mPanV = panV;

	mMetricsWidth = (float)width / xdpi * GLES2_HMD_INCH_TO_MILLIMETER;
	mMetricsHeight = (float)height / ydpi * GLES2_HMD_INCH_TO_MILLIMETER;

	mLeftEye = new Gles2HmdEye(firstTexUnit, mHmdModel,
		mScale, mPanH, mPanV, mMetricsWidth, mMetricsHeight,
		-mIpd / 2.f, GLES2_HMD_OFFSET - mDeviceMargin);
	mRightEye = new Gles2HmdEye(firstTexUnit, mHmdModel,
		mScale, mPanH, mPanV, mMetricsWidth, mMetricsHeight,
		mIpd / 2.f, GLES2_HMD_OFFSET - mDeviceMargin);
}


Gles2Hmd::~Gles2Hmd(
	void)
{
	if (mLeftEye)
		delete(mLeftEye);
	if (mRightEye)
		delete(mRightEye);
}


int Gles2Hmd::renderHmd(
	GLuint texture,
	unsigned int textureWidth,
	unsigned int textureHeight)
{
	int ret = 0;

	if (mLeftEye != NULL) {
		ret = mLeftEye->renderEye(texture,
			textureWidth, textureHeight);
		if (ret != 0)
			return ret;
	}

	if (mRightEye != NULL) {
		ret = mRightEye->renderEye(texture,
			textureWidth, textureHeight);
		if (ret != 0)
			return ret;
	}

	return 0;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
