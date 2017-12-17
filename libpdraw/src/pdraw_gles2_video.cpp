/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 video rendering
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

#include "pdraw_gles2_video.hpp"

#ifdef USE_GLES2

#define ULOG_TAG libpdraw
#include <ulog.h>
#include <math.h>

#include "pdraw_session.hpp"
#include "pdraw_media_video.hpp"

namespace Pdraw {


#define PDRAW_GLES2_VIDEO_DEFAULT_HFOV (78.)
#define PDRAW_GLES2_VIDEO_DEFAULT_VFOV (49.)


static const GLchar *videoVertexShader =
	"uniform mat4 transform_matrix;\n"
	"attribute vec4 position;\n"
	"attribute vec2 texcoord;\n"
	"varying vec2 v_texcoord;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position = transform_matrix * position;\n"
	"    v_texcoord = texcoord;\n"
	"}\n";

static const GLchar *videoNoconvFragmentShader =
#if defined(GL_ES_VERSION_2_0) && (defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#endif
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_FragColor = texture2D(s_texture_0, v_texcoord);\n"
	"}\n";

static const GLchar *video420PlanarFragmentShader =
#if defined(GL_ES_VERSION_2_0) && (defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#endif
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    float y = texture2D(s_texture_0, v_texcoord).r;\n"
	"    float u = texture2D(s_texture_1, v_texcoord).r - 0.5;\n"
	"    float v = texture2D(s_texture_2, v_texcoord).r - 0.5;\n"
	"    \n"
	"    float r = y + 1.402 * v;\n"
	"    float g = y - 0.344 * u - 0.714 * v;\n"
	"    float b = y + 1.772 * u;\n"
	"    \n"
	"    gl_FragColor = vec4(r, g, b, 1.0);\n"
	"}\n";

static const GLchar *video420SemiplanarFragmentShader =
#if defined(GL_ES_VERSION_2_0) && (defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#endif
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    float y = texture2D(s_texture_0, v_texcoord).r;\n"
	"    vec4 uv = texture2D(s_texture_1, v_texcoord);\n"
	"    float u = uv.r - 0.5;\n"
	"    float v = uv.a - 0.5;\n"
	"    \n"
	"    float r = y + 1.402 * v;\n"
	"    float g = y - 0.344 * u - 0.714 * v;\n"
	"    float b = y + 1.772 * u;\n"
	"    \n"
	"    gl_FragColor = vec4(r, g, b, 1.0);\n"
	"}\n";


Gles2Video::Gles2Video(
	Session *session,
	VideoMedia *media,
	unsigned int firstTexUnit)
{
	GLint vertexShader = 0, fragmentShaderNoconv = 0;
	GLint fragmentShaderYuvp = 0, fragmentShaderYuvsp = 0;
	GLint success = 0;
	GLenum gle;
	unsigned int i;

	mSession = session;
	mMedia = media;
	mFirstTexUnit = firstTexUnit;
	mPaddingWidth = mPaddingHeight = GLES2_VIDEO_PADDING_FBO_WIDTH;
	memset(mProgram, 0, sizeof(mProgram));
	memset(mTextures, 0, sizeof(mTextures));
	mPaddingFbo = 0;
	mPaddingFboTexture = 0;

	if (mMedia != NULL) {
		unsigned int width = 0, height = 0;
		mMedia->getDimensions(NULL, NULL, NULL, NULL, NULL, NULL,
			&width, &height, NULL, NULL);
		if ((width != 0) && (height != 0)) {
			if (width >= height) {
				mPaddingHeight = GLES2_VIDEO_PADDING_FBO_WIDTH *
					height / width;
			} else {
				mPaddingWidth = GLES2_VIDEO_PADDING_FBO_WIDTH *
					width / height;
			}
		}
	}

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM)) {
		ULOGE("Gles2Video: failed to create vertex shader");
		goto err;
	}

	glShaderSource(vertexShader, 1, &videoVertexShader, NULL);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		ULOGE("Gles2Video: vertex shader compilation failed '%s'",
			infoLog);
		goto err;
	}

	fragmentShaderNoconv = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderNoconv == 0) ||
		(fragmentShaderNoconv == GL_INVALID_ENUM)) {
		ULOGE("Gles2Video: failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShaderNoconv, 1,
		&videoNoconvFragmentShader, NULL);
	glCompileShader(fragmentShaderNoconv);
	glGetShaderiv(fragmentShaderNoconv, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderNoconv, 512, NULL, infoLog);
		ULOGE("Gles2Video: fragment shader compilation failed '%s'",
			infoLog);
		goto err;
	}

	fragmentShaderYuvsp = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderYuvsp == 0) ||
		(fragmentShaderYuvsp == GL_INVALID_ENUM)) {
		ULOGE("Gles2Video: failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShaderYuvsp, 1,
		&video420SemiplanarFragmentShader, NULL);
	glCompileShader(fragmentShaderYuvsp);
	glGetShaderiv(fragmentShaderYuvsp, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderYuvsp, 512, NULL, infoLog);
		ULOGE("Gles2Video: fragment shader compilation failed '%s'",
			infoLog);
		goto err;
	}

	fragmentShaderYuvp = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderYuvp == 0) ||
		(fragmentShaderYuvp == GL_INVALID_ENUM)) {
		ULOGE("Gles2Video: failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShaderYuvp, 1,
		&video420PlanarFragmentShader, NULL);
	glCompileShader(fragmentShaderYuvp);
	glGetShaderiv(fragmentShaderYuvp, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderYuvp, 512, NULL, infoLog);
		ULOGE("Gles2Video: fragment shader compilation failed '%s'",
			infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glCreateProgram();
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		vertexShader);
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		fragmentShaderNoconv);
	glLinkProgram(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]);
	glGetProgramiv(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			512, NULL, infoLog);
		ULOGE("Gles2Video: program link failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] =
		glCreateProgram();
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
		vertexShader);
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
		fragmentShaderYuvp);
	glLinkProgram(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB]);
	glGetProgramiv(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
		GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			512, NULL, infoLog);
		ULOGE("Gles2Video: program link failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB] =
		glCreateProgram();
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
		vertexShader);
	glAttachShader(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
		fragmentShaderYuvsp);
	glLinkProgram(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB]);
	glGetProgramiv(
		mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
		GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			512, NULL, infoLog);
		ULOGE("Gles2Video: program link failed '%s'", infoLog);
		goto err;
	}

	glDeleteShader(vertexShader);
	vertexShader = 0;
	glDeleteShader(fragmentShaderNoconv);
	fragmentShaderNoconv = 0;
	glDeleteShader(fragmentShaderYuvp);
	fragmentShaderYuvp = 0;
	glDeleteShader(fragmentShaderYuvsp);
	fragmentShaderYuvsp = 0;

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"transform_matrix");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"s_texture_0");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][1] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"s_texture_1");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][2] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"s_texture_2");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"texcoord");

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"transform_matrix");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"s_texture_0");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB][1] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"s_texture_1");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB][2] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"s_texture_2");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB],
			"texcoord");

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"transform_matrix");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"s_texture_0");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB][1] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"s_texture_1");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB][2] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"s_texture_2");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB],
			"texcoord");

	glGenTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures);

	for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i);
		glBindTexture(GL_TEXTURE_2D, mTextures[i]);

		glTexParameteri(GL_TEXTURE_2D,
			GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,
			GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D,
			GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D,
			GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}

	glGenFramebuffers(1, &mPaddingFbo);
	if (mPaddingFbo <= 0) {
		ULOGE("Gles2Video: failed to create framebuffer");
		goto err;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo);

	glGenTextures(1, &mPaddingFboTexture);
	if (mPaddingFboTexture <= 0) {
		ULOGE("Gles2Video: failed to create texture");
		goto err;
	}
	glActiveTexture(GL_TEXTURE0 +
		mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT);
	glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mPaddingWidth, mPaddingHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
		GL_TEXTURE_2D, mPaddingFboTexture, 0);

	gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (gle != GL_FRAMEBUFFER_COMPLETE) {
		ULOGE("Gles2Video: invalid framebuffer status");
		goto err;
	}

	glClear(GL_COLOR_BUFFER_BIT);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return;

err:
	if (mTextures[0]) {
		glDeleteTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures);
	}
	if (mPaddingFboTexture > 0) {
		glDeleteTextures(1, &mPaddingFboTexture);
	}
	if (mPaddingFbo > 0) {
		glDeleteFramebuffers(1, &mPaddingFbo);
	}
	if (vertexShader)
		glDeleteShader(vertexShader);
	if (fragmentShaderNoconv)
		glDeleteShader(fragmentShaderNoconv);
	if (fragmentShaderYuvp)
		glDeleteShader(fragmentShaderYuvp);
	if (fragmentShaderYuvsp)
		glDeleteShader(fragmentShaderYuvsp);
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] > 0) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_NONE]);
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] > 0) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB]);
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB]) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB]);
	}
	memset(mProgram, 0, sizeof(mProgram));
	memset(mTextures, 0, sizeof(mTextures));
	mPaddingFbo = 0;
	mPaddingFboTexture = 0;
}


Gles2Video::~Gles2Video(
	void)
{
	if (mTextures[0])
		glDeleteTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures);
	if (mPaddingFboTexture > 0)
		glDeleteTextures(1, &mPaddingFboTexture);
	if (mPaddingFbo > 0)
		glDeleteFramebuffers(1, &mPaddingFbo);
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] > 0) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_NONE]);
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB] > 0) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB]);
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB]) {
		glDeleteProgram(mProgram[
			GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB]);
	}
}


static void createProjectionMatrix(
	Eigen::Matrix4f &projMat,
	float viewHFov,
	float videoHFov,
	float windowW,
	float windowH,
	float near,
	float far)
{
	float k = tanf(videoHFov / 2.) / tanf(viewHFov / 2.);
	float w = k * windowW;
	float h = k * windowH;
	float a = -(near + far) / (near - far);
	float b = -((2 * far * near) / (far - near));

	projMat << w, 0, 0, 0,
		0, h, 0, 0,
		0, 0, a, b,
		0, 0, 1, 0;
}


int Gles2Video::loadFrame(
	uint8_t *framePlane[3],
	unsigned int frameStride[3],
	unsigned int frameWidth,
	unsigned int frameHeight,
	enum gles2_video_color_conversion colorConversion)
{
	unsigned int i;

	if ((frameWidth == 0) || (frameHeight == 0) || (frameStride[0] == 0)) {
		ULOGE("Gles2Video: invalid dimensions");
		return -1;
	}

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE:
		glUniform1i(mUniformSamplers[colorConversion][0],
			mFirstTexUnit + (intptr_t)framePlane[0]);
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB:
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i);
			glBindTexture(GL_TEXTURE_2D, mTextures[i]);
			glUniform1i(mUniformSamplers[colorConversion][i],
				mFirstTexUnit + i);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
				frameStride[i], frameHeight / ((i > 0) ? 2 : 1),
				0, GL_LUMINANCE,
				GL_UNSIGNED_BYTE, framePlane[i]);
		}
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB:
		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0);
		glBindTexture(GL_TEXTURE_2D, mTextures[0]);
		glUniform1i(mUniformSamplers[colorConversion][0],
			mFirstTexUnit + 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
			frameStride[0], frameHeight,
			0, GL_LUMINANCE, GL_UNSIGNED_BYTE, framePlane[0]);

		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1);
		glBindTexture(GL_TEXTURE_2D, mTextures[1]);
		glUniform1i(mUniformSamplers[colorConversion][1],
			mFirstTexUnit + 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
			frameStride[1] / 2, frameHeight / 2,
			0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, framePlane[1]);
		break;
	}

	return 0;
}


int Gles2Video::renderFrame(
	uint8_t *framePlane[3],
	unsigned int frameStride[3],
	unsigned int frameWidth,
	unsigned int frameHeight,
	unsigned int sarWidth,
	unsigned int sarHeight,
	unsigned int windowWidth,
	unsigned int windowHeight,
	unsigned int windowX,
	unsigned int windowY,
	enum gles2_video_color_conversion colorConversion,
	const struct vmeta_frame_v2 *metadata,
	bool headtracking,
	GLuint fbo)
{
	unsigned int i;
	float vertices[16];
	float texCoords[8];

	if ((frameWidth == 0) || (frameHeight == 0) ||
		(sarWidth == 0) || (sarHeight == 0) ||
		(windowWidth == 0) || (windowHeight == 0) ||
		(frameStride[0] == 0)) {
		ULOGE("Gles2Video: invalid dimensions");
		return -1;
	}

	glUseProgram(mProgram[colorConversion]);
	glEnable(GL_TEXTURE_2D);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE:
		glUniform1i(mUniformSamplers[colorConversion][0],
			mFirstTexUnit + (intptr_t)framePlane[0]);
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB:
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i);
			glBindTexture(GL_TEXTURE_2D, mTextures[i]);
			glUniform1i(mUniformSamplers[colorConversion][i],
				mFirstTexUnit + i);
		}
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB:
		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0);
		glBindTexture(GL_TEXTURE_2D, mTextures[0]);
		glUniform1i(mUniformSamplers[colorConversion][0],
			mFirstTexUnit + 0);

		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1);
		glBindTexture(GL_TEXTURE_2D, mTextures[1]);
		glUniform1i(mUniformSamplers[colorConversion][1],
			mFirstTexUnit + 1);
		break;
	}

	/* Keep the video aspect ratio */
	float windowAR = (float)windowWidth / (float)windowHeight;
	float sar = (float)sarWidth / (float)sarHeight;
	float videoAR = (float)frameWidth / (float)frameHeight * sar;
	float windowW = 1.;
	float windowH = windowAR;
	float ratioW = 1.;
	float ratioH = 1.;
	if (videoAR >= windowAR) {
		ratioW = 1.;
		ratioH = windowAR / videoAR;
		windowW = 1.;
		windowH = windowAR;
	} else {
		ratioW = videoAR / windowAR;
		ratioH = 1.;
		windowW = 1.;
		windowH = windowAR;
	}
	float videoW = ratioW / windowW;
	float videoH = ratioH / windowH;

	Eigen::Matrix4f viewMat = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f projMat = Eigen::Matrix4f::Identity();

	float hFov = 0.;
	float vFov = 0.;
	if (mMedia)
		mMedia->getFov(&hFov, &vFov);
	if (hFov == 0.)
		hFov = PDRAW_GLES2_VIDEO_DEFAULT_HFOV;
	if (vFov == 0.)
		vFov = PDRAW_GLES2_VIDEO_DEFAULT_VFOV;
	hFov = hFov * M_PI / 180.;
	vFov = vFov * M_PI / 180.;

	createProjectionMatrix(projMat, hFov, hFov,
		windowW, windowH, 0.1, 100.);

	if ((headtracking) && (mSession)) {
		Eigen::Quaternionf headQuat =
			mSession->getSelfMetadata()->getDebiasedHeadOrientation();
		Eigen::Matrix3f headRotNed = headQuat.toRotationMatrix();

		Eigen::Matrix3f camRot1 =
			Eigen::AngleAxisf(-metadata->base.cameraPan,
				Eigen::Vector3f::UnitZ()).matrix();
		Eigen::Matrix3f camRot2 =
			Eigen::AngleAxisf(-metadata->base.cameraTilt,
				Eigen::Vector3f::UnitY()).matrix();

		Eigen::Matrix3f viewRotNed = camRot2 * camRot1 * headRotNed;

		Eigen::Matrix3f viewRotLH;
		viewRotLH <<  viewRotNed(0, 0), -viewRotNed(1, 0), -viewRotNed(2, 0),
			-viewRotNed(0, 1),  viewRotNed(1, 1),  viewRotNed(2, 1),
			-viewRotNed(0, 2),  viewRotNed(1, 2),  viewRotNed(2, 2);
		Eigen::Matrix3f rot;
		rot <<  0,  0, -1,
			1,  0,  0,
			0, -1,  0;
		viewMat.block<3, 3>(0, 0) = rot.transpose() * viewRotLH * rot;
	}

	Eigen::Matrix4f xformMat = projMat * viewMat;

	if (headtracking) {
		/* Padding */
		glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo);
		glViewport(0, 0, mPaddingWidth, mPaddingHeight);
		glClear(GL_COLOR_BUFFER_BIT);

		vertices[0] = -1.;
		vertices[1] = 1.;
		vertices[2] = 1.;
		vertices[3] = 1.;
		vertices[4] = 1.;
		vertices[5] = 1.;
		vertices[6] = -1.;
		vertices[7] = -1.;
		vertices[8] = 1.;
		vertices[9] = 1.;
		vertices[10] = -1.;
		vertices[11] = 1.;

		Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
		glUniformMatrix4fv(mProgramTransformMatrix[colorConversion],
			1, false, id.data());

		glVertexAttribPointer(mPositionHandle[colorConversion],
			3, GL_FLOAT, false, 0, vertices);
		glEnableVertexAttribArray(mPositionHandle[colorConversion]);

		texCoords[0] = 0.0f;
		texCoords[1] = 1.0f;
		texCoords[2] = (float)frameWidth / (float)frameStride[0];
		texCoords[3] = 1.0f;
		texCoords[4] = 0.0f;
		texCoords[5] = 0.0f;
		texCoords[6] = (float)frameWidth / (float)frameStride[0];
		texCoords[7] = 0.0f;

		glVertexAttribPointer(mTexcoordHandle[colorConversion],
			2, GL_FLOAT, false, 0, texCoords);
		glEnableVertexAttribArray(mTexcoordHandle[colorConversion]);

		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		glDisableVertexAttribArray(mPositionHandle[colorConversion]);
		glDisableVertexAttribArray(mTexcoordHandle[colorConversion]);

		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glViewport(windowX, windowY, windowWidth, windowHeight);
		glUseProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]);
		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			GLES2_VIDEO_TEX_UNIT_COUNT);
		glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture);
		glUniform1i(
			mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][0],
			mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT);
		glUniformMatrix4fv(
			mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			1, false, xformMat.data());

		vertices[0] = -40. * videoW;
		vertices[1] = -40. * videoH;
		vertices[2] = 1.01;
		vertices[3] = 40. * videoW;
		vertices[4] = -40. * videoH;
		vertices[5] = 1.001;
		vertices[6] = -40. * videoW;
		vertices[7] = 40. * videoH;
		vertices[8] = 1.001;
		vertices[9] = 40. * videoW;
		vertices[10] = 40. * videoH;
		vertices[11] = 1.001;

		glVertexAttribPointer(
			mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			3, GL_FLOAT, false, 0, vertices);
		glEnableVertexAttribArray(
			mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]);

		texCoords[0] = -19.0f;
		texCoords[1] = 20.0f;
		texCoords[2] = 20.0f;
		texCoords[3] = 20.0f;
		texCoords[4] = -19.0f;
		texCoords[5] = -19.0f;
		texCoords[6] = 20.0f;
		texCoords[7] = -19.0f;

		glVertexAttribPointer(
			mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			2, GL_FLOAT, false, 0, texCoords);
		glEnableVertexAttribArray(
			mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]);

		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		glDisableVertexAttribArray(
			mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]);
		glDisableVertexAttribArray(
			mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]);

		glUseProgram(mProgram[colorConversion]);
	}

	glUniformMatrix4fv(mProgramTransformMatrix[colorConversion],
		1, false, xformMat.data());

	vertices[0] = -videoW;
	vertices[1] = -videoH;
	vertices[2] = 1.;
	vertices[3] = videoW;
	vertices[4] = -videoH;
	vertices[5] = 1.;
	vertices[6] = -videoW;
	vertices[7] = videoH;
	vertices[8] = 1.;
	vertices[9] = videoW;
	vertices[10] = videoH;
	vertices[11] = 1.;

	glVertexAttribPointer(mPositionHandle[colorConversion],
		3, GL_FLOAT, false, 0, vertices);
	glEnableVertexAttribArray(mPositionHandle[colorConversion]);

	texCoords[0] = 0.0f;
	texCoords[1] = 1.0f;
	texCoords[2] = (float)frameWidth / (float)frameStride[0];
	texCoords[3] = 1.0f;
	texCoords[4] = 0.0f;
	texCoords[5] = 0.0f;
	texCoords[6] = (float)frameWidth / (float)frameStride[0];
	texCoords[7] = 0.0f;

	glVertexAttribPointer(mTexcoordHandle[colorConversion],
		2, GL_FLOAT, false, 0, texCoords);
	glEnableVertexAttribArray(mTexcoordHandle[colorConversion]);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableVertexAttribArray(mPositionHandle[colorConversion]);
	glDisableVertexAttribArray(mTexcoordHandle[colorConversion]);

	return 0;
}


GLuint* Gles2Video::getTextures(
	void)
{
	return mTextures;
}


int Gles2Video::allocTextures(
	unsigned int videoWidth,
	unsigned int videoHeight)
{
	int ret = 0, i;

	for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
		glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i);
		glBindTexture(GL_TEXTURE_2D, mTextures[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
			videoWidth, videoHeight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	}

	return ret;
}


void Gles2Video::setVideoMedia(
	VideoMedia *media)
{
	mMedia = media;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
