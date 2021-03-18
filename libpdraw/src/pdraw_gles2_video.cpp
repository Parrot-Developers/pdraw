/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 video rendering
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

#include "pdraw_gles2_video.hpp"

//#include "LFClientEngine.h"

//LFClientEngine _lfClientEngine;

#ifdef USE_GLES2

#	include "pdraw_session.hpp"

#	include <math.h>
#	ifdef BCM_VIDEOCORE
#		include <EGL/egl.h>
#		include <EGL/eglext.h>
#		include <GLES2/gl2ext.h>
#	endif /* BCM_VIDEOCORE */

#	include <futils/futils.h>
#	define ULOG_TAG pdraw_gles2vid
#	include <ulog.h>
ULOG_DECLARE_TAG(pdraw_gles2vid);
//extern vmeta_frame* dgdframeMeta;

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "/Users/ddvergsten/XCodeProjects/RespondAR5-14-20PostMergeKEEP/RespondAR/SampleCode/SwiftSampleCode/SmartCam/SmartCam/LFClientWrapper2.h"
//#include "/Users/ddvergsten/XCodeProjects/ParrotDemoB2/groundsdk/packages/groundsdk-ios/demo/GroundSdkDemo/LFClientWrapper.h"
extern boost::mutex sviewMutex;
extern boost::shared_ptr<SViewData> _frameViewData;
extern bool _viewDataInitialized;
namespace Pdraw {


#	define PDRAW_GLES2_VIDEO_DEFAULT_HFOV (78.)
#	define PDRAW_GLES2_VIDEO_DEFAULT_VFOV (49.)

/* Blurred padding dark coef */
#	define PDRAW_BLURRED_PADDING_DARK_COEF (0.75f)

/* The temporal frequency of zebra pattern */
#	define PDRAW_ZEBRA_FREQUENCY_HZ (1.f)
/* The angle of zebra pattern relative to y axis */
#	define PDRAW_ZEBRA_ANGLE (60.f * M_PI / 180.f)
/* The weight in pixels of zebra pattern, relative to 1920 width */
#	define PDRAW_ZEBRA_WEIGHT (8.f)

#	define GLES2_VIDEO_BLUR_MIN_SIGMA 0.8f
#	define GLES2_VIDEO_BLUR_MAX_SIGMA 6.0f
#	define GLES2_VIDEO_BLURRED_PADDING_SIGMA 3.0f

#	define GLES2_VIDEO_HISTOGRAM_COMPUTE_INTERVAL_US 100000


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

static const GLchar *zebraFragmentShader =
	"uniform float zebra_phase;\n"
	"uniform float zebra_sat;\n"
	"uniform float zebra_weight;\n"
	"uniform mat2 zebra_mat;\n"
	"\n"
	"vec3 apply_zebra(vec3 avg, vec3 rgb)\n"
	"{\n"
	"    vec2 rot_pos = zebra_mat * gl_FragCoord.xy;\n"
	"    float z = mod(rot_pos.x + 2.0 * zebra_weight * zebra_phase,\n"
	"        zebra_weight * 2.0) - zebra_weight;\n"
	"    float zebra = smoothstep(zebra_weight / 2.0 - 0.5,\n"
	"        zebra_weight / 2.0 + 0.5, abs(z));\n"
	"    float pixel_val = min(min(avg.r, avg.g), avg.b);\n"
	"    float zebra_factor = step(zebra_sat, pixel_val);\n"
	"    rgb = mix(rgb, zebra * rgb, zebra_factor);\n"
	"    return rgb;\n"
	"}\n";

static const GLchar *textureNoconvFragmentShader =
#	ifdef BCM_VIDEOCORE
	"uniform samplerExternalOES s_texture_0;\n"
	"uniform samplerExternalOES s_texture_1;\n"
	"uniform samplerExternalOES s_texture_2;\n"
#	else /* BCM_VIDEOCORE */
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
#	endif /* BCM_VIDEOCORE */
	"uniform vec2 stride[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    return texture2D(s_texture_0, coord).rgb;\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    return texture2D(s_texture_0, coord + offset_px * stride[0]).rgb;\n"
	"}\n";

static const GLchar *textureI420FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, coord).r;\n"
	"    yuv.g = texture2D(s_texture_1, coord).r - 0.5;\n"
	"    yuv.b = texture2D(s_texture_2, coord).r - 0.5;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, coord + offset_px * stride[0]).r;\n"
	"    yuv.g = texture2D(s_texture_1, coord + offset_px * stride[1]).r - 0.5;\n"
	"    yuv.b = texture2D(s_texture_2, coord + offset_px * stride[2]).r - 0.5;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *textureNV12FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, coord).r;\n"
	"    yuv.gb = texture2D(s_texture_1, coord).ra - vec2(0.5);\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, coord + offset_px * stride[0]).r;\n"
	"    yuv.gb = texture2D(s_texture_1, coord + offset_px * stride[1]).ra - 0.5;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *videoFragmentShader =
#	ifdef BCM_VIDEOCORE
	"#extension GL_OES_EGL_image_external : require\n"
#	endif /* BCM_VIDEOCORE */
#	if defined(GL_ES_VERSION_2_0) &&                                      \
		(defined(ANDROID) || defined(__APPLE__))
	"precision highp float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform float sat_coef;\n"
	"uniform float light_coef;\n"
	"uniform float dark_coef;\n"
	"uniform float zebra_enable;\n"
	"uniform float zebra_avg_weights[9];\n"
	"\n"
	"vec3 apply_zebra(vec3 avg, vec3 rgb);\n"
	"vec3 read_rgb(vec2 coord);\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px);\n"
	"\n"
	"vec3 read_rgb_avg(vec2 coord)\n"
	"{\n"
	"    vec3 rgb = vec3(0.0);\n"
	"    for (int y = -1; y <= 1; y++)\n"
	"    {\n"
	"        for (int x = -1; x <= 1; x++)\n"
	"        {\n"
	"            vec2 offset = vec2(float(x), float(y));\n"
	"            rgb += read_rgb_with_offset(coord , offset)\n"
	"                  * zebra_avg_weights[(y + 1) * 3 + (x + 1)];\n"
	"        }\n"
	"    }\n"
	"    return rgb;\n"
	"}\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = read_rgb(v_texcoord);\n"
	"    \n"
	"    if (zebra_enable > 0.5)\n"
	"    {\n"
	"        vec3 avg = read_rgb_avg(v_texcoord);\n"
	"        rgb = apply_zebra(avg, rgb);\n"
	"    }\n"
	"    float luma = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;\n"
	"    rgb = mix(vec3(luma), rgb, sat_coef);\n"
	"    rgb = mix(vec3(1.0), rgb, light_coef);\n"
	"    rgb = mix(vec3(0.0), rgb, dark_coef);\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

static const GLchar *videoNoconvFragmentShaders[] = {
	videoFragmentShader,
	textureNoconvFragmentShader,
	zebraFragmentShader};

static const GLchar *videoNV12FragmentShaders[] = {videoFragmentShader,
						   textureNV12FragmentShader,
						   zebraFragmentShader};

static const GLchar *videoI420FragmentShaders[] = {videoFragmentShader,
						   textureI420FragmentShader,
						   zebraFragmentShader};

static const GLchar *blurHVertexShader =
	"attribute vec2 position;\n"
	"varying float v_texcoord_x[15];\n"
	"varying float v_texcoord_y;\n"
	"uniform float pixel_size;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position = vec4(position, 0.0, 1.0);\n"
	"    vec2 center_tex_coords = position * 0.5 + 0.5;\n"
	"    for (int i = -7; i <= 7; i++) {\n"
	"        float offset = pixel_size * float(i);\n"
	"        v_texcoord_x[i + 7] = center_tex_coords.x + offset;\n"
	"    }\n"
	"    v_texcoord_y = center_tex_coords.y;\n"
	"}\n";

static const GLchar *blurVVertexShader =
	"attribute vec2 position;\n"
	"varying float v_texcoord_x;\n"
	"varying float v_texcoord_y[15];\n"
	"uniform float pixel_size;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position = vec4(position, 0.0, 1.0);\n"
	"    vec2 center_tex_coords = position * 0.5 + 0.5;\n"
	"    for (int i = -7; i <= 7; i++) {\n"
	"        float offset = pixel_size * float(i);\n"
	"        v_texcoord_y[i + 7] = center_tex_coords.y + offset;\n"
	"    }\n"
	"    v_texcoord_x = center_tex_coords.x;\n"
	"}\n";

static const GLchar *blurHFragmentShader =
#	if defined(GL_ES_VERSION_2_0) &&                                      \
		(defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#	endif
	"varying float v_texcoord_x[15];\n"
	"varying float v_texcoord_y;\n"
	"uniform sampler2D s_texture;\n"
	"uniform float blur_weights[15];\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = vec3(0.0);\n"
	"    for (int i = 0; i < 15; i++) {\n"
	"        vec2 coords = vec2(v_texcoord_x[i], v_texcoord_y);\n"
	"        rgb += texture2D(s_texture, coords).rgb * blur_weights[i];\n"
	"    }\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

static const GLchar *blurVFragmentShader =
#	if defined(GL_ES_VERSION_2_0) &&                                      \
		(defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#	endif
	"varying float v_texcoord_x;\n"
	"varying float v_texcoord_y[15];\n"
	"uniform sampler2D s_texture;\n"
	"uniform float blur_weights[15];\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = vec3(0.0);\n"
	"    for (int i = 0; i < 15; i++) {\n"
	"        vec2 coords = vec2(v_texcoord_x, v_texcoord_y[i]);\n"
	"        rgb += texture2D(s_texture, coords).rgb * blur_weights[i];\n"
	"    }\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

static const GLchar *histogramVertexShader =
	"attribute vec4 position;\n"
	"attribute vec2 texcoord;\n"
	"varying vec2 v_texcoord;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position = position;\n"
	"    v_texcoord = texcoord;\n"
	"}\n";

static const GLchar *histogramFragmentShader =
#	ifdef BCM_VIDEOCORE
	"#extension GL_OES_EGL_image_external : require\n"
#	endif /* BCM_VIDEOCORE */
#	if defined(GL_ES_VERSION_2_0) &&                                      \
		(defined(ANDROID) || defined(__APPLE__))
	"precision mediump float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"\n"
	"vec3 read_rgb(vec2 coord);\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = read_rgb(v_texcoord);\n"
	"    float luma = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;\n"
	"    gl_FragColor = vec4(rgb, luma);\n"
	"}\n";

static const GLchar
	*histogramFragmentShaders[GLES2_VIDEO_COLOR_CONVERSION_MAX][2] = {
		{
			histogramFragmentShader,
			textureNoconvFragmentShader,
		},
		{
			histogramFragmentShader,
			textureI420FragmentShader,
		},
		{
			histogramFragmentShader,
			textureNV12FragmentShader,
		},
};

static const GLfloat yuv2RgbMats[][9] = {
	/* Limited range */
	{1.164f, 1.164f, 1.164f, 0.f, -0.392f, 2.017f, 1.596f, -0.813f, 0.f},
	/* Full range */
	{1.f, 1.f, 1.f, 0.f, -0.344f, 1.772f, 1.402f, -0.714f, 0.f}};

static const GLfloat yuv2RgbOffsets[][3] = {
	/* Limited range */
	{-0.0625f, 0.f, 0.f},
	/* Full range */
	{0.f, 0.f, 0.f},
};

static const GLfloat zebraAvgWeights[9] = {0.077847f,
					   0.123317f,
					   0.077847f,
					   0.123317f,
					   0.195346f,
					   0.123317f,
					   0.077847f,
					   0.123317f,
					   0.077847f};

boost::shared_ptr<SViewData> GetViewData(){
    boost::shared_ptr<SViewData> viewData(new SViewData);
    //_viewData.reset(new SViewData);
    viewData->dfov = 45.0;
    viewData->dFovVerticalAngle = 45.0;
    viewData->dFovHorizontalAngle = 55.0;
    viewData->dVehicleAltitude = 500.0;
    //46.764594, -92.151235 Lincoln Park
    //esko 46.685940, -92.363685
    viewData->dVehicleLat =46.685940;
    viewData->dVehicleLon =-92.363685;
    viewData->dCameraPitch = -30.0;
    viewData->dCameraHeading = 0.0;
    viewData->dCameraRoll = 0.0;
    viewData->dVehiclePitch = 0.0;
    viewData->dVehicleHeading = 0.0;
    viewData->dVehicleRoll = 0.0;
    viewData->dVehicldAltitudeAGL = 5000.0;
    return viewData;
    
}
//#include "ClientFactory.h"
Gles2Video::Gles2Video(Session *session,
		       GLuint defaultFbo,
		       unsigned int firstTexUnit)
{
	int ret;
	GLint vertexShader = 0, fragmentShaderNoconv = 0;
	GLint fragmentShaderYuvp = 0, fragmentShaderYuvsp = 0;
	GLint success = 0;
	unsigned int i;

    _lfClientEngine.InitShaders();

    _lfClientEngine.Init();
	mSession = session;
	mMedia = NULL;
	mVideoWidth = 0;
	mVideoHeight = 0;
	mFirstTexUnit = firstTexUnit;
	mDefaultFbo = defaultFbo;
	mTransition = GLES2_VIDEO_TRANSITION_NONE;
	mTransitionStartTime = 0;
	mTransitionDuration = 0;
	mTransitionHold = false;
	memset(mProgram, 0, sizeof(mProgram));
	memset(mProgramTransformMatrix, 0, sizeof(mProgramTransformMatrix));
	memset(mProgramYuv2RgbMatrix, 0, sizeof(mProgramYuv2RgbMatrix));
	memset(mProgramYuv2RgbOffset, 0, sizeof(mProgramYuv2RgbOffset));
	memset(mProgramStride, 0, sizeof(mProgramStride));
	memset(mProgramSatCoef, 0, sizeof(mProgramSatCoef));
	memset(mProgramLightCoef, 0, sizeof(mProgramLightCoef));
	memset(mProgramDarkCoef, 0, sizeof(mProgramDarkCoef));
	memset(mProgramZebraEnable, 0, sizeof(mProgramZebraEnable));
	memset(mProgramZebraThreshold, 0, sizeof(mProgramZebraThreshold));
	memset(mProgramZebraPhase, 0, sizeof(mProgramZebraPhase));
	memset(mProgramZebraWeight, 0, sizeof(mProgramZebraWeight));
	memset(mTextures, 0, sizeof(mTextures));
	mExtTexture = 0;
	memset(mUniformSamplers, 0, sizeof(mUniformSamplers));
	memset(mPositionHandle, 0, sizeof(mPositionHandle));
	memset(mTexcoordHandle, 0, sizeof(mTexcoordHandle));
	mBlurInit = false;
	mApplyBlur = false;
	memset(mBlurWeights, 0, sizeof(mBlurWeights));
	mBlurFboWidth = 0;
	mBlurFboHeight = 0;
	memset(mBlurFbo, 0, sizeof(mBlurFbo));
	memset(mBlurFboTexture, 0, sizeof(mBlurFboTexture));
	memset(mBlurProgram, 0, sizeof(mBlurProgram));
	memset(mBlurUniformPixelSize, 0, sizeof(mBlurUniformPixelSize));
	memset(mBlurUniformWeights, 0, sizeof(mBlurUniformWeights));
	memset(mBlurUniformSampler, 0, sizeof(mBlurUniformSampler));
	memset(mBlurPositionHandle, 0, sizeof(mBlurPositionHandle));
	mPaddingPass1Width = 0;
	mPaddingPass1Height = 0;
	mPaddingPass2Width = 0;
	mPaddingPass2Height = 0;
	memset(mPaddingBlurWeights, 0, sizeof(mPaddingBlurWeights));
	memset(mPaddingFbo, 0, sizeof(mPaddingFbo));
	memset(mPaddingFboTexture, 0, sizeof(mPaddingFboTexture));
	mHistogramInit = false;
	mHistogramLastComputeTime = 0;
	memset(mHistogramProgram, 0, sizeof(mHistogramProgram));
	memset(mHistogramYuv2RgbMatrix, 0, sizeof(mHistogramYuv2RgbMatrix));
	memset(mHistogramYuv2RgbOffset, 0, sizeof(mHistogramYuv2RgbOffset));
	memset(mHistogramUniformSampler, 0, sizeof(mHistogramUniformSampler));
	memset(mHistogramPositionHandle, 0, sizeof(mHistogramPositionHandle));
	memset(mHistogramTexcoordHandle, 0, sizeof(mHistogramTexcoordHandle));
	mHistogramFbo = 0;
	mHistogramFboTexture = 0;
	mHistogramBuffer = NULL;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		mHistogramValid[i] = false;
	memset(mHistogram, 0, sizeof(mHistogram));
	memset(mHistogramNorm, 0, sizeof(mHistogram));
	mSatCoef = 1.0f;
	mBaseSatCoef = 1.0f;
	mLightCoef = 1.0f;
	mBaseLightCoef = 1.0f;
	mDarkCoef = 1.0f;
	mBaseDarkCoef = 1.0f;
#	ifdef BCM_VIDEOCORE
	mEglImage = EGL_NO_IMAGE_KHR;
#	endif /* BCM_VIDEOCORE */

	GLCHK();

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		goto err;
	}

	glShaderSource(vertexShader, 1, &videoVertexShader, NULL);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		ULOGE("vertex shader compilation failed '%s'", infoLog);
		goto err;
	}

	fragmentShaderNoconv = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderNoconv == 0) ||
	    (fragmentShaderNoconv == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(
		fragmentShaderNoconv, 3, videoNoconvFragmentShaders, NULL);
	glCompileShader(fragmentShaderNoconv);
	glGetShaderiv(fragmentShaderNoconv, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderNoconv, 512, NULL, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	fragmentShaderYuvsp = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderYuvsp == 0) ||
	    (fragmentShaderYuvsp == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShaderYuvsp, 3, videoNV12FragmentShaders, NULL);
	glCompileShader(fragmentShaderYuvsp);
	glGetShaderiv(fragmentShaderYuvsp, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderYuvsp, 512, NULL, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	fragmentShaderYuvp = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderYuvp == 0) ||
	    (fragmentShaderYuvp == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShaderYuvp, 3, videoI420FragmentShaders, NULL);
	glCompileShader(fragmentShaderYuvp);
	glGetShaderiv(fragmentShaderYuvp, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderYuvp, 512, NULL, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] = glCreateProgram();
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		       vertexShader);
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		       fragmentShaderNoconv);
	glLinkProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]);
	glGetProgramiv(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		       GL_LINK_STATUS,
		       &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
				    512,
				    NULL,
				    infoLog);
		ULOGE("program link failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] = glCreateProgram();
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
		       vertexShader);
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
		       fragmentShaderYuvp);
	glLinkProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB]);
	glGetProgramiv(mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
		       GL_LINK_STATUS,
		       &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			512,
			NULL,
			infoLog);
		ULOGE("program link failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] = glCreateProgram();
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
		       vertexShader);
	glAttachShader(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
		       fragmentShaderYuvsp);
	glLinkProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB]);
	glGetProgramiv(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
		       GL_LINK_STATUS,
		       &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			512,
			NULL,
			infoLog);
		ULOGE("program link failed '%s'", infoLog);
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

	GLCHK();

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"transform_matrix");
	mProgramStride[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE], "stride");
	mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"sat_coef");
	mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"light_coef");
	mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"dark_coef");
	mProgramZebraEnable[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"zebra_enable");
	mProgramZebraThreshold[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"zebra_sat");
	mProgramZebraPhase[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"zebra_phase");
	mProgramZebraWeight[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"zebra_weight");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			"s_texture_0");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetAttribLocation(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
				    "position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE] =
		glGetAttribLocation(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE],
				    "texcoord");

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"transform_matrix");
	mProgramYuv2RgbMatrix[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"yuv2rgb_mat");
	mProgramYuv2RgbOffset[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"yuv2rgb_offset");
	mProgramStride[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"stride");
	mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"sat_coef");
	mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"light_coef");
	mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"dark_coef");
	mProgramZebraEnable[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"zebra_enable");
	mProgramZebraThreshold[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"zebra_sat");
	mProgramZebraPhase[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"zebra_phase");
	mProgramZebraWeight[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"zebra_weight");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"s_texture_0");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB][1] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"s_texture_1");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB][2] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"s_texture_2");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB],
			"texcoord");

	mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"transform_matrix");
	mProgramYuv2RgbMatrix[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"yuv2rgb_mat");
	mProgramYuv2RgbOffset[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"yuv2rgb_offset");
	mProgramStride[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"stride");
	mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"sat_coef");
	mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"light_coef");
	mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"dark_coef");
	mProgramZebraEnable[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"zebra_enable");
	mProgramZebraThreshold[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"zebra_sat");
	mProgramZebraPhase[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"zebra_phase");
	mProgramZebraWeight[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"zebra_weight");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB][0] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"s_texture_0");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB][1] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"s_texture_1");
	mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB][2] =
		glGetUniformLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"s_texture_2");
	mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"position");
	mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] =
		glGetAttribLocation(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB],
			"texcoord");

	GLCHK();

	ret = setupBlur();
	if (ret < 0)
		ULOG_ERRNO("setupBlur", -ret);

	GLCHK();

	ret = setupHistograms();
	if (ret < 0)
		ULOG_ERRNO("setupHistograms", -ret);

	GLCHK();

	/* Setup zebra shaders */
	setupZebra(GLES2_VIDEO_COLOR_CONVERSION_NONE);
	setupZebra(GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB);
	setupZebra(GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB);

	GLCHK(glGenTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures));

	for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
#	ifdef BCM_VIDEOCORE
		if (i == 0) {
			GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES,
					    mTextures[i]));
		} else {
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
		}
#	else /* BCM_VIDEOCORE */
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
#	endif /* BCM_VIDEOCORE */

		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	}

	GLCHK(glClear(GL_COLOR_BUFFER_BIT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, 0));

	return;

err:
	if (mTextures[0])
		GLCHK(glDeleteTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures));
	if (vertexShader)
		GLCHK(glDeleteShader(vertexShader));
	if (fragmentShaderNoconv)
		GLCHK(glDeleteShader(fragmentShaderNoconv));
	if (fragmentShaderYuvp)
		GLCHK(glDeleteShader(fragmentShaderYuvp));
	if (fragmentShaderYuvsp)
		GLCHK(glDeleteShader(fragmentShaderYuvsp));
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB]));
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB]));
	}
	memset(mProgram, 0, sizeof(mProgram));
	memset(mTextures, 0, sizeof(mTextures));
	cleanupBlur();
	cleanupHistograms();
}


Gles2Video::~Gles2Video(void)
{
    _viewDataInitialized = false;
	if (mTextures[0])
		GLCHK(glDeleteTextures(GLES2_VIDEO_TEX_UNIT_COUNT, mTextures));
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB]));
	}
	if (mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB] > 0) {
		GLCHK(glDeleteProgram(
			mProgram[GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB]));
	}

	cleanupBlur();
	cleanupPaddingFbo();
	cleanupHistograms();
}


int Gles2Video::setupBlur(void)
{
	int ret = 0;
	GLint vertexShaderH = 0, vertexShaderV = 0;
	GLint fragmentShaderH = 0, fragmentShaderV = 0;
	GLint success = 0;

	/* Free previous resources */
	cleanupBlur();

	/* Render sizes */
	mBlurFboWidth = GLES2_VIDEO_BLUR_FBO_TARGET_SIZE;
	mBlurFboHeight = GLES2_VIDEO_BLUR_FBO_TARGET_SIZE;

	/* Shaders compilation */
	vertexShaderH = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShaderH == 0) || (vertexShaderH == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(vertexShaderH, 1, &blurHVertexShader, NULL);
	glCompileShader(vertexShaderH);
	glGetShaderiv(vertexShaderH, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShaderH, 512, NULL, infoLog);
		ULOGE("vertex shader (H) compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	vertexShaderV = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShaderV == 0) || (vertexShaderV == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(vertexShaderV, 1, &blurVVertexShader, NULL);
	glCompileShader(vertexShaderV);
	glGetShaderiv(vertexShaderV, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShaderV, 512, NULL, infoLog);
		ULOGE("vertex shader (V) compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	fragmentShaderH = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderH == 0) || (fragmentShaderH == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(fragmentShaderH, 1, &blurHFragmentShader, NULL);
	glCompileShader(fragmentShaderH);
	glGetShaderiv(fragmentShaderH, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderH, 512, NULL, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	fragmentShaderV = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShaderV == 0) || (fragmentShaderV == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(fragmentShaderV, 1, &blurVFragmentShader, NULL);
	glCompileShader(fragmentShaderV);
	glGetShaderiv(fragmentShaderV, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderV, 512, NULL, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	/* Shaders link */
	mBlurProgram[0] = glCreateProgram();
	glAttachShader(mBlurProgram[0], vertexShaderH);
	glAttachShader(mBlurProgram[0], fragmentShaderH);
	glLinkProgram(mBlurProgram[0]);
	glGetProgramiv(mBlurProgram[0], GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(mBlurProgram[0], 512, NULL, infoLog);
		ULOGE("program link failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	mBlurProgram[1] = glCreateProgram();
	glAttachShader(mBlurProgram[1], vertexShaderV);
	glAttachShader(mBlurProgram[1], fragmentShaderV);
	glLinkProgram(mBlurProgram[1]);
	glGetProgramiv(mBlurProgram[1], GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetProgramInfoLog(mBlurProgram[1], 512, NULL, infoLog);
		ULOGE("program link failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	glDeleteShader(vertexShaderH);
	vertexShaderH = 0;
	glDeleteShader(vertexShaderV);
	vertexShaderV = 0;
	glDeleteShader(fragmentShaderH);
	fragmentShaderH = 0;
	glDeleteShader(fragmentShaderV);
	fragmentShaderV = 0;

	/* Attributes and uniforms handles */
	mBlurUniformPixelSize[0] =
		glGetUniformLocation(mBlurProgram[0], "pixel_size");
	mBlurUniformWeights[0] =
		glGetUniformLocation(mBlurProgram[0], "blur_weights");
	mBlurUniformSampler[0] =
		glGetUniformLocation(mBlurProgram[0], "s_texture");
	mBlurPositionHandle[0] =
		glGetAttribLocation(mBlurProgram[0], "position");
	mBlurUniformPixelSize[1] =
		glGetUniformLocation(mBlurProgram[1], "pixel_size");
	mBlurUniformWeights[1] =
		glGetUniformLocation(mBlurProgram[1], "blur_weights");
	mBlurUniformSampler[1] =
		glGetUniformLocation(mBlurProgram[1], "s_texture");
	mBlurPositionHandle[1] =
		glGetAttribLocation(mBlurProgram[1], "position");

	mBlurInit = true;
	return 0;

error:
	cleanupBlur();
	return ret;
}


void Gles2Video::cleanupBlur(void)
{
	cleanupBlurFbo();
	if (mBlurProgram[0] > 0) {
		GLCHK(glDeleteProgram(mBlurProgram[0]));
		mBlurProgram[0] = 0;
	}
	if (mBlurProgram[1] > 0) {
		GLCHK(glDeleteProgram(mBlurProgram[1]));
		mBlurProgram[1] = 0;
	}
	mBlurInit = false;
}


int Gles2Video::setupBlurFbo(unsigned int cropWidth, unsigned int cropHeight)
{
	int ret = 0;
	unsigned int i;
	GLenum gle;

	/* Free previous resources */
	cleanupBlurFbo();

	if (!mBlurInit)
		return 0;

	/* Render sizes */
	if (mVideoWidth > mVideoHeight) {
		mBlurFboWidth = GLES2_VIDEO_BLUR_FBO_TARGET_SIZE;
		mBlurFboHeight = (GLES2_VIDEO_BLUR_FBO_TARGET_SIZE *
					  mVideoHeight / mVideoWidth +
				  3) &
				 ~3;
	} else {
		mBlurFboWidth = (GLES2_VIDEO_BLUR_FBO_TARGET_SIZE *
					 mVideoWidth / mVideoHeight +
				 3) &
				~3;
		mBlurFboHeight = GLES2_VIDEO_BLUR_FBO_TARGET_SIZE;
	}

	/* Allocate FBOs and textures */
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	for (i = 0; i < 2; i++) {
		GLCHK(glGenFramebuffers(1, &mBlurFbo[i]));
		if (mBlurFbo[i] <= 0) {
			ULOGE("failed to create framebuffer");
			ret = -ENOMEM;
			goto error;
		}
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[i]));

		GLCHK(glGenTextures(1, &mBlurFboTexture[i]));
		if (mBlurFboTexture[i] <= 0) {
			ULOGE("failed to create texture");
			ret = -ENOMEM;
			goto error;
		}
		GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[i]));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_RGB,
				   mBlurFboWidth,
				   mBlurFboHeight,
				   0,
				   GL_RGB,
				   GL_UNSIGNED_BYTE,
				   NULL));

		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

		GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
					     GL_COLOR_ATTACHMENT0,
					     GL_TEXTURE_2D,
					     mBlurFboTexture[i],
					     0));

		gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (gle != GL_FRAMEBUFFER_COMPLETE) {
			ULOGE("invalid framebuffer status");
			ret = -EPROTO;
			goto error;
		}
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	return 0;

error:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	cleanupBlur();
	return ret;
}


void Gles2Video::cleanupBlurFbo(void)
{
	if (mBlurFboTexture[0] > 0) {
		GLCHK(glDeleteTextures(2, mBlurFboTexture));
		memset(mBlurFboTexture, 0, sizeof(mBlurFboTexture));
	}
	if (mBlurFbo[0] > 0) {
		GLCHK(glDeleteFramebuffers(2, mBlurFbo));
		memset(mBlurFbo, 0, sizeof(mBlurFbo));
	}
	mBlurFboWidth = 0;
	mBlurFboHeight = 0;
}


void Gles2Video::renderBlur(size_t framePlaneStride[3],
			    unsigned int frameHeight,
			    unsigned int cropLeft,
			    unsigned int cropTop,
			    unsigned int cropWidth,
			    unsigned int cropHeight,
			    const struct pdraw_rect *render_pos,
			    float videoW,
			    float videoH,
			    enum gles2_video_color_conversion colorConversion,
			    enum gles2_video_yuv_range yuvRange,
			    Eigen::Matrix4f &viewProjMat)
{
	unsigned int i;
	float stride[GLES2_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	bool mirrorTexture = false;

	if (!mBlurInit)
		return;

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[0]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	GLCHK(glUseProgram(mProgram[colorConversion]));

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
#	ifdef BCM_VIDEOCORE
		GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, mTextures[0]));
#	else /* BCM_VIDEOCORE */
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
#	endif /* BCM_VIDEOCORE */
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
				  mFirstTexUnit));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / frameHeight;
		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB:
		mirrorTexture = true;
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			int height = frameHeight / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[colorConversion][i],
					  mFirstTexUnit + i));
			stride[2 * i] = 1.f / framePlaneStride[i];
			stride[2 * i + 1] = 1.f / height;
		}

		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[colorConversion],
				  yuv2RgbOffsets[yuvRange][0],
				  yuv2RgbOffsets[yuvRange][1],
				  yuv2RgbOffsets[yuvRange][2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[colorConversion],
					 1,
					 GL_FALSE,
					 &yuv2RgbMats[yuvRange][0]));
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / frameHeight;

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / (framePlaneStride[1] / 2);
		stride[3] = 1.f / (frameHeight / 2);

		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[colorConversion],
				  yuv2RgbOffsets[yuvRange][0],
				  yuv2RgbOffsets[yuvRange][1],
				  yuv2RgbOffsets[yuvRange][2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[colorConversion],
					 1,
					 GL_FALSE,
					 &yuv2RgbMats[yuvRange][0]));
		break;
	}

	/* Disable overexposure zebras */
	updateZebra(NULL, colorConversion, false, 0.f);

	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = 1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = -1.;
	vertices[7] = 1.;
	vertices[8] = 1.;
	vertices[9] = 1.;
	vertices[10] = 1.;
	vertices[11] = 1.;

	Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[colorConversion], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramSatCoef[colorConversion], 1.f));
	GLCHK(glUniform1f(mProgramLightCoef[colorConversion], 1.f));
	GLCHK(glUniform1f(mProgramDarkCoef[colorConversion], 1.f));

	GLCHK(glVertexAttribPointer(mPositionHandle[colorConversion],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[colorConversion]));

	if (mirrorTexture) {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] = (float)cropTop / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)cropTop / (float)frameHeight;
	} else {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] = (float)cropTop / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)cropTop / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
	}

	GLCHK(glVertexAttribPointer(mTexcoordHandle[colorConversion],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[colorConversion]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[colorConversion]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[colorConversion]));

	/* Horizontal blur pass */
	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = -1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = 1.;
	vertices[7] = 1.;
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[1]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mBlurWeights));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[0], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0], 1.0 / mBlurFboWidth));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Vertical blur pass */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[0]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mBlurWeights));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mBlurFboHeight));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Render to screen */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(render_pos->x,
			 render_pos->y,
			 render_pos->width,
			 render_pos->height));
	GLCHK(glUseProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[0]));
	GLCHK(glUniform1i(
		mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][0],
		mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		1,
		false,
		viewProjMat.data()));
	GLCHK(glUniform1f(mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  mDarkCoef));

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

	texCoords[0] = 0.;
	texCoords[1] = 0.;
	texCoords[2] = 1.;
	texCoords[3] = 0.;
	texCoords[4] = 0.;
	texCoords[5] = 1.;
	texCoords[6] = 1.;
	texCoords[7] = 1.;

	GLCHK(glVertexAttribPointer(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		3,
		GL_FLOAT,
		false,
		0,
		vertices));
	GLCHK(glEnableVertexAttribArray(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

	GLCHK(glVertexAttribPointer(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		2,
		GL_FLOAT,
		false,
		0,
		texCoords));
	GLCHK(glEnableVertexAttribArray(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	GLCHK(glDisableVertexAttribArray(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

	GLCHK(glUseProgram(mProgram[colorConversion]));
}


int Gles2Video::setupPaddingFbo(unsigned int cropWidth,
				unsigned int cropHeight,
				enum pdraw_video_renderer_fill_mode fillMode)
{
	GLenum gle;
	unsigned int i;

	/* Free previous resources */
	cleanupPaddingFbo();

	if (!mBlurInit)
		return 0;

	if ((fillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP) &&
	    (fillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND))
		return 0;

	/* Render sizes */
	if (mVideoWidth > mVideoHeight) {
		mPaddingPass1Width = GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1;
		mPaddingPass1Height = (GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1 *
					       mVideoHeight / mVideoWidth +
				       3) &
				      ~3;
		mPaddingPass2Width = GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2;
		mPaddingPass2Height = (GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2 *
					       mVideoHeight / mVideoWidth +
				       3) &
				      ~3;
	} else {
		mPaddingPass1Width = (GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1 *
					      mVideoWidth / mVideoHeight +
				      3) &
				     ~3;
		mPaddingPass1Height = GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1;
		mPaddingPass2Width = (GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2 *
					      mVideoWidth / mVideoHeight +
				      3) &
				     ~3;
		mPaddingPass2Height = GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2;
	}

	pdraw_gaussianDistribution(mPaddingBlurWeights,
				   GLES2_VIDEO_BLUR_TAP_COUNT,
				   GLES2_VIDEO_BLURRED_PADDING_SIGMA);

	/* Allocate new resources */
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	for (i = 0; i < 4; i++) {
		GLCHK(glGenFramebuffers(1, &mPaddingFbo[i]));
		if (mPaddingFbo[i] <= 0) {
			ULOGE("failed to create framebuffer");
			goto err;
		}
		GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[i]));

		GLCHK(glGenTextures(1, &mPaddingFboTexture[i]));
		if (mPaddingFboTexture[i] <= 0) {
			ULOGE("failed to create texture");
			goto err;
		}
		GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[i]));
		GLCHK(glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_RGB,
			(i < 2) ? mPaddingPass1Width : mPaddingPass2Width,
			(i < 2) ? mPaddingPass1Height : mPaddingPass2Height,
			0,
			GL_RGB,
			GL_UNSIGNED_BYTE,
			NULL));

		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

		GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
					     GL_COLOR_ATTACHMENT0,
					     GL_TEXTURE_2D,
					     mPaddingFboTexture[i],
					     0));

		gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (gle != GL_FRAMEBUFFER_COMPLETE) {
			ULOGE("invalid framebuffer status");
			goto err;
		}
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	return 0;

err:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	cleanupPaddingFbo();
	return -EPROTO;
}


void Gles2Video::cleanupPaddingFbo(void)
{
	if (mPaddingFboTexture[0] > 0) {
		GLCHK(glDeleteTextures(4, mPaddingFboTexture));
		memset(mPaddingFboTexture, 0, sizeof(mPaddingFboTexture));
	}
	if (mPaddingFbo[0] > 0) {
		GLCHK(glDeleteFramebuffers(4, mPaddingFbo));
		memset(mPaddingFbo, 0, sizeof(mPaddingFbo));
	}
	mPaddingPass1Width = 0;
	mPaddingPass1Height = 0;
	mPaddingPass2Width = 0;
	mPaddingPass2Height = 0;
}


void Gles2Video::renderPadding(
	size_t framePlaneStride[3],
	unsigned int frameHeight,
	unsigned int cropLeft,
	unsigned int cropTop,
	unsigned int cropWidth,
	unsigned int cropHeight,
	const struct pdraw_rect *renderPos,
	float videoW,
	float videoH,
	float videoW2,
	float videoH2,
	float videoAR,
	float windowAR,
	enum pdraw_video_renderer_fill_mode fillMode,
	enum gles2_video_color_conversion colorConversion,
	enum gles2_video_yuv_range yuvRange,
	bool immersive,
	Eigen::Matrix4f &viewProjMat)
{
	unsigned int i;
	float stride[GLES2_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	bool mirrorTexture = false;

	if (!mBlurInit)
		return;

	if ((fillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP) &&
	    (fillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND))
		return;

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[0]));
	GLCHK(glViewport(0, 0, mPaddingPass1Width, mPaddingPass1Height));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));

	GLCHK(glUseProgram(mProgram[colorConversion]));

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
#	ifdef BCM_VIDEOCORE
		GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, mTextures[0]));
#	else /* BCM_VIDEOCORE */
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
#	endif /* BCM_VIDEOCORE */
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
				  mFirstTexUnit));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / frameHeight;
		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB:
		mirrorTexture = true;
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			int height = frameHeight / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[colorConversion][i],
					  mFirstTexUnit + i));
			stride[2 * i] = 1.f / framePlaneStride[i];
			stride[2 * i + 1] = 1.f / height;
		}

		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[colorConversion],
				  yuv2RgbOffsets[yuvRange][0],
				  yuv2RgbOffsets[yuvRange][1],
				  yuv2RgbOffsets[yuvRange][2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[colorConversion],
					 1,
					 GL_FALSE,
					 &yuv2RgbMats[yuvRange][0]));
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / frameHeight;

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / (framePlaneStride[1] / 2);
		stride[3] = 1.f / (frameHeight / 2);

		GLCHK(glUniform2fv(mProgramStride[colorConversion],
				   GLES2_VIDEO_TEX_UNIT_COUNT,
				   stride));
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[colorConversion],
				  yuv2RgbOffsets[yuvRange][0],
				  yuv2RgbOffsets[yuvRange][1],
				  yuv2RgbOffsets[yuvRange][2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[colorConversion],
					 1,
					 GL_FALSE,
					 &yuv2RgbMats[yuvRange][0]));
		break;
	}

	/* Disable overexposure zebras */
	updateZebra(NULL, colorConversion, false, 0.f);

	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = 1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = -1.;
	vertices[7] = 1.;
	vertices[8] = 1.;
	vertices[9] = 1.;
	vertices[10] = 1.;
	vertices[11] = 1.;

	Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[colorConversion], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramSatCoef[colorConversion], 1.f));
	GLCHK(glUniform1f(mProgramLightCoef[colorConversion], 1.f));
	GLCHK(glUniform1f(mProgramDarkCoef[colorConversion], 1.f));

	GLCHK(glVertexAttribPointer(mPositionHandle[colorConversion],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[colorConversion]));

	if (mirrorTexture) {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] = (float)cropTop / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)cropTop / (float)frameHeight;
	} else {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] = (float)cropTop / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)cropTop / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
	}

	GLCHK(glVertexAttribPointer(mTexcoordHandle[colorConversion],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[colorConversion]));

    //dgd:
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[colorConversion]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[colorConversion]));

	/* Pass 1 horizontal blur */
	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = -1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = 1.;
	vertices[7] = 1.;
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[1]));
	GLCHK(glViewport(0, 0, mPaddingPass1Width, mPaddingPass1Height));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[0], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0], 1.0 / mPaddingPass1Width));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 1 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mPaddingPass1Height));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Pass 2 horizontal blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[3]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[2]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[0], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0], 1.0 / mPaddingPass2Width));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 2 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[3]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mPaddingPass2Height));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GLES2_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Render to screen */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(renderPos->x,
			 renderPos->y,
			 renderPos->width,
			 renderPos->height));
	GLCHK(glUseProgram(mProgram[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[2]));
	GLCHK(glUniform1i(
		mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_NONE][0],
		mFirstTexUnit + GLES2_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		1,
		false,
		viewProjMat.data()));
	GLCHK(glUniform1f(mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_NONE],
			  (immersive) ? mDarkCoef
				      : PDRAW_BLURRED_PADDING_DARK_COEF *
						mDarkCoef));

	vertices[0] = -videoW2;
	vertices[1] = -videoH2;
	vertices[2] = 1.;
	vertices[3] = videoW2;
	vertices[4] = -videoH2;
	vertices[5] = 1.;
	vertices[6] = -videoW2;
	vertices[7] = videoH2;
	vertices[8] = 1.;
	vertices[9] = videoW2;
	vertices[10] = videoH2;
	vertices[11] = 1.;

	if (fillMode == PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND) {
		texCoords[0] = -(videoW2 / videoW / 2.) + 0.5;
		texCoords[1] = -(videoH2 / videoH / 2.) + 0.5;
		texCoords[2] = videoW2 / videoW / 2. + 0.5;
		texCoords[3] = -(videoH2 / videoH / 2.) + 0.5;
		texCoords[4] = -(videoW2 / videoW / 2.) + 0.5;
		texCoords[5] = videoH2 / videoH / 2. + 0.5;
		texCoords[6] = videoW2 / videoW / 2. + 0.5;
		texCoords[7] = videoH2 / videoH / 2. + 0.5;
	} else {
		texCoords[0] = 0.;
		texCoords[1] = 0.;
		texCoords[2] = 1.;
		texCoords[3] = 0.;
		texCoords[4] = 0.;
		texCoords[5] = 1.;
		texCoords[6] = 1.;
		texCoords[7] = 1.;
	}

	GLCHK(glVertexAttribPointer(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		3,
		GL_FLOAT,
		false,
		0,
		vertices));
	GLCHK(glEnableVertexAttribArray(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

	GLCHK(glVertexAttribPointer(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE],
		2,
		GL_FLOAT,
		false,
		0,
		texCoords));
	GLCHK(glEnableVertexAttribArray(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(
		mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));
	GLCHK(glDisableVertexAttribArray(
		mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_NONE]));

	GLCHK(glUseProgram(mProgram[colorConversion]));
}


void Gles2Video::setupZebra(enum gles2_video_color_conversion colorConversion)
{
	float co = cosf(PDRAW_ZEBRA_ANGLE);
	float si = sinf(PDRAW_ZEBRA_ANGLE);
	const GLfloat zebra_mat[] = {co, si, -si, co};

	GLCHK(glUseProgram(mProgram[colorConversion]));
	GLCHK(glUniformMatrix2fv(
		glGetUniformLocation(mProgram[colorConversion], "zebra_mat"),
		1,
		GL_FALSE,
		zebra_mat));
	GLCHK(glUniform1fv(glGetUniformLocation(mProgram[colorConversion],
						"zebra_avg_weights"),
			   9,
			   zebraAvgWeights));
}


void Gles2Video::updateZebra(struct pdraw_rect *contentPos,
			     enum gles2_video_color_conversion colorConversion,
			     bool enable,
			     float threshold)
{
	GLCHK(glUniform1f(mProgramZebraEnable[colorConversion],
			  enable ? 1.f : 0.f));
	GLCHK(glUniform1f(mProgramZebraThreshold[colorConversion], threshold));

	if (enable && contentPos != NULL) {
		struct timespec ts;
		uint64_t time_us;
		if (time_get_monotonic(&ts) < 0) {
			ULOGE("time_get_monotonic");
			return;
		}
		if (time_timespec_to_us(&ts, &time_us) < 0) {
			ULOGE("time_timespec_to_us");
			return;
		}
		uint64_t zebra_period_us =
			(uint64_t)(1000000.f / PDRAW_ZEBRA_FREQUENCY_HZ);
		float zebra_phase =
			(float)(time_us % zebra_period_us) / zebra_period_us;
		GLCHK(glUniform1f(mProgramZebraPhase[colorConversion],
				  zebra_phase));
		float zebra_weight =
			PDRAW_ZEBRA_WEIGHT * contentPos->width / 1920;
		GLCHK(glUniform1f(mProgramZebraWeight[colorConversion],
				  zebra_weight));
	}
}


int Gles2Video::setupHistograms(void)
{
	int ret = 0;
	GLenum gle;
	GLint success = 0;
	GLint vertexShaderHistogram = 0;
	GLint fragmentShaderHistogram[GLES2_VIDEO_COLOR_CONVERSION_MAX] = {0};
	unsigned int i;

	/* Buffers allocation */
	mHistogramBuffer =
		(uint8_t *)malloc(4 * GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
				  GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE);
	if (mHistogramBuffer == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		ret = -ENOMEM;
		goto error;
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		mHistogram[i] = (uint32_t *)malloc(256 * sizeof(uint32_t));
		if (mHistogram[i] == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			ret = -ENOMEM;
			goto error;
		}
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		mHistogramNorm[i] = (float *)calloc(256, sizeof(float));
		if (mHistogramNorm[i] == NULL) {
			ULOG_ERRNO("calloc", ENOMEM);
			ret = -ENOMEM;
			goto error;
		}
	}

	/* Shaders compilation */
	vertexShaderHistogram = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShaderHistogram == 0) ||
	    (vertexShaderHistogram == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(vertexShaderHistogram, 1, &histogramVertexShader, NULL);
	glCompileShader(vertexShaderHistogram);
	glGetShaderiv(vertexShaderHistogram, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShaderHistogram, 512, NULL, infoLog);
		ULOGE("vertex shader compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		fragmentShaderHistogram[i] = glCreateShader(GL_FRAGMENT_SHADER);
		if ((fragmentShaderHistogram[i] == 0) ||
		    (fragmentShaderHistogram[i] == GL_INVALID_ENUM)) {
			ULOGE("failed to create fragment shader");
			ret = -ENOMEM;
			goto error;
		}

		glShaderSource(fragmentShaderHistogram[i],
			       2,
			       histogramFragmentShaders[i],
			       NULL);
		glCompileShader(fragmentShaderHistogram[i]);
		glGetShaderiv(fragmentShaderHistogram[i],
			      GL_COMPILE_STATUS,
			      &success);
		if (!success) {
			GLchar infoLog[512];
			glGetShaderInfoLog(
				fragmentShaderHistogram[i], 512, NULL, infoLog);
			ULOGE("fragment shader compilation failed '%s'",
			      infoLog);
			ret = -EPROTO;
			goto error;
		}
	}

	/* Shaders link */
	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		mHistogramProgram[i] = glCreateProgram();
		glAttachShader(mHistogramProgram[i], vertexShaderHistogram);
		glAttachShader(mHistogramProgram[i],
			       fragmentShaderHistogram[i]);
		glLinkProgram(mHistogramProgram[i]);
		glGetProgramiv(mHistogramProgram[i], GL_LINK_STATUS, &success);
		if (!success) {
			GLchar infoLog[512];
			glGetProgramInfoLog(
				mHistogramProgram[i], 512, NULL, infoLog);
			ULOGE("program link failed '%s'", infoLog);
			ret = -EPROTO;
			goto error;
		}
	}

	glDeleteShader(vertexShaderHistogram);
	vertexShaderHistogram = 0;
	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		glDeleteShader(fragmentShaderHistogram[i]);
		fragmentShaderHistogram[i] = 0;
	}

	/* Uniforms and attribs */
	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		mHistogramYuv2RgbMatrix[i] = glGetUniformLocation(
			mHistogramProgram[i], "yuv2rgb_mat");
		mHistogramYuv2RgbOffset[i] = glGetUniformLocation(
			mHistogramProgram[i], "yuv2rgb_offset");
		mHistogramUniformSampler[i][0] = glGetUniformLocation(
			mHistogramProgram[i], "s_texture_0");
		mHistogramUniformSampler[i][1] = glGetUniformLocation(
			mHistogramProgram[i], "s_texture_1");
		mHistogramUniformSampler[i][2] = glGetUniformLocation(
			mHistogramProgram[i], "s_texture_2");
		mHistogramPositionHandle[i] =
			glGetAttribLocation(mHistogramProgram[i], "position");
		mHistogramTexcoordHandle[i] =
			glGetAttribLocation(mHistogramProgram[i], "texcoord");
	}

	/* Create the framebuffer */
	GLCHK(glGenFramebuffers(1, &mHistogramFbo));
	if (mHistogramFbo <= 0) {
		ULOGE("failed to create framebuffer");
		ret = -ENOMEM;
		goto error;
	}
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHistogramFbo));

	GLCHK(glGenTextures(1, &mHistogramFboTexture));
	if (mHistogramFboTexture <= 0) {
		ULOGE("failed to create texture");
		ret = -ENOMEM;
		goto error;
	}
	GLCHK(glBindTexture(GL_TEXTURE_2D, mHistogramFboTexture));
	GLCHK(glTexImage2D(GL_TEXTURE_2D,
			   0,
			   GL_RGBA,
			   GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   0,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   NULL));

	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER,
				     GL_COLOR_ATTACHMENT0,
				     GL_TEXTURE_2D,
				     mHistogramFboTexture,
				     0));

	gle = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (gle != GL_FRAMEBUFFER_COMPLETE) {
		ULOGE("invalid framebuffer status");
		ret = -EPROTO;
		goto error;
	}

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	mHistogramInit = true;
	return 0;

error:
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));

	if (vertexShaderHistogram)
		glDeleteShader(vertexShaderHistogram);
	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		if (fragmentShaderHistogram[i])
			glDeleteShader(fragmentShaderHistogram[i]);
	}

	cleanupHistograms();

	return ret;
}


void Gles2Video::cleanupHistograms(void)
{
	unsigned int i;

	if (mHistogramFboTexture > 0) {
		GLCHK(glDeleteTextures(1, &mHistogramFboTexture));
		mHistogramFboTexture = 0;
	}
	if (mHistogramFbo > 0) {
		GLCHK(glDeleteFramebuffers(1, &mHistogramFbo));
		mHistogramFbo = 0;
	}
	for (i = 0; i < GLES2_VIDEO_COLOR_CONVERSION_MAX; i++) {
		if (mHistogramProgram[i] > 0) {
			GLCHK(glDeleteProgram(mHistogramProgram[i]));
			mHistogramProgram[i] = 0;
		}
	}
	free(mHistogramBuffer);
	mHistogramBuffer = NULL;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		free(mHistogram[i]);
		mHistogram[i] = NULL;
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		free(mHistogramNorm[i]);
		mHistogramNorm[i] = NULL;
	}
	mHistogramInit = false;
}


void Gles2Video::computeHistograms(
	size_t framePlaneStride[3],
	unsigned int frameHeight,
	unsigned int cropLeft,
	unsigned int cropTop,
	unsigned int cropWidth,
	unsigned int cropHeight,
	const struct pdraw_rect *renderPos,
	enum gles2_video_color_conversion colorConversion,
	enum gles2_video_yuv_range yuvRange,
	bool enable)
{
	float vertices[12];
	float texCoords[8];
	unsigned int i, j;
	uint8_t *buf;
	uint32_t histoMax[PDRAW_HISTOGRAM_CHANNEL_MAX];
	struct timespec ts;
	uint64_t time_us;
	bool mirrorTexture = false;

	if ((!mHistogramInit) || (!enable)) {
		mHistogramLastComputeTime = 0;
		for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
			mHistogramValid[i] = false;
		return;
	}

	if (time_get_monotonic(&ts) < 0) {
		ULOGE("time_get_monotonic");
		return;
	}
	if (time_timespec_to_us(&ts, &time_us) < 0) {
		ULOGE("time_timespec_to_us");
		return;
	}
	if ((mHistogramLastComputeTime > 0) &&
	    (time_us < mHistogramLastComputeTime +
			       GLES2_VIDEO_HISTOGRAM_COMPUTE_INTERVAL_US))
		return;
	mHistogramLastComputeTime = time_us;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		mHistogramValid[i] = false;

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHistogramFbo));
	GLCHK(glDisable(GL_BLEND));
	GLCHK(glViewport(0,
			 0,
			 GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			 GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE));
	GLCHK(glClear(GL_COLOR_BUFFER_BIT));
	GLCHK(glUseProgram(mHistogramProgram[colorConversion]));

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
#	ifdef BCM_VIDEOCORE
		GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, mTextures[0]));
#	else /* BCM_VIDEOCORE */
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
#	endif /* BCM_VIDEOCORE */
		GLCHK(glUniform1i(mHistogramUniformSampler[colorConversion][0],
				  mFirstTexUnit));
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB:
		mirrorTexture = true;
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(
				mHistogramUniformSampler[colorConversion][i],
				mFirstTexUnit + i));
		}
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mHistogramUniformSampler[colorConversion][0],
				  mFirstTexUnit + 0));

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mHistogramUniformSampler[colorConversion][1],
				  mFirstTexUnit + 1));
		break;
	}

	GLCHK(glUniform3f(mHistogramYuv2RgbOffset[colorConversion],
			  yuv2RgbOffsets[yuvRange][0],
			  yuv2RgbOffsets[yuvRange][1],
			  yuv2RgbOffsets[yuvRange][2]));
	GLCHK(glUniformMatrix3fv(mHistogramYuv2RgbMatrix[colorConversion],
				 1,
				 GL_FALSE,
				 &yuv2RgbMats[yuvRange][0]));

	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = 1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = -1.;
	vertices[7] = 1.;
	vertices[8] = 1.;
	vertices[9] = 1.;
	vertices[10] = 1.;
	vertices[11] = 1.;

	GLCHK(glVertexAttribPointer(mHistogramPositionHandle[colorConversion],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices));
	GLCHK(glEnableVertexAttribArray(
		mHistogramPositionHandle[colorConversion]));

	if (mirrorTexture) {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] = (float)cropTop / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)cropTop / (float)frameHeight;
	} else {
		texCoords[0] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[1] = (float)cropTop / (float)frameHeight;
		texCoords[2] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)cropTop / (float)frameHeight;
		texCoords[4] = (float)cropLeft / (float)framePlaneStride[0];
		texCoords[5] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
		texCoords[6] = (float)(cropLeft + cropWidth) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)(cropTop + cropHeight) / (float)frameHeight;
	}

	GLCHK(glVertexAttribPointer(mHistogramTexcoordHandle[colorConversion],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(
		mHistogramTexcoordHandle[colorConversion]));

    //dgd
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(
		mHistogramPositionHandle[colorConversion]));
	GLCHK(glDisableVertexAttribArray(
		mHistogramTexcoordHandle[colorConversion]));

	GLCHK(glFinish());

	/* Read pixels to CPU buffer */
	GLCHK(glReadPixels(0,
			   0,
			   GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   mHistogramBuffer));

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(renderPos->x,
			 renderPos->y,
			 renderPos->width,
			 renderPos->height));

	/* Reset the histograms */
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		memset(mHistogram[i], 0, 256 * sizeof(uint32_t));

	/* Count the values */
	for (j = 0, buf = mHistogramBuffer;
	     j < GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
			 GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE;
	     j++) {
		for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
			mHistogram[i][*buf++]++;
	}

	/* Histograms normalization */
	memset(histoMax, 0, sizeof(histoMax));
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		for (j = 0; j < 256; j++) {
			if (mHistogram[i][j] > histoMax[i])
				histoMax[i] = mHistogram[i][j];
		}
	}
	histoMax[1] = (histoMax[0] > histoMax[1]) ? histoMax[0] : histoMax[1];
	histoMax[1] = (histoMax[1] > histoMax[2]) ? histoMax[1] : histoMax[2];
	histoMax[2] = histoMax[1];
	histoMax[0] = histoMax[1];
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		if (histoMax[i] == 0.) {
			memset(mHistogramNorm[i], 0, 256 * sizeof(float));
			continue;
		}
		for (j = 0; j < 256; j++) {
			mHistogramNorm[i][j] =
				(float)mHistogram[i][j] / (float)histoMax[i];
		}
		mHistogramValid[i] = true;
	}
}


void Gles2Video::getHistograms(float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX],
			       size_t histogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX])
{
	unsigned int i;

	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		if ((mHistogramValid[i]) && (mHistogramNorm[i] != NULL)) {
			histogram[i] = mHistogramNorm[i];
			histogramLen[i] = 256;
		}
	}
}


void Gles2Video::startTransition(enum gles2_video_transition transition,
				 uint64_t duration,
				 bool hold)
{
	if (mTransition != GLES2_VIDEO_TRANSITION_NONE)
		abortTransition();
	mTransition = transition;
	mTransitionDuration = duration;
	mTransitionHold = hold;
}


void Gles2Video::abortTransition(void)
{
	mTransition = GLES2_VIDEO_TRANSITION_NONE;
	mTransitionDuration = 0;
	mTransitionStartTime = 0;
	mTransitionHold = false;
}


void Gles2Video::updateTransition(void)
{
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	float progress, blurSigma;

	mApplyBlur = false;
	mSatCoef = mBaseSatCoef;
	mLightCoef = mBaseLightCoef;
	mDarkCoef = mBaseDarkCoef;

	if (mTransition == GLES2_VIDEO_TRANSITION_NONE)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0)
		ULOG_ERRNO("time_timespec_to_us", -res);
	if (mTransitionStartTime == 0)
		mTransitionStartTime = curTime;

	progress = (float)(curTime - mTransitionStartTime) /
		   (float)mTransitionDuration;
	if (progress > 1.0) {
		progress = 1.0;
		if (!mTransitionHold) {
			/* Transition finished */
			abortTransition();
			return;
		}
	}

	switch (mTransition) {
	case GLES2_VIDEO_TRANSITION_FADE_TO_BLACK:
		mDarkCoef = mBaseDarkCoef * (1. - progress);
		break;
	case GLES2_VIDEO_TRANSITION_FADE_FROM_BLACK:
		mDarkCoef = mBaseDarkCoef * progress;
		break;
	case GLES2_VIDEO_TRANSITION_FADE_TO_WHITE:
		mLightCoef = mBaseLightCoef * (1. - progress);
		break;
	case GLES2_VIDEO_TRANSITION_FADE_FROM_WHITE:
		mLightCoef = mBaseLightCoef * progress;
		break;
	case GLES2_VIDEO_TRANSITION_FADE_TO_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * (1. - progress);
		break;
	case GLES2_VIDEO_TRANSITION_FADE_FROM_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * progress;
		break;
	case GLES2_VIDEO_TRANSITION_FADE_TO_BLUR:
		blurSigma = progress * (GLES2_VIDEO_BLUR_MAX_SIGMA -
					GLES2_VIDEO_BLUR_MIN_SIGMA) +
			    GLES2_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(
			mBlurWeights, GLES2_VIDEO_BLUR_TAP_COUNT, blurSigma);
		mApplyBlur = mBlurInit;
		break;
	case GLES2_VIDEO_TRANSITION_FADE_FROM_BLUR:
		blurSigma = (1. - progress) * (GLES2_VIDEO_BLUR_MAX_SIGMA -
					       GLES2_VIDEO_BLUR_MIN_SIGMA) +
			    GLES2_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(
			mBlurWeights, GLES2_VIDEO_BLUR_TAP_COUNT, blurSigma);
		mApplyBlur = mBlurInit;
		break;
	default:
		ULOGE("unsupported transition type: %d", mTransition);
		break;
	}
}


int Gles2Video::loadFrame(const uint8_t *frameData,
			  size_t framePlaneOffset[3],
			  size_t framePlaneStride[3],
			  unsigned int frameWidth,
			  unsigned int frameHeight,
			  enum gles2_video_color_conversion colorConversion,
			  struct egl_display *eglDisplay)
{
	unsigned int i;

	if ((frameWidth == 0) || (frameHeight == 0)) {
		ULOGE("invalid dimensions");
		return -EINVAL;
	}

	GLCHK(glUseProgram(mProgram[colorConversion]));

	switch (colorConversion) {
	default:
	case GLES2_VIDEO_COLOR_CONVERSION_NONE: {
#	ifdef BCM_VIDEOCORE
		EGLDisplay display = (EGLDisplay)eglDisplay;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, mTextures[0]));
		if (mEglImage != EGL_NO_IMAGE_KHR) {
			eglDestroyImageKHR(display, mEglImage);
			mEglImage = EGL_NO_IMAGE_KHR;
		}
		mEglImage = eglCreateImageKHR(display,
					      EGL_NO_CONTEXT,
					      EGL_IMAGE_BRCM_MULTIMEDIA,
					      (EGLClientBuffer)frameData,
					      NULL);
		if (mEglImage == EGL_NO_IMAGE_KHR) {
			ULOGE("failed to create EGLImage");
			return -EPROTO;
		}
		GLCHK(glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES,
						   mEglImage));
		GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
				  mFirstTexUnit));
#	endif /* BCM_VIDEOCORE */
		break;
	}
	case GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB:
		if ((framePlaneOffset == NULL) || (framePlaneStride == NULL)) {
			ULOGE("invalid planes");
			return -EINVAL;
		}
		for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
			int height = frameHeight / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glTexImage2D(GL_TEXTURE_2D,
					   0,
					   GL_LUMINANCE,
					   framePlaneStride[i],
					   height,
					   0,
					   GL_LUMINANCE,
					   GL_UNSIGNED_BYTE,
					   frameData + framePlaneOffset[i]));
		}
		break;
	case GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB:
		if ((framePlaneOffset == NULL) || (framePlaneStride == NULL)) {
			ULOGE("invalid planes");
			return -EINVAL;
		}
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE,
				   framePlaneStride[0],
				   frameHeight,
				   0,
				   GL_LUMINANCE,
				   GL_UNSIGNED_BYTE,
				   frameData + framePlaneOffset[0]));

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE_ALPHA,
				   framePlaneStride[1] / 2,
				   frameHeight / 2,
				   0,
				   GL_LUMINANCE_ALPHA,
				   GL_UNSIGNED_BYTE,
				   frameData + framePlaneOffset[1]));
		break;
	}

	return 0;
}


int Gles2Video::renderFrame(size_t framePlaneStride[3],
			    unsigned int frameHeight,
			    unsigned int cropLeft,
			    unsigned int cropTop,
			    unsigned int cropWidth,
			    unsigned int cropHeight,
			    unsigned int sarWidth,
			    unsigned int sarHeight,
			    const struct pdraw_rect *renderPos,
			    struct pdraw_rect *contentPos,
			    Eigen::Matrix4f &viewProjMat,
			    enum gles2_video_color_conversion colorConversion,
			    enum gles2_video_yuv_range yuvRange,
			    const struct vmeta_frame *metadata,
			    const struct pdraw_video_renderer_params *params)
{
//    if(dgdframeMeta != NULL){
//        float v_fov = dgdframeMeta->v3.base.picture_vfov;
//    }
	int ret;
	unsigned int i;
	float stride[GLES2_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	bool mirrorTexture = false;
	float videoAR;

	if (renderPos == NULL) {
		ULOGE("invalid render position");
		return -EINVAL;
	}

	if ((cropWidth == 0) || (cropHeight == 0) || (sarWidth == 0) ||
	    (sarHeight == 0) || (renderPos->width == 0) ||
	    (renderPos->height == 0) || (framePlaneStride[0] == 0)) {
		ULOGE("invalid dimensions");
		return -EINVAL;
	}

	if ((mVideoWidth != cropWidth) || (mVideoHeight != cropHeight)) {
		mVideoWidth = cropWidth;
		mVideoHeight = cropHeight;
		ret = setupBlurFbo(mVideoWidth, mVideoHeight);
		if (ret < 0)
			ULOG_ERRNO("setupBlurFbo", -ret);
		ret = setupPaddingFbo(cropWidth, cropHeight, params->fill_mode);
		if (ret < 0)
			ULOG_ERRNO("setupPaddingFbo", -ret);
	}

	updateTransition();

	computeHistograms(framePlaneStride,
			  frameHeight,
			  cropLeft,
			  cropTop,
			  cropWidth,
			  cropHeight,
			  renderPos,
			  colorConversion,
			  yuvRange,
			  params->enable_histograms);

	/* Video fill mode */
	float windowAR = (float)renderPos->width / (float)renderPos->height;
	float sar = (float)sarWidth / (float)sarHeight;
	if (params->video_texture_dar_height != 0 &&
	    params->video_texture_dar_width != 0) {
		/* If the display aspect ratio is given,
		 * we apply it instead of the source width/height */
		videoAR = (float)params->video_texture_dar_width /
			  (float)params->video_texture_dar_height;
	} else {
		videoAR = (float)cropWidth / (float)cropHeight * sar;
	}

	float windowW = 1.;
	float windowH = windowAR;
	float ratioW = 1.;
	float ratioH = 1.;
	float ratioW2 = 1.;
	float ratioH2 = 1.;
	float videoW, videoH;
	switch (params->fill_mode) {
	default:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND:
		/* Maintain video aspect ratio without crop and add
		 * black borders if window aspect ratio and video
		 * aspect ratio differ */
		if (videoAR >= windowAR) {
			ratioW = 1.;
			ratioH = windowAR / videoAR;
			ratioW2 = videoAR / windowAR;
			ratioH2 = 1.;
		} else {
			ratioW = videoAR / windowAR;
			ratioH = 1.;
			ratioW2 = 1.;
			ratioH2 = windowAR / videoAR;
		}
		break;
	case PDRAW_VIDEO_RENDERER_FILL_MODE_CROP:
		/* Maintain video aspect ratio without black borders
		 * and add video crop if window aspect ratio and video
		 * aspect ratio differ */
		if (videoAR >= windowAR) {
			ratioW = videoAR / windowAR;
			ratioH = 1.;
		} else {
			ratioW = 1.;
			ratioH = windowAR / videoAR;
		}
		break;
	}
	videoW = ratioW / windowW * params->video_scale_factor;
	videoH = ratioH / windowH * params->video_scale_factor;
	float videoW2 = ratioW2 / windowW;
	float videoH2 = ratioH2 / windowH;

	if (contentPos) {
		int32_t dw;
		int32_t dh;
		contentPos->width =
			ratioW * renderPos->width * params->video_scale_factor;
		contentPos->height =
			ratioH * renderPos->height * params->video_scale_factor;

		dw = (int32_t)renderPos->width - (int32_t)contentPos->width;
		dh = (int32_t)renderPos->height - (int32_t)contentPos->height;
		contentPos->x = dw / 2;
		contentPos->y = dh / 2;
	}


	float hFov = 0.;
	float vFov = 0.;
	if (mMedia) {
		hFov = mMedia->hfov;
		vFov = mMedia->vfov;
	}
	if (hFov == 0.)
		hFov = PDRAW_GLES2_VIDEO_DEFAULT_HFOV;
	if (vFov == 0.)
		vFov = PDRAW_GLES2_VIDEO_DEFAULT_VFOV;
	hFov = hFov * M_PI / 180.;
	vFov = vFov * M_PI / 180.;

	GLCHK(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

	renderPadding(framePlaneStride,
		      frameHeight,
		      cropLeft,
		      cropTop,
		      cropWidth,
		      cropHeight,
		      renderPos,
		      videoW,
		      videoH,
		      videoW2,
		      videoH2,
		      videoAR,
		      windowAR,
		      params->fill_mode,
		      colorConversion,
		      yuvRange,
		      false,
		      viewProjMat);

	if (mApplyBlur) {
		renderBlur(framePlaneStride,
			   frameHeight,
			   cropLeft,
			   cropTop,
			   cropWidth,
			   cropHeight,
			   renderPos,
			   videoW,
			   videoH,
			   colorConversion,
			   yuvRange,
			   viewProjMat);
	} else {
		GLCHK(glUseProgram(mProgram[colorConversion]));

		switch (colorConversion) {
		default:
		case GLES2_VIDEO_COLOR_CONVERSION_NONE:
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
#	ifdef BCM_VIDEOCORE
			GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES,
					    mTextures[0]));
#	else /* BCM_VIDEOCORE */
			GLCHK(glBindTexture(GL_TEXTURE_2D,
					    (mExtTexture > 0) ? mExtTexture
							      : mTextures[0]));
#	endif /* BCM_VIDEOCORE */
			GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
					  mFirstTexUnit));
			stride[0] = 1.f / framePlaneStride[0];
			stride[1] = 1.f / frameHeight;
			GLCHK(glUniform2fv(mProgramStride[colorConversion],
					   GLES2_VIDEO_TEX_UNIT_COUNT,
					   stride));
			break;
		case GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB:
			mirrorTexture = true;
			for (i = 0; i < GLES2_VIDEO_TEX_UNIT_COUNT; i++) {
				int height = frameHeight / ((i > 0) ? 2 : 1);
				GLCHK(glActiveTexture(GL_TEXTURE0 +
						      mFirstTexUnit + i));
				GLCHK(glBindTexture(GL_TEXTURE_2D,
						    mTextures[i]));
				GLCHK(glUniform1i(
					mUniformSamplers[colorConversion][i],
					mFirstTexUnit + i));
				stride[2 * i] = 1.f / framePlaneStride[i];
				stride[2 * i + 1] = 1.f / height;
			}

			GLCHK(glUniform2fv(mProgramStride[colorConversion],
					   GLES2_VIDEO_TEX_UNIT_COUNT,
					   stride));
			GLCHK(glUniform3f(
				mProgramYuv2RgbOffset[colorConversion],
				yuv2RgbOffsets[yuvRange][0],
				yuv2RgbOffsets[yuvRange][1],
				yuv2RgbOffsets[yuvRange][2]));
			GLCHK(glUniformMatrix3fv(
				mProgramYuv2RgbMatrix[colorConversion],
				1,
				GL_FALSE,
				&yuv2RgbMats[yuvRange][0]));
			break;
		case GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB:
			mirrorTexture = true;
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
			GLCHK(glUniform1i(mUniformSamplers[colorConversion][0],
					  mFirstTexUnit + 0));
			stride[0] = 1.f / framePlaneStride[0];
			stride[1] = 1.f / frameHeight;

			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
			GLCHK(glUniform1i(mUniformSamplers[colorConversion][1],
					  mFirstTexUnit + 1));
			stride[2] = 1.f / (framePlaneStride[1] / 2);
			stride[3] = 1.f / (frameHeight / 2);

			GLCHK(glUniform2fv(mProgramStride[colorConversion],
					   GLES2_VIDEO_TEX_UNIT_COUNT,
					   stride));
			GLCHK(glUniform3f(
				mProgramYuv2RgbOffset[colorConversion],
				yuv2RgbOffsets[yuvRange][0],
				yuv2RgbOffsets[yuvRange][1],
				yuv2RgbOffsets[yuvRange][2]));
			GLCHK(glUniformMatrix3fv(
				mProgramYuv2RgbMatrix[colorConversion],
				1,
				GL_FALSE,
				&yuv2RgbMats[yuvRange][0]));
			break;
		}

		/* Update overexposure zebras */
		updateZebra(contentPos,
			    colorConversion,
			    params->enable_overexposure_zebras,
			    params->overexposure_zebras_threshold);

		GLCHK(glUniformMatrix4fv(
			mProgramTransformMatrix[colorConversion],
			1,
			false,
			viewProjMat.data()));
		GLCHK(glUniform1f(mProgramSatCoef[colorConversion], mSatCoef));
		GLCHK(glUniform1f(mProgramLightCoef[colorConversion],
				  mLightCoef));
		GLCHK(glUniform1f(mProgramDarkCoef[colorConversion],
				  mDarkCoef));

		vertices[0] = -videoW;
		vertices[1] = -videoH;
		vertices[2] = 1.;
		vertices[3] = videoW;
		vertices[4] = -videoH;
		vertices[5] = 1.;
		vertices[6] = -videoW;
		vertices[7] = videoH ;
		vertices[8] = 1.;
		vertices[9] = videoW;
		vertices[10] = videoH ;
		vertices[11] = 1.;

		GLCHK(glVertexAttribPointer(mPositionHandle[colorConversion],
					    3,
					    GL_FLOAT,
					    false,
					    0,
					    vertices));
		GLCHK(glEnableVertexAttribArray(
			mPositionHandle[colorConversion]));

		if (mirrorTexture) {
			texCoords[0] =
				(float)cropLeft / (float)framePlaneStride[0];
			texCoords[1] = (float)(cropTop + cropHeight) /
				       (float)frameHeight;
			texCoords[2] = (float)(cropLeft + cropWidth) /
				       (float)framePlaneStride[0];
			texCoords[3] = (float)(cropTop + cropHeight) /
				       (float)frameHeight;
			texCoords[4] =
				(float)cropLeft / (float)framePlaneStride[0];
			texCoords[5] = (float)cropTop / (float)frameHeight;
			texCoords[6] = (float)(cropLeft + cropWidth) /
				       (float)framePlaneStride[0];
			texCoords[7] = (float)cropTop / (float)frameHeight;
		} else {
			texCoords[0] =
				(float)cropLeft / (float)framePlaneStride[0];
			texCoords[1] = (float)cropTop / (float)frameHeight;
			texCoords[2] = (float)(cropLeft + cropWidth) /
				       (float)framePlaneStride[0];
			texCoords[3] = (float)cropTop / (float)frameHeight;
			texCoords[4] =
				(float)cropLeft / (float)framePlaneStride[0];
			texCoords[5] = (float)(cropTop + cropHeight) /
				       (float)frameHeight;
			texCoords[6] = (float)(cropLeft + cropWidth) /
				       (float)framePlaneStride[0];
			texCoords[7] = (float)(cropTop + cropHeight) /
				       (float)frameHeight;
		}

		GLCHK(glVertexAttribPointer(mTexcoordHandle[colorConversion],
					    2,
					    GL_FLOAT,
					    false,
					    0,
					    texCoords));
		GLCHK(glEnableVertexAttribArray(
			mTexcoordHandle[colorConversion]));
        int val = -1;
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &val);
		GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

        
        //1080, 1805 = vpport
        //_lfClientEngine.Render(0.0, 1010.8, 0.5 * videoW * 1080, 0.5 * videoH * 1805, GetViewData());
        //_lfClientEngine.Render(0.0, 397.1, 1080, 1407.9, GetViewData());
        //videoW is the shader ie -1 1 loation in the viewport
        //renderPos x/y is the size of the viewport we are rendering the video in
        unsigned int vidWidth = mVideoWidth;
        unsigned int vidHeight = mVideoHeight;
        double videoAspectRatio = (double)vidWidth / (double)vidHeight;
        double textureNormalizedWidth = videoW;
        double textureNormalizedheight = videoH;
        double textureAspectRation = textureNormalizedWidth / textureNormalizedheight;
        double vidH = (1.0 / videoAspectRatio) * renderPos->width;
        GLint m_viewport[4];
        boost::shared_ptr<SViewData> metaData(new SViewData);//create a new one just in case the other end hasn't set the current view data yet
        glGetIntegerv( GL_VIEWPORT, m_viewport );
        {
            boost::mutex::scoped_lock scoped_lock(sviewMutex);//lock so that we aren't in middle of creating the data on the other end
            metaData = _frameViewData;
        }

        double delta = 200.0;
        //setXY2(3.45, 5.67);
        float fx = 0.0f;
        float fy = 0.0f;
        bool triggered = PDCheckGetLatLon(&fx, &fy);
        double returnlat = 0.0;
        double returnlon = 0.0;
        if(triggered){
            std::cout<<"get lat lon for screenxy triggered!! with x"<<fx<<" y"<<fy<<std::endl;
            //dg house 43.009008947652944, -89.77785090285298
            //PDSetLatLonReady(43.009008947652944, -89.77785090285298);//dg s house
            //void GetLatLon(int screenX, int screenY, int screenWidth, int screenHeight, double* lat, double* lon, int vpwidth, int vpheight);
            
            //old method
            //_lfClientEngine.GetLatLon( fx,  fy, renderPos->width, renderPos->height, &returnlat, &returnlon, renderPos->width ,
             //                         vidH);

            std::cout<<"videoscalefactor = "<<params->video_scale_factor;
            double deltaY = (renderPos->height / 2.0 - (0.5 * vidH)) / 2.0 ;//off by a factor of 2?? why??
            std::cout<<"renderpos->height = "<<renderPos->height<<std::endl;
            std::cout<<"vidH = "<<vidH<<std::endl;
            std::cout<<"deltay = "<<deltaY<<std::endl;
            
            _lfClientEngine.InsertMarker(fx, fy - deltaY, renderPos->width, vidH, "city", "marker", 1.0, &returnlat, &returnlon);
            std::cout<<"returned lat/lon "<<returnlat<<" "<<returnlon<<std::endl;
            PDSetLatLonReady(returnlat, returnlon);
        }
        //getxy2(fx, fy);
        //std::cout<<"fx fy = "<<fx<<" "<<fy<<std::endl;
        if(_viewDataInitialized){

            //in here check to see if user clicked the screen and getxy, for now just
            //print a message
            _lfClientEngine.Render(renderPos->x, (renderPos->height / 2.0) - (0.5 * vidH), renderPos->width , vidH, metaData);
        }
        glBindBuffer(GL_ARRAY_BUFFER, val);
		GLCHK(glDisableVertexAttribArray(
			mPositionHandle[colorConversion]));
		GLCHK(glDisableVertexAttribArray(
			mTexcoordHandle[colorConversion]));
	}

	return 0;
}



void Gles2Video::setVideoMedia(VideoMedia *media)
{
	mMedia = media;
}


void Gles2Video::setExtTexture(GLuint texture)
{
	mExtTexture = texture;
}

} /* namespace Pdraw */

#endif /* USE_GLES2 */
