/**
 * Parrot Drones Audio and Video Vector library
 * OpenGL video rendering
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

#define ULOG_TAG pdraw_glvideo
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_gl_video.hpp"

#ifdef PDRAW_USE_GL

#	include "pdraw_session.hpp"

#	include <math.h>

#	include <futils/futils.h>

namespace Pdraw {


/* Blurred padding dark coef */
#	define PDRAW_BLURRED_PADDING_DARK_COEF (0.75f)

/* The temporal frequency of zebra pattern */
#	define PDRAW_ZEBRA_FREQUENCY_HZ (1.f)
/* The angle of zebra pattern relative to y axis */
#	define PDRAW_ZEBRA_ANGLE (60.f * M_PI / 180.f)
/* The weight in pixels of zebra pattern, relative to 1920 width */
#	define PDRAW_ZEBRA_WEIGHT (8.f)

#	define GL_VIDEO_BLUR_MIN_SIGMA 0.8f
#	define GL_VIDEO_BLUR_MAX_SIGMA 6.0f
#	define GL_VIDEO_BLURRED_PADDING_SIGMA 3.0f

#	define GL_VIDEO_HISTOGRAM_COMPUTE_INTERVAL_US 100000

#	define GL_VIDEO_FLASH_LIGHT_COEF 0.3f
#	define GL_VIDEO_FLASH_GAMMA_COEF 2.0f

#	ifdef GL_ES_VERSION_2_0
/* Default OpenGL ES Shading Language version (1.10.59) */
#		define GLSL_VERSION "#version 100\n"
#	else
/* Default OpenGL Shading Language version (1.00.17) */
#		define GLSL_VERSION "#version 110\n"
#	endif

static const GLchar *videoVertexShader =
	/* Explicitly needed on some platforms */
	GLSL_VERSION
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

static const GLchar *mbStatusFragmentShader =
	"const float MB_STATUS_UNKNOWN = 0.5 / 255.;\n"
	"const float MB_STATUS_VALID_ISLICE = 1.5 / 255.;\n"
	"const float MB_STATUS_VALID_PSLICE = 2.5 / 255.;\n"
	"const float MB_STATUS_MISSING_CONCEALED_PSLICE = 3.5 / 255.;\n"
	"const float MB_STATUS_MISSING = 4.5 / 255.;\n"
	"const float MB_STATUS_ERROR_PROPAGATION = 5.5 / 255.;\n"
	"const float MB_STATUS_MISSING_CONCEALED_ISLICE = 6.5 / 255.;\n"
	"const vec3 GREEN1 = vec3(0.2549, 0.6706, 0.3647);\n" /* #41ab5d */
	"const vec3 RED1 = vec3(0.7961, 0.1818, 0.1137);\n" /* #cb181d */
	"const vec3 RED2 = vec3(0.9373, 0.2314, 0.1725);\n" /* #ef3b2c */
	"const vec3 RED4 = vec3(0.9882, 0.5725, 0.4471);\n" /* #fc9272 */
	"const vec3 BLUE1 = vec3(0.4196, 0.6824, 0.8392);\n" /* #6baed6 */
	"const vec3 GREY1 = vec3(0.3333, 0.3333, 0.3333);\n"
	"uniform sampler2D s_texture_mb;\n"
	"\n"
	"vec3 apply_mb_status(vec2 coord, vec3 rgb)\n"
	"{\n"
	"    float mb_status = texture2D(s_texture_mb, coord).r;\n"
	"    if (mb_status <= MB_STATUS_UNKNOWN)\n"
	"        return mix(rgb, GREY1, 0.5);\n"
	"    else if (mb_status <= MB_STATUS_VALID_ISLICE)\n"
	"        return mix(rgb, GREEN1, 0.5);\n"
	"    else if (mb_status <= MB_STATUS_VALID_PSLICE)\n"
	"        return rgb;\n"
	"    else if (mb_status <= MB_STATUS_MISSING_CONCEALED_PSLICE)\n"
	"        return mix(rgb, RED4, 0.5);\n"
	"    else if (mb_status <= MB_STATUS_MISSING)\n"
	"        return mix(rgb, RED1, 0.5);\n"
	"    else if (mb_status <= MB_STATUS_ERROR_PROPAGATION)\n"
	"        return mix(rgb, BLUE1, 0.5);\n"
	"    else if (mb_status <= MB_STATUS_MISSING_CONCEALED_ISLICE)\n"
	"        return mix(rgb, RED2, 0.5);\n"
	"    return rgb;\n"
	"}\n";

static const GLchar *textureNoconvFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    return texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0)).rgb;\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    return texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0)).rgb;\n"
	"}\n";

static const GLchar *textureI420FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0)).r;\n"
	"    yuv.g = texture2D(s_texture_1, min(coord, max_coords[1] - stride[1] / 2.0)).r;\n"
	"    yuv.b = texture2D(s_texture_2, min(coord, max_coords[2] - stride[2] / 2.0)).r;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0)).r;\n"
	"    yuv.g = texture2D(s_texture_1, min(coord + offset_px * stride[1], max_coords[1] - stride[1] / 2.0)).r;\n"
	"    yuv.b = texture2D(s_texture_2, min(coord + offset_px * stride[2], max_coords[2] - stride[2] / 2.0)).r;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

/* YUV 4:2:0 planar with 16 bits data format in little endian
 * and 10 bits depth, padding in higher bits */
static const GLchar *textureI42010LELowFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec4 y = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0));\n"
	"    vec4 u = texture2D(s_texture_1, min(coord, max_coords[1] - stride[1] / 2.0));\n"
	"    vec4 v = texture2D(s_texture_2, min(coord, max_coords[2] - stride[2] / 2.0));\n"
	"    yuv.r = y.a * 64. + y.r / 4.;\n"
	"    yuv.g = u.a * 64. + u.r / 4.;\n"
	"    yuv.b = v.a * 64. + v.r / 4.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec4 y = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0));\n"
	"    vec4 u = texture2D(s_texture_1, min(coord + offset_px * stride[1], max_coords[1] - stride[1] / 2.0));\n"
	"    vec4 v = texture2D(s_texture_2, min(coord + offset_px * stride[2], max_coords[2] - stride[2] / 2.0));\n"
	"    yuv.r = y.a * 64. + y.r / 4.;\n"
	"    yuv.g = u.a * 64. + u.r / 4.;\n"
	"    yuv.b = v.a * 64. + v.r / 4.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *textureNV12FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0)).r;\n"
	"    yuv.gb = texture2D(s_texture_1, min(coord, max_coords[1] - stride[1] / 2.0)).ra;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    yuv.r = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0)).r;\n"
	"    yuv.gb = texture2D(s_texture_1, min(coord + offset_px * stride[1], max_coords[1] - stride[1] / 2.0)).ra;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

/* YUV 4:2:0 semi-planar with 16 bits data format in little endian
 * and 10 bits depth, padding in lower bits */
static const GLchar *textureNV1210LEHighFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec4 y = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0));\n"
	"    vec4 uv = texture2D(s_texture_1, min(coord, max_coords[1] - stride[1] / 2.0));\n"
	"    yuv.r = y.a + y.r / 256.;\n"
	"    yuv.g = uv.g + uv.b / 256.;\n"
	"    yuv.b = uv.a + uv.r / 256.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec4 y = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0));\n"
	"    vec4 uv = texture2D(s_texture_1, min(coord + offset_px * stride[1], max_coords[1] - stride[1] / 2.0));\n"
	"    yuv.r = y.a + y.r / 256.;\n"
	"    yuv.g = uv.g + uv.b / 256.;\n"
	"    yuv.b = uv.a + uv.r / 256.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *textureGrayFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    float gray = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0)).r;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    float gray = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0)).r;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *textureGray16FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0));\n"
	"    float gray = p.a + p.r / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0));\n"
	"    float gray = p.a + p.r / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *textureGray32FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(coord, max_coords[0] - stride[0] / 2.0));\n"
	"    float gray = p.a + p.r / 256. + p.g / 256. / 256. + p.b / 256. / 256. / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(coord + offset_px * stride[0], max_coords[0] - stride[0] / 2.0));\n"
	"    float gray = p.a + p.r / 256. + p.g / 256. / 256. + p.b / 256. / 256. / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *videoFragmentShader =
#	if defined(GL_ES_VERSION_2_0)
	"precision highp float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform float brightness_coef;\n"
	"uniform float contrast_coef;\n"
	"uniform float gamma_coef;\n"
	"uniform float sat_coef;\n"
	"uniform float light_coef;\n"
	"uniform float dark_coef;\n"
	"uniform float zebra_enable;\n"
	"uniform float zebra_avg_weights[9];\n"
	"uniform float mb_status_enable;\n"
	"\n"
	"vec3 apply_zebra(vec3 avg, vec3 rgb);\n"
	"vec3 apply_mb_status(vec2 coord, vec3 rgb);\n"
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
	"    if (zebra_enable > 0.5)\n"
	"    {\n"
	"        vec3 avg = read_rgb_avg(v_texcoord);\n"
	"        rgb = apply_zebra(avg, rgb);\n"
	"    }\n"
	"    float luma = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;\n"
	"    rgb = (rgb + vec3(brightness_coef)) * contrast_coef;\n"
	"    rgb = clamp(rgb, vec3(0.0), vec3(1.0));\n"
	"    rgb = pow(rgb, vec3(gamma_coef));\n"
	"    rgb = mix(vec3(luma), rgb, sat_coef);\n"
	"    rgb = mix(vec3(1.0), rgb, light_coef);\n"
	"    rgb = mix(vec3(0.0), rgb, dark_coef);\n"
	"    if (mb_status_enable > 0.5)\n"
	"    {\n"
	"        rgb = apply_mb_status(v_texcoord, rgb);\n"
	"    }\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

static const GLchar *simpleVideoFragmentShader =
#	if defined(GL_ES_VERSION_2_0)
	"precision mediump float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform float brightness_coef;\n"
	"uniform float contrast_coef;\n"
	"uniform float gamma_coef;\n"
	"uniform float sat_coef;\n"
	"uniform float light_coef;\n"
	"uniform float dark_coef;\n"
	"uniform float zebra_enable;\n"
	"uniform float zebra_avg_weights[9];\n"
	"\n"
	"vec3 read_rgb(vec2 coord);\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = read_rgb(v_texcoord);\n"
	"    float luma = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;\n"
	"    rgb = mix(vec3(luma), rgb, sat_coef);\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

const GLchar *GlVideo::videoFragmentShaders[2][PROGRAM_MAX][5] = {
	{
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureNoconvFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureI420FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureI42010LELowFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureNV12FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureNV1210LEHighFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureGrayFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureGray16FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			videoFragmentShader,
			textureGray32FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
	},
	{
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureNoconvFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureI420FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureI42010LELowFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureNV12FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureNV1210LEHighFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureGrayFragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureGray16FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
		{
			GLSL_VERSION,
			simpleVideoFragmentShader,
			textureGray32FragmentShader,
			zebraFragmentShader,
			mbStatusFragmentShader,
		},
	},
};

static const GLchar *simpleFragmentShader =
	/* Explicitly needed on some platforms */
	GLSL_VERSION
#	if defined(GL_ES_VERSION_2_0)
	"precision mediump float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = texture2D(s_texture, v_texcoord).rgb;\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";

static const GLchar *clearFragmentShader =
	/* Explicitly needed on some platforms */
	GLSL_VERSION
#	if defined(GL_ES_VERSION_2_0)
	"precision mediump float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform vec3 clear_color;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_FragColor = vec4(clear_color, 1.0);\n"
	"}\n";

static const GLchar *blurHVertexShader =
	/* Explicitly needed on some platforms */
	GLSL_VERSION
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
	/* Explicitly needed on some platforms */
	GLSL_VERSION
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
	/* Explicitly needed on some platforms */
	GLSL_VERSION
#	if defined(GL_ES_VERSION_2_0)
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
	/* Explicitly needed on some platforms */
	GLSL_VERSION
#	if defined(GL_ES_VERSION_2_0)
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
	/* Explicitly needed on some platforms */
	GLSL_VERSION
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
#	if defined(GL_ES_VERSION_2_0)
	"precision mediump float;\n"
#	endif
	"varying vec2 v_texcoord;\n"
	"uniform float brightness_coef;\n"
	"uniform float contrast_coef;\n"
	"uniform float gamma_coef;\n"
	"uniform vec3 rgb2luma_mat;\n"
	"uniform float rgb2luma_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord);\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 rgb = read_rgb(v_texcoord);\n"
	"    rgb = (rgb + brightness_coef) * contrast_coef;\n"
	"    rgb = clamp(rgb, vec3(0.0), vec3(1.0));\n"
	"    rgb = pow(rgb, vec3(gamma_coef));\n"
	"    float luma = rgb2luma_mat.r * rgb.r + rgb2luma_mat.g * rgb.g +"
	"        rgb2luma_mat.b * rgb.b + rgb2luma_offset;\n"
	"    gl_FragColor = vec4(rgb, luma);\n"
	"}\n";

const GLchar *GlVideo::histogramFragmentShaders[PROGRAM_MAX][3] = {
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureNoconvFragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureI420FragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureI42010LELowFragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureNV12FragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureNV1210LEHighFragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureGrayFragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureGray16FragmentShader,
	},
	{
		GLSL_VERSION,
		histogramFragmentShader,
		textureGray16FragmentShader,
	},
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


GlVideo::GlVideo(Session *session,
		 GLuint defaultFbo,
		 unsigned int firstTexUnit,
		 bool simplified)
{
	int ret;
	GLint vertexShader = 0, fragmentShader[PROGRAM_MAX] = {};
	GLint success = 0;
	unsigned int i;

	mSession = session;
	mVideoWidth = 0;
	mVideoHeight = 0;
	mFillMode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT;
	mFirstTexUnit = firstTexUnit;
	mDefaultFbo = defaultFbo;
	mTransition = GL_VIDEO_TRANSITION_NONE;
	mTransitionStartTime = 0;
	mTransitionDuration = 0;
	mTransitionHold = false;
	memset(mProgram, 0, sizeof(mProgram));
	memset(mProgramTransformMatrix, 0, sizeof(mProgramTransformMatrix));
	memset(mProgramYuv2RgbMatrix, 0, sizeof(mProgramYuv2RgbMatrix));
	memset(mProgramYuv2RgbOffset, 0, sizeof(mProgramYuv2RgbOffset));
	memset(mProgramStride, 0, sizeof(mProgramStride));
	memset(mProgramMaxCoords, 0, sizeof(mProgramMaxCoords));
	memset(mProgramBrightnessCoef, 0, sizeof(mProgramBrightnessCoef));
	memset(mProgramContrastCoef, 0, sizeof(mProgramContrastCoef));
	memset(mProgramGammaCoef, 0, sizeof(mProgramGammaCoef));
	memset(mProgramSatCoef, 0, sizeof(mProgramSatCoef));
	memset(mProgramLightCoef, 0, sizeof(mProgramLightCoef));
	memset(mProgramDarkCoef, 0, sizeof(mProgramDarkCoef));
	memset(mProgramZebraEnable, 0, sizeof(mProgramZebraEnable));
	memset(mProgramZebraThreshold, 0, sizeof(mProgramZebraThreshold));
	memset(mProgramZebraPhase, 0, sizeof(mProgramZebraPhase));
	memset(mProgramZebraWeight, 0, sizeof(mProgramZebraWeight));
	memset(mProgramMbStatusEnable, 0, sizeof(mProgramMbStatusEnable));
	mSimpleProgram = 0;
	mSimpleProgramTransformMatrix = 0;
	mSimpleProgramUniformSampler = 0;
	mSimpleProgramPositionHandle = 0;
	mSimpleProgramTexcoordHandle = 0;
	mClearProgram = 0;
	mClearProgramTransformMatrix = 0;
	mClearProgramPositionHandle = 0;
	mClearProgramTexcoordHandle = 0;
	mClearProgramColor = 0;
	memset(mTextures, 0, sizeof(mTextures));
	mMbStatusTexture = 0;
	mExtTexture = 0;
	memset(mUniformSamplers, 0, sizeof(mUniformSamplers));
	memset(mPositionHandle, 0, sizeof(mPositionHandle));
	memset(mTexcoordHandle, 0, sizeof(mTexcoordHandle));
	memset(mMbStatusUniformSampler, 0, sizeof(mMbStatusUniformSampler));
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
	memset(mHistogramRgb2LumaMatrix, 0, sizeof(mHistogramRgb2LumaMatrix));
	memset(mHistogramRgb2LumaOffset, 0, sizeof(mHistogramRgb2LumaOffset));
	memset(mHistogramBrightnessCoef, 0, sizeof(mHistogramBrightnessCoef));
	memset(mHistogramContrastCoef, 0, sizeof(mHistogramContrastCoef));
	memset(mHistogramGammaCoef, 0, sizeof(mHistogramGammaCoef));
	memset(mHistogramStride, 0, sizeof(mHistogramStride));
	memset(mHistogramMaxCoords, 0, sizeof(mHistogramMaxCoords));
	memset(mHistogramUniformSampler, 0, sizeof(mHistogramUniformSampler));
	memset(mHistogramPositionHandle, 0, sizeof(mHistogramPositionHandle));
	memset(mHistogramTexcoordHandle, 0, sizeof(mHistogramTexcoordHandle));
	mHistogramFbo = 0;
	mHistogramFboTexture = 0;
	mHistogramBuffer = nullptr;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		mHistogramValid[i] = false;
	memset(mHistogram, 0, sizeof(mHistogram));
	memset(mHistogramNorm, 0, sizeof(mHistogram));
	mBrightnessCoef = 0.0f;
	mContrastCoef = 1.0f;
	mGammaCoef = 1.0f;
	mSatCoef = 1.0f;
	mBaseSatCoef = 1.0f;
	mLightCoef = 1.0f;
	mBaseLightCoef = 1.0f;
	mDarkCoef = 1.0f;
	mBaseDarkCoef = 1.0f;
	mHasMbStatus = false;

	GLCHK();

	/* Vertex shader */
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShader == 0) || (vertexShader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		goto err;
	}

	glShaderSource(vertexShader, 1, &videoVertexShader, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
		ULOGE("vertex shader compilation failed '%s'", infoLog);
		goto err;
	}

	for (i = 0; i < PROGRAM_MAX; i++) {
		/* Fragment shader */
		fragmentShader[i] = glCreateShader(GL_FRAGMENT_SHADER);
		if ((fragmentShader[i] == 0) ||
		    (fragmentShader[i] == GL_INVALID_ENUM)) {
			ULOGE("failed to create fragment shader");
			goto err;
		}

		glShaderSource(fragmentShader[i],
			       5,
			       videoFragmentShaders[simplified ? 1 : 0][i],
			       nullptr);
		glCompileShader(fragmentShader[i]);
		glGetShaderiv(fragmentShader[i], GL_COMPILE_STATUS, &success);
		if (!success) {
			GLchar infoLog[512];
			glGetShaderInfoLog(
				fragmentShader[i], 512, nullptr, infoLog);
			ULOGE("fragment shader compilation failed '%s'",
			      infoLog);
			goto err;
		}

		/* Link shaders */
		mProgram[i] = glCreateProgram();
		glAttachShader(mProgram[i], vertexShader);
		glAttachShader(mProgram[i], fragmentShader[i]);
		glLinkProgram(mProgram[i]);
		glGetProgramiv(mProgram[i], GL_LINK_STATUS, &success);
		if (!success) {
			GLchar infoLog[512] = {};
			glGetProgramInfoLog(mProgram[i], 512, nullptr, infoLog);
			ULOGE("program link failed '%s'", infoLog);
			goto err;
		}

		glDeleteShader(fragmentShader[i]);
		fragmentShader[i] = 0;
	}

	/* Simple fragment shader */
	fragmentShader[0] = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShader[0] == 0) ||
	    (fragmentShader[0] == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShader[0], 1, &simpleFragmentShader, nullptr);
	glCompileShader(fragmentShader[0]);
	glGetShaderiv(fragmentShader[0], GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShader[0], 512, nullptr, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mSimpleProgram = glCreateProgram();
	glAttachShader(mSimpleProgram, vertexShader);
	glAttachShader(mSimpleProgram, fragmentShader[0]);
	glLinkProgram(mSimpleProgram);
	glGetProgramiv(mSimpleProgram, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512] = {};
		glGetProgramInfoLog(mSimpleProgram, 512, nullptr, infoLog);
		ULOGE("program link failed '%s'", infoLog);
		goto err;
	}

	glDeleteShader(fragmentShader[0]);
	fragmentShader[0] = 0;

	/* Clear fragment shader */
	fragmentShader[0] = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragmentShader[0] == 0) ||
	    (fragmentShader[0] == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto err;
	}

	glShaderSource(fragmentShader[0], 1, &clearFragmentShader, nullptr);
	glCompileShader(fragmentShader[0]);
	glGetShaderiv(fragmentShader[0], GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShader[0], 512, nullptr, infoLog);
		ULOGE("fragment shader compilation failed '%s'", infoLog);
		goto err;
	}

	/* Link shaders */
	mClearProgram = glCreateProgram();
	glAttachShader(mClearProgram, vertexShader);
	glAttachShader(mClearProgram, fragmentShader[0]);
	glLinkProgram(mClearProgram);
	glGetProgramiv(mClearProgram, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[512] = {};
		glGetProgramInfoLog(mClearProgram, 512, nullptr, infoLog);
		ULOGE("program link failed '%s'", infoLog);
		goto err;
	}

	glDeleteShader(fragmentShader[0]);
	fragmentShader[0] = 0;

	glDeleteShader(vertexShader);
	vertexShader = 0;

	GLCHK();

	for (i = 0; i < PROGRAM_MAX; i++) {
		mProgramTransformMatrix[i] =
			glGetUniformLocation(mProgram[i], "transform_matrix");
		mProgramYuv2RgbMatrix[i] =
			glGetUniformLocation(mProgram[i], "yuv2rgb_mat");
		mProgramYuv2RgbOffset[i] =
			glGetUniformLocation(mProgram[i], "yuv2rgb_offset");
		mProgramStride[i] = glGetUniformLocation(mProgram[i], "stride");
		mProgramMaxCoords[i] =
			glGetUniformLocation(mProgram[i], "max_coords");
		mProgramBrightnessCoef[i] =
			glGetUniformLocation(mProgram[i], "brightness_coef");
		mProgramContrastCoef[i] =
			glGetUniformLocation(mProgram[i], "contrast_coef");
		mProgramGammaCoef[i] =
			glGetUniformLocation(mProgram[i], "gamma_coef");
		mProgramSatCoef[i] =
			glGetUniformLocation(mProgram[i], "sat_coef");
		mProgramLightCoef[i] =
			glGetUniformLocation(mProgram[i], "light_coef");
		mProgramDarkCoef[i] =
			glGetUniformLocation(mProgram[i], "dark_coef");
		mProgramZebraEnable[i] =
			glGetUniformLocation(mProgram[i], "zebra_enable");
		mProgramZebraThreshold[i] =
			glGetUniformLocation(mProgram[i], "zebra_sat");
		mProgramZebraPhase[i] =
			glGetUniformLocation(mProgram[i], "zebra_phase");
		mProgramZebraWeight[i] =
			glGetUniformLocation(mProgram[i], "zebra_weight");
		mProgramMbStatusEnable[i] =
			glGetUniformLocation(mProgram[i], "mb_status_enable");
		mUniformSamplers[i][0] =
			glGetUniformLocation(mProgram[i], "s_texture_0");
		mUniformSamplers[i][1] =
			glGetUniformLocation(mProgram[i], "s_texture_1");
		mUniformSamplers[i][2] =
			glGetUniformLocation(mProgram[i], "s_texture_2");
		mPositionHandle[i] =
			glGetAttribLocation(mProgram[i], "position");
		mTexcoordHandle[i] =
			glGetAttribLocation(mProgram[i], "texcoord");
		mMbStatusUniformSampler[i] =
			glGetUniformLocation(mProgram[i], "s_texture_mb");
	}

	mSimpleProgramTransformMatrix =
		glGetUniformLocation(mSimpleProgram, "transform_matrix");
	mSimpleProgramUniformSampler =
		glGetUniformLocation(mSimpleProgram, "s_texture");
	mSimpleProgramPositionHandle =
		glGetAttribLocation(mSimpleProgram, "position");
	mSimpleProgramTexcoordHandle =
		glGetAttribLocation(mSimpleProgram, "texcoord");

	mClearProgramTransformMatrix =
		glGetUniformLocation(mClearProgram, "transform_matrix");
	mClearProgramPositionHandle =
		glGetAttribLocation(mClearProgram, "position");
	mClearProgramTexcoordHandle =
		glGetAttribLocation(mClearProgram, "texcoord");
	mClearProgramColor = glGetUniformLocation(mClearProgram, "clear_color");

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
	for (i = 0; i < PROGRAM_MAX; i++)
		setupZebra((Pdraw::GlVideo::program)i);

	GLCHK(glGenTextures(GL_VIDEO_TEX_UNIT_COUNT, mTextures));

	for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));

		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCHK(glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCHK(glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	}

	GLCHK(glGenTextures(1, &mMbStatusTexture));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT +
			      GL_VIDEO_FBO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mMbStatusTexture));
	GLCHK(glTexParameteri(
		GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
	GLCHK(glTexParameteri(
		GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCHK(glBindTexture(GL_TEXTURE_2D, 0));

	return;

err:
	if (mMbStatusTexture)
		GLCHK(glDeleteTextures(1, &mMbStatusTexture));
	if (mTextures[0])
		GLCHK(glDeleteTextures(GL_VIDEO_TEX_UNIT_COUNT, mTextures));
	if (vertexShader)
		GLCHK(glDeleteShader(vertexShader));
	for (i = 0; i < PROGRAM_MAX; i++) {
		if (fragmentShader[i])
			GLCHK(glDeleteShader(fragmentShader[i]));
		if (mProgram[i] > 0)
			GLCHK(glDeleteProgram(mProgram[i]));
	}
	if (mSimpleProgram > 0) {
		GLCHK(glDeleteProgram(mSimpleProgram));
		mSimpleProgram = 0;
	}
	if (mClearProgram > 0) {
		GLCHK(glDeleteProgram(mClearProgram));
		mClearProgram = 0;
	}
	memset(mProgram, 0, sizeof(mProgram));
	memset(mTextures, 0, sizeof(mTextures));
	mMbStatusTexture = 0;
	cleanupBlur();
	cleanupHistograms();
}


GlVideo::~GlVideo(void)
{
	if (mMbStatusTexture)
		GLCHK(glDeleteTextures(1, &mMbStatusTexture));
	if (mTextures[0])
		GLCHK(glDeleteTextures(GL_VIDEO_TEX_UNIT_COUNT, mTextures));
	for (unsigned int i = 0; i < PROGRAM_MAX; i++) {
		if (mProgram[i] > 0)
			GLCHK(glDeleteProgram(mProgram[i]));
	}
	if (mSimpleProgram > 0)
		GLCHK(glDeleteProgram(mSimpleProgram));
	if (mClearProgram > 0)
		GLCHK(glDeleteProgram(mClearProgram));

	cleanupBlur();
	cleanupPaddingFbo();
	cleanupHistograms();
}


enum GlVideo::program GlVideo::getProgram(const struct vdef_raw_format *format,
					  bool *swapUv) const
{
	*swapUv = false;
	if (vdef_raw_format_cmp(format, &vdef_i420)) {
		return PROGRAM_YUV_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_nv12)) {
		return PROGRAM_YUV_TO_RGB_SEMIPLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_nv21)) {
		*swapUv = true;
		return PROGRAM_YUV_TO_RGB_SEMIPLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_i420_10_16le)) {
		return PROGRAM_YUV_TO_RGB_PLANAR_10_16LE;
	} else if (vdef_raw_format_cmp(format, &vdef_nv12_10_16le_high)) {
		return PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH;
	} else if (vdef_raw_format_cmp(format, &vdef_gray)) {
		return PROGRAM_GRAY_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_raw16)) {
		return PROGRAM_GRAY16_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_raw32)) {
		return PROGRAM_GRAY32_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_rgb)) {
		return PROGRAM_NOCONV;
	} else if (vdef_raw_format_cmp(format, &vdef_opaque)) {
		return PROGRAM_NOCONV;
	} else {
		ULOGE("unsupported frame format");
		return PROGRAM_NOCONV;
	}
}


void GlVideo::fillYuv2RgbMatrix(enum vdef_matrix_coefs matrixCoefs,
				bool fullRange,
				bool swapUv,
				float yuv2RgbMatrix[9],
				float yuv2RgbOffset[3])
{
	int fr = fullRange ? 1 : 0;

	memcpy(yuv2RgbMatrix,
	       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr],
	       3 * sizeof(float));
	if (swapUv) {
		memcpy(yuv2RgbMatrix + 3,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 6,
		       3 * sizeof(float));
		memcpy(yuv2RgbMatrix + 6,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 3,
		       3 * sizeof(float));
		yuv2RgbOffset[0] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][0];
		yuv2RgbOffset[1] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][2];
		yuv2RgbOffset[2] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][1];
	} else {
		memcpy(yuv2RgbMatrix + 3,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 3,
		       3 * sizeof(float));
		memcpy(yuv2RgbMatrix + 6,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 6,
		       3 * sizeof(float));
		yuv2RgbOffset[0] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][0];
		yuv2RgbOffset[1] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][1];
		yuv2RgbOffset[2] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][2];
	}
}


unsigned int GlVideo::getTextureMaxUnpackAlignment(unsigned int width)
{
	if (width % 2)
		return 1;
	else if (width % 4)
		return 2;
	else if (width % 8)
		return 4;
	else
		return 8;
}


int GlVideo::setupBlur(void)
{
	int ret = 0;
	GLint vertexShaderH = 0, vertexShaderV = 0;
	GLint fragmentShaderH = 0, fragmentShaderV = 0;
	GLint success = 0;

	/* Free previous resources */
	cleanupBlur();

	/* Render sizes */
	mBlurFboWidth = GL_VIDEO_BLUR_FBO_TARGET_SIZE;
	mBlurFboHeight = GL_VIDEO_BLUR_FBO_TARGET_SIZE;

	/* Shaders compilation */
	vertexShaderH = glCreateShader(GL_VERTEX_SHADER);
	if ((vertexShaderH == 0) || (vertexShaderH == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		ret = -ENOMEM;
		goto error;
	}

	glShaderSource(vertexShaderH, 1, &blurHVertexShader, nullptr);
	glCompileShader(vertexShaderH);
	glGetShaderiv(vertexShaderH, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShaderH, 512, nullptr, infoLog);
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

	glShaderSource(vertexShaderV, 1, &blurVVertexShader, nullptr);
	glCompileShader(vertexShaderV);
	glGetShaderiv(vertexShaderV, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(vertexShaderV, 512, nullptr, infoLog);
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

	glShaderSource(fragmentShaderH, 1, &blurHFragmentShader, nullptr);
	glCompileShader(fragmentShaderH);
	glGetShaderiv(fragmentShaderH, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderH, 512, nullptr, infoLog);
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

	glShaderSource(fragmentShaderV, 1, &blurVFragmentShader, nullptr);
	glCompileShader(fragmentShaderV);
	glGetShaderiv(fragmentShaderV, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(fragmentShaderV, 512, nullptr, infoLog);
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
		GLchar infoLog[512] = {};
		glGetProgramInfoLog(mBlurProgram[0], 512, nullptr, infoLog);
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
		GLchar infoLog[512] = {};
		glGetProgramInfoLog(mBlurProgram[1], 512, nullptr, infoLog);
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


void GlVideo::cleanupBlur(void)
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


int GlVideo::setupBlurFbo(void)
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
		mBlurFboWidth = GL_VIDEO_BLUR_FBO_TARGET_SIZE;
		mBlurFboHeight = (GL_VIDEO_BLUR_FBO_TARGET_SIZE * mVideoHeight /
					  mVideoWidth +
				  3) &
				 ~3;
	} else {
		mBlurFboWidth = (GL_VIDEO_BLUR_FBO_TARGET_SIZE * mVideoWidth /
					 mVideoHeight +
				 3) &
				~3;
		mBlurFboHeight = GL_VIDEO_BLUR_FBO_TARGET_SIZE;
	}

	/* Allocate FBOs and textures */
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
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
				   nullptr));

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


void GlVideo::cleanupBlurFbo(void)
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


void GlVideo::renderBlur(
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_rect *renderPos,
	float videoW,
	float videoH,
	const Eigen::Matrix4f &viewProjMat)
{
	unsigned int i;
	float stride[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float maxCoords[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	float yuv2RgbMatrix[9];
	float yuv2RgbOffset[3];
	bool mirrorTexture = false, swapUv = false;

	if (!mBlurInit)
		return;

	enum program prog;
	prog = getProgram(format, &swapUv);

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[0]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));

	GLCHK(glUseProgram(mProgram[prog]));

	switch (prog) {
	default:
	case PROGRAM_GRAY_TO_RGB_PLANAR:
	case PROGRAM_GRAY16_TO_RGB_PLANAR:
	case PROGRAM_GRAY32_TO_RGB_PLANAR:
		mirrorTexture = true;
		/* Fall through */
	case PROGRAM_NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[prog][0], mFirstTexUnit));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	case PROGRAM_YUV_TO_RGB_PLANAR:
	case PROGRAM_YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = true;
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[prog][i],
					  mFirstTexUnit + i));
			stride[2 * i] = 1.f / framePlaneStride[i];
			stride[2 * i + 1] = 1.f / height;
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				(framePlaneStride[i] * ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				info->resolution.height;
		}
		break;
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR:
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[prog][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[prog][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / (framePlaneStride[1] / 2);
		stride[3] = 1.f / (info->resolution.height / 2);
		maxCoords[2] =
			(float)(crop->left + crop->width) / framePlaneStride[1];
		maxCoords[3] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	}

	GLCHK(glUniform2fv(
		mProgramStride[prog], GL_VIDEO_TEX_UNIT_COUNT, stride));
	GLCHK(glUniform2fv(
		mProgramMaxCoords[prog], GL_VIDEO_TEX_UNIT_COUNT, maxCoords));
	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mProgramYuv2RgbOffset[prog],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(
		mProgramYuv2RgbMatrix[prog], 1, GL_FALSE, yuv2RgbMatrix));

	/* Disable overexposure zebras */
	updateZebra(nullptr, prog, false, 0.f);

	/* Disable MB status display */
	GLCHK(glUniform1f(mProgramMbStatusEnable[prog], 0.f));

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
		mProgramTransformMatrix[prog], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramBrightnessCoef[prog], mBrightnessCoef));
	GLCHK(glUniform1f(mProgramContrastCoef[prog], mContrastCoef));
	GLCHK(glUniform1f(mProgramGammaCoef[prog], mGammaCoef));
	GLCHK(glUniform1f(mProgramSatCoef[prog], mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[prog], mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[prog], mDarkCoef));

	GLCHK(glVertexAttribPointer(
		mPositionHandle[prog], 3, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[prog]));

	if (mirrorTexture) {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)crop->top / (float)info->resolution.height;
	} else {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
	}

	GLCHK(glVertexAttribPointer(
		mTexcoordHandle[prog], 2, GL_FLOAT, false, 0, texCoords));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[prog]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[prog]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[prog]));

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
	GLCHK(glUniform1fv(
		mBlurUniformWeights[0], GL_VIDEO_BLUR_TAP_COUNT, mBlurWeights));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
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
	GLCHK(glUniform1fv(
		mBlurUniformWeights[1], GL_VIDEO_BLUR_TAP_COUNT, mBlurWeights));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mBlurFboHeight));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Render to screen */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(renderPos->x,
			 renderPos->y,
			 renderPos->width,
			 renderPos->height));
	GLCHK(glUseProgram(mSimpleProgram));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[0]));
	GLCHK(glUniform1i(mSimpleProgramUniformSampler,
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glUniformMatrix4fv(
		mSimpleProgramTransformMatrix, 1, false, viewProjMat.data()));

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
		mSimpleProgramPositionHandle, 3, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramPositionHandle));

	GLCHK(glVertexAttribPointer(mSimpleProgramTexcoordHandle,
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mSimpleProgramPositionHandle));
	GLCHK(glDisableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glUseProgram(mProgram[prog]));
}


int GlVideo::setupPaddingFbo(void)
{
	GLenum gle;
	unsigned int i;

	/* Free previous resources */
	cleanupPaddingFbo();

	if (!mBlurInit)
		return 0;

	if ((mFillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP) &&
	    (mFillMode != PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND))
		return 0;

	/* Render sizes */
	if (mVideoWidth > mVideoHeight) {
		mPaddingPass1Width = GL_VIDEO_PADDING_FBO_TARGET_SIZE_1;
		mPaddingPass1Height = (GL_VIDEO_PADDING_FBO_TARGET_SIZE_1 *
					       mVideoHeight / mVideoWidth +
				       3) &
				      ~3;
		mPaddingPass2Width = GL_VIDEO_PADDING_FBO_TARGET_SIZE_2;
		mPaddingPass2Height = (GL_VIDEO_PADDING_FBO_TARGET_SIZE_2 *
					       mVideoHeight / mVideoWidth +
				       3) &
				      ~3;
	} else {
		mPaddingPass1Width = (GL_VIDEO_PADDING_FBO_TARGET_SIZE_1 *
					      mVideoWidth / mVideoHeight +
				      3) &
				     ~3;
		mPaddingPass1Height = GL_VIDEO_PADDING_FBO_TARGET_SIZE_1;
		mPaddingPass2Width = (GL_VIDEO_PADDING_FBO_TARGET_SIZE_2 *
					      mVideoWidth / mVideoHeight +
				      3) &
				     ~3;
		mPaddingPass2Height = GL_VIDEO_PADDING_FBO_TARGET_SIZE_2;
	}

	pdraw_gaussianDistribution(mPaddingBlurWeights,
				   GL_VIDEO_BLUR_TAP_COUNT,
				   GL_VIDEO_BLURRED_PADDING_SIGMA);

	/* Allocate new resources */
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
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
			nullptr));

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


void GlVideo::cleanupPaddingFbo(void)
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


void GlVideo::renderPadding(
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_rect *renderPos,
	float videoW,
	float videoH,
	float videoW2,
	float videoH2,
	float videoAR,
	float windowAR,
	bool immersive,
	const Eigen::Matrix4f &viewProjMat)
{
	unsigned int i;
	float stride[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float maxCoords[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	float yuv2RgbMatrix[9];
	float yuv2RgbOffset[3];
	bool mirrorTexture = false, swapUv = false;

	if (!mBlurInit)
		return;

	if (mFillMode == PDRAW_VIDEO_RENDERER_FILL_MODE_FIT) {
		GLCHK(glViewport(renderPos->x,
				 renderPos->y,
				 renderPos->width,
				 renderPos->height));
		clear(viewProjMat);
		return;
	} else if ((mFillMode !=
		    PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP) &&
		   (mFillMode !=
		    PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND)) {
		return;
	}

	enum program prog;
	prog = getProgram(format, &swapUv);

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[0]));
	GLCHK(glViewport(0, 0, mPaddingPass1Width, mPaddingPass1Height));

	GLCHK(glUseProgram(mProgram[prog]));

	switch (prog) {
	default:
	case PROGRAM_GRAY_TO_RGB_PLANAR:
	case PROGRAM_GRAY16_TO_RGB_PLANAR:
	case PROGRAM_GRAY32_TO_RGB_PLANAR:
		mirrorTexture = true;
		/* Fall through */
	case PROGRAM_NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[prog][0], mFirstTexUnit));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	case PROGRAM_YUV_TO_RGB_PLANAR:
	case PROGRAM_YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = true;
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[prog][i],
					  mFirstTexUnit + i));
			stride[2 * i] = 1.f / framePlaneStride[i];
			stride[2 * i + 1] = 1.f / height;
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				(framePlaneStride[i] * ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				info->resolution.height;
		}
		break;
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR:
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[prog][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[prog][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / (framePlaneStride[1] / 2);
		stride[3] = 1.f / (info->resolution.height / 2);
		maxCoords[2] =
			(float)(crop->left + crop->width) / framePlaneStride[1];
		maxCoords[3] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	}

	GLCHK(glUniform2fv(
		mProgramStride[prog], GL_VIDEO_TEX_UNIT_COUNT, stride));
	GLCHK(glUniform2fv(
		mProgramMaxCoords[prog], GL_VIDEO_TEX_UNIT_COUNT, maxCoords));
	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mProgramYuv2RgbOffset[prog],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(
		mProgramYuv2RgbMatrix[prog], 1, GL_FALSE, yuv2RgbMatrix));

	/* Disable overexposure zebras */
	updateZebra(nullptr, prog, false, 0.f);

	/* Disable MB status display */
	GLCHK(glUniform1f(mProgramMbStatusEnable[prog], 0.f));

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
		mProgramTransformMatrix[prog], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramBrightnessCoef[prog], mBrightnessCoef));
	GLCHK(glUniform1f(mProgramContrastCoef[prog], mContrastCoef));
	GLCHK(glUniform1f(mProgramGammaCoef[prog], mGammaCoef));
	GLCHK(glUniform1f(mProgramSatCoef[prog], mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[prog], mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[prog],
			  (immersive) ? mDarkCoef
				      : PDRAW_BLURRED_PADDING_DARK_COEF *
						mDarkCoef));

	GLCHK(glVertexAttribPointer(
		mPositionHandle[prog], 3, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[prog]));

	if (mirrorTexture) {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)crop->top / (float)info->resolution.height;
	} else {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
	}

	GLCHK(glVertexAttribPointer(
		mTexcoordHandle[prog], 2, GL_FLOAT, false, 0, texCoords));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[prog]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[prog]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[prog]));

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
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[0], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0], 1.0 / mPaddingPass1Width));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 1 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mPaddingPass1Height));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Pass 2 horizontal blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[3]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[2]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[0], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0], 1.0 / mPaddingPass2Width));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 2 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[3]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(
		mBlurPositionHandle[1], 2, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1], 1.0 / mPaddingPass2Height));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[1]));

	/* Render to screen */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(renderPos->x,
			 renderPos->y,
			 renderPos->width,
			 renderPos->height));
	GLCHK(glUseProgram(mSimpleProgram));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[2]));
	GLCHK(glUniform1i(mSimpleProgramUniformSampler,
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glUniformMatrix4fv(
		mSimpleProgramTransformMatrix, 1, false, viewProjMat.data()));

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

	if (mFillMode == PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND) {
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
		mSimpleProgramPositionHandle, 3, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramPositionHandle));

	GLCHK(glVertexAttribPointer(mSimpleProgramTexcoordHandle,
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mSimpleProgramPositionHandle));
	GLCHK(glDisableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glUseProgram(mProgram[prog]));
}


void GlVideo::setupZebra(enum program prog)
{
	float co = cosf(PDRAW_ZEBRA_ANGLE);
	float si = sinf(PDRAW_ZEBRA_ANGLE);
	const GLfloat zebra_mat[] = {co, si, -si, co};

	GLCHK(glUseProgram(mProgram[prog]));
	GLCHK(glUniformMatrix2fv(
		glGetUniformLocation(mProgram[prog], "zebra_mat"),
		1,
		GL_FALSE,
		zebra_mat));
	GLCHK(glUniform1fv(
		glGetUniformLocation(mProgram[prog], "zebra_avg_weights"),
		9,
		zebraAvgWeights));
}


void GlVideo::updateZebra(struct pdraw_rect *contentPos,
			  enum program prog,
			  bool enable,
			  float threshold)
{
	GLCHK(glUniform1f(mProgramZebraEnable[prog], enable ? 1.f : 0.f));
	GLCHK(glUniform1f(mProgramZebraThreshold[prog], threshold));

	if (enable && contentPos != nullptr) {
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
		GLCHK(glUniform1f(mProgramZebraPhase[prog], zebra_phase));
		float zebra_weight =
			PDRAW_ZEBRA_WEIGHT * contentPos->width / 1920;
		GLCHK(glUniform1f(mProgramZebraWeight[prog], zebra_weight));
	}
}


int GlVideo::setupHistograms(void)
{
	int ret = 0;
	GLenum gle;
	GLint success = 0;
	GLint vertexShaderHistogram = 0;
	GLint fragmentShaderHistogram[PROGRAM_MAX] = {0};
	unsigned int i;

	/* Buffers allocation */
	mHistogramBuffer =
		(uint8_t *)malloc(4 * GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
				  GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE);
	if (mHistogramBuffer == nullptr) {
		ULOG_ERRNO("malloc", ENOMEM);
		ret = -ENOMEM;
		goto error;
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		mHistogram[i] = (uint32_t *)malloc(256 * sizeof(uint32_t));
		if (mHistogram[i] == nullptr) {
			ULOG_ERRNO("malloc", ENOMEM);
			ret = -ENOMEM;
			goto error;
		}
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		mHistogramNorm[i] = (float *)calloc(256, sizeof(float));
		if (mHistogramNorm[i] == nullptr) {
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

	glShaderSource(
		vertexShaderHistogram, 1, &histogramVertexShader, nullptr);
	glCompileShader(vertexShaderHistogram);
	glGetShaderiv(vertexShaderHistogram, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[512];
		glGetShaderInfoLog(
			vertexShaderHistogram, 512, nullptr, infoLog);
		ULOGE("vertex shader compilation failed '%s'", infoLog);
		ret = -EPROTO;
		goto error;
	}

	for (i = 0; i < PROGRAM_MAX; i++) {
		fragmentShaderHistogram[i] = glCreateShader(GL_FRAGMENT_SHADER);
		if ((fragmentShaderHistogram[i] == 0) ||
		    (fragmentShaderHistogram[i] == GL_INVALID_ENUM)) {
			ULOGE("failed to create fragment shader");
			ret = -ENOMEM;
			goto error;
		}

		glShaderSource(fragmentShaderHistogram[i],
			       3,
			       histogramFragmentShaders[i],
			       nullptr);
		glCompileShader(fragmentShaderHistogram[i]);
		glGetShaderiv(fragmentShaderHistogram[i],
			      GL_COMPILE_STATUS,
			      &success);
		if (!success) {
			GLchar infoLog[512];
			glGetShaderInfoLog(fragmentShaderHistogram[i],
					   512,
					   nullptr,
					   infoLog);
			ULOGE("fragment shader compilation failed '%s'",
			      infoLog);
			ret = -EPROTO;
			goto error;
		}
	}

	/* Shaders link */
	for (i = 0; i < PROGRAM_MAX; i++) {
		mHistogramProgram[i] = glCreateProgram();
		glAttachShader(mHistogramProgram[i], vertexShaderHistogram);
		glAttachShader(mHistogramProgram[i],
			       fragmentShaderHistogram[i]);
		glLinkProgram(mHistogramProgram[i]);
		glGetProgramiv(mHistogramProgram[i], GL_LINK_STATUS, &success);
		if (!success) {
			GLchar infoLog[512] = {};
			glGetProgramInfoLog(
				mHistogramProgram[i], 512, nullptr, infoLog);
			ULOGE("program link failed '%s'", infoLog);
			ret = -EPROTO;
			goto error;
		}
	}

	glDeleteShader(vertexShaderHistogram);
	vertexShaderHistogram = 0;
	for (i = 0; i < PROGRAM_MAX; i++) {
		glDeleteShader(fragmentShaderHistogram[i]);
		fragmentShaderHistogram[i] = 0;
	}

	/* Uniforms and attribs */
	for (i = 0; i < PROGRAM_MAX; i++) {
		mHistogramYuv2RgbMatrix[i] = glGetUniformLocation(
			mHistogramProgram[i], "yuv2rgb_mat");
		mHistogramYuv2RgbOffset[i] = glGetUniformLocation(
			mHistogramProgram[i], "yuv2rgb_offset");
		mHistogramRgb2LumaMatrix[i] = glGetUniformLocation(
			mHistogramProgram[i], "rgb2luma_mat");
		mHistogramRgb2LumaOffset[i] = glGetUniformLocation(
			mHistogramProgram[i], "rgb2luma_offset");
		mHistogramBrightnessCoef[i] = glGetUniformLocation(
			mHistogramProgram[i], "brightness_coef");
		mHistogramContrastCoef[i] = glGetUniformLocation(
			mHistogramProgram[i], "contrast_coef");
		mHistogramGammaCoef[i] = glGetUniformLocation(
			mHistogramProgram[i], "gamma_coef");
		mHistogramStride[i] =
			glGetUniformLocation(mHistogramProgram[i], "stride");
		mHistogramMaxCoords[i] = glGetUniformLocation(
			mHistogramProgram[i], "max_coords");
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
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   0,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   nullptr));

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
	for (i = 0; i < PROGRAM_MAX; i++) {
		if (fragmentShaderHistogram[i])
			glDeleteShader(fragmentShaderHistogram[i]);
	}

	cleanupHistograms();

	return ret;
}


void GlVideo::cleanupHistograms(void)
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
	for (i = 0; i < PROGRAM_MAX; i++) {
		if (mHistogramProgram[i] > 0) {
			GLCHK(glDeleteProgram(mHistogramProgram[i]));
			mHistogramProgram[i] = 0;
		}
	}
	free(mHistogramBuffer);
	mHistogramBuffer = nullptr;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		free(mHistogram[i]);
		mHistogram[i] = nullptr;
	}
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		free(mHistogramNorm[i]);
		mHistogramNorm[i] = nullptr;
	}
	mHistogramInit = false;
}


void GlVideo::computeHistograms(
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_rect *renderPos,
	bool enable)
{
	float stride[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float maxCoords[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	float yuv2RgbMatrix[9];
	float yuv2RgbOffset[3];
	unsigned int i, j;
	uint8_t *buf;
	uint32_t histoMax[PDRAW_HISTOGRAM_CHANNEL_MAX];
	struct timespec ts;
	uint64_t time_us;
	bool mirrorTexture = false, swapUv = false;

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
			       GL_VIDEO_HISTOGRAM_COMPUTE_INTERVAL_US))
		return;
	mHistogramLastComputeTime = time_us;
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		mHistogramValid[i] = false;

	enum program prog;
	prog = getProgram(format, &swapUv);

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHistogramFbo));
	GLCHK(glDisable(GL_BLEND));
	GLCHK(glViewport(0,
			 0,
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE));
	GLCHK(glUseProgram(mHistogramProgram[prog]));

	switch (prog) {
	default:
	case PROGRAM_GRAY_TO_RGB_PLANAR:
	case PROGRAM_GRAY16_TO_RGB_PLANAR:
	case PROGRAM_GRAY32_TO_RGB_PLANAR:
		mirrorTexture = true;
		/* Fall through */
	case PROGRAM_NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mHistogramUniformSampler[prog][0],
				  mFirstTexUnit));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	case PROGRAM_YUV_TO_RGB_PLANAR:
	case PROGRAM_YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = true;
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mHistogramUniformSampler[prog][i],
					  mFirstTexUnit + i));
			stride[2 * i] = 1.f / framePlaneStride[i];
			stride[2 * i + 1] = 1.f / height;
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				(framePlaneStride[i] * ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				info->resolution.height;
		}
		break;
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR:
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = true;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mHistogramUniformSampler[prog][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / framePlaneStride[0];
		stride[1] = 1.f / info->resolution.height;
		maxCoords[0] =
			(float)(crop->left + crop->width) / framePlaneStride[0];
		maxCoords[1] = (float)(crop->top + crop->height) /
			       info->resolution.height;

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mHistogramUniformSampler[prog][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / (framePlaneStride[1] / 2);
		stride[3] = 1.f / (info->resolution.height / 2);
		maxCoords[2] =
			(float)(crop->left + crop->width) / framePlaneStride[1];
		maxCoords[3] = (float)(crop->top + crop->height) /
			       info->resolution.height;
		break;
	}

	GLCHK(glUniform2fv(
		mHistogramStride[prog], GL_VIDEO_TEX_UNIT_COUNT, stride));
	GLCHK(glUniform2fv(
		mHistogramMaxCoords[prog], GL_VIDEO_TEX_UNIT_COUNT, maxCoords));

	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mHistogramYuv2RgbOffset[prog],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(
		mHistogramYuv2RgbMatrix[prog], 1, GL_FALSE, yuv2RgbMatrix));
	GLCHK(glUniform3f(
		mHistogramRgb2LumaMatrix[prog],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][0],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][3],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][6]));
	GLCHK(glUniform1f(
		mHistogramRgb2LumaOffset[prog],
		vdef_rgb_to_yuv_norm_offset[info->matrix_coefs][1][0]));
	GLCHK(glUniform1f(mHistogramBrightnessCoef[prog], mBrightnessCoef));
	GLCHK(glUniform1f(mHistogramContrastCoef[prog], mContrastCoef));
	GLCHK(glUniform1f(mHistogramGammaCoef[prog], mGammaCoef));

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

	GLCHK(glVertexAttribPointer(mHistogramPositionHandle[prog],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices));
	GLCHK(glEnableVertexAttribArray(mHistogramPositionHandle[prog]));

	if (mirrorTexture) {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] =
			(float)crop->top / (float)info->resolution.height;
	} else {
		texCoords[0] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[1] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[2] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[3] =
			(float)crop->top / (float)info->resolution.height;
		texCoords[4] = (float)crop->left / (float)framePlaneStride[0];
		texCoords[5] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
		texCoords[6] = (float)(crop->left + crop->width) /
			       (float)framePlaneStride[0];
		texCoords[7] = (float)(crop->top + crop->height) /
			       (float)info->resolution.height;
	}

	GLCHK(glVertexAttribPointer(mHistogramTexcoordHandle[prog],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords));
	GLCHK(glEnableVertexAttribArray(mHistogramTexcoordHandle[prog]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mHistogramPositionHandle[prog]));
	GLCHK(glDisableVertexAttribArray(mHistogramTexcoordHandle[prog]));

	GLCHK(glFinish());

	/* Read pixels to CPU buffer */
	GLCHK(glReadPixels(0,
			   0,
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
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
	     j < GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE;
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


void GlVideo::getHistograms(
	float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX],
	size_t histogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX]) const
{
	unsigned int i;

	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		if ((mHistogramValid[i]) && (mHistogramNorm[i] != nullptr)) {
			histogram[i] = mHistogramNorm[i];
			histogramLen[i] = 256;
		}
	}
}


void GlVideo::startTransition(enum gl_video_transition transition,
			      uint64_t duration,
			      bool hold)
{
	if (mTransition != GL_VIDEO_TRANSITION_NONE)
		abortTransition();
	mTransition = transition;
	mTransitionDuration = duration;
	mTransitionHold = hold;
}


void GlVideo::abortTransition(void)
{
	mTransition = GL_VIDEO_TRANSITION_NONE;
	mTransitionDuration = 0;
	mTransitionStartTime = 0;
	mTransitionHold = false;
}


void GlVideo::updateTransition(void)
{
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	float progress, blurSigma;

	mApplyBlur = false;
	mSatCoef = mBaseSatCoef;
	mLightCoef = mBaseLightCoef;
	mDarkCoef = mBaseDarkCoef;

	if (mTransition == GL_VIDEO_TRANSITION_NONE)
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
	case GL_VIDEO_TRANSITION_FADE_TO_BLACK:
		mDarkCoef = mBaseDarkCoef * (1. - progress);
		break;
	case GL_VIDEO_TRANSITION_FADE_FROM_BLACK:
		mDarkCoef = mBaseDarkCoef * progress;
		break;
	case GL_VIDEO_TRANSITION_FADE_TO_WHITE:
		mLightCoef = mBaseLightCoef * (1. - progress);
		break;
	case GL_VIDEO_TRANSITION_FADE_FROM_WHITE:
		mLightCoef = mBaseLightCoef * progress;
		break;
	case GL_VIDEO_TRANSITION_FADE_TO_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * (1. - progress);
		break;
	case GL_VIDEO_TRANSITION_FADE_FROM_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * progress;
		break;
	case GL_VIDEO_TRANSITION_FADE_TO_BLUR:
		blurSigma = progress * (GL_VIDEO_BLUR_MAX_SIGMA -
					GL_VIDEO_BLUR_MIN_SIGMA) +
			    GL_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(
			mBlurWeights, GL_VIDEO_BLUR_TAP_COUNT, blurSigma);
		mApplyBlur = mBlurInit;
		break;
	case GL_VIDEO_TRANSITION_FADE_FROM_BLUR:
		blurSigma = (1. - progress) * (GL_VIDEO_BLUR_MAX_SIGMA -
					       GL_VIDEO_BLUR_MIN_SIGMA) +
			    GL_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(
			mBlurWeights, GL_VIDEO_BLUR_TAP_COUNT, blurSigma);
		mApplyBlur = mBlurInit;
		break;
	case GL_VIDEO_TRANSITION_FLASH:
		mLightCoef = mBaseLightCoef *
			     (powf(progress, GL_VIDEO_FLASH_GAMMA_COEF) *
				      GL_VIDEO_FLASH_LIGHT_COEF +
			      1. - GL_VIDEO_FLASH_LIGHT_COEF);
		mSatCoef = mBaseSatCoef *
			   powf(progress, GL_VIDEO_FLASH_GAMMA_COEF);
		break;
	default:
		ULOGE("unsupported transition type: %d", mTransition);
		break;
	}
}


int GlVideo::loadFrame(const uint8_t *framePlanes[VDEF_RAW_MAX_PLANE_COUNT],
		       const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
		       const struct vdef_raw_format *format,
		       const struct vdef_frame_info *info,
		       const uint8_t *mbStatus)
{
	unsigned int i, align;
	bool swapUv = false;

	if ((info == nullptr) || (format == nullptr)) {
		ULOGE("invalid frame info");
		return -EINVAL;
	}
	if ((info->resolution.width == 0) || (info->resolution.height == 0)) {
		ULOGE("invalid dimensions");
		return -EINVAL;
	}
	if (framePlanes == nullptr) {
		ULOGE("invalid planes");
		return -EINVAL;
	}
	size_t _framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT];
	unsigned int planeCount = vdef_get_raw_frame_plane_count(format);
	if (framePlaneStride == nullptr) {
		ULOGE("invalid strides");
		return -EINVAL;
	}
	for (i = 0; i < planeCount; i++) {
		if (framePlaneStride[i] == 0) {
			ULOGE("invalid stride: %zu (plane %u)",
			      framePlaneStride[i],
			      i);
			return -EINVAL;
		}
		if (framePlaneStride[i] % (format->data_size / 8)) {
			ULOGE("invalid stride: %zu (plane %u)",
			      framePlaneStride[i],
			      i);
			return -EINVAL;
		}
		_framePlaneStride[i] =
			framePlaneStride[i] / (format->data_size / 8);
	}

	enum program prog;
	prog = getProgram(format, &swapUv);

	GLint savedAlign = 0;
	GLCHK(glGetIntegerv(GL_UNPACK_ALIGNMENT, &savedAlign));

	GLCHK(glUseProgram(mProgram[prog]));

	switch (prog) {
	default:
	case PROGRAM_NOCONV:
		break;
	case PROGRAM_YUV_TO_RGB_PLANAR:
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			align = getTextureMaxUnpackAlignment(
				_framePlaneStride[i]);
			GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
			GLCHK(glTexImage2D(GL_TEXTURE_2D,
					   0,
					   GL_LUMINANCE,
					   _framePlaneStride[i],
					   height,
					   0,
					   GL_LUMINANCE,
					   GL_UNSIGNED_BYTE,
					   framePlanes[i]));
		}
		break;
	case PROGRAM_YUV_TO_RGB_PLANAR_10_16LE:
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			align = getTextureMaxUnpackAlignment(
				_framePlaneStride[i] * 2);
			GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
			GLCHK(glTexImage2D(GL_TEXTURE_2D,
					   0,
					   GL_LUMINANCE_ALPHA,
					   _framePlaneStride[i],
					   height,
					   0,
					   GL_LUMINANCE_ALPHA,
					   GL_UNSIGNED_BYTE,
					   framePlanes[i]));
		}
		break;
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[0]);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE,
				   _framePlaneStride[0],
				   info->resolution.height,
				   0,
				   GL_LUMINANCE,
				   GL_UNSIGNED_BYTE,
				   framePlanes[0]));

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[i]);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE_ALPHA,
				   _framePlaneStride[1] / 2,
				   info->resolution.height / 2,
				   0,
				   GL_LUMINANCE_ALPHA,
				   GL_UNSIGNED_BYTE,
				   framePlanes[1]));
		break;
	case PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[0] * 2);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE_ALPHA,
				   _framePlaneStride[0],
				   info->resolution.height,
				   0,
				   GL_LUMINANCE_ALPHA,
				   GL_UNSIGNED_BYTE,
				   framePlanes[0]));

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[1] * 2);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_RGBA,
				   _framePlaneStride[1] / 2,
				   info->resolution.height / 2,
				   0,
				   GL_RGBA,
				   GL_UNSIGNED_BYTE,
				   framePlanes[1]));
		break;
	case PROGRAM_GRAY_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[0]);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE,
				   _framePlaneStride[0],
				   info->resolution.height,
				   0,
				   GL_LUMINANCE,
				   GL_UNSIGNED_BYTE,
				   framePlanes[0]));
		break;
	case PROGRAM_GRAY16_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[0]) * 2;
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE_ALPHA,
				   _framePlaneStride[0],
				   info->resolution.height,
				   0,
				   GL_LUMINANCE_ALPHA,
				   GL_UNSIGNED_BYTE,
				   framePlanes[0]));
		break;
	case PROGRAM_GRAY32_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(_framePlaneStride[0] * 4);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_RGBA,
				   _framePlaneStride[0],
				   info->resolution.height,
				   0,
				   GL_RGBA,
				   GL_UNSIGNED_BYTE,
				   framePlanes[0]));
		break;
	}

	if (mbStatus != nullptr) {
		unsigned int mbWidth = (info->resolution.width + 15) / 16;
		unsigned int mbHeight = (info->resolution.height + 15) / 16;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
				      GL_VIDEO_TEX_UNIT_COUNT +
				      GL_VIDEO_FBO_TEX_UNIT_COUNT));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mMbStatusTexture));
		align = getTextureMaxUnpackAlignment(mbWidth);
		GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, align));
		GLCHK(glTexImage2D(GL_TEXTURE_2D,
				   0,
				   GL_LUMINANCE,
				   mbWidth,
				   mbHeight,
				   0,
				   GL_LUMINANCE,
				   GL_UNSIGNED_BYTE,
				   mbStatus));
		mHasMbStatus = true;
	} else {
		mHasMbStatus = false;
	}

	GLCHK(glPixelStorei(GL_UNPACK_ALIGNMENT, savedAlign));
	return 0;
}


int GlVideo::renderFrame(
	const struct pdraw_rect *renderPos,
	struct pdraw_rect *contentPos,
	const Eigen::Matrix4f &viewProjMat,
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_video_renderer_params *params)
{
	int ret;
	unsigned int i;
	float stride[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float maxCoords[GL_VIDEO_TEX_UNIT_COUNT * 2] = {0};
	float vertices[12];
	float texCoords[8];
	float yuv2RgbMatrix[9];
	float yuv2RgbOffset[3];
	bool mirrorTexture = false, swapUv = false;
	float videoAR;
	GLboolean glBlendEnabled, glDepthTestEnabled;

	if ((renderPos == nullptr) || (renderPos->width == 0) ||
	    (renderPos->height == 0)) {
		ULOGE("invalid render position");
		return -EINVAL;
	}
	if ((info == nullptr) || (format == nullptr)) {
		ULOGE("invalid frame info");
		return -EINVAL;
	}
	if ((info->resolution.width == 0) || (info->resolution.height == 0) ||
	    (info->sar.width == 0) || (info->sar.height == 0)) {
		ULOGE("invalid dimensions");
		return -EINVAL;
	}
	size_t _framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT];
	unsigned int planeCount = vdef_get_raw_frame_plane_count(format);
	if (framePlaneStride == nullptr) {
		ULOGE("invalid strides");
		return -EINVAL;
	}
	for (i = 0; i < planeCount; i++) {
		if (framePlaneStride[i] == 0) {
			ULOGE("invalid stride: %zu (plane %u)",
			      framePlaneStride[i],
			      i);
			return -EINVAL;
		}
		if (framePlaneStride[i] % (format->data_size / 8)) {
			ULOGE("invalid stride: %zu (plane %u)",
			      framePlaneStride[i],
			      i);
			return -EINVAL;
		}
		_framePlaneStride[i] =
			framePlaneStride[i] / (format->data_size / 8);
	}

	struct vdef_frame_info _info = *info;
	if (_info.matrix_coefs == VDEF_MATRIX_COEFS_UNKNOWN) {
		/* Default to BT.709 */
		_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	}

	glBlendEnabled = glIsEnabled(GL_BLEND);
	if (glBlendEnabled)
		GLCHK(glDisable(GL_BLEND));
	glDepthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
	if (glDepthTestEnabled)
		GLCHK(glDisable(GL_DEPTH_TEST));

	enum program prog;
	prog = getProgram(format, &swapUv);

	bool setupBlur = ((mVideoWidth != _info.resolution.width) ||
			  (mVideoHeight != _info.resolution.height));
	bool setupPadding = (setupBlur || (mFillMode != params->fill_mode));
	mVideoWidth = _info.resolution.width;
	mVideoHeight = _info.resolution.height;
	mFillMode = params->fill_mode;

	if (setupBlur) {
		ret = setupBlurFbo();
		if (ret < 0)
			ULOG_ERRNO("setupBlurFbo", -ret);
	}
	if (setupPadding) {
		ret = setupPaddingFbo();
		if (ret < 0)
			ULOG_ERRNO("setupPaddingFbo", -ret);
	}

	updateTransition();

	computeHistograms(_framePlaneStride,
			  format,
			  &_info,
			  crop,
			  renderPos,
			  params->enable_histograms);

	/* Video fill mode */
	float windowAR = (float)renderPos->width / (float)renderPos->height;
	float sar = (float)_info.sar.width / (float)_info.sar.height;
	if (params->video_texture_dar_height != 0 &&
	    params->video_texture_dar_width != 0) {
		/* If the display aspect ratio is given,
		 * we apply it instead of the source width/height */
		videoAR = (float)params->video_texture_dar_width /
			  (float)params->video_texture_dar_height;
	} else {
		videoAR = (float)_info.resolution.width /
			  (float)_info.resolution.height * sar;
	}

	float windowW = 1.;
	float windowH = windowAR;
	float ratioW = 1.;
	float ratioH = 1.;
	float ratioW2 = 1.;
	float ratioH2 = 1.;
	switch (params->fill_mode) {
	default:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND:
		/* Maintain video aspect ratio without crop and add borders if
		 * window aspect ratio and video aspect ratio differ */
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
		/* Maintain video aspect ratio without borders and add video
		 * crop if window aspect ratio and video aspect ratio differ */
		if (videoAR >= windowAR) {
			ratioW = videoAR / windowAR;
			ratioH = 1.;
		} else {
			ratioW = 1.;
			ratioH = windowAR / videoAR;
		}
		break;
	}
	float videoW = ratioW / windowW;
	float videoH = ratioH / windowH;
	float videoW2 = ratioW2 / windowW;
	float videoH2 = ratioH2 / windowH;

	if (contentPos) {
		int32_t dw;
		int32_t dh;
		contentPos->width = ratioW * renderPos->width;
		contentPos->height = ratioH * renderPos->height;

		dw = (int32_t)renderPos->width - (int32_t)contentPos->width;
		dh = (int32_t)renderPos->height - (int32_t)contentPos->height;
		contentPos->x = dw / 2;
		contentPos->y = dh / 2;
	}

	if (videoAR != windowAR) {
		renderPadding(_framePlaneStride,
			      format,
			      &_info,
			      crop,
			      renderPos,
			      videoW,
			      videoH,
			      videoW2,
			      videoH2,
			      videoAR,
			      windowAR,
			      false,
			      viewProjMat);
	}

	if (mApplyBlur) {
		renderBlur(_framePlaneStride,
			   format,
			   &_info,
			   crop,
			   renderPos,
			   videoW,
			   videoH,
			   viewProjMat);
	} else {
		GLCHK(glUseProgram(mProgram[prog]));

		switch (prog) {
		default:
		case PROGRAM_GRAY_TO_RGB_PLANAR:
		case PROGRAM_GRAY16_TO_RGB_PLANAR:
		case PROGRAM_GRAY32_TO_RGB_PLANAR:
			mirrorTexture = true;
			/* Fall through */
		case PROGRAM_NOCONV:
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
			GLCHK(glBindTexture(GL_TEXTURE_2D,
					    (mExtTexture > 0) ? mExtTexture
							      : mTextures[0]));
			GLCHK(glUniform1i(mUniformSamplers[prog][0],
					  mFirstTexUnit));
			stride[0] = 1.f / _framePlaneStride[0];
			stride[1] = 1.f / _info.resolution.height;
			maxCoords[0] = (float)(crop->left + crop->width) /
				       _framePlaneStride[0];
			maxCoords[1] = (float)(crop->top + crop->height) /
				       _info.resolution.height;
			break;
		case PROGRAM_YUV_TO_RGB_PLANAR:
		case PROGRAM_YUV_TO_RGB_PLANAR_10_16LE:
			mirrorTexture = true;
			for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
				int height = _info.resolution.height /
					     ((i > 0) ? 2 : 1);
				GLCHK(glActiveTexture(GL_TEXTURE0 +
						      mFirstTexUnit + i));
				GLCHK(glBindTexture(GL_TEXTURE_2D,
						    mTextures[i]));
				GLCHK(glUniform1i(mUniformSamplers[prog][i],
						  mFirstTexUnit + i));
				stride[2 * i] = 1.f / _framePlaneStride[i];
				stride[2 * i + 1] = 1.f / height;
				maxCoords[2 * i] =
					(float)(crop->left + crop->width) /
					(_framePlaneStride[i] *
					 ((i > 0) ? 2 : 1));
				maxCoords[2 * i + 1] =
					(float)(crop->top + crop->height) /
					_info.resolution.height;
			}
			break;
		case PROGRAM_YUV_TO_RGB_SEMIPLANAR:
		case PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
			mirrorTexture = true;
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
			GLCHK(glUniform1i(mUniformSamplers[prog][0],
					  mFirstTexUnit + 0));
			stride[0] = 1.f / _framePlaneStride[0];
			stride[1] = 1.f / _info.resolution.height;
			maxCoords[0] = (float)(crop->left + crop->width) /
				       _framePlaneStride[0];
			maxCoords[1] = (float)(crop->top + crop->height) /
				       _info.resolution.height;

			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
			GLCHK(glUniform1i(mUniformSamplers[prog][1],
					  mFirstTexUnit + 1));
			stride[2] = 1.f / (_framePlaneStride[1] / 2);
			stride[3] = 1.f / (_info.resolution.height / 2);
			maxCoords[2] = (float)(crop->left + crop->width) /
				       _framePlaneStride[1];
			maxCoords[3] = (float)(crop->top + crop->height) /
				       _info.resolution.height;
			break;
		}

		GLCHK(glUniform2fv(
			mProgramStride[prog], GL_VIDEO_TEX_UNIT_COUNT, stride));
		GLCHK(glUniform2fv(mProgramMaxCoords[prog],
				   GL_VIDEO_TEX_UNIT_COUNT,
				   maxCoords));
		fillYuv2RgbMatrix(_info.matrix_coefs,
				  _info.full_range,
				  swapUv,
				  yuv2RgbMatrix,
				  yuv2RgbOffset);
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[prog],
				  yuv2RgbOffset[0],
				  yuv2RgbOffset[1],
				  yuv2RgbOffset[2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[prog],
					 1,
					 GL_FALSE,
					 yuv2RgbMatrix));

		/* Update overexposure zebras */
		updateZebra(contentPos,
			    prog,
			    params->enable_overexposure_zebras,
			    params->overexposure_zebras_threshold);

		/* MB status display */
		GLCHK(glUniform1f(mProgramMbStatusEnable[prog],
				  mHasMbStatus ? 1.f : 0.f));
		if (mHasMbStatus) {
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
					      GL_VIDEO_TEX_UNIT_COUNT +
					      GL_VIDEO_FBO_TEX_UNIT_COUNT));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mMbStatusTexture));
			GLCHK(glUniform1i(mMbStatusUniformSampler[prog],
					  mFirstTexUnit +
						  GL_VIDEO_TEX_UNIT_COUNT +
						  GL_VIDEO_FBO_TEX_UNIT_COUNT));
		}

		GLCHK(glUniformMatrix4fv(mProgramTransformMatrix[prog],
					 1,
					 false,
					 viewProjMat.data()));
		GLCHK(glUniform1f(mProgramBrightnessCoef[prog],
				  mBrightnessCoef));
		GLCHK(glUniform1f(mProgramContrastCoef[prog], mContrastCoef));
		GLCHK(glUniform1f(mProgramGammaCoef[prog], mGammaCoef));
		GLCHK(glUniform1f(mProgramSatCoef[prog], mSatCoef));
		GLCHK(glUniform1f(mProgramLightCoef[prog], mLightCoef));
		GLCHK(glUniform1f(mProgramDarkCoef[prog], mDarkCoef));

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

		GLCHK(glVertexAttribPointer(mPositionHandle[prog],
					    3,
					    GL_FLOAT,
					    false,
					    0,
					    vertices));
		GLCHK(glEnableVertexAttribArray(mPositionHandle[prog]));

		if (mirrorTexture) {
			texCoords[0] =
				(float)crop->left / (float)_framePlaneStride[0];
			texCoords[1] = (float)(crop->top + crop->height) /
				       (float)_info.resolution.height;
			texCoords[2] = (float)(crop->left + crop->width) /
				       (float)_framePlaneStride[0];
			texCoords[3] = (float)(crop->top + crop->height) /
				       (float)_info.resolution.height;
			texCoords[4] =
				(float)crop->left / (float)_framePlaneStride[0];
			texCoords[5] = (float)crop->top /
				       (float)_info.resolution.height;
			texCoords[6] = (float)(crop->left + crop->width) /
				       (float)_framePlaneStride[0];
			texCoords[7] = (float)crop->top /
				       (float)_info.resolution.height;
		} else {
			texCoords[0] =
				(float)crop->left / (float)_framePlaneStride[0];
			texCoords[1] = (float)crop->top /
				       (float)_info.resolution.height;
			texCoords[2] = (float)(crop->left + crop->width) /
				       (float)_framePlaneStride[0];
			texCoords[3] = (float)crop->top /
				       (float)_info.resolution.height;
			texCoords[4] =
				(float)crop->left / (float)_framePlaneStride[0];
			texCoords[5] = (float)(crop->top + crop->height) /
				       (float)_info.resolution.height;
			texCoords[6] = (float)(crop->left + crop->width) /
				       (float)_framePlaneStride[0];
			texCoords[7] = (float)(crop->top + crop->height) /
				       (float)_info.resolution.height;
		}

		GLCHK(glVertexAttribPointer(mTexcoordHandle[prog],
					    2,
					    GL_FLOAT,
					    false,
					    0,
					    texCoords));
		GLCHK(glEnableVertexAttribArray(mTexcoordHandle[prog]));

		GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

		GLCHK(glDisableVertexAttribArray(mPositionHandle[prog]));
		GLCHK(glDisableVertexAttribArray(mTexcoordHandle[prog]));
	}

	if (glBlendEnabled)
		GLCHK(glEnable(GL_BLEND));
	if (glDepthTestEnabled)
		GLCHK(glEnable(GL_DEPTH_TEST));

	return 0;
}


int GlVideo::clear(const Eigen::Matrix4f &viewProjMat)
{
	float vertices[12];

	GLCHK(glUseProgram(mClearProgram));

	GLCHK(glUniformMatrix4fv(
		mClearProgramTransformMatrix, 1, false, viewProjMat.data()));
	GLCHK(glUniform3f(mClearProgramColor, 0., 0., 0.));

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

	GLCHK(glVertexAttribPointer(
		mClearProgramPositionHandle, 3, GL_FLOAT, false, 0, vertices));
	GLCHK(glEnableVertexAttribArray(mClearProgramPositionHandle));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mClearProgramPositionHandle));

	return 0;
}


void GlVideo::setExtTexture(GLuint texture)
{
	mExtTexture = texture;
}

} /* namespace Pdraw */

#endif /* PDRAW_USE_GL */
