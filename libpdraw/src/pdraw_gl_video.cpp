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

#include "pdraw_gl_video.hpp"

#ifdef PDRAW_USE_GL

#	include "pdraw_session.hpp"

#	include <math.h>

#	include <futils/futils.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


/* Blurred padding dark coef */
constexpr float PDRAW_BLURRED_PADDING_DARK_COEF = 0.75f;

/* The temporal frequency of zebra pattern */
constexpr float PDRAW_ZEBRA_FREQUENCY_HZ = 1.f;
/* The angle of zebra pattern relative to y axis */
constexpr float PDRAW_ZEBRA_ANGLE = (60.f * static_cast<float>(M_PI) / 180.f);
/* The weight in pixels of zebra pattern, relative to 1920 width */
constexpr float PDRAW_ZEBRA_WEIGHT = 8.f;

constexpr float GL_VIDEO_BLUR_MIN_SIGMA = 0.8f;
constexpr float GL_VIDEO_BLUR_MAX_SIGMA = 6.0f;
constexpr float GL_VIDEO_BLURRED_PADDING_SIGMA = 3.0f;

constexpr size_t GL_VIDEO_HISTOGRAM_COMPUTE_INTERVAL_US = 100000;

constexpr float GL_VIDEO_FLASH_LIGHT_COEF = 0.3f;
constexpr float GL_VIDEO_FLASH_GAMMA_COEF = 2.0f;

#	ifdef GL_ES_VERSION_2_0
/* Default OpenGL ES Shading Language version (1.10.59) */
#		define GLSL_VERSION "#version 100\n"
#	else
/* Default OpenGL Shading Language version (1.00.17) */
#		define GLSL_VERSION "#version 110\n"
#	endif

static const GLchar *const videoVertexShader =
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

static const GLchar *const zebraFragmentShader =
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

static const GLchar *const mbStatusFragmentShader =
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

static const GLchar *const textureNoconvFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    return texture2D(s_texture_0, min(max_clamp[0], coord)).rgb;\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    return texture2D(s_texture_0, min(max_clamp[0], coord + offset_px * stride[0])).rgb;\n"
	"}\n";

static const GLchar *const textureI420FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y = min(max_clamp[0], coord);\n"
	"    vec2 texCoord_u = min(max_clamp[1], coord * max_coords_ratio[1]);\n"
	"    vec2 texCoord_v = min(max_clamp[2], coord * max_coords_ratio[2]);\n"
	"    yuv.r = texture2D(s_texture_0, texCoord_y).r;\n"
	"    yuv.g = texture2D(s_texture_1, texCoord_u).r;\n"
	"    yuv.b = texture2D(s_texture_2, texCoord_v).r;\n"
	"    return yuv2rgb_mat * (yuv + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y = min(max_clamp[0], coord + offset_px * stride[0]);\n"
	"    vec2 texCoord_u = min(max_clamp[1], coord * max_coords_ratio[1] + offset_px * stride[1]);\n"
	"    vec2 texCoord_v = min(max_clamp[2], coord * max_coords_ratio[2] + offset_px * stride[2]);\n"
	"    yuv.r = texture2D(s_texture_0, texCoord_y).r;\n"
	"    yuv.g = texture2D(s_texture_1, texCoord_u).r;\n"
	"    yuv.b = texture2D(s_texture_2, texCoord_v).r;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

/* YUV 4:2:0 planar with 16 bits data format in little endian
 * and 10 bits depth, padding in higher bits */
static const GLchar *const textureI42010LELowFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y = min(max_clamp[0], coord);\n"
	"    vec2 texCoord_u = min(max_clamp[1], coord * max_coords_ratio[1]);\n"
	"    vec2 texCoord_v = min(max_clamp[2], coord * max_coords_ratio[2]);\n"
	"    vec4 y = texture2D(s_texture_0, texCoord_y);\n"
	"    vec4 u = texture2D(s_texture_1, texCoord_u);\n"
	"    vec4 v = texture2D(s_texture_2, texCoord_v);\n"
	"    yuv.r = y.a * 64. + y.r / 4.;\n"
	"    yuv.g = u.a * 64. + u.r / 4.;\n"
	"    yuv.b = v.a * 64. + v.r / 4.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y = min(max_clamp[0], coord + offset_px * stride[0]);\n"
	"    vec2 texCoord_u = min(max_clamp[1], coord * max_coords_ratio[1] + offset_px * stride[1]);\n"
	"    vec2 texCoord_v = min(max_clamp[2], coord * max_coords_ratio[2] + offset_px * stride[2]);\n"
	"    vec4 y = texture2D(s_texture_0, texCoord_y);\n"
	"    vec4 u = texture2D(s_texture_1, texCoord_u);\n"
	"    vec4 v = texture2D(s_texture_2, texCoord_v);\n"
	"    yuv.r = y.a * 64. + y.r / 4.;\n"
	"    yuv.g = u.a * 64. + u.r / 4.;\n"
	"    yuv.b = v.a * 64. + v.r / 4.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *const textureNV12FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y  = min(max_clamp[0], coord);\n"
	"    vec2 texCoord_uv = min(max_clamp[1], coord * max_coords_ratio[1]);\n"
	"    yuv.r  = texture2D(s_texture_0,  texCoord_y).r;\n"
	"    yuv.gb = texture2D(s_texture_1, texCoord_uv).ra;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y  = min(max_clamp[0], coord + offset_px * stride[0]);\n"
	"    vec2 texCoord_uv = min(max_clamp[1], coord * max_coords_ratio[1] + offset_px * stride[1]);\n"
	"    yuv.r  = texture2D(s_texture_0,  texCoord_y).r;\n"
	"    yuv.gb = texture2D(s_texture_1, texCoord_uv).ra;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

/* YUV 4:2:0 semi-planar with 16 bits data format in little endian
 * and 10 bits depth, padding in lower bits */
static const GLchar *const textureNV1210LEHighFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y  = min(max_clamp[0], coord);\n"
	"    vec2 texCoord_uv = min(max_clamp[1], coord * max_coords_ratio[1]);\n"
	"    vec4 y  = texture2D(s_texture_0,  texCoord_y);\n"
	"    vec4 uv = texture2D(s_texture_1, texCoord_uv);\n"
	"    yuv.r = y.a + y.r / 256.;\n"
	"    yuv.g = uv.g + uv.b / 256.;\n"
	"    yuv.b = uv.a + uv.r / 256.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec2 texCoord_y  = min(max_clamp[0], coord + offset_px * stride[0]);\n"
	"    vec2 texCoord_uv = min(max_clamp[1], coord * max_coords_ratio[1] + offset_px * stride[1]);\n"
	"    vec4 y  = texture2D(s_texture_0,  texCoord_y);\n"
	"    vec4 uv = texture2D(s_texture_1, texCoord_uv);\n"
	"    yuv.r = y.a + y.r / 256.;\n"
	"    yuv.g = uv.g + uv.b / 256.;\n"
	"    yuv.b = uv.a + uv.r / 256.;\n"
	"    return yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"}\n";

static const GLchar *const textureGrayFragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    float gray = texture2D(s_texture_0, min(max_clamp[0], coord)).r;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    float gray = texture2D(s_texture_0, min(max_clamp[0], coord + offset_px * stride[0])).r;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *const textureGray16FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(max_clamp[0], coord));\n"
	"    float gray = p.a + p.r / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(max_clamp[0], coord + offset_px * stride[0]));\n"
	"    float gray = p.a + p.r / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *const textureGray32FragmentShader =
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform vec2 stride[3];\n"
	"uniform vec2 max_coords_ratio[3];\n"
	"uniform vec2 max_clamp[3];\n"
	"\n"
	"vec3 read_rgb(vec2 coord)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(max_clamp[0], coord));\n"
	"    float gray = p.a + p.r / 256. + p.g / 256. / 256. + p.b / 256. / 256. / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n"
	"\n"
	"vec3 read_rgb_with_offset(vec2 coord, vec2 offset_px)\n"
	"{\n"
	"    vec4 p = texture2D(s_texture_0, min(max_clamp[0], coord + offset_px * stride[0]));\n"
	"    float gray = p.a + p.r / 256. + p.g / 256. / 256. + p.b / 256. / 256. / 256.;\n"
	"    return vec3(gray, gray, gray);\n"
	"}\n";

static const GLchar *const videoFragmentShader =
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

static const GLchar *const simpleVideoFragmentShader =
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

const std::array<
	std::array<std::array<const GLchar *, 5>, GlVideo::PROGRAM_MAX>,
	2>
	GlVideo::videoFragmentShaders = {{
		{{
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
		}},
		{{
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
		}},
	}};

static const GLchar *const simpleFragmentShader =
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

static const GLchar *const clearFragmentShader =
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

static const GLchar *const blurHVertexShader =
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

static const GLchar *const blurVVertexShader =
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

static const GLchar *const blurHFragmentShader =
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

static const GLchar *const blurVFragmentShader =
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

static const GLchar *const histogramVertexShader =
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

static const GLchar *const histogramFragmentShader =
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

const std::array<std::array<const GLchar *, 3>, GlVideo::PROGRAM_MAX>
	GlVideo::histogramFragmentShaders = {{
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
	}};

static constexpr std::array<GLfloat, 9> zebraAvgWeights = {0.077847f,
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
		 bool simplified) :
		mSession(session),
		mFirstTexUnit(firstTexUnit), mDefaultFbo(defaultFbo)
{
	int ret;
	GLint vertexShader = 0;
	std::array<GLint, PROGRAM_MAX> fragmentShader{};
	GLint success = 0;
	unsigned int i;

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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog.data());
		ULOGE("vertex shader compilation failed '%s'", infoLog.data());
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

		glShaderSource(
			fragmentShader[i],
			5,
			videoFragmentShaders[simplified ? 1 : 0][i].data(),
			nullptr);
		glCompileShader(fragmentShader[i]);
		glGetShaderiv(fragmentShader[i], GL_COMPILE_STATUS, &success);
		if (!success) {
			std::array<char, 512> infoLog{};
			glGetShaderInfoLog(fragmentShader[i],
					   512,
					   nullptr,
					   infoLog.data());
			ULOGE("fragment shader compilation failed '%s'",
			      infoLog.data());
			goto err;
		}

		/* Link shaders */
		mProgram[i] = glCreateProgram();
		glAttachShader(mProgram[i], vertexShader);
		glAttachShader(mProgram[i], fragmentShader[i]);
		glLinkProgram(mProgram[i]);
		glGetProgramiv(mProgram[i], GL_LINK_STATUS, &success);
		if (!success) {
			std::array<char, 512> infoLog{};
			glGetProgramInfoLog(
				mProgram[i], 512, nullptr, infoLog.data());
			ULOGE("program link failed '%s'", infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(
			fragmentShader[0], 512, nullptr, infoLog.data());
		ULOGE("fragment shader compilation failed '%s'",
		      infoLog.data());
		goto err;
	}

	/* Link shaders */
	mSimpleProgram = glCreateProgram();
	glAttachShader(mSimpleProgram, vertexShader);
	glAttachShader(mSimpleProgram, fragmentShader[0]);
	glLinkProgram(mSimpleProgram);
	glGetProgramiv(mSimpleProgram, GL_LINK_STATUS, &success);
	if (!success) {
		std::array<char, 512> infoLog{};
		glGetProgramInfoLog(
			mSimpleProgram, 512, nullptr, infoLog.data());
		ULOGE("program link failed '%s'", infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(
			fragmentShader[0], 512, nullptr, infoLog.data());
		ULOGE("fragment shader compilation failed '%s'",
		      infoLog.data());
		goto err;
	}

	/* Link shaders */
	mClearProgram = glCreateProgram();
	glAttachShader(mClearProgram, vertexShader);
	glAttachShader(mClearProgram, fragmentShader[0]);
	glLinkProgram(mClearProgram);
	glGetProgramiv(mClearProgram, GL_LINK_STATUS, &success);
	if (!success) {
		std::array<char, 512> infoLog{};
		glGetProgramInfoLog(
			mClearProgram, 512, nullptr, infoLog.data());
		ULOGE("program link failed '%s'", infoLog.data());
		goto err;
	}

	glDeleteShader(fragmentShader[0]);
	fragmentShader[0] = 0;

	glDeleteShader(vertexShader);

	GLCHK();

	for (i = 0; i < PROGRAM_MAX; i++) {
		mProgramTransformMatrix[i] =
			glGetUniformLocation(mProgram[i], "transform_matrix");
		mProgramYuv2RgbMatrix[i] =
			glGetUniformLocation(mProgram[i], "yuv2rgb_mat");
		mProgramYuv2RgbOffset[i] =
			glGetUniformLocation(mProgram[i], "yuv2rgb_offset");
		mProgramStride[i] = glGetUniformLocation(mProgram[i], "stride");
		mProgramMaxCoordsRatio[i] =
			glGetUniformLocation(mProgram[i], "max_coords_ratio");
		mProgramMaxClamp[i] =
			glGetUniformLocation(mProgram[i], "max_clamp");
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
		setupZebra((Pdraw::GlVideo::Program)i);

	GLCHK(glGenTextures(GL_VIDEO_TEX_UNIT_COUNT, mTextures.data()));

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
		GLCHK(glDeleteTextures(GL_VIDEO_TEX_UNIT_COUNT,
				       mTextures.data()));
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
	mProgram.fill(0);
	mTextures.fill(0);
	mMbStatusTexture = 0;
	cleanupBlur();
	cleanupHistograms();
}


GlVideo::~GlVideo()
{
	if (mMbStatusTexture)
		GLCHK(glDeleteTextures(1, &mMbStatusTexture));
	if (mTextures[0])
		GLCHK(glDeleteTextures(GL_VIDEO_TEX_UNIT_COUNT,
				       mTextures.data()));
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


GlVideo::Program GlVideo::getProgram(const struct vdef_raw_format *format,
				     bool *swapUv) const
{
	*swapUv = false;
	if (vdef_raw_format_cmp(format, &vdef_i420)) {
		return Program::YUV_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_nv12)) {
		return Program::YUV_TO_RGB_SEMIPLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_nv21)) {
		*swapUv = true;
		return Program::YUV_TO_RGB_SEMIPLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_i420_10_16le)) {
		return Program::YUV_TO_RGB_PLANAR_10_16LE;
	} else if (vdef_raw_format_cmp(format, &vdef_nv12_10_16le_high)) {
		return Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH;
	} else if (vdef_raw_format_cmp(format, &vdef_gray)) {
		return Program::GRAY_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_raw16)) {
		return Program::GRAY16_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_raw32)) {
		return Program::GRAY32_TO_RGB_PLANAR;
	} else if (vdef_raw_format_cmp(format, &vdef_rgb)) {
		return Program::NOCONV;
	} else if (vdef_raw_format_cmp(format, &vdef_opaque)) {
		return Program::NOCONV;
	} else {
		ULOGE("unsupported frame format");
		return Program::NOCONV;
	}
}


void GlVideo::fillYuv2RgbMatrix(enum vdef_matrix_coefs matrixCoefs,
				bool fullRange,
				bool swapUv,
				std::array<float, 9> &yuv2RgbMatrix,
				std::array<float, 3> &yuv2RgbOffset) const
{
	int fr = fullRange ? 1 : 0;

	memcpy(yuv2RgbMatrix.data(),
	       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr],
	       3 * sizeof(float));
	if (swapUv) {
		memcpy(yuv2RgbMatrix.data() + 3,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 6,
		       3 * sizeof(float));
		memcpy(yuv2RgbMatrix.data() + 6,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 3,
		       3 * sizeof(float));
		yuv2RgbOffset[0] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][0];
		yuv2RgbOffset[1] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][2];
		yuv2RgbOffset[2] =
			vdef_yuv_to_rgb_norm_offset[matrixCoefs][fr][1];
	} else {
		memcpy(yuv2RgbMatrix.data() + 3,
		       vdef_yuv_to_rgb_norm_matrix[matrixCoefs][fr] + 3,
		       3 * sizeof(float));
		memcpy(yuv2RgbMatrix.data() + 6,
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


int GlVideo::setupBlur()
{
	int ret = 0;
	GLint vertexShaderH = 0;
	GLint vertexShaderV = 0;
	GLint fragmentShaderH = 0;
	GLint fragmentShaderV = 0;
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(vertexShaderH, 512, nullptr, infoLog.data());
		ULOGE("vertex shader (H) compilation failed '%s'",
		      infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(vertexShaderV, 512, nullptr, infoLog.data());
		ULOGE("vertex shader (V) compilation failed '%s'",
		      infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(
			fragmentShaderH, 512, nullptr, infoLog.data());
		ULOGE("fragment shader compilation failed '%s'",
		      infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(
			fragmentShaderV, 512, nullptr, infoLog.data());
		ULOGE("fragment shader compilation failed '%s'",
		      infoLog.data());
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
		std::array<char, 512> infoLog{};
		glGetProgramInfoLog(
			mBlurProgram[0], 512, nullptr, infoLog.data());
		ULOGE("program link failed '%s'", infoLog.data());
		ret = -EPROTO;
		goto error;
	}

	mBlurProgram[1] = glCreateProgram();
	glAttachShader(mBlurProgram[1], vertexShaderV);
	glAttachShader(mBlurProgram[1], fragmentShaderV);
	glLinkProgram(mBlurProgram[1]);
	glGetProgramiv(mBlurProgram[1], GL_LINK_STATUS, &success);
	if (!success) {
		std::array<char, 512> infoLog{};
		glGetProgramInfoLog(
			mBlurProgram[1], 512, nullptr, infoLog.data());
		ULOGE("program link failed '%s'", infoLog.data());
		ret = -EPROTO;
		goto error;
	}

	glDeleteShader(vertexShaderH);
	glDeleteShader(vertexShaderV);
	glDeleteShader(fragmentShaderH);
	glDeleteShader(fragmentShaderV);

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


void GlVideo::cleanupBlur()
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


int GlVideo::setupBlurFbo()
{
	int ret = 0;
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
	for (unsigned int i = 0; i < 2; i++) {
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


void GlVideo::cleanupBlurFbo()
{
	if (mBlurFboTexture[0] > 0) {
		GLCHK(glDeleteTextures(2, mBlurFboTexture.data()));
		mBlurFboTexture.fill(0);
	}
	if (mBlurFbo[0] > 0) {
		GLCHK(glDeleteFramebuffers(2, mBlurFbo.data()));
		mBlurFbo.fill(0);
	}
	mBlurFboWidth = 0;
	mBlurFboHeight = 0;
}


static inline void computeMaxCoordsRatioAndClamp(
	const std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> &stride,
	const std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> &maxCoords,
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> &maxCoordsRatio,
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> &maxClamp)
{
	for (unsigned int i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
		unsigned int x = 2 * i;
		unsigned int y = 2 * i + 1;
		maxCoordsRatio[x] = (maxCoords[0] != 0)
					    ? (maxCoords[x] / maxCoords[0])
					    : 1.f;
		maxCoordsRatio[y] = (maxCoords[1] != 0)
					    ? (maxCoords[y] / maxCoords[1])
					    : 1.f;
		maxClamp[x] = maxCoords[x] - stride[x] / 2.0f;
		maxClamp[y] = maxCoords[y] - stride[y] / 2.0f;
	}
}


void GlVideo::renderBlur(
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_rect *renderPos,
	float videoW,
	float videoH,
	bool verticalMirror,
	const Eigen::Matrix4f &viewProjMat) const
{
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> stride{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoords{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoordsRatio{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxClamp{};
	std::array<float, 12> vertices{};
	std::array<float, 8> texCoords{};
	std::array<float, 9> yuv2RgbMatrix{};
	std::array<float, 3> yuv2RgbOffset{};
	bool mirrorTexture = verticalMirror;
	bool swapUv = false;

	if (!mBlurInit)
		return;

	Program prog;
	prog = getProgram(format, &swapUv);

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[0]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));

	switch (prog) {
	default:
	case Program::GRAY_TO_RGB_PLANAR:
	case Program::GRAY16_TO_RGB_PLANAR:
	case Program::GRAY32_TO_RGB_PLANAR:
		mirrorTexture = !verticalMirror;
		/* Fall through */
	case Program::NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
				  mFirstTexUnit));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	case Program::YUV_TO_RGB_PLANAR:
	case Program::YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = !verticalMirror;
		for (unsigned int i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][i],
					  mFirstTexUnit + i));
			stride[2 * i] =
				1.f / static_cast<float>(framePlaneStride[i]);
			stride[2 * i + 1] = 1.f / static_cast<float>(height);
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				static_cast<float>(framePlaneStride[i] *
						   ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				static_cast<float>(info->resolution.height);
		}
		break;
	case Program::YUV_TO_RGB_SEMIPLANAR:
	case Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = !verticalMirror;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / static_cast<float>(framePlaneStride[1] / 2);
		stride[3] =
			1.f / static_cast<float>(info->resolution.height / 2);
		maxCoords[2] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[1]);
		maxCoords[3] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	}

	computeMaxCoordsRatioAndClamp(
		stride, maxCoords, maxCoordsRatio, maxClamp);

	GLCHK(glUniform2fv(mProgramStride[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   stride.data()));
	GLCHK(glUniform2fv(mProgramMaxCoordsRatio[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxCoordsRatio.data()));
	GLCHK(glUniform2fv(mProgramMaxClamp[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxClamp.data()));
	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mProgramYuv2RgbOffset[toIndex(prog)],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[toIndex(prog)],
				 1,
				 GL_FALSE,
				 yuv2RgbMatrix.data()));

	/* Disable overexposure zebras */
	updateZebra(nullptr, prog, false, 0.f);

	/* Disable MB status display */
	GLCHK(glUniform1f(mProgramMbStatusEnable[toIndex(prog)], 0.f));

	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = 1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = -1.f;
	vertices[7] = 1.f;
	vertices[8] = 1.f;
	vertices[9] = 1.f;
	vertices[10] = 1.f;
	vertices[11] = 1.f;

	Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[toIndex(prog)], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramBrightnessCoef[toIndex(prog)],
			  mBrightnessCoef));
	GLCHK(glUniform1f(mProgramContrastCoef[toIndex(prog)], mContrastCoef));
	GLCHK(glUniform1f(mProgramGammaCoef[toIndex(prog)], mGammaCoef));
	GLCHK(glUniform1f(mProgramSatCoef[toIndex(prog)], mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[toIndex(prog)], mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[toIndex(prog)], mDarkCoef));

	GLCHK(glVertexAttribPointer(mPositionHandle[toIndex(prog)],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[toIndex(prog)]));

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

	GLCHK(glVertexAttribPointer(mTexcoordHandle[toIndex(prog)],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords.data()));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[toIndex(prog)]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[toIndex(prog)]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[toIndex(prog)]));

	/* Horizontal blur pass */
	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = -1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = 1.f;
	vertices[7] = 1.f;
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[1]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mBlurWeights.data()));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[0],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0],
			  1.0f / static_cast<float>(mBlurFboWidth)));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Vertical blur pass */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mBlurFbo[0]));
	GLCHK(glViewport(0, 0, mBlurFboWidth, mBlurFboHeight));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mBlurWeights.data()));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mBlurFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[1],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1],
			  1.0f / static_cast<float>(mBlurFboHeight)));
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
	vertices[2] = 1.f;
	vertices[3] = videoW;
	vertices[4] = -videoH;
	vertices[5] = 1.f;
	vertices[6] = -videoW;
	vertices[7] = videoH;
	vertices[8] = 1.f;
	vertices[9] = videoW;
	vertices[10] = videoH;
	vertices[11] = 1.f;

	texCoords[0] = 0.f;
	texCoords[1] = 0.f;
	texCoords[2] = 1.f;
	texCoords[3] = 0.f;
	texCoords[4] = 0.f;
	texCoords[5] = 1.f;
	texCoords[6] = 1.f;
	texCoords[7] = 1.f;

	GLCHK(glVertexAttribPointer(mSimpleProgramPositionHandle,
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramPositionHandle));

	GLCHK(glVertexAttribPointer(mSimpleProgramTexcoordHandle,
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords.data()));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mSimpleProgramPositionHandle));
	GLCHK(glDisableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));
}


int GlVideo::setupPaddingFbo()
{
	GLenum gle;

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

	pdraw_gaussianDistribution(mPaddingBlurWeights.data(),
				   GL_VIDEO_BLUR_TAP_COUNT,
				   GL_VIDEO_BLURRED_PADDING_SIGMA);

	/* Allocate new resources */
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	for (unsigned int i = 0; i < 4; i++) {
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


void GlVideo::cleanupPaddingFbo()
{
	if (mPaddingFboTexture[0] > 0) {
		GLCHK(glDeleteTextures(4, mPaddingFboTexture.data()));
		mPaddingFboTexture.fill(0);
	}
	if (mPaddingFbo[0] > 0) {
		GLCHK(glDeleteFramebuffers(4, mPaddingFbo.data()));
		mPaddingFbo.fill(0);
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
	bool verticalMirror,
	bool immersive,
	const Eigen::Matrix4f &viewProjMat) const
{
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> stride{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoords{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoordsRatio{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxClamp{};
	std::array<float, 12> vertices{};
	std::array<float, 8> texCoords{};
	std::array<float, 9> yuv2RgbMatrix{};
	std::array<float, 3> yuv2RgbOffset{};
	bool mirrorTexture = verticalMirror;
	bool swapUv = false;

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

	Program prog;
	prog = getProgram(format, &swapUv);

	/* Pass 1 downscale */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[0]));
	GLCHK(glViewport(0, 0, mPaddingPass1Width, mPaddingPass1Height));

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));

	switch (prog) {
	default:
	case Program::GRAY_TO_RGB_PLANAR:
	case Program::GRAY16_TO_RGB_PLANAR:
	case Program::GRAY32_TO_RGB_PLANAR:
		mirrorTexture = !verticalMirror;
		/* Fall through */
	case Program::NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
				  mFirstTexUnit));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	case Program::YUV_TO_RGB_PLANAR:
	case Program::YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = !verticalMirror;
		for (unsigned int i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][i],
					  mFirstTexUnit + i));
			stride[2 * i] =
				1.f / static_cast<float>(framePlaneStride[i]);
			stride[2 * i + 1] = 1.f / static_cast<float>(height);
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				static_cast<float>(framePlaneStride[i] *
						   ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				static_cast<float>(info->resolution.height);
		}
		break;
	case Program::YUV_TO_RGB_SEMIPLANAR:
	case Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = !verticalMirror;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / static_cast<float>(framePlaneStride[1] / 2);
		stride[3] =
			1.f / static_cast<float>(info->resolution.height / 2);
		maxCoords[2] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[1]);
		maxCoords[3] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	}

	computeMaxCoordsRatioAndClamp(
		stride, maxCoords, maxCoordsRatio, maxClamp);

	GLCHK(glUniform2fv(mProgramStride[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   stride.data()));
	GLCHK(glUniform2fv(mProgramMaxCoordsRatio[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxCoordsRatio.data()));
	GLCHK(glUniform2fv(mProgramMaxClamp[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxClamp.data()));
	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mProgramYuv2RgbOffset[toIndex(prog)],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[toIndex(prog)],
				 1,
				 GL_FALSE,
				 yuv2RgbMatrix.data()));

	/* Disable overexposure zebras */
	updateZebra(nullptr, prog, false, 0.f);

	/* Disable MB status display */
	GLCHK(glUniform1f(mProgramMbStatusEnable[toIndex(prog)], 0.f));

	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = 1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = -1.f;
	vertices[7] = 1.f;
	vertices[8] = 1.f;
	vertices[9] = 1.f;
	vertices[10] = 1.f;
	vertices[11] = 1.f;

	Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
	GLCHK(glUniformMatrix4fv(
		mProgramTransformMatrix[toIndex(prog)], 1, false, id.data()));
	GLCHK(glUniform1f(mProgramBrightnessCoef[toIndex(prog)],
			  mBrightnessCoef));
	GLCHK(glUniform1f(mProgramContrastCoef[toIndex(prog)], mContrastCoef));
	GLCHK(glUniform1f(mProgramGammaCoef[toIndex(prog)], mGammaCoef));
	GLCHK(glUniform1f(mProgramSatCoef[toIndex(prog)], mSatCoef));
	GLCHK(glUniform1f(mProgramLightCoef[toIndex(prog)], mLightCoef));
	GLCHK(glUniform1f(mProgramDarkCoef[toIndex(prog)],
			  (immersive) ? mDarkCoef
				      : PDRAW_BLURRED_PADDING_DARK_COEF *
						mDarkCoef));

	GLCHK(glVertexAttribPointer(mPositionHandle[toIndex(prog)],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mPositionHandle[toIndex(prog)]));

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

	GLCHK(glVertexAttribPointer(mTexcoordHandle[toIndex(prog)],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords.data()));
	GLCHK(glEnableVertexAttribArray(mTexcoordHandle[toIndex(prog)]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mPositionHandle[toIndex(prog)]));
	GLCHK(glDisableVertexAttribArray(mTexcoordHandle[toIndex(prog)]));

	/* Pass 1 horizontal blur */
	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = -1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = 1.f;
	vertices[7] = 1.f;
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[1]));
	GLCHK(glViewport(0, 0, mPaddingPass1Width, mPaddingPass1Height));
	GLCHK(glUseProgram(mBlurProgram[0]));
	GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
			      GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[0]));
	GLCHK(glUniform1i(mBlurUniformSampler[0],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[0],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0],
			  1.0f / static_cast<float>(mPaddingPass1Width)));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights.data()));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 1 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[1]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[1],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1],
			  1.0f / static_cast<float>(mPaddingPass1Height)));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights.data()));
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
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[0],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[0]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[0],
			  1.0f / static_cast<float>(mPaddingPass2Width)));
	GLCHK(glUniform1fv(mBlurUniformWeights[0],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights.data()));
	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
	GLCHK(glDisableVertexAttribArray(mBlurPositionHandle[0]));

	/* Pass 2 vertical blur */
	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mPaddingFbo[2]));
	GLCHK(glViewport(0, 0, mPaddingPass2Width, mPaddingPass2Height));
	GLCHK(glUseProgram(mBlurProgram[1]));
	GLCHK(glBindTexture(GL_TEXTURE_2D, mPaddingFboTexture[3]));
	GLCHK(glUniform1i(mBlurUniformSampler[1],
			  mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT));
	GLCHK(glVertexAttribPointer(mBlurPositionHandle[1],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mBlurPositionHandle[1]));
	GLCHK(glUniform1f(mBlurUniformPixelSize[1],
			  1.0f / static_cast<float>(mPaddingPass2Height)));
	GLCHK(glUniform1fv(mBlurUniformWeights[1],
			   GL_VIDEO_BLUR_TAP_COUNT,
			   mPaddingBlurWeights.data()));
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
	vertices[2] = 1.f;
	vertices[3] = videoW2;
	vertices[4] = -videoH2;
	vertices[5] = 1.f;
	vertices[6] = -videoW2;
	vertices[7] = videoH2;
	vertices[8] = 1.f;
	vertices[9] = videoW2;
	vertices[10] = videoH2;
	vertices[11] = 1.f;

	if (mFillMode == PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND) {
		texCoords[0] = -(videoW2 / videoW / 2.f) + 0.5f;
		texCoords[1] = -(videoH2 / videoH / 2.f) + 0.5f;
		texCoords[2] = videoW2 / videoW / 2.f + 0.5f;
		texCoords[3] = -(videoH2 / videoH / 2.f) + 0.5f;
		texCoords[4] = -(videoW2 / videoW / 2.f) + 0.5f;
		texCoords[5] = videoH2 / videoH / 2.f + 0.5f;
		texCoords[6] = videoW2 / videoW / 2.f + 0.5f;
		texCoords[7] = videoH2 / videoH / 2.f + 0.5f;
	} else {
		texCoords[0] = 0.f;
		texCoords[1] = 0.f;
		texCoords[2] = 1.f;
		texCoords[3] = 0.f;
		texCoords[4] = 0.f;
		texCoords[5] = 1.f;
		texCoords[6] = 1.f;
		texCoords[7] = 1.f;
	}

	GLCHK(glVertexAttribPointer(mSimpleProgramPositionHandle,
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramPositionHandle));

	GLCHK(glVertexAttribPointer(mSimpleProgramTexcoordHandle,
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords.data()));
	GLCHK(glEnableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(mSimpleProgramPositionHandle));
	GLCHK(glDisableVertexAttribArray(mSimpleProgramTexcoordHandle));

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));
}


void GlVideo::setupZebra(Program prog) const
{
	float co = cosf(PDRAW_ZEBRA_ANGLE);
	float si = sinf(PDRAW_ZEBRA_ANGLE);
	const std::array<GLfloat, 4> zebra_mat = {co, si, -si, co};

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));
	GLCHK(glUniformMatrix2fv(
		glGetUniformLocation(mProgram[toIndex(prog)], "zebra_mat"),
		1,
		GL_FALSE,
		zebra_mat.data()));
	GLCHK(glUniform1fv(glGetUniformLocation(mProgram[toIndex(prog)],
						"zebra_avg_weights"),
			   9,
			   zebraAvgWeights.data()));
}


void GlVideo::updateZebra(const struct pdraw_rect *contentPos,
			  Program prog,
			  bool enable,
			  float threshold) const
{
	GLCHK(glUniform1f(mProgramZebraEnable[toIndex(prog)],
			  enable ? 1.f : 0.f));
	GLCHK(glUniform1f(mProgramZebraThreshold[toIndex(prog)], threshold));

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
		auto zebra_period_us =
			(uint64_t)(1000000.f / PDRAW_ZEBRA_FREQUENCY_HZ);
		float zebra_phase =
			static_cast<float>(time_us % zebra_period_us) /
			static_cast<float>(zebra_period_us);
		GLCHK(glUniform1f(mProgramZebraPhase[toIndex(prog)],
				  zebra_phase));
		float zebra_weight = PDRAW_ZEBRA_WEIGHT *
				     static_cast<float>(contentPos->width) /
				     1920.f;
		GLCHK(glUniform1f(mProgramZebraWeight[toIndex(prog)],
				  zebra_weight));
	}
}


int GlVideo::setupHistograms()
{
	int ret = 0;
	GLenum gle;
	GLint success = 0;
	GLint vertexShaderHistogram = 0;
	std::array<GLint, PROGRAM_MAX> fragmentShaderHistogram{};
	unsigned int i;

	/* Buffers allocation */
	mHistogramBuffer.resize(4 * GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
				GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE);
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
		std::array<char, 512> infoLog{};
		glGetShaderInfoLog(
			vertexShaderHistogram, 512, nullptr, infoLog.data());
		ULOGE("vertex shader compilation failed '%s'", infoLog.data());
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
			       histogramFragmentShaders[i].data(),
			       nullptr);
		glCompileShader(fragmentShaderHistogram[i]);
		glGetShaderiv(fragmentShaderHistogram[i],
			      GL_COMPILE_STATUS,
			      &success);
		if (!success) {
			std::array<char, 512> infoLog{};
			glGetShaderInfoLog(fragmentShaderHistogram[i],
					   512,
					   nullptr,
					   infoLog.data());
			ULOGE("fragment shader compilation failed '%s'",
			      infoLog.data());
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
			std::array<char, 512> infoLog{};
			glGetProgramInfoLog(mHistogramProgram[i],
					    512,
					    nullptr,
					    infoLog.data());
			ULOGE("program link failed '%s'", infoLog.data());
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
		mHistogramMaxCoordsRatio[i] = glGetUniformLocation(
			mHistogramProgram[i], "max_coords_ratio");
		mHistogramMaxClamp[i] =
			glGetUniformLocation(mHistogramProgram[i], "max_clamp");
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


void GlVideo::cleanupHistograms()
{
	if (mHistogramFboTexture > 0) {
		GLCHK(glDeleteTextures(1, &mHistogramFboTexture));
		mHistogramFboTexture = 0;
	}
	if (mHistogramFbo > 0) {
		GLCHK(glDeleteFramebuffers(1, &mHistogramFbo));
		mHistogramFbo = 0;
	}
	for (unsigned int i = 0; i < PROGRAM_MAX; i++) {
		if (mHistogramProgram[i] > 0) {
			GLCHK(glDeleteProgram(mHistogramProgram[i]));
			mHistogramProgram[i] = 0;
		}
	}
	mHistogramBuffer.clear();
	mHistogramInit = false;
}


void GlVideo::computeHistograms(
	const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
	const struct vdef_raw_format *format,
	const struct vdef_frame_info *info,
	const struct vdef_rect *crop,
	const struct pdraw_rect *renderPos,
	bool verticalMirror,
	bool enable)
{
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> stride{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoords{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoordsRatio{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxClamp{};
	std::array<float, 12> vertices{};
	std::array<float, 8> texCoords{};
	std::array<float, 9> yuv2RgbMatrix{};
	std::array<float, 3> yuv2RgbOffset{};
	unsigned int i;
	unsigned int j;
	const uint8_t *buf;
	std::array<uint32_t, PDRAW_HISTOGRAM_CHANNEL_MAX> histoMax{};
	struct timespec ts;
	uint64_t time_us;
	bool mirrorTexture = verticalMirror;
	bool swapUv = false;

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

	Program prog;
	prog = getProgram(format, &swapUv);

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mHistogramFbo));
	GLCHK(glDisable(GL_BLEND));
	GLCHK(glViewport(0,
			 0,
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE));
	GLCHK(glUseProgram(mHistogramProgram[toIndex(prog)]));

	switch (prog) {
	default:
	case Program::GRAY_TO_RGB_PLANAR:
	case Program::GRAY16_TO_RGB_PLANAR:
	case Program::GRAY32_TO_RGB_PLANAR:
		mirrorTexture = !verticalMirror;
		/* Fall through */
	case Program::NOCONV:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D,
				    (mExtTexture > 0) ? mExtTexture
						      : mTextures[0]));
		GLCHK(glUniform1i(mHistogramUniformSampler[toIndex(prog)][0],
				  mFirstTexUnit));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	case Program::YUV_TO_RGB_PLANAR:
	case Program::YUV_TO_RGB_PLANAR_10_16LE:
		mirrorTexture = !verticalMirror;
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			GLCHK(glUniform1i(
				mHistogramUniformSampler[toIndex(prog)][i],
				mFirstTexUnit + i));
			stride[2 * i] =
				1.f / static_cast<float>(framePlaneStride[i]);
			stride[2 * i + 1] = 1.f / static_cast<float>(height);
			maxCoords[2 * i] =
				(float)(crop->left + crop->width) /
				static_cast<float>(framePlaneStride[i] *
						   ((i > 0) ? 2 : 1));
			maxCoords[2 * i + 1] =
				(float)(crop->top + crop->height) /
				static_cast<float>(info->resolution.height);
		}
		break;
	case Program::YUV_TO_RGB_SEMIPLANAR:
	case Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		mirrorTexture = !verticalMirror;
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		GLCHK(glUniform1i(mHistogramUniformSampler[toIndex(prog)][0],
				  mFirstTexUnit + 0));
		stride[0] = 1.f / static_cast<float>(framePlaneStride[0]);
		stride[1] = 1.f / static_cast<float>(info->resolution.height);
		maxCoords[0] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[0]);
		maxCoords[1] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);

		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
		GLCHK(glUniform1i(mHistogramUniformSampler[toIndex(prog)][1],
				  mFirstTexUnit + 1));
		stride[2] = 1.f / static_cast<float>(framePlaneStride[1] / 2);
		stride[3] =
			1.f / static_cast<float>(info->resolution.height / 2);
		maxCoords[2] = (float)(crop->left + crop->width) /
			       static_cast<float>(framePlaneStride[1]);
		maxCoords[3] = (float)(crop->top + crop->height) /
			       static_cast<float>(info->resolution.height);
		break;
	}

	computeMaxCoordsRatioAndClamp(
		stride, maxCoords, maxCoordsRatio, maxClamp);

	GLCHK(glUniform2fv(mHistogramStride[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   stride.data()));
	GLCHK(glUniform2fv(mHistogramMaxCoordsRatio[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxCoordsRatio.data()));
	GLCHK(glUniform2fv(mHistogramMaxClamp[toIndex(prog)],
			   GL_VIDEO_TEX_UNIT_COUNT,
			   maxClamp.data()));

	fillYuv2RgbMatrix(info->matrix_coefs,
			  info->full_range,
			  swapUv,
			  yuv2RgbMatrix,
			  yuv2RgbOffset);
	GLCHK(glUniform3f(mHistogramYuv2RgbOffset[toIndex(prog)],
			  yuv2RgbOffset[0],
			  yuv2RgbOffset[1],
			  yuv2RgbOffset[2]));
	GLCHK(glUniformMatrix3fv(mHistogramYuv2RgbMatrix[toIndex(prog)],
				 1,
				 GL_FALSE,
				 yuv2RgbMatrix.data()));
	GLCHK(glUniform3f(
		mHistogramRgb2LumaMatrix[toIndex(prog)],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][0],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][3],
		vdef_rgb_to_yuv_norm_matrix[info->matrix_coefs][1][6]));
	GLCHK(glUniform1f(
		mHistogramRgb2LumaOffset[toIndex(prog)],
		vdef_rgb_to_yuv_norm_offset[info->matrix_coefs][1][0]));
	GLCHK(glUniform1f(mHistogramBrightnessCoef[toIndex(prog)],
			  mBrightnessCoef));
	GLCHK(glUniform1f(mHistogramContrastCoef[toIndex(prog)],
			  mContrastCoef));
	GLCHK(glUniform1f(mHistogramGammaCoef[toIndex(prog)], mGammaCoef));

	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = 1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = -1.f;
	vertices[7] = 1.f;
	vertices[8] = 1.f;
	vertices[9] = 1.f;
	vertices[10] = 1.f;
	vertices[11] = 1.f;

	GLCHK(glVertexAttribPointer(mHistogramPositionHandle[toIndex(prog)],
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
	GLCHK(glEnableVertexAttribArray(
		mHistogramPositionHandle[toIndex(prog)]));

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

	GLCHK(glVertexAttribPointer(mHistogramTexcoordHandle[toIndex(prog)],
				    2,
				    GL_FLOAT,
				    false,
				    0,
				    texCoords.data()));
	GLCHK(glEnableVertexAttribArray(
		mHistogramTexcoordHandle[toIndex(prog)]));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

	GLCHK(glDisableVertexAttribArray(
		mHistogramPositionHandle[toIndex(prog)]));
	GLCHK(glDisableVertexAttribArray(
		mHistogramTexcoordHandle[toIndex(prog)]));

	GLCHK(glFinish());

	/* Read pixels to CPU buffer */
	GLCHK(glReadPixels(0,
			   0,
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE,
			   GL_RGBA,
			   GL_UNSIGNED_BYTE,
			   mHistogramBuffer.data()));

	GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, mDefaultFbo));
	GLCHK(glViewport(renderPos->x,
			 renderPos->y,
			 renderPos->width,
			 renderPos->height));

	/* Reset the histograms */
	for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
		mHistogram[i].fill(0);

	/* Count the values */
	for (j = 0, buf = mHistogramBuffer.data();
	     j < GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE *
			 GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE;
	     j++) {
		for (i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
			mHistogram[i][*buf++]++;
	}

	/* Histograms normalization */
	histoMax.fill(0);
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
			mHistogramNorm[i].fill(0.f);
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
	for (unsigned int i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++) {
		if (mHistogramValid[i]) {
			histogram[i] =
				const_cast<float *>(mHistogramNorm[i].data());
			histogramLen[i] = 256;
		}
	}
}


void GlVideo::startTransition(GlVideoTransition transition,
			      uint64_t duration,
			      bool hold)
{
	if (mTransition != GlVideoTransition::NONE)
		abortTransition();
	mTransition = transition;
	mTransitionDuration = duration;
	mTransitionHold = hold;
}


void GlVideo::abortTransition()
{
	mTransition = GlVideoTransition::NONE;
	mTransitionDuration = 0;
	mTransitionStartTime = 0;
	mTransitionHold = false;
}


void GlVideo::updateTransition()
{
	int res;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	float progress;
	float blurSigma;

	mApplyBlur = false;
	mSatCoef = mBaseSatCoef;
	mLightCoef = mBaseLightCoef;
	mDarkCoef = mBaseDarkCoef;

	if (mTransition == GlVideoTransition::NONE)
		return;

	res = time_get_monotonic(&ts);
	if (res < 0)
		ULOG_ERRNO("time_get_monotonic", -res);
	res = time_timespec_to_us(&ts, &curTime);
	if (res < 0)
		ULOG_ERRNO("time_timespec_to_us", -res);
	if (mTransitionStartTime == 0)
		mTransitionStartTime = curTime;

	progress = static_cast<float>(curTime - mTransitionStartTime) /
		   static_cast<float>(mTransitionDuration);
	if (progress > 1.0f) {
		progress = 1.0f;
		if (!mTransitionHold) {
			/* Transition finished */
			abortTransition();
			return;
		}
	}

	switch (mTransition) {
	case GlVideoTransition::FADE_TO_BLACK:
		mDarkCoef = mBaseDarkCoef * (1.f - progress);
		break;
	case GlVideoTransition::FADE_FROM_BLACK:
		mDarkCoef = mBaseDarkCoef * progress;
		break;
	case GlVideoTransition::FADE_TO_WHITE:
		mLightCoef = mBaseLightCoef * (1.f - progress);
		break;
	case GlVideoTransition::FADE_FROM_WHITE:
		mLightCoef = mBaseLightCoef * progress;
		break;
	case GlVideoTransition::FADE_TO_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * (1.f - progress);
		break;
	case GlVideoTransition::FADE_FROM_BLACK_AND_WHITE:
		mSatCoef = mBaseSatCoef * progress;
		break;
	case GlVideoTransition::FADE_TO_BLUR:
		blurSigma = progress * (GL_VIDEO_BLUR_MAX_SIGMA -
					GL_VIDEO_BLUR_MIN_SIGMA) +
			    GL_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(mBlurWeights.data(),
					   GL_VIDEO_BLUR_TAP_COUNT,
					   blurSigma);
		mApplyBlur = mBlurInit;
		break;
	case GlVideoTransition::FADE_FROM_BLUR:
		blurSigma = (1.f - progress) * (GL_VIDEO_BLUR_MAX_SIGMA -
						GL_VIDEO_BLUR_MIN_SIGMA) +
			    GL_VIDEO_BLUR_MIN_SIGMA;
		pdraw_gaussianDistribution(mBlurWeights.data(),
					   GL_VIDEO_BLUR_TAP_COUNT,
					   blurSigma);
		mApplyBlur = mBlurInit;
		break;
	case GlVideoTransition::FLASH:
		mLightCoef = mBaseLightCoef *
			     (powf(progress, GL_VIDEO_FLASH_GAMMA_COEF) *
				      GL_VIDEO_FLASH_LIGHT_COEF +
			      1.f - GL_VIDEO_FLASH_LIGHT_COEF);
		mSatCoef = mBaseSatCoef *
			   powf(progress, GL_VIDEO_FLASH_GAMMA_COEF);
		break;
	default:
		ULOGE("unsupported transition type: %u",
		      static_cast<unsigned int>(mTransition));
		break;
	}
}


int GlVideo::loadFrame(const uint8_t *framePlanes[VDEF_RAW_MAX_PLANE_COUNT],
		       const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
		       const struct vdef_raw_format *format,
		       const struct vdef_frame_info *info,
		       const uint8_t *mbStatus)
{
	unsigned int i;
	unsigned int align;
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
	std::array<size_t, VDEF_RAW_MAX_PLANE_COUNT> _framePlaneStride = {};
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

	Program prog;
	prog = getProgram(format, &swapUv);

	GLint savedAlign = 0;
	GLCHK(glGetIntegerv(GL_UNPACK_ALIGNMENT, &savedAlign));

	GLCHK(glUseProgram(mProgram[toIndex(prog)]));

	switch (prog) {
	default:
	case Program::NOCONV:
		break;
	case Program::YUV_TO_RGB_PLANAR:
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			align = getTextureMaxUnpackAlignment(
				static_cast<unsigned int>(
					_framePlaneStride[i]));
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
	case Program::YUV_TO_RGB_PLANAR_10_16LE:
		for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
			int height =
				info->resolution.height / ((i > 0) ? 2 : 1);
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + i));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[i]));
			align = getTextureMaxUnpackAlignment(
				static_cast<unsigned int>(_framePlaneStride[i] *
							  2));
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
	case Program::YUV_TO_RGB_SEMIPLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[0]));
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
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[i]));
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
	case Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[0] * 2));
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
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[1] * 2));
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
	case Program::GRAY_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[0]));
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
	case Program::GRAY16_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[0] * 2));
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
	case Program::GRAY32_TO_RGB_PLANAR:
		GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
		GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
		align = getTextureMaxUnpackAlignment(
			static_cast<unsigned int>(_framePlaneStride[0] * 4));
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
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> stride{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoords{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxCoordsRatio{};
	std::array<float, GL_VIDEO_TEX_UNIT_COUNT * 2> maxClamp{};
	std::array<float, 12> vertices{};
	std::array<float, 8> texCoords{};
	std::array<float, 9> yuv2RgbMatrix{};
	std::array<float, 3> yuv2RgbOffset{};
	bool mirrorTexture = params->vertical_mirror;
	bool swapUv = false;
	float videoAR;
	GLboolean glBlendEnabled;
	GLboolean glDepthTestEnabled;

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
	std::array<size_t, VDEF_RAW_MAX_PLANE_COUNT> _framePlaneStride = {};
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

	Program prog;
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

	computeHistograms(_framePlaneStride.data(),
			  format,
			  &_info,
			  crop,
			  renderPos,
			  params->vertical_mirror,
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

	float windowW = 1.f;
	float windowH = windowAR;
	float ratioW = 1.f;
	float ratioH = 1.f;
	float ratioW2 = 1.f;
	float ratioH2 = 1.f;
	switch (params->fill_mode) {
	default:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP:
	case PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND:
		/* Maintain video aspect ratio without crop and add borders if
		 * window aspect ratio and video aspect ratio differ */
		if (videoAR >= windowAR) {
			ratioW = 1.f;
			ratioH = windowAR / videoAR;
			ratioW2 = videoAR / windowAR;
			ratioH2 = 1.f;
		} else {
			ratioW = videoAR / windowAR;
			ratioH = 1.f;
			ratioW2 = 1.f;
			ratioH2 = windowAR / videoAR;
		}
		break;
	case PDRAW_VIDEO_RENDERER_FILL_MODE_CROP:
		/* Maintain video aspect ratio without borders and add video
		 * crop if window aspect ratio and video aspect ratio differ */
		if (videoAR >= windowAR) {
			ratioW = videoAR / windowAR;
			ratioH = 1.f;
		} else {
			ratioW = 1.f;
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
		contentPos->width = static_cast<unsigned int>(
			ratioW * static_cast<float>(renderPos->width));
		contentPos->height = static_cast<unsigned int>(
			ratioH * static_cast<float>(renderPos->height));

		dw = (int32_t)renderPos->width - (int32_t)contentPos->width;
		dh = (int32_t)renderPos->height - (int32_t)contentPos->height;
		contentPos->x = dw / 2;
		contentPos->y = dh / 2;
	}

	if (videoAR != windowAR) {
		renderPadding(_framePlaneStride.data(),
			      format,
			      &_info,
			      crop,
			      renderPos,
			      videoW,
			      videoH,
			      videoW2,
			      videoH2,
			      params->vertical_mirror,
			      false,
			      viewProjMat);
	}

	if (mApplyBlur) {
		renderBlur(_framePlaneStride.data(),
			   format,
			   &_info,
			   crop,
			   renderPos,
			   videoW,
			   videoH,
			   params->vertical_mirror,
			   viewProjMat);
	} else {
		GLCHK(glUseProgram(mProgram[toIndex(prog)]));

		switch (prog) {
		default:
		case Program::GRAY_TO_RGB_PLANAR:
		case Program::GRAY16_TO_RGB_PLANAR:
		case Program::GRAY32_TO_RGB_PLANAR:
			mirrorTexture = !params->vertical_mirror;
			/* Fall through */
		case Program::NOCONV:
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit));
			GLCHK(glBindTexture(GL_TEXTURE_2D,
					    (mExtTexture > 0) ? mExtTexture
							      : mTextures[0]));
			GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
					  mFirstTexUnit));
			stride[0] =
				1.f / static_cast<float>(_framePlaneStride[0]);
			stride[1] = 1.f /
				    static_cast<float>(_info.resolution.height);
			maxCoords[0] = (float)(crop->left + crop->width) /
				       static_cast<float>(_framePlaneStride[0]);
			maxCoords[1] =
				(float)(crop->top + crop->height) /
				static_cast<float>(_info.resolution.height);
			break;
		case Program::YUV_TO_RGB_PLANAR:
		case Program::YUV_TO_RGB_PLANAR_10_16LE:
			mirrorTexture = !params->vertical_mirror;
			for (i = 0; i < GL_VIDEO_TEX_UNIT_COUNT; i++) {
				int height = _info.resolution.height /
					     ((i > 0) ? 2 : 1);
				GLCHK(glActiveTexture(GL_TEXTURE0 +
						      mFirstTexUnit + i));
				GLCHK(glBindTexture(GL_TEXTURE_2D,
						    mTextures[i]));
				GLCHK(glUniform1i(
					mUniformSamplers[toIndex(prog)][i],
					mFirstTexUnit + i));
				stride[2 * i] =
					1.f / static_cast<float>(
						      _framePlaneStride[i]);
				stride[2 * i + 1] =
					1.f / static_cast<float>(height);
				maxCoords[2 * i] =
					(float)(crop->left + crop->width) /
					static_cast<float>(
						_framePlaneStride[i] *
						((i > 0) ? 2 : 1));
				maxCoords[2 * i + 1] =
					(float)(crop->top + crop->height) /
					static_cast<float>(
						_info.resolution.height);
			}
			break;
		case Program::YUV_TO_RGB_SEMIPLANAR:
		case Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH:
			mirrorTexture = !params->vertical_mirror;
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 0));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[0]));
			GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][0],
					  mFirstTexUnit + 0));
			stride[0] =
				1.f / static_cast<float>(_framePlaneStride[0]);
			stride[1] = 1.f /
				    static_cast<float>(_info.resolution.height);
			maxCoords[0] = (float)(crop->left + crop->width) /
				       static_cast<float>(_framePlaneStride[0]);
			maxCoords[1] =
				(float)(crop->top + crop->height) /
				static_cast<float>(_info.resolution.height);

			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit + 1));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mTextures[1]));
			GLCHK(glUniform1i(mUniformSamplers[toIndex(prog)][1],
					  mFirstTexUnit + 1));
			stride[2] = 1.f / static_cast<float>(
						  _framePlaneStride[1] / 2);
			stride[3] = 1.f / static_cast<float>(
						  _info.resolution.height / 2);
			maxCoords[2] = (float)(crop->left + crop->width) /
				       static_cast<float>(_framePlaneStride[1]);
			maxCoords[3] =
				(float)(crop->top + crop->height) /
				static_cast<float>(_info.resolution.height);
			break;
		}

		computeMaxCoordsRatioAndClamp(
			stride, maxCoords, maxCoordsRatio, maxClamp);

		GLCHK(glUniform2fv(mProgramStride[toIndex(prog)],
				   GL_VIDEO_TEX_UNIT_COUNT,
				   stride.data()));
		GLCHK(glUniform2fv(mProgramMaxCoordsRatio[toIndex(prog)],
				   GL_VIDEO_TEX_UNIT_COUNT,
				   maxCoordsRatio.data()));
		GLCHK(glUniform2fv(mProgramMaxClamp[toIndex(prog)],
				   GL_VIDEO_TEX_UNIT_COUNT,
				   maxClamp.data()));
		fillYuv2RgbMatrix(_info.matrix_coefs,
				  _info.full_range,
				  swapUv,
				  yuv2RgbMatrix,
				  yuv2RgbOffset);
		GLCHK(glUniform3f(mProgramYuv2RgbOffset[toIndex(prog)],
				  yuv2RgbOffset[0],
				  yuv2RgbOffset[1],
				  yuv2RgbOffset[2]));
		GLCHK(glUniformMatrix3fv(mProgramYuv2RgbMatrix[toIndex(prog)],
					 1,
					 GL_FALSE,
					 yuv2RgbMatrix.data()));

		/* Update overexposure zebras */
		updateZebra(contentPos,
			    prog,
			    params->enable_overexposure_zebras,
			    params->overexposure_zebras_threshold);

		/* MB status display */
		GLCHK(glUniform1f(mProgramMbStatusEnable[toIndex(prog)],
				  mHasMbStatus ? 1.f : 0.f));
		if (mHasMbStatus) {
			GLCHK(glActiveTexture(GL_TEXTURE0 + mFirstTexUnit +
					      GL_VIDEO_TEX_UNIT_COUNT +
					      GL_VIDEO_FBO_TEX_UNIT_COUNT));
			GLCHK(glBindTexture(GL_TEXTURE_2D, mMbStatusTexture));
			GLCHK(glUniform1i(
				mMbStatusUniformSampler[toIndex(prog)],
				mFirstTexUnit + GL_VIDEO_TEX_UNIT_COUNT +
					GL_VIDEO_FBO_TEX_UNIT_COUNT));
		}

		GLCHK(glUniformMatrix4fv(mProgramTransformMatrix[toIndex(prog)],
					 1,
					 false,
					 viewProjMat.data()));
		GLCHK(glUniform1f(mProgramBrightnessCoef[toIndex(prog)],
				  mBrightnessCoef));
		GLCHK(glUniform1f(mProgramContrastCoef[toIndex(prog)],
				  mContrastCoef));
		GLCHK(glUniform1f(mProgramGammaCoef[toIndex(prog)],
				  mGammaCoef));
		GLCHK(glUniform1f(mProgramSatCoef[toIndex(prog)], mSatCoef));
		GLCHK(glUniform1f(mProgramLightCoef[toIndex(prog)],
				  mLightCoef));
		GLCHK(glUniform1f(mProgramDarkCoef[toIndex(prog)], mDarkCoef));

		vertices[0] = -videoW;
		vertices[1] = -videoH;
		vertices[2] = 1.f;
		vertices[3] = videoW;
		vertices[4] = -videoH;
		vertices[5] = 1.f;
		vertices[6] = -videoW;
		vertices[7] = videoH;
		vertices[8] = 1.f;
		vertices[9] = videoW;
		vertices[10] = videoH;
		vertices[11] = 1.f;

		GLCHK(glVertexAttribPointer(mPositionHandle[toIndex(prog)],
					    3,
					    GL_FLOAT,
					    false,
					    0,
					    vertices.data()));
		GLCHK(glEnableVertexAttribArray(
			mPositionHandle[toIndex(prog)]));

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

		GLCHK(glVertexAttribPointer(mTexcoordHandle[toIndex(prog)],
					    2,
					    GL_FLOAT,
					    false,
					    0,
					    texCoords.data()));
		GLCHK(glEnableVertexAttribArray(
			mTexcoordHandle[toIndex(prog)]));

		GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

		GLCHK(glDisableVertexAttribArray(
			mPositionHandle[toIndex(prog)]));
		GLCHK(glDisableVertexAttribArray(
			mTexcoordHandle[toIndex(prog)]));
	}

	if (glBlendEnabled)
		GLCHK(glEnable(GL_BLEND));
	if (glDepthTestEnabled)
		GLCHK(glEnable(GL_DEPTH_TEST));

	return 0;
}


int GlVideo::clear(const Eigen::Matrix4f &viewProjMat) const
{
	std::array<float, 12> vertices{};

	GLCHK(glUseProgram(mClearProgram));

	GLCHK(glUniformMatrix4fv(
		mClearProgramTransformMatrix, 1, false, viewProjMat.data()));
	GLCHK(glUniform3f(mClearProgramColor, 0.f, 0.f, 0.f));

	vertices[0] = -1.f;
	vertices[1] = -1.f;
	vertices[2] = 1.f;
	vertices[3] = 1.f;
	vertices[4] = -1.f;
	vertices[5] = 1.f;
	vertices[6] = -1.f;
	vertices[7] = 1.f;
	vertices[8] = 1.f;
	vertices[9] = 1.f;
	vertices[10] = 1.f;
	vertices[11] = 1.f;

	GLCHK(glVertexAttribPointer(mClearProgramPositionHandle,
				    3,
				    GL_FLOAT,
				    false,
				    0,
				    vertices.data()));
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
