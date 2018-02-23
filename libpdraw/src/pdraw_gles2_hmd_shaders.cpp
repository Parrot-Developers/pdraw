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

#ifdef USE_GLES2

#include "pdraw_gles2_common.hpp"

namespace Pdraw {

const GLchar *pdraw_gles2HmdVertexShader =
#if defined(GL_ES_VERSION_2_0) && (defined(ANDROID) || defined(__APPLE__))
	"precision highp float;\n"
#endif
	"uniform vec2 EyeToSourceUVScale;\n"
	"uniform vec2 EyeToSourceUVOffset;\n"
	"uniform vec2 EyeToSourceScale;\n"
	"uniform vec2 EyeToSourceOffset;\n"
	"uniform int ChromaticAberrationCorrection;\n"
	"uniform int Rotation;\n"
	"\n"
	"attribute vec2 Position;\n"
	"attribute vec4 Color;\n"
	"\n"
	"attribute vec2 TexCoord0;\n"
	"attribute vec2 TexCoord1;\n"
	"attribute vec2 TexCoord2;\n"
	"\n"
	"varying vec2 oTexCoord0;\n"
	"varying vec2 oTexCoord1;\n"
	"varying vec2 oTexCoord2;\n"
	"\n"
	"varying vec4 oColor;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position.x = Position.x * EyeToSourceScale.x "
		"+ EyeToSourceOffset.x;\n"
	"    gl_Position.y = Position.y * EyeToSourceScale.y + "
		"EyeToSourceOffset.y;\n"
	"    gl_Position.z = 0.5;\n"
	"    gl_Position.w = 1.0;\n"
	"\n"
	"    float red_color_distorsion = 0.0;\n"
	"    float green_color_distorsion = 0.0;\n"
	"    float blue_color_distorsion = 0.0;\n"
	"\n"
	"    if (ChromaticAberrationCorrection == 1)\n"
	"    {\n"
	"        red_color_distorsion = -0.006;\n"
	"        blue_color_distorsion = 0.009;\n"
	"    }\n"
	"\n"
	"    /* Vertex inputs are in TanEyeAngle space for the R,G,B channels\n"
	"     * (i.e. after chromatic aberration and distortion).\n"
	"     * Scale them into the correct [0-1],[0-1] UV lookup space\n"
	"     * (depending on eye) */\n"
	"    vec2 RotatedTexCoord0 = TexCoord0;\n"
	"    vec2 RotatedTexCoord1 = TexCoord1;\n"
	"    vec2 RotatedTexCoord2 = TexCoord2;\n"
	"\n"
	"    if (Rotation == 90)\n"
	"    {\n"
	"        RotatedTexCoord0 = vec2(TexCoord0.y, 1.0 - TexCoord0.x);\n"
	"        RotatedTexCoord1 = vec2(TexCoord1.y, 1.0 - TexCoord1.x);\n"
	"        RotatedTexCoord2 = vec2(TexCoord2.y, 1.0 - TexCoord2.x);\n"
	"    }\n"
	"    else if (Rotation == 180)\n"
	"    {\n"
	"        RotatedTexCoord0 = vec2(1.0 - TexCoord0.x, "
		"1.0 - TexCoord0.y);\n"
	"        RotatedTexCoord1 = vec2(1.0 - TexCoord1.x, "
		"1.0 - TexCoord1.y);\n"
	"        RotatedTexCoord2 = vec2(1.0 - TexCoord2.x, "
		"1.0 - TexCoord2.y);\n"
	"    }\n"
	"    else if (Rotation == 270)\n"
	"    {\n"
	"        RotatedTexCoord0 = vec2(1.0 - TexCoord0.y, TexCoord0.x);\n"
	"        RotatedTexCoord1 = vec2(1.0 - TexCoord1.y, TexCoord1.x);\n"
	"        RotatedTexCoord2 = vec2(1.0 - TexCoord2.y, TexCoord2.x);\n"
	"    }\n"
	"\n"
	"    oTexCoord0 = ((RotatedTexCoord0 - vec2(0.5,0.5)) * "
		"EyeToSourceUVScale * (1.0 + red_color_distorsion)) + "
		"vec2(0.5,0.5) + EyeToSourceUVOffset;\n"
	"    oTexCoord1 = ((RotatedTexCoord1 - vec2(0.5,0.5)) * "
		"EyeToSourceUVScale * (1.0 + green_color_distorsion)) + "
		"vec2(0.5,0.5) + EyeToSourceUVOffset;\n"
	"    oTexCoord2 = ((RotatedTexCoord2 - vec2(0.5,0.5)) * "
		"EyeToSourceUVScale * (1.0 + blue_color_distorsion)) + "
		"vec2(0.5,0.5) + EyeToSourceUVOffset;\n"
	"\n"
	"    oColor = Color; // Used for vignette fade.\n"
	"}\n";


const GLchar *pdraw_gles2HmdFragmentShader =
#if defined(GL_ES_VERSION_2_0) && (defined(ANDROID) || defined(__APPLE__))
	"precision highp float;\n"
	"uniform sampler2D Texture0;\n"
#elif 0 && defined(GL_ES_VERSION_2_0) && defined(ANDROID)
	"/* important to include in order to use rendered\n"
	" * Android View to gl texture */\n"
	"#extension GL_OES_EGL_image_external : require\n"
	"precision highp float;\n"
	"/* make sure to use samplerExternalOES instead of sampler2D */\n"
	"uniform samplerExternalOES Texture0; /* the input texture */\n"
#else
	"uniform sampler2D Texture0;\n"
#endif
	"\n"
	"uniform int LensLimits;\n"
	"\n"
	"varying vec4 oColor;\n"
	"varying vec2 oTexCoord0;\n"
	"varying vec2 oTexCoord1;\n"
	"varying vec2 oTexCoord2;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    float ResultA = 0.0;\n"
	"\n"
	"    float ResultR;\n"
	"\n"
	"    if (oTexCoord0.x > 1.0 || oTexCoord0.y > 1.0 || "
		"oTexCoord0.x < 0.0 || oTexCoord0.y < 0.0)\n"
	"    {\n"
	"        ResultR = (LensLimits == 1 ? 0.1 : 0.0);\n"
	"    }\n"
	"    else\n"
	"    {\n"
	"        ResultR = texture2D(Texture0, oTexCoord0).r;\n"
	"        ResultA = texture2D(Texture0, oTexCoord1).a;\n"
	"    }\n"
	"\n"
	"    float ResultG;\n"
	"    if (oTexCoord1.x > 1.0 || oTexCoord1.y > 1.0 || "
		"oTexCoord1.x < 0.0 || oTexCoord1.y < 0.0)\n"
	"    {\n"
	"        ResultG = (LensLimits == 1 ? 0.2 : 0.0);\n"
	"    }\n"
	"    else\n"
	"    {\n"
	"        ResultG = texture2D(Texture0, oTexCoord1).g;\n"
	"    }\n"
	"\n"
	"    float ResultB;\n"
	"    if (oTexCoord2.x > 1.0 || oTexCoord2.y > 1.0 || "
		"oTexCoord2.x < 0.0 || oTexCoord2.y < 0.0)\n"
	"    {\n"
	"        ResultB = (LensLimits == 1 ? 0.3 : 0.0);\n"
	"    }\n"
	"    else\n"
	"    {\n"
	"        ResultB = texture2D(Texture0, oTexCoord2).b;\n"
	"    }\n"
	"\n"
	"    gl_FragColor = vec4(ResultR * oColor.r, ResultG * oColor.g, "
		"ResultB * oColor.b, ResultA);\n"
	"}\n";

} /* namespace Pdraw */

#endif /* USE_GLES2 */
