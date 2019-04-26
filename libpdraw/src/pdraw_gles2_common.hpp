/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 common header
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

#ifndef _PDRAW_GLES2_COMMON_HPP_
#define _PDRAW_GLES2_COMMON_HPP_

#ifdef USE_GLES2

#	if defined(BCM_VIDEOCORE) || defined(ANDROID_NDK)
#		include <GLES2/gl2.h>
#	elif defined(__APPLE__)
#		include <TargetConditionals.h>
#		if TARGET_OS_IPHONE
#			include <OpenGLES/ES2/gl.h>
#			include <OpenGLES/ES2/glext.h>
#		else
#			include <GLFW/glfw3.h>
#			include <OpenGL/OpenGL.h>
#			include <OpenGL/glext.h>
#		endif
#	else
#		define GLFW_INCLUDE_ES2
#		include <GLFW/glfw3.h>
#	endif

/* Uncomment to enable extra GL error checking */
//#	define CHECK_GL_ERRORS
#	if defined(CHECK_GL_ERRORS)
#		warning CHECK_GL_ERRORS is enabled
#		include <assert.h>
#		define GLCHK(X)                                               \
			do {                                                   \
				GLenum err = GL_NO_ERROR;                      \
				X;                                             \
				while ((err = glGetError())) {                 \
					ULOGE("GL error 0x%x in " #X           \
					      " file %s line %d",              \
					      err,                             \
					      __FILE__,                        \
					      __LINE__);                       \
					assert(err == GL_NO_ERROR);            \
				}                                              \
			} while (0)
#	else
#		define GLCHK(X) X
#	endif /* CHECK_GL_ERRORS */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_COMMON_HPP_ */
