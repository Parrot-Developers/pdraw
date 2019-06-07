/**
 * Parrot Drones Awesome Video Viewer
 * OpenGL ES 2.0 HUD rendering library
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

#include "pdraw_gles2hud_priv.h"


static const GLchar *hud_vertex_shader =
	"uniform mat4 transform_matrix;\n"
	"attribute vec4 vPosition;\n"
	"void main() {\n"
	"    gl_Position = transform_matrix * vPosition;\n"
	"}\n";

static const GLchar *hud_fragment_shader =
#if defined(GL_ES_VERSION_2_0) &&                                              \
	(defined(ANDROID) || defined(__APPLE__) || defined(MESON))
	"precision mediump float;\n"
#endif
	"uniform vec4 vColor;\n"
	"void main() {\n"
	"    gl_FragColor = vColor;\n"
	"}\n";

static const GLchar *hud_tex_vertex_shader =
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

static const GLchar *hud_tex_fragment_shader =
#if defined(GL_ES_VERSION_2_0) &&                                              \
	(defined(ANDROID) || defined(__APPLE__) || defined(MESON))
	"precision mediump float;\n"
#endif
	"uniform vec4 vColor;\n"
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_FragColor = vec4(vColor.r, vColor.g, vColor.b, "
	"        texture2D(s_texture, v_texcoord).r);\n"
	"}\n";


static int pdraw_gles2hud_load_texture_from_buffer(struct pdraw_gles2hud *self,
						   const uint8_t *buffer,
						   int width,
						   int height,
						   int texunit)
{
	GLuint tex;

	if (buffer == nullptr)
		return -EINVAL;
	if ((width <= 0) || (height <= 0))
		return -EINVAL;

	GLCHK(glGenTextures(1, &tex));

	GLCHK(glActiveTexture(GL_TEXTURE0 + texunit));
	GLCHK(glBindTexture(GL_TEXTURE_2D, tex));

	GLCHK(glTexImage2D(GL_TEXTURE_2D,
			   0,
			   GL_LUMINANCE,
			   width,
			   height,
			   0,
			   GL_LUMINANCE,
			   GL_UNSIGNED_BYTE,
			   buffer));

	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCHK(glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	return (int)tex;
}


int pdraw_gles2hud_create_programs(struct pdraw_gles2hud *self)
{
	int res;
	GLint vertex_shader = 0, fragment_shader = 0;
	GLint success = 0;

	GLCHK();

	/* Shape drawing shaders */
	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertex_shader == 0) || (vertex_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		goto error;
	}

	glShaderSource(vertex_shader, 1, &hud_vertex_shader, nullptr);
	glCompileShader(vertex_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(vertex_shader, 512, nullptr, info_log);
		ULOGE("vertex shader compilation failed '%s'", info_log);
		goto error;
	}

	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragment_shader == 0) || (fragment_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto error;
	}

	glShaderSource(fragment_shader, 1, &hud_fragment_shader, nullptr);
	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(fragment_shader, 512, nullptr, info_log);
		ULOGE("fragment shader compilation failed '%s'", info_log);
		goto error;
	}

	self->program = glCreateProgram();
	glAttachShader(self->program, vertex_shader);
	glAttachShader(self->program, fragment_shader);
	glLinkProgram(self->program);
	glGetProgramiv(self->program, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetProgramInfoLog(self->program, 512, nullptr, info_log);
		ULOGE("program link failed '%s'", info_log);
		goto error;
	}

	glDeleteShader(vertex_shader);
	vertex_shader = 0;
	glDeleteShader(fragment_shader);
	fragment_shader = 0;

	self->position_handle = glGetAttribLocation(self->program, "vPosition");
	self->transform_matrix_handle =
		glGetUniformLocation(self->program, "transform_matrix");
	self->color_handle = glGetUniformLocation(self->program, "vColor");

	/* Texture drawing shaders */
	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	if ((vertex_shader == 0) || (vertex_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		goto error;
	}

	GLCHK();

	glShaderSource(vertex_shader, 1, &hud_tex_vertex_shader, nullptr);
	glCompileShader(vertex_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(vertex_shader, 512, nullptr, info_log);
		ULOGE("vertex shader compilation failed '%s'", info_log);
		goto error;
	}

	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	if ((fragment_shader == 0) || (fragment_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		goto error;
	}

	glShaderSource(fragment_shader, 1, &hud_tex_fragment_shader, nullptr);
	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(fragment_shader, 512, nullptr, info_log);
		ULOGE("fragment shader compilation failed '%s'", info_log);
		goto error;
	}

	self->tex_program = glCreateProgram();
	glAttachShader(self->tex_program, vertex_shader);
	glAttachShader(self->tex_program, fragment_shader);
	glLinkProgram(self->tex_program);
	glGetProgramiv(self->tex_program, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetProgramInfoLog(self->tex_program, 512, nullptr, info_log);
		ULOGE("program link failed '%s'", info_log);
		goto error;
	}

	glDeleteShader(vertex_shader);
	vertex_shader = 0;
	glDeleteShader(fragment_shader);
	fragment_shader = 0;

	self->tex_uniform_sampler =
		glGetUniformLocation(self->tex_program, "s_texture");
	self->tex_position_handle =
		glGetAttribLocation(self->tex_program, "position");
	self->tex_texcoord_handle =
		glGetAttribLocation(self->tex_program, "texcoord");
	self->tex_transform_matrix_handle =
		glGetUniformLocation(self->tex_program, "transform_matrix");
	self->tex_color_handle =
		glGetUniformLocation(self->tex_program, "vColor");

	GLCHK();

	self->icons_texunit = self->first_texunit;
	res = pdraw_gles2hud_load_texture_from_buffer(self,
						      hud_icons,
						      hud_icons_width,
						      hud_icons_height,
						      self->icons_texunit);
	if (res < 0) {
		ULOG_ERRNO("pdraw_gles2hud_load_texture_from_buffer", -res);
		goto error;
	}
	self->icons_texture = (GLuint)res;

	self->text_texunit = self->first_texunit + 1;
	res = pdraw_gles2hud_load_texture_from_buffer(self,
						      profont_36::image,
						      profont_36::image_width,
						      profont_36::image_height,
						      self->text_texunit);
	if (res < 0) {
		ULOG_ERRNO("pdraw_gles2hud_load_texture_from_buffer", -res);
		goto error;
	}
	self->text_texture = (GLuint)res;

	return 0;

error:
	if (vertex_shader != 0)
		glDeleteShader(vertex_shader);
	if (fragment_shader != 0)
		glDeleteShader(fragment_shader);
	if (self->program > 0) {
		glDeleteProgram(self->program);
		self->program = 0;
	}
	if (self->tex_program > 0) {
		glDeleteProgram(self->tex_program);
		self->tex_program = 0;
	}
	if (self->icons_texture > 0) {
		glDeleteTextures(1, &self->icons_texture);
		self->icons_texture = 0;
	}
	if (self->text_texture > 0) {
		glDeleteTextures(1, &self->text_texture);
		self->text_texture = 0;
	}
	return -EPROTO;
}
