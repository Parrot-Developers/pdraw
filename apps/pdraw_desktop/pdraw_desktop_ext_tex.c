/**
 * Parrot Drones Audio and Video Vector
 * Desktop player application
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

#include "pdraw_desktop.h"


static const GLchar *vertex_shader =
	"attribute vec4 position;\n"
	"attribute vec2 texcoord;\n"
	"varying vec2 v_texcoord;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    gl_Position = position;\n"
	"    v_texcoord = texcoord;\n"
	"}\n";

static const GLchar *fragment_shader =
#if defined(GL_ES_VERSION_2_0)
	"precision mediump float;\n"
#endif
	"varying vec2 v_texcoord;\n"
	"uniform sampler2D s_texture_0;\n"
	"uniform sampler2D s_texture_1;\n"
	"uniform sampler2D s_texture_2;\n"
	"uniform mat3 yuv2rgb_mat;\n"
	"uniform vec3 yuv2rgb_offset;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec3 yuv;\n"
	"    vec3 rgb;\n"
	"    yuv.r = texture2D(s_texture_0, v_texcoord).r;\n"
	"    yuv.g = texture2D(s_texture_1, v_texcoord).r - 0.5;\n"
	"    yuv.b = texture2D(s_texture_2, v_texcoord).r - 0.5;\n"
	"    rgb = yuv2rgb_mat * (yuv.rgb + yuv2rgb_offset);\n"
	"    rgb = vec3(rgb.b, rgb.g, rgb.r);\n"
	"    gl_FragColor = vec4(rgb, 1.0);\n"
	"}\n";


static const GLfloat yuv2rgb_mat[9] = {
	1.164f,
	1.164f,
	1.164f,
	0.f,
	-0.392f,
	2.017f,
	1.596f,
	-0.813f,
	0.f,
};

static const GLfloat yuv2rgb_offset[3] = {
	-0.0625f,
	0.f,
	0.f,
};


int pdraw_desktop_ext_tex_setup(struct pdraw_desktop *self)
{
	int ret = 0;
	GLint v_shader = 0, f_shader = 0;
	GLint success = 0;
	unsigned int i;

	if (!self->ext_tex)
		return 0;

	v_shader = glCreateShader(GL_VERTEX_SHADER);
	if ((v_shader == 0) || (v_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create vertex shader");
		ret = -ENOMEM;
		goto err;
	}

	glShaderSource(v_shader, 1, &vertex_shader, NULL);
	glCompileShader(v_shader);
	glGetShaderiv(v_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(v_shader, 512, NULL, info_log);
		ULOGE("vertex shader compilation failed '%s'", info_log);
		ret = -EPROTO;
		goto err;
	}

	f_shader = glCreateShader(GL_FRAGMENT_SHADER);
	if ((f_shader == 0) || (f_shader == GL_INVALID_ENUM)) {
		ULOGE("failed to create fragment shader");
		ret = -ENOMEM;
		goto err;
	}

	glShaderSource(f_shader, 1, &fragment_shader, NULL);
	glCompileShader(f_shader);
	glGetShaderiv(f_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar info_log[512];
		glGetShaderInfoLog(f_shader, 512, NULL, info_log);
		ULOGE("fragment shader compilation failed '%s'", info_log);
		ret = -EPROTO;
		goto err;
	}

	self->ext_tex_program = glCreateProgram();
	glAttachShader(self->ext_tex_program, v_shader);
	glAttachShader(self->ext_tex_program, f_shader);
	glLinkProgram(self->ext_tex_program);
	glGetProgramiv(self->ext_tex_program, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar info_log[512] = {};
		glGetShaderInfoLog(self->ext_tex_program, 512, NULL, info_log);
		ULOGE("program link failed '%s'", info_log);
		ret = -EPROTO;
		goto err;
	}

	glDeleteShader(v_shader);
	v_shader = 0;
	glDeleteShader(f_shader);
	f_shader = 0;

	self->ext_tex_yuv2rgb_matrix =
		glGetUniformLocation(self->ext_tex_program, "yuv2rgb_mat");
	self->ext_tex_yuv2rgb_offset =
		glGetUniformLocation(self->ext_tex_program, "yuv2rgb_offset");
	self->ext_tex_uniform_samplers[0] =
		glGetUniformLocation(self->ext_tex_program, "s_texture_0");
	self->ext_tex_uniform_samplers[1] =
		glGetUniformLocation(self->ext_tex_program, "s_texture_1");
	self->ext_tex_uniform_samplers[2] =
		glGetUniformLocation(self->ext_tex_program, "s_texture_2");
	self->ext_tex_position_handle =
		glGetAttribLocation(self->ext_tex_program, "position");
	self->ext_tex_texcoord_handle =
		glGetAttribLocation(self->ext_tex_program, "texcoord");

	glGenTextures(3, self->ext_tex_textures);
	for (i = 0; i < 3; i++) {
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, self->ext_tex_textures[i]);

		glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(
			GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(
			GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}

	return 0;

err:
	if (v_shader > 0)
		glDeleteShader(v_shader);
	if (f_shader > 0)
		glDeleteShader(f_shader);
	pdraw_desktop_ext_tex_cleanup(self);
	return ret;
}


int pdraw_desktop_ext_tex_cleanup(struct pdraw_desktop *self)
{
	if (self->ext_tex_textures[0] > 0) {
		glDeleteTextures(3, self->ext_tex_textures);
		memset(self->ext_tex_textures,
		       0,
		       sizeof(self->ext_tex_textures));
	}
	if (self->ext_tex_program > 0) {
		glDeleteProgram(self->ext_tex_program);
		self->ext_tex_program = 0;
	}

	return 0;
}


int pdraw_desktop_ext_tex_load(struct pdraw_desktop *self,
			       struct pdraw_backend *pdraw,
			       struct pdraw_video_renderer *renderer,
			       const struct pdraw_media_info *media_info,
			       struct mbuf_raw_video_frame *frame,
			       const void *frame_userdata,
			       size_t frame_userdata_len)
{
	int ret;
	unsigned int i, nplanes;
	float vertices[8];
	float texcoords[8];
	const void *planes[VDEF_RAW_MAX_PLANE_COUNT] = {0};
	float ar1, ar2, h, v;

	struct vdef_raw_frame frame_info;

	if (!self->ext_tex)
		return -ENOSYS;

	if ((pdraw == NULL) || (renderer == NULL) || (media_info == NULL) ||
	    (frame == NULL))
		return -EINVAL;

	ret = mbuf_raw_video_frame_get_frame_info(frame, &frame_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return ret;
	}
	if (!vdef_raw_format_cmp(&frame_info.format, &vdef_i420)) {
		ULOGE("unsupported video format");
		return -ENOSYS;
	}
	if (vdef_dim_is_null(&frame_info.info.resolution)) {
		ULOGE("invalid frame dimensions");
		return -EINVAL;
	}
	nplanes = vdef_get_raw_frame_plane_count(&frame_info.format);
	for (i = 0; i < nplanes; i++) {
		size_t dummyLen;
		ret = mbuf_raw_video_frame_get_plane(
			frame, i, &planes[i], &dummyLen);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_plane", -ret);
			goto out;
		}
	}

	glUseProgram(self->ext_tex_program);
	glClear(GL_COLOR_BUFFER_BIT);

	for (i = 0; i < 3; i++) {
		unsigned int height =
			frame_info.info.resolution.height / ((i > 0) ? 2 : 1);
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, self->ext_tex_textures[i]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_LUMINANCE,
			     frame_info.plane_stride[i],
			     height,
			     0,
			     GL_LUMINANCE,
			     GL_UNSIGNED_BYTE,
			     planes[i]);
		glUniform1i(self->ext_tex_uniform_samplers[i], i);
	}

	glUniform3f(self->ext_tex_yuv2rgb_offset,
		    yuv2rgb_offset[0],
		    yuv2rgb_offset[1],
		    yuv2rgb_offset[2]);
	glUniformMatrix3fv(
		self->ext_tex_yuv2rgb_matrix, 1, GL_FALSE, &yuv2rgb_mat[0]);

	vertices[0] = -1.;
	vertices[1] = -1.;
	vertices[2] = 1.;
	vertices[3] = -1.;
	vertices[4] = -1.;
	vertices[5] = 1.;
	vertices[6] = 1.;
	vertices[7] = 1.;

	glVertexAttribPointer(self->ext_tex_position_handle,
			      2,
			      GL_FLOAT,
			      GL_FALSE,
			      0,
			      vertices);
	glEnableVertexAttribArray(self->ext_tex_position_handle);

	/* Test: take a 2:3 crop at the center */
	ar1 = (float)frame_info.info.resolution.width /
	      (float)frame_info.info.resolution.height;
	ar2 = 2.f / 3.f;
	if (ar1 < ar2) {
		h = 1.f;
		v = ar2 / ar1;
	} else {
		h = ar2 / ar1;
		v = 1.f;
	}
	h = (1.f - h) / 2.f;
	v = (1.f - v) / 2.f;
	texcoords[0] = h * (float)frame_info.info.resolution.width /
		       (float)frame_info.plane_stride[0];
	texcoords[1] = 1.f - v;
	texcoords[2] = (1.f - h) * (float)frame_info.info.resolution.width /
		       (float)frame_info.plane_stride[0];
	texcoords[3] = 1.f - v;
	texcoords[4] = h * (float)frame_info.info.resolution.width /
		       (float)frame_info.plane_stride[0];
	texcoords[5] = v;
	texcoords[6] = (1.f - h) * (float)frame_info.info.resolution.width /
		       (float)frame_info.plane_stride[0];
	texcoords[7] = v;

	glVertexAttribPointer(self->ext_tex_texcoord_handle,
			      2,
			      GL_FLOAT,
			      GL_FALSE,
			      0,
			      texcoords);
	glEnableVertexAttribArray(self->ext_tex_texcoord_handle);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableVertexAttribArray(self->ext_tex_position_handle);
	glDisableVertexAttribArray(self->ext_tex_texcoord_handle);

out:
	for (i = 0; i < nplanes; i++) {
		if (planes[i] == NULL)
			continue;
		int err =
			mbuf_raw_video_frame_release_plane(frame, i, planes[i]);
		if (err < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane(%u)",
				   -err,
				   i);
		}
	}
	return 0;
}
