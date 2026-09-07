/**
 * Parrot Drones Audio and Video Vector
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

#include "pdraw_gles2hud_priv.hpp"


void pdraw_gles2hud_get_text_dimensions(const std::string &str,
					float size,
					float scalew,
					float scaleh,
					float *width,
					float *height)
{
	float w;
	float h;

	const profont_36::file_header *glyph_info = &profont_36::font;
	float cx = 0.;
	const char *c = str.c_str();
	while (*c != '\0') {
		if (*c == '\n') {
			break;
		} else {
			const profont_36::glyph_info &g =
				glyph_info->glyphs[(int)(*c)];
			cx += g.norm.advance;
		}
		c++;
	}
	w = cx;
	h = glyph_info->norm.ascent + glyph_info->norm.descent;
	w *= size * scalew;
	h *= size * scaleh;

	if (width)
		*width = w;
	if (height)
		*height = h;
}


void pdraw_gles2hud_draw_text(const struct pdraw_gles2hud *self,
			      const std::string &str,
			      float x,
			      float y,
			      float size,
			      float scalew,
			      float scaleh,
			      TextAlign halign,
			      TextAlign valign,
			      const std::array<float, 4> &color)
{
	float w;
	float h;
	std::array<float, 8> vertices;
	std::array<float, 8> texcoords;
	const profont_36::file_header *glyph_info = &profont_36::font;

	pdraw_gles2hud_get_text_dimensions(str, size, scalew, scaleh, &w, &h);

	switch (halign) {
	default:
	case TextAlign::LEFT:
		break;
	case TextAlign::CENTER:
		x -= w / 2;
		break;
	case TextAlign::RIGHT:
		x -= w;
		break;
	}

	switch (valign) {
	default:
	case TextAlign::TOP:
		y -= h;
		break;
	case TextAlign::MIDDLE:
		y -= h / 2;
		break;
	case TextAlign::BOTTOM:
		break;
	}

	GLCHK(glUniform4fv(self->tex_color_handle, 1, color.data()));

	const char *c = str.c_str();
	float cx = 0.;
	while (*c != '\0') {
		if (*c == '\n')
			break;

		const profont_36::glyph_info &g = glyph_info->glyphs[(int)(*c)];
		vertices[0] = x + cx + g.norm.offx * size * scalew;
		vertices[1] = y - g.norm.offy * size * scaleh;
		vertices[2] =
			x + cx + (g.norm.offx + g.norm.width) * size * scalew;
		vertices[3] = y - g.norm.offy * size * scaleh;
		vertices[4] = x + cx + g.norm.offx * size * scalew;
		vertices[5] = y - (g.norm.offy + g.norm.height) * size * scaleh;
		vertices[6] =
			x + cx + (g.norm.offx + g.norm.width) * size * scalew;
		vertices[7] = y - (g.norm.offy + g.norm.height) * size * scaleh;

		texcoords[0] = g.norm.u;
		texcoords[1] = g.norm.v + g.norm.height;
		texcoords[2] = g.norm.u + g.norm.width;
		texcoords[3] = g.norm.v + g.norm.height;
		texcoords[4] = g.norm.u;
		texcoords[5] = g.norm.v;
		texcoords[6] = g.norm.u + g.norm.width;
		texcoords[7] = g.norm.v;

		GLCHK(glVertexAttribPointer(self->tex_position_handle,
					    2,
					    GL_FLOAT,
					    false,
					    0,
					    vertices.data()));
		GLCHK(glVertexAttribPointer(self->tex_texcoord_handle,
					    2,
					    GL_FLOAT,
					    false,
					    0,
					    texcoords.data()));
		GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));

		cx += g.norm.advance * size * scalew;
		c++;
	}
}
