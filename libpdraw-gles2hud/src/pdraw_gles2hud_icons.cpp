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


void pdraw_gles2hud_draw_icon(const struct pdraw_gles2hud *self,
			      int index,
			      float x,
			      float y,
			      float size,
			      float scalew,
			      float scaleh,
			      const std::array<float, 4> &color)
{
	const int ix = index % 3;
	const int iy = index / 3;

	const std::array<float, 8> vertices = {
		x - size * scalew / 2.f,
		y - size * scaleh / 2.f,
		x + size * scalew / 2.f,
		y - size * scaleh / 2.f,
		x - size * scalew / 2.f,
		y + size * scaleh / 2.f,
		x + size * scalew / 2.f,
		y + size * scaleh / 2.f,
	};
	const std::array<float, 8> texcoords = {
		static_cast<float>(ix) / 3.f,
		(static_cast<float>(iy) + 0.99f) / 3.f,
		(static_cast<float>(ix) + 0.99f) / 3.f,
		(static_cast<float>(iy) + 0.99f) / 3.f,
		static_cast<float>(ix) / 3.f,
		static_cast<float>(iy) / 3.f,
		(static_cast<float>(ix) + 0.99f) / 3.f,
		static_cast<float>(iy) / 3.f,
	};

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

	GLCHK(glUniform4fv(self->tex_color_handle, 1, color.data()));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
}
