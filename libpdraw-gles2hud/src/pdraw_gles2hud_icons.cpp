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

#include "pdraw_gles2hud_priv.h"


void pdraw_gles2hud_draw_icon(struct pdraw_gles2hud *self,
			      int index,
			      float x,
			      float y,
			      float size,
			      float scalew,
			      float scaleh,
			      const float color[4])
{
	float vertices[8];
	float texcoords[8];

	vertices[0] = x - size * scalew / 2.;
	vertices[1] = y - size * scaleh / 2.;
	vertices[2] = x + size * scalew / 2.;
	vertices[3] = y - size * scaleh / 2.;
	vertices[4] = x - size * scalew / 2.;
	vertices[5] = y + size * scaleh / 2.;
	vertices[6] = x + size * scalew / 2.;
	vertices[7] = y + size * scaleh / 2.;

	GLCHK(glVertexAttribPointer(
		self->tex_position_handle, 2, GL_FLOAT, false, 0, vertices));

	int ix = index % 3;
	int iy = index / 3;

	texcoords[0] = ((float)ix + 0.) / 3.;
	texcoords[1] = ((float)iy + 0.99) / 3.;
	texcoords[2] = ((float)ix + 0.99) / 3.;
	texcoords[3] = ((float)iy + 0.99) / 3.;
	texcoords[4] = ((float)ix + 0.) / 3.;
	texcoords[5] = ((float)iy + 0.) / 3.;
	texcoords[6] = ((float)ix + 0.99) / 3.;
	texcoords[7] = ((float)iy + 0.) / 3.;

	GLCHK(glVertexAttribPointer(
		self->tex_texcoord_handle, 2, GL_FLOAT, false, 0, texcoords));

	GLCHK(glUniform4fv(self->tex_color_handle, 1, color));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
}
