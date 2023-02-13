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


void pdraw_gles2hud_draw_line(struct pdraw_gles2hud *self,
			      float x1,
			      float y1,
			      float x2,
			      float y2,
			      const float color[4],
			      float line_width)
{
	float vertices[4];

	vertices[0] = x1;
	vertices[1] = y1;
	vertices[2] = x2;
	vertices[3] = y2;

	GLCHK(glLineWidth(line_width));

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_LINES, 0, 2));
}


void pdraw_gles2hud_draw_rect(struct pdraw_gles2hud *self,
			      float x1,
			      float y1,
			      float x2,
			      float y2,
			      const float color[4],
			      float line_width)
{
	float vertices[8];

	vertices[0] = x1;
	vertices[1] = y1;
	vertices[2] = x1;
	vertices[3] = y2;
	vertices[4] = x2;
	vertices[5] = y2;
	vertices[6] = x2;
	vertices[7] = y1;

	GLCHK(glLineWidth(line_width));

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_LINE_LOOP, 0, 4));
}


void pdraw_gles2hud_draw_filled_rect(struct pdraw_gles2hud *self,
				     float x1,
				     float y1,
				     float x2,
				     float y2,
				     const float color[4])
{
	float vertices[8];

	vertices[0] = x1;
	vertices[1] = y1;
	vertices[2] = x1;
	vertices[3] = y2;
	vertices[4] = x2;
	vertices[5] = y1;
	vertices[6] = x2;
	vertices[7] = y2;

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
}


void pdraw_gles2hud_draw_arc(struct pdraw_gles2hud *self,
			     float cx,
			     float cy,
			     float rx,
			     float ry,
			     float start_angle,
			     float span_angle,
			     int num_segments,
			     const float color[4],
			     float line_width)
{
	int i;
	float theta = span_angle / (float)num_segments;
	float c = cosf(theta);
	float s = sinf(theta);
	float t;

	float x = cosf(start_angle);
	float y = sinf(start_angle);

	float vertices[2 * (num_segments + 1)];

	for (i = 0; i <= num_segments; i++) {
		vertices[2 * i] = x * rx + cx;
		vertices[2 * i + 1] = y * ry + cy;

		/* apply the rotation */
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}

	GLCHK(glLineWidth(line_width));

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_LINE_STRIP, 0, num_segments + 1));
}


void pdraw_gles2hud_draw_ellipse(struct pdraw_gles2hud *self,
				 float cx,
				 float cy,
				 float rx,
				 float ry,
				 int num_segments,
				 const float color[4],
				 float line_width)
{
	int i;
	float theta = 2. * M_PI / (float)num_segments;
	float c = cosf(theta);
	float s = sinf(theta);
	float t;

	/* start at angle = 0 */
	float x = 1.;
	float y = 0.;

	float vertices[2 * num_segments];

	for (i = 0; i < num_segments; i++) {
		vertices[2 * i] = x * rx + cx;
		vertices[2 * i + 1] = y * ry + cy;

		/* apply the rotation */
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}

	GLCHK(glLineWidth(line_width));

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_LINE_LOOP, 0, num_segments));
}


void pdraw_gles2hud_draw_filled_ellipse(struct pdraw_gles2hud *self,
					float cx,
					float cy,
					float rx,
					float ry,
					int num_segments,
					const float color[4])
{
	int i;
	num_segments &= ~1;
	float theta = 2. * M_PI / (float)num_segments;
	float c = cosf(theta);
	float s = sinf(theta);
	float t;

	/* start at angle = 0 */
	float x = 1.;
	float y = 0.;

	float vertices[(3 * (num_segments / 2) + 1) * 2];

	for (i = 0; i < num_segments / 2; i++) {
		vertices[6 * i] = x * rx + cx;
		vertices[6 * i + 1] = y * ry + cy;

		/* apply the rotation */
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;

		vertices[6 * i + 2] = cx;
		vertices[6 * i + 3] = cy;

		vertices[6 * i + 4] = x * rx + cx;
		vertices[6 * i + 5] = y * ry + cy;

		/* apply the rotation */
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}
	vertices[6 * i] = x * rx + cx;
	vertices[6 * i + 1] = y * ry + cy;

	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices));

	GLCHK(glUniform4fv(self->color_handle, 1, color));

	GLCHK(glDrawArrays(GL_TRIANGLE_STRIP, 0, 3 * (num_segments / 2) + 1));
}
