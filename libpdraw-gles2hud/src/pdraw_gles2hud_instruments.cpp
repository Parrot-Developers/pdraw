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


void pdraw_gles2hud_draw_vumeter(const struct pdraw_gles2hud *self,
				 float x,
				 float y,
				 float r,
				 float value,
				 float val_min,
				 float val_max,
				 float critical_min,
				 float critical_max,
				 const std::array<float, 4> &color,
				 const std::array<float, 4> &critical_color)
{
	x *= self->ratio_w;
	y *= self->ratio_h;
	auto span = static_cast<float>(4. * M_PI / 3.);
	auto start = static_cast<float>((M_PI - span) / 2.);
	pdraw_gles2hud_draw_arc(self,
				x,
				y,
				r * self->ratio_w,
				r * self->ratio_w * self->aspect_ratio,
				start,
				span,
				20,
				color,
				2.f);
	if ((critical_min >= val_min) && (critical_min <= val_max) &&
	    (critical_max >= val_min) && (critical_max <= val_max) &&
	    (critical_min < critical_max)) {
		float start2 = start + (1.f - (critical_max - val_min) /
						      (val_max - val_min)) *
					       span;
		float end2 = start + (1.f - (critical_min - val_min) /
						    (val_max - val_min)) *
					     span;
		pdraw_gles2hud_draw_arc(self,
					x,
					y,
					r * self->ratio_w * 0.9f,
					r * self->ratio_w * self->aspect_ratio *
						0.9f,
					start2,
					end2 - start2,
					10,
					critical_color,
					2.f);
	}

	if ((value < val_min) || (value > val_max))
		return;

	float angle =
		start + (1.f - (value - val_min) / (val_max - val_min)) * span;
	float x1 = x + r * self->ratio_w * 0.4f * cosf(angle);
	float y1 =
		y + r * self->ratio_w * self->aspect_ratio * 0.4f * sinf(angle);
	float x2 = x + r * self->ratio_w * 0.9f * cosf(angle);
	float y2 =
		y + r * self->ratio_w * self->aspect_ratio * 0.9f * sinf(angle);
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
}


void pdraw_gles2hud_draw_artificial_horizon(const struct pdraw_gles2hud *self,
					    const struct vmeta_euler *drone,
					    const struct vmeta_euler *frame,
					    const std::array<float, 4> &color)
{
	float x1;
	float y1;
	float x2;
	float y2;
	float height = self->config.central_zone_size * self->ratio_w *
		       self->aspect_ratio;
	int steps = 6;

	/* Scale */
	for (int i = -steps; i <= steps; i++) {
		if ((i != 0) && (i & 1)) {
			pdraw_gles2hud_draw_line(
				self,
				-0.01f * self->ratio_w,
				static_cast<float>(i) * height / 2.f /
					static_cast<float>(steps),
				0.01f * self->ratio_w,
				static_cast<float>(i) * height / 2.f /
					static_cast<float>(steps),
				color,
				2.f);
		}
	}

	/* Horizon */
	x1 = -0.5f * self->config.central_zone_size * self->ratio_w *
	     cosf(frame->phi);
	y1 = -0.5f * self->config.central_zone_size * self->ratio_w *
	     self->aspect_ratio * sinf(frame->phi);
	x2 = 0.5f * self->config.central_zone_size * self->ratio_w *
	     cosf(frame->phi);
	y2 = 0.5f * self->config.central_zone_size * self->ratio_w *
	     self->aspect_ratio * sinf(frame->phi);
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);

	/* Drone */
	std::array<float, 10> vertices;
	float drone_y = static_cast<float>(drone->theta / (M_PI / 18 * steps)) *
			height / 2.f;
	vertices[0] = -0.06f * self->ratio_w * cosf(frame->phi - drone->phi);
	vertices[1] = -0.06f * self->ratio_w * self->aspect_ratio *
			      sinf(frame->phi - drone->phi) +
		      drone_y;
	vertices[2] = -0.015f * self->ratio_w * cosf(frame->phi - drone->phi);
	vertices[3] = -0.015f * self->ratio_w * self->aspect_ratio *
			      sinf(frame->phi - drone->phi) +
		      drone_y;
	vertices[4] = 0.015f * self->ratio_w * sinf(frame->phi - drone->phi);
	vertices[5] = -0.015f * self->ratio_w * self->aspect_ratio *
			      cosf(frame->phi - drone->phi) +
		      drone_y;
	vertices[6] = 0.015f * self->ratio_w * cosf(frame->phi - drone->phi);
	vertices[7] = 0.015f * self->ratio_w * self->aspect_ratio *
			      sinf(frame->phi - drone->phi) +
		      drone_y;
	vertices[8] = 0.06f * self->ratio_w * cosf(frame->phi - drone->phi);
	vertices[9] = 0.06f * self->ratio_w * self->aspect_ratio *
			      sinf(frame->phi - drone->phi) +
		      drone_y;
	GLCHK(glLineWidth(6.f));
	GLCHK(glVertexAttribPointer(
		self->position_handle, 2, GL_FLOAT, false, 0, vertices.data()));
	GLCHK(glUniform4fv(self->color_handle, 1, color.data()));
	GLCHK(glDrawArrays(GL_LINE_STRIP, 0, 5));
}


void pdraw_gles2hud_draw_roll(const struct pdraw_gles2hud *self,
			      float drone_roll,
			      const std::array<float, 4> &color)
{
	int i;
	float rotation;
	float x1;
	float y1;
	float x2;
	float y2;
	float width = 0.12f * self->ratio_w;
	float y_offset = self->config.roll_zone_v_offset * self->ratio_h;
	int steps = 6;

	pdraw_gles2hud_draw_arc(
		self,
		0.f,
		y_offset,
		width,
		width * self->aspect_ratio,
		static_cast<float>(M_PI * (90. - 10. * steps) / 180.),
		static_cast<float>(M_PI * 20. * steps / 180.),
		100,
		color,
		2.f);
	rotation = static_cast<float>(M_PI / 2.) - drone_roll;
	x1 = (width - 0.012f * self->ratio_w) * cosf(rotation);
	y1 = (width - 0.012f * self->ratio_w) * self->aspect_ratio *
		     sinf(rotation) +
	     y_offset;
	x2 = (width + 0.012f * self->ratio_w) * cosf(rotation);
	y2 = (width + 0.012f * self->ratio_w) * self->aspect_ratio *
		     sinf(rotation) +
	     y_offset;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);

	for (i = -steps,
	    rotation = static_cast<float>(M_PI * (90. - 10. * steps) / 180.);
	     i <= steps;
	     i++, rotation += static_cast<float>(M_PI * 10. / 180.)) {
		int angle = (i * 10 + 60 + 360) % 360;
		if (angle <= 120) {
			x1 = width * cosf(rotation);
			y1 = width * self->aspect_ratio * sinf(rotation) +
			     y_offset;
			x2 = (width - 0.008f * self->ratio_w) * cosf(rotation);
			y2 = (width - 0.008f * self->ratio_w) *
				     self->aspect_ratio * sinf(rotation) +
			     y_offset;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
		}
	}

	x1 = 0.f;
	y1 = y_offset;
	x2 = 0.f;
	y2 = y_offset;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
}


void pdraw_gles2hud_draw_heading(const struct pdraw_gles2hud *self,
				 float drone_yaw,
				 float horizontal_speed,
				 float speed_psi,
				 const std::array<float, 4> &color)
{
	int i;
	int heading = ((int)(drone_yaw * RAD_TO_DEG) + 360) % 360;
	float rotation;
	float x1;
	float y1;
	float x2;
	float y2;
	float width = 0.12f * self->ratio_w;
	float y_offset = self->config.heading_zone_v_offset * self->ratio_h;

	pdraw_gles2hud_draw_arc(self,
				0.f,
				y_offset,
				width,
				width * self->aspect_ratio,
				static_cast<float>(M_PI * 20. / 180.),
				static_cast<float>(M_PI * 140. / 180.),
				100,
				color,
				2.f);
	x1 = 0.f;
	y1 = y_offset + width * self->aspect_ratio;
	x2 = 0.f;
	y2 = y_offset + (width + 0.01f * self->ratio_w) * self->aspect_ratio;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);

	for (i = 0, rotation = drone_yaw + static_cast<float>(M_PI / 2.);
	     i < 36;
	     i++, rotation += static_cast<float>(M_PI * 10. / 180.)) {
		int angle = (heading + i * 10 + 70 + 360) % 360;
		if (angle <= 140) {
			x1 = width * cosf(rotation);
			y1 = width * self->aspect_ratio * sinf(rotation) +
			     y_offset;
			x2 = (width - 0.01f * self->ratio_w) * cosf(rotation);
			y2 = (width - 0.01f * self->ratio_w) *
				     self->aspect_ratio * sinf(rotation) +
			     y_offset;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
		}
	}

	if (horizontal_speed >= 0.2f) {
		rotation =
			drone_yaw - speed_psi + static_cast<float>(M_PI / 2.);
		x1 = 0.045f * self->ratio_w * cosf(rotation);
		y1 = 0.045f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation) +
		     y_offset;
		x2 = 0.020f * self->ratio_w * cosf(rotation);
		y2 = 0.020f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation) +
		     y_offset;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		x1 = 0.045f * self->ratio_w * cosf(rotation);
		y1 = 0.045f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation) +
		     y_offset;
		x2 = 0.010f * self->ratio_w *
			     cosf(rotation -
				  static_cast<float>(5. * M_PI / 6.)) +
		     x1;
		y2 = 0.010f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation -
				  static_cast<float>(5. * M_PI / 6.)) +
		     y1;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		x1 = 0.045f * self->ratio_w * cosf(rotation);
		y1 = 0.045f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation) +
		     y_offset;
		x2 = 0.010f * self->ratio_w *
			     cosf(rotation +
				  static_cast<float>(5. * M_PI / 6.)) +
		     x1;
		y2 = 0.010f * self->ratio_w * self->aspect_ratio *
			     sinf(rotation +
				  static_cast<float>(5. * M_PI / 6.)) +
		     y1;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	}
}


void pdraw_gles2hud_draw_altitude(const struct pdraw_gles2hud *self,
				  double altitude,
				  float ground_distance,
				  float down_speed,
				  const std::array<float, 4> &color)
{
	std::string altitude_str;

	if (std::isnan(altitude))
		altitude = 0.;

	altitude_str = fmtstr("%.1fm", altitude);

	float x_offset = self->config.central_zone_size * self->ratio_w;
	float height = self->config.central_zone_size * self->ratio_w *
		       self->aspect_ratio;
	float altitude_interval = height / 20.f;

	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 -height / 2.f,
				 x_offset,
				 height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 -height / 2.f,
				 x_offset + 0.08f * self->ratio_w,
				 -height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 height / 2.f,
				 x_offset + 0.08f * self->ratio_w,
				 height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_rect(self,
				 x_offset + 0.03f * self->ratio_w,
				 -0.017f * self->ratio_w * self->aspect_ratio,
				 x_offset + 0.03f * self->ratio_w +
					 0.1f * self->ratio_w,
				 0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 0.f,
				 x_offset + 0.03f * self->ratio_w,
				 -0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 0.f,
				 x_offset + 0.03f * self->ratio_w,
				 0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);

	float y = static_cast<float>(ceil(altitude) - altitude) *
		  altitude_interval;
	auto alt_int = ((int)ceil(altitude));
	int alt_mod5 = alt_int % 5;
	while (y < height / 2.f) {
		pdraw_gles2hud_draw_line(
			self,
			x_offset,
			y,
			x_offset + ((alt_mod5 == 0) ? 0.03f * self->ratio_w
						    : 0.01f * self->ratio_w),
			y,
			color,
			2.f);
		if ((!alt_mod5) &&
		    (y > -height / 2.f + 0.03f * self->ratio_w) &&
		    (y < height / 2.f - 0.03f * self->ratio_w) &&
		    (!((y > -0.03f * self->ratio_w) &&
		       (y < 0.03f * self->ratio_w)))) {
			altitude_str = fmtstr("%d", alt_int);
		}
		y += altitude_interval;
		alt_int++;
		alt_mod5 = alt_int % 5;
	}
	y = static_cast<float>(-(altitude - floor(altitude))) *
	    altitude_interval;
	alt_int = ((int)floor(altitude));
	alt_mod5 = alt_int % 5;
	while (y > -height / 2.f) {
		pdraw_gles2hud_draw_line(
			self,
			x_offset,
			y,
			x_offset + ((alt_mod5 == 0) ? 0.03f * self->ratio_w
						    : 0.01f * self->ratio_w),
			y,
			color,
			2.f);
		if ((!alt_mod5) &&
		    (y > -height / 2.f +
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (y < height / 2.f -
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (!((y > -0.017f * self->ratio_w * self->aspect_ratio) &&
		       (y < 0.017f * self->ratio_w * self->aspect_ratio)))) {
			altitude_str = fmtstr("%d", alt_int);
		}
		y -= altitude_interval;
		alt_int--;
		alt_mod5 = alt_int % 5;
	}

	/* Ground distance */
	y = -ground_distance * height / 20.f;
	if ((y < height / 2.f) &&
	    (y > -height / 2.f + 0.01f * self->ratio_w * self->aspect_ratio)) {
		if ((y > -0.012f * self->ratio_w * self->aspect_ratio) &&
		    (y < 0.022f * self->ratio_w * self->aspect_ratio)) {
			pdraw_gles2hud_draw_line(self,
						 x_offset,
						 y,
						 x_offset +
							 0.03f * self->ratio_w,
						 y,
						 color,
						 2.f);
		} else {
			pdraw_gles2hud_draw_line(self,
						 x_offset,
						 y,
						 x_offset +
							 0.06f * self->ratio_w,
						 y,
						 color,
						 2.f);
			pdraw_gles2hud_draw_line(
				self,
				x_offset + 0.03f * self->ratio_w,
				y,
				x_offset + 0.04f * self->ratio_w,
				y - 0.01f * self->ratio_w * self->aspect_ratio,
				color,
				2.f);
			pdraw_gles2hud_draw_line(
				self,
				x_offset + 0.04f * self->ratio_w,
				y,
				x_offset + 0.05f * self->ratio_w,
				y - 0.01f * self->ratio_w * self->aspect_ratio,
				color,
				2.f);
			pdraw_gles2hud_draw_line(
				self,
				x_offset + 0.05f * self->ratio_w,
				y,
				x_offset + 0.06f * self->ratio_w,
				y - 0.01f * self->ratio_w * self->aspect_ratio,
				color,
				2.f);
		}
		pdraw_gles2hud_draw_line(self,
					 x_offset,
					 y,
					 x_offset + 0.01f * self->ratio_w,
					 y - 0.01f * self->ratio_w *
							 self->aspect_ratio,
					 color,
					 2.f);
		pdraw_gles2hud_draw_line(self,
					 x_offset + 0.01f * self->ratio_w,
					 y,
					 x_offset + 0.02f * self->ratio_w,
					 y - 0.01f * self->ratio_w *
							 self->aspect_ratio,
					 color,
					 2.f);
		pdraw_gles2hud_draw_line(self,
					 x_offset + 0.02f * self->ratio_w,
					 y,
					 x_offset + 0.03f * self->ratio_w,
					 y - 0.01f * self->ratio_w *
							 self->aspect_ratio,
					 color,
					 2.f);
	}

	/* Speed indication */
	if (fabs(down_speed) >= 0.2f) {
		float x1;
		float y1;
		float x2;
		float y2;
		x1 = x_offset + 0.15f * self->ratio_w;
		y1 = -0.017f * self->ratio_w * self->aspect_ratio;
		x2 = x_offset + 0.15f * self->ratio_w;
		y2 = 0.017f * self->ratio_w * self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		if (down_speed < 0.f) {
			x1 = x_offset + 0.15f * self->ratio_w;
			y1 = 0.017f * self->ratio_w * self->aspect_ratio;
			x2 = x_offset + 0.15f * self->ratio_w -
			     0.0056f * self->ratio_w;
			y2 = 0.017f * self->ratio_w * self->aspect_ratio -
			     0.0098f * self->ratio_w * self->aspect_ratio;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
			x1 = x_offset + 0.15f * self->ratio_w;
			y1 = 0.017f * self->ratio_w * self->aspect_ratio;
			x2 = x_offset + 0.15f * self->ratio_w +
			     0.0056f * self->ratio_w;
			y2 = 0.017f * self->ratio_w * self->aspect_ratio -
			     0.0098f * self->ratio_w * self->aspect_ratio;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
		} else {
			x1 = x_offset + 0.15f * self->ratio_w;
			y1 = -0.017f * self->ratio_w * self->aspect_ratio;
			x2 = x_offset + 0.15f * self->ratio_w -
			     0.0056f * self->ratio_w;
			y2 = -0.017f * self->ratio_w * self->aspect_ratio +
			     0.0098f * self->ratio_w * self->aspect_ratio;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
			x1 = x_offset + 0.15f * self->ratio_w;
			y1 = -0.017f * self->ratio_w * self->aspect_ratio;
			x2 = x_offset + 0.15f * self->ratio_w +
			     0.0056f * self->ratio_w;
			y2 = -0.017f * self->ratio_w * self->aspect_ratio +
			     0.0098f * self->ratio_w * self->aspect_ratio;
			pdraw_gles2hud_draw_line(
				self, x1, y1, x2, y2, color, 2.f);
		}
	}
}


void pdraw_gles2hud_draw_speed(const struct pdraw_gles2hud *self,
			       float horizontal_speed,
			       const std::array<float, 4> &color)
{
	std::string speed_str = fmtstr("%.1fm/s", horizontal_speed);

	float x_offset = -self->config.central_zone_size * self->ratio_w;
	float height = self->config.central_zone_size * self->ratio_w *
		       self->aspect_ratio;
	float speed_interval = height / 20.f;

	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 -height / 2.f,
				 x_offset,
				 height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 -height / 2.f,
				 x_offset - 0.08f * self->ratio_w,
				 -height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 height / 2.f,
				 x_offset - 0.08f * self->ratio_w,
				 height / 2.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_rect(self,
				 x_offset - 0.03f * self->ratio_w,
				 -0.017f * self->ratio_w * self->aspect_ratio,
				 x_offset - 0.03f * self->ratio_w -
					 0.1f * self->ratio_w,
				 0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 0.f,
				 x_offset - 0.03f * self->ratio_w,
				 -0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(self,
				 x_offset,
				 0.f,
				 x_offset - 0.03f * self->ratio_w,
				 0.017f * self->ratio_w * self->aspect_ratio,
				 color,
				 2.f);

	float y = (ceil(horizontal_speed) - horizontal_speed) * speed_interval;
	auto spd_int = ((int)ceil(horizontal_speed));
	int spd_mod5 = spd_int % 5;
	while (y < height / 2.f) {
		pdraw_gles2hud_draw_line(
			self,
			x_offset,
			y,
			x_offset - ((spd_mod5 == 0) ? 0.03f * self->ratio_w
						    : 0.01f * self->ratio_w),
			y,
			color,
			2.f);
		if ((!spd_mod5) &&
		    (y > -height / 2.f +
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (y < height / 2.f -
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (!((y > -0.017f * self->ratio_w * self->aspect_ratio) &&
		       (y < 0.017f * self->ratio_w * self->aspect_ratio)))) {
			speed_str = fmtstr("%d", spd_int);
		}
		y += speed_interval;
		spd_int++;
		spd_mod5 = spd_int % 5;
	}
	y = -(horizontal_speed - floor(horizontal_speed)) * speed_interval;
	spd_int = ((int)floor(horizontal_speed));
	spd_mod5 = spd_int % 5;
	while (y > -height / 2.f) {
		pdraw_gles2hud_draw_line(
			self,
			x_offset,
			y,
			x_offset - ((spd_mod5 == 0) ? 0.03f * self->ratio_w
						    : 0.01f * self->ratio_w),
			y,
			color,
			2.f);
		if ((!spd_mod5) &&
		    (y > -height / 2.f +
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (y < height / 2.f -
				 0.017f * self->ratio_w * self->aspect_ratio) &&
		    (!((y > -0.017f * self->ratio_w * self->aspect_ratio) &&
		       (y < 0.017f * self->ratio_w * self->aspect_ratio)))) {
			speed_str = fmtstr("%d", spd_int);
		}
		y -= speed_interval;
		spd_int--;
		spd_mod5 = spd_int % 5;
	}
}


void pdraw_gles2hud_draw_controller_radar(const struct pdraw_gles2hud *self,
					  double distance,
					  float controller_radar_angle,
					  const std::array<float, 4> &color)
{
	float width = 0.08f * self->ratio_w;
	float x_offset = self->config.radar_zone_h_offset * self->ratio_w;
	float y_offset = self->config.radar_zone_v_offset * self->ratio_h;
	float x1;
	float y1;
	float x2;
	float y2;

	pdraw_gles2hud_draw_ellipse(self,
				    x_offset,
				    y_offset,
				    width,
				    width * self->aspect_ratio,
				    100,
				    color,
				    2.f);
	x1 = x_offset;
	y1 = y_offset + width * self->aspect_ratio;
	x2 = x_offset;
	y2 = y_offset + (width + 0.008f * self->ratio_w) * self->aspect_ratio;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);

	if (distance > 50.) {
		x1 = x_offset -
		     width / 3.f * sinf(controller_radar_angle / 2.f);
		y1 = y_offset + width / 3.f *
					cosf(controller_radar_angle / 2.f) *
					self->aspect_ratio;
		x2 = x_offset - width * sinf(controller_radar_angle / 2.f);
		y2 = y_offset + width * cosf(controller_radar_angle / 2.f) *
					self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		x1 = x_offset +
		     width / 3.f * sinf(controller_radar_angle / 2.f);
		y1 = y_offset + width / 3.f *
					cosf(controller_radar_angle / 2.f) *
					self->aspect_ratio;
		x2 = x_offset + width * sinf(controller_radar_angle / 2.f);
		y2 = y_offset + width * cosf(controller_radar_angle / 2.f) *
					self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	}
}


void pdraw_gles2hud_draw_record_timeline(const struct pdraw_gles2hud *self,
					 uint64_t current_time,
					 uint64_t duration,
					 const std::array<float, 4> &color)
{
	float x_offset = self->config.right_zone_h_offset * self->ratio_w;
	float y_offset = self->config.roll_zone_v_offset * self->ratio_h +
			 0.12f * self->ratio_w * self->aspect_ratio;
	float width = 0.4f * self->ratio_w;
	float height = 0.015f * self->ratio_w * self->aspect_ratio;
	float x1;
	float y1;
	float x2;
	float y2;
	float cw = 0.f;
	float rw = 0.f;

	uint64_t remaining_time = duration - current_time;
	unsigned int c_hrs = 0;
	unsigned int c_min = 0;
	unsigned int c_sec = 0;
	unsigned int c_msec = 0;
	unsigned int r_hrs = 0;
	unsigned int r_min = 0;
	unsigned int r_sec = 0;
	unsigned int r_msec = 0;
	unsigned int d_hrs = 0;
	unsigned int d_min = 0;
	unsigned int d_sec = 0;
	unsigned int d_msec = 0;
	pdraw_gles2hud_friendly_time_from_us(
		current_time, &c_hrs, &c_min, &c_sec, &c_msec);
	pdraw_gles2hud_friendly_time_from_us(
		remaining_time, &r_hrs, &r_min, &r_sec, &r_msec);
	pdraw_gles2hud_friendly_time_from_us(
		duration, &d_hrs, &d_min, &d_sec, &d_msec);
	std::string str;
	if (d_hrs) {
		str = fmtstr(
			"+%02d:%02d:%02d.%03d", c_hrs, c_min, c_sec, c_msec);
	} else {
		str = fmtstr("+%02d:%02d.%03d", c_min, c_sec, c_msec);
	}
	pdraw_gles2hud_get_text_dimensions(str,
					   0.15f * self->ratio_w,
					   1.f,
					   self->aspect_ratio,
					   &cw,
					   nullptr);
	cw += 0.012f;
	if (d_hrs) {
		str = fmtstr(
			"-%02d:%02d:%02d.%03d", r_hrs, r_min, r_sec, r_msec);
	} else {
		str = fmtstr("-%02d:%02d.%03d", r_min, r_sec, r_msec);
	}
	pdraw_gles2hud_get_text_dimensions(str,
					   0.15f * self->ratio_w,
					   1.f,
					   self->aspect_ratio,
					   &rw,
					   nullptr);
	rw += 0.01f;

	x1 = x_offset - rw;
	y2 = y_offset;
	y1 = y2;
	x2 = x_offset - width + cw;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	x2 = x_offset - rw;
	x1 = x2;
	y1 = y_offset + height / 2.f;
	y2 = y_offset - height / 2.f;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	x2 = x_offset - width + cw;
	x1 = x2;
	y1 = y_offset + height / 2.f;
	y2 = y_offset - height / 2.f;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	x2 = x_offset - rw -
	     (1.f -
	      static_cast<float>(current_time) / static_cast<float>(duration)) *
		     (width - rw - cw);
	x1 = x2;
	y1 = y_offset + height / 2.f;
	y2 = y_offset - height / 2.f;
	pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
}


void pdraw_gles2hud_draw_cot(const struct pdraw_gles2hud *self,
			     float x,
			     float y,
			     const std::array<float, 4> &color)
{
	pdraw_gles2hud_draw_line(self,
				 x * 2.f - 1.f - 0.012f * self->ratio_w,
				 y * 2.f - 1.f,
				 x * 2.f - 1.f + 0.012f * self->ratio_w,
				 y * 2.f - 1.f,
				 color,
				 2.f);
	pdraw_gles2hud_draw_line(
		self,
		x * 2.f - 1.f,
		y * 2.f - 1.f - 0.012f * self->ratio_w * self->aspect_ratio,
		x * 2.f - 1.f,
		y * 2.f - 1.f + 0.012f * self->ratio_w * self->aspect_ratio,
		color,
		2.f);
	pdraw_gles2hud_draw_arc(self,
				x * 2.f - 1.f,
				y * 2.f - 1.f,
				0.008f * self->ratio_w,
				0.008f * self->ratio_w * self->aspect_ratio,
				static_cast<float>(M_PI * 25. / 180.),
				static_cast<float>(M_PI * 40. / 180.),
				10,
				color,
				2.f);
	pdraw_gles2hud_draw_arc(self,
				x * 2.f - 1.f,
				y * 2.f - 1.f,
				0.008f * self->ratio_w,
				0.008f * self->ratio_w * self->aspect_ratio,
				static_cast<float>(M_PI * 115. / 180.),
				static_cast<float>(M_PI * 40. / 180.),
				10,
				color,
				2.f);
	pdraw_gles2hud_draw_arc(self,
				x * 2.f - 1.f,
				y * 2.f - 1.f,
				0.008f * self->ratio_w,
				0.008f * self->ratio_w * self->aspect_ratio,
				static_cast<float>(M_PI * 205. / 180.),
				static_cast<float>(M_PI * 40. / 180.),
				10,
				color,
				2.f);
	pdraw_gles2hud_draw_arc(self,
				x * 2.f - 1.f,
				y * 2.f - 1.f,
				0.008f * self->ratio_w,
				0.008f * self->ratio_w * self->aspect_ratio,
				static_cast<float>(M_PI * 295. / 180.),
				static_cast<float>(M_PI * 40. / 180.),
				10,
				color,
				2.f);
}


void pdraw_gles2hud_draw_flight_path_vector(const struct pdraw_gles2hud *self,
					    const struct vmeta_euler *frame,
					    float speed_theta,
					    float speed_psi,
					    const std::array<float, 4> &color)
{
	float x = (speed_psi - frame->psi) / self->h_fov * 2.f * self->ratio_w;
	float y = (speed_theta - frame->theta) / self->v_fov * 2.f *
		  self->ratio_h;
	float x1;
	float y1;
	float x2;
	float y2;
	float tx;
	float ty;
	float rotation = frame->phi;

	if ((x > -self->ratio_w) && (x < self->ratio_w) &&
	    (y > -self->ratio_h) && (y < self->ratio_h)) {
		pdraw_gles2hud_draw_ellipse(self,
					    x,
					    y,
					    0.02f * self->ratio_w,
					    0.02f * self->ratio_w *
						    self->aspect_ratio,
					    40,
					    color,
					    2.f);
		tx = 0.f;
		ty = 0.02f;
		x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		tx = 0.f;
		ty = 0.03f;
		x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		tx = -0.02f;
		ty = 0.f;
		x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		tx = -0.03f;
		ty = 0.f;
		x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
		tx = 0.02f;
		ty = 0.f;
		x1 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y1 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		tx = 0.03f;
		ty = 0.f;
		x2 = x + (tx * cosf(rotation) - ty * sinf(rotation)) *
				 self->ratio_w;
		y2 = y + (tx * sinf(rotation) + ty * cosf(rotation)) *
				 self->ratio_w * self->aspect_ratio;
		pdraw_gles2hud_draw_line(self, x1, y1, x2, y2, color, 2.f);
	}
}


void pdraw_gles2hud_draw_framing_grid(const struct pdraw_gles2hud *self,
				      const struct pdraw_rect *render_pos,
				      const struct pdraw_rect *content_pos,
				      const std::array<float, 4> &color)
{
	float x_left;
	float x_right;
	float x_third;
	float y_top;
	float y_bottom;
	float y_third;

	x_left = static_cast<float>(content_pos->x) /
			 static_cast<float>(render_pos->width) * 2.f -
		 1.f;
	x_right = (static_cast<float>(content_pos->x + content_pos->width) +
		   0.5f) / static_cast<float>(render_pos->width) *
			  2.f -
		  1.f;
	x_third = (x_right - x_left) / 3.f;
	y_top = 1.f - static_cast<float>(content_pos->y) /
			      static_cast<float>(render_pos->height) * 2.f;
	y_bottom = 1.f -
		   (static_cast<float>(content_pos->y + content_pos->height) +
		    0.5f) / static_cast<float>(render_pos->height) *
			   2.f;
	y_third = (y_top - y_bottom) / 3.f;

	if (x_left != -1.f) {
		pdraw_gles2hud_draw_line(
			self, x_left, y_top, x_left, y_bottom, color, 1.f);
	}
	pdraw_gles2hud_draw_line(self,
				 x_left + x_third,
				 y_top,
				 x_left + x_third,
				 y_bottom,
				 color,
				 1.f);
	pdraw_gles2hud_draw_line(self,
				 x_right - x_third,
				 y_top,
				 x_right - x_third,
				 y_bottom,
				 color,
				 1.f);
	if (x_right != 1.f) {
		pdraw_gles2hud_draw_line(
			self, x_right, y_top, x_right, y_bottom, color, 1.f);
	}
	if (y_top != 1.f) {
		pdraw_gles2hud_draw_line(
			self, x_left, y_top, x_right, y_top, color, 1.f);
	}
	pdraw_gles2hud_draw_line(self,
				 x_left,
				 y_top - y_third,
				 x_right,
				 y_top - y_third,
				 color,
				 1.f);
	pdraw_gles2hud_draw_line(self,
				 x_left,
				 y_bottom + y_third,
				 x_right,
				 y_bottom + y_third,
				 color,
				 1.f);
	if (y_bottom != -1.f) {
		pdraw_gles2hud_draw_line(
			self, x_left, y_bottom, x_right, y_bottom, color, 1.f);
	}
}


void pdraw_gles2hud_draw_histograms(
	const struct pdraw_gles2hud *self,
	const struct pdraw_video_frame_extra *frame_extra)
{
	unsigned int i;
	unsigned int j;
	std::array<float, 4> color_background = {0.0f, 0.0f, 0.0f, 0.2f};
	std::array<float, 4> color_white = {1.0f, 1.0f, 1.0f, 0.4f};
	std::array<std::array<float, 4>, 3> color = {{
		{1.0f, 0.0f, 0.0f, 1.0f},
		{0.0f, 1.0f, 0.0f, 1.0f},
		{0.0f, 0.0f, 1.0f, 1.0f},
	}};
	std::array<float, 4> cur_color = {0.0f, 0.0f, 0.0f, 1.0f};
	float width = 0.333f * 2.0f * self->ratio_w;
	float height = 0.333f * self->ratio_h;
	float offset_x = (1.0f - 0.166f) * self->ratio_w - width;
	float offset_y1 = (1.0f - 0.166f) * self->ratio_h - height;
	float offset_y2 =
		(1.0f - 0.166f) * self->ratio_h - height * 2.f - 0.04f;
	float x;
	float y1;
	float y2;
	float bin_width;
	std::array<float, 3> val;
	float prev_val;
	std::array<int, 3> idx;

	if (frame_extra->histogram[PDRAW_HISTOGRAM_CHANNEL_LUMA]) {
		pdraw_gles2hud_draw_filled_rect(self,
						offset_x,
						offset_y1,
						offset_x + width,
						offset_y1 + height,
						color_background);
		bin_width =
			width / static_cast<float>(
					frame_extra->histogram_len
						[PDRAW_HISTOGRAM_CHANNEL_LUMA]);
		for (i = 0;
		     i <
		     frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_LUMA];
		     i++) {
			y1 = offset_y1 +
			     frame_extra->histogram
					     [PDRAW_HISTOGRAM_CHANNEL_LUMA][i] *
				     height;
			x = offset_x + static_cast<float>(i) * bin_width;
			pdraw_gles2hud_draw_filled_rect(self,
							x,
							offset_y1,
							x + bin_width,
							y1,
							color_white);
		}
	}
	if (frame_extra->histogram[PDRAW_HISTOGRAM_CHANNEL_RED] &&
	    frame_extra->histogram[PDRAW_HISTOGRAM_CHANNEL_GREEN] &&
	    frame_extra->histogram[PDRAW_HISTOGRAM_CHANNEL_BLUE] &&
	    (frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_RED] ==
	     frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_GREEN]) &&
	    (frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_RED] ==
	     frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_BLUE])) {
		pdraw_gles2hud_draw_filled_rect(self,
						offset_x,
						offset_y2,
						offset_x + width,
						offset_y2 + height,
						color_background);
		bin_width =
			width / static_cast<float>(
					frame_extra->histogram_len
						[PDRAW_HISTOGRAM_CHANNEL_RED]);
		for (i = 0;
		     i <
		     frame_extra->histogram_len[PDRAW_HISTOGRAM_CHANNEL_RED];
		     i++) {
			x = offset_x + static_cast<float>(i) * bin_width;
			val[0] =
				frame_extra
					->histogram[PDRAW_HISTOGRAM_CHANNEL_RED]
						   [i];
			val[1] = frame_extra->histogram
					 [PDRAW_HISTOGRAM_CHANNEL_GREEN][i];
			val[2] = frame_extra->histogram
					 [PDRAW_HISTOGRAM_CHANNEL_BLUE][i];
			if (val[0] < val[1]) {
				if (val[0] < val[2]) {
					idx[0] = 0;
					if (val[1] < val[2]) {
						idx[1] = 1;
						idx[2] = 2;
					} else {
						idx[1] = 2;
						idx[2] = 1;
					}
				} else {
					idx[0] = 2;
					idx[1] = 0;
					idx[2] = 1;
				}
			} else {
				if (val[1] < val[2]) {
					idx[0] = 1;
					if (val[0] < val[2]) {
						idx[1] = 0;
						idx[2] = 2;
					} else {
						idx[1] = 2;
						idx[2] = 0;
					}
				} else {
					idx[0] = 2;
					idx[1] = 1;
					idx[2] = 0;
				}
			}
			for (j = 0, prev_val = 0.f; j < 3; j++) {
				if (val[idx[j]] <= 0.f)
					continue;
				cur_color[0] = 0.f;
				cur_color[1] = 0.f;
				cur_color[2] = 0.f;
				for (unsigned int k = j; k < 3; k++) {
					cur_color[0] += color[idx[k]][0];
					cur_color[1] += color[idx[k]][1];
					cur_color[2] += color[idx[k]][2];
				}
				y1 = offset_y2 + prev_val * height;
				y2 = offset_y2 + val[idx[j]] * height;
				prev_val = val[idx[j]];
				cur_color[3] = 0.4f;
				pdraw_gles2hud_draw_filled_rect(self,
								x,
								y1,
								x + bin_width,
								y2,
								cur_color);
			}
		}
	}
}
