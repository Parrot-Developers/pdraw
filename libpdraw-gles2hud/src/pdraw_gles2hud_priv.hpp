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

#pragma once

#include <errno.h>
#include <math.h>
#include <stdio.h>

#define ULOG_TAG pdraw_gles2hud
#include <pdraw/pdraw_gles2hud.h>
#include <ulog.h>
#include <video-metadata/vmeta.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>


/* OpenGL headers */
#if defined(__APPLE__)
#	include <TargetConditionals.h>
#	if TARGET_OS_IPHONE
#		include <OpenGLES/ES2/gl.h>
#		include <OpenGLES/ES2/glext.h>
#	else
#		define GL_GLEXT_PROTOTYPES
#		include <OpenGL/OpenGL.h>
#		include <OpenGL/gl.h>
#		include <OpenGL/glext.h>
#	endif
#elif defined(_WIN32)
#	include <epoxy/gl.h>
#else
#	include <GLES2/gl2.h>
#endif


/* Uncomment to enable extra GL error checking */
/* #define CHECK_GL_ERRORS */
#if defined(CHECK_GL_ERRORS)
#	warning CHECK_GL_ERRORS is enabled
#	include <assert.h>
#	define GLCHK(X)                                                       \
		do {                                                           \
			GLenum err = GL_NO_ERROR;                              \
			X;                                                     \
			while ((err = glGetError())) {                         \
				ULOGE("%s:%d: GL error 0x%x in " #X,           \
				      __func__,                                \
				      __LINE__,                                \
				      err);                                    \
				assert(err == GL_NO_ERROR);                    \
			}                                                      \
		} while (0)
#else /* CHECK_GL_ERRORS */
#	define GLCHK(X) X
#endif /* CHECK_GL_ERRORS */


#define RAD_TO_DEG (57.295779513f)

/* Default FOV from Bebop 2 */
#define PDRAW_GLES2HUD_DEFAULT_HFOV (78.)
#define PDRAW_GLES2HUD_DEFAULT_VFOV (49.)


static constexpr int hud_icons_width = 212;
static constexpr int hud_icons_height = 210;
extern const std::array<uint8_t, hud_icons_width * hud_icons_height> hud_icons;


namespace profont_36 {

struct glyph_info {
	struct { /* pixel oriented data */
		int u;
		int v;
		int width;
		int height;
		int advance;
		int offx;
		int offy;
	} pix;
	struct { /* normalized data */
		float u;
		float v; /* position in the map in normalized coords */
		float width;
		float height;
		float advance;
		float offx;
		float offy;
	} norm;
};

struct file_header {
	int texwidth;
	int texheight;
	struct {
		int ascent;
		int descent;
		int linegap;
	} pix;
	struct {
		float ascent;
		float descent;
		float linegap;
	} norm;
	std::array<glyph_info, 256> glyphs;
};

extern const file_header font;
static constexpr int image_width = 256;
static constexpr int image_height = 202;
extern const std::array<unsigned char, image_width * image_height> image;

} /* namespace profont_36 */


enum class TextAlign {
	LEFT = 0,
	TOP = 0,
	CENTER = 1,
	MIDDLE = 1,
	RIGHT = 2,
	BOTTOM = 2,
};


struct pdraw_gles2hud {
	struct pdraw_gles2hud_config config;

	unsigned int first_texunit;

	float ratio_w;
	float ratio_h;
	float aspect_ratio;
	float h_fov;
	float v_fov;

	/* Shape drawing */
	GLint program;
	GLint position_handle;
	GLint transform_matrix_handle;
	GLint color_handle;

	/* Texture drawing */
	GLint tex_program;
	GLint tex_uniform_sampler;
	GLint tex_position_handle;
	GLint tex_texcoord_handle;
	GLint tex_transform_matrix_handle;
	GLint tex_color_handle;
	GLuint icons_texture;
	unsigned int icons_texunit;
	GLuint text_texture;
	unsigned int text_texunit;
};


int pdraw_gles2hud_create_programs(struct pdraw_gles2hud *self);


void pdraw_gles2hud_draw_line(const struct pdraw_gles2hud *self,
			      float x1,
			      float y1,
			      float x2,
			      float y2,
			      const std::array<float, 4> &color,
			      float line_width);


void pdraw_gles2hud_draw_rect(const struct pdraw_gles2hud *self,
			      float x1,
			      float y1,
			      float x2,
			      float y2,
			      const std::array<float, 4> &color,
			      float line_width);


void pdraw_gles2hud_draw_filled_rect(const struct pdraw_gles2hud *self,
				     float x1,
				     float y1,
				     float x2,
				     float y2,
				     const std::array<float, 4> &color);


void pdraw_gles2hud_draw_arc(const struct pdraw_gles2hud *self,
			     float cx,
			     float cy,
			     float rx,
			     float ry,
			     float start_angle,
			     float span_angle,
			     int num_segments,
			     const std::array<float, 4> &color,
			     float line_width);


void pdraw_gles2hud_draw_ellipse(const struct pdraw_gles2hud *self,
				 float cx,
				 float cy,
				 float rx,
				 float ry,
				 int num_segments,
				 const std::array<float, 4> &color,
				 float line_width);


void pdraw_gles2hud_draw_filled_ellipse(const struct pdraw_gles2hud *self,
					float cx,
					float cy,
					float rx,
					float ry,
					int num_segments,
					const std::array<float, 4> &color);


void pdraw_gles2hud_draw_icon(const struct pdraw_gles2hud *self,
			      int index,
			      float x,
			      float y,
			      float size,
			      float scalew,
			      float scaleh,
			      const std::array<float, 4> &color);


template <typename... Args> std::string fmtstr(const char *fmt, Args... args)
{
	std::array<char, 64> buf;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
	snprintf(buf.data(), buf.size(), fmt, args...);
#pragma GCC diagnostic pop
	return buf.data();
}


void pdraw_gles2hud_get_text_dimensions(const std::string &str,
					float size,
					float scalew,
					float scaleh,
					float *width,
					float *height);


void pdraw_gles2hud_draw_text(const struct pdraw_gles2hud *self,
			      const std::string &str,
			      float x,
			      float y,
			      float size,
			      float scalew,
			      float scaleh,
			      TextAlign halign,
			      TextAlign valign,
			      const std::array<float, 4> &color);


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
				 const std::array<float, 4> &critical_color);


void pdraw_gles2hud_draw_artificial_horizon(const struct pdraw_gles2hud *self,
					    const struct vmeta_euler *drone,
					    const struct vmeta_euler *frame,
					    const std::array<float, 4> &color);


void pdraw_gles2hud_draw_roll(const struct pdraw_gles2hud *self,
			      float drone_roll,
			      const std::array<float, 4> &color);


void pdraw_gles2hud_draw_heading(const struct pdraw_gles2hud *self,
				 float drone_yaw,
				 float horizontal_speed,
				 float speed_psi,
				 const std::array<float, 4> &color);


void pdraw_gles2hud_draw_altitude(const struct pdraw_gles2hud *self,
				  double altitude,
				  float ground_distance,
				  float down_speed,
				  const std::array<float, 4> &color);


void pdraw_gles2hud_draw_speed(const struct pdraw_gles2hud *self,
			       float horizontal_speed,
			       const std::array<float, 4> &color);


void pdraw_gles2hud_draw_controller_radar(const struct pdraw_gles2hud *self,
					  double distance,
					  float controller_radar_angle,
					  const std::array<float, 4> &color);


void pdraw_gles2hud_draw_record_timeline(const struct pdraw_gles2hud *self,
					 uint64_t current_time,
					 uint64_t duration,
					 const std::array<float, 4> &color);


void pdraw_gles2hud_draw_cot(const struct pdraw_gles2hud *self,
			     float x,
			     float y,
			     const std::array<float, 4> &color);


void pdraw_gles2hud_draw_flight_path_vector(const struct pdraw_gles2hud *self,
					    const struct vmeta_euler *frame,
					    float speed_theta,
					    float speed_psi,
					    const std::array<float, 4> &color);


void pdraw_gles2hud_draw_framing_grid(const struct pdraw_gles2hud *self,
				      const struct pdraw_rect *render_pos,
				      const struct pdraw_rect *content_pos,
				      const std::array<float, 4> &color);


void pdraw_gles2hud_draw_histograms(
	const struct pdraw_gles2hud *self,
	const struct pdraw_video_frame_extra *frame_extra);


static inline void pdraw_gles2hud_friendly_time_from_us(uint64_t time,
							unsigned int *hrs,
							unsigned int *min,
							unsigned int *sec,
							unsigned int *msec)
{
	unsigned int _hrs =
		(unsigned int)((time + 500) / 1000 / 60 / 60) / 1000;
	unsigned int _min =
		(unsigned int)((time + 500) / 1000 / 60 - _hrs * 60000) / 1000;
	unsigned int _sec = (unsigned int)((time + 500) / 1000 -
					   _hrs * 60 * 60000 - _min * 60000) /
			    1000;
	auto _msec = (unsigned int)((time + 500) / 1000 - _hrs * 60 * 60000 -
				    _min * 60000 - _sec * 1000);
	if (hrs)
		*hrs = _hrs;
	if (min)
		*min = _min;
	if (sec)
		*sec = _sec;
	if (msec)
		*msec = _msec;
}


static inline void pdraw_gles2hud_coords_distance_and_bearing(double latitude1,
							      double longitude1,
							      double latitude2,
							      double longitude2,
							      double *distance,
							      double *bearing)
{
	/* http://www.igismap.com/haversine-formula-calculate-geographic-distance-earth/
	 */
	/* http://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
	 */
	double a;
	double c;
	double d;
	double x;
	double y;
	double b;
	double r = 6371000.; /* earth radius */
	double lat1 = latitude1 * M_PI / 180.;
	double lon1 = longitude1 * M_PI / 180.;
	double lat2 = latitude2 * M_PI / 180.;
	double lon2 = longitude2 * M_PI / 180.;
	a = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2) +
	    cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) *
		    sin((lon2 - lon1) / 2);
	c = 2 * atan2(sqrt(a), sqrt(1. - a));
	d = r * c;
	x = cos(lat2) * sin(lon2 - lon1);
	y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
	b = atan2(x, y);
	if (distance)
		*distance = d;
	if (bearing)
		*bearing = b;
}
