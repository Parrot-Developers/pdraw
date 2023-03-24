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
ULOG_DECLARE_TAG(pdraw_gles2hud);

/* used to test the radar on records */
#if 0
#	define DEBUG_RADAR
#endif


enum drone_model {
	/* Unknown drone model */
	DRONE_MODEL_UNKNOWN = 0,

	/* Parrot Bebop */
	DRONE_MODEL_BEBOP,

	/* Parrot Bebop 2 */
	DRONE_MODEL_BEBOP2,

	/* Parrot Disco */
	DRONE_MODEL_DISCO,

	/* Parrot Bluegrass */
	DRONE_MODEL_BLUEGRASS,

	/* Parrot Anafi */
	DRONE_MODEL_ANAFI,

	/* Parrot Anafi Thermal */
	DRONE_MODEL_ANAFI_THERMAL,
};


static const int drone_model_icon_index[] = {
	2,
	0,
	2,
	1,
	2,
	2,
	2,
};

static const char *heading_str[] = {
	".N.",
	"NW",
	"W",
	"SW",
	"S",
	"SE",
	"E",
	"NE",
};

static const char *flying_state_str[] = {
	"LANDED",
	"TAKING OFF",
	"HOVERING",
	"FLYING",
	"LANDING",
	"EMERGENCY",
};

static const char *piloting_mode_str[] = {
	"MANUAL",
	"RETURN HOME",
	"FLIGHT PLAN",
	"FOLLOW ME",
};

static const float color_green[4] = {0.0f, 0.9f, 0.0f, 1.0f};

static const float color_dark_green[4] = {0.0f, 0.5f, 0.0f, 1.0f};

static const float color_black_alpha[4] = {0.0f, 0.0f, 0.0f, 0.66f};


static enum drone_model
get_drone_model(const struct vmeta_session *session_meta)
{
	if (strcmp(session_meta->model_id, "0901") == 0)
		return DRONE_MODEL_BEBOP;
	else if (strcmp(session_meta->model_id, "090c") == 0)
		return DRONE_MODEL_BEBOP2;
	else if (strcmp(session_meta->model_id, "090e") == 0)
		return DRONE_MODEL_DISCO;
	else if (strcmp(session_meta->model_id, "0916") == 0)
		return DRONE_MODEL_BLUEGRASS;
	else if (strcmp(session_meta->model_id, "0914") == 0)
		return DRONE_MODEL_ANAFI;
	else if (strcmp(session_meta->model_id, "0919") == 0)
		return DRONE_MODEL_ANAFI_THERMAL;
	else if (strcmp(session_meta->model, "Bebop") == 0)
		return DRONE_MODEL_BEBOP;
	else if (strcmp(session_meta->model, "Bebop 2") == 0)
		return DRONE_MODEL_BEBOP2;
	else if (strcmp(session_meta->model, "Disco") == 0)
		return DRONE_MODEL_DISCO;
	else if (strcmp(session_meta->model, "Bluegrass") == 0)
		return DRONE_MODEL_BLUEGRASS;
	else if (strcmp(session_meta->model, "ANAFI") == 0)
		return DRONE_MODEL_ANAFI;
	else if (strcmp(session_meta->model, "Anafi") == 0)
		return DRONE_MODEL_ANAFI;
	else if (strcmp(session_meta->model, "AnafiThermal") == 0)
		return DRONE_MODEL_ANAFI_THERMAL;
	else if (strcmp(session_meta->friendly_name, "Parrot Bebop") == 0)
		return DRONE_MODEL_BEBOP;
	else if (strcmp(session_meta->friendly_name, "Parrot Bebop 2") == 0)
		return DRONE_MODEL_BEBOP2;
	else if (strcmp(session_meta->friendly_name, "Parrot Disco") == 0)
		return DRONE_MODEL_DISCO;
	else
		return DRONE_MODEL_UNKNOWN;
}


int pdraw_gles2hud_new(const struct pdraw_gles2hud_config *config,
		       struct pdraw_gles2hud **hud)
{
	int res, err;
	struct pdraw_gles2hud *self;

	if (config == nullptr)
		return -EINVAL;
	if (hud == nullptr)
		return -EINVAL;

	self = (struct pdraw_gles2hud *)calloc(1, sizeof(*self));
	if (self == nullptr)
		return -ENOMEM;

	self->config = *config;
	if (self->config.central_zone_size <= 0.) {
		self->config.central_zone_size =
			PDRAW_GLES2HUD_DEFAULT_CENTRAL_ZONE_SIZE;
	}
	if (self->config.heading_zone_v_offset <= 0.) {
		self->config.heading_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_HEADING_ZONE_V_OFFSET;
	}
	if (self->config.roll_zone_v_offset <= 0.) {
		self->config.roll_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_ROLL_ZONE_V_OFFSET;
	}
	if (self->config.vu_meter_zone_h_offset <= 0.) {
		self->config.vu_meter_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_VU_METER_ZONE_H_OFFSET;
	}
	if (self->config.vu_meter_v_interval <= 0.) {
		self->config.vu_meter_v_interval =
			PDRAW_GLES2HUD_DEFAULT_VU_METER_V_INTERVAL;
	}
	if (self->config.right_zone_h_offset <= 0.) {
		self->config.right_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_RIGHT_ZONE_H_OFFSET;
	}
	if (self->config.radar_zone_h_offset <= 0.) {
		self->config.radar_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_H_OFFSET;
	}
	if (self->config.radar_zone_v_offset <= 0.) {
		self->config.radar_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_V_OFFSET;
	}
	if (self->config.text_size <= 0.) {
		self->config.text_size = PDRAW_GLES2HUD_DEFAULT_TEXT_SIZE;
	}
	if (self->config.small_icon_size <= 0.) {
		self->config.small_icon_size =
			PDRAW_GLES2HUD_DEFAULT_SMALL_ICON_SIZE;
	}
	if (self->config.medium_icon_size <= 0.) {
		self->config.medium_icon_size =
			PDRAW_GLES2HUD_DEFAULT_MEDIUM_ICON_SIZE;
	}
	if (self->config.scale <= 0.) {
		self->config.scale = PDRAW_GLES2HUD_DEFAULT_SCALE;
	}

	self->ratio_w = 1.;
	self->ratio_h = 1.;
	self->aspect_ratio = 1.;
	self->h_fov = 0.;
	self->v_fov = 0.;

	res = pdraw_gles2hud_create_programs(self);
	if (res < 0) {
		ULOG_ERRNO("pdraw_gles2hud_create_programs", -res);
		goto error;
	}

	*hud = self;
	return 0;

error:
	err = pdraw_gles2hud_destroy(self);
	if (err < 0)
		ULOG_ERRNO("pdraw_gles2hud_destroy", -err);
	return res;
}


int pdraw_gles2hud_destroy(struct pdraw_gles2hud *self)
{
	if (self == nullptr)
		return 0;

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

	free(self);

	return 0;
}


int pdraw_gles2hud_get_config(struct pdraw_gles2hud *self,
			      struct pdraw_gles2hud_config *config)
{
	if (self == nullptr)
		return -EINVAL;
	if (config == nullptr)
		return -EINVAL;

	*config = self->config;

	return 0;
}


int pdraw_gles2hud_set_config(struct pdraw_gles2hud *self,
			      const struct pdraw_gles2hud_config *config)
{
	if (self == nullptr)
		return -EINVAL;
	if (config == nullptr)
		return -EINVAL;

	self->config = *config;
	if (self->config.central_zone_size <= 0.) {
		self->config.central_zone_size =
			PDRAW_GLES2HUD_DEFAULT_CENTRAL_ZONE_SIZE;
	}
	if (self->config.heading_zone_v_offset <= 0.) {
		self->config.heading_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_HEADING_ZONE_V_OFFSET;
	}
	if (self->config.roll_zone_v_offset <= 0.) {
		self->config.roll_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_ROLL_ZONE_V_OFFSET;
	}
	if (self->config.vu_meter_zone_h_offset <= 0.) {
		self->config.vu_meter_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_VU_METER_ZONE_H_OFFSET;
	}
	if (self->config.vu_meter_v_interval <= 0.) {
		self->config.vu_meter_v_interval =
			PDRAW_GLES2HUD_DEFAULT_VU_METER_V_INTERVAL;
	}
	if (self->config.right_zone_h_offset <= 0.) {
		self->config.right_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_RIGHT_ZONE_H_OFFSET;
	}
	if (self->config.radar_zone_h_offset <= 0.) {
		self->config.radar_zone_h_offset =
			PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_H_OFFSET;
	}
	if (self->config.radar_zone_v_offset <= 0.) {
		self->config.radar_zone_v_offset =
			PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_V_OFFSET;
	}
	if (self->config.text_size <= 0.) {
		self->config.text_size = PDRAW_GLES2HUD_DEFAULT_TEXT_SIZE;
	}
	if (self->config.small_icon_size <= 0.) {
		self->config.small_icon_size =
			PDRAW_GLES2HUD_DEFAULT_SMALL_ICON_SIZE;
	}
	if (self->config.medium_icon_size <= 0.) {
		self->config.medium_icon_size =
			PDRAW_GLES2HUD_DEFAULT_MEDIUM_ICON_SIZE;
	}
	if (self->config.scale <= 0.) {
		self->config.scale = PDRAW_GLES2HUD_DEFAULT_SCALE;
	}

	return 0;
}


static void pdraw_gles2hud_get_fov(struct pdraw_gles2hud *self,
				   const struct vmeta_session *session_meta,
				   struct vmeta_frame *frame_meta)
{
	if ((frame_meta) && (frame_meta->type == VMETA_FRAME_TYPE_V3)) {
		self->h_fov = frame_meta->v3.base.picture_hfov;
		self->v_fov = frame_meta->v3.base.picture_vfov;
	} else if ((session_meta) && (session_meta->picture_fov.has_horz) &&
		   (session_meta->picture_fov.has_horz)) {
		self->h_fov = session_meta->picture_fov.horz;
		self->v_fov = session_meta->picture_fov.vert;
	} else {
		self->h_fov = PDRAW_GLES2HUD_DEFAULT_HFOV;
		self->v_fov = PDRAW_GLES2HUD_DEFAULT_VFOV;
	}
	self->h_fov *= M_PI / 180.;
	self->v_fov *= M_PI / 180.;
}


static int pdraw_gles2hud_render_piloting(
	struct pdraw_gles2hud *self,
	const struct pdraw_rect *render_pos,
	const struct pdraw_rect *content_pos,
	const float view_proj_mat[16],
	const struct pdraw_media_info *media_info,
	struct vmeta_frame *frame_meta,
	const struct pdraw_video_frame_extra *frame_extra,
	const struct pdraw_gles2hud_controller_meta *ctrl_meta,
	const struct pdraw_gles2hud_drone_meta *drone_meta)
{
	int i, j, steps, angle_deg;
	float cy, delta_x, delta_y, angle;
	Eigen::Matrix4f xform_mat;
	Eigen::Matrix4f model_mat;
	Eigen::Matrix4f view_proj;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
			view_proj(i, j) = view_proj_mat[i * 4 + j];
	}

	if (self == nullptr)
		return -EINVAL;
	if ((render_pos == nullptr) || (render_pos->width == 0) ||
	    (render_pos->height == 0))
		return -EINVAL;
	if ((content_pos == nullptr) || (content_pos->width == 0) ||
	    (content_pos->height == 0))
		return -EINVAL;
	if (media_info == nullptr)
		return -EINVAL;
	if (frame_meta == nullptr)
		return -EINVAL;
	if (frame_extra == nullptr)
		return -EINVAL;
	if (ctrl_meta == nullptr)
		return -EINVAL;
	if (drone_meta == nullptr)
		return -EINVAL;

	uint8_t battery_percentage;
	vmeta_frame_get_battery_percentage(frame_meta, &battery_percentage);
	int8_t wifi_rssi;
	vmeta_frame_get_wifi_rssi(frame_meta, &wifi_rssi);
	enum vmeta_flying_state flying_state;
	vmeta_frame_get_flying_state(frame_meta, &flying_state);
	enum vmeta_piloting_mode piloting_mode;
	vmeta_frame_get_piloting_mode(frame_meta, &piloting_mode);
	enum drone_model drone_model =
		get_drone_model(media_info->session_meta);
	int drone_model_icon = drone_model_icon_index[drone_model];

	/* Drone location and ground distance */
	struct vmeta_location location;
	vmeta_frame_get_location(frame_meta, &location);
	double ground_distance;
	vmeta_frame_get_ground_distance(frame_meta, &ground_distance);
	if ((drone_model == DRONE_MODEL_DISCO) && (location.valid) &&
	    (media_info->session_meta->takeoff_loc.valid)) {
		ground_distance = location.altitude_egm96amsl -
				  media_info->session_meta->takeoff_loc
					  .altitude_egm96amsl;
	}

	/* Drone speeds */
	struct vmeta_ned speed;
	vmeta_frame_get_speed_ned(frame_meta, &speed);
	float air_speed;
	vmeta_frame_get_air_speed(frame_meta, &air_speed);
	float horizontal_speed =
		sqrtf(speed.north * speed.north + speed.east * speed.east);
	float speed_psi = atan2f(speed.east, speed.north);
	/* UNUSED
	 * float speed_rho = sqrtf(speed.north * speed.north +
	 * 			speed.east * speed.east +
	 * 			speed.down * speed.down);
	 * float speed_theta = M_PI / 2 - acosf(speed.down / speed_rho); */

	/* Drone attitude and frame orientation */
	struct vmeta_euler drone_attitude;
	vmeta_frame_get_drone_euler(frame_meta, &drone_attitude);
	struct vmeta_euler frame_orientation;
	vmeta_frame_get_frame_euler(frame_meta, &frame_orientation);
	int heading_int = ((int)(drone_attitude.psi * RAD_TO_DEG) + 360) % 360;

	/* Picture field of view */
	pdraw_gles2hud_get_fov(self, media_info->session_meta, frame_meta);

	/* Distace to take-off */
	double takeoff_distance = 0.;
	double takeoff_bearing = 0.;
	double takeoff_elevation = 0.;
	if ((location.valid) && (media_info->session_meta->takeoff_loc.valid)) {
		pdraw_gles2hud_coords_distance_and_bearing(
			location.latitude,
			location.longitude,
			media_info->session_meta->takeoff_loc.latitude,
			media_info->session_meta->takeoff_loc.longitude,
			&takeoff_distance,
			&takeoff_bearing);
		double alt_diff = 0.;
		if (!std::isnan(media_info->session_meta->takeoff_loc
					.altitude_wgs84ellipsoid) &&
		    !std::isnan(location.altitude_wgs84ellipsoid != NAN)) {
			alt_diff = media_info->session_meta->takeoff_loc
					   .altitude_wgs84ellipsoid -
				   location.altitude_wgs84ellipsoid;
		} else if (!std::isnan(media_info->session_meta->takeoff_loc
					       .altitude_egm96amsl) &&
			   !std::isnan(location.altitude_egm96amsl)) {
			alt_diff = media_info->session_meta->takeoff_loc
					   .altitude_egm96amsl -
				   location.altitude_egm96amsl;
		}
		takeoff_elevation = atan2(alt_diff, takeoff_distance);
	}

	/* Distace to pilot */
	double self_distance = 0.;
	double self_bearing = 0.;
	double self_elevation = 0.;
	if ((location.valid) && (ctrl_meta->location.valid)) {
		pdraw_gles2hud_coords_distance_and_bearing(
			location.latitude,
			location.longitude,
			ctrl_meta->location.latitude,
			ctrl_meta->location.longitude,
			&self_distance,
			&self_bearing);
		double alt_diff = 0.;
		if (!std::isnan(ctrl_meta->location.altitude_wgs84ellipsoid) &&
		    !std::isnan(location.altitude_wgs84ellipsoid)) {
			alt_diff = ctrl_meta->location.altitude_wgs84ellipsoid -
				   location.altitude_wgs84ellipsoid;
		} else if (!std::isnan(
				   ctrl_meta->location.altitude_egm96amsl) &&
			   !std::isnan(location.altitude_egm96amsl)) {
			alt_diff = ctrl_meta->location.altitude_egm96amsl -
				   location.altitude_egm96amsl;
		}
		self_elevation = atan2(alt_diff, self_distance);
	}

	/* Controller orientation */
	struct vmeta_euler ctrl_orientation;
	memset(&ctrl_orientation, 0, sizeof(ctrl_orientation));
#ifdef DEBUG_RADAR /* used to test the radar on records */
	int ctrl_orientation_valid = 1;
#else /* DEBUG_RADAR */
	int ctrl_orientation_valid = ctrl_meta->has_quat;
	if (ctrl_orientation_valid) {
		vmeta_quat_to_euler(&ctrl_meta->quat, &ctrl_orientation);
	}
#endif /* DEBUG_RADAR */

	self->ratio_w = (float)content_pos->width / render_pos->width *
			self->config.scale;
	self->ratio_h = (float)content_pos->height / render_pos->height *
			self->config.scale;
	self->aspect_ratio = (float)render_pos->width / render_pos->height;

	GLCHK(glUseProgram(self->program));

	GLCHK(glEnableVertexAttribArray(self->position_handle));
	GLCHK(glEnableVertexAttribArray(self->color_handle));

	GLCHK(glUniformMatrix4fv(
		self->transform_matrix_handle, 1, false, view_proj_mat));

#if 0
	if (horizontal_speed >= 0.2)
		pdraw_gles2hud_draw_flight_path_vector(self, &frame_orientation,
						       speed_theta, speed_psi,
						       color_green);
#endif

	/* Draw instruments */
	pdraw_gles2hud_draw_artificial_horizon(
		self, &drone_attitude, &frame_orientation, color_green);
	pdraw_gles2hud_draw_roll(self, drone_attitude.phi, color_green);
	pdraw_gles2hud_draw_heading(self,
				    drone_attitude.psi,
				    horizontal_speed,
				    speed_psi,
				    color_green);
	double altitude = 0.;
	if (!std::isnan(location.altitude_wgs84ellipsoid))
		altitude = location.altitude_wgs84ellipsoid;
	else if (!std::isnan(location.altitude_egm96amsl))
		altitude = location.altitude_egm96amsl;
	pdraw_gles2hud_draw_altitude(
		self, altitude, ground_distance, speed.down, color_green);
	pdraw_gles2hud_draw_speed(self, horizontal_speed, color_green);
#ifdef DEBUG_RADAR /* used to test the radar on records */
	if ((location.valid) && (session_meta->takeoff_loc.valid) &&
	    (ctrl_orientation_valid) && (ctrl_meta->radar_angle > 0.)) {
		pdraw_gles2hud_draw_controller_radar(self,
						     takeoff_distance,
						     takeoff_bearing,
						     ctrl_orientation.psi,
						     drone_attitude.psi,
						     ctrl_meta->radar_angle *
							     M_PI / 180.,
						     color_green);
	}
#else /* DEBUG_RADAR */
	if ((location.valid) && (ctrl_meta->location.valid) &&
	    (ctrl_orientation_valid) && (ctrl_meta->radar_angle > 0.)) {
		pdraw_gles2hud_draw_controller_radar(self,
						     self_distance,
						     self_bearing,
						     ctrl_orientation.psi,
						     drone_attitude.psi,
						     ctrl_meta->radar_angle *
							     M_PI / 180.,
						     color_green);
	}
#endif /* DEBUG_RADAR */
	if ((media_info->duration > 0) &&
	    (media_info->duration != (uint64_t)-1)) {
		pdraw_gles2hud_draw_record_timeline(self,
						    frame_extra->play_timestamp,
						    media_info->duration,
						    color_green);
	} else if (media_info->playback_type == PDRAW_PLAYBACK_TYPE_LIVE) {
		pdraw_gles2hud_draw_recording_status(
			self, drone_meta->recording_duration, color_green);
	}
	pdraw_gles2hud_draw_vumeter(self,
				    self->config.vu_meter_zone_h_offset,
				    -self->config.vu_meter_v_interval,
				    0.05,
				    battery_percentage,
				    0.,
				    100.,
				    0.,
				    20.,
				    color_green,
				    color_dark_green);
	pdraw_gles2hud_draw_vumeter(self,
				    self->config.vu_meter_zone_h_offset,
				    0.0,
				    0.05,
				    wifi_rssi,
				    -90.,
				    -20.,
				    -90.,
				    -70.,
				    color_green,
				    color_dark_green);
	pdraw_gles2hud_draw_vumeter(self,
				    self->config.vu_meter_zone_h_offset,
				    self->config.vu_meter_v_interval,
				    0.05,
				    location.sv_count,
				    0.,
				    30.,
				    0.,
				    5.,
				    color_green,
				    color_dark_green);

	GLCHK(glDisableVertexAttribArray(self->position_handle));
	GLCHK(glDisableVertexAttribArray(self->color_handle));

	GLCHK(glEnable(GL_BLEND));
	GLCHK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

	GLCHK(glUseProgram(self->tex_program));
	GLCHK(glUniformMatrix4fv(
		self->tex_transform_matrix_handle, 1, false, view_proj_mat));

	GLCHK(glActiveTexture(GL_TEXTURE0 + self->icons_texunit));
	GLCHK(glBindTexture(GL_TEXTURE_2D, self->icons_texture));
	GLCHK(glUniform1i(self->tex_uniform_sampler, self->icons_texunit));
	GLCHK(glEnableVertexAttribArray(self->tex_position_handle));
	GLCHK(glEnableVertexAttribArray(self->tex_texcoord_handle));
	GLCHK(glEnableVertexAttribArray(self->tex_color_handle));

	pdraw_gles2hud_draw_icon(
		self,
		3,
		self->config.vu_meter_zone_h_offset * self->ratio_w,
		(-self->config.vu_meter_v_interval - 0.01) * self->ratio_h,
		self->config.small_icon_size,
		self->ratio_w,
		self->ratio_w * self->aspect_ratio,
		color_green);
	pdraw_gles2hud_draw_icon(self,
				 4,
				 self->config.vu_meter_zone_h_offset *
					 self->ratio_w,
				 (0.0 - 0.01) * self->ratio_h,
				 self->config.small_icon_size,
				 self->ratio_w,
				 self->ratio_w * self->aspect_ratio,
				 color_green);
	pdraw_gles2hud_draw_icon(
		self,
		5,
		self->config.vu_meter_zone_h_offset * self->ratio_w,
		(self->config.vu_meter_v_interval - 0.01) * self->ratio_h,
		self->config.small_icon_size,
		self->ratio_w,
		self->ratio_w * self->aspect_ratio,
		color_green);
	if (drone_model != DRONE_MODEL_UNKNOWN) {
		pdraw_gles2hud_draw_icon(self,
					 drone_model_icon,
					 0.0,
					 self->config.heading_zone_v_offset *
						 self->ratio_h,
					 self->config.small_icon_size,
					 self->ratio_w,
					 self->ratio_w * self->aspect_ratio,
					 color_green);
	}
	float friendly_name_x_offset =
		(self->config.vu_meter_zone_h_offset - 0.05) * self->ratio_w;
	if ((strlen(media_info->session_meta->friendly_name) > 0) &&
	    (drone_model != DRONE_MODEL_UNKNOWN)) {
		pdraw_gles2hud_draw_icon(
			self,
			drone_model_icon,
			friendly_name_x_offset + 0.025 * self->ratio_w,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.12 * self->ratio_w * self->aspect_ratio,
			self->config.medium_icon_size,
			self->ratio_w,
			self->ratio_w * self->aspect_ratio,
			color_green);
		friendly_name_x_offset += 0.06 * self->ratio_w;
	}

#ifdef DEBUG_RADAR /* used to test the radar on records */
	if ((location.valid) && (session_meta->takeoff_loc.valid) &&
	    (ctrl_orientation_valid)) {
#else /* DEBUG_RADAR */
	if ((location.valid) && (ctrl_meta->location.valid) &&
	    (ctrl_orientation_valid)) {
#endif /* DEBUG_RADAR */
		float x = self->config.radar_zone_h_offset * self->ratio_w;
		float y = self->config.radar_zone_v_offset * self->ratio_h;
		pdraw_gles2hud_draw_icon(self,
					 8,
					 x,
					 y,
					 self->config.small_icon_size,
					 self->ratio_w,
					 self->ratio_w * self->aspect_ratio,
					 color_green);
		if ((drone_model != DRONE_MODEL_UNKNOWN) &&
		    (takeoff_distance > 50.)) {
#ifdef DEBUG_RADAR /* used to test the radar on records */
			angle = M_PI / 2. + ctrl_orientation.psi -
				(takeoff_bearing + M_PI);
#else /* DEBUG_RADAR */
			angle = M_PI / 2. + ctrl_orientation.psi -
				(self_bearing + M_PI);
#endif /* DEBUG_RADAR */
			delta_x = x + 0.06 * cosf(angle) * self->ratio_w;
			delta_y = y + 0.06 * sinf(angle) * self->ratio_w *
					      self->aspect_ratio;
			angle = ctrl_orientation.psi - drone_attitude.psi;
			model_mat << cosf(angle), -sinf(angle), 0., delta_x,
				sinf(angle) * self->aspect_ratio,
				cosf(angle) * self->aspect_ratio, 0., delta_y,
				0., 0., 1., 0., 0., 0., 0., 1.;
			xform_mat = model_mat * view_proj;
			GLCHK(glUniformMatrix4fv(
				self->tex_transform_matrix_handle,
				1,
				false,
				xform_mat.data()));
			pdraw_gles2hud_draw_icon(self,
						 drone_model_icon,
						 0.,
						 0.,
						 self->config.small_icon_size,
						 self->ratio_w,
						 self->ratio_w,
						 color_green);
		}
	}

	/* Heading controller icon */
	if (takeoff_distance > 50.) {
		/* TODO: pilot */
		cy = 0.15 * self->ratio_w;
		delta_x = 0.;
		delta_y = self->config.heading_zone_v_offset * self->ratio_h;
		angle = drone_attitude.psi - takeoff_bearing;
		int angle_deg = ((int)(angle * 180. / M_PI + 70. + 360.)) % 360;
		if (angle_deg <= 140) {
			model_mat << cosf(angle), -sinf(angle), 0., delta_x,
				sinf(angle) * self->aspect_ratio,
				cosf(angle) * self->aspect_ratio, 0., delta_y,
				0., 0., 1., 0., 0., 0., 0., 1.;
			xform_mat = model_mat * view_proj;
			GLCHK(glUniformMatrix4fv(
				self->tex_transform_matrix_handle,
				1,
				false,
				xform_mat.data()));
			pdraw_gles2hud_draw_icon(self,
						 8,
						 0.,
						 cy,
						 self->config.small_icon_size,
						 self->ratio_w,
						 self->ratio_w,
						 color_green);
		}
	}

	GLCHK(glActiveTexture(GL_TEXTURE0 + self->text_texunit));
	GLCHK(glBindTexture(GL_TEXTURE_2D, self->text_texture));
	GLCHK(glUniform1i(self->tex_uniform_sampler, self->text_texunit));

	GLCHK(glUniformMatrix4fv(
		self->tex_transform_matrix_handle, 1, false, view_proj_mat));

	char str[20];
	snprintf(str, sizeof(str), "%d%%", battery_percentage);
	pdraw_gles2hud_draw_text(
		self,
		str,
		self->config.vu_meter_zone_h_offset * self->ratio_w,
		(-self->config.vu_meter_v_interval - 0.07) * self->ratio_h,
		self->config.text_size * self->ratio_w,
		1.,
		self->aspect_ratio,
		PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
		PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
		color_green);
	snprintf(str, sizeof(str), "%ddBm", wifi_rssi);
	pdraw_gles2hud_draw_text(self,
				 str,
				 self->config.vu_meter_zone_h_offset *
					 self->ratio_w,
				 (0.0 - 0.07) * self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				 PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				 color_green);
	snprintf(str, sizeof(str), "%d", location.sv_count);
	pdraw_gles2hud_draw_text(
		self,
		str,
		self->config.vu_meter_zone_h_offset * self->ratio_w,
		(self->config.vu_meter_v_interval - 0.07) * self->ratio_h,
		self->config.text_size * self->ratio_w,
		1.,
		self->aspect_ratio,
		PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
		PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
		color_green);
	snprintf(str, sizeof(str), "ALT");
	pdraw_gles2hud_draw_text(self,
				 str,
				 self->config.central_zone_size * self->ratio_w,
				 (self->config.central_zone_size - 0.01) *
					 self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
				 PDRAW_GLES2HUD_TEXT_ALIGN_BOTTOM,
				 color_green);
	snprintf(str, sizeof(str), "%.1fm", altitude);
	pdraw_gles2hud_draw_text(self,
				 str,
				 (self->config.central_zone_size + 0.04) *
					 self->ratio_w,
				 0.0 * self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
				 PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
				 color_green);
	if (drone_model == DRONE_MODEL_DISCO) {
		if ((location.valid) &&
		    (media_info->session_meta->takeoff_loc.valid)) {
			double alt_diff = 0.;
			if (!std::isnan(media_info->session_meta->takeoff_loc
						.altitude_wgs84ellipsoid) &&
			    !std::isnan(location.altitude_wgs84ellipsoid)) {
				alt_diff = location.altitude_wgs84ellipsoid -
					   media_info->session_meta->takeoff_loc
						   .altitude_wgs84ellipsoid;
			} else if (!std::isnan(
					   media_info->session_meta->takeoff_loc
						   .altitude_egm96amsl) &&
				   !std::isnan(location.altitude_egm96amsl)) {
				alt_diff = location.altitude_egm96amsl -
					   media_info->session_meta->takeoff_loc
						   .altitude_egm96amsl;
			}
			snprintf(str, sizeof(str), "DELTA: %+.1fm", alt_diff);
			pdraw_gles2hud_draw_text(
				self,
				str,
				self->config.central_zone_size * self->ratio_w,
				(-self->config.central_zone_size + 0.01) *
					self->ratio_h,
				self->config.text_size * self->ratio_w,
				1.,
				self->aspect_ratio,
				PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
				PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				color_green);
		}
	} else {
		snprintf(str, sizeof(str), "GND: %.1fm", ground_distance);
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.central_zone_size * self->ratio_w,
			(-self->config.central_zone_size + 0.01) *
				self->ratio_h,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
			PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
			color_green);
	}
	snprintf(str, sizeof(str), "SPD");
	pdraw_gles2hud_draw_text(
		self,
		str,
		-self->config.central_zone_size * self->ratio_w,
		(self->config.central_zone_size - 0.01) * self->ratio_h,
		self->config.text_size * self->ratio_w,
		1.,
		self->aspect_ratio,
		PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
		PDRAW_GLES2HUD_TEXT_ALIGN_BOTTOM,
		color_green);
	snprintf(str, sizeof(str), "%.1fm/s", horizontal_speed);
	pdraw_gles2hud_draw_text(self,
				 str,
				 -(self->config.central_zone_size + 0.04) *
					 self->ratio_w,
				 0.0 * self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
				 PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
				 color_green);
	if (air_speed != -1.) {
		snprintf(str, sizeof(str), "AIR: %4.1fm/s", air_speed);
		pdraw_gles2hud_draw_text(
			self,
			str,
			-self->config.central_zone_size * self->ratio_w,
			(-self->config.central_zone_size + 0.01) *
				self->ratio_h,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
			color_green);
	}
	if (takeoff_distance != 0.) {
		/* TODO: pilot */
		snprintf(str, sizeof(str), "DIST: %.0fm", takeoff_distance);
		pdraw_gles2hud_draw_text(
			self,
			str,
			0.0,
			(-self->config.central_zone_size / 2. - 0.10) *
				self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}
	if ((flying_state == VMETA_FLYING_STATE_TAKINGOFF) ||
	    (flying_state == VMETA_FLYING_STATE_LANDING) ||
	    (flying_state == VMETA_FLYING_STATE_EMERGENCY)) {
		pdraw_gles2hud_draw_text(
			self,
			flying_state_str[flying_state],
			0.0,
			(self->config.central_zone_size / 2. + 0.10) *
				self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	} else if ((piloting_mode == VMETA_PILOTING_MODE_RETURN_HOME) ||
		   (piloting_mode == VMETA_PILOTING_MODE_FLIGHT_PLAN)) {
		pdraw_gles2hud_draw_text(
			self,
			piloting_mode_str[piloting_mode],
			0.0,
			(self->config.central_zone_size / 2. + 0.10) *
				self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}
	if (ctrl_meta->battery_percentage < 255) {
		if (ctrl_meta->battery_percentage <= 100) {
			snprintf(str,
				 sizeof(str),
				 "CTRL BAT: %d%%",
				 ctrl_meta->battery_percentage);
		} else {
			snprintf(str, sizeof(str), "CTRL BAT: --%%");
		}
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			0. * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}
	if (media_info->playback_type == PDRAW_PLAYBACK_TYPE_LIVE) {
		snprintf(str,
			 sizeof(str),
			 "CTRL LOC: %s",
			 (ctrl_meta->location.valid) ? "OK" : "NOK");
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			0.05 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}
	if (strlen(media_info->session_meta->friendly_name) > 0)
		pdraw_gles2hud_draw_text(
			self,
			media_info->session_meta->friendly_name,
			friendly_name_x_offset,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.12 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	if ((frame_extra->play_timestamp > 0) &&
	    (frame_extra->play_timestamp != (uint64_t)-1) &&
	    (media_info->duration > 0) &&
	    (media_info->duration != (uint64_t)-1)) {
		uint64_t remaining_time =
			media_info->duration - frame_extra->play_timestamp;
		unsigned int c_hrs = 0, c_min = 0, c_sec = 0, c_msec = 0;
		unsigned int r_hrs = 0, r_min = 0, r_sec = 0, r_msec = 0;
		unsigned int d_hrs = 0, d_min = 0, d_sec = 0, d_msec = 0;
		pdraw_gles2hud_friendly_time_from_us(
			frame_extra->play_timestamp,
			&c_hrs,
			&c_min,
			&c_sec,
			&c_msec);
		pdraw_gles2hud_friendly_time_from_us(
			remaining_time, &r_hrs, &r_min, &r_sec, &r_msec);
		pdraw_gles2hud_friendly_time_from_us(
			media_info->duration, &d_hrs, &d_min, &d_sec, &d_msec);
		if (d_hrs) {
			snprintf(str,
				 sizeof(str),
				 "+%02d:%02d:%02d.%03d",
				 c_hrs,
				 c_min,
				 c_sec,
				 c_msec);
		} else {
			snprintf(str,
				 sizeof(str),
				 "+%02d:%02d.%03d",
				 c_min,
				 c_sec,
				 c_msec);
		}
		pdraw_gles2hud_draw_text(
			self,
			str,
			(self->config.right_zone_h_offset - 0.4) *
				self->ratio_w,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.12 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
		if (d_hrs) {
			snprintf(str,
				 sizeof(str),
				 "-%02d:%02d:%02d.%03d",
				 r_hrs,
				 r_min,
				 r_sec,
				 r_msec);
		} else {
			snprintf(str,
				 sizeof(str),
				 "-%02d:%02d.%03d",
				 r_min,
				 r_sec,
				 r_msec);
		}
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.12 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
		if (d_hrs) {
			snprintf(str,
				 sizeof(str),
				 "DUR: %02d:%02d:%02d",
				 d_hrs,
				 d_min,
				 d_sec);
		} else {
			snprintf(str,
				 sizeof(str),
				 "DUR: %02d:%02d",
				 d_min,
				 d_sec);
		}
		pdraw_gles2hud_draw_text(
			self,
			str,
			(self->config.right_zone_h_offset - 0.2) *
				self->ratio_w,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.10 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
			PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
			color_green);
	} else if (media_info->playback_type == PDRAW_PLAYBACK_TYPE_LIVE) {
		if (drone_meta->recording_duration > 0) {
			unsigned int d_hrs = 0, d_min = 0;
			unsigned int d_sec = 0, d_msec = 0;
			pdraw_gles2hud_friendly_time_from_us(
				drone_meta->recording_duration,
				&d_hrs,
				&d_min,
				&d_sec,
				&d_msec);
			if (d_hrs) {
				snprintf(str,
					 sizeof(str),
					 "REC %02d:%02d:%02d",
					 d_hrs,
					 d_min,
					 d_sec);
			} else {
				snprintf(str,
					 sizeof(str),
					 "REC %02d:%02d",
					 d_min,
					 d_sec);
			}
			pdraw_gles2hud_draw_text(
				self,
				str,
				self->config.right_zone_h_offset *
					self->ratio_w,
				self->config.roll_zone_v_offset *
						self->ratio_h +
					0.12 * self->ratio_w *
						self->aspect_ratio,
				self->config.text_size * self->ratio_w,
				1.,
				self->aspect_ratio,
				PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
				PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
				color_green);
		} else {
			pdraw_gles2hud_draw_text(
				self,
				"REC",
				self->config.right_zone_h_offset *
					self->ratio_w,
				self->config.roll_zone_v_offset *
						self->ratio_h +
					0.12 * self->ratio_w *
						self->aspect_ratio,
				self->config.text_size * self->ratio_w,
				1.,
				self->aspect_ratio,
				PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
				PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
				color_green);
		}
	}
	snprintf(str, sizeof(str), "%03d", heading_int);
	pdraw_gles2hud_draw_text(self,
				 str,
				 0. * self->ratio_w,
				 (self->config.heading_zone_v_offset + 0.10) *
					 self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				 PDRAW_GLES2HUD_TEXT_ALIGN_BOTTOM,
				 color_green);

	float height = self->config.central_zone_size * self->ratio_w *
		       self->aspect_ratio;
	steps = 6;
	for (i = -steps; i <= steps; i++) {
		if ((i != 0) && (!(i & 1))) {
			snprintf(str, sizeof(str), "%+2d ", i * 10);
			pdraw_gles2hud_draw_text(
				self,
				str,
				0. * self->ratio_w,
				i * height / 2 / steps,
				self->config.text_size * self->ratio_w,
				1.,
				self->aspect_ratio,
				PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
				color_green);
		}
	}

	/* Roll text */
	cy = 0.10 * self->ratio_w;
	delta_x = 0.;
	delta_y = self->config.roll_zone_v_offset * self->ratio_h;
	steps = 2;
	model_mat << 1., 0., 0., delta_x, 0., 1., 0., delta_y, 0., 0., 1., 0.,
		0., 0., 0., 1.;
	for (i = -steps, angle = M_PI * (-30. * steps) / 180.; i <= steps;
	     i++, angle += M_PI * 30. / 180.) {
		angle_deg = (i * 30 + 60 + 360) % 360;
		if (angle_deg <= 120) {
			model_mat(0, 0) = cosf(angle);
			model_mat(1, 0) = sinf(angle) * self->aspect_ratio;
			model_mat(0, 1) = -sinf(angle);
			model_mat(1, 1) = cosf(angle) * self->aspect_ratio;
			xform_mat = model_mat * view_proj;
			GLCHK(glUniformMatrix4fv(
				self->tex_transform_matrix_handle,
				1,
				false,
				xform_mat.data()));
			if (i == 0)
				snprintf(str, sizeof(str), "0");
			else
				snprintf(str, sizeof(str), "%+2d ", -i * 30);
			pdraw_gles2hud_draw_text(
				self,
				str,
				0.,
				cy,
				self->config.text_size * self->ratio_w,
				1.,
				1.,
				PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				color_green);
		}
	}

	/* Heading text */
	cy = 0.10 * self->ratio_w;
	delta_x = 0.;
	delta_y = self->config.heading_zone_v_offset * self->ratio_h;
	model_mat << 1., 0., 0., delta_x, 0., 1., 0., delta_y, 0., 0., 1., 0.,
		0., 0., 0., 1.;
	for (i = 0, angle = drone_attitude.psi; i < 8;
	     i++, angle += M_PI / 4.) {
		angle_deg = (heading_int + i * 45 + 70 + 360) % 360;
		if (angle_deg <= 140) {
			model_mat(0, 0) = cosf(angle);
			model_mat(1, 0) = sinf(angle) * self->aspect_ratio;
			model_mat(0, 1) = -sinf(angle);
			model_mat(1, 1) = cosf(angle) * self->aspect_ratio;
			xform_mat = model_mat * view_proj;
			GLCHK(glUniformMatrix4fv(
				self->tex_transform_matrix_handle,
				1,
				false,
				xform_mat.data()));
			pdraw_gles2hud_draw_text(
				self,
				heading_str[i],
				0.,
				cy,
				self->config.text_size,
				1.,
				1.,
				PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				color_green);
		}
	}

	/* Radar text */
#ifdef DEBUG_RADAR /* used to test the radar on records */
	if ((location.valid) && (session_meta->takeoff_loc.valid) &&
	    (ctrl_orientation_valid)) {
#else /* DEBUG_RADAR */
	if ((location.valid) && (ctrl_meta->location.valid) &&
	    (ctrl_orientation_valid)) {
#endif /* DEBUG_RADAR */
		cy = 0.09 * self->ratio_w;
		delta_x = self->config.radar_zone_h_offset * self->ratio_w;
		delta_y = self->config.radar_zone_v_offset * self->ratio_h;
		model_mat << 1., 0., 0., delta_x, 0., 1., 0., delta_y, 0., 0.,
			1., 0., 0., 0., 0., 1.;
		for (i = 0, angle = ctrl_orientation.psi; i < 8;
		     i += 2, angle += M_PI / 2.) {
			model_mat(0, 0) = cosf(angle);
			model_mat(1, 0) = sinf(angle) * self->aspect_ratio;
			model_mat(0, 1) = -sinf(angle);
			model_mat(1, 1) = cosf(angle) * self->aspect_ratio;
			xform_mat = model_mat * view_proj;
			GLCHK(glUniformMatrix4fv(
				self->tex_transform_matrix_handle,
				1,
				false,
				xform_mat.data()));
			pdraw_gles2hud_draw_text(
				self,
				heading_str[i],
				0.,
				cy,
				self->config.text_size,
				1.,
				1.,
				PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				PDRAW_GLES2HUD_TEXT_ALIGN_BOTTOM,
				color_green);
		}
	}

	GLCHK(glDisableVertexAttribArray(self->tex_position_handle));
	GLCHK(glDisableVertexAttribArray(self->tex_texcoord_handle));
	GLCHK(glDisableVertexAttribArray(self->tex_color_handle));

	return 0;
}


static int pdraw_gles2hud_render_imaging(
	struct pdraw_gles2hud *self,
	const struct pdraw_rect *render_pos,
	const struct pdraw_rect *content_pos,
	const float view_proj_mat[16],
	const struct pdraw_media_info *media_info,
	struct vmeta_frame *frame_meta,
	const struct pdraw_video_frame_extra *frame_extra,
	const struct pdraw_gles2hud_controller_meta *ctrl_meta,
	const struct pdraw_gles2hud_drone_meta *drone_meta)
{
	if (self == nullptr)
		return -EINVAL;
	if ((render_pos == nullptr) || (render_pos->width == 0) ||
	    (render_pos->height == 0))
		return -EINVAL;
	if ((content_pos == nullptr) || (content_pos->width == 0) ||
	    (content_pos->height == 0))
		return -EINVAL;
	if (media_info == nullptr)
		return -EINVAL;
	if (frame_meta == nullptr)
		return -EINVAL;
	if (frame_extra == nullptr)
		return -EINVAL;
	if (ctrl_meta == nullptr)
		return -EINVAL;
	if (drone_meta == nullptr)
		return -EINVAL;

	/* Picture field of view */
	pdraw_gles2hud_get_fov(self, media_info->session_meta, frame_meta);

	self->ratio_w = (float)content_pos->width / render_pos->width *
			self->config.scale;
	self->ratio_h = (float)content_pos->height / render_pos->height *
			self->config.scale;
	self->aspect_ratio = (float)render_pos->width / render_pos->height;

	GLCHK(glEnable(GL_BLEND));
	GLCHK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	GLCHK(glUseProgram(self->program));

	GLCHK(glEnableVertexAttribArray(self->position_handle));
	GLCHK(glEnableVertexAttribArray(self->color_handle));

	GLCHK(glUniformMatrix4fv(
		self->transform_matrix_handle, 1, false, view_proj_mat));

	pdraw_gles2hud_draw_framing_grid(
		self, render_pos, content_pos, color_black_alpha);
	pdraw_gles2hud_draw_histograms(self, frame_extra);

	GLCHK(glDisableVertexAttribArray(self->position_handle));
	GLCHK(glDisableVertexAttribArray(self->color_handle));

	return 0;
}


int pdraw_gles2hud_render(
	struct pdraw_gles2hud *self,
	enum pdraw_gles2hud_type type,
	const struct pdraw_rect *render_pos,
	const struct pdraw_rect *content_pos,
	const float view_proj_mat[16],
	const struct pdraw_media_info *media_info,
	struct vmeta_frame *frame_meta,
	const struct pdraw_video_frame_extra *frame_extra,
	const struct pdraw_gles2hud_controller_meta *ctrl_meta,
	const struct pdraw_gles2hud_drone_meta *drone_meta)
{
	switch (type) {
	case PDRAW_GLES2HUD_TYPE_PILOTING:
		return pdraw_gles2hud_render_piloting(self,
						      render_pos,
						      content_pos,
						      view_proj_mat,
						      media_info,
						      frame_meta,
						      frame_extra,
						      ctrl_meta,
						      drone_meta);
	case PDRAW_GLES2HUD_TYPE_IMAGING:
		return pdraw_gles2hud_render_imaging(self,
						     render_pos,
						     content_pos,
						     view_proj_mat,
						     media_info,
						     frame_meta,
						     frame_extra,
						     ctrl_meta,
						     drone_meta);
	default:
		ULOGE("unsupported HUD type: %d", type);
		return -ENOSYS;
	}
}
