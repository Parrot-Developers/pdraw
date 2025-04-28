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
ULOG_DECLARE_TAG(pdraw_gles2hud);

/* used to test the radar on records */
#if 0
#	define DEBUG_RADAR
#endif


#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))


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

	/* Parrot ANAFI 4K */
	DRONE_MODEL_ANAFI_4K,

	/* Parrot ANAFI Thermal */
	DRONE_MODEL_ANAFI_THERMAL,

	/* Parrot ANAFI UA */
	DRONE_MODEL_ANAFI_UA,

	/* Parrot ANAFI USA */
	DRONE_MODEL_ANAFI_USA,

	/* Parrot ANAFI Ai */
	DRONE_MODEL_ANAFI_AI,

	/* Parrot ANAFI3 MIL */
	DRONE_MODEL_ANAFI3_MIL,

	/* Parrot ANAFI3 GOV */
	DRONE_MODEL_ANAFI3_GOV,
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

static const char *link_type_str[] = {
	"",
	"LO",
	"LAN",
	"WLAN",
	"CELL",
};

static const float color_green[4] = {0.0f, 0.9f, 0.0f, 1.0f};

static const float color_dark_green[4] = {0.0f, 0.5f, 0.0f, 1.0f};

static const float color_black_alpha[4] = {0.0f, 0.0f, 0.0f, 0.66f};

static const float color_dark_blue[4] = {0.0f, 0.0f, 0.54f, 1.0f};


static const struct {
	const char *model_id;
	const char *model_str;
	const char *friendly_name;
	int icon_index;
} drone_model_info[] = {
	{"", "", "", -1}, /* DRONE_MODEL_UNKNOWN */
	{"0901", "Bebop", "Parrot Bebop", 0}, /* DRONE_MODEL_BEBOP */
	{"090c", "Bebop 2", "Parrot Bebop 2", 2}, /* DRONE_MODEL_BEBOP2 */
	{"090e", "Disco", "Parrot Disco", 1}, /* DRONE_MODEL_DISCO */
	{"0916", "Bluegrass", nullptr, 2}, /* DRONE_MODEL_BLUEGRASS */
	{"0914", "Anafi", nullptr, 2}, /* DRONE_MODEL_ANAFI_4K */
	{"0919", "AnafiThermal", nullptr, 2}, /* DRONE_MODEL_ANAFI_THERMAL */
	{"091b", "AnafiUA", nullptr, 2}, /* DRONE_MODEL_ANAFI_UA */
	{"091e", "AnafiUSA", nullptr, 2}, /* DRONE_MODEL_ANAFI_USA */
	{"091a", "ANAFI Ai", nullptr, 2}, /* DRONE_MODEL_ANAFI_AI */
	{"0920", "ANAFI3-MIL", nullptr, 2}, /* DRONE_MODEL_ANAFI3_MIL */
	{"0920", "ANAFI3-GOV", nullptr, 2}, /* DRONE_MODEL_ANAFI3_GOV
					       TODO: model_id not
					       available yet */
};


static enum drone_model
get_drone_model(const struct vmeta_session *session_meta)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		session_meta == nullptr, EINVAL, DRONE_MODEL_UNKNOWN);

	for (size_t i = 0; i < ARRAY_SIZE(drone_model_info); i++) {
		bool found = strcasecmp(drone_model_info[i].model_id,
					session_meta->model_id) == 0;
		found |= strcasecmp(drone_model_info[i].model_str,
				    session_meta->model) == 0;
		found |= drone_model_info[i].friendly_name != nullptr &&
			 strcasecmp(drone_model_info[i].friendly_name,
				    session_meta->friendly_name) == 0;
		if (found)
			return (enum drone_model)i;
	}
	return DRONE_MODEL_UNKNOWN;
}


static const char *get_active_link(struct vmeta_frame *meta)
{
	int res = 0, count = 0;
	const char *ret = "";
	const Vmeta__TimedMetadata *tm;
	const Vmeta__LinkMetadata *link;
	size_t i;

	if (meta->type != VMETA_FRAME_TYPE_PROTO)
		return link_type_str[VMETA__LINK_TYPE__LINK_TYPE_WLAN];

	res = vmeta_frame_proto_get_unpacked(meta, &tm);
	if (res < 0)
		return "";

	if (tm->n_links != 1)
		goto out;

	link = tm->links[0];
	if (link->protocol_case != VMETA__LINK_METADATA__PROTOCOL_STARFISH) {
		ret = link_type_str[VMETA__LINK_TYPE__LINK_TYPE_WLAN];
		goto out;
	}

	for (i = 0; i < link->starfish->n_links; i++) {
		if (!link->starfish->links[i]->active)
			continue;
		count++;
		ret = link_type_str[link->starfish->links[i]->type];
	}

	if (count > 1)
		ret = "";

out:
	vmeta_frame_proto_release_unpacked(meta, tm);
	return ret;
}

static const struct {
	const char *object_name;
	enum _Vmeta__TrackingClass object_class;
} tracking_object_map[] = {
	{"Undefined", VMETA__TRACKING_CLASS__TC_UNDEFINED},
	{"Person", VMETA__TRACKING_CLASS__TC_PERSON},
	{"Animal", VMETA__TRACKING_CLASS__TC_ANIMAL},
	{"Bicycle", VMETA__TRACKING_CLASS__TC_BICYCLE},
	{"Boat", VMETA__TRACKING_CLASS__TC_BOAT},
	{"Car", VMETA__TRACKING_CLASS__TC_CAR},
	{"Horse", VMETA__TRACKING_CLASS__TC_HORSE},
	{"Motorbike", VMETA__TRACKING_CLASS__TC_MOTORBIKE},
};


static const char *
get_tracking_object(const enum _Vmeta__TrackingClass object_class)
{
	for (size_t i = 0; i < ARRAY_SIZE(tracking_object_map); i++) {
		if (object_class == tracking_object_map[i].object_class)
			return tracking_object_map[i].object_name;
	}
	return tracking_object_map[0].object_name;
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
	if (self->config.text_size_tracking <= 0.) {
		self->config.text_size_tracking =
			PDRAW_GLES2HUD_DEFAULT_TEXT_SIZE_TRACKING;
	}
	if (self->config.text_tracking_v_offset <= 0.) {
		self->config.text_tracking_v_offset =
			PDRAW_GLES2HUD_DEFAULT_TEXT_TRACKING_V_OFFSET;
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
	const struct pdraw_gles2hud_controller_meta *ctrl_meta)
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

	uint8_t battery_percentage;
	vmeta_frame_get_battery_percentage(frame_meta, &battery_percentage);
	const char *active_link = get_active_link(frame_meta);
	uint8_t link_quality;
	vmeta_frame_get_link_quality(frame_meta, &link_quality);
	int8_t wifi_rssi;
	vmeta_frame_get_wifi_rssi(frame_meta, &wifi_rssi);
	enum vmeta_flying_state flying_state;
	vmeta_frame_get_flying_state(frame_meta, &flying_state);
	enum vmeta_piloting_mode piloting_mode;
	vmeta_frame_get_piloting_mode(frame_meta, &piloting_mode);
	enum drone_model drone_model =
		get_drone_model(media_info->video.session_meta);
	int drone_model_icon = drone_model_info[drone_model].icon_index;

	/* Drone location and ground distance */
	struct vmeta_location location;
	vmeta_frame_get_location(frame_meta, &location);
	double ground_distance;
	vmeta_frame_get_ground_distance(frame_meta, &ground_distance);
	if ((drone_model == DRONE_MODEL_DISCO) && (location.valid) &&
	    (media_info->video.session_meta->takeoff_loc.valid)) {
		ground_distance = location.altitude_egm96amsl -
				  media_info->video.session_meta->takeoff_loc
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
	pdraw_gles2hud_get_fov(
		self, media_info->video.session_meta, frame_meta);

	/* Distace to take-off */
	double takeoff_distance = 0.;
	double takeoff_bearing = 0.;
	double takeoff_elevation = 0.;
	if ((location.valid) &&
	    (media_info->video.session_meta->takeoff_loc.valid)) {
		pdraw_gles2hud_coords_distance_and_bearing(
			location.latitude,
			location.longitude,
			media_info->video.session_meta->takeoff_loc.latitude,
			media_info->video.session_meta->takeoff_loc.longitude,
			&takeoff_distance,
			&takeoff_bearing);
		double alt_diff = 0.;
		if (!std::isnan(media_info->video.session_meta->takeoff_loc
					.altitude_wgs84ellipsoid) &&
		    !std::isnan(location.altitude_wgs84ellipsoid)) {
			alt_diff = media_info->video.session_meta->takeoff_loc
					   .altitude_wgs84ellipsoid -
				   location.altitude_wgs84ellipsoid;
		} else if (!std::isnan(
				   media_info->video.session_meta->takeoff_loc
					   .altitude_egm96amsl) &&
			   !std::isnan(location.altitude_egm96amsl)) {
			alt_diff = media_info->video.session_meta->takeoff_loc
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

	/* Cursor on Target */
	struct vmeta_location lfic_loc;
	float lfic_x;
	float lfic_y;
	double lfic_estimated_precision;
	double lfic_grid_precision;
	vmeta_frame_get_lfic(frame_meta,
			     &lfic_loc,
			     &lfic_x,
			     &lfic_y,
			     &lfic_estimated_precision,
			     &lfic_grid_precision);

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
	double altitude = NAN;
	const char *altitude_ref = "";
	if (location.valid) {
		if (!std::isnan(location.altitude_egm96amsl)) {
			altitude = location.altitude_egm96amsl;
			altitude_ref = " (EGM96)";
		} else if (!std::isnan(location.altitude_wgs84ellipsoid)) {
			altitude = location.altitude_wgs84ellipsoid;
			altitude_ref = " (WGS84)";
		}
	}
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
				    -100.,
				    -20.,
				    -100.,
				    -70.,
				    color_green,
				    color_dark_green);
	for (i = 0; i < link_quality; i++) {
		pdraw_gles2hud_draw_filled_rect(
			self,
			(self->config.vu_meter_zone_h_offset + i * 0.015 -
			 0.035) *
				self->ratio_w,
			(0.0 - 0.025) * self->ratio_w * self->aspect_ratio,
			(self->config.vu_meter_zone_h_offset + i * 0.015 -
			 0.025) *
				self->ratio_w,
			(0.0 - 0.035) * self->ratio_w * self->aspect_ratio,
			color_green);
	}
	for (; link_quality > 0 && i < 5; i++) {
		pdraw_gles2hud_draw_rect(
			self,
			(self->config.vu_meter_zone_h_offset + i * 0.015 -
			 0.035) *
				self->ratio_w,
			(0.0 - 0.025) * self->ratio_w * self->aspect_ratio,
			(self->config.vu_meter_zone_h_offset + i * 0.015 -
			 0.025) *
				self->ratio_w,
			(0.0 - 0.035) * self->ratio_w * self->aspect_ratio,
			color_green,
			2.);
	}
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

	/* Cursor on Target cross */
	if (lfic_loc.valid)
		pdraw_gles2hud_draw_cot(self, lfic_x, lfic_y, color_green);

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
	if ((strlen(media_info->video.session_meta->friendly_name) > 0) &&
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
	snprintf(str, sizeof(str), "BAT: %d%%", battery_percentage);
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
	str[0] = '\0';
	if (strlen(active_link)) {
		char str_rssi[20] = "";
		if (wifi_rssi != 0) {
			snprintf(str_rssi,
				 sizeof(str_rssi),
				 " %ddBm",
				 wifi_rssi);
		}
		snprintf(str, sizeof(str), "%s%s", active_link, str_rssi);
	} else if (wifi_rssi != 0) {
		snprintf(str, sizeof(str), "%ddBm", wifi_rssi);
	} else {
		snprintf(str, sizeof(str), "N/A");
	}
	pdraw_gles2hud_draw_text(self,
				 str,
				 self->config.vu_meter_zone_h_offset *
					 self->ratio_w,
				 (0.0 - 0.08) * self->ratio_h,
				 self->config.text_size * self->ratio_w,
				 1.,
				 self->aspect_ratio,
				 PDRAW_GLES2HUD_TEXT_ALIGN_CENTER,
				 PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				 color_green);
	if (location.valid)
		snprintf(str, sizeof(str), "SAT: %d", location.sv_count);
	else
		snprintf(str, sizeof(str), "SAT: N/A");
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
	snprintf(str, sizeof(str), "ALT%s", altitude_ref);
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
	if (!std::isnan(altitude))
		snprintf(str, sizeof(str), "%.1fm", altitude);
	else
		snprintf(str, sizeof(str), "N/A");
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
		    (media_info->video.session_meta->takeoff_loc.valid)) {
			double alt_diff = 0.;
			if (!std::isnan(
				    media_info->video.session_meta->takeoff_loc
					    .altitude_wgs84ellipsoid) &&
			    !std::isnan(location.altitude_wgs84ellipsoid)) {
				alt_diff = location.altitude_wgs84ellipsoid -
					   media_info->video.session_meta
						   ->takeoff_loc
						   .altitude_wgs84ellipsoid;
			} else if (!std::isnan(media_info->video.session_meta
						       ->takeoff_loc
						       .altitude_egm96amsl) &&
				   !std::isnan(location.altitude_egm96amsl)) {
				alt_diff = location.altitude_egm96amsl -
					   media_info->video.session_meta
						   ->takeoff_loc
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
			 (ctrl_meta->location.valid) ? "OK" : "N/A");
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
	if (strlen(media_info->video.session_meta->friendly_name) > 0) {
		pdraw_gles2hud_draw_text(
			self,
			media_info->video.session_meta->friendly_name,
			friendly_name_x_offset,
			self->config.roll_zone_v_offset * self->ratio_h +
				0.12 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}

	/* Cursor on Target text */
	if (lfic_loc.valid) {
		snprintf(str, sizeof(str), "COT:");
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			self->config.heading_zone_v_offset * self->ratio_h +
				0.04 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
		snprintf(str, sizeof(str), "%+.8f", lfic_loc.latitude);
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			self->config.heading_zone_v_offset * self->ratio_h +
				0.02 * self->ratio_w * self->aspect_ratio,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
		snprintf(str, sizeof(str), "%+.8f", lfic_loc.longitude);
		pdraw_gles2hud_draw_text(
			self,
			str,
			self->config.right_zone_h_offset * self->ratio_w,
			self->config.heading_zone_v_offset * self->ratio_h,
			self->config.text_size * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_RIGHT,
			PDRAW_GLES2HUD_TEXT_ALIGN_MIDDLE,
			color_green);
	}

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
				self->config.text_size * self->ratio_w,
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
	const struct pdraw_gles2hud_controller_meta *ctrl_meta)
{
	int err;
	struct vmeta_rectf mask = {};

	/* Picture field of view */
	pdraw_gles2hud_get_fov(
		self, media_info->video.session_meta, frame_meta);

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

	err = vmeta_frame_get_thermal_mask(frame_meta, &mask);
	if (err == 0) {
		float x1 = (mask.left - 0.5) * 2;
		float y1 = -(mask.top - 0.5) * 2;
		float x2 = x1 + mask.width * 2;
		float y2 = y1 - mask.height * 2;
		pdraw_gles2hud_draw_rect(self,
					 x1 * self->ratio_w,
					 y1 * self->ratio_h,
					 x2 * self->ratio_w,
					 y2 * self->ratio_h,
					 color_black_alpha,
					 5.0);
	}

	pdraw_gles2hud_draw_histograms(self, frame_extra);

	GLCHK(glDisableVertexAttribArray(self->position_handle));
	GLCHK(glDisableVertexAttribArray(self->color_handle));

	return 0;
}

static int pdraw_gles2hud_render_tracking(
	struct pdraw_gles2hud *self,
	const struct pdraw_rect *render_pos,
	const struct pdraw_rect *content_pos,
	const float view_proj_mat[16],
	const struct pdraw_media_info *media_info,
	struct vmeta_frame *frame_meta,
	const struct pdraw_video_frame_extra *frame_extra,
	const struct pdraw_gles2hud_controller_meta *ctrl_meta)
{
	int res = 0;
	size_t i;
	float x1;
	float y1;
	float x2;
	float y2;
	char str[20];
	const Vmeta__TimedMetadata *tm;

	/* Picture field of view */
	pdraw_gles2hud_get_fov(
		self, media_info->video.session_meta, frame_meta);

	self->ratio_w = (float)content_pos->width / render_pos->width *
			self->config.scale;
	self->ratio_h = (float)content_pos->height / render_pos->height *
			self->config.scale;
	self->aspect_ratio = (float)render_pos->width / render_pos->height;

	res = vmeta_frame_proto_get_unpacked(frame_meta, &tm);
	if (res < 0)
		return 0;

	if (tm->proposal == nullptr && tm->tracking == nullptr)
		goto out;

	GLCHK(glUseProgram(self->program));

	GLCHK(glEnableVertexAttribArray(self->position_handle));
	GLCHK(glEnableVertexAttribArray(self->color_handle));

	GLCHK(glUniformMatrix4fv(
		self->transform_matrix_handle, 1, false, view_proj_mat));

	if (tm->proposal) {
		for (i = 0; i < tm->proposal->n_proposals; i++) {
			x1 = (tm->proposal->proposals[i]->x - 0.5) * 2;
			y1 = -(tm->proposal->proposals[i]->y - 0.5) * 2;
			x2 = x1 + tm->proposal->proposals[i]->width * 2;
			y2 = y1 - tm->proposal->proposals[i]->height * 2;

			pdraw_gles2hud_draw_rect(self,
						 x1 * self->ratio_w,
						 y1 * self->ratio_h,
						 x2 * self->ratio_w,
						 y2 * self->ratio_h,
						 color_dark_green,
						 5.0);
		}
	}

	if (tm->tracking &&
	    tm->tracking->state == VMETA__TRACKING_STATE__TS_TRACKING) {
		x1 = (tm->tracking->target->x - 0.5) * 2;
		y1 = -(tm->tracking->target->y - 0.5) * 2;
		x2 = x1 + tm->tracking->target->width * 2;
		y2 = y1 - tm->tracking->target->height * 2;

		pdraw_gles2hud_draw_rect(self,
					 x1 * self->ratio_w,
					 y1 * self->ratio_h,
					 x2 * self->ratio_w,
					 y2 * self->ratio_h,
					 color_dark_blue,
					 5.0);
	}

	GLCHK(glDisableVertexAttribArray(self->position_handle));
	GLCHK(glDisableVertexAttribArray(self->color_handle));

	GLCHK(glEnable(GL_BLEND));
	GLCHK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

	GLCHK(glUseProgram(self->tex_program));
	GLCHK(glUniformMatrix4fv(
		self->tex_transform_matrix_handle, 1, false, view_proj_mat));

	GLCHK(glEnableVertexAttribArray(self->tex_position_handle));
	GLCHK(glEnableVertexAttribArray(self->tex_texcoord_handle));

	GLCHK(glActiveTexture(GL_TEXTURE0 + self->text_texunit));
	GLCHK(glBindTexture(GL_TEXTURE_2D, self->text_texture));
	GLCHK(glUniform1i(self->tex_uniform_sampler, self->text_texunit));

	GLCHK(glUniformMatrix4fv(
		self->tex_transform_matrix_handle, 1, false, view_proj_mat));

	if (tm->proposal) {
		for (i = 0; i < tm->proposal->n_proposals; i++) {
			x1 = (tm->proposal->proposals[i]->x - 0.5) * 2;
			y1 = -(tm->proposal->proposals[i]->y - 0.5) * 2;

			snprintf(str,
				 sizeof(str),
				 "[%d]%s %.2f",
				 tm->proposal->proposals[i]->uid,
				 get_tracking_object(tm->proposal->proposals[i]
							     ->object_class),
				 tm->proposal->proposals[i]->confidence);

			pdraw_gles2hud_draw_text(
				self,
				str,
				x1 * self->ratio_w,
				(y1 + self->config.text_tracking_v_offset) *
					self->ratio_h,
				self->config.text_size_tracking * self->ratio_w,
				1.,
				self->aspect_ratio,
				PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
				PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
				color_black_alpha);
		}
	}

	if (tm->tracking &&
	    tm->tracking->state == VMETA__TRACKING_STATE__TS_TRACKING) {
		x1 = (tm->tracking->target->x - 0.5) * 2;
		y1 = -(tm->tracking->target->y - 0.5) * 2;

		snprintf(
			str,
			sizeof(str),
			"[%d]%s %.2f",
			tm->tracking->target->uid,
			get_tracking_object(tm->tracking->target->object_class),
			tm->tracking->target->confidence);
		pdraw_gles2hud_draw_text(
			self,
			str,
			x1 * self->ratio_w,
			(y1 + self->config.text_tracking_v_offset) *
				self->ratio_h,
			self->config.text_size_tracking * self->ratio_w,
			1.,
			self->aspect_ratio,
			PDRAW_GLES2HUD_TEXT_ALIGN_LEFT,
			PDRAW_GLES2HUD_TEXT_ALIGN_TOP,
			color_dark_blue);
	}

	GLCHK(glDisableVertexAttribArray(self->tex_position_handle));
	GLCHK(glDisableVertexAttribArray(self->tex_texcoord_handle));

out:
	vmeta_frame_proto_release_unpacked(frame_meta, tm);

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
	const struct pdraw_gles2hud_controller_meta *ctrl_meta)
{

	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(render_pos == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(render_pos->width == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(render_pos->height == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(content_pos == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(content_pos->width == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(content_pos->height == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(media_info == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame_meta == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame_extra == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctrl_meta == nullptr, EINVAL);

	switch (type) {
	case PDRAW_GLES2HUD_TYPE_PILOTING:
		return pdraw_gles2hud_render_piloting(self,
						      render_pos,
						      content_pos,
						      view_proj_mat,
						      media_info,
						      frame_meta,
						      frame_extra,
						      ctrl_meta);
	case PDRAW_GLES2HUD_TYPE_IMAGING:
		return pdraw_gles2hud_render_imaging(self,
						     render_pos,
						     content_pos,
						     view_proj_mat,
						     media_info,
						     frame_meta,
						     frame_extra,
						     ctrl_meta);
	case PDRAW_GLES2HUD_TYPE_TRACKING:
		return pdraw_gles2hud_render_tracking(self,
						      render_pos,
						      content_pos,
						      view_proj_mat,
						      media_info,
						      frame_meta,
						      frame_extra,
						      ctrl_meta);
	default:
		ULOGE("unsupported HUD type: %d", type);
		return -ENOSYS;
	}
}
