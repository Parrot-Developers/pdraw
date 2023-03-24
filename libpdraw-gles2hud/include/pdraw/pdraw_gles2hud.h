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

#ifndef _PDRAW_GLES2HUD_H_
#define _PDRAW_GLES2HUD_H_

#include <inttypes.h>

#include <pdraw/pdraw_defs.h>
#include <video-metadata/vmeta.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef PDRAW_GLES2HUD_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_GLES2HUD_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_GLES2HUD_API                                     \
			__attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_GLES2HUD_API_EXPORTS */
#	define PDRAW_GLES2HUD_API
#endif /* !PDRAW_GLES2HUD_API_EXPORTS */


/* HUD configuration default values */
#define PDRAW_GLES2HUD_DEFAULT_SCALE (1.00f)
#define PDRAW_GLES2HUD_DEFAULT_TEXT_SIZE (0.15f)
#define PDRAW_GLES2HUD_DEFAULT_SMALL_ICON_SIZE (0.040f)
#define PDRAW_GLES2HUD_DEFAULT_MEDIUM_ICON_SIZE (0.050f)
#define PDRAW_GLES2HUD_DEFAULT_CENTRAL_ZONE_SIZE (0.25f)
#define PDRAW_GLES2HUD_DEFAULT_HEADING_ZONE_V_OFFSET (-0.80f)
#define PDRAW_GLES2HUD_DEFAULT_ROLL_ZONE_V_OFFSET (0.50f)
#define PDRAW_GLES2HUD_DEFAULT_VU_METER_ZONE_H_OFFSET (-0.60f)
#define PDRAW_GLES2HUD_DEFAULT_VU_METER_V_INTERVAL (-0.30f)
#define PDRAW_GLES2HUD_DEFAULT_RIGHT_ZONE_H_OFFSET (0.65f)
#define PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_H_OFFSET (0.45f)
#define PDRAW_GLES2HUD_DEFAULT_RADAR_ZONE_V_OFFSET (-0.65f)


/* Forward declarations */
struct pdraw_gles2hud;


/* HUD type */
enum pdraw_gles2hud_type {
	/* Piloting HUD with instruments for navigation */
	PDRAW_GLES2HUD_TYPE_PILOTING = 0,

	/* Imaging HUD for shooting videos and taking pictures */
	PDRAW_GLES2HUD_TYPE_IMAGING,
};


/* HUD configuration */
struct pdraw_gles2hud_config {
	/* Global HUD scale */
	float scale;

	/* HUD text size */
	float text_size;

	/* Small icons size */
	float small_icon_size;

	/* Medium icons size */
	float medium_icon_size;

	/* Placement parameters */
	float central_zone_size;
	float heading_zone_v_offset;
	float roll_zone_v_offset;
	float vu_meter_zone_h_offset;
	float vu_meter_v_interval;
	float right_zone_h_offset;
	float radar_zone_h_offset;
	float radar_zone_v_offset;
};


/* clang-format off */
/* Controller metadata */
struct pdraw_gles2hud_controller_meta {
	/* Controller location */
	struct vmeta_location location;

	/* Controller orientation is valid */
	uint32_t has_quat:1;

	/* Controller orientation */
	struct vmeta_quaternion quat;

	/* Controller wifi radar angle (deg) */
	float radar_angle;

	/* Controller battery level (0..100;
	 * > 100 means charging; 255 means not available) */
	uint8_t battery_percentage;
};
/* clang-format on */


/* Drone metadata */
struct pdraw_gles2hud_drone_meta {
	/* Time since video recording started
	 * (microseconds; null means not recording) */
	uint64_t recording_duration;
};


/**
 * Create a HUD instance.
 * The configuration structure must be filled; null values will be replaced
 * by the default values.
 * The instance handle is returned through the hud parameter.
 * When no longer needed, the instance must be freed using the
 * pdraw_gles2hud_destroy() function.
 * This function must be called on a thread with a current GLES2 context.
 * @param config: HUD configuration
 * @param hud: HUD instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_GLES2HUD_API int
pdraw_gles2hud_new(const struct pdraw_gles2hud_config *config,
		   struct pdraw_gles2hud **hud);


/**
 * Free a HUD instance.
 * This function frees all resources associated with a HUD instance.
 * This function must be called on a thread with a current GLES2 context.
 * @param self: HUD instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_GLES2HUD_API int pdraw_gles2hud_destroy(struct pdraw_gles2hud *self);


/**
 * Get the HUD configuration.
 * This function returns the HUD configuration by filling the provided
 * configuration structure.
 * This function must be called on a thread with a current GLES2 context.
 * @param self: HUD instance handle
 * @param config: HUD configuration (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_GLES2HUD_API int
pdraw_gles2hud_get_config(struct pdraw_gles2hud *self,
			  struct pdraw_gles2hud_config *config);


/**
 * Set the HUD configuration.
 * This function sets the HUD configuration; null values will be replaced
 * by the default values.
 * This function must be called on a thread with a current GLES2 context.
 * @param self: HUD instance handle
 * @param config: HUD configuration
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_GLES2HUD_API int
pdraw_gles2hud_set_config(struct pdraw_gles2hud *self,
			  const struct pdraw_gles2hud_config *config);


/**
 * Render the HUD of the given type.
 * This function must be called on a thread with a current GLES2 context.
 * @param self: HUD instance handle
 * @param type: HUD type to render
 * @param content_pos: content position (video rectangle)
 * @param view_proj_mat: view * projection matrix
 * @param media_info: media information
 * @param frame_meta: frame metadata
 * @param frame_extra: frame extra metadata
 * @param ctrl_meta: controller metadata
 * @param drone_meta: drone metadata
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_GLES2HUD_API int
pdraw_gles2hud_render(struct pdraw_gles2hud *self,
		      enum pdraw_gles2hud_type type,
		      const struct pdraw_rect *render_pos,
		      const struct pdraw_rect *content_pos,
		      const float view_proj_mat[16],
		      const struct pdraw_media_info *media_info,
		      struct vmeta_frame *frame_meta,
		      const struct pdraw_video_frame_extra *frame_extra,
		      const struct pdraw_gles2hud_controller_meta *ctrl_meta,
		      const struct pdraw_gles2hud_drone_meta *drone_meta);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_PDRAW_GLES2HUD_H_ */
