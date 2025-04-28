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

#ifndef _PDRAW_DESKTOP_H_
#define _PDRAW_DESKTOP_H_

#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define ULOG_TAG pdraw_desktop
#include <ulog.h>

#ifdef _WIN32
#	include <windows.h>
#	include <winsock2.h>
#	undef near
#	undef far
#endif /* !_WIN32 */

#if defined(_WIN32)
#	include <epoxy/gl.h>
#elif defined(__APPLE__)
#	include <OpenGL/OpenGL.h>
#	include <OpenGL/gl.h>
#else
#	include <GLES2/gl2.h>
#endif

#include <SDL.h>
#include <futils/futils.h>
#include <pdraw/pdraw_backend.h>
#include <pdraw/pdraw_gles2hud.h>
#ifdef BUILD_LIBPDRAW_OVERLAYER
#	include <pdraw/pdraw_overlayer.h>
#endif
#include <video-defs/vdefs.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#define APP_NAME "PDrAW"
#define RTSP_LIVE_URI "live"
#define DEFAULT_ZEBRAS_THRES (0.95f)
#define MAX_RENDERERS 9


enum pdraw_desktop_event {
	PDRAW_DESKTOP_EVENT_OPEN = 0,
	PDRAW_DESKTOP_EVENT_OPEN_RESP,
	PDRAW_DESKTOP_EVENT_UNRECOVERABLE_ERROR,
	PDRAW_DESKTOP_EVENT_READY_TO_PLAY,
	PDRAW_DESKTOP_EVENT_PLAY_RESP,
	PDRAW_DESKTOP_EVENT_PAUSE_RESP,
	PDRAW_DESKTOP_EVENT_SEEK_RESP,
	PDRAW_DESKTOP_EVENT_STOP_RESP,
	PDRAW_DESKTOP_EVENT_ADD_RENDERER,
	PDRAW_DESKTOP_EVENT_TOGGLE_MEDIA,
};


struct pdraw_desktop_renderer {
	struct pdraw_video_renderer *renderer;
	unsigned int media_id;
	unsigned int pending_media_id;
	struct pdraw_rect render_pos;
	struct pdraw_overlayer *overlayer;
	struct pdraw_gles2hud *gles2hud;
};


struct pdraw_desktop {
	char *url;
	int is_file;
	int is_vipc;
	char *local_addr;
	uint16_t local_stream_port;
	uint16_t local_control_port;
	char *remote_addr;
	uint16_t remote_stream_port;
	uint16_t remote_control_port;

	SDL_Window *window;
	SDL_GLContext gl_context;
	uint32_t user_event;
	unsigned int window_width;
	unsigned int window_height;
	int fullscreen;
	int always_on_top;

	int enable_overlay;
#ifdef BUILD_LIBPDRAW_OVERLAYER
	enum pdraw_overlayer_layout overlayer_layout;
#endif
	int enable_hud;
	enum pdraw_gles2hud_type hud_type;
	float hud_view_proj_mat[16];
	int enable_norm;
	int enable_zebras;
	float zebras_threshold;

	int stopped;
	struct pdraw_backend *pdraw;
	struct pdraw_demuxer *demuxer;
	char *demuxer_media_list;
	struct pdraw_vipc_source *source;
	unsigned int media_count;
	unsigned int video_media_count;
	unsigned int video_renderer_pending_media_id;
	struct pdraw_desktop_renderer video_renderers[MAX_RENDERERS];
	unsigned int video_renderer_count;
	struct pdraw_audio_renderer *audio_renderer;
	float speed;
	int speed_sign;

	uint8_t skyctrl_battery_percentage;
	struct vmeta_location skyctrl_location;

	int ext_tex;
	GLint ext_tex_program;
	GLint ext_tex_yuv2rgb_matrix;
	GLint ext_tex_yuv2rgb_offset;
	GLuint ext_tex_textures[3];
	GLint ext_tex_uniform_samplers[3];
	GLint ext_tex_position_handle;
	GLint ext_tex_texcoord_handle;

	enum pdraw_video_renderer_scheduling_mode default_scheduling_mode;
	enum pdraw_video_renderer_fill_mode default_fill_mode;

	struct pdraw_muxer *recorder;
	unsigned int recorder_media_id;

	struct pdraw_chapter *chapter_list;
	size_t chapter_count;
};


extern const struct pdraw_backend_video_renderer_cbs render_cbs;


extern const struct pdraw_backend_audio_renderer_cbs audio_render_cbs;


int pdraw_desktop_open(struct pdraw_desktop *self);


void pdraw_desktop_close(struct pdraw_desktop *self);


void pdraw_desktop_toggle_play_pause(struct pdraw_desktop *self);


void pdraw_desktop_toggle_speed_sign(struct pdraw_desktop *self);


void pdraw_desktop_speed_down(struct pdraw_desktop *self);


void pdraw_desktop_speed_up(struct pdraw_desktop *self);


void pdraw_desktop_previous_frame(struct pdraw_desktop *self);


void pdraw_desktop_next_frame(struct pdraw_desktop *self);


void pdraw_desktop_seek_back_10s(struct pdraw_desktop *self);


void pdraw_desktop_seek_forward_10s(struct pdraw_desktop *self);


void pdraw_desktop_seek_to_prev_chapter(struct pdraw_desktop *self);


void pdraw_desktop_seek_to_next_chapter(struct pdraw_desktop *self);


void pdraw_desktop_goto_beginning(struct pdraw_desktop *self);


void pdraw_desktop_goto_end(struct pdraw_desktop *self);


void pdraw_desktop_toggle_start_stop_recorder(struct pdraw_desktop *self);


void pdraw_desktop_dump_pipeline(struct pdraw_desktop *self);


void pdraw_desktop_change_scheduling_mode(struct pdraw_desktop *self);


void pdraw_desktop_toggle_demuxer_media(struct pdraw_desktop *self);


void pdraw_desktop_change_fill_mode(struct pdraw_desktop *self);


void pdraw_desktop_toggle_mb_status(struct pdraw_desktop *self);


int pdraw_desktop_ui_init(struct pdraw_desktop *self);


int pdraw_desktop_ui_destroy(struct pdraw_desktop *self);


void pdraw_desktop_ui_send_quit_event(void);


void pdraw_desktop_ui_send_user_event(struct pdraw_desktop *self,
				      enum pdraw_desktop_event event,
				      void *data1,
				      void *data2);


void pdraw_desktop_ui_add_media(struct pdraw_desktop *self,
				unsigned int media_id,
				unsigned int media_type);


void pdraw_desktop_ui_resize(struct pdraw_desktop *self);


int pdraw_desktop_ui_loop(struct pdraw_desktop *self);


void pdraw_desktop_view_create_matrices(struct pdraw_desktop *self,
					unsigned int width,
					unsigned int height,
					float *view_mat,
					float *proj_mat,
					float near,
					float far);


int pdraw_desktop_ext_tex_setup(struct pdraw_desktop *self);


int pdraw_desktop_ext_tex_cleanup(struct pdraw_desktop *self);


int pdraw_desktop_ext_tex_load(struct pdraw_desktop *self,
			       struct pdraw_backend *pdraw,
			       struct pdraw_video_renderer *renderer,
			       const struct pdraw_media_info *media_info,
			       struct mbuf_raw_video_frame *frame,
			       const void *frame_userdata,
			       size_t frame_userdata_len);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_PDRAW_DESKTOP_H_ */
