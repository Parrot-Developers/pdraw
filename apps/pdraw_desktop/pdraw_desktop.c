/**
 * Parrot Drones Awesome Video Viewer
 * Desktop application
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

#include "pdraw_desktop.h"

ULOG_DECLARE_TAG(pdraw_desktop);


void pdraw_desktop_resize(struct pdraw_desktop *self,
			  unsigned int width,
			  unsigned int height)
{
	int res;
	struct pdraw_rect rect;
	memset(&rect, 0, sizeof(rect));
	rect.width = width;
	rect.height = height;
	res = pdraw_be_resize_video_renderer(
		self->pdraw, self->renderer, &rect);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_resize_video_renderer", -res);
}


void pdraw_desktop_open(struct pdraw_desktop *self)
{
	int res;
	if (self->url != NULL) {
		res = pdraw_be_open_url(self->pdraw, self->url);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_open_url", -res);
	} else {
		res = pdraw_be_open_single_stream(self->pdraw,
						  self->local_addr,
						  self->local_stream_port,
						  self->local_control_port,
						  self->remote_addr,
						  self->remote_stream_port,
						  self->remote_control_port);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_open_single_stream", -res);
			return;
		}
	}
}


void pdraw_desktop_close(struct pdraw_desktop *self)
{
	int res;
	if (self->gles2hud != NULL) {
		res = pdraw_gles2hud_destroy(self->gles2hud);
		if (res < 0)
			ULOG_ERRNO("pdraw_gles2hud_destroy", -res);
		self->gles2hud = NULL;
	}
	if (self->renderer != NULL) {
		res = pdraw_be_stop_video_renderer(self->pdraw, self->renderer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_stop_video_renderer", -res);
		self->renderer = NULL;
	}
	res = pdraw_be_close(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_close", -res);
}


void pdraw_desktop_toggle_play_pause(struct pdraw_desktop *self)
{
	int res;
	if (pdraw_be_is_paused(self->pdraw) != 0) {
		res = pdraw_be_play_with_speed(self->pdraw,
					       self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_play_with_speed", -res);
	} else {
		res = pdraw_be_pause(self->pdraw);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_pause", -res);
	}
}


void pdraw_desktop_toggle_speed_sign(struct pdraw_desktop *self)
{
	int res;
	self->speed_sign *= -1;
	if (pdraw_be_is_paused(self->pdraw) == 0) {
		res = pdraw_be_play_with_speed(self->pdraw,
					       self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_play_with_speed", -res);
	}
}


void pdraw_desktop_speed_down(struct pdraw_desktop *self)
{
	int res;
	self->speed /= 2;
	if (pdraw_be_is_paused(self->pdraw) == 0) {
		res = pdraw_be_play_with_speed(self->pdraw,
					       self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_play_with_speed", -res);
	}
}


void pdraw_desktop_speed_up(struct pdraw_desktop *self)
{
	int res;
	self->speed *= 2;
	if (pdraw_be_is_paused(self->pdraw) == 0) {
		res = pdraw_be_play_with_speed(self->pdraw,
					       self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_play_with_speed", -res);
	}
}


void pdraw_desktop_previous_frame(struct pdraw_desktop *self)
{
	int res = pdraw_be_previous_frame(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_previous_frame", -res);
}


void pdraw_desktop_next_frame(struct pdraw_desktop *self)
{
	int res = pdraw_be_next_frame(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_next_frame", -res);
}


void pdraw_desktop_seek_back_10s(struct pdraw_desktop *self)
{
	int res = pdraw_be_seek_back(self->pdraw, 10000000, 0);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_seek_back", -res);
}


void pdraw_desktop_seek_forward_10s(struct pdraw_desktop *self)
{
	int res = pdraw_be_seek_forward(self->pdraw, 10000000, 0);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_seek_forward", -res);
}


void pdraw_desktop_goto_beginning(struct pdraw_desktop *self)
{
	int res = pdraw_be_seek_to(self->pdraw, 0, 1);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_seek_to", -res);
}


void pdraw_desktop_goto_end(struct pdraw_desktop *self)
{
	int res = pdraw_be_seek_to(self->pdraw, (uint64_t)-1, 1);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_seek_to", -res);
}


static void
open_resp_cb(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_OPEN_RESP,
					 (void *)(intptr_t)status,
					 NULL);
}


static void
close_resp_cb(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_CLOSE_RESP,
					 (void *)(intptr_t)status,
					 NULL);
}


static void unrecoverable_error_cb(struct pdraw_backend *pdraw, void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s", __func__);

	pdraw_desktop_ui_send_user_event(
		self, PDRAW_DESKTOP_EVENT_UNRECOVERABLE_ERROR, NULL, NULL);
}


static int select_demuxer_media_cb(struct pdraw_backend *pdraw,
				   const struct pdraw_demuxer_media *medias,
				   size_t count,
				   void *userdata)
{
	int id, ret;
	if (count == 0)
		return -ENOENT;
	if (count == 1)
		return medias[0].media_id;

	printf("Select demuxer media id:\n");
	for (size_t i = 0; i < count; i++)
		printf(" - %d: %s\n", medias[i].media_id, medias[i].name);
	printf(" > ");
	ret = scanf("%d", &id);
	if (ret != 1)
		return 0;
	return id;
}


static void media_added_cb(struct pdraw_backend *pdraw,
			   const struct pdraw_media_info *info,
			   void *userdata)
{
	ULOGI("%s id=%d", __func__, info->id);
}


static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *userdata)
{
	ULOGI("%s id=%d", __func__, info->id);
}


static void
ready_to_play_cb(struct pdraw_backend *pdraw, int ready, void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s ready=%d", __func__, ready);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_READY_TO_PLAY,
					 (void *)(intptr_t)ready,
					 NULL);
}


static void
end_of_range_cb(struct pdraw_backend *pdraw, uint64_t timestamp, void *userdata)
{
	ULOGI("%s timestamp=%" PRIu64, __func__, timestamp);
}


static void play_resp_cb(struct pdraw_backend *pdraw,
			 int status,
			 uint64_t timestamp,
			 float speed,
			 void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s) timestamp=%" PRIu64 " speed=%f",
	      __func__,
	      status,
	      strerror(-status),
	      timestamp,
	      speed);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_PLAY_RESP,
					 (void *)(intptr_t)status,
					 (void *)(intptr_t)timestamp);
}


static void pause_resp_cb(struct pdraw_backend *pdraw,
			  int status,
			  uint64_t timestamp,
			  void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s) timestamp=%" PRIu64,
	      __func__,
	      status,
	      strerror(-status),
	      timestamp);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_PAUSE_RESP,
					 (void *)(intptr_t)status,
					 (void *)(intptr_t)timestamp);
}


static void seek_resp_cb(struct pdraw_backend *pdraw,
			 int status,
			 uint64_t timestamp,
			 float speed,
			 void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s) timestamp=%" PRIu64 " speed=%f",
	      __func__,
	      status,
	      strerror(-status),
	      timestamp,
	      speed);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_SEEK_RESP,
					 (void *)(intptr_t)status,
					 (void *)(intptr_t)timestamp);
}


static void
socket_created_cb(struct pdraw_backend *pdraw, int fd, void *userdata)
{
	ULOGI("%s fd=%d", __func__, fd);
}


static int load_texture_cb(struct pdraw_backend *pdraw,
			   struct pdraw_video_renderer *renderer,
			   unsigned int texture_width,
			   unsigned int texture_height,
			   const struct pdraw_session_info *session_info,
			   const struct vmeta_session *session_meta,
			   const struct pdraw_video_frame *frame,
			   const void *frame_userdata,
			   size_t frame_userdata_len,
			   void *userdata)
{
	struct pdraw_desktop *self = userdata;
	return pdraw_desktop_ext_tex_load(self,
					  pdraw,
					  renderer,
					  session_info,
					  session_meta,
					  frame,
					  frame_userdata,
					  frame_userdata_len);
}


static void render_overlay_cb(struct pdraw_backend *pdraw,
			      struct pdraw_video_renderer *renderer,
			      const struct pdraw_rect *render_pos,
			      const struct pdraw_rect *content_pos,
			      const float *view_mat,
			      const float *proj_mat,
			      const struct pdraw_session_info *session_info,
			      const struct vmeta_session *session_meta,
			      const struct vmeta_frame *frame_meta,
			      const struct pdraw_video_frame_extra *frame_extra,
			      void *userdata)
{
	int res;
	struct pdraw_desktop *self = userdata;
	float view_proj_mat[16] = {
		1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.};

	if (self->gles2hud == NULL)
		return;

	struct pdraw_gles2hud_controller_meta ctrl_meta;
	memset(&ctrl_meta, 0, sizeof(ctrl_meta));
	ctrl_meta.battery_percentage = self->skyctrl_battery_percentage;
	ctrl_meta.location = self->skyctrl_location;
	struct pdraw_gles2hud_drone_meta drone_meta;
	memset(&drone_meta, 0, sizeof(drone_meta));
	if ((self->rec_start != 0) && (self->rec_stop == 0)) {
		time_t tm;
		time(&tm);
		uint64_t rec_dur =
			((uint64_t)tm * 1000 > self->rec_start)
				? (uint64_t)tm * 1000 - self->rec_start
				: 0;
		drone_meta.recording_duration = rec_dur * 1000;
	} else {
		drone_meta.recording_duration = 0;
	}

	res = pdraw_gles2hud_render(self->gles2hud,
				    self->hud_type,
				    render_pos,
				    content_pos,
				    view_proj_mat,
				    session_info,
				    session_meta,
				    frame_meta,
				    frame_extra,
				    &ctrl_meta,
				    &drone_meta);
	if (res < 0)
		ULOG_ERRNO("pdraw_gles2hud_render", -res);
}


static const struct pdraw_backend_cbs be_cbs = {
	.open_resp = &open_resp_cb,
	.close_resp = &close_resp_cb,
	.unrecoverable_error = &unrecoverable_error_cb,
	.select_demuxer_media = &select_demuxer_media_cb,
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
	.ready_to_play = &ready_to_play_cb,
	.end_of_range = &end_of_range_cb,
	.play_resp = &play_resp_cb,
	.pause_resp = &pause_resp_cb,
	.seek_resp = &seek_resp_cb,
	.socket_created = &socket_created_cb,
};


static struct pdraw_backend_video_renderer_cbs render_cbs = {
	.load_texture = &load_texture_cb,
	.render_overlay = &render_overlay_cb,
};


static void sighandler(int signum)
{
	ULOGI("stopping...");

	pdraw_desktop_ui_send_quit_event();

	signal(SIGINT, SIG_DFL);
}


static void welcome(void)
{
	printf("  ___ ___       ___      __\n"
	       " | _ \\   \\ _ _ /_\\ \\    / /\n"
	       " |  _/ |) | '_/ _ \\ \\/\\/ /\n"
	       " |_| |___/|_|/_/ \\_\\_/\\_/\n\n");
	printf("Parrot Drones Awesome Video Viewer\n"
	       "Copyright (c) 2016 Aurelien Barre\n"
	       "Copyright (c) 2017 Parrot Drones SAS\n\n");
}


enum args_id {
	ARGS_ID_HMD = 256,
	ARGS_ID_ZEBRAS,
	ARGS_ID_EXT_TEX,
};


static const char short_options[] = "hu:i:s:c:S:C:FH:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"url", required_argument, NULL, 'u'},
	{"ip", required_argument, NULL, 'i'},
	{"lstrmp", required_argument, NULL, 's'},
	{"lctrlp", required_argument, NULL, 'c'},
	{"rstrmp", required_argument, NULL, 'S'},
	{"rctrlp", required_argument, NULL, 'C'},
	{"fullscreen", no_argument, NULL, 'F'},
	{"hud", required_argument, NULL, 'H'},
	{"hmd", required_argument, NULL, ARGS_ID_HMD},
	{"zebras", required_argument, NULL, ARGS_ID_ZEBRAS},
	{"ext-tex", no_argument, NULL, ARGS_ID_EXT_TEX},
	{0, 0, 0, 0},
};


static void usage(char *prog_name)
{
	printf("Usage: %s [options]\n\n"
	       "Options:\n\n"
	       "  -h | --help                      "
	       "Print this message\n\n"
	       "  -u | --url <url>                 "
	       "Stream URL (rtsp://*) or filename (*.mp4)\n\n"
	       "  -i | --ip <ip_address>           "
	       "Direct RTP/AVP H.264 reception with a remote IP address\n"
	       "                                   "
	       "(optional; local ports must also be configured)\n\n"
	       "  -s | --lstrmp <port>             "
	       "Local stream port for direct RTP/AVP reception (required)\n\n"
	       "  -c | --lctrlp <port>             "
	       "Local control port for direct RTP/AVP reception (required)\n\n"
	       "  -S | --rstrmp <port>             "
	       "Remote stream port for direct RTP/AVP reception (optional)\n\n"
	       "  -C | --rctrlp <port>             "
	       "Remote control port for direct RTP/AVP reception (optional)\n\n"
	       "  -F | --fullscreen                "
	       "Start in full-screen mode\n\n"
	       "  -H | --hud <type>                "
	       "Enable the HUD (type: 0=piloting, 1=imaging)\n\n"
	       "       --hmd <model>               "
	       "HMD distortion correction with model id\n"
	       "                                   "
	       "(0='Parrot Cockpitglasses', 1='Parrot Cockpitglasses 2')\n\n"
	       "       --zebras <threshold>        "
	       "Enable overexposure zebras (threshold: [0.0 .. 1.0],\n"
	       "                                   "
	       "default: 0.95, out of range means default)\n"
	       "       --ext-tex                   "
	       "Enable testing the external texture loading callback\n"
	       "\n",
	       prog_name);
}


static int summary(struct pdraw_desktop *self)
{
	if (self->is_file) {
		printf("Offline playing of file '%s'\n\n", self->url);
	} else if (self->url != NULL) {
		printf("Streaming from URL '%s'\n\n", self->url);
	} else if ((self->local_stream_port != 0) &&
		   (self->local_control_port != 0)) {
		printf("Direct RTP/AVP H.264 reception: %s->%s "
		       "RTP:%d->%d RTCP:%d<->%d\n\n",
		       (self->remote_addr) ? self->remote_addr : "(any)",
		       (self->local_addr) ? self->local_addr : "(any)",
		       self->remote_stream_port,
		       self->local_stream_port,
		       self->remote_control_port,
		       self->local_control_port);
	} else {
		printf("Nothing to do...\n\n");
		return -EINVAL;
	}
	return 0;
}


int main(int argc, char **argv)
{
	int res, status = EXIT_SUCCESS;
	int idx, c;
	struct pdraw_desktop *self = NULL;

	welcome();

#ifdef _WIN32
	/* Initialize winsock API */
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 0), &wsadata);
#endif /* _WIN32 */

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	self->speed = 1.0;
	self->speed_sign = 1;
	self->skyctrl_battery_percentage = 255;

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			goto out;

		case 'u':
			free(self->url);
			self->url = strdup(optarg);
			if ((self->url) &&
			    ((strlen(self->url) <= 7) ||
			     (strncmp(self->url, "rtsp://", 7) != 0)))
				self->is_file = 1;
			break;

		case 'i':
			free(self->remote_addr);
			self->remote_addr = strdup(optarg);
			break;

		case 's':
			self->local_stream_port = atoi(optarg);
			break;

		case 'c':
			self->local_control_port = atoi(optarg);
			break;

		case 'S':
			self->remote_stream_port = atoi(optarg);
			break;

		case 'C':
			self->remote_control_port = atoi(optarg);
			break;

		case 'F':
			self->fullscreen = 1;
			break;

		case 'H':
			self->hud_type = atoi(optarg);
			self->enable_hud = 1;
			break;

		case ARGS_ID_HMD:
			self->hmd_model = atoi(optarg);
			self->enable_hmd = 1;
			break;

		case ARGS_ID_ZEBRAS:
			self->enable_zebras = 1;
			self->zebras_threshold = atof(optarg);
			if ((self->zebras_threshold < 0.f) ||
			    (self->zebras_threshold > 1.f))
				self->zebras_threshold = DEFAULT_ZEBRAS_THRES;
			break;

		case ARGS_ID_EXT_TEX:
			self->ext_tex = 1;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	res = summary(self);
	if (res < 0) {
		usage(argv[0]);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create the UI */
	res = pdraw_desktop_ui_init(self);
	if (res < 0) {
		status = EXIT_FAILURE;
		goto out;
	}

	signal(SIGINT, sighandler);

	/* Create PDrAW instance */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	res = pdraw_be_set_self_friendly_name(self->pdraw, APP_NAME);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_self_friendly_name", -res);
	res = pdraw_be_set_self_serial_number(self->pdraw,
					      "00000000"); /* TODO */
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_self_serial_number", -res);
	res = pdraw_be_set_self_software_version(self->pdraw,
						 "0.0.0"); /* TODO */
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_self_software_version", -res);
	res = pdraw_be_set_hmd_model_setting(self->pdraw, self->hmd_model);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_hmd_model_setting", -res);

	/* Create the renderer */
	struct pdraw_video_renderer_params params;
	struct pdraw_rect rect;
	memset(&params, 0, sizeof(params));
	memset(&rect, 0, sizeof(rect));
	params.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	params.enable_transition_flags = 0xFFFFFFFF;
	params.enable_hmd_distortion_correction = self->enable_hmd;
	params.enable_overexposure_zebras = self->enable_zebras;
	params.overexposure_zebras_threshold = self->zebras_threshold;
	params.enable_histograms =
		((self->enable_hud) &&
		 (self->hud_type == PDRAW_GLES2HUD_TYPE_IMAGING))
			? 1
			: 0;

	/* Test: take a 2:3 crop at the center and force a 2:3 DAR and a 1600
	 * pixels texture width (only enabled with the '--ext-tex' option) */
	params.video_texture_width = 1600;
	params.video_texture_dar_width = 2;
	params.video_texture_dar_height = 3;

	rect.height = self->window_height;
	rect.width = self->window_width;
	res = pdraw_be_start_video_renderer(self->pdraw,
					    &rect,
					    &params,
					    &render_cbs,
					    self,
					    &self->renderer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_start_video_renderer", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	if (self->enable_hud) {
		struct pdraw_gles2hud_config hud_config;
		memset(&hud_config, 0, sizeof(hud_config));
		res = pdraw_gles2hud_new(&hud_config, &self->gles2hud);
		if (res < 0)
			ULOG_ERRNO("pdraw_gles2hud_new", -res);
	}
	res = pdraw_desktop_ext_tex_setup(self);
	if (res < 0)
		ULOG_ERRNO("pdraw_desktop_ext_tex_setup", -res);

	pdraw_desktop_ui_send_user_event(
		self, PDRAW_DESKTOP_EVENT_OPEN, NULL, NULL);

	/* Run the UI loop */
	pdraw_desktop_ui_loop(self);

out:
	if (self != NULL) {
		pdraw_desktop_ext_tex_cleanup(self);
		if (self->gles2hud != NULL) {
			res = pdraw_gles2hud_destroy(self->gles2hud);
			if (res < 0)
				ULOG_ERRNO("pdraw_gles2hud_destroy", -res);
		}
		if (self->pdraw != NULL) {
			res = pdraw_be_destroy(self->pdraw);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_destroy", -res);
		}
		pdraw_desktop_ui_destroy(self);
		free(self->url);
		free(self->local_addr);
		free(self->remote_addr);
		free(self);
	}

#ifdef _WIN32
	/* Cleanup winsock API */
	WSACleanup();
#endif /* _WIN32 */

	printf("\nHasta la vista, PDrAW!\n\n");
	exit(status);
}
