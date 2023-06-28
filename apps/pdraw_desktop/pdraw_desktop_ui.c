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


static const struct {
	unsigned int h;
	unsigned int v;
} layout[MAX_RENDERERS] = {
	{1, 1},
	{2, 1},
	{2, 2},
	{2, 2},
	{3, 2},
	{3, 2},
	{3, 3},
	{3, 3},
	{3, 3},
};


static void user_event(struct pdraw_desktop *self, SDL_UserEvent *event)
{
	int status;
	unsigned int media_id;
	switch (event->code) {
	case PDRAW_DESKTOP_EVENT_OPEN:
		pdraw_desktop_open(self);
		break;
	case PDRAW_DESKTOP_EVENT_OPEN_RESP:
		status = (int)(intptr_t)event->data1;
		if (status < 0)
			pdraw_desktop_close(self);
		break;
	case PDRAW_DESKTOP_EVENT_UNRECOVERABLE_ERROR:
		pdraw_desktop_close(self);
		break;
	case PDRAW_DESKTOP_EVENT_READY_TO_PLAY:
		if (event->data1)
			pdraw_desktop_toggle_play_pause(self);
		break;
	case PDRAW_DESKTOP_EVENT_PLAY_RESP:
		break;
	case PDRAW_DESKTOP_EVENT_PAUSE_RESP:
		break;
	case PDRAW_DESKTOP_EVENT_SEEK_RESP:
		break;
	case PDRAW_DESKTOP_EVENT_STOP_RESP:
		self->stopped = 1;
		break;
	case PDRAW_DESKTOP_EVENT_ADD_RENDERER:
		media_id = (unsigned int)(intptr_t)event->data1;
		pdraw_desktop_ui_add_media(self, media_id);
		break;
	default:
		break;
	}
}


static void sdl_event(struct pdraw_desktop *self, SDL_Event *event)
{
	int res;

	switch (event->type) {
	case SDL_QUIT:
		pdraw_desktop_close(self);
		break;
	case SDL_WINDOWEVENT:
		if (event->window.event == SDL_WINDOWEVENT_RESIZED) {
			self->window_width = event->window.data1;
			self->window_height = event->window.data2;
			pdraw_desktop_ui_resize(self);
		}
		break;
	case SDL_KEYDOWN:
		switch (event->key.keysym.sym) {
		case SDLK_ESCAPE:
			pdraw_desktop_close(self);
			break;
		case SDLK_SPACE:
			pdraw_desktop_toggle_play_pause(self);
			break;
		case SDLK_PAGEUP:
			pdraw_desktop_seek_back_10s(self);
			break;
		case SDLK_PAGEDOWN:
			pdraw_desktop_seek_forward_10s(self);
			break;
		case SDLK_LEFT:
			if (self->demuxer == NULL)
				break;
			if (pdraw_be_demuxer_is_paused(self->pdraw,
						       self->demuxer) == 0)
				pdraw_desktop_seek_back_10s(self);
			else
				pdraw_desktop_previous_frame(self);
			break;
		case SDLK_RIGHT:
			if (self->demuxer == NULL)
				break;
			if (pdraw_be_demuxer_is_paused(self->pdraw,
						       self->demuxer) == 0)
				pdraw_desktop_seek_forward_10s(self);
			else
				pdraw_desktop_next_frame(self);
			break;
		case SDLK_HOME:
			pdraw_desktop_goto_beginning(self);
			break;
		case SDLK_END:
			pdraw_desktop_goto_end(self);
			break;
		case SDLK_BACKSPACE:
		case SDLK_KP_ENTER:
			pdraw_desktop_toggle_speed_sign(self);
			break;
		case SDLK_MINUS:
		case SDLK_KP_MINUS:
			pdraw_desktop_speed_down(self);
			break;
		case SDLK_EQUALS:
		case SDLK_PLUS:
		case SDLK_KP_PLUS:
			pdraw_desktop_speed_up(self);
			break;
		case SDLK_RETURN:
			self->fullscreen ^= 1;
			res = SDL_SetWindowFullscreen(
				self->window,
				(self->fullscreen)
					? SDL_WINDOW_FULLSCREEN_DESKTOP
					: 0);
			if (res < 0) {
				ULOGW("SDL_SetWindowFullscreen() failed: %d",
				      res);
			}
			break;
		case SDLK_r:
			pdraw_desktop_toggle_start_stop_recorder(self);
			break;
		case SDLK_d:
			pdraw_desktop_dump_pipeline(self);
			break;
		case SDLK_f:
			pdraw_desktop_change_fill_mode(self);
			break;
		case SDLK_s:
			pdraw_desktop_change_scheduling_mode(self);
			break;
		}
		break;
	default:
		break;
	}
}


int pdraw_desktop_ui_init(struct pdraw_desktop *self)
{
	int res;

	res = SDL_Init(SDL_INIT_VIDEO);
	if (res < 0) {
		ULOGE("SDL_Init() failed: %d(%s)", res, SDL_GetError());
		res = -EPROTO;
		return res;
	}

	SDL_DisplayMode dm;
	res = SDL_GetDesktopDisplayMode(0, &dm);
	if (res < 0) {
		ULOGE("SDL_GetDesktopDisplayMode() failed: %d(%s)",
		      res,
		      SDL_GetError());
		res = -EPROTO;
		return res;
	}
	ULOGI("display mode: %dx%d %dHz", dm.w, dm.h, dm.refresh_rate);
	self->window_width = dm.w / 2;
	self->window_height = dm.h / 2;

	self->window = SDL_CreateWindow(
		APP_NAME,
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		self->window_width,
		self->window_height,
		SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
			((self->fullscreen) ? SDL_WINDOW_FULLSCREEN_DESKTOP
					    : 0) |
			((self->always_on_top) ? SDL_WINDOW_ALWAYS_ON_TOP : 0));
	if (self->window == NULL) {
		ULOGE("SDL_CreateWindow() failed: %s", SDL_GetError());
		res = -EPROTO;
		return res;
	}

	self->gl_context = SDL_GL_CreateContext(self->window);
	if (self->gl_context == NULL) {
		ULOGE("SDL_GL_CreateContext() failed: %s", SDL_GetError());
		res = -EPROTO;
		return res;
	}

#ifndef _WIN32
	res = SDL_GL_SetSwapInterval(1);
	if (res < 0) {
		ULOGE("SDL_GL_SetSwapInterval() failed: %d(%s)",
		      res,
		      SDL_GetError());
		res = -EPROTO;
		return res;
	}
#endif /* !_WIN32 */

	self->user_event = SDL_RegisterEvents(1);
	if (self->user_event == (uint32_t)-1) {
		ULOGE("SDL_RegisterEvents() failed");
		res = -EPROTO;
		return res;
	}

	return 0;
}


int pdraw_desktop_ui_destroy(struct pdraw_desktop *self)
{
	int res;

	for (unsigned int i = 0; i < self->renderer_count; i++) {
		if (self->gles2hud[i] != NULL) {
			res = pdraw_gles2hud_destroy(self->gles2hud[i]);
			if (res < 0)
				ULOG_ERRNO("pdraw_gles2hud_destroy", -res);
			self->gles2hud[i] = NULL;
		}
		if (self->renderer[i] != NULL) {
			res = pdraw_be_video_renderer_destroy(
				self->pdraw, self->renderer[i]);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_video_renderer_destroy",
					   -res);
			self->renderer[i] = NULL;
		}
	}

	if (self->gl_context)
		SDL_GL_DeleteContext(self->gl_context);
	SDL_Quit();

	return 0;
}


void pdraw_desktop_ui_send_quit_event(void)
{
	SDL_Event event;
	memset(&event, 0, sizeof(event));
	event.type = SDL_QUIT;
	SDL_PushEvent(&event);
}


void pdraw_desktop_ui_send_user_event(struct pdraw_desktop *self,
				      enum pdraw_desktop_event evt,
				      void *data1,
				      void *data2)
{
	SDL_Event event;
	memset(&event, 0, sizeof(event));
	event.type = self->user_event;
	event.user.code = evt;
	event.user.data1 = data1;
	event.user.data2 = data2;
	SDL_PushEvent(&event);
}


static void
get_rect(struct pdraw_desktop *self, struct pdraw_rect *rect, unsigned int idx)
{
	unsigned int layout_idx = self->media_count - 1;
	unsigned int h = layout[layout_idx].h;
	unsigned int v = layout[layout_idx].v;
	unsigned int x = idx % h;
	unsigned int y = v - 1 - idx / h;
	if (idx / h == v - 1) {
		/* Last line */
		idx -= (self->media_count / h) * h;
		h = self->media_count - (self->media_count / h) * h;
		if (h == 0)
			h = layout[layout_idx].h;
		x = idx % h;
	}
	rect->x = self->window_width * x / h;
	rect->y = self->window_height * y / v;
	rect->width = self->window_width / h;
	rect->height = self->window_height / v;
}


void pdraw_desktop_ui_add_media(struct pdraw_desktop *self,
				unsigned int media_id)
{
	int res, inc;
	unsigned int i;

	/* Try to reuse an existing renderer without media */
	for (i = 0; i < self->renderer_count; i++) {
		if ((self->renderer[i] == NULL) ||
		    (self->renderer_media_id[i] != 0))
			continue;
		res = pdraw_be_video_renderer_set_media_id(
			self->pdraw, self->renderer[i], media_id);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_set_media_id",
				   -res);
			return;
		}
		self->renderer_media_id[i] = media_id;
		return;
	}

	/* Find a free renderer slot to use */
	for (i = 0; i < self->renderer_count; i++) {
		if (self->renderer[i] == NULL)
			break;
	}
	inc = i == self->renderer_count;
	if (self->renderer_count >= MAX_RENDERERS)
		return;
	if (self->renderer_count >= self->media_count)
		return;

	/* Create the renderer */
	struct pdraw_video_renderer_params params = {0};
	params.scheduling_mode = self->default_scheduling_mode;
	params.fill_mode = self->default_fill_mode;
	params.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;
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

	get_rect(self, &self->render_pos[i], i);
	res = pdraw_be_video_renderer_new(self->pdraw,
					  media_id,
					  &self->render_pos[i],
					  &params,
					  &render_cbs,
					  self,
					  &self->renderer[i]);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_video_renderer_new", -res);
		return;
	}
	if (self->enable_hud) {
		struct pdraw_gles2hud_config hud_config;
		memset(&hud_config, 0, sizeof(hud_config));
		res = pdraw_gles2hud_new(&hud_config, &self->gles2hud[i]);
		if (res < 0)
			ULOG_ERRNO("pdraw_gles2hud_new", -res);
	}
	res = pdraw_desktop_ext_tex_setup(self);
	if (res < 0)
		ULOG_ERRNO("pdraw_desktop_ext_tex_setup", -res);
	if (inc)
		self->renderer_count++;
	self->renderer_media_id[i] = media_id;
}


void pdraw_desktop_ui_resize(struct pdraw_desktop *self)
{
	for (unsigned int i = 0; i < self->renderer_count; i++) {
		int res;
		get_rect(self, &self->render_pos[i], i);
		res = pdraw_be_video_renderer_resize(
			self->pdraw, self->renderer[i], &self->render_pos[i]);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_video_renderer_resize", -res);
	}
}


int pdraw_desktop_ui_loop(struct pdraw_desktop *self)
{
	int res;
	SDL_Event event;
	float view_mat[16];
	float proj_mat[16];

	while (!self->stopped) {
		while ((!self->stopped) && (SDL_PollEvent(&event))) {
			if (event.type == self->user_event)
				user_event(self, &event.user);
			else
				sdl_event(self, &event);
		}
		glViewport(0, 0, self->window_width, self->window_height);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		for (unsigned int i = 0; i < self->renderer_count; i++) {
			if (self->renderer[i] == NULL)
				continue;
			pdraw_desktop_view_create_matrices(
				self,
				self->render_pos[i].width,
				self->render_pos[i].height,
				view_mat,
				proj_mat,
				0.1f,
				100.f);
			res = pdraw_be_video_renderer_render_mat(
				self->pdraw,
				self->renderer[i],
				NULL,
				view_mat,
				proj_mat);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_video_renderer_render_mat",
					   -res);
		}
		SDL_GL_SwapWindow(self->window);
	}

	return 0;
}
