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


static void user_event(struct pdraw_desktop *self, SDL_UserEvent *event)
{
	int status;
	switch (event->code) {
	case PDRAW_DESKTOP_EVENT_OPEN:
		pdraw_desktop_open(self);
		break;
	case PDRAW_DESKTOP_EVENT_OPEN_RESP:
		status = (int)(intptr_t)event->data1;
		if (status < 0)
			pdraw_desktop_close(self);
		break;
	case PDRAW_DESKTOP_EVENT_CLOSE_RESP:
		self->stopped = 1;
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
		if ((event->window.event == SDL_WINDOWEVENT_RESIZED) &&
		    (self->renderer != NULL)) {
			self->window_width = event->window.data1;
			self->window_height = event->window.data2;
			pdraw_desktop_resize(
				self, self->window_width, self->window_height);
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
			if (pdraw_be_is_paused(self->pdraw) == 0)
				pdraw_desktop_seek_back_10s(self);
			else
				pdraw_desktop_previous_frame(self);
			break;
		case SDLK_RIGHT:
			if (pdraw_be_is_paused(self->pdraw) == 0)
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
					    : 0));
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
#endif /* _WIN32 */

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
		if (self->renderer != NULL) {
			pdraw_desktop_view_create_matrices(
				self, view_mat, proj_mat, 0.1f, 100.f);
			res = pdraw_be_render_video_mat(self->pdraw,
							self->renderer,
							NULL,
							view_mat,
							proj_mat);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_render_video_mat", -res);
		}
		SDL_GL_SwapWindow(self->window);
	}

	return 0;
}
