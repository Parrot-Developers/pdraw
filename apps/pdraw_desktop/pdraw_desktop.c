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

#include "pdraw_desktop.h"

ULOG_DECLARE_TAG(pdraw_desktop);


static const struct pdraw_backend_muxer_cbs muxer_cbs;
static const struct pdraw_backend_demuxer_cbs demuxer_cbs;
static const struct pdraw_backend_vipc_source_cbs source_cbs;


int pdraw_desktop_open(struct pdraw_desktop *self)
{
	int res;
	if (self->url != NULL) {
		if (self->is_vipc) {
			struct pdraw_vipc_source_params params = {
				.address = self->url,
			};
			res = pdraw_be_vipc_source_new(self->pdraw,
						       &params,
						       &source_cbs,
						       self,
						       &self->source);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_vipc_source_new", -res);
			self->media_count = 1;
			self->video_media_count = 1;
		} else {
			struct pdraw_demuxer_params params = {0};
			res = pdraw_be_demuxer_new_from_url(self->pdraw,
							    self->url,
							    &params,
							    &demuxer_cbs,
							    self,
							    &self->demuxer);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_demuxer_new_from_url",
					   -res);
		}
	} else {
		struct pdraw_demuxer_params params = {0};
		res = pdraw_be_demuxer_new_single_stream(
			self->pdraw,
			self->local_addr,
			self->local_stream_port,
			self->local_control_port,
			self->remote_addr,
			self->remote_stream_port,
			self->remote_control_port,
			&params,
			&demuxer_cbs,
			self,
			&self->demuxer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_new_single_stream", -res);
	}
	return res;
}


void pdraw_desktop_close(struct pdraw_desktop *self)
{
	int res;
	for (unsigned int i = 0; i < self->video_renderer_count; i++) {
		if (self->video_renderers[i].gles2hud != NULL) {
			res = pdraw_gles2hud_destroy(
				self->video_renderers[i].gles2hud);
			if (res < 0)
				ULOG_ERRNO("pdraw_gles2hud_destroy", -res);
			self->video_renderers[i].gles2hud = NULL;
		}
		if (self->video_renderers[i].renderer != NULL) {
			res = pdraw_be_video_renderer_destroy(
				self->pdraw, self->video_renderers[i].renderer);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_video_renderer_destroy",
					   -res);
			self->video_renderers[i].renderer = NULL;
		}
	}
	if (self->audio_renderer != NULL) {
		res = pdraw_be_audio_renderer_destroy(self->pdraw,
						      self->audio_renderer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_audio_renderer_destroy", -res);
		self->audio_renderer = NULL;
	}
	if (self->demuxer != NULL) {
		res = pdraw_be_demuxer_close(self->pdraw, self->demuxer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_close", -res);
	}
	if (self->source != NULL) {
		res = pdraw_be_vipc_source_destroy(self->pdraw, self->source);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_vipc_source_destroy", -res);
		else
			self->source = NULL;
	}
	if (self->demuxer == NULL) {
		res = pdraw_be_stop(self->pdraw);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_stop", -res);
	}
}


void pdraw_desktop_toggle_play_pause(struct pdraw_desktop *self)
{
	int res;
	if (self->demuxer != NULL) {
		if (pdraw_be_demuxer_is_paused(self->pdraw, self->demuxer) !=
		    0) {
			res = pdraw_be_demuxer_play_with_speed(
				self->pdraw,
				self->demuxer,
				self->speed * self->speed_sign);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_demuxer_play_with_speed",
					   -res);
		} else {
			res = pdraw_be_demuxer_pause(self->pdraw,
						     self->demuxer);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_demuxer_pause", -res);
		}
	}
	if (self->source != NULL) {
		if (pdraw_be_vipc_source_is_paused(self->pdraw, self->source) !=
		    0) {
			res = pdraw_be_vipc_source_play(self->pdraw,
							self->source);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_vipc_source_play", -res);
		} else {
			res = pdraw_be_vipc_source_pause(self->pdraw,
							 self->source);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_vipc_source_pause", -res);
		}
	}
}


void pdraw_desktop_toggle_speed_sign(struct pdraw_desktop *self)
{
	int res;
	if (self->demuxer == NULL)
		return;
	self->speed_sign *= -1;
	if (pdraw_be_demuxer_is_paused(self->pdraw, self->demuxer) == 0) {
		res = pdraw_be_demuxer_play_with_speed(
			self->pdraw,
			self->demuxer,
			self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_play_with_speed", -res);
	}
}


void pdraw_desktop_speed_down(struct pdraw_desktop *self)
{
	int res;
	if (self->demuxer == NULL)
		return;
	self->speed /= 2;
	if (pdraw_be_demuxer_is_paused(self->pdraw, self->demuxer) == 0) {
		res = pdraw_be_demuxer_play_with_speed(
			self->pdraw,
			self->demuxer,
			self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_play_with_speed", -res);
	}
}


void pdraw_desktop_speed_up(struct pdraw_desktop *self)
{
	int res;
	if (self->demuxer == NULL)
		return;
	self->speed *= 2;
	if (pdraw_be_demuxer_is_paused(self->pdraw, self->demuxer) == 0) {
		res = pdraw_be_demuxer_play_with_speed(
			self->pdraw,
			self->demuxer,
			self->speed * self->speed_sign);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_play_with_speed", -res);
	}
}


void pdraw_desktop_previous_frame(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_previous_frame(self->pdraw, self->demuxer);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_previous_frame", -res);
}


void pdraw_desktop_next_frame(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_next_frame(self->pdraw, self->demuxer);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_next_frame", -res);
}


void pdraw_desktop_seek_back_10s(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_seek_back(
		self->pdraw, self->demuxer, 10000000, 0);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_seek_back", -res);
}


void pdraw_desktop_seek_forward_10s(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_seek_forward(
		self->pdraw, self->demuxer, 10000000, 0);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_seek_forward", -res);
}


void pdraw_desktop_seek_to_prev_chapter(struct pdraw_desktop *self)
{
	uint64_t cur_ts;
	/* Delta timestamp (us) to skip current chapter when seeking to previous
	 * chapter */
	uint64_t delta_ts = 1 * 1000 * 1000; /* 1 sec */
	if (self->demuxer == NULL)
		return;

	if (self->chapter_count == 0) {
		ULOGI("%s: no chapters", __func__);
		return;
	}

	cur_ts = pdraw_be_demuxer_get_current_time(self->pdraw, self->demuxer);
	if (cur_ts == 0)
		return;

	for (int i = (int)self->chapter_count - 1; i >= 0; i--) {
		struct pdraw_chapter chap = self->chapter_list[i];
		if ((chap.ts_us + delta_ts) >= cur_ts)
			continue;
		/* Chapter found before ts: seek */
		int err = pdraw_be_demuxer_seek_to(
			self->pdraw, self->demuxer, chap.ts_us, 0);
		if (err < 0)
			ULOG_ERRNO("pdraw_be_demuxer_seek_to", -err);
		else
			ULOGI("seek to prev chapter #%d: '%s'", i, chap.name);
		break;
	}
}


void pdraw_desktop_seek_to_next_chapter(struct pdraw_desktop *self)
{
	uint64_t cur_ts;
	if (self->demuxer == NULL)
		return;

	if (self->chapter_count == 0) {
		ULOGI("%s: no chapters", __func__);
		return;
	}

	cur_ts = pdraw_be_demuxer_get_current_time(self->pdraw, self->demuxer);
	if (cur_ts == 0)
		return;

	for (int i = 0; i < (int)self->chapter_count; i++) {
		struct pdraw_chapter chap = self->chapter_list[i];
		if (chap.ts_us <= cur_ts)
			continue;
		/* Chapter found after ts: seek */
		int err = pdraw_be_demuxer_seek_to(
			self->pdraw, self->demuxer, chap.ts_us, 0);
		if (err < 0)
			ULOG_ERRNO("pdraw_be_demuxer_seek_to", -err);
		else
			ULOGI("seek to next chapter #%d: '%s'", i, chap.name);
		break;
	}
}


void pdraw_desktop_goto_beginning(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_seek_to(self->pdraw, self->demuxer, 0, 1);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_seek_to", -res);
}


void pdraw_desktop_goto_end(struct pdraw_desktop *self)
{
	if (self->demuxer == NULL)
		return;
	int res = pdraw_be_demuxer_seek_to(
		self->pdraw, self->demuxer, (uint64_t)-1, 1);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_seek_to", -res);
}


void pdraw_desktop_toggle_start_stop_recorder(struct pdraw_desktop *self)
{
	int res;
	if (self->recorder == NULL) {
		uint64_t epoch_sec = 0;
		int32_t utc_offset_sec = 0;
		struct tm tm;
		char *file_path = NULL;
		struct pdraw_muxer_params muxer_params = {0};

		time_local_get(&epoch_sec, &utc_offset_sec);
		time_local_to_tm(epoch_sec, utc_offset_sec, &tm);

		res = asprintf(&file_path,
			       "pdraw_rec_%04d%02d%02d_%02d%02d%02d_%d.mp4",
			       tm.tm_year + 1900,
			       tm.tm_mon + 1,
			       tm.tm_mday,
			       tm.tm_hour,
			       tm.tm_min,
			       tm.tm_sec,
			       getpid());
		if (res <= 0) {
			ULOG_ERRNO("asprintf", ENOMEM);
			return;
		}

		res = pdraw_be_muxer_new(self->pdraw,
					 file_path,
					 &muxer_params,
					 &muxer_cbs,
					 self,
					 &self->recorder);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_muxer_new", -res);
			free(file_path);
			return;
		}
		free(file_path);

		if (self->recorder_media_id == 0) {
			/* No media yet, nothing more to do */
			return;
		}

		ULOGI("adding media %d to MP4 muxer", self->recorder_media_id);
		res = pdraw_be_muxer_add_media(self->pdraw,
					       self->recorder,
					       self->recorder_media_id,
					       NULL);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_muxer_add_media", -res);
			return;
		}
	} else {
		res = pdraw_be_muxer_close(self->pdraw, self->recorder);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_muxer_close", -res);
			res = pdraw_be_muxer_destroy(self->pdraw,
						     self->recorder);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_muxer_destroy", -res);
			self->recorder = NULL;
			return;
		}
	}
}


void pdraw_desktop_dump_pipeline(struct pdraw_desktop *self)
{
	int res;
	uint64_t epoch_sec = 0;
	int32_t utc_offset_sec = 0;
	struct tm tm;
	char *file_path = NULL;

	time_local_get(&epoch_sec, &utc_offset_sec);
	time_local_to_tm(epoch_sec, utc_offset_sec, &tm);

	res = asprintf(&file_path,
		       "pdraw_pipeline_%04d%02d%02d_%02d%02d%02d_%d.dot",
		       tm.tm_year + 1900,
		       tm.tm_mon + 1,
		       tm.tm_mday,
		       tm.tm_hour,
		       tm.tm_min,
		       tm.tm_sec,
		       getpid());
	if (res <= 0) {
		ULOG_ERRNO("asprintf", ENOMEM);
		return;
	}

	res = pdraw_be_dump_pipeline(self->pdraw, file_path);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_dump_pipeline", -res);

	free(file_path);
}


void pdraw_desktop_change_scheduling_mode(struct pdraw_desktop *self)
{
	int err;
	unsigned int i;
	struct pdraw_video_renderer_params params;

	for (i = 0; i < self->video_renderer_count; i++) {
		err = pdraw_be_video_renderer_get_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_get_params(%u)",
				   -err,
				   i);
			return;
		}
		params.scheduling_mode++;
		if (params.scheduling_mode >=
		    PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_MAX) {
			params.scheduling_mode =
				PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP;
		}
		err = pdraw_be_video_renderer_set_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_set_params(%u)",
				   -err,
				   i);
			return;
		}
		ULOGI("set video renderer %u mode to %s",
		      i,
		      pdraw_video_renderer_scheduling_mode_str(
			      params.scheduling_mode));
	}
}


void pdraw_desktop_toggle_demuxer_media(struct pdraw_desktop *self)
{
	int err = 0;
	struct pdraw_demuxer_media *media_list = NULL;
	size_t media_count = 0;
	uint32_t selected_medias = 0;
	uint32_t new_selected_medias = 0;
	int selected_list[10] = {};
	size_t selected_list_count = 0;
	int allowed_list[10] = {};
	size_t allowed_list_count = 0;

	err = pdraw_be_demuxer_get_media_list(self->pdraw,
					      self->demuxer,
					      &media_list,
					      &media_count,
					      &selected_medias);
	if (err < 0) {
		ULOG_ERRNO("pdraw_be_demuxer_get_media_list", -err);
		return;
	}

	for (size_t i = 0; i < media_count; i++) {
		int cur_media_id = media_list[i].media_id;
		/* Skip non-video medias */
		if (!(media_list[i].type == PDRAW_MEDIA_TYPE_VIDEO))
			continue;
		allowed_list[allowed_list_count] = cur_media_id;
		allowed_list_count++;
		if (!(selected_medias & (1 << cur_media_id)))
			continue;
		selected_list[selected_list_count] = cur_media_id;
		selected_list_count++;
	}
	if (selected_list_count >= allowed_list_count) {
		ULOGW("all allowed media are selected, cannot toggle");
		goto out;
	}

	for (size_t i = 0; i < allowed_list_count; i++)
		ULOGD(" allowed_list[%zu] = %d", i, allowed_list[i]);
	for (size_t i = 0; i < selected_list_count; i++)
		ULOGD("selected_list[%zu] = %d", i, selected_list[i]);

	for (size_t i = 0; i < selected_list_count; i++) {
		for (size_t j = 0; i < allowed_list_count; j++) {
			int next_allowed_idx = (j + 1) % allowed_list_count;
			int next_allowed_media = allowed_list[next_allowed_idx];
			if (((next_allowed_idx > (int)j) &&
			     (next_allowed_media > selected_list[i])) ||
			    ((next_allowed_idx < (int)j) &&
			     (next_allowed_media < selected_list[i]))) {
				ULOGD("updating selected_list[%zu] (=%d) to %d",
				      i,
				      selected_list[i],
				      next_allowed_media);
				selected_list[i] = next_allowed_media;
				goto next;
			}
		}
		/* clang-format off */
next:
		/* clang-format on */
		continue;
	}

	for (size_t i = 0; i < selected_list_count; i++)
		new_selected_medias |= (1 << selected_list[i]);

	ULOGD("select media: %x", new_selected_medias);
	err = pdraw_be_demuxer_select_media(
		self->pdraw, self->demuxer, new_selected_medias);
	if (err < 0)
		ULOG_ERRNO("pdraw_be_demuxer_get_media_list", -err);

out:
	/* Release media_list */
	for (unsigned int i = 0; i < media_count; i++) {
		free((void *)media_list[i].name);
		free((void *)media_list[i].uri);
	}
	free(media_list);
}


void pdraw_desktop_change_fill_mode(struct pdraw_desktop *self)
{
	int err;
	unsigned int i;
	struct pdraw_video_renderer_params params;

	for (i = 0; i < self->video_renderer_count; i++) {
		err = pdraw_be_video_renderer_get_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_get_params(%u)",
				   -err,
				   i);
			return;
		}
		params.fill_mode++;
		if (params.fill_mode >= PDRAW_VIDEO_RENDERER_FILL_MODE_MAX)
			params.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT;
		err = pdraw_be_video_renderer_set_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_set_params(%u)",
				   -err,
				   i);
			return;
		}
		ULOGI("set video renderer %u mode to %s",
		      i,
		      pdraw_video_renderer_fill_mode_str(params.fill_mode));
	}
}


void pdraw_desktop_toggle_mb_status(struct pdraw_desktop *self)
{
	int err;
	unsigned int i;
	struct pdraw_video_renderer_params params;
	uint32_t dbg_flags = 0;
	const char *env_dbg_flags = getenv(PDRAW_VIDEO_RENDERER_DBG_FLAGS);
	if (env_dbg_flags != NULL) {
		char *endptr = NULL;
		errno = 0;
		long parsedint = strtol(env_dbg_flags, &endptr, 0);
		if (env_dbg_flags[0] == '\0' || endptr[0] != '\0' ||
		    parsedint < 0 || errno != 0) {
			err = -errno;
			ULOG_ERRNO("strtol: %s", -err, env_dbg_flags);
			return;
		}
		dbg_flags = parsedint;
	}
	dbg_flags ^= PDRAW_VIDEO_RENDERER_DBG_FLAG_MB_STATUS_OVERLAY;
	char new_dbg_flags[12];
	err = snprintf(new_dbg_flags, sizeof(new_dbg_flags), "%u", dbg_flags);
	if (err <= 0) {
		err = -errno;
		ULOG_ERRNO("snprintf", -err);
		return;
	}

#ifndef _WIN32
	err = setenv(PDRAW_VIDEO_RENDERER_DBG_FLAGS, new_dbg_flags, 1);
	if (err < 0) {
		err = -errno;
		ULOG_ERRNO("setenv", -err);
		return;
	}
#else
	err = SetEnvironmentVariableA(PDRAW_VIDEO_RENDERER_DBG_FLAGS,
				      new_dbg_flags);
	if (err == 0) {
		err = -EPROTO;
		ULOG_ERRNO("SetEnvironmentVariableA", -err);
		return;
	}
#endif

	ULOGI("toggle MB status overlay %s",
	      (dbg_flags & PDRAW_VIDEO_RENDERER_DBG_FLAG_MB_STATUS_OVERLAY)
		      ? "ON"
		      : "OFF");

	/* Reload renderer params to apply changes */
	for (i = 0; i < self->video_renderer_count; i++) {
		err = pdraw_be_video_renderer_get_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_get_params(%u)",
				   -err,
				   i);
			return;
		}
		err = pdraw_be_video_renderer_set_params(
			self->pdraw,
			self->video_renderers[i].renderer,
			&params);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_video_renderer_set_params(%u)",
				   -err,
				   i);
			return;
		}
	}
}


static void
stop_resp_cb(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_STOP_RESP,
					 (void *)(intptr_t)status,
					 NULL);
}


static void media_added_cb(struct pdraw_backend *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_userdata,
			   void *userdata)
{
	int res;
	struct pdraw_desktop *self = userdata;
	ULOGI("%s id=%d path=%s", __func__, info->id, info->path);

	if ((self->recorder_media_id == 0) &&
	    (info->type == PDRAW_MEDIA_TYPE_VIDEO) &&
	    (info->video.format == VDEF_FRAME_TYPE_CODED) &&
	    (info->video.coded.format.encoding == VDEF_ENCODING_H264)) {
		self->recorder_media_id = info->id;
		if (self->recorder != NULL) {
			ULOGI("adding media %d to MP4 muxer",
			      self->recorder_media_id);
			res = pdraw_be_muxer_add_media(self->pdraw,
						       self->recorder,
						       self->recorder_media_id,
						       NULL);
			if (res < 0) {
				ULOG_ERRNO("pdraw_be_muxer_add_media", -res);
				return;
			}
		}
	}

	if (((info->type == PDRAW_MEDIA_TYPE_VIDEO) &&
	     (info->video.format == VDEF_FRAME_TYPE_RAW)) ||
	    (info->type == PDRAW_MEDIA_TYPE_AUDIO &&
	     info->audio.format.encoding == ADEF_ENCODING_PCM)) {
		pdraw_desktop_ui_send_user_event(
			self,
			PDRAW_DESKTOP_EVENT_ADD_RENDERER,
			(void *)(intptr_t)info->id,
			(void *)(intptr_t)info->type);
	}
}


static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	struct pdraw_desktop *self = userdata;
	ULOGI("%s id=%d path=%s", __func__, info->id, info->path);
	/* Cleanup any pending media */
	for (unsigned int i = 0; i < self->video_renderer_count; i++) {
		if (self->video_renderers[i].pending_media_id == info->id)
			self->video_renderers[i].pending_media_id = 0;
	}
	if (self->video_renderer_pending_media_id == info->id)
		self->video_renderer_pending_media_id = 0;
}


static void
socket_created_cb(struct pdraw_backend *pdraw, int fd, void *userdata)
{
	ULOGI("%s fd=%d", __func__, fd);
}


static void open_resp_cb(struct pdraw_backend *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 void *userdata)
{
	int err;
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_OPEN_RESP,
					 (void *)(intptr_t)status,
					 NULL);

	err = pdraw_be_demuxer_get_chapter_list(
		pdraw, demuxer, &self->chapter_list, &self->chapter_count);
	if (err < 0) {
		if ((err != -ENOENT) && (err != -ENOSYS))
			ULOG_ERRNO("pdraw_be_demuxer_get_chapter_list", -err);
	} else {
		ULOGI("= Chapter list =");
		for (size_t i = 0; i < self->chapter_count; i++) {
			struct pdraw_chapter chap = self->chapter_list[i];
			ULOGI(" # %02d:%02d.%02us: %s",
			      (unsigned int)(chap.ts_us / 1000000) / 60,
			      (unsigned int)(chap.ts_us / 1000000) % 60,
			      (unsigned int)(chap.ts_us / 10000) % 100,
			      chap.name);
		}
		ULOGI("================");
	}
}


static void muxer_unrecoverable_error_cb(struct pdraw_backend *pdraw,
					 struct pdraw_muxer *muxer,
					 int status,
					 void *userdata)
{
	int err;

	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d", __func__, status);

	/* Stop recorder */
	err = pdraw_be_muxer_destroy(self->pdraw, self->recorder);
	if (err < 0) {
		ULOG_ERRNO("pdraw_be_muxer_destroy", -err);
		return;
	}
	self->recorder = NULL;
}


static void muxer_close_resp_cb(struct pdraw_backend *pdraw,
				struct pdraw_muxer *muxer,
				int status,
				void *userdata)
{
	int res;
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	if (self->recorder != NULL) {
		res = pdraw_be_muxer_destroy(self->pdraw, self->recorder);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_muxer_destroy", -res);
		self->recorder = NULL;
	}
}


static void close_resp_cb(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
			  int status,
			  void *userdata)
{
	int res;
	struct pdraw_desktop *self = userdata;

	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));

	if (self->demuxer != NULL) {
		res = pdraw_be_demuxer_destroy(self->pdraw, self->demuxer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_demuxer_destroy", -res);
		self->demuxer = NULL;
	}
	if (self->chapter_list != NULL) {
		for (size_t i = 0; i < self->chapter_count; i++) {
			struct pdraw_chapter chap = self->chapter_list[i];
			free((void *)chap.name);
		}
		free(self->chapter_list);
	}
	res = pdraw_be_stop(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_stop", -res);
}


static void unrecoverable_error_cb(struct pdraw_backend *pdraw,
				   struct pdraw_demuxer *demuxer,
				   void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s", __func__);

	pdraw_desktop_ui_send_user_event(
		self, PDRAW_DESKTOP_EVENT_UNRECOVERABLE_ERROR, NULL, NULL);
}


static int select_media_cb(struct pdraw_backend *pdraw,
			   struct pdraw_demuxer *demuxer,
			   const struct pdraw_demuxer_media *medias,
			   size_t count,
			   uint32_t selected_medias,
			   void *userdata)
{
	struct pdraw_desktop *self = userdata;
	int id, ids = 0;
	char s[100];
	char *str = s, *id_str = NULL, *temp = NULL;
	unsigned int default_media_count = 0;
	unsigned int default_video_media_count = 0;
	size_t slen;
	bool found = false;

	if (count == 0)
		return -ENOENT;
	if (count == 1) {
		self->media_count = 1;
		self->video_media_count =
			(medias[0].type == PDRAW_MEDIA_TYPE_VIDEO);
		return 1 << medias[0].media_id;
	}

	for (size_t i = 0; i < count; i++) {
		if (medias[i].is_default) {
			default_media_count++;
			if (medias[i].type == PDRAW_MEDIA_TYPE_VIDEO)
				default_video_media_count++;
		}
	}

	if (self->demuxer_media_list) {
		str = self->demuxer_media_list;
		goto parse;
	}

	printf("Select demuxer media id(s);\n"
	       "either empty/zero for the default media set, "
	       "a single media id (e.g. \"1\")\n"
	       "or a comma-separated list of media ids (e.g. \"1,3\"):\n");
	for (size_t i = 0; i < count; i++) {
		switch (medias[i].type) {
		case PDRAW_MEDIA_TYPE_AUDIO: {
			printf(" %c [%c] %d (%s): %s\n",
			       medias[i].is_default ? '*' : '-',
			       ((1 << medias[i].media_id) & selected_medias)
				       ? 's'
				       : ' ',
			       medias[i].media_id,
			       pdraw_media_type_str(medias[i].type),
			       medias[i].name);
			break;
		}
		case PDRAW_MEDIA_TYPE_VIDEO: {
			printf(" %c [%c] %d (%s): %s [%s]%s%s%s\n",
			       medias[i].is_default ? '*' : '-',
			       ((1 << medias[i].media_id) & selected_medias)
				       ? 's'
				       : ' ',
			       medias[i].media_id,
			       pdraw_media_type_str(medias[i].type),
			       medias[i].name,
			       vmeta_camera_type_to_str(
				       medias[i]
					       .video.session_meta.camera_type),
			       (medias[i].video.session_meta.camera_spectrum !=
				VMETA_CAMERA_SPECTRUM_UNKNOWN)
				       ? "["
				       : "",
			       (medias[i].video.session_meta.camera_spectrum !=
				VMETA_CAMERA_SPECTRUM_UNKNOWN)
				       ? vmeta_camera_spectrum_to_str(
						 medias[i]
							 .video.session_meta
							 .camera_spectrum)
				       : "",
			       (medias[i].video.session_meta.camera_spectrum !=
				VMETA_CAMERA_SPECTRUM_UNKNOWN)
				       ? "]"
				       : "");
			break;
		}
		default: {
			ULOGE("invalid media type: %s",
			      pdraw_media_type_str(medias[i].type));
			break;
		}
		}
	}
	printf(" > ");
	if (!fgets(s, sizeof(s), stdin))
		goto use_default_media;
	slen = strlen(s);
	if (slen > 0 && s[slen - 1] == '\n')
		s[slen - 1] = '\0';
	slen = strlen(s);
	if (slen == 0) {
		self->media_count = default_media_count;
		self->video_media_count = default_video_media_count;
		return 0;
	}

parse:
	if (strchr(str, ',')) {
		/* The string has multiple parts */
		id_str = strtok_r(str, ",", &temp);
		if (id_str == NULL) {
			/* strtok_r returning NULL means that the string is
			 * empty */
			goto use_default_media;
		}
		while (id_str) {
			found = false;
			id = atoi(id_str);
			ids |= (1 << id);
			self->media_count++;
			for (size_t i = 0; i < count; i++) {
				if (medias[i].media_id == id) {
					found = true;
					if (medias[i].type ==
					    PDRAW_MEDIA_TYPE_VIDEO) {
						self->video_media_count++;
					}
					break;
				}
			}
			if (!found)
				goto use_default_media;
			id_str = strtok_r(NULL, ",", &temp);
		}
	} else {
		/* Only a single number */
		id = atoi(str);
		if (strcmp(str, "all") == 0) {
			self->media_count = count;
			self->video_media_count = 0;
			for (size_t i = 0; i < count; i++) {
				ids |= (1 << medias[i].media_id);
				if (medias[i].type == PDRAW_MEDIA_TYPE_VIDEO)
					self->video_media_count++;
			}
		} else if (id == 0) {
			ids = 0;
			self->media_count = default_media_count;
			self->video_media_count = default_video_media_count;
		} else if (id == -1) {
			/* Keep currently selected medias */
			if (selected_medias != 0)
				ids = selected_medias;
			else
				goto use_default_media;
		} else {
			ids = (1 << id);
			self->media_count = 1;
			self->video_media_count = 0;
			for (size_t i = 0; i < count; i++) {
				if (medias[i].media_id == id) {
					found = true;
					if (medias[i].type ==
					    PDRAW_MEDIA_TYPE_VIDEO) {
						self->video_media_count++;
					}
					break;
				}
			}
			if (!found)
				goto use_default_media;
		}
	}
	return ids;

use_default_media:
	printf("Unable to read input, using default media\n");
	self->media_count = default_media_count;
	self->video_media_count = default_video_media_count;
	return 0;
}


static void ready_to_play_cb(struct pdraw_backend *pdraw,
			     struct pdraw_demuxer *demuxer,
			     int ready,
			     void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s ready=%d", __func__, ready);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_READY_TO_PLAY,
					 (void *)(intptr_t)ready,
					 NULL);
}


static void end_of_range_cb(struct pdraw_backend *pdraw,
			    struct pdraw_demuxer *demuxer,
			    uint64_t timestamp,
			    void *userdata)
{
	ULOGI("%s timestamp=%" PRIu64, __func__, timestamp);
}


static void play_resp_cb(struct pdraw_backend *pdraw,
			 struct pdraw_demuxer *demuxer,
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
			  struct pdraw_demuxer *demuxer,
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
			 struct pdraw_demuxer *demuxer,
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
source_ready_to_play_cb(struct pdraw_backend *pdraw,
			struct pdraw_vipc_source *source,
			int ready,
			enum pdraw_vipc_source_eos_reason eos_reason,
			void *userdata)
{
	struct pdraw_desktop *self = userdata;

	ULOGI("%s ready=%d", __func__, ready);

	pdraw_desktop_ui_send_user_event(self,
					 PDRAW_DESKTOP_EVENT_READY_TO_PLAY,
					 (void *)(intptr_t)ready,
					 NULL);
}


static void source_configured_cb(struct pdraw_backend *pdraw,
				 struct pdraw_vipc_source *source,
				 int status,
				 const struct vdef_format_info *info,
				 const struct vdef_rectf *crop,
				 void *userdata)
{
	ULOGI("%s: status=%d(%s)", __func__, status, strerror(status));
}


static void video_renderer_media_added_cb(struct pdraw_backend *pdraw,
					  struct pdraw_video_renderer *renderer,
					  const struct pdraw_media_info *info,
					  void *userdata)
{
	struct pdraw_desktop *self = userdata;
	ULOGI("%s id=%d path=%s", __func__, info->id, info->path);
	for (unsigned int i = 0; i < self->video_renderer_count; i++) {
		if (self->video_renderers[i].pending_media_id != info->id)
			continue;
		/* Pending media has been added */
		self->video_renderers[i].media_id = info->id;
		self->video_renderers[i].pending_media_id = 0;
		break;
	}
}


static void
video_renderer_media_removed_cb(struct pdraw_backend *pdraw,
				struct pdraw_video_renderer *renderer,
				const struct pdraw_media_info *info,
				int restart,
				void *userdata)
{
	struct pdraw_desktop *self = userdata;
	ULOGI("%s id=%d path=%s", __func__, info->id, info->path);
	for (unsigned int i = 0; i < self->video_renderer_count; i++) {
		if (self->video_renderers[i].media_id != info->id)
			continue;
		self->video_renderers[i].media_id = 0;
		if (self->video_renderer_pending_media_id != 0) {
			/* Add pending media */
			ULOGI("%s: add pending media id: %d",
			      __func__,
			      self->video_renderer_pending_media_id);
			pdraw_desktop_ui_send_user_event(
				self,
				PDRAW_DESKTOP_EVENT_ADD_RENDERER,
				(void *)(intptr_t)
					self->video_renderer_pending_media_id,
				(void *)(intptr_t)PDRAW_MEDIA_TYPE_VIDEO);
		}
		break;
	}
}


static void
audio_renderer_media_removed_cb(struct pdraw_backend *pdraw,
				struct pdraw_audio_renderer *renderer,
				const struct pdraw_media_info *info,
				void *userdata)
{
	/* Nothing to do here */
	return;
}


static int load_texture_cb(struct pdraw_backend *pdraw,
			   struct pdraw_video_renderer *video_renderer,
			   unsigned int texture_width,
			   unsigned int texture_height,
			   const struct pdraw_media_info *media_info,
			   struct mbuf_raw_video_frame *frame,
			   const void *frame_userdata,
			   size_t frame_userdata_len,
			   void *userdata)
{
	struct pdraw_desktop *self = userdata;
	return pdraw_desktop_ext_tex_load(self,
					  pdraw,
					  video_renderer,
					  media_info,
					  frame,
					  frame_userdata,
					  frame_userdata_len);
}


static void render_overlay_cb(struct pdraw_backend *pdraw,
			      struct pdraw_video_renderer *video_renderer,
			      const struct pdraw_rect *render_pos,
			      const struct pdraw_rect *content_pos,
			      const float *view_mat,
			      const float *proj_mat,
			      const struct pdraw_media_info *media_info,
			      struct vmeta_frame *frame_meta,
			      const struct pdraw_video_frame_extra *frame_extra,
			      void *userdata)
{
	int res;
	struct pdraw_desktop *self = userdata;
	struct pdraw_gles2hud *gles2hud = NULL;
#ifdef BUILD_LIBPDRAW_OVERLAYER
	struct pdraw_overlayer *overlayer = NULL;
#endif
	float view_proj_mat[16] = {
		1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.};

	/* Find the corresponding gles2hud instance */
	for (unsigned int i = 0; i < self->video_renderer_count; i++) {
		if (self->video_renderers[i].renderer != video_renderer)
			continue;
		gles2hud = self->video_renderers[i].gles2hud;
#ifdef BUILD_LIBPDRAW_OVERLAYER
		overlayer = self->video_renderers[i].overlayer;
#endif
		break;
	}

#ifdef BUILD_LIBPDRAW_OVERLAYER
	if (overlayer != NULL) {
		res = pdraw_overlayer_render(overlayer,
					     render_pos,
					     content_pos,
					     media_info,
					     frame_meta,
					     frame_extra);
		if (res < 0)
			ULOG_ERRNO("pdraw_overlayer_render", -res);
		return;
	}
#endif

	if (gles2hud == NULL)
		return;

	/* Don't render if metadata are not available */
	if (media_info == NULL || frame_meta == NULL || frame_extra == NULL)
		return;

	struct pdraw_gles2hud_controller_meta ctrl_meta;
	memset(&ctrl_meta, 0, sizeof(ctrl_meta));
	ctrl_meta.battery_percentage = self->skyctrl_battery_percentage;
	ctrl_meta.location = self->skyctrl_location;

	res = pdraw_gles2hud_render(gles2hud,
				    self->hud_type,
				    render_pos,
				    content_pos,
				    view_proj_mat,
				    media_info,
				    frame_meta,
				    frame_extra,
				    &ctrl_meta);
	if (res < 0)
		ULOG_ERRNO("pdraw_gles2hud_render", -res);
}


static const struct pdraw_backend_cbs be_cbs = {
	.stop_resp = &stop_resp_cb,
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
	.socket_created = &socket_created_cb,
};


static const struct pdraw_backend_muxer_cbs muxer_cbs = {
	.unrecoverable_error = &muxer_unrecoverable_error_cb,
	.close_resp = &muxer_close_resp_cb,
};


static const struct pdraw_backend_demuxer_cbs demuxer_cbs = {
	.open_resp = &open_resp_cb,
	.close_resp = &close_resp_cb,
	.unrecoverable_error = &unrecoverable_error_cb,
	.select_media = &select_media_cb,
	.ready_to_play = &ready_to_play_cb,
	.end_of_range = &end_of_range_cb,
	.play_resp = &play_resp_cb,
	.pause_resp = &pause_resp_cb,
	.seek_resp = &seek_resp_cb,
};


static const struct pdraw_backend_vipc_source_cbs source_cbs = {
	.ready_to_play = &source_ready_to_play_cb,
	.configured = &source_configured_cb,
};


const struct pdraw_backend_video_renderer_cbs render_cbs = {
	.media_added = &video_renderer_media_added_cb,
	.media_removed = &video_renderer_media_removed_cb,
	.load_texture = &load_texture_cb,
	.render_overlay = &render_overlay_cb,
};


const struct pdraw_backend_audio_renderer_cbs audio_render_cbs = {
	.media_removed = &audio_renderer_media_removed_cb,
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
	printf("Parrot Drones Audio and Video Vector - "
	       "desktop player application\n"
	       "Copyright (c) 2016 Aurelien Barre\n"
	       "Copyright (c) 2017 Parrot Drones SAS\n\n");
}


enum args_id {
	ARGS_ID_ZEBRAS = 256,
	ARGS_ID_NORM,
	ARGS_ID_EXT_TEX,
	ARGS_ID_DEMUX,
	ARGS_ID_SCHEDMODE,
	ARGS_ID_FILLMODE,
};


static const char short_options[] = "hu:i:s:c:S:C:FTH:O:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"url", required_argument, NULL, 'u'},
	{"ip", required_argument, NULL, 'i'},
	{"lstrmp", required_argument, NULL, 's'},
	{"lctrlp", required_argument, NULL, 'c'},
	{"rstrmp", required_argument, NULL, 'S'},
	{"rctrlp", required_argument, NULL, 'C'},
	{"demux", required_argument, NULL, ARGS_ID_DEMUX},
	{"fullscreen", no_argument, NULL, 'F'},
	{"always-on-top", no_argument, NULL, 'T'},
	{"hud", required_argument, NULL, 'H'},
	{"overlay", required_argument, NULL, 'O'},
	{"zebras", required_argument, NULL, ARGS_ID_ZEBRAS},
	{"norm", no_argument, NULL, ARGS_ID_NORM},
	{"ext-tex", no_argument, NULL, ARGS_ID_EXT_TEX},
	{"tex-mex", no_argument, NULL, ARGS_ID_EXT_TEX},
	{"sched-mode", required_argument, NULL, ARGS_ID_SCHEDMODE},
	{"fill-mode", required_argument, NULL, ARGS_ID_FILLMODE},
	{0, 0, 0, 0},
};


static void usage(char *prog_name)
{
	printf("Usage: %s [options]\n\n"
	       "Options:\n\n"
	       "  -h | --help                      "
	       "Print this message\n\n"
	       "  -u | --url <url>                 "
	       "Stream URL (rtsp://*)"
#ifdef BUILD_LIBVIDEO_IPC
	       ", VIPC address (unix:*)"
#endif
	       " or filename (*.mp4)\n\n"
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
	       "       --demux <list>              "
	       "Optional demuxer media selection list, 0 for default, \"all\" "
	       "to select all medias (e.g. \"0\" or \"1,3,5\" or \"all\");\n"
	       "                                   "
	       "if not specified the user will be prompted if necessary\n\n"
	       "  -F | --fullscreen                "
	       "Start in full-screen mode\n\n"
	       "  -T | --always-on-top             "
	       "Force window to stay on top\n\n"
	       "  -H | --hud <type>                "
	       "Enable the HUD ("
#ifdef BUILD_LIBPDRAW_OVERLAYER
	       "exclusive with the overlay; "
#endif
	       "type: 0=piloting, 1=imaging, 2=tracking)\n\n"
#ifdef BUILD_LIBPDRAW_OVERLAYER
	       "  -O | --overlay <layout>          "
	       "Enable the video overlay (exclusive with the HUD; "
	       "layout: 0=stream_sharing, 1=stream_sharing_simplified, 2=debug)"
	       "\n\n"
#endif
	       "       --zebras <threshold>        "
	       "Enable overexposure zebras (threshold: [0.0 .. 1.0],\n"
	       "                                   "
	       "default: 0.95, out of range means default)\n\n"
	       "       --norm                      "
	       "Enable automatic normalization of the video\n"
	       "                                   "
	       "(e.g. for raw thermal video tracks in records)\n\n"
	       "       --ext-tex                   "
	       "Enable testing the external texture loading callback\n\n"
	       "       --sched-mode                "
	       "Set default video renderer scheduling mode (ASAP, ADAPTIVE)\n\n"
	       "       --fill-mode                 "
	       "Set default video renderer fill-mode (FIT, CROP, "
	       "FIT_PAD_BLUR_CROP, FIT_PAD_BLUR_EXTEND)\n\n",
	       prog_name);
}


static int summary(struct pdraw_desktop *self)
{
	if (self->is_file) {
		printf("Offline playing of file '%s'\n\n", self->url);
	} else if (self->is_vipc) {
		printf("Playing from video IPC '%s'\n\n", self->url);
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


static enum pdraw_video_renderer_scheduling_mode
parse_scheduling_mode(const char *value)
{
	enum pdraw_video_renderer_scheduling_mode fm;
	for (fm = 0; fm < PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_MAX; fm++) {
		if (strcasecmp(value,
			       pdraw_video_renderer_scheduling_mode_str(fm)) ==
		    0)
			return fm;
	}
	fm = PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE;
	printf("Invalid fill mode value '%s', using '%s' instead",
	       value,
	       pdraw_video_renderer_scheduling_mode_str(fm));
	return fm;
}


static enum pdraw_video_renderer_fill_mode parse_fill_mode(const char *value)
{
	enum pdraw_video_renderer_fill_mode fm;
	for (fm = 0; fm < PDRAW_VIDEO_RENDERER_FILL_MODE_MAX; fm++) {
		if (strcasecmp(value, pdraw_video_renderer_fill_mode_str(fm)) ==
		    0)
			return fm;
	}
	fm = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	printf("Invalid fill mode value '%s', using '%s' instead",
	       value,
	       pdraw_video_renderer_fill_mode_str(fm));
	return fm;
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
	self->default_scheduling_mode =
		PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE;
	self->default_fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;

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
			if ((self->url) && (strlen(self->url) > 5) &&
			    (strncmp(self->url, "unix:", 5) == 0)) {
#ifdef BUILD_LIBVIDEO_IPC
				self->is_vipc = 1;
#else
				self->is_vipc = 0;
#endif
			} else if ((self->url) &&
				   ((strlen(self->url) <= 7) ||
				    (strncmp(self->url, "rtsp://", 7) != 0))) {
				self->is_file = 1;
			}
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

		case ARGS_ID_DEMUX:
			free(self->demuxer_media_list);
			self->demuxer_media_list = strdup(optarg);
			break;

		case 'F':
			self->fullscreen = 1;
			break;

		case 'T':
			self->always_on_top = 1;
			break;

		case 'H':
			if (self->enable_overlay) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			self->hud_type = atoi(optarg);
			self->enable_hud = 1;
			break;

#ifdef BUILD_LIBPDRAW_OVERLAYER
		case 'O':
			if (self->enable_hud) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			self->enable_overlay = 1;
			self->overlayer_layout = atoi(optarg);
			if (self->overlayer_layout < 0 ||
			    self->overlayer_layout >=
				    PDRAW_OVERLAYER_LAYOUT_MAX_COUNT) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			break;
#endif

		case ARGS_ID_ZEBRAS:
			self->enable_zebras = 1;
			self->zebras_threshold = atof(optarg);
			if ((self->zebras_threshold < 0.f) ||
			    (self->zebras_threshold > 1.f))
				self->zebras_threshold = DEFAULT_ZEBRAS_THRES;
			break;

		case ARGS_ID_NORM:
			self->enable_norm = 1;
			break;

		case ARGS_ID_EXT_TEX:
			self->ext_tex = 1;
			break;

		case ARGS_ID_SCHEDMODE:
			self->default_scheduling_mode =
				parse_scheduling_mode(optarg);
			break;

		case ARGS_ID_FILLMODE:
			self->default_fill_mode = parse_fill_mode(optarg);
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
	res = pdraw_be_set_friendly_name_setting(self->pdraw, APP_NAME);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_friendly_name_setting", -res);
	res = pdraw_be_set_serial_number_setting(self->pdraw,
						 "00000000"); /* TODO */
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_serial_number_setting", -res);
	res = pdraw_be_set_software_version_setting(self->pdraw,
						    "0.0.0"); /* TODO */
	if (res < 0)
		ULOG_ERRNO("pdraw_be_set_software_version_setting", -res);

	pdraw_desktop_ui_send_user_event(
		self, PDRAW_DESKTOP_EVENT_OPEN, NULL, NULL);

	/* Run the UI loop */
	pdraw_desktop_ui_loop(self);

out:
	if (self != NULL) {
		pdraw_desktop_ext_tex_cleanup(self);
		pdraw_desktop_ui_destroy(self);
		if (self->pdraw != NULL) {
			if (self->recorder != NULL) {
				res = pdraw_be_muxer_destroy(self->pdraw,
							     self->recorder);
				if (res < 0) {
					ULOG_ERRNO("pdraw_be_muxer_destroy",
						   -res);
				}
			}
			if (self->source != NULL) {
				res = pdraw_be_vipc_source_destroy(
					self->pdraw, self->source);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_vipc_source_destroy",
						-res);
				}
			}
			res = pdraw_be_destroy(self->pdraw);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_destroy", -res);
		}
		free(self->url);
		free(self->local_addr);
		free(self->remote_addr);
		free(self->demuxer_media_list);
		free(self);
	}

#ifdef _WIN32
	/* Cleanup winsock API */
	WSACleanup();
#endif /* _WIN32 */

	printf("\nHasta la vista, PDrAW!\n\n");
	exit(status);
}
