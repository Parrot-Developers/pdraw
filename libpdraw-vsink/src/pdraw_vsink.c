/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

#include <errno.h>
#include <pthread.h>
#include <unistd.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

#define ULOG_TAG pdraw_vsink
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_vsink);

#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw/pdraw.h>


#define UNUSED(x) (void)(x)


struct pdraw_vsink {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	bool starting;
	bool cond_ready;
	bool frame_ready;
	pthread_t thread;
	int thread_launched;
	int thread_should_stop;
	struct pomp_loop *loop;
	struct pdraw *pdraw;
	struct pdraw_demuxer *demuxer;
	struct pdraw_raw_video_sink *sink;
	struct mbuf_raw_video_frame_queue *queue;
	struct pdraw_media_info *media_info;
	char *url;
	enum pdraw_playback_mode playback_mode;
	enum vmeta_camera_type camera_type;
	int result;
	struct pdraw_vsink_cbs cbs;
	void *cbs_userdata;
};


static void *run_loop_thread(void *ptr)
{
	struct pdraw_vsink *self = ptr;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	int err = pthread_setname_np("pdraw_vsink");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#	endif
#else
	int err = pthread_setname_np(pthread_self(), "pdraw_vsink");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);
#endif

	while (!self->thread_should_stop)
		pomp_loop_wait_and_process(self->loop, -1);

	return NULL;
}


static void delete_pdraw_idle(void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	if (self->queue != NULL) {
		struct pomp_evt *evt = NULL;
		res = mbuf_raw_video_frame_queue_get_event(self->queue, &evt);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event",
				   -res);
		} else {
			res = pomp_evt_detach_from_loop(evt, self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_evt_detach_from_loop", -res);
		}
		self->queue = NULL;
	}

	if (self->sink != NULL) {
		res = pdraw_raw_video_sink_destroy(self->pdraw, self->sink);
		if (res < 0)
			ULOG_ERRNO("pdraw_video_sink_destroy", -res);
		self->sink = NULL;
	}

	if (self->demuxer != NULL) {
		res = pdraw_demuxer_destroy(self->pdraw, self->demuxer);
		if (res < 0)
			ULOG_ERRNO("pdraw_demuxer_destroy", -res);
		self->demuxer = NULL;
	}

	res = pdraw_destroy(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_destroy", -res);
	self->pdraw = NULL;
	pthread_mutex_lock(&self->mutex);
	self->cond_ready = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void queue_event_cb(struct pomp_evt *evt, void *userdata)
{
	UNUSED(evt);

	struct pdraw_vsink *self = userdata;
	struct mbuf_raw_video_frame *frame = NULL;
	struct mbuf_ancillary_data *ancillary_data = NULL;
	struct pdraw_video_frame *frame_info = NULL;
	int res;

	if (self->cbs.frame_ready == NULL) {
		pthread_mutex_lock(&self->mutex);
		self->frame_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
		return;
	}

	res = mbuf_raw_video_frame_queue_pop(self->queue, &frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
		goto out;
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillary_data);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data", -res);
		goto out;
	}
	frame_info = (struct pdraw_video_frame *)mbuf_ancillary_data_get_buffer(
		ancillary_data, NULL);

	self->cbs.frame_ready(frame, frame_info, self->cbs_userdata);

out:
	if (ancillary_data != NULL)
		mbuf_ancillary_data_unref(ancillary_data);
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
}


static void vsink_media_added_cb(struct pdraw *pdraw,
				 struct pdraw_raw_video_sink *sink,
				 const struct pdraw_media_info *info,
				 void *userdata)
{
	UNUSED(pdraw);
	UNUSED(sink);
	UNUSED(userdata);

	ULOGI("%s: id=%d", __func__, info->id);
}


static void vsink_media_removed_cb(struct pdraw *pdraw,
				   struct pdraw_raw_video_sink *sink,
				   const struct pdraw_media_info *info,
				   int restart,
				   void *userdata)
{
	UNUSED(pdraw);
	UNUSED(sink);
	UNUSED(userdata);

	ULOGI("%s: id=%d (restart: %d)", __func__, info->id, restart);
}


static void
flush_cb(struct pdraw *pdraw, struct pdraw_raw_video_sink *sink, void *userdata)
{
	UNUSED(userdata);

	int res;
	struct mbuf_raw_video_frame_queue *queue;

	queue = pdraw_raw_video_sink_get_queue(pdraw, sink);
	if (queue == NULL) {
		ULOG_ERRNO("pdraw_raw_video_sink_get_queue", EPROTO);
		return;
	}

	res = mbuf_raw_video_frame_queue_flush(queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -res);
		return;
	}

	res = pdraw_raw_video_sink_queue_flushed(pdraw, sink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_raw_video_sink_queue_flushed", -res);
		return;
	}
}


static void
drain_cb(struct pdraw *pdraw, struct pdraw_raw_video_sink *sink, void *userdata)
{
	UNUSED(userdata);

	int res;
	struct mbuf_raw_video_frame_queue *queue;

	queue = pdraw_raw_video_sink_get_queue(pdraw, sink);
	if (queue == NULL) {
		ULOG_ERRNO("pdraw_raw_video_sink_get_queue", EPROTO);
		return;
	}

	res = mbuf_raw_video_frame_queue_flush(queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -res);
		return;
	}

	res = pdraw_raw_video_sink_queue_drained(pdraw, sink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_raw_video_sink_queue_drained", -res);
		return;
	}
}


static const struct pdraw_raw_video_sink_cbs vsink_cbs = {
	.media_added = &vsink_media_added_cb,
	.media_removed = &vsink_media_removed_cb,
	.flush = &flush_cb,
	.drain = &drain_cb,
};


static void open_resp_cb(struct pdraw *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 void *userdata)
{
	UNUSED(pdraw);
	UNUSED(demuxer);

	struct pdraw_vsink *self = userdata;

	ULOGI("%s: status=%d", __func__, status);

	if (status != 0) {
		pthread_mutex_lock(&self->mutex);
		self->result = status;
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
	}
}


static int select_media_cb(struct pdraw *pdraw,
			   struct pdraw_demuxer *demuxer,
			   const struct pdraw_demuxer_media *medias,
			   size_t count,
			   uint32_t selected_medias,
			   void *userdata)
{
	UNUSED(pdraw);
	UNUSED(demuxer);
	UNUSED(selected_medias);

	struct pdraw_vsink *self = userdata;

	for (size_t i = 0; i < count; i++) {
		if (medias[i].type != PDRAW_MEDIA_TYPE_VIDEO)
			continue;
		if (self->camera_type == VMETA_CAMERA_TYPE_UNKNOWN &&
		    medias[i].is_default) {
			ULOGI("%s: selecting media '%s'",
			      __func__,
			      medias[i].name);
			return 1 << medias[i].media_id;
		}
		if (vmeta_camera_type_subtype_pair_cmp(
			    medias[i].video.session_meta.camera_type,
			    medias[i].video.session_meta.camera_subtype,
			    self->camera_type,
			    VMETA_CAMERA_SUBTYPE_UNKNOWN) == 1) {
			ULOGI("%s: selecting media '%s'",
			      __func__,
			      medias[i].name);
			return 1 << medias[i].media_id;
		}
	}

	ULOGI("%s: no media selected", __func__);
	return -ECANCELED;
}


static void ready_to_play_cb(struct pdraw *pdraw,
			     struct pdraw_demuxer *demuxer,
			     int ready,
			     void *userdata)
{
	UNUSED(pdraw);
	UNUSED(demuxer);

	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("%s: ready=%d", __func__, ready);

	if (ready) {
		res = pdraw_demuxer_play(self->pdraw, self->demuxer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_demuxer_play", -res);
			pthread_mutex_lock(&self->mutex);
			self->result = res;
			self->cond_ready = true;
			pthread_cond_signal(&self->cond);
			pthread_mutex_unlock(&self->mutex);
		}
	}
}


static void play_resp_cb(struct pdraw *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 uint64_t timestamp,
			 float speed,
			 void *userdata)
{
	UNUSED(pdraw);
	UNUSED(demuxer);
	UNUSED(timestamp);
	UNUSED(userdata);

	ULOGI("%s: status=%d speed=%f", __func__, status, speed);

	if (status != 0)
		ULOG_ERRNO("play_resp_cb", -status);
}


static const struct pdraw_demuxer_cbs demuxer_cbs = {
	.open_resp = &open_resp_cb,
	.select_media = &select_media_cb,
	.ready_to_play = &ready_to_play_cb,
	.play_resp = &play_resp_cb,
};


static void stop_resp_cb(struct pdraw *pdraw, int status, void *userdata)
{
	UNUSED(pdraw);

	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("%s: status=%d", __func__, status);

	res = pomp_loop_idle_add(self->loop, delete_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		pthread_mutex_lock(&self->mutex);
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
	}
}


static void media_added_cb(struct pdraw *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_userdata,
			   void *userdata)
{
	UNUSED(pdraw);
	UNUSED(element_userdata);

	int res = 0;
	struct pdraw_vsink *self = userdata;
	struct pomp_evt *evt = NULL;

	if (info->type != PDRAW_MEDIA_TYPE_VIDEO ||
	    info->video.format != VDEF_FRAME_TYPE_RAW)
		return;

	ULOGI("%s: id=%d", __func__, info->id);

	pthread_mutex_lock(&self->mutex);

	if (self->media_info)
		pdraw_media_info_free(self->media_info);
	self->media_info = pdraw_media_info_dup(info);
	if (self->media_info == NULL) {
		ULOG_ERRNO("pdraw_media_info_dup", ENOMEM);
		goto out;
	}

	if (self->sink == NULL) {
		struct pdraw_video_sink_params params;
		memset(&params, 0, sizeof(params));
		params.queue_max_count = 1;
		res = pdraw_raw_video_sink_new(self->pdraw,
					       info->id,
					       &params,
					       &vsink_cbs,
					       self,
					       &self->sink);
		if (res < 0) {
			ULOG_ERRNO("pdraw_raw_video_sink_new", -res);
			goto out;
		}

		self->queue =
			pdraw_raw_video_sink_get_queue(self->pdraw, self->sink);
		if (self->queue == NULL) {
			ULOG_ERRNO("pdraw_raw_video_sink_get_queue", EPROTO);
			res = -EPROTO;
			goto out;
		}

		res = mbuf_raw_video_frame_queue_get_event(self->queue, &evt);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event",
				   -res);
			goto out;
		}
		res = pomp_evt_attach_to_loop(
			evt, self->loop, &queue_event_cb, self);
		if (res < 0) {
			ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
			goto out;
		}
	} else {
		res = pdraw_raw_video_sink_set_media_id(
			self->pdraw, self->sink, info->id);
		if (res < 0) {
			ULOG_ERRNO("pdraw_raw_video_sink_set_media_id", -res);
			goto out;
		}
	}

out:
	if (self->starting) {
		self->result = res;
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
	}
	pthread_mutex_unlock(&self->mutex);
}


static void media_removed_cb(struct pdraw *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	UNUSED(pdraw);
	UNUSED(element_userdata);
	UNUSED(userdata);

	ULOGI("%s: id=%d", __func__, info->id);
}


static const struct pdraw_cbs pdraw_cbs = {
	.stop_resp = &stop_resp_cb,
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
};


static void start_pdraw_idle(void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;
	struct pdraw_demuxer_params params = {
		.autodecoding_mode = 0,
		.playback_mode = self->playback_mode,
	};

	res = pdraw_new(self->loop, &pdraw_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_new", -res);
		pthread_mutex_lock(&self->mutex);
		self->result = res;
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
		return;
	}

	res = pdraw_demuxer_new_from_url(self->pdraw,
					 self->url,
					 &params,
					 &demuxer_cbs,
					 self,
					 &self->demuxer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_demuxer_new_from_url", -res);
		pthread_mutex_lock(&self->mutex);
		self->result = res;
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
		return;
	}
}


static void stop_pdraw_idle(void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	pthread_mutex_lock(&self->mutex);

	if (self->queue != NULL) {
		struct pomp_evt *evt = NULL;
		res = mbuf_raw_video_frame_queue_get_event(self->queue, &evt);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event",
				   -res);
		} else {
			res = pomp_evt_detach_from_loop(evt, self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_evt_detach_from_loop", -res);
		}
		self->queue = NULL;
	}

	if (self->demuxer != NULL) {
		res = pdraw_demuxer_close(self->pdraw, self->demuxer);
		if (res < 0) {
			pthread_mutex_unlock(&self->mutex);
			ULOG_ERRNO("pdraw_demuxer_close", -res);
			goto error;
		}
	}

	pthread_mutex_unlock(&self->mutex);

	res = pdraw_stop(self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_stop", -res);
		goto error;
	}

	return;

error:
	res = pomp_loop_idle_add(self->loop, delete_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		pthread_mutex_lock(&self->mutex);
		self->cond_ready = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
	}
}


int pdraw_vsink_start(const struct pdraw_vsink_params *params,
		      struct pdraw_media_info **media_info,
		      struct pdraw_vsink **ret_obj)
{
	int res;
	int err;

	ULOG_ERRNO_RETURN_ERR_IF(params == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->url == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	struct pdraw_vsink *self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->cond_ready = false;

	self->url = strdup(params->url);
	if (self->url == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("strdup", -res);
		goto error;
	}
	self->playback_mode = params->playback_mode;
	self->camera_type = params->camera_type;
	self->cbs = params->cbs;
	self->cbs_userdata = params->cbs_userdata;

	res = pthread_mutex_init(&self->mutex, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_mutex_init", -res);
		goto error;
	}

	res = pthread_cond_init(&self->cond, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_cond_init", -res);
		goto error;
	}

	self->loop = pomp_loop_new();
	if (self->loop == NULL) {
		res = -ENOMEM;
		ULOGE("failed to create pomp loop");
		goto error;
	}

	res = pthread_create(&self->thread, NULL, run_loop_thread, self);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_create", -res);
		goto error;
	}
	self->starting = true;
	self->thread_launched = 1;

	res = pomp_loop_idle_add(self->loop, start_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		goto error;
	}

	pthread_mutex_lock(&self->mutex);
	while (!self->cond_ready)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->cond_ready = false;
	self->starting = false;
	res = self->result;
	pthread_mutex_unlock(&self->mutex);

	if (res < 0) {
		ULOG_ERRNO("failed to start pdraw vsink", -res);
		goto error;
	}

	if (media_info != NULL)
		*media_info = self->media_info;

	*ret_obj = self;
	return 0;

error:
	err = pdraw_vsink_stop(self);
	if (err < 0)
		ULOG_ERRNO("pdraw_vsink_stop", -res);
	return res;
}


int pdraw_vsink_stop(struct pdraw_vsink *self)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->pdraw != NULL) {
		res = pomp_loop_idle_add(self->loop, stop_pdraw_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		pthread_mutex_lock(&self->mutex);
		while (!self->cond_ready)
			pthread_cond_wait(&self->cond, &self->mutex);
		self->cond_ready = false;
		pthread_mutex_unlock(&self->mutex);
	}

	if (self->thread_launched) {
		self->thread_should_stop = 1;
		res = pomp_loop_wakeup(self->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -res);
		res = pthread_join(self->thread, NULL);
		if (res != 0)
			ULOG_ERRNO("pthread_join", res);
		res = pomp_loop_destroy(self->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_destroy", -res);
		self->loop = NULL;
	}

	res = pthread_mutex_destroy(&self->mutex);
	if (res != 0)
		ULOG_ERRNO("pthread_mutex_destroy", res);

	res = pthread_cond_destroy(&self->cond);
	if (res != 0)
		ULOG_ERRNO("pthread_cond_destroy", res);

	pdraw_media_info_free(self->media_info);
	free(self->url);
	free(self);

	return 0;
}


int pdraw_vsink_get_frame(struct pdraw_vsink *self,
			  int timeout_ms,
			  struct mbuf_mem *frame_memory,
			  struct pdraw_video_frame *frame_info,
			  struct mbuf_raw_video_frame **ret_frame)
{
	int res;
	struct mbuf_raw_video_frame *in_frame = NULL;
	struct mbuf_mem *memory = frame_memory;
	struct mbuf_ancillary_data *ancillary_data = NULL;
	struct pdraw_video_frame *in_frame_info = NULL;
	bool own_mem = false;
	struct timespec ts_timeout = {};
	struct timespec ts_now = {};

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->queue == NULL, EAGAIN);

	if (self->cbs.frame_ready != NULL) {
		ULOGE("%s is unavailable when the frame_ready "
		      "callback is implemented",
		      __func__);
		return -EPERM;
	}

	*ret_frame = NULL;
	do {
		res = mbuf_raw_video_frame_queue_pop(self->queue, &in_frame);
		if (res == -EAGAIN) {
			if (timeout_ms == 0)
				return -EAGAIN;

			pthread_mutex_lock(&self->mutex);

			if (timeout_ms > 0) {
				time_get_realtime(&ts_now);
				time_timespec_add_us(&ts_now,
						     timeout_ms * 1000,
						     &ts_timeout);
			}
			while (!self->frame_ready) {
				if (timeout_ms > 0) {
					res = pthread_cond_timedwait(
						&self->cond,
						&self->mutex,
						&ts_timeout);
				} else {
					res = pthread_cond_wait(&self->cond,
								&self->mutex);
				}
				if (res == ETIMEDOUT) {
					self->frame_ready = false;
					pthread_mutex_unlock(&self->mutex);
					return -res;
				}
			}
			self->frame_ready = false;
			pthread_mutex_unlock(&self->mutex);
		} else if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
			return res;
		} else {
			break;
		}
	} while (1);

	if (!memory) {
		ssize_t len;
		/* Need to allocate our own memory */
		len = mbuf_raw_video_frame_get_packed_size(in_frame, false);
		if (len <= 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_packed_size",
				   -len);
			goto out;
		}
		own_mem = true;
		res = mbuf_mem_generic_new(len, &memory);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_generic_new", -res);
			goto out;
		}
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		in_frame, PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillary_data);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data", -res);
		goto out;
	}
	in_frame_info =
		(struct pdraw_video_frame *)mbuf_ancillary_data_get_buffer(
			ancillary_data, NULL);
	*frame_info = *in_frame_info;

	res = mbuf_raw_video_frame_copy(in_frame, memory, true, ret_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_copy", -res);
		goto out;
	}
	res = mbuf_raw_video_frame_finalize(*ret_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_copy", -res);
		goto out;
	}

out:
	if (own_mem && memory)
		mbuf_mem_unref(memory);
	mbuf_raw_video_frame_unref(in_frame);
	mbuf_ancillary_data_unref(ancillary_data);
	if (res < 0) {
		mbuf_raw_video_frame_unref(*ret_frame);
		*ret_frame = NULL;
	}
	return res;
}
