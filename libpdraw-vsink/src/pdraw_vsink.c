/**
 * Parrot Drones Awesome Video Viewer
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

#define ULOG_TAG pdraw_vsink
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_vsink);

#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw/pdraw.h>


struct pdraw_vsink {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
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
	int result;
};


static void *run_loop_thread(void *ptr)
{
	struct pdraw_vsink *self = ptr;

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
	pthread_cond_signal(&self->cond);
}


static void queue_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct pdraw_vsink *self = userdata;
	pthread_cond_signal(&self->cond);
}


static void
flush_cb(struct pdraw *pdraw, struct pdraw_raw_video_sink *sink, void *userdata)
{
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


static const struct pdraw_raw_video_sink_cbs vsink_cbs = {
	.flush = &flush_cb,
};


static void open_resp_cb(struct pdraw *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 void *userdata)
{
	struct pdraw_vsink *self = userdata;

	ULOGI("open_resp status=%d", status);

	if (status != 0) {
		pthread_mutex_lock(&self->mutex);
		self->result = status;
		pthread_mutex_unlock(&self->mutex);
		pthread_cond_signal(&self->cond);
	}
}


static void ready_to_play_cb(struct pdraw *pdraw,
			     struct pdraw_demuxer *demuxer,
			     int ready,
			     void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("ready_to_play ready=%d", ready);

	if (ready) {
		res = pdraw_demuxer_play(pdraw, self->demuxer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_demuxer_play", -res);
			pthread_mutex_lock(&self->mutex);
			self->result = res;
			pthread_mutex_unlock(&self->mutex);
			pthread_cond_signal(&self->cond);
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
	ULOGI("play_resp status=%d speed=%f", status, speed);

	if (status != 0)
		ULOG_ERRNO("play_resp_cb", -status);
}


static const struct pdraw_demuxer_cbs demuxer_cbs = {
	.open_resp = &open_resp_cb,
	.ready_to_play = &ready_to_play_cb,
	.play_resp = &play_resp_cb,
};


static void stop_resp_cb(struct pdraw *pdraw, int status, void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("stop_resp status=%d", status);

	res = pomp_loop_idle_add(self->loop, delete_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		pthread_cond_signal(&self->cond);
	}
}


static void media_added_cb(struct pdraw *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_userdata,
			   void *userdata)
{
	int res = 0;
	struct pdraw_vsink *self = userdata;
	struct pomp_evt *evt = NULL;

	if (info->type != PDRAW_MEDIA_TYPE_VIDEO ||
	    info->video.format != VDEF_FRAME_TYPE_RAW ||
	    info->video.type != PDRAW_VIDEO_TYPE_DEFAULT_CAMERA)
		return;

	ULOGI("media_added id=%d", info->id);

	pthread_mutex_lock(&self->mutex);

	if (self->sink != NULL) {
		res = pdraw_raw_video_sink_destroy(self->pdraw, self->sink);
		if (res < 0) {
			ULOG_ERRNO("pdraw_raw_video_sink_destroy", -res);
			goto out;
		}
		self->sink = NULL;
	}

	self->media_info = pdraw_media_info_dup(info);
	if (self->media_info == NULL) {
		ULOG_ERRNO("pdraw_media_info_dup", ENOMEM);
		goto out;
	}


	struct pdraw_video_sink_params params;
	memset(&params, 0, sizeof(params));
	params.queue_max_count = 1;
	res = pdraw_raw_video_sink_new(
		self->pdraw, info->id, &params, &vsink_cbs, self, &self->sink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_raw_video_sink_new", -res);
		goto out;
	}

	self->queue = pdraw_raw_video_sink_get_queue(self->pdraw, self->sink);
	if (self->queue == NULL) {
		ULOG_ERRNO("pdraw_raw_video_sink_get_queue", EPROTO);
		res = -EPROTO;
		goto out;
	}

	res = mbuf_raw_video_frame_queue_get_event(self->queue, &evt);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
		goto out;
	}
	res = pomp_evt_attach_to_loop(evt, self->loop, &queue_event_cb, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto out;
	}

out:
	self->result = res;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void media_removed_cb(struct pdraw *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("media_removed id=%d", info->id);

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

	if (self->sink != NULL) {
		res = pdraw_raw_video_sink_destroy(self->pdraw, self->sink);
		if (res < 0)
			ULOG_ERRNO("pdraw_raw_video_sink_destroy", -res);
		self->sink = NULL;
	}

	pthread_mutex_unlock(&self->mutex);
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

	res = pdraw_new(self->loop, &pdraw_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_new", -res);
		pthread_mutex_lock(&self->mutex);
		self->result = res;
		pthread_mutex_unlock(&self->mutex);
		pthread_cond_signal(&self->cond);
		return;
	}

	res = pdraw_demuxer_new_from_url(
		self->pdraw, self->url, &demuxer_cbs, self, &self->demuxer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_demuxer_new_from_url", -res);
		pthread_mutex_lock(&self->mutex);
		self->result = res;
		pthread_mutex_unlock(&self->mutex);
		pthread_cond_signal(&self->cond);
		return;
	}
}


static void stop_pdraw_idle(void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	pthread_mutex_lock(&self->mutex);

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
		pthread_cond_signal(&self->cond);
	}
}


int pdraw_vsink_start(const char *url,
		      struct pdraw_media_info **media_info,
		      struct pdraw_vsink **ret_obj,
			  time_t timeout_seconds)
{
	int res, err;

	ULOG_ERRNO_RETURN_ERR_IF(url == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	struct pdraw_vsink *self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->url = strdup(url);
	if (self->url == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("strdup", -res);
		goto error;
	}

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
	self->thread_launched = 1;

	res = pomp_loop_idle_add(self->loop, start_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		goto error;
	}

	pthread_mutex_lock(&self->mutex);

	if(timeout_seconds > 0)
	{
		// Create timeout value
		struct timespec max_wait = {0, 0};
		const int gettime_rv = clock_gettime(CLOCK_REALTIME, &max_wait);
		max_wait.tv_sec += timeout_seconds;
		const time_wait = pthread_cond_timedwait(&self->cond, &self->mutex, &max_wait);
	}
	else{
		pthread_cond_wait(&self->cond, &self->mutex);
	}

	res = self->result;
	pthread_mutex_unlock(&self->mutex);

	if(time_wait) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_pop timeout", -res);
		goto error;
	}

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
		pthread_cond_wait(&self->cond, &self->mutex);
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
			  struct mbuf_mem *frame_memory,
			  struct pdraw_video_frame *frame_info,
			  struct mbuf_raw_video_frame **ret_frame,
			  time_t timeout_seconds)
{
	int res;
	struct mbuf_raw_video_frame *in_frame = NULL;
	struct mbuf_mem *memory = frame_memory;
	struct mbuf_ancillary_data *ancillary_data = NULL;
	struct pdraw_video_frame *in_frame_info = NULL;
	bool own_mem = false;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->queue == NULL, EAGAIN);

	*ret_frame = NULL;

	do {
		res = mbuf_raw_video_frame_queue_pop(self->queue, &in_frame);
		if (res == -EAGAIN) {
			pthread_mutex_lock(&self->mutex);

			if(timeout_seconds > 0)
			{
				// Create timeout value
				struct timespec max_wait = {0, 0};
				const int gettime_rv = clock_gettime(CLOCK_REALTIME, &max_wait);
				max_wait.tv_sec += timeout_seconds;
				res = pthread_cond_timedwait(&self->cond, &self->mutex, &max_wait);
			}
			else{
				pthread_cond_wait(&self->cond, &self->mutex);
			}

			pthread_mutex_unlock(&self->mutex);

			if(res) {
				ULOG_ERRNO("mbuf_raw_video_frame_queue_pop timeout", -res);
				return res;
			}
		} else if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
			return res;
		} else {
			break;
		}
	} while (1);

	if (!memory) {
		const void *buf;
		size_t len;
		/* Need to allocate our own memory */
		res = mbuf_raw_video_frame_get_packed_buffer(
			in_frame, &buf, &len);
		if (res == -EPROTO) {
			/* Frame is not packed but we have len, nothing to do */
		} else if (res <= 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer",
				   -res);
			goto out;
		} else {
			/* Immediately release, we only need the size */
			mbuf_raw_video_frame_release_packed_buffer(in_frame,
								   buf);
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
