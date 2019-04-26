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
#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw/pdraw.h>
#include <video-buffers/vbuf.h>
#include <video-buffers/vbuf_generic.h>


struct pdraw_vsink {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	pthread_t thread;
	int thread_launched;
	int thread_should_stop;
	struct pomp_loop *loop;
	struct pdraw *pdraw;
	struct pdraw_video_sink *sink;
	struct vbuf_queue *queue;
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

	res = pdraw_destroy(self->pdraw);
	if (res < 0)
		ULOG_ERRNO("pdraw_destroy", -res);
	self->pdraw = NULL;
	pthread_cond_signal(&self->cond);
}


static void
flush_cb(struct pdraw *pdraw, struct pdraw_video_sink *sink, void *userdata)
{
	int res;
	struct vbuf_queue *queue;

	queue = pdraw_get_video_sink_queue(pdraw, sink);
	if (queue == NULL) {
		ULOG_ERRNO("pdraw_get_video_sink_queue", EPROTO);
		return;
	}

	res = vbuf_queue_flush(queue);
	if (res < 0) {
		ULOG_ERRNO("vbuf_queue_flush", -res);
		return;
	}

	res = pdraw_video_sink_queue_flushed(pdraw, sink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_video_sink_queue_flushed", -res);
		return;
	}
}


static void open_resp_cb(struct pdraw *pdraw, int status, void *userdata)
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


static void close_resp_cb(struct pdraw *pdraw, int status, void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("close_resp status=%d", status);

	res = pomp_loop_idle_add(self->loop, delete_pdraw_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		pthread_cond_signal(&self->cond);
	}
}

static void ready_to_play_cb(struct pdraw *pdraw, int ready, void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("ready_to_play ready=%d", ready);

	if (ready) {
		res = pdraw_play(pdraw);
		if (res < 0) {
			ULOG_ERRNO("pdraw_play", -res);
			pthread_mutex_lock(&self->mutex);
			self->result = res;
			pthread_mutex_unlock(&self->mutex);
			pthread_cond_signal(&self->cond);
		}
	}
}

static void play_resp_cb(struct pdraw *pdraw,
			 int status,
			 uint64_t timestamp,
			 float speed,
			 void *userdata)
{
	ULOGI("play_resp status=%d speed=%f", status, speed);

	if (status != 0)
		ULOG_ERRNO("play_resp_cb", -status);
}


static const struct pdraw_video_sink_cbs vsink_cbs = {
	.flush = &flush_cb,
};


static void media_added_cb(struct pdraw *pdraw,
			   const struct pdraw_media_info *info,
			   void *userdata)
{
	int res = 0;
	struct pdraw_vsink *self = userdata;

	if (info->type != PDRAW_MEDIA_TYPE_VIDEO ||
	    info->video.format != PDRAW_VIDEO_MEDIA_FORMAT_YUV ||
	    info->video.type != PDRAW_VIDEO_TYPE_DEFAULT_CAMERA)
		return;

	ULOGI("media_added id=%d", info->id);

	pthread_mutex_lock(&self->mutex);

	if (self->sink != NULL) {
		res = pdraw_stop_video_sink(self->pdraw, self->sink);
		if (res < 0) {
			ULOG_ERRNO("pdraw_stop_video_sink", -res);
			goto out;
		}
		self->sink = NULL;
	}

	struct pdraw_video_sink_params params;
	memset(&params, 0, sizeof(params));
	params.queue_max_count = 1;
	params.queue_drop_when_full = 1;
	res = pdraw_start_video_sink(
		self->pdraw, info->id, &params, &vsink_cbs, self, &self->sink);
	if (res < 0) {
		ULOG_ERRNO("pdraw_start_video_sink", -res);
		goto out;
	}

	self->queue = pdraw_get_video_sink_queue(self->pdraw, self->sink);
	if (self->queue == NULL) {
		ULOG_ERRNO("pdraw_get_video_sink_queue", EPROTO);
		res = -EPROTO;
		goto out;
	}

out:
	self->result = res;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void media_removed_cb(struct pdraw *pdraw,
			     const struct pdraw_media_info *info,
			     void *userdata)
{
	int res;
	struct pdraw_vsink *self = userdata;

	ULOGI("media_removed id=%d", info->id);

	pthread_mutex_lock(&self->mutex);

	if (self->sink != NULL) {
		res = pdraw_stop_video_sink(self->pdraw, self->sink);
		if (res < 0)
			ULOG_ERRNO("pdraw_stop_video_sink", -res);
		self->sink = NULL;
	}

	pthread_mutex_unlock(&self->mutex);
}

static const struct pdraw_cbs pdraw_cbs = {
	.open_resp = &open_resp_cb,
	.close_resp = &close_resp_cb,
	.ready_to_play = &ready_to_play_cb,
	.play_resp = &play_resp_cb,
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

	res = pdraw_open_url(self->pdraw, self->url);
	if (res < 0) {
		ULOG_ERRNO("pdraw_open_url", -res);
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

	if (self->sink != NULL) {
		res = pdraw_stop_video_sink(self->pdraw, self->sink);
		if (res < 0) {
			pthread_mutex_unlock(&self->mutex);
			ULOG_ERRNO("pdraw_stop_video_sink", -res);
			goto error;
		}
		self->sink = NULL;
	}

	pthread_mutex_unlock(&self->mutex);

	res = pdraw_close(self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_close", -res);
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


int pdraw_vsink_start(const char *url, struct pdraw_vsink **ret_obj)
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
	pthread_cond_wait(&self->cond, &self->mutex);
	res = self->result;
	pthread_mutex_unlock(&self->mutex);

	if (res < 0) {
		ULOG_ERRNO("failed to start pdraw vsink", -res);
		goto error;
	}

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

	free(self->url);
	free(self);

	return 0;
}


int pdraw_vsink_get_frame(struct pdraw_vsink *self,
			  int timeout_ms,
			  struct pdraw_video_frame *frame,
			  struct vbuf_buffer *buffer)
{
	int res;
	struct vbuf_buffer *buf = NULL;
	struct pdraw_video_frame *meta = NULL;
	size_t capacity;
	uint8_t *data;
	unsigned int i;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buffer == NULL, EINVAL);

	pthread_mutex_lock(&self->mutex);

	res = vbuf_queue_pop(self->queue, timeout_ms, &buf);
	if (res < 0) {
		pthread_mutex_unlock(&self->mutex);
		ULOG_ERRNO("vbuf_queue_pop", -res);
		return res;
	}

	res = vbuf_metadata_get(buf, self->sink, NULL, NULL, (uint8_t **)&meta);
	pthread_mutex_unlock(&self->mutex);
	if (res < 0) {
		ULOG_ERRNO("vbuf_metadata_get", -res);
		goto out;
	}

	if ((meta->yuv.format != PDRAW_YUV_FORMAT_I420) &&
	    (meta->yuv.format != PDRAW_YUV_FORMAT_NV12)) {
		res = -ENOSYS;
		ULOGE("unsupported YUV format (%d)", meta->yuv.format);
		goto out;
	}
	if ((meta->yuv.width == 0) || (meta->yuv.height == 0) ||
	    (meta->yuv.crop_width == 0) || (meta->yuv.crop_height == 0)) {
		res = -ENOSYS;
		ULOGE("unsupported dimensions (%dx%d, crop %dx%d)",
		      meta->yuv.width,
		      meta->yuv.height,
		      meta->yuv.crop_width,
		      meta->yuv.crop_height);
		goto out;
	}

	capacity = meta->yuv.crop_width * meta->yuv.crop_height * 3 / 2;

	res = vbuf_set_capacity(buffer, capacity);
	if (res < 0) {
		ULOG_ERRNO("vbuf_set_capacity", -res);
		goto out;
	}

	data = vbuf_get_data(buffer);
	if (data == NULL) {
		res = -EIO;
		ULOG_ERRNO("vbuf_get_data", -res);
		goto out;
	}

	*frame = *meta;
	frame->yuv.width = meta->yuv.crop_width;
	frame->yuv.height = meta->yuv.crop_height;
	frame->yuv.crop_left = 0;
	frame->yuv.crop_top = 0;
	frame->yuv.crop_width = meta->yuv.crop_width;
	frame->yuv.crop_height = meta->yuv.crop_height;
	switch (meta->yuv.format) {
	case PDRAW_YUV_FORMAT_I420:
		frame->yuv.plane[0] = data;
		frame->yuv.plane[1] =
			data + frame->yuv.width * frame->yuv.height;
		frame->yuv.plane[2] =
			data + frame->yuv.width * frame->yuv.height * 5 / 4;
		frame->yuv.stride[0] = frame->yuv.width;
		frame->yuv.stride[1] = frame->yuv.width / 2;
		frame->yuv.stride[2] = frame->yuv.width / 2;
		for (i = 0; i < frame->yuv.height; i++) {
			memcpy((uint8_t *)frame->yuv.plane[0] +
				       i * frame->yuv.width,
			       meta->yuv.plane[0] + i * meta->yuv.stride[0],
			       frame->yuv.width);
		}
		for (i = 0; i < frame->yuv.height / 2; i++) {
			memcpy((uint8_t *)frame->yuv.plane[1] +
				       i * frame->yuv.width / 2,
			       meta->yuv.plane[1] + i * meta->yuv.stride[1],
			       frame->yuv.width / 2);
			memcpy((uint8_t *)frame->yuv.plane[2] +
				       i * frame->yuv.width / 2,
			       meta->yuv.plane[2] + i * meta->yuv.stride[2],
			       frame->yuv.width / 2);
		}
		break;
	case PDRAW_YUV_FORMAT_NV12:
		frame->yuv.plane[0] = data;
		frame->yuv.plane[1] =
			data + frame->yuv.width * frame->yuv.height;
		frame->yuv.plane[2] = NULL;
		frame->yuv.stride[0] = frame->yuv.width;
		frame->yuv.stride[1] = frame->yuv.width;
		frame->yuv.stride[2] = 0;
		for (i = 0; i < frame->yuv.height; i++) {
			memcpy((uint8_t *)frame->yuv.plane[0] +
				       i * frame->yuv.width,
			       meta->yuv.plane[0] + i * meta->yuv.stride[0],
			       frame->yuv.width);
		}
		for (i = 0; i < frame->yuv.height / 2; i++) {
			memcpy((uint8_t *)frame->yuv.plane[1] +
				       i * frame->yuv.width,
			       meta->yuv.plane[1] + i * meta->yuv.stride[1],
			       frame->yuv.width);
		}
		break;
	default:
		break;
	}

out:
	vbuf_unref(buf);
	return res;
}
