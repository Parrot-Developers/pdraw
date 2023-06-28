/**
 * Parrot Drones Awesome Video Viewer
 * Video IPC source to sink test program
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
#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>

#define ULOG_TAG pdraw_vipcsourcesink_test
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_vipcsourcesink_test);

#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw_backend.h>
#include <video-defs/vdefs.h>
#include <video-raw/vraw.h>


struct pdraw_backend_app {
	pthread_mutex_t mutex;
	bool mutex_created;
	pthread_cond_t cond;
	bool cond_created;
	bool stopping;
	struct pdraw_backend *pdraw;
	struct pdraw_vipc_source *source;
	void *source_element_userdata;
	struct pdraw_raw_video_sink *sink;
	bool media_added;
	bool media_removed;
	bool stop_resp;
	int stop_resp_status;
	struct vraw_writer *writer;
	struct pdraw_media_info out_info;
	struct mbuf_raw_video_frame_queue *out_queue;
	unsigned int out_count;
};


static void ready_to_play_cb(struct pdraw_backend *pdraw,
			     struct pdraw_vipc_source *source,
			     int ready,
			     enum pdraw_vipc_source_eos_reason eos_reason,
			     void *userdata)
{
	struct pdraw_backend_app *self = userdata;

	ULOGI("%s: ready=%d eos_reason=%s",
	      __func__,
	      ready,
	      pdraw_vipc_source_eos_reason_str(eos_reason));

	if (self->stopping)
		return;

	if (ready) {
		int err = pdraw_be_vipc_source_play(self->pdraw, self->source);
		if (err < 0) {
			ULOG_ERRNO("pdraw_be_vipc_source_play", -err);
			return;
		}
	} else {
		self->stopping = true;
	}
}


static void configured_cb(struct pdraw_backend *pdraw,
			  struct pdraw_vipc_source *source,
			  int status,
			  const struct vdef_format_info *info,
			  const struct vdef_rectf *crop,
			  void *userdata)
{
	ULOGI("%s: status=%d(%s) %ux%u@%.2ffps crop=%.2f,%.2f->%.2f,%.2f",
	      __func__,
	      status,
	      strerror(status),
	      info->resolution.width,
	      info->resolution.height,
	      (info->framerate.den != 0)
		      ? (float)info->framerate.num / info->framerate.den
		      : 0.f,
	      crop->left,
	      crop->top,
	      crop->left + crop->width,
	      crop->top + crop->height);
}


static const struct pdraw_backend_vipc_source_cbs source_cbs = {
	.ready_to_play = &ready_to_play_cb,
	.configured = &configured_cb,
};


static void sink_flush_cb(struct pdraw_backend *pdraw,
			  struct pdraw_raw_video_sink *sink,
			  void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res;

	ULOGI("%s", __func__);

	if (self->out_queue == NULL)
		return;

	pthread_mutex_lock(&self->mutex);

	res = mbuf_raw_video_frame_queue_flush(self->out_queue);
	if (res < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -res);

	res = pdraw_be_raw_video_sink_queue_flushed(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_raw_video_sink_queue_flushed", -res);

	pthread_mutex_unlock(&self->mutex);
}


static const struct pdraw_backend_raw_video_sink_cbs sink_cbs = {
	.flush = &sink_flush_cb,
};


static void
stop_resp_cb(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));
	pthread_mutex_lock(&self->mutex);
	self->stop_resp = true;
	self->stop_resp_status = status;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void media_added_cb(struct pdraw_backend *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_userdata,
			   void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res;

	ULOGI("%s id=%d", __func__, info->id);

	if (self->stopping)
		return;
	if (element_userdata != self->source_element_userdata)
		return;
	if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
		return;
	if (info->video.format != VDEF_FRAME_TYPE_RAW)
		return;

	struct pdraw_video_sink_params sink_params = {0};
	self->out_info = *info;

	res = pdraw_be_raw_video_sink_new(self->pdraw,
					  info->id,
					  &sink_params,
					  &sink_cbs,
					  self,
					  &self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_raw_video_sink_new", -res);

	self->out_queue =
		pdraw_be_raw_video_sink_get_queue(self->pdraw, self->sink);
	if (self->out_queue == NULL)
		ULOG_ERRNO("pdraw_be_raw_video_sink_get_queue", EPROTO);

	pthread_mutex_lock(&self->mutex);
	self->media_added = true;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res;

	ULOGI("%s id=%d", __func__, info->id);

	if (element_userdata != self->source_element_userdata)
		return;
	if (self->sink == NULL)
		return;

	res = pdraw_be_raw_video_sink_destroy(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_raw_video_sink_destroy", -res);
	self->sink = NULL;

	res = pdraw_be_vipc_source_destroy(self->pdraw, self->source);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_vipc_source_destroy", -res);
	self->source = NULL;

	pthread_mutex_lock(&self->mutex);
	self->media_removed = true;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void
socket_created_cb(struct pdraw_backend *pdraw, int fd, void *userdata)
{
	ULOGI("%s fd=%d", __func__, fd);
}


static const struct pdraw_backend_cbs be_cbs = {
	.stop_resp = &stop_resp_cb,
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
	.socket_created = &socket_created_cb,
};


static void process_output(struct pdraw_backend_app *self)
{
	int res = 0, err;

	if (self->out_queue == NULL)
		return;

	pthread_mutex_lock(&self->mutex);

	while (res == 0) {
		struct mbuf_raw_video_frame *frame = NULL;
		struct vraw_frame vraw_frame = {0};
		struct vdef_raw_frame info;
		unsigned int plane_count = 0;

		res = mbuf_raw_video_frame_queue_pop(self->out_queue, &frame);
		if (res < 0) {
			if (res != -EAGAIN) {
				ULOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					   -res);
			}
			break;
		}

		res = mbuf_raw_video_frame_get_frame_info(frame, &info);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
			goto out;
		}

		plane_count = vdef_get_raw_frame_plane_count(&info.format);
		for (unsigned int i = 0; i < plane_count; i++) {
			size_t len;
			res = mbuf_raw_video_frame_get_plane(
				frame,
				i,
				(const void **)&vraw_frame.cdata[i],
				&len);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_get_plane",
					   -res);
				goto out;
			}
		}
		vraw_frame.frame = info;

		ULOGI("write frame #%d ts=%" PRIu64,
		      vraw_frame.frame.info.index,
		      vraw_frame.frame.info.timestamp);
		self->out_count++;

		/* Write the frame */
		res = vraw_writer_frame_write(self->writer, &vraw_frame);
		if (res < 0) {
			ULOG_ERRNO("vraw_writer_frame_write", -res);
			goto out;
		}

		/* clang-format off */
out:
		/* clang-format on */
		for (unsigned int i = 0; i < plane_count; i++) {
			if (vraw_frame.cdata[i] == NULL)
				continue;
			err = mbuf_raw_video_frame_release_plane(
				frame, i, vraw_frame.cdata[i]);
			if (err < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_release_plane",
					   -err);
			}
		}

		if (frame != NULL) {
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0)
				ULOG_ERRNO("mbuf_raw_video_frame_unref", -err);
		}
	}

	pthread_mutex_unlock(&self->mutex);
}


static const char short_options[] = "h";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{0, 0, 0, 0},
};


static void welcome(int argc, char **argv)
{
	printf("%s - Parrot Drones Awesome Video Viewer "
	       "Video IPC source to sink test program\n\n",
	       argv[0]);
}


static void usage(int argc, char **argv)
{
	/* clang-format off */
	printf("Usage: %s [options] <vipc_address> <output_file>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
		       "Print this message\n"
	       "\n",
	       argv[0]);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int res, status = EXIT_SUCCESS;
	const char *input = NULL, *output = NULL;
	struct pdraw_backend_app *self = NULL;
	struct pdraw_vipc_source_params source_params = {0};
	struct vraw_writer_config writer_config = {0};

	welcome(argc, argv);

	/* Command-line parameters */
	int idx, c;
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argc, argv);
			exit(EXIT_SUCCESS);
			break;

		default:
			usage(argc, argv);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (argc - optind < 2) {
		usage(argc, argv);
		exit(EXIT_FAILURE);
	}

	input = argv[optind];
	output = argv[optind + 1];

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}

	res = pthread_mutex_init(&self->mutex, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->mutex_created = true;

	res = pthread_cond_init(&self->cond, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_cond_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->cond_created = true;

	/* Create PDrAW instance */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("created");

	/* Create the video IPC source */
	source_params = (struct pdraw_vipc_source_params){
		.address = input,
	};
	res = pdraw_be_vipc_source_new(
		self->pdraw, &source_params, &source_cbs, self, &self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_vipc_source_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->source_element_userdata = self->source;
	pthread_mutex_lock(&self->mutex);
	while (!self->media_added)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->media_added = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source created");

	if (self->out_info.type != PDRAW_MEDIA_TYPE_VIDEO) {
		ULOGE("unsupported media type (%s)",
		      pdraw_media_type_str(self->out_info.type));
		status = EXIT_FAILURE;
		goto out;
	}
	if (self->out_info.video.format != VDEF_FRAME_TYPE_RAW) {
		ULOGE("unsupported video type (%s)",
		      vdef_frame_type_to_str(self->out_info.video.format));
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create the raw file writer */
	writer_config.format = self->out_info.video.raw.format;
	writer_config.info = self->out_info.video.raw.info;
	if ((strlen(output) > 4) &&
	    (strcmp(output + strlen(output) - 4, ".y4m") == 0))
		writer_config.y4m = 1;

	ULOGI("server address: %s", input);
	ULOGI("format: " VDEF_RAW_FORMAT_TO_STR_FMT,
	      VDEF_RAW_FORMAT_TO_STR_ARG(&writer_config.format));
	ULOGI("bit depth: %d bits", writer_config.format.data_size);
	ULOGI("dimensions: %dx%d",
	      writer_config.info.resolution.width,
	      writer_config.info.resolution.height);
	ULOGI("framerate: %d/%d",
	      writer_config.info.framerate.num,
	      writer_config.info.framerate.den);
	ULOGI("SAR: %d:%d",
	      writer_config.info.sar.width,
	      writer_config.info.sar.height);

	res = vraw_writer_new(output, &writer_config, &self->writer);
	if (res < 0) {
		ULOG_ERRNO("vraw_writer_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Main loop */
	while (!self->stopping) {
		process_output(self);
		usleep(1000);
	}

	/* Process the remaining frames */
	/* TODO: there should be an API to drain the remaining frames */

	pthread_mutex_lock(&self->mutex);
	while (!self->media_removed)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->media_removed = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source destroyed");

	/* Stop PDrAW */
	res = pdraw_be_stop(self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_stop", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	pthread_mutex_lock(&self->mutex);
	while (!self->stop_resp)
		pthread_cond_wait(&self->cond, &self->mutex);
	res = self->stop_resp_status;
	self->stop_resp_status = 0;
	self->stop_resp = false;
	pthread_mutex_unlock(&self->mutex);
	if (res < 0) {
		ULOG_ERRNO("stop_resp", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("stopped");

out:
	if (self != NULL) {
		if (self->pdraw != NULL) {
			if (self->sink != NULL) {
				res = pdraw_be_raw_video_sink_destroy(
					self->pdraw, self->sink);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_raw_video_"
						"sink_destroy",
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
		if (self->writer != NULL) {
			res = vraw_writer_destroy(self->writer);
			if (res < 0)
				ULOG_ERRNO("vraw_writer_destroy", -res);
		}
		if (self->cond_created) {
			res = pthread_cond_destroy(&self->cond);
			if (res != 0)
				ULOG_ERRNO("pthread_cond_destroy", res);
		}
		if (self->mutex_created) {
			res = pthread_mutex_destroy(&self->mutex);
			if (res != 0)
				ULOG_ERRNO("pthread_mutex_destroy", res);
		}
		free(self);
	}

	printf("%s\n", (status == EXIT_SUCCESS) ? "Success!" : "Failed!");
	exit(status);
}
