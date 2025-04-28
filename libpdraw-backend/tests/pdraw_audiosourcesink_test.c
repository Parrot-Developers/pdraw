/**
 * Parrot Drones Audio and Video Vector
 * Audio source to sink test program
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

#define ULOG_TAG pdraw_audiosourcesink_test
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_audiosourcesink_test);

#include <audio-defs/adefs.h>
#include <audio-raw/araw.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <pdraw/pdraw_backend.h>


#define FRIENDLY_NAME "pdraw_audiosourcesink_test"


struct pdraw_backend_app {
	pthread_mutex_t mutex;
	bool mutex_created;
	pthread_cond_t cond;
	bool cond_created;
	struct pdraw_backend *pdraw;
	struct pdraw_audio_source *source;
	void *source_element_userdata;
	struct pdraw_audio_sink *sink;
	bool media_added;
	bool media_removed;
	bool flushed_resp;
	bool stop_resp;
	int stop_resp_status;
	struct araw_reader *reader;
	struct araw_writer *writer;
	struct mbuf_audio_frame_queue *in_queue;
	struct mbuf_audio_frame_queue *out_queue;
	unsigned int out_count;
};


static void source_flushed_cb(struct pdraw_backend *pdraw,
			      struct pdraw_audio_source *source,
			      void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s", __func__);
	pthread_mutex_lock(&self->mutex);
	self->flushed_resp = true;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static const struct pdraw_backend_audio_source_cbs source_cbs = {
	.flushed = &source_flushed_cb,
};


static void sink_flush_cb(struct pdraw_backend *pdraw,
			  struct pdraw_audio_sink *sink,
			  void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res;

	ULOGI("%s", __func__);

	if (self->out_queue == NULL)
		return;

	pthread_mutex_lock(&self->mutex);

	res = mbuf_audio_frame_queue_flush(self->out_queue);
	if (res < 0)
		ULOG_ERRNO("mbuf_audio_frame_queue_flush", -res);

	res = pdraw_be_audio_sink_queue_flushed(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_audio_sink_queue_flushed", -res);

	pthread_mutex_unlock(&self->mutex);
}


static const struct pdraw_backend_audio_sink_cbs sink_cbs = {
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

	pthread_mutex_lock(&self->mutex);

	ULOGI("%s id=%d", __func__, info->id);

	if (element_userdata != self->source_element_userdata) {
		pthread_mutex_unlock(&self->mutex);
		return;
	}
	if (info->type != PDRAW_MEDIA_TYPE_AUDIO) {
		pthread_mutex_unlock(&self->mutex);
		return;
	}
	pthread_mutex_unlock(&self->mutex);

	res = pdraw_be_audio_sink_new(
		self->pdraw, info->id, &sink_cbs, self, &self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_audio_sink_new", -res);

	self->out_queue =
		pdraw_be_audio_sink_get_queue(self->pdraw, self->sink);
	if (self->out_queue == NULL)
		ULOG_ERRNO("pdraw_be_audio_sink_get_queue", EPROTO);

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

	res = pdraw_be_audio_sink_destroy(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_audio_sink_destroy", -res);
	self->sink = NULL;

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
		struct mbuf_audio_frame *frame = NULL;
		struct araw_frame araw_frame = {0};
		struct adef_frame info = {0};
		const void *data = NULL;
		size_t len = 0;

		res = mbuf_audio_frame_queue_pop(self->out_queue, &frame);
		if (res < 0) {
			if (res != -EAGAIN)
				ULOG_ERRNO("mbuf_audio_frame_queue_pop", -res);
			break;
		}

		res = mbuf_audio_frame_get_frame_info(frame, &info);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_get_frame_info", -res);
			goto out;
		}

		res = mbuf_audio_frame_get_buffer(frame, &data, &len);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_get_buffer", -res);
			goto out;
		}

		araw_frame.frame = info;
		araw_frame.cdata = data;
		araw_frame.cdata_length = len;

		ULOGI("write frame #%d ts=%" PRIu64,
		      araw_frame.frame.info.index,
		      araw_frame.frame.info.timestamp);
		self->out_count++;

		/* Write the frame */
		res = araw_writer_frame_write(self->writer, &araw_frame);
		if (res < 0) {
			ULOG_ERRNO("araw_writer_frame_write", -res);
			goto out;
		}

		/* clang-format off */
out:
		/* clang-format on */
		if (frame != NULL) {
			if (data != NULL) {
				err = mbuf_audio_frame_release_buffer(frame,
								      data);
				if (err < 0) {
					ULOG_ERRNO(
						"mbuf_audio_frame_"
						"release_buffer",
						-err);
				}
			}
			err = mbuf_audio_frame_unref(frame);
			if (err < 0)
				ULOG_ERRNO("mbuf_audio_frame_unref", -err);
		}
	}

	pthread_mutex_unlock(&self->mutex);
}


static const char short_options[] = "hf:n:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"format", required_argument, NULL, 'f'},
	{"count", required_argument, NULL, 'n'},
	{0, 0, 0, 0},
};


static void welcome(int argc, char **argv)
{
	printf("%s - Parrot Drones Audio and Video Vector - "
	       "Audio source to sink test program\n\n",
	       argv[0]);
}


static void usage(int argc, char **argv)
{
	/* clang-format off */
	printf("Usage: %s [options] <input_file> <output_file>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
		       "Print this message\n"
	       "  -f | --format <format>             "
		       "Input file data format (e.g. \"PCM_16_44KHZ_STEREO\")\n"
	       "  -n | --count <n>                   "
		       "Process at most n frames\n"
	       "\n",
	       argv[0]);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int res, status = EXIT_SUCCESS;
	char *input = NULL, *output = NULL;
	struct pdraw_backend_app *self = NULL;
	struct pdraw_audio_source_params source_params = {0};
	struct araw_reader_config reader_config = {0};
	struct araw_writer_config writer_config = {0};
	size_t frame_len = 0;
	unsigned int max_count = UINT_MAX, in_count = 0;

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

		case 'f':
			res = adef_format_from_str(optarg,
						   &reader_config.format);
			if (res != 0) {
				ULOG_ERRNO("adef_format_from_str", -res);
				usage(argc, argv);
				exit(EXIT_FAILURE);
			}
			break;

		case 'n':
			sscanf(optarg, "%d", &max_count);
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

	/* Create the raw file reader */
	res = araw_reader_new(input, &reader_config, &self->reader);
	if (res < 0) {
		ULOG_ERRNO("araw_reader_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	res = araw_reader_get_config(self->reader, &reader_config);
	if (res < 0) {
		ULOG_ERRNO("araw_reader_get_config", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	frame_len = 1024 * reader_config.format.channel_count *
		    (reader_config.format.bit_depth / 8);

	ULOGI("format: " ADEF_FORMAT_TO_STR_FMT,
	      ADEF_FORMAT_TO_STR_ARG(&reader_config.format));
	if (max_count != UINT_MAX)
		ULOGI("max count: %d frames", max_count);

	/* Create the raw file writer */
	writer_config.format = reader_config.format;

	res = araw_writer_new(output, &writer_config, &self->writer);
	if (res < 0) {
		ULOG_ERRNO("araw_writer_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create PDrAW instance */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("created");

	/* Create the audio source */
	source_params = (struct pdraw_audio_source_params){
		.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY,
		.duration = 0, /* this is not known */
		.audio.format = reader_config.format,
	};
	pthread_mutex_lock(&self->mutex);
	res = pdraw_be_audio_source_new(
		self->pdraw, &source_params, &source_cbs, self, &self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_audio_source_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->source_element_userdata = self->source;
	while (!self->media_added)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->media_added = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source created");

	self->in_queue =
		pdraw_be_audio_source_get_queue(self->pdraw, self->source);
	if (self->in_queue == NULL) {
		ULOG_ERRNO("pdraw_be_audio_source_get_queue", EPROTO);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Main loop */
	res = 0;
	while ((res == 0) && (in_count < max_count)) {
		struct araw_frame in_frame = {0};
		struct mbuf_audio_frame *frame = NULL;
		struct mbuf_mem *mem = NULL;
		uint8_t *data = NULL;
		size_t capacity = 0;
		int err;

		res = mbuf_mem_generic_new(frame_len, &mem);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_generic_new", -res);
			status = EXIT_FAILURE;
			goto end;
		}
		res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_get_data", -res);
			status = EXIT_FAILURE;
			goto end;
		}

		res = araw_reader_frame_read(
			self->reader, data, capacity, &in_frame);
		if ((res < 0) && (res != -ENOENT)) {
			ULOG_ERRNO("araw_reader_frame_read", -res);
			status = EXIT_FAILURE;
			goto end;
		}
		if (res == -ENOENT)
			goto end;

		ULOGI("read frame #%d ts=%" PRIu64,
		      in_frame.frame.info.index,
		      in_frame.frame.info.timestamp);
		in_count++;

		res = mbuf_audio_frame_new(&in_frame.frame, &frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_new", -res);
			status = EXIT_FAILURE;
			goto end;
		}

		res = mbuf_audio_frame_set_buffer(frame, mem, 0, capacity);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_set_buffer", -res);
			status = EXIT_FAILURE;
			goto end;
		}

		res = mbuf_audio_frame_finalize(frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_finalize", -res);
			status = EXIT_FAILURE;
			goto end;
		}

		res = mbuf_audio_frame_queue_push(self->in_queue, frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_audio_frame_queue_push", -res);
			status = EXIT_FAILURE;
			goto end;
		}

		/* clang-format off */
end:
		/* clang-format on */
		if (frame != NULL) {
			err = mbuf_audio_frame_unref(frame);
			if (err < 0)
				ULOG_ERRNO("mbuf_audio_frame_unref", -err);
		}
		if (mem != NULL) {
			err = mbuf_mem_unref(mem);
			if (err < 0)
				ULOG_ERRNO("mbuf_mem_unref", -err);
		}

		process_output(self);
	}

	/* Process the remaining frames */
	while (self->out_count < in_count) {
		/* TODO: there should be an API to drain the remaining frames */
		ULOGI("%u %u", self->out_count, in_count);
		usleep(5000);
		process_output(self);
	}

	/* Flush the audio source */
	res = pdraw_be_audio_source_flush(self->pdraw, self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_audio_source_flush", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	pthread_mutex_lock(&self->mutex);
	while (!self->flushed_resp)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->flushed_resp = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source flushed");

	/* Destroy the audio source */
	res = pdraw_be_audio_source_destroy(self->pdraw, self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_audio_source_destroy", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->source = NULL;
	self->in_queue = NULL;
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
				res = pdraw_be_audio_sink_destroy(self->pdraw,
								  self->sink);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_audio_sink_destroy",
						-res);
				}
			}
			if (self->source != NULL) {
				res = pdraw_be_audio_source_destroy(
					self->pdraw, self->source);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_audio_source_destroy",
						-res);
				}
			}
			res = pdraw_be_destroy(self->pdraw);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_destroy", -res);
		}
		if (self->writer != NULL) {
			res = araw_writer_destroy(self->writer);
			if (res < 0)
				ULOG_ERRNO("araw_writer_destroy", -res);
		}
		if (self->reader != NULL) {
			res = araw_reader_destroy(self->reader);
			if (res < 0)
				ULOG_ERRNO("araw_reader_destroy", -res);
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
