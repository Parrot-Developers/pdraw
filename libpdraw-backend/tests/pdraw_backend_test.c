/**
 * Parrot Drones Awesome Video Viewer
 * PDrAW back-end library test program
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
#include <stdio.h>

#define ULOG_TAG pdraw_backend_test
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_backend_test);

#include <pdraw/pdraw_backend.h>


struct pdraw_backend_app {
	pthread_mutex_t mutex;
	int mutex_created;
	pthread_cond_t cond;
	int cond_created;
	struct pdraw_backend *pdraw;
	struct pdraw_demuxer *demuxer;
	int open_resp;
	int open_resp_status;
	int ready_to_play_changed;
	int ready_to_play;
	int play_resp;
	int play_resp_status;
	int close_resp;
	int close_resp_status;
	int stop_resp;
	int stop_resp_status;
};


static void
stop_resp_cb(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));
	pthread_mutex_lock(&self->mutex);
	self->stop_resp = 1;
	self->stop_resp_status = status;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void media_added_cb(struct pdraw_backend *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_userdata,
			   void *userdata)
{
	ULOGI("%s id=%d", __func__, info->id);
}


static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	ULOGI("%s id=%d", __func__, info->id);
}


static void
socket_created_cb(struct pdraw_backend *pdraw, int fd, void *userdata)
{
	ULOGI("%s fd=%d", __func__, fd);
}


static struct pdraw_backend_cbs be_cbs = {
	.stop_resp = &stop_resp_cb,
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
	.socket_created = &socket_created_cb,
};


static void open_resp_cb(struct pdraw_backend *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));
	pthread_mutex_lock(&self->mutex);
	self->open_resp = 1;
	self->open_resp_status = status;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void close_resp_cb(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
			  int status,
			  void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s status=%d(%s)", __func__, status, strerror(-status));
	pthread_mutex_lock(&self->mutex);
	self->close_resp = 1;
	self->close_resp_status = status;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void unrecoverable_error_cb(struct pdraw_backend *pdraw,
				   struct pdraw_demuxer *demuxer,
				   void *userdata)
{
	ULOGI("%s", __func__);
}


static void ready_to_play_cb(struct pdraw_backend *pdraw,
			     struct pdraw_demuxer *demuxer,
			     int ready,
			     void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s ready=%d", __func__, ready);
	pthread_mutex_lock(&self->mutex);
	self->ready_to_play_changed = 1;
	self->ready_to_play = ready;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
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
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s status=%d(%s) timestamp=%" PRIu64 " speed=%f",
	      __func__,
	      status,
	      strerror(-status),
	      timestamp,
	      speed);
	pthread_mutex_lock(&self->mutex);
	self->play_resp = 1;
	self->play_resp_status = status;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static void pause_resp_cb(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
			  int status,
			  uint64_t timestamp,
			  void *userdata)
{
	ULOGI("%s status=%d(%s) timestamp=%" PRIu64,
	      __func__,
	      status,
	      strerror(-status),
	      timestamp);
}


static void seek_resp_cb(struct pdraw_backend *pdraw,
			 struct pdraw_demuxer *demuxer,
			 int status,
			 uint64_t timestamp,
			 float speed,
			 void *userdata)
{
	ULOGI("%s status=%d(%s) timestamp=%" PRIu64 " speed=%f",
	      __func__,
	      status,
	      strerror(-status),
	      timestamp,
	      speed);
}


static struct pdraw_backend_demuxer_cbs demuxer_cbs = {
	.open_resp = &open_resp_cb,
	.close_resp = &close_resp_cb,
	.unrecoverable_error = &unrecoverable_error_cb,
	.ready_to_play = &ready_to_play_cb,
	.end_of_range = &end_of_range_cb,
	.play_resp = &play_resp_cb,
	.pause_resp = &pause_resp_cb,
	.seek_resp = &seek_resp_cb,
};


static void welcome(char *prog_name)
{
	printf("%s - Parrot Drones Awesome Video Viewer "
	       "back-end library test program\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	printf("Usage: %s <file or url>\n\n", prog_name);
}


int main(int argc, char **argv)
{
	int res, status = EXIT_SUCCESS;
	struct pdraw_backend_app *self = NULL;

	welcome(argv[0]);

	if (argc < 2) {
		usage(argv[0]);
		status = EXIT_FAILURE;
		goto out;
	}

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
	self->mutex_created = 1;

	res = pthread_cond_init(&self->cond, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_cond_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->cond_created = 1;

	/* Create PDrAW instance */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("created");

	/* Open URL or file */
	res = pdraw_be_demuxer_new_from_url(
		self->pdraw, argv[1], &demuxer_cbs, self, &self->demuxer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_demuxer_new_from_url", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	pthread_mutex_lock(&self->mutex);
	while (!self->open_resp)
		pthread_cond_wait(&self->cond, &self->mutex);
	res = self->open_resp_status;
	self->open_resp_status = 0;
	self->open_resp = 0;
	pthread_mutex_unlock(&self->mutex);
	if (res < 0) {
		ULOG_ERRNO("open_resp", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("demuxer opened");

	/* Wait for 'ready to play' event and then play */
	pthread_mutex_lock(&self->mutex);
	while (!self->ready_to_play_changed)
		pthread_cond_wait(&self->cond, &self->mutex);
	res = self->ready_to_play;
	self->ready_to_play_changed = 0;
	pthread_mutex_unlock(&self->mutex);
	if (res) {
		res = pdraw_be_demuxer_play(self->pdraw, self->demuxer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_demuxer_play", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		pthread_mutex_lock(&self->mutex);
		while (!self->play_resp)
			pthread_cond_wait(&self->cond, &self->mutex);
		res = self->play_resp_status;
		self->play_resp_status = 0;
		self->play_resp = 0;
		pthread_mutex_unlock(&self->mutex);
		if (res < 0) {
			ULOG_ERRNO("play_resp", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		ULOGI("playing");
	}

	/* Play for 10s */
	sleep(10);

	/* Close the demuxer */
	res = pdraw_be_demuxer_close(self->pdraw, self->demuxer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_demuxer_close", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	pthread_mutex_lock(&self->mutex);
	while (!self->close_resp)
		pthread_cond_wait(&self->cond, &self->mutex);
	res = self->close_resp_status;
	self->close_resp_status = 0;
	self->close_resp = 0;
	pthread_mutex_unlock(&self->mutex);
	if (res < 0) {
		ULOG_ERRNO("close_resp", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("demuxer closed");
	res = pdraw_be_demuxer_destroy(self->pdraw, self->demuxer);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_destroy", -res);
	self->demuxer = NULL;

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
	self->stop_resp = 0;
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
			if (self->demuxer != NULL) {
				res = pdraw_be_demuxer_destroy(self->pdraw,
							       self->demuxer);
				if (res < 0) {
					ULOG_ERRNO("pdraw_be_demuxer_destroy",
						   -res);
				}
			}
			res = pdraw_be_destroy(self->pdraw);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_destroy", -res);
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
