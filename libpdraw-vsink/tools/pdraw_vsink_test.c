/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library test program
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
#include <time.h>
#include <unistd.h>

#define ULOG_TAG pdraw_vsink_test
#include <ulog.h>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw/pdraw.h>
#include <pdraw/pdraw_defs.h>
#include <stdatomic.h>
#include <video-defs/vdefs.h>
#include <video-metadata/vmeta.h>

ULOG_DECLARE_TAG(ULOG_TAG);

static atomic_int frame_count;


/* Can be called from any thread */
static void print_coded_frame_info(const struct pdraw_video_frame *frame_info,
				   struct mbuf_coded_video_frame *frame,
				   int frame_index)
{
	struct vmeta_frame *frame_meta = NULL;
	int err = 0;

	/* Get frame information */
	ULOGI("frame #%d (width=%u height=%u)",
	      frame_index,
	      frame_info->coded.info.resolution.width,
	      frame_info->coded.info.resolution.height);

	/* Get the video metadata */
	err = mbuf_coded_video_frame_get_metadata(frame, &frame_meta);
	if (err < 0 && err != -ENOENT)
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -err);
	if (frame_meta != NULL) {
		uint8_t battery_percentage;
		vmeta_frame_get_battery_percentage(frame_meta,
						   &battery_percentage);
		ULOGI("metadata: battery_percentage=%d%%", battery_percentage);
	}

	vmeta_frame_unref(frame_meta);
}


/* Can be called from any thread */
static void print_raw_frame_info(const struct pdraw_video_frame *frame_info,
				 struct mbuf_raw_video_frame *frame,
				 int frame_index)
{
	unsigned int plane_count;
	struct vmeta_frame *frame_meta = NULL;
	int err = 0;

	/* Get frame information */
	ULOGI("frame #%d (width=%d height=%d)",
	      frame_index,
	      frame_info->raw.info.resolution.width,
	      frame_info->raw.info.resolution.height);

	/* Get the video metadata */
	err = mbuf_raw_video_frame_get_metadata(frame, &frame_meta);
	if (err < 0 && err != -ENOENT)
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -err);
	if (frame_meta != NULL) {
		uint8_t battery_percentage;
		vmeta_frame_get_battery_percentage(frame_meta,
						   &battery_percentage);
		ULOGI("metadata: battery_percentage=%d%%", battery_percentage);
	}

	/* Get the frame planes */
	plane_count = vdef_get_raw_frame_plane_count(&frame_info->raw.format);
	for (unsigned int k = 0; k < plane_count; k++) {
		const void *plane = NULL;
		size_t plane_len;
		err = mbuf_raw_video_frame_get_plane(
			frame, k, &plane, &plane_len);
		if (err < 0) {
			ULOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -err, k);
			continue;
		}
		ULOGI("plane[%d]: addr=%p stride=%zu",
		      k,
		      plane,
		      frame_info->raw.plane_stride[k]);
		err = mbuf_raw_video_frame_release_plane(frame, k, plane);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane(%u)",
				   -err,
				   k);
	}

	vmeta_frame_unref(frame_meta);
}


/* Called from the pdraw_vsink thread */
static void frame_ready_cb(struct pdraw_vsink_frame *frame,
			   const struct pdraw_video_frame *frame_info,
			   void *userdata)
{

	if (frame->type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
		print_coded_frame_info(
			frame_info, frame->coded, atomic_load(&frame_count));
	} else {
		print_raw_frame_info(
			frame_info, frame->raw, atomic_load(&frame_count));
	}

	atomic_fetch_add(&frame_count, 1);
}


static void print_media_info(struct pdraw_media_info *media_info,
			     enum pdraw_vsink_video_media_type video_media_type)
{
	const struct vdef_format_info *info;

	ULOGI("media_info: name=%s, path=%s",
	      media_info->name,
	      media_info->path);

	if (video_media_type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED)
		info = &media_info->video.coded.info;
	else
		info = &media_info->video.raw.info;

	ULOGI("media_info: duration=%.3fs, res=%ux%u, framerate=%u/%u",
	      media_info->duration / 1000000.0,
	      info->resolution.width,
	      info->resolution.height,
	      info->framerate.num,
	      info->framerate.den);

	pdraw_media_info_free(media_info);
}


/* Called from the main thread */
static int async_test(const char *url,
		      enum pdraw_playback_mode playback_mode,
		      enum vmeta_camera_type camera_type,
		      int count,
		      enum pdraw_vsink_video_media_type video_media_type)
{
	int res;
	struct pdraw_vsink *vsink = NULL;
	struct pdraw_media_info *media_info = NULL;
	const struct timespec sleep_1ms = {
		.tv_sec = 0, .tv_nsec = 1000 * 1000, /* 1ms */
	};
	struct pdraw_vsink_params params = {
		.url = url,
		.playback_mode = playback_mode,
		.camera_type = camera_type,
		.video_media_type = video_media_type,
	};
	struct pdraw_vsink_cbs cbs = {
		.frame_ready = &frame_ready_cb,
	};

	res = pdraw_vsink_start(&params, &cbs, NULL, &media_info, &vsink);
	if (res < 0 || media_info == NULL) {
		ULOG_ERRNO("pdraw_vsink_start", -res);
		exit(EXIT_FAILURE);
	}
	ULOGI("started");

	print_media_info(media_info, video_media_type);

	while (atomic_load(&frame_count) < count) {
		/* Wait */
		nanosleep(&sleep_1ms, NULL);
	}

	res = pdraw_vsink_stop(vsink);
	if (res < 0)
		ULOG_ERRNO("pdraw_vsink_stop", -res);
	/* media_info won't be valid after pdraw_vsink_stop() */

	return res;
}


/* Called from the main thread */
static int sync_test(const char *url,
		     enum pdraw_playback_mode playback_mode,
		     enum vmeta_camera_type camera_type,
		     int timeout_ms,
		     int count,
		     enum pdraw_vsink_video_media_type video_media_type)
{
	int res;
	int i = 0;
	struct pdraw_vsink *vsink = NULL;
	struct pdraw_media_info *media_info = NULL;
	const struct timespec sleep_1ms = {
		.tv_sec = 0, .tv_nsec = 1000 * 1000, /* 1ms */
	};
	struct pdraw_vsink_params params = {
		.url = url,
		.playback_mode = playback_mode,
		.camera_type = camera_type,
		.video_media_type = video_media_type,
	};

	res = pdraw_vsink_start(&params, NULL, NULL, &media_info, &vsink);
	if (res < 0 || media_info == NULL) {
		ULOG_ERRNO("pdraw_vsink_start", -res);
		exit(EXIT_FAILURE);
	}
	ULOGI("started");

	print_media_info(media_info, video_media_type);

	while (i < count) {
		struct pdraw_video_frame frame_info = {0};
		struct pdraw_vsink_frame vframe;

		/* Get a new frame */
		res = pdraw_vsink_get_frame(
			vsink, timeout_ms, NULL, &frame_info, &vframe);
		if (res < 0) {
			ULOG_ERRNO("pdraw_vsink_get_frame", -res);
			nanosleep(&sleep_1ms, NULL);
			continue;
		}
		if (vframe.type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
			print_coded_frame_info(&frame_info, vframe.coded, i);
			mbuf_coded_video_frame_unref(vframe.coded);
		} else {
			print_raw_frame_info(&frame_info, vframe.raw, i);
			mbuf_raw_video_frame_unref(vframe.raw);
		}
		i++;
	}

	res = pdraw_vsink_stop(vsink);
	if (res < 0)
		ULOG_ERRNO("pdraw_vsink_stop", -res);
	/* media_info won't be valid after pdraw_vsink_stop() */

	return res;
}


enum args_id {
	ARGS_ID_NO_DECODE = 256,
};


static const char short_options[] = "ht:oc:n:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"offline", no_argument, NULL, 'o'},
	{"camera-type", required_argument, NULL, 'c'},
	{"count", required_argument, NULL, 'n'},
	{"timeout", required_argument, NULL, 't'},
	{"no-decode", no_argument, NULL, ARGS_ID_NO_DECODE},
};


static void usage(char *prog_name)
{
	/* clang-format off */
	printf("Usage: %s [OPTIONS] filepath\n"
	       "Options:\n"
	       "  -h | --help                          "
		       "Print this message\n"
	       "  -o | --offline                       "
		       "Offline demuxing mode (ignore framerate and output "
		       "frames as fast as possible)\n"
	       "  -c | --camera-type <cam>             "
		       "Camera type (e.g. 'front', 'horizontal-stereo-left', "
		       "'down-stereo-right'; default camera if not specified)\n"
	       "  -n | --count <n>                     "
		       "Process at most n frames\n"
	       "  -t | --timeout <num>                 "
		       "Set the maximum time to wait (in ms) to get a frame, "
		       "0 to return immediately (non-blocking mode) or -1 for "
		       "infinite wait\n"
	       "     | --no-decode                     "
		       "Don't decode the frames\n"
	       "\n",
	       prog_name);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int status = EXIT_SUCCESS;
	int res;
	int idx;
	int c;
	char *url = NULL;
	enum vmeta_camera_type camera_type = VMETA_CAMERA_TYPE_UNKNOWN;
	int timeout_ms = -1;
	int count = 20;
	enum pdraw_playback_mode playback_mode = PDRAW_PLAYBACK_MODE_REALTIME;
	enum pdraw_vsink_video_media_type video_media_type =
		PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;

	atomic_init(&frame_count, 0);

	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {

		case 'h':
			usage(argv[0]);
			goto out;

		case 'o':
			playback_mode = PDRAW_PLAYBACK_MODE_OFFLINE;
			break;

		case 'c':
			camera_type = vmeta_camera_type_from_str(optarg);
			break;

		case 'n':
			sscanf(optarg, "%d", &count);
			if (count <= 0) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			break;

		case 't':
			sscanf(optarg, "%d", &timeout_ms);
			break;

		case ARGS_ID_NO_DECODE:
			video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	if (argc - optind < 1) {
		usage(argv[0]);
		goto out;
	}

	url = argv[optind];
	if (url == NULL) {
		usage(argv[0]);
		goto out;
	}

	/* Sync mode: polling using pdraw_vsink_get_frame() */
	res = sync_test(url,
			playback_mode,
			camera_type,
			timeout_ms,
			count,
			video_media_type);
	if (res < 0)
		ULOG_ERRNO("sync_test", -res);

	/* Async mode: notify using the frame_ready callback */
	res = async_test(
		url, playback_mode, camera_type, count, video_media_type);
	if (res < 0)
		ULOG_ERRNO("async_test", -res);

	ULOGI("%s", (status == EXIT_SUCCESS) ? "success!" : "failed!");

out:
	exit(status);
}
