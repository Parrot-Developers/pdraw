/**
 * Parrot Drones Awesome Video Viewer
 * Coded video source to sink test program
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
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <windows.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <sys/mman.h>
#endif /* !_WIN32 */

#define ULOG_TAG pdraw_codedsourcesink_test
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_codedsourcesink_test);

#include <h264/h264.h>
#include <h265/h265.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <pdraw/pdraw_backend.h>
#include <video-defs/vdefs.h>


#define DEFAULT_TS_INC 33333
#define DEFAULT_FRAME_LEN (3840 * 2160 * 3 / 4)

#define FRIENDLY_NAME "pdraw_codedsourcesink_test"


union nalu_type {
	enum h264_nalu_type h264;
	enum h265_nalu_type h265;
};


struct pdraw_backend_app {
	pthread_mutex_t mutex;
	bool mutex_created;
	pthread_cond_t cond;
	bool cond_created;
	unsigned int max_count;
	struct vdef_format_info format_info;
	struct pdraw_backend *pdraw;
	struct pdraw_coded_video_source *source;
	void *source_element_userdata;
	struct pdraw_coded_video_sink *sink;
	bool configured;
	bool media_added;
	bool media_removed;
	bool flushed_resp;
	bool stop_resp;
	int stop_resp_status;
#ifdef _WIN32
	HANDLE in_file;
	HANDLE in_file_map;
#else
	int in_fd;
#endif
	void *in_data;
	size_t in_len;
	size_t in_off;
	struct mbuf_mem *in_mem;
	size_t in_mem_offset;
	unsigned int input_count;
	struct vdef_coded_frame in_info;
	struct mbuf_coded_video_frame *in_frame;
	union {
		struct h264_reader *h264;
		struct h265_reader *h265;
	} reader;
	uint8_t *vps;
	size_t vps_size;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	uint64_t ts_inc;
	uint32_t out_index;
	FILE *out_file;
	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *out_queue;
};


static void source_flushed_cb(struct pdraw_backend *pdraw,
			      struct pdraw_coded_video_source *source,
			      void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	ULOGI("%s", __func__);
	pthread_mutex_lock(&self->mutex);
	self->flushed_resp = true;
	pthread_mutex_unlock(&self->mutex);
	pthread_cond_signal(&self->cond);
}


static const struct pdraw_backend_coded_video_source_cbs source_cbs = {
	.flushed = &source_flushed_cb,
};


static void sink_flush_cb(struct pdraw_backend *pdraw,
			  struct pdraw_coded_video_sink *sink,
			  void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res;

	ULOGI("%s", __func__);

	if (self->out_queue == NULL)
		return;

	pthread_mutex_lock(&self->mutex);

	res = mbuf_coded_video_frame_queue_flush(self->out_queue);
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_queue_flush", -res);

	res = pdraw_be_coded_video_sink_queue_flushed(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_coded_video_sink_queue_flushed", -res);

	pthread_mutex_unlock(&self->mutex);
}


static const struct pdraw_backend_coded_video_sink_cbs sink_cbs = {
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

	if (element_userdata != self->source_element_userdata)
		return;
	if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
		return;
	if (info->video.format != VDEF_FRAME_TYPE_CODED)
		return;

	if (strcmp(info->session_meta->friendly_name, FRIENDLY_NAME) != 0)
		ULOGW("mismatch on friendly_name session metadata");

	struct pdraw_video_sink_params sink_params = {0};

	res = pdraw_be_coded_video_sink_new(self->pdraw,
					    info->id,
					    &sink_params,
					    &sink_cbs,
					    self,
					    &self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_coded_video_sink_new", -res);

	self->out_queue =
		pdraw_be_coded_video_sink_get_queue(self->pdraw, self->sink);
	if (self->out_queue == NULL)
		ULOG_ERRNO("pdraw_be_coded_video_sink_get_queue", EPROTO);

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

	res = pdraw_be_coded_video_sink_destroy(self->pdraw, self->sink);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_coded_video_sink_destroy", -res);
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


static void unmap_file(struct pdraw_backend_app *self)
{
#ifdef _WIN32
	if (self->in_data != NULL)
		UnmapViewOfFile(self->in_data);
	self->in_data = NULL;
	if (self->in_file_map != INVALID_HANDLE_VALUE)
		CloseHandle(self->in_file_map);
	self->in_file_map = INVALID_HANDLE_VALUE;
	if (self->in_file != INVALID_HANDLE_VALUE)
		CloseHandle(self->in_file);
	self->in_file = INVALID_HANDLE_VALUE;
#else
	if (self->in_fd >= 0) {
		if (self->in_data != NULL)
			munmap(self->in_data, self->in_len);
		self->in_data = NULL;
		close(self->in_fd);
		self->in_fd = -1;
	}
#endif
}


static int map_file(struct pdraw_backend_app *self, const char *input_file)
{
	int res;

#ifdef _WIN32
	BOOL ret;
	LARGE_INTEGER filesize;

	self->in_file = CreateFileA(input_file,
				    GENERIC_READ,
				    0,
				    NULL,
				    OPEN_EXISTING,
				    FILE_ATTRIBUTE_NORMAL,
				    NULL);
	if (self->in_file == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileA('%s')", -res, input_file);
		goto error;
	}

	self->in_file_map = CreateFileMapping(
		self->in_file, NULL, PAGE_READONLY, 0, 0, NULL);
	if (self->in_file_map == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileMapping('%s')", -res, input_file);
		goto error;
	}

	ret = GetFileSizeEx(self->in_file, &filesize);
	if (ret == FALSE) {
		res = -EIO;
		ULOG_ERRNO("GetFileSizeEx('%s')", -res, input_file);
		goto error;
	}
	self->in_len = filesize.QuadPart;

	self->in_data =
		MapViewOfFile(self->in_file_map, FILE_MAP_READ, 0, 0, 0);
	if (self->in_data == NULL) {
		res = -EIO;
		ULOG_ERRNO("MapViewOfFile('%s')", -res, input_file);
		goto error;
	}
#else
	/* Try to open input file */
	self->in_fd = open(input_file, O_RDONLY);
	if (self->in_fd < 0) {
		res = -errno;
		ULOG_ERRNO("open('%s')", -res, input_file);
		goto error;
	}

	/* Get size and map it */
	self->in_len = lseek(self->in_fd, 0, SEEK_END);
	if (self->in_len == (size_t)-1) {
		res = -errno;
		ULOG_ERRNO("lseek", -res);
		goto error;
	}

	self->in_data = mmap(
		NULL, self->in_len, PROT_READ, MAP_PRIVATE, self->in_fd, 0);
	if (self->in_data == MAP_FAILED) {
		res = -errno;
		ULOG_ERRNO("mmap", -res);
		goto error;
	}
#endif

	return 0;

error:
	unmap_file(self);
	return res;
}


static inline unsigned int gcd(unsigned int a, unsigned int b)
{
	int c;
	while (a != 0) {
		c = a;
		a = b % a;
		b = c;
	}
	return b;
}


static void h264_to_vdef_info(const struct h264_info *in,
			      struct vdef_format_info *out)
{
	out->framerate.num = in->framerate_num;
	out->framerate.den = in->framerate_den;
	if (out->framerate.den) {
		unsigned int divider =
			gcd(out->framerate.num, out->framerate.den);
		out->framerate.den /= divider;
		out->framerate.num /= divider;
	}
	out->bit_depth = in->bit_depth_luma;
	out->full_range = in->full_range;
	if (in->colour_description_present) {
		out->color_primaries =
			vdef_color_primaries_from_h264(in->colour_primaries);
		out->transfer_function = vdef_transfer_function_from_h264(
			in->transfer_characteristics);
		out->matrix_coefs =
			vdef_matrix_coefs_from_h264(in->matrix_coefficients);
	} else {
		out->color_primaries = VDEF_COLOR_PRIMARIES_UNKNOWN;
		out->transfer_function = VDEF_TRANSFER_FUNCTION_UNKNOWN;
		out->matrix_coefs = VDEF_MATRIX_COEFS_UNKNOWN;
	}
	out->resolution.width = in->crop_width;
	out->resolution.height = in->crop_height;
	out->sar.width = in->sar_width;
	out->sar.height = in->sar_height;
}


static void h265_to_vdef_info(const struct h265_info *in,
			      struct vdef_format_info *out)
{
	out->framerate.num = in->framerate_num;
	out->framerate.den = in->framerate_den;
	if (out->framerate.den) {
		unsigned int divider =
			gcd(out->framerate.num, out->framerate.den);
		out->framerate.den /= divider;
		out->framerate.num /= divider;
	}
	out->bit_depth = in->bit_depth_luma;
	out->full_range = in->full_range;
	if (in->colour_description_present) {
		out->color_primaries =
			vdef_color_primaries_from_h265(in->colour_primaries);
		out->transfer_function = vdef_transfer_function_from_h265(
			in->transfer_characteristics);
		out->matrix_coefs =
			vdef_matrix_coefs_from_h265(in->matrix_coefficients);
	} else {
		out->color_primaries = VDEF_COLOR_PRIMARIES_UNKNOWN;
		out->transfer_function = VDEF_TRANSFER_FUNCTION_UNKNOWN;
		out->matrix_coefs = VDEF_MATRIX_COEFS_UNKNOWN;
	}
	out->resolution.width = in->crop_width;
	out->resolution.height = in->crop_height;
	out->sar.width = in->sar_width;
	out->sar.height = in->sar_height;
}


static int configure(struct pdraw_backend_app *self)
{
	int res;

	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264: {
		struct h264_info info;
		res = h264_get_info(self->sps,
				    self->sps_size,
				    self->pps,
				    self->pps_size,
				    &info);
		if (res < 0) {
			ULOG_ERRNO("h264_get_info", -res);
			return res;
		}
		h264_to_vdef_info(&info, &self->format_info);
		break;
	}
	case VDEF_ENCODING_H265: {
		struct h265_info info;
		res = h265_get_info(self->vps,
				    self->vps_size,
				    self->sps,
				    self->sps_size,
				    self->pps,
				    self->pps_size,
				    &info);

		if (res < 0) {
			ULOG_ERRNO("h265_get_info", -res);
			return res;
		}
		h265_to_vdef_info(&info, &self->format_info);
		break;
	}
	default:
		break;
	}

	vdef_format_to_frame_info(&self->format_info, &self->in_info.info);
	self->in_info.info.timescale = 1000000;
	if ((self->format_info.framerate.num != 0) &&
	    (self->format_info.framerate.den != 0)) {
		self->ts_inc = self->format_info.framerate.den * 1000000 /
			       self->format_info.framerate.num;
	}

	/* Create the coded video source */
	struct pdraw_video_source_params source_params = {
		.queue_max_count = 0,
		.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY,
		.duration = 0, /* this is not known */
		.video.format = VDEF_FRAME_TYPE_CODED,
		.video.type = PDRAW_VIDEO_TYPE_DEFAULT_CAMERA,
		.video.coded.format = self->in_info.format,
		.video.coded.info = self->format_info,
	};
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		if (self->sps_size >
		    sizeof(source_params.video.coded.h264.sps)) {
			res = -ENOBUFS;
			ULOG_ERRNO("memcpy", -res);
			return res;
		}
		memcpy(source_params.video.coded.h264.sps,
		       self->sps,
		       self->sps_size);
		source_params.video.coded.h264.spslen = self->sps_size;
		if (self->pps_size >
		    sizeof(source_params.video.coded.h264.pps)) {
			res = -ENOBUFS;
			ULOG_ERRNO("memcpy", -res);
			return res;
		}
		memcpy(source_params.video.coded.h264.pps,
		       self->pps,
		       self->pps_size);
		source_params.video.coded.h264.ppslen = self->pps_size;
		break;
	case VDEF_ENCODING_H265:
		if (self->vps_size >
		    sizeof(source_params.video.coded.h265.vps)) {
			res = -ENOBUFS;
			ULOG_ERRNO("memcpy", -res);
			return res;
		}
		memcpy(source_params.video.coded.h265.vps,
		       self->vps,
		       self->vps_size);
		source_params.video.coded.h265.vpslen = self->vps_size;
		if (self->sps_size >
		    sizeof(source_params.video.coded.h265.sps)) {
			res = -ENOBUFS;
			ULOG_ERRNO("memcpy", -res);
			return res;
		}
		memcpy(source_params.video.coded.h265.sps,
		       self->sps,
		       self->sps_size);
		source_params.video.coded.h265.spslen = self->sps_size;
		if (self->pps_size >
		    sizeof(source_params.video.coded.h265.pps)) {
			res = -ENOBUFS;
			ULOG_ERRNO("memcpy", -res);
			return res;
		}
		memcpy(source_params.video.coded.h265.pps,
		       self->pps,
		       self->pps_size);
		source_params.video.coded.h265.ppslen = self->pps_size;
		break;
	default:
		break;
	}
	snprintf(source_params.session_meta.friendly_name,
		 sizeof(source_params.session_meta.friendly_name),
		 "%s",
		 FRIENDLY_NAME);
	res = pdraw_be_coded_video_source_new(
		self->pdraw, &source_params, &source_cbs, self, &self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_coded_video_source_new", -res);
		return res;
	}
	self->source_element_userdata = self->source;
	pthread_mutex_lock(&self->mutex);
	while (!self->media_added)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->media_added = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source created");

	self->in_queue = pdraw_be_coded_video_source_get_queue(self->pdraw,
							       self->source);
	if (self->in_queue == NULL) {
		ULOG_ERRNO("pdraw_be_coded_video_source_get_queue", EPROTO);
		return res;
	}

	self->configured = true;
	return 0;
}


static int au_process(struct pdraw_backend_app *self)
{
	int res = 0, err;

	if ((self->in_frame == NULL) || (!self->configured))
		return 0;

	if ((self->max_count > 0) && (self->input_count >= self->max_count))
		goto cleanup;

	res = mbuf_coded_video_frame_set_frame_info(self->in_frame,
						    &self->in_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_set_frame_info:input", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_finalize(self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize:input", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_queue_push(self->in_queue, self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push:input", -res);
		goto cleanup;
	}
	self->input_count++;

	ULOGI("read frame #%d ts=%" PRIu64 " size=%zi",
	      self->in_info.info.index,
	      self->in_info.info.timestamp,
	      mbuf_coded_video_frame_get_packed_size(self->in_frame));

cleanup:
	err = mbuf_coded_video_frame_unref(self->in_frame);
	if (err < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_unref:input", -err);
	self->in_frame = NULL;
	err = mbuf_mem_unref(self->in_mem);
	if (err < 0)
		ULOG_ERRNO("mbuf_mem_unref:input", -err);
	self->in_mem = NULL;
	self->in_info.info.index++;
	self->in_info.info.timestamp += self->ts_inc;
	self->in_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	return res;
}


static int append_to_frame(struct pdraw_backend_app *self,
			   struct mbuf_coded_video_frame *frame,
			   struct mbuf_mem *mem,
			   const uint8_t *data,
			   size_t len,
			   union nalu_type type)
{
	int res;
	size_t au_offset, capacity;
	size_t nalu_offset = 4;
	uint8_t *au_data, *nalu_data;
	uint32_t start;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	au_offset = self->in_mem_offset;
	res = mbuf_mem_get_data(mem, (void **)&au_data, &capacity);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		return res;
	}
	if (capacity < au_offset + nalu_offset + len) {
		ULOGE("memory too small for frame");
		return -ENOBUFS;
	}
	if (au_data == NULL) {
		ULOG_ERRNO("mbuf_mem_get_data", EPROTO);
		return -EPROTO;
	}
	nalu_data = au_data + au_offset;
	start = htonl(0x00000001);
	memcpy(nalu_data, &start, sizeof(uint32_t));
	memcpy(nalu_data + nalu_offset, data, len);
	self->in_mem_offset = au_offset + nalu_offset + len;

	struct vdef_nalu nalu = {
		.size = len + nalu_offset,
	};
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		nalu.h264.type = type.h264;
		break;
	case VDEF_ENCODING_H265:
		nalu.h265.type = type.h265;
		break;
	default:
		ULOGE("unsupported encoding %s",
		      vdef_encoding_to_str(self->in_info.format.encoding));
		return -EPROTO;
	}
	res = mbuf_coded_video_frame_add_nalu(frame, mem, au_offset, &nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		return res;
	}

	return 0;
}


static int au_end(struct pdraw_backend_app *self)
{
	int res = 0;

	if (self->in_frame != NULL) {
		res = au_process(self);
		if (res < 0)
			ULOG_ERRNO("au_process", -res);
	}

	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_stop(self->reader.h264);
		if (res < 0)
			ULOG_ERRNO("h264_reader_stop", -res);
		break;
	case VDEF_ENCODING_H265:
		res = h265_reader_stop(self->reader.h265);
		if (res < 0)
			ULOG_ERRNO("h265_reader_stop", -res);
		break;
	default:
		break;
	}

	return res;
}


static void h264_au_end_cb(struct h264_ctx *ctx, void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res = au_end(userdata);
	if (res < 0) {
		res = h264_reader_stop(self->reader.h264);
		if (res < 0)
			ULOG_ERRNO("h264_reader_stop", -res);
	}
}


static void h265_au_end_cb(struct h265_ctx *ctx, void *userdata)
{
	struct pdraw_backend_app *self = userdata;
	int res = au_end(userdata);
	if (res < 0) {
		res = h265_reader_stop(self->reader.h265);
		if (res < 0)
			ULOG_ERRNO("h265_reader_stop", -res);
	}
}


static void nalu_end(struct pdraw_backend_app *self,
		     union nalu_type type,
		     const uint8_t *buf,
		     size_t len)
{
	int res;

	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		if ((type.h264 == H264_NALU_TYPE_SPS) && (self->sps == NULL)) {
			self->sps_size = len;
			self->sps = malloc(self->sps_size);
			if (self->sps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->sps, buf, len);
			ULOGI("SPS found");
		} else if ((type.h264 == H264_NALU_TYPE_PPS) &&
			   (self->pps == NULL)) {
			self->pps_size = len;
			self->pps = malloc(self->pps_size);
			if (self->pps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->pps, buf, len);
			ULOGI("PPS found");
		} else if (type.h264 == H264_NALU_TYPE_SLICE_IDR) {
			self->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		break;
	case VDEF_ENCODING_H265:
		if ((type.h265 == H265_NALU_TYPE_VPS_NUT) &&
		    self->vps == NULL) {
			self->vps_size = len;
			self->vps = malloc(self->vps_size);
			if (self->vps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->vps, buf, len);
			ULOGI("VPS found");
		} else if ((type.h265 == H265_NALU_TYPE_SPS_NUT) &&
			   self->sps == NULL) {
			self->sps_size = len;
			self->sps = malloc(self->sps_size);
			if (self->sps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->sps, buf, len);
			ULOGI("SPS found");
		} else if ((type.h265 == H265_NALU_TYPE_PPS_NUT) &&
			   self->pps == NULL) {
			self->pps_size = len;
			self->pps = malloc(self->pps_size);
			if (self->pps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->pps, buf, len);
			ULOGI("PPS found");
		} else if (type.h265 == H265_NALU_TYPE_IDR_W_RADL ||
			   type.h265 == H265_NALU_TYPE_IDR_N_LP) {
			self->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		break;
	default:
		break;
	}

	int ps_ready = 0;
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		ps_ready = (self->sps != NULL) && (self->pps != NULL);
		break;
	case VDEF_ENCODING_H265:
		ps_ready = (self->vps != NULL) && (self->sps != NULL) &&
			   (self->pps != NULL);
		break;
	default:
		break;
	}

	/* Configuration */
	if ((!self->configured) && ps_ready) {
		res = configure(self);
		if (res < 0) {
			ULOG_ERRNO("configure", -res);
			return;
		}
	}

	/* Get an input buffer */
	if (self->in_mem == NULL) {
		size_t frame_len = self->format_info.resolution.width *
				   self->format_info.resolution.height * 3 / 4;
		if (frame_len == 0)
			frame_len = DEFAULT_FRAME_LEN;
		res = mbuf_mem_generic_new(frame_len, &self->in_mem);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_generic_new", -res);
			return;
		}
		self->in_mem_offset = 0;
	}

	/* Create the frame */
	if (self->in_frame == NULL) {
		res = mbuf_coded_video_frame_new(&self->in_info,
						 &self->in_frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_new:input", -res);
			return;
		}
	}

	/* Add the NALU to the input frame */
	res = append_to_frame(
		self, self->in_frame, self->in_mem, buf, len, type);
	if (res < 0)
		ULOG_ERRNO("append_to_frame", -res);
}


static void h264_nalu_end_cb(struct h264_ctx *ctx,
			     enum h264_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h264_nalu_header *nh,
			     void *userdata)
{
	nalu_end(userdata, (union nalu_type){.h264 = type}, buf, len);
}


static void h265_nalu_end_cb(struct h265_ctx *ctx,
			     enum h265_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h265_nalu_header *nh,
			     void *userdata)
{
	nalu_end(userdata, (union nalu_type){.h265 = type}, buf, len);
}


static const struct h264_ctx_cbs h264_cbs = {
	.au_end = h264_au_end_cb,
	.nalu_end = h264_nalu_end_cb,
};


static const struct h265_ctx_cbs h265_cbs = {
	.au_end = h265_au_end_cb,
	.nalu_end = h265_nalu_end_cb,
};


static void process_output(struct pdraw_backend_app *self)
{
	int res = 0, err;

	if (self->out_queue == NULL)
		return;

	pthread_mutex_lock(&self->mutex);

	while (res == 0) {
		struct mbuf_coded_video_frame *frame = NULL;
		struct vdef_coded_frame info;
		size_t i, nalu_count;
		const uint8_t *data;
		const void *nalu_data;
		struct vdef_nalu nalu;

		res = mbuf_coded_video_frame_queue_pop(self->out_queue, &frame);
		if (res < 0) {
			if (res != -EAGAIN) {
				ULOG_ERRNO("mbuf_coded_video_frame_queue_pop",
					   -res);
			}
			break;
		}

		res = mbuf_coded_video_frame_get_frame_info(frame, &info);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
				   -res);
			goto out;
		}

		ULOGI("write frame #%d ts=%" PRIu64 " size=%zi",
		      info.info.index,
		      info.info.timestamp,
		      mbuf_coded_video_frame_get_packed_size(frame));
		self->out_index = info.info.index;

		/* Write the frame */
		nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
		for (i = 0; i < nalu_count; i++) {
			res = mbuf_coded_video_frame_get_nalu(
				frame, i, &nalu_data, &nalu);
			if (res < 0) {
				ULOG_ERRNO("mbuf_coded_video_frame_get_nalu",
					   -res);
				goto out;
			}
			data = nalu_data;

			res = fwrite(data, nalu.size, 1, self->out_file);
			if (res != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
			}

			res = mbuf_coded_video_frame_release_nalu(
				frame, i, nalu_data);
			if (res < 0) {
				ULOG_ERRNO(
					"mbuf_coded_video_frame_release_nalu",
					-res);
				goto out;
			}
		}

		/* clang-format off */
out:
		/* clang-format on */
		if (frame != NULL) {
			err = mbuf_coded_video_frame_unref(frame);
			if (err < 0)
				ULOG_ERRNO("mbuf_coded_video_frame_unref",
					   -err);
		}
	}

	pthread_mutex_unlock(&self->mutex);
}


static const char short_options[] = "he:n:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"encoding", required_argument, NULL, 'e'},
	{"count", required_argument, NULL, 'n'},
	{0, 0, 0, 0},
};


static void welcome(int argc, char **argv)
{
	printf("%s - Parrot Drones Awesome Video Viewer "
	       "Coded video source to sink test program\n\n",
	       argv[0]);
}


static void usage(int argc, char **argv)
{
	/* clang-format off */
	printf("Usage: %s [options] <input_file> <output_file>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
		       "Print this message\n"
	       "  -e | --encoding <val>              "
		       "Input file encoding (either \"H264\" or \"H265\")\n"
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
	struct vdef_coded_format format = {0};
	unsigned int max_count = 0;

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

		case 'e':
			format.encoding = vdef_encoding_from_str(optarg);
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

	if (format.encoding != VDEF_ENCODING_H264 &&
	    format.encoding != VDEF_ENCODING_H265) {
		printf("Unsupported encoding\n\n");
		usage(argc, argv);
		exit(EXIT_FAILURE);
	}
	format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	input = argv[optind];
	output = argv[optind + 1];

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	self->max_count = max_count;
	self->in_info.format = format;
	self->ts_inc = DEFAULT_TS_INC;

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

	/* Map the input file */
	res = map_file(self, input);
	if (res < 0) {
		ULOGE("failed to open file '%s'", input);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create reader */
	switch (format.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_new(&h264_cbs, self, &self->reader.h264);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		break;

	case VDEF_ENCODING_H265:
		res = h265_reader_new(&h265_cbs, self, &self->reader.h265);
		if (res < 0) {
			ULOG_ERRNO("h265_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		break;
	default:
		break;
	}

	/* Output file */
	self->out_file = fopen(output, "wb");
	if (!self->out_file) {
		ULOGE("failed to open file '%s'", output);
		status = EXIT_FAILURE;
		goto out;
	}

	ULOGI("format: " VDEF_CODED_FORMAT_TO_STR_FMT,
	      VDEF_CODED_FORMAT_TO_STR_ARG(&format));
	if (self->max_count != 0)
		ULOGI("max count: %d frames", self->max_count);

	/* Create PDrAW instance */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("created");

	/* Main loop */
	res = 0;
	while (res == 0) {
		size_t off = 0;
		switch (self->in_info.format.encoding) {
		case VDEF_ENCODING_H264:
			res = h264_reader_parse(self->reader.h264,
						0,
						(uint8_t *)self->in_data +
							self->in_off,
						self->in_len - self->in_off,
						&off);
			if (res < 0) {
				ULOG_ERRNO("h264_reader_parse", -res);
				break;
			}
			break;
		case VDEF_ENCODING_H265:
			res = h265_reader_parse(self->reader.h265,
						0,
						(uint8_t *)self->in_data +
							self->in_off,
						self->in_len - self->in_off,
						&off);
			if (res < 0) {
				ULOG_ERRNO("h265_reader_parse", -res);
				break;
			}
			break;
		default:
			break;
		}

		self->in_off += off;
		if ((self->in_off >= self->in_len) ||
		    ((self->max_count > 0) &&
		     (self->input_count >= self->max_count))) {
			if (self->in_frame != NULL) {
				/* Process the last AU in the file */
				res = au_process(self);
				if (res < 0)
					ULOG_ERRNO("au_process", -res);
			}
			break;
		}

		process_output(self);
	}

	/* Process the remaining frames */
	while (((self->max_count == 0) &&
		(self->out_index < self->in_info.info.index - 1)) ||
	       ((self->max_count > 0) &&
		(self->out_index < self->max_count - 1))) {
		/* TODO: there should be an API to drain the remaining
		 * frames */
		usleep(5000);
		process_output(self);
	}

	/* Flush the coded video source */
	res = pdraw_be_coded_video_source_flush(self->pdraw, self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_coded_video_source_flush", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	pthread_mutex_lock(&self->mutex);
	while (!self->flushed_resp)
		pthread_cond_wait(&self->cond, &self->mutex);
	self->flushed_resp = false;
	pthread_mutex_unlock(&self->mutex);
	ULOGI("source flushed");

	/* Destroy the coded video source */
	res = pdraw_be_coded_video_source_destroy(self->pdraw, self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_coded_video_source_destroy", -res);
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
				res = pdraw_be_coded_video_sink_destroy(
					self->pdraw, self->sink);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_coded_video_"
						"sink_destroy",
						-res);
				}
			}
			if (self->source != NULL) {
				res = pdraw_be_coded_video_source_destroy(
					self->pdraw, self->source);
				if (res < 0) {
					ULOG_ERRNO(
						"pdraw_be_coded_video_"
						"source_destroy",
						-res);
				}
			}
			res = pdraw_be_destroy(self->pdraw);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_destroy", -res);
		}
		if (self->out_file)
			fclose(self->out_file);
		switch (self->in_info.format.encoding) {
		case VDEF_ENCODING_H264:
			if (self->reader.h264 != NULL) {
				res = h264_reader_destroy(self->reader.h264);
				if (res < 0)
					ULOG_ERRNO("h264_reader_destroy", -res);
			}
			break;
		case VDEF_ENCODING_H265:
			if (self->reader.h265 != NULL) {
				res = h265_reader_destroy(self->reader.h265);
				if (res < 0)
					ULOG_ERRNO("h265_reader_destroy", -res);
			}
			break;
		default:
			break;
		}
		unmap_file(self);
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
		free(self->vps);
		free(self->sps);
		free(self->pps);
		free(self);
	}

	printf("%s\n", (status == EXIT_SUCCESS) ? "Success!" : "Failed!");
	exit(status);
}
