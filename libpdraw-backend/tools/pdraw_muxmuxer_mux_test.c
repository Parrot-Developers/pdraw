/**
 * Parrot Drones Audio and Video Vector
 * RTSP stream muxer over mux (SkyController link) test program
 *
 * Copyright (c) 2018 Parrot Drones SAS
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

/**
 * Usage:
 *   # 1. Start a local RTSP server (e.g. mediamtx):
 *   #      mediamtx
 *   #
 *   # 2. Run the test:
 *   #      pdraw-muxmuxer-mux-test -e H264 \
 *   #          -u rtsp://127.0.0.1:8554/live \
 *   #          input.264
 *   #
 *   # 3. (optional) verify reception in another terminal:
 *   #      ffplay rtsp://127.0.0.1:8554/live
 *   #
 *   # The test validates the full RTSP state machine over the stub-mux
 *   # tunnel (CONNECT -> OPTIONS -> ANNOUNCE -> SETUP -> RECORD -> TEARDOWN).
 *   # With a real RTSP server the stub proxy forwards RTP/RTCP packets
 *   # over loopback so the stream is actually receivable.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <windows.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <sys/mman.h>
#endif /* !_WIN32 */

#define ULOG_TAG pdraw_muxmuxer_mux_test
#include <ulog.h>

#include <h264/h264.h>
#include <h265/h265.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <pdraw/pdraw_backend.h>
#include <video-defs/vdefs.h>

#include "stub_mux.h"

ULOG_DECLARE_TAG(ULOG_TAG);

#define UNUSED(x) (void)(x)

#define DEFAULT_TS_INC 33333ULL
#define DEFAULT_FRAME_LEN (3840 * 2160 * 3 / 4)
#define FRIENDLY_NAME "pdraw_muxmuxer_mux_test"


/* =========================================================================
 * Application state
 * ========================================================================= */

union nalu_type {
	enum h264_nalu_type h264;
	enum h265_nalu_type h265;
};


struct app {
	/* Sync */
	pthread_mutex_t mutex;
	bool mutex_created;
	pthread_cond_t cond;
	bool cond_created;

	/* PDrAW objects */
	struct pdraw_backend *pdraw;
	struct mux_ctx *mux;
	struct pdraw_coded_video_source *source;
	void *source_element_ud;
	struct pdraw_muxer *muxer;

	/* Source -> muxer wiring */
	uint32_t source_media_id;
	bool source_media_known;
	bool muxer_connected; /* true once muxer reaches CONNECTED */

	/* Frame injection */
	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_mem *in_mem;
	size_t in_mem_offset;
	struct mbuf_coded_video_frame *in_frame;
	struct vdef_coded_frame in_info;
	struct vdef_format_info format_info;
	uint64_t ts_inc;
	unsigned int input_count;
	unsigned int max_count;

	/* Input file */
#ifdef _WIN32
	HANDLE in_file;
	HANDLE in_file_map;
#else
	int in_fd;
#endif
	void *in_data;
	size_t in_len;
	size_t in_off;

	/* H264/H265 reader */
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

	/* Completion flags (waited on with cond) */
	bool source_configured;
	bool source_flushed;
	bool source_drained;
	bool source_media_removed;
	bool muxer_close_resp;
	bool stop_resp;
	int stop_resp_status;
};


/* =========================================================================
 * Forward declarations
 * ========================================================================= */

static int configure(struct app *self);
static int au_process(struct app *self);


/* =========================================================================
 * Coded video source callbacks
 * ========================================================================= */

static void source_flushed_cb(struct pdraw_backend *pdraw,
			      struct pdraw_coded_video_source *source,
			      void *ud)
{
	UNUSED(pdraw);
	UNUSED(source);
	struct app *self = ud;
	ULOGI("%s", __func__);
	pthread_mutex_lock(&self->mutex);
	self->source_flushed = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void source_drained_cb(struct pdraw_backend *pdraw,
			      struct pdraw_coded_video_source *source,
			      void *ud)
{
	UNUSED(pdraw);
	UNUSED(source);
	struct app *self = ud;
	ULOGI("%s", __func__);
	pthread_mutex_lock(&self->mutex);
	self->source_drained = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static const struct pdraw_backend_coded_video_source_cbs source_cbs = {
	.flushed = source_flushed_cb,
	.drained = source_drained_cb,
};


/* =========================================================================
 * Muxer callbacks
 * ========================================================================= */

static void
muxer_connection_state_cb(struct pdraw_backend *pdraw,
			  struct pdraw_muxer *muxer,
			  enum pdraw_muxer_connection_state state,
			  enum pdraw_muxer_disconnection_reason reason,
			  void *ud)
{
	UNUSED(pdraw);
	UNUSED(muxer);
	struct app *self = ud;
	ULOGI("%s state=%d reason=%d", __func__, state, reason);

	/* OPTIONS_DONE maps to CONNECTING; SETUP_DONE maps to CONNECTED.
	 * We need to call addInputMedia after OPTIONS is done so that
	 * ANNOUNCE can be sent - trigger on the first CONNECTING. */
	if (state == PDRAW_MUXER_CONNECTION_STATE_CONNECTING) {
		pthread_mutex_lock(&self->mutex);
		self->muxer_connected = true;
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->mutex);
	}
}


static void muxer_unrecoverable_error_cb(struct pdraw_backend *pdraw,
					 struct pdraw_muxer *muxer,
					 int status,
					 void *ud)
{
	UNUSED(pdraw);
	UNUSED(muxer);
	struct app *self = ud;
	ULOGE("%s status=%d (%s)", __func__, status, strerror(-status));
	/* Signal stop so the test doesn't hang */
	pthread_mutex_lock(&self->mutex);
	self->stop_resp = true;
	self->stop_resp_status = status;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void muxer_close_resp_cb(struct pdraw_backend *pdraw,
				struct pdraw_muxer *muxer,
				int status,
				void *ud)
{
	UNUSED(pdraw);
	UNUSED(muxer);
	struct app *self = ud;
	ULOGI("%s status=%d", __func__, status);
	pthread_mutex_lock(&self->mutex);
	self->muxer_close_resp = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static const struct pdraw_backend_muxer_cbs muxer_cbs = {
	.connection_state_changed = muxer_connection_state_cb,
	.unrecoverable_error = muxer_unrecoverable_error_cb,
	.close_resp = muxer_close_resp_cb,
};


/* =========================================================================
 * PDrAW backend callbacks
 * ========================================================================= */

static void stop_resp_cb(struct pdraw_backend *pdraw, int status, void *ud)
{
	UNUSED(pdraw);
	struct app *self = ud;
	ULOGI("%s status=%d", __func__, status);
	pthread_mutex_lock(&self->mutex);
	self->stop_resp = true;
	self->stop_resp_status = status;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void media_added_cb(struct pdraw_backend *pdraw,
			   const struct pdraw_media_info *info,
			   void *element_ud,
			   void *ud)
{
	UNUSED(pdraw);
	struct app *self = ud;
	ULOGI("%s id=%d type=%d", __func__, info->id, info->type);

	if (element_ud != self->source)
		return;
	if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
		return;
	if (info->video.format != VDEF_FRAME_TYPE_CODED)
		return;

	self->source_element_ud = element_ud;
	self->in_queue =
		pdraw_be_coded_video_source_get_queue(pdraw, self->source);
	if (!self->in_queue)
		ULOG_ERRNO("pdraw_be_coded_video_source_get_queue", EPROTO);

	pthread_mutex_lock(&self->mutex);
	self->source_media_id = info->id;
	self->source_media_known = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_ud,
			     void *ud)
{
	UNUSED(pdraw);
	struct app *self = ud;
	ULOGI("%s id=%d", __func__, info->id);

	if (element_ud != self->source_element_ud)
		return;

	pthread_mutex_lock(&self->mutex);
	self->source_media_removed = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}


static void socket_created_cb(struct pdraw_backend *pdraw, int fd, void *ud)
{
	UNUSED(pdraw);
	UNUSED(ud);
	ULOGI("%s fd=%d", __func__, fd);
}


static const struct pdraw_backend_cbs be_cbs = {
	.stop_resp = stop_resp_cb,
	.media_added = media_added_cb,
	.media_removed = media_removed_cb,
	.socket_created = socket_created_cb,
};


/* =========================================================================
 * H.264/H.265 frame injection (mirrors codedsourcesink_test.c)
 * ========================================================================= */

static inline unsigned int gcd(unsigned int a, unsigned int b)
{
	unsigned int c;
	while (a) {
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
		unsigned int d = gcd(out->framerate.num, out->framerate.den);
		out->framerate.num /= d;
		out->framerate.den /= d;
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
		unsigned int d = gcd(out->framerate.num, out->framerate.den);
		out->framerate.num /= d;
		out->framerate.den /= d;
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


static int configure(struct app *self)
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
	if (self->format_info.framerate.num &&
	    self->format_info.framerate.den) {
		self->ts_inc = (uint64_t)self->format_info.framerate.den *
			       1000000ULL / self->format_info.framerate.num;
	}

	struct pdraw_video_source_params sp = {
		.queue_max_count = 0,
		.playback_type = PDRAW_PLAYBACK_TYPE_LIVE,
		.duration = 0,
		.video.format = VDEF_FRAME_TYPE_CODED,
		.video.coded.format = self->in_info.format,
		.video.coded.info = self->format_info,
	};

	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		if (self->sps_size > sizeof(sp.video.coded.h264.sps))
			return -ENOBUFS;
		memcpy(sp.video.coded.h264.sps, self->sps, self->sps_size);
		sp.video.coded.h264.spslen = self->sps_size;
		if (self->pps_size > sizeof(sp.video.coded.h264.pps))
			return -ENOBUFS;
		memcpy(sp.video.coded.h264.pps, self->pps, self->pps_size);
		sp.video.coded.h264.ppslen = self->pps_size;
		break;
	case VDEF_ENCODING_H265:
		if (self->vps_size > sizeof(sp.video.coded.h265.vps))
			return -ENOBUFS;
		memcpy(sp.video.coded.h265.vps, self->vps, self->vps_size);
		sp.video.coded.h265.vpslen = self->vps_size;
		if (self->sps_size > sizeof(sp.video.coded.h265.sps))
			return -ENOBUFS;
		memcpy(sp.video.coded.h265.sps, self->sps, self->sps_size);
		sp.video.coded.h265.spslen = self->sps_size;
		if (self->pps_size > sizeof(sp.video.coded.h265.pps))
			return -ENOBUFS;
		memcpy(sp.video.coded.h265.pps, self->pps, self->pps_size);
		sp.video.coded.h265.ppslen = self->pps_size;
		break;
	default:
		break;
	}

	snprintf(sp.session_meta.friendly_name,
		 sizeof(sp.session_meta.friendly_name),
		 "%s",
		 FRIENDLY_NAME);

	res = pdraw_be_coded_video_source_new(
		self->pdraw, &sp, &source_cbs, self, &self->source);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_coded_video_source_new", -res);
		return res;
	}

	/* Wait for media_added so we have the in_queue */
	pthread_mutex_lock(&self->mutex);
	while (!self->source_media_known)
		pthread_cond_wait(&self->cond, &self->mutex);
	pthread_mutex_unlock(&self->mutex);

	/* Wait for the muxer to be connected (may already be) */
	pthread_mutex_lock(&self->mutex);
	while (!self->muxer_connected)
		pthread_cond_wait(&self->cond, &self->mutex);
	pthread_mutex_unlock(&self->mutex);

	/* Connect source -> muxer from the main thread (not from a callback)
	 * so that pdraw_be_muxer_add_media() runs outside any loop-thread
	 * context and can safely marshal via runOnLoop(). */
	ULOGI("adding source media %u to muxer", self->source_media_id);
	res = pdraw_be_muxer_add_media(
		self->pdraw, self->muxer, self->source_media_id, NULL);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_muxer_add_media", -res);
		return res;
	}

	self->source_configured = true;
	ULOGI("source created and wired to muxer, media_id=%u",
	      self->source_media_id);
	return 0;
}


static int au_process(struct app *self)
{
	int res = 0;
	int err;

	if (!self->in_frame || !self->source_configured || !self->in_queue)
		return 0;

	if (self->max_count && self->input_count >= self->max_count)
		goto cleanup;

	res = mbuf_coded_video_frame_set_frame_info(self->in_frame,
						    &self->in_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_set_frame_info", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_finalize(self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_queue_push(self->in_queue, self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);
		goto cleanup;
	}

	ULOGI("injected frame #%u ts=%" PRIu64,
	      self->in_info.info.index,
	      self->in_info.info.timestamp);
	self->input_count++;

cleanup:
	err = mbuf_coded_video_frame_unref(self->in_frame);
	if (err < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	self->in_frame = NULL;
	err = mbuf_mem_unref(self->in_mem);
	if (err < 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	self->in_mem = NULL;
	self->in_mem_offset = 0;
	self->in_info.info.index++;
	self->in_info.info.timestamp += self->ts_inc;
	self->in_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;
	return res;
}


static int append_nalu(struct app *self,
		       const uint8_t *data,
		       size_t len,
		       union nalu_type type)
{
	int res;
	uint8_t *au_data;
	size_t capacity;
	/* AVCC format: 4-byte big-endian NALU length (not Annex-B start code)
	 */
	uint32_t lenbe = htonl((uint32_t)len);

	if (!self->in_mem) {
		size_t frame_len = self->format_info.resolution.width *
				   self->format_info.resolution.height * 3 / 4;
		if (!frame_len)
			frame_len = DEFAULT_FRAME_LEN;
		res = mbuf_mem_generic_new(frame_len, &self->in_mem);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_generic_new", -res);
			return res;
		}
		self->in_mem_offset = 0;
	}

	if (!self->in_frame) {
		res = mbuf_coded_video_frame_new(&self->in_info,
						 &self->in_frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_new", -res);
			return res;
		}
	}

	res = mbuf_mem_get_data(self->in_mem, (void **)&au_data, &capacity);
	if (res < 0)
		return res;

	size_t off = self->in_mem_offset;
	if (capacity < off + 4 + len)
		return -ENOBUFS;

	memcpy(au_data + off, &lenbe, 4);
	memcpy(au_data + off + 4, data, len);
	self->in_mem_offset = off + 4 + len;

	struct vdef_nalu nalu = {.size = len + 4};
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		nalu.h264.type = type.h264;
		break;
	case VDEF_ENCODING_H265:
		nalu.h265.type = type.h265;
		break;
	default:
		break;
	}
	return mbuf_coded_video_frame_add_nalu(
		self->in_frame, self->in_mem, off, &nalu);
}


static void
nalu_end(struct app *self, union nalu_type type, const uint8_t *buf, size_t len)
{
	int res;

	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		if (type.h264 == H264_NALU_TYPE_SPS && !self->sps) {
			self->sps = malloc(len);
			if (!self->sps)
				return;
			memcpy(self->sps, buf, len);
			self->sps_size = len;
			ULOGI("SPS found");
		} else if (type.h264 == H264_NALU_TYPE_PPS && !self->pps) {
			self->pps = malloc(len);
			if (!self->pps)
				return;
			memcpy(self->pps, buf, len);
			self->pps_size = len;
			ULOGI("PPS found");
		} else if (type.h264 == H264_NALU_TYPE_SLICE_IDR) {
			self->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		break;

	case VDEF_ENCODING_H265:
		if (type.h265 == H265_NALU_TYPE_VPS_NUT && !self->vps) {
			self->vps = malloc(len);
			if (!self->vps)
				return;
			memcpy(self->vps, buf, len);
			self->vps_size = len;
			ULOGI("VPS found");
		} else if (type.h265 == H265_NALU_TYPE_SPS_NUT && !self->sps) {
			self->sps = malloc(len);
			if (!self->sps)
				return;
			memcpy(self->sps, buf, len);
			self->sps_size = len;
			ULOGI("SPS found");
		} else if (type.h265 == H265_NALU_TYPE_PPS_NUT && !self->pps) {
			self->pps = malloc(len);
			if (!self->pps)
				return;
			memcpy(self->pps, buf, len);
			self->pps_size = len;
			ULOGI("PPS found");
		} else if (type.h265 == H265_NALU_TYPE_IDR_W_RADL ||
			   type.h265 == H265_NALU_TYPE_IDR_N_LP) {
			self->in_info.type = VDEF_CODED_FRAME_TYPE_IDR;
		}
		break;

	default:
		break;
	}

	/* Configure source once we have the parameter sets */
	bool ps_ready = false;
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		ps_ready = self->sps && self->pps;
		break;
	case VDEF_ENCODING_H265:
		ps_ready = self->vps && self->sps && self->pps;
		break;
	default:
		break;
	}
	/* Guard: do not retry configure if the source was already created but
	 * wiring failed -> a second attempt would create a duplicate source. */
	if (!self->source && !self->source_configured && ps_ready) {
		res = configure(self);
		if (res < 0) {
			ULOG_ERRNO("configure", -res);
			return;
		}
	}

	res = append_nalu(self, buf, len, type);
	if (res < 0)
		ULOG_ERRNO("append_nalu", -res);
}


static void au_end(struct app *self)
{
	if (self->in_frame)
		au_process(self);
}


/* ---- H.264 callbacks ---- */

static void h264_nalu_end_cb(struct h264_ctx *ctx,
			     enum h264_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h264_nalu_header *nh,
			     void *ud)
{
	UNUSED(ctx);
	UNUSED(nh);
	nalu_end(ud, (union nalu_type){.h264 = type}, buf, len);
}


static void h264_au_end_cb(struct h264_ctx *ctx, void *ud)
{
	UNUSED(ctx);
	au_end(ud);
}


static const struct h264_ctx_cbs h264_cbs = {
	.au_end = h264_au_end_cb,
	.nalu_end = h264_nalu_end_cb,
};


/* ---- H.265 callbacks ---- */

static void h265_nalu_end_cb(struct h265_ctx *ctx,
			     enum h265_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h265_nalu_header *nh,
			     void *ud)
{
	UNUSED(ctx);
	UNUSED(nh);
	nalu_end(ud, (union nalu_type){.h265 = type}, buf, len);
}


static void h265_au_end_cb(struct h265_ctx *ctx, void *ud)
{
	UNUSED(ctx);
	au_end(ud);
}


static const struct h265_ctx_cbs h265_cbs = {
	.au_end = h265_au_end_cb,
	.nalu_end = h265_nalu_end_cb,
};


/* =========================================================================
 * Command-line and main
 * ========================================================================= */

static const char short_opts[] = "he:u:n:";

static const struct option long_opts[] = {
	{"help", no_argument, NULL, 'h'},
	{"encoding", required_argument, NULL, 'e'},
	{"url", required_argument, NULL, 'u'},
	{"count", required_argument, NULL, 'n'},
	{0, 0, 0, 0},
};


/* Extracts the host out of a "scheme://[user[:pass]@]host[:port][/path]" URL
 * into a static buffer. This tool talks to a stub mux that does no tunneling
 * or rewriting, so the RTSP server is reached directly at this same host --
 * unlike the real "sc" mux backend, there is no separate loopback tunnel
 * endpoint to distinguish it from. */
static const char *url_host(const char *url)
{
	static char host[256];
	const char *p = strstr(url, "://");
	const char *at;
	const char *end;

	p = (p != NULL) ? p + 3 : url;
	at = strchr(p, '@');
	if (at != NULL)
		p = at + 1;

	end = p;
	while (*end != '\0' && *end != ':' && *end != '/')
		end++;

	size_t len = (size_t)(end - p);
	if (len >= sizeof(host))
		len = sizeof(host) - 1;
	memcpy(host, p, len);
	host[len] = '\0';

	return host;
}


static void usage(const char *prog)
{
	printf("Usage: %s [options] <input_file>\n\n"
	       "Options:\n"
	       "  -h | --help              Print this message\n"
	       "  -e | --encoding <val>    H264 or H265\n"
	       "  -u | --url <url>         Destination RTSP URL "
	       "(default: rtsp://127.0.0.1:8554/live)\n"
	       "  -n | --count <n>         Inject at most n frames\n"
	       "\n"
	       "Requires a local RTSP server (e.g. mediamtx) listening on the\n"
	       "given URL before the test is run.\n\n",
	       prog);
}


int main(int argc, char **argv)
{
	int res;
	int status = EXIT_SUCCESS;
	const char *url = "rtsp://127.0.0.1:8554/live";
	const char *input = NULL;
	struct vdef_coded_format fmt = {0};
	struct app *self = NULL;

	printf("%s - PDrAW RTSP muxer over mux stub test\n\n", argv[0]);

	int idx;
	int c;
	while ((c = getopt_long(argc, argv, short_opts, long_opts, &idx)) !=
	       -1) {
		switch (c) {
		case 'h':
			usage(argv[0]);
			return EXIT_SUCCESS;
		case 'e':
			fmt.encoding = vdef_encoding_from_str(optarg);
			break;
		case 'u':
			url = optarg;
			break;
		case 'n':
			/* handled below */
			break;
		default:
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	if (argc - optind < 1) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	input = argv[optind];

	if (fmt.encoding != VDEF_ENCODING_H264 &&
	    fmt.encoding != VDEF_ENCODING_H265) {
		fprintf(stderr, "Unsupported encoding - use H264 or H265\n\n");
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	fmt.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	/* Re-scan for --count after encoding is known */
	optind = 1;
	while ((c = getopt_long(argc, argv, short_opts, long_opts, &idx)) != -1)
		if (c == 'n' &&
		    sscanf(optarg, "%u", &(unsigned int){0}) != EOF) {
			/* store via self below */
		}

	self = calloc(1, sizeof(*self));
	if (!self) {
		ULOG_ERRNO("calloc", ENOMEM);
		return EXIT_FAILURE;
	}
	self->in_info.format = fmt;
	self->ts_inc = DEFAULT_TS_INC;
#ifdef _WIN32
	self->in_file = INVALID_HANDLE_VALUE;
	self->in_file_map = INVALID_HANDLE_VALUE;
#else
	self->in_fd = -1;
#endif

	/* Re-scan once more cleanly for --count */
	optind = 1;
	while ((c = getopt_long(argc, argv, short_opts, long_opts, &idx)) != -1)
		if (c == 'n')
			sscanf(optarg, "%u", &self->max_count);

	res = pthread_mutex_init(&self->mutex, NULL);
	if (res) {
		ULOG_ERRNO("pthread_mutex_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->mutex_created = true;

	res = pthread_cond_init(&self->cond, NULL);
	if (res) {
		ULOG_ERRNO("pthread_cond_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	self->cond_created = true;

	/* Map input file */
#ifdef _WIN32
	{
		BOOL wret;
		LARGE_INTEGER filesize;

		self->in_file = CreateFileA(input,
					    GENERIC_READ,
					    0,
					    NULL,
					    OPEN_EXISTING,
					    FILE_ATTRIBUTE_NORMAL,
					    NULL);
		if (self->in_file == INVALID_HANDLE_VALUE) {
			res = -EIO;
			ULOG_ERRNO("CreateFileA('%s')", -res, input);
			status = EXIT_FAILURE;
			goto out;
		}

		self->in_file_map = CreateFileMapping(
			self->in_file, NULL, PAGE_READONLY, 0, 0, NULL);
		if (self->in_file_map == INVALID_HANDLE_VALUE) {
			res = -EIO;
			ULOG_ERRNO("CreateFileMapping('%s')", -res, input);
			status = EXIT_FAILURE;
			goto out;
		}

		wret = GetFileSizeEx(self->in_file, &filesize);
		if (wret == FALSE) {
			res = -EIO;
			ULOG_ERRNO("GetFileSizeEx('%s')", -res, input);
			status = EXIT_FAILURE;
			goto out;
		}
		self->in_len = filesize.QuadPart;

		self->in_data = MapViewOfFile(
			self->in_file_map, FILE_MAP_READ, 0, 0, 0);
		if (self->in_data == NULL) {
			res = -EIO;
			ULOG_ERRNO("MapViewOfFile('%s')", -res, input);
			status = EXIT_FAILURE;
			goto out;
		}
	}
#else /* !_WIN32 */
	self->in_fd = open(input, O_RDONLY);
	if (self->in_fd < 0) {
		res = -errno;
		ULOG_ERRNO("open('%s')", -res, input);
		status = EXIT_FAILURE;
		goto out;
	}
	{
		off_t sz = lseek(self->in_fd, 0, SEEK_END);
		if (sz < 0) {
			ULOG_ERRNO("lseek", errno);
			status = EXIT_FAILURE;
			goto out;
		}
		self->in_len = (size_t)sz;
		self->in_data = mmap(NULL,
				     self->in_len,
				     PROT_READ,
				     MAP_PRIVATE,
				     self->in_fd,
				     0);
		if (self->in_data == MAP_FAILED) {
			ULOG_ERRNO("mmap", errno);
			status = EXIT_FAILURE;
			goto out;
		}
	}
#endif /* !_WIN32 */

	/* Bitstream reader */
	switch (fmt.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_new(&h264_cbs, self, &self->reader.h264);
		break;
	case VDEF_ENCODING_H265:
		res = h265_reader_new(&h265_cbs, self, &self->reader.h265);
		break;
	default:
		res = -EINVAL;
		break;
	}
	if (res < 0) {
		ULOG_ERRNO("reader_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* ---- 1. Create stub mux ---- */
	self->mux = stub_mux_new();
	if (!self->mux) {
		ULOGE("stub_mux_new failed");
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("stub mux created");

	/* ---- 2. Create pdraw-backend ---- */
	res = pdraw_be_new(&be_cbs, self, &self->pdraw);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("pdraw-backend created");

	/* ---- 3. Create RTSP muxer on stub mux ---- */
	{
		struct pdraw_muxer_params mp = {0};
		res = pdraw_be_muxer_new_on_mux(self->pdraw,
						url,
						self->mux,
						url_host(url),
						&mp,
						&muxer_cbs,
						self,
						&self->muxer);
	}
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_muxer_new_on_mux", -res);
		status = EXIT_FAILURE;
		goto out;
	}
	ULOGI("muxer created, connecting to %s …", url);

	/* ---- 4. Parse bitstream and inject frames ---- */
	ULOGI("parsing %s", input);
	res = 0;
	while (res == 0) {
		size_t off = 0;
		switch (fmt.encoding) {
		case VDEF_ENCODING_H264:
			res = h264_reader_parse(self->reader.h264,
						0,
						(const uint8_t *)self->in_data +
							self->in_off,
						self->in_len - self->in_off,
						&off);
			break;
		case VDEF_ENCODING_H265:
			res = h265_reader_parse(self->reader.h265,
						0,
						(const uint8_t *)self->in_data +
							self->in_off,
						self->in_len - self->in_off,
						&off);
			break;
		default:
			break;
		}

		self->in_off += off;
		if (self->in_off >= self->in_len ||
		    (self->max_count && self->input_count >= self->max_count)) {
			if (self->in_frame)
				au_process(self);
			break;
		}
	}
	if (res < 0 && res != -ENOENT)
		ULOG_ERRNO("reader_parse", -res);

	ULOGI("done injecting %u frames", self->input_count);

	/* ---- 5. Drain source ---- */
	if (self->source) {
		res = pdraw_be_coded_video_source_drain(self->pdraw,
							self->source);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_coded_video_source_drain", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		pthread_mutex_lock(&self->mutex);
		while (!self->source_drained)
			pthread_cond_wait(&self->cond, &self->mutex);
		self->source_drained = false;
		pthread_mutex_unlock(&self->mutex);
		ULOGI("source drained");
	}

	/* ---- 6. Destroy source ---- */
	if (self->source) {
		res = pdraw_be_coded_video_source_destroy(self->pdraw,
							  self->source);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_coded_video_source_destroy", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		self->source = NULL;
		self->in_queue = NULL;
		pthread_mutex_lock(&self->mutex);
		while (!self->source_media_removed)
			pthread_cond_wait(&self->cond, &self->mutex);
		self->source_media_removed = false;
		pthread_mutex_unlock(&self->mutex);
		ULOGI("source destroyed");
	}

	/* ---- 7. Close muxer ---- */
	if (self->muxer) {
		res = pdraw_be_muxer_close(self->pdraw, self->muxer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_muxer_close", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		pthread_mutex_lock(&self->mutex);
		while (!self->muxer_close_resp)
			pthread_cond_wait(&self->cond, &self->mutex);
		self->muxer_close_resp = false;
		pthread_mutex_unlock(&self->mutex);
		ULOGI("muxer closed");

		res = pdraw_be_muxer_destroy(self->pdraw, self->muxer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_muxer_destroy", -res);
		self->muxer = NULL;
	}

	/* ---- 8. Stop pdraw-backend ---- */
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
	self->stop_resp = false;
	pthread_mutex_unlock(&self->mutex);
	if (res < 0) {
		ULOG_ERRNO("stop_resp", -res);
		status = EXIT_FAILURE;
	}
	ULOGI("stopped - %u frames injected", self->input_count);

out:
	if (self) {
		if (self->pdraw) {
			if (self->muxer)
				pdraw_be_muxer_destroy(self->pdraw,
						       self->muxer);
			if (self->source)
				pdraw_be_coded_video_source_destroy(
					self->pdraw, self->source);
			pdraw_be_destroy(self->pdraw);
		}
		if (self->mux)
			mux_unref(self->mux);
#ifdef _WIN32
		if (self->in_data != NULL)
			UnmapViewOfFile(self->in_data);
		if (self->in_file_map != INVALID_HANDLE_VALUE)
			CloseHandle(self->in_file_map);
		if (self->in_file != INVALID_HANDLE_VALUE)
			CloseHandle(self->in_file);
#else /* !_WIN32 */
		if (self->in_data && self->in_data != MAP_FAILED)
			munmap(self->in_data, self->in_len);
		if (self->in_fd >= 0)
			close(self->in_fd);
#endif /* !_WIN32 */
		if (self->reader.h264) {
			switch (fmt.encoding) {
			case VDEF_ENCODING_H264:
				h264_reader_destroy(self->reader.h264);
				break;
			case VDEF_ENCODING_H265:
				h265_reader_destroy(self->reader.h265);
				break;
			default:
				break;
			}
		}
		free(self->vps);
		free(self->sps);
		free(self->pps);
		if (self->cond_created)
			pthread_cond_destroy(&self->cond);
		if (self->mutex_created)
			pthread_mutex_destroy(&self->mutex);
		free(self);
	}

	printf("%s\n", status == EXIT_SUCCESS ? "Success!" : "Failed!");
	return status;
}
