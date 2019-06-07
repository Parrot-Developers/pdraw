/**
 * Parrot Drones Awesome Video Viewer Library
 * GStreamer PDrAW source plugin
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

/* clang-format off */
#include <gst/gst.h>
#include <gst/video/video-format.h>
#include <string.h>

#include "gstpdrawsrc.h"

/** Log error with errno and gstreamer object */
#define LOG_ERRNO_OBJECT(_obj, _fct, _err) \
  GST_ERROR_OBJECT(_obj, "%s err=%d(%s)", \
      _fct, _err, g_strerror(_err))

#define SUPPORTED_RAW_FORMATS "{ I420, NV12 }"

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      "video/x-h264, "
        "width = (int) [16, MAX], "
        "height = (int) [16, MAX], "
        "framerate = (fraction) [0/1, MAX], "
        "stream-format = (string) { byte-stream, avc }"
    ));

struct _GstPdrawSrc
{
  /* parent */
  GstPushSrc parent;

  /* media added condition */
  GMutex media_lock;
  GCond media_cond;
  const struct pdraw_media_info *media_info;

  /* setup condition, i.e. vsink started */
  GMutex setup_lock;
  GCond setup_cond;
  gboolean started;

  /* PDrAW elements */
  struct pdraw_backend *pdraw;
  struct pdraw_demuxer *demuxer;
  struct pdraw_coded_video_sink *sink;
  struct mbuf_coded_video_frame_queue *queue;

  /* EOS */
  gboolean eos;

  /* Properties */
  gchar *url;

  /* helpers */
  struct mbuf_coded_video_frame *negotiation_frame;
};

/* Type definition */
#define gst_pdraw_src_parent_class parent_class
G_DEFINE_TYPE (GstPdrawSrc, gst_pdraw_src, GST_TYPE_PUSH_SRC);

/* properties */
enum
{
  PROP_0,
  PROP_URL,
};

/* GObject methods */
static void gst_pdraw_src_finalize (GObject * object);
static void gst_pdraw_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_pdraw_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

/* GstBaseSrc methods */
static gboolean gst_pdraw_src_start (GstBaseSrc * bsrc);
static gboolean gst_pdraw_src_stop (GstBaseSrc * bsrc);
static gboolean gst_pdraw_src_negotiate (GstBaseSrc * bsrc);

/* GstPushSrc methods */
static GstFlowReturn gst_pdraw_src_create (GstPushSrc * psrc, GstBuffer ** buf);

/* helpers */
static struct mbuf_coded_video_frame * get_frame_from_vsink (GstPdrawSrc * pdrawsrc);
static GstBuffer * make_codec_data (const struct pdraw_video_info * info);
static gboolean update_codec_data (GstPdrawSrc * pdrawsrc, GstCaps * caps);

/* PDrAW callbacks */
static void stop_resp (struct pdraw_backend * pdraw, int status,
    void * userdata);
static void media_added (struct pdraw_backend * pdraw,
    const struct pdraw_media_info * info, void * userdata);
static void media_removed (struct pdraw_backend * pdraw,
    const struct pdraw_media_info * info, void * userdata);
static void open_resp (struct pdraw_backend * pdraw,
    struct pdraw_demuxer * demuxer, int status, void * userdata);
static void close_resp (struct pdraw_backend * pdraw,
    struct pdraw_demuxer * demuxer, int status, void * userdata);
static void ready_to_play (struct pdraw_backend * pdraw,
    struct pdraw_demuxer * demuxer, int ready, void * userdata);
static void end_of_range (struct pdraw_backend *pdraw,
    struct pdraw_demuxer * demuxer, uint64_t timestamp, void * userdata);
static void play_resp (struct pdraw_backend * pdraw,
    struct pdraw_demuxer * demuxer, int status, uint64_t timestamp,
    float speed, void * userdata);
static void sink_flush (struct pdraw_backend * pdraw,
    struct pdraw_coded_video_sink * sink, void * userdata);

static const struct pdraw_backend_cbs pdraw_cbs = {
  .stop_resp = &stop_resp,
  .media_added = &media_added,
  .media_removed = &media_removed,
};

static const struct pdraw_backend_demuxer_cbs demuxer_cbs = {
  .open_resp = &open_resp,
  .close_resp = &close_resp,
  .ready_to_play = &ready_to_play,
  .end_of_range = &end_of_range,
  .play_resp = &play_resp,
};

static const struct pdraw_backend_coded_video_sink_cbs vsink_cbs = {
  .flush = &sink_flush,
};

/* helpers */
static struct mbuf_coded_video_frame *
get_frame_from_vsink (GstPdrawSrc * pdrawsrc)
{
  struct mbuf_coded_video_frame *frame = NULL;
  int res;

  g_mutex_lock (&pdrawsrc->setup_lock);

  while ((!pdrawsrc->started) || (pdrawsrc->queue == NULL))
    g_cond_wait (&pdrawsrc->setup_cond, &pdrawsrc->setup_lock);

  if (pdrawsrc->queue == NULL)
    goto out;

  if (g_atomic_int_get (&pdrawsrc->eos))
    goto out;

  res = mbuf_coded_video_frame_queue_pop (pdrawsrc->queue, &frame);
  if (res < 0 && res != -EAGAIN)
      LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_queue_pop", -res);

out:
  g_mutex_unlock (&pdrawsrc->setup_lock);
  return frame;
}

static GstBuffer *
make_codec_data (const struct pdraw_video_info * info)
{
  GstBuffer *buf = NULL;

  switch (info->format)
  {
    case VDEF_FRAME_TYPE_CODED:
      {
        gsize avc_size, sps_size, pps_size;
        guint off = 0;
        guint8 *data;
        const guint8 *sps, *pps;

        sps_size = info->coded.h264.spslen;
        pps_size = info->coded.h264.ppslen;
        sps = info->coded.h264.sps;
        pps = info->coded.h264.pps;

        avc_size = sps_size + pps_size + 11;
        data = g_malloc0 (avc_size);

        data[off++] = 0x01;           /* AVCDecoderConfiguration version 1 */
        data[off++] = sps[1];         /* profle_idc                        */
        data[off++] = sps[2];         /* profile_compatibility             */
        data[off++] = sps[3];         /* level_idc                         */
        data[off++] = 0xFC | (4 - 1); /* NALU Length Size Minus One        */

        data[off++] = 0xE0 | 1;       /* number of SPS NALUs */

        /* SPS size and SPS NALU data */
        data[off++] = (sps_size >> 8) & 0xFF;
        data[off++] = sps_size & 0xFF;
        memcpy (data + off, sps, sps_size);
        off += sps_size;

        data[off++] = 0x01;           /* number of PPS NALUs */

        /* PPS size and PPS NALU data */
        data[off++] = (pps_size >> 8) & 0xFF;
        data[off++] = pps_size & 0xFF;
        memcpy (data + off, pps, pps_size);
        off += pps_size;

        buf = gst_buffer_new_allocate (NULL, avc_size, NULL);
        if (buf)
          gst_buffer_fill (buf, 0, data, avc_size);

        GST_MEMDUMP ("avc-header", data, avc_size);
        g_free (data);

        break;
      }
    case VDEF_FRAME_TYPE_RAW: /* do nothing */
    default:
      break;
  }

  return buf;
}

static gboolean
update_codec_data (GstPdrawSrc * pdrawsrc, GstCaps * caps)
{
  gboolean ret = FALSE;
  const struct pdraw_video_frame *meta = NULL;
  int res;
  GstBuffer *codec_data_buf;
  struct mbuf_ancillary_data *ancillaryData = NULL;
  struct mbuf_coded_video_frame *frame = pdrawsrc->negotiation_frame;

  if (frame == NULL) {
    GST_ERROR_OBJECT (pdrawsrc, "no frame to update codec data");
    goto out;
  }

  res = mbuf_coded_video_frame_get_ancillary_data (frame,
      PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillaryData);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_get_ancillary_data",
        -res);
    goto out;
  }
  meta = mbuf_ancillary_data_get_buffer (ancillaryData, NULL);

  /* TODO: H265 support */
  if ((meta->format != VDEF_FRAME_TYPE_CODED) ||
      (meta->coded.format.encoding != VDEF_ENCODING_H264)) {
    GST_INFO_OBJECT (pdrawsrc, "no codec data needed for video format");
    ret = TRUE;
    goto out;
  }

  if (meta->coded.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC)
    gst_caps_set_simple (caps, "stream-format", G_TYPE_STRING,
        "avc", NULL);
  else if (meta->coded.format.data_format ==
           VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
    gst_caps_set_simple (caps, "stream-format", G_TYPE_STRING,
        "byte-stream", NULL);

  codec_data_buf = make_codec_data (&pdrawsrc->media_info->video);
  if (codec_data_buf) {
    gst_caps_set_simple (caps, "codec_data", GST_TYPE_BUFFER,
        codec_data_buf, NULL);
    gst_buffer_unref (codec_data_buf);
  }

  ret = TRUE;

out:
  if (ancillaryData)
    mbuf_ancillary_data_unref (ancillaryData);
  return ret;
}

/* PDrAW callbacks */
static void
stop_resp (struct pdraw_backend * pdraw, int status, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;

  if (status < 0) {
    GST_ERROR_OBJECT (pdrawsrc, "%s: failed status=%d(%s)",
        __func__, -status, g_strerror(-status));
  }

  /* TODO */
}

static void
media_added (struct pdraw_backend *pdraw, const struct pdraw_media_info *info,
    void *userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;
  int res;
  struct pdraw_video_sink_params params = {
    .queue_max_count = 1,
  };

  if (info->type != PDRAW_MEDIA_TYPE_VIDEO ||
      info->video.format != VDEF_FRAME_TYPE_CODED ||
      info->video.type != PDRAW_VIDEO_TYPE_DEFAULT_CAMERA)
    return;

  g_mutex_lock (&pdrawsrc->setup_lock);

  if (pdrawsrc->sink != NULL)
    goto out;

  res = pdraw_be_coded_video_sink_new (pdrawsrc->pdraw, info->id, &params,
      &vsink_cbs, pdrawsrc, &pdrawsrc->sink);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_coded_video_sink_new", -res);
    goto out;
  }

  /* save media info for caps negotiation */
  g_mutex_lock (&pdrawsrc->media_lock);
  pdrawsrc->media_info = g_slice_dup (struct pdraw_media_info, info);
  g_cond_signal (&pdrawsrc->media_cond);
  g_mutex_unlock (&pdrawsrc->media_lock);

  GST_DEBUG_OBJECT (pdrawsrc, "media added: id=%d, type=%d",
      pdrawsrc->media_info->id, pdrawsrc->media_info->type);

  pdrawsrc->queue = pdraw_be_coded_video_sink_get_queue (pdrawsrc->pdraw,
      pdrawsrc->sink);
  if (pdrawsrc->queue == NULL) {
    res = -EPROTO;
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_coded_video_sink_get_queue", -res);
    goto out;
  }

out:
  g_cond_signal (&pdrawsrc->setup_cond);
  g_mutex_unlock (&pdrawsrc->setup_lock);
}

static void
media_removed (struct pdraw_backend * pdraw,
    const struct pdraw_media_info * info, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;
  int res;

  g_mutex_lock (&pdrawsrc->setup_lock);

  if (pdrawsrc->sink == NULL)
    goto out;
  if (pdrawsrc->media_info->id != info->id)
    goto out;

  res = pdraw_be_coded_video_sink_destroy (pdrawsrc->pdraw, pdrawsrc->sink);
  if (res < 0)
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_coded_video_sink_destroy", -res);
  pdrawsrc->sink = NULL;
  pdrawsrc->queue = NULL;
  g_slice_free (struct pdraw_media_info, (void *) pdrawsrc->media_info);
  pdrawsrc->media_info = NULL;

out:
  g_cond_signal (&pdrawsrc->setup_cond);
  g_mutex_unlock (&pdrawsrc->setup_lock);
}

static void
open_resp (struct pdraw_backend * pdraw, struct pdraw_demuxer * demuxer,
    int status, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;

  if (status < 0) {
    GST_ERROR_OBJECT (pdrawsrc, "%s: failed status=%d(%s)",
        __func__, -status, g_strerror(-status));
  }

  g_mutex_lock (&pdrawsrc->setup_lock);
  pdrawsrc->started = TRUE;
  g_cond_signal (&pdrawsrc->setup_cond);
  g_mutex_unlock (&pdrawsrc->setup_lock);
}

static void
close_resp (struct pdraw_backend * pdraw, struct pdraw_demuxer * demuxer,
    int status, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;

  if (status < 0) {
    GST_ERROR_OBJECT (pdrawsrc, "%s: failed status=%d(%s)",
        __func__, -status, g_strerror(-status));
  }

  g_mutex_lock (&pdrawsrc->setup_lock);
  pdrawsrc->started = FALSE;
  g_cond_signal (&pdrawsrc->setup_cond);
  g_mutex_unlock (&pdrawsrc->setup_lock);
}

static void
ready_to_play (struct pdraw_backend * pdraw, struct pdraw_demuxer * demuxer,
    int ready, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;
  int res;

  if (!ready)
    return;

  res = pdraw_be_demuxer_play (pdraw, demuxer);
  if (res < 0)
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_demuxer_play", -res);
}

static void
end_of_range (struct pdraw_backend * pdraw, struct pdraw_demuxer * demuxer,
    uint64_t timestamp, void * userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;

  GST_INFO_OBJECT (pdrawsrc, "end of media reached: send EOS upstream");
  g_atomic_int_set (&pdrawsrc->eos, TRUE);
}

static void
play_resp (struct pdraw_backend *pdraw, struct pdraw_demuxer * demuxer,
    int status, uint64_t timestamp, float speed, void *userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;

  if (status < 0) {
    GST_ERROR_OBJECT (pdrawsrc, "%s: failed status=%d(%s)",
        __func__, -status, g_strerror(-status));
  }
}

static void
sink_flush (struct pdraw_backend * pdraw, struct pdraw_coded_video_sink * sink,
    void *userdata)
{
  GstPdrawSrc *pdrawsrc = userdata;
  struct mbuf_coded_video_frame_queue *queue;
  int res;

  queue = pdraw_be_coded_video_sink_get_queue (pdraw, sink);
  if (queue == NULL) {
    res = -EPROTO;
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_coded_video_sink_get_queue", -res);
    return;
  }

  res = mbuf_coded_video_frame_queue_flush (queue);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_queue_flush", -res);
    return;
  }

  res = pdraw_be_coded_video_sink_queue_flushed (pdraw, sink);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_coded_video_sink_queue_flushed",
        -res);
    return;
  }
}

static void
gst_pdraw_src_class_init (GstPdrawSrcClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
  GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);

  gobject_class->finalize = GST_DEBUG_FUNCPTR (gst_pdraw_src_finalize);
  gobject_class->set_property = GST_DEBUG_FUNCPTR (gst_pdraw_src_set_property);
  gobject_class->get_property = GST_DEBUG_FUNCPTR (gst_pdraw_src_get_property);

  gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_pdraw_src_start);
  gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_pdraw_src_stop);
  gstbasesrc_class->negotiate = GST_DEBUG_FUNCPTR (gst_pdraw_src_negotiate);

  gstpushsrc_class->create = GST_DEBUG_FUNCPTR (gst_pdraw_src_create);

  g_object_class_install_property (gobject_class, PROP_URL,
      g_param_spec_string ("url", "URL",
          "Either RTSP URL or path to MP4 file", "",
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_set_static_metadata (gstelement_class,
    "GstPdrawSrc",
    "Source/Video",
    "Parrot PDrAW source",
    "Parrot Drones <www.parrot.com>");

  gst_element_class_add_static_pad_template (gstelement_class, &src_template);
}

static void
gst_pdraw_src_init (GstPdrawSrc * pdrawsrc)
{
  GstPad *srcpad = GST_BASE_SRC_PAD (pdrawsrc);

  g_mutex_init (&pdrawsrc->media_lock);
  g_cond_init (&pdrawsrc->media_cond);

  g_mutex_init (&pdrawsrc->setup_lock);
  g_cond_init (&pdrawsrc->setup_cond);

  gst_base_src_set_format (GST_BASE_SRC (pdrawsrc), GST_FORMAT_TIME);
  gst_base_src_set_live (GST_BASE_SRC (pdrawsrc), TRUE);

  gst_pad_use_fixed_caps (srcpad);
}

/* GObject methods */
static void
gst_pdraw_src_finalize (GObject * object)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (object);
  int res;

  if (pdrawsrc->pdraw != NULL) {
    g_mutex_lock (&pdrawsrc->setup_lock);

    while ((pdrawsrc->sink != NULL) || (pdrawsrc->started))
      g_cond_wait (&pdrawsrc->setup_cond, &pdrawsrc->setup_lock);

    g_mutex_unlock (&pdrawsrc->setup_lock);

    res = pdraw_be_destroy (pdrawsrc->pdraw);
    if (res < 0)
      LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_destroy", -res);
    pdrawsrc->pdraw = NULL;
    pdrawsrc->sink = NULL;
    pdrawsrc->queue = NULL;
    g_slice_free (struct pdraw_media_info, (void *) pdrawsrc->media_info);
  }

  g_mutex_clear (&pdrawsrc->setup_lock);
  g_cond_clear (&pdrawsrc->setup_cond);

  g_mutex_clear (&pdrawsrc->media_lock);
  g_cond_clear (&pdrawsrc->media_cond);

  g_free (pdrawsrc->url);
  pdrawsrc->url = NULL;

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
gst_pdraw_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (object);

  switch (prop_id) {
    case PROP_URL:
      g_free (pdrawsrc->url);
      pdrawsrc->url = g_value_dup_string (value);
      /* TODO: need to reconfigure in case this property changed in
       * PLAYING state */
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (pdrawsrc, prop_id, pspec);
      break;
  }
}

static void
gst_pdraw_src_get_property (GObject * object, guint prop_id, GValue * value,
    GParamSpec * pspec)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (object);

  switch (prop_id) {
    case PROP_URL:
      g_value_set_string (value, pdrawsrc->url);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (pdrawsrc, prop_id, pspec);
      break;
  }
}

/* GstBaseSrc methods */
static gboolean
gst_pdraw_src_start (GstBaseSrc * bsrc)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (bsrc);
  gboolean ret = FALSE;
  int res;

  if (pdrawsrc->pdraw == NULL) {
    res = pdraw_be_new (&pdraw_cbs, pdrawsrc, &pdrawsrc->pdraw);
    if (res < 0) {
      LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_new", -res);
      goto out;
    }
  }

  res = pdraw_be_set_pipeline_mode_setting (pdrawsrc->pdraw,
      PDRAW_PIPELINE_MODE_DECODE_NONE);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_set_pipeline_mode_setting", -res);
    goto out;
  }

  res = pdraw_be_demuxer_new_from_url (pdrawsrc->pdraw, pdrawsrc->url,
      &demuxer_cbs, pdrawsrc, &pdrawsrc->demuxer);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_open_url", -res);
    goto out;
  }

  ret = TRUE;

out:
  return ret;
}

static gboolean
gst_pdraw_src_stop (GstBaseSrc * bsrc)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (bsrc);
  int res;

  if (pdrawsrc->pdraw == NULL)
    return TRUE;

  res = pdraw_be_demuxer_close (pdrawsrc->pdraw, pdrawsrc->demuxer);
  if (res < 0) {
    LOG_ERRNO_OBJECT (pdrawsrc, "pdraw_be_demuxer_close", -res);
    return FALSE;
  }

  return TRUE;
}

static gboolean
gst_pdraw_src_negotiate (GstBaseSrc * bsrc)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (bsrc);
  const struct pdraw_video_info *video_info;
  gboolean ret = FALSE;
  GstCaps *allowed_caps, *desired_caps = NULL;
  GstCaps *our_caps = NULL, *peer_caps = NULL;

  allowed_caps = gst_pad_get_allowed_caps (GST_BASE_SRC_PAD (bsrc));
  GST_DEBUG_OBJECT (pdrawsrc, "allowed caps: %" GST_PTR_FORMAT, allowed_caps);

  if (gst_caps_is_empty (allowed_caps)) {
    GST_ERROR_OBJECT (pdrawsrc, "empty caps");
    gst_caps_unref (allowed_caps);
    goto out;
  }

  g_mutex_lock (&pdrawsrc->media_lock);
  GST_DEBUG_OBJECT (pdrawsrc, "waiting for media to be added");
  while (pdrawsrc->media_info == NULL)
    g_cond_wait (&pdrawsrc->media_cond, &pdrawsrc->media_lock);
  g_mutex_unlock (&pdrawsrc->media_lock);


  video_info = &pdrawsrc->media_info->video;

  /* get a first frame and store it to be used by _create later */
  pdrawsrc->negotiation_frame = get_frame_from_vsink (pdrawsrc);
  if (pdrawsrc->negotiation_frame == NULL) {
    GST_ERROR_OBJECT (pdrawsrc, "failed to get first frame for negotiation");
    goto out;
  }

  switch (video_info->format)
  {
    case VDEF_FRAME_TYPE_CODED:
      /* TODO: support H.265 */
      desired_caps = gst_caps_new_simple ("video/x-h264",
          "width", G_TYPE_INT, video_info->coded.info.resolution.width,
          "height", G_TYPE_INT, video_info->coded.info.resolution.height,
          NULL);
      update_codec_data (pdrawsrc, desired_caps);
      break;
    case VDEF_FRAME_TYPE_RAW:
      GST_FIXME_OBJECT (pdrawsrc, "handle YUV format");
      goto out;
    default:
      GST_WARNING_OBJECT (pdrawsrc, "cannot handle this video format: %d",
          video_info->format);
      goto out;
  }

  if (!gst_caps_can_intersect (desired_caps, allowed_caps)) {
    GST_ERROR_OBJECT (pdrawsrc, "cannot find common caps with upstream");
    goto out;
  }

  our_caps = gst_caps_intersect (desired_caps, allowed_caps);
  gst_caps_unref (desired_caps);
  gst_caps_unref (allowed_caps);

  our_caps = gst_caps_fixate (our_caps);
  GST_DEBUG_OBJECT (pdrawsrc, "desired caps %" GST_PTR_FORMAT, our_caps);

  /* get the peer pad caps filtered out with our caps */
  peer_caps = gst_pad_peer_query_caps (GST_BASE_SRC_PAD (bsrc), our_caps);
  GST_DEBUG_OBJECT (pdrawsrc, "filtered caps of peer: %" GST_PTR_FORMAT,
      peer_caps);

  if (!gst_caps_is_fixed (peer_caps)) {
    GST_ERROR_OBJECT (pdrawsrc, "peer caps not fixed: %" GST_PTR_FORMAT,
        peer_caps);
    goto out;
  }

  ret = gst_base_src_set_caps (bsrc, peer_caps);

out:
  if (peer_caps)
    gst_caps_unref (peer_caps);
  if (our_caps)
    gst_caps_unref (our_caps);
  return ret;
}

static void
out_mem_unref (gpointer data)
{
  (void) mbuf_coded_video_frame_unref ((struct mbuf_coded_video_frame *) data);
}

/* GstPushSrc methods */
static GstFlowReturn
gst_pdraw_src_create (GstPushSrc * psrc, GstBuffer ** buf)
{
  GstPdrawSrc *pdrawsrc = GST_PDRAW_SRC (psrc);
  struct mbuf_coded_video_frame *frame = NULL;
  struct mbuf_ancillary_data *ancillarydata = NULL;
  const struct pdraw_video_frame *meta = NULL;
  const void *data;
  struct vdef_coded_frame frameinfo;
  size_t size;
  gssize gsize;
  GstBuffer *newbuf = NULL;
  int res;
  gboolean silent = FALSE;

  /* loop until a non silent frame can be produced */
  do
  {
    if (pdrawsrc->negotiation_frame) {
      frame = pdrawsrc->negotiation_frame;
      pdrawsrc->negotiation_frame = NULL;
    } else {
      frame = get_frame_from_vsink (pdrawsrc);
    }

    if (frame == NULL) {
      if (G_UNLIKELY (g_atomic_int_compare_and_exchange (&pdrawsrc->eos, TRUE,
          FALSE))) {
        goto eos;
      } else {
        goto error;
      }
    }

    res = mbuf_coded_video_frame_get_frame_info (frame, &frameinfo);
    if (res < 0) {
      LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_get_frame_info",
          -res);
      goto error;
    }

    res = mbuf_coded_video_frame_get_packed_buffer (frame, &data, &size);
    if (res == 0) {
      /* Frame is already packed */
      mbuf_coded_video_frame_ref (frame);
    } else if (res == -EPROTO) {
      /* Frame is not packed, copy & pack it */
      struct mbuf_mem *mem;
      struct mbuf_coded_video_frame *new_frame;
      res = mbuf_mem_generic_new (size, &mem);
      if (res < 0) {
        LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_mem_generic_new", -res);
        goto error;
      }
      res = mbuf_coded_video_frame_copy (frame, mem, &new_frame);
      if (res < 0) {
        LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_copy", -res);
        goto error;
      }
      mbuf_coded_video_frame_unref (frame);
      frame = new_frame;
      res = mbuf_coded_video_frame_get_packed_buffer (frame, &data, &size);
      if (res < 0) {
        LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_get_packed_buffer",
            -res);
        goto error;
      }
    } else {
      LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_get_packed_buffer",
          -res);
      goto error;
    }
    gsize = size;

    res = mbuf_coded_video_frame_get_ancillary_data (frame,
        PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillarydata);
    if (res < 0) {
      LOG_ERRNO_OBJECT (pdrawsrc, "mbuf_coded_video_frame_get_ancillary_data", -res);
      goto error;
    }
    meta = mbuf_ancillary_data_get_buffer (ancillarydata, NULL);

    switch (meta->format)
    {
    case VDEF_FRAME_TYPE_CODED:
      GST_DEBUG_OBJECT (pdrawsrc,
          "coded video format=" VDEF_CODED_FORMAT_TO_STR_FMT
          ", is_silent=%d, is_sync=%d, is_ref=%d, size=%zu, PTS=%lu",
          VDEF_CODED_FORMAT_TO_STR_ARG (&meta->coded.format),
          !!(meta->coded.info.flags & VDEF_FRAME_FLAG_SILENT),
          meta->is_sync,
          meta->is_ref,
          size,
          meta->ntp_raw_timestamp);
      break;
    case VDEF_FRAME_TYPE_RAW:
      GST_FIXME_OBJECT (pdrawsrc, "handle raw formats");
      break;
    default:
      break;
    }

    silent = (meta->coded.info.flags & VDEF_FRAME_FLAG_SILENT);
    if (silent) {
      mbuf_ancillary_data_unref (ancillarydata);
      ancillarydata = NULL;
      mbuf_coded_video_frame_release_packed_buffer(frame, data);
      mbuf_coded_video_frame_unref(frame);
    }
  } while (silent);
  if (ancillarydata) {
    mbuf_ancillary_data_unref (ancillarydata);
    ancillarydata = NULL;
  }

  newbuf = gst_buffer_new_wrapped_full (GST_MEMORY_FLAG_READONLY,
      (gpointer) data, gsize, 0, gsize, frame, out_mem_unref);
  if (newbuf == NULL) {
    res = -ENOMEM;
    LOG_ERRNO_OBJECT (pdrawsrc, "gst_buffer_new_wrapped_full", -res);
    goto error;
  }

  /* Release the packed buffer here, since gstreamer will only unref the frame.
   * This is sketchy, because it means releasing the read-lock, allowing another
   * thread to modify the frame while it is being decoded, but we're the only
   * consumer in this case, so we can do it.
   */
  res = mbuf_coded_video_frame_release_packed_buffer(frame, data);
  data = NULL;

  GST_BUFFER_PTS (newbuf) = meta->ntp_raw_timestamp * 1000; /* to nanoseconds */

  *buf = newbuf;
  return GST_FLOW_OK;

eos:
  g_assert (frame == NULL && newbuf == NULL);
  return GST_FLOW_EOS;

error:
  if (ancillarydata)
    mbuf_ancillary_data_unref (ancillarydata);
  if (frame && data)
    mbuf_coded_video_frame_release_packed_buffer (frame, data);
  if (frame)
    mbuf_coded_video_frame_unref (frame);
  return GST_FLOW_ERROR;
}

/* plugin init */
#define PACKAGE "gst-pdraw"
#define PACKAGE_VERSION "1.0.0"
#define PARROT_LICENSE "Proprietary"
#define PACKAGE_NAME "GstPdraw"
#define PACKAGE_ORIGIN "http://www.parrot.com"

GST_DEBUG_CATEGORY (gst_pdraw_src_debug);
#define GST_CAT_DEFAULT gst_pdraw_src_debug

static gboolean
plugin_init (GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT (gst_pdraw_src_debug, "pdrawsrc", 0,
      "PDrAW source debug category");

  return gst_element_register (plugin, "pdrawsrc", GST_RANK_NONE,
      GST_TYPE_PDRAW_SRC);
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR, GST_VERSION_MINOR, pdraw,
    "Parrot PDrAW plugin", plugin_init, PACKAGE_VERSION,
    PARROT_LICENSE, PACKAGE_NAME, PACKAGE_ORIGIN);
